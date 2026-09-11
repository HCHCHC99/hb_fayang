# 时钟及 ADC 中断架构

## 一、Timer0 资源总览

HC32F460 有两组 Timer0：TMR0_1 和 TMR0_2，每组有 A/B 两个通道。

| 定时器 | 通道 | 归属 | 用途 | 周期 | 模式 | 代码位置 |
|--------|------|------|------|------|------|----------|
| TMR0_1 | CH_A | `Timer0_Unit1` | 系统 tick 中断 | 1000µs (1kHz) | CPU 中断 (INT006) | `Hardware.c:255` |
| TMR0_1 | **CH_B** | **`Adc.c`** | **ADC AOS 硬件触发** | **200µs (5kHz)** | 硬件触发→ADC | `Adc.c:431` → `Adc.h:61` |
| TMR0_2 | CH_A | `Timer0_Unit2` | 定时中断 | 1000µs (1kHz) | CPU 中断 | `Hardware.c:251` |
| TMR0_2 | CH_B | `Timer0_Unit2` | 定时中断 | 5000µs (5ms) | CPU 中断 | `Hardware.c:252` |

> **历史遗留**: TMR0_1 CH_B 原本由 `Timer0_Unit1` 管理（500µs 中断），但 `Adc.c` 会覆盖其配置。该冲突已修复：`Hardware.c:253` 已注释掉 Timer0_Unit1 对 CH_B 的初始化。

---

## 二、ADC 初始化链路

### 2.1 调用顺序

```
main()
  Hardware_Init()                          // Hardware.c:244
    ├─ TMR0_Unit2_Init(CH_A, 1000µs)
    ├─ TMR0_Unit2_Init(CH_B, 5000µs)
    ├─ TMR0_Unit1_Init(CH_A, 1000µs)      // CH_B 已注释
    └─ AOS_Init()                          // Aos.c:21  建立硬件事件路由
  
  ESystem_Init()                           // App_Motor_Project.c:611
    ├─ RegisterAllDevices()
    │   ├─ ADC_Device_Create(CH5, PA5)     // 电流传感器 ADC
    │   ├─ DeviceManager_Register(ID_ADC_CURRENT=11)
    │   ├─ ADC_Device_Create(CH4, PA04)    // 电压 ADC
    │   └─ DeviceManager_Register(ID_ADC_VOLTAGE=12)
    │
    ├─ Device_Init(ID_ADC_CURRENT)         // 第一个 ADC 设备触发硬件初始化
    │   └─ ADC_Device_Init()               // dev_adc.c:175
    │       └─ ADC_AdpLayerInit()          // dev_adc.c:68
    │           ├─ Adc_Create()            // Adc.c:298  注册到 Adc.c 实例表
    │           ├─ GPIO_Init(PA5, ANALOG)  // 配置模拟引脚
    │           ├─ ADC_ChCmd(CH5, ENABLE)  // 使能通道
    │           ├─ Adc_Init()              // ★ 只执行一次 (s_bAdpAdcInitialized 保护)
    │           │   ├─ Adc_InitConfig()    // 初始化 ADC1, 配置所有引脚
    │           │   ├─ Timer0_Config(200)  // 配置 TMR0_1 CH_B = 200µs ★
    │           │   ├─ Adc_HardTriggerConfig()  // ADC 硬件触发使能
    │           │   └─ Adc_IrqConfig()     // 注册 EOCA 中断回调 ★
    │           └─ Adc_Start()             // 启动 TMR0_1 CH_B
    │
    └─ Device_Init(ID_ADC_VOLTAGE)         // 第二个 ADC 设备
        └─ ADC_AdpLayerInit()
            └─ Adc_EnableInterrupt()       // 仅补充使能中断（硬件已初始化）
```

### 2.2 两个 ADC 设备

| 设备 ID | 用途 | 引脚 | ADC 通道 | 上层消费者 |
|---------|------|------|----------|-----------|
| `ID_ADC_CURRENT` (11) | 电流传感器原始值 | PA5 | CH5 | `dev_sensor` |
| `ID_ADC_VOLTAGE` (12) | 母线电压原始值 | PA04 | CH4 | `dev_voltage` |

设备定义: `App_Motor_Project.h:55-56`, 创建: `App_Motor_Project.c:239-269`

---

## 三、ADC 定时器中断链路（硬件触发→EOCA 中断）

### 3.1 完整信号链路

```
TMR0_1 CH_B                           AOS 路由                  ADC1
  ┌──────────┐                    ┌─────────────┐         ┌──────────────┐
  │ 计数器    │  比较匹配(CMP_B)   │ AOS_ADC1_0  │  EVT0   │ SEQ_A        │
  │ 0→77→0   │ ────────────────→ │ ←────────── │ ──────→ │ 单次转换      │
  │ (200µs)  │                    │ src: TMR0_1 │         │ CH4 + CH5    │
  └──────────┘                    │   _CMP_B    │         └──────┬───────┘
                                  └─────────────┘                │
                                                           转换完成 EOCA
                                                                 │
                                                    ┌────────────┴───────┐
                                                    │ INT116_IRQn (116)  │
                                                    │ NVIC 优先级: 6     │
                                                    └────────┬──────────┘
                                                             │
                                                    ADC1_SeqA_IrqCallback()
                                                      ├─ GPIO_TOGGLE(PA07)
                                                      ├─ ADC_ClearStatus(EOCA)
                                                      └─ Adc_ProcessInterruptChannels()
                                                           └─ 遍历所有 interrupt 通道
                                                              读取 ADC_GetValue()
```

### 3.2 关键组件

| 组件 | 代码位置 | 说明 |
|------|----------|------|
| Timer0 比较值计算 | `Adc.c:162-175` | `Timer0_Config()` — DIV256 + 比较值 |
| AOS 路由建立 | `Aos.c:21-36` | `AOS_Init()` — `AOS_Connect(AOS_ADC1_0, EVT_SRC_TMR0_1_CMP_B)` |
| ADC 硬件触发配置 | `Adc.c:190-194` | `Adc_HardTriggerConfig()` — `ADC_TriggerConfig(SEQ_A, HARDTRIG_EVT0)` |
| EOCA 中断注册 | `Adc.c:218-226` | `Adc_IrqConfig()` — `INTC_IrqSignIn(INT_SRC_ADC1_EOCA → INT116_IRQn)` |
| EOCA 中断回调 | `Adc.c:274-285` | `ADC1_SeqA_IrqCallback()` |

---

## 四、修改采样间隔

**唯一修改位置**: `Adp/Adc.h` 第 61 行

```c
#define ADC_SAMPLE_INTERVAL_US          (200U)
//  200 = 5kHz,  500 = 2kHz,  1000 = 1kHz
```

Timer0 时钟 = PCLK1 / DIV256。以 PCLK1=100MHz 为例：

| `ADC_SAMPLE_INTERVAL_US` | 比较值 | 采样率 | 备注 |
|--------------------------|--------|--------|------|
| 100 | 38 | 10kHz | 极限，需验证不溢出 |
| **200** | **77** | **5kHz** | 当前值 |
| 500 | 194 | 2kHz | |
| 1000 | 389 | 1kHz | 原来的值 |

> 修改后重新编译烧录即生效。**不需要改任何 .c 文件**。

---

## 五、dev_adc → dev_sensor 采样数据流

```
ADC 硬件 (200µs EOCA)           dev_adc Update          dev_sensor Update
      │                              │                        │
      ├─ sample → u16LatestValue ────┤                        │
      ├─ sample → u16LatestValue     │                        │
      ├─ sample → u16LatestValue     │                        │
      ├─ sample → u16LatestValue     │                        │
      ├─ sample → u16LatestValue ────┤                        │
      │                              ├─ Adc_GetLatestValue()   │
      │                              │  → u16VoltageMv        │
      │                              │  (取最新一次)           │
      │                              │                        ├─ Device_Read(ID_ADC_CURRENT)
      │                              │                        │  → u16AdcVoltageMv
      │                              │                        ├─ Sensor_CalcCurrent()
      │                              │                        │  → (mV - 1650) × 1000 / 264
      │                              │                        └─ Sensor_CheckOvercurrent()
      │                              │                           (时间窗口模式)
```

### 5.1 Update 间隔设置

| 设备 | interval | 含义 | 代码位置 |
|------|----------|------|----------|
| `ID_ADC_CURRENT` | **0** | 每轮主循环更新 | `App_Motor_Project.c:440` |
| `ID_SENSOR_CURRENT` | **0** | 每轮主循环更新 | `App_Motor_Project.c:445` |
| `ID_ADC_VOLTAGE` | 1 | 每 1ms 更新 | `App_Motor_Project.c:441` |
| `ID_VOLTAGE_BUS` | 10 | 每 10ms 更新 | `App_Motor_Project.c:444` |

`interval=0` 含义: `DeviceManager_ShouldUpdate()` 中跳过时间判断，每轮 `DeviceManager_UpdateAll()` 都执行（`device_manager.c:37-38`）。实际更新频率取决于主循环速度。

### 5.2 过流检测机制

- **模式**: `OVERCURRENT_MODE_TIME_WINDOW` (时间窗口，非采样计数)
- **原理**: `nbDelay` 基于 `tickTimer` 真实毫秒流逝计时，与采样频率无关
- **配置寄存器**: `0x2716` (过流阈值 mA), `0x271E` (判定时间 ms)
- **代码**: `dev_sensor.c:230-303` (时间窗口逻辑), `dev_sensor.c:306-334` (统一入口)

---

## 六、PA7 调试翻转引脚

### 6.1 功能

每次 ADC EOCA 中断触发时翻转 PA07 电平，用于示波器验证采样间隔：

```
期望波形: 方波，频率 = 采样率 / 2
  200µs 采样 → 100µs 高 + 100µs 低 → 2.5kHz 方波
```

### 6.2 控制宏

`Adp/Adc.h` 第 68 行:

```c
#define ADC_DEBUG_TOGGLE_ENABLE    // 注释掉则关闭 PA7 翻转
```

### 6.3 代码位置

| 功能 | 文件 | 行号 |
|------|------|------|
| 控制宏 | `Adp/Adc.h` | 68 |
| 引脚定义 | `Adp/Adc.h` | 72-73 |
| GPIO 初始化 | `Adp/Adc.c` | 119-124 |
| 中断内翻转 | `Adp/Adc.c` | 278-282 |

### 6.4 换引脚

只改 `Adp/Adc.h`:

```c
#define ADC_DEBUG_TOGGLE_PORT           (GPIO_PORT_A)
#define ADC_DEBUG_TOGGLE_PIN            (GPIO_PIN_07)
```

---

## 七、相关源文件索引

| 文件 | 作用 |
|------|------|
| `Adp/Adc.h` | ADC 硬件驱动头文件 — 采样间隔宏、引脚宏 |
| `Adp/Adc.c` | ADC 硬件驱动 — Timer0 配置、EOCA 中断、硬件触发 |
| `Adp/Aos.c` | AOS 事件路由 — TMR0→ADC 硬件连接 |
| `Adp/Timer0_Unit1.c` | TMR0_1 中断驱动 — CH_A 1ms tick |
| `Adp/Timer0_Unit2.c` | TMR0_2 中断驱动 — CH_A 1ms, CH_B 5ms |
| `Adp/Hardware.c` | 硬件统一初始化 — `Hardware_Init()` |
| `Dev/dev_adc.c` | ADC 设备抽象层 — `ADC_Device_Init/Update` |
| `Dev/dev_sensor.c` | 电流传感器设备 — 电流计算、过流检测 |
| `Dev/dev_voltage.c` | 电压传感器设备 — 母��电压计算、过压/欠压检测 |
| `Dev/device_manager.c` | 设备管理器 — Update 调度 (`interval=0` 逻辑在:37-38) |
| `App/App_Motor_Project.c` | 应用层 — `ESystem_Init()`, 设备注册, Update 间隔设置 |
| `App/App_Motor_Project.h` | 应用层头文件 — 设备 ID 定义、引脚宏 |
