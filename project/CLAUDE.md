# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

HB_chuchai — 基于 HC32F460 (Cortex-M4) 的直流电机推窗控制系统，RS485/Modbus RTU 通信。

**当前代码状态：原板版本**，通过 `main.h` 中 `BOARD_VERSION = 0` 控制。

**源代码根目录**：`HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2/`（下文路径均为该目录下的相对路径）。

## 硬件板本切换

`template/source/main.h` 第30行：
```c
#define BOARD_VERSION   0   // 0=原HB_chuchai板(当前), 1=整合板
```

该宏控制以下所有差异（无需单独修改各模块）：
- 电机控制: GPIO(0) / PWM(1)
- 电流传感器: 霍尔(0) / 差分运放(1)
- 分压电阻: 110k(0) / 150k(1)，下分压电阻固定 10k，分压比 = (top+bottom)/bottom
- 电压ADC: PA04(0) / PA06(1)
- RS485 DIR: PB14(0) / PA3(1)

## 编译/烧录

- IDE: Keil MDK V5.06
- 主工程文件: `template/MDK/template - 副本.uvprojx`（备用: `template - 副本 - 副本.uvprojx`）
- 芯片包: HC32F460PETB
- 无 CI/自动化构建 — 编译在 Keil IDE 内手动完成

## 代码架构

### Adp 层 - 硬件适配层
直接操作 MCU 外设寄存器，与 HC32F460 DDL 及 CMSIS 交互。
- `Pwm.c/h`, `Adc.c/h`, `Gpio_io.c/h` — PWM/ADC/GPIO 驱动
- `Motor_hall.c/h` — 霍尔传感器 GPIO 中断 + 转速/方向计算 + 脉冲累积（`g_s32HallPulseAccum`）
- `rs485.c/h` — USART + RS485 DIR 引脚控制
- `Dma.c/h`, `Sysclk.c/h`, `Timer0_Unit1/2.c/h`, `hc32f46x_flash.c/h`, `timer6_timebase.c/h` — 系统外设
- `Adapter.c/h`, `Hardware.c/h`, `Aos.c/h` — 硬件初始化/适配封装
- `Template_Pwm.c/h` — 模板 PWM（与主 PWM 独立，可能用于测试）

### Dev 层 - 设备抽象层
每个设备封装为 `DeviceNode_t` 注册到 DeviceManager，通过 EventBus 解耦通信。
- `dev_motor.c/h` — 电机仲裁器（占空比缓升/缓降、方向切换 pending 机制）
- `dev_rturn.c/h` — 旋转角度追踪（**Pulse-Direct 方案**，过流→重新校准角度，含方向锁定 LockState 机制）
- `dev_sensor.c/h` — 电流传感器（窗口滤波 + 滞回，手动清除模式）
- `dev_voltage.c/h` — 电压监控（过压/欠压，含 400mV 补偿，手动清除模式）
- `EventBus.c/h` — 发布/订阅事件总线（Topic 优先级队列）
- `device_manager.c/h` — 设备注册/调度/互斥
- `dev_motor_hall.c/h`, `dev_adc.c/h`, `dev_pwm.c/h`, `dev_power.c/h`, `dev_hall.c/h`, `dev_io.c/h`

### App 层 - 应用协调层
- `App_Motor_Project.c/h` — 系统初始化(ESystem_Init)、主循环(ESystem_MainLoop)、热重载(App_ReloadConfig)、仿真模式
- `App_Modbus.c/h` — Modbus RTU 协议处理，CRC16，帧解析（#if MOTOR_CONTROL_MODE == 1 时也处理 PWM 控制）
- `App_FaultHandler.c/h` — 故障聚合与清除（订阅 EventBus）

### Utils 层 - 基础设施
- `App_Params.c/h` — Flash 参数持久化 + Modbus 寄存器读写映射 + 实时数据结构
- `App_RunAngle.c/h` — 绝对角度追踪（独立于 dev_rturn，掉电保留零点，欠压自动保存，回零逻辑）
- `param_manager.c/h` — Flash 读写引擎（磨损均衡，支持多实例：常规参数用扇区56-62，绝对角度用扇区55）
- `param_validator.c/h` — Modbus 写入值校验（限幅 + 精度取整）
- `rtt_manager.c/h` — RTT 日志 + 模块级调试开关
- `TickTimer.c/h`, `msg_queue.c/h`, `ring_buf.c/h`, `lock.c/h`

## 关键实现细节

### 角度获取: Pulse-Direct 方案 (方案 C)
- `dev_rturn.c`: 角度不用 RPM 积分，改为 `fCurrentAngle = fMinAngle + g_s32HallPulseAccum * 360 / 12 / ratio`
- RPM 计算链路保留不动（用于转速显示和堵转检测）
- 开窗限位不压制角度（保留真实超调值），关窗限位保持硬下限
- 过流校准时同步复位 `g_s32HallPulseAccum = 0`

### 霍尔脉冲累计
- `Motor_hall.c`: `last_total` 和 `first_run` 从 static 局部变量改为实例成员
- 累加器每次 update (1ms) 都运行，不再依赖 1 秒定时器
- 方向为 STOP/NONE 时回退到 `last_valid_direction`
- 脉冲数计算公式: `CALC_PULSES_PER_REV = pole_pairs × hall_count × 2`

### PWM 电机控制 (MOTOR_CONTROL_MODE)
- `MOTOR_CONTROL_MODE` 在 `Dev/dev_motor.h:32-38` 中由 `BOARD_VERSION` 决定：原板=0 (GPIO), 整合板=1 (PWM)
- 当前 `BOARD_VERSION = 0` → `MOTOR_CONTROL_MODE = 0`（GPIO 控制，非 PWM）
- `main.c` 中 `Motor_Pwm_Init()` 调用被注释掉；主循环中 PWM_Update 由 `#if MOTOR_CONTROL_MODE == 1` 守卫 — 当前未启用
- PWM 配置: TMRA_4, PB6-PB9, 4通道, 20kHz, 低有效 (active LOW)
- 停止使用 98% duty（刹车），旧版用 50% 交替极性

### 预驱芯片 SDH21263 真值表

SDH21263 是 3 相 BLDC 预驱，本项目用其中 2 个半桥组成 H 桥驱动直流有刷电机。PWM 频率 20kHz，低有效。

| MCU 引脚 | PWM 通道 | SDH21263 输入 | 半桥 |
|----------|---------|---------------|------|
| PB6 | TMRA_4 CH1 | HIN1/LIN1 | Phase U |
| PB7 | TMRA_4 CH2 | HIN2/LIN2 | Phase V |
| PB8 | TMRA_4 CH3 | HIN3/LIN3 | Phase W |
| PB9 | TMRA_4 CH4 | EN 或互补输入 | — |

| 模式 | CH1 (PB6) | CH2 (PB7) | CH3 (PB8) | CH4 (PB9) | 电机 |
|------|-----------|-----------|-----------|-----------|------|
| 正转 (开窗) | 低 duty | 低 duty | 高 duty | 高 duty | FWD |
| 反转 (关窗) | 高 duty | 高 duty | 低 duty | 低 duty | REV |
| 停止 (刹车) | 98% | 98% | 98% | 98% | Brake |

### 过流/过压清除模式
- 电流传感器: `OVERCURRENT_CLEAR_MODE = OVERCURRENT_CLEAR_MANUAL`（通过 Modbus REG_FAULT_STATUS 清除）
- 电压监控: `VOLTAGE_CLEAR_MODE = VOLTAGE_CLEAR_MANUAL`（同上）
- 电压补偿: `VOLTAGE_COMPENSATION_MV = 400`（补偿外部二极管压降等）

### Modbus 参数校验
- `Utils/param_validator.c/h`: 独立校验模块
- 四个寄存器有校验规则：
  - 过压阈值 0x2714: 25.0~27.0V, 步进 0.2V
  - 欠压阈值 0x2715: 21.0~23.0V, 步进 0.2V
  - 过流阈值 0x2716: 0~2300mA, 步进 50mA
  - 过流判定时间 0x271E: 0~2000ms, 步进 20ms
- 减速比 0x3712: 1.0~6553.5 (单位 0.1)
- 极对数 0x3713: 1~100
- 流程: CRC 校验 → 最大最小值限幅 → 精度取整 → 写入 Flash → 回令

### 绝对角度 (App_RunAngle)
- `Utils/App_RunAngle.c/h`: 独立于 `dev_rturn` 的角度追踪模块
- 核心公式: `delta = g_s32HallPulseAccum(当前) - s_last_accum(上次)` → `s_abs_offset_x10 += delta × 3600 / (极对数 × 霍尔数 × 2 × 减速比)`
- Flash 存储: 扇区55，使用 `param_manager` 多实例引擎（独立于常规参数扇区56-62），魔数 `0xAB5AAB5A`/`0xBA5ABA5A`
- 欠压自动保存: `App_FaultHandler` 检测到欠压时自动调用 `RunAngle_Cmd(ABS_CMD_SAVE)` 将 RAM 偏移写入 Flash
- 回零逻辑: 偏移 > +阈值 → REV, 偏移 < -阈值 → FWD, |偏移| ≤ 阈值 → ESTOP；偏移符号翻转也触发 ESTOP
- 与实时角度 (0x2731) 独立运行、互不影响

### 减速比精度
- 寄存器 0x3712 单位改为 0.1 (值 ×10)，原值 1183 对应新值 11830
- 硬编码默认值: `RTURN_REDUCTION_RATIO = 11830.0f`，`PARAM_DEFAULT_RTURN_REDUCTION_RATIO = 11830`
- 硬编码角度限位: `RTURN_MAX_ANGLE = 88.0°`, `RTURN_MIN_ANGLE = -2.0°`

## Modbus 寄存器地址分区

| 地址范围 | 用途 | 读写 |
|---------|------|------|
| 0x2700-0x271F | 可配置参数 (Flash 持久化) | R/W (0x03/0x06) |
| 0x2720 | 控制命令 | W (0x06) |
| 0x2730-0x273F | 实时数据 (RAM, 只读) | R (0x03) |
| 0x2740 | 故障状态 | R/W |
| 0x3700-0x371F | 高级参数 (Flash 持久化) | R/W (0x03/0x06) |
| → 0x3716-0x371B | 绝对角度 (独立Flash扇区55) | 混合读写 |

### 关键寄存器

| 地址 | 名称 | 单位/范围 |
|------|------|-----------|
| 0x2710 | 设备地址 | 1~247 |
| 0x2711 | 目标转速 | r/min |
| 0x2712 | 目标角度 | 0.1° |
| 0x2714 | 过压阈值 | 0.1V |
| 0x2715 | 欠压阈值 | 0.1V |
| 0x2716 | 过流阈值 | 1mA |
| 0x271C | 关窗极限角度 | 0.1° |
| 0x271D | 开窗极限角度 | 0.1° |
| 0x271E | 过流判定时间 | 1ms |
| 0x2720 | 控制命令 | bit0=启动, bit1=停止, bit2=急停, bit4=正转, bit5=反转, bit6=保存绝对位置 |
| 0x2730 | 实时转速 | r/min |
| 0x2731 | 实时角度 | 0.1° |
| 0x2732 | 实时电压 | 0.1V |
| 0x2733 | 实时电流 | 1mA |
| 0x2737 | 实时方向 | — |
| 0x2740 | 故障状态 | bit0=过压, bit1=过流, bit2=过热, bit3=复位, bit4=过载, bit5=堵转, bit6=欠压 |
| 0x3710 | 霍尔方向 | 0=正向, 1=反向 |
| 0x3711 | 电机方向 | 0=正向, 1=反向 |
| 0x3712 | 减速比 | ×0.1 |
| 0x3713 | 极对数 | 1~100 |
| 0x3714-0x3715 | 霍尔脉冲累计 | 低16位/高16位 (int32) |
| 0x3716-0x3717 | 绝对角度 RAM 偏移 | 低16位/高16位 (int32, 0.1°) |
| 0x3718-0x3719 | 绝对角度 Flash 偏移 | 低16位/高16位 (int32, 0.1°) |
| 0x371A | 绝对角度指令 | W: 0=设零点, 1=保存到Flash, 2=回到零点 |
| 0x371B | 回零阈值 | R/W (0.1°, 默认1, 范围0~200) |

## 初始化顺序

```
main():
  Hardware_Init() → RS485_Init() → Modbus_Init() → ESystem_Init() → FaultHandler_Init() → EventBus_Enable()

ESystem_Init() 内部:
  EventBus_Init → DeviceManager_Init → RegisterAllDevices → SetupEventBusSubscriptions
  → Device_Init(ID_MOTOR)  # 电机先于其他设备初始化
  → Device_Init(其余设备)
  → SetDeviceUpdateIntervals → DeviceManager_EnableAllUpdate → RunAngle_Init()
```

## EventBus 核心主题

| Topic | 发布者 | 订阅者 | 用途 |
|-------|--------|--------|------|
| TOPIC_CURRENT_ALARM | dev_sensor | dev_motor, dev_rturn, App_FaultHandler | 过流报警/解除 |
| TOPIC_VOLTAGE_ALARM | dev_voltage | dev_motor, App_FaultHandler | 过压/欠压 → 欠压时自动触发 RunAngle_Cmd(SAVE) |
| TOPIC_RTURN_LIMIT | dev_rturn | dev_motor | 角度限位 → 电机阻塞 |
| TOPIC_MOTOR_SPEED_FEEDBACK | dev_motor_hall | dev_rturn, dev_motor | 转速反馈 |
| TOPIC_MANUAL_RS485 | App_Modbus/App_Params | dev_motor | RS485 手动控制 |

未使用/预留 Topic: `TOPIC_FAULT_CLEAR`（故障清除通过直接函数调用实现）、`TOPIC_CAN_EVENT`、`TOPIC_MOTOR_CMD`、`TOPIC_MOTOR_DRIVE_EXEC`、`TOPIC_ALARM`

## 热重载 (App_ReloadConfig)

`App_ReloadConfig()` 在 Modbus 参数写入后调用，热重载以下运行时配置（无需复位）：
- 电压阈值: 过压/欠压 + 滞回 + 触发次数
- 电流阈值: 过流阈值 + 触发/释放窗口 + 滞回
- 减速比和角度限位 (更新 dev_rturn)
- 电机霍尔极对数
- 回零阈值 (0x371B) 写入后立即生效

## 仿真模式

`App_Motor_Project.h` 中 `ENABLE_SIMULATION_MODE` 默认为 1（开启）。
- 开启时：`Sim_ProcessInput()` / `Sim_PublishEvents()` 生成模拟传感器数据
- 关闭时（=0）：使用真实硬件 ADC/GPIO
- 模拟数据通过全局 `SystemSim_t g_sim` 控制

## 调试日志

通过 `rtt_manager.h` 中的宏控制模块级调试输出（SEGGER RTT 通道 0）。

**当前已启用的调试开关**（其他均注释掉）：

| 宏 | 控制的模块 |
|----|-----------|
| DEV_SENSOR | 电流传感器（高频） |
| DEV_SENSOR_REAL | 电流传感器真实模式 |
| DEV_SENSOR_SLOW | 电流传感器（低频） |
| DEBUG_SENSOR_SLOW | 电流传感器低频详细调试 |

### 所有可用调试开关

| 宏 | 控制的模块 |
|----|-----------|
| ADP_CLOCK_DEBUG | Sysclk 时钟调试 |
| ADP_RS485_DEBUG / _FRAME / _WARN / _ERR | RS485 通信 |
| ADP_DMA_DEBUG | DMA 传输 |
| ADC_Adp_DEBUG | ADC 驱动 |
| ADP_FLASH_DEBUG | Flash 操作 |
| APP_MODBUS_INIT_DBG / _POLL / _RX / _PARSE / _CRC / _AUTO | Modbus 协议 |
| APP_FAULT_HANDLER_DBG | 故障处理 |
| DEV_EVENT_BUS / DEV_EVENT_BUS_VERBOSE | EventBus |
| DEV_RTURN | 旋转追踪 |
| DEV_POWER | 电源 |
| DEV_MOTOR | 电机仲裁器 |
| DEV_HALL | 霍尔传感器 |
| DEV_VOLTAGE | 电压监控 |
| DEV_ADC | ADC |
| DEV_MOTOR_HALL / DEV_MOTOR_HALL_OUTPUT | 电机霍尔 |
| UTILS_RING_BUF | 环形缓冲区 |
| PARAM_DEBUG | 参数管理 |
| APP_PARAMS_DBG | 参数读写 (App_Params) |
| QUEUE_INIT/SEND/RECV/STATS_PRINT | 消息队列 |
| LOCK_INIT/TRY/LOCK/UNLOCK_PRINT | 锁操作 |

使用 `INTERVAL_DECLARE(name, interval_ms)` 宏创建间隔打印器，-1 表示禁用。

## 辅助工具

- `modbus_tool.py` / `dist/modbus_tool.exe` — 交互式 Modbus 指令生成器 v4.1（含绝对角度）
  - 主菜单 7: 绝对角度（读RAM/Flash偏移、保存、回零、阈值设置）
  - 主菜单 8: 开发者选项（密码 5858），含设定绝对零点
- `modbus_test_cmds.py` — 脚本式 Modbus 指令生成器
- `实时数据使用说明.md` — 实时数据 API 使用指南
- `电流控制逻辑说明.md` — 过流检测三层处理详细时序
- `电机霍尔方案.md` — 角度计算方案分析（含 ABC 方案对比、Pulse-Direct 实现总结）
- `电机霍尔脉冲.md` — 双链路（Path 1 角度 + Path 2 脉冲）架构说明
- `绝对角度.md` — 绝对角度功能完整说明（数据模型、寄存器、操作指南、回零逻辑）
- `chuchai vs HandB.md` — BOARD_VERSION 0/1 硬件差异详细对比（ADC引脚、电机控制、分压电阻、电流传感器、RS485 DIR）

## 已知问题/注意事项

1. **文件编码混杂** — `dev_motor.c/h`, `dev_rturn.c/h`, `dev_voltage.c/h`, `App_Modbus.c` 为 UTF-16LE 编码；`param_validator.c` 为 UTF-8；部分 Adp 文件为 GB2312/GBK。编辑前务必确认编码，建议用支持 BOM 检测的编辑器。
2. 新增 Flash 字段后需 `Modbus_Init` 检测并强制保存默认值
3. PWM 停止使用 98% 交替极性，预驱芯片 SDH21263 需确认刹车效果
4. 减速比改为 0.1 精度后，已有设备 Flash 中的旧值需重新写入
5. `Motor_Pwm_Init()` 在 main.c 中被注释，PWM 更新由 `MOTOR_CONTROL_MODE` 宏守卫
6. 当前 `BOARD_VERSION = 0`（原板），如需切换到整合板改为 1
7. 心跳包功能尚未实现

## /init 自动阅读清单

执行 /init 或开始新会话时，应阅读以下关键代码以理解当前状态：

### 参数 Flash 存储
- `Utils/App_Params.h` — 寄存器地址宏、默认值宏、AppParamRecord_t/AppRealTimeData_t 结构体
- `Utils/App_Params.c` — Param_ReadByReg/Param_WriteByReg 读写分发
- `Utils/param_manager.h` — Flash 读写引擎接口
- `Utils/param_validator.h` — 参数校验规则

### 角度 & 转速获取
- `Adp/Motor_hall.c` — 霍尔中断、脉冲计数、RPM 计算、g_s32HallPulseAccum 累加
- `Adp/Motor_hall.h` — motor_hall_config_t 结构体、方向枚举
- `Dev/dev_motor_hall.c` — MotorHall 设备封装
- `Dev/dev_rturn.c` — 角度计算（Pulse-Direct）、限位检测、过流处理
- `Dev/dev_rturn.h` — RTurn_Device_t 结构体

### 电流传感器
- `Dev/dev_sensor.h` — Sensor_Device_t、传感器类型选择、过流模式、校准
- `Dev/dev_sensor.c` — 电流计算、过流检测（计数/时间窗口两种模式）、EventBus 发布

### 电压传感器
- `Dev/dev_voltage.h` — Voltage_Device_t、分压比配置、过压/欠压阈值
- `Dev/dev_voltage.c` — 电压计算、过压/欠压检测、EventBus 发布

### 系统入口
- `template/source/main.c` — 初始化顺序、主循环
- `template/source/main.h` — BOARD_VERSION

### 绝对角度
- `Utils/App_RunAngle.h` — 绝对角度接口、AbsAngleRecord_t 结构体、指令宏
- `Utils/App_RunAngle.c` — 脉冲累积追踪、设零/保存/回零、欠压自动保存
