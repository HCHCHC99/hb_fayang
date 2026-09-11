# EventBus 事件拓扑说明

## 一、所有 Topic 列表

| 枚举值 | Topic 名称 | 发布者 | 订阅者 | 说明 |
|--------|-----------|--------|--------|------|
| 0 | TOPIC_POWER | dev_power | Motor_OnPowerEvent | 电源状态变化 |
| 1 | TOPIC_LIMIT_HARD | dev_hall(?) | Motor_OnHardLimit | 硬件限位开关 |
| 2 | TOPIC_LIMIT_SOFT | — | — | 软件限位（预留） |
| 3 | TOPIC_CAN_EVENT | CAN模块 | Motor_OnCANEvent | CAN总线事件（预留） |
| 4 | TOPIC_MOTOR_CMD | Modbus | Motor_OnManualIO | Modbus电机控制命令 |
| 5 | TOPIC_MOTOR_SPEED_FEEDBACK | dev_motor_hall | Motor_OnSpeedFeedback | 转速反馈 |
| 6 | TOPIC_MOTOR_DRIVE_EXEC | — | — | 驱动执行（预留） |
| 7 | TOPIC_MANUAL_IO | IO/Modbus | Motor_OnManualIO | 手动IO控制事件 |
| 8 | TOPIC_ALARM | dev_sensor | Motor_OnOvercurrent | 通用报警（已废弃） |
| 9 | TOPIC_VOLTAGE_ALARM | dev_voltage | Motor_OnVoltageAlarm, FaultHandler_OnVoltageAlarm | 电压报警 |
| 10 | TOPIC_CURRENT_ALARM | dev_sensor | RTurn_OnCurrentAlarm, FaultHandler_OnCurrentAlarm | 电流报警（过流检测） |
| 11 | TOPIC_RTURN_LIMIT | dev_rturn | Motor_OnRTurnLimit | 旋转限位事件 |
| 12 | TOPIC_FAULT_CLEAR | App_Modbus | FaultHandler_ClearFault | 故障清除命令 |
| 13 | TOPIC_MANUAL_RS485 | App_Modbus | Motor_OnManualIO | RS485手动控制 |
| 14 | **TOPIC_OVERCURRENT_FWD** | dev_rturn | **Motor_OnOvercurrentFwd** | **正转(开窗)过流 → block_fwd** |
| 15 | **TOPIC_OVERCURRENT_REV** | dev_rturn | **Motor_OnOvercurrentRev** | **反转(关窗)过流 → block_rev** |

> TOPIC_OVERCURRENT_FWD 和 TOPIC_OVERCURRENT_REV 为本次新增（2026-07-22），用于双向熔断机制。

---

## 二、核心事件流

### 2.1 过流事件流（完整链路）

```
dev_sensor 检测电流超阈值
  │
  └─ TOPIC_CURRENT_ALARM (u8IsActive=1)
       │
       ├─→ RTurn_OnCurrentAlarm (优先级1, 先执行)
       │     └─ RTurn_HandleOvercurrent
       │          ├─ 方向=FWD(开窗)
       │          │   ├─ SetFault(FAULT_BIT_OVERCURRENT_FWD)
       │          │   ├─ TOPIC_OVERCURRENT_FWD(u8IsActive=1)  ──→ Motor_OnOvercurrentFwd → block_fwd += OVERCUR_FWD
       │          │   ├─ TOPIC_OVERCURRENT_REV(u8IsActive=1)  ──→ Motor_OnOvercurrentRev → block_rev += OVERCUR_REV
       │          │   └─ TOPIC_RTURN_LIMIT(FWD, u8IsActive=1) ──→ Motor_OnRTurnLimit    → block_fwd += RTURN_FWD
       │          │
       │          └─ 方向=REV(关窗)
       │               ├─ RunAngle_TryCalibrate()
       │               │   ├─ TRUE  → 校准，不报故障
       │               │   └─ FALSE → SetFault(FAULT_BIT_OVERCURRENT_REV)
       │               │              + 双向发布(同FWD)
       │               └─ TOPIC_RTURN_LIMIT(REV, u8IsActive=1) ──→ Motor_OnRTurnLimit → block_rev += RTURN_REV
       │
       └─→ FaultHandler_OnCurrentAlarm (优先级0, 后执行)
             ├─ FWD → SetFault(FAULT_BIT_OVERCURRENT_FWD)  (冗余记录)
             └─ REV → 仅日志（RTurn 已处理）
```

### 2.2 故障清除流

```
485 写 0x2740 bit2=1
  │
  └─ App_Modbus → FaultHandler_ClearFault(FAULT_TYPE_OVERCURRENT)
       │
       ├─ Sensor_Device_ClearAlarm()
       │    └─ 清除传感器报警状态
       │
       ├─ Motor_ClearOvercurrentBlock()
       │    └─ 直接移除 block_fwd 的 DEV_ID_OVERCUR_FWD（双保险）
       │
       ├─ RealTime_ClearFault(FAULT_BIT_OVERCURRENT_FWD | FAULT_BIT_OVERCURRENT_REV)
       │    └─ 清除双向故障位
       │
       └─ 发布双向释放事件:
            ├─ TOPIC_OVERCURRENT_FWD(u8IsActive=0) → Motor_OnOvercurrentFwd → 移除 block_fwd 的 OVERCUR_FWD
            └─ TOPIC_OVERCURRENT_REV(u8IsActive=0) → Motor_OnOvercurrentRev → 移除 block_rev 的 OVERCUR_REV
```

### 2.3 RTurn 限位流

```
RTurn_UpdateAngle / RTurn_HandleOvercurrent
  │
  └─ TOPIC_RTURN_LIMIT
       ├─ u8IsActive=1 (限位触发)
       │    └─ Motor_OnRTurnLimit
       │         ├─ FWD → block_fwd += DEV_ID_RTURN_FWD, clear allow_fwd
       │         └─ REV → block_rev += DEV_ID_RTURN_REV, clear allow_rev
       │
       └─ u8IsActive=0 (限位释放: 方向切换)
            └─ Motor_OnRTurnLimit
                 ├─ FWD → block_fwd -= DEV_ID_RTURN_FWD
                 └─ REV → block_rev -= DEV_ID_RTURN_REV
```

### 2.4 电压报警流

```
dev_voltage 检测电压异常
  │
  └─ TOPIC_VOLTAGE_ALARM (u8IsActive=1)
       │
       ├─→ Motor_OnVoltageAlarm
       │     ├─ 过压 → block_fwd += OVERVOLTAGE_FWD, block_rev += OVERVOLTAGE_REV
       │     └─ 欠压 → block_fwd += UNDERVOLTAGE_FWD, block_rev += UNDERVOLTAGE_REV
       │
       └─→ FaultHandler_OnVoltageAlarm
             ├─ 过压 → SetFault(FAULT_BIT_OVERVOLTAGE)
             └─ 欠压 → SetFault(FAULT_BIT_UNDERVOLTAGE) + 保存绝对角度
```

---

## 三、订阅优先级

同一 Topic 可被多个订阅者订阅，优先级值越小越先执行：

| Topic | 订阅者 | 优先级 |
|-------|--------|:--:|
| TOPIC_CURRENT_ALARM | RTurn_OnCurrentAlarm | 1 (先) |
| TOPIC_CURRENT_ALARM | FaultHandler_OnCurrentAlarm | 0 (后) |
| TOPIC_VOLTAGE_ALARM | Motor_OnVoltageAlarm | 0 |
| TOPIC_VOLTAGE_ALARM | FaultHandler_OnVoltageAlarm | 1 |

**设计原则**：RTurn 先于 FaultHandler，确保方向锁定和校准在故障记录之前完成。

---

## 四、事件结构体

### Current_AlarmEvent_t（过流报警）

```c
typedef struct {
    int32_t   s32CurrentMa;     // 当前电流(mA)
    int32_t   s32ThresholdMa;   // 触发阈值(mA)
    uint8_t   u8IsActive;       // 1=触发, 0=释放
} Current_AlarmEvent_t;
```

使用于：TOPIC_CURRENT_ALARM, TOPIC_OVERCURRENT_FWD, TOPIC_OVERCURRENT_REV

### RTurn_LimitEvent_t（旋转限位）

```c
typedef struct {
    uint8_t   u8Direction;      // RTURN_LIMIT_FORWARD(1) / RTURN_LIMIT_REVERSE(2)
    float     fAngle;           // 触发时角度(°)
    uint8_t   u8IsActive;       // 1=触发, 0=释放
} RTurn_LimitEvent_t;
```

### Voltage_AlarmEvent_t（电压报警）

```c
typedef struct {
    uint8_t   u8AlarmType;      // VOLTAGE_ALARM_OVERVOLTAGE / VOLTAGE_ALARM_UNDERVOLTAGE
    uint32_t  u32BusVoltageMv;  // 总线电压(mV)
    uint32_t  u32ThresholdMv;   // 阈值(mV)
    uint8_t   u8IsActive;       // 1=触发, 0=释放
} Voltage_AlarmEvent_t;
```

---

## 五、数据流全景图

```
                      ┌─────────────┐
                      │  dev_sensor │  电流检测
                      └──────┬──────┘
                             │ TOPIC_CURRENT_ALARM
              ┌──────────────┼──────────────┐
              ▼              ▼              ▼
      ┌──────────┐   ┌──────────┐   ┌──────────────┐
      │dev_rturn │   │App_Fault │   │ Motor_OnCur  │
      │RTurn_On  │   │ Handler  │   │ rentAlarm    │
      │Current   │   └────┬─────┘   │ (已禁用)     │
      │Alarm     │        │         └──────────────┘
      └────┬─────┘        │
           │              │
    ┌──────┴──────┐       │
    │             │       │
    ▼             ▼       │
TOPIC_       TOPIC_        │
OVERCUR      OVERCUR       │
_FWD         _REV          │
    │             │        │
    ▼             ▼        │
Motor_On     Motor_On      │
Overcur      Overcur       │
Fwd          Rev           │
    │             │        │
    ▼             ▼        ▼
┌─────────────────────────────────┐
│       dev_motor 仲裁系统         │
│  block_fwd += OVERCUR_FWD      │
│  block_rev += OVERCUR_REV      │
│        双向锁死                  │
└─────────────────────────────────┘

           ┌──────────┐
           │dev_rturn │
           │UpdateAngle│  轮询检测
           └────┬─────┘
                │ TOPIC_RTURN_LIMIT
                ▼
           ┌──────────┐
           │dev_motor │
           │Motor_On  │
           │RTurnLimit│
           └────┬─────┘
                │
                ▼
           ┌──────────────────┐
           │ block_fwd/rev    │
           │ += RTURN_FWD/REV │
           │ 单方向锁          │
           └──────────────────┘

           ┌──────────┐
           │dev_voltage│
           └────┬─────┘
                │ TOPIC_VOLTAGE_ALARM
      ┌─────────┴─────────┐
      ▼                   ▼
Motor_OnVoltage     FaultHandler_
Alarm               OnVoltageAlarm
      │                   │
      ▼                   ▼
block_fwd/rev        SetFault /
+= OVER/UNDER        ClearFault
VOLTAGE_FWD/REV
```

---

## 六、关键事件时序

### 过流触发到电机停转

```
T+0ms:   传感器检测到过流
T+~1ms:  dev_sensor 发布 TOPIC_CURRENT_ALARM
T+~2ms:  RTurn_OnCurrentAlarm 执行
         → 方向判断、校准或报故障
         → 发布 TOPIC_OVERCURRENT_FWD/REV (如报故障)
         → 发布 TOPIC_RTURN_LIMIT
T+~3ms:  Motor_OnOvercurrentFwd/Rev 执行 → block 写入
T+~4ms:  Motor_OnRTurnLimit 执行 → block 写入
T+~50ms: Motor_Update 触发仲裁 → 停转
```

> 实际停转延迟取决于仲裁周期（50ms），事件回调在微秒级完成。

### 故障清除到电机恢复

```
T+0ms:   485 写 0x2740
T+~1ms:  FaultHandler_ClearFault 执行
         → 清除故障位
         → 发布 TOPIC_OVERCURRENT_FWD/REV(u8IsActive=0)
T+~2ms:  Motor_OnOvercurrentFwd/Rev 执行 → block 移除
T+~50ms: Motor_Update 触发仲裁 → 可响应新指令
```
