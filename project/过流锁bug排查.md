# 过流锁 bug 排查

## 现象

读故障寄存器 0x2740 = 0x0000（无过流故障），但正转和反转指令均无法转动电机。

## 方向锁机制（当前版本：已修复双向熔断）

```
block_fwd: DEV_ID_IO_FWD, DEV_ID_OVERCUR_FWD, DEV_ID_RTURN_FWD
block_rev: DEV_ID_IO_REV, DEV_ID_OVERCUR_REV, DEV_ID_RTURN_REV
```

### 锁触发机制

| 锁 | 队列 | 触发 | 清除 |
|----|------|------|------|
| DEV_ID_IO_FWD | block_fwd | 初始化 / IO STOP 指令 | IO FWD 指令 |
| DEV_ID_IO_REV | block_rev | 初始化 / IO STOP 指令 | IO REV 指令 |
| DEV_ID_OVERCUR_FWD | block_fwd | FWD过流故障 → TOPIC_OVERCURRENT_FWD(u8IsActive=1) | 手动清故障 → u8IsActive=0 |
| DEV_ID_OVERCUR_REV | block_rev | REV过流故障 → TOPIC_OVERCURRENT_REV(u8IsActive=1) | 手动清故障 → u8IsActive=0 |
| DEV_ID_RTURN_FWD | block_fwd | RTurn 发布 RTURN_LIMIT_FORWARD(u8IsActive=1) | 方向切换为 REV 时 RTurn 发布 u8IsActive=0 释放 |
| DEV_ID_RTURN_REV | block_rev | RTurn 发布 RTURN_LIMIT_REVERSE(u8IsActive=1) | 方向切换为 FWD 时 RTurn 发布 u8IsActive=0 释放 |

### 设计变更说明

1. **故障位拆分**：FAULT_BIT_OVERCURRENT 拆为 FAULT_BIT_OVERCURRENT_FWD(bit1) 和 FAULT_BIT_OVERCURRENT_REV(bit2)
2. **双向熔断**：无论正转或反转过流故障，同时封锁两个方向
3. **Motor_OnCurrentAlarm 已禁用**：RTurn 直接发布 TOPIC_OVERCURRENT_FWD/REV，不再依赖 dev_sensor 事件
4. **SetAllow block 检查**：block 非空时拒绝 allow 写入，防止僵尸指令

---

## 复现路径分析

### 场景 A：关窗过流在阈值内（正常关窗到位）— 已修复

```
1. 电机在关窗极限位置附近，角度在 [-3°, -1°] 内
   → 发 REV → 电机反转（轻微动一下就堵转）
   → 过流检测 → TOPIC_CURRENT_ALARM

2. RTurn_HandleOvercurrent: 方向=REV
   → 角度在阈值内 → RunAngle_TryCalibrate()=true → 校准
   → 发布 RTURN_LIMIT_REVERSE(u8IsActive=1)
   → 不设故障位，不发布双向过流事件
   → Motor_OnRTurnLimit: block_rev += DEV_ID_RTURN_REV

3. 结果:
   → block_fwd: 空（或 IO_FWD 默认锁，IO FWD 指令可解除）
   → block_rev: DEV_ID_RTURN_REV
   → fault=0x0000

   仅 REV 被锁。发 FWD 可转（方向切换释放 RTURN_REV 锁）。
   此为正常行为：关窗到位后单向锁反转，允许开窗。
```

### 场景 B：关窗过流在阈值外（异物卡住）— 已修复（双向熔断）

```
1. 电机角度在阈值外，发 REV → 关窗过流
   → RTurn_HandleOvercurrent: 方向=REV
   → 角度在阈值外 → RunAngle_TryCalibrate()=false
   → RealTime_SetFault(FAULT_BIT_OVERCURRENT_REV)
   → RTurn_PublishBidirectionalOvercurrent(1):
       TOPIC_OVERCURRENT_FWD(u8IsActive=1) → block_fwd += DEV_ID_OVERCUR_FWD
       TOPIC_OVERCURRENT_REV(u8IsActive=1) → block_rev += DEV_ID_OVERCUR_REV
   → 发布 RTURN_LIMIT_REV → block_rev += DEV_ID_RTURN_REV

3. 结果: fault=bit2=1, 双向锁死
   → 必须通过 485 清除故障才能恢复
```

### 场景 C：开窗过流 — 已修复（双向熔断）

```
1. 发 FWD → 电机正转 → 异物卡住 → 过流
   → RTurn_HandleOvercurrent: 方向=FWD
   → RealTime_SetFault(FAULT_BIT_OVERCURRENT_FWD)
   → RTurn_PublishBidirectionalOvercurrent(1):
       block_fwd += DEV_ID_OVERCUR_FWD
       block_rev += DEV_ID_OVERCUR_REV
   → 发布 RTURN_LIMIT_FWD → block_fwd += DEV_ID_RTURN_FWD

3. 结果: fault=bit1=1, 双向锁死
```

### 场景 D：故障清除后的僵尸指令 — 已修复（SetAllow block检查）

```
1. 过流故障 → 双向锁死 (block_fwd 和 block_rev 均有数据)

2. 用户发送 IO 指令（如 FWD）
   → Motor_OnManualIO: Remove IO_FWD from block_fwd
   → SetAllow(allow_fwd, block_fwd, IO_FWD)
   → block_fwd 仍有 OVERCUR_FWD/RTURN_FWD → count > 0 → 拒绝！
   → allow_fwd 不写入僵尸指令

3. 用户清除故障
   → block_fwd -= OVERCUR_FWD, block_rev -= OVERCUR_REV
   → 方向切换可清除 RTURN 锁
   → allow 队列干净，不会自动恢复

   不会再出现"清完故障后僵尸指令自动恢复"的问题。
```

## 结论

**原 bug "两向全锁 + fault=0" 已通过以下修改根除：**

1. **故障位拆分**：正转/反转过流各自有独立故障位，不再混淆
2. **双向熔断**：任何方向故障 → 两个方向同时封锁 → 清除时双向同时释放
3. **SetAllow block 检查**：block 非空时禁止 allow 写入，消除僵尸指令
4. **Motor_OnCurrentAlarm 禁用**：简化过流处理链路，统一由 RTurn 分发

调试建议：如仍遇到锁死问题，通过 RTT 日志查看：
1. `Motor_ArbitrationDecision` — block_fwd/block_rev 队列内容
2. `[RTurn]` — 锁触发和释放时机
3. `[SET_ALLOW]` — SetAllow 被拒绝的情况
4. `0x2740` 故障寄存器 — 确认哪个故障位被置位
