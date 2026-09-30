#ifndef __APP_MOTION_TIMEOUT_H__
#define __APP_MOTION_TIMEOUT_H__

#include <stdint.h>
#include <stdbool.h>

/*=============================================================================
 * 控制超时故障检测 (FAULT_BIT_MOTION_TIMEOUT, bit7)
 *
 * 状态机: IDLE --Start()--> RUNNING --到点仍未停转--> Trip
 *                          \--Cancel()--> IDLE
 *
 * 说明:
 *  - 计时起点 = 指令被受理(门禁通过、发布运动事件)时刻, 起算方式为
 *    deadline = now + timeout_ms; 不采用"首次观测到在转才计时"。
 *  - 到点判定: armed && now >= deadline && !Param_IsMotorStopped() -> 触发;
 *    若到点时电机已停转(到位/堵转/极限)则视为完成, 自动解除不触发。
 *  - Trip 内部置位 bit7 + 双向锁定(急停), 并自动 disarm 避免重复触发。
 *  - 1ms 节拍调用 MotionTimeout_Update()。
 *============================================================================*/

/**
 * @brief  初始化超时看门狗(清零状态)
 */
void MotionTimeout_Init(void);

/**
 * @brief  启动一次超时检测(受理指令时调用)
 * @param  timeout_ms  超时时长(ms); 0 = 受理瞬间即判定(不延时)
 * @note   会覆盖当前正在进行的检测(新指令 = 取消旧的 + 按新参数重新起算)
 *         是否"关闭检测"由调用方决定是否调用本函数, 本函数不再用 0 表示关闭
 */
void MotionTimeout_Start(uint32_t timeout_ms);

/**
 * @brief  取消当前超时检测(停转/急停/上锁/清除故障/新指令时调用)
 */
void MotionTimeout_Cancel(void);

/**
 * @brief  1ms 节拍更新: 到点且仍未停转则触发控制超时故障
 */
void MotionTimeout_Update(void);

/**
 * @brief  查询当前是否处于超时检测中
 * @retval true   检测进行中
 * @retval false  空闲
 */
bool MotionTimeout_IsActive(void);

#endif /* __APP_MOTION_TIMEOUT_H__ */