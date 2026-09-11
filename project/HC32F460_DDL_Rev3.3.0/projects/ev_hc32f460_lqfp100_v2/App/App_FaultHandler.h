#ifndef __APP_FAULT_HANDLER_H__
#define __APP_FAULT_HANDLER_H__

#include <stdint.h>
#include <stdbool.h>

/*=============================================================================
 * 故障处理器初始化
 * 订阅电压、电流等相关事件，更新 App_Params 中的故障状态
 *============================================================================*/

/**
 * @brief  故障清除事件结构体（用于 EventBus 发布 TOPIC_FAULT_CLEAR）
 */
typedef struct {
    uint8_t  u8FaultType;       // 故障类型：0=过压, 1=欠压, 2=过流等
    uint32_t u32Timestamp;      // 清除时间戳
} FaultClearEvent_t;

// 故障类型常量
#define FAULT_TYPE_OVERVOLTAGE      0
#define FAULT_TYPE_UNDERVOLTAGE     1
#define FAULT_TYPE_OVERCURRENT      2

/**
 * @brief  初始化故障处理器
 * @note   在系统初始化阶段调用（如 main 函数中）
 */
void FaultHandler_Init(void);

/**
 * @brief  故障清除回调 - 由 Modbus 写 REG_FAULT_STATUS 触发
 * @param  u8FaultType 故障类型（FAULT_TYPE_OVERVOLTAGE / FAULT_TYPE_UNDERVOLTAGE 等）
 * @note   仅在 VOLTAGE_CLEAR_MODE == VOLTAGE_CLEAR_MANUAL 时使用
 *         此函数会：
 *         1. 调用 Voltage_Device_ClearAlarm() 清除电压告警状态
 *         2. 调用 Motor_ClearVoltageBlock() 移除电机仲裁器中的 block 指令
 */
void FaultHandler_ClearFault(uint8_t u8FaultType);

#endif /* __APP_FAULT_HANDLER_H__ */
