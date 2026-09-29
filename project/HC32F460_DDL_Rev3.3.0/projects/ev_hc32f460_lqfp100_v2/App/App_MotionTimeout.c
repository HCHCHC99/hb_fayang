/**
 *******************************************************************************
 * @file  App_MotionTimeout.c
 * @brief 控制超时故障检测看门狗 (FAULT_BIT_MOTION_TIMEOUT, bit7)
 *
 *  计时起点 = 指令被受理(门禁通过、发布运动事件)时刻。
 *  到点若电机仍在转(!Param_IsMotorStopped()) -> 报控制超时并双向锁定。
 *  到点若已停转(到位/堵转/极限) -> 视为完成, 自动解除不触发。
 *******************************************************************************
 */

#include "App_MotionTimeout.h"
#include "App_Params.h"         /* RealTime_SetFault, Param_IsMotorStopped, FAULT_BIT_xxx */
#include "dev_motor.h"          /* Motor_OnMotionTimeout */
#include "TickTimer.h"
#include "rtt_manager.h"        /* MAIN_D */

/*=============================================================================
 * RAM 状态
 *============================================================================*/
static bool     s_armed    = false;     /* 检测是否已启动 */
static uint32_t s_deadline = 0U;        /* 到点时刻(ms) */

/*=============================================================================
 * Public API
 *============================================================================*/

void MotionTimeout_Init(void)
{
    s_armed    = false;
    s_deadline = 0U;
}

void MotionTimeout_Start(uint32_t timeout_ms)
{
    if (timeout_ms == 0U)
    {
        /* timeout=0 表示关闭本次检测 */
        return;
    }

    s_deadline = (uint32_t)tickTimer_GetCount() + timeout_ms;
    s_armed    = true;

    MAIN_D("[MTOUT] Start: timeout=%lu ms, deadline=%lu\r\n",
           (unsigned long)timeout_ms, (unsigned long)s_deadline);
}

void MotionTimeout_Cancel(void)
{
    if (s_armed)
    {
        s_armed = false;
        MAIN_D("[MTOUT] Cancel\r\n");
    }
}

bool MotionTimeout_IsActive(void)
{
    return s_armed;
}

void MotionTimeout_Update(void)
{
    if (!s_armed)
    {
        return;
    }

    /* 未到点: 直接返回 (有符号比较, 兼容 32 位回绕) */
    if ((int32_t)((uint32_t)tickTimer_GetCount() - s_deadline) < 0)
    {
        return;
    }

    /* 到达截止时刻: 做一次判定 */
    if (Param_IsMotorStopped())
    {
        /* 已停转(到位/堵转/极限) -> 视为完成, 自动解除 */
        s_armed = false;
        MAIN_D("[MTOUT] Deadline reached but motor stopped, disarm\r\n");
        return;
    }

    /* 仍在转: 报控制超时故障 + 急停双向锁定; trip 后自动 disarm 避免重复 */
    s_armed = false;
    RealTime_SetFault(FAULT_BIT_MOTION_TIMEOUT);
    Motor_OnMotionTimeout();

    MAIN_D("[MTOUT] TRIP! motor still running at deadline, set fault bit7 + block both dirs\r\n");
}

/*******************************************************************************
 * EOF
 ******************************************************************************/