#include "dev_rturn.h"
#include "TickTimer.h"
#include "rtt_log.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "dev_sensor.h"
#include "dev_motor.h"
#include "App_Params.h"   /* g_s32HallPulseAccum for pulse-direct angle */
#include "App_RunAngle.h"

/* 双向过流锁事件发布辅助: 故障位触发时同时封锁FWD和REV */
static void RTurn_PublishBidirectionalOvercurrent(uint8_t u8IsActive) {
    Current_AlarmEvent_t stcEvent;
    stcEvent.s32CurrentMa = 0;
    stcEvent.s32ThresholdMa = 0;
    stcEvent.u8IsActive = u8IsActive;
    EventBus_Publish(TOPIC_OVERCURRENT_FWD, &stcEvent);
    EventBus_Publish(TOPIC_OVERCURRENT_REV, &stcEvent);
}

static uint32_t s_u32LastLockPrintTime = 0;
static uint32_t s_u32LastLockDebugPrintTime = 0;
static uint32_t s_u32LastLimitPrintTime = 0;
#define LOCK_PRINT_INTERVAL_MS         2000
#define LOCK_DEBUG_PRINT_INTERVAL_MS   4000
#define LIMIT_PRINT_INTERVAL_MS        4000

// ========== Keil Watch锟斤拷锟斤拷全锟街憋拷锟斤拷 ==========
volatile float    g_fDbgRTurnAngle       = 0.0f;
volatile float    g_fDbgRTurnSpeed       = 0.0f;
volatile uint8_t  g_u8DbgRTurnDir        = 0;
volatile uint8_t  g_u8DbgRTurnDesiredDir = 0;
volatile uint8_t  g_u8DbgRTurnLockedDir  = 0;
volatile uint8_t  g_u8DbgRTurnLockActive = 0;
volatile uint8_t  g_u8DbgRTurnCalibrated = 0;
volatile uint8_t  g_u8DbgRTurnLimitTrig  = 0;

// ========== 锟节诧拷锟斤拷锟斤拷锟斤拷锟斤拷 ==========
static DeviceResult_t RTurn_GetDesiredDirection(RTurn_Device_t* pstcDev, uint8_t* pu8MotorDir) {
    if (!pstcDev || !pu8MotorDir) return RESULT_PARAM_ERR;
    
    Motor_StateInfo_t stcMotorState;
    DeviceResult_t res = Device_Read(pstcDev->stcConfig.u8MotorArbiterDevId, 
                                      &stcMotorState, sizeof(Motor_StateInfo_t));
    
    if (res == RESULT_OK) {
        *pu8MotorDir = (uint8_t)stcMotorState.desired_dir;
        RTURN_DEBUG("Read motor state: desired_dir=%d, state=%d\r\n", 
                    stcMotorState.desired_dir, stcMotorState.state);
        return RESULT_OK;
    }
    
    RTURN_DEBUG("Failed to read motor state, res=%d\r\n", res);
    return RESULT_ERROR;
}

static uint8_t RTurn_ConvertMotorDirToRTurnDir(uint8_t u8MotorDir, uint8_t u8ReverseOutput) {
    if (u8MotorDir == MOTOR_DIRECTION_NONE) {
        return RTURN_DIR_STOP;
    }
    
    if (u8ReverseOutput) {
        if (u8MotorDir == MOTOR_DIRECTION_FORWARD) {
            return RTURN_DIR_REVERSE;
        } else if (u8MotorDir == MOTOR_DIRECTION_REVERSE) {
            return RTURN_DIR_FORWARD;
        }
    } else {
        if (u8MotorDir == MOTOR_DIRECTION_FORWARD) {
            return RTURN_DIR_FORWARD;
        } else if (u8MotorDir == MOTOR_DIRECTION_REVERSE) {
            return RTURN_DIR_REVERSE;
        }
    }
    
    return RTURN_DIR_STOP;
}

static DeviceResult_t RTurn_GetMotorSpeedAndDir(RTurn_Device_t* pstcDev, float* pfRpm, uint8_t* pu8MotorDir) {
    if (!pstcDev) return RESULT_PARAM_ERR;
    
    uint8_t u8DevId = pstcDev->stcConfig.u8MotorHallDevId;
    
    if (pfRpm) {
        DeviceResult_t res = Device_Read(u8DevId, pfRpm, sizeof(float));
        if (res != RESULT_OK) {
            RTURN_DEBUG("Read RPM failed, res=%d\r\n", res);
        }
    }
    
    if (pu8MotorDir) {
        DeviceCommandData_t cmd;
        cmd.cmd = CMD_MOTOR_HALL_GET_DIRECTION;
        cmd.params = NULL;
        cmd.param_size = 0;
        cmd.response = pu8MotorDir;
        cmd.response_size = sizeof(uint8_t);
        
        DeviceResult_t res = Device_Control(u8DevId, &cmd);
        if (res != RESULT_OK) {
            RTURN_DEBUG("Read motor direction failed, res=%d\r\n", res);
        }
    }
    
    return RESULT_OK;
}

static float RTurn_RpmToAngularSpeed(float fRpm, float fReductionRatio) {
    if (fReductionRatio <= 0) return 0;
    return fRpm * 360.0f / fReductionRatio / 60.0f;
}

// 锟斤拷取锟斤拷锟斤拷锟斤拷锟芥警状态锟斤拷0=锟斤拷锟斤拷, 1=锟斤拷锟斤拷锟芥警锟斤拷
static uint8_t RTurn_ReadSensorAlarm(RTurn_Device_t* pstcDev) {
    if (!pstcDev) return 0;
    
    uint8_t u8Alarm = 0;
    DeviceCommandData_t cmd;
    cmd.cmd = CMD_SENSOR_GET_ALARM_STATUS;
    cmd.params = NULL;
    cmd.param_size = 0;
    cmd.response = &u8Alarm;
    cmd.response_size = sizeof(uint8_t);
    
    DeviceResult_t res = Device_Control(pstcDev->stcConfig.u8SensorDevId, &cmd);
    if (res != RESULT_OK) {
        RTURN_DEBUG("Read sensor alarm failed, res=%d\r\n", res);
        return 0;
    }
    
    return u8Alarm;
}

static void RTurn_UpdateAngle(RTurn_Device_t* pstcDev) {
    if (!pstcDev) return;
    
    uint32_t u32Now = tickTimer_GetCount();
    uint32_t u32DeltaMs = u32Now - pstcDev->u32LastAngleTime;
    
    if (u32DeltaMs == 0) return;
    
    float fRpm = 0;
    uint8_t u8MotorDir = MOTOR_DIRECTION_NONE;
    DeviceResult_t res = RTurn_GetMotorSpeedAndDir(pstcDev, &fRpm, &u8MotorDir);
    
    if (res != RESULT_OK) {
        RTURN_DEBUG("Failed to get motor speed and direction\r\n");
        pstcDev->u32LastAngleTime = u32Now;
        return;
    }
    
    pstcDev->fCurrentSpeed = RTurn_RpmToAngularSpeed(fRpm, pstcDev->stcConfig.fReductionRatio);
    pstcDev->u8CurrentDir = RTurn_ConvertMotorDirToRTurnDir(u8MotorDir, pstcDev->stcConfig.u8ReverseOutput);
    
    // 锟斤拷取锟斤拷锟斤拷锟斤拷锟斤拷锟劫诧拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷诮锟斤拷锟斤拷卸锟�
    uint8_t u8DesiredDir = MOTOR_DIRECTION_NONE;
    RTurn_GetDesiredDirection(pstcDev, &u8DesiredDir);
    uint8_t u8DesiredRTurnDir = RTurn_ConvertMotorDirToRTurnDir(u8DesiredDir, pstcDev->stcConfig.u8ReverseOutput);
    
    // ========== 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟津（凤拷锟斤拷4锟斤拷 ==========
    // 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟叫э拷锟斤拷锟絊TOP锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
    // 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷时锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷驯锟斤拷锟斤拷锟劫诧拷锟斤拷锟斤拷锟絊TOP锟斤拷使锟矫伙拷锟芥方锟斤拷
    if (u8DesiredRTurnDir != RTURN_DIR_STOP) {
        pstcDev->u8LastDesiredDir = u8DesiredRTurnDir;
    }
    
    // ========== 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟芥警状态锟斤拷锟斤拷锟斤拷锟斤拷校准状态锟斤拷 ==========
    // 锟斤拷锟斤拷4锟斤拷锟脚匡拷锟斤拷锟斤拷锟斤拷使锟矫伙拷锟芥方锟斤拷锟斤拷为锟斤拷选
    // 锟斤拷锟斤拷锟角帮拷锟斤拷锟斤拷锟斤拷锟轿猄TOP锟斤拷锟斤拷锟芥方锟斤拷锟斤拷效锟斤拷使锟矫伙拷锟芥方锟斤拷
    {
        uint8_t u8CheckDir = u8DesiredRTurnDir;
        if (u8CheckDir == RTURN_DIR_STOP && pstcDev->u8LastDesiredDir != RTURN_DIR_STOP) {
            u8CheckDir = pstcDev->u8LastDesiredDir;
        }
        
        if (!pstcDev->stcLockState.u8LockActive && u8CheckDir != RTURN_DIR_STOP && !pstcDev->u8LimitTriggered) {
            uint8_t u8Alarm = RTurn_ReadSensorAlarm(pstcDev);
            
            if (u8Alarm) {
                // 锟斤拷锟斤拷锟芥警锟揭凤拷锟斤拷锟斤拷效锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
                uint8_t u8LimitDir = (u8CheckDir == RTURN_DIR_FORWARD) ? RTURN_LIMIT_FORWARD : RTURN_LIMIT_REVERSE;
                
                if (u8CheckDir == RTURN_DIR_REVERSE) {
                    /* 锟截憋拷时锟斤拷锟斤拷锟斤拷锟斤拷锟矫角讹拷为锟斤拷锟斤拷位锟角度ｏ拷锟斤拷锟斤拷锟轿拷锟叫Ｗ� */
                                        if (RunAngle_TryCalibrate()) {
                        g_s32HallPulseAccum = 0;
                        pstcDev->fCurrentAngle = pstcDev->stcConfig.fMinAngle;
                        if (!pstcDev->u8Calibrated) {
                            pstcDev->u8Calibrated = 1;
                            RTURN_OUT("Calibrated! Angle set to min angle: %f deg\r\n", pstcDev->fCurrentAngle);
                        }
                        RunAngle_OnCalibration();
                    } else {
                        RealTime_SetFault(FAULT_BIT_OVERCURRENT_REV);
                        RTurn_PublishBidirectionalOvercurrent(1);
                    }
                }
                /* 锟斤拷时锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟矫角度ｏ拷锟斤拷锟街碉拷前锟角讹拷 */
                
                pstcDev->stcLockState.u8LockedDir = u8CheckDir;
                pstcDev->stcLockState.u8LockActive = 1;
                pstcDev->u8LimitTriggered = 1;
                
                int32_t s32AngleInt = (int32_t)(pstcDev->fCurrentAngle * 100);
                RTURN_OUT("LIMIT TRIGGERED (active check)! Dir=%s, Angle=%ld.%02ld deg, Calibrated=%d, UsedCachedDir=%d\r\n",
                          (u8LimitDir == RTURN_LIMIT_FORWARD) ? "FORWARD" : "REVERSE",
                          (long)(s32AngleInt / 100), (long)(s32AngleInt % 100),
                          pstcDev->u8Calibrated,
                          (u8DesiredRTurnDir == RTURN_DIR_STOP) ? 1 : 0);
                
                RTurn_LimitEvent_t stcEvent;
                stcEvent.u8Direction = u8LimitDir;
                stcEvent.fAngle = pstcDev->fCurrentAngle;
                stcEvent.u8IsActive = 1;
                EventBus_Publish(TOPIC_RTURN_LIMIT, &stcEvent);
            }
        }
    }
    
    // ========== 锟斤拷锟斤拷锟斤拷锟斤拷呒锟斤拷锟斤拷锟斤拷锟斤拷锟叫Ｗ甲刺拷锟� ==========
    if (pstcDev->stcLockState.u8LockActive) {
        if ((pstcDev->stcLockState.u8LockedDir == RTURN_DIR_FORWARD && u8DesiredRTurnDir == RTURN_DIR_REVERSE) ||
            (pstcDev->stcLockState.u8LockedDir == RTURN_DIR_REVERSE && u8DesiredRTurnDir == RTURN_DIR_FORWARD)) {
            
            uint8_t u8ReleaseDir = (pstcDev->stcLockState.u8LockedDir == RTURN_DIR_FORWARD) ? RTURN_LIMIT_FORWARD : RTURN_LIMIT_REVERSE;
            
            pstcDev->stcLockState.u8LockActive = 0;
            pstcDev->stcLockState.u8LockedDir = 0;
            pstcDev->u8LimitTriggered = 0;
            
            RTURN_OUT("LOCK RELEASED! Desired direction changed to opposite, release dir=%d\r\n", u8ReleaseDir);
            
            RTurn_LimitEvent_t stcEvent;
            stcEvent.u8Direction = u8ReleaseDir;
            stcEvent.fAngle = pstcDev->fCurrentAngle;
            stcEvent.u8IsActive = 0;
            EventBus_Publish(TOPIC_RTURN_LIMIT, &stcEvent);
            
            // 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟芥警状态锟斤拷锟斤拷锟斤拷锟斤拷校准状态锟斤拷
            if (u8DesiredRTurnDir != RTURN_DIR_STOP) {
                uint8_t u8Alarm = RTurn_ReadSensorAlarm(pstcDev);
                
                if (u8Alarm) {
                    // 锟斤拷然锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟铰凤拷锟斤拷
                    uint8_t u8NewLimitDir = (u8DesiredRTurnDir == RTURN_DIR_FORWARD) ? RTURN_LIMIT_FORWARD : RTURN_LIMIT_REVERSE;
                    
                    if (u8DesiredRTurnDir == RTURN_DIR_REVERSE) {
                        /* 锟截憋拷时锟斤拷锟斤拷锟斤拷锟斤拷锟矫角讹拷为锟斤拷锟斤拷位锟角度ｏ拷锟斤拷锟斤拷校准状态锟铰诧拷锟斤拷锟矫ｏ拷 */
                        if (pstcDev->u8Calibrated) {
                            if (RunAngle_TryCalibrate()) {
                                g_s32HallPulseAccum = 0;
                                pstcDev->fCurrentAngle = pstcDev->stcConfig.fMinAngle;
                                RunAngle_OnCalibration();
                            } else {
                                RealTime_SetFault(FAULT_BIT_OVERCURRENT_REV);
                                RTurn_PublishBidirectionalOvercurrent(1);
                            }
                        }
                    }
                    /* 锟斤拷时锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟矫角度ｏ拷锟斤拷锟街碉拷前锟角讹拷 */

                    pstcDev->stcLockState.u8LockedDir = u8DesiredRTurnDir;
                    pstcDev->stcLockState.u8LockActive = 1;
                    pstcDev->u8LimitTriggered = 1;
                    
                    int32_t s32AngleInt = (int32_t)(pstcDev->fCurrentAngle * 100);
                    RTURN_OUT("LOCK RE-LOCK! Still overcurrent, new Dir=%s, Angle=%ld.%02ld deg, Calibrated=%d\r\n",
                              (u8NewLimitDir == RTURN_LIMIT_FORWARD) ? "FORWARD" : "REVERSE",
                              (long)(s32AngleInt / 100), (long)(s32AngleInt % 100),
                              pstcDev->u8Calibrated);
                    
                    RTurn_LimitEvent_t stcNewEvent;
                    stcNewEvent.u8Direction = u8NewLimitDir;
                    stcNewEvent.fAngle = pstcDev->fCurrentAngle;
                    stcNewEvent.u8IsActive = 1;
                    EventBus_Publish(TOPIC_RTURN_LIMIT, &stcNewEvent);
                }
            }
        }
    }
    
    // ========== 校准锟斤拷锟斤拷呒锟斤拷锟斤拷嵌然锟斤拷帧锟斤拷锟轿伙拷锟斤拷龋锟� ==========
    if (pstcDev->u8Calibrated) {
        // 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷前锟剿讹拷锟斤拷锟斤拷锟斤拷锟斤拷
        // - 锟截闭ｏ拷锟斤拷转锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟角度伙拷锟街ｏ拷锟角度讹拷锟斤拷锟斤拷 fMinAngle
        // - 锟津开ｏ拷锟斤拷转锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟街ｏ拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟桔加ｏ拷锟斤拷直锟斤拷锟斤拷锟酵Ｖ�
        if (pstcDev->stcLockState.u8LockActive && 
            pstcDev->stcLockState.u8LockedDir == pstcDev->u8CurrentDir) {
            
            if (pstcDev->stcLockState.u8LockedDir == RTURN_DIR_REVERSE) {
                /* 锟截憋拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟角度伙拷锟街ｏ拷锟角度讹拷锟斤拷 */
                if (u32Now - s_u32LastLockDebugPrintTime >= LOCK_DEBUG_PRINT_INTERVAL_MS) {
                    s_u32LastLockDebugPrintTime = u32Now;
                    RTURN_DEBUG("Direction %d is locked (REVERSE), ignore angle change\r\n", pstcDev->u8CurrentDir);
                }
                pstcDev->u32LastAngleTime = u32Now;
                return;
            }
            /* 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟街ｏ拷直锟斤拷锟斤拷锟酵Ｖ� */
            if (u32Now - s_u32LastLockDebugPrintTime >= LOCK_DEBUG_PRINT_INTERVAL_MS) {
                s_u32LastLockDebugPrintTime = u32Now;
                RTURN_DEBUG("Direction %d is locked (FORWARD), allow angle integration\r\n", pstcDev->u8CurrentDir);
            }
        }
        
        /* Pulse-direct angle: each Hall edge = 360/12/ratio degrees.
         * Eliminates RPM estimation error during acceleration/deceleration.
         * g_s32HallPulseAccum is direction-aware (+FWD/-REV), updated every 1ms. */
        pstcDev->fCurrentAngle = pstcDev->stcConfig.fMinAngle +
            (float)g_s32HallPulseAccum * 360.0f / 12.0f / pstcDev->stcConfig.fReductionRatio;
        
        // 锟斤拷转锟角讹拷锟斤拷锟睫硷拷猓拷嵌鹊锟轿伙拷锟斤拷锟斤拷锟�
        if (pstcDev->fCurrentAngle >= pstcDev->stcConfig.fMaxAngle) {
            // NOTE: 锟斤拷钳位锟角度ｏ拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷实值锟斤拷锟斤拷取
            
            // 只锟叫碉拷锟斤拷位未锟斤拷锟斤拷锟斤拷时锟脚凤拷锟斤拷锟铰硷拷
            if (!pstcDev->u8LimitTriggered) {
                pstcDev->u8LimitTriggered = 1;
                
                // 锟斤拷锟斤拷锟斤拷锟斤拷状态锟斤拷锟斤拷
                pstcDev->stcLockState.u8LockedDir = RTURN_DIR_FORWARD;
                pstcDev->stcLockState.u8LockActive = 1;
                
                RTurn_LimitEvent_t stcEvent;
                stcEvent.u8Direction = RTURN_LIMIT_FORWARD;  // 锟斤拷转锟斤拷位
                stcEvent.fAngle = pstcDev->fCurrentAngle;
                stcEvent.u8IsActive = 1;
                EventBus_Publish(TOPIC_RTURN_LIMIT, &stcEvent);
                
                RTURN_OUT("Position limit reached! Forward blocked\r\n");
            }
        }
        
        // 锟斤拷转锟角讹拷锟斤拷锟睫硷拷猓褐伙拷锟斤拷锟斤拷嵌龋锟斤拷锟斤拷锟斤拷锟斤拷锟轿伙拷锟斤拷锟�
        // 锟斤拷转锟斤拷位锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷山嵌却锟斤拷锟�
        if (pstcDev->fCurrentAngle <= pstcDev->stcConfig.fMinAngle) {
            pstcDev->fCurrentAngle = pstcDev->stcConfig.fMinAngle;
            // 注锟解：锟斤拷锟斤不锟斤拷锟斤拷锟斤拷位锟铰硷拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷状态
        }
    }
    
    pstcDev->u32LastAngleTime = u32Now;
}

// ========== 锟斤拷锟斤拷锟铰硷拷锟斤拷锟斤拷 ==========
// 锟斤拷 EventBus 锟截碉拷 RTurn_OnCurrentAlarm 锟斤拷锟斤拷
// 锟秸碉拷锟斤拷锟斤拷锟铰硷拷锟斤拷直锟斤拷锟斤拷锟斤拷锟斤拷前锟斤拷锟斤拷锟斤拷锟津，诧拷锟斤拷锟叫讹拷锟斤拷值锟斤拷锟斤拷值锟叫讹拷锟斤拷 dev_sensor 锟斤拷锟斤拷桑锟�
static void RTurn_HandleOvercurrent(RTurn_Device_t* pstcDev) {
    if (!pstcDev) return;
    
    // 锟斤拷锟斤拷丫锟斤拷锟斤拷锟斤拷锟斤拷锟轿伙拷汛锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷馗锟斤拷锟斤拷锟�
    if (pstcDev->stcLockState.u8LockActive || pstcDev->u8LimitTriggered) {
        return;
    }
    
    // 只使锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟劫诧拷锟斤拷锟斤拷锟斤拷姆锟斤拷锟斤拷锟斤拷卸锟斤拷锟轿伙拷锟斤拷锟�
    uint8_t u8DesiredDir = MOTOR_DIRECTION_NONE;
    RTurn_GetDesiredDirection(pstcDev, &u8DesiredDir);
    uint8_t u8CurrentDir = RTurn_ConvertMotorDirToRTurnDir(u8DesiredDir, pstcDev->stcConfig.u8ReverseOutput);
    
    // 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷效时锟脚达拷锟斤拷
    if (u8CurrentDir != RTURN_DIR_STOP) {
        
        uint8_t u8LimitDir = (u8CurrentDir == RTURN_DIR_FORWARD) ? RTURN_LIMIT_FORWARD : RTURN_LIMIT_REVERSE;
        
        if (!pstcDev->u8Calibrated) {
            /* ========== 未校准状态 ========== */
            if (u8CurrentDir == RTURN_DIR_FORWARD) {
                /* 开窗过流: 锁方向 + 报正转过流故障 + 双向封锁电机 */
                RealTime_SetFault(FAULT_BIT_OVERCURRENT_FWD);
                RTurn_PublishBidirectionalOvercurrent(1);
                RTURN_OUT("LIMIT TRIGGERED (uncalibrated, FORWARD)! Lock direction, set fault FWD\r\n");
            } else {
                /* 锟截闭癸拷锟斤拷锟斤拷校准锟缴癸拷锟斤拷锟斤拷锟矫角讹拷为锟斤拷锟斤拷位锟角讹拷 */
                if (RunAngle_TryCalibrate()) {
                    g_s32HallPulseAccum = 0;
                    pstcDev->fCurrentAngle = pstcDev->stcConfig.fMinAngle;
                    pstcDev->u8Calibrated = 1;
                    RunAngle_OnCalibration();
                } else {
                    RealTime_SetFault(FAULT_BIT_OVERCURRENT_REV);
                    RTurn_PublishBidirectionalOvercurrent(1);
                }
                RTURN_OUT("LIMIT TRIGGERED (uncalibrated, REVERSE)! Calibrated=1, Angle=%ld.%02ld deg\r\n",
                          (long)((int32_t)(pstcDev->fCurrentAngle * 100) / 100),
                          (long)((int32_t)(pstcDev->fCurrentAngle * 100) % 100));
            }
            
            /* 未校准状态锟铰ｏ拷锟津开和关闭癸拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷影锟斤拷锟劫诧拷锟斤拷锟斤拷 */
            pstcDev->stcLockState.u8LockedDir = u8CurrentDir;
            pstcDev->stcLockState.u8LockActive = 1;
            pstcDev->u8LimitTriggered = 1;
            
            RTurn_LimitEvent_t stcEvent;
            stcEvent.u8Direction = u8LimitDir;
            stcEvent.fAngle = pstcDev->fCurrentAngle;
            stcEvent.u8IsActive = 1;
            EventBus_Publish(TOPIC_RTURN_LIMIT, &stcEvent);
        } else {
            /* ========== 锟斤拷校准状态 ========== */
            if (u8CurrentDir == RTURN_DIR_REVERSE) {
                /* 关窗过流: 校准或报故障 + 双向封锁电机 */
                if (RunAngle_TryCalibrate()) {
                    pstcDev->fCurrentAngle = pstcDev->stcConfig.fMinAngle;
                    g_s32HallPulseAccum = 0;
                    RunAngle_OnCalibration();
                } else {
                    RealTime_SetFault(FAULT_BIT_OVERCURRENT_REV);
                    RTurn_PublishBidirectionalOvercurrent(1);
                }
            } else {
                /* 开窗过流: 锁方向 + 报正转过流故障 + 双向封锁电机 */
                RealTime_SetFault(FAULT_BIT_OVERCURRENT_FWD);
                RTurn_PublishBidirectionalOvercurrent(1);
            }
            /* 锁方向, 保持当前角度 */

            pstcDev->stcLockState.u8LockedDir = u8CurrentDir;
            pstcDev->stcLockState.u8LockActive = 1;
            pstcDev->u8LimitTriggered = 1;
            
            int32_t s32AngleInt = (int32_t)(pstcDev->fCurrentAngle * 100);
            RTURN_OUT("LIMIT TRIGGERED (calibrated)! Dir=%s, Angle=%ld.%02ld deg, LockDir=%d\r\n",
                      (u8LimitDir == RTURN_LIMIT_FORWARD) ? "FORWARD" : "REVERSE",
                      (long)(s32AngleInt / 100), (long)(s32AngleInt % 100),
                      pstcDev->stcLockState.u8LockedDir);
            
            RTurn_LimitEvent_t stcEvent;
            stcEvent.u8Direction = u8LimitDir;
            stcEvent.fAngle = pstcDev->fCurrentAngle;
            stcEvent.u8IsActive = 1;
            EventBus_Publish(TOPIC_RTURN_LIMIT, &stcEvent);
        }
    }
}

static void RTurn_ClearLock(RTurn_Device_t* pstcDev) {
    if (!pstcDev) return;
    
    pstcDev->stcLockState.u8LockedDir = 0;
    pstcDev->stcLockState.u8LockActive = 0;
    pstcDev->u8LimitTriggered = 0;
    
    RTURN_OUT("Lock cleared\r\n");
}

// ========== EventBus锟截碉拷锟斤拷锟斤拷 ==========

void RTurn_OnCurrentAlarm(void* payload) {
    Current_AlarmEvent_t* pstcEvent = (Current_AlarmEvent_t*)payload;
    
    if (!pstcEvent->u8IsActive) return;
    
    // 锟斤拷锟斤拷锟斤拷锟斤拷锟借备锟斤拷通锟斤拷 ops.init 锟斤拷锟斤拷指锟斤拷锟揭碉拷 RTurn 锟借备
    for (uint8_t i = 0; i < MAX_DEVICES; i++) {
        DeviceNode_t* pstcNode = DeviceManager_Get(i);
        if (pstcNode && pstcNode->used && pstcNode->ops.init == RTurn_Device_Init) {
            RTurn_Device_t* pstcDev = (RTurn_Device_t*)pstcNode->private_data;
            RTurn_HandleOvercurrent(pstcDev);
            break;
        }
    }
}

// ========== 锟斤拷准锟借备锟斤拷锟斤拷实锟斤拷 ==========

DeviceResult_t RTurn_Device_Init(void* handle) {
    RTurn_Device_t* pstcDev = (RTurn_Device_t*)handle;
    if (!pstcDev) return RESULT_PARAM_ERR;
    
    RTURN_DEBUG("Init: MotorHall ID=%d, Sensor ID=%d, ReverseOutput=%d\r\n",
                pstcDev->stcConfig.u8MotorHallDevId,
                pstcDev->stcConfig.u8SensorDevId,
                pstcDev->stcConfig.u8ReverseOutput);
    
    int32_t s32RatioInt = (int32_t)(pstcDev->stcConfig.fReductionRatio * 100);
    int32_t s32MaxAngleInt = (int32_t)(pstcDev->stcConfig.fMaxAngle * 100);
    int32_t s32MinAngleInt = (int32_t)(pstcDev->stcConfig.fMinAngle * 100);
    RTURN_DEBUG("Mechanical: Ratio=%ld.%02ld, MaxAngle=%ld.%02ld deg, MinAngle=%ld.%02ld deg\r\n",
                (long)(s32RatioInt / 100), (long)(s32RatioInt % 100),
                (long)(s32MaxAngleInt / 100), (long)(s32MaxAngleInt % 100),
                (long)(s32MinAngleInt / 100), (long)(s32MinAngleInt % 100));
    
    pstcDev->fCurrentAngle = 0;
    pstcDev->fCurrentSpeed = 0;
    pstcDev->u8CurrentDir = RTURN_DIR_STOP;
    pstcDev->u8Calibrated = 0;
    pstcDev->u8LimitTriggered = 0;
    pstcDev->stcLockState.u8LockedDir = 0;
    pstcDev->stcLockState.u8LockActive = 0;
    pstcDev->u32LastAngleTime = tickTimer_GetCount();
    
    DeviceNode_t* pstcMotorNode = DeviceManager_Get(pstcDev->stcConfig.u8MotorHallDevId);
    if (!pstcMotorNode || !pstcMotorNode->private_data) {
        RTURN_DEBUG("Warning: MotorHall device ID=%d not found!\r\n", 
                    pstcDev->stcConfig.u8MotorHallDevId);
    }
    
    DeviceNode_t* pstcSensorNode = DeviceManager_Get(pstcDev->stcConfig.u8SensorDevId);
    if (!pstcSensorNode || !pstcSensorNode->private_data) {
        RTURN_DEBUG("Warning: Sensor device ID=%d not found!\r\n", 
                    pstcDev->stcConfig.u8SensorDevId);
    }
    
    pstcDev->u8Initialized = 1;
    pstcDev->u32LastUpdateTime = tickTimer_GetCount();
    
    RTURN_DEBUG("Init success\r\n");
    
    return RESULT_OK;
}

DeviceResult_t RTurn_Device_Deinit(void* handle) {
    RTurn_Device_t* pstcDev = (RTurn_Device_t*)handle;
    if (!pstcDev) return RESULT_PARAM_ERR;
    
    RTURN_DEBUG("Deinit\r\n");
    pstcDev->u8Initialized = 0;
    return RESULT_OK;
}

DeviceResult_t RTurn_Device_Read(void* handle, void* data, uint32_t size) {
    RTurn_Device_t* pstcDev = (RTurn_Device_t*)handle;
    if (!pstcDev || !data) return RESULT_PARAM_ERR;
    if (!pstcDev->u8Initialized) return RESULT_ERROR;
    
    if (size == sizeof(RTurn_ReadResponse_t)) {
        RTurn_ReadResponse_t* pstcResp = (RTurn_ReadResponse_t*)data;
        pstcResp->fAngle = pstcDev->fCurrentAngle;
        pstcResp->fSpeed = pstcDev->fCurrentSpeed;
        pstcResp->u8Direction = pstcDev->u8CurrentDir;
        pstcResp->u8LockedDir = pstcDev->stcLockState.u8LockedDir;
        pstcResp->u8Calibrated = pstcDev->u8Calibrated;
        return RESULT_OK;
    }
    
    return RESULT_PARAM_ERR;
}

DeviceResult_t RTurn_Device_Write(void* handle, const void* data, uint32_t size) {
    (void)handle;
    (void)data;
    (void)size;
    return RESULT_ERROR;
}

DeviceResult_t RTurn_Device_Control(void* handle, DeviceCommandData_t* pstcCmd) {
    RTurn_Device_t* pstcDev = (RTurn_Device_t*)handle;
    if (!pstcDev || !pstcCmd) return RESULT_PARAM_ERR;
    if (!pstcDev->u8Initialized) return RESULT_ERROR;
    
    switch (pstcCmd->cmd) {
        case CMD_RTURN_GET_ANGLE:
            if (pstcCmd->response && pstcCmd->response_size >= sizeof(int32_t)) {
                *(int32_t*)pstcCmd->response = (int32_t)(pstcDev->fCurrentAngle * 100);
                return RESULT_OK;
            }
            return RESULT_PARAM_ERR;
            
        case CMD_RTURN_GET_ANGLE_DEG:
            if (pstcCmd->response && pstcCmd->response_size >= sizeof(float)) {
                *(float*)pstcCmd->response = pstcDev->fCurrentAngle;
                return RESULT_OK;
            }
            return RESULT_PARAM_ERR;
            
        case CMD_RTURN_GET_SPEED:
            if (pstcCmd->response && pstcCmd->response_size >= sizeof(float)) {
                *(float*)pstcCmd->response = pstcDev->fCurrentSpeed;
                return RESULT_OK;
            }
            return RESULT_PARAM_ERR;
            
        case CMD_RTURN_GET_DIRECTION:
            if (pstcCmd->response && pstcCmd->response_size >= sizeof(uint8_t)) {
                *(uint8_t*)pstcCmd->response = pstcDev->u8CurrentDir;
                return RESULT_OK;
            }
            return RESULT_PARAM_ERR;
            
        case CMD_RTURN_GET_LOCKED_DIR:
            if (pstcCmd->response && pstcCmd->response_size >= sizeof(uint8_t)) {
                *(uint8_t*)pstcCmd->response = pstcDev->stcLockState.u8LockedDir;
                return RESULT_OK;
            }
            return RESULT_PARAM_ERR;
            
        case CMD_RTURN_RESET_POSITION:
            pstcDev->fCurrentAngle = 0;
            pstcDev->u8Calibrated = 1;
            RTURN_OUT("Position reset to 0, calibrated=1\r\n");
            return RESULT_OK;
            
        case CMD_RTURN_CLEAR_LOCK:
            RTurn_ClearLock(pstcDev);
            return RESULT_OK;
            
        case CMD_RTURN_GET_CALIBRATED:
            if (pstcCmd->response && pstcCmd->response_size >= sizeof(uint8_t)) {
                *(uint8_t*)pstcCmd->response = pstcDev->u8Calibrated;
                return RESULT_OK;
            }
            return RESULT_PARAM_ERR;
            
        default:
            RTURN_DEBUG("Unknown cmd=0x%08X\r\n", pstcCmd->cmd);
            return RESULT_ERROR;
    }
}

DeviceResult_t RTurn_Device_Update(void* handle) {
    RTurn_Device_t* pstcDev = (RTurn_Device_t*)handle;
    if (!pstcDev || !pstcDev->u8Initialized) return RESULT_ERROR;
    
    uint32_t u32Now = tickTimer_GetCount();
    
    if (u32Now - pstcDev->u32LastUpdateTime < pstcDev->stcConfig.u16UpdateIntervalMs) {
        return RESULT_OK;
    }
    pstcDev->u32LastUpdateTime = u32Now;
    
    RTurn_UpdateAngle(pstcDev);
    
    // ========== 刷锟斤拷Keil Watch锟斤拷锟斤拷全锟街憋拷锟斤拷 ==========
    g_fDbgRTurnAngle       = pstcDev->fCurrentAngle;
    g_fDbgRTurnSpeed       = pstcDev->fCurrentSpeed;
    g_u8DbgRTurnDir        = pstcDev->u8CurrentDir;
    g_u8DbgRTurnLockedDir  = pstcDev->stcLockState.u8LockedDir;
    g_u8DbgRTurnLockActive = pstcDev->stcLockState.u8LockActive;
    g_u8DbgRTurnCalibrated = pstcDev->u8Calibrated;
    g_u8DbgRTurnLimitTrig  = pstcDev->u8LimitTriggered;
    
    // 锟斤拷取锟斤拷锟斤拷锟斤拷锟斤拷锟劫诧拷锟斤拷锟斤拷锟斤拷锟斤拷锟剿拷碌锟饺拷直锟斤拷锟�
    {
        uint8_t u8DesiredDir = MOTOR_DIRECTION_NONE;
        RTurn_GetDesiredDirection(pstcDev, &u8DesiredDir);
        g_u8DbgRTurnDesiredDir = RTurn_ConvertMotorDirToRTurnDir(u8DesiredDir, pstcDev->stcConfig.u8ReverseOutput);
    }
    
    // ========== 每2000ms锟斤拷印一锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷状态 ==========
    if (u32Now - s_u32LastLockPrintTime >= LOCK_PRINT_INTERVAL_MS) {
        s_u32LastLockPrintTime = u32Now;
        
        if (pstcDev->stcLockState.u8LockActive) {
            const char* pcLockDir = (pstcDev->stcLockState.u8LockedDir == RTURN_DIR_FORWARD) ? "FORWARD" :
                                    (pstcDev->stcLockState.u8LockedDir == RTURN_DIR_REVERSE) ? "REVERSE" : "NONE";
            int32_t s32AngleInt = (int32_t)(pstcDev->fCurrentAngle * 10);
            RTURN_OUT("RTurn: LOCK ACTIVE - Dir=%s, Angle=%ld.%ld deg\r\n", 
                      pcLockDir, (long)(s32AngleInt / 10), (long)(s32AngleInt % 10));
        } else {
            if (pstcDev->u8Calibrated) {
                int32_t s32AngleInt = (int32_t)(pstcDev->fCurrentAngle * 10);
                int32_t s32SpeedInt = (int32_t)(pstcDev->fCurrentSpeed * 10);
                RTURN_OUT("RTurn: No lock, Angle=%ld.%ld deg, Speed=%ld.%ld deg/s\r\n",
                          (long)(s32AngleInt / 10), (long)(s32AngleInt % 10),
                          (long)(s32SpeedInt / 10), (long)(s32SpeedInt % 10));
            } else {
                RTURN_OUT("RTurn: No lock, Not calibrated (waiting for limit trigger)\r\n");
            }
        }
    }
    
    // ========== 每4000ms锟斤拷印一锟轿碉拷前锟角讹拷 ==========
    if (pstcDev->u8Calibrated && (u32Now - s_u32LastLimitPrintTime >= LIMIT_PRINT_INTERVAL_MS)) {
        s_u32LastLimitPrintTime = u32Now;
        int32_t s32AngleInt = (int32_t)(pstcDev->fCurrentAngle * 100);
        RTURN_OUT("RTurn: Current Angle=%ld.%02ld deg, Dir=%d, Lock=%d\r\n",
                  (long)(s32AngleInt / 100), (long)(s32AngleInt % 100),
                  pstcDev->u8CurrentDir,
                  pstcDev->stcLockState.u8LockActive);
    }
    
    return RESULT_OK;
}

// ========== 圆锟斤拷转锟斤拷锟斤拷锟斤拷锟截讹拷锟接匡拷 ==========

float RTurn_Device_GetAngle(RTurn_Device_t* pstcDev) {
    if (!pstcDev || !pstcDev->u8Initialized) return 0;
    return pstcDev->fCurrentAngle;
}

float RTurn_Device_GetAngleDeg(RTurn_Device_t* pstcDev) {
    return RTurn_Device_GetAngle(pstcDev);
}

float RTurn_Device_GetSpeed(RTurn_Device_t* pstcDev) {
    if (!pstcDev || !pstcDev->u8Initialized) return 0;
    return pstcDev->fCurrentSpeed;
}

uint8_t RTurn_Device_GetDirection(RTurn_Device_t* pstcDev) {
    if (!pstcDev || !pstcDev->u8Initialized) return 0;
    return pstcDev->u8CurrentDir;
}

uint8_t RTurn_Device_GetLockedDir(RTurn_Device_t* pstcDev) {
    if (!pstcDev || !pstcDev->u8Initialized) return 0;
    return pstcDev->stcLockState.u8LockedDir;
}

void RTurn_Device_ResetPosition(RTurn_Device_t* pstcDev) {
    if (!pstcDev) return;
    pstcDev->fCurrentAngle = 0;
    pstcDev->u8Calibrated = 1;
    RTURN_OUT("Position reset\r\n");
}

void RTurn_Device_ClearLock(RTurn_Device_t* pstcDev) {
    RTurn_ClearLock(pstcDev);
}

uint8_t RTurn_Device_IsCalibrated(RTurn_Device_t* pstcDev) {
    if (!pstcDev || !pstcDev->u8Initialized) return 0;
    return pstcDev->u8Calibrated;
}

RTurn_Device_t* RTurn_Device_Create(const RTurn_Config_t* pstcConfig) {
    if (!pstcConfig) return NULL;
    
    RTurn_Device_t* pstcDev = (RTurn_Device_t*)malloc(sizeof(RTurn_Device_t));
    if (!pstcDev) return NULL;
    
    memset(pstcDev, 0, sizeof(RTurn_Device_t));
    memcpy(&pstcDev->stcConfig, pstcConfig, sizeof(RTurn_Config_t));
    
    if (pstcDev->stcConfig.u16UpdateIntervalMs == 0) {
        pstcDev->stcConfig.u16UpdateIntervalMs = 50;
    }
    
    int32_t s32RatioInt = (int32_t)(pstcConfig->fReductionRatio * 100);
    RTURN_DEBUG("Create: MotorHall ID=%d, Sensor ID=%d, Ratio=%ld.%02ld, Reverse=%d\r\n",
                pstcConfig->u8MotorHallDevId,
                pstcConfig->u8SensorDevId,
                (long)(s32RatioInt / 100), (long)(s32RatioInt % 100),
                pstcConfig->u8ReverseOutput);
    
    pstcDev->u8Initialized = 0;
    return pstcDev;
}

// ========== 全锟街诧拷锟斤拷锟斤拷锟斤拷锟斤拷 ==========
const DeviceOps_t g_rturn_ops = {
    .init = RTurn_Device_Init,
    .deinit = RTurn_Device_Deinit,
    .read = RTurn_Device_Read,
    .write = RTurn_Device_Write,
    .control = RTurn_Device_Control,
    .update = RTurn_Device_Update
};
