#ifndef DEV_SENSOR_H_
#define DEV_SENSOR_H_

#include "device_manager.h"
#include "dev_adc.h"
#include "TickTimer.h"
#include <stdint.h>
#include <stdbool.h>
#include "Adapter.h"
#include "rtt_manager.h"

// ========== ��ѹ��ѹģʽѡ�� ==========
// 0: �޷�ѹ - ���������ֱ�ӽ��� MCU 0~3.3V ��Ӧ0~2A ����
// 1: �з�ѹ - �ⲿ��ѹ��·����
#define SENSOR_VOLTAGE_DIVIDER_ENABLE    0   // 1=�з�ѹ, 0=�޷�ѹ


// ========== У׼���� ==========
#define SENSOR_CALIB_DELAY_MS       (120U)   // �ϵ��ȴ�ʱ�䣬�ܿ������� (ms)
#define SENSOR_CALIB_STABLE_MS      (50U)    // ADC �ȶ��ȴ�ʱ�� (ms)


// ========== ��ѹ��ѹ�������з�ѹģʽʱ��Ч�� ==========
#if SENSOR_VOLTAGE_DIVIDER_ENABLE
    #define SENSOR_DIVIDER_R1            (10000)   // �Ϸ�ѹ���� R1 (ŷķ)
    #define SENSOR_DIVIDER_R2            (20000)   // �·�ѹ���� R2 (ŷķ)


#endif

#ifdef DEV_SENSOR_EMA_DEBUG
    #define SENSOR_EMA_DBG(fmt, ...)    MAIN_D("[SENSOR_EMA] " fmt, ##__VA_ARGS__)
#else
    #define SENSOR_EMA_DBG(fmt, ...)    ((void)0)
#endif


// ========== Ӳ���屾������������ѡ�� ==========
// �� main.h ��ͨ�� BOARD_VERSION ͳһ����
#include "main.h"
#if BOARD_VERSION == 0
    // ԭHB_chuchai�壺�������������� (���1650mV, ������66mV/A, ���̡�25A)
    #define SENSOR_TYPE_DIFF_AMP_ENABLE    0
#else
    // ���ϰ壺����˷� (Vout = I * 0.1, 0mVΪ0A, 100mV/A)
    #define SENSOR_TYPE_DIFF_AMP_ENABLE    1
#endif


#ifdef DEV_SENSOR_TIMER_DEBUG
    #define SENSOR_TIMER_DBG(fmt, ...)    MAIN_D("[SENSOR_TIMER] " fmt, ##__VA_ARGS__)
#else
    #define SENSOR_TIMER_DBG(fmt, ...)    ((void)0)
#endif


// ========== ���Ժ궨�� ==========
#ifdef DEV_SENSOR
    #define SENSOR_DEBUG(fmt, ...)    MAIN_D("[SENSOR] " fmt, ##__VA_ARGS__)
#else
    #define SENSOR_DEBUG(fmt, ...)    ((void)0)
#endif

#ifdef DEV_SENSOR_REAL
    #define SENSOR_REAL_DEBUG(fmt, ...)    MAIN_D("[SENSOR_REAL] " fmt, ##__VA_ARGS__)
#else
    #define SENSOR_REAL_DEBUG(fmt, ...)    ((void)0)
#endif

#ifdef DEV_SENSOR_SLOW
    #define SENSOR_DEBUG_SLOW(fmt, ...)    MAIN_D("[SENSOR] " fmt, ##__VA_ARGS__)
#else
    #define SENSOR_DEBUG_SLOW(fmt, ...)    ((void)0)
#endif

// ========== ģ��ģʽ�궨�� ==========
// #define SENSOR_SIMULATION_MODE

// ========== �������ģʽ ==========
#define OVERCURRENT_MODE_SAMPLE_COUNT    0
#define OVERCURRENT_MODE_TIME_WINDOW     1

// ========== �����澯���ģʽѡ�� ==========
#define OVERCURRENT_CLEAR_AUTO      0
#define OVERCURRENT_CLEAR_MANUAL    1

#ifndef OVERCURRENT_CLEAR_MODE
#define OVERCURRENT_CLEAR_MODE      OVERCURRENT_CLEAR_MANUAL   
#endif

// ========== DEBUG ���ڻ����� ==========
#define DEBUG_SENSOR_WINDOW_BUFFER
#define SENSOR_WINDOW_BUFFER_SIZE     (200)

// ========== ������Ӳ�����������ݰ����Զ�ѡ�� ==========
#if SENSOR_TYPE_DIFF_AMP_ENABLE
    // ===== ����˷�ģʽ���� (3.3V����, ��2.5A����) =====
    // ���=1650mV(Vcc/2), ������=264mV/A
    #define SENSOR_RAW_ZERO_MV             (0)
    #define SENSOR_RAW_SENSITIVITY_MV_PER_A (100)
#else
    // ===== ��������������ģʽ���� (ԭHB_chuchai) =====
    #define SENSOR_RAW_ZERO_MV             (1650)
    #define SENSOR_RAW_SENSITIVITY_MV_PER_A (264)
#endif

#if SENSOR_VOLTAGE_DIVIDER_ENABLE
    #define SENSOR_VOUT_ZERO_MA_INT     (SENSOR_RAW_ZERO_MV * SENSOR_DIVIDER_R2 / (SENSOR_DIVIDER_R1 + SENSOR_DIVIDER_R2))
    #define SENSOR_SENSITIVITY_INT      (SENSOR_RAW_SENSITIVITY_MV_PER_A * SENSOR_DIVIDER_R2 / (SENSOR_DIVIDER_R1 + SENSOR_DIVIDER_R2))
    #define SENSOR_VOUT_ZERO_MV         ((float)SENSOR_VOUT_ZERO_MA_INT)
    #define SENSOR_SENSITIVITY_MV_PER_A ((float)SENSOR_SENSITIVITY_INT)
#else
    #define SENSOR_VOUT_ZERO_MA_INT     (SENSOR_RAW_ZERO_MV)
    #define SENSOR_SENSITIVITY_INT      (SENSOR_RAW_SENSITIVITY_MV_PER_A)
    #define SENSOR_VOUT_ZERO_MV         ((float)SENSOR_VOUT_ZERO_MA_INT)
    #define SENSOR_SENSITIVITY_MV_PER_A ((float)SENSOR_SENSITIVITY_INT)
#endif

// ========== 100ms average current (for Modbus 0x2733 realtime display) ==========
// Independent of overcurrent detection: overcurrent still uses instant
// s32CurrentMa, so fault response speed is not affected
#define SENSOR_AVG_WINDOW_MS         (100U)   // average window duration (ms)

// ========== 传感器设备命令定义 ==========
#define CMD_SENSOR_GET_CURRENT_MA      (CMD_BASE_ADC + 0x20)
#define CMD_SENSOR_GET_CURRENT_AX100   (CMD_BASE_ADC + 0x21)
#define CMD_SENSOR_SET_SIM_VALUE       (CMD_BASE_ADC + 0x22)
#define CMD_SENSOR_GET_ALARM_STATUS    (CMD_BASE_ADC + 0x23)
#define CMD_SENSOR_GET_CALIBRATION     (CMD_BASE_ADC + 0x24)
#define CMD_SENSOR_GET_CURRENT_MA_AVG  (CMD_BASE_ADC + 0x25)   /* 100ms window average current (mA) */

// ========== У׼�����ṹ�� ==========
typedef struct {
    int32_t s32ZeroOffsetMv;
    int16_t s16SensitivityScale;
    int32_t s32CalibrationValid;
} Sensor_Calibration_t;

// ========== �������豸���� ==========
typedef struct {
    uint8_t     u8AdcDevId;
    int32_t     s32OvercurrentThresholdMa;
    int32_t     s32OvercurrentHysteresisMa;
    uint8_t     u8OvercurrentMode;
    uint16_t    u16TriggerWindowSize;
    uint16_t    u16ReleaseWindowSize;
    uint32_t    u32TriggerWindowMs;
    uint32_t    u32ReleaseWindowMs;
} Sensor_Config_t;

// ========== �澯״̬�ṹ ==========
typedef struct {
    uint8_t  u8OvercurrentAlarm;
    uint16_t u16ConsecutiveCount;
    NonBlockingDelay_t stcTriggerTimer;
    NonBlockingDelay_t stcReleaseTimer;
    uint8_t  u8TimerRunning;
} Sensor_AlarmState_t;

// ========== �����澯�¼��ṹ�� ==========
typedef struct {
    int32_t  s32CurrentMa;
    int32_t  s32ThresholdMa;
    uint8_t  u8IsActive;
} Current_AlarmEvent_t;

// ========== �������豸�ṹ�� ==========
typedef struct {
    Sensor_Config_t     stcConfig;
    uint8_t             u8Initialized;
    Sensor_Calibration_t stcCalibration;
    uint16_t            u16AdcRawValue;
    uint16_t            u16AdcVoltageMv;
    int32_t             s32CurrentMa;
    int16_t             s16CurrentAx100;
    Sensor_AlarmState_t stcAlarmState;
    uint32_t            u32LastUpdateTime;
    uint8_t             u8Calibrated;
    uint32_t            u32InitTime;

    /* 100ms average current state (updated in Sensor_Device_Update) */
    int32_t             s32AvgCurrentMa;    /* last completed window average (mA) */
    int32_t             s32AvgAccumMa;      /* in-window sample sum (mA) */
    uint32_t            u32AvgSampleCnt;    /* in-window sample count */
    uint32_t            u32AvgWindowStart;  /* window start tick (ms) */

    struct {
        uint8_t  u8CalibDelayDone;   // 120ms ��ʱ�Ƿ������
        uint32_t u32CalibStartTime;  // 50ms У׼��ʼʱ��
    } stcCalibState;
        
} Sensor_Device_t;

// ========== ����Ӧ�ṹ�� ==========
typedef struct {
    int32_t  s32CurrentMa;
    int16_t  s16CurrentAx100;
    uint16_t u16AdcRawValue;
    uint16_t u16AdcVoltageMv;
} Sensor_ReadResponse_t;

// ========== ��׼�豸�ӿ� ==========
DeviceResult_t Sensor_Device_Init(void* handle);
DeviceResult_t Sensor_Device_Deinit(void* handle);
DeviceResult_t Sensor_Device_Read(void* handle, void* data, uint32_t size);
DeviceResult_t Sensor_Device_Write(void* handle, const void* data, uint32_t size);
DeviceResult_t Sensor_Device_Control(void* handle, DeviceCommandData_t* cmd);
DeviceResult_t Sensor_Device_Update(void* handle);

// ========== ������ȡֵ�ӿ� ==========
int32_t Sensor_Device_GetCurrentMA(Sensor_Device_t* pstcDev);
int16_t Sensor_Device_GetCurrentAx100(Sensor_Device_t* pstcDev);
Sensor_Device_t* Sensor_Device_Create(const Sensor_Config_t* pstcConfig);

// ========== У׼�ӿ� ==========
void Sensor_Device_CalibrateZero(Sensor_Device_t* pstcDev);
void Sensor_Device_SetSensitivityScale(Sensor_Device_t* pstcDev, int16_t s16ScalePercent);
void Sensor_Device_GetCalibration(Sensor_Device_t* pstcDev, Sensor_Calibration_t* pstcCal);

// ========== ģ��ģʽ�ӿ� ==========
#ifdef SENSOR_SIMULATION_MODE
void Sensor_SetSimulationValue(uint16_t u16VoltageMv);      // ����ģ���ԭʼ��ѹ(mV)
void Sensor_SetSimulationCurrent(int32_t s32CurrentMa);     // ֱ������ģ�����ֵ(mA)
uint16_t Sensor_GetSimulationSensorRawMv(void);             // ��ȡ��ǰģ�⴫������ԭʼֵ
#endif

// ========== �����澯�ֶ�����ӿ� ==========
void Sensor_Device_ClearAlarm(Sensor_Device_t* pstcDev);

// ========== ȫ�ֲ��������� ==========
extern const DeviceOps_t g_sensor_ops;

#endif /* DEV_SENSOR_H_ */
