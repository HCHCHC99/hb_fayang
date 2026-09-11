#ifndef __APP_PARAMS_H__
#define __APP_PARAMS_H__

#include <stdint.h>
#include <stdbool.h>
#include "rtt_manager.h"

/*=============================================================================
 *   调试宏定义
 * 统一使用 rtt_manager.h 中的调试宏，通过 APP_PARAMS_DBG 控制
 *============================================================================*/
#ifdef APP_PARAMS_DBG
    #define PARAMS_DBG(fmt, ...)    MAIN_D("[PARAMS] " fmt, ##__VA_ARGS__)
#else
    #define PARAMS_DBG(fmt, ...)    ((void)0)
#endif

#ifdef APP_REALTIME_DBG
    #define REALTIME_DBG(fmt, ...)  MAIN_D("[REALTIME] " fmt, ##__VA_ARGS__)
#else
    #define REALTIME_DBG(fmt, ...)  ((void)0)
#endif

/*=============================================================================
 * Modbus 协议保持寄存器地址定义
 *   所有保持寄存器 (Holding Register，支持 03/06/10 命令)
 *============================================================================*/

/* --- 配置参数（存储于Flash，掉电保存） --- */
#define REG_NODE_ID                 (0x2710U)   /* 设备地址 1~247 (uint16_t) */
#define REG_TARGET_SPEED            (0x2711U)   /* 目标转速 r/min (int16_t) */
#define REG_TARGET_ANGLE            (0x2712U)   /* 目标角度 0.1度 (int16_t) */
#define REG_VOLTAGE_UPPER_LIMIT     (0x2714U)   /* 过压阈值 0.1V (uint16_t) */
#define REG_VOLTAGE_LOWER_LIMIT     (0x2715U)   /* 欠压阈值 0.1V (uint16_t) */
#define REG_CURRENT_UPPER_LIMIT     (0x2716U)   /* 过流阈值 1mA (uint16_t) */
#define REG_CLOSE_LIMIT_ANGLE       (0x271CU)   /* 关窗极限角度 0.1度 (int16_t) */
#define REG_OPEN_LIMIT_ANGLE        (0x271DU)   /* 开窗极限角度 0.1度 (int16_t) */
#define REG_CURRENT_DETECT_MS       (0x271EU)   /* 过流判定时间 1ms (uint16_t) */

/* --- 高级控制参数（不存Flash，但定义在Flash地址范围，仅RAM有效） --- */
#define REG_MOTOR_HALL_DIR          (0x3710U)   /* 霍尔方向: 0=正常, 1=反转 (uint16_t) */
#define REG_MOTOR_DIR               (0x3711U)   /* 电机方向: 0=正常, 1=反转 (uint16_t) */
#define REG_RTURN_REDUCTION_RATIO   (0x3712U)   /* 减速比 (uint16_t) */
#define REG_MOTOR_HALL_POLE_PAIRS   (0x3713U)   /* 电机极对数 (uint16_t) */
#define REG_MOTOR_HALL_COUNT_LO     (0x3714U)   /* 霍尔脉冲累计低16位 (int32_t 累计值) */
#define REG_MOTOR_HALL_COUNT_HI     (0x3715U)   /* 霍尔脉冲累计高16位 */

/* --- 绝对角度相关 (0x2721~0x2726) --- */
#define REG_ABS_ANGLE_LO            (0x2721U)   /* RAM实时偏移低16位 (int32_t, 0.1度) */
#define REG_ABS_ANGLE_HI            (0x2722U)   /* RAM实时偏移高16位 */
#define REG_FLASH_ABS_LO            (0x2723U)   /* Flash已保存偏移低16位 (int32_t, 0.1度) */
#define REG_FLASH_ABS_HI            (0x2724U)   /* Flash已保存偏移高16位 */
#define REG_ABS_CMD                 (0x2725U)   /* W: 0=设基准点并保存, 1=回关窗基准点, 2=保存到Flash, 3=设基准点 */
#define REG_ABS_THRESHOLD           (0x2726U)   /* R/W: 回基准点/回目标停止阈值 (0.1度, 默认1, 0~200) */
#define REG_ABS_TARGET_LO           (0x2727U)   /* R/W: 目标角度 int32低16位 (0.1度); 0x10连续写触发回目标 */
#define REG_ABS_TARGET_HI           (0x2728U)   /* R/W: 目标角度 int32高16位 (0.1度) */
#define REG_CALIB_UPPER_X10         (0x2729U)   /* R/W: 关窗过流校准有效角度上限 0.1度 (默认200=20.0度) */
#define REG_CALIB_LOWER_X10         (0x272AU)   /* R/W: 关窗过流校准有效角度下限 0.1度 (默认-20=-2.0度) */
#define REG_JOG_FWD_X10             (0x272BU)   /* W: 开窗方向点动偏移 0.1度 (uint16) */
#define REG_JOG_REV_X10             (0x272CU)   /* W: 关窗方向点动偏移 0.1度 (uint16) */

/* --- 心跳包 (只读，不存Flash) --- */
#define REG_HEARTBEAT               (0x271FU)   /* 心跳包: 读回值 = 设备地址 (0x2710) */

/* 注意: 0x2713, 0x2717-0x271B 为保留地址，请勿通过 Modbus 访问 */

/* --- 控制命令 (写入直接执行，不存Flash) --- */
#define REG_CTRL_CMD                (0x2720U)   /* 控制命令寄存器 (uint16_t) */

/* REG_CTRL_CMD (0x2720) 位定义：
 * 写入命令:
 *   bit0 = 1: 解锁 RS485 控制，解锁后允许执行运动指令
 *   bit1 = 1: 停止(上锁 RS485 控制)
 *   bit2 = 1: 急停(取消正转/反转，上锁 RS485 控制)
 *   bit4 = 1: 正转(即使已解锁状态也有效，与 bit5 互斥)
 *   bit5 = 1: 反转(即使已解锁状态也有效，与 bit4 互斥)
 * 读取当前状态:
 *   bit4 = 1: 当前正在正转
 *   bit5 = 1: 当前正在反转
 *   bit4=0 且 bit5=0: 当前停止
 */
#define CTRL_CMD_START              (0x0001U)   /* bit0: 解锁控制指令 */
#define CTRL_CMD_STOP               (0x0002U)   /* bit1: 停止指令 */
#define CTRL_CMD_ESTOP              (0x0004U)   /* bit2: 急停指令 */
#define CTRL_CMD_ABS_SAVE           (0x0040U)   /* bit6: 将当前绝对角度位置保存到 Flash */
#define CTRL_CMD_FWD                (0x0010U)   /* bit4: 正转指令/状态 */
#define CTRL_CMD_REV                (0x0020U)   /* bit5: 反转指令/状态 */

/* --- 实时数据 (只读，不存Flash) --- */
#define REG_REAL_SPEED              (0x2730U)   /* 实时转速 r/min (int16_t) */
#define REG_REAL_ANGLE              (0x2731U)   /* 实时角度 0.1度 (int16_t) */
#define REG_REAL_VOLTAGE            (0x2732U)   /* 电压 0.1V (uint16_t) */
#define REG_REAL_CURRENT            (0x2733U)   /* 电流 1mA (uint16_t) */
#define REG_REAL_DIRECTION          (0x2737U)   /* 实时方向 (int16_t) */

/* --- 故障状态 (只读，不存Flash) --- */
#define REG_FAULT_STATUS            (0x2740U)   /* 故障状态 (uint16_t) */

/*=============================================================================
 * 故障状态位定义 (REG_FAULT_STATUS, 0x2740)
 * 1=故障发生, 0=正常
 *============================================================================*/
#define FAULT_BIT_OVERVOLTAGE       (0x0001U)   /* bit0: 过压 */
#define FAULT_BIT_OVERCURRENT_FWD   (0x0002U)   /* bit1: 正转(开窗)过流 */
#define FAULT_BIT_OVERCURRENT_REV   (0x0004U)   /* bit2: 反转(关窗)过流 */
#define FAULT_BIT_RESET             (0x0008U)   /* bit3: 复位 */
#define FAULT_BIT_OVERLOAD          (0x0010U)   /* bit4: 过载 */
#define FAULT_BIT_STALL             (0x0020U)   /* bit5: 堵转 */
#define FAULT_BIT_UNDERVOLTAGE      (0x0040U)   /* bit6: 欠压 */

/*=============================================================================
 * Flash 存储参数默认值定义
 *  对应 AppParamRecord_t 结构体成员，用于 Flash 初始化
 * 注意: 电压阈值单位 0.1V，电流阈值单位 mA
 *============================================================================*/

/* --- Modbus 协议主要参数 (存储于Flash，掉电保存) --- */
#define PARAM_DEFAULT_NODE_ID                   (1U)        /* 设备地址 1~247 */
#define PARAM_DEFAULT_TARGET_SPEED              (0)         /* 目标转速 r/min */
#define PARAM_DEFAULT_TARGET_ANGLE              (0)         /* 目标角度 0.1度 */
#define PARAM_DEFAULT_VOLTAGE_UPPER_LIMIT       (270U)      /* 过压阈值 0.1V (26.0V) */
#define PARAM_DEFAULT_VOLTAGE_LOWER_LIMIT       (210U)      /* 欠压阈值 0.1V (21.0V) */
#define PARAM_DEFAULT_CURRENT_UPPER_LIMIT       (1100)      /* 过流阈值 1mA (5A) */
#define PARAM_DEFAULT_CURRENT_DETECT_MS         (40)        /* 过流判定时间 1ms */
#define PARAM_DEFAULT_MOTOR_HALL_DIR            (0)         /* 霍尔方向 0=正常, 1=反转 */
#define PARAM_DEFAULT_MOTOR_DIR                 (1)         /* 电机方向 0=正常, 1=反转 */
#define PARAM_DEFAULT_RTURN_REDUCTION_RATIO     (11830)     /* 减速比 x0.1 */
#define PARAM_DEFAULT_MOTOR_HALL_POLE_PAIRS     (3)         /* 电机极对数 */
#define PARAM_DEFAULT_CLOSE_LIMIT_ANGLE         (-20)       /* 关窗极限角度 0.1度 */
#define PARAM_DEFAULT_OPEN_LIMIT_ANGLE          (880)       /* 开窗极限角度 0.1度 */
#define PARAM_DEFAULT_BAUD_RATE                 (9600UL)    /* 默认波特率 */

/* --- 内部参数 (不对外 Modbus，存 Flash 掉电保存) --- */
#define PARAM_DEFAULT_VOLTAGE_UPPER_HYSTERESIS  (20U)       /* 过压迟滞 0.1V (2.0V) */
#define PARAM_DEFAULT_VOLTAGE_LOWER_HYSTERESIS  (20U)       /* 欠压迟滞 0.1V (2.0V) */
#define PARAM_DEFAULT_OVERVOLTAGE_TRIGGER_CNT   (1U)        /* 过压触发计数 */
#define PARAM_DEFAULT_UNDERVOLTAGE_TRIGGER_CNT  (1U)        /* 欠压触发计数 */
#define PARAM_DEFAULT_CURRENT_HYSTERESIS_MA     (0U)        /* 过流迟滞 1mA (0A) - 设为0表示无迟滞 */
#define PARAM_DEFAULT_CURRENT_RELEASE_MS        (200U)      /* 过流释放时间窗口 1ms (0.2s) */
#define PARAM_DEFAULT_OVERCURRENT_TRIGGER_CNT   (1U)        /* 过流触发计数 (时间窗口模式可选择使用) */
#define PARAM_DEFAULT_CALIB_UPPER_X10           (10)       /* 关窗过流校准有效角度上限 (0.1度) */
#define PARAM_DEFAULT_CALIB_LOWER_X10           (-50)       /* 关窗过流校准有效角度下限 (0.1度) */

/*=============================================================================
 * 调试开关宏定义 (不存 Flash，通过宏定义开关)
 *============================================================================*/

/* --- 调试开关 --- */
#define APP_PARAMS_REALTIME_DBG                 /* 开启实时数据调试打印(每5秒一次) */
// #define APP_PARAMS_REALTIME_SIMULATE          /* 开启实时数据模拟(每5秒角度+1) */
#define APP_PARAMS_SIM_DBG                      /* 开启模拟实时数据调试打印 */

/* --- 实时数据来源选择 --- */
/* 注释掉此行则使用模拟实时数据 (SimRealtimeData, Keil Debug 可修改);
 * 取消注释则使用真实设备 dev_rturn 获取实时数据 */
#define APP_PARAMS_USE_DEV_RTURN

/*=============================================================================
 * 应用参数记录结构体 (兼容 Modbus 协议定义)
 * 注意: 结构体布局直接对应 Flash 的存储布局，修改需谨慎
 *       head_magic 和 tail_magic 作为 Flash 识别标记
 *       checksum 作为校验和
 *============================================================================*/
#pragma pack(4)
typedef struct {
    /* 头部信息 */
    uint32_t head_magic;        /* 头部魔数 (0x55AA55AA) */
    uint32_t sequence_id;       /* 序列号 (每次修改递增) */
    uint32_t erase_count;       /* Flash 擦写次数记录 */

    /* Modbus 协议主要参数 (保持寄存器地址，存储于Flash，掉电保存) */
    uint16_t node_id;               /* 0x2710: 设备地址 1~247 */
    int16_t  target_speed;          /* 0x2711: 目标转速 (r/min) */
    int16_t  target_angle;          /* 0x2712: 目标角度 (0.1度) */
    uint16_t voltage_upper_limit;   /* 0x2714: 过压阈值 (0.1V) */
    uint16_t voltage_lower_limit;   /* 0x2715: 欠压阈值 (0.1V) */
    uint16_t current_upper_limit;   /* 0x2716: 过流阈值 (1mA) */
    uint16_t current_detect_ms;     /* 0x271E: 过流判定时间 (1ms) */
    int16_t  close_limit_angle;     /* 0x271C: 关窗极限角度 (0.1度) */
    int16_t  open_limit_angle;      /* 0x271D: 开窗极限角度 (0.1度) */
    uint32_t baud_rate;             /* 波特率 (内部参数, 不对外 Modbus) */

    /* 内部参数 (不对外 Modbus，存 Flash 掉电保存) */
    uint16_t voltage_upper_hysteresis;   /* 过压迟滞 (0.1V) */
    uint16_t voltage_lower_hysteresis;   /* 欠压迟滞 (0.1V) */
    uint8_t  overvoltage_trigger_count;  /* 过压触发计数 */
    uint8_t  undervoltage_trigger_count; /* 欠压触发计数 */
    uint16_t current_hysteresis_ma;      /* 过流迟滞 (mA) */
    uint16_t current_release_ms;         /* 过流释放时间窗口 (ms) */
    uint8_t  overcurrent_trigger_count;  /* 过流触发计数 (时间窗口模式可选择使用) */
    uint16_t  motor_hall_dir;               /* 0x3710: 霍尔方向 0=正常 1=反转 */
    uint16_t  motor_dir;                    /* 0x3711: 电机方向 0=正常 1=反转 */
    uint16_t  rturn_reduction_ratio;          /* 0x3712: 减速比 x10 */
    uint16_t  motor_hall_pole_pairs;          /* 0x3713: 电机极对数 */
    int16_t   calib_upper_x10;                /* 0x2729: 关窗过流校准有效角度上限 (0.1度) */
    int16_t   calib_lower_x10;                /* 0x272A: 关窗过流校准有效角度下限 (0.1度) */

    /* 尾部信息 */
    uint32_t checksum;              /* CRC32校验和 */
    uint32_t tail_magic;            /* 尾部魔数 (0xAA44AA44) */
} AppParamRecord_t;
#pragma pack()

/*=============================================================================
 * 实时数据结构体 (不存 Flash，运行时使用)
 *============================================================================*/
typedef struct {
    int16_t  real_speed;        /* 0x2730: 实时转速 (r/min) */
    int16_t  real_angle;        /* 0x2731: 实时角度 (0.1度) */
    uint16_t real_voltage;      /* 0x2732: 电压 (0.1V) */
    uint16_t real_current;      /* 0x2733: 电流 (1mA) */
    int16_t  real_direction;    /* 0x2737: 实时方向 */
    uint16_t fault_status;      /* 0x2740: 故障状态 (见 FAULT_BIT_xxx 定义) */
} AppRealTimeData_t;

/*=============================================================================
 * 模拟实时数据结构体 (用于 Keil Debug 环境修改)
 *============================================================================*/
typedef struct {
    volatile int16_t  speed;        /* 实时转速 r/min */
    volatile int16_t  angle;        /* 实时角度 0.1度 */
    volatile int16_t  angle_abs;    /* 绝对角度 (0.1度) - 累加值 */
    volatile uint16_t voltage;      /* 电压 0.1V */
    volatile uint16_t current;      /* 电流 1mA */
    volatile int16_t  direction;    /* 实时方向 */
    volatile uint16_t fault_set;    /* 故障设置: 写 FAULT_BIT_xxx 可置位对应故障位 */
    volatile uint16_t fault_clear;  /* 故障清除: 写 FAULT_BIT_xxx 可清除对应故障位 */
    volatile uint16_t ctrl_cmd;     /* 控制命令模拟 (参考 REG_CTRL_CMD 定义) */
} SimRealtimeData_t;

/*=============================================================================
 * 全局变量声明
 *============================================================================*/
extern AppParamRecord_t g_AppParam;         /* 应用参数 (Flash存储) */
extern AppRealTimeData_t g_RealTimeData;    /* 实时数据 (不存Flash) */
extern volatile int32_t g_s32HallPulseAccum;  /* 霍尔脉冲累计值 */
extern SimRealtimeData_t g_SimRealtimeData; /* 模拟实时数据 (Keil Debug 可修改) */

/*=============================================================================
 * 寄存器读写接口函数
 *============================================================================*/

/**
 * @brief     根据寄存器地址读取寄存器值
 * @param  regAddr   寄存器地址 (REG_xxx)
 * @param  pValue    输出值指针
 * @return 0 成功; -3 无效地址
 */
int32_t Param_ReadByReg(uint16_t regAddr, uint16_t *pValue);

/**
 * @brief     根据寄存器地址写入寄存器值
 * @param  regAddr   寄存器地址 (REG_xxx)
 * @param  value     要写入的值
 * @return 0 成功; -3 无效地址
 */
int32_t Param_WriteByReg(uint16_t regAddr, uint16_t value);

/*=============================================================================
 * 实时数据操作函数
 *============================================================================*/

/**
 * @brief   设置故障位为1(置位)
 * @param  bitMask     故障位掩码 (FAULT_BIT_xxx)
 */
void RealTime_SetFault(uint16_t bitMask);

/**
 * @brief   清除故障位为0(清零)
 * @param  bitMask     故障位掩码 (FAULT_BIT_xxx)
 */
void RealTime_ClearFault(uint16_t bitMask);

/**
 * @brief   检查故障位
 * @param  bitMask     故障位掩码 (FAULT_BIT_xxx)
 * @retval true       该故障位为1
 * @retval false      该故障位为0
 */
bool RealTime_CheckFault(uint16_t bitMask);

/**
 * @brief     更新实时数据 (从传感器或模拟数据更新)
 * @param  speed       实时转速 (r/min), 传 NULL 表示不更新
 * @param  angle       实时角度 (0.1度), 传 NULL 表示不更新
 * @param  voltage     电压 (0.1V), 传 NULL 表示不更新
 * @param  current     电流 (1mA), 传 NULL 表示不更新
 * @param  direction   实时方向, 传 NULL 表示不更新
 */
void RealTime_Update(const int16_t  *speed,
                     const int16_t  *angle,
                     const uint16_t *voltage,
                     const uint16_t *current,
                     const int16_t  *direction);

/**
 * @brief     从设备读取实时数据并更新到 g_RealTimeData
 *            APP_PARAMS_USE_DEV_RTURN 开启时生效
 * @param  u8RTurnDevId   dev_rturn 设备 ID
 * @param  u8VoltageDevId dev_voltage 设备 ID
 * @param  u8SensorDevId  dev_sensor 设备 ID
 */
void RealTime_UpdateFromDevice(uint8_t u8RTurnDevId,
                               uint8_t u8VoltageDevId,
                               uint8_t u8SensorDevId);

/**
 * @brief    打印实时数据 (调试用)
 */
void RealTime_PrintDebug(void);

/**
 * @brief  实时数据模拟 (每5秒角度+1，内部自动处理方向位反转)
 *            APP_PARAMS_REALTIME_SIMULATE 开启时生效
 */
void RealTime_Simulate(void);

/*=============================================================================
 * 模拟实时数据操作函数 (用于 Keil Debug 环境修改)
 *============================================================================*/

/**
 * @brief    初始化模拟实时数据
 */
void SimRealtime_Init(void);

/**
 * @brief  同步模拟数据到真实实时数据 (在主循环中调用)
 *            将 g_SimRealtimeData 中的值同步到 g_RealTimeData
 */
void SimRealtime_Sync(void);

/**
 * @brief    打印模拟实时数据当前值(调试用)
 */
void SimRealtime_PrintDebug(void);

/*=============================================================================
 * 状态查询函数 (用于绝对角度控制和模拟使用)
 *============================================================================*/

/**
 * @brief    查询 RS485 控制是否已解锁
 * @retval true    已解锁 (bit3~bit5 允许写入)
 * @retval false   未解锁 (bit3~bit5 禁止写入)
 */
bool Param_IsCtrlUnlocked(void);

/**
 * @brief    查询电机是否处于停止状态
 *         同时检查 dev_motor_hall 和 dev_motor 状态
 * @retval true      电机停止
 * @retval false     电机正转或反转
 */
bool Param_IsMotorStopped(void);

#endif /* __APP_PARAMS_H__ */
