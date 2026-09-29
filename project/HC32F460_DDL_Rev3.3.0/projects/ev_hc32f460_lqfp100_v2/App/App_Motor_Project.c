#include "App_Motor_Project.h"
#include "hc32_ll.h"
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include "rtt_log.h"
#include "dev_rturn.h"          // 旋转限位设备
#include "App_Params.h"         // 全局参数 g_AppParam 和 Modbus 寄存器
#include "App_RunAngle.h"
#include "App_MotionTimeout.h"  // 控制超时故障检测

SystemSim_t g_sim = {
    .sim_pwr_pos = 0,
    .sim_pwr_neg = 0,
    .sim_hall_up = 0,
    .sim_hall_down = 0,
    .sim_io_fwd = 0,
    .sim_io_rev = 0,
    .sim_io_speed = 85.0f,
    .sim_adc_val = 1500,
    .sim_motor_speed = 0,
    .sim_motor_dir = 0
};

SystemStatus_t g_status = {
    .motor_status = 0,
    .power_status = 0,
    .hall_status = 0,
    .io_status = 0,
    .current_duty = 0.0f
};

// 旋转限位设备全局指针
RTurn_Device_t* g_rturn_dev = NULL;

// ========== 静态设备实例 ==========
static PWM_Device_t g_pwm_dev;
static MotorDevice_t g_motor_dev;
// Keil Watch 中可直接观察电机仲裁器内部状态
MotorDevice_t* volatile g_pMotorDevWatch = NULL;
static Power_Device_t* g_pwr_pos_dev = NULL;
static Power_Device_t* g_pwr_neg_dev = NULL;
static IO_Device_t* g_io_fwd_dev = NULL;
static IO_Device_t* g_io_rev_dev = NULL;
static Hall_Device_t* g_hall_up_dev = NULL;
static Hall_Device_t* g_hall_down_dev = NULL;
static Voltage_Device_t* g_voltage_bus_dev = NULL;
static Sensor_Device_t* g_sensor_current_dev = NULL;
static MotorHall_Device_t* g_motor_hall_dev = NULL;

ADC_Device_t* g_adc_current_dev = NULL;
ADC_Device_t* g_adc_voltage_dev = NULL;

// ========== 内部函数声明 ==========
static void RegisterAllDevices(void);
static void SetupEventBusSubscriptions(void);
static void UpdateStatusIndicators(void);
static void ProcessDeviceUpdates(void);
static void SetDeviceUpdateIntervals(void);

// ========== 注册所有设备 ==========

static void RegisterAllDevices(void) {
    // // 1. 注册 PWM 设备（已由 main.c 直接初始化）
    // PWM_Config_t pwmConfig = {
    //     .tmr = NULL,  // 暂不配置，由外部初始化
    //     .periphClk = 0,
    //     .channel = 0,
    //     .port = 0,
    //     .pin = 0,
    //     .pinFunc = 0,
    //     .countMode = 0,
    //     .countDir = 0,
    //     .defaultFreqHz = 20000,
    //     .defaultDutyPercent = 0,
    //     .activePolarity = PWM_ACTIVE_HIGH
    // };

    // memcpy(&g_pwm_dev.config, &pwmConfig, sizeof(PWM_Config_t));

    // DeviceOps_t pwm_ops = {
    //     .init = PWM_Device_Init,
    //     .deinit = PWM_Device_Deinit,
    //     .read = PWM_Device_Read,
    //     .write = PWM_Device_Write,
    //     .control = PWM_Device_Control,
    //     .update = PWM_Device_Update
    // };
    // DeviceManager_Register(ID_PWM_MOTOR, "PWM_Motor", DEVICE_TYPE_PWM, &g_pwm_dev, pwm_ops);

    // 2. 注册正电源设备
    Power_Config_t pwrPosCfg = {
        .port = GPIO_PORT_C,
        .pin = GPIO_PIN_13,
        .active_level = 1,
        .power_id = 0,
        .debounce_ms = 20,
        .window_size = 5,
        .sample_interval = 5
    };
    g_pwr_pos_dev = Power_Device_Create(&pwrPosCfg);
    DeviceManager_Register(ID_PWR_POS, "PowerPositive", DEVICE_TYPE_POWER,
                           g_pwr_pos_dev, g_power_ops);

    // // 3. 注册负电源设备（暂未使用 PB2）
    // Power_Config_t pwrNegCfg = {
    //     .port = GPIO_PORT_C,
    //     .pin = GPIO_PIN_14,
    //     .active_level = 1,
    //     .power_id = 1,
    //     .debounce_ms = 20,
    //     .window_size = 5,
    //     .sample_interval = 5
    // };
    // g_pwr_neg_dev = Power_Device_Create(&pwrNegCfg);
    // DeviceManager_Register(ID_PWR_NEG, "PowerNegative", DEVICE_TYPE_POWER,
    //                        g_pwr_neg_dev, g_power_ops);

    // 4. 注册霍尔限位设备（已废弃，使用旋转限位替代）

    // Hall_Config_t upCfg = {
    //     .port = GPIO_PORT_B,
    //     .pin = GPIO_PIN_02,
    //     .active_level = 0,           // 低电平触发（取决于硬件设计 - 常闭型）
    //     .bind_dir = DIR_FWD,
    //     .is_soft_limit = 0,
    //     .debounce_ms = 20,
    //     .window_size = 3,
    //     .sample_interval = 2,
    //     .hall_id = 0
    // };
    // g_hall_up_dev = Hall_Device_Create(&upCfg);
    // DeviceManager_Register(ID_HALL_UP, "LimitUp", DEVICE_TYPE_HALL,
    //                     g_hall_up_dev, g_hall_ops);

    // // 5. 注册下限位开关 (PC14)
    // Hall_Config_t downCfg = {
    //     .port = GPIO_PORT_B,
    //     .pin = GPIO_PIN_10,
    //     .active_level = 0,           // 低电平触发（取决于硬件设计 - 常闭型）
    //     .bind_dir = DIR_REV,
    //     .is_soft_limit = 0,
    //     .debounce_ms = 20,
    //     .window_size = 3,
    //     .sample_interval = 2,
    //     .hall_id = 1
    // };
    // g_hall_down_dev = Hall_Device_Create(&downCfg);
    // DeviceManager_Register(ID_HALL_DOWN, "LimitDown", DEVICE_TYPE_HALL,
    //                     g_hall_down_dev, g_hall_ops);

    // // 6. 注册正转IO设备
    // IO_Config_t ioFwdCfg = {
    //     .port = GPIO_PORT_B,
    //     .pin = GPIO_PIN_13,
    //     .active_level = 1,
    //     .debounce_ms = 20,
    //     .window_size = 3,
    //     .sample_interval = 2,
    //     .io_id = 0
    // };
    // g_io_fwd_dev = IO_Device_Create(&ioFwdCfg);
    // DeviceManager_Register(ID_IO_FWD, "IO_Forward", DEVICE_TYPE_IO,
    //                        g_io_fwd_dev, g_io_ops);

    // // 7. 注册反转IO设备
    // IO_Config_t ioRevCfg = {
    //     .port = GPIO_PORT_B,
    //     .pin = GPIO_PIN_12,
    //     .active_level = 1,
    //     .debounce_ms = 20,
    //     .window_size = 3,
    //     .sample_interval = 2,
    //     .io_id = 1
    // };
    // g_io_rev_dev = IO_Device_Create(&ioRevCfg);
    // DeviceManager_Register(ID_IO_REV, "IO_Reverse", DEVICE_TYPE_IO,
    //                        g_io_rev_dev, g_io_ops);

    // 8. 注册电机仲裁设备
    DeviceOps_t motor_ops = {
        .init = Motor_Init,
        .deinit = Motor_Deinit,
        .read = Motor_Read,
        .write = Motor_Write,
        .control = Motor_Control,
        .update = Motor_Update
    };
    DeviceManager_Register(ID_MOTOR, "MotorArbitrator", DEVICE_TYPE_MOTOR,
                           &g_motor_dev, motor_ops);
    // Keil Watch 中设置全局指针以便观察
    g_pMotorDevWatch = &g_motor_dev;

    // 注册电机霍尔设备
    motor_hall_config_t motorHallCfg = {
        .hall_a_port = PIN_HALL_A_PORT,
        .hall_a_pin = PIN_HALL_A_PIN,
        .hall_b_port = PIN_HALL_B_PORT,
        .hall_b_pin = PIN_HALL_B_PIN,
        .eirq_ch_a = HALL_EIRQ_CH_A,
        .eirq_ch_b = HALL_EIRQ_CH_B,
        .irqn_a = HALL_IRQN_A,
        .irqn_b = HALL_IRQN_B,
        .irq_src_a = HALL_IRQ_SRC_A,
        .irq_src_b = HALL_IRQ_SRC_B,
        .irq_priority = 2,
        .pole_pairs = (uint8_t)g_AppParam.motor_hall_pole_pairs,  // 从Flash读取
        .hall_count = 2,
        .custom_pulses_per_rev = 0
    };

    MotorHall_DeviceConfig_t motorHallDevCfg = {
        .motor_id = 0,
        .update_interval_ms = 1
    };

    g_motor_hall_dev = MotorHall_Device_Create(&motorHallCfg, &motorHallDevCfg);

    MAIN_D("Before Register: motor_hall_dev=0x%p\r\n", g_motor_hall_dev);
    MAIN_D("Before Register: motor_hall_dev->config.hall_a_pin=0x%04X\r\n", g_motor_hall_dev->config.hall_a_pin);

    DeviceManager_Register(ID_MOTOR_HALL, "MotorHall", DEVICE_TYPE_HALL,
                        g_motor_hall_dev, g_motor_hall_ops);

    // 验证注册是否成功
    DeviceNode_t* check_node = DeviceManager_Get(ID_MOTOR_HALL);
    if (check_node && check_node->private_data) {
        MotorHall_Device_t* check_dev = (MotorHall_Device_t*)check_node->private_data;
        MAIN_D("After Register: check_dev=0x%p\r\n", check_dev);
        MAIN_D("After Register: check_dev->config.hall_a_pin=0x%04X\r\n", check_dev->config.hall_a_pin);
    }

    // ============================================================
    // ADC 设备注册
    // 注意 ADC 硬件初始化在 dev_adc.c 的 ADC_AdpLayerInit() 中完成
    // 这里只注册设备，ADC 转换由定时器触发或软件触发
    // ============================================================

    // 电流检测 ADC (PA5, CH5) - 电流传感器
    ADC_Config_t adcCurrentCfg = {
        .u8AdcId = 0,                       // 对应 ADC_AdpLayerInit 中的索引
        .u8Channel = 5,                     // ADC_CH5 = PA5
        .u8Port = GPIO_PORT_A,
        .u16Pin = GPIO_PIN_05,
        .enAcqMode = ADC_ACQ_MODE_INTERRUPT, // 中断模式
        .u16DmaBufferSize = 0,              // 不使用DMA
        .u8DmaChannel = 0                   // 不使用DMA
    };

    ADC_Device_t* adc_current_dev = ADC_Device_Create(&adcCurrentCfg);
    g_adc_current_dev = adc_current_dev;
    DeviceManager_Register(ID_ADC_CURRENT, "ADC_Current", DEVICE_TYPE_ADC,
                        adc_current_dev, g_adc_ops);

    // 电压检测 ADC
    //   HB_chuchai 板使用 PA04, CH4
    //   整合板 HandB 使用 PA06, CH6
    //   根据主板的 BOARD_VERSION 自动选择 PA04+CH4 或 PA06+CH6
    ADC_Config_t adcVoltageCfg = {
        .u8AdcId = 1,                       // 对应 ADC_AdpLayerInit 中的索引
        .u8Channel = PIN_ADC_VOLTAGE_CH,                     // ADC_CH6 = PA06 (整合板); HB_chuchai: CH4 = PA04
        .u8Port = PIN_ADC_VOLTAGE_PORT,
        .u16Pin = PIN_ADC_VOLTAGE_PIN,              // PA06 (整合板); HB_chuchai: GPIO_PIN_04
        .enAcqMode = ADC_ACQ_MODE_INTERRUPT, // 中断模式
        .u16DmaBufferSize = 0,              // 不使用DMA
        .u8DmaChannel = 0                   // 不使用DMA
    };

    ADC_Device_t* adc_voltage_dev = ADC_Device_Create(&adcVoltageCfg);
    g_adc_voltage_dev = adc_voltage_dev;
    DeviceManager_Register(ID_ADC_VOLTAGE, "ADC_Voltage", DEVICE_TYPE_ADC,
                        adc_voltage_dev, g_adc_ops);

    // 创建电压母线设备（基于 ADC_VOLTAGE 计算）
    // 从 Flash 参数 g_AppParam 中读取阈值
    {
        // 从 Flash 读取参数，单位 0.1V -> mV
        uint32_t u32OvervoltageThresholdMv = (uint32_t)g_AppParam.voltage_upper_limit * 100UL;
        uint32_t u32UndervoltageThresholdMv = (uint32_t)g_AppParam.voltage_lower_limit * 100UL;
        uint32_t u32OvervoltageHysteresisMv = (uint32_t)g_AppParam.voltage_upper_hysteresis * 100UL;
        uint32_t u32UndervoltageHysteresisMv = (uint32_t)g_AppParam.voltage_lower_hysteresis * 100UL;
        uint8_t u8OvervoltageTriggerCount = g_AppParam.overvoltage_trigger_count;
        uint8_t u8UndervoltageTriggerCount = g_AppParam.undervoltage_trigger_count;

        // 从 Flash 读取的值可能为 0，使用默认值补充
        if (u32OvervoltageThresholdMv == 0) {
            u32OvervoltageThresholdMv = PARAM_DEFAULT_VOLTAGE_UPPER_LIMIT * 100UL;
            PARAMS_DBG("[VOLTAGE] Overvoltage threshold is 0, using default: %lu mV", u32OvervoltageThresholdMv);
        }
        if (u32UndervoltageThresholdMv == 0) {
            u32UndervoltageThresholdMv = PARAM_DEFAULT_VOLTAGE_LOWER_LIMIT * 100UL;
            PARAMS_DBG("[VOLTAGE] Undervoltage threshold is 0, using default: %lu mV", u32UndervoltageThresholdMv);
        }
        if (u32OvervoltageHysteresisMv == 0) {
            u32OvervoltageHysteresisMv = PARAM_DEFAULT_VOLTAGE_UPPER_HYSTERESIS * 100UL;
        }
        if (u32UndervoltageHysteresisMv == 0) {
            u32UndervoltageHysteresisMv = PARAM_DEFAULT_VOLTAGE_LOWER_HYSTERESIS * 100UL;
        }
        /* 过压确认窗固定为29ms: 1ms节拍 x 连续30次越限(首末样本跨度>=29ms)
         * Flash 中可能存有旧默认值 0~30，向上拉到当前默认值 30 */
        if (u8OvervoltageTriggerCount < PARAM_DEFAULT_OVERVOLTAGE_TRIGGER_CNT) {
            u8OvervoltageTriggerCount = PARAM_DEFAULT_OVERVOLTAGE_TRIGGER_CNT;
        }
        /* [欠压判定窗 11~12ms] 1ms节拍 x 连续12次越限(首末样本跨度=11ms)
         * 触发延迟理想 11~12ms; 欠压计数直接采用默认值、不读Flash:
         * Flash 可能存有旧值(1/3/5/6/11/30), 下限保护只能向上拉,
         * 无法把旧值30降回12, 故欠压改为强制赋值 */
        u8UndervoltageTriggerCount = PARAM_DEFAULT_UNDERVOLTAGE_TRIGGER_CNT;

        MAIN_D("[APP] Voltage config from Flash: upper=%lu mV (0.1V=%d), lower=%lu mV (0.1V=%d), "
            "upper_hys=%lu mV, lower_hys=%lu mV, ov_cnt=%d, uv_cnt=%d",
            u32OvervoltageThresholdMv, g_AppParam.voltage_upper_limit,
            u32UndervoltageThresholdMv, g_AppParam.voltage_lower_limit,
            u32OvervoltageHysteresisMv, u32UndervoltageHysteresisMv,
            u8OvervoltageTriggerCount, u8UndervoltageTriggerCount);

        Voltage_Config_t voltageBusCfg = {
            .u8AdcDevId = ID_ADC_VOLTAGE,

            // 过压配置
            .u32OvervoltageThresholdMv = u32OvervoltageThresholdMv,
            .u32OvervoltageHysteresisMv = u32OvervoltageHysteresisMv,
            .u8OvervoltageTriggerCount = u8OvervoltageTriggerCount,

            // 欠压配置
            .u32UndervoltageThresholdMv = u32UndervoltageThresholdMv,
            .u32UndervoltageHysteresisMv = u32UndervoltageHysteresisMv,
            .u8UndervoltageTriggerCount = u8UndervoltageTriggerCount,
        };
        g_voltage_bus_dev = Voltage_Device_Create(&voltageBusCfg);
        DeviceManager_Register(ID_VOLTAGE_BUS, "VoltageBus", DEVICE_TYPE_SENSOR,
                            g_voltage_bus_dev, g_voltage_ops);
    }

    // 创建电流传感器设备
    // 从 Modbus 寄存器读取阈值参数
    {
        int32_t s32ThresholdMa = (int32_t)g_AppParam.current_upper_limit;
        uint32_t u32TriggerMs = (uint32_t)g_AppParam.current_detect_ms;
        uint32_t u32ReleaseMs = (uint32_t)g_AppParam.current_release_ms;
        int32_t s32HysteresisMa = (int32_t)g_AppParam.current_hysteresis_ma;
        uint8_t u8TriggerCount = g_AppParam.overcurrent_trigger_count;

        // 从 Flash 读取的值可能为 0，使用默认值补充
        // 注意 int32_t 类型 -1 表示未初始化，g_AppParam 默认值在 Param_Init 中设置
        if (s32ThresholdMa < 0) {
            s32ThresholdMa = PARAM_DEFAULT_CURRENT_UPPER_LIMIT;
            PARAMS_DBG("[CURRENT] Threshold is invalid (%ld), using default: %ld mA",
                       (long)s32ThresholdMa, (long)PARAM_DEFAULT_CURRENT_UPPER_LIMIT);
        }
        if (u32TriggerMs == 0) {
            u32TriggerMs = PARAM_DEFAULT_CURRENT_DETECT_MS;
        }
        if (u32ReleaseMs == 0) {
            u32ReleaseMs = PARAM_DEFAULT_CURRENT_RELEASE_MS;
        }
        if (s32HysteresisMa == 0) {
            s32HysteresisMa = PARAM_DEFAULT_CURRENT_HYSTERESIS_MA;
        }
        if (u8TriggerCount == 0) {
            u8TriggerCount = PARAM_DEFAULT_OVERCURRENT_TRIGGER_CNT;
        }

        MAIN_D("[APP] Sensor config from Flash: threshold=%ldmA, trigger_window=%ldms, release_window=%ldms, hysteresis=%ldmA, trigger_cnt=%d",
            (long)s32ThresholdMa, (long)u32TriggerMs, (long)u32ReleaseMs, (long)s32HysteresisMa, (int)u8TriggerCount);

        Sensor_Config_t sensorCurrentCfg = {
            .u8AdcDevId = ID_ADC_CURRENT,

            .s32OvercurrentThresholdMa = s32ThresholdMa,
            .s32OvercurrentHysteresisMa = s32HysteresisMa,

            .u8OvercurrentMode = OVERCURRENT_MODE_TIME_WINDOW,  // 使用时间窗口模式

            // 计数模式参数（时间窗口模式不使用）
            .u16TriggerWindowSize = 0,
            .u16ReleaseWindowSize = 0,

            // 时间窗口模式参数
            .u32TriggerWindowMs = u32TriggerMs,
            .u32ReleaseWindowMs = u32ReleaseMs,
        };
        g_sensor_current_dev = Sensor_Device_Create(&sensorCurrentCfg);
        DeviceManager_Register(ID_SENSOR_CURRENT, "SensorCurrent", DEVICE_TYPE_SENSOR,
                            g_sensor_current_dev, g_sensor_ops);
    }

    // 注册旋转限位设备
    RTurn_Config_t rturnCfg = {
        .u8MotorHallDevId = ID_MOTOR_HALL,                      // 霍尔脉冲输入
        .u8MotorArbiterDevId = ID_MOTOR,                        // 电机仲裁器（获取期望方向）
        .u8SensorDevId = ID_SENSOR_CURRENT,                     // 电流传感器（过流检测）
        .fReductionRatio = (float)g_AppParam.rturn_reduction_ratio / 10.0f,  // 从 Flash 读取减速比
        .fMaxAngle = (float)g_AppParam.open_limit_angle / 10.0f,                           // 开窗极限
        .fMinAngle = (float)g_AppParam.close_limit_angle / 10.0f,                           // 关窗极限
        .u8ReverseOutput = RTURN_REVERSE_OUTPUT,                // 是否反转输出
        .u8DeviceId = ID_RTURN,
        .u16UpdateIntervalMs = RTURN_UPDATE_INTERVAL_MS         // 更新间隔
    };

    g_rturn_dev = RTurn_Device_Create(&rturnCfg);
    DeviceManager_Register(ID_RTURN, "RTurn", DEVICE_TYPE_SENSOR,
                        g_rturn_dev, g_rturn_ops);
}

// ========== EventBus 订阅设置 ==========
static void SetupEventBusSubscriptions(void) {
    // 按优先级注册（优先级 0 最高，数值越大优先级越低）
    EventBus_Subscribe(TOPIC_POWER, Motor_OnPowerEvent, 0);
    EventBus_Subscribe(TOPIC_LIMIT_HARD, Motor_OnHardLimit, 0);
    EventBus_Subscribe(TOPIC_LIMIT_SOFT, Motor_OnHardLimit, 0);
    EventBus_Subscribe(TOPIC_MANUAL_IO, Motor_OnManualIO, 0);
    EventBus_Subscribe(TOPIC_MOTOR_SPEED_FEEDBACK, Motor_OnSpeedFeedback, 0);
    // 通用过流事件（已废弃，使用定向过流）
    EventBus_Subscribe(TOPIC_ALARM, Motor_OnOvercurrent, 0);
    EventBus_Subscribe(TOPIC_VOLTAGE_ALARM, Motor_OnVoltageAlarm, 0);
    // 电流告警由旋转限位处理
    EventBus_Subscribe(TOPIC_CURRENT_ALARM, RTurn_OnCurrentAlarm, 1);
    // EventBus_Subscribe(TOPIC_CURRENT_ALARM, Motor_OnCurrentAlarm, 1);  // 已禁用: 开窗过流由 RTurn 锁定, 关窗过流由 RTurn 校准/报故障

    // 旋转限位事件由电机仲裁器处理
    EventBus_Subscribe(TOPIC_RTURN_LIMIT, Motor_OnRTurnLimit, 0);
    // 正转(开窗)过流 + 反转(关窗)过流: 双向封锁电机
    EventBus_Subscribe(TOPIC_OVERCURRENT_FWD, Motor_OnOvercurrentFwd, 0);
    EventBus_Subscribe(TOPIC_OVERCURRENT_REV, Motor_OnOvercurrentRev, 0);
    // RS485 手动控制通过 Modbus REG_CTRL_CMD 触发
    EventBus_Subscribe(TOPIC_MANUAL_RS485, Motor_OnManualIO, 0);
}

// ========== 设置设备更新间隔 ==========
static void SetDeviceUpdateIntervals(void) {
    DeviceManager_SetUpdateInterval(ID_PWR_POS, 1);
    DeviceManager_SetUpdateInterval(ID_PWR_NEG, 1);
    // 限位开关已禁用（改用旋转限位）
    // DeviceManager_SetUpdateInterval(ID_HALL_UP, 1);    // 上限位
    // DeviceManager_SetUpdateInterval(ID_HALL_DOWN, 1);  // 下限位
    // DeviceManager_SetUpdateInterval(ID_IO_FWD, 1000);
    // DeviceManager_SetUpdateInterval(ID_IO_REV, 1000);
    // DeviceManager_SetUpdateInterval(ID_PWM_MOTOR, 10000);
    DeviceManager_SetUpdateInterval(ID_MOTOR, 1);
    DeviceManager_SetUpdateInterval(ID_MOTOR_HALL, 1);  // 1ms

    DeviceManager_SetUpdateInterval(ID_ADC_CURRENT, 1);   // ADC 快速采样
    DeviceManager_SetUpdateInterval(ID_ADC_VOLTAGE, 1);   // ADC 快速采样

    // 传感器设备使用较慢的更新间隔
    DeviceManager_SetUpdateInterval(ID_VOLTAGE_BUS, 1);    // 1ms 检测一次电压 (1ms节拍x连续30次越限=29ms确认窗)
    DeviceManager_SetUpdateInterval(ID_SENSOR_CURRENT, 1); // 10ms 检测一次电流
    DeviceManager_SetUpdateInterval(ID_RTURN, 1);          // 1ms 更新一次角度
}

// ========== 模拟模式处理函数 ==========
#if ENABLE_SIMULATION_MODE

void Sim_ProcessInput(void) {
    static uint32_t last_update = 0;
    uint32_t now = tickTimer_GetCount();

    if (now - last_update < 100) return;
    last_update = now;
}

void Sim_PublishEvents(void) {
    uint32_t now = tickTimer_GetCount();
    (void)now;  // 避免编译警告

    // 电源事件
    static uint8_t last_pwr_pos = 0;
    static uint8_t last_pwr_neg = 0;

    if (g_sim.sim_pwr_pos != last_pwr_pos) {
        MotorPowerEvent_t pwrEvent = {.power_id = 0, .is_on = (g_sim.sim_pwr_pos != 0)};
        EventBus_Publish(TOPIC_POWER, &pwrEvent);
        MAIN_D("[SIM] POWER_POS changed: %d -> %d\r\n", last_pwr_pos, g_sim.sim_pwr_pos);
        last_pwr_pos = g_sim.sim_pwr_pos;
    }

    if (g_sim.sim_pwr_neg != last_pwr_neg) {
        MotorPowerEvent_t pwrEvent = {.power_id = 1, .is_on = (g_sim.sim_pwr_neg != 0)};
        EventBus_Publish(TOPIC_POWER, &pwrEvent);
        MAIN_D("[SIM] POWER_NEG changed: %d -> %d\r\n", last_pwr_neg, g_sim.sim_pwr_neg);
        last_pwr_neg = g_sim.sim_pwr_neg;
    }

    // 限位事件
    static uint8_t last_hall_up = 0;
    static uint8_t last_hall_down = 0;

    if (g_sim.sim_hall_up != last_hall_up) {
        MotorLimitEvent_t limitEvent = {.dir = DIR_FWD, .is_active = (g_sim.sim_hall_up != 0)};
        EventBus_Publish(TOPIC_LIMIT_HARD, &limitEvent);
        MAIN_D("[SIM] HALL_UP changed: %d -> %d\r\n", last_hall_up, g_sim.sim_hall_up);
        last_hall_up = g_sim.sim_hall_up;
    }

    if (g_sim.sim_hall_down != last_hall_down) {
        MotorLimitEvent_t limitEvent = {.dir = DIR_REV, .is_active = (g_sim.sim_hall_down != 0)};
        EventBus_Publish(TOPIC_LIMIT_HARD, &limitEvent);
        MAIN_D("[SIM] HALL_DOWN changed: %d -> %d\r\n", last_hall_down, g_sim.sim_hall_down);
        last_hall_down = g_sim.sim_hall_down;
    }

    // IO 事件 - 使用边沿触发
    static uint8_t last_io_fwd = 0;
    static uint8_t last_io_rev = 0;

    // 正转IO事件
    if (g_sim.sim_io_fwd != last_io_fwd) {
        MAIN_D("[SIM] IO_FWD changed: %d -> %d\r\n", last_io_fwd, g_sim.sim_io_fwd);

        MotorManualIOEvent_t ioEvent;
        memset(&ioEvent, 0, sizeof(MotorManualIOEvent_t));  // 清空结构体

        if (g_sim.sim_io_fwd) {
            ioEvent.dir = DIR_FWD;
            ioEvent.type = CMD_TYPE_RUN_FWD;
            ioEvent.speed = g_sim.sim_io_speed;
        } else {
            ioEvent.dir = DIR_FWD;
            ioEvent.type = CMD_TYPE_STOP;
            ioEvent.speed = 0.0f;
        }

        // 打印调试信息
        int32_t speed_int = (int32_t)(ioEvent.speed * 10);
        MAIN_D("[SIM] Publishing IO_FWD event: dir=%d, type=%d, speed=%ld.%ld%%\r\n",
               ioEvent.dir, ioEvent.type, (long)(speed_int / 10), (long)(speed_int % 10));

        EventBus_Publish(TOPIC_MANUAL_IO, &ioEvent);
        last_io_fwd = g_sim.sim_io_fwd;
    }

    // 反转IO事件
    if (g_sim.sim_io_rev != last_io_rev) {
        MAIN_D("[SIM] IO_REV changed: %d -> %d\r\n", last_io_rev, g_sim.sim_io_rev);

        MotorManualIOEvent_t ioEvent;
        memset(&ioEvent, 0, sizeof(MotorManualIOEvent_t));  // 清空结构体

        if (g_sim.sim_io_rev) {
            ioEvent.dir = DIR_REV;
            ioEvent.type = CMD_TYPE_RUN_REV;
            ioEvent.speed = g_sim.sim_io_speed;
        } else {
            ioEvent.dir = DIR_REV;
            ioEvent.type = CMD_TYPE_STOP;
            ioEvent.speed = 0.0f;
        }

        // 打印调试信息
        int32_t speed_int = (int32_t)(ioEvent.speed * 10);
        MAIN_D("[SIM] Publishing IO_REV event: dir=%d, type=%d, speed=%ld.%ld%%\r\n",
               ioEvent.dir, ioEvent.type, (long)(speed_int / 10), (long)(speed_int % 10));

        EventBus_Publish(TOPIC_MANUAL_IO, &ioEvent);
        last_io_rev = g_sim.sim_io_rev;
    }
}

#endif

// ========== 更新状态指示器 ==========

static void UpdateStatusIndicators(void) {
    static uint32_t last_update = 0;
    uint32_t now = tickTimer_GetCount();

    if (now - last_update < 50) return;
    last_update = now;

    // 从电机仲裁器读取状态
    const MotorDebugInfo_t* dbg = Motor_GetDebugInfo(&g_motor_dev);
    if (dbg) {
        if (dbg->state == MS_IDLE) {
            g_status.motor_status = MOTOR_STOPPED;
        } else if (dbg->active_dir == DIR_FWD) {
            g_status.motor_status = MOTOR_FORWARD;
        } else if (dbg->active_dir == DIR_REV) {
            g_status.motor_status = MOTOR_REVERSE;
        }
        g_status.current_duty = dbg->current_duty;
    }

    // 电源状态（模拟模式）
    if (g_sim.sim_pwr_pos && g_sim.sim_pwr_neg) {
        g_status.power_status = POWER_BOTH_ON;
    } else if (g_sim.sim_pwr_pos) {
        g_status.power_status = POWER_POS_ON;
    } else if (g_sim.sim_pwr_neg) {
        g_status.power_status = POWER_NEG_ON;
    } else {
        g_status.power_status = POWER_BOTH_OFF;
    }

    // 限位状态（模拟模式）
    if (g_sim.sim_hall_up && g_sim.sim_hall_down) {
        g_status.hall_status = HALL_BOTH_LIMIT;
    } else if (g_sim.sim_hall_up) {
        g_status.hall_status = HALL_UP_LIMIT;
    } else if (g_sim.sim_hall_down) {
        g_status.hall_status = HALL_DOWN_LIMIT;
    } else {
        g_status.hall_status = HALL_NO_LIMIT;
    }

    g_status.io_status = (g_sim.sim_io_fwd ? 1 : (g_sim.sim_io_rev ? 2 : 0));
}

static void ProcessDeviceUpdates(void) {
    DeviceManager_UpdateAll();
}

// ========== 系统初始化函数 ==========
void ESystem_Init(void) {
    // 1. 先初始化 EventBus
    EventBus_Init();

    // 2. 初始化 DeviceManager，并关联 EventBus
    DeviceManagerConfig_t config = {
        .operation_timeout_ms = 500,
        .enable_mutex = 1,
        .auto_subscribe = 0
    };
    DeviceManager_Init(&config);

    // 3. 注册所有设备
    RegisterAllDevices();

    // 4. 设置 EventBus 订阅（必须在设备注册之后）
    SetupEventBusSubscriptions();

    // 6. 先初始化电机仲裁器
    //  因为其他设备可能依赖仲裁器状态
    MAIN_D("[APP] Initializing motor arbiter first...\r\n");
    Device_Init(ID_MOTOR);

    // 再初始化其他所有设备
    MAIN_D("[APP] Initializing remaining devices...\r\n");
    for (uint8_t i = 0; i < MAX_DEVICES; i++) {
        if (i == ID_MOTOR) continue;  // 已初始化
        Device_Init(i);
    }

    // 6. 设置各设备的更新间隔
    SetDeviceUpdateIntervals();

    // 7. 启用所有设备的自动更新
    DeviceManager_EnableAllUpdate();

    // 8. 初始化绝对角度模块（在 param_manager 就绪后）
    RunAngle_Init();

    // 9. 初始化控制超时检测看门狗
    MotionTimeout_Init();

    memset(&g_status, 0, sizeof(SystemStatus_t));
}

// ========== 主循环函数 ==========
// ========== 配置重载函数 ==========
/**
 * @brief    当 g_AppParam 被 Modbus 修改后，重新加载配置到各设备
 *        通常在 Modbus 写操作完成后调用
 */
void App_ReloadConfig(void)
{
    MAIN_D("[RELOAD] Reloading config from g_AppParam...\r\n");

    /* --- 电压设备：重载阈值和参数 --- */
    if (g_voltage_bus_dev != NULL)
    {
        g_voltage_bus_dev->stcConfig.u32OvervoltageThresholdMv =
            (uint32_t)g_AppParam.voltage_upper_limit * 100UL;
        g_voltage_bus_dev->stcConfig.u32UndervoltageThresholdMv =
            (uint32_t)g_AppParam.voltage_lower_limit * 100UL;
        g_voltage_bus_dev->stcConfig.u32OvervoltageHysteresisMv =
            (uint32_t)g_AppParam.voltage_upper_hysteresis * 100UL;
        g_voltage_bus_dev->stcConfig.u32UndervoltageHysteresisMv =
            (uint32_t)g_AppParam.voltage_lower_hysteresis * 100UL;
        /* 过压确认窗固定为29ms: Flash 旧值向上拉到默认值 30 */
        g_voltage_bus_dev->stcConfig.u8OvervoltageTriggerCount =
            (g_AppParam.overvoltage_trigger_count < PARAM_DEFAULT_OVERVOLTAGE_TRIGGER_CNT)
                ? PARAM_DEFAULT_OVERVOLTAGE_TRIGGER_CNT
                : g_AppParam.overvoltage_trigger_count;
        /* [欠压判定窗 11~12ms] 欠压计数强制默认值12, 不读Flash(旧值无法向下拉) */
        g_voltage_bus_dev->stcConfig.u8UndervoltageTriggerCount =
            PARAM_DEFAULT_UNDERVOLTAGE_TRIGGER_CNT;

        MAIN_D("[RELOAD] Voltage: over=%lu mV, under=%lu mV\r\n",
               g_voltage_bus_dev->stcConfig.u32OvervoltageThresholdMv,
               g_voltage_bus_dev->stcConfig.u32UndervoltageThresholdMv);
    }

    /* --- 电流传感器：重载阈值和参数 --- */
    if (g_sensor_current_dev != NULL)
    {
        g_sensor_current_dev->stcConfig.s32OvercurrentThresholdMa =
            (int32_t)g_AppParam.current_upper_limit;
        g_sensor_current_dev->stcConfig.u32TriggerWindowMs =
            (uint32_t)g_AppParam.current_detect_ms;
        g_sensor_current_dev->stcConfig.u32ReleaseWindowMs =
            (uint32_t)g_AppParam.current_release_ms;
        g_sensor_current_dev->stcConfig.s32OvercurrentHysteresisMa =
            (int32_t)g_AppParam.current_hysteresis_ma;

        MAIN_D("[RELOAD] Current: threshold=%ld mA, trigger=%lu ms, release=%lu ms\r\n",
               (long)g_sensor_current_dev->stcConfig.s32OvercurrentThresholdMa,
               (unsigned long)g_sensor_current_dev->stcConfig.u32TriggerWindowMs,
               (unsigned long)g_sensor_current_dev->stcConfig.u32ReleaseWindowMs);
    }

    /* 设备地址 (node_id) 由 Flash 存储，在 Param_Init 时已加载 */

    /* --- 旋转限位：重载减速比和角度范围 --- */
    if (g_rturn_dev != NULL)
    {
        g_rturn_dev->stcConfig.fReductionRatio =
            (float)g_AppParam.rturn_reduction_ratio / 10.0f;
        g_rturn_dev->stcConfig.fMaxAngle =
            (float)g_AppParam.open_limit_angle / 10.0f;
        g_rturn_dev->stcConfig.fMinAngle =
            (float)g_AppParam.close_limit_angle / 10.0f;
        MAIN_D("[RELOAD] Reduction ratio: %.1f, MaxAngle: %.1f, MinAngle: %.1f\r\n",
               (double)g_rturn_dev->stcConfig.fReductionRatio,
               (double)g_rturn_dev->stcConfig.fMaxAngle,
               (double)g_rturn_dev->stcConfig.fMinAngle);
    }

    /* --- 电机霍尔：重载极对数 --- */
    if (g_motor_hall_dev != NULL && g_motor_hall_dev->handle != NULL)
    {
        motor_hall_set_pole_pairs(g_motor_hall_dev->handle,
                                  (uint8_t)g_AppParam.motor_hall_pole_pairs);
        g_motor_hall_dev->config.pole_pairs =
            (uint8_t)g_AppParam.motor_hall_pole_pairs;
        MAIN_D("[RELOAD] Pole pairs: %d\r\n",
               (int)g_AppParam.motor_hall_pole_pairs);
    }

    MAIN_D("[RELOAD] Config reload complete.\r\n");
}

void ESystem_MainLoop(void) {
    static uint32_t last_loop_time = 0;
    uint32_t now = tickTimer_GetCount();

    if (now - last_loop_time < 1) return;
    last_loop_time = now;

#if ENABLE_SIMULATION_MODE
    Sim_ProcessInput();
    Sim_PublishEvents();
#endif

    ProcessDeviceUpdates();
    RunAngle_Update();         /* 根据霍尔脉冲增量跟踪绝对角度 */
    MotionTimeout_Update();    /* 控制超时检测 (1ms 节拍) */
    UpdateStatusIndicators();
}
