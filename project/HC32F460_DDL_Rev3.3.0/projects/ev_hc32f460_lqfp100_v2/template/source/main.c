#include "main.h"
#include "Hardware.h"
#include "rtt_log.h"
#include "timer6_timebase.h"
#include "Motor_hall.h"
#include "TickTimer.h"
#include "device_manager.h"
#include "App_Motor_Project.h"
#include "param_manager.h"
#include "Gpio_io.h"
#include "rs485.h"
#include "App_Modbus.h"
#include "App_Params.h"
#include "App_FaultHandler.h"
#include "rtt_manager.h"   // 包含 INTERVAL_DECLARE / INTERVAL_PRINT 宏
#include "Pwm.h"
#include "Template_Pwm.h"
#include "hc32_ll_utility.h"
#include "hc32_ll_efm.h"

/*=============================================================================
 * 全局变量定义
 *============================================================================*/

/*=============================================================================
 * 全局PWM实例（供dev_motor使用）
 *============================================================================*/
pwm_t g_motor_pwm_ch1;  // PB6
pwm_t g_motor_pwm_ch2;  // PB7
pwm_t g_motor_pwm_ch3;  // PB8
pwm_t g_motor_pwm_ch4;  // PB9

/*=============================================================================
 * 电机PWM初始化（4通道，20kHz，低有效）
 *============================================================================*/
static void Motor_Pwm_Init(void)
{
    // 解锁GPIO寄存器（允许修改GPIO配置）
    LL_PERIPH_WE(LL_PERIPH_GPIO);

    // CH1: PB6 - 低有效
    g_motor_pwm_ch1 = PWM_Init(CM_TMRA_4, FCG2_PERIPH_TMRA_4, TMRA_CH1,
                                GPIO_PORT_B, GPIO_PIN_06, GPIO_FUNC_4,
                                TMRA_MD_SAWTOOTH, TMRA_DIR_UP,
                                6000, 0, PWM_ACTIVE_LOW);

    // CH2: PB7 - 低有效
    g_motor_pwm_ch2 = PWM_Init(CM_TMRA_4, FCG2_PERIPH_TMRA_4, TMRA_CH2,
                                GPIO_PORT_B, GPIO_PIN_07, GPIO_FUNC_4,
                                TMRA_MD_SAWTOOTH, TMRA_DIR_UP,
                                6000, 0, PWM_ACTIVE_LOW);

    // CH3: PB8 - 低有效
    g_motor_pwm_ch3 = PWM_Init(CM_TMRA_4, FCG2_PERIPH_TMRA_4, TMRA_CH3,
                                GPIO_PORT_B, GPIO_PIN_08, GPIO_FUNC_4,
                                TMRA_MD_SAWTOOTH, TMRA_DIR_UP,
                                6000, 0, PWM_ACTIVE_LOW);

    // CH4: PB9 - 低有效
    g_motor_pwm_ch4 = PWM_Init(CM_TMRA_4, FCG2_PERIPH_TMRA_4, TMRA_CH4,
                                GPIO_PORT_B, GPIO_PIN_09, GPIO_FUNC_4,
                                TMRA_MD_SAWTOOTH, TMRA_DIR_UP,
                                6000, 0, PWM_ACTIVE_LOW);

    // 锁定GPIO寄存器（配置完成后锁定）
    LL_PERIPH_WP(LL_PERIPH_GPIO);

    // 解锁FCG寄存器（允许使能定时器时钟）
    LL_PERIPH_WE(LL_PERIPH_FCG);

    // 启动4路PWM定时器
    PWM_Start(&g_motor_pwm_ch1);
    PWM_Start(&g_motor_pwm_ch2);
    PWM_Start(&g_motor_pwm_ch3);
    PWM_Start(&g_motor_pwm_ch4);

    // 使能输出
    PWM_OutputCmd(&g_motor_pwm_ch1, PWM_OUTPUT_ENABLE);
    PWM_OutputCmd(&g_motor_pwm_ch2, PWM_OUTPUT_ENABLE);
    PWM_OutputCmd(&g_motor_pwm_ch3, PWM_OUTPUT_ENABLE);
    PWM_OutputCmd(&g_motor_pwm_ch4, PWM_OUTPUT_ENABLE);

    // 锁定FCG寄存器
    LL_PERIPH_WP(LL_PERIPH_FCG);

    MAIN_D("Motor PWM initialized: 4 channels, 20kHz, low active\r\n");
}

/*=============================================================================
 * 临时验证：广播分配ID方案 - slot哈希序列测试（验证后可删）
 * 验证点：
 *  1. 同一UID连续10轮(round=0~9)调用哈希，输出是否每次不同
 *  2. 两台不同UID(本机 与 本机+1 模拟同批次相邻芯片)的10轮slot序列
 *     是否会出现完全一致（序列相关）
 * 结果存 volatile 缓冲，可在 Keil Watch 查看，同时RTT打印
 *============================================================================*/
volatile uint32_t g_hash_test_a_h[10];     /* UID_A(本机真实UID) 各轮哈希原值 */
volatile uint8_t  g_hash_test_a_slot[10];  /* UID_A 各轮slot */
volatile uint32_t g_hash_test_b_h[10];     /* UID_B(本机UID+1,模拟另一台) 各轮哈希原值 */
volatile uint8_t  g_hash_test_b_slot[10];  /* UID_B 各轮slot */

/* MurmurHash3 finalizer 风格雪崩混合，round为轮次号（分配协议正式函数） */
static uint32_t Assign_Hash(uint32_t uid0, uint32_t uid1, uint32_t uid2, uint8_t round)
{
    uint32_t h = uid0 ^ (uid1 * 0x9E3779B9u) ^ (uid2 * 0x85EBCA6Bu) ^ ((uint32_t)round * 0xC2B2AE35u);
    h ^= h >> 16;  h *= 0x7FEB352Du;  h ^= h >> 15;
    h *= 0x846CA68Bu;  h ^= h >> 16;
    return h;
}

static void Hash_Sequence_Test(void)
{
    stc_efm_unique_id_t uid;
    EFM_GetUID(&uid);

    /* UID_B = 本机UID各分量+1，模拟同批次流水号相邻的另一台芯片 */
    uint32_t b0 = uid.u32UniqueID0 + 1u;
    uint32_t b1 = uid.u32UniqueID1 + 1u;
    uint32_t b2 = uid.u32UniqueID2 + 1u;

    for (uint8_t r = 0; r < 10; r++) {
        uint32_t ha = Assign_Hash(uid.u32UniqueID0, uid.u32UniqueID1, uid.u32UniqueID2, r);
        uint32_t hb = Assign_Hash(b0, b1, b2, r);
        g_hash_test_a_h[r]    = ha;
        g_hash_test_a_slot[r] = (uint8_t)(ha % 128u);
        g_hash_test_b_h[r]    = hb;
        g_hash_test_b_slot[r] = (uint8_t)(hb % 128u);
        MAIN_D("hash r=%u  A: h=%08X slot=%u   B: h=%08X slot=%u\r\n",
               r, ha, g_hash_test_a_slot[r], hb, g_hash_test_b_slot[r]);
    }
}

/*=============================================================================
 * 主循环阻塞测量 (LOOPWATCH)
 * 用 Timer6 硬件计数器 (µs 级, PCLK0/64, 16位) 测量 while(1) 单次循环体
 * 执行耗时; 用 tickTimer (1ms) 粗测相邻两次进入间隔, 覆盖 Timer6 计数
 * 回绕导致的长阻塞误读。
 * 观察方式: Keil Watch 查看 g_loopIterMaxUs / g_loopGapMaxMs /
 *          g_loopOverBudgetCnt, 或 RTT 每 5 秒打印 [LOOPWATCH]
 * 说明: 不直接用 Timer6_Timebase_DeltaToUs(), 其内部 32 位乘法在
 *       delta 较大时会溢出, 此处用 64 位运算自行换算
 *============================================================================*/
volatile uint32_t g_loopIterLastUs = 0;     /* 最近一次循环体执行耗时 (µs) */
volatile uint32_t g_loopIterMaxUs = 0;      /* 循环体执行耗时最大值 (µs) */
volatile uint32_t g_loopOverBudgetCnt = 0;  /* 循环体耗时超过 1000µs 的次数 */
volatile uint32_t g_loopGapMaxMs = 0;       /* 相邻两次进入主循环最大间隔 (ms) */
static uint32_t s_loopWatchLastPrintTick = 0;
static uint32_t s_loopPrevEnterTick = 0;

static void LoopWatch_Sample(uint32_t t0_cnt, uint32_t t0_tick)
{
    uint32_t t1_cnt = Timer6_Timebase_GetCounter();
    uint32_t freq = Timer6_Timebase_GetFrequency();
    uint32_t delta = (t1_cnt >= t0_cnt) ? (t1_cnt - t0_cnt)
                                        : (0x10000u - t0_cnt + t1_cnt);
    uint32_t iter_us = 0;
    uint32_t now_tick;

    if (freq != 0) {
        iter_us = (uint32_t)(((uint64_t)delta * 1000000ULL) / freq);
    }
    g_loopIterLastUs = iter_us;
    if (iter_us > g_loopIterMaxUs) {
        g_loopIterMaxUs = iter_us;
    }
    if (iter_us > 1000u) {
        g_loopOverBudgetCnt++;
    }

    /* 相邻两次进入主循环的间隔 (1ms 粗分辨率, 无回绕问题) */
    now_tick = tickTimer_GetCount();
    if (s_loopPrevEnterTick != 0) {
        uint32_t gap_ms = now_tick - s_loopPrevEnterTick;
        if (gap_ms > g_loopGapMaxMs) {
            g_loopGapMaxMs = gap_ms;
        }
    }
    s_loopPrevEnterTick = t0_tick;

    /* 每 5 秒打印一次统计 (打印耗时会计入下一轮测量, RTT 写 RAM 影响极小) */
    if ((now_tick - s_loopWatchLastPrintTick) >= 5000u) {
        s_loopWatchLastPrintTick = now_tick;
        MAIN_D("[LOOPWATCH] iter=%lu us, max=%lu us, over1ms=%lu, gap_max=%lu ms\r\n",
               (unsigned long)g_loopIterLastUs,
               (unsigned long)g_loopIterMaxUs,
               (unsigned long)g_loopOverBudgetCnt,
               (unsigned long)g_loopGapMaxMs);
    }
}

int main(void)
{
    Hardware_Init();

    /* RS485 初始化 - 配置 USART 和 GPIO，等待中断处理 */
    RS485_Init();

    /* Modbus 初始化 - 配置协议栈，注册读写回调函数 */
    Modbus_Init();

    ESystem_Init();

    /* 初始化电机PWM（4通道，在电机设备初始化之前） */
    // Motor_Pwm_Init();

    /* 故障处理器初始化 - 注册电压/电流/温度等故障检测回调 */
    FaultHandler_Init();

    /*=========================================================================
     * 电机模式控制变量 - 在 Keil Watch 窗口中可动态修改
     * 0: 停止, 1: 正转, 2: 反转
     *=========================================================================*/
    // volatile uint8_t motor_mode = 0;

    // MotorDevice_t* motor = NULL;       // TODO: 从设备管理器中获取电机设备指针

    EventBus_Enable();

    /* 临时验证：slot哈希序列（验证后可删） */
    Hash_Sequence_Test();

    /* 测试计数器 */
    uint32_t test_counter = 0;

    while (1)
    {
        /* 主循环阻塞测量: 记录进入时刻 */
        uint32_t loop_t0_cnt = Timer6_Timebase_GetCounter();
        uint32_t loop_t0_tick = tickTimer_GetCount();

        ESystem_MainLoop();

#if MOTOR_CONTROL_MODE == 1
        // 更新4通道PWM状态
        PWM_Update(&g_motor_pwm_ch1);
        PWM_Update(&g_motor_pwm_ch2);
        PWM_Update(&g_motor_pwm_ch3);
        PWM_Update(&g_motor_pwm_ch4);
#endif

        RS485_Poll();
        Modbus_Poll();

        /* 主循环阻塞测量: 统计本轮循环体耗时 */
        LoopWatch_Sample(loop_t0_cnt, loop_t0_tick);

        // // 根据电机模式变量控制电机运行状态
        // if (motor_mode == 0) {
        //     Motor_OnArbitrationStop(motor);
        // } else if (motor_mode == 1) {
        //     Motor_OnArbitrationFwd(motor, 0.0f);
        // } else if (motor_mode == 2) {
        //     Motor_OnArbitrationRev(motor, 0.0f);
        // }

    }
}
