/**
 *******************************************************************************
 * @file  App_RunAngle.c
 * @brief Absolute angle offset — tracks angular displacement from user zero
 *        using Hall pulse delta, persisted to Flash via param_manager.
 *******************************************************************************
 */

#include "App_RunAngle.h"
#include "App_Params.h"         /* g_s32HallPulseAccum, g_AppParam */
#include "param_manager.h"
#include "EventBus.h"           /* TOPIC_MANUAL_RS485 */
#include "dev_motor.h"          /* MotorManualIOEvent_t, CMD_TYPE_*, DIR_* */
#include <stdlib.h>             /* abs() */
#include <string.h>

/*=============================================================================
 * RAM state
 *============================================================================*/
static int32_t s_abs_offset_x10 = -10;      /* real-time absolute angle offset (0.1 deg) */
static int32_t s_last_accum     = 0;        /* snapshot of g_s32HallPulseAccum at last update */
static float   s_angle_accumulator = 0.0f;  /* fractional angle accumulator (0.1 deg units) */
static int32_t s_target_x10     = 0;        /* goto-target angle (0.1 deg), set via 0x2727/0x2728 */
static bool    s_goto_zero_active = false;  /* goto-zero in progress */
static bool    s_goto_zero_initial_sign;    /* sign of offset when goto-zero started */
static bool    s_goto_target_active = false;/* goto-target in progress */

AbsAngleRecord_t g_AbsAngle;              /* Flash mirror */

/*=============================================================================
 * Runtime + Config for param_manager multi-instance (sector 55)
 *============================================================================*/
static Param_Runtime_t s_Runtime;

static const Param_Config_t s_Config = {
    .pParamBuf      = &g_AbsAngle,
    .paramSize      = sizeof(AbsAngleRecord_t),
    .magicHead      = ABS_ANGLE_MAGIC_HEAD,
    .magicTail      = ABS_ANGLE_MAGIC_TAIL,
    .checksumOffset = offsetof(AbsAngleRecord_t, checksum),
    .seqOffset      = offsetof(AbsAngleRecord_t, sequence_id),
    .eraseCntOffset = offsetof(AbsAngleRecord_t, erase_count),
    .secStart       = 55,
    .secEnd         = 55,
};

/*=============================================================================
 * Defaults callback
 *============================================================================*/
static void RunAngle_SetDefaults(void)
{
    g_AbsAngle.magic                 = ABS_ANGLE_MAGIC_HEAD;
    g_AbsAngle.sequence_id           = 0;
    g_AbsAngle.erase_count           = 0;
    g_AbsAngle.abs_offset_x10        = 0;
    g_AbsAngle.goto_zero_thresh_x10  = ABS_THRESH_DEFAULT_X10;
    g_AbsAngle.reserved              = 0;
    g_AbsAngle.checksum              = 0;
    g_AbsAngle.tail_magic            = ABS_ANGLE_MAGIC_TAIL;
}

/*=============================================================================
 * deg_x10 per pulse = 3600 / (pole_pairs * hall_count * 2 * ratio)
 *   pole_pairs from g_AppParam.motor_hall_pole_pairs (default 3)
 *   hall_count = 2 (hardcoded)
 *   ratio = g_AppParam.rturn_reduction_ratio / 10.0
 *============================================================================*/
static float DegPerPulse_x10(void)
{
    uint8_t pp = (uint8_t)g_AppParam.motor_hall_pole_pairs;
    float   ratio = (float)g_AppParam.rturn_reduction_ratio / 10.0f;

    if (pp == 0 || g_AppParam.rturn_reduction_ratio == 0) {
        return 0.0f;
    }
    return 3600.0f / (float)(pp * 2 * 2) / ratio;
}

/*=============================================================================
 * Public API
 *============================================================================*/

void RunAngle_Init(void)
{
    /* Load latest valid record from Flash (or write defaults on first boot) */
    Param_Init(&s_Config, &s_Runtime, RunAngle_SetDefaults);

    /* Validate threshold */
    if (g_AbsAngle.goto_zero_thresh_x10 < 0
        || g_AbsAngle.goto_zero_thresh_x10 > ABS_THRESH_MAX_X10) {
        g_AbsAngle.goto_zero_thresh_x10 = ABS_THRESH_DEFAULT_X10;
    }

    /* RAM = Flash value at power-up */
    s_abs_offset_x10 = g_AbsAngle.abs_offset_x10;

    /* Snapshot current pulse accumulator so subsequent delta starts from zero */
    // __disable_irq();
    s_last_accum = g_s32HallPulseAccum;
    // __enable_irq();

    /* Reset fractional accumulator on init */
    s_angle_accumulator = 0.0f;

    MAIN_D("[ABSA] Init: ram=%ld (0.1deg), flash=%ld, accum_base=%ld\r\n",
           (long)s_abs_offset_x10, (long)g_AbsAngle.abs_offset_x10, (long)s_last_accum);
}

void RunAngle_Update(void)
{
    int32_t accum;
    // __disable_irq();
    accum = g_s32HallPulseAccum;
    // __enable_irq();

    int32_t delta = accum - s_last_accum;

    /* Update angle offset from pulse delta (if any) */
    if (delta != 0) {
        s_last_accum = accum;

        float k = DegPerPulse_x10();
        if (k != 0.0f) {
            /* Accumulate fractional angle with floating point precision */
            s_angle_accumulator += (float)delta * k;

            /* Extract integer part and add to offset, keep fractional remainder */
            int32_t add_angle = (int32_t)s_angle_accumulator;
            if (add_angle != 0) {
                s_abs_offset_x10 += add_angle;
                s_angle_accumulator -= (float)add_angle;
            }
        }
    }

    /* Goto-ref: once motor moves past close-limit reference or enters threshold, stop.
     * Only one direction command was sent at start — no continuous re-issue. */
    if (s_goto_zero_active) {
        int32_t ref = (int32_t)g_AppParam.close_limit_angle;
        int32_t dist = s_abs_offset_x10 - ref;
        if (abs(dist) <= g_AbsAngle.goto_zero_thresh_x10
            || (dist > 0) != s_goto_zero_initial_sign) {
            MotorManualIOEvent_t ev;
            memset(&ev, 0, sizeof(ev));
            ev.type = CMD_TYPE_STOP;        /* ESTOP (same as CTRL_CMD_ESTOP) */
            ev.dir  = DIR_NONE;
            EventBus_Publish(TOPIC_MANUAL_RS485, &ev);
            s_goto_zero_active = false;
            MAIN_D("[ABSA] Goto-ref complete: offset=%ld, ref=%ld, dist=%ld\r\n",
                   (long)s_abs_offset_x10, (long)ref, (long)dist);
        }
    }

    /* Goto-target: stop when within threshold of target, or overshoot past it */
    if (s_goto_target_active) {
        int32_t dist = s_abs_offset_x10 - s_target_x10;
        if (abs(dist) <= g_AbsAngle.goto_zero_thresh_x10
            || (dist > 0) != (s_goto_zero_initial_sign)) {
            MotorManualIOEvent_t ev;
            memset(&ev, 0, sizeof(ev));
            ev.type = CMD_TYPE_STOP;
            ev.dir  = DIR_NONE;
            EventBus_Publish(TOPIC_MANUAL_RS485, &ev);
            s_goto_target_active = false;
            MAIN_D("[ABSA] Goto-target complete: offset=%ld, target=%ld, dist=%ld\r\n",
                   (long)s_abs_offset_x10, (long)s_target_x10, (long)dist);
        }
    }
}

int32_t RunAngle_GetOffset_x10(void)
{
    return s_abs_offset_x10;
}

int32_t RunAngle_GetFlashOffset_x10(void)
{
    return g_AbsAngle.abs_offset_x10;
}

void RunAngle_Cmd(uint16_t cmd)
{
    if (cmd == ABS_CMD_SET_ZERO || cmd == ABS_CMD_SET_ZERO_THEN_SAVE) {
        /* Set current position to close-limit reference angle (0x271C) */
        int32_t ref = (int32_t)g_AppParam.close_limit_angle;
        s_abs_offset_x10 = ref;
        g_AbsAngle.abs_offset_x10 = ref;
        /* Reset fractional accumulator when setting reference point */
        s_angle_accumulator = 0.0f;
        Param_Save(&s_Config, &s_Runtime);

        MAIN_D("[ABSA] Reference set: RAM=%ld, Flash=%ld (0x271C=%ld)\r\n",
               (long)ref, (long)ref, (long)g_AppParam.close_limit_angle);
    } else if (cmd == ABS_CMD_SAVE) {
        /* Persist current RAM value to Flash */
        g_AbsAngle.abs_offset_x10 = s_abs_offset_x10;
        Param_Save(&s_Config, &s_Runtime);

        MAIN_D("[ABSA] Saved: ram=%ld -> flash=%ld\r\n",
               (long)s_abs_offset_x10, (long)g_AbsAngle.abs_offset_x10);
    } else if (cmd == ABS_CMD_GOTO_ZERO) {
        RunAngle_GotoZero();
    }
}

void RunAngle_GotoZero(void)
{
    int32_t ref = (int32_t)g_AppParam.close_limit_angle;

    /* Gate 1: control must be unlocked (same lock as REG_CTRL_CMD bit3~bit5) */
    if (!Param_IsCtrlUnlocked()) {
        MAIN_D("[ABSA] Goto-ref rejected: RS485 control locked\r\n");
        return;
    }

    /* Gate 2: motor must be stopped before accepting goto-ref */
    if (!Param_IsMotorStopped()) {
        MAIN_D("[ABSA] Goto-ref rejected: motor is running\r\n");
        return;
    }

    int32_t dist = s_abs_offset_x10 - ref;

    MotorManualIOEvent_t ev;
    memset(&ev, 0, sizeof(ev));

    if (dist > g_AbsAngle.goto_zero_thresh_x10) {
        /* Beyond reference → REV to go back */
        ev.dir  = DIR_REV;
        ev.type = CMD_TYPE_RUN_REV;
        s_goto_zero_active = true;
        s_goto_zero_initial_sign = true;   /* positive dist → expect sign flip to false */
        MAIN_D("[ABSA] Goto-ref: cur=%ld, ref=%ld, dist=%ld (>0), REV\r\n",
               (long)s_abs_offset_x10, (long)ref, (long)dist);
    } else if (dist < -g_AbsAngle.goto_zero_thresh_x10) {
        /* Before reference → FWD to go forward */
        ev.dir  = DIR_FWD;
        ev.type = CMD_TYPE_RUN_FWD;
        s_goto_zero_active = true;
        s_goto_zero_initial_sign = false;  /* negative dist → expect sign flip to true */
        MAIN_D("[ABSA] Goto-ref: cur=%ld, ref=%ld, dist=%ld (<0), FWD\r\n",
               (long)s_abs_offset_x10, (long)ref, (long)dist);
    } else {
        /* Already at reference — nothing to do */
        MAIN_D("[ABSA] Goto-ref: cur=%ld, ref=%ld, already at reference\r\n",
               (long)s_abs_offset_x10, (long)ref);
        return;
    }

    EventBus_Publish(TOPIC_MANUAL_RS485, &ev);
}

void RunAngle_GotoTarget(void)
{
    /* Gate 1: control must be unlocked */
    if (!Param_IsCtrlUnlocked()) {
        MAIN_D("[ABSA] Goto-target rejected: RS485 control locked\r\n");
        return;
    }

    /* Gate 2: motor must be stopped */
    if (!Param_IsMotorStopped()) {
        MAIN_D("[ABSA] Goto-target rejected: motor is running\r\n");
        return;
    }

    int32_t dist = s_abs_offset_x10 - s_target_x10;

    MotorManualIOEvent_t ev;
    memset(&ev, 0, sizeof(ev));

    if (dist > g_AbsAngle.goto_zero_thresh_x10) {
        /* Current offset > target → REV to go back */
        ev.dir  = DIR_REV;
        ev.type = CMD_TYPE_RUN_REV;
        s_goto_target_active = true;
        s_goto_zero_initial_sign = true;    /* positive dist → expect sign flip to false */
        MAIN_D("[ABSA] Goto-target: cur=%ld, target=%ld, dist=%ld (>0), REV to target\r\n",
               (long)s_abs_offset_x10, (long)s_target_x10, (long)dist);
    } else if (dist < -g_AbsAngle.goto_zero_thresh_x10) {
        /* Current offset < target → FWD to go forward */
        ev.dir  = DIR_FWD;
        ev.type = CMD_TYPE_RUN_FWD;
        s_goto_target_active = true;
        s_goto_zero_initial_sign = false;   /* negative dist → expect sign flip to true */
        MAIN_D("[ABSA] Goto-target: cur=%ld, target=%ld, dist=%ld (<0), FWD to target\r\n",
               (long)s_abs_offset_x10, (long)s_target_x10, (long)dist);
    } else {
        /* Already at target */
        MAIN_D("[ABSA] Goto-target: cur=%ld, target=%ld, already at target\r\n",
               (long)s_abs_offset_x10, (long)s_target_x10);
        return;
    }

    EventBus_Publish(TOPIC_MANUAL_RS485, &ev);
}

int32_t RunAngle_GetTarget_x10(void)
{
    return s_target_x10;
}

void RunAngle_SetTarget_x10(int32_t target_x10)
{
    s_target_x10 = target_x10;
    MAIN_D("[ABSA] Target set: %ld (0.1 deg)\r\n", (long)target_x10);
}

void RunAngle_SetThreshold(uint16_t thresh_x10)
{
    /* Clamp to valid range */
    if (thresh_x10 < 0) {
        thresh_x10 = 0;
    } else if (thresh_x10 > ABS_THRESH_MAX_X10) {
        thresh_x10 = ABS_THRESH_MAX_X10;
    }

    g_AbsAngle.goto_zero_thresh_x10 = thresh_x10;
    Param_Save(&s_Config, &s_Runtime);

    MAIN_D("[ABSA] Threshold set: %u (0.1 deg)\r\n", (unsigned int)thresh_x10);
}

void RunAngle_OnCalibration(void)
{
    int32_t ref = (int32_t)g_AppParam.close_limit_angle;

    // __disable_irq();
    s_abs_offset_x10 = ref;
    s_last_accum = g_s32HallPulseAccum;   /* accum was just zeroed by dev_rturn */
    /* Reset fractional accumulator on calibration */
    s_angle_accumulator = 0.0f;
    // __enable_irq();

    g_AbsAngle.abs_offset_x10 = ref;
    Param_Save(&s_Config, &s_Runtime);    /* persist to Flash sector 55 */

    MAIN_D("[ABSA] Calibration: RAM=%ld, Flash=%ld (0x271C=%ld)\r\n",
           (long)ref, (long)ref, (long)g_AppParam.close_limit_angle);
}

/** @brief Check if current absolute angle is within calibration range.
 *  @retval true   angle in [calib_lower, calib_upper], caller should proceed
 *  @retval false  angle out of range, caller should set fault */
bool RunAngle_TryCalibrate(void)
{
    int32_t lower = (int32_t)g_AppParam.calib_lower_x10;
    int32_t upper = (int32_t)g_AppParam.calib_upper_x10;

    if (s_abs_offset_x10 >= lower && s_abs_offset_x10 <= upper) {
        MAIN_D("[ABSA] TryCalibrate: angle=%ld in [%ld,%ld], OK\r\n",
               (long)s_abs_offset_x10, (long)lower, (long)upper);
        return true;
    }
    MAIN_D("[ABSA] TryCalibrate: angle=%ld OUT of [%ld,%ld], rejected\r\n",
           (long)s_abs_offset_x10, (long)lower, (long)upper);
    return false;
}

void RunAngle_JogFwd(uint16_t offset_x10)
{
    s_target_x10 = s_abs_offset_x10 + (int32_t)offset_x10;
    MAIN_D("[ABSA] JogFwd: offset=%u, target=%ld (0.1 deg)\r\n",
           (unsigned int)offset_x10, (long)s_target_x10);
    RunAngle_GotoTarget();
}

void RunAngle_JogRev(uint16_t offset_x10)
{
    s_target_x10 = s_abs_offset_x10 - (int32_t)offset_x10;
    MAIN_D("[ABSA] JogRev: offset=%u, target=%ld (0.1 deg)\r\n",
           (unsigned int)offset_x10, (long)s_target_x10);
    RunAngle_GotoTarget();
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
