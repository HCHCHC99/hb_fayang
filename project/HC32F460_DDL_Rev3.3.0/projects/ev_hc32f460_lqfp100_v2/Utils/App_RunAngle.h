#ifndef __APP_RUNANGLE_H__
#define __APP_RUNANGLE_H__

#include <stdint.h>
#include <stdbool.h>
#include "param_manager.h"

/*=============================================================================
 * Absolute angle tracking module
 *
 * Registers:
 *   0x2721/0x2722  Read RAM offset (int32_t, 0.1 deg)
 *   0x2723/0x2724  Read Flash offset (int32_t, 0.1 deg)
 *   0x2725         Write: 0=set ref+save, 1=goto ref, 2=save, 3=set ref
 *   0x2726         R/W: stop threshold (uint16_t, 0.1 deg, default 1, shared)
 *   0x2727/0x2728  R/W: goto-target angle (int32_t, 0.1 deg; 0x10 write triggers)
 *
 * Reference point (close-limit reference) is defined by 0x271C close_limit_angle.
 *============================================================================*/

/*=============================================================================
 * Flash record (sector 55)
 *============================================================================*/
#pragma pack(4)
typedef struct {
    uint32_t magic;                 /* 0xAB5AAB5A */
    uint32_t sequence_id;
    uint32_t erase_count;
    int32_t  abs_offset_x10;        /* persisted absolute angle offset (0.1 deg) */
    int16_t  goto_zero_thresh_x10;  /* stop threshold (0.1 deg, default 1, range 0~200) */
    uint16_t reserved;              /* padding to 4-byte alignment */
    uint32_t checksum;              /* CRC32 */
    uint32_t tail_magic;            /* 0xBA5ABA5A */
} AbsAngleRecord_t;
#pragma pack()

#define ABS_ANGLE_MAGIC_HEAD   0xAB5AAB5A
#define ABS_ANGLE_MAGIC_TAIL   0xBA5ABA5A

/* Default threshold: 0.1 deg */
#define ABS_THRESH_DEFAULT_X10    6
#define ABS_THRESH_MIN_X10        0
#define ABS_THRESH_MAX_X10      200   /* 20.0 deg max */

/* Command values for 0x2725 — reference point defined by 0x271C (close_limit_angle) */
#define ABS_CMD_SET_ZERO_THEN_SAVE  0x0000U  /* Set reference + save to Flash */
#define ABS_CMD_GOTO_ZERO           0x0001U  /* Goto reference position */
#define ABS_CMD_SAVE                0x0002U  /* Save current offset to Flash */
#define ABS_CMD_SET_ZERO            0x0003U  /* Set reference (RAM/Flash = 0x271C, save) */

/*=============================================================================
 * Public API
 *============================================================================*/

void RunAngle_Init(void);
void RunAngle_Update(void);
int32_t RunAngle_GetOffset_x10(void);
int32_t RunAngle_GetFlashOffset_x10(void);
void RunAngle_Cmd(uint16_t cmd);
void RunAngle_GotoZero(void);
void RunAngle_GotoTarget(void);

/** @brief Set goto-zero threshold (called on Modbus write 0x2726, hot-reload) */
void RunAngle_SetThreshold(uint16_t thresh_x10);

/** @brief Get/set goto-target angle in 0.1 deg (int32_t, registers 0x2727/0x2728) */
int32_t RunAngle_GetTarget_x10(void);
void    RunAngle_SetTarget_x10(int32_t target_x10);

/** @brief Called by dev_rturn when calibration zeroes g_s32HallPulseAccum.
 *  Sets RAM offset and Flash persisted value to close_limit_angle (0x271C),
 *  and re-syncs the pulse baseline to prevent delta corruption. */
void RunAngle_OnCalibration(void);

/** @brief Try to calibrate on close-window overcurrent.
 *  Checks s_abs_offset_x10 against [calib_lower, calib_upper] from g_AppParam.
 *  If in range, calls RunAngle_OnCalibration() and returns true.
 *  If out of range, returns false (caller should set FAULT_BIT_OVERCURRENT_FWD). */
bool RunAngle_TryCalibrate(void);

/** @brief Jog offset in window-open direction (FWD).
 *  target = current + offset_x10, then calls RunAngle_GotoTarget(). */
void RunAngle_JogFwd(uint16_t offset_x10);

/** @brief Jog offset in window-close direction (REV).
 *  target = current - offset_x10, then calls RunAngle_GotoTarget(). */
void RunAngle_JogRev(uint16_t offset_x10);

extern AbsAngleRecord_t g_AbsAngle;

#endif /* __APP_RUNANGLE_H__ */
