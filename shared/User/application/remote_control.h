/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */


#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H
#include "struct_typedef.h"
#include "bsp_rc.h"
#include "config.h"

#define SBUS_RX_BUF_NUM BSP_RC_SBUS_RX_BUF_NUM

#define RC_FRAME_LENGTH BSP_RC_SBUS_FRAME_LENGTH

#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)
#define RC_CH_VALUE_ABS_LEGACY  ((uint16_t)(RC_CH_VALUE_MAX - RC_CH_VALUE_OFFSET))
#define RC_CH_VALUE_ABS_MAX     ((uint16_t)1024)

#ifndef MANUAL_INPUT_SRC_IMAGE
// Targets without a dedicated image-link input source reuse the reserved USB slot.
#define MANUAL_INPUT_SRC_IMAGE MANUAL_INPUT_SRC_USB
#endif

#ifndef MANUAL_INPUT_SRC_MAX
#define MANUAL_INPUT_SRC_MAX MANUAL_INPUT_SRC_USB
#endif

static __inline int16_t rc_scale_axis_by_abs(int16_t raw, int16_t in_abs_max, int16_t out_abs_max)
{
    int32_t value = raw;
    int32_t in_max = in_abs_max;
    int32_t out_max = out_abs_max;

    if (in_max <= 0 || out_max <= 0)
    {
        return 0;
    }

    if (value > in_max)
    {
        value = in_max;
    }
    else if (value < -in_max)
    {
        value = -in_max;
    }

    value *= out_max;
    if (value >= 0)
    {
        value += in_max / 2;
    }
    else
    {
        value -= in_max / 2;
    }

    return (int16_t)(value / in_max);
}

static __inline uint16_t rc_scale_u16_by_abs(uint16_t value, uint16_t in_abs_max, uint16_t out_abs_max)
{
    uint32_t in_max = in_abs_max;
    uint32_t out_max = out_abs_max;
    uint32_t scaled = value;

    if (in_max == 0u || out_max == 0u)
    {
        return 0u;
    }

    scaled = (scaled * out_max + (in_max / 2u)) / in_max;
    if (scaled > 65535u)
    {
        scaled = 65535u;
    }

    return (uint16_t)scaled;
}

static __inline fp32 rc_scale_fp32_by_abs(fp32 value, fp32 in_abs_max, fp32 out_abs_max)
{
    if (in_abs_max <= 0.0f || out_abs_max <= 0.0f)
    {
        return 0.0f;
    }

    return value * (out_abs_max / in_abs_max);
}

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)
#define switch_is_down(s)       (s == RC_SW_DOWN)
#define switch_is_mid(s)        (s == RC_SW_MID)
#define switch_is_up(s)         (s == RC_SW_UP)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W            ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S            ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A            ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D            ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT        ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL         ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q            ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E            ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R            ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F            ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G            ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z            ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X            ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C            ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V            ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B            ((uint16_t)1 << 15)
/* ----------------------- Data Struct ------------------------------------- */
typedef __packed struct
{
        __packed struct
        {
                int16_t ch[5];
                char s[2];
        } rc;
        __packed struct
        {
                int16_t x;
                int16_t y;
                int16_t z;
                uint8_t press_l;
                uint8_t press_r;
        } mouse;
        __packed struct
        {
                uint16_t v;
        } key;

} RC_ctrl_t;

/* ----------------------- Internal Data ----------------------------------- */

extern void remote_control_init(void);
extern void remote_control_on_sbus_frame(const uint8_t frame[RC_FRAME_LENGTH]);
extern const RC_ctrl_t *get_remote_control_point(void);
extern void remote_control_set_rc(const RC_ctrl_t *rc);
// Update a specific manual input source (see MANUAL_INPUT_SRC_* in config.h).
// NOTE: get_remote_control_point() always returns the same pointer; this updates its contents.
extern void remote_control_set_rc_source(uint8_t source, const RC_ctrl_t *rc);
extern void remote_control_log_raw_source(uint8_t source,
                                          uint8_t proto,
                                          uint8_t range_mode,
                                          uint8_t channel_count,
                                          const int16_t *ch_raw,
                                          const uint8_t sw_raw[2],
                                          const RC_ctrl_t *decoded);
extern uint8_t remote_control_get_active_source(void);
extern void remote_control_refresh(void);
extern uint32_t remote_control_get_sbus_frame_count(void);
extern uint32_t remote_control_get_set_source_count(void);
extern uint8_t RC_data_is_error(void);
extern void slove_RC_lost(void);
extern void slove_data_error(void);
#endif
