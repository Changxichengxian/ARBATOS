/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */


#ifndef MANUAL_INPUT_H
#define MANUAL_INPUT_H
#include <stddef.h>

#include "types.h"
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
#pragma pack(push, 1)

typedef struct manual_input_rc
{
        int16_t ch[5];
        char s[2];
} manual_input_rc_t;

typedef struct manual_input_mouse
{
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t press_l;
        uint8_t press_r;
} manual_input_mouse_t;

typedef struct manual_input_key
{
        uint16_t v;
} manual_input_key_t;

typedef struct manual_input_state
{
        manual_input_rc_t rc;
        manual_input_mouse_t mouse;
        manual_input_key_t key;
} manual_input_state_t;

#pragma pack(pop)

#define MANUAL_INPUT_RC_SIZE_BYTES      12u
#define MANUAL_INPUT_MOUSE_SIZE_BYTES   8u
#define MANUAL_INPUT_KEY_SIZE_BYTES     2u
#define MANUAL_INPUT_STATE_SIZE_BYTES   22u
#define MANUAL_INPUT_RC_OFFSET_BYTES    0u
#define MANUAL_INPUT_MOUSE_OFFSET_BYTES 12u
#define MANUAL_INPUT_KEY_OFFSET_BYTES   20u

typedef char manual_input_rc_size_check[(sizeof(manual_input_rc_t) == MANUAL_INPUT_RC_SIZE_BYTES) ? 1 : -1];
typedef char manual_input_mouse_size_check[(sizeof(manual_input_mouse_t) == MANUAL_INPUT_MOUSE_SIZE_BYTES) ? 1 : -1];
typedef char manual_input_key_size_check[(sizeof(manual_input_key_t) == MANUAL_INPUT_KEY_SIZE_BYTES) ? 1 : -1];
typedef char manual_input_state_size_check[(sizeof(manual_input_state_t) == MANUAL_INPUT_STATE_SIZE_BYTES) ? 1 : -1];
typedef char manual_input_rc_offset_check[(offsetof(manual_input_state_t, rc) == MANUAL_INPUT_RC_OFFSET_BYTES) ? 1 : -1];
typedef char manual_input_mouse_offset_check[(offsetof(manual_input_state_t, mouse) == MANUAL_INPUT_MOUSE_OFFSET_BYTES) ? 1 : -1];
typedef char manual_input_key_offset_check[(offsetof(manual_input_state_t, key) == MANUAL_INPUT_KEY_OFFSET_BYTES) ? 1 : -1];

/*
 * Manual input layers:
 * - `rc_sbus_task.c`: only drains board-level SBUS/DBUS frames and forwards them here.
 * - `manual_input.c`: decodes raw frames, merges DBUS/ELRS/Image sources, and owns the shared `manual_input_state_t`.
 * - `control_input.c`: remaps the merged `manual_input_state_t` into game-facing axes/switches via `g_config.input`.
 *
 * If you want to:
 * - change SBUS/DBUS decode: edit `manual_input_on_sbus_frame()` / `sbus_to_rc()` in `manual_input.c`
 * - change which source wins: edit `manual_input_update_source()` and the `manual_input_*` merge helpers
 * - change axis/switch mapping: edit `control_input.c` and the `input` block in `Robotconfig/<TARGET>/config.c`
 */

/* Preferred names for new code. Legacy `remote_control_*` names stay for compatibility. */
extern void manual_input_init(void);
extern void manual_input_on_sbus_frame(const uint8_t frame[RC_FRAME_LENGTH]);
extern const manual_input_state_t *manual_input_get_current_rc(void);
extern void manual_input_update_source(uint8_t source, const manual_input_state_t *rc);
extern uint8_t manual_input_get_active_source(void);
extern void manual_input_refresh(void);
extern uint32_t manual_input_get_sbus_frame_count(void);
extern uint32_t manual_input_get_set_source_count(void);

extern void remote_control_init(void);
extern void remote_control_on_sbus_frame(const uint8_t frame[RC_FRAME_LENGTH]);
extern const manual_input_state_t *get_remote_control_point(void);
extern void remote_control_set_rc(const manual_input_state_t *rc);
extern void remote_control_set_rc_source(uint8_t source, const manual_input_state_t *rc);
extern void remote_control_log_raw_source(uint8_t source,
                                          uint8_t proto,
                                          uint8_t range_mode,
                                          uint8_t channel_count,
                                          const int16_t *ch_raw,
                                          const uint8_t sw_raw[2],
                                          const manual_input_state_t *decoded);
extern uint8_t remote_control_get_active_source(void);
extern void remote_control_refresh(void);
extern uint32_t remote_control_get_sbus_frame_count(void);
extern uint32_t remote_control_get_set_source_count(void);
extern uint8_t RC_data_is_error(void);
extern void slove_RC_lost(void);
extern void slove_data_error(void);
#endif
