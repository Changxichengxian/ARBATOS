/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "ManualInput.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "main.h"

#include <string.h>

#include "RobotConfig.h"
#include "ControlInput.h"
#include "BspKey.h"

#include "DetectTask.h"
#include "SdLog.h"

#ifndef RC_CHANNEL_ERROR_VALUE
#define RC_CHANNEL_ERROR_VALUE ((int16_t)(RC_CH_VALUE_ABS_MAX + 64u))
#endif
static int16_t RC_abs(int16_t value);
/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, ManualInputState *rc_ctrl);

//remote control data
static ManualInputState rc_ctrl;

typedef struct
{
    ManualInputState rc;
    TickType_t last_update_tick;
    uint8_t valid;
} ManualInputSrcState;

// Per-source raw snapshots (index == MANUAL_INPUT_SRC_*).
static ManualInputSrcState manual_src[MANUAL_INPUT_SRC_MAX + 1u];
static uint8_t manual_active_src = MANUAL_INPUT_SRC_AUTO;
static TickType_t ManualInputRefreshTick = 0u;
static uint8_t ManualInputRefreshDirty = 1u;
static uint32_t ManualInputDirtySeq = 1u;
static uint32_t ManualInputRefreshSeq = 0u;
static uint32_t g_remote_control_sbus_frame_cnt = 0u;
static uint32_t g_remote_control_set_source_cnt = 0u;
static TimerHandle_t ManualInputRefreshTimer = NULL;
static StaticTimer_t ManualInputRefreshTimerBuffer;

typedef struct
{
    uint8_t from_isr;
    UBaseType_t saved_mask;
} ManualInputCriticalState;

static void ManualInputResetRc(ManualInputState *rc);
static void ManualInputSanitizeSwitch(ManualInputState *rc);
static void ManualInputApplyBoardKey(ManualInputState *rc);
static uint8_t ManualInputSrcIsActive(const ManualInputSrcState *src_state,
                                          uint8_t src,
                                          TickType_t now_tick,
                                          TickType_t timeout_tick);
static uint8_t ManualInputPickLatest(const ManualInputSrcState *src_state,
                                        TickType_t now_tick,
                                        TickType_t timeout_tick);
static void ManualInputCommitOutput(const ManualInputState *out, uint8_t active_src);
static void ManualInputMarkDirty(void);
static void ManualInputRefreshIfNeeded(uint8_t force);
static void ManualInputRefreshTimerCallback(TimerHandle_t timer);
static void ManualInputUpdateOutput(void);
static void remote_control_log_source_switch(uint8_t prev_src, uint8_t next_src);
static void remote_control_log_sbus_raw_frame(const uint8_t frame[RC_FRAME_LENGTH]);

static ManualInputCriticalState ManualInputEnterCritical(void)
{
    ManualInputCriticalState state;

    state.from_isr = (__get_IPSR() != 0U) ? 1u : 0u;
    state.saved_mask = 0u;
    if (state.from_isr != 0u)
    {
        state.saved_mask = taskENTER_CRITICAL_FROM_ISR();
    }
    else
    {
        taskENTER_CRITICAL();
    }
    return state;
}

static void ManualInputExitCritical(ManualInputCriticalState state)
{
    if (state.from_isr != 0u)
    {
        taskEXIT_CRITICAL_FROM_ISR(state.saved_mask);
    }
    else
    {
        taskEXIT_CRITICAL();
    }
}

/**
  * @brief          remote control init
  * @param[in]      none
  * @retval         none
  */
void ManualInputInit(void)
{
    TickType_t refresh_period_tick = pdMS_TO_TICKS(5u);

    ManualInputResetRc(&rc_ctrl);
    ControlInputUpdateFromManualInput(&rc_ctrl);
    for (uint8_t i = 0u; i <= (uint8_t)MANUAL_INPUT_SRC_MAX; i++)
    {
        ManualInputResetRc(&manual_src[i].rc);
        manual_src[i].last_update_tick = 0u;
        manual_src[i].valid = 0u;
    }
    manual_active_src = MANUAL_INPUT_SRC_AUTO;
    ManualInputRefreshTick = 0u;
    ManualInputRefreshDirty = 1u;
    ManualInputDirtySeq = 1u;
    ManualInputRefreshSeq = 0u;
    g_remote_control_sbus_frame_cnt = 0u;
    g_remote_control_set_source_cnt = 0u;
    if (refresh_period_tick == 0u)
    {
        refresh_period_tick = 1u;
    }

    ManualInputRefreshTimer = xTimerCreateStatic("ManualInputRefresh",
                                                    refresh_period_tick,
                                                    pdTRUE,
                                                    NULL,
                                                    ManualInputRefreshTimerCallback,
                                                    &ManualInputRefreshTimerBuffer);
    if (ManualInputRefreshTimer != NULL)
    {
        (void)xTimerStart(ManualInputRefreshTimer, 0u);
    }

    BspRcSbusInit();
}

void remote_control_init(void)
{
    ManualInputInit();
}
/**
  * @brief          get remote control data point
  * @param[in]      none
  * @retval         remote control data point
  */
const ManualInputState *ManualInputGetCurrentRc(void)
{
    return &rc_ctrl;
}

uint8_t ManualInputGetCurrentCopy(ManualInputState *out)
{
    if (out == NULL)
    {
        return 0u;
    }

    ManualInputRefreshIfNeeded(0u);

    ManualInputCriticalState critical = ManualInputEnterCritical();
    *out = rc_ctrl;
    ManualInputExitCritical(critical);
    return 1u;
}

const ManualInputState *get_remote_control_point(void)
{
    return ManualInputGetCurrentRc();
}

void remote_control_set_rc(const ManualInputState *rc)
{
    ManualInputUpdateSource(MANUAL_INPUT_SRC_ELRS, rc);
}

void ManualInputUpdateSource(uint8_t source, const ManualInputState *rc)
{
    const TickType_t now_tick = xTaskGetTickCount();

    if (rc == NULL)
    {
        return;
    }
    if (source == 0u || source > (uint8_t)MANUAL_INPUT_SRC_MAX)
    {
        return;
    }

    ManualInputCriticalState critical = ManualInputEnterCritical();
    manual_src[source].rc = *rc;
    manual_src[source].last_update_tick = now_tick;
    manual_src[source].valid = 1u;
    g_remote_control_set_source_cnt++;
    ManualInputRefreshDirty = 1u;
    ManualInputDirtySeq++;
    ManualInputExitCritical(critical);
    ManualInputRefreshIfNeeded(1u);
    DetectHook(DBUS_TOE);
}

void remote_control_set_rc_source(uint8_t source, const ManualInputState *rc)
{
    ManualInputUpdateSource(source, rc);
}

void ManualInputOnSbusFrame(const uint8_t frame[RC_FRAME_LENGTH])
{
    if (frame == NULL)
    {
        return;
    }

    ManualInputState rc = {0};
    sbus_to_rc(frame, &rc);
    remote_control_log_sbus_raw_frame(frame);
    ManualInputCriticalState critical = ManualInputEnterCritical();
    g_remote_control_sbus_frame_cnt++;
    ManualInputExitCritical(critical);
    ManualInputUpdateSource(MANUAL_INPUT_SRC_DBUS, &rc);
}

void remote_control_on_sbus_frame(const uint8_t frame[RC_FRAME_LENGTH])
{
    ManualInputOnSbusFrame(frame);
}

void remote_control_log_raw_source(uint8_t source,
                                   uint8_t proto,
                                   uint8_t range_mode,
                                   uint8_t channel_count,
                                   const int16_t *ch_raw,
                                   const uint8_t sw_raw[2],
                                   const ManualInputState *decoded)
{
    if (SdLogIsActive() == 0u || ch_raw == NULL || channel_count == 0u)
    {
        return;
    }
    if (channel_count > 16u)
    {
        channel_count = 16u;
    }

    sdlog_manual_input_raw_t pkt = {0};
    pkt.source = source;
    pkt.proto = proto;
    pkt.range_mode = range_mode;
    pkt.channel_count = channel_count;

    for (uint8_t i = 0u; i < channel_count; i++)
    {
        pkt.ch_raw[i] = ch_raw[i];
    }

    if (sw_raw != NULL)
    {
        pkt.sw[0] = sw_raw[0];
        pkt.sw[1] = sw_raw[1];
    }
    else if (decoded != NULL)
    {
        pkt.sw[0] = (uint8_t)decoded->rc.s[0];
        pkt.sw[1] = (uint8_t)decoded->rc.s[1];
    }

    if (decoded != NULL)
    {
        pkt.mouse_x = decoded->mouse.x;
        pkt.mouse_y = decoded->mouse.y;
        pkt.mouse_z = decoded->mouse.z;
        pkt.key_value = decoded->key.v;
        if (decoded->mouse.press_l != 0u)
        {
            pkt.mouse_btns |= 0x01u;
        }
        if (decoded->mouse.press_r != 0u)
        {
            pkt.mouse_btns |= 0x02u;
        }
    }

    SdLogWrite(SDLOG_TAG_MANUAL_INPUT_RAW, &pkt, (uint16_t)sizeof(pkt));
}

uint8_t ManualInputGetActiveSource(void)
{
    uint8_t active_src;

    ManualInputRefreshIfNeeded(0u);

    ManualInputCriticalState critical = ManualInputEnterCritical();
    active_src = manual_active_src;
    ManualInputExitCritical(critical);
    return active_src;
}

uint8_t remote_control_get_active_source(void)
{
    return ManualInputGetActiveSource();
}

void ManualInputRefresh(void)
{
    ManualInputMarkDirty();
    ManualInputRefreshIfNeeded(1u);
}

void remote_control_refresh(void)
{
    ManualInputRefresh();
}

uint32_t ManualInputGetSbusFrameCount(void)
{
    uint32_t count;

    ManualInputCriticalState critical = ManualInputEnterCritical();
    count = g_remote_control_sbus_frame_cnt;
    ManualInputExitCritical(critical);
    return count;
}

uint32_t remote_control_get_sbus_frame_count(void)
{
    return ManualInputGetSbusFrameCount();
}

uint32_t ManualInputGetSetSourceCount(void)
{
    uint32_t count;

    ManualInputCriticalState critical = ManualInputEnterCritical();
    count = g_remote_control_set_source_cnt;
    ManualInputExitCritical(critical);
    return count;
}

uint32_t remote_control_get_set_source_count(void)
{
    return ManualInputGetSetSourceCount();
}

static void ManualInputResetRc(ManualInputState *rc)
{
    if (rc == NULL)
    {
        return;
    }

    memset(rc, 0, sizeof(*rc));
    rc->rc.s[0] = (char)RC_SW_DOWN;
    rc->rc.s[1] = (char)RC_SW_DOWN;
}

static void ManualInputSanitizeSwitch(ManualInputState *rc)
{
    if (rc == NULL)
    {
        return;
    }

    const uint8_t s0 = (uint8_t)rc->rc.s[0];
    const uint8_t s1 = (uint8_t)rc->rc.s[1];
    if (!(s0 == RC_SW_UP || s0 == RC_SW_MID || s0 == RC_SW_DOWN))
    {
        rc->rc.s[0] = (char)RC_SW_DOWN;
    }
    if (!(s1 == RC_SW_UP || s1 == RC_SW_MID || s1 == RC_SW_DOWN))
    {
        rc->rc.s[1] = (char)RC_SW_DOWN;
    }
}

static void ManualInputApplyBoardKey(ManualInputState *rc)
{
    if (rc == NULL)
    {
        return;
    }

    const uint16_t mask = g_config.manual_input.BoardKeyKeyMask;
    if (mask == 0u)
    {
        return;
    }
    if (BspKeyReadRawDown() != 0u)
    {
        rc->key.v |= mask;
    }
}

static uint8_t ManualInputSrcIsActive(const ManualInputSrcState *src_state,
                                          uint8_t src,
                                          TickType_t now_tick,
                                          TickType_t timeout_tick)
{
    if (src_state == NULL || src == 0u || src > (uint8_t)MANUAL_INPUT_SRC_MAX)
    {
        return 0u;
    }
    if (src_state[src].valid == 0u)
    {
        return 0u;
    }
    if (timeout_tick == 0u)
    {
        return 1u;
    }
    const TickType_t age = (TickType_t)(now_tick - src_state[src].last_update_tick);
    return (age <= timeout_tick) ? 1u : 0u;
}

static uint8_t ManualInputPickLatest(const ManualInputSrcState *src_state,
                                        TickType_t now_tick,
                                        TickType_t timeout_tick)
{
    uint8_t best = 0u;
    TickType_t best_age = (TickType_t)0xFFFFFFFFu;

    for (uint8_t src = (uint8_t)MANUAL_INPUT_SRC_DBUS; src <= (uint8_t)MANUAL_INPUT_SRC_MAX; src++)
    {
        if (!ManualInputSrcIsActive(src_state, src, now_tick, timeout_tick))
        {
            continue;
        }

        const TickType_t age = (TickType_t)(now_tick - src_state[src].last_update_tick);
        if (age < best_age)
        {
            best_age = age;
            best = src;
        }
    }

    return best;
}

static void ManualInputCommitOutput(const ManualInputState *out, uint8_t active_src)
{
    uint8_t prev_src;

    if (out == NULL)
    {
        return;
    }

    ManualInputCriticalState critical = ManualInputEnterCritical();
    prev_src = manual_active_src;
    rc_ctrl = *out;
    manual_active_src = active_src;
    ManualInputExitCritical(critical);

    ControlInputUpdateFromManualInput(out);

    if (prev_src != active_src)
    {
        remote_control_log_source_switch(prev_src, active_src);
    }
}

static void ManualInputMarkDirty(void)
{
    ManualInputCriticalState critical = ManualInputEnterCritical();
    ManualInputRefreshDirty = 1u;
    ManualInputDirtySeq++;
    ManualInputExitCritical(critical);
}

static void ManualInputRefreshIfNeeded(uint8_t force)
{
    const ManualInputConfig *cfg = &g_config.manual_input;
    const TickType_t now_tick = xTaskGetTickCount();
    const uint8_t needs_periodic_refresh =
        (cfg->source_timeout_ms != 0u || cfg->BoardKeyKeyMask != 0u) ? 1u : 0u;
    uint8_t dirty;
    TickType_t refresh_tick;
    uint32_t dirty_seq;
    uint32_t refresh_seq;

    ManualInputCriticalState critical = ManualInputEnterCritical();
    dirty = ManualInputRefreshDirty;
    refresh_tick = ManualInputRefreshTick;
    dirty_seq = ManualInputDirtySeq;
    refresh_seq = ManualInputRefreshSeq;
    ManualInputExitCritical(critical);

    if (force == 0u && dirty == 0u && dirty_seq == refresh_seq)
    {
        if (needs_periodic_refresh == 0u)
        {
            return;
        }
        if (refresh_tick == now_tick)
        {
            return;
        }
    }

    ManualInputUpdateOutput();
    critical = ManualInputEnterCritical();
    ManualInputRefreshTick = now_tick;
    ManualInputRefreshSeq = dirty_seq;
    if (ManualInputDirtySeq == dirty_seq)
    {
        ManualInputRefreshDirty = 0u;
    }
    ManualInputExitCritical(critical);
}

static void ManualInputRefreshTimerCallback(TimerHandle_t timer)
{
    (void)timer;
    ManualInputRefreshIfNeeded(0u);
}

static void ManualInputUpdateOutput(void)
{
    const ManualInputConfig *cfg = &g_config.manual_input;
    const TickType_t now_tick = xTaskGetTickCount();
    const TickType_t timeout_tick = (cfg->source_timeout_ms == 0u) ? 0u : pdMS_TO_TICKS(cfg->source_timeout_ms);
    ManualInputSrcState src_snapshot[MANUAL_INPUT_SRC_MAX + 1u];

    ManualInputCriticalState critical = ManualInputEnterCritical();
    memcpy(src_snapshot, manual_src, sizeof(src_snapshot));
    ManualInputExitCritical(critical);

    const uint8_t latest = ManualInputPickLatest(src_snapshot, now_tick, timeout_tick);

    if (cfg->mix_mode == MANUAL_INPUT_MIX_MERGE)
    {
        if (latest == 0u)
        {
            ManualInputState out;
            ManualInputResetRc(&out);
            ManualInputApplyBoardKey(&out);
            ManualInputCommitOutput(&out, MANUAL_INPUT_SRC_AUTO);
            return;
        }

        ManualInputState out;
        ManualInputResetRc(&out);

        // Switches/mouse follow "latest"; keys are merged.
        out.rc.s[0] = src_snapshot[latest].rc.rc.s[0];
        out.rc.s[1] = src_snapshot[latest].rc.rc.s[1];
        ManualInputSanitizeSwitch(&out);

        for (uint8_t src = (uint8_t)MANUAL_INPUT_SRC_DBUS; src <= (uint8_t)MANUAL_INPUT_SRC_MAX; src++)
        {
            if (!ManualInputSrcIsActive(src_snapshot, src, now_tick, timeout_tick))
            {
                continue;
            }

            for (uint8_t ch = 0u; ch < 5u; ch++)
            {
                const int16_t v = src_snapshot[src].rc.rc.ch[ch];
                if (RC_abs(v) > RC_abs(out.rc.ch[ch]))
                {
                    out.rc.ch[ch] = v;
                }
            }

            const int16_t mx = src_snapshot[src].rc.mouse.x;
            const int16_t my = src_snapshot[src].rc.mouse.y;
            const int16_t mz = src_snapshot[src].rc.mouse.z;
            if (RC_abs(mx) > RC_abs(out.mouse.x)) out.mouse.x = mx;
            if (RC_abs(my) > RC_abs(out.mouse.y)) out.mouse.y = my;
            if (RC_abs(mz) > RC_abs(out.mouse.z)) out.mouse.z = mz;

            out.key.v |= src_snapshot[src].rc.key.v;
            out.mouse.press_l |= src_snapshot[src].rc.mouse.press_l;
            out.mouse.press_r |= src_snapshot[src].rc.mouse.press_r;
        }

        ManualInputApplyBoardKey(&out);
        ManualInputCommitOutput(&out, latest);
        return;
    }

    uint8_t selected = cfg->active_source;
    if (selected == MANUAL_INPUT_SRC_AUTO)
    {
        selected = latest;
    }
    else if (!ManualInputSrcIsActive(src_snapshot, selected, now_tick, timeout_tick))
    {
        selected = latest;
    }

    if (selected == 0u)
    {
        ManualInputState out;
        ManualInputResetRc(&out);
        ManualInputApplyBoardKey(&out);
        ManualInputCommitOutput(&out, MANUAL_INPUT_SRC_AUTO);
        return;
    }

    ManualInputState out = src_snapshot[selected].rc;
    ManualInputSanitizeSwitch(&out);
    ManualInputApplyBoardKey(&out);
    ManualInputCommitOutput(&out, selected);
}

uint8_t RC_data_is_error(void)
{
    ManualInputState rc_snapshot;

    // Pure check (no side effects): do not modify rc_ctrl here.
    ManualInputCriticalState critical = ManualInputEnterCritical();
    rc_snapshot = rc_ctrl;
    ManualInputExitCritical(critical);

    if (RC_abs(rc_snapshot.rc.ch[0]) > RC_CHANNEL_ERROR_VALUE) return 1;
    if (RC_abs(rc_snapshot.rc.ch[1]) > RC_CHANNEL_ERROR_VALUE) return 1;
    if (RC_abs(rc_snapshot.rc.ch[2]) > RC_CHANNEL_ERROR_VALUE) return 1;
    if (RC_abs(rc_snapshot.rc.ch[3]) > RC_CHANNEL_ERROR_VALUE) return 1;
    if (rc_snapshot.rc.s[0] == 0) return 1;
    if (rc_snapshot.rc.s[1] == 0) return 1;
    return 0;
}

void slove_RC_lost(void)
{
    RC_restart(SBUS_RX_BUF_NUM);
}
void slove_data_error(void)
{
    RC_restart(SBUS_RX_BUF_NUM);
}

static int16_t RC_abs(int16_t value)
{
    if (value > 0)
    {
        return value;
    }
    else
    {
        return -value;
    }
}

static void remote_control_log_source_switch(uint8_t prev_src, uint8_t next_src)
{
    uint32_t set_source_cnt;

    if (SdLogIsActive() == 0u)
    {
        return;
    }

    ManualInputCriticalState critical = ManualInputEnterCritical();
    set_source_cnt = g_remote_control_set_source_cnt;
    ManualInputExitCritical(critical);

    sdlog_event_t evt = {0};
    evt.event_id = SDLOG_EVT_MANUAL_SOURCE_SWITCH;
    evt.arg0_u16 = next_src;
    evt.arg1_u32 = prev_src;
    evt.arg2_u32 = set_source_cnt;
    SdLogWrite(SDLOG_TAG_EVENT, &evt, (uint16_t)sizeof(evt));
}

static void remote_control_log_sbus_raw_frame(const uint8_t frame[RC_FRAME_LENGTH])
{
    if (frame == NULL)
    {
        return;
    }

    int16_t ch_raw[5] = {0};
    const uint8_t sw_raw[2] = {
        (uint8_t)((frame[5] >> 4) & 0x03u),
        (uint8_t)(((frame[5] >> 4) & 0x0Cu) >> 2),
    };
    ManualInputState decoded = {0};

    ch_raw[0] = (int16_t)((frame[0] | (frame[1] << 8)) & 0x07FFu);
    ch_raw[1] = (int16_t)(((frame[1] >> 3) | (frame[2] << 5)) & 0x07FFu);
    ch_raw[2] = (int16_t)(((frame[2] >> 6) | (frame[3] << 2) | (frame[4] << 10)) & 0x07FFu);
    ch_raw[3] = (int16_t)(((frame[4] >> 1) | (frame[5] << 7)) & 0x07FFu);
    ch_raw[4] = (int16_t)(frame[16] | (frame[17] << 8));

    decoded.rc.s[0] = (char)sw_raw[0];
    decoded.rc.s[1] = (char)sw_raw[1];
    decoded.mouse.x = (int16_t)(frame[6] | (frame[7] << 8));
    decoded.mouse.y = (int16_t)(frame[8] | (frame[9] << 8));
    decoded.mouse.z = (int16_t)(frame[10] | (frame[11] << 8));
    decoded.mouse.press_l = frame[12];
    decoded.mouse.press_r = frame[13];
    decoded.key.v = (uint16_t)(frame[14] | (frame[15] << 8));

    remote_control_log_raw_source(MANUAL_INPUT_SRC_DBUS,
                                  SDLOG_MANUAL_INPUT_PROTO_DBUS,
                                  SDLOG_MANUAL_INPUT_RANGE_RAW_11BIT,
                                  5u,
                                  ch_raw,
                                  sw_raw,
                                  &decoded);
}
/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, ManualInputState *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
                         (sbus_buf[4] << 10)) &0x07ff;
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                       //!< Switch right
    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
    rc_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
    rc_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
    rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 //NULL

    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;

    for (uint8_t i = 0u; i < 5u; i++)
    {
        rc_ctrl->rc.ch[i] = rc_scale_axis_by_abs(rc_ctrl->rc.ch[i],
                                                 (int16_t)RC_CH_VALUE_ABS_LEGACY,
                                                 (int16_t)RC_CH_VALUE_ABS_MAX);
    }
}
