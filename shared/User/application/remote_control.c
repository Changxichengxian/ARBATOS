/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */


#include "remote_control.h"

#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

#include "app_config.h"
#include "app_input.h"
#include "bsp_key.h"

#include "detect_task.h"
#include "sdlog.h"



//遥控器出错数据上限
#define RC_CHANNAL_ERROR_VALUE ((int16_t)(RC_CH_VALUE_ABS_MAX + 64u))



//取正函数
#ifndef RC_CHANNAL_ERROR_VALUE
#define RC_CHANNAL_ERROR_VALUE ((int16_t)(RC_CH_VALUE_ABS_MAX + 64u))
#endif
static int16_t RC_abs(int16_t value);
/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指
  * @retval         none
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);

//remote control data 
//遥控器控制变量
RC_ctrl_t rc_ctrl;

typedef struct
{
    RC_ctrl_t rc;
    TickType_t last_update_tick;
    uint8_t valid;
} manual_input_src_state_t;

// Per-source raw snapshots (index == MANUAL_INPUT_SRC_*).
static manual_input_src_state_t manual_src[MANUAL_INPUT_SRC_MAX + 1u];
static uint8_t manual_active_src = MANUAL_INPUT_SRC_AUTO;
static uint32_t g_remote_control_sbus_frame_cnt = 0u;
static uint32_t g_remote_control_set_source_cnt = 0u;

static void manual_input_reset_rc(RC_ctrl_t *rc);
static void manual_input_sanitize_switch(RC_ctrl_t *rc);
static void manual_input_apply_board_key(RC_ctrl_t *rc);
static uint8_t manual_input_src_is_active(uint8_t src, TickType_t now_tick, TickType_t timeout_tick);
static uint8_t manual_input_pick_latest(TickType_t now_tick, TickType_t timeout_tick);
static void manual_input_commit_output(const RC_ctrl_t *out, uint8_t active_src);
static void manual_input_update_output(void);
static void remote_control_log_source_switch(uint8_t prev_src, uint8_t next_src);
static void remote_control_log_sbus_raw_frame(const uint8_t frame[RC_FRAME_LENGTH]);
// SBUS/DBUS 的 DMA/IRQ 接收放在 BSP(bsp_rc.c)；这里只负责协议解析和输入融合。


/**
  * @brief          remote control init
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          遥控器初始化
  * @param[in]      none
  * @retval         none
  */
void remote_control_init(void)
{
    manual_input_reset_rc(&rc_ctrl);
    app_input_update_from_rc(&rc_ctrl);
    for (uint8_t i = 0u; i <= (uint8_t)MANUAL_INPUT_SRC_MAX; i++)
    {
        manual_input_reset_rc(&manual_src[i].rc);
        manual_src[i].last_update_tick = 0u;
        manual_src[i].valid = 0u;
    }
    manual_active_src = MANUAL_INPUT_SRC_AUTO;
    g_remote_control_sbus_frame_cnt = 0u;
    g_remote_control_set_source_cnt = 0u;

    bsp_rc_sbus_init();
}
/**
  * @brief          get remote control data point
  * @param[in]      none
  * @retval         remote control data point
  */
/**
  * @brief          获取遥控器数据指针
  * @param[in]      none
  * @retval         遥控器数据指针
  */
const RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;
}

void remote_control_set_rc(const RC_ctrl_t *rc)
{
    remote_control_set_rc_source(MANUAL_INPUT_SRC_ELRS, rc);
}

void remote_control_set_rc_source(uint8_t source, const RC_ctrl_t *rc)
{
    if (rc == NULL)
    {
        return;
    }
    if (source == 0u || source > (uint8_t)MANUAL_INPUT_SRC_MAX)
    {
        return;
    }

    manual_src[source].rc = *rc;
    manual_src[source].last_update_tick = xTaskGetTickCount();
    manual_src[source].valid = 1u;
    g_remote_control_set_source_cnt++;

    manual_input_update_output();
    detect_hook(DBUS_TOE);
}

void remote_control_on_sbus_frame(const uint8_t frame[RC_FRAME_LENGTH])
{
    if (frame == NULL)
    {
        return;
    }

    RC_ctrl_t rc = {0};
    sbus_to_rc(frame, &rc);
    remote_control_log_sbus_raw_frame(frame);
    g_remote_control_sbus_frame_cnt++;
    remote_control_set_rc_source(MANUAL_INPUT_SRC_DBUS, &rc);
}

void remote_control_log_raw_source(uint8_t source,
                                   uint8_t proto,
                                   uint8_t range_mode,
                                   uint8_t channel_count,
                                   const int16_t *ch_raw,
                                   const uint8_t sw_raw[2],
                                   const RC_ctrl_t *decoded)
{
    if (sdlog_is_active() == 0u || ch_raw == NULL || channel_count == 0u)
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

    sdlog_write(SDLOG_TAG_MANUAL_INPUT_RAW, &pkt, (uint16_t)sizeof(pkt));
}

uint8_t remote_control_get_active_source(void)
{
    return manual_active_src;
}

void remote_control_refresh(void)
{
    manual_input_update_output();
}

uint32_t remote_control_get_sbus_frame_count(void)
{
    return g_remote_control_sbus_frame_cnt;
}

uint32_t remote_control_get_set_source_count(void)
{
    return g_remote_control_set_source_cnt;
}

static void manual_input_reset_rc(RC_ctrl_t *rc)
{
    if (rc == NULL)
    {
        return;
    }

    memset(rc, 0, sizeof(*rc));
    rc->rc.s[0] = (char)RC_SW_DOWN;
    rc->rc.s[1] = (char)RC_SW_DOWN;
}

static void manual_input_sanitize_switch(RC_ctrl_t *rc)
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

static void manual_input_apply_board_key(RC_ctrl_t *rc)
{
    if (rc == NULL)
    {
        return;
    }

    const uint16_t mask = g_app_config.manual_input.board_key_key_mask;
    if (mask == 0u)
    {
        return;
    }
    if (bsp_key_read_raw_down() != 0u)
    {
        rc->key.v |= mask;
    }
}

static uint8_t manual_input_src_is_active(uint8_t src, TickType_t now_tick, TickType_t timeout_tick)
{
    if (src == 0u || src > (uint8_t)MANUAL_INPUT_SRC_MAX)
    {
        return 0u;
    }
    if (manual_src[src].valid == 0u)
    {
        return 0u;
    }
    if (timeout_tick == 0u)
    {
        return 1u;
    }
    const TickType_t age = (TickType_t)(now_tick - manual_src[src].last_update_tick);
    return (age <= timeout_tick) ? 1u : 0u;
}

static uint8_t manual_input_pick_latest(TickType_t now_tick, TickType_t timeout_tick)
{
    uint8_t best = 0u;
    TickType_t best_age = (TickType_t)0xFFFFFFFFu;

    for (uint8_t src = (uint8_t)MANUAL_INPUT_SRC_DBUS; src <= (uint8_t)MANUAL_INPUT_SRC_MAX; src++)
    {
        if (!manual_input_src_is_active(src, now_tick, timeout_tick))
        {
            continue;
        }

        const TickType_t age = (TickType_t)(now_tick - manual_src[src].last_update_tick);
        if (age < best_age)
        {
            best_age = age;
            best = src;
        }
    }

    return best;
}

static void manual_input_commit_output(const RC_ctrl_t *out, uint8_t active_src)
{
    const uint8_t prev_src = manual_active_src;

    if (out == NULL)
    {
        return;
    }

    rc_ctrl = *out;
    manual_active_src = active_src;
    app_input_update_from_rc(&rc_ctrl);

    if (prev_src != active_src)
    {
        remote_control_log_source_switch(prev_src, active_src);
    }
}

static void manual_input_update_output(void)
{
    const manual_input_config_t *cfg = &g_app_config.manual_input;
    const TickType_t now_tick = xTaskGetTickCount();
    const TickType_t timeout_tick = (cfg->source_timeout_ms == 0u) ? 0u : pdMS_TO_TICKS(cfg->source_timeout_ms);
    const uint8_t latest = manual_input_pick_latest(now_tick, timeout_tick);

    if (cfg->mix_mode == MANUAL_INPUT_MIX_MERGE)
    {
        if (latest == 0u)
        {
            RC_ctrl_t out;
            manual_input_reset_rc(&out);
            manual_input_apply_board_key(&out);
            manual_input_commit_output(&out, MANUAL_INPUT_SRC_AUTO);
            return;
        }

        RC_ctrl_t out;
        manual_input_reset_rc(&out);

        // Switches/mouse follow "latest"; keys are merged.
        out.rc.s[0] = manual_src[latest].rc.rc.s[0];
        out.rc.s[1] = manual_src[latest].rc.rc.s[1];
        manual_input_sanitize_switch(&out);

        for (uint8_t src = (uint8_t)MANUAL_INPUT_SRC_DBUS; src <= (uint8_t)MANUAL_INPUT_SRC_MAX; src++)
        {
            if (!manual_input_src_is_active(src, now_tick, timeout_tick))
            {
                continue;
            }

            for (uint8_t ch = 0u; ch < 5u; ch++)
            {
                const int16_t v = manual_src[src].rc.rc.ch[ch];
                if (RC_abs(v) > RC_abs(out.rc.ch[ch]))
                {
                    out.rc.ch[ch] = v;
                }
            }

            const int16_t mx = manual_src[src].rc.mouse.x;
            const int16_t my = manual_src[src].rc.mouse.y;
            const int16_t mz = manual_src[src].rc.mouse.z;
            if (RC_abs(mx) > RC_abs(out.mouse.x)) out.mouse.x = mx;
            if (RC_abs(my) > RC_abs(out.mouse.y)) out.mouse.y = my;
            if (RC_abs(mz) > RC_abs(out.mouse.z)) out.mouse.z = mz;

            out.key.v |= manual_src[src].rc.key.v;
            out.mouse.press_l |= manual_src[src].rc.mouse.press_l;
            out.mouse.press_r |= manual_src[src].rc.mouse.press_r;
        }

        manual_input_apply_board_key(&out);
        manual_input_commit_output(&out, latest);
        return;
    }

    uint8_t selected = cfg->active_source;
    if (selected == MANUAL_INPUT_SRC_AUTO)
    {
        selected = latest;
    }
    else if (!manual_input_src_is_active(selected, now_tick, timeout_tick))
    {
        selected = latest;
    }

    if (selected == 0u)
    {
        RC_ctrl_t out;
        manual_input_reset_rc(&out);
        manual_input_apply_board_key(&out);
        manual_input_commit_output(&out, MANUAL_INPUT_SRC_AUTO);
        return;
    }

    RC_ctrl_t out = manual_src[selected].rc;
    manual_input_sanitize_switch(&out);
    manual_input_apply_board_key(&out);
    manual_input_commit_output(&out, selected);
}

//判断遥控器数据是否出错，
uint8_t RC_data_is_error(void)
{
    // Pure check (no side effects): do not modify rc_ctrl here.
    if (RC_abs(rc_ctrl.rc.ch[0]) > RC_CHANNAL_ERROR_VALUE) return 1;
    if (RC_abs(rc_ctrl.rc.ch[1]) > RC_CHANNAL_ERROR_VALUE) return 1;
    if (RC_abs(rc_ctrl.rc.ch[2]) > RC_CHANNAL_ERROR_VALUE) return 1;
    if (RC_abs(rc_ctrl.rc.ch[3]) > RC_CHANNAL_ERROR_VALUE) return 1;
    if (rc_ctrl.rc.s[0] == 0) return 1;
    if (rc_ctrl.rc.s[1] == 0) return 1;
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

//取正函数
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
    if (sdlog_is_active() == 0u)
    {
        return;
    }

    sdlog_event_t evt = {0};
    evt.event_id = SDLOG_EVT_MANUAL_SOURCE_SWITCH;
    evt.arg0_u16 = next_src;
    evt.arg1_u32 = prev_src;
    evt.arg2_u32 = g_remote_control_set_source_cnt;
    sdlog_write(SDLOG_TAG_EVENT, &evt, (uint16_t)sizeof(evt));
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
    RC_ctrl_t decoded = {0};

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
/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指
  * @retval         none
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
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
