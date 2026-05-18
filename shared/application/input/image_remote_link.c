/*
 * SPDX-FileCopyrightText: 2026 陈卓 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈卓 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "image_remote_link.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "config.h"
#include "control_input.h"
#include "manual_input.h"
#include "CRC8_CRC16.h"

#define IMAGE_REMOTE_FRAME_SOF          0xA5u
#define IMAGE_REMOTE_DMA_RX_BUF_SIZE    512u
#define IMAGE_REMOTE_RM_FRAME_MAX_SIZE  64u
#define IMAGE_REMOTE_VT13_FRAME_SIZE    21u
#define IMAGE_REMOTE_RC_MAGIC0          'R'
#define IMAGE_REMOTE_RC_MAGIC1          'C'
#define IMAGE_REMOTE_RC_VERSION         1u
#define IMAGE_REMOTE_RC_RANGE_DBUS      0u
#define IMAGE_REMOTE_RC_RANGE_VT13      1u
#define IMAGE_REMOTE_RC_BTN_LEFT        (1u << 0)
#define IMAGE_REMOTE_RC_BTN_RIGHT       (1u << 1)
#define IMAGE_REMOTE_RC_ABS_MAX_DBUS    ((int16_t)RC_CH_VALUE_ABS_LEGACY)
#define IMAGE_REMOTE_RC_ABS_MAX_VT13    ((int16_t)RC_CH_VALUE_ABS_MAX)
#define IMAGE_REMOTE_CMD_CUSTOM_CONTROLLER_RX 0x0302u
#define IMAGE_REMOTE_CMD_CUSTOM_CLIENT_RX     0x0311u
#define IMAGE_REMOTE_KEY_FLAG(value, mask) ((((value) & (mask)) != 0u) ? 1u : 0u)

typedef struct __attribute__((packed))
{
    uint8_t magic0;
    uint8_t magic1;
    uint8_t version;
    uint8_t range_mode;
    int16_t ch[5];
    uint8_t sw[2];
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint8_t mouse_btns;
    uint16_t key_value;
    uint8_t reserved[5];
} image_remote_rc_packet_t;

typedef char _check_image_remote_rc_packet_size[(sizeof(image_remote_rc_packet_t) == 30u) ? 1 : -1];

typedef enum
{
    IMAGE_REMOTE_PARSE_IDLE = 0,
    IMAGE_REMOTE_PARSE_RM,
    IMAGE_REMOTE_PARSE_VT13,
} image_remote_parse_mode_e;

static uint8_t image_remote_dma_rx_buf[IMAGE_REMOTE_DMA_RX_BUF_SIZE];
static volatile uint16_t image_remote_dma_pos = 0u;
static uint16_t image_remote_dma_last_pos = 0u;
static volatile uint32_t image_remote_dma_wrap_cnt = 0u;
static uint32_t image_remote_dma_last_wrap_cnt = 0u;
static volatile uint8_t image_remote_dma_active = 0u;
static volatile uint8_t image_remote_dma_restart_req = 0u;
static image_remote_parse_mode_e image_remote_parse_mode = IMAGE_REMOTE_PARSE_IDLE;
static uint8_t image_remote_rm_buf[IMAGE_REMOTE_RM_FRAME_MAX_SIZE];
static uint16_t image_remote_rm_pos = 0u;
static uint16_t image_remote_rm_expected = 0u;
static uint8_t image_remote_vt13_buf[IMAGE_REMOTE_VT13_FRAME_SIZE];
static uint8_t image_remote_vt13_pos = 0u;
static volatile uint32_t image_remote_last_rx_tick_ms = 0u;
static volatile uint32_t image_remote_frame_cnt = 0u;
static volatile uint32_t image_remote_controller_frame_cnt = 0u;
static volatile uint32_t image_remote_client_frame_cnt = 0u;
static volatile uint32_t image_remote_vt13_frame_cnt = 0u;
static volatile uint32_t image_remote_crc_error_cnt = 0u;
static volatile uint32_t image_remote_parse_error_cnt = 0u;
static volatile uint32_t image_remote_restart_cnt = 0u;
static volatile uint16_t image_remote_last_cmd_id = 0u;
static volatile uint8_t image_remote_last_range_mode = 0u;
static image_remote_state_t image_remote_state = {0};

static void image_remote_store(const image_remote_state_t *state);
static void image_remote_link_process_to(uint16_t pos);
static void image_remote_link_reset_parser(void);
static void image_remote_link_feed_byte(uint8_t b);
static void image_remote_link_handle_rm_frame(const uint8_t *frame, uint16_t frame_len);
static void image_remote_link_handle_vt13_frame(const uint8_t *frame, uint16_t frame_len);
static bool_t image_remote_link_try_decode_custom_rc(const uint8_t *data);
static int16_t image_remote_link_scale_axis(int16_t raw, int16_t raw_abs_max);
static uint8_t image_remote_link_sanitize_switch(uint8_t value);
static uint8_t image_remote_link_map_vt13_switch1(uint8_t value);
static uint8_t image_remote_link_map_vt13_switch2(uint8_t stop, uint8_t left, uint8_t right);

bool image_remote_get_state(image_remote_state_t *out)
{
    if (out == NULL)
    {
        return false;
    }

    taskENTER_CRITICAL();
    *out = image_remote_state;
    taskEXIT_CRITICAL();
    return (out->valid != 0u);
}

bool image_remote_auto_aim_requested(void)
{
    image_remote_state_t state;
    if (!image_remote_get_state(&state))
    {
        return false;
    }
    if (manual_input_get_active_source() != MANUAL_INPUT_SRC_IMAGE)
    {
        return false;
    }
    return ((g_config.manual_input.vt13.auto_aim_pause_enable != 0u) && (state.pause != 0u)) ||
           ((g_config.manual_input.vt13.auto_aim_mouse_r_enable != 0u) && (state.mouse_r != 0u));
}

bool image_remote_aux_fire_requested(void)
{
    image_remote_state_t state;
    if (!image_remote_get_state(&state))
    {
        return false;
    }
    if (manual_input_get_active_source() != MANUAL_INPUT_SRC_IMAGE)
    {
        return false;
    }
    return ((g_config.manual_input.vt13.aux_fire_btn_l_enable != 0u) && (state.btn_l != 0u)) ||
           ((g_config.manual_input.vt13.aux_fire_mouse_l_enable != 0u) && (state.mouse_l != 0u));
}

void image_remote_link_get_stats(sdlog_image_link_stats_t *out)
{
    if (out == NULL)
    {
        return;
    }

    memset(out, 0, sizeof(*out));
    taskENTER_CRITICAL();
    out->last_rx_tick_ms = image_remote_last_rx_tick_ms;
    out->frame_count = image_remote_frame_cnt;
    out->controller_frame_count = image_remote_controller_frame_cnt;
    out->client_frame_count = image_remote_client_frame_cnt;
    out->vt13_frame_count = image_remote_vt13_frame_cnt;
    out->crc_error_count = image_remote_crc_error_cnt;
    out->parse_error_count = image_remote_parse_error_cnt;
    out->restart_count = image_remote_restart_cnt;
    out->last_cmd_id = image_remote_last_cmd_id;
    out->port_active = image_remote_dma_active;
    out->last_range_mode = image_remote_last_range_mode;
    taskEXIT_CRITICAL();
}

void image_remote_link_start(void)
{
    image_remote_link_stop();
    image_remote_dma_pos = 0u;
    image_remote_dma_last_pos = 0u;
    image_remote_dma_wrap_cnt = 0u;
    image_remote_dma_last_wrap_cnt = 0u;
    image_remote_dma_restart_req = 0u;
    image_remote_link_reset_parser();

    bsp_aux_link_set_rx_event_cb(image_remote_link_on_rx_event);
    bsp_aux_link_set_rx_byte_cb(NULL);
    bsp_aux_link_set_error_cb(image_remote_link_on_uart_error);

    if (bsp_aux_link_rx_has_dma() == 0u)
    {
        return;
    }
    if (bsp_aux_link_rx_to_idle_dma_start(image_remote_dma_rx_buf, (uint16_t)IMAGE_REMOTE_DMA_RX_BUF_SIZE) != 0)
    {
        return;
    }

    image_remote_dma_active = 1u;
}

void image_remote_link_stop(void)
{
    image_remote_dma_active = 0u;
    image_remote_dma_restart_req = 0u;
    image_remote_dma_pos = 0u;
    image_remote_dma_last_pos = 0u;
    image_remote_dma_wrap_cnt = 0u;
    image_remote_dma_last_wrap_cnt = 0u;
    image_remote_link_reset_parser();

    bsp_aux_link_set_rx_event_cb(NULL);
    bsp_aux_link_set_rx_byte_cb(NULL);
    bsp_aux_link_set_error_cb(NULL);
    bsp_aux_link_rx_it_stop();
}

void image_remote_link_poll(void)
{
    if (bsp_aux_link_get_baudrate() != IMAGE_REMOTE_LINK_BAUD)
    {
        return;
    }
    if (image_remote_dma_restart_req != 0u)
    {
        image_remote_dma_restart_req = 0u;
        image_remote_link_start();
        return;
    }
    if (image_remote_dma_active == 0u)
    {
        return;
    }

    const uint32_t wrap_cnt = image_remote_dma_wrap_cnt;
    const uint16_t pos = image_remote_dma_pos;

    while (image_remote_dma_last_wrap_cnt != wrap_cnt)
    {
        image_remote_link_process_to((uint16_t)IMAGE_REMOTE_DMA_RX_BUF_SIZE);
        image_remote_dma_last_wrap_cnt++;
    }
    image_remote_link_process_to(pos);
    image_remote_dma_last_wrap_cnt = wrap_cnt;
}

void image_remote_link_on_rx_event(uint16_t size, bsp_aux_link_rx_event_e evt)
{
    if (bsp_aux_link_get_baudrate() != IMAGE_REMOTE_LINK_BAUD || image_remote_dma_active == 0u)
    {
        return;
    }

    if (evt == BSP_AUX_LINK_RXEVENT_IDLE && size >= (uint16_t)IMAGE_REMOTE_DMA_RX_BUF_SIZE)
    {
        return;
    }

    if (evt == BSP_AUX_LINK_RXEVENT_TC)
    {
        image_remote_dma_wrap_cnt++;
    }

    image_remote_dma_pos = (size >= (uint16_t)IMAGE_REMOTE_DMA_RX_BUF_SIZE) ? 0u : size;
}

uint8_t image_remote_link_on_uart_error(void)
{
    image_remote_link_reset_parser();
    if (image_remote_dma_active == 0u)
    {
        return 0u;
    }

    image_remote_restart_cnt++;
    image_remote_dma_restart_req = 1u;
    return 1u;
}

static void image_remote_store(const image_remote_state_t *state)
{
    if (state == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    image_remote_state = *state;
    taskEXIT_CRITICAL();
}

static void image_remote_link_process_to(uint16_t pos)
{
    const uint16_t size = (uint16_t)IMAGE_REMOTE_DMA_RX_BUF_SIZE;
    uint16_t last = image_remote_dma_last_pos;

    if (pos > size)
    {
        pos = (uint16_t)(pos % size);
    }
    if (last >= size)
    {
        last = 0u;
    }

    if (last <= pos)
    {
        for (uint16_t i = last; i < pos; i++)
        {
            image_remote_link_feed_byte(image_remote_dma_rx_buf[i]);
        }
    }
    else
    {
        for (uint16_t i = last; i < size; i++)
        {
            image_remote_link_feed_byte(image_remote_dma_rx_buf[i]);
        }
        for (uint16_t i = 0u; i < pos; i++)
        {
            image_remote_link_feed_byte(image_remote_dma_rx_buf[i]);
        }
    }

    image_remote_dma_last_pos = (pos == size) ? 0u : pos;
}

static void image_remote_link_reset_parser(void)
{
    image_remote_parse_mode = IMAGE_REMOTE_PARSE_IDLE;
    image_remote_rm_pos = 0u;
    image_remote_rm_expected = 0u;
    image_remote_vt13_pos = 0u;
}

static void image_remote_link_feed_byte(uint8_t b)
{
retry_parse:
    switch (image_remote_parse_mode)
    {
    case IMAGE_REMOTE_PARSE_IDLE:
        if (b == IMAGE_REMOTE_FRAME_SOF)
        {
            image_remote_parse_mode = IMAGE_REMOTE_PARSE_RM;
            image_remote_rm_pos = 0u;
            image_remote_rm_expected = 0u;
            image_remote_rm_buf[image_remote_rm_pos++] = b;
        }
        else if (b == 0xA9u)
        {
            image_remote_parse_mode = IMAGE_REMOTE_PARSE_VT13;
            image_remote_vt13_pos = 0u;
            image_remote_vt13_buf[image_remote_vt13_pos++] = b;
        }
        return;

    case IMAGE_REMOTE_PARSE_RM:
        if (image_remote_rm_pos >= (uint16_t)IMAGE_REMOTE_RM_FRAME_MAX_SIZE)
        {
            image_remote_parse_error_cnt++;
            image_remote_link_reset_parser();
            goto retry_parse;
        }

        image_remote_rm_buf[image_remote_rm_pos++] = b;

        if (image_remote_rm_pos == 5u)
        {
            if (!verify_CRC8_check_sum(image_remote_rm_buf, 5u))
            {
                image_remote_crc_error_cnt++;
                image_remote_link_reset_parser();
                goto retry_parse;
            }

            const uint16_t payload_len = (uint16_t)(image_remote_rm_buf[1] | ((uint16_t)image_remote_rm_buf[2] << 8));
            image_remote_rm_expected = (uint16_t)(payload_len + 9u);
            if (image_remote_rm_expected < 9u || image_remote_rm_expected > (uint16_t)IMAGE_REMOTE_RM_FRAME_MAX_SIZE)
            {
                image_remote_parse_error_cnt++;
                image_remote_link_reset_parser();
                goto retry_parse;
            }
        }

        if (image_remote_rm_expected != 0u && image_remote_rm_pos >= image_remote_rm_expected)
        {
            if (verify_CRC16_check_sum(image_remote_rm_buf, image_remote_rm_expected))
            {
                image_remote_link_handle_rm_frame(image_remote_rm_buf, image_remote_rm_expected);
            }
            else
            {
                image_remote_crc_error_cnt++;
            }
            image_remote_link_reset_parser();
        }
        return;

    case IMAGE_REMOTE_PARSE_VT13:
        if (image_remote_vt13_pos == 1u && b != 0x53u)
        {
            image_remote_parse_error_cnt++;
            image_remote_link_reset_parser();
            goto retry_parse;
        }
        if (image_remote_vt13_pos >= IMAGE_REMOTE_VT13_FRAME_SIZE)
        {
            image_remote_parse_error_cnt++;
            image_remote_link_reset_parser();
            goto retry_parse;
        }

        image_remote_vt13_buf[image_remote_vt13_pos++] = b;
        if (image_remote_vt13_pos >= IMAGE_REMOTE_VT13_FRAME_SIZE)
        {
            image_remote_link_handle_vt13_frame(image_remote_vt13_buf, IMAGE_REMOTE_VT13_FRAME_SIZE);
            image_remote_link_reset_parser();
        }
        return;

    default:
        image_remote_parse_error_cnt++;
        image_remote_link_reset_parser();
        goto retry_parse;
    }
}

static void image_remote_link_handle_rm_frame(const uint8_t *frame, uint16_t frame_len)
{
    if (frame == NULL || frame_len < 9u)
    {
        return;
    }

    const uint16_t payload_len = (uint16_t)(frame[1] | ((uint16_t)frame[2] << 8));
    if ((uint16_t)(payload_len + 9u) != frame_len || payload_len != (uint16_t)sizeof(image_remote_rc_packet_t))
    {
        image_remote_parse_error_cnt++;
        return;
    }

    const uint16_t cmd_id = (uint16_t)(frame[5] | ((uint16_t)frame[6] << 8));
    const uint8_t *payload = &frame[7];

    if (cmd_id == IMAGE_REMOTE_CMD_CUSTOM_CONTROLLER_RX || cmd_id == IMAGE_REMOTE_CMD_CUSTOM_CLIENT_RX)
    {
        image_remote_last_rx_tick_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        image_remote_frame_cnt++;
        image_remote_last_cmd_id = cmd_id;
        if (cmd_id == IMAGE_REMOTE_CMD_CUSTOM_CONTROLLER_RX)
        {
            image_remote_controller_frame_cnt++;
        }
        else
        {
            image_remote_client_frame_cnt++;
        }
        (void)image_remote_link_try_decode_custom_rc(payload);
    }
}

static void image_remote_link_handle_vt13_frame(const uint8_t *frame, uint16_t frame_len)
{
    if (frame == NULL || frame_len != IMAGE_REMOTE_VT13_FRAME_SIZE)
    {
        return;
    }
    if (frame[0] != 0xA9u || frame[1] != 0x53u)
    {
        image_remote_parse_error_cnt++;
        return;
    }
    if (!verify_CRC16_check_sum((uint8_t *)frame, frame_len))
    {
        image_remote_crc_error_cnt++;
        return;
    }

    int16_t ch_raw[5] = {0};
    ch_raw[0] = (int16_t)(((uint16_t)frame[2] | ((uint16_t)frame[3] << 8)) & 0x07FFu);
    ch_raw[1] = (int16_t)((((uint16_t)frame[3] >> 3) | ((uint16_t)frame[4] << 5)) & 0x07FFu);
    ch_raw[2] = (int16_t)((((uint16_t)frame[4] >> 6) | ((uint16_t)frame[5] << 2) | ((uint16_t)frame[6] << 10)) & 0x07FFu);
    ch_raw[3] = (int16_t)((((uint16_t)frame[6] >> 1) | ((uint16_t)frame[7] << 7)) & 0x07FFu);
    ch_raw[4] = (int16_t)((((uint16_t)frame[8] >> 1) | ((uint16_t)frame[9] << 7)) & 0x07FFu);
    const uint8_t vt13_pause = (uint8_t)((frame[7] >> 6) & 0x01u);
    const uint8_t vt13_btn_l = (uint8_t)((frame[7] >> 7) & 0x01u);
    const uint8_t vt13_btn_r = (uint8_t)(frame[8] & 0x01u);
    const uint16_t vt13_dial = (uint16_t)(((uint16_t)frame[8] >> 1) | ((uint16_t)frame[9] << 7));
    const uint8_t vt13_trigger = (uint8_t)((frame[9] >> 4) & 0x01u);
    const uint8_t vt13_mouse_l = (uint8_t)(frame[16] & 0x01u);
    const uint8_t vt13_mouse_r = (uint8_t)((frame[16] >> 1) & 0x01u);
    const uint8_t vt13_mouse_mid = (uint8_t)((frame[16] >> 2) & 0x01u);

    manual_input_state_t rc = {0};
    rc.rc.ch[0] = image_remote_link_scale_axis((int16_t)(ch_raw[0] - RC_CH_VALUE_OFFSET), IMAGE_REMOTE_RC_ABS_MAX_VT13);
    rc.rc.ch[1] = image_remote_link_scale_axis((int16_t)(ch_raw[1] - RC_CH_VALUE_OFFSET), IMAGE_REMOTE_RC_ABS_MAX_VT13);
    rc.rc.ch[2] = image_remote_link_scale_axis((int16_t)(ch_raw[2] - RC_CH_VALUE_OFFSET), IMAGE_REMOTE_RC_ABS_MAX_VT13);
    rc.rc.ch[3] = image_remote_link_scale_axis((int16_t)(ch_raw[3] - RC_CH_VALUE_OFFSET), IMAGE_REMOTE_RC_ABS_MAX_VT13);
    rc.rc.ch[4] = image_remote_link_scale_axis((int16_t)(ch_raw[4] - RC_CH_VALUE_OFFSET), IMAGE_REMOTE_RC_ABS_MAX_VT13);

    rc.rc.s[0] = (char)image_remote_link_map_vt13_switch1((uint8_t)((frame[7] >> 4) & 0x03u));
    rc.rc.s[1] = (char)image_remote_link_map_vt13_switch2(vt13_pause, vt13_btn_l, vt13_btn_r);

    rc.mouse.x = (int16_t)((uint16_t)frame[10] | ((uint16_t)frame[11] << 8));
    rc.mouse.y = (int16_t)((uint16_t)frame[12] | ((uint16_t)frame[13] << 8));
    rc.mouse.z = (int16_t)((uint16_t)frame[14] | ((uint16_t)frame[15] << 8));
    rc.mouse.press_l = vt13_mouse_l;
    rc.mouse.press_r = vt13_mouse_r;
    rc.key.v = (uint16_t)(frame[17] | ((uint16_t)frame[18] << 8));

    image_remote_last_rx_tick_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    image_remote_frame_cnt++;
    image_remote_vt13_frame_cnt++;
    image_remote_last_cmd_id = 0u;
    image_remote_last_range_mode = SDLOG_MANUAL_INPUT_RANGE_RAW_11BIT;
    remote_control_log_raw_source(MANUAL_INPUT_SRC_IMAGE,
                                  SDLOG_MANUAL_INPUT_PROTO_IMAGE_VT13,
                                  SDLOG_MANUAL_INPUT_RANGE_RAW_11BIT,
                                  5u,
                                  ch_raw,
                                  NULL,
                                  &rc);
    image_remote_state_t state = {
        .valid = 1u,
        .proto = SDLOG_MANUAL_INPUT_PROTO_IMAGE_VT13,
        .range_mode = SDLOG_MANUAL_INPUT_RANGE_RAW_11BIT,
        .raw_ch = { ch_raw[0], ch_raw[1], ch_raw[2], ch_raw[3], ch_raw[4] },
        .ch = { rc.rc.ch[0], rc.rc.ch[1], rc.rc.ch[2], rc.rc.ch[3], rc.rc.ch[4] },
        .s = { rc.rc.s[0], rc.rc.s[1] },
        .mouse_x = rc.mouse.x,
        .mouse_y = rc.mouse.y,
        .mouse_z = rc.mouse.z,
        .mouse_l = vt13_mouse_l,
        .mouse_r = vt13_mouse_r,
        .mouse_mid = vt13_mouse_mid,
        .pause = vt13_pause,
        .btn_l = vt13_btn_l,
        .btn_r = vt13_btn_r,
        .trigger = vt13_trigger,
        .dial = vt13_dial,
        .key_value = rc.key.v,
        .key_w = IMAGE_REMOTE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_W),
        .key_s = IMAGE_REMOTE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_S),
        .key_a = IMAGE_REMOTE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_A),
        .key_d = IMAGE_REMOTE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_D),
        .key_shift = IMAGE_REMOTE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_SHIFT),
        .key_ctrl = IMAGE_REMOTE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_CTRL),
        .key_q = IMAGE_REMOTE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_Q),
        .key_e = IMAGE_REMOTE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_E),
        .key_r = IMAGE_REMOTE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_R),
        .key_f = IMAGE_REMOTE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_F),
        .key_g = IMAGE_REMOTE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_G),
        .key_z = IMAGE_REMOTE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_Z),
        .key_x = IMAGE_REMOTE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_X),
        .key_c = IMAGE_REMOTE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_C),
        .key_v = IMAGE_REMOTE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_V),
        .key_b = IMAGE_REMOTE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_B),
        .last_rx_tick_ms = image_remote_last_rx_tick_ms,
    };
    image_remote_store(&state);
    manual_input_update_source(MANUAL_INPUT_SRC_IMAGE, &rc);
}

static bool_t image_remote_link_try_decode_custom_rc(const uint8_t *data)
{
    if (data == NULL)
    {
        return 0;
    }

    image_remote_rc_packet_t pkt;
    memcpy(&pkt, data, sizeof(pkt));
    if (pkt.magic0 != (uint8_t)IMAGE_REMOTE_RC_MAGIC0 ||
        pkt.magic1 != (uint8_t)IMAGE_REMOTE_RC_MAGIC1 ||
        pkt.version != IMAGE_REMOTE_RC_VERSION)
    {
        return 0;
    }

    manual_input_state_t rc = {0};
    int16_t raw_ch[5] = {0};
    uint8_t raw_sw[2] = {0};
    uint8_t log_range_mode = 0u;
    for (uint8_t i = 0u; i < 5u; i++)
    {
        raw_ch[i] = pkt.ch[i];
        if (pkt.range_mode == IMAGE_REMOTE_RC_RANGE_DBUS)
        {
            log_range_mode = SDLOG_MANUAL_INPUT_RANGE_CENTERED_660;
            rc.rc.ch[i] = image_remote_link_scale_axis(pkt.ch[i], IMAGE_REMOTE_RC_ABS_MAX_DBUS);
        }
        else if (pkt.range_mode == IMAGE_REMOTE_RC_RANGE_VT13)
        {
            log_range_mode = SDLOG_MANUAL_INPUT_RANGE_CENTERED_1024;
            rc.rc.ch[i] = image_remote_link_scale_axis(pkt.ch[i], IMAGE_REMOTE_RC_ABS_MAX_VT13);
        }
        else
        {
            image_remote_parse_error_cnt++;
            return 0;
        }
    }

    raw_sw[0] = pkt.sw[0];
    raw_sw[1] = pkt.sw[1];
    rc.rc.s[0] = (char)image_remote_link_sanitize_switch(pkt.sw[0]);
    rc.rc.s[1] = (char)image_remote_link_sanitize_switch(pkt.sw[1]);
    rc.mouse.x = pkt.mouse_x;
    rc.mouse.y = pkt.mouse_y;
    rc.mouse.z = pkt.mouse_z;
    rc.mouse.press_l = ((pkt.mouse_btns & IMAGE_REMOTE_RC_BTN_LEFT) != 0u) ? 1u : 0u;
    rc.mouse.press_r = ((pkt.mouse_btns & IMAGE_REMOTE_RC_BTN_RIGHT) != 0u) ? 1u : 0u;
    rc.key.v = pkt.key_value;

    remote_control_log_raw_source(MANUAL_INPUT_SRC_IMAGE,
                                  SDLOG_MANUAL_INPUT_PROTO_IMAGE_CUSTOM,
                                  log_range_mode,
                                  5u,
                                  raw_ch,
                                  raw_sw,
                                  &rc);
    image_remote_last_range_mode = log_range_mode;
    image_remote_state_t state = {
        .valid = 1u,
        .proto = SDLOG_MANUAL_INPUT_PROTO_IMAGE_CUSTOM,
        .range_mode = log_range_mode,
        .raw_ch = { raw_ch[0], raw_ch[1], raw_ch[2], raw_ch[3], raw_ch[4] },
        .ch = { rc.rc.ch[0], rc.rc.ch[1], rc.rc.ch[2], rc.rc.ch[3], rc.rc.ch[4] },
        .s = { rc.rc.s[0], rc.rc.s[1] },
        .mouse_x = rc.mouse.x,
        .mouse_y = rc.mouse.y,
        .mouse_z = rc.mouse.z,
        .mouse_l = rc.mouse.press_l,
        .mouse_r = rc.mouse.press_r,
        .mouse_mid = 0u,
        .pause = 0u,
        .btn_l = 0u,
        .btn_r = 0u,
        .trigger = 0u,
        .dial = rc.rc.ch[4],
        .key_value = rc.key.v,
        .key_w = IMAGE_REMOTE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_W),
        .key_s = IMAGE_REMOTE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_S),
        .key_a = IMAGE_REMOTE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_A),
        .key_d = IMAGE_REMOTE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_D),
        .key_shift = IMAGE_REMOTE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_SHIFT),
        .key_ctrl = IMAGE_REMOTE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_CTRL),
        .key_q = IMAGE_REMOTE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_Q),
        .key_e = IMAGE_REMOTE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_E),
        .key_r = IMAGE_REMOTE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_R),
        .key_f = IMAGE_REMOTE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_F),
        .key_g = IMAGE_REMOTE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_G),
        .key_z = IMAGE_REMOTE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_Z),
        .key_x = IMAGE_REMOTE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_X),
        .key_c = IMAGE_REMOTE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_C),
        .key_v = IMAGE_REMOTE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_V),
        .key_b = IMAGE_REMOTE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_B),
        .last_rx_tick_ms = image_remote_last_rx_tick_ms,
    };
    image_remote_store(&state);
    manual_input_update_source(MANUAL_INPUT_SRC_IMAGE, &rc);
    return 1;
}

static int16_t image_remote_link_scale_axis(int16_t raw, int16_t raw_abs_max)
{
    if (raw_abs_max <= 0)
    {
        return 0;
    }
    return rc_scale_axis_by_abs(raw, raw_abs_max, (int16_t)RC_CH_VALUE_ABS_MAX);
}

static uint8_t image_remote_link_sanitize_switch(uint8_t value)
{
    if (value == RC_SW_UP || value == RC_SW_MID || value == RC_SW_DOWN)
    {
        return value;
    }
    return RC_SW_DOWN;
}

static uint8_t image_remote_link_map_vt13_switch1(uint8_t value)
{
    if (value == g_config.manual_input.vt13.switch1_safe_value)
    {
        return input_switch_pos_to_raw(g_config.manual_input.semantics.gimbal_safe_pos);
    }
    if (value == g_config.manual_input.vt13.switch1_normal_value || value == 3u)
    {
        return input_switch_pos_to_raw(g_config.manual_input.semantics.chassis_follow_pos);
    }
    if (value == g_config.manual_input.vt13.switch1_spin_value)
    {
        return input_switch_pos_to_raw(g_config.manual_input.semantics.chassis_spin_pos);
    }
    return input_switch_pos_to_raw(g_config.manual_input.semantics.chassis_spin_pos);
}

static uint8_t image_remote_link_map_vt13_switch2(uint8_t stop, uint8_t left, uint8_t right)
{
    if (stop != 0u)
    {
        return input_switch_pos_to_raw(g_config.manual_input.vt13.switch2_pause_pos);
    }
    if (left != 0u)
    {
        return input_switch_pos_to_raw(g_config.manual_input.vt13.switch2_btn_l_pos);
    }
    if (right != 0u)
    {
        return input_switch_pos_to_raw(g_config.manual_input.vt13.switch2_btn_r_pos);
    }
    return input_switch_pos_to_raw(g_config.manual_input.vt13.switch2_btn_r_pos);
}
