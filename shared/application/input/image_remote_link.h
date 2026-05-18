/*
 * SPDX-FileCopyrightText: 2026 陈卓 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈卓 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef IMAGE_REMOTE_LINK_H
#define IMAGE_REMOTE_LINK_H

#include <stdbool.h>
#include <stdint.h>

#include "types.h"
#include "sdlog.h"
#include "bsp_usart.h"

#define IMAGE_REMOTE_LINK_BAUD 921600u

typedef struct
{
    uint8_t valid;
    uint8_t proto;
    uint8_t range_mode;
    int16_t raw_ch[5];
    int16_t ch[5];
    char s[2];
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint8_t mouse_l;
    uint8_t mouse_r;
    uint8_t mouse_mid;
    uint8_t pause;
    uint8_t btn_l;
    uint8_t btn_r;
    uint8_t trigger;
    uint16_t dial;
    uint16_t key_value;
    uint8_t key_w;
    uint8_t key_s;
    uint8_t key_a;
    uint8_t key_d;
    uint8_t key_shift;
    uint8_t key_ctrl;
    uint8_t key_q;
    uint8_t key_e;
    uint8_t key_r;
    uint8_t key_f;
    uint8_t key_g;
    uint8_t key_z;
    uint8_t key_x;
    uint8_t key_c;
    uint8_t key_v;
    uint8_t key_b;
    uint32_t last_rx_tick_ms;
} image_remote_state_t;

bool image_remote_get_state(image_remote_state_t *out);
bool image_remote_auto_aim_requested(void);
bool image_remote_aux_fire_requested(void);
void image_remote_link_get_stats(sdlog_image_link_stats_t *out);

void image_remote_link_start(void);
void image_remote_link_stop(void);
void image_remote_link_poll(void);
void image_remote_link_on_rx_event(uint16_t size, bsp_aux_link_rx_event_e evt);
uint8_t image_remote_link_on_uart_error(void);

#endif
