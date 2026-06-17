/*
 * SPDX-FileCopyrightText: 2026 陈卓 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef IMAGE_REMOTE_LINK_H
#define IMAGE_REMOTE_LINK_H

#include <stdbool.h>
#include <stdint.h>

#include "types.h"
#include "SdLog.h"
#include "BspUsart.h"

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
} ImageRemoteState;

bool ImageRemoteGetState(ImageRemoteState *out);
bool ImageRemoteAutoAimRequested(void);
bool ImageRemoteAuxFireRequested(void);
void ImageRemoteLinkGetStats(sdlog_image_link_stats_t *out);

void ImageRemoteLinkStart(void);
void ImageRemoteLinkStop(void);
void ImageRemoteLinkPoll(void);
void ImageRemoteLinkOnRxEvent(uint16_t size, BspAuxLinkRxEvent evt);
uint8_t ImageRemoteLinkOnUartError(void);

#endif
