/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */


#ifndef USB_TASK_H
#define USB_TASK_H

#include <stdbool.h>
#include "struct_typedef.h"
#include "sdlog.h"

// Frame sent to vision (legacy, kept for compatibility; not used for TX now)
typedef struct __attribute__((packed)) GimbalToVision
{
    uint8_t head[2];
    uint8_t mode;        // 0: idle, 1: auto-aim, 2: small buff, 3: large buff
    float   q[4];        // wxyz
    float   yaw;
    float   yaw_vel;
    float   pitch;
    float   pitch_vel;
    float   bullet_speed;
    uint16_t bullet_count;
    uint16_t crc16;
} GimbalToVision;

// Vision -> gimbal control frame (29 bytes: 2+1+6*4+2)
typedef struct __attribute__((packed)) VisionToGimbal
{
    uint8_t head[2];
    uint8_t mode;      // 0: idle, 1: control no-fire, 2: control and fire
    float   yaw;
    float   yaw_vel;
    float   yaw_acc;
    float   pitch;
    float   pitch_vel;
    float   pitch_acc;
    uint16_t crc16;
} VisionToGimbal;

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
} uart1_image_remote_state_t;

void usb_task(void const * argument);

// Get latest vision command; copy to out if available and return true
bool vision_take_latest(VisionToGimbal *out);
bool uart1_image_remote_get_state(uart1_image_remote_state_t *out);
bool uart1_image_auto_aim_requested(void);
bool uart1_image_aux_fire_requested(void);

// Called by usbd_cdc_if.c when data is received
void vision_usb_rx_callback(uint8_t *buf, uint32_t len);
void uart1_image_link_get_stats(sdlog_image_link_stats_t *out);

#endif
