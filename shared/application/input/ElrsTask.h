/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */


#ifndef ELRS_TASK_H
#define ELRS_TASK_H

#include <stdint.h>

#include "BspUsart.h"

// ELRS/CRSF baud rate on the aux link port.
#define ELRS_LINK_BAUD 420000u
#define ELRS_LINK_STATS_TIMEOUT_MS 250u

/*
 * 产品安全策略：ELRS 必须持续提供 0x14 Link Statistics 才能成为控制来源。
 * 只发送 0x16 通道帧的泛 CRSF 设备不受支持；这不是 CRSF 协议的强制要求。
 */

#define ELRS_LINK_STATE_WAIT_STATS 0u
#define ELRS_LINK_STATE_WAIT_RC    1u
#define ELRS_LINK_STATE_UP         2u
#define ELRS_LINK_STATE_DOWN       3u

typedef struct
{
    uint32_t last_rc_tick_ms;
    uint32_t last_stats_tick_ms;
    uint32_t valid_frame_count;
    uint32_t rc_frame_count;
    uint32_t rc_publish_count;
    uint32_t link_stats_count;
    uint32_t crc_error_count;
    uint32_t length_error_count;
    uint32_t sync_drop_count;
    uint32_t channel_reject_count;
    uint32_t health_reject_count;
    uint32_t link_down_count;
    uint32_t stats_timeout_count;
    uint32_t restart_count;
    uint32_t overflow_count;
    uint8_t uplink_rssi_1;
    uint8_t uplink_rssi_2;
    uint8_t uplink_lq;
    int8_t uplink_snr;
    uint8_t state;
    uint8_t port_active;
    uint8_t reserved[2];
} ElrsLinkStats;

// FreeRTOS task entry (created in freertos.c).
void ElrsLinkTask(void const *argument);

// Start/stop ELRS RX on the aux link port.
void ElrsLinkRxStart(void);
void ElrsLinkStop(void);
void ElrsLinkGetStats(ElrsLinkStats *out);

// Callbacks from the aux link ISR context.
void ElrsLinkOnRxEvent(uint16_t size, BspAuxLinkRxEvent evt);
void ElrsLinkOnItByte(uint8_t b);
bool_t ElrsLinkOnUartError(void);

#endif
