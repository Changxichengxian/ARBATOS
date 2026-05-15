/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 Xie Yuhan <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef ROBOT_MSG_H
#define ROBOT_MSG_H

#include <stdint.h>

#include "struct_typedef.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    MSG_SOURCE_NONE = 0,
    MSG_SOURCE_MANUAL,
    MSG_SOURCE_HOST,
    MSG_SOURCE_VISION,
    MSG_SOURCE_REFEREE,
    MSG_SOURCE_AUTONOMY,
    MSG_SOURCE_TEST,
    MSG_SOURCE_SAFETY,
} msg_source_e;

typedef enum
{
    MSG_HEALTH_UNKNOWN = 0,
    MSG_HEALTH_OK,
    MSG_HEALTH_DEGRADED,
    MSG_HEALTH_FAULT,
} msg_health_e;

typedef struct
{
    uint8_t valid;
    uint8_t source; // msg_source_e
    uint16_t size;
    uint32_t seq;
    uint32_t tick_ms;
} msg_header_t;

static inline void msg_header_init(msg_header_t *header,
                                   msg_source_e source,
                                   uint16_t size,
                                   uint32_t tick_ms,
                                   uint32_t seq)
{
    if (header == 0)
    {
        return;
    }

    header->valid = 1u;
    header->source = (uint8_t)source;
    header->size = size;
    header->seq = seq;
    header->tick_ms = tick_ms;
}

typedef struct
{
    uint8_t valid;
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperature;
    int16_t last_ecd;
} motor_measure_state_t;

#ifdef __cplusplus
}
#endif

#endif
