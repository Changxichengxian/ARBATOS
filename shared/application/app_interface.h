/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 Xie Yuhan <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef APP_INTERFACE_H
#define APP_INTERFACE_H

#include <stdint.h>

#include "struct_typedef.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    APP_IF_SOURCE_NONE = 0,
    APP_IF_SOURCE_MANUAL,
    APP_IF_SOURCE_HOST,
    APP_IF_SOURCE_VISION,
    APP_IF_SOURCE_REFEREE,
    APP_IF_SOURCE_AUTONOMY,
    APP_IF_SOURCE_TEST,
    APP_IF_SOURCE_SAFETY,
} app_interface_source_e;

typedef enum
{
    APP_IF_HEALTH_UNKNOWN = 0,
    APP_IF_HEALTH_OK,
    APP_IF_HEALTH_DEGRADED,
    APP_IF_HEALTH_FAULT,
} app_interface_health_e;

typedef struct
{
    uint8_t valid;
    uint8_t source; // app_interface_source_e
    uint16_t size;
    uint32_t seq;
    uint32_t tick_ms;
} app_interface_header_t;

static inline void app_interface_header_init(app_interface_header_t *header,
                                             app_interface_source_e source,
                                             uint16_t size,
                                             uint32_t tick_ms)
{
    if (header == 0)
    {
        return;
    }

    header->seq = (header->valid != 0u) ? (header->seq + 1u) : 1u;
    header->valid = 1u;
    header->source = (uint8_t)source;
    header->size = size;
    header->tick_ms = tick_ms;
}

#ifdef __cplusplus
}
#endif

#endif
