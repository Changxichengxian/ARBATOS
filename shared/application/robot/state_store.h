/*
 * SPDX-FileCopyrightText: 2026 陈舜 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈舜 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef STATE_STORE_H
#define STATE_STORE_H

#include "types.h"

typedef enum
{
    STATE_GIMBAL = 0,
    STATE_CHASSIS,
    STATE_SHOOT,
    STATE_WHEELLEG_CMD,
    STATE_WHEELLEG_STATE,
    STATE_WHEELLEG_STATUS,
    STATE_WHEELLEG_DEBUG,
    STATE_ARM_STATUS,
    STATE_COUNT,
} state_id_e;

#define STATE_STORE_MAX_BYTES 512u

typedef struct
{
    uint8_t valid;
    uint16_t size;
    uint32_t seq;
} state_info_t;

uint8_t state_store_write(state_id_e id, const void *payload, uint16_t size);
uint8_t state_store_read(state_id_e id, void *out, uint16_t size);
state_info_t state_store_info(state_id_e id);

#endif
