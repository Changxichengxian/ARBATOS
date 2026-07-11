/*
 * SPDX-FileCopyrightText: 2026 陈舜 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef STATE_STORE_H
#define STATE_STORE_H

#include "Types.h"

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
    STATE_IMU,
    STATE_COUNT,
} state_id_e;

#define STATE_STORE_GIMBAL_BYTES          440u
#define STATE_STORE_CHASSIS_BYTES         276u
#define STATE_STORE_SHOOT_BYTES           452u
#define STATE_STORE_WHEELLEG_CMD_BYTES     32u
#define STATE_STORE_WHEELLEG_STATE_BYTES  156u
#define STATE_STORE_WHEELLEG_STATUS_BYTES 160u
#define STATE_STORE_WHEELLEG_DEBUG_BYTES  184u
#define STATE_STORE_ARM_STATUS_BYTES      276u
#define STATE_STORE_IMU_BYTES              84u
#define STATE_STORE_MAX_BYTES             STATE_STORE_SHOOT_BYTES

typedef struct
{
    uint8_t valid;
    uint16_t size;
    uint32_t seq;
    uint32_t write_tick_ms;
    uint32_t age_ms;
    uint32_t write_drop_count;
} state_info_t;

uint8_t StateStoreWrite(state_id_e id, const void *payload, uint16_t size);
uint8_t StateStoreRead(state_id_e id, void *out, uint16_t size);
uint8_t StateStoreReadSnapshot(state_id_e id, void *out, uint16_t size, state_info_t *info);
state_info_t StateStoreInfo(state_id_e id);

#endif
