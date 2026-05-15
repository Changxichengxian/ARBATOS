/*
 * SPDX-FileCopyrightText: 2026 陈舜 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈舜 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "state_store.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

typedef struct
{
    uint8_t valid;
    uint16_t size;
    uint32_t seq;
    uint8_t payload[STATE_STORE_MAX_BYTES];
} state_slot_t;

static state_slot_t s_state_slots[STATE_COUNT];

static uint8_t state_is_valid(state_id_e id)
{
    return ((uint32_t)id < (uint32_t)STATE_COUNT) ? 1u : 0u;
}

static void state_store_lock(void)
{
    vTaskSuspendAll();
}

static void state_store_unlock(void)
{
    (void)xTaskResumeAll();
}

uint8_t state_store_write(state_id_e id, const void *payload, uint16_t size)
{
    if (state_is_valid(id) == 0u || payload == NULL || size == 0u || size > STATE_STORE_MAX_BYTES)
    {
        return 0u;
    }

    state_store_lock();
    memcpy(s_state_slots[id].payload, payload, size);
    s_state_slots[id].size = size;
    s_state_slots[id].seq++;
    s_state_slots[id].valid = 1u;
    state_store_unlock();
    return 1u;
}

uint8_t state_store_read(state_id_e id, void *out, uint16_t size)
{
    if (state_is_valid(id) == 0u || out == NULL || size == 0u)
    {
        return 0u;
    }

    uint8_t ok = 0u;
    state_store_lock();
    if (s_state_slots[id].valid != 0u && s_state_slots[id].size == size)
    {
        memcpy(out, s_state_slots[id].payload, size);
        ok = 1u;
    }
    state_store_unlock();
    return ok;
}

state_info_t state_store_info(state_id_e id)
{
    state_info_t info = {0};
    if (state_is_valid(id) == 0u)
    {
        return info;
    }

    state_store_lock();
    info.valid = s_state_slots[id].valid;
    info.size = s_state_slots[id].size;
    info.seq = s_state_slots[id].seq;
    state_store_unlock();
    return info;
}
