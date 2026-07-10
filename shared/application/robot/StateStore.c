/*
 * SPDX-FileCopyrightText: 2026 陈舜 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "StateStore.h"

#include <string.h>

#include "BspTime.h"
#include "cmsis_compiler.h"
#include "FreeRTOS.h"
#include "task.h"

#define STATE_STORE_BANK_COUNT 2u

typedef struct
{
    uint8_t valid;
    uint16_t size;
    uint32_t seq;
    uint32_t write_tick_ms;
    uint8_t payload[STATE_STORE_MAX_BYTES];
} state_bank_t;

typedef struct
{
    uint32_t publish_seq;
    uint32_t write_drop_count;
    uint16_t readers[STATE_STORE_BANK_COUNT];
    uint8_t active_index;
    uint8_t writer_busy;
    state_bank_t banks[STATE_STORE_BANK_COUNT];
} state_slot_t;

static state_slot_t s_state_slots[STATE_COUNT];

static uint8_t state_is_valid(state_id_e id)
{
    return ((uint32_t)id < (uint32_t)STATE_COUNT) ? 1u : 0u;
}

static void StateStoreLock(void)
{
    taskENTER_CRITICAL();
}

static void StateStoreUnlock(void)
{
    taskEXIT_CRITICAL();
}

uint8_t StateStoreWrite(state_id_e id, const void *payload, uint16_t size)
{
    if (state_is_valid(id) == 0u || payload == NULL || size == 0u || size > STATE_STORE_MAX_BYTES)
    {
        return 0u;
    }

    state_slot_t *slot = &s_state_slots[id];
    state_bank_t *bank;
    uint32_t next_seq;
    uint8_t write_index;
    const uint32_t write_tick_ms = BspTimeGetTickMs();

    StateStoreLock();
    write_index = (uint8_t)(slot->active_index ^ 1u);
    if (slot->writer_busy != 0u || slot->readers[write_index] != 0u)
    {
        slot->write_drop_count++;
        StateStoreUnlock();
        return 0u;
    }
    slot->writer_busy = 1u;
    next_seq = slot->publish_seq + 1u;
    StateStoreUnlock();

    bank = &slot->banks[write_index];

    memcpy(bank->payload, payload, size);
    bank->size = size;
    bank->seq = next_seq;
    bank->write_tick_ms = write_tick_ms;
    bank->valid = 1u;

    __DMB();
    StateStoreLock();
    slot->active_index = write_index;
    slot->publish_seq = next_seq;
    slot->writer_busy = 0u;
    StateStoreUnlock();
    return 1u;
}

uint8_t StateStoreRead(state_id_e id, void *out, uint16_t size)
{
    return StateStoreReadSnapshot(id, out, size, NULL);
}

uint8_t StateStoreReadSnapshot(state_id_e id, void *out, uint16_t size, state_info_t *info)
{
    if (state_is_valid(id) == 0u || out == NULL || size == 0u)
    {
        return 0u;
    }

    state_slot_t *slot = &s_state_slots[id];
    state_bank_t *bank;
    state_info_t candidate = {0};
    uint8_t read_index;

    StateStoreLock();
    read_index = slot->active_index;
    bank = &slot->banks[read_index];
    candidate.valid = bank->valid;
    candidate.size = bank->size;
    candidate.seq = bank->seq;
    candidate.write_tick_ms = bank->write_tick_ms;
    candidate.write_drop_count = slot->write_drop_count;
    if (candidate.valid == 0u || candidate.size != size || candidate.seq != slot->publish_seq)
    {
        StateStoreUnlock();
        if (info != NULL)
        {
            *info = candidate;
        }
        return 0u;
    }
    slot->readers[read_index]++;
    StateStoreUnlock();

    memcpy(out, bank->payload, size);

    __DMB();
    StateStoreLock();
    slot->readers[read_index]--;
    StateStoreUnlock();

    candidate.age_ms = BspTimeGetTickMs() - candidate.write_tick_ms;
    if (info != NULL)
    {
        *info = candidate;
    }
    return 1u;
}

state_info_t StateStoreInfo(state_id_e id)
{
    state_info_t info = {0};
    if (state_is_valid(id) == 0u)
    {
        return info;
    }

    state_slot_t *slot = &s_state_slots[id];
    StateStoreLock();
    const state_bank_t *bank = &slot->banks[slot->active_index];
    info.valid = bank->valid;
    info.size = bank->size;
    info.seq = bank->seq;
    info.write_tick_ms = bank->write_tick_ms;
    info.write_drop_count = slot->write_drop_count;
    StateStoreUnlock();
    if (info.valid != 0u)
    {
        info.age_ms = BspTimeGetTickMs() - info.write_tick_ms;
    }
    return info;
}
