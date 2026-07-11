/*
 * SPDX-FileCopyrightText: 2026 陈舜 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "RobotConfig.h"
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

#if ROBOT_TASK_BUILD_SINGLE_GIMBAL || ROBOT_TASK_BUILD_DUAL_YAW_GIMBAL
#define STATE_STORE_GIMBAL_CAPACITY STATE_STORE_GIMBAL_BYTES
#else
#define STATE_STORE_GIMBAL_CAPACITY 0u
#endif

#if ROBOT_TASK_BUILD_CLASSIC_CHASSIS
#define STATE_STORE_CHASSIS_CAPACITY STATE_STORE_CHASSIS_BYTES
#else
#define STATE_STORE_CHASSIS_CAPACITY 0u
#endif

#if ROBOT_TASK_BUILD_SHOOT_RM
#define STATE_STORE_SHOOT_CAPACITY STATE_STORE_SHOOT_BYTES
#else
#define STATE_STORE_SHOOT_CAPACITY 0u
#endif

#if ROBOT_TASK_BUILD_WHEELLEG_MIT
#define STATE_STORE_WHEELLEG_CMD_CAPACITY STATE_STORE_WHEELLEG_CMD_BYTES
#define STATE_STORE_WHEELLEG_STATE_CAPACITY STATE_STORE_WHEELLEG_STATE_BYTES
#define STATE_STORE_WHEELLEG_STATUS_CAPACITY STATE_STORE_WHEELLEG_STATUS_BYTES
#define STATE_STORE_WHEELLEG_DEBUG_CAPACITY STATE_STORE_WHEELLEG_DEBUG_BYTES
#else
#define STATE_STORE_WHEELLEG_CMD_CAPACITY 0u
#define STATE_STORE_WHEELLEG_STATE_CAPACITY 0u
#define STATE_STORE_WHEELLEG_STATUS_CAPACITY 0u
#define STATE_STORE_WHEELLEG_DEBUG_CAPACITY 0u
#endif

#if ROBOT_TASK_BUILD_ARM
#define STATE_STORE_ARM_STATUS_CAPACITY STATE_STORE_ARM_STATUS_BYTES
#else
#define STATE_STORE_ARM_STATUS_CAPACITY 0u
#endif

#if ROBOT_TASK_BUILD_IMU
#define STATE_STORE_IMU_CAPACITY STATE_STORE_IMU_BYTES
#else
#define STATE_STORE_IMU_CAPACITY 0u
#endif

#define STATE_STORE_GIMBAL_OFFSET 0u
#define STATE_STORE_CHASSIS_OFFSET \
    (STATE_STORE_GIMBAL_OFFSET + STATE_STORE_GIMBAL_CAPACITY)
#define STATE_STORE_SHOOT_OFFSET \
    (STATE_STORE_CHASSIS_OFFSET + STATE_STORE_CHASSIS_CAPACITY)
#define STATE_STORE_WHEELLEG_CMD_OFFSET \
    (STATE_STORE_SHOOT_OFFSET + STATE_STORE_SHOOT_CAPACITY)
#define STATE_STORE_WHEELLEG_STATE_OFFSET \
    (STATE_STORE_WHEELLEG_CMD_OFFSET + STATE_STORE_WHEELLEG_CMD_CAPACITY)
#define STATE_STORE_WHEELLEG_STATUS_OFFSET \
    (STATE_STORE_WHEELLEG_STATE_OFFSET + STATE_STORE_WHEELLEG_STATE_CAPACITY)
#define STATE_STORE_WHEELLEG_DEBUG_OFFSET \
    (STATE_STORE_WHEELLEG_STATUS_OFFSET + STATE_STORE_WHEELLEG_STATUS_CAPACITY)
#define STATE_STORE_ARM_STATUS_OFFSET \
    (STATE_STORE_WHEELLEG_DEBUG_OFFSET + STATE_STORE_WHEELLEG_DEBUG_CAPACITY)
#define STATE_STORE_IMU_OFFSET \
    (STATE_STORE_ARM_STATUS_OFFSET + STATE_STORE_ARM_STATUS_CAPACITY)
#define STATE_STORE_PAYLOAD_BYTES \
    (STATE_STORE_IMU_OFFSET + STATE_STORE_IMU_CAPACITY)
#define STATE_STORE_STORAGE_BYTES \
    ((STATE_STORE_PAYLOAD_BYTES != 0u) ? STATE_STORE_PAYLOAD_BYTES : 1u)

/* 未编译的控制模块容量为 0；每个 bank 只保留本目标真实会发布的状态。 */
typedef union
{
    uint32_t align;
    uint8_t bytes[STATE_STORE_STORAGE_BYTES];
} state_payload_bank_t;

static state_slot_t s_state_slots[STATE_COUNT];
static state_payload_bank_t s_state_payload[STATE_STORE_BANK_COUNT];

static const uint16_t s_state_capacity[STATE_COUNT] = {
    STATE_STORE_GIMBAL_CAPACITY,
    STATE_STORE_CHASSIS_CAPACITY,
    STATE_STORE_SHOOT_CAPACITY,
    STATE_STORE_WHEELLEG_CMD_CAPACITY,
    STATE_STORE_WHEELLEG_STATE_CAPACITY,
    STATE_STORE_WHEELLEG_STATUS_CAPACITY,
    STATE_STORE_WHEELLEG_DEBUG_CAPACITY,
    STATE_STORE_ARM_STATUS_CAPACITY,
    STATE_STORE_IMU_CAPACITY,
};

static const uint16_t s_state_offset[STATE_COUNT] = {
    STATE_STORE_GIMBAL_OFFSET,
    STATE_STORE_CHASSIS_OFFSET,
    STATE_STORE_SHOOT_OFFSET,
    STATE_STORE_WHEELLEG_CMD_OFFSET,
    STATE_STORE_WHEELLEG_STATE_OFFSET,
    STATE_STORE_WHEELLEG_STATUS_OFFSET,
    STATE_STORE_WHEELLEG_DEBUG_OFFSET,
    STATE_STORE_ARM_STATUS_OFFSET,
    STATE_STORE_IMU_OFFSET,
};

static uint8_t state_is_valid(state_id_e id)
{
    return ((uint32_t)id < (uint32_t)STATE_COUNT) ? 1u : 0u;
}

static uint8_t *StateStorePayload(state_id_e id, uint8_t bank)
{
    return &s_state_payload[bank].bytes[s_state_offset[id]];
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
    if (state_is_valid(id) == 0u || payload == NULL || size == 0u ||
        size > s_state_capacity[id])
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

    memcpy(StateStorePayload(id, write_index), payload, size);
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

    memcpy(out, StateStorePayload(id, read_index), size);

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
