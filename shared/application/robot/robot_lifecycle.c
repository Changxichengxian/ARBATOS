/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include "robot_lifecycle.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

#include "config.h"
#include "control_input.h"
#include "detect_task.h"
#include "manual_input.h"

typedef struct
{
    uint8_t from_isr;
    UBaseType_t saved_mask;
} robot_lifecycle_critical_state_t;

static robot_lifecycle_snapshot_t g_robot_lifecycle;
static uint8_t g_robot_lifecycle_inited;

static robot_lifecycle_critical_state_t robot_lifecycle_enter_critical(void)
{
    robot_lifecycle_critical_state_t state;

    state.from_isr = (__get_IPSR() != 0U) ? 1u : 0u;
    state.saved_mask = 0u;
    if (state.from_isr != 0u)
    {
        state.saved_mask = taskENTER_CRITICAL_FROM_ISR();
    }
    else
    {
        taskENTER_CRITICAL();
    }
    return state;
}

static void robot_lifecycle_exit_critical(robot_lifecycle_critical_state_t state)
{
    if (state.from_isr != 0u)
    {
        taskEXIT_CRITICAL_FROM_ISR(state.saved_mask);
    }
    else
    {
        taskEXIT_CRITICAL();
    }
}

static uint8_t robot_lifecycle_manual_online_now(void)
{
    return (uint8_t)((manual_input_get_active_source() != MANUAL_INPUT_SRC_AUTO &&
                      toe_is_error(DBUS_TOE) == 0u) ? 1u : 0u);
}

static uint8_t robot_lifecycle_manual_safe_now(uint8_t manual_online)
{
    if (manual_online == 0u)
    {
        return 0u;
    }

    return control_input_switch_is_pos((uint16_t)control_input_switch(INPUT_SW_GIMBAL_MODE),
                                       g_config.manual_input.semantics.gimbal_safe_pos);
}

static robot_lifecycle_state_e robot_lifecycle_resolve(uint8_t manual_online,
                                                       uint8_t manual_safe,
                                                       const robot_lifecycle_snapshot_t *snapshot,
                                                       robot_lifecycle_reason_e *reason)
{
    if (snapshot->fault_latched != 0u)
    {
        *reason = (snapshot->reason == ROBOT_LIFECYCLE_REASON_FATAL_FAULT) ?
                  ROBOT_LIFECYCLE_REASON_FATAL_FAULT : ROBOT_LIFECYCLE_REASON_FAULT_LATCHED;
        return ROBOT_LIFECYCLE_FAULT;
    }
    if (manual_online == 0u)
    {
        *reason = ROBOT_LIFECYCLE_REASON_MANUAL_OFFLINE;
        return ROBOT_LIFECYCLE_SAFE;
    }
    if (manual_safe != 0u)
    {
        *reason = ROBOT_LIFECYCLE_REASON_MANUAL_SAFE_SWITCH;
        return ROBOT_LIFECYCLE_SAFE;
    }

    *reason = ROBOT_LIFECYCLE_REASON_NONE;
    return ROBOT_LIFECYCLE_ACTIVE;
}

static void robot_lifecycle_commit(robot_lifecycle_state_e next_state,
                                   robot_lifecycle_reason_e reason,
                                   uint8_t manual_online,
                                   uint8_t manual_safe)
{
    const uint32_t now = HAL_GetTick();
    robot_lifecycle_critical_state_t critical = robot_lifecycle_enter_critical();

    if (g_robot_lifecycle_inited == 0u)
    {
        memset(&g_robot_lifecycle, 0, sizeof(g_robot_lifecycle));
        g_robot_lifecycle.state = ROBOT_LIFECYCLE_BOOT;
        g_robot_lifecycle.prev_state = ROBOT_LIFECYCLE_BOOT;
        g_robot_lifecycle.reason = ROBOT_LIFECYCLE_REASON_BOOT;
        g_robot_lifecycle.enter_tick = now;
        g_robot_lifecycle_inited = 1u;
    }

    if (g_robot_lifecycle.state != next_state)
    {
        g_robot_lifecycle.prev_state = g_robot_lifecycle.state;
        g_robot_lifecycle.state = next_state;
        g_robot_lifecycle.enter_tick = now;
        g_robot_lifecycle.transition_count++;
    }

    g_robot_lifecycle.reason = reason;
    g_robot_lifecycle.manual_online = manual_online;
    g_robot_lifecycle.manual_safe = manual_safe;
    g_robot_lifecycle.output_allowed = (next_state == ROBOT_LIFECYCLE_ACTIVE) ? 1u : 0u;

    robot_lifecycle_exit_critical(critical);
}

void robot_lifecycle_init(void)
{
    robot_lifecycle_critical_state_t critical = robot_lifecycle_enter_critical();

    memset(&g_robot_lifecycle, 0, sizeof(g_robot_lifecycle));
    g_robot_lifecycle.state = ROBOT_LIFECYCLE_BOOT;
    g_robot_lifecycle.prev_state = ROBOT_LIFECYCLE_BOOT;
    g_robot_lifecycle.reason = ROBOT_LIFECYCLE_REASON_BOOT;
    g_robot_lifecycle.enter_tick = HAL_GetTick();
    g_robot_lifecycle_inited = 1u;

    robot_lifecycle_exit_critical(critical);
}

void robot_lifecycle_update(void)
{
    robot_lifecycle_snapshot_t snapshot;
    robot_lifecycle_reason_e reason;
    robot_lifecycle_state_e next_state;
    const uint8_t manual_online = robot_lifecycle_manual_online_now();
    const uint8_t manual_safe = robot_lifecycle_manual_safe_now(manual_online);
    robot_lifecycle_critical_state_t critical = robot_lifecycle_enter_critical();

    if (g_robot_lifecycle_inited == 0u)
    {
        memset(&g_robot_lifecycle, 0, sizeof(g_robot_lifecycle));
        g_robot_lifecycle.state = ROBOT_LIFECYCLE_BOOT;
        g_robot_lifecycle.prev_state = ROBOT_LIFECYCLE_BOOT;
        g_robot_lifecycle.reason = ROBOT_LIFECYCLE_REASON_BOOT;
        g_robot_lifecycle.enter_tick = HAL_GetTick();
        g_robot_lifecycle_inited = 1u;
    }
    snapshot = g_robot_lifecycle;

    robot_lifecycle_exit_critical(critical);

    next_state = robot_lifecycle_resolve(manual_online,
                                         manual_safe,
                                         &snapshot,
                                         &reason);
    robot_lifecycle_commit(next_state, reason, manual_online, manual_safe);
}

robot_lifecycle_state_e robot_lifecycle_current(void)
{
    robot_lifecycle_snapshot_t snapshot;

    (void)robot_lifecycle_get_snapshot(&snapshot);
    return snapshot.state;
}

uint8_t robot_lifecycle_output_allowed(void)
{
    robot_lifecycle_snapshot_t snapshot;

    (void)robot_lifecycle_get_snapshot(&snapshot);
    return snapshot.output_allowed;
}

uint8_t robot_lifecycle_get_snapshot(robot_lifecycle_snapshot_t *out)
{
    robot_lifecycle_critical_state_t critical;

    if (out == NULL)
    {
        return 0u;
    }

    robot_lifecycle_update();

    critical = robot_lifecycle_enter_critical();
    *out = g_robot_lifecycle;
    robot_lifecycle_exit_critical(critical);

    return 1u;
}

void robot_lifecycle_enter_fault(robot_lifecycle_reason_e reason)
{
    const uint32_t now = HAL_GetTick();
    robot_lifecycle_critical_state_t critical = robot_lifecycle_enter_critical();

    if (g_robot_lifecycle_inited == 0u)
    {
        memset(&g_robot_lifecycle, 0, sizeof(g_robot_lifecycle));
        g_robot_lifecycle.prev_state = ROBOT_LIFECYCLE_BOOT;
        g_robot_lifecycle_inited = 1u;
    }

    if (g_robot_lifecycle.state != ROBOT_LIFECYCLE_FAULT)
    {
        g_robot_lifecycle.prev_state = g_robot_lifecycle.state;
        g_robot_lifecycle.transition_count++;
    }

    g_robot_lifecycle.state = ROBOT_LIFECYCLE_FAULT;
    g_robot_lifecycle.reason = (reason == ROBOT_LIFECYCLE_REASON_NONE) ?
                               ROBOT_LIFECYCLE_REASON_FATAL_FAULT : reason;
    g_robot_lifecycle.enter_tick = now;
    g_robot_lifecycle.output_allowed = 0u;
    g_robot_lifecycle.fault_latched = 1u;

    robot_lifecycle_exit_critical(critical);
}

void robot_lifecycle_clear_fault(void)
{
    robot_lifecycle_critical_state_t critical = robot_lifecycle_enter_critical();

    if (g_robot_lifecycle_inited == 0u)
    {
        memset(&g_robot_lifecycle, 0, sizeof(g_robot_lifecycle));
        g_robot_lifecycle.state = ROBOT_LIFECYCLE_BOOT;
        g_robot_lifecycle.prev_state = ROBOT_LIFECYCLE_BOOT;
        g_robot_lifecycle.reason = ROBOT_LIFECYCLE_REASON_BOOT;
        g_robot_lifecycle.enter_tick = HAL_GetTick();
        g_robot_lifecycle_inited = 1u;
    }

    g_robot_lifecycle.fault_latched = 0u;
    robot_lifecycle_exit_critical(critical);

    robot_lifecycle_update();
}

uint8_t robot_lifecycle_fault_latched(void)
{
    robot_lifecycle_snapshot_t snapshot;

    (void)robot_lifecycle_get_snapshot(&snapshot);
    return snapshot.fault_latched;
}

const char *robot_lifecycle_name(robot_lifecycle_state_e state)
{
    switch (state)
    {
    case ROBOT_LIFECYCLE_BOOT:
        return "BOOT";
    case ROBOT_LIFECYCLE_SAFE:
        return "SAFE";
    case ROBOT_LIFECYCLE_ACTIVE:
        return "ACTIVE";
    case ROBOT_LIFECYCLE_FAULT:
        return "FAULT";
    default:
        return "UNKNOWN";
    }
}
