/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include "RobotLifecycle.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

#include "ControlInput.h"
#include "ManualInputSnapshot.h"

typedef struct
{
    uint8_t from_isr;
    UBaseType_t saved_mask;
} RobotLifecycleCriticalState;

static RobotLifecycleSnapshot g_robot_lifecycle;
static uint8_t g_robot_lifecycle_inited;

static RobotLifecycleCriticalState RobotLifecycleEnterCritical(void)
{
    RobotLifecycleCriticalState state;

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

static void RobotLifecycleExitCritical(RobotLifecycleCriticalState state)
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

static uint8_t RobotLifecycleManualOnline(const ManualInputSnapshot *manualInput)
{
    return (uint8_t)(manualInput != NULL && manualInput->online != 0u);
}

static uint8_t RobotLifecycleManualSafe(const ManualInputSnapshot *manualInput,
                                        uint8_t manualOnline)
{
    if (manualOnline == 0u || manualInput == NULL)
    {
        return 0u;
    }

    return ControlInputSwitchIsPos(
        manualInput->control.sw[INPUT_SW_GIMBAL_MODE],
        manualInput->semantics.GimbalSafePos);
}

static RobotLifecycleState RobotLifecycleResolve(uint8_t manual_online,
                                                       uint8_t manual_safe,
                                                       const RobotLifecycleSnapshot *snapshot,
                                                       RobotLifecycleReason *reason)
{
    if (snapshot->fault_latched != 0u)
    {
        *reason = ROBOT_LIFECYCLE_REASON_FATAL_FAULT;
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
    if (snapshot->startup_safe_seen == 0u)
    {
        *reason = ROBOT_LIFECYCLE_REASON_STARTUP_SAFE_REQUIRED;
        return ROBOT_LIFECYCLE_SAFE;
    }

    *reason = ROBOT_LIFECYCLE_REASON_NONE;
    return ROBOT_LIFECYCLE_ACTIVE;
}

static void RobotLifecycleCommit(RobotLifecycleState next_state,
                                   RobotLifecycleReason reason,
                                   uint8_t manual_online,
                                   uint8_t manual_safe)
{
    const uint32_t now = HAL_GetTick();
    RobotLifecycleCriticalState critical = RobotLifecycleEnterCritical();

    if (g_robot_lifecycle_inited == 0u)
    {
        memset(&g_robot_lifecycle, 0, sizeof(g_robot_lifecycle));
        g_robot_lifecycle.state = ROBOT_LIFECYCLE_BOOT;
        g_robot_lifecycle.prev_state = ROBOT_LIFECYCLE_BOOT;
        g_robot_lifecycle.reason = ROBOT_LIFECYCLE_REASON_BOOT;
        g_robot_lifecycle.enter_tick = now;
        g_robot_lifecycle_inited = 1u;
    }

    /* 解析与提交之间可能被异常入口抢占；锁存故障绝不能被旧输入结论重新放行。 */
    if (g_robot_lifecycle.fault_latched != 0u)
    {
        next_state = ROBOT_LIFECYCLE_FAULT;
        reason = ROBOT_LIFECYCLE_REASON_FATAL_FAULT;
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
    g_robot_lifecycle.update_tick = now;

    RobotLifecycleExitCritical(critical);
}

void RobotLifecycleInit(void)
{
    RobotLifecycleCriticalState critical = RobotLifecycleEnterCritical();

    if (g_robot_lifecycle_inited == 0u)
    {
        memset(&g_robot_lifecycle, 0, sizeof(g_robot_lifecycle));
        g_robot_lifecycle.state = ROBOT_LIFECYCLE_BOOT;
        g_robot_lifecycle.prev_state = ROBOT_LIFECYCLE_BOOT;
        g_robot_lifecycle.reason = ROBOT_LIFECYCLE_REASON_BOOT;
        g_robot_lifecycle.enter_tick = HAL_GetTick();
        g_robot_lifecycle.update_tick = g_robot_lifecycle.enter_tick;
        g_robot_lifecycle_inited = 1u;
    }

    RobotLifecycleExitCritical(critical);
}

static void RobotLifecycleUpdateFromInput(const ManualInputSnapshot *manualInput)
{
    RobotLifecycleSnapshot snapshot;
    RobotLifecycleReason reason;
    RobotLifecycleState next_state;
    const uint8_t manual_online = RobotLifecycleManualOnline(manualInput);
    const uint8_t manual_safe = RobotLifecycleManualSafe(manualInput, manual_online);
    RobotLifecycleCriticalState critical = RobotLifecycleEnterCritical();

    if (g_robot_lifecycle_inited == 0u)
    {
        memset(&g_robot_lifecycle, 0, sizeof(g_robot_lifecycle));
        g_robot_lifecycle.state = ROBOT_LIFECYCLE_BOOT;
        g_robot_lifecycle.prev_state = ROBOT_LIFECYCLE_BOOT;
        g_robot_lifecycle.reason = ROBOT_LIFECYCLE_REASON_BOOT;
        g_robot_lifecycle.enter_tick = HAL_GetTick();
        g_robot_lifecycle_inited = 1u;
    }
    if (manualInput != NULL && manualInput->semanticsSeq != 0u &&
        g_robot_lifecycle.manual_semantics_seq != manualInput->semanticsSeq)
    {
        /* 安全档定义一旦变化，旧的解锁资格立即失效。 */
        g_robot_lifecycle.manual_semantics_seq = manualInput->semanticsSeq;
        g_robot_lifecycle.startup_safe_seen = 0u;
    }
    if (manualInput != NULL && manualInput->authoritySeq != 0u &&
        g_robot_lifecycle.manual_authority_seq != manualInput->authoritySeq)
    {
        /* 代表控制来源换权后，新来源必须重新给出本代安全档。 */
        g_robot_lifecycle.manual_authority_seq = manualInput->authoritySeq;
        g_robot_lifecycle.startup_safe_seen = 0u;
    }
    if (manual_online != 0u && manual_safe != 0u)
    {
        /* 每次 MCU 启动后必须真实见过一次安全档，复位时保持运行档不会自动重新上力。 */
        g_robot_lifecycle.startup_safe_seen = 1u;
    }
    snapshot = g_robot_lifecycle;

    RobotLifecycleExitCritical(critical);

    next_state = RobotLifecycleResolve(manual_online,
                                         manual_safe,
                                         &snapshot,
                                         &reason);
    RobotLifecycleCommit(next_state, reason, manual_online, manual_safe);
}

void RobotLifecycleUpdate(void)
{
    ManualInputSnapshot manualInput;
    const ManualInputSnapshot *frameInput =
        (ManualInputSnapshotRead(&manualInput) != 0u) ? &manualInput : NULL;

    RobotLifecycleUpdateFromInput(frameInput);
}

RobotLifecycleState RobotLifecycleCurrent(void)
{
    RobotLifecycleSnapshot snapshot;

    (void)RobotLifecycleGetSnapshot(&snapshot);
    return snapshot.state;
}

uint8_t RobotLifecycleOutputAllowed(void)
{
    RobotLifecycleSnapshot snapshot;

    (void)RobotLifecycleGetSnapshot(&snapshot);
    return snapshot.output_allowed;
}

uint8_t RobotLifecycleGetSnapshot(RobotLifecycleSnapshot *out)
{
    RobotLifecycleCriticalState critical;

    if (out == NULL)
    {
        return 0u;
    }

    critical = RobotLifecycleEnterCritical();
    *out = g_robot_lifecycle;
    RobotLifecycleExitCritical(critical);

    if (out->output_allowed != 0u &&
        (HAL_GetTick() - out->update_tick) > ROBOT_LIFECYCLE_UPDATE_TIMEOUT_MS)
    {
        /* 唯一推进者停滞时，任何独立输出任务读取到的都必须是锁定结论。 */
        out->state = ROBOT_LIFECYCLE_SAFE;
        out->reason = ROBOT_LIFECYCLE_REASON_UPDATE_STALE;
        out->output_allowed = 0u;
        out->manual_online = 0u;
        out->manual_safe = 0u;
    }

    return 1u;
}

void RobotLifecycleEnterFatalFault(void)
{
    const uint32_t now = HAL_GetTick();
    RobotLifecycleCriticalState critical = RobotLifecycleEnterCritical();

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
    g_robot_lifecycle.reason = ROBOT_LIFECYCLE_REASON_FATAL_FAULT;
    g_robot_lifecycle.enter_tick = now;
    g_robot_lifecycle.update_tick = now;
    g_robot_lifecycle.output_allowed = 0u;
    g_robot_lifecycle.fault_latched = 1u;

    RobotLifecycleExitCritical(critical);
}

uint8_t RobotLifecycleFaultLatched(void)
{
    RobotLifecycleSnapshot snapshot;

    (void)RobotLifecycleGetSnapshot(&snapshot);
    return snapshot.fault_latched;
}

const char *RobotLifecycleName(RobotLifecycleState state)
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
