/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "ArmTask.h"

#include "cmsis_os.h"

#include "ManualInputSnapshot.h"
#include "Watch.h"
#include "BspTime.h"
#include "RobotTaskBuildConfig.h"
#include "RobotSafety.h"
#include "ControlMgr.h"

#include "ArmInputPolicy.h"
#include "ArmMotorTable.h"
#include "ArmMotion.h"

#include <string.h>

#define ARM_TASK_PERIOD_MS 5u

static void ArmWriteStatus(uint16_t key_mask);
static uint8_t ArmControlMgrAllows(void);

static uint32_t s_arm_status_seq = 0u;

static void ArmWriteStatus(uint16_t key_mask)
{
    ArmStatus status;
    ArmMotionFaultStatus fault = {0};
    const ArmJ0UnitreeState *j0 = NULL;

    memset(&status, 0, sizeof(status));
    msg_header_init(&status.header,
                    MSG_SOURCE_MANUAL,
                    (uint16_t)sizeof(status),
                    BspTimeGetTickMs(),
                    ++s_arm_status_seq);
    status.mode = (uint8_t)((key_mask != 0u) ? ARM_MODE_MANUAL : ARM_MODE_HOLD);
    status.key_mask = key_mask;
    status.deadman_hold_ctrl = g_arm_deadman_hold_ctrl;
    status.key_speed_scale = g_arm_key_speed_scale;
    status.key_kd = g_arm_key_kd;
    status.j0_current = g_arm_j0_current;

    if (ArmMotionGetFaultStatus(&fault) != 0u)
    {
        status.fault_configured_mask = fault.configuredMask;
        status.fault_active_mask = fault.activeMask;
        status.fault_blocking_mask = fault.blockingMask;
        status.fault_recovery_mask = fault.recoveryMask;
        status.fault_inhibit_mask = fault.inhibitMask;
        status.fault_hold_zero_mask = fault.holdZeroMask;
        status.fault_inhibit_fail_count = fault.inhibitFailCount;
        status.fault_release_fail_count = fault.releaseFailCount;
        status.fault_initialized = fault.initialized;
        for (uint8_t i = 0u; i < ARM_JOINT_COUNT; i++)
        {
            status.fault_reason[i] = fault.reason[i];
        }
    }

    for (uint8_t i = 0u; i < ARM_JOINT_COUNT; i++)
    {
        const ArmMotorFeedback *feedback = ArmMotionGetFeedback(i);
        if (feedback == NULL)
        {
            continue;
        }

        status.motor[i] = *feedback;
    }

    for (uint8_t i = 0u; i < ARM_JOINT_COUNT; i++)
    {
        const uint32_t bit = 1u << i;

        if (fault.initialized != 0u &&
            (fault.configuredMask & bit) != 0u &&
            (fault.activeMask & bit) == 0u &&
            (fault.blockingMask & bit) == 0u)
        {
            status.active_joint_count++;
        }
    }

    j0 = ArmMotionGetJ0UnitreeState();
    if (j0 != NULL)
    {
        status.j0_unitree = *j0;
    }

    (void)ArmStatusWrite(&status);
}

static uint8_t ArmControlMgrAllows(void)
{
    ControlCtx context = {0};

    context.tick_ms = BspTimeGetTickMs();
    context.dt_s = (float)ARM_TASK_PERIOD_MS * 0.001f;

    if (ControlMgrUpdateDomain(ControlDomainArm, &context) != ControlResultOk)
    {
        return 0u;
    }

    return (ControlMgrActiveId(ControlDomainArm) == ControlIdArmMotion) ? 1u : 0u;
}

void ArmTask(void const *argument)
{
    ArmInputGate inputGate;
    uint16_t actionKeyMask = 0u;

    (void)argument;
    for (uint8_t i = 0u; i < (uint8_t)ARM_MOTOR_COUNT; i++)
    {
        actionKeyMask |= g_arm_motor_table[i].key_mask;
    }
    ArmInputGateInit(&inputGate);
    ArmMotionInit();

    for (;;)
    {
        ManualInputSnapshot manualInput;
        const uint8_t inputValid = ManualInputSnapshotRead(&manualInput);
        const uint8_t outputLocked = RobotSafetyOutputLocked();
        const uint8_t managerAllowed = ArmControlMgrAllows();
        const uint8_t controlAllowed =
            (uint8_t)(inputValid != 0u &&
                      manualInput.online != 0u &&
                      outputLocked == 0u &&
                      managerAllowed != 0u);
        const uint16_t observedKeys =
            (inputValid != 0u) ? manualInput.manual.key.v : 0u;
        ArmInputGateSync(&inputGate,
                         (inputValid != 0u) ? manualInput.authoritySeq : 0u,
                         (inputValid != 0u) ? manualInput.semanticsSeq : 0u);
        const uint16_t keyMask =
            ArmInputGateApply(&inputGate,
                              controlAllowed,
                              actionKeyMask,
                              observedKeys);

        WatchTaskBeat(WATCH_TASK_ARM);
        ArmMotionStepManual(keyMask);
        ArmWriteStatus(keyMask);
        osDelay(ARM_TASK_PERIOD_MS);
    }
}

const ArmMotorFeedback *ArmGetFeedback(uint8_t index)
{
    return ArmMotionGetFeedback(index);
}

const ArmJ0UnitreeState *ArmJ0UnitreeGetState(void)
{
    return ArmMotionGetJ0UnitreeState();
}

uint8_t CAN_rx_process_extra_frame(uint8_t bus, uint16_t std_id, uint8_t dlc, const uint8_t data[8])
{
#if ROBOT_TASK_BUILD_ARM
    return ArmMotionProcessCanFeedback(bus, std_id, dlc, data);
#else
    (void)bus;
    (void)std_id;
    (void)dlc;
    (void)data;
    return 0u;
#endif
}
