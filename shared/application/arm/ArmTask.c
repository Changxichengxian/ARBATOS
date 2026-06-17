/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "ArmTask.h"

#include "cmsis_os.h"

#include "ManualInput.h"
#include "Watch.h"
#include "BspTime.h"
#include "RobotTaskBuildConfig.h"
#include "ControlMgr.h"

#include "ArmMotion.h"

#include <string.h>

#define ARM_TASK_PERIOD_MS 5u

static void ArmWriteStatus(uint16_t key_mask);
static uint8_t ArmControlMgrAllows(void);

static uint32_t s_arm_status_seq = 0u;

static void ArmWriteStatus(uint16_t key_mask)
{
    ArmStatus status;
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

    for (uint8_t i = 0u; i < ARM_JOINT_COUNT; i++)
    {
        const ArmMotorFeedback *feedback = ArmMotionGetFeedback(i);
        if (feedback == NULL)
        {
            continue;
        }

        status.motor[i] = *feedback;
        if (feedback->online != 0u)
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
    (void)argument;
    ArmMotionInit();

    for (;;)
    {
        ManualInputState rc_snapshot;
        const uint16_t key_mask =
            (ManualInputGetCurrentCopy(&rc_snapshot) != 0u) ? rc_snapshot.key.v : 0u;

        WatchTaskBeat(WATCH_TASK_ARM);
        if (ArmControlMgrAllows() == 0u)
        {
            ArmMotionStepManual(0u);
            ArmWriteStatus(0u);
            osDelay(ARM_TASK_PERIOD_MS);
            continue;
        }

        ArmMotionStepManual(key_mask);
        ArmWriteStatus(key_mask);
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
