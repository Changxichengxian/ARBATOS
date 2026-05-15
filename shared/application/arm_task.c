/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "arm_task.h"

#include "cmsis_os.h"

#include "manual_input.h"
#include "watch.h"
#include "bsp_time.h"

#include "arm_motion.h"

#include <string.h>

static void arm_publish_status(uint16_t key_mask);

static uint32_t s_arm_status_seq = 0u;

static void arm_publish_status(uint16_t key_mask)
{
    arm_status_t status;
    const arm_j0_unitree_state_t *j0 = NULL;

    memset(&status, 0, sizeof(status));
    app_interface_header_init(&status.header,
                              APP_IF_SOURCE_MANUAL,
                              (uint16_t)sizeof(status),
                              bsp_time_get_tick_ms(),
                              ++s_arm_status_seq);
    status.mode = (uint8_t)((key_mask != 0u) ? ARM_INTERFACE_MODE_MANUAL : ARM_INTERFACE_MODE_HOLD);
    status.enabled = 1u;
    status.key_mask = key_mask;
    status.deadman_hold_ctrl = g_arm_deadman_hold_ctrl;
    status.key_speed_scale = g_arm_key_speed_scale;
    status.key_kd = g_arm_key_kd;
    status.j0_current = g_arm_j0_current;

    for (uint8_t i = 0u; i < APP_ARM_JOINT_COUNT; i++)
    {
        const arm_motor_feedback_t *feedback = arm_motion_get_feedback(i);
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

    j0 = arm_motion_get_j0_unitree_state();
    if (j0 != NULL)
    {
        status.j0_unitree = *j0;
    }

    (void)arm_status_publish(&status);
}

void arm_task(void const *argument)
{
    const manual_input_state_t *rc = NULL;

    (void)argument;
    rc = get_remote_control_point();
    arm_motion_init();

    for (;;)
    {
        const uint16_t key_mask = (rc != NULL) ? rc->key.v : 0u;

        watch_task_beat(WATCH_TASK_ARM);
        arm_motion_step_manual(key_mask);
        arm_publish_status(key_mask);
        osDelay(5u);
    }
}

const arm_motor_feedback_t *arm_get_feedback(uint8_t index)
{
    return arm_motion_get_feedback(index);
}

const arm_j0_unitree_state_t *arm_j0_unitree_get_state(void)
{
    return arm_motion_get_j0_unitree_state();
}

uint8_t CAN_rx_process_extra_frame(uint8_t bus, uint16_t std_id, uint8_t dlc, const uint8_t data[8])
{
    return arm_motion_process_can_feedback(bus, std_id, dlc, data);
}
