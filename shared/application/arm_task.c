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

#include "arm_motion.h"

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
