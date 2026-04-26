/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "can_feedback_rx_task.h"

#include "FreeRTOS.h"
#include "task.h"

#include "bsp_can.h"
#include "CAN_receive.h"
#include "watch.h"
#include "rt_profiler.h"
#include "robot_task_profile.h"

void can_feedback_rx_task(void const *pvParameters)
{
    (void)pvParameters;

    bsp_can_rx_attach_task(xTaskGetCurrentTaskHandle());

    bsp_can_frame_t f;
    for (;;)
    {
        if (bsp_can_rx_pending() == 0u)
        {
            watch_task_wait(WATCH_TASK_CAN_FEEDBACK_RX);
            (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        }
        watch_task_beat(WATCH_TASK_CAN_FEEDBACK_RX);
        const uint64_t wake_start_us = rt_profiler_begin();
        const uint32_t max_frames = robot_profile_can_feedback_rx_max_frames_per_wake();
        const uint32_t budget_us = robot_profile_can_feedback_rx_budget_us();
        uint32_t processed = 0u;

        while (processed < max_frames && bsp_can_rx_pop(&f))
        {
            CAN_rx_process_frame(f.bus, f.std_id, f.dlc, f.data);
            processed++;
            if (budget_us != 0u &&
                (uint32_t)(BSP_DWT_GetUs() - wake_start_us) >= budget_us)
            {
                break;
            }
        }
        rt_profiler_end(RT_PROFILER_CAN_FEEDBACK_RX_WAKE, wake_start_us);
        if (bsp_can_rx_pending() != 0u)
        {
            taskYIELD();
        }
    }
}
