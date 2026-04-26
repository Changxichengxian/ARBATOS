/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "rc_sbus_task.h"

#include "FreeRTOS.h"
#include "task.h"

#include "bsp_rc.h"
#include "manual_input.h"
#include "watch.h"

void manual_input_sbus_rx_task(void const *pvParameters)
{
    (void)pvParameters;

    bsp_rc_sbus_rx_attach_task(xTaskGetCurrentTaskHandle());

    uint8_t frame[BSP_RC_SBUS_FRAME_LENGTH];

    // Drain any frames received before the task handle is attached (startup window).
    // Otherwise the ring may fill up and never notify again (push fails -> no notify),
    // causing permanent loss of SBUS/DBUS input until reset.
    while (bsp_rc_sbus_rx_pop(frame))
    {
        manual_input_on_sbus_frame(frame);
    }
    watch_task_beat(WATCH_TASK_RC_SBUS);

    for (;;)
    {
        watch_task_wait(WATCH_TASK_RC_SBUS);
        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        watch_task_beat(WATCH_TASK_RC_SBUS);

        while (bsp_rc_sbus_rx_pop(frame))
        {
            manual_input_on_sbus_frame(frame);
        }
    }
}

void rc_sbus_task(void const *pvParameters)
{
    manual_input_sbus_rx_task(pvParameters);
}
