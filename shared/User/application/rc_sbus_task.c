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
#include "remote_control.h"
#include "app_watch.h"

void rc_sbus_task(void const *pvParameters)
{
    (void)pvParameters;

    bsp_rc_sbus_rx_attach_task(xTaskGetCurrentTaskHandle());

    uint8_t frame[BSP_RC_SBUS_FRAME_LENGTH];

    // Drain any frames received before the task handle is attached (startup window).
    // Otherwise the ring may fill up and never notify again (push fails -> no notify),
    // causing permanent loss of SBUS/DBUS input until reset.
    while (bsp_rc_sbus_rx_pop(frame))
    {
        remote_control_on_sbus_frame(frame);
    }
    app_watch_task_beat(APP_WATCH_TASK_RC_SBUS);

    for (;;)
    {
        app_watch_task_wait(APP_WATCH_TASK_RC_SBUS);
        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        app_watch_task_beat(APP_WATCH_TASK_RC_SBUS);

        while (bsp_rc_sbus_rx_pop(frame))
        {
            remote_control_on_sbus_frame(frame);
        }
    }
}
