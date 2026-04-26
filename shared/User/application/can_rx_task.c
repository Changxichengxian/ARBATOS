/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "can_rx_task.h"

#include "FreeRTOS.h"
#include "task.h"

#include "bsp_can.h"
#include "CAN_receive.h"
#include "app_watch.h"

void can_rx_task(void const *pvParameters)
{
    (void)pvParameters;

    bsp_can_rx_attach_task(xTaskGetCurrentTaskHandle());

    bsp_can_frame_t f;
    for (;;)
    {
        app_watch_task_wait(APP_WATCH_TASK_CAN_RX);
        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        app_watch_task_beat(APP_WATCH_TASK_CAN_RX);

        while (bsp_can_rx_pop(&f))
        {
            CAN_rx_process_frame(f.bus, f.std_id, f.dlc, f.data);
        }
    }
}
