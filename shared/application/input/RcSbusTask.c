/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "RcSbusTask.h"

#include "FreeRTOS.h"
#include "task.h"

#include "BspRc.h"
#include "ManualInput.h"
#include "Watch.h"

void ManualInputSbusRxTask(void const *pvParameters)
{
    (void)pvParameters;

    BspRcSbusRxAttachTask(xTaskGetCurrentTaskHandle());

    uint8_t frame[BSP_RC_SBUS_FRAME_LENGTH];

    // Drain any frames received before the task handle is attached (startup window).
    // Otherwise the ring may fill up and never notify again (push fails -> no notify),
    // causing permanent loss of SBUS/DBUS input until reset.
    while (BspRcSbusRxPop(frame))
    {
        ManualInputOnSbusFrame(frame);
    }
    WatchTaskBeat(WATCH_TASK_RC_SBUS);

    for (;;)
    {
        WatchTaskWait(WATCH_TASK_RC_SBUS);
        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        WatchTaskBeat(WATCH_TASK_RC_SBUS);

        while (BspRcSbusRxPop(frame))
        {
            ManualInputOnSbusFrame(frame);
        }
    }
}

void RcSbusTask(void const *pvParameters)
{
    ManualInputSbusRxTask(pvParameters);
}
