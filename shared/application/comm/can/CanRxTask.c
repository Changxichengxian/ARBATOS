/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "CanRxTask.h"

#include "FreeRTOS.h"
#include "task.h"

#include "BspCan.h"
#include "BspTime.h"
#include "CanReceive.h"
#include "PowerMeter.h"
#include "RobotConfig.h"
#include "Watch.h"
#include "RtProf.h"
#include "RobotTaskProfile.h"

void CanRxTask(void const *pvParameters)
{
    (void)pvParameters;

    PowerMeterInit(g_config.powerMeter.enable,
                   g_config.powerMeter.canBus,
                   g_config.powerMeter.canId,
                   g_config.powerMeter.freshTimeoutMs);
    BspCanRxAttachTask(xTaskGetCurrentTaskHandle());

    BspCanFrame f;
    for (;;)
    {
        if (BspCanRxPending() == 0u)
        {
            WatchTaskWait(WATCH_TASK_CAN_FEEDBACK_RX);
            (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        }
        WatchTaskBeat(WATCH_TASK_CAN_FEEDBACK_RX);
        const uint64_t wake_start_us = RtProfBegin();
        const uint32_t max_frames = RobotProfileCanFeedbackRxMaxFramesPerWake();
        const uint32_t budget_us = RobotProfileCanFeedbackRxBudgetUs();
        uint32_t processed = 0u;

        while (processed < max_frames && BspCanRxPop(&f))
        {
            PowerMeterCanRx(f.bus, f.std_id, f.dlc, f.flags, f.data, f.rxTickMs);
            CAN_rx_process_frame(f.bus, f.std_id, f.dlc, f.data);
            processed++;
            if (budget_us != 0u &&
                (uint32_t)(BSP_DWT_GetUs() - wake_start_us) >= budget_us)
            {
                break;
            }
        }
        RtProfEnd(RtProfCanRxWake, wake_start_us);
        if (BspCanRxPending() != 0u)
        {
            vTaskDelay(1u);
        }
    }
}
