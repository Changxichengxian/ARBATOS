/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ControlRuntimeTask.h"

#include "ControlRuntime.h"
#include "FreeRTOS.h"
#include "RtProf.h"
#include "Watch.h"
#include "cmsis_os.h"
#include "task.h"

static void ControlRuntimeTaskLoop(ControlDomain domain,
                                   WatchTaskId watchTask,
                                   RtProfId profiler)
{
    TickType_t lastWake = xTaskGetTickCount();

    for (;;)
    {
        const uint64_t startedUs = RtProfBegin();
        const uint16_t periodMs = ControlRuntimePeriodMs(domain);
        const TickType_t delayStart = xTaskGetTickCount();

        WatchTaskBeat(watchTask);
        (void)ControlRuntimeRunDomain(domain);
        RtProfEnd(profiler, startedUs);
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(periodMs));
        if (xTaskGetTickCount() == delayStart)
        {
            vTaskDelay(1u);
            lastWake = xTaskGetTickCount();
        }
    }
}

void ControlChassisTask(void const *argument)
{
    (void)argument;
    ControlRuntimeTaskLoop(ControlDomainChassis,
                           WATCH_TASK_CHASSIS_CONTROL,
                           RtProfChassisLoop);
}

void ControlGimbalTask(void const *argument)
{
    (void)argument;
    ControlRuntimeTaskLoop(ControlDomainGimbal,
                           WATCH_TASK_GIMBAL_CONTROL,
                           RtProfGimbalLoop);
}
