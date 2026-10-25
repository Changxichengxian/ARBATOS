/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include "RobotConfig.h"
#include "RobotTaskBuildConfig.h"

#if ROBOT_TASK_BUILD_SHOOT_RM
#include "ShootControlTask.h"
#include "ShootCtrl.h"
#include "BspTime.h"
#include "ControlInput.h"
#include "ManualInputSnapshot.h"
#include "RobotLifecycle.h"
#include "RobotTaskProfile.h"
#include "RtProf.h"
#include "Watch.h"
#include "FreeRTOS.h"
#include "task.h"

void ShootControlTask(void const *argument)
{
    TickType_t lastWake;

    (void)argument;
    /* 沿用现有车型的 201 ms 启动等待，给输入和电机首帧留出时间。 */
    vTaskDelay(pdMS_TO_TICKS(201u));
    lastWake = xTaskGetTickCount();
    ShootCtrlPrepare();
    for (;;) {
        const uint64_t startedUs = RtProfBegin();
        const uint16_t periodMs = RobotProfileShootControlPeriodMs();
        const TickType_t delayStart = xTaskGetTickCount();
        ManualInputSnapshot manual = {0};
        ShootCtrlInput input = {0};
        ShootCtrlOutput output = {0};

        WatchTaskBeat(WATCH_TASK_SHOOT_CONTROL);
        const uint8_t haveInput = ManualInputSnapshotRead(&manual);
        input.manualInput = (haveInput != 0u) ? &manual : NULL;
        input.tickMs = BspTimeGetTickMs();
        input.periodMs = periodMs;
        /* 保留原云台安全档对发射的约束；独立调度不能扩大输出权限。 */
        input.forceSafe = (uint8_t)(haveInput == 0u || manual.online == 0u || manual.dataValid == 0u ||
            manual.sourceTimeoutMs == 0u || manual.sourceAgeMs > manual.sourceTimeoutMs ||
            ControlInputSwitchIsPos(manual.control.sw[INPUT_SW_GIMBAL_MODE], manual.semantics.GimbalSafePos) != 0u ||
            RobotLifecycleOutputAllowed() == 0u);
        (void)ShootCtrlStep(&input, &output);
        RtProfEnd(RtProfShootLoop, startedUs);
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(periodMs));
        if (xTaskGetTickCount() == delayStart) {
            vTaskDelay(1u);
            lastWake = xTaskGetTickCount();
        }
    }
}
#endif
