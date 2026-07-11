/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-07-10
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef ROBOT_TASK_BUILD_CLASSIC_CHASSIS
#include "RobotConfig.h"
#endif
#include "RobotTaskBuildConfig.h"

#include "ChassisCtrl.h"

#include <stddef.h>

#if ROBOT_TASK_BUILD_CLASSIC_CHASSIS
#include "ChassisRuntime.h"
#endif

#ifndef CHASSIS_CTRL_DEFAULT_PERIOD_MS
#define CHASSIS_CTRL_DEFAULT_PERIOD_MS 2u
#endif

#define CHASSIS_CTRL_MOTOR_COUNT 4u

static uint8_t s_chassisPrepared;
#if ROBOT_TASK_BUILD_CLASSIC_CHASSIS
static uint8_t s_chassisInitialized;
static uint8_t s_chassisRuntimeSafe = 1u;

static void ChassisCtrlRuntimeStop(void)
{
    if (s_chassisInitialized != 0u)
    {
        ChassisRuntimeStop();
    }
    s_chassisRuntimeSafe = 1u;
}
#endif

static void ChassisCtrlClearOutput(ControlCtx *context)
{
    if (context != NULL && context->output != NULL)
    {
        ChassisCtrlOutput *output = (ChassisCtrlOutput *)context->output;

        for (uint8_t i = 0u; i < CHASSIS_CTRL_MOTOR_COUNT; i++)
        {
            output->motorCurrent[i] = 0;
        }
        ControlOutputPermitClear(&output->outputPermit);
    }
}

static ControlResult ChassisCtrlEnter(const ControlController *controller,
                                      ControlCtx *context)
{
    (void)controller;
    ChassisCtrlClearOutput(context);

    if (s_chassisPrepared == 0u || context == NULL ||
        context->input == NULL || context->output == NULL)
    {
        return ControlResultNotActive;
    }

#if ROBOT_TASK_BUILD_CLASSIC_CHASSIS
    if (s_chassisInitialized == 0u)
    {
        ChassisRuntimeInit();
        s_chassisInitialized = 1u;
        s_chassisRuntimeSafe = 1u;
    }
    else
    {
        /* 重切只清运行输出，硬件和算法状态不重复初始化。 */
        ChassisCtrlRuntimeStop();
    }
    return ControlResultOk;
#else
    return ControlResultNotActive;
#endif
}

static ControlResult ChassisCtrlUpdate(const ControlController *controller,
                                       ControlCtx *context)
{
    const ChassisCtrlInput *input;
    ChassisCtrlOutput *output;

    (void)controller;
    ChassisCtrlClearOutput(context);
    if (s_chassisPrepared == 0u || context == NULL ||
        context->input == NULL || context->output == NULL)
    {
        return ControlResultBadArgument;
    }

    input = (const ChassisCtrlInput *)context->input;
    output = (ChassisCtrlOutput *)context->output;

#if ROBOT_TASK_BUILD_CLASSIC_CHASSIS
    if (s_chassisInitialized == 0u)
    {
        return ControlResultNotActive;
    }

    if (input->forceSafe != 0u)
    {
        ChassisRuntimeSafeStep(input->manualInput,
                               input->tickMs,
                               input->periodMs,
                               &context->outputPermit);
        s_chassisRuntimeSafe = 1u;
        return ControlResultOk;
    }

    ChassisRuntimeStep(input->manualInput,
                       input->tickMs,
                       input->periodMs,
                       &context->outputPermit,
                       output->motorCurrent);
    s_chassisRuntimeSafe = 0u;
    return ControlResultOk;
#else
    (void)input;
    (void)output;
    return ControlResultNotActive;
#endif
}

static ControlResult ChassisCtrlLeave(const ControlController *controller,
                                      ControlCtx *context)
{
    (void)controller;
    ChassisCtrlClearOutput(context);

#if ROBOT_TASK_BUILD_CLASSIC_CHASSIS
    ChassisCtrlRuntimeStop();
#endif
    return ControlResultOk;
}

const ControlController *ChassisCtrlDesc(void)
{
    static const char *const inputs[] = {
        "input.manual",
        "state.gimbal",
        "sensor.imu",
    };
    static const char *const outputs[] = {
        "motor.chassis0",
        "motor.chassis1",
        "motor.chassis2",
        "motor.chassis3",
    };
    static const ControlController controller = {
        .id = ControlIdClassicChassis,
        .domain = ControlDomainChassis,
        .claim_mask = ControlResChassisWheels,
        .name = "controller.classic_chassis",
        .meta = {
            .period_ms = CHASSIS_CTRL_DEFAULT_PERIOD_MS,
            .input_count = 3u,
            .output_count = 4u,
            .inputs = inputs,
            .outputs = outputs,
        },
        .enter = ChassisCtrlEnter,
        .update = ChassisCtrlUpdate,
        .exit = ChassisCtrlLeave,
        .stop = ChassisCtrlLeave,
    };

    return &controller;
}

void ChassisCtrlPrepare(void)
{
    /* 初始化由 ControlMgr 首次 enter 完成，准备阶段不访问电机和传感器。 */
    s_chassisPrepared = 1u;
}

ControlResult ChassisCtrlStep(const ChassisCtrlInput *input, ChassisCtrlOutput *output)
{
    ControlCtx context = {0};

    if (output != NULL)
    {
        for (uint8_t i = 0u; i < CHASSIS_CTRL_MOTOR_COUNT; i++)
        {
            output->motorCurrent[i] = 0;
        }
        ControlOutputPermitClear(&output->outputPermit);
    }
    if (input == NULL || output == NULL)
    {
        return ControlResultBadArgument;
    }
    if (s_chassisPrepared == 0u)
    {
        return ControlResultNotActive;
    }

#if ROBOT_TASK_BUILD_CLASSIC_CHASSIS
    ControlResult result;

    context.dt_s = (float)input->periodMs * 0.001f;
    context.tick_ms = input->tickMs;
    context.input = (void *)input;
    context.output = output;
    result = ControlMgrUpdateDomain(ControlDomainChassis, &context);
    if (result == ControlResultOk)
    {
        output->outputPermit = context.outputPermit;
    }
    else
    {
        ControlOutputPermitClear(&output->outputPermit);
    }
    if (result != ControlResultOk &&
        s_chassisInitialized != 0u &&
        s_chassisRuntimeSafe == 0u)
    {
        /* 管理层异常时也要立即清掉上一帧留在 LowCmd 的残留电流。 */
        ChassisCtrlRuntimeStop();
    }
    return result;
#else
    (void)context;
    return ControlResultNotActive;
#endif
}
