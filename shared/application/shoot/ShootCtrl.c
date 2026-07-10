/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-07-10
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef ROBOT_TASK_BUILD_SHOOT_RM
#include "RobotConfig.h"
#endif
#include "RobotTaskBuildConfig.h"

#include "ShootCtrl.h"

#include <stddef.h>

#if ROBOT_TASK_BUILD_SHOOT_RM
#include "ShootRuntime.h"
#endif

#ifndef SHOOT_CTRL_DEFAULT_PERIOD_MS
#define SHOOT_CTRL_DEFAULT_PERIOD_MS 1u
#endif

static uint8_t s_shootPrepared;
#if ROBOT_TASK_BUILD_SHOOT_RM
static uint8_t s_shootInitialized;
static uint8_t s_shootRuntimeSafe;

static void ShootCtrlRuntimeStop(void)
{
    ShootRuntimeStop();
    s_shootRuntimeSafe = 1u;
}
#endif

static void ShootCtrlClearOutput(ControlCtx *context)
{
    if (context != NULL && context->output != NULL)
    {
        ShootCtrlOutput *output = (ShootCtrlOutput *)context->output;

        output->triggerCurrent = 0;
    }
}

static ControlResult ShootCtrlEnter(const ControlController *controller,
                                    ControlCtx *context)
{
    (void)controller;
    ShootCtrlClearOutput(context);

    if (s_shootPrepared == 0u || context == NULL)
    {
        return ControlResultNotActive;
    }

#if ROBOT_TASK_BUILD_SHOOT_RM
    if (s_shootInitialized == 0u)
    {
        ShootRuntimeInit();
        s_shootInitialized = 1u;
        s_shootRuntimeSafe = 1u;
    }
    else
    {
        /* 重新进入时只清输出，不重复初始化运行状态。 */
        ShootCtrlRuntimeStop();
    }
    return ControlResultOk;
#else
    return ControlResultNotActive;
#endif
}

static ControlResult ShootCtrlUpdate(const ControlController *controller,
                                     ControlCtx *context)
{
    const ShootCtrlInput *input;
    ShootCtrlOutput *output;

    (void)controller;
    ShootCtrlClearOutput(context);
    if (s_shootPrepared == 0u || context == NULL ||
        context->input == NULL || context->output == NULL)
    {
        return ControlResultBadArgument;
    }

    input = (const ShootCtrlInput *)context->input;
    output = (ShootCtrlOutput *)context->output;

#if ROBOT_TASK_BUILD_SHOOT_RM
    if (input->forceSafe != 0u)
    {
        ShootCtrlRuntimeStop();
        return ControlResultOk;
    }

    if (s_shootInitialized == 0u)
    {
        return ControlResultNotActive;
    }

    output->triggerCurrent = ShootRuntimeStep();
    s_shootRuntimeSafe = 0u;
    return ControlResultOk;
#else
    (void)input;
    (void)output;
    return ControlResultNotActive;
#endif
}

static ControlResult ShootCtrlLeave(const ControlController *controller,
                                    ControlCtx *context)
{
    (void)controller;
    ShootCtrlClearOutput(context);

#if ROBOT_TASK_BUILD_SHOOT_RM
    ShootCtrlRuntimeStop();
#endif
    return ControlResultOk;
}

const ControlController *ShootCtrlDesc(void)
{
    static const char *const inputs[] = {
        "input.manual",
    };
    static const char *const outputs[] = {
        "motor.trigger",
        "motor.friction0",
        "motor.friction1",
        "motor.friction2",
        "motor.friction3",
    };
    static const ControlController controller = {
        .id = ControlIdShoot,
        .domain = ControlDomainShoot,
        .claim_mask = ControlResShootTrigger | ControlResShootFriction,
        .name = "controller.ShootRm",
        .meta = {
            .period_ms = SHOOT_CTRL_DEFAULT_PERIOD_MS,
            .input_count = 1u,
            .output_count = 5u,
            .inputs = inputs,
            .outputs = outputs,
        },
        .enter = ShootCtrlEnter,
        .update = ShootCtrlUpdate,
        .exit = ShootCtrlLeave,
        .stop = ShootCtrlLeave,
    };

    return &controller;
}

void ShootCtrlPrepare(void)
{
    /* 真实初始化留到 ControlMgr 首次 enter，避免启动阶段抢先访问硬件。 */
    s_shootPrepared = 1u;
}

ControlResult ShootCtrlStep(const ShootCtrlInput *input, ShootCtrlOutput *output)
{
    ControlCtx context = {0};

    if (output != NULL)
    {
        output->triggerCurrent = 0;
    }
    if (input == NULL || output == NULL)
    {
        return ControlResultBadArgument;
    }
    if (s_shootPrepared == 0u)
    {
        return ControlResultNotActive;
    }

#if ROBOT_TASK_BUILD_SHOOT_RM
    ControlResult result;

    context.dt_s = (float)input->periodMs * 0.001f;
    context.tick_ms = input->tickMs;
    context.input = (void *)input;
    context.output = output;
    result = ControlMgrUpdateDomain(ControlDomainShoot, &context);
    if (result != ControlResultOk &&
        s_shootInitialized != 0u &&
        s_shootRuntimeSafe == 0u)
    {
        /* 管理层异常也要立即清掉 Runtime 直接写入 LowCmd 的摩擦轮命令。 */
        ShootCtrlRuntimeStop();
    }
    return result;
#else
    (void)context;
    return ControlResultNotActive;
#endif
}
