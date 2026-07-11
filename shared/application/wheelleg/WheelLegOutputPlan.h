/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef WHEELLEG_OUTPUT_PLAN_H
#define WHEELLEG_OUTPUT_PLAN_H

#include <stdint.h>

#include "ControlCore.h"
#include "ControlOutputPermit.h"

#ifdef __cplusplus
extern "C" {
#endif

#define WHEELLEG_OUTPUT_AXIS_COUNT 6u
#define WHEELLEG_OUTPUT_ALL_MASK ((uint8_t)((1u << WHEELLEG_OUTPUT_AXIS_COUNT) - 1u))

typedef enum
{
    WheelLegOutputLeftFront = 0u,
    WheelLegOutputLeftBack,
    WheelLegOutputLeftWheel,
    WheelLegOutputRightFront,
    WheelLegOutputRightBack,
    WheelLegOutputRightWheel,
    WheelLegOutputAxisCount = WHEELLEG_OUTPUT_AXIS_COUNT,
} WheelLegOutputAxis;

typedef struct
{
    MotorId id[WHEELLEG_OUTPUT_AXIS_COUNT];
    MotorCmd cmd[WHEELLEG_OUTPUT_AXIS_COUNT];
    uint8_t commandMask;
    uint8_t publishAttempted;
} WheelLegOutputPlan;

typedef uint8_t (*WheelLegOutputPublish)(const MotorId *ids,
                                        const MotorCmd *cmds,
                                        uint8_t count,
                                        const ControlOutputPermit *permit);

/*
 * 每帧总是建立 LF/LB/LW/RF/RB/RW 六槽计划。未被当前模式覆盖的槽位
 * 使用显式 Disable，避免测试或降级路径沿用上一周期的残留命令。
 */
static inline uint8_t WheelLegOutputPlanBegin(WheelLegOutputPlan *plan,
                                              const MotorId ids[WHEELLEG_OUTPUT_AXIS_COUNT])
{
    uint8_t i;

    if (plan == NULL || ids == NULL)
    {
        return 0u;
    }

    plan->commandMask = 0u;
    plan->publishAttempted = 0u;
    for (i = 0u; i < WHEELLEG_OUTPUT_AXIS_COUNT; i++)
    {
        plan->id[i] = ids[i];
        control_core_cmd_set_disable(&plan->cmd[i]);
    }
    return 1u;
}

static inline uint8_t WheelLegOutputPlanSet(WheelLegOutputPlan *plan,
                                            WheelLegOutputAxis axis,
                                            const MotorCmd *cmd)
{
    if (plan == NULL || cmd == NULL ||
        (uint8_t)axis >= (uint8_t)WheelLegOutputAxisCount ||
        cmd->active == 0u || MotorModeKnown(cmd->mode) == 0u)
    {
        return 0u;
    }

    plan->cmd[(uint8_t)axis] = *cmd;
    plan->commandMask |= (uint8_t)(1u << (uint8_t)axis);
    return 1u;
}

static inline uint8_t WheelLegOutputPlanSetStateTorque(WheelLegOutputPlan *plan,
                                                       WheelLegOutputAxis axis,
                                                       fp32 position,
                                                       fp32 velocity,
                                                       fp32 kp,
                                                       fp32 kd,
                                                       fp32 torque)
{
    MotorCmd cmd;

    control_core_cmd_set_state_torque(&cmd, position, velocity, kp, kd, torque);
    return WheelLegOutputPlanSet(plan, axis, &cmd);
}

/*
 * publishAttempted 在调用下层前置位，保证同一计划即使发布被拒也不会退化成
 * 第二批或逐轴重试；下个控制周期必须重新 Begin 后再发布。
 */
static inline uint8_t WheelLegOutputPlanCommit(WheelLegOutputPlan *plan,
                                               const ControlOutputPermit *permit,
                                               WheelLegOutputPublish publish)
{
    if (plan == NULL || permit == NULL || publish == NULL ||
        plan->publishAttempted != 0u)
    {
        return 0u;
    }

    plan->publishAttempted = 1u;
    return publish(plan->id,
                   plan->cmd,
                   WHEELLEG_OUTPUT_AXIS_COUNT,
                   permit);
}

#ifdef __cplusplus
}
#endif

#endif
