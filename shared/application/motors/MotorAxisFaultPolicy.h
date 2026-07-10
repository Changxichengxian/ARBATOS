/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_AXIS_FAULT_POLICY_H
#define MOTOR_AXIS_FAULT_POLICY_H

#include <stdint.h>

typedef struct
{
    uint32_t desiredMask;
    uint32_t acquireMask;
    uint32_t releaseMask;
    uint32_t holdZeroMask;
} MotorAxisFaultInhibitPlan;

/* 恢复轴在释放禁写的这一帧仍保持零输出，下一完整控制帧才重新接管。 */
static inline MotorAxisFaultInhibitPlan MotorAxisFaultInhibitPlanMake(uint32_t configuredMask,
                                                                      uint32_t blockingMask,
                                                                      uint32_t heldMask)
{
    MotorAxisFaultInhibitPlan plan;

    plan.desiredMask = configuredMask & blockingMask;
    plan.acquireMask = plan.desiredMask;
    plan.releaseMask = heldMask & ~plan.desiredMask;
    plan.holdZeroMask = plan.desiredMask | plan.releaseMask;
    return plan;
}

static inline uint8_t MotorAxisFaultMustHoldZero(uint32_t holdZeroMask, uint8_t axis)
{
    if (axis >= 32u)
    {
        return 0u;
    }
    return ((holdZeroMask & (1u << axis)) != 0u) ? 1u : 0u;
}

#endif
