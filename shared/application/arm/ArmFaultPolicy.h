/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ARM_FAULT_POLICY_H
#define ARM_FAULT_POLICY_H

#include <stdint.h>

typedef struct
{
    uint32_t desiredMask;
    uint32_t acquireMask;
    uint32_t releaseMask;
    uint32_t holdZeroMask;
} ArmFaultInhibitPlan;

/* 恢复轴在 releaseMask 出现的这一帧仍包含在 holdZeroMask，下一帧才恢复输出。 */
static inline ArmFaultInhibitPlan ArmFaultInhibitPlanMake(uint32_t configuredMask,
                                                          uint32_t faultBlockingMask,
                                                          uint32_t mitEligibleMask,
                                                          uint8_t manualStopMit,
                                                          uint32_t heldMask)
{
    ArmFaultInhibitPlan plan;

    plan.desiredMask = faultBlockingMask & configuredMask;
    if (manualStopMit != 0u)
    {
        plan.desiredMask |= mitEligibleMask & configuredMask;
    }
    plan.acquireMask = plan.desiredMask;
    plan.releaseMask = heldMask & ~plan.desiredMask;
    plan.holdZeroMask = plan.desiredMask | plan.releaseMask;
    return plan;
}

#endif
