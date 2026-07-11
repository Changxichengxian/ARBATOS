/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SHOOT_FAULT_POLICY_H
#define SHOOT_FAULT_POLICY_H

#include <stdint.h>

typedef struct
{
    uint8_t desiredMask;
    uint8_t acquireMask;
    uint8_t releaseMask;
    uint8_t holdZeroMask;
} ShootFaultInhibitPlan;

static inline uint8_t ShootGimbalStateBlocksFire(uint8_t fresh,
                                                 uint8_t valid,
                                                 uint8_t fireAllowed)
{
    return (fresh == 0u || valid == 0u || fireAllowed == 0u) ? 1u : 0u;
}

static inline ShootFaultInhibitPlan ShootFaultInhibitPlanMake(uint8_t configuredMask,
                                                              uint8_t blockedMask,
                                                              uint8_t heldMask)
{
    ShootFaultInhibitPlan plan;

    plan.desiredMask = (uint8_t)(configuredMask & blockedMask);
    plan.acquireMask = plan.desiredMask;
    plan.releaseMask = (uint8_t)(heldMask & (uint8_t)~plan.desiredMask);
    plan.holdZeroMask = (uint8_t)(plan.desiredMask | plan.releaseMask);
    return plan;
}

static inline uint8_t ShootFrictionFaultBlocksTrigger(uint8_t blockingMask,
                                                       uint8_t frictionMask)
{
    return ((blockingMask & frictionMask) != 0u) ? 1u : 0u;
}

#endif
