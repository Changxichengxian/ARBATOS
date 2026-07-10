/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-07-10
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef CHASSIS_INPUT_POLICY_H
#define CHASSIS_INPUT_POLICY_H

#include <stddef.h>
#include <stdint.h>

typedef struct
{
    uint32_t authoritySeq;
    uint32_t semanticsSeq;
    uint8_t waitRelease;
} ChassisInputGate;

static inline uint8_t ChassisInputGateApply(ChassisInputGate *gate,
                                            uint32_t authoritySeq,
                                            uint32_t semanticsSeq,
                                            uint8_t controlAllowed,
                                            uint8_t spinRequested,
                                            uint8_t actionKeyRequested)
{
    if (gate == NULL)
    {
        return 0u;
    }

    if (controlAllowed == 0u || authoritySeq == 0u || semanticsSeq == 0u)
    {
        gate->waitRelease = 1u;
        return 0u;
    }
    if (gate->authoritySeq != authoritySeq || gate->semanticsSeq != semanticsSeq)
    {
        gate->authoritySeq = authoritySeq;
        gate->semanticsSeq = semanticsSeq;
        gate->waitRelease = 1u;
    }

    if (gate->waitRelease != 0u)
    {
        if (actionKeyRequested == 0u && spinRequested == 0u)
        {
            gate->waitRelease = 0u;
        }
        return 0u;
    }

    return 1u;
}

#endif
