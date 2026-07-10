/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-07-10
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef SERVO_INPUT_POLICY_H
#define SERVO_INPUT_POLICY_H

#include <stddef.h>
#include <stdint.h>

typedef struct
{
    uint32_t authoritySeq;
    uint32_t semanticsSeq;
    uint8_t waitRelease;
} ServoInputGate;

static inline void ServoInputGateInit(ServoInputGate *gate)
{
    if (gate != NULL)
    {
        gate->authoritySeq = 0u;
        gate->semanticsSeq = 0u;
        gate->waitRelease = 1u;
    }
}

static inline void ServoInputGateSync(ServoInputGate *gate,
                                      uint32_t authoritySeq,
                                      uint32_t semanticsSeq)
{
    if (gate != NULL && authoritySeq != 0u && semanticsSeq != 0u &&
        (gate->authoritySeq != authoritySeq || gate->semanticsSeq != semanticsSeq))
    {
        gate->authoritySeq = authoritySeq;
        gate->semanticsSeq = semanticsSeq;
        gate->waitRelease = 1u;
    }
}

/* 只要求四个舵机动作键释放，Shift 等修饰键可以继续保持。 */
static inline uint16_t ServoInputGateApply(ServoInputGate *gate,
                                           uint8_t controlAllowed,
                                           uint16_t actionKeyMask,
                                           uint16_t observedKeys)
{
    if (gate == NULL)
    {
        return 0u;
    }

    if (controlAllowed == 0u)
    {
        gate->waitRelease = 1u;
        return 0u;
    }

    if (gate->waitRelease != 0u)
    {
        if ((observedKeys & actionKeyMask) == 0u)
        {
            gate->waitRelease = 0u;
        }
        return 0u;
    }

    return observedKeys;
}

static inline uint8_t ServoInputGateReady(const ServoInputGate *gate)
{
    return (gate != NULL && gate->waitRelease == 0u) ? 1u : 0u;
}

#endif
