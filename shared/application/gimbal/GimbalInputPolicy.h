/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-07-10
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef GIMBAL_INPUT_POLICY_H
#define GIMBAL_INPUT_POLICY_H

#include <stddef.h>
#include <stdint.h>

typedef struct
{
    uint32_t authoritySeq;
    uint32_t semanticsSeq;
    uint8_t waitTurnRelease;
} GimbalInputGate;

/* 返回非零表示旧的 180 度动作必须立即取消。 */
static inline uint8_t GimbalInputGateApplyTurn(GimbalInputGate *gate,
                                               uint32_t authoritySeq,
                                               uint32_t semanticsSeq,
                                               uint8_t controlAllowed,
                                               uint8_t observedTurnDown,
                                               uint8_t *effectiveTurnDown)
{
    uint8_t cancelTurn = 0u;

    if (effectiveTurnDown != NULL)
    {
        *effectiveTurnDown = 0u;
    }
    if (gate == NULL || effectiveTurnDown == NULL)
    {
        return 1u;
    }

    if (controlAllowed == 0u || authoritySeq == 0u || semanticsSeq == 0u)
    {
        gate->waitTurnRelease = 1u;
        return 1u;
    }
    if (gate->authoritySeq != authoritySeq || gate->semanticsSeq != semanticsSeq)
    {
        gate->authoritySeq = authoritySeq;
        gate->semanticsSeq = semanticsSeq;
        gate->waitTurnRelease = 1u;
        cancelTurn = 1u;
    }

    if (gate->waitTurnRelease != 0u)
    {
        if (observedTurnDown == 0u)
        {
            gate->waitTurnRelease = 0u;
        }
        return cancelTurn;
    }

    *effectiveTurnDown = observedTurnDown;
    return cancelTurn;
}

static inline void GimbalInputGateBlock(GimbalInputGate *gate)
{
    if (gate != NULL)
    {
        gate->waitTurnRelease = 1u;
    }
}

#endif
