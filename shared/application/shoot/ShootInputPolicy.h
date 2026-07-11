/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SHOOT_INPUT_POLICY_H
#define SHOOT_INPUT_POLICY_H

#include <stdint.h>

#define SHOOT_INPUT_REARM_LEFT  (1u << 0)
#define SHOOT_INPUT_REARM_RIGHT (1u << 1)

typedef struct
{
    uint16_t lastSwitch;
    uint32_t semanticsSeq;
    uint32_t actionSeq;
    uint8_t initialized;
    uint8_t fireEngaged;
    uint8_t mouseRearmMask;
} ShootInputGateState;

static inline void ShootInputGateReset(ShootInputGateState *state, uint16_t stopRaw)
{
    if (state == 0)
    {
        return;
    }

    state->initialized = 0u;
    state->fireEngaged = 0u;
    state->lastSwitch = stopRaw;
}

static inline void ShootInputGateBlockMouse(ShootInputGateState *state)
{
    if (state != 0)
    {
        state->mouseRearmMask = SHOOT_INPUT_REARM_LEFT | SHOOT_INPUT_REARM_RIGHT;
    }
}

static inline void ShootInputGateRequireRearm(ShootInputGateState *state,
                                               uint16_t stopRaw)
{
    ShootInputGateReset(state, stopRaw);
    ShootInputGateBlockMouse(state);
}

static inline uint8_t ShootInputGateSyncSemantics(ShootInputGateState *state,
                                                   uint32_t semanticsSeq,
                                                   uint16_t stopRaw)
{
    if (state == 0 || semanticsSeq == 0u || state->semanticsSeq == semanticsSeq)
    {
        return 0u;
    }

    state->semanticsSeq = semanticsSeq;
    ShootInputGateRequireRearm(state, stopRaw);
    return 1u;
}

static inline uint8_t ShootInputGateSyncAction(ShootInputGateState *state,
                                                uint32_t actionSeq,
                                                uint16_t stopRaw)
{
    if (state == 0 || actionSeq == 0u || state->actionSeq == actionSeq)
    {
        return 0u;
    }

    state->actionSeq = actionSeq;
    ShootInputGateRequireRearm(state, stopRaw);
    return 1u;
}

static inline uint16_t ShootInputGateSwitch(ShootInputGateState *state,
                                            uint16_t rawSwitch,
                                            uint8_t manualOnline,
                                            uint16_t stopRaw,
                                            uint16_t readyRaw,
                                            uint16_t fireRaw)
{
    uint16_t effectiveSwitch = rawSwitch;

    if (state == 0 || manualOnline == 0u)
    {
        ShootInputGateReset(state, stopRaw);
        return stopRaw;
    }

    if (state->initialized == 0u)
    {
        state->initialized = 1u;
        state->fireEngaged = 0u;
        state->lastSwitch = rawSwitch;
    }

    if (rawSwitch != fireRaw)
    {
        state->fireEngaged = 0u;
    }
    else if (state->lastSwitch != fireRaw)
    {
        state->fireEngaged = 1u;
    }

    if (rawSwitch == fireRaw && state->fireEngaged == 0u)
    {
        effectiveSwitch = readyRaw;
    }

    state->lastSwitch = rawSwitch;
    return effectiveSwitch;
}

static inline void ShootInputGateSyncSafeMouse(ShootInputGateState *state,
                                                uint8_t manualOnline,
                                                uint8_t pressLeft,
                                                uint8_t pressRight)
{
    if (state == 0)
    {
        return;
    }
    if (manualOnline == 0u)
    {
        ShootInputGateBlockMouse(state);
        return;
    }

    if (pressLeft != 0u)
    {
        state->mouseRearmMask |= SHOOT_INPUT_REARM_LEFT;
    }
    else
    {
        state->mouseRearmMask &= (uint8_t)~SHOOT_INPUT_REARM_LEFT;
    }
    if (pressRight != 0u)
    {
        state->mouseRearmMask |= SHOOT_INPUT_REARM_RIGHT;
    }
    else
    {
        state->mouseRearmMask &= (uint8_t)~SHOOT_INPUT_REARM_RIGHT;
    }
}

static inline void ShootInputGateApplyMouse(ShootInputGateState *state,
                                             uint8_t *pressLeft,
                                             uint8_t *pressRight)
{
    if (state == 0 || pressLeft == 0 || pressRight == 0)
    {
        return;
    }

    if ((state->mouseRearmMask & SHOOT_INPUT_REARM_LEFT) != 0u)
    {
        if (*pressLeft == 0u)
        {
            state->mouseRearmMask &= (uint8_t)~SHOOT_INPUT_REARM_LEFT;
        }
        *pressLeft = 0u;
    }
    if ((state->mouseRearmMask & SHOOT_INPUT_REARM_RIGHT) != 0u)
    {
        if (*pressRight == 0u)
        {
            state->mouseRearmMask &= (uint8_t)~SHOOT_INPUT_REARM_RIGHT;
        }
        *pressRight = 0u;
    }
}

static inline void ShootInputGateApplyFrameMouse(ShootInputGateState *state,
                                                  uint8_t manualOnline,
                                                  uint8_t *pressLeft,
                                                  uint8_t *pressRight)
{
    if (state == 0 || pressLeft == 0 || pressRight == 0)
    {
        return;
    }
    if (manualOnline == 0u)
    {
        /* 离线的零值不能证明按键已释放，必须保守保持重启锁。 */
        ShootInputGateBlockMouse(state);
        *pressLeft = 0u;
        *pressRight = 0u;
        return;
    }

    ShootInputGateApplyMouse(state, pressLeft, pressRight);
}

static inline uint16_t ShootInputGateBlockFireFrame(ShootInputGateState *state,
                                                     uint16_t rawSwitch,
                                                     uint8_t manualOnline,
                                                     uint16_t stopRaw,
                                                     uint16_t readyRaw,
                                                     uint16_t fireRaw,
                                                     uint8_t *pressLeft,
                                                     uint8_t *pressRight)
{
    ShootInputGateRequireRearm(state, stopRaw);
    if (pressLeft != 0)
    {
        *pressLeft = 0u;
    }
    if (pressRight != 0)
    {
        *pressRight = 0u;
    }

    return ShootInputGateSwitch(state,
                                rawSwitch,
                                manualOnline,
                                stopRaw,
                                readyRaw,
                                fireRaw);
}

#endif
