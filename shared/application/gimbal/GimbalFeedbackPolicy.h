/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef GIMBAL_FEEDBACK_POLICY_H
#define GIMBAL_FEEDBACK_POLICY_H

#include <stddef.h>
#include <stdint.h>

#define GIMBAL_FEEDBACK_IMU_RECOVERY_MS 200u
#define GIMBAL_ENCODER_TWO_PI            6.28318530718f

typedef enum
{
    GIMBAL_FEEDBACK_IMU_NORMAL = 0u,
    GIMBAL_FEEDBACK_ENCODER_DEGRADED = 1u,
} GimbalFeedbackMode;

typedef struct
{
    uint32_t recoveryStartMs;
    uint32_t transitionSeq;
    uint32_t lastRequiredMask;
    uint32_t lastFallbackMask;
    uint32_t lastDegradedMask;
    uint32_t lastBlockMask;
    uint32_t transitionAxisMask;
    uint8_t mode;
    uint8_t initialized;
    uint8_t recoveryStartValid;
    uint8_t recoveryPending;
    uint8_t transitionZeroPending;
} GimbalFeedbackPolicyState;

typedef struct
{
    uint32_t transitionSeq;
    uint32_t encoderDegradedMask;
    uint32_t feedbackBlockMask;
    uint32_t transitionZeroMask;
    uint8_t mode;
    uint8_t recoveryPending;
    uint8_t modeChanged;
    uint8_t transitionPending;
} GimbalFeedbackPolicyOutput;

typedef struct
{
    uint32_t transitionSeq;
    uint32_t waitMask;
    uint32_t safeInhibitMask;
    uint32_t cmdSeq[3];
    uint32_t cmdTick[3];
    uint8_t motorId[3];
    uint8_t armed;
} GimbalFeedbackZeroBarrier;

typedef struct
{
    uint32_t blockedMask;
    uint32_t publishMask;
    uint32_t waitMask;
    uint32_t cmdSeq[3];
    uint32_t cmdTick[3];
    uint8_t motorId[3];
} GimbalFeedbackRouteDebt;

static inline uint32_t GimbalFeedbackMaskKnown(uint32_t mask)
{
    return mask & 0x07u;
}

static inline void GimbalFeedbackPolicyInit(GimbalFeedbackPolicyState *state)
{
    if (state == NULL)
    {
        return;
    }

    state->recoveryStartMs = 0u;
    state->transitionSeq = 0u;
    state->lastRequiredMask = 0u;
    state->lastFallbackMask = 0u;
    state->lastDegradedMask = 0u;
    state->lastBlockMask = 0u;
    state->transitionAxisMask = 0u;
    state->mode = (uint8_t)GIMBAL_FEEDBACK_IMU_NORMAL;
    state->initialized = 0u;
    state->recoveryStartValid = 0u;
    state->recoveryPending = 0u;
    state->transitionZeroPending = 0u;
}

static inline GimbalFeedbackPolicyOutput GimbalFeedbackPolicyStep(
    GimbalFeedbackPolicyState *state,
    uint32_t nowMs,
    uint8_t imuOnline,
    uint8_t recoveryInputSafe,
    uint32_t imuRequiredMask,
    uint32_t encoderFallbackMask)
{
    GimbalFeedbackPolicyOutput out = {0};
    const uint32_t requiredMask = GimbalFeedbackMaskKnown(imuRequiredMask);
    const uint32_t fallbackMask = GimbalFeedbackMaskKnown(encoderFallbackMask);
    const uint32_t effectiveFallbackMask = fallbackMask & requiredMask;
    uint8_t hadPreviousRoute;

    if (state == NULL)
    {
        out.mode = (uint8_t)GIMBAL_FEEDBACK_ENCODER_DEGRADED;
        out.feedbackBlockMask = requiredMask;
        return out;
    }

    hadPreviousRoute = state->initialized;
    if (state->initialized == 0u)
    {
        state->mode = (imuOnline != 0u) ?
                          (uint8_t)GIMBAL_FEEDBACK_IMU_NORMAL :
                          (uint8_t)GIMBAL_FEEDBACK_ENCODER_DEGRADED;
        state->initialized = 1u;
    }
    else if (state->mode == (uint8_t)GIMBAL_FEEDBACK_IMU_NORMAL)
    {
        if (imuOnline == 0u)
        {
            state->mode = (uint8_t)GIMBAL_FEEDBACK_ENCODER_DEGRADED;
            out.modeChanged = 1u;
        }
    }
    else if (imuOnline == 0u)
    {
        state->recoveryStartMs = 0u;
        state->recoveryStartValid = 0u;
        state->recoveryPending = 0u;
    }
    else
    {
        if (state->recoveryStartValid == 0u)
        {
            state->recoveryStartMs = nowMs;
            state->recoveryStartValid = 1u;
        }
        state->recoveryPending = 1u;
        if ((uint32_t)(nowMs - state->recoveryStartMs) >=
                (uint32_t)GIMBAL_FEEDBACK_IMU_RECOVERY_MS &&
            recoveryInputSafe != 0u)
        {
            state->mode = (uint8_t)GIMBAL_FEEDBACK_IMU_NORMAL;
            state->recoveryStartMs = 0u;
            state->recoveryStartValid = 0u;
            state->recoveryPending = 0u;
            out.modeChanged = 1u;
        }
    }

    out.mode = state->mode;
    out.recoveryPending = state->recoveryPending;
    if (state->mode == (uint8_t)GIMBAL_FEEDBACK_ENCODER_DEGRADED)
    {
        out.encoderDegradedMask = requiredMask & fallbackMask;
        out.feedbackBlockMask = requiredMask & ~fallbackMask;
    }
    if (hadPreviousRoute != 0u &&
        (out.modeChanged != 0u ||
         state->lastRequiredMask != requiredMask ||
         state->lastFallbackMask != effectiveFallbackMask ||
         state->lastDegradedMask != out.encoderDegradedMask ||
         state->lastBlockMask != out.feedbackBlockMask))
    {
        state->transitionSeq++;
        state->transitionZeroPending = 1u;
        state->transitionAxisMask |= state->lastRequiredMask | requiredMask;
    }
    state->lastRequiredMask = requiredMask;
    state->lastFallbackMask = effectiveFallbackMask;
    state->lastDegradedMask = out.encoderDegradedMask;
    state->lastBlockMask = out.feedbackBlockMask;
    if (state->transitionZeroPending != 0u)
    {
        out.transitionSeq = state->transitionSeq;
        out.transitionZeroMask = state->transitionAxisMask;
        out.transitionPending = 1u;
    }
    return out;
}

static inline void GimbalFeedbackZeroBarrierInit(
    GimbalFeedbackZeroBarrier *barrier)
{
    if (barrier == NULL)
    {
        return;
    }

    barrier->transitionSeq = 0u;
    barrier->waitMask = 0u;
    barrier->safeInhibitMask = 0u;
    for (uint8_t axis = 0u; axis < 3u; axis++)
    {
        barrier->cmdSeq[axis] = 0u;
        barrier->cmdTick[axis] = 0u;
        barrier->motorId[axis] = 0u;
    }
    barrier->armed = 0u;
}

static inline uint8_t GimbalFeedbackZeroBarrierExcludeSafe(
    GimbalFeedbackZeroBarrier *barrier,
    uint32_t axisMask)
{
    if (barrier == NULL || barrier->armed == 0u ||
        axisMask == 0u || (axisMask & (axisMask - 1u)) != 0u ||
        (axisMask & 0x07u) == 0u)
    {
        return 0u;
    }
    barrier->safeInhibitMask |= axisMask;
    return 1u;
}

static inline void GimbalFeedbackRouteDebtInit(
    GimbalFeedbackRouteDebt *debt)
{
    if (debt == NULL)
    {
        return;
    }

    debt->blockedMask = 0u;
    debt->publishMask = 0u;
    debt->waitMask = 0u;
    for (uint8_t axis = 0u; axis < 3u; axis++)
    {
        debt->cmdSeq[axis] = 0u;
        debt->cmdTick[axis] = 0u;
        debt->motorId[axis] = 0u;
    }
}

static inline uint32_t GimbalFeedbackRouteDebtMask(
    const GimbalFeedbackRouteDebt *debt)
{
    return (debt != NULL) ?
               GimbalFeedbackMaskKnown(debt->blockedMask |
                                       debt->publishMask |
                                       debt->waitMask) :
               0u;
}

static inline uint8_t GimbalFeedbackRouteDebtHold(
    GimbalFeedbackRouteDebt *debt,
    uint32_t axisMask,
    uint8_t motorId,
    uint32_t cmdSeq,
    uint32_t cmdTick)
{
    uint8_t axis;

    if (debt == NULL ||
        axisMask == 0u || (axisMask & (axisMask - 1u)) != 0u ||
        (axisMask & 0x07u) == 0u)
    {
        return 0u;
    }
    axis = (axisMask == 0x01u) ? 0u : ((axisMask == 0x02u) ? 1u : 2u);
    debt->blockedMask |= axisMask;
    debt->publishMask &= ~axisMask;
    debt->waitMask &= ~axisMask;
    debt->motorId[axis] = motorId;
    debt->cmdSeq[axis] = cmdSeq;
    debt->cmdTick[axis] = cmdTick;
    return 1u;
}

static inline uint8_t GimbalFeedbackRouteDebtRequestPublish(
    GimbalFeedbackRouteDebt *debt,
    uint32_t axisMask,
    uint8_t motorId)
{
    uint8_t axis;

    if (debt == NULL ||
        axisMask == 0u || (axisMask & (axisMask - 1u)) != 0u ||
        (axisMask & 0x07u) == 0u)
    {
        return 0u;
    }
    axis = (axisMask == 0x01u) ? 0u : ((axisMask == 0x02u) ? 1u : 2u);
    debt->blockedMask &= ~axisMask;
    debt->publishMask |= axisMask;
    debt->waitMask &= ~axisMask;
    debt->motorId[axis] = motorId;
    debt->cmdSeq[axis] = 0u;
    debt->cmdTick[axis] = 0u;
    return 1u;
}

static inline uint8_t GimbalFeedbackRouteDebtWait(
    GimbalFeedbackRouteDebt *debt,
    uint32_t axisMask,
    uint8_t motorId,
    uint32_t cmdSeq,
    uint32_t cmdTick)
{
    uint8_t axis;

    if (debt == NULL ||
        axisMask == 0u || (axisMask & (axisMask - 1u)) != 0u ||
        (axisMask & 0x07u) == 0u)
    {
        return 0u;
    }
    axis = (axisMask == 0x01u) ? 0u : ((axisMask == 0x02u) ? 1u : 2u);
    debt->blockedMask &= ~axisMask;
    debt->publishMask &= ~axisMask;
    debt->waitMask |= axisMask;
    debt->motorId[axis] = motorId;
    debt->cmdSeq[axis] = cmdSeq;
    debt->cmdTick[axis] = cmdTick;
    return 1u;
}

static inline uint8_t GimbalFeedbackRouteDebtComplete(
    GimbalFeedbackRouteDebt *debt,
    uint32_t axisMask)
{
    uint8_t axis;

    if (debt == NULL ||
        axisMask == 0u || (axisMask & (axisMask - 1u)) != 0u ||
        (axisMask & 0x07u) == 0u)
    {
        return 0u;
    }
    axis = (axisMask == 0x01u) ? 0u : ((axisMask == 0x02u) ? 1u : 2u);
    debt->blockedMask &= ~axisMask;
    debt->publishMask &= ~axisMask;
    debt->waitMask &= ~axisMask;
    debt->motorId[axis] = 0u;
    debt->cmdSeq[axis] = 0u;
    debt->cmdTick[axis] = 0u;
    return 1u;
}

static inline void GimbalFeedbackZeroBarrierBegin(
    GimbalFeedbackZeroBarrier *barrier,
    uint32_t transitionSeq)
{
    GimbalFeedbackZeroBarrierInit(barrier);
    if (barrier != NULL)
    {
        barrier->transitionSeq = transitionSeq;
        barrier->armed = 1u;
    }
}

static inline uint8_t GimbalFeedbackZeroBarrierAdd(
    GimbalFeedbackZeroBarrier *barrier,
    uint32_t axisMask,
    uint8_t motorId,
    uint32_t cmdSeq,
    uint32_t cmdTick)
{
    uint8_t axis;

    if (barrier == NULL || barrier->armed == 0u ||
        axisMask == 0u || (axisMask & (axisMask - 1u)) != 0u ||
        (axisMask & 0x07u) == 0u)
    {
        return 0u;
    }
    axis = (axisMask == 0x01u) ? 0u : ((axisMask == 0x02u) ? 1u : 2u);
    barrier->waitMask |= axisMask;
    barrier->motorId[axis] = motorId;
    barrier->cmdSeq[axis] = cmdSeq;
    barrier->cmdTick[axis] = cmdTick;
    return 1u;
}

static inline uint8_t GimbalFeedbackZeroBarrierMatches(
    const GimbalFeedbackZeroBarrier *barrier,
    uint32_t transitionSeq)
{
    return (uint8_t)(barrier != NULL &&
                     barrier->armed != 0u &&
                     barrier->transitionSeq == transitionSeq);
}

static inline void GimbalFeedbackPolicyConsumeTransition(
    GimbalFeedbackPolicyState *state)
{
    if (state != NULL)
    {
        state->transitionZeroPending = 0u;
        state->transitionAxisMask = 0u;
    }
}

static inline int32_t GimbalEncoderRelativeCount(uint16_t ecd,
                                                  uint16_t offsetEcd,
                                                  uint32_t encoderRange)
{
    int32_t relative;
    const int32_t fullRange = (int32_t)encoderRange;
    const int32_t halfRange = fullRange / 2;

    if (encoderRange < 2u || encoderRange > 65536u)
    {
        return 0;
    }

    relative = (int32_t)ecd - (int32_t)offsetEcd;
    if (relative > halfRange)
    {
        relative -= fullRange;
    }
    else if (relative < -halfRange)
    {
        relative += fullRange;
    }
    return relative;
}

static inline uint16_t GimbalEncoderNormalizeCount(int32_t count,
                                                    uint32_t encoderRange)
{
    int32_t normalized;

    if (encoderRange < 2u || encoderRange > 65536u)
    {
        return 0u;
    }
    normalized = count % (int32_t)encoderRange;
    if (normalized < 0)
    {
        normalized += (int32_t)encoderRange;
    }
    return (uint16_t)normalized;
}

static inline float GimbalEncoderAngleRad(uint16_t ecd,
                                           uint16_t offsetEcd,
                                           uint32_t encoderRange)
{
    if (encoderRange < 2u || encoderRange > 65536u)
    {
        return 0.0f;
    }
    return (float)GimbalEncoderRelativeCount(ecd, offsetEcd, encoderRange) *
           (GIMBAL_ENCODER_TWO_PI / (float)encoderRange);
}

#endif
