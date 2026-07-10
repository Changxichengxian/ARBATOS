/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MotorHealth.h"

#include <string.h>

typedef char MotorHealthMotorCountFits[
    ((uint32_t)MotorCount <= (uint32_t)MOTOR_HEALTH_MAX_MOTORS) ? 1 : -1];

static uint8_t MotorHealthIdValid(MotorId id)
{
    return ((uint32_t)id < (uint32_t)MotorCount) ? 1u : 0u;
}

static void MotorHealthResultInit(MotorId id, MotorHealthResult *out)
{
    (void)memset(out, 0, sizeof(*out));
    out->motorId = id;
    out->ageMs = MOTOR_HEALTH_AGE_UNKNOWN;
}

static void MotorHealthMarkReadFailed(MotorHealthResult *out)
{
    if (out == NULL)
    {
        return;
    }

    out->reasonMask |= (uint16_t)MOTOR_HEALTH_REASON_READ_FAILED;
    out->healthy = 0u;
}

uint8_t MotorHealthEval(MotorId id,
                        const MotorState *feedback,
                        uint32_t nowMs,
                        uint32_t timeoutMs,
                        MotorHealthResult *out)
{
    if (out == NULL)
    {
        return 0u;
    }

    MotorHealthResultInit(id, out);
    if (MotorHealthIdValid(id) == 0u)
    {
        out->reasonMask = (uint16_t)MOTOR_HEALTH_REASON_INVALID_ID;
        return 0u;
    }

    if (feedback == NULL)
    {
        out->reasonMask = (uint16_t)MOTOR_HEALTH_REASON_NO_FEEDBACK;
        return 1u;
    }

    out->feedback = *feedback;
    out->feedbackValid = 1u;

    if (feedback->rxCount == 0u)
    {
        out->reasonMask |= (uint16_t)MOTOR_HEALTH_REASON_NO_FEEDBACK;
    }
    else
    {
        /* 无符号减法允许系统毫秒计数自然回绕。 */
        out->ageMs = nowMs - feedback->lastRxTick;
        /*
         * 调用方通常先取 nowMs 再读反馈；接收中断可能在两者之间把
         * lastRxTick 更新到下一毫秒。只容忍这一毫秒竞态，不能把更大
         * 的时间错误或真实长期陈旧反馈误判为健康。
         */
        if (out->ageMs == UINT32_MAX)
        {
            out->ageMs = 0u;
        }
        if (out->ageMs > timeoutMs)
        {
            out->reasonMask |= (uint16_t)MOTOR_HEALTH_REASON_TIMEOUT;
        }
        else
        {
            out->fresh = 1u;
        }
    }

    if (feedback->driveState == (uint8_t)MotorDriveStateOffline)
    {
        out->reasonMask |= (uint16_t)MOTOR_HEALTH_REASON_DRIVE_OFFLINE;
    }
    else if (feedback->driveState == (uint8_t)MotorDriveStateFault)
    {
        out->reasonMask |= (uint16_t)MOTOR_HEALTH_REASON_DRIVE_FAULT;
    }

    out->healthy = (out->reasonMask == (uint16_t)MOTOR_HEALTH_REASON_NONE) ? 1u : 0u;
    return 1u;
}

uint8_t MotorHealthEvalMany(const MotorId *ids,
                            const MotorState *feedback,
                            uint8_t count,
                            uint32_t nowMs,
                            uint32_t timeoutMs,
                            MotorHealthBatch *out)
{
    uint8_t allValid = 1u;

    if (out == NULL)
    {
        return 0u;
    }

    (void)memset(out, 0, sizeof(*out));
    if (count > (uint8_t)MOTOR_HEALTH_MAX_MOTORS ||
        (count != 0u && (ids == NULL || feedback == NULL)))
    {
        return 0u;
    }

    out->count = count;
    for (uint8_t i = 0u; i < count; i++)
    {
        if (MotorHealthEval(ids[i], &feedback[i], nowMs, timeoutMs, &out->item[i]) == 0u)
        {
            allValid = 0u;
        }
        if (out->item[i].healthy == 0u)
        {
            out->faultMask |= (uint32_t)(1u << i);
        }
    }

    return allValid;
}

uint8_t MotorHealthRead(MotorId id,
                        uint32_t nowMs,
                        uint32_t timeoutMs,
                        MotorHealthResult *out)
{
    MotorState feedback;

    if (out == NULL)
    {
        return 0u;
    }
    if (MotorHealthIdValid(id) == 0u)
    {
        (void)MotorHealthEval(id, NULL, nowMs, timeoutMs, out);
        return 0u;
    }

    if (LowStateGetMotor(id, &feedback) == 0u)
    {
        (void)MotorHealthEval(id, NULL, nowMs, timeoutMs, out);
        MotorHealthMarkReadFailed(out);
        return 0u;
    }

    return MotorHealthEval(id, &feedback, nowMs, timeoutMs, out);
}

uint8_t MotorHealthReadMany(const MotorId *ids,
                            uint8_t count,
                            uint32_t nowMs,
                            uint32_t timeoutMs,
                            MotorHealthBatch *out)
{
    uint8_t allRead = 1u;

    if (out == NULL)
    {
        return 0u;
    }

    (void)memset(out, 0, sizeof(*out));
    if (count > (uint8_t)MOTOR_HEALTH_MAX_MOTORS || (count != 0u && ids == NULL))
    {
        return 0u;
    }

    out->count = count;
    for (uint8_t i = 0u; i < count; i++)
    {
        if (MotorHealthRead(ids[i], nowMs, timeoutMs, &out->item[i]) == 0u)
        {
            allRead = 0u;
        }
        if (out->item[i].healthy == 0u)
        {
            out->faultMask |= (uint32_t)(1u << i);
        }
    }

    return allRead;
}
