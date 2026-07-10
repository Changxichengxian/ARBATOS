/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CHASSIS_SNAPSHOT_POLICY_H
#define CHASSIS_SNAPSHOT_POLICY_H

#include <stdint.h>
#include <string.h>

#include "CanReceive.h"
#include "GimbalState.h"
#include "LowCmd.h"

#define CHASSIS_SNAPSHOT_MOTOR_COUNT 4u

typedef struct
{
    uint8_t valid;
    uint8_t online;
    uint8_t turnaroundActive;
    uint8_t frameValid;
    uint8_t chassisStop;
    uint8_t followAvailable;
    uint8_t yawRelativeTurn;
    uint32_t yawEcdRange;
    fp32 followOffsetRad;
    fp32 frameYawRelative;
    GimbalMotorState yaw;
    GimbalMotorState pitch;
} ChassisGimbalSnapshot;

typedef struct
{
    MotorId id[CHASSIS_SNAPSHOT_MOTOR_COUNT];
    uint8_t axis[CHASSIS_SNAPSHOT_MOTOR_COUNT];
    uint8_t count;
    uint8_t unconfiguredMask;
    uint8_t invalidMask;
} ChassisMotorReadPlan;

static inline void ChassisGimbalSnapshotBuild(ChassisGimbalSnapshot *out,
                                              const GimbalState *state,
                                              uint8_t readFreshOk)
{
    if (out == NULL)
    {
        return;
    }

    (void)memset(out, 0, sizeof(*out));
    if (readFreshOk == 0u || state == NULL || state->valid == 0u)
    {
        return;
    }

    out->valid = 1u;
    out->online = state->online;
    out->turnaroundActive = state->turnaround_active;
    out->frameValid = state->turnaround_frame_valid;
    out->chassisStop = state->ChassisStop;
    out->followAvailable = state->follow_available;
    out->followOffsetRad = state->turnaround_follow_offset_rad;
    out->frameYawRelative = state->turnaround_frame_yaw_relative;
    out->yaw = state->yaw;
    out->pitch = state->pitch;
}

static inline void ChassisMotorReadPlanBuild(const MotorId axisId[CHASSIS_SNAPSHOT_MOTOR_COUNT],
                                             const uint8_t configured[CHASSIS_SNAPSHOT_MOTOR_COUNT],
                                             const uint8_t bound[CHASSIS_SNAPSHOT_MOTOR_COUNT],
                                             ChassisMotorReadPlan *out)
{
    if (out == NULL)
    {
        return;
    }

    (void)memset(out, 0, sizeof(*out));
    if (axisId == NULL || configured == NULL || bound == NULL)
    {
        out->invalidMask = (uint8_t)((1u << CHASSIS_SNAPSHOT_MOTOR_COUNT) - 1u);
        return;
    }

    for (uint8_t i = 0u; i < CHASSIS_SNAPSHOT_MOTOR_COUNT; i++)
    {
        const uint8_t bit = (uint8_t)(1u << i);

        if (configured[i] == 0u)
        {
            out->unconfiguredMask |= bit;
            continue;
        }
        if (bound[i] == 0u || (uint32_t)axisId[i] >= (uint32_t)MotorCount)
        {
            out->invalidMask |= bit;
            continue;
        }

        out->id[out->count] = axisId[i];
        out->axis[out->count] = i;
        out->count++;
    }
}

static inline void ChassisMotorMeasureFromState(motor_measure_t *out,
                                                const MotorState *feedback,
                                                uint8_t feedbackReadOk)
{
    if (out == NULL)
    {
        return;
    }

    (void)memset(out, 0, sizeof(*out));
    if (feedbackReadOk == 0u || feedback == NULL)
    {
        return;
    }

    out->ecd = feedback->ecd;
    out->speed_rpm = feedback->speedRpm;
    out->given_current = feedback->current;
    out->temperate = feedback->temperature;
    out->last_ecd = (int16_t)feedback->lastEcd;
}

#endif
