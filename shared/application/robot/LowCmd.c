/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "LowCmd.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stddef.h>
#include <string.h>

static MotorCmd gMotorCmd[MotorCount];
static MotorState gMotorState[MotorCount];
static uint32_t gLowCmdSeq;
static uint32_t gLowStateTick;

static uint32_t LowCmdNowMs(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static uint8_t MotorIdValid(MotorId id)
{
    return ((uint32_t)id < (uint32_t)MotorCount) ? 1u : 0u;
}

static uint8_t MotorCmdValid(const MotorCmd *cmd)
{
    return (uint8_t)(cmd != NULL && MotorModeKnown(cmd->mode));
}

static void LowCmdStamp(MotorCmd *cmd, uint32_t seq, uint32_t tick)
{
    if (cmd == NULL)
    {
        return;
    }

    cmd->seq = seq;
    cmd->tick = tick;
}

static void LowCmdSetMotorMany_unchecked(const MotorId *ids, const MotorCmd *cmds, uint8_t count)
{
    const uint32_t tick = LowCmdNowMs();

    taskENTER_CRITICAL();
    gLowCmdSeq++;
    for (uint8_t i = 0u; i < count; i++)
    {
        gMotorCmd[ids[i]] = cmds[i];
        LowCmdStamp(&gMotorCmd[ids[i]], gLowCmdSeq, tick);
    }
    taskEXIT_CRITICAL();
}

void LowCmdClearAll(void)
{
    taskENTER_CRITICAL();
    (void)memset(gMotorCmd, 0, sizeof(gMotorCmd));
    gLowCmdSeq++;
    taskEXIT_CRITICAL();
}

void LowCmdClear(MotorId id)
{
    if (MotorIdValid(id) == 0u)
    {
        return;
    }

    taskENTER_CRITICAL();
    (void)memset(&gMotorCmd[id], 0, sizeof(gMotorCmd[id]));
    gMotorCmd[id].seq = ++gLowCmdSeq;
    gMotorCmd[id].tick = LowCmdNowMs();
    taskEXIT_CRITICAL();
}

uint8_t LowCmdSetMotorMany(const MotorId *ids, const MotorCmd *cmds, uint8_t count)
{
    if (count > (uint8_t)MotorCount)
    {
        return 0u;
    }
    if (count != 0u && (ids == NULL || cmds == NULL))
    {
        return 0u;
    }

    for (uint8_t i = 0u; i < count; i++)
    {
        if (MotorIdValid(ids[i]) == 0u || MotorCmdValid(&cmds[i]) == 0u)
        {
            return 0u;
        }
    }

    LowCmdSetMotorMany_unchecked(ids, cmds, count);
    return 1u;
}

uint8_t LowCmdSetMotor(MotorId id, const MotorCmd *cmd)
{
    return LowCmdSetMotorMany(&id, cmd, 1u);
}

const char *MotorModeName(MotorMode mode)
{
    switch (mode)
    {
    case MotorModeNone:
        return "none";
    case MotorModeDisable:
        return "disable";
    case MotorModeDamping:
        return "damping";
    case MotorModeCurrent:
        return "current";
    case MotorModeStateTorque:
        return "state_torque";
    case MotorModePosVel:
        return "pos_vel";
    case MotorModeSpeed:
        return "speed";
    case MotorModeForcePos:
        return "force_pos";
    default:
        return "unknown";
    }
}

uint32_t LowCmdSeq(void)
{
    uint32_t seq;

    taskENTER_CRITICAL();
    seq = gLowCmdSeq;
    taskEXIT_CRITICAL();
    return seq;
}

uint8_t LowCmdGet(LowCmd *out)
{
    if (out == NULL)
    {
        return 0u;
    }

    taskENTER_CRITICAL();
    out->seq = gLowCmdSeq;
    out->tick = LowCmdNowMs();
    (void)memcpy(out->motorCmd, gMotorCmd, sizeof(gMotorCmd));
    taskEXIT_CRITICAL();
    return 1u;
}

void LowCmdSetCurrent(MotorId id, int16_t current)
{
    MotorCmd cmd;

    (void)memset(&cmd, 0, sizeof(cmd));
    cmd.active = 1u;
    cmd.mode = (uint8_t)MotorModeCurrent;
    cmd.current = current;
    (void)LowCmdSetMotor(id, &cmd);
}

uint8_t LowCmdSetCurrentMany(const MotorId *ids, const int16_t *currents, uint8_t count)
{
    MotorCmd cmds[MotorCount];

    if (count > (uint8_t)MotorCount)
    {
        return 0u;
    }
    if (count != 0u && (ids == NULL || currents == NULL))
    {
        return 0u;
    }

    for (uint8_t i = 0u; i < count; i++)
    {
        if (MotorIdValid(ids[i]) == 0u)
        {
            return 0u;
        }

        (void)memset(&cmds[i], 0, sizeof(cmds[i]));
        cmds[i].active = 1u;
        cmds[i].mode = (uint8_t)MotorModeCurrent;
        cmds[i].current = currents[i];
    }

    LowCmdSetMotorMany_unchecked(ids, cmds, count);
    return 1u;
}

int16_t LowCmdGetCurrent(MotorId id)
{
    int16_t current;

    if (MotorIdValid(id) == 0u)
    {
        return 0;
    }

    taskENTER_CRITICAL();
    current = gMotorCmd[id].current;
    taskEXIT_CRITICAL();
    return current;
}

uint8_t LowCmdGetCurrentMany(const MotorId *ids, int16_t *out, uint8_t count)
{
    if (count != 0u && (ids == NULL || out == NULL))
    {
        return 0u;
    }

    for (uint8_t i = 0u; i < count; i++)
    {
        if (MotorIdValid(ids[i]) == 0u)
        {
            return 0u;
        }
    }

    taskENTER_CRITICAL();
    for (uint8_t i = 0u; i < count; i++)
    {
        out[i] = gMotorCmd[ids[i]].current;
    }
    taskEXIT_CRITICAL();
    return 1u;
}

void LowCmdSetStateTorque(MotorId id, const MotorCmd *cmd)
{
    MotorCmd tmp;

    if (MotorIdValid(id) == 0u || cmd == NULL)
    {
        return;
    }

    tmp = *cmd;
    tmp.active = 1u;
    tmp.mode = (uint8_t)MotorModeStateTorque;
    (void)LowCmdSetMotor(id, &tmp);
}

void LowCmdSetSpeed(MotorId id, fp32 velocity, fp32 kd, fp32 torque)
{
    MotorCmd cmd;

    (void)memset(&cmd, 0, sizeof(cmd));
    cmd.active = 1u;
    cmd.mode = (uint8_t)MotorModeSpeed;
    cmd.dq = velocity;
    cmd.kd = kd;
    cmd.tau = torque;
    (void)LowCmdSetMotor(id, &cmd);
}

uint8_t LowCmdGetMotor(MotorId id, MotorCmd *out)
{
    if (MotorIdValid(id) == 0u || out == NULL)
    {
        return 0u;
    }

    taskENTER_CRITICAL();
    *out = gMotorCmd[id];
    taskEXIT_CRITICAL();
    return 1u;
}

uint8_t LowCmdGetMotorMany(const MotorId *ids, MotorCmd *out, uint8_t count)
{
    if (count != 0u && (ids == NULL || out == NULL))
    {
        return 0u;
    }

    for (uint8_t i = 0u; i < count; i++)
    {
        if (MotorIdValid(ids[i]) == 0u)
        {
            return 0u;
        }
    }

    taskENTER_CRITICAL();
    for (uint8_t i = 0u; i < count; i++)
    {
        out[i] = gMotorCmd[ids[i]];
    }
    taskEXIT_CRITICAL();
    return 1u;
}

const MotorCmd *LowCmdGetMotorPtr(MotorId id)
{
    if (MotorIdValid(id) == 0u)
    {
        return NULL;
    }

    return &gMotorCmd[id];
}

void LowStateClearAll(void)
{
    taskENTER_CRITICAL();
    (void)memset(gMotorState, 0, sizeof(gMotorState));
    gLowStateTick = LowCmdNowMs();
    taskEXIT_CRITICAL();
}

void LowStateUpdateMotor(MotorId id, const MotorState *feedback)
{
    if (MotorIdValid(id) == 0u || feedback == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    gMotorState[id] = *feedback;
    gLowStateTick = feedback->lastRxTick;
    taskEXIT_CRITICAL();
}

uint8_t LowStateGet(LowState *out)
{
    if (out == NULL)
    {
        return 0u;
    }

    taskENTER_CRITICAL();
    out->tick = gLowStateTick;
    (void)memcpy(out->motorState, gMotorState, sizeof(gMotorState));
    taskEXIT_CRITICAL();
    return 1u;
}

uint8_t LowStateGetMotor(MotorId id, MotorState *out)
{
    if (MotorIdValid(id) == 0u || out == NULL)
    {
        return 0u;
    }

    taskENTER_CRITICAL();
    *out = gMotorState[id];
    taskEXIT_CRITICAL();
    return 1u;
}

uint8_t LowStateGetMotorMany(const MotorId *ids, MotorState *out, uint8_t count)
{
    if (count != 0u && (ids == NULL || out == NULL))
    {
        return 0u;
    }

    for (uint8_t i = 0u; i < count; i++)
    {
        if (MotorIdValid(ids[i]) == 0u)
        {
            return 0u;
        }
    }

    taskENTER_CRITICAL();
    for (uint8_t i = 0u; i < count; i++)
    {
        out[i] = gMotorState[ids[i]];
    }
    taskEXIT_CRITICAL();
    return 1u;
}

const MotorState *LowStateGetMotorPtr(MotorId id)
{
    if (MotorIdValid(id) == 0u)
    {
        return NULL;
    }

    return &gMotorState[id];
}
