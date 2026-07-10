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
#include "main.h"

#include <stddef.h>
#include <string.h>

static MotorCmd gMotorCmd[MotorCount];
static uint16_t gLowCmdInhibitWriter[MotorCount];
static MotorState gMotorState[MotorCount];
static MotorApplied gMotorApplied[MotorCount];
static MotorCmd gMotorCmdSnapshot[MotorCount];
static MotorState gMotorStateSnapshot[MotorCount];
static MotorApplied gMotorAppliedSnapshot[MotorCount];
static uint32_t gLowCmdSeq;
static uint32_t gLowStateTick;
static LowCmdDiag gLowCmdDiag;

#define LOWCMD_SNAPSHOT_RETRY_COUNT 3u

typedef struct
{
    uint8_t from_isr;
    UBaseType_t saved_mask;
} LowCmdCriticalState;

static uint32_t LowCmdNowMs(void)
{
    return HAL_GetTick();
}

static LowCmdCriticalState LowCmdEnterCritical(void)
{
    LowCmdCriticalState state;

    state.from_isr = (__get_IPSR() != 0U) ? 1u : 0u;
    state.saved_mask = 0u;
    if (state.from_isr != 0u)
    {
        state.saved_mask = taskENTER_CRITICAL_FROM_ISR();
    }
    else
    {
        taskENTER_CRITICAL();
    }
    return state;
}

static void LowCmdExitCritical(LowCmdCriticalState state)
{
    if (state.from_isr != 0u)
    {
        taskEXIT_CRITICAL_FROM_ISR(state.saved_mask);
    }
    else
    {
        taskEXIT_CRITICAL();
    }
}

static uint8_t MotorIdValid(MotorId id)
{
    return ((uint32_t)id < (uint32_t)MotorCount) ? 1u : 0u;
}

static uint8_t MotorCmdValid(const MotorCmd *cmd)
{
    return (uint8_t)(cmd != NULL && MotorModeKnown(cmd->mode));
}

/*
 * 每次只在临界区内复制一个电机命令，把连续关中断时间限制在一个
 * MotorCmd 的大小。批量写每次都会更新 gLowCmdSeq；读完后序号不变，
 * 才说明这一批命令来自同一个一致快照。持续竞争时保留整批锁内兜底，
 * 避免高频写入让发送任务长期拿不到命令。
 */
static uint32_t LowCmdCopyMotorSnapshot(const MotorId *ids, MotorCmd *out, uint8_t count)
{
    uint32_t seq_begin = 0u;
    uint32_t seq_end = 0u;

    if (count == 0u)
    {
        LowCmdCriticalState critical = LowCmdEnterCritical();
        seq_end = gLowCmdSeq;
        LowCmdExitCritical(critical);
        return seq_end;
    }

    for (uint8_t attempt = 0u; attempt < LOWCMD_SNAPSHOT_RETRY_COUNT; attempt++)
    {
        for (uint8_t i = 0u; i < count; i++)
        {
            const MotorId id = (ids != NULL) ? ids[i] : (MotorId)i;
            LowCmdCriticalState critical = LowCmdEnterCritical();

            if (i == 0u)
            {
                seq_begin = gLowCmdSeq;
            }
            out[i] = gMotorCmd[id];
            if (i == (uint8_t)(count - 1u))
            {
                seq_end = gLowCmdSeq;
            }
            LowCmdExitCritical(critical);
        }

        if (seq_begin == seq_end)
        {
            return seq_end;
        }

        {
            LowCmdCriticalState critical = LowCmdEnterCritical();
            gLowCmdDiag.snapshot_retry_count++;
            LowCmdExitCritical(critical);
        }
    }

    {
        LowCmdCriticalState critical = LowCmdEnterCritical();
        gLowCmdDiag.snapshot_fallback_count++;
        seq_end = gLowCmdSeq;
        for (uint8_t i = 0u; i < count; i++)
        {
            const MotorId id = (ids != NULL) ? ids[i] : (MotorId)i;
            out[i] = gMotorCmd[id];
        }
        LowCmdExitCritical(critical);
    }
    return seq_end;
}

static uint16_t LowCmdWriterOrDefault(uint16_t writer)
{
    return (writer == (uint16_t)LOWCMD_WRITER_NONE) ? (uint16_t)LOWCMD_WRITER_CONTROL : writer;
}

static uint8_t LowCmdWriterValid(uint16_t writer)
{
    switch (writer)
    {
    case (uint16_t)LOWCMD_WRITER_CONTROL:
    case (uint16_t)LOWCMD_WRITER_MANUAL:
    case (uint16_t)LOWCMD_WRITER_HOST:
    case (uint16_t)LOWCMD_WRITER_SAFETY:
    case (uint16_t)LOWCMD_WRITER_FAULT:
        return 1u;
    default:
        return 0u;
    }
}

static uint8_t LowCmdModeSafe(uint8_t mode)
{
    return (uint8_t)(mode == (uint8_t)MotorModeDisable ||
                     mode == (uint8_t)MotorModeDamping ||
                     mode == (uint8_t)MotorModeNone);
}

static uint8_t MotorModeNeedsDefaultTimeout(uint8_t mode)
{
    switch ((MotorMode)mode)
    {
    case MotorModeCurrent:
    case MotorModeStateTorque:
    case MotorModePosVel:
    case MotorModeSpeed:
    case MotorModeForcePos:
        return 1u;
    default:
        return 0u;
    }
}

static void LowCmdStamp(MotorCmd *cmd, uint32_t seq, uint32_t tick)
{
    if (cmd == NULL)
    {
        return;
    }

    cmd->seq = seq;
    cmd->tick = tick;
    if (cmd->timeoutMs == 0u && MotorModeNeedsDefaultTimeout(cmd->mode) != 0u)
    {
        cmd->timeoutMs = (uint16_t)LOWCMD_DEFAULT_TIMEOUT_MS;
    }
}

static void LowCmdRecordReject(uint16_t writer, uint16_t owner, uint32_t tick)
{
    gLowCmdDiag.rejected_count++;
    gLowCmdDiag.last_reject_tick = tick;
    gLowCmdDiag.last_reject_writer = writer;
    gLowCmdDiag.last_reject_owner = owner;
}

static uint8_t LowCmdCanWriteLocked(MotorId id, uint16_t writer, const MotorCmd *cmd, uint32_t tick)
{
    const MotorCmd *owner = &gMotorCmd[id];
    const uint16_t inhibit_writer = gLowCmdInhibitWriter[id];

    if (gLowCmdDiag.emergency_active != 0u &&
        writer < gLowCmdDiag.emergency_writer &&
        (cmd == NULL || LowCmdModeSafe(cmd->mode) == 0u))
    {
        LowCmdRecordReject(writer, gLowCmdDiag.emergency_writer, tick);
        return 0u;
    }

    if (inhibit_writer != (uint16_t)LOWCMD_WRITER_NONE && writer < inhibit_writer)
    {
        LowCmdRecordReject(writer, inhibit_writer, tick);
        return 0u;
    }

    if (owner->active != 0u &&
        owner->writer != (uint16_t)LOWCMD_WRITER_NONE &&
        writer < owner->writer &&
        (uint32_t)(tick - owner->tick) < (uint32_t)LOWCMD_PRIORITY_HOLD_MS)
    {
        LowCmdRecordReject(writer, owner->writer, tick);
        return 0u;
    }

    return 1u;
}

static uint8_t LowCmdSetMotorMany_checked(const MotorId *ids, const MotorCmd *cmds, uint8_t count, uint16_t writer)
{
    const uint32_t tick = LowCmdNowMs();
    const uint16_t resolved_writer = LowCmdWriterOrDefault(writer);

    if (LowCmdWriterValid(resolved_writer) == 0u)
    {
        return 0u;
    }

    LowCmdCriticalState critical = LowCmdEnterCritical();
    for (uint8_t i = 0u; i < count; i++)
    {
        /* writer 只由可信 API 参数决定，MotorCmd 载荷不能自行提权。 */
        if (LowCmdCanWriteLocked(ids[i], resolved_writer, &cmds[i], tick) == 0u)
        {
            LowCmdExitCritical(critical);
            return 0u;
        }
    }

    gLowCmdSeq++;
    for (uint8_t i = 0u; i < count; i++)
    {
        gMotorCmd[ids[i]] = cmds[i];
        gMotorCmd[ids[i]].writer = resolved_writer;
        LowCmdStamp(&gMotorCmd[ids[i]], gLowCmdSeq, tick);
    }
    gLowCmdDiag.seq = gLowCmdSeq;
    LowCmdExitCritical(critical);
    return 1u;
}

void LowCmdClearAll(void)
{
    LowCmdCriticalState critical = LowCmdEnterCritical();
    (void)memset(gMotorCmd, 0, sizeof(gMotorCmd));
    (void)memset(gLowCmdInhibitWriter, 0, sizeof(gLowCmdInhibitWriter));
    (void)memset(&gLowCmdDiag, 0, sizeof(gLowCmdDiag));
    gLowCmdSeq++;
    gLowCmdDiag.seq = gLowCmdSeq;
    LowCmdExitCritical(critical);
}

void LowCmdClear(MotorId id)
{
    if (MotorIdValid(id) == 0u)
    {
        return;
    }

    LowCmdCriticalState critical = LowCmdEnterCritical();
    {
        const uint32_t tick = LowCmdNowMs();
        const uint16_t writer = (uint16_t)LOWCMD_WRITER_CONTROL;
        if (LowCmdCanWriteLocked(id, writer, NULL, tick) != 0u)
        {
            (void)memset(&gMotorCmd[id], 0, sizeof(gMotorCmd[id]));
            gMotorCmd[id].seq = ++gLowCmdSeq;
            gMotorCmd[id].tick = tick;
            gLowCmdDiag.seq = gLowCmdSeq;
        }
    }
    LowCmdExitCritical(critical);
}

uint8_t LowCmdClearManyFrom(const MotorId *ids, uint8_t count, uint16_t writer)
{
    const uint32_t tick = LowCmdNowMs();
    const uint16_t resolved_writer = LowCmdWriterOrDefault(writer);

    if (LowCmdWriterValid(resolved_writer) == 0u ||
        count > (uint8_t)MotorCount || (count != 0u && ids == NULL))
    {
        return 0u;
    }
    if (count == 0u)
    {
        return 1u;
    }
    for (uint8_t i = 0u; i < count; i++)
    {
        if (MotorIdValid(ids[i]) == 0u)
        {
            return 0u;
        }
        for (uint8_t j = 0u; j < i; j++)
        {
            if (ids[i] == ids[j])
            {
                return 0u;
            }
        }
    }

    LowCmdCriticalState critical = LowCmdEnterCritical();
    for (uint8_t i = 0u; i < count; i++)
    {
        if (LowCmdCanWriteLocked(ids[i], resolved_writer, NULL, tick) == 0u)
        {
            LowCmdExitCritical(critical);
            return 0u;
        }
    }

    gLowCmdSeq++;
    for (uint8_t i = 0u; i < count; i++)
    {
        (void)memset(&gMotorCmd[ids[i]], 0, sizeof(gMotorCmd[ids[i]]));
        gMotorCmd[ids[i]].writer = resolved_writer;
        LowCmdStamp(&gMotorCmd[ids[i]], gLowCmdSeq, tick);
    }
    gLowCmdDiag.seq = gLowCmdSeq;
    LowCmdExitCritical(critical);
    return 1u;
}

uint8_t LowCmdInhibitManyFrom(const MotorId *ids, uint8_t count, uint16_t writer)
{
    const uint32_t tick = LowCmdNowMs();
    const uint16_t resolved_writer = LowCmdWriterOrDefault(writer);
    uint8_t changed = 0u;

    if (LowCmdWriterValid(resolved_writer) == 0u ||
        count > (uint8_t)MotorCount || (count != 0u && ids == NULL))
    {
        return 0u;
    }
    if (count == 0u)
    {
        return 1u;
    }
    for (uint8_t i = 0u; i < count; i++)
    {
        if (MotorIdValid(ids[i]) == 0u)
        {
            return 0u;
        }
        for (uint8_t j = 0u; j < i; j++)
        {
            if (ids[i] == ids[j])
            {
                return 0u;
            }
        }
    }

    LowCmdCriticalState critical = LowCmdEnterCritical();
    for (uint8_t i = 0u; i < count; i++)
    {
        if (gLowCmdInhibitWriter[ids[i]] == resolved_writer)
        {
            continue;
        }
        if (LowCmdCanWriteLocked(ids[i], resolved_writer, NULL, tick) == 0u)
        {
            LowCmdExitCritical(critical);
            return 0u;
        }
        changed = 1u;
    }

    if (changed == 0u)
    {
        LowCmdExitCritical(critical);
        return 1u;
    }

    gLowCmdSeq++;
    for (uint8_t i = 0u; i < count; i++)
    {
        if (gLowCmdInhibitWriter[ids[i]] == resolved_writer)
        {
            continue;
        }
        gLowCmdInhibitWriter[ids[i]] = resolved_writer;
        gLowCmdDiag.inhibit_mask |= 1ul << (uint32_t)ids[i];
        (void)memset(&gMotorCmd[ids[i]], 0, sizeof(gMotorCmd[ids[i]]));
        gMotorCmd[ids[i]].writer = resolved_writer;
        LowCmdStamp(&gMotorCmd[ids[i]], gLowCmdSeq, tick);
    }
    gLowCmdDiag.inhibit_acquire_count++;
    gLowCmdDiag.seq = gLowCmdSeq;
    LowCmdExitCritical(critical);
    return 1u;
}

uint8_t LowCmdReleaseInhibitManyFrom(const MotorId *ids, uint8_t count, uint16_t writer)
{
    const uint32_t tick = LowCmdNowMs();
    const uint16_t resolved_writer = LowCmdWriterOrDefault(writer);
    uint8_t changed = 0u;

    if (LowCmdWriterValid(resolved_writer) == 0u ||
        count > (uint8_t)MotorCount || (count != 0u && ids == NULL))
    {
        return 0u;
    }
    if (count == 0u)
    {
        return 1u;
    }
    for (uint8_t i = 0u; i < count; i++)
    {
        if (MotorIdValid(ids[i]) == 0u)
        {
            return 0u;
        }
        for (uint8_t j = 0u; j < i; j++)
        {
            if (ids[i] == ids[j])
            {
                return 0u;
            }
        }
    }

    LowCmdCriticalState critical = LowCmdEnterCritical();
    for (uint8_t i = 0u; i < count; i++)
    {
        const uint16_t inhibit_writer = gLowCmdInhibitWriter[ids[i]];

        if (inhibit_writer != (uint16_t)LOWCMD_WRITER_NONE &&
            resolved_writer < inhibit_writer)
        {
            LowCmdRecordReject(resolved_writer, inhibit_writer, tick);
            LowCmdExitCritical(critical);
            return 0u;
        }
        if (inhibit_writer != (uint16_t)LOWCMD_WRITER_NONE)
        {
            changed = 1u;
        }
    }

    for (uint8_t i = 0u; i < count; i++)
    {
        gLowCmdInhibitWriter[ids[i]] = (uint16_t)LOWCMD_WRITER_NONE;
        gLowCmdDiag.inhibit_mask &= ~(1ul << (uint32_t)ids[i]);
    }
    if (changed != 0u)
    {
        gLowCmdDiag.inhibit_release_count++;
    }
    LowCmdExitCritical(critical);
    return 1u;
}

uint8_t LowCmdGetInhibitWriter(MotorId id, uint16_t *out)
{
    if (MotorIdValid(id) == 0u || out == NULL)
    {
        return 0u;
    }

    LowCmdCriticalState critical = LowCmdEnterCritical();
    *out = gLowCmdInhibitWriter[id];
    LowCmdExitCritical(critical);
    return 1u;
}

uint8_t LowCmdSetMotorMany(const MotorId *ids, const MotorCmd *cmds, uint8_t count)
{
    return LowCmdSetMotorManyFrom(ids, cmds, count, LOWCMD_WRITER_CONTROL);
}

uint8_t LowCmdSetMotorManyFrom(const MotorId *ids, const MotorCmd *cmds, uint8_t count, uint16_t writer)
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

    return LowCmdSetMotorMany_checked(ids, cmds, count, writer);
}

uint8_t LowCmdSetMotor(MotorId id, const MotorCmd *cmd)
{
    return LowCmdSetMotorFrom(id, cmd, LOWCMD_WRITER_CONTROL);
}

uint8_t LowCmdSetMotorFrom(MotorId id, const MotorCmd *cmd, uint16_t writer)
{
    return LowCmdSetMotorManyFrom(&id, cmd, 1u, writer);
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

    LowCmdCriticalState critical = LowCmdEnterCritical();
    seq = gLowCmdSeq;
    LowCmdExitCritical(critical);
    return seq;
}

uint8_t LowCmdGet(LowCmd *out)
{
    if (out == NULL)
    {
        return 0u;
    }

    out->seq = LowCmdCopyMotorSnapshot(NULL, out->motorCmd, (uint8_t)MotorCount);
    out->tick = LowCmdNowMs();
    return 1u;
}

void LowCmdSetDisable(MotorId id)
{
    LowCmdSetDisableFrom(id, LOWCMD_WRITER_CONTROL);
}

void LowCmdSetDisableFrom(MotorId id, uint16_t writer)
{
    MotorCmd cmd;

    (void)memset(&cmd, 0, sizeof(cmd));
    cmd.active = 1u;
    cmd.mode = (uint8_t)MotorModeDisable;
    cmd.writer = LowCmdWriterOrDefault(writer);
    (void)LowCmdSetMotorFrom(id, &cmd, writer);
}

void LowCmdSetDamping(MotorId id, fp32 kd, fp32 tau)
{
    MotorCmd cmd;

    (void)memset(&cmd, 0, sizeof(cmd));
    cmd.active = 1u;
    cmd.mode = (uint8_t)MotorModeDamping;
    cmd.dq = 0.0f;
    cmd.kd = kd;
    cmd.tau = tau;
    (void)LowCmdSetMotor(id, &cmd);
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
    return LowCmdSetCurrentManyFrom(ids, currents, count, LOWCMD_WRITER_CONTROL);
}

uint8_t LowCmdSetCurrentManyFrom(const MotorId *ids, const int16_t *currents, uint8_t count, uint16_t writer)
{
    const uint16_t resolved_writer = LowCmdWriterOrDefault(writer);
    const uint32_t tick = LowCmdNowMs();
    MotorCmd current_cmd;

    if (LowCmdWriterValid(resolved_writer) == 0u || count > (uint8_t)MotorCount)
    {
        return 0u;
    }
    if (count != 0u && (ids == NULL || currents == NULL))
    {
        return 0u;
    }

    (void)memset(&current_cmd, 0, sizeof(current_cmd));
    current_cmd.active = 1u;
    current_cmd.mode = (uint8_t)MotorModeCurrent;
    current_cmd.writer = resolved_writer;

    for (uint8_t i = 0u; i < count; i++)
    {
        if (MotorIdValid(ids[i]) == 0u)
        {
            return 0u;
        }
    }

    {
        LowCmdCriticalState critical = LowCmdEnterCritical();

        for (uint8_t i = 0u; i < count; i++)
        {
            if (LowCmdCanWriteLocked(ids[i], resolved_writer, &current_cmd, tick) == 0u)
            {
                LowCmdExitCritical(critical);
                return 0u;
            }
        }

        if (count != 0u)
        {
            gLowCmdSeq++;
            for (uint8_t i = 0u; i < count; i++)
            {
                MotorCmd *dst = &gMotorCmd[ids[i]];

                *dst = current_cmd;
                dst->current = currents[i];
                LowCmdStamp(dst, gLowCmdSeq, tick);
            }
            gLowCmdDiag.seq = gLowCmdSeq;
        }
        LowCmdExitCritical(critical);
    }
    return 1u;
}

int16_t LowCmdGetCurrent(MotorId id)
{
    int16_t current;

    if (MotorIdValid(id) == 0u)
    {
        return 0;
    }

    LowCmdCriticalState critical = LowCmdEnterCritical();
    current = gMotorCmd[id].current;
    LowCmdExitCritical(critical);
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

    LowCmdCriticalState critical = LowCmdEnterCritical();
    for (uint8_t i = 0u; i < count; i++)
    {
        out[i] = gMotorCmd[ids[i]].current;
    }
    LowCmdExitCritical(critical);
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

    LowCmdCriticalState critical = LowCmdEnterCritical();
    *out = gMotorCmd[id];
    LowCmdExitCritical(critical);
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

    (void)LowCmdCopyMotorSnapshot(ids, out, count);
    return 1u;
}

const MotorCmd *LowCmdGetMotorPtr(MotorId id)
{
    if (MotorIdValid(id) == 0u ||
        LowCmdGetMotor(id, &gMotorCmdSnapshot[id]) == 0u)
    {
        return NULL;
    }

    return &gMotorCmdSnapshot[id];
}

uint8_t LowCmdEnterEmergencyStop(uint16_t writer)
{
    const uint32_t tick = LowCmdNowMs();
    const uint16_t resolved_writer = LowCmdWriterOrDefault(writer);

    if (LowCmdWriterValid(resolved_writer) == 0u)
    {
        return 0u;
    }

    LowCmdCriticalState critical = LowCmdEnterCritical();
    gLowCmdSeq++;
    gLowCmdDiag.emergency_active = 1u;
    gLowCmdDiag.emergency_writer = resolved_writer;
    gLowCmdDiag.emergency_stop_count++;
    gLowCmdDiag.seq = gLowCmdSeq;
    for (uint8_t i = 0u; i < (uint8_t)MotorCount; i++)
    {
        (void)memset(&gMotorCmd[i], 0, sizeof(gMotorCmd[i]));
        gMotorCmd[i].active = 1u;
        gMotorCmd[i].mode = (uint8_t)MotorModeDisable;
        gMotorCmd[i].writer = resolved_writer;
        LowCmdStamp(&gMotorCmd[i], gLowCmdSeq, tick);
    }
    LowCmdExitCritical(critical);
    return 1u;
}

uint8_t LowCmdClearEmergencyStop(uint16_t writer)
{
    const uint16_t resolved_writer = LowCmdWriterOrDefault(writer);

    if (LowCmdWriterValid(resolved_writer) == 0u)
    {
        return 0u;
    }

    LowCmdCriticalState critical = LowCmdEnterCritical();
    if (gLowCmdDiag.emergency_active != 0u &&
        resolved_writer < gLowCmdDiag.emergency_writer)
    {
        LowCmdRecordReject(resolved_writer, gLowCmdDiag.emergency_writer, LowCmdNowMs());
        LowCmdExitCritical(critical);
        return 0u;
    }

    gLowCmdDiag.emergency_active = 0u;
    gLowCmdDiag.emergency_writer = (uint16_t)LOWCMD_WRITER_NONE;
    LowCmdExitCritical(critical);
    return 1u;
}

uint8_t LowCmdEmergencyActive(void)
{
    uint8_t active;

    LowCmdCriticalState critical = LowCmdEnterCritical();
    active = gLowCmdDiag.emergency_active;
    LowCmdExitCritical(critical);
    return active;
}

uint8_t LowCmdGetDiag(LowCmdDiag *out)
{
    if (out == NULL)
    {
        return 0u;
    }

    LowCmdCriticalState critical = LowCmdEnterCritical();
    *out = gLowCmdDiag;
    out->seq = gLowCmdSeq;
    LowCmdExitCritical(critical);
    return 1u;
}

void LowStateClearAll(void)
{
    LowCmdCriticalState critical = LowCmdEnterCritical();
    (void)memset(gMotorState, 0, sizeof(gMotorState));
    (void)memset(gMotorApplied, 0, sizeof(gMotorApplied));
    gLowStateTick = LowCmdNowMs();
    LowCmdExitCritical(critical);
}

void LowStateUpdateMotor(MotorId id, const MotorState *feedback)
{
    if (MotorIdValid(id) == 0u || feedback == NULL)
    {
        return;
    }

    LowCmdCriticalState critical = LowCmdEnterCritical();
    gMotorState[id] = *feedback;
    gLowStateTick = feedback->lastRxTick;
    LowCmdExitCritical(critical);
}

void LowStateUpdateApplied(MotorId id, const MotorApplied *applied)
{
    if (MotorIdValid(id) == 0u || applied == NULL)
    {
        return;
    }

    LowCmdCriticalState critical = LowCmdEnterCritical();
    gMotorApplied[id] = *applied;
    gLowStateTick = applied->tick;
    LowCmdExitCritical(critical);
}

uint8_t LowStateGet(LowState *out)
{
    if (out == NULL)
    {
        return 0u;
    }

    LowCmdCriticalState critical = LowCmdEnterCritical();
    out->tick = gLowStateTick;
    (void)memcpy(out->motorState, gMotorState, sizeof(gMotorState));
    (void)memcpy(out->motorApplied, gMotorApplied, sizeof(gMotorApplied));
    LowCmdExitCritical(critical);
    return 1u;
}

uint8_t LowStateGetMotor(MotorId id, MotorState *out)
{
    if (MotorIdValid(id) == 0u || out == NULL)
    {
        return 0u;
    }

    LowCmdCriticalState critical = LowCmdEnterCritical();
    *out = gMotorState[id];
    LowCmdExitCritical(critical);
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

    LowCmdCriticalState critical = LowCmdEnterCritical();
    for (uint8_t i = 0u; i < count; i++)
    {
        out[i] = gMotorState[ids[i]];
    }
    LowCmdExitCritical(critical);
    return 1u;
}

const MotorState *LowStateGetMotorPtr(MotorId id)
{
    if (MotorIdValid(id) == 0u ||
        LowStateGetMotor(id, &gMotorStateSnapshot[id]) == 0u)
    {
        return NULL;
    }

    return &gMotorStateSnapshot[id];
}

uint8_t LowStateGetApplied(MotorId id, MotorApplied *out)
{
    if (MotorIdValid(id) == 0u || out == NULL)
    {
        return 0u;
    }

    LowCmdCriticalState critical = LowCmdEnterCritical();
    *out = gMotorApplied[id];
    LowCmdExitCritical(critical);
    return 1u;
}

const MotorApplied *LowStateGetAppliedPtr(MotorId id)
{
    if (MotorIdValid(id) == 0u ||
        LowStateGetApplied(id, &gMotorAppliedSnapshot[id]) == 0u)
    {
        return NULL;
    }

    return &gMotorAppliedSnapshot[id];
}
