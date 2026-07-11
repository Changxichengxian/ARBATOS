/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "LowCmd.h"

#include "ControlMgr.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

#include <float.h>
#include <stddef.h>
#include <string.h>

/* MotorCmd 保持纯算法载荷；所有权只存在于 LowCmd 内部发布记录。 */
typedef struct
{
    MotorCmd cmd;
    ControlOutputStamp owner;
} LowCmdRecord;

static LowCmdRecord gLowCmdRecord[MotorCount];
static uint16_t gLowCmdInhibitWriter[MotorCount];
static MotorState gMotorState[MotorCount];
static MotorApplied gMotorApplied[MotorCount];
static MotorTxReceipt gMotorTxReceipt[MotorCount];
static MotorCmd gMotorCmdSnapshot[MotorCount];
static MotorState gMotorStateSnapshot[MotorCount];
static MotorApplied gMotorAppliedSnapshot[MotorCount];
static uint32_t gLowCmdSeq;
static uint32_t gLowCmdSeqEpoch;
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

static LowCmdVersion LowCmdVersionCurrentLocked(void)
{
    LowCmdVersion version;

    version.epoch = gLowCmdSeqEpoch;
    version.seq = gLowCmdSeq;
    return version;
}

static uint8_t LowCmdVersionEqual(LowCmdVersion left,
                                  LowCmdVersion right)
{
    return (uint8_t)(left.epoch == right.epoch && left.seq == right.seq);
}

static LowCmdVersion LowCmdAdvanceVersionLocked(void)
{
    gLowCmdSeq++;
    if (gLowCmdSeq == 0u)
    {
        gLowCmdSeqEpoch++;
    }
    gLowCmdDiag.seq = gLowCmdSeq;
    return LowCmdVersionCurrentLocked();
}

static uint8_t MotorIdValid(MotorId id)
{
    return ((uint32_t)id < (uint32_t)MotorCount) ? 1u : 0u;
}

static uint8_t MotorCmdValid(const MotorCmd *cmd)
{
    return (uint8_t)(cmd != NULL && MotorModeKnown(cmd->mode));
}

static uint8_t LowCmdFloatFinite(fp32 value)
{
    /* ARMCC5 没有稳定可用的 isfinite；有序比较同时拒绝 NaN 和无穷。 */
    return (uint8_t)(value <= (fp32)FLT_MAX && value >= (fp32)-FLT_MAX);
}

static uint8_t LowCmdMotorCmdEqual(const MotorCmd *left, const MotorCmd *right)
{
    if (left == NULL || right == NULL)
    {
        return 0u;
    }

    return (left->active == right->active &&
            left->mode == right->mode &&
            left->timeoutMs == right->timeoutMs &&
            left->writer == right->writer &&
            left->seq == right->seq &&
            left->tick == right->tick &&
            left->current == right->current &&
            left->q == right->q &&
            left->dq == right->dq &&
            left->kp == right->kp &&
            left->kd == right->kd &&
            left->tau == right->tau &&
            left->seqEpoch == right->seqEpoch) ? 1u : 0u;
}

static uint8_t LowCmdOwnerAbsent(const ControlOutputStamp *owner)
{
    return (uint8_t)(owner != NULL &&
                     owner->authorityEpoch == 0u &&
                     owner->cycleSeq == 0u &&
                     owner->controllerId == 0u &&
                     owner->domain == 0u &&
                     owner->valid == 0u);
}

static uint8_t LowCmdSafetyFallbackValid(const MotorCmd *cmd)
{
    if (cmd == NULL || cmd->active == 0u || cmd->current != 0 ||
        cmd->q != 0.0f || cmd->dq != 0.0f || cmd->kp != 0.0f ||
        cmd->tau != 0.0f)
    {
        return 0u;
    }

    if (cmd->mode == (uint8_t)MotorModeCurrent)
    {
        return (cmd->kd == 0.0f) ? 1u : 0u;
    }
    if (cmd->mode == (uint8_t)MotorModeDamping)
    {
        return (uint8_t)(LowCmdFloatFinite(cmd->kd) != 0u && cmd->kd >= 0.0f);
    }
    return 0u;
}

/*
 * 每次只在临界区内复制一个电机命令，把连续关中断时间限制在一个
 * MotorCmd 的大小。批量写每次都会更新 gLowCmdSeq；读完后序号不变，
 * 才说明这一批命令来自同一个一致快照。持续竞争时保留整批锁内兜底，
 * 避免高频写入让发送任务长期拿不到命令。
 */
static LowCmdVersion LowCmdCopyMotorSnapshot(const MotorId *ids,
                                             MotorCmd *cmds,
                                             ControlOutputStamp *owners,
                                             uint8_t count)
{
    LowCmdVersion version_begin = {0u, 0u};
    LowCmdVersion version_end = {0u, 0u};

    if (count == 0u)
    {
        LowCmdCriticalState critical = LowCmdEnterCritical();
        version_end = LowCmdVersionCurrentLocked();
        LowCmdExitCritical(critical);
        return version_end;
    }

    for (uint8_t attempt = 0u; attempt < LOWCMD_SNAPSHOT_RETRY_COUNT; attempt++)
    {
        for (uint8_t i = 0u; i < count; i++)
        {
            const MotorId id = (ids != NULL) ? ids[i] : (MotorId)i;
            LowCmdCriticalState critical = LowCmdEnterCritical();

            if (i == 0u)
            {
                version_begin = LowCmdVersionCurrentLocked();
            }
            cmds[i] = gLowCmdRecord[id].cmd;
            if (owners != NULL)
            {
                owners[i] = gLowCmdRecord[id].owner;
            }
            if (i == (uint8_t)(count - 1u))
            {
                version_end = LowCmdVersionCurrentLocked();
            }
            LowCmdExitCritical(critical);
        }

        if (LowCmdVersionEqual(version_begin, version_end) != 0u)
        {
            return version_end;
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
        version_end = LowCmdVersionCurrentLocked();
        for (uint8_t i = 0u; i < count; i++)
        {
            const MotorId id = (ids != NULL) ? ids[i] : (MotorId)i;
            cmds[i] = gLowCmdRecord[id].cmd;
            if (owners != NULL)
            {
                owners[i] = gLowCmdRecord[id].owner;
            }
        }
        LowCmdExitCritical(critical);
    }
    return version_end;
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

static void LowCmdStamp(MotorCmd *cmd,
                        LowCmdVersion version,
                        uint32_t tick)
{
    if (cmd == NULL)
    {
        return;
    }

    cmd->seq = version.seq;
    cmd->tick = tick;
    cmd->seqEpoch = version.epoch;
    if (cmd->timeoutMs == 0u && MotorModeNeedsDefaultTimeout(cmd->mode) != 0u)
    {
        cmd->timeoutMs = (uint16_t)LOWCMD_DEFAULT_TIMEOUT_MS;
    }
}

static void LowCmdRecordStoreLocked(LowCmdRecord *record,
                                    const MotorCmd *cmd,
                                    uint16_t writer,
                                    LowCmdVersion version,
                                    uint32_t tick,
                                    const ControlOutputStamp *owner)
{
    if (record == NULL || cmd == NULL)
    {
        return;
    }

    (void)memset(record, 0, sizeof(*record));
    record->cmd = *cmd;
    record->cmd.writer = writer;
    LowCmdStamp(&record->cmd, version, tick);
    gMotorTxReceipt[(uint32_t)(record - gLowCmdRecord)].valid = 0u;
    gMotorTxReceipt[(uint32_t)(record - gLowCmdRecord)].queued = 0u;
    if (owner != NULL)
    {
        record->owner = *owner;
    }
}

static void LowCmdRecordClearLocked(LowCmdRecord *record,
                                    uint16_t writer,
                                    LowCmdVersion version,
                                    uint32_t tick,
                                    const ControlOutputStamp *owner)
{
    if (record == NULL)
    {
        return;
    }

    (void)memset(record, 0, sizeof(*record));
    record->cmd.writer = writer;
    LowCmdStamp(&record->cmd, version, tick);
    gMotorTxReceipt[(uint32_t)(record - gLowCmdRecord)].valid = 0u;
    gMotorTxReceipt[(uint32_t)(record - gLowCmdRecord)].queued = 0u;
    if (owner != NULL)
    {
        record->owner = *owner;
    }
}

static uint8_t LowCmdIdsMask(const MotorId *ids, uint8_t count, uint32_t *out)
{
    uint32_t mask = 0u;

    if (out == NULL || count > (uint8_t)MotorCount ||
        (count != 0u && ids == NULL))
    {
        return 0u;
    }
    for (uint8_t i = 0u; i < count; i++)
    {
        uint32_t bit;

        if (MotorIdValid(ids[i]) == 0u)
        {
            return 0u;
        }
        bit = (uint32_t)1u << (uint32_t)ids[i];
        if ((mask & bit) != 0u)
        {
            return 0u;
        }
        mask |= bit;
    }

    *out = mask;
    return 1u;
}

static void LowCmdRecordReject(uint16_t writer, uint16_t owner, uint32_t tick)
{
    gLowCmdDiag.rejected_count++;
    gLowCmdDiag.last_reject_tick = tick;
    gLowCmdDiag.last_reject_writer = writer;
    gLowCmdDiag.last_reject_owner = owner;
}

/* 调用者已经持有 LowCmd 的任务临界区；ControlMgr 只会嵌套同一把任务锁。 */
static uint8_t LowCmdPermitValidLocked(const ControlOutputPermit *permit,
                                       uint32_t required_mask,
                                       uint32_t tick)
{
    if (ControlMgrOutputPermitValid(permit, required_mask) != 0u)
    {
        return 1u;
    }

    gLowCmdDiag.rejected_count++;
    gLowCmdDiag.permit_reject_count++;
    gLowCmdDiag.last_permit_reject_mask = required_mask;
    gLowCmdDiag.last_reject_tick = tick;
    gLowCmdDiag.last_reject_writer = (uint16_t)LOWCMD_WRITER_CONTROL;
    gLowCmdDiag.last_reject_owner = (uint16_t)LOWCMD_WRITER_NONE;
    return 0u;
}

static uint8_t LowCmdCanWriteLocked(MotorId id, uint16_t writer, const MotorCmd *cmd, uint32_t tick)
{
    const MotorCmd *owner = &gLowCmdRecord[id].cmd;
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

static uint8_t LowCmdSetMotorManyChecked(const MotorId *ids,
                                         const MotorCmd *cmds,
                                         uint8_t count,
                                         uint16_t writer,
                                         const ControlOutputPermit *permit,
                                         uint32_t required_mask)
{
    const uint32_t tick = LowCmdNowMs();
    const uint16_t resolved_writer = LowCmdWriterOrDefault(writer);

    if (LowCmdWriterValid(resolved_writer) == 0u)
    {
        return 0u;
    }

    LowCmdCriticalState critical = LowCmdEnterCritical();
    if (permit != NULL && LowCmdPermitValidLocked(permit, required_mask, tick) == 0u)
    {
        LowCmdExitCritical(critical);
        return 0u;
    }
    for (uint8_t i = 0u; i < count; i++)
    {
        /* writer 只由可信 API 参数决定，MotorCmd 载荷不能自行提权。 */
        if (LowCmdCanWriteLocked(ids[i], resolved_writer, &cmds[i], tick) == 0u)
        {
            LowCmdExitCritical(critical);
            return 0u;
        }
    }

    {
        const LowCmdVersion version = LowCmdAdvanceVersionLocked();

        for (uint8_t i = 0u; i < count; i++)
        {
            LowCmdRecordStoreLocked(&gLowCmdRecord[ids[i]],
                                    &cmds[i],
                                    resolved_writer,
                                    version,
                                    tick,
                                    (permit != NULL) ? &permit->stamp : NULL);
        }
    }
    LowCmdExitCritical(critical);
    return 1u;
}

void LowCmdClearAll(void)
{
    const uint32_t tick = LowCmdNowMs();
    LowCmdCriticalState critical = LowCmdEnterCritical();
    (void)memset(gLowCmdInhibitWriter, 0, sizeof(gLowCmdInhibitWriter));
    (void)memset(&gLowCmdDiag, 0, sizeof(gLowCmdDiag));
    {
        const LowCmdVersion version = LowCmdAdvanceVersionLocked();

        for (uint8_t i = 0u; i < (uint8_t)MotorCount; i++)
        {
            LowCmdRecordClearLocked(&gLowCmdRecord[i],
                                    (uint16_t)LOWCMD_WRITER_NONE,
                                    version,
                                    tick,
                                    NULL);
        }
    }
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
            const LowCmdVersion version = LowCmdAdvanceVersionLocked();

            LowCmdRecordClearLocked(&gLowCmdRecord[id],
                                    (uint16_t)LOWCMD_WRITER_NONE,
                                    version,
                                    tick,
                                    NULL);
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

    {
        const LowCmdVersion version = LowCmdAdvanceVersionLocked();

        for (uint8_t i = 0u; i < count; i++)
        {
            LowCmdRecordClearLocked(&gLowCmdRecord[ids[i]],
                                    resolved_writer,
                                    version,
                                    tick,
                                    NULL);
        }
    }
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

    {
        const LowCmdVersion version = LowCmdAdvanceVersionLocked();

        for (uint8_t i = 0u; i < count; i++)
        {
            if (gLowCmdInhibitWriter[ids[i]] == resolved_writer)
            {
                continue;
            }
            gLowCmdInhibitWriter[ids[i]] = resolved_writer;
            gLowCmdDiag.inhibit_mask |= 1ul << (uint32_t)ids[i];
            LowCmdRecordClearLocked(&gLowCmdRecord[ids[i]],
                                    resolved_writer,
                                    version,
                                    tick,
                                    NULL);
        }
    }
    gLowCmdDiag.inhibit_acquire_count++;
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

uint8_t LowCmdRecoverSafetyInhibitManyWithPermit(const MotorId *ids,
                                                  uint8_t count,
                                                  const ControlOutputPermit *permit)
{
    const uint32_t tick = LowCmdNowMs();
    ControlOutputPermit permit_copy;
    uint32_t required_mask;
    uint8_t changed = 0u;

    if (__get_IPSR() != 0U || permit == NULL ||
        LowCmdIdsMask(ids, count, &required_mask) == 0u)
    {
        return 0u;
    }
    permit_copy = *permit;

    {
        LowCmdCriticalState critical = LowCmdEnterCritical();

        if (LowCmdPermitValidLocked(&permit_copy, required_mask, tick) == 0u)
        {
            LowCmdExitCritical(critical);
            return 0u;
        }
        if (gLowCmdDiag.emergency_active != 0u)
        {
            LowCmdRecordReject((uint16_t)LOWCMD_WRITER_CONTROL,
                               gLowCmdDiag.emergency_writer,
                               tick);
            LowCmdExitCritical(critical);
            return 0u;
        }
        for (uint8_t i = 0u; i < count; i++)
        {
            const uint16_t inhibit_writer = gLowCmdInhibitWriter[ids[i]];
            const MotorCmd *cmd = &gLowCmdRecord[ids[i]].cmd;

            /* SAFETY 禁写不能借恢复接口覆盖仍由更高层持有的局部活动命令。 */
            if (cmd->active != 0u &&
                cmd->writer > (uint16_t)LOWCMD_WRITER_SAFETY)
            {
                LowCmdRecordReject((uint16_t)LOWCMD_WRITER_CONTROL,
                                   cmd->writer,
                                   tick);
                LowCmdExitCritical(critical);
                return 0u;
            }

            if (inhibit_writer == (uint16_t)LOWCMD_WRITER_SAFETY)
            {
                changed = 1u;
                continue;
            }
            if (inhibit_writer != (uint16_t)LOWCMD_WRITER_NONE)
            {
                LowCmdRecordReject((uint16_t)LOWCMD_WRITER_CONTROL,
                                   inhibit_writer,
                                   tick);
                LowCmdExitCritical(critical);
                return 0u;
            }
        }

        if (changed != 0u)
        {
            const LowCmdVersion version = LowCmdAdvanceVersionLocked();

            for (uint8_t i = 0u; i < count; i++)
            {
                if (gLowCmdInhibitWriter[ids[i]] != (uint16_t)LOWCMD_WRITER_SAFETY)
                {
                    continue;
                }
                gLowCmdInhibitWriter[ids[i]] = (uint16_t)LOWCMD_WRITER_NONE;
                gLowCmdDiag.inhibit_mask &= ~(1ul << (uint32_t)ids[i]);
                LowCmdRecordClearLocked(&gLowCmdRecord[ids[i]],
                                        (uint16_t)LOWCMD_WRITER_CONTROL,
                                        version,
                                        tick,
                                        &permit_copy.stamp);
            }
            gLowCmdDiag.inhibit_release_count++;
        }
        LowCmdExitCritical(critical);
    }
    return 1u;
}

uint8_t LowCmdRecoverSafetyInhibitWithPermit(MotorId id,
                                              const ControlOutputPermit *permit)
{
    return LowCmdRecoverSafetyInhibitManyWithPermit(&id, 1u, permit);
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

    return LowCmdSetMotorManyChecked(ids, cmds, count, writer, NULL, 0u);
}

uint8_t LowCmdSetMotor(MotorId id, const MotorCmd *cmd)
{
    return LowCmdSetMotorFrom(id, cmd, LOWCMD_WRITER_CONTROL);
}

uint8_t LowCmdSetMotorFrom(MotorId id, const MotorCmd *cmd, uint16_t writer)
{
    return LowCmdSetMotorManyFrom(&id, cmd, 1u, writer);
}

uint8_t LowCmdSetMotorManyWithPermit(const MotorId *ids,
                                     const MotorCmd *cmds,
                                     uint8_t count,
                                     const ControlOutputPermit *permit)
{
    ControlOutputPermit permit_copy;
    uint32_t required_mask;

    if (__get_IPSR() != 0U || permit == NULL ||
        LowCmdIdsMask(ids, count, &required_mask) == 0u ||
        (count != 0u && cmds == NULL))
    {
        return 0u;
    }
    permit_copy = *permit;
    for (uint8_t i = 0u; i < count; i++)
    {
        if (MotorCmdValid(&cmds[i]) == 0u)
        {
            return 0u;
        }
    }
    if (count == 0u)
    {
        LowCmdCriticalState critical = LowCmdEnterCritical();
        const uint8_t valid = LowCmdPermitValidLocked(&permit_copy, 0u, LowCmdNowMs());

        LowCmdExitCritical(critical);
        return valid;
    }

    return LowCmdSetMotorManyChecked(ids,
                                     cmds,
                                     count,
                                     (uint16_t)LOWCMD_WRITER_CONTROL,
                                     &permit_copy,
                                     required_mask);
}

uint8_t LowCmdSetMotorWithPermit(MotorId id,
                                 const MotorCmd *cmd,
                                 const ControlOutputPermit *permit)
{
    return LowCmdSetMotorManyWithPermit(&id, cmd, 1u, permit);
}

uint8_t LowCmdClearManyWithPermit(const MotorId *ids,
                                  uint8_t count,
                                  const ControlOutputPermit *permit)
{
    const uint32_t tick = LowCmdNowMs();
    const uint16_t writer = (uint16_t)LOWCMD_WRITER_CONTROL;
    ControlOutputPermit permit_copy;
    uint32_t required_mask;

    if (__get_IPSR() != 0U || permit == NULL ||
        LowCmdIdsMask(ids, count, &required_mask) == 0u)
    {
        return 0u;
    }
    permit_copy = *permit;

    {
        LowCmdCriticalState critical = LowCmdEnterCritical();

        if (LowCmdPermitValidLocked(&permit_copy, required_mask, tick) == 0u)
        {
            LowCmdExitCritical(critical);
            return 0u;
        }
        if (count == 0u)
        {
            LowCmdExitCritical(critical);
            return 1u;
        }
        for (uint8_t i = 0u; i < count; i++)
        {
            if (LowCmdCanWriteLocked(ids[i], writer, NULL, tick) == 0u)
            {
                LowCmdExitCritical(critical);
                return 0u;
            }
        }

        {
            const LowCmdVersion version = LowCmdAdvanceVersionLocked();

            for (uint8_t i = 0u; i < count; i++)
            {
                LowCmdRecordClearLocked(&gLowCmdRecord[ids[i]],
                                        writer,
                                        version,
                                        tick,
                                        &permit_copy.stamp);
            }
        }
        LowCmdExitCritical(critical);
    }
    return 1u;
}

uint8_t LowCmdClearWithPermit(MotorId id, const ControlOutputPermit *permit)
{
    return LowCmdClearManyWithPermit(&id, 1u, permit);
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

uint8_t LowCmdVersionGet(LowCmdVersion *out)
{
    LowCmdCriticalState critical;

    if (out == NULL)
    {
        return 0u;
    }
    critical = LowCmdEnterCritical();
    *out = LowCmdVersionCurrentLocked();
    LowCmdExitCritical(critical);
    return 1u;
}

uint8_t LowCmdGet(LowCmd *out)
{
    LowCmdVersion version;

    if (out == NULL)
    {
        return 0u;
    }

    version = LowCmdCopyMotorSnapshot(NULL,
                                      out->motorCmd,
                                      NULL,
                                      (uint8_t)MotorCount);
    out->seq = version.seq;
    out->seqEpoch = version.epoch;
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
            const LowCmdVersion version = LowCmdAdvanceVersionLocked();

            for (uint8_t i = 0u; i < count; i++)
            {
                MotorCmd cmd = current_cmd;

                cmd.current = currents[i];
                LowCmdRecordStoreLocked(&gLowCmdRecord[ids[i]],
                                        &cmd,
                                        resolved_writer,
                                        version,
                                        tick,
                                        NULL);
            }
        }
        LowCmdExitCritical(critical);
    }
    return 1u;
}

uint8_t LowCmdSetCurrentManyWithPermit(const MotorId *ids,
                                       const int16_t *currents,
                                       uint8_t count,
                                       const ControlOutputPermit *permit)
{
    const uint16_t writer = (uint16_t)LOWCMD_WRITER_CONTROL;
    const uint32_t tick = LowCmdNowMs();
    ControlOutputPermit permit_copy;
    MotorCmd current_cmd;
    uint32_t required_mask;

    if (__get_IPSR() != 0U || permit == NULL ||
        LowCmdIdsMask(ids, count, &required_mask) == 0u ||
        (count != 0u && currents == NULL))
    {
        return 0u;
    }
    permit_copy = *permit;
    (void)memset(&current_cmd, 0, sizeof(current_cmd));
    current_cmd.active = 1u;
    current_cmd.mode = (uint8_t)MotorModeCurrent;

    {
        LowCmdCriticalState critical = LowCmdEnterCritical();

        if (LowCmdPermitValidLocked(&permit_copy, required_mask, tick) == 0u)
        {
            LowCmdExitCritical(critical);
            return 0u;
        }
        for (uint8_t i = 0u; i < count; i++)
        {
            if (LowCmdCanWriteLocked(ids[i], writer, &current_cmd, tick) == 0u)
            {
                LowCmdExitCritical(critical);
                return 0u;
            }
        }

        if (count != 0u)
        {
            const LowCmdVersion version = LowCmdAdvanceVersionLocked();

            for (uint8_t i = 0u; i < count; i++)
            {
                MotorCmd cmd = current_cmd;

                cmd.current = currents[i];
                LowCmdRecordStoreLocked(&gLowCmdRecord[ids[i]],
                                        &cmd,
                                        writer,
                                        version,
                                        tick,
                                        &permit_copy.stamp);
            }
        }
        LowCmdExitCritical(critical);
    }
    return 1u;
}

uint8_t LowCmdSetCurrentWithPermit(MotorId id,
                                   int16_t current,
                                   const ControlOutputPermit *permit)
{
    return LowCmdSetCurrentManyWithPermit(&id, &current, 1u, permit);
}

int16_t LowCmdGetCurrent(MotorId id)
{
    int16_t current;

    if (MotorIdValid(id) == 0u)
    {
        return 0;
    }

    LowCmdCriticalState critical = LowCmdEnterCritical();
    current = gLowCmdRecord[id].cmd.current;
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
        out[i] = gLowCmdRecord[ids[i]].cmd.current;
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
    *out = gLowCmdRecord[id].cmd;
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

    (void)LowCmdCopyMotorSnapshot(ids, out, NULL, count);
    return 1u;
}

uint8_t LowCmdGetMotorStamped(MotorId id,
                              MotorCmd *cmd,
                              ControlOutputStamp *owner)
{
    if (MotorIdValid(id) == 0u || cmd == NULL || owner == NULL)
    {
        return 0u;
    }

    {
        LowCmdCriticalState critical = LowCmdEnterCritical();

        *cmd = gLowCmdRecord[id].cmd;
        *owner = gLowCmdRecord[id].owner;
        LowCmdExitCritical(critical);
    }
    return 1u;
}

uint8_t LowCmdGetMotorManyStamped(const MotorId *ids,
                                  MotorCmd *cmds,
                                  ControlOutputStamp *owners,
                                  uint8_t count)
{
    if (count > (uint8_t)MotorCount ||
        (count != 0u && (ids == NULL || cmds == NULL || owners == NULL)))
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

    (void)LowCmdCopyMotorSnapshot(ids, cmds, owners, count);
    return 1u;
}

uint8_t LowCmdOutputSnapshotAuthorized(MotorId id,
                                       const MotorCmd *cached,
                                       const ControlOutputStamp *cachedOwner)
{
    LowCmdCriticalState critical;
    const LowCmdRecord *latest;
    uint16_t inhibit_writer;
    uint8_t authorized = 0u;

    if (MotorIdValid(id) == 0u || cached == NULL)
    {
        return 0u;
    }

    /* 这三类快照只会让执行器失能，允许旧缓存继续朝安全方向收敛。 */
    if (cached->active == 0u ||
        cached->mode == (uint8_t)MotorModeNone ||
        cached->mode == (uint8_t)MotorModeDisable)
    {
        return 1u;
    }

    critical = LowCmdEnterCritical();
    latest = &gLowCmdRecord[id];
    inhibit_writer = gLowCmdInhibitWriter[id];

    if (cachedOwner == NULL ||
        LowCmdMotorCmdEqual(cached, &latest->cmd) == 0u ||
        ControlOutputStampEqual(cachedOwner, &latest->owner) == 0u)
    {
        LowCmdExitCritical(critical);
        return 0u;
    }

    if (cached->writer == (uint16_t)LOWCMD_WRITER_CONTROL &&
        cachedOwner->valid == 1u)
    {
        if ((inhibit_writer == (uint16_t)LOWCMD_WRITER_NONE ||
             inhibit_writer <= (uint16_t)LOWCMD_WRITER_CONTROL) &&
            ControlMgrOutputStampValid(cachedOwner,
                                       (uint32_t)1u << (uint32_t)id) != 0u)
        {
            authorized = 1u;
        }
    }
    else if (cached->writer == (uint16_t)LOWCMD_WRITER_SAFETY &&
             inhibit_writer == (uint16_t)LOWCMD_WRITER_SAFETY &&
             LowCmdOwnerAbsent(cachedOwner) != 0u &&
             LowCmdSafetyFallbackValid(cached) != 0u)
    {
        authorized = 1u;
    }

    LowCmdExitCritical(critical);
    return authorized;
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
    const LowCmdVersion version = LowCmdAdvanceVersionLocked();
    gLowCmdDiag.emergency_active = 1u;
    gLowCmdDiag.emergency_writer = resolved_writer;
    gLowCmdDiag.emergency_stop_count++;
    for (uint8_t i = 0u; i < (uint8_t)MotorCount; i++)
    {
        MotorCmd cmd;

        (void)memset(&cmd, 0, sizeof(cmd));
        cmd.active = 1u;
        cmd.mode = (uint8_t)MotorModeDisable;
        LowCmdRecordStoreLocked(&gLowCmdRecord[i],
                                &cmd,
                                resolved_writer,
                                version,
                                tick,
                                NULL);
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
    (void)memset(gMotorTxReceipt, 0, sizeof(gMotorTxReceipt));
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

void LowStateUpdateTxQueued(MotorId id, const MotorTxIdentity *identity)
{
    if (MotorIdValid(id) == 0u || identity == NULL)
    {
        return;
    }

    {
        LowCmdCriticalState critical = LowCmdEnterCritical();
        const MotorCmd *latest = &gLowCmdRecord[id].cmd;

        if (MotorTxIdentityMatchesCmd(identity, latest) != 0u)
        {
            MotorTxReceipt *receipt = &gMotorTxReceipt[id];
            const uint8_t sameIdentity = (uint8_t)(
                receipt->cmdSeq == identity->cmdSeq &&
                receipt->cmdTick == identity->cmdTick &&
                receipt->writer == identity->writer &&
                receipt->mode == identity->mode &&
                receipt->current == identity->current &&
                receipt->cmdSeqEpoch == identity->cmdSeqEpoch);

            if (sameIdentity == 0u)
            {
                (void)memset(receipt, 0, sizeof(*receipt));
            }
            receipt->cmdSeq = identity->cmdSeq;
            receipt->cmdTick = identity->cmdTick;
            receipt->cmdSeqEpoch = identity->cmdSeqEpoch;
            receipt->queuedTick = LowCmdNowMs();
            receipt->current = identity->current;
            receipt->writer = identity->writer;
            receipt->queued = 1u;
            receipt->mode = identity->mode;
        }
        LowCmdExitCritical(critical);
    }
}

void LowStateUpdateTxComplete(MotorId id,
                              const MotorTxIdentity *identity)
{
    if (MotorIdValid(id) == 0u || identity == NULL)
    {
        return;
    }

    {
        LowCmdCriticalState critical = LowCmdEnterCritical();
        const MotorCmd *latest = &gLowCmdRecord[id].cmd;

        if (MotorTxIdentityMatchesCmd(identity, latest) != 0u)
        {
            MotorTxReceipt *receipt = &gMotorTxReceipt[id];
            const uint8_t sameIdentity = (uint8_t)(
                receipt->cmdSeq == identity->cmdSeq &&
                receipt->cmdTick == identity->cmdTick &&
                receipt->writer == identity->writer &&
                receipt->mode == identity->mode &&
                receipt->current == identity->current &&
                receipt->cmdSeqEpoch == identity->cmdSeqEpoch);
            const uint32_t now = LowCmdNowMs();

            if (sameIdentity == 0u)
            {
                (void)memset(receipt, 0, sizeof(*receipt));
            }
            receipt->cmdSeq = identity->cmdSeq;
            receipt->cmdTick = identity->cmdTick;
            receipt->cmdSeqEpoch = identity->cmdSeqEpoch;
            if (receipt->queued == 0u)
            {
                receipt->queuedTick = now;
            }
            receipt->completedTick = now;
            receipt->current = identity->current;
            receipt->writer = identity->writer;
            receipt->valid = 1u;
            receipt->queued = 1u;
            receipt->mode = identity->mode;
        }
        LowCmdExitCritical(critical);
    }
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

uint8_t LowStateGetTxReceipt(MotorId id, MotorTxReceipt *out)
{
    if (MotorIdValid(id) == 0u || out == NULL)
    {
        return 0u;
    }

    {
        LowCmdCriticalState critical = LowCmdEnterCritical();
        *out = gMotorTxReceipt[id];
        LowCmdExitCritical(critical);
    }
    return 1u;
}
