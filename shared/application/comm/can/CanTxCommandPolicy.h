/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CAN_TX_COMMAND_POLICY_H
#define CAN_TX_COMMAND_POLICY_H

#include "LowCmd.h"

#define CAN_TX_CMD_FUTURE_TOLERANCE_MS 1u

typedef struct
{
    uint32_t seq;
    uint8_t valid;
} CanTxCmdExpiryLatch;

static inline uint8_t CanTxCmdIsLocalDisable(const MotorCmd *cmd)
{
    return (uint8_t)(cmd != NULL &&
                     cmd->active != 0u &&
                     cmd->mode == (uint8_t)MotorModeDisable &&
                     cmd->seq == 0u &&
                     cmd->writer == (uint16_t)LOWCMD_WRITER_NONE &&
                     cmd->timeoutMs == 0u);
}

/* tick=0 是上电首毫秒的合法时刻，不能被当成“永不超时”标记。 */
static inline uint8_t CanTxCmdExpired(const MotorCmd *cmd, uint32_t nowMs)
{
    uint32_t ageMs;

    if (cmd == NULL || cmd->active == 0u || cmd->timeoutMs == 0u)
    {
        return 0u;
    }

    ageMs = nowMs - cmd->tick;
    if (ageMs <= (uint32_t)cmd->timeoutMs)
    {
        return 0u;
    }

    /* 只容忍读时钟与发布时钟之间的一毫秒竞态。 */
    if (nowMs < cmd->tick &&
        (cmd->tick - nowMs) <= (uint32_t)CAN_TX_CMD_FUTURE_TOLERANCE_MS)
    {
        return 0u;
    }
    return 1u;
}

/* 记住已过期的发布代，避免毫秒计数完整回绕后旧命令短暂复活。 */
static inline uint8_t CanTxCmdExpiryLatchCheck(CanTxCmdExpiryLatch *latch,
                                               const MotorCmd *cmd,
                                               uint32_t nowMs)
{
    if (latch == NULL || cmd == NULL)
    {
        return 0u;
    }
    /* CanTx 临时合成的 Disable 不是新发布代，不得清掉旧代过期记忆。 */
    if (CanTxCmdIsLocalDisable(cmd) != 0u)
    {
        return 0u;
    }
    if (latch->valid != 0u)
    {
        if (latch->seq == cmd->seq)
        {
            return 1u;
        }
        latch->valid = 0u;
    }
    if (CanTxCmdExpired(cmd, nowMs) == 0u)
    {
        return 0u;
    }

    latch->seq = cmd->seq;
    latch->valid = 1u;
    return 1u;
}

/* 缓存命令只有仍是当前发布版本，且 writer 不低于局部禁写 owner 时才可继续发送。 */
static inline uint8_t CanTxCachedCmdAuthorized(const MotorCmd *cached,
                                               const MotorCmd *latest,
                                               uint16_t inhibitWriter)
{
    uint16_t cachedWriter;

    if (cached == NULL || latest == NULL || latest->active == 0u)
    {
        return 0u;
    }
    if (cached->seq != latest->seq || cached->writer != latest->writer)
    {
        return 0u;
    }

    cachedWriter = (cached->writer == (uint16_t)LOWCMD_WRITER_NONE) ?
                       (uint16_t)LOWCMD_WRITER_CONTROL : cached->writer;
    if (inhibitWriter != (uint16_t)LOWCMD_WRITER_NONE && cachedWriter < inhibitWriter)
    {
        return 0u;
    }
    return 1u;
}

#endif
