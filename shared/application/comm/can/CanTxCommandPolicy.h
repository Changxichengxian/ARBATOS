/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CAN_TX_COMMAND_POLICY_H
#define CAN_TX_COMMAND_POLICY_H

#include "LowCmd.h"

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
