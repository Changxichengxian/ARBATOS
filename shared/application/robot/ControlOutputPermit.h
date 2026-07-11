/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-07-11
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef CONTROL_OUTPUT_PERMIT_H
#define CONTROL_OUTPUT_PERMIT_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 输出许可会跨控制回调和发送队列传递，因此只使用固定宽度字段。
 * controllerId/domain 是许可身份；authorityEpoch/cycleSeq 共同限定有效周期。
 */
typedef struct
{
    uint32_t authorityEpoch;
    uint32_t cycleSeq;
    uint16_t controllerId;
    uint8_t domain;
    uint8_t valid;
} ControlOutputStamp;

typedef struct
{
    ControlOutputStamp stamp;
    uint32_t actuatorMask;
} ControlOutputPermit;

static inline void ControlOutputStampClear(ControlOutputStamp *stamp)
{
    if (stamp == NULL)
    {
        return;
    }

    stamp->authorityEpoch = 0u;
    stamp->cycleSeq = 0u;
    stamp->controllerId = 0u;
    stamp->domain = 0u;
    stamp->valid = 0u;
}

static inline void ControlOutputPermitClear(ControlOutputPermit *permit)
{
    if (permit == NULL)
    {
        return;
    }

    ControlOutputStampClear(&permit->stamp);
    permit->actuatorMask = 0u;
}

static inline uint8_t ControlOutputPermitAllows(const ControlOutputPermit *permit,
                                                uint32_t requiredMask)
{
    /* 这里只检查许可携带的范围；当前授权仍须由 ControlMgr 复核。 */
    if (permit == NULL || permit->stamp.valid != 1u)
    {
        return 0u;
    }

    return ((permit->actuatorMask & requiredMask) == requiredMask) ? 1u : 0u;
}

static inline uint8_t ControlOutputStampEqual(const ControlOutputStamp *left,
                                              const ControlOutputStamp *right)
{
    if (left == NULL || right == NULL)
    {
        return 0u;
    }

    return (left->authorityEpoch == right->authorityEpoch &&
            left->cycleSeq == right->cycleSeq &&
            left->controllerId == right->controllerId &&
            left->domain == right->domain &&
            left->valid == right->valid) ? 1u : 0u;
}

#ifdef __cplusplus
}
#endif

#endif
