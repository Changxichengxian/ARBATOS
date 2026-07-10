/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_FEEDBACK_ECD_POLICY_H
#define MOTOR_FEEDBACK_ECD_POLICY_H

#include <stddef.h>
#include <stdint.h>

#include "LowCmd.h"

/* rxCount 的 0 表示从未收到反馈，32 位回绕时跳过这个保留值。 */
static inline uint32_t MotorFeedbackRxCountNext(uint32_t rxCount)
{
    rxCount++;
    return (rxCount != 0u) ? rxCount : 1u;
}

/*
 * RS485 发送周期会反复发布同一份接收状态。只有接收计数变化时才推进
 * 编码器上一值；首份有效样本用当前值自初始化，避免产生虚假跳变。
 */
static inline uint8_t MotorFeedbackEcdResolve(const MotorState *previous,
                                              uint32_t rxCount,
                                              uint16_t ecd,
                                              uint16_t *lastEcd)
{
    if (lastEcd == NULL)
    {
        return 0u;
    }

    if (rxCount == 0u)
    {
        *lastEcd = (previous != NULL && previous->rxCount != 0u) ?
            previous->lastEcd : ecd;
        return 0u;
    }
    if (previous == NULL || previous->rxCount == 0u)
    {
        *lastEcd = ecd;
        return 1u;
    }
    if (rxCount != previous->rxCount)
    {
        *lastEcd = previous->ecd;
        return 1u;
    }

    *lastEcd = previous->lastEcd;
    return 0u;
}

#endif
