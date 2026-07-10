/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_INST_BEST_EFFORT_H
#define MOTOR_INST_BEST_EFFORT_H

#include "LowCmd.h"

/* 正常路径保留一次批量原子写；仅在批量被拒时逐轴降级，隔离轴不再拖住健康轴。 */
static inline uint8_t MotorInstLowCmdSetBestEffort(const MotorId *ids,
                                                   const MotorCmd *cmds,
                                                   uint8_t count)
{
    uint8_t written = 0u;

    if (count == 0u)
    {
        return 0u;
    }
    if (LowCmdSetMotorMany(ids, cmds, count) != 0u)
    {
        return count;
    }

    for (uint8_t i = 0u; i < count; i++)
    {
        if (LowCmdSetMotor(ids[i], &cmds[i]) != 0u)
        {
            written++;
        }
    }
    return written;
}

static inline uint8_t MotorInstLowCmdSetCurrentBestEffort(const MotorId *ids,
                                                          const int16_t *currents,
                                                          uint8_t count)
{
    uint8_t written = 0u;

    if (count == 0u)
    {
        return 0u;
    }
    if (LowCmdSetCurrentMany(ids, currents, count) != 0u)
    {
        return count;
    }

    for (uint8_t i = 0u; i < count; i++)
    {
        if (LowCmdSetCurrentMany(&ids[i], &currents[i], 1u) != 0u)
        {
            written++;
        }
    }
    return written;
}

#endif
