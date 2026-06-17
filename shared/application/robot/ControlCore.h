/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CONTROL_CORE_H
#define CONTROL_CORE_H

#include <stdint.h>
#include <string.h>

#include "LowCmd.h"
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    CONTROL_CORE_RESULT_OK = 0u,
    CONTROL_CORE_RESULT_DISABLED,
    CONTROL_CORE_RESULT_BAD_ARGUMENT,
    CONTROL_CORE_RESULT_FAULT,
} control_core_result_e;

typedef struct
{
    fp32 dt_s;
    uint32_t tick_ms;
    uint32_t flags;
} control_core_step_t;

static inline fp32 control_core_clamp_fp32(fp32 value, fp32 min_value, fp32 max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

static inline int16_t control_core_abs_i16(int16_t value)
{
    if (value == INT16_MIN)
    {
        return INT16_MAX;
    }
    return (value < 0) ? (int16_t)(-value) : value;
}

static inline void control_core_cmd_clear(MotorCmd *cmd)
{
    if (cmd != NULL)
    {
        (void)memset(cmd, 0, sizeof(*cmd));
        cmd->mode = (uint8_t)MotorModeNone;
    }
}

static inline void control_core_cmd_clear_many(MotorCmd *cmds, uint8_t count)
{
    uint8_t i;

    if (cmds == NULL)
    {
        return;
    }

    for (i = 0u; i < count; i++)
    {
        control_core_cmd_clear(&cmds[i]);
    }
}

static inline void control_core_cmd_set_current(MotorCmd *cmd, int16_t current)
{
    if (cmd != NULL)
    {
        control_core_cmd_clear(cmd);
        cmd->active = 1u;
        cmd->mode = (uint8_t)MotorModeCurrent;
        cmd->current = current;
    }
}

static inline void control_core_cmd_set_disable(MotorCmd *cmd)
{
    if (cmd != NULL)
    {
        control_core_cmd_clear(cmd);
        cmd->active = 1u;
        cmd->mode = (uint8_t)MotorModeDisable;
    }
}

static inline void control_core_cmd_set_damping(MotorCmd *cmd, fp32 kd, fp32 tau)
{
    if (cmd != NULL)
    {
        control_core_cmd_clear(cmd);
        cmd->active = 1u;
        cmd->mode = (uint8_t)MotorModeDamping;
        cmd->dq = 0.0f;
        cmd->kd = kd;
        cmd->tau = tau;
    }
}

static inline void control_core_cmd_set_speed(MotorCmd *cmd, fp32 velocity, fp32 kd, fp32 torque)
{
    if (cmd != NULL)
    {
        control_core_cmd_clear(cmd);
        cmd->active = 1u;
        cmd->mode = (uint8_t)MotorModeSpeed;
        cmd->dq = velocity;
        cmd->kd = kd;
        cmd->tau = torque;
    }
}

static inline void control_core_cmd_set_state_torque(MotorCmd *cmd,
                                                     fp32 position,
                                                     fp32 velocity,
                                                     fp32 kp,
                                                     fp32 kd,
                                                     fp32 torque)
{
    if (cmd != NULL)
    {
        control_core_cmd_clear(cmd);
        cmd->active = 1u;
        cmd->mode = (uint8_t)MotorModeStateTorque;
        cmd->q = position;
        cmd->dq = velocity;
        cmd->kp = kp;
        cmd->kd = kd;
        cmd->tau = torque;
    }
}

#ifdef __cplusplus
}
#endif

#endif
