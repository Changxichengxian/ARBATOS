/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CONTROL_CORE_H
#define CONTROL_CORE_H

#include <stdint.h>
#include <string.h>

#include "actuator_cmd.h"
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

static inline void control_core_cmd_clear(actuator_cmd_t *cmd)
{
    if (cmd != NULL)
    {
        (void)memset(cmd, 0, sizeof(*cmd));
        cmd->mode = (uint8_t)ACTUATOR_CMD_MODE_NONE;
    }
}

static inline void control_core_cmd_clear_many(actuator_cmd_t *cmds, uint8_t count)
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

static inline void control_core_cmd_set_current(actuator_cmd_t *cmd, int16_t current)
{
    if (cmd != NULL)
    {
        control_core_cmd_clear(cmd);
        cmd->active = 1u;
        cmd->mode = (uint8_t)ACTUATOR_CMD_MODE_CURRENT;
        cmd->current = current;
    }
}

static inline void control_core_cmd_set_speed(actuator_cmd_t *cmd, fp32 velocity, fp32 kd, fp32 torque)
{
    if (cmd != NULL)
    {
        control_core_cmd_clear(cmd);
        cmd->active = 1u;
        cmd->mode = (uint8_t)ACTUATOR_CMD_MODE_SPEED;
        cmd->velocity = velocity;
        cmd->kd = kd;
        cmd->torque = torque;
    }
}

static inline void control_core_cmd_set_state_torque(actuator_cmd_t *cmd,
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
        cmd->mode = (uint8_t)ACTUATOR_CMD_MODE_STATE_TORQUE;
        cmd->position = position;
        cmd->velocity = velocity;
        cmd->kp = kp;
        cmd->kd = kd;
        cmd->torque = torque;
    }
}

#ifdef __cplusplus
}
#endif

#endif
