/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CHASSIS_CORE_H
#define CHASSIS_CORE_H

#include <stdint.h>

#include "control_core.h"
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CHASSIS_CORE_WHEEL_COUNT 4u

typedef enum
{
    CHASSIS_CORE_WHEEL_TYPE_MECANUM = 0u,
    CHASSIS_CORE_WHEEL_TYPE_XDRIVE = 1u,
} chassis_core_wheel_type_e;

typedef struct
{
    fp32 vx_mps;
    fp32 vy_mps;
    fp32 wz_radps;
    fp32 yaw_relative_rad;
    fp32 yaw_rate_radps;
    fp32 wheel_speed[CHASSIS_CORE_WHEEL_COUNT];
    uint8_t wheel_online[CHASSIS_CORE_WHEEL_COUNT];
} chassis_core_state_t;

typedef struct
{
    uint8_t enabled;
    uint8_t follow_gimbal;
    uint8_t raw_output;
    fp32 vx_mps;
    fp32 vy_mps;
    fp32 wz_radps;
} chassis_core_cmd_t;

typedef struct
{
    uint8_t wheel_type; // chassis_core_wheel_type_e
    fp32 motor_distance_to_center;
    fp32 max_wheel_speed;
} chassis_core_kinematic_config_t;

typedef struct
{
    fp32 wheel_speed_set[CHASSIS_CORE_WHEEL_COUNT];
    int16_t current[CHASSIS_CORE_WHEEL_COUNT];
    MotorCmd actuator[CHASSIS_CORE_WHEEL_COUNT];
} chassis_core_output_t;

static inline fp32 chassis_core_abs_fp32(fp32 value)
{
    return (value < 0.0f) ? -value : value;
}

static inline void chassis_core_vector_to_wheel_speed(const chassis_core_kinematic_config_t *cfg,
                                                      fp32 vx_set,
                                                      fp32 vy_set,
                                                      fp32 wz_set,
                                                      fp32 wheel_speed[CHASSIS_CORE_WHEEL_COUNT])
{
    const fp32 distance = (cfg != NULL) ? cfg->motor_distance_to_center : 0.0f;
    const uint8_t wheel_type = (cfg != NULL) ? cfg->wheel_type : (uint8_t)CHASSIS_CORE_WHEEL_TYPE_MECANUM;
    const fp32 yaw_term = distance * wz_set;

    if (wheel_speed == NULL)
    {
        return;
    }

    if (wheel_type == (uint8_t)CHASSIS_CORE_WHEEL_TYPE_XDRIVE)
    {
        wheel_speed[0] = vx_set + vy_set + yaw_term;
        wheel_speed[1] = -vx_set + vy_set + yaw_term;
        wheel_speed[2] = -vx_set - vy_set + yaw_term;
        wheel_speed[3] = vx_set - vy_set + yaw_term;
    }
    else
    {
        wheel_speed[0] = vx_set - vy_set - yaw_term;
        wheel_speed[1] = vx_set + vy_set + yaw_term;
        wheel_speed[2] = vx_set + vy_set - yaw_term;
        wheel_speed[3] = vx_set - vy_set + yaw_term;
    }
}

static inline void chassis_core_limit_wheel_speed(fp32 max_wheel_speed,
                                                  fp32 wheel_speed[CHASSIS_CORE_WHEEL_COUNT])
{
    uint8_t i;
    fp32 max_abs = 0.0f;

    if (wheel_speed == NULL || max_wheel_speed <= 0.0f)
    {
        return;
    }

    for (i = 0u; i < CHASSIS_CORE_WHEEL_COUNT; i++)
    {
        const fp32 value_abs = chassis_core_abs_fp32(wheel_speed[i]);
        if (max_abs < value_abs)
        {
            max_abs = value_abs;
        }
    }

    if (max_abs > max_wheel_speed)
    {
        const fp32 scale = max_wheel_speed / max_abs;
        for (i = 0u; i < CHASSIS_CORE_WHEEL_COUNT; i++)
        {
            wheel_speed[i] *= scale;
        }
    }
}

static inline void chassis_core_step_velocity(const chassis_core_kinematic_config_t *cfg,
                                              const chassis_core_cmd_t *cmd,
                                              chassis_core_output_t *out)
{
    if (out == NULL)
    {
        return;
    }

    if (cmd == NULL || cmd->enabled == 0u)
    {
        for (uint8_t i = 0u; i < CHASSIS_CORE_WHEEL_COUNT; i++)
        {
            out->wheel_speed_set[i] = 0.0f;
        }
        return;
    }

    chassis_core_vector_to_wheel_speed(cfg, cmd->vx_mps, cmd->vy_mps, cmd->wz_radps, out->wheel_speed_set);
    if (cmd->raw_output == 0u && cfg != NULL)
    {
        chassis_core_limit_wheel_speed(cfg->max_wheel_speed, out->wheel_speed_set);
    }
}

static inline void chassis_core_fill_current_output(chassis_core_output_t *out)
{
    uint8_t i;

    if (out == NULL)
    {
        return;
    }

    for (i = 0u; i < CHASSIS_CORE_WHEEL_COUNT; i++)
    {
        control_core_cmd_set_current(&out->actuator[i], out->current[i]);
    }
}

#ifdef __cplusplus
}
#endif

#endif
