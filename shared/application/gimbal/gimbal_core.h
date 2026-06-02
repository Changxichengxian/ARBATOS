/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef GIMBAL_CORE_H
#define GIMBAL_CORE_H

#include <stdint.h>

#include "control_core.h"
#include "gimbal_pid.h"
#include "pid.h"
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    GIMBAL_CORE_AXIS_YAW = 0u,
    GIMBAL_CORE_AXIS_PITCH,
    GIMBAL_CORE_AXIS_COUNT
} gimbal_core_axis_e;

typedef enum
{
    GIMBAL_CORE_MODE_RAW = 0u,
    GIMBAL_CORE_MODE_ANGLE = 1u,
} gimbal_core_mode_e;

typedef struct
{
    fp32 angle_rad;
    fp32 angle_set_rad;
    fp32 gyro_radps;
    fp32 gyro_set_radps;
    fp32 motor_speed_radps;
    uint8_t online;
} gimbal_core_axis_state_t;

typedef struct
{
    uint8_t enabled;
    uint8_t raw_current;
    fp32 yaw_delta_rad;
    fp32 pitch_delta_rad;
    fp32 yaw_rate_radps;
    fp32 pitch_rate_radps;
} gimbal_core_cmd_t;

typedef struct
{
    uint8_t mode; // gimbal_core_mode_e
    fp32 angle_rad;
    fp32 angle_set_rad;
    fp32 gyro_radps;
    fp32 gyro_set_radps;
    fp32 raw_current;
} gimbal_core_axis_input_t;

typedef struct
{
    fp32 gyro_set_radps;
    fp32 current_set;
    int16_t given_current;
    MotorCmd actuator;
} gimbal_core_axis_output_t;

typedef struct
{
    int16_t current[GIMBAL_CORE_AXIS_COUNT];
    MotorCmd actuator[GIMBAL_CORE_AXIS_COUNT];
} gimbal_core_output_t;

static inline void gimbal_core_step_axis_base(gimbal_PID_t *angle_pid,
                                              pid_type_def *gyro_pid,
                                              const gimbal_core_axis_input_t *in,
                                              gimbal_core_axis_output_t *out)
{
    if (out == NULL)
    {
        return;
    }

    out->gyro_set_radps = 0.0f;
    out->current_set = 0.0f;
    out->given_current = 0;
    control_core_cmd_clear(&out->actuator);

    if (in == NULL)
    {
        return;
    }

    if (in->mode == (uint8_t)GIMBAL_CORE_MODE_RAW)
    {
        out->gyro_set_radps = in->gyro_set_radps;
        out->current_set = in->raw_current;
    }
    else if (angle_pid != NULL && gyro_pid != NULL)
    {
        out->gyro_set_radps = gimbal_PID_calc(angle_pid, in->angle_rad, in->angle_set_rad, in->gyro_radps);
        out->current_set = PID_calc(gyro_pid, in->gyro_radps, out->gyro_set_radps);
    }

    out->given_current = (int16_t)out->current_set;
    control_core_cmd_set_current(&out->actuator, out->given_current);
}

static inline void gimbal_core_fill_current_output(gimbal_core_output_t *out)
{
    uint8_t i;

    if (out == NULL)
    {
        return;
    }

    for (i = 0u; i < (uint8_t)GIMBAL_CORE_AXIS_COUNT; i++)
    {
        control_core_cmd_set_current(&out->actuator[i], out->current[i]);
    }
}

#ifdef __cplusplus
}
#endif

#endif
