/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef GIMBAL_CORE_H
#define GIMBAL_CORE_H

#include <stdint.h>

#include "ControlCore.h"
#include "GimbalPid.h"
#include "Pid.h"
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    GIMBAL_CORE_AXIS_YAW = 0u,
    GIMBAL_CORE_AXIS_PITCH,
    GIMBAL_CORE_AXIS_COUNT
} GimbalCoreAxis;

typedef enum
{
    GIMBAL_CORE_MODE_RAW = 0u,
    GIMBAL_CORE_MODE_ANGLE = 1u,
} GimbalCoreMode;

typedef struct
{
    fp32 angle_rad;
    fp32 angle_set_rad;
    fp32 gyro_radps;
    fp32 gyro_set_radps;
    fp32 motor_speed_radps;
    uint8_t online;
} GimbalCoreAxisState;

typedef struct
{
    uint8_t enabled;
    uint8_t raw_current;
    fp32 yaw_delta_rad;
    fp32 pitch_delta_rad;
    fp32 yaw_rate_radps;
    fp32 pitch_rate_radps;
} GimbalCoreCmd;

typedef struct
{
    uint8_t mode; // GimbalCoreMode
    fp32 angle_rad;
    fp32 angle_set_rad;
    fp32 gyro_radps;
    fp32 gyro_set_radps;
    fp32 raw_current;
} GimbalCoreAxisInput;

typedef struct
{
    fp32 gyro_set_radps;
    fp32 current_set;
    int16_t given_current;
    MotorCmd actuator;
} GimbalCoreAxisOutput;

typedef struct
{
    int16_t current[GIMBAL_CORE_AXIS_COUNT];
    MotorCmd actuator[GIMBAL_CORE_AXIS_COUNT];
} GimbalCoreOutput;

static inline void GimbalCoreStepAxisBase(GimbalPid *angle_pid,
                                              PidTypeDef *gyro_pid,
                                              const GimbalCoreAxisInput *in,
                                              GimbalCoreAxisOutput *out)
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
        out->gyro_set_radps = GimbalPidCalc(angle_pid, in->angle_rad, in->angle_set_rad, in->gyro_radps);
        out->current_set = PID_calc(gyro_pid, in->gyro_radps, out->gyro_set_radps);
    }

    out->given_current = (int16_t)out->current_set;
    control_core_cmd_set_current(&out->actuator, out->given_current);
}

static inline void GimbalCoreFillCurrentOutput(GimbalCoreOutput *out)
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
