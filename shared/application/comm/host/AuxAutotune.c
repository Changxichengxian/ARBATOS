/*
 * SPDX-FileCopyrightText: 2026 Chen Xuan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "AuxAutotune.h"

#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "AuxPort.h"
#include "BspUsart.h"
#include "ChassisControlTask.h"
#include "GimbalControlTask.h"
#include "HostTuneBridge.h"
#include "RobotTaskProfile.h"
#include "user_lib.h"

typedef struct
{
    uint8_t enabled;
    uint8_t target;
    uint16_t period_ms;
    uint32_t last_tick_ms;
} AuxAutotuneStream;

static AuxAutotuneStream AuxAutotune = {0};
static uint8_t AuxAutotuneFrame[(8u + 1u) * 4u];

void AuxAutotuneResetTiming(void)
{
    AuxAutotune.last_tick_ms = 0u;
}

void AuxAutotuneSetPeriodMs(uint32_t period_ms)
{
    if (period_ms > 1000u)
    {
        period_ms = 1000u;
    }
    AuxAutotune.period_ms = (uint16_t)period_ms;
    AuxAutotune.last_tick_ms = 0u;
}

bool_t AuxAutotuneStart(AuxAutotuneTarget target)
{
    if (!AuxAutotuneTargetIsActive(target))
    {
        return 0;
    }

    AuxAutotune.enabled = 1u;
    AuxAutotune.target = (uint8_t)target;
    if (AuxAutotune.period_ms == 0u)
    {
        AuxAutotune.period_ms = 20u;
    }
    AuxAutotune.last_tick_ms = 0u;
    return 1;
}

void AuxAutotuneStop(void)
{
    AuxAutotune.enabled = 0u;
    AuxAutotune.target = (uint8_t)AUX_AUTOTUNE_TARGET_NONE;
    AuxAutotune.last_tick_ms = 0u;
    if (AuxAutotune.period_ms == 0u)
    {
        AuxAutotune.period_ms = 20u;
    }
}

bool_t AuxAutotuneParseTarget(const char *s, AuxAutotuneTarget *out)
{
    if (s == NULL || out == NULL)
    {
        return 0;
    }

    if (strcmp(s, "ps") == 0 || strcmp(s, "pitch_speed") == 0)
    {
        *out = AUX_AUTOTUNE_TARGET_PITCH_SPEED;
        return 1;
    }
    if (strcmp(s, "pa") == 0 || strcmp(s, "pitch_angle") == 0)
    {
        *out = AUX_AUTOTUNE_TARGET_PITCH_ANGLE;
        return 1;
    }
    if (strcmp(s, "ys") == 0 || strcmp(s, "yaw_speed") == 0)
    {
        *out = AUX_AUTOTUNE_TARGET_YAW_SPEED;
        return 1;
    }
    if (strcmp(s, "ya") == 0 || strcmp(s, "yaw_angle") == 0)
    {
        *out = AUX_AUTOTUNE_TARGET_YAW_ANGLE;
        return 1;
    }
    if (strcmp(s, "cf") == 0 || strcmp(s, "ChassisFollow") == 0)
    {
        *out = AUX_AUTOTUNE_TARGET_CHASSIS_FOLLOW;
        return 1;
    }
    if (strcmp(s, "cm") == 0 || strcmp(s, "ChassisMotorSpeed") == 0)
    {
        *out = AUX_AUTOTUNE_TARGET_CHASSIS_MOTOR_SPEED;
        return 1;
    }

    return 0;
}

bool_t AuxAutotuneTargetIsActive(AuxAutotuneTarget target)
{
    switch (target)
    {
    case AUX_AUTOTUNE_TARGET_PITCH_SPEED:
    case AUX_AUTOTUNE_TARGET_PITCH_ANGLE:
    case AUX_AUTOTUNE_TARGET_YAW_SPEED:
    case AUX_AUTOTUNE_TARGET_YAW_ANGLE:
        return (bool_t)RobotProfileNeedSingleGimbalControlTask();
    case AUX_AUTOTUNE_TARGET_CHASSIS_FOLLOW:
    case AUX_AUTOTUNE_TARGET_CHASSIS_MOTOR_SPEED:
        return (bool_t)RobotProfileNeedClassicChassisControlTask();
    default:
        return 0;
    }
}

static bool_t AuxAutotuneFillFields(fp32 *fields, uint32_t *out_tick_ms)
{
    if (fields == NULL || out_tick_ms == NULL)
    {
        return 0;
    }

    // Pack the selected PID into fixed fields for the tuning stream.
    const uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    *out_tick_ms = now_ms;
    fields[0] = (fp32)now_ms;

    if (!AuxAutotuneTargetIsActive((AuxAutotuneTarget)AuxAutotune.target))
    {
        return 0;
    }

    switch ((AuxAutotuneTarget)AuxAutotune.target)
    {
    case AUX_AUTOTUNE_TARGET_PITCH_SPEED:
    {
        const GimbalMotor *motor = get_pitch_motor_point();
        if (motor == NULL)
        {
            return 0;
        }
        const PidTypeDef *pid = &motor->GimbalMotorGyroPid;
        fields[1] = pid->set;
        fields[2] = pid->fdb;
        fields[3] = pid->out;
        fields[4] = pid->error[0];
        fields[5] = pid->Kp;
        fields[6] = pid->Ki;
        fields[7] = pid->Kd;
        return 1;
    }
    case AUX_AUTOTUNE_TARGET_PITCH_ANGLE:
    {
        const GimbalMotor *motor = get_pitch_motor_point();
        if (motor == NULL)
        {
            return 0;
        }
        const GimbalPid *pid = &motor->GimbalMotorAnglePid;
        fields[1] = pid->set;
        fields[2] = pid->get;
        fields[3] = pid->out;
        fields[4] = pid->err;
        fields[5] = pid->kp;
        fields[6] = pid->ki;
        fields[7] = pid->kd;
        return 1;
    }
    case AUX_AUTOTUNE_TARGET_YAW_SPEED:
    {
        const GimbalMotor *motor = get_yaw_motor_point();
        if (motor == NULL)
        {
            return 0;
        }
        const PidTypeDef *pid = &motor->GimbalMotorGyroPid;
        fields[1] = pid->set;
        fields[2] = pid->fdb;
        fields[3] = pid->out;
        fields[4] = pid->error[0];
        fields[5] = pid->Kp;
        fields[6] = pid->Ki;
        fields[7] = pid->Kd;
        return 1;
    }
    case AUX_AUTOTUNE_TARGET_YAW_ANGLE:
    {
        const GimbalMotor *motor = get_yaw_motor_point();
        if (motor == NULL)
        {
            return 0;
        }
        const GimbalPid *pid = &motor->GimbalMotorAnglePid;
        fields[1] = pid->set;
        fields[2] = pid->get;
        fields[3] = pid->out;
        fields[4] = pid->err;
        fields[5] = pid->kp;
        fields[6] = pid->ki;
        fields[7] = pid->kd;
        return 1;
    }
    case AUX_AUTOTUNE_TARGET_CHASSIS_FOLLOW:
    {
        const ChassisMove *chassis = get_chassis_move_point();
        if (chassis == NULL)
        {
            return 0;
        }
        const PidTypeDef *pid = &chassis->ChassisAnglePid;
        const fp32 setpoint = chassis->ChassisYawOffsetSet;
        const fp32 input = chassis->ChassisYawOffset;
        fields[1] = setpoint;
        fields[2] = input;
        fields[3] = pid->out;
        fields[4] = rad_format(setpoint - input);
        fields[5] = pid->Kp;
        fields[6] = pid->Ki;
        fields[7] = pid->Kd;
        return 1;
    }
    case AUX_AUTOTUNE_TARGET_CHASSIS_MOTOR_SPEED:
    {
        const ChassisMove *chassis = get_chassis_move_point();
        if (chassis == NULL)
        {
            return 0;
        }

        fp32 set_sum = 0.0f;
        fp32 fdb_sum = 0.0f;
        fp32 out_sum = 0.0f;
        for (uint8_t i = 0u; i < 4u; i++)
        {
            const PidTypeDef *PidI = &chassis->motor_speed_pid[i];
            set_sum += PidI->set;
            fdb_sum += PidI->fdb;
            out_sum += PidI->out;
        }

        const PidTypeDef *pid = &chassis->motor_speed_pid[0];
        fields[1] = set_sum * 0.25f;
        fields[2] = fdb_sum * 0.25f;
        fields[3] = out_sum * 0.25f;
        fields[4] = fields[1] - fields[2];
        fields[5] = pid->Kp;
        fields[6] = pid->Ki;
        fields[7] = pid->Kd;
        return 1;
    }
    case AUX_AUTOTUNE_TARGET_NONE:
    default:
        return 0;
    }
}

bool_t AuxAutotuneTrySendFrame(void)
{
    if (AuxAutotune.enabled == 0u)
    {
        return 0;
    }
    if (!AuxPortIsTuneMode(BspAuxLinkGetBaudrate()))
    {
        return 0;
    }

    uint16_t period_ms = AuxAutotune.period_ms;
    if (period_ms == 0u)
    {
        period_ms = 20u;
    }

    const uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    if ((uint32_t)(now_ms - AuxAutotune.last_tick_ms) < period_ms)
    {
        return 0;
    }

    if (!BspAuxLinkTxReady())
    {
        return 0;
    }

    fp32 fields[8] = {0};
    uint32_t sample_tick_ms = 0u;
    if (!AuxAutotuneFillFields(fields, &sample_tick_ms))
    {
        return 0;
    }

    for (uint8_t i = 0u; i < 8u; i++)
    {
        memcpy(&AuxAutotuneFrame[i * 4u], &fields[i], 4u);
    }

    const uint32_t tail = 0x7F800000u;
    memcpy(&AuxAutotuneFrame[8u * 4u], &tail, 4u);

    if (BspAuxLinkTxDma(AuxAutotuneFrame, (uint16_t)sizeof(AuxAutotuneFrame)) == 0)
    {
        AuxAutotune.last_tick_ms = sample_tick_ms;
        return 1;
    }

    return 0;
}
