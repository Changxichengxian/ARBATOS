/*
 * SPDX-FileCopyrightText: 2026 Chen Xuan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "aux_autotune.h"

#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "aux_port.h"
#include "bsp_usart.h"
#include "chassis_control_task.h"
#include "gimbal_control_task.h"
#include "host_tune_bridge.h"
#include "robot_task_profile.h"
#include "user_lib.h"

typedef struct
{
    uint8_t enabled;
    uint8_t target;
    uint16_t period_ms;
    uint32_t last_tick_ms;
} aux_autotune_stream_t;

static aux_autotune_stream_t aux_autotune = {0};
static uint8_t aux_autotune_frame[(8u + 1u) * 4u];

void aux_autotune_reset_timing(void)
{
    aux_autotune.last_tick_ms = 0u;
}

void aux_autotune_set_period_ms(uint32_t period_ms)
{
    if (period_ms > 1000u)
    {
        period_ms = 1000u;
    }
    aux_autotune.period_ms = (uint16_t)period_ms;
    aux_autotune.last_tick_ms = 0u;
}

bool_t aux_autotune_start(aux_autotune_target_e target)
{
    if (!aux_autotune_target_is_active(target))
    {
        return 0;
    }

    aux_autotune.enabled = 1u;
    aux_autotune.target = (uint8_t)target;
    if (aux_autotune.period_ms == 0u)
    {
        aux_autotune.period_ms = 20u;
    }
    aux_autotune.last_tick_ms = 0u;
    return 1;
}

void aux_autotune_stop(void)
{
    aux_autotune.enabled = 0u;
    aux_autotune.target = (uint8_t)AUX_AUTOTUNE_TARGET_NONE;
    aux_autotune.last_tick_ms = 0u;
    if (aux_autotune.period_ms == 0u)
    {
        aux_autotune.period_ms = 20u;
    }
}

bool_t aux_autotune_parse_target(const char *s, aux_autotune_target_e *out)
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
    if (strcmp(s, "cf") == 0 || strcmp(s, "chassis_follow") == 0)
    {
        *out = AUX_AUTOTUNE_TARGET_CHASSIS_FOLLOW;
        return 1;
    }
    if (strcmp(s, "cm") == 0 || strcmp(s, "chassis_motor_speed") == 0)
    {
        *out = AUX_AUTOTUNE_TARGET_CHASSIS_MOTOR_SPEED;
        return 1;
    }

    return 0;
}

bool_t aux_autotune_target_is_active(aux_autotune_target_e target)
{
    switch (target)
    {
    case AUX_AUTOTUNE_TARGET_PITCH_SPEED:
    case AUX_AUTOTUNE_TARGET_PITCH_ANGLE:
    case AUX_AUTOTUNE_TARGET_YAW_SPEED:
    case AUX_AUTOTUNE_TARGET_YAW_ANGLE:
        return (bool_t)robot_profile_need_single_gimbal_control_task();
    case AUX_AUTOTUNE_TARGET_CHASSIS_FOLLOW:
    case AUX_AUTOTUNE_TARGET_CHASSIS_MOTOR_SPEED:
        return (bool_t)robot_profile_need_classic_chassis_control_task();
    default:
        return 0;
    }
}

static bool_t aux_autotune_fill_fields(fp32 *fields, uint32_t *out_tick_ms)
{
    if (fields == NULL || out_tick_ms == NULL)
    {
        return 0;
    }

    // Pack the selected PID into fixed fields for the tuning stream.
    const uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    *out_tick_ms = now_ms;
    fields[0] = (fp32)now_ms;

    if (!aux_autotune_target_is_active((aux_autotune_target_e)aux_autotune.target))
    {
        return 0;
    }

    switch ((aux_autotune_target_e)aux_autotune.target)
    {
    case AUX_AUTOTUNE_TARGET_PITCH_SPEED:
    {
        const gimbal_motor_t *motor = get_pitch_motor_point();
        if (motor == NULL)
        {
            return 0;
        }
        const pid_type_def *pid = &motor->gimbal_motor_gyro_pid;
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
        const gimbal_motor_t *motor = get_pitch_motor_point();
        if (motor == NULL)
        {
            return 0;
        }
        const gimbal_PID_t *pid = &motor->gimbal_motor_angle_pid;
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
        const gimbal_motor_t *motor = get_yaw_motor_point();
        if (motor == NULL)
        {
            return 0;
        }
        const pid_type_def *pid = &motor->gimbal_motor_gyro_pid;
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
        const gimbal_motor_t *motor = get_yaw_motor_point();
        if (motor == NULL)
        {
            return 0;
        }
        const gimbal_PID_t *pid = &motor->gimbal_motor_angle_pid;
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
        const chassis_move_t *chassis = get_chassis_move_point();
        if (chassis == NULL)
        {
            return 0;
        }
        const pid_type_def *pid = &chassis->chassis_angle_pid;
        const fp32 setpoint = chassis->chassis_yaw_offset_set;
        const fp32 input = chassis->chassis_yaw_offset;
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
        const chassis_move_t *chassis = get_chassis_move_point();
        if (chassis == NULL)
        {
            return 0;
        }

        fp32 set_sum = 0.0f;
        fp32 fdb_sum = 0.0f;
        fp32 out_sum = 0.0f;
        for (uint8_t i = 0u; i < 4u; i++)
        {
            const pid_type_def *pid_i = &chassis->motor_speed_pid[i];
            set_sum += pid_i->set;
            fdb_sum += pid_i->fdb;
            out_sum += pid_i->out;
        }

        const pid_type_def *pid = &chassis->motor_speed_pid[0];
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

bool_t aux_autotune_try_send_frame(void)
{
    if (aux_autotune.enabled == 0u)
    {
        return 0;
    }
    if (!aux_port_is_tune_mode(bsp_aux_link_get_baudrate()))
    {
        return 0;
    }

    uint16_t period_ms = aux_autotune.period_ms;
    if (period_ms == 0u)
    {
        period_ms = 20u;
    }

    const uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    if ((uint32_t)(now_ms - aux_autotune.last_tick_ms) < period_ms)
    {
        return 0;
    }

    if (!bsp_aux_link_tx_ready())
    {
        return 0;
    }

    fp32 fields[8] = {0};
    uint32_t sample_tick_ms = 0u;
    if (!aux_autotune_fill_fields(fields, &sample_tick_ms))
    {
        return 0;
    }

    for (uint8_t i = 0u; i < 8u; i++)
    {
        memcpy(&aux_autotune_frame[i * 4u], &fields[i], 4u);
    }

    const uint32_t tail = 0x7F800000u;
    memcpy(&aux_autotune_frame[8u * 4u], &tail, 4u);

    if (bsp_aux_link_tx_dma(aux_autotune_frame, (uint16_t)sizeof(aux_autotune_frame)) == 0)
    {
        aux_autotune.last_tick_ms = sample_tick_ms;
        return 1;
    }

    return 0;
}
