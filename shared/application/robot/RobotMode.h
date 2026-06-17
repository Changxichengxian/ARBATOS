/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ROBOT_MODE_H
#define ROBOT_MODE_H

#include "config.h"

static inline robot_run_mode_e robot_mode_current(void)
{
    const uint8_t mode = g_config.operation.mode;
    return (mode <= (uint8_t)ROBOT_RUN_MODE_MAX) ?
               (robot_run_mode_e)mode :
               ROBOT_RUN_MODE_FULL;
}

static inline robot_run_variant_e robot_mode_variant(void)
{
    const uint8_t variant = g_config.operation.variant;
    return (variant <= (uint8_t)ROBOT_RUN_VARIANT_MAX) ?
               (robot_run_variant_e)variant :
               ROBOT_RUN_VARIANT_NORMAL;
}

static inline robot_cali_target_e robot_mode_cali_target(void)
{
    const uint8_t target = g_config.operation.cali_target;
    return (target <= (uint8_t)ROBOT_CALI_TARGET_MAX) ?
               (robot_cali_target_e)target :
               ROBOT_CALI_TARGET_NONE;
}

static inline MotorId robot_mode_target_motor(void)
{
    const uint8_t id = g_config.operation.target_motor;
    return (id < (uint8_t)MotorCount) ? (MotorId)id : MotorCount;
}

static inline uint8_t robot_mode_is_single_task(RobotTaskModule module)
{
    return (uint8_t)(robot_mode_current() == ROBOT_RUN_MODE_SINGLE_TASK &&
                     g_config.operation.target_task == (uint8_t)module);
}

static inline uint8_t robot_mode_is_entertain(void)
{
    return (uint8_t)(robot_mode_current() == ROBOT_RUN_MODE_ENTERTAIN);
}

static inline uint8_t robot_mode_is_calibration(robot_cali_target_e target)
{
    return (uint8_t)(robot_mode_current() == ROBOT_RUN_MODE_CALIBRATION &&
                     robot_mode_cali_target() == target);
}

static inline uint8_t robot_mode_allow_motor(MotorId id)
{
    if (robot_mode_current() != ROBOT_RUN_MODE_SINGLE_MOTOR)
    {
        return 1u;
    }

    return (uint8_t)(id == robot_mode_target_motor());
}

static inline uint8_t robot_mode_force_chassis_only(void)
{
    return (uint8_t)(robot_mode_is_single_task(ROBOT_TASK_MODULE_CLASSIC_CHASSIS) != 0u ||
                     robot_mode_variant() == ROBOT_RUN_VARIANT_CHASSIS_ONLY);
}

static inline uint8_t robot_mode_allow_chassis(void)
{
    const robot_run_mode_e mode = robot_mode_current();

    if (mode == ROBOT_RUN_MODE_SINGLE_MOTOR)
    {
        return 0u;
    }
    if (mode == ROBOT_RUN_MODE_SINGLE_TASK)
    {
        return robot_mode_is_single_task(ROBOT_TASK_MODULE_CLASSIC_CHASSIS);
    }

    return (uint8_t)(mode == ROBOT_RUN_MODE_FULL ||
                     mode == ROBOT_RUN_MODE_ENTERTAIN);
}

static inline uint8_t robot_mode_gimbal_yaw_only(void)
{
    return (uint8_t)(robot_mode_variant() == ROBOT_RUN_VARIANT_GIMBAL_YAW_ONLY);
}

static inline uint8_t robot_mode_gimbal_yaw_easy(void)
{
    return (uint8_t)(robot_mode_variant() == ROBOT_RUN_VARIANT_GIMBAL_YAW_EASY);
}

static inline uint8_t robot_mode_gimbal_pitch_only(void)
{
    return (uint8_t)(robot_mode_variant() == ROBOT_RUN_VARIANT_GIMBAL_PITCH_ONLY);
}

static inline uint8_t robot_mode_gimbal_pitch_cali(void)
{
    return robot_mode_is_calibration(ROBOT_CALI_TARGET_PITCH);
}

static inline uint8_t robot_mode_allow_shoot_fric(void)
{
    const robot_run_mode_e mode = robot_mode_current();
    const robot_run_variant_e variant = robot_mode_variant();

    if (mode == ROBOT_RUN_MODE_ENTERTAIN ||
        mode == ROBOT_RUN_MODE_CALIBRATION ||
        mode == ROBOT_RUN_MODE_SINGLE_MOTOR)
    {
        return 0u;
    }
    if (mode == ROBOT_RUN_MODE_SINGLE_TASK)
    {
        return (uint8_t)(variant == ROBOT_RUN_VARIANT_SHOOT_FRIC_ONLY ||
                         variant == ROBOT_RUN_VARIANT_SHOOT_COMBO);
    }

    return 1u;
}

static inline uint8_t robot_mode_allow_shoot_trigger(void)
{
    const robot_run_mode_e mode = robot_mode_current();
    const robot_run_variant_e variant = robot_mode_variant();

    if (mode == ROBOT_RUN_MODE_ENTERTAIN ||
        mode == ROBOT_RUN_MODE_CALIBRATION ||
        mode == ROBOT_RUN_MODE_SINGLE_MOTOR)
    {
        return 0u;
    }
    if (mode == ROBOT_RUN_MODE_SINGLE_TASK)
    {
        return (uint8_t)(variant == ROBOT_RUN_VARIANT_SHOOT_TRIGGER_ONLY ||
                         variant == ROBOT_RUN_VARIANT_SHOOT_COMBO);
    }

    return 1u;
}

static inline uint8_t robot_mode_wheelleg_left_leg_swing(void)
{
    return (uint8_t)(robot_mode_variant() == ROBOT_RUN_VARIANT_WHEELLEG_LEFT_LEG_SWING);
}

static inline uint8_t robot_mode_wheelleg_foot_trajectory(void)
{
    return (uint8_t)(robot_mode_variant() == ROBOT_RUN_VARIANT_WHEELLEG_FOOT_TRAJECTORY);
}

#endif
