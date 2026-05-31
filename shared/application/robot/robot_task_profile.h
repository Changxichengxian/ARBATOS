/*
 * SPDX-FileCopyrightText: 2026 陈轮 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轮 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-09
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#pragma once

#include <stdint.h>
#include <string.h>

#include "config.h"

// Platform defaults. A target can override these macros in project defines
// without changing the fast-path task code.
#ifndef ROBOT_PROFILE_GIMBAL_CONTROL_DEFAULT_PERIOD_MS
#define ROBOT_PROFILE_GIMBAL_CONTROL_DEFAULT_PERIOD_MS 1u
#endif

#ifndef ROBOT_PROFILE_CHASSIS_CONTROL_DEFAULT_PERIOD_MS
#define ROBOT_PROFILE_CHASSIS_CONTROL_DEFAULT_PERIOD_MS 2u
#endif

#ifndef ROBOT_PROFILE_CAN_COMMAND_TX_PERIOD_MS
#define ROBOT_PROFILE_CAN_COMMAND_TX_PERIOD_MS 1u
#endif

#ifndef ROBOT_PROFILE_CAN_COMMAND_TX_LOG_PERIOD_MS
#define ROBOT_PROFILE_CAN_COMMAND_TX_LOG_PERIOD_MS 10u
#endif

#ifndef ROBOT_PROFILE_CAN_FEEDBACK_RX_MAX_FRAMES_PER_WAKE
#define ROBOT_PROFILE_CAN_FEEDBACK_RX_MAX_FRAMES_PER_WAKE 32u
#endif

#ifndef ROBOT_PROFILE_CAN_FEEDBACK_RX_BUDGET_US
#define ROBOT_PROFILE_CAN_FEEDBACK_RX_BUDGET_US 200u
#endif

#ifndef ROBOT_PROFILE_GIMBAL_CONTROL_BUDGET_US
#define ROBOT_PROFILE_GIMBAL_CONTROL_BUDGET_US 700u
#endif

#ifndef ROBOT_PROFILE_CHASSIS_CONTROL_BUDGET_US
#define ROBOT_PROFILE_CHASSIS_CONTROL_BUDGET_US 1200u
#endif

#ifndef ROBOT_PROFILE_CAN_COMMAND_TX_BUDGET_US
#define ROBOT_PROFILE_CAN_COMMAND_TX_BUDGET_US 300u
#endif

#ifndef ROBOT_PROFILE_CAN_FEEDBACK_RX_PROFILE_BUDGET_US
#define ROBOT_PROFILE_CAN_FEEDBACK_RX_PROFILE_BUDGET_US 300u
#endif

#ifndef ROBOT_PROFILE_SDLOG_WRITE_BUDGET_US
#define ROBOT_PROFILE_SDLOG_WRITE_BUDGET_US 50u
#endif

#ifndef ROBOT_PROFILE_SDLOG_COMPRESS_BUDGET_US
#define ROBOT_PROFILE_SDLOG_COMPRESS_BUDGET_US 500u
#endif

#ifndef ROBOT_PROFILE_SDLOG_BLOCK_WRITE_BUDGET_US
#define ROBOT_PROFILE_SDLOG_BLOCK_WRITE_BUDGET_US 5000u
#endif

#ifndef ROBOT_PROFILE_SDLOG_SYNC_BUDGET_US
#define ROBOT_PROFILE_SDLOG_SYNC_BUDGET_US 10000u
#endif

#ifndef ROBOT_PROFILE_WATCH_TASK_BEAT_BUDGET_US
#define ROBOT_PROFILE_WATCH_TASK_BEAT_BUDGET_US 10u
#endif

#ifndef ROBOT_PROFILE_WATCH_TASK_BEAT_MIN_PERIOD_MS
#define ROBOT_PROFILE_WATCH_TASK_BEAT_MIN_PERIOD_MS 10u
#endif

static inline uint16_t robot_profile_period_or_default_u16(uint16_t value, uint16_t fallback)
{
    return (value == 0u) ? fallback : value;
}

static inline uint16_t robot_profile_gimbal_control_period_ms(void)
{
    return robot_profile_period_or_default_u16(g_config.gimbal.control_period_ms,
                                               ROBOT_PROFILE_GIMBAL_CONTROL_DEFAULT_PERIOD_MS);
}

static inline uint16_t robot_profile_chassis_control_period_ms(void)
{
    return robot_profile_period_or_default_u16(g_config.chassis.control_period_ms,
                                               ROBOT_PROFILE_CHASSIS_CONTROL_DEFAULT_PERIOD_MS);
}

static inline float robot_profile_chassis_control_period_s(void)
{
    return (float)robot_profile_chassis_control_period_ms() * 0.001f;
}

static inline uint16_t robot_profile_can_command_tx_period_ms(void)
{
    return robot_profile_period_or_default_u16(ROBOT_PROFILE_CAN_COMMAND_TX_PERIOD_MS, 1u);
}

static inline uint16_t robot_profile_can_command_tx_log_period_ms(void)
{
    return robot_profile_period_or_default_u16(ROBOT_PROFILE_CAN_COMMAND_TX_LOG_PERIOD_MS,
                                               robot_profile_can_command_tx_period_ms());
}

static inline uint32_t robot_profile_can_feedback_rx_max_frames_per_wake(void)
{
    return (ROBOT_PROFILE_CAN_FEEDBACK_RX_MAX_FRAMES_PER_WAKE == 0u) ?
               1u :
               (uint32_t)ROBOT_PROFILE_CAN_FEEDBACK_RX_MAX_FRAMES_PER_WAKE;
}

static inline uint32_t robot_profile_can_feedback_rx_budget_us(void)
{
    return (uint32_t)ROBOT_PROFILE_CAN_FEEDBACK_RX_BUDGET_US;
}

typedef struct
{
    robot_task_module_e module;
    const char *name;
} robot_task_module_desc_t;

static inline const robot_task_module_desc_t *robot_profile_known_modules(uint8_t *count)
{
    static const robot_task_module_desc_t modules[] = {
        {ROBOT_TASK_MODULE_RC_SBUS, "task.rc_sbus"},
        {ROBOT_TASK_MODULE_HEALTH_MONITOR, "task.health_monitor"},
        {ROBOT_TASK_MODULE_SDLOG, "task.sdlog"},
        {ROBOT_TASK_MODULE_CAN_COMMAND_TX, "task.can_command_tx"},
        {ROBOT_TASK_MODULE_CAN_FEEDBACK_RX, "task.can_feedback_rx"},
        {ROBOT_TASK_MODULE_CLASSIC_CHASSIS, "task.classic_chassis"},
        {ROBOT_TASK_MODULE_WHEELLEG_SERVO, "task.wheelleg_servo"},
        {ROBOT_TASK_MODULE_WHEELLEG_MIT, "task.wheelleg_mit"},
        {ROBOT_TASK_MODULE_SINGLE_GIMBAL, "task.single_gimbal"},
        {ROBOT_TASK_MODULE_DUAL_YAW_GIMBAL, "task.dual_yaw_gimbal"},
        {ROBOT_TASK_MODULE_ARM, "task.arm"},
        {ROBOT_TASK_MODULE_IMU, "task.imu"},
        {ROBOT_TASK_MODULE_HOST_LINK, "task.host_link"},
        {ROBOT_TASK_MODULE_ELRS_LINK, "task.elrs_link"},
        {ROBOT_TASK_MODULE_REFEREE_RX, "task.referee_rx"},
        {ROBOT_TASK_MODULE_BATTERY_MONITOR, "task.battery_monitor"},
        {ROBOT_TASK_MODULE_SERVO, "task.servo"},
        {ROBOT_TASK_MODULE_CALIBRATION, "task.calibration"},
        {ROBOT_TASK_MODULE_STATUS_LED, "task.status_led"},
        {ROBOT_TASK_MODULE_STARTUP_SERVICE, "task.startup_service"},
    };

    if (count != NULL)
    {
        *count = (uint8_t)(sizeof(modules) / sizeof(modules[0]));
    }

    return modules;
}

static inline uint8_t robot_profile_module_count(void)
{
    const uint8_t count = g_config.profile.task_module_count;

    return (count > ROBOT_TASK_MODULE_MAX) ? ROBOT_TASK_MODULE_MAX : count;
}

static inline robot_task_module_e robot_profile_module_at(uint8_t index)
{
    if (index >= robot_profile_module_count())
    {
        return ROBOT_TASK_MODULE_NONE;
    }

    return (robot_task_module_e)g_config.profile.task_modules[index];
}

static inline const char *robot_profile_module_name(robot_task_module_e module)
{
    uint8_t count = 0u;
    const robot_task_module_desc_t *modules = robot_profile_known_modules(&count);

    for (uint8_t i = 0u; i < count; i++)
    {
        if (modules[i].module == module)
        {
            return modules[i].name;
        }
    }

    return "task.unknown";
}

static inline uint8_t robot_profile_find_module_by_name(const char *name, robot_task_module_e *out)
{
    uint8_t count = 0u;
    const robot_task_module_desc_t *modules = robot_profile_known_modules(&count);

    if (name == NULL)
    {
        return 0u;
    }

    for (uint8_t i = 0u; i < count; i++)
    {
        if (strcmp(modules[i].name, name) == 0)
        {
            if (out != NULL)
            {
                *out = modules[i].module;
            }
            return 1u;
        }
    }

    return 0u;
}

static inline uint8_t robot_profile_module_enabled(robot_task_module_e module)
{
    const uint8_t limit = robot_profile_module_count();

    for (uint8_t i = 0u; i < limit; i++)
    {
        if (robot_profile_module_at(i) == module)
        {
            return 1u;
        }
    }

    return 0u;
}

static inline uint8_t robot_profile_module_enabled_by_name(const char *name)
{
    robot_task_module_e module = ROBOT_TASK_MODULE_NONE;

    if (robot_profile_find_module_by_name(name, &module) == 0u)
    {
        return 0u;
    }

    return robot_profile_module_enabled(module);
}

static inline uint8_t robot_profile_need_classic_chassis_control_task(void)
{
    return robot_profile_module_enabled(ROBOT_TASK_MODULE_CLASSIC_CHASSIS);
}

static inline uint8_t robot_profile_need_wheelleg_servo_task(void)
{
    return robot_profile_module_enabled(ROBOT_TASK_MODULE_WHEELLEG_SERVO);
}

static inline uint8_t robot_profile_is_wheelleg_mit(void)
{
    return robot_profile_module_enabled(ROBOT_TASK_MODULE_WHEELLEG_MIT);
}

static inline uint8_t robot_profile_need_single_gimbal_control_task(void)
{
    return robot_profile_module_enabled(ROBOT_TASK_MODULE_SINGLE_GIMBAL);
}

static inline uint8_t robot_profile_need_dual_gimbal_control_task(void)
{
    return robot_profile_module_enabled(ROBOT_TASK_MODULE_DUAL_YAW_GIMBAL);
}

static inline uint8_t robot_profile_need_arm_task(void)
{
    return robot_profile_module_enabled(ROBOT_TASK_MODULE_ARM);
}
