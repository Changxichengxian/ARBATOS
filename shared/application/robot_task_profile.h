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

static inline uint8_t robot_profile_need_classic_chassis_control_task(void)
{
    return (uint8_t)(g_config.profile.locomotion_family == LOCOMOTION_FAMILY_CLASSIC_CHASSIS);
}

static inline uint8_t robot_profile_need_wheelleg_servo_task(void)
{
    return (uint8_t)(g_config.profile.locomotion_family == LOCOMOTION_FAMILY_WHEELLEG_SERVO);
}

static inline uint8_t robot_profile_need_wheelleg_mit_task(void)
{
    return (uint8_t)(g_config.profile.locomotion_family == LOCOMOTION_FAMILY_WHEELLEG_MIT);
}

static inline uint8_t robot_profile_need_single_gimbal_control_task(void)
{
    return (uint8_t)(g_config.profile.gimbal_family == GIMBAL_FAMILY_SINGLE);
}

static inline uint8_t robot_profile_need_dual_gimbal_control_task(void)
{
    return (uint8_t)(g_config.profile.gimbal_family == GIMBAL_FAMILY_DUAL);
}

static inline uint8_t robot_profile_need_arm_task(void)
{
    return (uint8_t)(g_config.profile.arm_family != ARM_FAMILY_NONE);
}
