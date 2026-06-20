/*
 * SPDX-FileCopyrightText: 2026 陈轮 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-09
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#pragma once

#include <stdint.h>
#include <string.h>

#include "RobotConfig.h"

typedef enum
{
    ROBOT_PROFILE_KIND_UNKNOWN = 0u,
    ROBOT_PROFILE_KIND_HERO,
    ROBOT_PROFILE_KIND_INFANTRY,
    ROBOT_PROFILE_KIND_WHEELLEG,
    ROBOT_PROFILE_KIND_SENTRY,
    ROBOT_PROFILE_KIND_CARRIER,
    ROBOT_PROFILE_KIND_CUSTOM,
} RobotProfileKind;

typedef enum
{
    ROBOT_BOARD_KIND_UNKNOWN = 0u,
    ROBOT_BOARD_KIND_STM32F407,
    ROBOT_BOARD_KIND_STM32F427,
    ROBOT_BOARD_KIND_STM32H7,
    ROBOT_BOARD_KIND_CUSTOM,
} RobotBoardKind;

#ifndef ROBOT_PROFILE_KIND
#define ROBOT_PROFILE_KIND ROBOT_PROFILE_KIND_CUSTOM
#endif

#ifndef ROBOT_BOARD_KIND
#define ROBOT_BOARD_KIND ROBOT_BOARD_KIND_CUSTOM
#endif

#ifndef ROBOT_PROFILE_NAME
#ifdef ARBATOS_TARGET_NAME
#define ROBOT_PROFILE_NAME ARBATOS_TARGET_NAME
#else
#define ROBOT_PROFILE_NAME "unknown-target"
#endif
#endif

#ifndef ROBOT_BOARD_NAME
#ifdef ARBATOS_BOARD_NAME
#define ROBOT_BOARD_NAME ARBATOS_BOARD_NAME
#else
#define ROBOT_BOARD_NAME "unknown-board"
#endif
#endif

#ifndef ROBOT_BOARD_CPU_HZ
#define ROBOT_BOARD_CPU_HZ 0u
#endif

#ifndef ROBOT_BOARD_CAN_BUS_COUNT
#define ROBOT_BOARD_CAN_BUS_COUNT 2u
#endif

#ifndef ROBOT_BOARD_HAS_FPU
#define ROBOT_BOARD_HAS_FPU 1u
#endif

typedef struct
{
    const char *profile_name;
    const char *profile_kind_name;
    const char *BoardName;
    const char *BoardKindName;
    uint8_t profile_kind;
    uint8_t BoardKind;
    uint8_t can_bus_count;
    uint8_t has_fpu;
    uint32_t cpu_hz;
} RobotProfileIdentity;

static inline const char *RobotProfileName(void)
{
    return ROBOT_PROFILE_NAME;
}

static inline RobotProfileKind RobotProfileGetKind(void)
{
    return (RobotProfileKind)ROBOT_PROFILE_KIND;
}

static inline const char *RobotProfileKindName(RobotProfileKind kind)
{
    switch (kind)
    {
    case ROBOT_PROFILE_KIND_HERO:
        return "hero";
    case ROBOT_PROFILE_KIND_INFANTRY:
        return "infantry";
    case ROBOT_PROFILE_KIND_WHEELLEG:
        return "wheelleg";
    case ROBOT_PROFILE_KIND_SENTRY:
        return "sentry";
    case ROBOT_PROFILE_KIND_CARRIER:
        return "carrier";
    case ROBOT_PROFILE_KIND_CUSTOM:
        return "custom";
    default:
        return "unknown";
    }
}

static inline const char *RobotBoardName(void)
{
    return ROBOT_BOARD_NAME;
}

static inline RobotBoardKind RobotBoardGetKind(void)
{
    return (RobotBoardKind)ROBOT_BOARD_KIND;
}

static inline const char *RobotBoardKindName(RobotBoardKind kind)
{
    switch (kind)
    {
    case ROBOT_BOARD_KIND_STM32F407:
        return "stm32f407";
    case ROBOT_BOARD_KIND_STM32F427:
        return "stm32f427";
    case ROBOT_BOARD_KIND_STM32H7:
        return "stm32h7";
    case ROBOT_BOARD_KIND_CUSTOM:
        return "custom";
    default:
        return "unknown";
    }
}

static inline uint32_t RobotBoardCpuHz(void)
{
    return (uint32_t)ROBOT_BOARD_CPU_HZ;
}

static inline uint8_t RobotBoardCanBusCount(void)
{
    return (uint8_t)ROBOT_BOARD_CAN_BUS_COUNT;
}

static inline uint8_t RobotBoardHasFpu(void)
{
    return (uint8_t)((ROBOT_BOARD_HAS_FPU != 0u) ? 1u : 0u);
}

static inline void RobotProfileFillIdentity(RobotProfileIdentity *out)
{
    if (out == NULL)
    {
        return;
    }

    out->profile_name = RobotProfileName();
    out->profile_kind = (uint8_t)RobotProfileGetKind();
    out->profile_kind_name = RobotProfileKindName(RobotProfileGetKind());
    out->BoardName = RobotBoardName();
    out->BoardKind = (uint8_t)RobotBoardGetKind();
    out->BoardKindName = RobotBoardKindName(RobotBoardGetKind());
    out->cpu_hz = RobotBoardCpuHz();
    out->can_bus_count = RobotBoardCanBusCount();
    out->has_fpu = RobotBoardHasFpu();
}

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

#ifndef ROBOT_PROFILE_WHEELLEG_MIT_CONTROL_BUDGET_US
#define ROBOT_PROFILE_WHEELLEG_MIT_CONTROL_BUDGET_US 1500u
#endif

#ifndef ROBOT_PROFILE_WHEELLEG_MIT_CONTROL_DEFAULT_PERIOD_MS
#define ROBOT_PROFILE_WHEELLEG_MIT_CONTROL_DEFAULT_PERIOD_MS 3u
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

#ifndef ROBOT_TASK_MODULE_CUSTOM_BASE
#define ROBOT_TASK_MODULE_CUSTOM_BASE 128u
#endif

typedef uint8_t RobotTaskModuleId;

static inline uint16_t RobotProfilePeriodOrDefaultU16(uint16_t value, uint16_t fallback)
{
    return (value == 0u) ? fallback : value;
}

static inline uint16_t RobotProfileGimbalControlPeriodMs(void)
{
    return RobotProfilePeriodOrDefaultU16(g_config.gimbal.control_period_ms,
                                               ROBOT_PROFILE_GIMBAL_CONTROL_DEFAULT_PERIOD_MS);
}

static inline uint16_t RobotProfileChassisControlPeriodMs(void)
{
    return RobotProfilePeriodOrDefaultU16(g_config.chassis.control_period_ms,
                                               ROBOT_PROFILE_CHASSIS_CONTROL_DEFAULT_PERIOD_MS);
}

static inline uint16_t RobotProfileWheellegMitControlPeriodMs(void)
{
    return RobotProfilePeriodOrDefaultU16(g_config.WheelLegMit.control_period_ms,
                                               ROBOT_PROFILE_WHEELLEG_MIT_CONTROL_DEFAULT_PERIOD_MS);
}

static inline float RobotProfileChassisControlPeriodS(void)
{
    return (float)RobotProfileChassisControlPeriodMs() * 0.001f;
}

static inline uint16_t RobotProfileCanCommandTxPeriodMs(void)
{
    return RobotProfilePeriodOrDefaultU16(ROBOT_PROFILE_CAN_COMMAND_TX_PERIOD_MS, 1u);
}

static inline uint16_t RobotProfileCanCommandTxLogPeriodMs(void)
{
    return RobotProfilePeriodOrDefaultU16(ROBOT_PROFILE_CAN_COMMAND_TX_LOG_PERIOD_MS,
                                               RobotProfileCanCommandTxPeriodMs());
}

static inline uint32_t RobotProfileCanFeedbackRxMaxFramesPerWake(void)
{
    return (ROBOT_PROFILE_CAN_FEEDBACK_RX_MAX_FRAMES_PER_WAKE == 0u) ?
               1u :
               (uint32_t)ROBOT_PROFILE_CAN_FEEDBACK_RX_MAX_FRAMES_PER_WAKE;
}

static inline uint32_t RobotProfileCanFeedbackRxBudgetUs(void)
{
    return (uint32_t)ROBOT_PROFILE_CAN_FEEDBACK_RX_BUDGET_US;
}

typedef struct
{
    RobotTaskModuleId module;
    const char *name;
} RobotTaskModuleDesc;

static inline const RobotTaskModuleDesc *RobotProfileKnownModules(uint8_t *count)
{
    static const RobotTaskModuleDesc modules[] = {
        {ROBOT_TASK_MODULE_RC_SBUS, "task.rc_sbus"},
        {ROBOT_TASK_MODULE_HEALTH_MONITOR, "task.health_monitor"},
        {ROBOT_TASK_MODULE_SDLOG, "task.sdlog"},
        {ROBOT_TASK_MODULE_CAN_COMMAND_TX, "task.can_command_tx"},
        {ROBOT_TASK_MODULE_CAN_FEEDBACK_RX, "task.can_feedback_rx"},
        {ROBOT_TASK_MODULE_CLASSIC_CHASSIS, "task.classic_chassis"},
        {ROBOT_TASK_MODULE_WHEELLEG_SERVO, "task.WheelLegServo"},
        {ROBOT_TASK_MODULE_WHEELLEG_MIT, "task.WheelLegMit"},
        {ROBOT_TASK_MODULE_SINGLE_GIMBAL, "task.single_gimbal"},
        {ROBOT_TASK_MODULE_DUAL_YAW_GIMBAL, "task.DualYawGimbal"},
        {ROBOT_TASK_MODULE_ARM, "task.arm"},
        {ROBOT_TASK_MODULE_IMU, "task.imu"},
        {ROBOT_TASK_MODULE_HOST_LINK, "task.host_link"},
        {ROBOT_TASK_MODULE_ELRS_LINK, "task.ElrsLink"},
        {ROBOT_TASK_MODULE_REFEREE_RX, "task.RefereeRx"},
        {ROBOT_TASK_MODULE_BATTERY_MONITOR, "task.BatteryMonitor"},
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

static inline uint8_t RobotProfileModuleCount(void)
{
    const uint8_t count = g_config.profile.task_module_count;

    return (count > ROBOT_TASK_MODULE_MAX) ? ROBOT_TASK_MODULE_MAX : count;
}

static inline RobotTaskModuleId RobotProfileModuleIdAt(uint8_t index)
{
    if (index >= RobotProfileModuleCount())
    {
        return (RobotTaskModuleId)ROBOT_TASK_MODULE_NONE;
    }

    return (RobotTaskModuleId)g_config.profile.task_modules[index];
}

static inline RobotTaskModule RobotProfileModuleAt(uint8_t index)
{
    return (RobotTaskModule)RobotProfileModuleIdAt(index);
}

static inline const char *RobotProfileModuleName(RobotTaskModuleId module)
{
    uint8_t count = 0u;
    const RobotTaskModuleDesc *modules = RobotProfileKnownModules(&count);

    for (uint8_t i = 0u; i < count; i++)
    {
        if (modules[i].module == module)
        {
            return modules[i].name;
        }
    }

    return "task.unknown";
}

static inline uint8_t RobotProfileFindModuleByName(const char *name, RobotTaskModule *out)
{
    uint8_t count = 0u;
    const RobotTaskModuleDesc *modules = RobotProfileKnownModules(&count);

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
                *out = (RobotTaskModule)modules[i].module;
            }
            return 1u;
        }
    }

    return 0u;
}

static inline uint8_t RobotProfileModuleIdEnabled(RobotTaskModuleId module)
{
    const uint8_t limit = RobotProfileModuleCount();

    for (uint8_t i = 0u; i < limit; i++)
    {
        if (RobotProfileModuleIdAt(i) == module)
        {
            return 1u;
        }
    }

    return 0u;
}

static inline uint8_t RobotProfileModuleEnabled(RobotTaskModule module)
{
    return RobotProfileModuleIdEnabled((RobotTaskModuleId)module);
}

static inline uint8_t RobotProfileModuleEnabledByName(const char *name)
{
    RobotTaskModule module = ROBOT_TASK_MODULE_NONE;

    if (RobotProfileFindModuleByName(name, &module) == 0u)
    {
        return 0u;
    }

    return RobotProfileModuleEnabled(module);
}

static inline uint8_t RobotProfileNeedClassicChassisControlTask(void)
{
    return RobotProfileModuleEnabled(ROBOT_TASK_MODULE_CLASSIC_CHASSIS);
}

static inline uint8_t RobotProfileNeedWheelLegServoTask(void)
{
    return RobotProfileModuleEnabled(ROBOT_TASK_MODULE_WHEELLEG_SERVO);
}

static inline uint8_t RobotProfileIsWheelLegMit(void)
{
    return RobotProfileModuleEnabled(ROBOT_TASK_MODULE_WHEELLEG_MIT);
}

static inline uint8_t RobotProfileNeedSingleGimbalControlTask(void)
{
    return RobotProfileModuleEnabled(ROBOT_TASK_MODULE_SINGLE_GIMBAL);
}

static inline uint8_t RobotProfileNeedDualGimbalControlTask(void)
{
    return RobotProfileModuleEnabled(ROBOT_TASK_MODULE_DUAL_YAW_GIMBAL);
}

static inline uint8_t RobotProfileNeedArmTask(void)
{
    return RobotProfileModuleEnabled(ROBOT_TASK_MODULE_ARM);
}
