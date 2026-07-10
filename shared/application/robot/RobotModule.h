/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-06-20
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef ROBOT_MODULE_H
#define ROBOT_MODULE_H

#include <stddef.h>
#include <stdint.h>

#include "RobotTaskProfile.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ROBOT_MODULE_FLAG_HAS_TASK (1u << 0)
#define ROBOT_MODULE_FLAG_REQUIRED_BY_PROFILE (1u << 1)
#define ROBOT_MODULE_FLAG_FAST_PATH (1u << 2)
#define ROBOT_MODULE_FLAG_EVENT_DRIVEN (1u << 3)
#define ROBOT_MODULE_FLAG_SAFETY_RELATED (1u << 4)

#define ROBOT_MODULE_ARRAY_COUNT(array_) ((uint8_t)(sizeof(array_) / sizeof((array_)[0])))

typedef enum
{
    RobotResourceNone = 0u,
    RobotResourceManualInput,
    RobotResourceControlInput,
    RobotResourceLowCmd,
    RobotResourceLowState,
    RobotResourceMotorInst,
    RobotResourceMotorFeedback,
    RobotResourceCan1,
    RobotResourceCan2,
    RobotResourceCan3,
    RobotResourceRcUart,
    RobotResourceElrsUart,
    RobotResourceHostUart,
    RobotResourceRefereeUart,
    RobotResourceAuxTelem,
    RobotResourceImu,
    RobotResourceImuSensor,
    RobotResourceAdc,
    RobotResourceSdCard,
    RobotResourceSdLog,
    RobotResourceWatch,
    RobotResourceRtProf,
    RobotResourceRuntimeDiag,
    RobotResourceLifecycle,
    RobotResourceControlMgr,
    RobotResourceChassisState,
    RobotResourceChassisOutput,
    RobotResourceGimbalState,
    RobotResourceGimbalOutput,
    RobotResourceShootOutput,
    RobotResourceWheellegState,
    RobotResourceWheellegOutput,
    RobotResourceArmOutput,
    RobotResourceHostLink,
    RobotResourceElrsLink,
    RobotResourceRefereeLink,
    RobotResourceBattery,
    RobotResourceServoPwm,
    RobotResourceServoOutput,
    RobotResourceCalibration,
    RobotResourceStatusLedGpio,
    RobotResourceStatusLed,
    RobotResourceStartupGate,
    RobotResourceCount
} RobotResourceId;

typedef enum
{
    RobotModuleKindUnknown = 0u,
    RobotModuleKindInput,
    RobotModuleKindComm,
    RobotModuleKindDevice,
    RobotModuleKindService,
    RobotModuleKindControl,
    RobotModuleKindSafety,
} RobotModuleKind;

typedef enum
{
    RobotModulePriorityUnknown = 0u,
    RobotModulePriorityLow,
    RobotModulePriorityNormal,
    RobotModulePriorityAboveNormal,
    RobotModulePriorityHigh,
    RobotModulePriorityRealtime,
} RobotModulePriority;

typedef struct
{
    RobotResourceId id;
    const char *name;
} RobotResourceDesc;

typedef struct
{
    RobotTaskModuleId taskModule;
    const char *name;
    const char *taskName;
    RobotModuleKind kind;
    uint16_t defaultPeriodMs;
    uint16_t defaultBudgetUs;
    uint16_t defaultStackWords;
    uint8_t defaultPriority;
    uint8_t flags;
    const RobotResourceId *requires;
    uint8_t requireCount;
    const RobotResourceId *provides;
    uint8_t provideCount;
} RobotModuleDesc;

static const RobotResourceId sRobotModuleReqRcSbus[] = {
    RobotResourceRcUart,
};
static const RobotResourceId sRobotModuleProRcSbus[] = {
    RobotResourceManualInput,
    RobotResourceControlInput,
};

static const RobotResourceId sRobotModuleReqHealthMonitor[] = {
    RobotResourceWatch,
    RobotResourceRtProf,
};
static const RobotResourceId sRobotModuleProHealthMonitor[] = {
    RobotResourceRuntimeDiag,
};

static const RobotResourceId sRobotModuleReqSdLog[] = {
    RobotResourceSdCard,
    RobotResourceRuntimeDiag,
};
static const RobotResourceId sRobotModuleProSdLog[] = {
    RobotResourceSdLog,
};

static const RobotResourceId sRobotModuleReqCanCommandTx[] = {
    RobotResourceLowCmd,
    RobotResourceMotorInst,
    RobotResourceCan1,
    RobotResourceCan2,
};
static const RobotResourceId sRobotModuleProCanCommandTx[] = {
    RobotResourceMotorInst,
};

static const RobotResourceId sRobotModuleReqCanFeedbackRx[] = {
    RobotResourceCan1,
    RobotResourceCan2,
    RobotResourceMotorInst,
};
static const RobotResourceId sRobotModuleProCanFeedbackRx[] = {
    RobotResourceLowState,
    RobotResourceMotorFeedback,
};

static const RobotResourceId sRobotModuleReqClassicChassis[] = {
    RobotResourceControlInput,
    RobotResourceMotorInst,
    RobotResourceLowCmd,
    RobotResourceImu,
    RobotResourceControlMgr,
};
static const RobotResourceId sRobotModuleProClassicChassis[] = {
    RobotResourceChassisState,
    RobotResourceChassisOutput,
};

static const RobotResourceId sRobotModuleReqWheelleg[] = {
    RobotResourceControlInput,
    RobotResourceMotorInst,
    RobotResourceLowCmd,
    RobotResourceImu,
    RobotResourceControlMgr,
};
static const RobotResourceId sRobotModuleProWheelleg[] = {
    RobotResourceWheellegState,
    RobotResourceWheellegOutput,
};

static const RobotResourceId sRobotModuleReqGimbal[] = {
    RobotResourceControlInput,
    RobotResourceMotorInst,
    RobotResourceLowCmd,
    RobotResourceImu,
    RobotResourceControlMgr,
};
static const RobotResourceId sRobotModuleProGimbal[] = {
    RobotResourceGimbalState,
    RobotResourceGimbalOutput,
    RobotResourceShootOutput,
};

static const RobotResourceId sRobotModuleReqArm[] = {
    RobotResourceControlInput,
    RobotResourceMotorInst,
    RobotResourceLowCmd,
};
static const RobotResourceId sRobotModuleProArm[] = {
    RobotResourceArmOutput,
};

static const RobotResourceId sRobotModuleReqImu[] = {
    RobotResourceImuSensor,
};
static const RobotResourceId sRobotModuleProImu[] = {
    RobotResourceImu,
};

static const RobotResourceId sRobotModuleReqHostLink[] = {
    RobotResourceHostUart,
    RobotResourceAuxTelem,
};
static const RobotResourceId sRobotModuleProHostLink[] = {
    RobotResourceHostLink,
};

static const RobotResourceId sRobotModuleReqElrsLink[] = {
    RobotResourceElrsUart,
};
static const RobotResourceId sRobotModuleProElrsLink[] = {
    RobotResourceElrsLink,
    RobotResourceManualInput,
    RobotResourceControlInput,
};

static const RobotResourceId sRobotModuleReqRefereeRx[] = {
    RobotResourceRefereeUart,
};
static const RobotResourceId sRobotModuleProRefereeRx[] = {
    RobotResourceRefereeLink,
};

static const RobotResourceId sRobotModuleReqBatteryMonitor[] = {
    RobotResourceAdc,
};
static const RobotResourceId sRobotModuleProBatteryMonitor[] = {
    RobotResourceBattery,
};

static const RobotResourceId sRobotModuleReqServo[] = {
    RobotResourceControlInput,
    RobotResourceServoPwm,
};
static const RobotResourceId sRobotModuleProServo[] = {
    RobotResourceServoOutput,
};

static const RobotResourceId sRobotModuleReqCalibration[] = {
    RobotResourceImu,
    RobotResourceMotorInst,
    RobotResourceLowCmd,
};
static const RobotResourceId sRobotModuleProCalibration[] = {
    RobotResourceCalibration,
};

static const RobotResourceId sRobotModuleReqStatusLed[] = {
    RobotResourceStatusLedGpio,
};
static const RobotResourceId sRobotModuleProStatusLed[] = {
    RobotResourceStatusLed,
};

static const RobotResourceId sRobotModuleReqStartupService[] = {
    RobotResourceLifecycle,
};
static const RobotResourceId sRobotModuleProStartupService[] = {
    RobotResourceStartupGate,
};

static inline const RobotResourceDesc *RobotResourceKnown(uint8_t *count)
{
    static const RobotResourceDesc resources[] = {
        {RobotResourceNone, "resource.none"},
        {RobotResourceManualInput, "input.manual"},
        {RobotResourceControlInput, "input.control"},
        {RobotResourceLowCmd, "runtime.lowcmd"},
        {RobotResourceLowState, "runtime.lowstate"},
        {RobotResourceMotorInst, "actuator.motor_inst"},
        {RobotResourceMotorFeedback, "actuator.motor_feedback"},
        {RobotResourceCan1, "bus.can1"},
        {RobotResourceCan2, "bus.can2"},
        {RobotResourceCan3, "bus.can3"},
        {RobotResourceRcUart, "port.rc_uart"},
        {RobotResourceElrsUart, "port.elrs_uart"},
        {RobotResourceHostUart, "port.host_uart"},
        {RobotResourceRefereeUart, "port.referee_uart"},
        {RobotResourceAuxTelem, "link.aux_telem"},
        {RobotResourceImu, "sensor.imu"},
        {RobotResourceImuSensor, "device.imu_sensor"},
        {RobotResourceAdc, "device.adc"},
        {RobotResourceSdCard, "device.sd_card"},
        {RobotResourceSdLog, "service.sdlog"},
        {RobotResourceWatch, "runtime.watch"},
        {RobotResourceRtProf, "runtime.rtprof"},
        {RobotResourceRuntimeDiag, "runtime.diag"},
        {RobotResourceLifecycle, "runtime.lifecycle"},
        {RobotResourceControlMgr, "runtime.control_mgr"},
        {RobotResourceChassisState, "state.chassis"},
        {RobotResourceChassisOutput, "output.chassis"},
        {RobotResourceGimbalState, "state.gimbal"},
        {RobotResourceGimbalOutput, "output.gimbal"},
        {RobotResourceShootOutput, "output.shoot"},
        {RobotResourceWheellegState, "state.wheelleg"},
        {RobotResourceWheellegOutput, "output.wheelleg"},
        {RobotResourceArmOutput, "output.arm"},
        {RobotResourceHostLink, "link.host"},
        {RobotResourceElrsLink, "link.elrs"},
        {RobotResourceRefereeLink, "link.referee"},
        {RobotResourceBattery, "sensor.battery"},
        {RobotResourceServoPwm, "device.servo_pwm"},
        {RobotResourceServoOutput, "output.servo"},
        {RobotResourceCalibration, "service.calibration"},
        {RobotResourceStatusLedGpio, "device.status_led_gpio"},
        {RobotResourceStatusLed, "service.status_led"},
        {RobotResourceStartupGate, "service.startup_gate"},
    };

    if (count != NULL)
    {
        *count = ROBOT_MODULE_ARRAY_COUNT(resources);
    }

    return resources;
}

static inline const char *RobotResourceName(RobotResourceId id)
{
    uint8_t count = 0u;
    const RobotResourceDesc *resources = RobotResourceKnown(&count);

    for (uint8_t i = 0u; i < count; i++)
    {
        if (resources[i].id == id)
        {
            return resources[i].name;
        }
    }

    return "resource.unknown";
}

static inline const char *RobotModuleKindName(RobotModuleKind kind)
{
    switch (kind)
    {
    case RobotModuleKindInput:
        return "input";
    case RobotModuleKindComm:
        return "comm";
    case RobotModuleKindDevice:
        return "device";
    case RobotModuleKindService:
        return "service";
    case RobotModuleKindControl:
        return "control";
    case RobotModuleKindSafety:
        return "safety";
    default:
        return "unknown";
    }
}

static inline const char *RobotModulePriorityName(uint8_t priority)
{
    switch ((RobotModulePriority)priority)
    {
    case RobotModulePriorityLow:
        return "low";
    case RobotModulePriorityNormal:
        return "normal";
    case RobotModulePriorityAboveNormal:
        return "above_normal";
    case RobotModulePriorityHigh:
        return "high";
    case RobotModulePriorityRealtime:
        return "realtime";
    default:
        return "unknown";
    }
}

static inline const RobotModuleDesc *RobotModuleKnownModules(uint8_t *count)
{
    static const RobotModuleDesc modules[] = {
        {ROBOT_TASK_MODULE_RC_SBUS,
         "module.rc_sbus",
         "task.rc_sbus",
         RobotModuleKindInput,
         0u,
         0u,
         256u,
         (uint8_t)RobotModulePriorityAboveNormal,
         (uint8_t)(ROBOT_MODULE_FLAG_HAS_TASK | ROBOT_MODULE_FLAG_REQUIRED_BY_PROFILE),
         sRobotModuleReqRcSbus,
         ROBOT_MODULE_ARRAY_COUNT(sRobotModuleReqRcSbus),
         sRobotModuleProRcSbus,
         ROBOT_MODULE_ARRAY_COUNT(sRobotModuleProRcSbus)},
        {ROBOT_TASK_MODULE_HEALTH_MONITOR,
         "module.health_monitor",
         "task.health_monitor",
         RobotModuleKindSafety,
         ROBOT_PROFILE_WATCH_TASK_BEAT_MIN_PERIOD_MS,
         ROBOT_PROFILE_WATCH_TASK_BEAT_BUDGET_US,
         256u,
         (uint8_t)RobotModulePriorityNormal,
         (uint8_t)(ROBOT_MODULE_FLAG_HAS_TASK |
                   ROBOT_MODULE_FLAG_REQUIRED_BY_PROFILE |
                   ROBOT_MODULE_FLAG_SAFETY_RELATED),
         sRobotModuleReqHealthMonitor,
         ROBOT_MODULE_ARRAY_COUNT(sRobotModuleReqHealthMonitor),
         sRobotModuleProHealthMonitor,
         ROBOT_MODULE_ARRAY_COUNT(sRobotModuleProHealthMonitor)},
        {ROBOT_TASK_MODULE_SDLOG,
         "module.sdlog",
         "task.sdlog",
         RobotModuleKindService,
         0u,
         ROBOT_PROFILE_SDLOG_BLOCK_WRITE_BUDGET_US,
         512u,
         (uint8_t)RobotModulePriorityLow,
         (uint8_t)ROBOT_MODULE_FLAG_HAS_TASK,
         sRobotModuleReqSdLog,
         ROBOT_MODULE_ARRAY_COUNT(sRobotModuleReqSdLog),
         sRobotModuleProSdLog,
         ROBOT_MODULE_ARRAY_COUNT(sRobotModuleProSdLog)},
        {ROBOT_TASK_MODULE_CAN_COMMAND_TX,
         "module.can_command_tx",
         "task.can_command_tx",
         RobotModuleKindComm,
         ROBOT_PROFILE_CAN_COMMAND_TX_PERIOD_MS,
         ROBOT_PROFILE_CAN_COMMAND_TX_BUDGET_US,
         256u,
         (uint8_t)RobotModulePriorityAboveNormal,
         (uint8_t)(ROBOT_MODULE_FLAG_HAS_TASK | ROBOT_MODULE_FLAG_FAST_PATH),
         sRobotModuleReqCanCommandTx,
         ROBOT_MODULE_ARRAY_COUNT(sRobotModuleReqCanCommandTx),
         sRobotModuleProCanCommandTx,
         ROBOT_MODULE_ARRAY_COUNT(sRobotModuleProCanCommandTx)},
        {ROBOT_TASK_MODULE_CAN_FEEDBACK_RX,
         "module.can_feedback_rx",
         "task.can_feedback_rx",
         RobotModuleKindComm,
         0u,
         ROBOT_PROFILE_CAN_FEEDBACK_RX_PROFILE_BUDGET_US,
         256u,
         (uint8_t)RobotModulePriorityHigh,
         (uint8_t)(ROBOT_MODULE_FLAG_HAS_TASK | ROBOT_MODULE_FLAG_FAST_PATH | ROBOT_MODULE_FLAG_EVENT_DRIVEN),
         sRobotModuleReqCanFeedbackRx,
         ROBOT_MODULE_ARRAY_COUNT(sRobotModuleReqCanFeedbackRx),
         sRobotModuleProCanFeedbackRx,
         ROBOT_MODULE_ARRAY_COUNT(sRobotModuleProCanFeedbackRx)},
        {ROBOT_TASK_MODULE_CLASSIC_CHASSIS,
         "module.classic_chassis",
         "task.classic_chassis",
         RobotModuleKindControl,
         ROBOT_PROFILE_CHASSIS_CONTROL_DEFAULT_PERIOD_MS,
         ROBOT_PROFILE_CHASSIS_CONTROL_BUDGET_US,
         768u,
         (uint8_t)RobotModulePriorityAboveNormal,
         (uint8_t)(ROBOT_MODULE_FLAG_HAS_TASK | ROBOT_MODULE_FLAG_FAST_PATH | ROBOT_MODULE_FLAG_SAFETY_RELATED),
         sRobotModuleReqClassicChassis,
         ROBOT_MODULE_ARRAY_COUNT(sRobotModuleReqClassicChassis),
         sRobotModuleProClassicChassis,
         ROBOT_MODULE_ARRAY_COUNT(sRobotModuleProClassicChassis)},
        {ROBOT_TASK_MODULE_WHEELLEG_SERVO,
         "module.wheelleg_servo",
         "task.WheelLegServo",
         RobotModuleKindControl,
         ROBOT_PROFILE_WHEELLEG_MIT_CONTROL_DEFAULT_PERIOD_MS,
         ROBOT_PROFILE_WHEELLEG_MIT_CONTROL_BUDGET_US,
         768u,
         (uint8_t)RobotModulePriorityAboveNormal,
         (uint8_t)(ROBOT_MODULE_FLAG_HAS_TASK | ROBOT_MODULE_FLAG_FAST_PATH | ROBOT_MODULE_FLAG_SAFETY_RELATED),
         sRobotModuleReqWheelleg,
         ROBOT_MODULE_ARRAY_COUNT(sRobotModuleReqWheelleg),
         sRobotModuleProWheelleg,
         ROBOT_MODULE_ARRAY_COUNT(sRobotModuleProWheelleg)},
        {ROBOT_TASK_MODULE_WHEELLEG_MIT,
         "module.wheelleg_mit",
         "task.WheelLegMit",
         RobotModuleKindControl,
         ROBOT_PROFILE_WHEELLEG_MIT_CONTROL_DEFAULT_PERIOD_MS,
         ROBOT_PROFILE_WHEELLEG_MIT_CONTROL_BUDGET_US,
         768u,
         (uint8_t)RobotModulePriorityAboveNormal,
         (uint8_t)(ROBOT_MODULE_FLAG_HAS_TASK | ROBOT_MODULE_FLAG_FAST_PATH | ROBOT_MODULE_FLAG_SAFETY_RELATED),
         sRobotModuleReqWheelleg,
         ROBOT_MODULE_ARRAY_COUNT(sRobotModuleReqWheelleg),
         sRobotModuleProWheelleg,
         ROBOT_MODULE_ARRAY_COUNT(sRobotModuleProWheelleg)},
        {ROBOT_TASK_MODULE_SINGLE_GIMBAL,
         "module.single_gimbal",
         "task.single_gimbal",
         RobotModuleKindControl,
         ROBOT_PROFILE_GIMBAL_CONTROL_DEFAULT_PERIOD_MS,
         ROBOT_PROFILE_GIMBAL_CONTROL_BUDGET_US,
         1024u,
         (uint8_t)RobotModulePriorityHigh,
         (uint8_t)(ROBOT_MODULE_FLAG_HAS_TASK | ROBOT_MODULE_FLAG_FAST_PATH | ROBOT_MODULE_FLAG_SAFETY_RELATED),
         sRobotModuleReqGimbal,
         ROBOT_MODULE_ARRAY_COUNT(sRobotModuleReqGimbal),
         sRobotModuleProGimbal,
         ROBOT_MODULE_ARRAY_COUNT(sRobotModuleProGimbal)},
        {ROBOT_TASK_MODULE_DUAL_YAW_GIMBAL,
         "module.dual_yaw_gimbal",
         "task.DualYawGimbal",
         RobotModuleKindControl,
         ROBOT_PROFILE_GIMBAL_CONTROL_DEFAULT_PERIOD_MS,
         ROBOT_PROFILE_GIMBAL_CONTROL_BUDGET_US,
         1024u,
         (uint8_t)RobotModulePriorityHigh,
         (uint8_t)(ROBOT_MODULE_FLAG_HAS_TASK | ROBOT_MODULE_FLAG_FAST_PATH | ROBOT_MODULE_FLAG_SAFETY_RELATED),
         sRobotModuleReqGimbal,
         ROBOT_MODULE_ARRAY_COUNT(sRobotModuleReqGimbal),
         sRobotModuleProGimbal,
         ROBOT_MODULE_ARRAY_COUNT(sRobotModuleProGimbal)},
        {ROBOT_TASK_MODULE_ARM,
         "module.arm",
         "task.arm",
         RobotModuleKindControl,
         0u,
         0u,
         768u,
         (uint8_t)RobotModulePriorityNormal,
         (uint8_t)(ROBOT_MODULE_FLAG_HAS_TASK | ROBOT_MODULE_FLAG_SAFETY_RELATED),
         sRobotModuleReqArm,
         ROBOT_MODULE_ARRAY_COUNT(sRobotModuleReqArm),
         sRobotModuleProArm,
         ROBOT_MODULE_ARRAY_COUNT(sRobotModuleProArm)},
        {ROBOT_TASK_MODULE_IMU,
         "module.imu",
         "task.imu",
         RobotModuleKindDevice,
         1u,
         0u,
         1024u,
         (uint8_t)RobotModulePriorityRealtime,
         (uint8_t)(ROBOT_MODULE_FLAG_HAS_TASK | ROBOT_MODULE_FLAG_FAST_PATH),
         sRobotModuleReqImu,
         ROBOT_MODULE_ARRAY_COUNT(sRobotModuleReqImu),
         sRobotModuleProImu,
         ROBOT_MODULE_ARRAY_COUNT(sRobotModuleProImu)},
        {ROBOT_TASK_MODULE_HOST_LINK,
         "module.host_link",
         "task.host_link",
         RobotModuleKindComm,
         0u,
         0u,
         512u,
         (uint8_t)RobotModulePriorityNormal,
         (uint8_t)ROBOT_MODULE_FLAG_HAS_TASK,
         sRobotModuleReqHostLink,
         ROBOT_MODULE_ARRAY_COUNT(sRobotModuleReqHostLink),
         sRobotModuleProHostLink,
         ROBOT_MODULE_ARRAY_COUNT(sRobotModuleProHostLink)},
        {ROBOT_TASK_MODULE_ELRS_LINK,
         "module.elrs_link",
         "task.ElrsLink",
         RobotModuleKindInput,
         0u,
         0u,
         256u,
         (uint8_t)RobotModulePriorityAboveNormal,
         (uint8_t)ROBOT_MODULE_FLAG_HAS_TASK,
         sRobotModuleReqElrsLink,
         ROBOT_MODULE_ARRAY_COUNT(sRobotModuleReqElrsLink),
         sRobotModuleProElrsLink,
         ROBOT_MODULE_ARRAY_COUNT(sRobotModuleProElrsLink)},
        {ROBOT_TASK_MODULE_REFEREE_RX,
         "module.referee_rx",
         "task.RefereeRx",
         RobotModuleKindComm,
         0u,
         0u,
         128u,
         (uint8_t)RobotModulePriorityNormal,
         (uint8_t)ROBOT_MODULE_FLAG_HAS_TASK,
         sRobotModuleReqRefereeRx,
         ROBOT_MODULE_ARRAY_COUNT(sRobotModuleReqRefereeRx),
         sRobotModuleProRefereeRx,
         ROBOT_MODULE_ARRAY_COUNT(sRobotModuleProRefereeRx)},
        {ROBOT_TASK_MODULE_BATTERY_MONITOR,
         "module.battery_monitor",
         "task.BatteryMonitor",
         RobotModuleKindDevice,
         0u,
         0u,
         128u,
         (uint8_t)RobotModulePriorityNormal,
         (uint8_t)ROBOT_MODULE_FLAG_HAS_TASK,
         sRobotModuleReqBatteryMonitor,
         ROBOT_MODULE_ARRAY_COUNT(sRobotModuleReqBatteryMonitor),
         sRobotModuleProBatteryMonitor,
         ROBOT_MODULE_ARRAY_COUNT(sRobotModuleProBatteryMonitor)},
        {ROBOT_TASK_MODULE_SERVO,
         "module.servo",
         "task.servo",
         RobotModuleKindControl,
         0u,
         0u,
         128u,
         (uint8_t)RobotModulePriorityNormal,
         (uint8_t)ROBOT_MODULE_FLAG_HAS_TASK,
         sRobotModuleReqServo,
         ROBOT_MODULE_ARRAY_COUNT(sRobotModuleReqServo),
         sRobotModuleProServo,
         ROBOT_MODULE_ARRAY_COUNT(sRobotModuleProServo)},
        {ROBOT_TASK_MODULE_CALIBRATION,
         "module.calibration",
         "task.calibration",
         RobotModuleKindService,
         0u,
         0u,
         512u,
         (uint8_t)RobotModulePriorityNormal,
         (uint8_t)(ROBOT_MODULE_FLAG_HAS_TASK | ROBOT_MODULE_FLAG_SAFETY_RELATED),
         sRobotModuleReqCalibration,
         ROBOT_MODULE_ARRAY_COUNT(sRobotModuleReqCalibration),
         sRobotModuleProCalibration,
         ROBOT_MODULE_ARRAY_COUNT(sRobotModuleProCalibration)},
        {ROBOT_TASK_MODULE_STATUS_LED,
         "module.status_led",
         "task.status_led",
         RobotModuleKindService,
         0u,
         0u,
         256u,
         (uint8_t)RobotModulePriorityNormal,
         (uint8_t)ROBOT_MODULE_FLAG_HAS_TASK,
         sRobotModuleReqStatusLed,
         ROBOT_MODULE_ARRAY_COUNT(sRobotModuleReqStatusLed),
         sRobotModuleProStatusLed,
         ROBOT_MODULE_ARRAY_COUNT(sRobotModuleProStatusLed)},
        {ROBOT_TASK_MODULE_STARTUP_SERVICE,
         "module.startup_service",
         "task.startup_service",
         RobotModuleKindService,
         0u,
         0u,
         512u,
         (uint8_t)RobotModulePriorityNormal,
         (uint8_t)ROBOT_MODULE_FLAG_HAS_TASK,
         sRobotModuleReqStartupService,
         ROBOT_MODULE_ARRAY_COUNT(sRobotModuleReqStartupService),
         sRobotModuleProStartupService,
         ROBOT_MODULE_ARRAY_COUNT(sRobotModuleProStartupService)},
    };

    if (count != NULL)
    {
        *count = ROBOT_MODULE_ARRAY_COUNT(modules);
    }

    return modules;
}

static inline const RobotModuleDesc *RobotModuleFindByTaskModule(RobotTaskModuleId module)
{
    uint8_t count = 0u;
    const RobotModuleDesc *modules = RobotModuleKnownModules(&count);

    for (uint8_t i = 0u; i < count; i++)
    {
        if (modules[i].taskModule == module)
        {
            return &modules[i];
        }
    }

    return NULL;
}

static inline const char *RobotModuleName(RobotTaskModuleId module)
{
    const RobotModuleDesc *desc = RobotModuleFindByTaskModule(module);

    return (desc != NULL) ? desc->name : RobotProfileModuleName(module);
}

static inline uint16_t RobotModulePeriodMs(const RobotModuleDesc *desc)
{
    if (desc == NULL)
    {
        return 0u;
    }

    switch ((RobotTaskModule)desc->taskModule)
    {
    case ROBOT_TASK_MODULE_CAN_COMMAND_TX:
        return RobotProfileCanCommandTxPeriodMs();
    case ROBOT_TASK_MODULE_CLASSIC_CHASSIS:
        return RobotProfileChassisControlPeriodMs();
    case ROBOT_TASK_MODULE_WHEELLEG_SERVO:
    case ROBOT_TASK_MODULE_WHEELLEG_MIT:
        return RobotProfileWheellegMitControlPeriodMs();
    case ROBOT_TASK_MODULE_SINGLE_GIMBAL:
    case ROBOT_TASK_MODULE_DUAL_YAW_GIMBAL:
        return RobotProfileGimbalControlPeriodMs();
    default:
        return desc->defaultPeriodMs;
    }
}

static inline uint16_t RobotModuleBudgetUs(const RobotModuleDesc *desc)
{
    return (desc != NULL) ? desc->defaultBudgetUs : 0u;
}

#ifdef __cplusplus
}
#endif

#endif
