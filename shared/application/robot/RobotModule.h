/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
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

/* 目标生成头可按车型覆盖任务栈和优先级；宿主环境保留目录中的通用值。 */
#ifndef ROBOT_TASK_SELECTED_STACK
#define ROBOT_TASK_SELECTED_STACK(symbol, stack_m, stack_other) (stack_other)
#endif

#ifndef ROBOT_TASK_SELECTED_PRIORITY
#define ROBOT_TASK_SELECTED_PRIORITY(symbol, fallback) (fallback)
#endif

#ifndef ROBOT_CONTROL_CHASSIS_PERIOD_MS
#define ROBOT_CONTROL_CHASSIS_PERIOD_MS RobotProfileChassisControlPeriodMs()
#endif

#ifndef ROBOT_CONTROL_GIMBAL_PERIOD_MS
#define ROBOT_CONTROL_GIMBAL_PERIOD_MS RobotProfileGimbalControlPeriodMs()
#endif

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

/* 通用算法只声明真实提供的输出，不能冒充旧控制链的状态和发射服务。 */
static const RobotResourceId sRobotModuleReqControlChassis[] = {
    RobotResourceControlInput, RobotResourceMotorInst, RobotResourceLowCmd, RobotResourceControlMgr,
#if ROBOT_CONTROL_CHASSIS_REQUIRE_IMU
    RobotResourceImu,
#endif
};
static const RobotResourceId sRobotModuleProControlChassis[] = {
    RobotResourceChassisOutput,
};
static const RobotResourceId sRobotModuleReqControlGimbal[] = {
    RobotResourceControlInput, RobotResourceMotorInst, RobotResourceLowCmd, RobotResourceControlMgr,
#if ROBOT_CONTROL_GIMBAL_REQUIRE_IMU
    RobotResourceImu,
#endif
};
static const RobotResourceId sRobotModuleProControlGimbal[] = {
    RobotResourceGimbalOutput,
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
#define ROBOT_TASK(symbol, id, moduleName, taskName, kind, period, budget, stackM, stackOther, priority, flags, resourceSuffix, header, entry, order, dependencies, source) \
        {ROBOT_TASK_MODULE_##symbol,                                                                                                            \
         moduleName,                                                                                                                            \
         taskName,                                                                                                                              \
         kind,                                                                                                                                  \
         period,                                                                                                                                \
         budget,                                                                                                                                \
         ROBOT_TASK_SELECTED_STACK(symbol, stackM, stackOther),                                                                               \
         (uint8_t)ROBOT_TASK_SELECTED_PRIORITY(symbol, priority),                                                                             \
         (uint8_t)(flags),                                                                                                                      \
         sRobotModuleReq##resourceSuffix,                                                                                                      \
         ROBOT_MODULE_ARRAY_COUNT(sRobotModuleReq##resourceSuffix),                                                                            \
         sRobotModulePro##resourceSuffix,                                                                                                      \
         ROBOT_MODULE_ARRAY_COUNT(sRobotModulePro##resourceSuffix)},
#include "RobotTaskCatalog.def"
#undef ROBOT_TASK
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
    case ROBOT_TASK_MODULE_CONTROL_CHASSIS:
        return ROBOT_CONTROL_CHASSIS_PERIOD_MS;
    case ROBOT_TASK_MODULE_WHEELLEG_SERVO:
    case ROBOT_TASK_MODULE_WHEELLEG_MIT:
        return RobotProfileWheellegMitControlPeriodMs();
    case ROBOT_TASK_MODULE_SINGLE_GIMBAL:
    case ROBOT_TASK_MODULE_DUAL_YAW_GIMBAL:
        return RobotProfileGimbalControlPeriodMs();
    case ROBOT_TASK_MODULE_CONTROL_GIMBAL:
        return ROBOT_CONTROL_GIMBAL_PERIOD_MS;
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
