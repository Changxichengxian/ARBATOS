/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ROBOT_CONTROL_REGISTRY_H
#define ROBOT_CONTROL_REGISTRY_H

#include "CanReceive.h"
#include "ControlMgr.h"
#include "MotorInst.h"
#include "RobotTaskBuildConfig.h"
#include "RobotTaskProfile.h"
#if ROBOT_TASK_BUILD_CAN_COMMAND_TX
#include "CanTxTask.h"
#endif

static inline void RobotControlRegisterIfEnabled(const ControlController *controller,
                                                     RobotTaskModule module)
{
    if (controller == NULL || RobotProfileModuleEnabled(module) == 0u)
    {
        return;
    }

    (void)ControlMgrRegister(controller);
}

static inline void RobotControlRegisterProfileDefaults(void)
{
    /*
     * Registration only declares resources for diagnostics/arbitration.
     * Boot activation is handled by RobotControlStartProfileDefaults().
     */
#if ROBOT_TASK_BUILD_CLASSIC_CHASSIS
    static const char *const ChassisOutputs[] = {
        "motor.chassis0",
        "motor.chassis1",
        "motor.chassis2",
        "motor.chassis3",
    };
#endif
#if ROBOT_TASK_BUILD_SINGLE_GIMBAL
    static const char *const single_gimbal_outputs[] = {
        "motor.yaw",
        "motor.pitch",
    };
#endif
#if ROBOT_TASK_BUILD_DUAL_YAW_GIMBAL
    static const char *const DualYawGimbalOutputs[] = {
        "motor.yaw",
        "motor.yaw_upper",
        "motor.pitch",
    };
#endif
#if ROBOT_TASK_BUILD_SHOOT_RM
    static const char *const ShootOutputs[] = {
        "motor.trigger",
        "motor.friction0",
        "motor.friction1",
        "motor.friction2",
        "motor.friction3",
    };
#endif
#if ROBOT_TASK_BUILD_ARM
    static const char *const ArmOutputs[] = {
        "motor.arm0",
        "motor.arm1",
        "motor.arm2",
        "motor.arm3",
        "motor.arm4",
        "motor.arm5",
    };
#endif
#if ROBOT_TASK_BUILD_WHEELLEG_MIT
    static const char *const WheelLegMitOutputs[] = {
        "motor.left_front",
        "motor.left_back",
        "motor.right_front",
        "motor.right_back",
        "motor.left_wheel",
        "motor.right_wheel",
    };
#endif

#if ROBOT_TASK_BUILD_CLASSIC_CHASSIS
    static const ControlController classic_chassis = {
        .id = ControlIdClassicChassis,
        .domain = ControlDomainChassis,
        .claim_mask = ControlResChassisWheels,
        .name = "controller.classic_chassis",
        .meta = {
            .period_ms = ROBOT_PROFILE_CHASSIS_CONTROL_DEFAULT_PERIOD_MS,
            .output_count = 4u,
            .outputs = ChassisOutputs,
        },
    };
#endif
#if ROBOT_TASK_BUILD_SINGLE_GIMBAL
    static const ControlController single_gimbal = {
        .id = ControlIdSingleGimbal,
        .domain = ControlDomainGimbal,
        .claim_mask = ControlResGimbalYaw | ControlResGimbalPitch,
        .name = "controller.single_gimbal",
        .meta = {
            .period_ms = ROBOT_PROFILE_GIMBAL_CONTROL_DEFAULT_PERIOD_MS,
            .output_count = 2u,
            .outputs = single_gimbal_outputs,
        },
    };
#endif
#if ROBOT_TASK_BUILD_DUAL_YAW_GIMBAL
    static const ControlController DualYawGimbal = {
        .id = ControlIdDualYawGimbal,
        .domain = ControlDomainGimbal,
        .claim_mask = ControlResGimbalYaw | ControlResGimbalPitch,
        .name = "controller.DualYawGimbal",
        .meta = {
            .period_ms = ROBOT_PROFILE_GIMBAL_CONTROL_DEFAULT_PERIOD_MS,
            .output_count = 3u,
            .outputs = DualYawGimbalOutputs,
        },
    };
#endif
#if ROBOT_TASK_BUILD_SHOOT_RM
    static const ControlController shoot = {
        .id = ControlIdShoot,
        .domain = ControlDomainShoot,
        .claim_mask = ControlResShootTrigger | ControlResShootFriction,
        .name = "controller.ShootRm",
        .meta = {
            .period_ms = ROBOT_PROFILE_GIMBAL_CONTROL_DEFAULT_PERIOD_MS,
            .output_count = 5u,
            .outputs = ShootOutputs,
        },
    };
#endif
#if ROBOT_TASK_BUILD_ARM
    static const ControlController arm = {
        .id = ControlIdArmMotion,
        .domain = ControlDomainArm,
        .claim_mask = ControlResArm,
        .name = "controller.ArmMotion",
        .meta = {
            .period_ms = 5u,
            .output_count = 6u,
            .outputs = ArmOutputs,
        },
    };
#endif
#if ROBOT_TASK_BUILD_WHEELLEG_MIT
    static const ControlController WheelLegMit = {
        .id = ControlIdWheellegMitBalance,
        .domain = ControlDomainWheelleg,
        .claim_mask = ControlResWheellegLeftLeg |
                      ControlResWheellegRightLeg |
                      ControlResWheellegLeftWheel |
                      ControlResWheellegRightWheel,
        .name = "controller.WheelLegMit",
        .meta = {
            .period_ms = 1u,
            .output_count = 6u,
            .outputs = WheelLegMitOutputs,
        },
    };
#endif

#if ROBOT_TASK_BUILD_CLASSIC_CHASSIS
    RobotControlRegisterIfEnabled(&classic_chassis, ROBOT_TASK_MODULE_CLASSIC_CHASSIS);
#endif
#if ROBOT_TASK_BUILD_SINGLE_GIMBAL
    RobotControlRegisterIfEnabled(&single_gimbal, ROBOT_TASK_MODULE_SINGLE_GIMBAL);
#endif
#if ROBOT_TASK_BUILD_DUAL_YAW_GIMBAL
    RobotControlRegisterIfEnabled(&DualYawGimbal, ROBOT_TASK_MODULE_DUAL_YAW_GIMBAL);
#endif
#if ROBOT_TASK_BUILD_SHOOT_RM
    if (RobotProfileModuleEnabled(ROBOT_TASK_MODULE_SINGLE_GIMBAL) != 0u ||
        RobotProfileModuleEnabled(ROBOT_TASK_MODULE_DUAL_YAW_GIMBAL) != 0u)
    {
        (void)ControlMgrRegister(&shoot);
    }
#endif
#if ROBOT_TASK_BUILD_ARM
    RobotControlRegisterIfEnabled(&arm, ROBOT_TASK_MODULE_ARM);
#endif
#if ROBOT_TASK_BUILD_WHEELLEG_MIT
    RobotControlRegisterIfEnabled(&WheelLegMit, ROBOT_TASK_MODULE_WHEELLEG_MIT);
#endif
}

static inline void RobotControlStartProfileDefaults(void)
{
#if ROBOT_TASK_BUILD_CLASSIC_CHASSIS
    if (RobotProfileModuleEnabled(ROBOT_TASK_MODULE_CLASSIC_CHASSIS) != 0u)
    {
        (void)ControlMgrSwitch(ControlIdClassicChassis, ControlReasonProfile);
    }
#endif
#if ROBOT_TASK_BUILD_SINGLE_GIMBAL
    if (RobotProfileModuleEnabled(ROBOT_TASK_MODULE_SINGLE_GIMBAL) != 0u)
    {
        (void)ControlMgrSwitch(ControlIdSingleGimbal, ControlReasonProfile);
    }
#endif
#if ROBOT_TASK_BUILD_DUAL_YAW_GIMBAL
    if (RobotProfileModuleEnabled(ROBOT_TASK_MODULE_DUAL_YAW_GIMBAL) != 0u)
    {
        (void)ControlMgrSwitch(ControlIdDualYawGimbal, ControlReasonProfile);
    }
#endif
#if ROBOT_TASK_BUILD_SHOOT_RM
    if (RobotProfileModuleEnabled(ROBOT_TASK_MODULE_SINGLE_GIMBAL) != 0u ||
        RobotProfileModuleEnabled(ROBOT_TASK_MODULE_DUAL_YAW_GIMBAL) != 0u)
    {
        (void)ControlMgrSwitch(ControlIdShoot, ControlReasonProfile);
    }
#endif
#if ROBOT_TASK_BUILD_ARM
    if (RobotProfileModuleEnabled(ROBOT_TASK_MODULE_ARM) != 0u)
    {
        (void)ControlMgrSwitch(ControlIdArmMotion, ControlReasonProfile);
    }
#endif
#if ROBOT_TASK_BUILD_WHEELLEG_MIT
    if (RobotProfileModuleEnabled(ROBOT_TASK_MODULE_WHEELLEG_MIT) != 0u)
    {
        (void)ControlMgrSwitch(ControlIdWheellegMitBalance, ControlReasonProfile);
    }
#endif
}

static inline void RobotControlBootstrapProfileDefaults(void)
{
    ControlCtx context = {0};

    MotorInstRefresh();
    CAN_rx_prepare_motor_measure_points();
    ControlMgrInit();
    RobotControlRegisterProfileDefaults();
    RobotControlStartProfileDefaults();
    (void)ControlMgrUpdateAll(&context);
#if ROBOT_TASK_BUILD_CAN_COMMAND_TX
    CanTxEmergencyPrepare();
#endif
}

#endif
