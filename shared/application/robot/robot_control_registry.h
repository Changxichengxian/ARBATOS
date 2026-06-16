/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ROBOT_CONTROL_REGISTRY_H
#define ROBOT_CONTROL_REGISTRY_H

#include "CAN_receive.h"
#include "ControlMgr.h"
#include "MotorInst.h"
#include "robot_task_build_config.h"
#include "robot_task_profile.h"

static inline void robot_control_register_if_enabled(const ControlController *controller,
                                                     robot_task_module_e module)
{
    if (controller == NULL || robot_profile_module_enabled(module) == 0u)
    {
        return;
    }

    (void)ControlMgrRegister(controller);
}

static inline void robot_control_register_profile_defaults(void)
{
    /*
     * Registration only declares resources for diagnostics/arbitration.
     * Boot activation is handled by robot_control_start_profile_defaults().
     */
#if ROBOT_TASK_BUILD_CLASSIC_CHASSIS
    static const char *const chassis_outputs[] = {
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
    static const char *const dual_yaw_gimbal_outputs[] = {
        "motor.yaw",
        "motor.yaw_upper",
        "motor.pitch",
    };
#endif
#if ROBOT_TASK_BUILD_SHOOT_RM
    static const char *const shoot_outputs[] = {
        "motor.trigger",
        "motor.friction0",
        "motor.friction1",
        "motor.friction2",
        "motor.friction3",
    };
#endif
#if ROBOT_TASK_BUILD_ARM
    static const char *const arm_outputs[] = {
        "motor.arm0",
        "motor.arm1",
        "motor.arm2",
        "motor.arm3",
        "motor.arm4",
        "motor.arm5",
    };
#endif
#if ROBOT_TASK_BUILD_WHEELLEG_MIT
    static const char *const wheelleg_mit_outputs[] = {
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
            .outputs = chassis_outputs,
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
    static const ControlController dual_yaw_gimbal = {
        .id = ControlIdDualYawGimbal,
        .domain = ControlDomainGimbal,
        .claim_mask = ControlResGimbalYaw | ControlResGimbalPitch,
        .name = "controller.dual_yaw_gimbal",
        .meta = {
            .period_ms = ROBOT_PROFILE_GIMBAL_CONTROL_DEFAULT_PERIOD_MS,
            .output_count = 3u,
            .outputs = dual_yaw_gimbal_outputs,
        },
    };
#endif
#if ROBOT_TASK_BUILD_SHOOT_RM
    static const ControlController shoot = {
        .id = ControlIdShoot,
        .domain = ControlDomainShoot,
        .claim_mask = ControlResShootTrigger | ControlResShootFriction,
        .name = "controller.shoot_rm",
        .meta = {
            .period_ms = ROBOT_PROFILE_GIMBAL_CONTROL_DEFAULT_PERIOD_MS,
            .output_count = 5u,
            .outputs = shoot_outputs,
        },
    };
#endif
#if ROBOT_TASK_BUILD_ARM
    static const ControlController arm = {
        .id = ControlIdArmMotion,
        .domain = ControlDomainArm,
        .claim_mask = ControlResArm,
        .name = "controller.arm_motion",
        .meta = {
            .period_ms = 5u,
            .output_count = 6u,
            .outputs = arm_outputs,
        },
    };
#endif
#if ROBOT_TASK_BUILD_WHEELLEG_MIT
    static const ControlController wheelleg_mit = {
        .id = ControlIdWheellegMitBalance,
        .domain = ControlDomainWheelleg,
        .claim_mask = ControlResWheellegLeftLeg |
                      ControlResWheellegRightLeg |
                      ControlResWheellegLeftWheel |
                      ControlResWheellegRightWheel,
        .name = "controller.wheelleg_mit",
        .meta = {
            .period_ms = 1u,
            .output_count = 6u,
            .outputs = wheelleg_mit_outputs,
        },
    };
#endif

#if ROBOT_TASK_BUILD_CLASSIC_CHASSIS
    robot_control_register_if_enabled(&classic_chassis, ROBOT_TASK_MODULE_CLASSIC_CHASSIS);
#endif
#if ROBOT_TASK_BUILD_SINGLE_GIMBAL
    robot_control_register_if_enabled(&single_gimbal, ROBOT_TASK_MODULE_SINGLE_GIMBAL);
#endif
#if ROBOT_TASK_BUILD_DUAL_YAW_GIMBAL
    robot_control_register_if_enabled(&dual_yaw_gimbal, ROBOT_TASK_MODULE_DUAL_YAW_GIMBAL);
#endif
#if ROBOT_TASK_BUILD_SHOOT_RM
    if (robot_profile_module_enabled(ROBOT_TASK_MODULE_SINGLE_GIMBAL) != 0u ||
        robot_profile_module_enabled(ROBOT_TASK_MODULE_DUAL_YAW_GIMBAL) != 0u)
    {
        (void)ControlMgrRegister(&shoot);
    }
#endif
#if ROBOT_TASK_BUILD_ARM
    robot_control_register_if_enabled(&arm, ROBOT_TASK_MODULE_ARM);
#endif
#if ROBOT_TASK_BUILD_WHEELLEG_MIT
    robot_control_register_if_enabled(&wheelleg_mit, ROBOT_TASK_MODULE_WHEELLEG_MIT);
#endif
}

static inline void robot_control_start_profile_defaults(void)
{
#if ROBOT_TASK_BUILD_CLASSIC_CHASSIS
    if (robot_profile_module_enabled(ROBOT_TASK_MODULE_CLASSIC_CHASSIS) != 0u)
    {
        (void)ControlMgrSwitch(ControlIdClassicChassis, ControlReasonProfile);
    }
#endif
#if ROBOT_TASK_BUILD_SINGLE_GIMBAL
    if (robot_profile_module_enabled(ROBOT_TASK_MODULE_SINGLE_GIMBAL) != 0u)
    {
        (void)ControlMgrSwitch(ControlIdSingleGimbal, ControlReasonProfile);
    }
#endif
#if ROBOT_TASK_BUILD_DUAL_YAW_GIMBAL
    if (robot_profile_module_enabled(ROBOT_TASK_MODULE_DUAL_YAW_GIMBAL) != 0u)
    {
        (void)ControlMgrSwitch(ControlIdDualYawGimbal, ControlReasonProfile);
    }
#endif
#if ROBOT_TASK_BUILD_SHOOT_RM
    if (robot_profile_module_enabled(ROBOT_TASK_MODULE_SINGLE_GIMBAL) != 0u ||
        robot_profile_module_enabled(ROBOT_TASK_MODULE_DUAL_YAW_GIMBAL) != 0u)
    {
        (void)ControlMgrSwitch(ControlIdShoot, ControlReasonProfile);
    }
#endif
#if ROBOT_TASK_BUILD_ARM
    if (robot_profile_module_enabled(ROBOT_TASK_MODULE_ARM) != 0u)
    {
        (void)ControlMgrSwitch(ControlIdArmMotion, ControlReasonProfile);
    }
#endif
#if ROBOT_TASK_BUILD_WHEELLEG_MIT
    if (robot_profile_module_enabled(ROBOT_TASK_MODULE_WHEELLEG_MIT) != 0u)
    {
        (void)ControlMgrSwitch(ControlIdWheellegMitBalance, ControlReasonProfile);
    }
#endif
}

static inline void robot_control_bootstrap_profile_defaults(void)
{
    ControlCtx context = {0};

    MotorInstRefresh();
    CAN_rx_prepare_motor_measure_points();
    ControlMgrInit();
    robot_control_register_profile_defaults();
    robot_control_start_profile_defaults();
    (void)ControlMgrUpdateAll(&context);
}

#endif
