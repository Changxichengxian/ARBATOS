/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ROBOT_CONTROL_REGISTRY_H
#define ROBOT_CONTROL_REGISTRY_H

#include "control_manager.h"
#include "robot_task_profile.h"

static inline void robot_control_register_if_enabled(const control_controller_t *controller,
                                                     robot_task_module_e module)
{
    if (controller == NULL || robot_profile_module_enabled(module) == 0u)
    {
        return;
    }

    (void)control_manager_register(controller);
}

static inline void robot_control_register_profile_defaults(void)
{
    /*
     * Default registration declares resources for diagnostics/arbitration only.
     * It does not make controllers active during boot.
     */
    static const char *const chassis_outputs[] = {
        "motor.chassis0",
        "motor.chassis1",
        "motor.chassis2",
        "motor.chassis3",
    };
    static const char *const single_gimbal_outputs[] = {
        "motor.yaw",
        "motor.pitch",
        "motor.trigger",
    };
    static const char *const dual_yaw_gimbal_outputs[] = {
        "motor.yaw",
        "motor.yaw_upper",
        "motor.pitch",
        "motor.trigger",
    };
    static const char *const shoot_outputs[] = {
        "motor.trigger",
        "motor.friction0",
        "motor.friction1",
        "motor.friction2",
        "motor.friction3",
    };
    static const char *const arm_outputs[] = {
        "motor.arm0",
        "motor.arm1",
        "motor.arm2",
        "motor.arm3",
        "motor.arm4",
        "motor.arm5",
    };

    static const control_controller_t classic_chassis = {
        .id = CONTROL_CONTROLLER_CLASSIC_CHASSIS,
        .domain = CONTROL_DOMAIN_CHASSIS,
        .claim_mask = CONTROL_RESOURCE_CHASSIS_WHEELS,
        .name = "controller.classic_chassis",
        .meta = {
            .period_ms = ROBOT_PROFILE_CHASSIS_CONTROL_DEFAULT_PERIOD_MS,
            .output_count = 4u,
            .outputs = chassis_outputs,
        },
    };
    static const control_controller_t single_gimbal = {
        .id = CONTROL_CONTROLLER_SINGLE_GIMBAL,
        .domain = CONTROL_DOMAIN_GIMBAL,
        .claim_mask = CONTROL_RESOURCE_GIMBAL_YAW | CONTROL_RESOURCE_GIMBAL_PITCH,
        .name = "controller.single_gimbal",
        .meta = {
            .period_ms = ROBOT_PROFILE_GIMBAL_CONTROL_DEFAULT_PERIOD_MS,
            .output_count = 3u,
            .outputs = single_gimbal_outputs,
        },
    };
    static const control_controller_t dual_yaw_gimbal = {
        .id = CONTROL_CONTROLLER_DUAL_YAW_GIMBAL,
        .domain = CONTROL_DOMAIN_GIMBAL,
        .claim_mask = CONTROL_RESOURCE_GIMBAL_YAW | CONTROL_RESOURCE_GIMBAL_PITCH,
        .name = "controller.dual_yaw_gimbal",
        .meta = {
            .period_ms = ROBOT_PROFILE_GIMBAL_CONTROL_DEFAULT_PERIOD_MS,
            .output_count = 4u,
            .outputs = dual_yaw_gimbal_outputs,
        },
    };
    static const control_controller_t shoot = {
        .id = CONTROL_CONTROLLER_SHOOT,
        .domain = CONTROL_DOMAIN_SHOOT,
        .claim_mask = CONTROL_RESOURCE_SHOOT_TRIGGER | CONTROL_RESOURCE_SHOOT_FRICTION,
        .name = "controller.shoot_rm",
        .meta = {
            .period_ms = ROBOT_PROFILE_GIMBAL_CONTROL_DEFAULT_PERIOD_MS,
            .output_count = 5u,
            .outputs = shoot_outputs,
        },
    };
    static const control_controller_t arm = {
        .id = CONTROL_CONTROLLER_ARM_MOTION,
        .domain = CONTROL_DOMAIN_ARM,
        .claim_mask = CONTROL_RESOURCE_ARM,
        .name = "controller.arm_motion",
        .meta = {
            .period_ms = 5u,
            .output_count = 6u,
            .outputs = arm_outputs,
        },
    };
    static const control_controller_t wheelleg_mit = {
        .id = CONTROL_CONTROLLER_WHEELLEG_MIT_BALANCE,
        .domain = CONTROL_DOMAIN_WHEELLEG,
        .claim_mask = CONTROL_RESOURCE_WHEELLEG_LEFT_LEG |
                      CONTROL_RESOURCE_WHEELLEG_RIGHT_LEG |
                      CONTROL_RESOURCE_WHEELLEG_LEFT_WHEEL |
                      CONTROL_RESOURCE_WHEELLEG_RIGHT_WHEEL,
        .name = "controller.wheelleg_mit",
        .meta = {
            .period_ms = 1u,
            .output_count = 6u,
            .outputs = arm_outputs,
        },
    };

    robot_control_register_if_enabled(&classic_chassis, ROBOT_TASK_MODULE_CLASSIC_CHASSIS);
    robot_control_register_if_enabled(&single_gimbal, ROBOT_TASK_MODULE_SINGLE_GIMBAL);
    robot_control_register_if_enabled(&dual_yaw_gimbal, ROBOT_TASK_MODULE_DUAL_YAW_GIMBAL);
    if (robot_profile_module_enabled(ROBOT_TASK_MODULE_SINGLE_GIMBAL) != 0u ||
        robot_profile_module_enabled(ROBOT_TASK_MODULE_DUAL_YAW_GIMBAL) != 0u)
    {
        (void)control_manager_register(&shoot);
    }
    robot_control_register_if_enabled(&arm, ROBOT_TASK_MODULE_ARM);
    robot_control_register_if_enabled(&wheelleg_mit, ROBOT_TASK_MODULE_WHEELLEG_MIT);
}

#endif
