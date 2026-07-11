/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ROBOT_CONTROL_REGISTRY_H
#define ROBOT_CONTROL_REGISTRY_H

#include "CanReceive.h"
#include "ControlActuatorPolicy.h"
#include "ControlMgr.h"
#include "MotorInst.h"
#include "RobotDeviceConfig.h"
#include "RobotTaskBuildConfig.h"
#include "RobotTaskProfile.h"
#if ROBOT_TASK_BUILD_CAN_COMMAND_TX
#include "CanTxTask.h"
#endif
#if ROBOT_TASK_BUILD_CLASSIC_CHASSIS
#include "ChassisCtrl.h"
#endif
#if ROBOT_TASK_BUILD_SHOOT_RM
#include "ShootCtrl.h"
#endif

typedef char RobotControlActuatorMaskWidthCheck[
    ((uint32_t)MotorCount <= 32u) ? 1 : -1];

static inline void RobotControlAuditInventory(ControlActuatorAudit *audit)
{
    uint32_t configured_mask = 0u;

    if (audit == NULL)
    {
        return;
    }

    for (uint8_t i = 0u; i < MotorInstCount(); i++)
    {
        const MotorInst *inst = MotorInstGet(i);

        if (inst != NULL)
        {
            (void)ControlActuatorAuditAdd(audit,
                                          &configured_mask,
                                          (uint16_t)MotorInstId(inst));
        }
    }

    for (uint8_t i = 0u; i < MotorRouteCount(); i++)
    {
        const MotorRoute *route = MotorRouteGet(i);

        if (route != NULL)
        {
            ControlActuatorRouteView route_view;

            (void)memset(&route_view, 0, sizeof(route_view));
            route_view.canId = route->canId;
            route_view.enabled = route->enabled;
            route_view.transport = route->transport;
            route_view.protocol = route->protocol;
            route_view.isRmGroup = route->isRmGroup;
            route_view.bus = route->bus;
            route_view.hasLimits = (route->mitLimits != NULL) ? 1u : 0u;
            route_view.canBusCount = RobotBoardCanBusCount();
            route_view.rs485PortCount = RobotBoardRs485PortCount();
            if (route->node != NULL)
            {
                route_view.deviceId =
                    (route->protocol == (uint8_t)MOTOR_PROTOCOL_N6014B_RS485 &&
                     route->node->feedback_id_enable != 0u) ?
                        route->node->feedback_id : (uint16_t)route->node->can_id;
            }

            if (ControlActuatorRouteRoutable(&route_view) != 0u)
            {
                (void)ControlActuatorAuditAdd(audit,
                                              &audit->routableMask,
                                              (uint16_t)route->motorId);
            }
        }
    }

    for (uint8_t i = 0u; i < RobotConfigDeviceTableCount(); i++)
    {
        const RobotDeviceConfigEntry *entry = &g_config.devices.entry[i];

        if (entry->kind == ROBOT_DEVICE_TABLE_KIND_MOTOR &&
            entry->source_id != ROBOT_DEVICE_SOURCE_NONE &&
            entry->source_id >= (uint16_t)MotorCount)
        {
            audit->invalidIdCount++;
        }
    }
}

static inline uint8_t RobotControlResolveDeviceActuator(const char *name,
                                                        uint16_t *raw_id,
                                                        uint8_t *invalid_source)
{
    if (raw_id == NULL || invalid_source == NULL)
    {
        return 0u;
    }
    *raw_id = (uint16_t)MotorCount;
    *invalid_source = 0u;
    if (name == NULL)
    {
        return 0u;
    }

    for (uint8_t i = 0u; i < RobotConfigDeviceTableCount(); i++)
    {
        const RobotDeviceConfigEntry *entry = &g_config.devices.entry[i];
        const char *entry_name;

        if (entry->kind != ROBOT_DEVICE_TABLE_KIND_MOTOR)
        {
            continue;
        }
        entry_name = (entry->name != NULL) ? entry->name :
            RobotConfigMotorDefaultName(entry->role, entry->role_index);
        if (entry_name == NULL || strcmp(entry_name, name) != 0)
        {
            continue;
        }

        {
            MotorId resolved;
            const MotorId fallback =
                RobotConfigMotorDefaultActuator(entry->role, entry->role_index);

            if (ControlActuatorResolveSourceId(entry->source_id,
                                               fallback,
                                               &resolved) != 0u)
            {
                *raw_id = (uint16_t)resolved;
                return 1u;
            }
        }
        if (entry->source_id != ROBOT_DEVICE_SOURCE_NONE)
        {
            /* 非法 source_id 的旧回退仍由设备层保留，但不能冒充真实所有权。 */
            *invalid_source = 1u;
        }
        return 0u;
    }
    return 0u;
}

static inline uint32_t RobotControlResolveNamedActuators(const ControlController *controller,
                                                         ControlActuatorAudit *audit)
{
    uint32_t mask = 0u;
    uint8_t count;

    if (controller == NULL || audit == NULL)
    {
        return 0u;
    }

    count = controller->meta.output_count;
    if (count > (uint8_t)MotorCount)
    {
        ControlActuatorAuditAddUnresolved(audit, count);
        return 0u;
    }
    if (count != 0u && controller->meta.outputs == NULL)
    {
        ControlActuatorAuditAddUnresolved(audit, count);
        return 0u;
    }
    for (uint8_t i = 0u; i < count; i++)
    {
        uint16_t raw_id;
        uint8_t invalid_source;

        if (RobotControlResolveDeviceActuator(controller->meta.outputs[i],
                                              &raw_id,
                                              &invalid_source) == 0u)
        {
            if (invalid_source == 0u)
            {
                ControlActuatorAuditAddUnresolved(audit, 1u);
            }
            continue;
        }

        ControlActuatorAuditDeclare(
            audit,
            &mask,
            raw_id,
            (MotorInstFindByName(controller->meta.outputs[i]) != NULL) ? 1u : 0u);
    }
    return mask;
}

static inline uint32_t RobotControlResolveRawActuators(const uint8_t *ids,
                                                       uint8_t count,
                                                       ControlActuatorAudit *audit)
{
    uint32_t mask = 0u;

    if (audit == NULL)
    {
        return 0u;
    }
    if (ids == NULL)
    {
        ControlActuatorAuditAddUnresolved(audit, count);
        return 0u;
    }

    for (uint8_t i = 0u; i < count; i++)
    {
        const uint16_t raw_id = ids[i];

        ControlActuatorAuditDeclare(
            audit,
            &mask,
            raw_id,
            (raw_id < (uint16_t)MotorCount &&
             MotorInstFindByMotor((MotorId)raw_id) != NULL) ? 1u : 0u);
    }
    return mask;
}

static inline void RobotControlRegisterNamed(const ControlController *controller,
                                             ControlActuatorAudit *audit)
{
    if (controller != NULL)
    {
        ControlController resolved = *controller;

        resolved.actuator_mask = RobotControlResolveNamedActuators(controller, audit);
        (void)ControlMgrRegister(&resolved);
        return;
    }

    (void)ControlMgrRegister(controller);
}

static inline void RobotControlRegisterIfEnabled(const ControlController *controller,
                                                 RobotTaskModule module,
                                                 ControlActuatorAudit *audit)
{
    if (RobotProfileModuleEnabled(module) == 0u)
    {
        return;
    }

    /* 启用模块的空描述也交给 ControlMgr 记录，启动失败不能静默消失。 */
    RobotControlRegisterNamed(controller, audit);
}

static inline void RobotControlRegisterProfileDefaults(void)
{
    ControlActuatorAudit actuator_audit = {0};

    /*
     * Registration only declares resources for diagnostics/arbitration.
     * Boot activation is handled by RobotControlStartProfileDefaults().
     */
    RobotControlAuditInventory(&actuator_audit);
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
    RobotControlRegisterIfEnabled(ChassisCtrlDesc(),
                                  ROBOT_TASK_MODULE_CLASSIC_CHASSIS,
                                  &actuator_audit);
#endif
#if ROBOT_TASK_BUILD_SINGLE_GIMBAL
    RobotControlRegisterIfEnabled(&single_gimbal,
                                  ROBOT_TASK_MODULE_SINGLE_GIMBAL,
                                  &actuator_audit);
#endif
#if ROBOT_TASK_BUILD_DUAL_YAW_GIMBAL
    RobotControlRegisterIfEnabled(&DualYawGimbal,
                                  ROBOT_TASK_MODULE_DUAL_YAW_GIMBAL,
                                  &actuator_audit);
#endif
#if ROBOT_TASK_BUILD_SHOOT_RM
    if (RobotProfileModuleEnabled(ROBOT_TASK_MODULE_SINGLE_GIMBAL) != 0u ||
        RobotProfileModuleEnabled(ROBOT_TASK_MODULE_DUAL_YAW_GIMBAL) != 0u)
    {
        RobotControlRegisterNamed(ShootCtrlDesc(), &actuator_audit);
    }
#endif
#if ROBOT_TASK_BUILD_ARM
    RobotControlRegisterIfEnabled(&arm, ROBOT_TASK_MODULE_ARM, &actuator_audit);
#endif
#if ROBOT_TASK_BUILD_WHEELLEG_MIT
    if (RobotProfileModuleEnabled(ROBOT_TASK_MODULE_WHEELLEG_MIT) != 0u)
    {
        const uint8_t wheelleg_actuators[] = {
            g_config.WheelLegMit.left_front_actuator,
            g_config.WheelLegMit.left_back_actuator,
            g_config.WheelLegMit.left_wheel_actuator,
            g_config.WheelLegMit.right_front_actuator,
            g_config.WheelLegMit.right_back_actuator,
            g_config.WheelLegMit.right_wheel_actuator,
        };
        ControlController resolved = WheelLegMit;

        resolved.actuator_mask = RobotControlResolveRawActuators(
            wheelleg_actuators,
            (uint8_t)(sizeof(wheelleg_actuators) / sizeof(wheelleg_actuators[0])),
            &actuator_audit);
        (void)ControlMgrRegister(&resolved);
    }
#endif
    (void)ControlMgrSetActuatorAudit(&actuator_audit);
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
    MotorInstRefresh();
    CAN_rx_prepare_motor_measure_points();
    ControlMgrInit();
    RobotControlRegisterProfileDefaults();
    RobotControlStartProfileDefaults();
#if ROBOT_TASK_BUILD_CAN_COMMAND_TX
    CanTxEmergencyPrepare();
#endif
}

#endif
