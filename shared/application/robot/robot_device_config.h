/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 Xie Yuhan <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef ROBOT_DEVICE_CONFIG_H
#define ROBOT_DEVICE_CONFIG_H

#include <stdint.h>
#include <string.h>

#include "actuator_cmd.h"
#include "config.h"
#include "control_manager.h"

typedef enum
{
    ROBOT_CONFIG_DEVICE_KIND_UNKNOWN = 0u,
    ROBOT_CONFIG_DEVICE_KIND_MOTOR,
    ROBOT_CONFIG_DEVICE_KIND_CUSTOM_BASE = 128u,
} robot_config_device_kind_e;

typedef enum
{
    ROBOT_CONFIG_MOTOR_GROUP_CHASSIS = 0u,
    ROBOT_CONFIG_MOTOR_GROUP_YAW,
    ROBOT_CONFIG_MOTOR_GROUP_YAW_UPPER,
    ROBOT_CONFIG_MOTOR_GROUP_PITCH,
    ROBOT_CONFIG_MOTOR_GROUP_TRIGGER,
    ROBOT_CONFIG_MOTOR_GROUP_FRICTION,
    ROBOT_CONFIG_MOTOR_GROUP_ARM,
    ROBOT_CONFIG_MOTOR_GROUP_CUSTOM_BASE = 128u,
} robot_config_motor_group_e;

typedef struct
{
    const char *name;
    uint8_t kind;
    uint8_t group;
    uint8_t group_index;
    uint8_t fallback_bus;
    actuator_id_e actuator_id;
    const motor_node_param_t *node;
} robot_config_motor_device_t;

typedef struct
{
    const char *name;
    uint8_t kind;
    uint8_t group;
    uint8_t group_index;
    uint8_t reserved0;
    uint16_t source_id;
    const void *config;
} robot_config_device_t;

static inline uint8_t robot_config_motor_device_count(void)
{
    return (uint8_t)(12u + (uint8_t)MOTOR_ARM_JOINT_COUNT);
}

static inline const char *robot_config_chassis_motor_name(uint8_t index)
{
    switch (index)
    {
    case 0u:
        return "motor.chassis0";
    case 1u:
        return "motor.chassis1";
    case 2u:
        return "motor.chassis2";
    case 3u:
        return "motor.chassis3";
    default:
        return NULL;
    }
}

static inline const char *robot_config_friction_motor_name(uint8_t index)
{
    switch (index)
    {
    case 0u:
        return "motor.friction0";
    case 1u:
        return "motor.friction1";
    case 2u:
        return "motor.friction2";
    case 3u:
        return "motor.friction3";
    default:
        return NULL;
    }
}

static inline const char *robot_config_arm_motor_name(uint8_t index)
{
    switch (index)
    {
    case 0u:
        return "motor.arm0";
    case 1u:
        return "motor.arm1";
    case 2u:
        return "motor.arm2";
    case 3u:
        return "motor.arm3";
    case 4u:
        return "motor.arm4";
    case 5u:
        return "motor.arm5";
    default:
        return NULL;
    }
}

static inline uint8_t robot_config_motor_device_get(uint8_t index, robot_config_motor_device_t *out)
{
    if (out == NULL || index >= robot_config_motor_device_count())
    {
        return 0u;
    }

    out->kind = (uint8_t)ROBOT_CONFIG_DEVICE_KIND_MOTOR;
    out->group_index = 0u;
    out->fallback_bus = 1u;

    if (index < 4u)
    {
        out->name = robot_config_chassis_motor_name(index);
        out->group = (uint8_t)ROBOT_CONFIG_MOTOR_GROUP_CHASSIS;
        out->group_index = index;
        out->actuator_id = actuator_id_chassis(index);
        out->node = &g_config.motor.chassis[index];
        return 1u;
    }

    switch (index)
    {
    case 4u:
        out->name = "motor.yaw";
        out->group = (uint8_t)ROBOT_CONFIG_MOTOR_GROUP_YAW;
        out->actuator_id = ACTUATOR_ID_YAW;
        out->node = &g_config.motor.yaw;
        return 1u;
    case 5u:
        out->name = "motor.yaw_upper";
        out->group = (uint8_t)ROBOT_CONFIG_MOTOR_GROUP_YAW_UPPER;
        out->actuator_id = ACTUATOR_ID_YAW_UPPER;
        out->node = &g_config.motor.yaw_upper;
        return 1u;
    case 6u:
        out->name = "motor.pitch";
        out->group = (uint8_t)ROBOT_CONFIG_MOTOR_GROUP_PITCH;
        out->actuator_id = ACTUATOR_ID_PITCH;
        out->node = &g_config.motor.pitch;
        return 1u;
    case 7u:
        out->name = "motor.trigger";
        out->group = (uint8_t)ROBOT_CONFIG_MOTOR_GROUP_TRIGGER;
        out->actuator_id = ACTUATOR_ID_TRIGGER;
        out->node = &g_config.motor.trigger;
        return 1u;
    default:
        break;
    }

    if (index < 12u)
    {
        const uint8_t friction_index = (uint8_t)(index - 8u);

        out->name = robot_config_friction_motor_name(friction_index);
        out->group = (uint8_t)ROBOT_CONFIG_MOTOR_GROUP_FRICTION;
        out->group_index = friction_index;
        out->fallback_bus = 2u;
        out->actuator_id = actuator_id_friction(friction_index);
        out->node = &g_config.motor.friction[friction_index];
        return 1u;
    }

    {
        const uint8_t arm_index = (uint8_t)(index - 12u);

        out->name = robot_config_arm_motor_name(arm_index);
        out->group = (uint8_t)ROBOT_CONFIG_MOTOR_GROUP_ARM;
        out->group_index = arm_index;
        out->fallback_bus = (arm_index == 0u) ? 1u : 2u;
        out->actuator_id = actuator_id_arm_joint(arm_index);
        out->node = &g_config.motor.arm[arm_index];
        return (out->name != NULL) ? 1u : 0u;
    }
}

static inline uint8_t robot_config_motor_device_find_by_name(const char *name, robot_config_motor_device_t *out)
{
    const uint8_t count = robot_config_motor_device_count();

    if (name == NULL)
    {
        return 0u;
    }

    for (uint8_t i = 0u; i < count; i++)
    {
        robot_config_motor_device_t device;

        if (robot_config_motor_device_get(i, &device) != 0u &&
            device.name != NULL &&
            strcmp(device.name, name) == 0)
        {
            if (out != NULL)
            {
                *out = device;
            }
            return 1u;
        }
    }

    return 0u;
}

static inline uint8_t robot_config_motor_device_find_by_actuator(actuator_id_e actuator_id,
                                                                 robot_config_motor_device_t *out)
{
    const uint8_t count = robot_config_motor_device_count();

    for (uint8_t i = 0u; i < count; i++)
    {
        robot_config_motor_device_t device;

        if (robot_config_motor_device_get(i, &device) != 0u &&
            device.actuator_id == actuator_id)
        {
            if (out != NULL)
            {
                *out = device;
            }
            return 1u;
        }
    }

    return 0u;
}

static inline uint8_t robot_config_device_count(void)
{
    return robot_config_motor_device_count();
}

static inline uint8_t robot_config_device_get(uint8_t index, robot_config_device_t *out)
{
    robot_config_motor_device_t motor;

    if (out == NULL || robot_config_motor_device_get(index, &motor) == 0u)
    {
        return 0u;
    }

    out->name = motor.name;
    out->kind = motor.kind;
    out->group = motor.group;
    out->group_index = motor.group_index;
    out->reserved0 = 0u;
    out->source_id = (uint16_t)motor.actuator_id;
    out->config = motor.node;
    return 1u;
}

static inline uint8_t robot_config_device_find_by_name(const char *name, robot_config_device_t *out)
{
    const uint8_t count = robot_config_device_count();

    if (name == NULL)
    {
        return 0u;
    }

    for (uint8_t i = 0u; i < count; i++)
    {
        robot_config_device_t device;

        if (robot_config_device_get(i, &device) != 0u &&
            device.name != NULL &&
            strcmp(device.name, name) == 0)
        {
            if (out != NULL)
            {
                *out = device;
            }
            return 1u;
        }
    }

    return 0u;
}

static inline uint8_t robot_config_device_resolve_names(const char *const *names,
                                                        uint8_t count,
                                                        robot_config_device_t *out,
                                                        uint8_t out_cap)
{
    uint8_t resolved = 0u;

    if (names == NULL || out == NULL || count > out_cap)
    {
        return 0u;
    }

    for (uint8_t i = 0u; i < count; i++)
    {
        if (robot_config_device_find_by_name(names[i], &out[i]) != 0u)
        {
            resolved++;
        }
        else
        {
            (void)memset(&out[i], 0, sizeof(out[i]));
        }
    }

    return resolved;
}

static inline uint8_t robot_config_device_resolve_source_ids(const char *const *names,
                                                             uint8_t count,
                                                             uint16_t *out,
                                                             uint8_t out_cap)
{
    uint8_t resolved = 0u;

    if (names == NULL || out == NULL || count > out_cap)
    {
        return 0u;
    }

    for (uint8_t i = 0u; i < count; i++)
    {
        robot_config_device_t device;

        if (robot_config_device_find_by_name(names[i], &device) != 0u)
        {
            out[i] = device.source_id;
            resolved++;
        }
        else
        {
            out[i] = 0xFFFFu;
        }
    }

    return resolved;
}

static inline uint8_t robot_config_device_resolve_controller_inputs(const control_controller_t *controller,
                                                                    robot_config_device_t *out,
                                                                    uint8_t out_cap)
{
    if (controller == NULL)
    {
        return 0u;
    }

    return robot_config_device_resolve_names(controller->meta.inputs,
                                            controller->meta.input_count,
                                            out,
                                            out_cap);
}

static inline uint8_t robot_config_device_resolve_controller_outputs(const control_controller_t *controller,
                                                                     robot_config_device_t *out,
                                                                     uint8_t out_cap)
{
    if (controller == NULL)
    {
        return 0u;
    }

    return robot_config_device_resolve_names(controller->meta.outputs,
                                            controller->meta.output_count,
                                            out,
                                            out_cap);
}

#endif
