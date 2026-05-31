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

#ifndef ROBOT_CONFIG_DEVICE_BINDING_MAX_INPUTS
#define ROBOT_CONFIG_DEVICE_BINDING_MAX_INPUTS 8u
#endif

#ifndef ROBOT_CONFIG_DEVICE_BINDING_MAX_OUTPUTS
#define ROBOT_CONFIG_DEVICE_BINDING_MAX_OUTPUTS 8u
#endif

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

typedef struct
{
    uint8_t input_count;
    uint8_t output_count;
    uint8_t input_resolved;
    uint8_t output_resolved;
    robot_config_device_t inputs[ROBOT_CONFIG_DEVICE_BINDING_MAX_INPUTS];
    robot_config_device_t outputs[ROBOT_CONFIG_DEVICE_BINDING_MAX_OUTPUTS];
} robot_config_device_binding_t;

static inline uint8_t robot_config_motor_device_legacy_count(void)
{
    return (uint8_t)(12u + (uint8_t)MOTOR_ARM_JOINT_COUNT);
}

static inline uint8_t robot_config_device_table_count(void)
{
    const uint8_t count = g_config.devices.count;

    if (count == 0u)
    {
        return 0u;
    }
    return (count > (uint8_t)ROBOT_DEVICE_CONFIG_MAX) ? (uint8_t)ROBOT_DEVICE_CONFIG_MAX : count;
}

static inline uint8_t robot_config_device_table_active(void)
{
    return (robot_config_device_table_count() != 0u) ? 1u : 0u;
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

static inline uint8_t robot_config_motor_device_fill(uint8_t group,
                                                     uint8_t group_index,
                                                     robot_config_motor_device_t *out)
{
    if (out == NULL)
    {
        return 0u;
    }

    (void)memset(out, 0, sizeof(*out));
    out->kind = (uint8_t)ROBOT_CONFIG_DEVICE_KIND_MOTOR;
    out->group = group;
    out->group_index = group_index;
    out->fallback_bus = 1u;

    switch ((robot_config_motor_group_e)group)
    {
    case ROBOT_CONFIG_MOTOR_GROUP_CHASSIS:
        if (group_index >= 4u)
        {
            return 0u;
        }

        out->name = robot_config_chassis_motor_name(group_index);
        out->actuator_id = actuator_id_chassis(group_index);
        out->node = &g_config.motor.chassis[group_index];
        return 1u;
    case ROBOT_CONFIG_MOTOR_GROUP_YAW:
        out->name = "motor.yaw";
        out->actuator_id = ACTUATOR_ID_YAW;
        out->node = &g_config.motor.yaw;
        return 1u;
    case ROBOT_CONFIG_MOTOR_GROUP_YAW_UPPER:
        out->name = "motor.yaw_upper";
        out->actuator_id = ACTUATOR_ID_YAW_UPPER;
        out->node = &g_config.motor.yaw_upper;
        return 1u;
    case ROBOT_CONFIG_MOTOR_GROUP_PITCH:
        out->name = "motor.pitch";
        out->actuator_id = ACTUATOR_ID_PITCH;
        out->node = &g_config.motor.pitch;
        return 1u;
    case ROBOT_CONFIG_MOTOR_GROUP_TRIGGER:
        out->name = "motor.trigger";
        out->actuator_id = ACTUATOR_ID_TRIGGER;
        out->node = &g_config.motor.trigger;
        return 1u;
    case ROBOT_CONFIG_MOTOR_GROUP_FRICTION:
        if (group_index >= 4u)
        {
            return 0u;
        }

        out->name = robot_config_friction_motor_name(group_index);
        out->fallback_bus = 2u;
        out->actuator_id = actuator_id_friction(group_index);
        out->node = &g_config.motor.friction[group_index];
        return 1u;
    case ROBOT_CONFIG_MOTOR_GROUP_ARM:
        if (group_index >= (uint8_t)MOTOR_ARM_JOINT_COUNT)
        {
            return 0u;
        }

        out->name = robot_config_arm_motor_name(group_index);
        out->fallback_bus = (group_index == 0u) ? 1u : 2u;
        out->actuator_id = actuator_id_arm_joint(group_index);
        out->node = &g_config.motor.arm[group_index];
        return (out->name != NULL) ? 1u : 0u;
    default:
        return 0u;
    }
}

static inline uint8_t robot_config_motor_device_count(void)
{
    if (robot_config_device_table_active() != 0u)
    {
        const uint8_t count = robot_config_device_table_count();
        uint8_t motor_count = 0u;

        for (uint8_t i = 0u; i < count; i++)
        {
            if (g_config.devices.entry[i].kind == ROBOT_DEVICE_TABLE_KIND_MOTOR)
            {
                motor_count++;
            }
        }

        return motor_count;
    }

    return robot_config_motor_device_legacy_count();
}

static inline uint8_t robot_config_motor_device_get(uint8_t index, robot_config_motor_device_t *out)
{
    if (out == NULL)
    {
        return 0u;
    }

    if (robot_config_device_table_active() != 0u)
    {
        const uint8_t count = robot_config_device_table_count();
        uint8_t motor_index = 0u;

        for (uint8_t i = 0u; i < count; i++)
        {
            const robot_device_config_entry_t *entry = &g_config.devices.entry[i];

            if (entry->kind != ROBOT_DEVICE_TABLE_KIND_MOTOR)
            {
                continue;
            }
            if (motor_index == index)
            {
                return robot_config_motor_device_fill(entry->group, entry->group_index, out);
            }
            motor_index++;
        }

        return 0u;
    }

    if (index < 4u)
    {
        return robot_config_motor_device_fill((uint8_t)ROBOT_CONFIG_MOTOR_GROUP_CHASSIS, index, out);
    }

    switch (index)
    {
    case 4u:
        return robot_config_motor_device_fill((uint8_t)ROBOT_CONFIG_MOTOR_GROUP_YAW, 0u, out);
    case 5u:
        return robot_config_motor_device_fill((uint8_t)ROBOT_CONFIG_MOTOR_GROUP_YAW_UPPER, 0u, out);
    case 6u:
        return robot_config_motor_device_fill((uint8_t)ROBOT_CONFIG_MOTOR_GROUP_PITCH, 0u, out);
    case 7u:
        return robot_config_motor_device_fill((uint8_t)ROBOT_CONFIG_MOTOR_GROUP_TRIGGER, 0u, out);
    default:
        break;
    }

    if (index < 12u)
    {
        return robot_config_motor_device_fill((uint8_t)ROBOT_CONFIG_MOTOR_GROUP_FRICTION,
                                              (uint8_t)(index - 8u),
                                              out);
    }

    return robot_config_motor_device_fill((uint8_t)ROBOT_CONFIG_MOTOR_GROUP_ARM,
                                          (uint8_t)(index - 12u),
                                          out);
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
    if (robot_config_device_table_active() != 0u)
    {
        return robot_config_device_table_count();
    }

    return robot_config_motor_device_count();
}

static inline uint8_t robot_config_device_get(uint8_t index, robot_config_device_t *out)
{
    robot_config_motor_device_t motor;

    if (out == NULL)
    {
        return 0u;
    }

    (void)memset(out, 0, sizeof(*out));

    if (robot_config_device_table_active() != 0u)
    {
        const robot_device_config_entry_t *entry;

        if (index >= robot_config_device_table_count())
        {
            return 0u;
        }

        entry = &g_config.devices.entry[index];
        out->kind = entry->kind;
        out->group = entry->group;
        out->group_index = entry->group_index;
        out->source_id = 0xFFFFu;

        if (entry->kind == ROBOT_DEVICE_TABLE_KIND_MOTOR &&
            robot_config_motor_device_fill(entry->group, entry->group_index, &motor) != 0u)
        {
            out->name = motor.name;
            out->kind = motor.kind;
            out->group = motor.group;
            out->group_index = motor.group_index;
            out->source_id = (uint16_t)motor.actuator_id;
            out->config = motor.node;
        }

        return 1u;
    }

    if (robot_config_motor_device_get(index, &motor) == 0u)
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

static inline uint8_t robot_config_device_bind_controller(const control_controller_t *controller,
                                                          robot_config_device_binding_t *binding)
{
    if (controller == NULL || binding == NULL)
    {
        return 0u;
    }

    if (controller->meta.input_count > (uint8_t)ROBOT_CONFIG_DEVICE_BINDING_MAX_INPUTS ||
        controller->meta.output_count > (uint8_t)ROBOT_CONFIG_DEVICE_BINDING_MAX_OUTPUTS)
    {
        return 0u;
    }

    (void)memset(binding, 0, sizeof(*binding));
    binding->input_count = controller->meta.input_count;
    binding->output_count = controller->meta.output_count;

    if (binding->input_count != 0u)
    {
        binding->input_resolved =
            robot_config_device_resolve_controller_inputs(controller,
                                                          binding->inputs,
                                                          (uint8_t)ROBOT_CONFIG_DEVICE_BINDING_MAX_INPUTS);
        if (binding->input_resolved != binding->input_count)
        {
            return 0u;
        }
    }

    if (binding->output_count != 0u)
    {
        binding->output_resolved =
            robot_config_device_resolve_controller_outputs(controller,
                                                           binding->outputs,
                                                           (uint8_t)ROBOT_CONFIG_DEVICE_BINDING_MAX_OUTPUTS);
        if (binding->output_resolved != binding->output_count)
        {
            return 0u;
        }
    }

    return 1u;
}

#endif
