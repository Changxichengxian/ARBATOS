/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef ROBOT_DEVICE_CONFIG_H
#define ROBOT_DEVICE_CONFIG_H

#include <stdint.h>
#include <string.h>

#include "LowCmd.h"
#include "config.h"
#include "ControlMgr.h"

#ifndef ROBOT_CONFIG_DEVICE_BINDING_MAX_INPUTS
#define ROBOT_CONFIG_DEVICE_BINDING_MAX_INPUTS 8u
#endif

#ifndef ROBOT_CONFIG_DEVICE_BINDING_MAX_OUTPUTS
#define ROBOT_CONFIG_DEVICE_BINDING_MAX_OUTPUTS 8u
#endif

typedef enum
{
    ROBOT_CONFIG_DEVICE_KIND_UNKNOWN = 0u,
    ROBOT_CONFIG_DEVICE_KIND_MOTOR = ROBOT_DEVICE_TABLE_KIND_MOTOR,
    ROBOT_CONFIG_DEVICE_KIND_SENSOR = ROBOT_DEVICE_TABLE_KIND_SENSOR,
    ROBOT_CONFIG_DEVICE_KIND_INPUT = ROBOT_DEVICE_TABLE_KIND_INPUT,
    ROBOT_CONFIG_DEVICE_KIND_COMM = ROBOT_DEVICE_TABLE_KIND_COMM,
    ROBOT_CONFIG_DEVICE_KIND_SERVICE = ROBOT_DEVICE_TABLE_KIND_SERVICE,
    ROBOT_CONFIG_DEVICE_KIND_CUSTOM_BASE = ROBOT_DEVICE_TABLE_KIND_CUSTOM_BASE,
} robot_config_device_kind_e;

typedef enum
{
    ROBOT_CONFIG_DEVICE_ROLE_NONE = ROBOT_DEVICE_ROLE_NONE,
    ROBOT_CONFIG_DEVICE_ROLE_IMU = ROBOT_DEVICE_ROLE_IMU,
    ROBOT_CONFIG_DEVICE_ROLE_MANUAL_INPUT = ROBOT_DEVICE_ROLE_MANUAL_INPUT,
    ROBOT_CONFIG_DEVICE_ROLE_BATTERY = ROBOT_DEVICE_ROLE_BATTERY,
    ROBOT_CONFIG_DEVICE_ROLE_AUX_TELEM = ROBOT_DEVICE_ROLE_AUX_TELEM,
    ROBOT_CONFIG_DEVICE_ROLE_SDLOG = ROBOT_DEVICE_ROLE_SDLOG,
    ROBOT_CONFIG_DEVICE_ROLE_CUSTOM_BASE = ROBOT_DEVICE_ROLE_CUSTOM_BASE,
} robot_config_device_role_e;

typedef enum
{
    ROBOT_CONFIG_MOTOR_GROUP_CHASSIS = ROBOT_DEVICE_MOTOR_ROLE_CHASSIS,
    ROBOT_CONFIG_MOTOR_GROUP_YAW = ROBOT_DEVICE_MOTOR_ROLE_YAW,
    ROBOT_CONFIG_MOTOR_GROUP_YAW_UPPER = ROBOT_DEVICE_MOTOR_ROLE_YAW_UPPER,
    ROBOT_CONFIG_MOTOR_GROUP_PITCH = ROBOT_DEVICE_MOTOR_ROLE_PITCH,
    ROBOT_CONFIG_MOTOR_GROUP_TRIGGER = ROBOT_DEVICE_MOTOR_ROLE_TRIGGER,
    ROBOT_CONFIG_MOTOR_GROUP_FRICTION = ROBOT_DEVICE_MOTOR_ROLE_FRICTION,
    ROBOT_CONFIG_MOTOR_GROUP_ARM = ROBOT_DEVICE_MOTOR_ROLE_ARM,
    ROBOT_CONFIG_MOTOR_GROUP_CUSTOM_BASE = ROBOT_DEVICE_MOTOR_ROLE_CUSTOM_BASE,
} robot_config_motor_group_e;

typedef struct
{
    const char *name;
    uint8_t kind;
    uint8_t group;
    uint8_t group_index;
    uint8_t fallback_bus;
    MotorId actuator_id;
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

static inline uint8_t robot_config_device_table_count(void)
{
    const uint8_t count = g_config.devices.count;

    if (count == 0u)
    {
        return 0u;
    }
    return (count > (uint8_t)ROBOT_DEVICE_CONFIG_MAX) ? (uint8_t)ROBOT_DEVICE_CONFIG_MAX : count;
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

static inline MotorId robot_config_source_or_default(uint16_t source_id, MotorId fallback)
{
    return (source_id < (uint16_t)MotorCount) ? (MotorId)source_id : fallback;
}

static inline const void *robot_config_ptr_from_offset(uint32_t config_offset)
{
    if (config_offset == (uint32_t)ROBOT_DEVICE_CONFIG_OFFSET_NONE ||
        config_offset >= (uint32_t)sizeof(g_config))
    {
        return NULL;
    }

    return (const void *)(((const uint8_t *)&g_config) + config_offset);
}

static inline const motor_node_param_t *robot_config_motor_node_from_offset(uint32_t config_offset)
{
    if (config_offset == (uint32_t)ROBOT_DEVICE_CONFIG_OFFSET_NONE ||
        config_offset > ((uint32_t)sizeof(g_config) - (uint32_t)sizeof(motor_node_param_t)))
    {
        return NULL;
    }

    return (const motor_node_param_t *)robot_config_ptr_from_offset(config_offset);
}

static inline const char *robot_config_motor_default_name(uint8_t group, uint8_t group_index)
{
    switch ((robot_config_motor_group_e)group)
    {
    case ROBOT_CONFIG_MOTOR_GROUP_CHASSIS:
        return robot_config_chassis_motor_name(group_index);
    case ROBOT_CONFIG_MOTOR_GROUP_YAW:
        return "motor.yaw";
    case ROBOT_CONFIG_MOTOR_GROUP_YAW_UPPER:
        return "motor.yaw_upper";
    case ROBOT_CONFIG_MOTOR_GROUP_PITCH:
        return "motor.pitch";
    case ROBOT_CONFIG_MOTOR_GROUP_TRIGGER:
        return "motor.trigger";
    case ROBOT_CONFIG_MOTOR_GROUP_FRICTION:
        return robot_config_friction_motor_name(group_index);
    case ROBOT_CONFIG_MOTOR_GROUP_ARM:
        return robot_config_arm_motor_name(group_index);
    default:
        return NULL;
    }
}

static inline MotorId robot_config_motor_default_actuator(uint8_t group, uint8_t group_index)
{
    switch ((robot_config_motor_group_e)group)
    {
    case ROBOT_CONFIG_MOTOR_GROUP_CHASSIS:
        return MotorIdRange(Motor0, group_index, 4u);
    case ROBOT_CONFIG_MOTOR_GROUP_YAW:
        return Motor4;
    case ROBOT_CONFIG_MOTOR_GROUP_YAW_UPPER:
        return Motor5;
    case ROBOT_CONFIG_MOTOR_GROUP_PITCH:
        return Motor6;
    case ROBOT_CONFIG_MOTOR_GROUP_TRIGGER:
        return Motor7;
    case ROBOT_CONFIG_MOTOR_GROUP_FRICTION:
        return MotorIdRange(Motor8, group_index, 4u);
    case ROBOT_CONFIG_MOTOR_GROUP_ARM:
        return MotorIdRange(Motor12, group_index, (uint8_t)MOTOR_ARM_JOINT_COUNT);
    default:
        return MotorCount;
    }
}

static inline uint8_t robot_config_motor_default_bus(uint8_t group, uint8_t group_index)
{
    if (group == (uint8_t)ROBOT_CONFIG_MOTOR_GROUP_FRICTION)
    {
        return 2u;
    }
    if (group == (uint8_t)ROBOT_CONFIG_MOTOR_GROUP_ARM)
    {
        return (group_index == 0u) ? 1u : 2u;
    }
    return 1u;
}

static inline uint8_t robot_config_motor_device_fill(const char *name,
                                                     uint8_t group,
                                                     uint8_t group_index,
                                                     uint8_t fallback_bus,
                                                     uint16_t source_id,
                                                     uint32_t config_offset,
                                                     robot_config_motor_device_t *out)
{
    const motor_node_param_t *node;
    const char *default_name;
    const MotorId fallback_actuator = robot_config_motor_default_actuator(group, group_index);
    const uint8_t known_group = (group < (uint8_t)ROBOT_CONFIG_MOTOR_GROUP_CUSTOM_BASE) ? 1u : 0u;

    if (out == NULL)
    {
        return 0u;
    }

    default_name = robot_config_motor_default_name(group, group_index);
    if (known_group != 0u && (default_name == NULL || fallback_actuator == MotorCount))
    {
        return 0u;
    }

    node = robot_config_motor_node_from_offset(config_offset);
    if (node == NULL)
    {
        return 0u;
    }

    (void)memset(out, 0, sizeof(*out));
    out->kind = (uint8_t)ROBOT_CONFIG_DEVICE_KIND_MOTOR;
    out->name = (name != NULL) ? name : default_name;
    out->group = group;
    out->group_index = group_index;
    out->fallback_bus = (fallback_bus == 0u) ? robot_config_motor_default_bus(group, group_index) : fallback_bus;
    out->actuator_id = robot_config_source_or_default(source_id, fallback_actuator);
    out->node = node;

    if (out->name == NULL || out->actuator_id == MotorCount)
    {
        return 0u;
    }

    return 1u;
}

static inline uint8_t robot_config_motor_device_count(void)
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

static inline uint8_t robot_config_motor_device_get(uint8_t index, robot_config_motor_device_t *out)
{
    if (out == NULL)
    {
        return 0u;
    }

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
            return robot_config_motor_device_fill(entry->name,
                                                  entry->role,
                                                  entry->role_index,
                                                  entry->fallback_bus,
                                                  entry->source_id,
                                                  entry->config_offset,
                                                  out);
        }
        motor_index++;
    }

    return 0u;
}

/* Device-table scans are for init/config/diagnostics, not high-rate control loops. */
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

static inline uint8_t robot_config_motor_device_find_by_actuator(MotorId actuator_id,
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
    return robot_config_device_table_count();
}

static inline uint8_t robot_config_device_get(uint8_t index, robot_config_device_t *out)
{
    robot_config_motor_device_t motor;

    if (out == NULL)
    {
        return 0u;
    }

    (void)memset(out, 0, sizeof(*out));

    const robot_device_config_entry_t *entry;

    if (index >= robot_config_device_table_count())
    {
        return 0u;
    }

    entry = &g_config.devices.entry[index];
    out->kind = entry->kind;
    out->name = entry->name;
    out->group = entry->role;
    out->group_index = entry->role_index;
    out->source_id = entry->source_id;
    out->config = robot_config_ptr_from_offset(entry->config_offset);

    if (entry->kind == ROBOT_DEVICE_TABLE_KIND_MOTOR &&
        robot_config_motor_device_fill(entry->name,
                                       entry->role,
                                       entry->role_index,
                                       entry->fallback_bus,
                                       entry->source_id,
                                       entry->config_offset,
                                       &motor) != 0u)
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

/* Name resolution below walks the configured device list; resolve once before fast loops. */
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

static inline uint8_t robot_config_device_resolve_controller_inputs(const ControlController *controller,
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

static inline uint8_t robot_config_device_resolve_controller_outputs(const ControlController *controller,
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

static inline uint8_t robot_config_device_bind_controller(const ControlController *controller,
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
