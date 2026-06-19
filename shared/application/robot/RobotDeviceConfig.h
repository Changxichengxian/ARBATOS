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
#include "RobotConfig.h"
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
} RobotConfigDeviceKind;

typedef enum
{
    ROBOT_CONFIG_DEVICE_ROLE_NONE = ROBOT_DEVICE_ROLE_NONE,
    ROBOT_CONFIG_DEVICE_ROLE_IMU = ROBOT_DEVICE_ROLE_IMU,
    ROBOT_CONFIG_DEVICE_ROLE_MANUAL_INPUT = ROBOT_DEVICE_ROLE_MANUAL_INPUT,
    ROBOT_CONFIG_DEVICE_ROLE_BATTERY = ROBOT_DEVICE_ROLE_BATTERY,
    ROBOT_CONFIG_DEVICE_ROLE_AUX_TELEM = ROBOT_DEVICE_ROLE_AUX_TELEM,
    ROBOT_CONFIG_DEVICE_ROLE_SDLOG = ROBOT_DEVICE_ROLE_SDLOG,
    ROBOT_CONFIG_DEVICE_ROLE_CUSTOM_BASE = ROBOT_DEVICE_ROLE_CUSTOM_BASE,
} RobotConfigDeviceRole;

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
} RobotConfigMotorGroup;

typedef struct
{
    const char *name;
    uint8_t kind;
    uint8_t group;
    uint8_t group_index;
    uint8_t fallback_bus;
    MotorId actuator_id;
    const motor_node_param_t *node;
} RobotConfigMotorDevice;

typedef struct
{
    const char *name;
    uint8_t kind;
    uint8_t group;
    uint8_t group_index;
    uint8_t reserved0;
    uint16_t source_id;
    const void *config;
} RobotConfigDevice;

typedef struct
{
    uint8_t input_count;
    uint8_t output_count;
    uint8_t input_resolved;
    uint8_t output_resolved;
    RobotConfigDevice inputs[ROBOT_CONFIG_DEVICE_BINDING_MAX_INPUTS];
    RobotConfigDevice outputs[ROBOT_CONFIG_DEVICE_BINDING_MAX_OUTPUTS];
} RobotConfigDeviceBinding;

static inline uint8_t RobotConfigDeviceTableCount(void)
{
    const uint8_t count = g_config.devices.count;

    if (count == 0u)
    {
        return 0u;
    }
    return (count > (uint8_t)ROBOT_DEVICE_CONFIG_MAX) ? (uint8_t)ROBOT_DEVICE_CONFIG_MAX : count;
}

static inline const char *RobotConfigChassisMotorName(uint8_t index)
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

static inline const char *RobotConfigFrictionMotorName(uint8_t index)
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

static inline const char *RobotConfigArmMotorName(uint8_t index)
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

static inline MotorId RobotConfigSourceOrDefault(uint16_t source_id, MotorId fallback)
{
    return (source_id < (uint16_t)MotorCount) ? (MotorId)source_id : fallback;
}

static inline const void *RobotConfigPtrFromOffset(uint32_t ConfigOffset)
{
    if (ConfigOffset == (uint32_t)ROBOT_DEVICE_CONFIG_OFFSET_NONE ||
        ConfigOffset >= (uint32_t)sizeof(g_config))
    {
        return NULL;
    }

    return (const void *)(((const uint8_t *)&g_config) + ConfigOffset);
}

static inline const motor_node_param_t *RobotConfigMotorNodeFromOffset(uint32_t ConfigOffset)
{
    if (ConfigOffset == (uint32_t)ROBOT_DEVICE_CONFIG_OFFSET_NONE ||
        ConfigOffset > ((uint32_t)sizeof(g_config) - (uint32_t)sizeof(motor_node_param_t)))
    {
        return NULL;
    }

    return (const motor_node_param_t *)RobotConfigPtrFromOffset(ConfigOffset);
}

static inline const char *RobotConfigMotorDefaultName(uint8_t group, uint8_t group_index)
{
    switch ((RobotConfigMotorGroup)group)
    {
    case ROBOT_CONFIG_MOTOR_GROUP_CHASSIS:
        return RobotConfigChassisMotorName(group_index);
    case ROBOT_CONFIG_MOTOR_GROUP_YAW:
        return "motor.yaw";
    case ROBOT_CONFIG_MOTOR_GROUP_YAW_UPPER:
        return "motor.yaw_upper";
    case ROBOT_CONFIG_MOTOR_GROUP_PITCH:
        return "motor.pitch";
    case ROBOT_CONFIG_MOTOR_GROUP_TRIGGER:
        return "motor.trigger";
    case ROBOT_CONFIG_MOTOR_GROUP_FRICTION:
        return RobotConfigFrictionMotorName(group_index);
    case ROBOT_CONFIG_MOTOR_GROUP_ARM:
        return RobotConfigArmMotorName(group_index);
    default:
        return NULL;
    }
}

static inline MotorId RobotConfigMotorDefaultActuator(uint8_t group, uint8_t group_index)
{
    switch ((RobotConfigMotorGroup)group)
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

static inline uint8_t RobotConfigMotorDefaultBus(uint8_t group, uint8_t group_index)
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

static inline uint8_t RobotConfigMotorDeviceFill(const char *name,
                                                     uint8_t group,
                                                     uint8_t group_index,
                                                     uint8_t fallback_bus,
                                                     uint16_t source_id,
                                                     uint32_t ConfigOffset,
                                                     RobotConfigMotorDevice *out)
{
    const motor_node_param_t *node;
    const char *default_name;
    const MotorId fallback_actuator = RobotConfigMotorDefaultActuator(group, group_index);
    const uint8_t known_group = (group < (uint8_t)ROBOT_CONFIG_MOTOR_GROUP_CUSTOM_BASE) ? 1u : 0u;

    if (out == NULL)
    {
        return 0u;
    }

    default_name = RobotConfigMotorDefaultName(group, group_index);
    if (known_group != 0u && (default_name == NULL || fallback_actuator == MotorCount))
    {
        return 0u;
    }

    node = RobotConfigMotorNodeFromOffset(ConfigOffset);
    if (node == NULL)
    {
        return 0u;
    }

    (void)memset(out, 0, sizeof(*out));
    out->kind = (uint8_t)ROBOT_CONFIG_DEVICE_KIND_MOTOR;
    out->name = (name != NULL) ? name : default_name;
    out->group = group;
    out->group_index = group_index;
    out->fallback_bus = (fallback_bus == 0u) ? RobotConfigMotorDefaultBus(group, group_index) : fallback_bus;
    out->actuator_id = RobotConfigSourceOrDefault(source_id, fallback_actuator);
    out->node = node;

    if (out->name == NULL || out->actuator_id == MotorCount)
    {
        return 0u;
    }

    return 1u;
}

static inline uint8_t RobotConfigMotorDeviceCount(void)
{
    const uint8_t count = RobotConfigDeviceTableCount();
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

static inline uint8_t RobotConfigMotorDeviceGet(uint8_t index, RobotConfigMotorDevice *out)
{
    if (out == NULL)
    {
        return 0u;
    }

    const uint8_t count = RobotConfigDeviceTableCount();
    uint8_t motor_index = 0u;

    for (uint8_t i = 0u; i < count; i++)
    {
        const RobotDeviceConfigEntry *entry = &g_config.devices.entry[i];

        if (entry->kind != ROBOT_DEVICE_TABLE_KIND_MOTOR)
        {
            continue;
        }
        if (motor_index == index)
        {
            return RobotConfigMotorDeviceFill(entry->name,
                                                  entry->role,
                                                  entry->role_index,
                                                  entry->fallback_bus,
                                                  entry->source_id,
                                                  entry->ConfigOffset,
                                                  out);
        }
        motor_index++;
    }

    return 0u;
}

/* Device-table scans are for init/config/diagnostics, not high-rate control loops. */
static inline uint8_t RobotConfigMotorDeviceFindByName(const char *name, RobotConfigMotorDevice *out)
{
    const uint8_t count = RobotConfigMotorDeviceCount();

    if (name == NULL)
    {
        return 0u;
    }

    for (uint8_t i = 0u; i < count; i++)
    {
        RobotConfigMotorDevice device;

        if (RobotConfigMotorDeviceGet(i, &device) != 0u &&
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

static inline uint8_t RobotConfigMotorDeviceFindByActuator(MotorId actuator_id,
                                                                 RobotConfigMotorDevice *out)
{
    const uint8_t count = RobotConfigMotorDeviceCount();

    for (uint8_t i = 0u; i < count; i++)
    {
        RobotConfigMotorDevice device;

        if (RobotConfigMotorDeviceGet(i, &device) != 0u &&
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

static inline uint8_t RobotConfigDeviceCount(void)
{
    return RobotConfigDeviceTableCount();
}

static inline uint8_t RobotConfigDeviceGet(uint8_t index, RobotConfigDevice *out)
{
    RobotConfigMotorDevice motor;

    if (out == NULL)
    {
        return 0u;
    }

    (void)memset(out, 0, sizeof(*out));

    const RobotDeviceConfigEntry *entry;

    if (index >= RobotConfigDeviceTableCount())
    {
        return 0u;
    }

    entry = &g_config.devices.entry[index];
    out->kind = entry->kind;
    out->name = entry->name;
    out->group = entry->role;
    out->group_index = entry->role_index;
    out->source_id = entry->source_id;
    out->config = RobotConfigPtrFromOffset(entry->ConfigOffset);

    if (entry->kind == ROBOT_DEVICE_TABLE_KIND_MOTOR &&
        RobotConfigMotorDeviceFill(entry->name,
                                       entry->role,
                                       entry->role_index,
                                       entry->fallback_bus,
                                       entry->source_id,
                                       entry->ConfigOffset,
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
static inline uint8_t RobotConfigDeviceFindByName(const char *name, RobotConfigDevice *out)
{
    const uint8_t count = RobotConfigDeviceCount();

    if (name == NULL)
    {
        return 0u;
    }

    for (uint8_t i = 0u; i < count; i++)
    {
        RobotConfigDevice device;

        if (RobotConfigDeviceGet(i, &device) != 0u &&
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

static inline uint8_t RobotConfigDeviceResolveNames(const char *const *names,
                                                        uint8_t count,
                                                        RobotConfigDevice *out,
                                                        uint8_t out_cap)
{
    uint8_t resolved = 0u;

    if (names == NULL || out == NULL || count > out_cap)
    {
        return 0u;
    }

    for (uint8_t i = 0u; i < count; i++)
    {
        if (RobotConfigDeviceFindByName(names[i], &out[i]) != 0u)
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

static inline uint8_t RobotConfigDeviceResolveSourceIds(const char *const *names,
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
        RobotConfigDevice device;

        if (RobotConfigDeviceFindByName(names[i], &device) != 0u)
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

static inline uint8_t RobotConfigDeviceResolveControllerInputs(const ControlController *controller,
                                                                    RobotConfigDevice *out,
                                                                    uint8_t out_cap)
{
    if (controller == NULL)
    {
        return 0u;
    }

    return RobotConfigDeviceResolveNames(controller->meta.inputs,
                                            controller->meta.input_count,
                                            out,
                                            out_cap);
}

static inline uint8_t RobotConfigDeviceResolveControllerOutputs(const ControlController *controller,
                                                                     RobotConfigDevice *out,
                                                                     uint8_t out_cap)
{
    if (controller == NULL)
    {
        return 0u;
    }

    return RobotConfigDeviceResolveNames(controller->meta.outputs,
                                            controller->meta.output_count,
                                            out,
                                            out_cap);
}

static inline uint8_t RobotConfigDeviceBindController(const ControlController *controller,
                                                          RobotConfigDeviceBinding *binding)
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
            RobotConfigDeviceResolveControllerInputs(controller,
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
            RobotConfigDeviceResolveControllerOutputs(controller,
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
