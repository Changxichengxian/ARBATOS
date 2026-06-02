/*
 * SPDX-FileCopyrightText: 2026 闄堣僵 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "motor_instance.h"

#include "control_manager.h"
#include "detect_task.h"
#include "motor_config.h"
#include "robot_device_config.h"
#include "robot_task_profile.h"

#include <string.h>

#define MOTOR_INSTANCE_FEEDBACK_LOOKUP_CAPACITY (ACTUATOR_ID__COUNT * 2u)

typedef struct
{
    uint8_t used;
    uint8_t bus;
    uint16_t std_id;
    const motor_instance_t *inst;
} motor_instance_feedback_lookup_slot_t;

static motor_measure_t s_motor_measure[ACTUATOR_ID__COUNT];
static motor_instance_t s_motor_instances[ACTUATOR_ID__COUNT];
static motor_instance_t *s_motor_instance_by_actuator[ACTUATOR_ID__COUNT];
static motor_instance_feedback_lookup_slot_t s_motor_feedback_lookup[MOTOR_INSTANCE_FEEDBACK_LOOKUP_CAPACITY];
static uint8_t s_motor_instance_count = 0u;
static uint8_t s_motor_instance_ready = 0u;

static void motor_instance_add(actuator_id_e actuator_id,
                               motor_instance_role_e role,
                               uint8_t role_index,
                               uint8_t fallback_bus,
                               uint8_t detect_toe,
                               uint8_t use_detect,
                               const char *name,
                               const motor_node_param_t *node,
                               motor_measure_t *measure)
{
    motor_instance_t *inst = NULL;

    if (s_motor_instance_count >= (uint8_t)ACTUATOR_ID__COUNT)
    {
        return;
    }
    if ((uint32_t)actuator_id >= (uint32_t)ACTUATOR_ID__COUNT)
    {
        return;
    }

    inst = &s_motor_instances[s_motor_instance_count++];
    inst->actuator_id = actuator_id;
    inst->role = role;
    inst->role_index = role_index;
    inst->fallback_bus = fallback_bus;
    inst->detect_toe = detect_toe;
    inst->use_detect = use_detect;
    inst->name = name;
    inst->node = node;
    inst->measure = measure;
    s_motor_instance_by_actuator[actuator_id] = inst;
}

static motor_instance_role_e motor_instance_role_from_config(uint8_t group)
{
    switch ((robot_config_motor_group_e)group)
    {
    case ROBOT_CONFIG_MOTOR_GROUP_CHASSIS:
        return MOTOR_INSTANCE_ROLE_CHASSIS;
    case ROBOT_CONFIG_MOTOR_GROUP_YAW:
        return MOTOR_INSTANCE_ROLE_YAW;
    case ROBOT_CONFIG_MOTOR_GROUP_YAW_UPPER:
        return MOTOR_INSTANCE_ROLE_YAW_UPPER;
    case ROBOT_CONFIG_MOTOR_GROUP_PITCH:
        return MOTOR_INSTANCE_ROLE_PITCH;
    case ROBOT_CONFIG_MOTOR_GROUP_TRIGGER:
        return MOTOR_INSTANCE_ROLE_TRIGGER;
    case ROBOT_CONFIG_MOTOR_GROUP_FRICTION:
        return MOTOR_INSTANCE_ROLE_FRICTION;
    case ROBOT_CONFIG_MOTOR_GROUP_ARM:
        return MOTOR_INSTANCE_ROLE_ARM;
    default:
        return MOTOR_INSTANCE_ROLE_CHASSIS;
    }
}

static uint8_t motor_instance_detect_toe_from_config(const robot_config_motor_device_t *device)
{
    if (device == NULL)
    {
        return MOTOR_INSTANCE_INVALID_DETECT_TOE;
    }

    switch ((robot_config_motor_group_e)device->group)
    {
    case ROBOT_CONFIG_MOTOR_GROUP_CHASSIS:
        return (uint8_t)(CHASSIS_MOTOR1_TOE + device->group_index);
    case ROBOT_CONFIG_MOTOR_GROUP_YAW:
        return YAW_GIMBAL_MOTOR_TOE;
    case ROBOT_CONFIG_MOTOR_GROUP_PITCH:
        return PITCH_GIMBAL_MOTOR_TOE;
    case ROBOT_CONFIG_MOTOR_GROUP_TRIGGER:
        return TRIGGER_MOTOR_TOE;
    default:
        return MOTOR_INSTANCE_INVALID_DETECT_TOE;
    }
}

static uint8_t motor_instance_use_detect_from_config(const robot_config_motor_device_t *device)
{
    if (device == NULL)
    {
        return 0u;
    }

    switch ((robot_config_motor_group_e)device->group)
    {
    case ROBOT_CONFIG_MOTOR_GROUP_CHASSIS:
    case ROBOT_CONFIG_MOTOR_GROUP_YAW:
    case ROBOT_CONFIG_MOTOR_GROUP_PITCH:
    case ROBOT_CONFIG_MOTOR_GROUP_TRIGGER:
        return 1u;
    default:
        return 0u;
    }
}

static motor_measure_t *motor_instance_measure_from_actuator(actuator_id_e actuator_id)
{
    if ((uint32_t)actuator_id >= (uint32_t)ACTUATOR_ID__COUNT)
    {
        return NULL;
    }

    return &s_motor_measure[actuator_id];
}

static uint8_t motor_instance_arm_role_enabled(void)
{
    return (uint8_t)(robot_profile_need_arm_task() || robot_profile_is_wheelleg_mit());
}

static uint8_t motor_instance_rx_enabled(const motor_instance_t *inst)
{
    if (motor_instance_enabled(inst) == 0u)
    {
        return 0u;
    }
    if (inst->role == MOTOR_INSTANCE_ROLE_ARM && motor_instance_arm_role_enabled() == 0u)
    {
        return 0u;
    }
    return 1u;
}

static uint8_t motor_instance_feedback_hash(uint8_t bus, uint16_t std_id)
{
    return (uint8_t)(((uint16_t)(std_id ^ ((uint16_t)bus << 5u))) %
                     (uint16_t)MOTOR_INSTANCE_FEEDBACK_LOOKUP_CAPACITY);
}

static void motor_instance_feedback_lookup_clear(void)
{
    (void)memset(s_motor_feedback_lookup, 0, sizeof(s_motor_feedback_lookup));
}

static void motor_instance_feedback_lookup_insert(const motor_instance_t *inst)
{
    uint8_t slot = 0u;
    const uint8_t bus = (inst != NULL) ? motor_instance_bus(inst) : 0u;
    const uint16_t std_id = (inst != NULL) ? motor_cfg_feedback_id(inst->node) : 0u;

    if (inst == NULL ||
        motor_instance_rx_enabled(inst) == 0u ||
        bus == 0u ||
        motor_cfg_transport(inst->node) != MOTOR_TRANSPORT_CAN ||
        motor_cfg_can_id(inst->node) == 0u)
    {
        return;
    }

    slot = motor_instance_feedback_hash(bus, std_id);
    for (uint8_t i = 0u; i < (uint8_t)MOTOR_INSTANCE_FEEDBACK_LOOKUP_CAPACITY; i++)
    {
        motor_instance_feedback_lookup_slot_t *entry =
            &s_motor_feedback_lookup[(uint8_t)((slot + i) % (uint8_t)MOTOR_INSTANCE_FEEDBACK_LOOKUP_CAPACITY)];

        if (entry->used == 0u)
        {
            entry->used = 1u;
            entry->bus = bus;
            entry->std_id = std_id;
            entry->inst = inst;
            return;
        }
        if (entry->bus == bus && entry->std_id == std_id)
        {
            return;
        }
    }
}

static void motor_instance_feedback_lookup_rebuild(void)
{
    motor_instance_feedback_lookup_clear();

    for (uint8_t pass = 0u; pass < 2u; pass++)
    {
        for (uint8_t i = 0u; i < s_motor_instance_count; i++)
        {
            const motor_instance_t *inst = &s_motor_instances[i];

            if ((pass == 0u && inst->role != MOTOR_INSTANCE_ROLE_ARM) ||
                (pass != 0u && inst->role == MOTOR_INSTANCE_ROLE_ARM))
            {
                continue;
            }

            motor_instance_feedback_lookup_insert(inst);
        }
    }
}

static const motor_instance_t *motor_instance_feedback_lookup_get(uint8_t bus, uint16_t std_id)
{
    const uint8_t slot = motor_instance_feedback_hash(bus, std_id);

    for (uint8_t i = 0u; i < (uint8_t)MOTOR_INSTANCE_FEEDBACK_LOOKUP_CAPACITY; i++)
    {
        const motor_instance_feedback_lookup_slot_t *entry =
            &s_motor_feedback_lookup[(uint8_t)((slot + i) % (uint8_t)MOTOR_INSTANCE_FEEDBACK_LOOKUP_CAPACITY)];

        if (entry->used == 0u)
        {
            return NULL;
        }
        if (entry->bus == bus && entry->std_id == std_id)
        {
            return entry->inst;
        }
    }

    return NULL;
}

static void motor_instance_ensure(void)
{
    if (s_motor_instance_ready == 0u)
    {
        motor_instance_refresh();
    }
}

void motor_instance_refresh(void)
{
    const uint8_t device_count = robot_config_motor_device_count();

    s_motor_instance_count = 0u;
    (void)memset(s_motor_instances, 0, sizeof(s_motor_instances));
    (void)memset(s_motor_instance_by_actuator, 0, sizeof(s_motor_instance_by_actuator));
    motor_instance_feedback_lookup_clear();

    for (uint8_t i = 0u; i < device_count; i++)
    {
        robot_config_motor_device_t device;
        motor_measure_t *measure;

        if (robot_config_motor_device_get(i, &device) == 0u)
        {
            continue;
        }

        measure = motor_instance_measure_from_actuator(device.actuator_id);
        if (measure == NULL)
        {
            continue;
        }

        motor_instance_add(device.actuator_id,
                           motor_instance_role_from_config(device.group),
                           device.group_index,
                           device.fallback_bus,
                           motor_instance_detect_toe_from_config(&device),
                           motor_instance_use_detect_from_config(&device),
                           device.name,
                           device.node,
                           measure);
    }

    motor_instance_feedback_lookup_rebuild();
    s_motor_instance_ready = 1u;
}

uint8_t motor_instance_count(void)
{
    motor_instance_ensure();
    return s_motor_instance_count;
}

const motor_instance_t *motor_instance_get(uint8_t index)
{
    motor_instance_ensure();
    if (index >= s_motor_instance_count)
    {
        return NULL;
    }
    return &s_motor_instances[index];
}

const motor_instance_t *motor_instance_find_by_actuator(actuator_id_e id)
{
    motor_instance_ensure();
    if ((uint32_t)id >= (uint32_t)ACTUATOR_ID__COUNT)
    {
        return NULL;
    }
    return s_motor_instance_by_actuator[id];
}

const motor_instance_t *motor_instance_find_by_name(const char *name)
{
    uint8_t i = 0u;

    if (name == NULL)
    {
        return NULL;
    }

    motor_instance_ensure();
    for (i = 0u; i < s_motor_instance_count; i++)
    {
        if (s_motor_instances[i].name != NULL &&
            strcmp(s_motor_instances[i].name, name) == 0)
        {
            return &s_motor_instances[i];
        }
    }
    return NULL;
}

uint8_t motor_instance_resolve_actuator_ids(const char *const *names, uint8_t count, actuator_id_e *out, uint8_t out_cap)
{
    uint8_t resolved = 0u;

    if (names == NULL || out == NULL || count > out_cap)
    {
        return 0u;
    }

    for (uint8_t i = 0u; i < count; i++)
    {
        robot_config_motor_device_t device;

        if (robot_config_motor_device_find_by_name(names[i], &device) != 0u)
        {
            out[i] = device.actuator_id;
            resolved++;
        }
        else
        {
            out[i] = ACTUATOR_ID__COUNT;
        }
    }

    return resolved;
}

uint8_t motor_instance_bind_current_outputs(const char *const *names,
                                            uint8_t count,
                                            motor_instance_current_binding_t *out,
                                            uint8_t out_cap)
{
    uint8_t bound = 0u;

    if (names == NULL || out == NULL || count > out_cap)
    {
        return 0u;
    }

    for (uint8_t i = 0u; i < count; i++)
    {
        const motor_instance_t *inst = motor_instance_find_by_name(names[i]);

        out[i].actuator_id = ACTUATOR_ID__COUNT;
        out[i].enabled = 0u;

        if (inst == NULL || motor_instance_enabled(inst) == 0u)
        {
            continue;
        }

        out[i].actuator_id = inst->actuator_id;
        out[i].enabled = 1u;
        bound++;
    }

    return bound;
}

const char *motor_instance_name(const motor_instance_t *inst)
{
    return (inst != NULL) ? inst->name : NULL;
}

actuator_id_e motor_instance_actuator_id(const motor_instance_t *inst)
{
    return (inst != NULL) ? inst->actuator_id : ACTUATOR_ID__COUNT;
}

actuator_id_e motor_instance_actuator_id_by_name(const char *name)
{
    return motor_instance_actuator_id(motor_instance_find_by_name(name));
}

uint8_t motor_instance_bus(const motor_instance_t *inst)
{
    if (inst == NULL || inst->node == NULL)
    {
        return 0u;
    }
    return motor_cfg_can_bus(inst->fallback_bus, inst->node);
}

uint8_t motor_instance_enabled(const motor_instance_t *inst)
{
    if (inst == NULL || inst->node == NULL)
    {
        return 0u;
    }
    return (motor_cfg_node_id(inst->node) != 0u) ? 1u : 0u;
}

static uint8_t motor_instance_resolve_cmd_target(const char *name, actuator_id_e *out)
{
    const motor_instance_t *inst = motor_instance_find_by_name(name);

    if (inst == NULL || out == NULL || motor_instance_enabled(inst) == 0u)
    {
        return 0u;
    }

    *out = inst->actuator_id;
    return 1u;
}

static uint8_t motor_instance_id_cmd_enabled(actuator_id_e id)
{
    const motor_instance_t *inst = motor_instance_find_by_actuator(id);

    return (uint8_t)(inst != NULL && motor_instance_enabled(inst) != 0u);
}

static uint8_t motor_instance_node_cmd_caps(const motor_node_param_t *node)
{
    const motor_model_db_entry_t *entry;
    uint8_t caps = (uint8_t)ACTUATOR_CMD_CAP_CURRENT;
    const motor_transport_e transport = motor_cfg_transport(node);
    const motor_protocol_e protocol = motor_cfg_protocol(node);

    if (node == NULL || motor_cfg_node_id(node) == 0u)
    {
        return 0u;
    }

    caps |= (uint8_t)ACTUATOR_CMD_CAP_FEEDBACK;
    if (transport == MOTOR_TRANSPORT_CAN && protocol == MOTOR_PROTOCOL_RM_GROUP)
    {
        return (uint8_t)(caps |
                         (uint8_t)ACTUATOR_CMD_CAP_STATE_TORQUE |
                         (uint8_t)ACTUATOR_CMD_CAP_POS_VEL |
                         (uint8_t)ACTUATOR_CMD_CAP_SPEED |
                         (uint8_t)ACTUATOR_CMD_CAP_FORCE_POS);
    }

    entry = motor_cfg_model_db(node->model);
    if (entry == NULL)
    {
        return caps;
    }
    if ((entry->caps & (uint8_t)MOTOR_MODEL_CAP_MIT) != 0u)
    {
        caps |= (uint8_t)ACTUATOR_CMD_CAP_STATE_TORQUE;
    }
    if ((entry->caps & (uint8_t)MOTOR_MODEL_CAP_POS_VEL) != 0u)
    {
        caps |= (uint8_t)ACTUATOR_CMD_CAP_POS_VEL;
    }
    if ((entry->caps & (uint8_t)MOTOR_MODEL_CAP_SPEED) != 0u)
    {
        caps |= (uint8_t)ACTUATOR_CMD_CAP_SPEED;
    }
    if ((entry->caps & (uint8_t)MOTOR_MODEL_CAP_FORCE_POS) != 0u)
    {
        caps |= (uint8_t)ACTUATOR_CMD_CAP_FORCE_POS;
    }

    return caps;
}

uint8_t motor_instance_model_id(actuator_id_e id)
{
    const motor_node_param_t *node = motor_instance_node(id);

    return (node != NULL) ? (uint8_t)node->model : 0xFFu;
}

uint8_t motor_instance_transport_id(actuator_id_e id)
{
    const motor_node_param_t *node = motor_instance_node(id);

    return (node != NULL) ? (uint8_t)motor_cfg_transport(node) : 0u;
}

uint8_t motor_instance_protocol_id(actuator_id_e id)
{
    const motor_node_param_t *node = motor_instance_node(id);

    return (node != NULL) ? (uint8_t)motor_cfg_protocol(node) : 0u;
}

uint8_t motor_instance_control_mode_id(actuator_id_e id)
{
    const motor_node_param_t *node = motor_instance_node(id);

    return (node != NULL) ? (uint8_t)motor_cfg_control_mode(node) : 0u;
}

uint8_t motor_instance_cmd_caps_id(actuator_id_e id)
{
    const motor_instance_t *inst = motor_instance_find_by_actuator(id);

    if (inst == NULL || motor_instance_enabled(inst) == 0u)
    {
        return 0u;
    }

    return motor_instance_node_cmd_caps(inst->node);
}

uint8_t motor_instance_cmd_mode_supported_id(actuator_id_e id, actuator_cmd_mode_e mode)
{
    const uint8_t caps = motor_instance_cmd_caps_id(id);

    if (mode == ACTUATOR_CMD_MODE_NONE)
    {
        return (motor_instance_find_by_actuator(id) != NULL) ? 1u : 0u;
    }
    if (actuator_cmd_mode_known((uint8_t)mode) == 0u || caps == 0u)
    {
        return 0u;
    }

    switch (mode)
    {
    case ACTUATOR_CMD_MODE_CURRENT:
        return (uint8_t)((caps & (uint8_t)ACTUATOR_CMD_CAP_CURRENT) != 0u);
    case ACTUATOR_CMD_MODE_STATE_TORQUE:
        return (uint8_t)((caps & (uint8_t)ACTUATOR_CMD_CAP_STATE_TORQUE) != 0u);
    case ACTUATOR_CMD_MODE_POS_VEL:
        return (uint8_t)((caps & (uint8_t)ACTUATOR_CMD_CAP_POS_VEL) != 0u);
    case ACTUATOR_CMD_MODE_SPEED:
        return (uint8_t)((caps & (uint8_t)ACTUATOR_CMD_CAP_SPEED) != 0u);
    case ACTUATOR_CMD_MODE_FORCE_POS:
        return (uint8_t)((caps & (uint8_t)ACTUATOR_CMD_CAP_FORCE_POS) != 0u);
    default:
        return 0u;
    }
}

uint8_t motor_instance_cmd_clear_id(actuator_id_e id)
{
    if ((uint32_t)id >= (uint32_t)ACTUATOR_ID__COUNT ||
        motor_instance_find_by_actuator(id) == NULL)
    {
        return 0u;
    }

    actuator_cmd_clear(id);
    return 1u;
}

uint8_t motor_instance_cmd_set_ids(const actuator_id_e *ids, const actuator_cmd_t *cmds, uint8_t count)
{
    if (count > (uint8_t)ACTUATOR_ID__COUNT)
    {
        return 0u;
    }
    if (count != 0u && (ids == NULL || cmds == NULL))
    {
        return 0u;
    }

    for (uint8_t i = 0u; i < count; i++)
    {
        if (motor_instance_id_cmd_enabled(ids[i]) == 0u ||
            motor_instance_cmd_mode_supported_id(ids[i], (actuator_cmd_mode_e)cmds[i].mode) == 0u)
        {
            return 0u;
        }
    }

    return actuator_cmd_set_many(ids, cmds, count);
}

uint8_t motor_instance_cmd_set_ids_best_effort(const actuator_id_e *ids, const actuator_cmd_t *cmds, uint8_t count)
{
    actuator_id_e writable_ids[ACTUATOR_ID__COUNT];
    actuator_cmd_t writable_cmds[ACTUATOR_ID__COUNT];
    uint8_t written = 0u;

    if (ids == NULL || cmds == NULL || count > (uint8_t)ACTUATOR_ID__COUNT)
    {
        return 0u;
    }

    for (uint8_t i = 0u; i < count; i++)
    {
        if (motor_instance_id_cmd_enabled(ids[i]) == 0u ||
            motor_instance_cmd_mode_supported_id(ids[i], (actuator_cmd_mode_e)cmds[i].mode) == 0u)
        {
            continue;
        }

        writable_ids[written] = ids[i];
        writable_cmds[written] = cmds[i];
        written++;
    }

    if (actuator_cmd_set_many(writable_ids, writable_cmds, written) == 0u)
    {
        return 0u;
    }

    return written;
}

uint8_t motor_instance_cmd_set_current_id(actuator_id_e id, int16_t current)
{
    if (motor_instance_id_cmd_enabled(id) == 0u)
    {
        return 0u;
    }

    actuator_cmd_set_current(id, current);
    return 1u;
}

uint8_t motor_instance_cmd_set_state_torque_id(actuator_id_e id, const actuator_cmd_t *cmd)
{
    actuator_cmd_t tmp;

    if (cmd == NULL)
    {
        return 0u;
    }

    tmp = *cmd;
    tmp.active = 1u;
    tmp.mode = (uint8_t)ACTUATOR_CMD_MODE_STATE_TORQUE;
    return motor_instance_cmd_set_ids(&id, &tmp, 1u);
}

uint8_t motor_instance_cmd_set_state_torque_ids(const actuator_id_e *ids, const actuator_cmd_t *cmds, uint8_t count)
{
    actuator_cmd_t prepared[ACTUATOR_ID__COUNT];

    if (count > (uint8_t)ACTUATOR_ID__COUNT)
    {
        return 0u;
    }
    if (count != 0u && (ids == NULL || cmds == NULL))
    {
        return 0u;
    }

    for (uint8_t i = 0u; i < count; i++)
    {
        prepared[i] = cmds[i];
        prepared[i].active = 1u;
        prepared[i].mode = (uint8_t)ACTUATOR_CMD_MODE_STATE_TORQUE;
    }

    return motor_instance_cmd_set_ids(ids, prepared, count);
}

uint8_t motor_instance_cmd_set_state_torque_ids_best_effort(const actuator_id_e *ids,
                                                            const actuator_cmd_t *cmds,
                                                            uint8_t count)
{
    actuator_cmd_t prepared[ACTUATOR_ID__COUNT];

    if (count > (uint8_t)ACTUATOR_ID__COUNT)
    {
        return 0u;
    }
    if (count != 0u && (ids == NULL || cmds == NULL))
    {
        return 0u;
    }

    for (uint8_t i = 0u; i < count; i++)
    {
        prepared[i] = cmds[i];
        prepared[i].active = 1u;
        prepared[i].mode = (uint8_t)ACTUATOR_CMD_MODE_STATE_TORQUE;
    }

    return motor_instance_cmd_set_ids_best_effort(ids, prepared, count);
}

uint8_t motor_instance_cmd_set_speed_id(actuator_id_e id, fp32 velocity, fp32 kd, fp32 torque)
{
    actuator_cmd_t cmd;

    (void)memset(&cmd, 0, sizeof(cmd));
    cmd.active = 1u;
    cmd.mode = (uint8_t)ACTUATOR_CMD_MODE_SPEED;
    cmd.velocity = velocity;
    cmd.kd = kd;
    cmd.torque = torque;
    return motor_instance_cmd_set_ids(&id, &cmd, 1u);
}

uint8_t motor_instance_cmd_clear(const char *name)
{
    return motor_instance_cmd_clear_id(motor_instance_actuator_id_by_name(name));
}

uint8_t motor_instance_cmd_set_current(const char *name, int16_t current)
{
    actuator_id_e id;

    if (motor_instance_resolve_cmd_target(name, &id) == 0u)
    {
        return 0u;
    }

    return motor_instance_cmd_set_current_id(id, current);
}

uint8_t motor_instance_cmd_set_state_torque(const char *name, const actuator_cmd_t *cmd)
{
    actuator_id_e id;

    if (motor_instance_resolve_cmd_target(name, &id) == 0u)
    {
        return 0u;
    }

    return motor_instance_cmd_set_state_torque_id(id, cmd);
}

uint8_t motor_instance_cmd_set_speed(const char *name, fp32 velocity, fp32 kd, fp32 torque)
{
    actuator_id_e id;

    if (motor_instance_resolve_cmd_target(name, &id) == 0u)
    {
        return 0u;
    }

    return motor_instance_cmd_set_speed_id(id, velocity, kd, torque);
}

uint8_t motor_instance_cmd_get_copy(const char *name, actuator_cmd_t *out)
{
    const actuator_id_e id = motor_instance_actuator_id_by_name(name);

    if (id == ACTUATOR_ID__COUNT || out == NULL)
    {
        return 0u;
    }

    return actuator_cmd_get_copy(id, out);
}

uint8_t motor_instance_feedback_get_copy(const char *name, actuator_feedback_t *out)
{
    const actuator_id_e id = motor_instance_actuator_id_by_name(name);

    if (id == ACTUATOR_ID__COUNT || out == NULL)
    {
        return 0u;
    }

    return actuator_feedback_get_copy(id, out);
}

uint8_t motor_instance_cmd_set_current_ids(const actuator_id_e *ids, const int16_t *currents, uint8_t count)
{
    if (ids == NULL || currents == NULL)
    {
        return 0u;
    }

    for (uint8_t i = 0u; i < count; i++)
    {
        if (motor_instance_id_cmd_enabled(ids[i]) == 0u)
        {
            return 0u;
        }
    }

    return actuator_cmd_set_current_many(ids, currents, count);
}

uint8_t motor_instance_cmd_set_current_many(const char *const *names, const int16_t *currents, uint8_t count)
{
    actuator_id_e ids[ACTUATOR_ID__COUNT];

    if (count > (uint8_t)ACTUATOR_ID__COUNT)
    {
        return 0u;
    }
    if (motor_instance_resolve_actuator_ids(names, count, ids, (uint8_t)ACTUATOR_ID__COUNT) != count)
    {
        return 0u;
    }

    return motor_instance_cmd_set_current_ids(ids, currents, count);
}

uint8_t motor_instance_cmd_set_current_ids_best_effort(const actuator_id_e *ids, const int16_t *currents, uint8_t count)
{
    actuator_id_e writable_ids[ACTUATOR_ID__COUNT];
    int16_t writable_currents[ACTUATOR_ID__COUNT];
    uint8_t written = 0u;

    if (ids == NULL || currents == NULL || count > (uint8_t)ACTUATOR_ID__COUNT)
    {
        return 0u;
    }

    for (uint8_t i = 0u; i < count; i++)
    {
        if (motor_instance_id_cmd_enabled(ids[i]) == 0u)
        {
            continue;
        }

        writable_ids[written] = ids[i];
        writable_currents[written] = currents[i];
        written++;
    }

    if (actuator_cmd_set_current_many(writable_ids, writable_currents, written) == 0u)
    {
        return 0u;
    }

    return written;
}

uint8_t motor_instance_cmd_set_current_many_best_effort(const char *const *names, const int16_t *currents, uint8_t count)
{
    actuator_id_e writable_ids[ACTUATOR_ID__COUNT];
    int16_t writable_currents[ACTUATOR_ID__COUNT];
    uint8_t written = 0u;

    if (names == NULL || currents == NULL || count > (uint8_t)ACTUATOR_ID__COUNT)
    {
        return 0u;
    }

    for (uint8_t i = 0u; i < count; i++)
    {
        actuator_id_e id;

        if (motor_instance_resolve_cmd_target(names[i], &id) == 0u)
        {
            continue;
        }

        writable_ids[written] = id;
        writable_currents[written] = currents[i];
        written++;
    }

    if (actuator_cmd_set_current_many(writable_ids, writable_currents, written) == 0u)
    {
        return 0u;
    }

    return written;
}

uint8_t motor_instance_cmd_set_current_bindings_best_effort(const motor_instance_current_binding_t *bindings,
                                                            const int16_t *currents,
                                                            uint8_t count)
{
    actuator_id_e writable_ids[ACTUATOR_ID__COUNT];
    int16_t writable_currents[ACTUATOR_ID__COUNT];
    uint8_t written = 0u;

    if (bindings == NULL || currents == NULL || count > (uint8_t)ACTUATOR_ID__COUNT)
    {
        return 0u;
    }

    for (uint8_t i = 0u; i < count; i++)
    {
        if (bindings[i].enabled == 0u ||
            (uint32_t)bindings[i].actuator_id >= (uint32_t)ACTUATOR_ID__COUNT)
        {
            continue;
        }

        writable_ids[written] = bindings[i].actuator_id;
        writable_currents[written] = currents[i];
        written++;
    }

    if (actuator_cmd_set_current_many(writable_ids, writable_currents, written) == 0u)
    {
        return 0u;
    }

    return written;
}

uint8_t motor_instance_feedback_get_copy_ids(const actuator_id_e *ids, actuator_feedback_t *out, uint8_t count)
{
    if (ids == NULL || out == NULL)
    {
        return 0u;
    }

    for (uint8_t i = 0u; i < count; i++)
    {
        if ((uint32_t)ids[i] >= (uint32_t)ACTUATOR_ID__COUNT)
        {
            return 0u;
        }
    }

    return actuator_feedback_get_copy_many(ids, out, count);
}

uint8_t motor_instance_feedback_get_copy_many(const char *const *names, actuator_feedback_t *out, uint8_t count)
{
    actuator_id_e ids[ACTUATOR_ID__COUNT];

    if (count > (uint8_t)ACTUATOR_ID__COUNT)
    {
        return 0u;
    }
    if (motor_instance_resolve_actuator_ids(names, count, ids, (uint8_t)ACTUATOR_ID__COUNT) != count)
    {
        return 0u;
    }

    return motor_instance_feedback_get_copy_ids(ids, out, count);
}

uint8_t motor_instance_resolve_controller_outputs(const struct control_controller *controller,
                                                  actuator_id_e *out,
                                                  uint8_t out_cap)
{
    if (controller == NULL)
    {
        return 0u;
    }

    return motor_instance_resolve_actuator_ids(controller->meta.outputs,
                                              controller->meta.output_count,
                                              out,
                                              out_cap);
}

const motor_instance_t *motor_instance_find_feedback(uint8_t bus, uint16_t std_id)
{
    motor_instance_ensure();
    return motor_instance_feedback_lookup_get(bus, std_id);
}

const motor_node_param_t *motor_instance_node(actuator_id_e id)
{
    const motor_instance_t *inst = motor_instance_find_by_actuator(id);
    return (inst != NULL) ? inst->node : NULL;
}

motor_measure_t *motor_instance_measure(actuator_id_e id)
{
    const motor_instance_t *inst = motor_instance_find_by_actuator(id);
    return (inst != NULL) ? inst->measure : NULL;
}

const motor_measure_t *motor_instance_measure_const(actuator_id_e id)
{
    return motor_instance_measure(id);
}
