/*
 * SPDX-FileCopyrightText: 2026 闄堣僵 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 闄堣僵 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "motor_instance.h"

#include "detect_task.h"
#include "motor_config.h"
#include "robot_task_profile.h"

#include <string.h>

static motor_measure_t s_motor_chassis[4];
static motor_measure_t s_motor_yaw;
static motor_measure_t s_motor_yaw_upper;
static motor_measure_t s_motor_pitch;
static motor_measure_t s_motor_trigger;
static motor_measure_t s_motor_friction[4];
static motor_measure_t s_motor_arm[MOTOR_ARM_JOINT_COUNT];

static motor_instance_t s_motor_instances[ACTUATOR_ID__COUNT];
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

static void motor_instance_ensure(void)
{
    if (s_motor_instance_ready == 0u)
    {
        motor_instance_refresh();
    }
}

void motor_instance_refresh(void)
{
    uint8_t i = 0u;

    s_motor_instance_count = 0u;
    (void)memset(s_motor_instances, 0, sizeof(s_motor_instances));

    for (i = 0u; i < 4u; i++)
    {
        motor_instance_add(actuator_id_chassis(i),
                           MOTOR_INSTANCE_ROLE_CHASSIS,
                           i,
                           1u,
                           (uint8_t)(CHASSIS_MOTOR1_TOE + i),
                           1u,
                           (i == 0u) ? "motor.chassis0" :
                           (i == 1u) ? "motor.chassis1" :
                           (i == 2u) ? "motor.chassis2" : "motor.chassis3",
                           &g_config.motor.chassis[i],
                           &s_motor_chassis[i]);
    }

    motor_instance_add(ACTUATOR_ID_YAW,
                       MOTOR_INSTANCE_ROLE_YAW,
                       0u,
                       1u,
                       YAW_GIMBAL_MOTOR_TOE,
                       1u,
                       "motor.yaw",
                       &g_config.motor.yaw,
                       &s_motor_yaw);
    motor_instance_add(ACTUATOR_ID_YAW_UPPER,
                       MOTOR_INSTANCE_ROLE_YAW_UPPER,
                       0u,
                       1u,
                       MOTOR_INSTANCE_INVALID_DETECT_TOE,
                       0u,
                       "motor.yaw_upper",
                       &g_config.motor.yaw_upper,
                       &s_motor_yaw_upper);
    motor_instance_add(ACTUATOR_ID_PITCH,
                       MOTOR_INSTANCE_ROLE_PITCH,
                       0u,
                       1u,
                       PITCH_GIMBAL_MOTOR_TOE,
                       1u,
                       "motor.pitch",
                       &g_config.motor.pitch,
                       &s_motor_pitch);
    motor_instance_add(ACTUATOR_ID_TRIGGER,
                       MOTOR_INSTANCE_ROLE_TRIGGER,
                       0u,
                       1u,
                       TRIGGER_MOTOR_TOE,
                       1u,
                       "motor.trigger",
                       &g_config.motor.trigger,
                       &s_motor_trigger);

    for (i = 0u; i < 4u; i++)
    {
        motor_instance_add(actuator_id_friction(i),
                           MOTOR_INSTANCE_ROLE_FRICTION,
                           i,
                           2u,
                           MOTOR_INSTANCE_INVALID_DETECT_TOE,
                           0u,
                           (i == 0u) ? "motor.friction0" :
                           (i == 1u) ? "motor.friction1" :
                           (i == 2u) ? "motor.friction2" : "motor.friction3",
                           &g_config.motor.friction[i],
                           &s_motor_friction[i]);
    }

    for (i = 0u; i < (uint8_t)MOTOR_ARM_JOINT_COUNT; i++)
    {
        motor_instance_add(actuator_id_arm_joint(i),
                           MOTOR_INSTANCE_ROLE_ARM,
                           i,
                           (i == 0u) ? 1u : 2u,
                           MOTOR_INSTANCE_INVALID_DETECT_TOE,
                           0u,
                           (i == 0u) ? "motor.arm0" :
                           (i == 1u) ? "motor.arm1" :
                           (i == 2u) ? "motor.arm2" :
                           (i == 3u) ? "motor.arm3" :
                           (i == 4u) ? "motor.arm4" : "motor.arm5",
                           &g_config.motor.arm[i],
                           &s_motor_arm[i]);
    }

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
    uint8_t i = 0u;

    motor_instance_ensure();
    for (i = 0u; i < s_motor_instance_count; i++)
    {
        if (s_motor_instances[i].actuator_id == id)
        {
            return &s_motor_instances[i];
        }
    }
    return NULL;
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
        out[i] = motor_instance_actuator_id_by_name(names[i]);
        if (out[i] != ACTUATOR_ID__COUNT)
        {
            resolved++;
        }
    }

    return resolved;
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

uint8_t motor_instance_cmd_clear(const char *name)
{
    const actuator_id_e id = motor_instance_actuator_id_by_name(name);

    if (id == ACTUATOR_ID__COUNT)
    {
        return 0u;
    }

    actuator_cmd_clear(id);
    return 1u;
}

uint8_t motor_instance_cmd_set_current(const char *name, int16_t current)
{
    actuator_id_e id;

    if (motor_instance_resolve_cmd_target(name, &id) == 0u)
    {
        return 0u;
    }

    actuator_cmd_set_current(id, current);
    return 1u;
}

uint8_t motor_instance_cmd_set_state_torque(const char *name, const actuator_cmd_t *cmd)
{
    actuator_id_e id;

    if (cmd == NULL || motor_instance_resolve_cmd_target(name, &id) == 0u)
    {
        return 0u;
    }

    actuator_cmd_set_state_torque(id, cmd);
    return 1u;
}

uint8_t motor_instance_cmd_set_speed(const char *name, fp32 velocity, fp32 kd, fp32 torque)
{
    actuator_id_e id;

    if (motor_instance_resolve_cmd_target(name, &id) == 0u)
    {
        return 0u;
    }

    actuator_cmd_set_speed(id, velocity, kd, torque);
    return 1u;
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

    for (uint8_t i = 0u; i < count; i++)
    {
        actuator_cmd_set_current(ids[i], currents[i]);
    }

    return 1u;
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

    for (uint8_t i = 0u; i < count; i++)
    {
        if (actuator_feedback_get_copy(ids[i], &out[i]) == 0u)
        {
            return 0u;
        }
    }

    return 1u;
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

const motor_instance_t *motor_instance_find_feedback(uint8_t bus, uint16_t std_id)
{
    uint8_t pass = 0u;
    uint8_t i = 0u;

    motor_instance_ensure();
    for (pass = 0u; pass < 2u; pass++)
    {
        for (i = 0u; i < s_motor_instance_count; i++)
        {
            const motor_instance_t *inst = &s_motor_instances[i];

            if ((pass == 0u && inst->role != MOTOR_INSTANCE_ROLE_ARM) ||
                (pass != 0u && inst->role == MOTOR_INSTANCE_ROLE_ARM))
            {
                continue;
            }
            if (motor_instance_rx_enabled(inst) == 0u)
            {
                continue;
            }
            if (motor_instance_bus(inst) != bus)
            {
                continue;
            }
            if (motor_cfg_transport(inst->node) != MOTOR_TRANSPORT_CAN)
            {
                continue;
            }
            if (motor_cfg_can_id(inst->node) == 0u)
            {
                continue;
            }
            if (std_id == motor_cfg_feedback_id(inst->node))
            {
                return inst;
            }
        }
    }

    return NULL;
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
