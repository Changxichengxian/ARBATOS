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
    inst->node = node;
    inst->measure = measure;
}

static uint8_t motor_instance_arm_role_enabled(void)
{
    return robot_profile_is_wheelleg_mit();
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
                           &g_config.motor.chassis[i],
                           &s_motor_chassis[i]);
    }

    motor_instance_add(ACTUATOR_ID_YAW,
                       MOTOR_INSTANCE_ROLE_YAW,
                       0u,
                       1u,
                       YAW_GIMBAL_MOTOR_TOE,
                       1u,
                       &g_config.motor.yaw,
                       &s_motor_yaw);
    motor_instance_add(ACTUATOR_ID_YAW_UPPER,
                       MOTOR_INSTANCE_ROLE_YAW_UPPER,
                       0u,
                       1u,
                       MOTOR_INSTANCE_INVALID_DETECT_TOE,
                       0u,
                       &g_config.motor.yaw_upper,
                       &s_motor_yaw_upper);
    motor_instance_add(ACTUATOR_ID_PITCH,
                       MOTOR_INSTANCE_ROLE_PITCH,
                       0u,
                       1u,
                       PITCH_GIMBAL_MOTOR_TOE,
                       1u,
                       &g_config.motor.pitch,
                       &s_motor_pitch);
    motor_instance_add(ACTUATOR_ID_TRIGGER,
                       MOTOR_INSTANCE_ROLE_TRIGGER,
                       0u,
                       1u,
                       TRIGGER_MOTOR_TOE,
                       1u,
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
