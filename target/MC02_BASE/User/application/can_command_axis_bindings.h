/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef CAN_COMMAND_AXIS_BINDINGS_H
#define CAN_COMMAND_AXIS_BINDINGS_H

#include "actuator_cmd.h"
#include "config.h"

static inline int16_t can_tx_axis_limit_i16(int16_t current, int16_t max_abs)
{
    if (max_abs <= 0)
    {
        return current;
    }
    if (current > max_abs)
    {
        return max_abs;
    }
    if (current < -max_abs)
    {
        return (int16_t)-max_abs;
    }
    return current;
}

static const motor_node_param_t s_can_tx_axis_chassis0_node =
{
    .model = MOTOR_MODEL_3508,
    .can_id = 1u,
};
static const motor_node_param_t s_can_tx_axis_chassis1_node =
{
    .model = MOTOR_MODEL_3508,
    .can_id = 2u,
};
static const motor_node_param_t s_can_tx_axis_chassis2_node =
{
    .model = MOTOR_MODEL_3508,
    .can_id = 7u,
};
static const motor_node_param_t s_can_tx_axis_chassis3_node =
{
    .model = MOTOR_MODEL_3508,
    .can_id = 5u,
};
static const motor_node_param_t s_can_tx_axis_yaw_node =
{
    .model = MOTOR_MODEL_6020,
    .can_id = 1u,
};
static const motor_node_param_t s_can_tx_axis_yaw_upper_node =
{
    .model = MOTOR_MODEL_6020,
    .can_id = 0u,
};
static const motor_node_param_t s_can_tx_axis_pitch_node =
{
    .model = MOTOR_MODEL_3510,
    .can_id = 6u,
};
static const motor_node_param_t s_can_tx_axis_trigger_node =
{
    .model = MOTOR_MODEL_3510,
    .can_id = 7u,
};
static const motor_node_param_t s_can_tx_axis_friction0_node =
{
    .model = MOTOR_MODEL_3510,
    .can_id = 1u,
};
static const motor_node_param_t s_can_tx_axis_friction1_node =
{
    .model = MOTOR_MODEL_3510,
    .can_id = 2u,
};
static const motor_node_param_t s_can_tx_axis_friction2_node =
{
    .model = MOTOR_MODEL_3510,
    .can_id = 3u,
};
static const motor_node_param_t s_can_tx_axis_friction3_node =
{
    .model = MOTOR_MODEL_3510,
    .can_id = 4u,
};

#define CAN_TX_AXIS_CHASSIS0_ACTUATOR_ID ACTUATOR_ID_CHASSIS0
#define CAN_TX_AXIS_CHASSIS0_FALLBACK_BUS 1u
#define CAN_TX_AXIS_CHASSIS0_NODE() (&s_can_tx_axis_chassis0_node)
#define CAN_TX_AXIS_CHASSIS0_CAN_ID() 0x201u
#define CAN_TX_AXIS_CHASSIS0_IS_RM_GROUP() 1u
#define CAN_TX_AXIS_CHASSIS0_LIMIT_CURRENT(current_) can_tx_axis_limit_i16((current_), 16000)

#define CAN_TX_AXIS_CHASSIS1_ACTUATOR_ID ACTUATOR_ID_CHASSIS1
#define CAN_TX_AXIS_CHASSIS1_FALLBACK_BUS 1u
#define CAN_TX_AXIS_CHASSIS1_NODE() (&s_can_tx_axis_chassis1_node)
#define CAN_TX_AXIS_CHASSIS1_CAN_ID() 0x202u
#define CAN_TX_AXIS_CHASSIS1_IS_RM_GROUP() 1u
#define CAN_TX_AXIS_CHASSIS1_LIMIT_CURRENT(current_) can_tx_axis_limit_i16((current_), 16000)

#define CAN_TX_AXIS_CHASSIS2_ACTUATOR_ID ACTUATOR_ID_CHASSIS2
#define CAN_TX_AXIS_CHASSIS2_FALLBACK_BUS 1u
#define CAN_TX_AXIS_CHASSIS2_NODE() (&s_can_tx_axis_chassis2_node)
#define CAN_TX_AXIS_CHASSIS2_CAN_ID() 0x207u
#define CAN_TX_AXIS_CHASSIS2_IS_RM_GROUP() 1u
#define CAN_TX_AXIS_CHASSIS2_LIMIT_CURRENT(current_) can_tx_axis_limit_i16((current_), 16000)

#define CAN_TX_AXIS_CHASSIS3_ACTUATOR_ID ACTUATOR_ID_CHASSIS3
#define CAN_TX_AXIS_CHASSIS3_FALLBACK_BUS 1u
#define CAN_TX_AXIS_CHASSIS3_NODE() (&s_can_tx_axis_chassis3_node)
#define CAN_TX_AXIS_CHASSIS3_CAN_ID() 0x205u
#define CAN_TX_AXIS_CHASSIS3_IS_RM_GROUP() 1u
#define CAN_TX_AXIS_CHASSIS3_LIMIT_CURRENT(current_) can_tx_axis_limit_i16((current_), 16000)

#define CAN_TX_AXIS_YAW_ACTUATOR_ID ACTUATOR_ID_YAW
#define CAN_TX_AXIS_YAW_FALLBACK_BUS 1u
#define CAN_TX_AXIS_YAW_NODE() (&s_can_tx_axis_yaw_node)
#define CAN_TX_AXIS_YAW_CAN_ID() 0x205u
#define CAN_TX_AXIS_YAW_IS_RM_GROUP() 1u
#define CAN_TX_AXIS_YAW_LIMIT_CURRENT(current_) can_tx_axis_limit_i16((current_), 30000)

#define CAN_TX_AXIS_YAW_UPPER_ACTUATOR_ID ACTUATOR_ID_YAW_UPPER
#define CAN_TX_AXIS_YAW_UPPER_FALLBACK_BUS 1u
#define CAN_TX_AXIS_YAW_UPPER_NODE() (&s_can_tx_axis_yaw_upper_node)
#define CAN_TX_AXIS_YAW_UPPER_CAN_ID() 0u
#define CAN_TX_AXIS_YAW_UPPER_IS_RM_GROUP() 1u
#define CAN_TX_AXIS_YAW_UPPER_LIMIT_CURRENT(current_) can_tx_axis_limit_i16((current_), 30000)

#define CAN_TX_AXIS_PITCH_ACTUATOR_ID ACTUATOR_ID_PITCH
#define CAN_TX_AXIS_PITCH_FALLBACK_BUS 1u
#define CAN_TX_AXIS_PITCH_NODE() (&s_can_tx_axis_pitch_node)
#define CAN_TX_AXIS_PITCH_CAN_ID() 0x206u
#define CAN_TX_AXIS_PITCH_IS_RM_GROUP() 1u
#define CAN_TX_AXIS_PITCH_LIMIT_CURRENT(current_) can_tx_axis_limit_i16((current_), 16000)

#define CAN_TX_AXIS_TRIGGER_ACTUATOR_ID ACTUATOR_ID_TRIGGER
#define CAN_TX_AXIS_TRIGGER_FALLBACK_BUS 1u
#define CAN_TX_AXIS_TRIGGER_NODE() (&s_can_tx_axis_trigger_node)
#define CAN_TX_AXIS_TRIGGER_CAN_ID() 0x207u
#define CAN_TX_AXIS_TRIGGER_IS_RM_GROUP() 1u
#define CAN_TX_AXIS_TRIGGER_LIMIT_CURRENT(current_) can_tx_axis_limit_i16((current_), 16000)

#define CAN_TX_AXIS_FRICTION0_ACTUATOR_ID ACTUATOR_ID_FRICTION0
#define CAN_TX_AXIS_FRICTION0_FALLBACK_BUS 2u
#define CAN_TX_AXIS_FRICTION0_NODE() (&s_can_tx_axis_friction0_node)
#define CAN_TX_AXIS_FRICTION0_CAN_ID() 0x201u
#define CAN_TX_AXIS_FRICTION0_IS_RM_GROUP() 1u
#define CAN_TX_AXIS_FRICTION0_LIMIT_CURRENT(current_) can_tx_axis_limit_i16((current_), 16000)

#define CAN_TX_AXIS_FRICTION1_ACTUATOR_ID ACTUATOR_ID_FRICTION1
#define CAN_TX_AXIS_FRICTION1_FALLBACK_BUS 2u
#define CAN_TX_AXIS_FRICTION1_NODE() (&s_can_tx_axis_friction1_node)
#define CAN_TX_AXIS_FRICTION1_CAN_ID() 0x202u
#define CAN_TX_AXIS_FRICTION1_IS_RM_GROUP() 1u
#define CAN_TX_AXIS_FRICTION1_LIMIT_CURRENT(current_) can_tx_axis_limit_i16((current_), 16000)

#define CAN_TX_AXIS_FRICTION2_ACTUATOR_ID ACTUATOR_ID_FRICTION2
#define CAN_TX_AXIS_FRICTION2_FALLBACK_BUS 2u
#define CAN_TX_AXIS_FRICTION2_NODE() (&s_can_tx_axis_friction2_node)
#define CAN_TX_AXIS_FRICTION2_CAN_ID() 0x203u
#define CAN_TX_AXIS_FRICTION2_IS_RM_GROUP() 1u
#define CAN_TX_AXIS_FRICTION2_LIMIT_CURRENT(current_) can_tx_axis_limit_i16((current_), 16000)

#define CAN_TX_AXIS_FRICTION3_ACTUATOR_ID ACTUATOR_ID_FRICTION3
#define CAN_TX_AXIS_FRICTION3_FALLBACK_BUS 2u
#define CAN_TX_AXIS_FRICTION3_NODE() (&s_can_tx_axis_friction3_node)
#define CAN_TX_AXIS_FRICTION3_CAN_ID() 0x204u
#define CAN_TX_AXIS_FRICTION3_IS_RM_GROUP() 1u
#define CAN_TX_AXIS_FRICTION3_LIMIT_CURRENT(current_) can_tx_axis_limit_i16((current_), 16000)

#endif
