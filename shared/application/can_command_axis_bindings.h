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
#include "motor_config.h"

#define CAN_TX_AXIS_CHASSIS0_ACTUATOR_ID ACTUATOR_ID_CHASSIS0
#define CAN_TX_AXIS_CHASSIS0_FALLBACK_BUS 1u
#define CAN_TX_AXIS_CHASSIS0_NODE() (&g_config.motor.chassis[0])
#define CAN_TX_AXIS_CHASSIS0_CAN_ID() motor_cfg_can_id(CAN_TX_AXIS_CHASSIS0_NODE())
#define CAN_TX_AXIS_CHASSIS0_IS_RM_GROUP() motor_cfg_is_rm_group_protocol(CAN_TX_AXIS_CHASSIS0_NODE())
#define CAN_TX_AXIS_CHASSIS0_LIMIT_CURRENT(current_) motor_cfg_limit_current_node(CAN_TX_AXIS_CHASSIS0_NODE(), (current_))

#define CAN_TX_AXIS_CHASSIS1_ACTUATOR_ID ACTUATOR_ID_CHASSIS1
#define CAN_TX_AXIS_CHASSIS1_FALLBACK_BUS 1u
#define CAN_TX_AXIS_CHASSIS1_NODE() (&g_config.motor.chassis[1])
#define CAN_TX_AXIS_CHASSIS1_CAN_ID() motor_cfg_can_id(CAN_TX_AXIS_CHASSIS1_NODE())
#define CAN_TX_AXIS_CHASSIS1_IS_RM_GROUP() motor_cfg_is_rm_group_protocol(CAN_TX_AXIS_CHASSIS1_NODE())
#define CAN_TX_AXIS_CHASSIS1_LIMIT_CURRENT(current_) motor_cfg_limit_current_node(CAN_TX_AXIS_CHASSIS1_NODE(), (current_))

#define CAN_TX_AXIS_CHASSIS2_ACTUATOR_ID ACTUATOR_ID_CHASSIS2
#define CAN_TX_AXIS_CHASSIS2_FALLBACK_BUS 1u
#define CAN_TX_AXIS_CHASSIS2_NODE() (&g_config.motor.chassis[2])
#define CAN_TX_AXIS_CHASSIS2_CAN_ID() motor_cfg_can_id(CAN_TX_AXIS_CHASSIS2_NODE())
#define CAN_TX_AXIS_CHASSIS2_IS_RM_GROUP() motor_cfg_is_rm_group_protocol(CAN_TX_AXIS_CHASSIS2_NODE())
#define CAN_TX_AXIS_CHASSIS2_LIMIT_CURRENT(current_) motor_cfg_limit_current_node(CAN_TX_AXIS_CHASSIS2_NODE(), (current_))

#define CAN_TX_AXIS_CHASSIS3_ACTUATOR_ID ACTUATOR_ID_CHASSIS3
#define CAN_TX_AXIS_CHASSIS3_FALLBACK_BUS 1u
#define CAN_TX_AXIS_CHASSIS3_NODE() (&g_config.motor.chassis[3])
#define CAN_TX_AXIS_CHASSIS3_CAN_ID() motor_cfg_can_id(CAN_TX_AXIS_CHASSIS3_NODE())
#define CAN_TX_AXIS_CHASSIS3_IS_RM_GROUP() motor_cfg_is_rm_group_protocol(CAN_TX_AXIS_CHASSIS3_NODE())
#define CAN_TX_AXIS_CHASSIS3_LIMIT_CURRENT(current_) motor_cfg_limit_current_node(CAN_TX_AXIS_CHASSIS3_NODE(), (current_))

#define CAN_TX_AXIS_YAW_ACTUATOR_ID ACTUATOR_ID_YAW
#define CAN_TX_AXIS_YAW_FALLBACK_BUS 1u
#define CAN_TX_AXIS_YAW_NODE() (&g_config.motor.yaw)
#define CAN_TX_AXIS_YAW_CAN_ID() motor_cfg_can_id(CAN_TX_AXIS_YAW_NODE())
#define CAN_TX_AXIS_YAW_IS_RM_GROUP() motor_cfg_is_rm_group_protocol(CAN_TX_AXIS_YAW_NODE())
#define CAN_TX_AXIS_YAW_LIMIT_CURRENT(current_) motor_cfg_limit_current_node(CAN_TX_AXIS_YAW_NODE(), (current_))

#define CAN_TX_AXIS_YAW_UPPER_ACTUATOR_ID ACTUATOR_ID_YAW_UPPER
#define CAN_TX_AXIS_YAW_UPPER_FALLBACK_BUS 1u
#define CAN_TX_AXIS_YAW_UPPER_NODE() (&g_config.motor.yaw_upper)
#define CAN_TX_AXIS_YAW_UPPER_CAN_ID() motor_cfg_can_id(CAN_TX_AXIS_YAW_UPPER_NODE())
#define CAN_TX_AXIS_YAW_UPPER_IS_RM_GROUP() motor_cfg_is_rm_group_protocol(CAN_TX_AXIS_YAW_UPPER_NODE())
#define CAN_TX_AXIS_YAW_UPPER_LIMIT_CURRENT(current_) motor_cfg_limit_current_node(CAN_TX_AXIS_YAW_UPPER_NODE(), (current_))

#define CAN_TX_AXIS_PITCH_ACTUATOR_ID ACTUATOR_ID_PITCH
#define CAN_TX_AXIS_PITCH_FALLBACK_BUS 1u
#define CAN_TX_AXIS_PITCH_NODE() (&g_config.motor.pitch)
#define CAN_TX_AXIS_PITCH_CAN_ID() motor_cfg_can_id(CAN_TX_AXIS_PITCH_NODE())
#define CAN_TX_AXIS_PITCH_IS_RM_GROUP() motor_cfg_is_rm_group_protocol(CAN_TX_AXIS_PITCH_NODE())
#define CAN_TX_AXIS_PITCH_LIMIT_CURRENT(current_) motor_cfg_limit_current_node(CAN_TX_AXIS_PITCH_NODE(), (current_))

#define CAN_TX_AXIS_TRIGGER_ACTUATOR_ID ACTUATOR_ID_TRIGGER
#define CAN_TX_AXIS_TRIGGER_FALLBACK_BUS 1u
#define CAN_TX_AXIS_TRIGGER_NODE() (&g_config.motor.trigger)
#define CAN_TX_AXIS_TRIGGER_CAN_ID() motor_cfg_can_id(CAN_TX_AXIS_TRIGGER_NODE())
#define CAN_TX_AXIS_TRIGGER_IS_RM_GROUP() motor_cfg_is_rm_group_protocol(CAN_TX_AXIS_TRIGGER_NODE())
#define CAN_TX_AXIS_TRIGGER_LIMIT_CURRENT(current_) motor_cfg_limit_current_node(CAN_TX_AXIS_TRIGGER_NODE(), (current_))

#define CAN_TX_AXIS_FRICTION0_ACTUATOR_ID ACTUATOR_ID_FRICTION0
#define CAN_TX_AXIS_FRICTION0_FALLBACK_BUS 2u
#define CAN_TX_AXIS_FRICTION0_NODE() (&g_config.motor.friction[0])
#define CAN_TX_AXIS_FRICTION0_CAN_ID() motor_cfg_can_id(CAN_TX_AXIS_FRICTION0_NODE())
#define CAN_TX_AXIS_FRICTION0_IS_RM_GROUP() motor_cfg_is_rm_group_protocol(CAN_TX_AXIS_FRICTION0_NODE())
#define CAN_TX_AXIS_FRICTION0_LIMIT_CURRENT(current_) motor_cfg_limit_current_node(CAN_TX_AXIS_FRICTION0_NODE(), (current_))

#define CAN_TX_AXIS_FRICTION1_ACTUATOR_ID ACTUATOR_ID_FRICTION1
#define CAN_TX_AXIS_FRICTION1_FALLBACK_BUS 2u
#define CAN_TX_AXIS_FRICTION1_NODE() (&g_config.motor.friction[1])
#define CAN_TX_AXIS_FRICTION1_CAN_ID() motor_cfg_can_id(CAN_TX_AXIS_FRICTION1_NODE())
#define CAN_TX_AXIS_FRICTION1_IS_RM_GROUP() motor_cfg_is_rm_group_protocol(CAN_TX_AXIS_FRICTION1_NODE())
#define CAN_TX_AXIS_FRICTION1_LIMIT_CURRENT(current_) motor_cfg_limit_current_node(CAN_TX_AXIS_FRICTION1_NODE(), (current_))

#define CAN_TX_AXIS_FRICTION2_ACTUATOR_ID ACTUATOR_ID_FRICTION2
#define CAN_TX_AXIS_FRICTION2_FALLBACK_BUS 2u
#define CAN_TX_AXIS_FRICTION2_NODE() (&g_config.motor.friction[2])
#define CAN_TX_AXIS_FRICTION2_CAN_ID() motor_cfg_can_id(CAN_TX_AXIS_FRICTION2_NODE())
#define CAN_TX_AXIS_FRICTION2_IS_RM_GROUP() motor_cfg_is_rm_group_protocol(CAN_TX_AXIS_FRICTION2_NODE())
#define CAN_TX_AXIS_FRICTION2_LIMIT_CURRENT(current_) motor_cfg_limit_current_node(CAN_TX_AXIS_FRICTION2_NODE(), (current_))

#define CAN_TX_AXIS_FRICTION3_ACTUATOR_ID ACTUATOR_ID_FRICTION3
#define CAN_TX_AXIS_FRICTION3_FALLBACK_BUS 2u
#define CAN_TX_AXIS_FRICTION3_NODE() (&g_config.motor.friction[3])
#define CAN_TX_AXIS_FRICTION3_CAN_ID() motor_cfg_can_id(CAN_TX_AXIS_FRICTION3_NODE())
#define CAN_TX_AXIS_FRICTION3_IS_RM_GROUP() motor_cfg_is_rm_group_protocol(CAN_TX_AXIS_FRICTION3_NODE())
#define CAN_TX_AXIS_FRICTION3_LIMIT_CURRENT(current_) motor_cfg_limit_current_node(CAN_TX_AXIS_FRICTION3_NODE(), (current_))

#define CAN_TX_AXIS_ARM_FALLBACK_BUS(index_) (((index_) == 0u) ? 1u : 2u)
#define CAN_TX_AXIS_ARM_ACTUATOR_ID(index_) actuator_id_arm_joint((index_))
#define CAN_TX_AXIS_ARM_NODE(index_) (&g_config.motor.arm[(index_)])
#define CAN_TX_AXIS_ARM_CAN_ID(index_) motor_cfg_can_id(CAN_TX_AXIS_ARM_NODE((index_)))
#define CAN_TX_AXIS_ARM_IS_RM_GROUP(index_) motor_cfg_is_rm_group_protocol(CAN_TX_AXIS_ARM_NODE((index_)))

#endif
