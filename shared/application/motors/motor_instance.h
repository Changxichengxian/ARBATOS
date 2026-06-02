/*
 * SPDX-FileCopyrightText: 2026 闄堣僵 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef MOTOR_INSTANCE_H
#define MOTOR_INSTANCE_H

#include <stdint.h>

#include "actuator_cmd.h"
#include "CAN_receive.h"
#include "config.h"

struct control_controller;

#define MOTOR_INSTANCE_INVALID_DETECT_TOE 0xFFu

typedef enum
{
    MOTOR_INSTANCE_ROLE_CHASSIS = 0u,
    MOTOR_INSTANCE_ROLE_YAW,
    MOTOR_INSTANCE_ROLE_YAW_UPPER,
    MOTOR_INSTANCE_ROLE_PITCH,
    MOTOR_INSTANCE_ROLE_TRIGGER,
    MOTOR_INSTANCE_ROLE_FRICTION,
    MOTOR_INSTANCE_ROLE_ARM,
} motor_instance_role_e;

typedef struct
{
    actuator_id_e actuator_id;
    motor_instance_role_e role;
    uint8_t role_index;
    uint8_t fallback_bus;
    uint8_t detect_toe;
    uint8_t use_detect;
    const char *name;
    const motor_node_param_t *node;
    motor_measure_t *measure;
} motor_instance_t;

typedef struct
{
    actuator_id_e actuator_id;
    uint8_t enabled;
} motor_instance_current_binding_t;

/*
 * Usage budget:
 * - Init/config/diagnostics code may use name lookup and resolve helpers.
 * - High-rate control loops should use actuator_id or pre-bound current bindings.
 */
void motor_instance_refresh(void);
uint8_t motor_instance_count(void);
const motor_instance_t *motor_instance_get(uint8_t index);
const motor_instance_t *motor_instance_find_by_actuator(actuator_id_e id);
const motor_instance_t *motor_instance_find_by_name(const char *name);
const motor_instance_t *motor_instance_find_feedback(uint8_t bus, uint16_t std_id);
uint8_t motor_instance_resolve_actuator_ids(const char *const *names, uint8_t count, actuator_id_e *out, uint8_t out_cap);
uint8_t motor_instance_bind_current_outputs(const char *const *names,
                                            uint8_t count,
                                            motor_instance_current_binding_t *out,
                                            uint8_t out_cap);

const char *motor_instance_name(const motor_instance_t *inst);
actuator_id_e motor_instance_actuator_id(const motor_instance_t *inst);
actuator_id_e motor_instance_actuator_id_by_name(const char *name);
uint8_t motor_instance_bus(const motor_instance_t *inst);
uint8_t motor_instance_enabled(const motor_instance_t *inst);
const motor_node_param_t *motor_instance_node(actuator_id_e id);
motor_measure_t *motor_instance_measure(actuator_id_e id);
const motor_measure_t *motor_instance_measure_const(actuator_id_e id);
uint8_t motor_instance_model_id(actuator_id_e id);
uint8_t motor_instance_transport_id(actuator_id_e id);
uint8_t motor_instance_protocol_id(actuator_id_e id);
uint8_t motor_instance_control_mode_id(actuator_id_e id);
uint8_t motor_instance_cmd_caps_id(actuator_id_e id);
uint8_t motor_instance_cmd_mode_supported_id(actuator_id_e id, actuator_cmd_mode_e mode);

uint8_t motor_instance_cmd_clear_id(actuator_id_e id);
uint8_t motor_instance_cmd_set_ids(const actuator_id_e *ids, const actuator_cmd_t *cmds, uint8_t count);
uint8_t motor_instance_cmd_set_ids_best_effort(const actuator_id_e *ids, const actuator_cmd_t *cmds, uint8_t count);
uint8_t motor_instance_cmd_set_current_id(actuator_id_e id, int16_t current);
uint8_t motor_instance_cmd_set_state_torque_id(actuator_id_e id, const actuator_cmd_t *cmd);
uint8_t motor_instance_cmd_set_state_torque_ids(const actuator_id_e *ids, const actuator_cmd_t *cmds, uint8_t count);
uint8_t motor_instance_cmd_set_state_torque_ids_best_effort(const actuator_id_e *ids,
                                                            const actuator_cmd_t *cmds,
                                                            uint8_t count);
uint8_t motor_instance_cmd_set_speed_id(actuator_id_e id, fp32 velocity, fp32 kd, fp32 torque);
uint8_t motor_instance_cmd_clear(const char *name);
uint8_t motor_instance_cmd_set_current(const char *name, int16_t current);
uint8_t motor_instance_cmd_set_state_torque(const char *name, const actuator_cmd_t *cmd);
uint8_t motor_instance_cmd_set_speed(const char *name, fp32 velocity, fp32 kd, fp32 torque);
uint8_t motor_instance_cmd_get_copy(const char *name, actuator_cmd_t *out);
uint8_t motor_instance_feedback_get_copy(const char *name, actuator_feedback_t *out);
uint8_t motor_instance_cmd_set_current_ids(const actuator_id_e *ids, const int16_t *currents, uint8_t count);
uint8_t motor_instance_cmd_set_current_many(const char *const *names, const int16_t *currents, uint8_t count);
uint8_t motor_instance_cmd_set_current_ids_best_effort(const actuator_id_e *ids, const int16_t *currents, uint8_t count);
uint8_t motor_instance_cmd_set_current_many_best_effort(const char *const *names, const int16_t *currents, uint8_t count);
uint8_t motor_instance_cmd_set_current_bindings_best_effort(const motor_instance_current_binding_t *bindings,
                                                            const int16_t *currents,
                                                            uint8_t count);
uint8_t motor_instance_feedback_get_copy_ids(const actuator_id_e *ids, actuator_feedback_t *out, uint8_t count);
uint8_t motor_instance_feedback_get_copy_many(const char *const *names, actuator_feedback_t *out, uint8_t count);
uint8_t motor_instance_resolve_controller_outputs(const struct control_controller *controller,
                                                  actuator_id_e *out,
                                                  uint8_t out_cap);

#endif
