/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef MOTOR_INSTANCE_H
#define MOTOR_INSTANCE_H

#include <stdint.h>

#include "LowCmd.h"
#include "CAN_receive.h"
#include "config.h"
#include "motor_model_db.h"

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
    MotorId actuator_id;
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
    MotorId motorId;
    motor_instance_role_e role;
    uint8_t roleIndex;
    uint8_t fallbackBus;
    uint8_t bus;
    uint8_t enabled;
    uint8_t transport;
    uint8_t protocol;
    uint8_t controlMode;
    uint8_t isRmGroup;
    uint8_t cmdCaps;
    uint8_t model;
    uint16_t canId;
    uint16_t feedbackId;
    const char *name;
    const motor_node_param_t *node;
    motor_measure_t *measure;
    const motor_model_mit_limits_t *mitLimits;
} motor_route_t;

typedef struct
{
    MotorId actuator_id;
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
const motor_instance_t *motor_instance_find_by_actuator(MotorId id);
const motor_instance_t *motor_instance_find_by_name(const char *name);
const motor_instance_t *motor_instance_find_feedback(uint8_t bus, uint16_t std_id);
uint8_t motor_route_count(void);
const motor_route_t *motor_route_get(uint8_t index);
const motor_route_t *motor_route_find_by_motor(MotorId id);
uint8_t motor_instance_resolve_actuator_ids(const char *const *names, uint8_t count, MotorId *out, uint8_t out_cap);
uint8_t motor_instance_bind_current_outputs(const char *const *names,
                                            uint8_t count,
                                            motor_instance_current_binding_t *out,
                                            uint8_t out_cap);

const char *motor_instance_name(const motor_instance_t *inst);
MotorId motor_instance_actuator_id(const motor_instance_t *inst);
MotorId motor_instance_actuator_id_by_name(const char *name);
uint8_t motor_instance_bus(const motor_instance_t *inst);
uint8_t motor_instance_enabled(const motor_instance_t *inst);
const motor_node_param_t *motor_instance_node(MotorId id);
motor_measure_t *motor_instance_measure(MotorId id);
const motor_measure_t *motor_instance_measure_const(MotorId id);
uint8_t motor_instance_model_id(MotorId id);
uint8_t motor_instance_transport_id(MotorId id);
uint8_t motor_instance_protocol_id(MotorId id);
uint8_t motor_instance_control_mode_id(MotorId id);
uint8_t motor_instance_cmd_caps_id(MotorId id);
uint8_t motor_instance_cmd_mode_supported_id(MotorId id, MotorMode mode);

uint8_t motor_instance_cmd_clear_id(MotorId id);
uint8_t motor_instance_cmd_set_ids(const MotorId *ids, const MotorCmd *cmds, uint8_t count);
uint8_t motor_instance_cmd_set_ids_best_effort(const MotorId *ids, const MotorCmd *cmds, uint8_t count);
uint8_t motor_instance_cmd_set_current_id(MotorId id, int16_t current);
uint8_t motor_instance_cmd_set_state_torque_id(MotorId id, const MotorCmd *cmd);
uint8_t motor_instance_cmd_set_state_torque_ids(const MotorId *ids, const MotorCmd *cmds, uint8_t count);
uint8_t motor_instance_cmd_set_state_torque_ids_best_effort(const MotorId *ids,
                                                            const MotorCmd *cmds,
                                                            uint8_t count);
uint8_t motor_instance_cmd_set_disable_id(MotorId id);
uint8_t motor_instance_cmd_set_damping_id(MotorId id, fp32 kd, fp32 tau);
uint8_t motor_instance_cmd_set_speed_id(MotorId id, fp32 velocity, fp32 kd, fp32 torque);
uint8_t motor_instance_cmd_clear(const char *name);
uint8_t motor_instance_cmd_set_disable(const char *name);
uint8_t motor_instance_cmd_set_damping(const char *name, fp32 kd, fp32 tau);
uint8_t motor_instance_cmd_set_current(const char *name, int16_t current);
uint8_t motor_instance_cmd_set_state_torque(const char *name, const MotorCmd *cmd);
uint8_t motor_instance_cmd_set_speed(const char *name, fp32 velocity, fp32 kd, fp32 torque);
uint8_t motor_instance_cmd_get_copy(const char *name, MotorCmd *out);
uint8_t motor_instance_feedback_get_copy(const char *name, MotorState *out);
uint8_t motor_instance_cmd_set_current_ids(const MotorId *ids, const int16_t *currents, uint8_t count);
uint8_t motor_instance_cmd_set_current_many(const char *const *names, const int16_t *currents, uint8_t count);
uint8_t motor_instance_cmd_set_current_ids_best_effort(const MotorId *ids, const int16_t *currents, uint8_t count);
uint8_t motor_instance_cmd_set_current_many_best_effort(const char *const *names, const int16_t *currents, uint8_t count);
uint8_t motor_instance_cmd_set_current_bindings_best_effort(const motor_instance_current_binding_t *bindings,
                                                            const int16_t *currents,
                                                            uint8_t count);
uint8_t motor_instance_feedback_get_copy_ids(const MotorId *ids, MotorState *out, uint8_t count);
uint8_t motor_instance_feedback_get_copy_many(const char *const *names, MotorState *out, uint8_t count);
uint8_t motor_instance_resolve_controller_outputs(const struct control_controller *controller,
                                                  MotorId *out,
                                                  uint8_t out_cap);

#endif
