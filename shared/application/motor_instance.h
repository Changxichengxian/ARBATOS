/*
 * SPDX-FileCopyrightText: 2026 闄堣僵 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 闄堣僵 <2811158416@qq.com>
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
    const motor_node_param_t *node;
    motor_measure_t *measure;
} motor_instance_t;

void motor_instance_refresh(void);
uint8_t motor_instance_count(void);
const motor_instance_t *motor_instance_get(uint8_t index);
const motor_instance_t *motor_instance_find_by_actuator(actuator_id_e id);
const motor_instance_t *motor_instance_find_feedback(uint8_t bus, uint16_t std_id);

uint8_t motor_instance_bus(const motor_instance_t *inst);
uint8_t motor_instance_enabled(const motor_instance_t *inst);
const motor_node_param_t *motor_instance_node(actuator_id_e id);
motor_measure_t *motor_instance_measure(actuator_id_e id);
const motor_measure_t *motor_instance_measure_const(actuator_id_e id);

#endif
