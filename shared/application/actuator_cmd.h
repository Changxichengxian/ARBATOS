/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef ACTUATOR_CMD_H
#define ACTUATOR_CMD_H

#include <stdint.h>

#include "struct_typedef.h"

typedef enum
{
    ACTUATOR_ID_CHASSIS0 = 0u,
    ACTUATOR_ID_CHASSIS1,
    ACTUATOR_ID_CHASSIS2,
    ACTUATOR_ID_CHASSIS3,
    ACTUATOR_ID_YAW,
    ACTUATOR_ID_YAW_UPPER,
    ACTUATOR_ID_PITCH,
    ACTUATOR_ID_TRIGGER,
    ACTUATOR_ID_FRICTION0,
    ACTUATOR_ID_FRICTION1,
    ACTUATOR_ID_FRICTION2,
    ACTUATOR_ID_FRICTION3,
    ACTUATOR_ID_ARM_J0,
    ACTUATOR_ID_ARM_J1,
    ACTUATOR_ID_ARM_J2,
    ACTUATOR_ID_ARM_J3,
    ACTUATOR_ID_ARM_J4,
    ACTUATOR_ID_ARM_J5,
    ACTUATOR_ID__COUNT
} actuator_id_e;

typedef enum
{
    ACTUATOR_CMD_MODE_NONE = 0u,
    ACTUATOR_CMD_MODE_CURRENT,
    ACTUATOR_CMD_MODE_STATE_TORQUE, // position/velocity/kp/kd/torque, transport-independent
    ACTUATOR_CMD_MODE_POS_VEL,
    ACTUATOR_CMD_MODE_SPEED,
    ACTUATOR_CMD_MODE_FORCE_POS,
} actuator_cmd_mode_e;

typedef enum
{
    ACTUATOR_TRANSPORT_NONE = 0u,
    ACTUATOR_TRANSPORT_CAN,
    ACTUATOR_TRANSPORT_RS485,
} actuator_transport_e;

typedef struct
{
    uint8_t active;
    uint8_t mode; // actuator_cmd_mode_e
    int16_t current;
    fp32 position;
    fp32 velocity;
    fp32 kp;
    fp32 kd;
    fp32 torque;
} actuator_cmd_t;

typedef struct
{
    uint8_t online;
    uint8_t bus;
    uint8_t rx_dlc;
    uint8_t transport; // actuator_transport_e
    uint16_t rx_id;
    uint32_t rx_count;
    uint32_t last_rx_tick;
    fp32 position;
    fp32 velocity;
    fp32 torque;
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t current;
    uint8_t temperature;
} actuator_feedback_t;

actuator_id_e actuator_id_chassis(uint8_t index);
actuator_id_e actuator_id_friction(uint8_t index);
actuator_id_e actuator_id_arm_joint(uint8_t index);

void actuator_cmd_clear_all(void);
void actuator_cmd_set_current(actuator_id_e id, int16_t current);
int16_t actuator_cmd_get_current(actuator_id_e id);
void actuator_cmd_set_state_torque(actuator_id_e id, const actuator_cmd_t *cmd);
void actuator_cmd_set_speed(actuator_id_e id, fp32 velocity, fp32 kd, fp32 torque);
uint8_t actuator_cmd_get_copy(actuator_id_e id, actuator_cmd_t *out);
const actuator_cmd_t *actuator_cmd_get_ptr(actuator_id_e id);

void actuator_feedback_clear_all(void);
void actuator_feedback_update(actuator_id_e id, const actuator_feedback_t *feedback);
uint8_t actuator_feedback_get_copy(actuator_id_e id, actuator_feedback_t *out);
const actuator_feedback_t *actuator_feedback_get_ptr(actuator_id_e id);

// Role-based current commands. Control tasks should prefer these names.
void actuator_cmd_set_chassis_current(uint8_t index, int16_t current);
int16_t actuator_cmd_get_chassis_current(uint8_t index);
void actuator_cmd_set_yaw_current(int16_t current);
int16_t actuator_cmd_get_yaw_current(void);
void actuator_cmd_set_yaw_upper_current(int16_t current);
int16_t actuator_cmd_get_yaw_upper_current(void);
void actuator_cmd_set_pitch_current(int16_t current);
int16_t actuator_cmd_get_pitch_current(void);
void actuator_cmd_set_trigger_current(int16_t current);
int16_t actuator_cmd_get_trigger_current(void);
void actuator_cmd_set_friction_current(uint8_t index, int16_t current);
int16_t actuator_cmd_get_friction_current(uint8_t index);

// Compatibility wrappers for older call sites.
void actuator_cmd_set_chassis_current_can1(uint8_t index, int16_t current);
int16_t actuator_cmd_get_chassis_current_can1(uint8_t index);

void actuator_cmd_set_yaw_current_can1(int16_t current);
int16_t actuator_cmd_get_yaw_current_can1(void);

void actuator_cmd_set_yaw_upper_current_can1(int16_t current);
int16_t actuator_cmd_get_yaw_upper_current_can1(void);

void actuator_cmd_set_pitch_current_can1(int16_t current);
int16_t actuator_cmd_get_pitch_current_can1(void);

void actuator_cmd_set_trigger_current_can1(int16_t current);
int16_t actuator_cmd_get_trigger_current_can1(void);

void actuator_cmd_set_friction_current_can2(uint8_t index, int16_t current);
int16_t actuator_cmd_get_friction_current_can2(uint8_t index);

#endif
