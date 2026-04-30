/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef MOTOR_MODEL_DB_H
#define MOTOR_MODEL_DB_H

#include <stdint.h>

#include "config.h"

typedef enum
{
    MOTOR_MODEL_RX_FMT_NONE = 0,
    MOTOR_MODEL_RX_FMT_RM_STD,
    MOTOR_MODEL_RX_FMT_820R,
    MOTOR_MODEL_RX_FMT_6623,
    MOTOR_MODEL_RX_FMT__COUNT
} motor_model_rx_format_e;

typedef enum
{
    MOTOR_MODEL_CAP_MIT = 1u << 0,
    MOTOR_MODEL_CAP_POS_VEL = 1u << 1,
    MOTOR_MODEL_CAP_SPEED = 1u << 2,
    MOTOR_MODEL_CAP_FORCE_POS = 1u << 3,
    MOTOR_MODEL_CAP_CAN_PARAM_RW = 1u << 4,
    MOTOR_MODEL_CAP_CAN_SAVE = 1u << 5,
    MOTOR_MODEL_CAP_CAN_BAUD_RW = 1u << 6,
    MOTOR_MODEL_CAP_ENABLE_CMD = 1u << 7,
} motor_model_cap_e;

#define MOTOR_MODEL_RX_OFF_NONE 0xFFu

typedef enum
{
    MOTOR_MODEL_RX_CUR_SET_NONE = 0u,
    MOTOR_MODEL_RX_CUR_SET_SAME_AS_MEAS,
    MOTOR_MODEL_RX_CUR_SET_FROM_FRAME,
} motor_model_rx_current_set_policy_e;

typedef struct
{
    uint8_t speed_rpm_off;
    uint8_t current_meas_off;
    uint8_t current_set_off;
    uint8_t temp_off;
    uint8_t current_set_policy;
} motor_model_rx_desc_t;

typedef struct
{
    motor_model_param_t base;
    motor_model_rx_format_e rx_format;
    uint8_t default_protocol;
    uint8_t default_control_mode;
    uint8_t caps;
    int16_t cmd_current_range_abs;
    fp32 torque_current_range_a;
    int16_t fb_current_meas_range_abs;
} motor_model_db_entry_t;

const motor_model_db_entry_t *motor_model_db_get(motor_model_e model);
const motor_model_rx_desc_t *motor_model_db_get_rx_desc(motor_model_e model);

#endif
