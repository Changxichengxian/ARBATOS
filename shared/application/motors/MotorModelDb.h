/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef MOTOR_MODEL_DB_H
#define MOTOR_MODEL_DB_H

#include <stdint.h>

#include "RobotConfig.h"

typedef enum
{
    MOTOR_MODEL_RX_FMT_NONE = 0,
    MOTOR_MODEL_RX_FMT_RM_STD,
    MOTOR_MODEL_RX_FMT_820R,
    MOTOR_MODEL_RX_FMT_6623,
    MOTOR_MODEL_RX_FMT__COUNT
} MotorModelRxFormat;

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
} MotorModelCap;

#define MOTOR_MODEL_RX_OFF_NONE 0xFFu

typedef enum
{
    MOTOR_MODEL_RX_CUR_SET_NONE = 0u,
    MOTOR_MODEL_RX_CUR_SET_SAME_AS_MEAS,
    MOTOR_MODEL_RX_CUR_SET_FROM_FRAME,
} MotorModelRxCurrentSetPolicy;

typedef struct
{
    uint8_t speed_rpm_off;
    uint8_t current_meas_off;
    uint8_t current_set_off;
    uint8_t temp_off;
    uint8_t current_set_policy;
} MotorModelRxDesc;

typedef struct
{
    // MIT frame scaling limits. Keep these matched with PMAX/VMAX/TMAX in the driver,
    // not with the motor's mechanical peak capability.
    fp32 position_max;
    fp32 velocity_max;
    fp32 kp_max;
    fp32 kd_max;
    fp32 torque_max;
} MotorModelMitLimits;

typedef struct
{
    uint8_t dm_driver_generation; // 0 when not a Damiao generation-marked model.
    uint8_t encoder_bits;
    fp32 rated_torque_nm;
    fp32 peak_torque_nm;
    fp32 rated_speed_rpm;
    fp32 max_no_load_speed_rpm_24v;
    fp32 max_no_load_speed_rpm_48v;
    fp32 rated_current_a; // Legacy manuals do not split phase/supply current.
    fp32 peak_current_a;
    fp32 rated_phase_current_a_24v;
    fp32 rated_supply_current_a_24v;
    fp32 peak_phase_current_a_24v;
    fp32 peak_supply_current_a_24v;
    fp32 rated_phase_current_a_48v;
    fp32 rated_supply_current_a_48v;
    fp32 peak_phase_current_a_48v;
    fp32 peak_supply_current_a_48v;
    fp32 phase_inductance_uh;
    fp32 phase_resistance_mohm;
    fp32 weight_g;
} MotorModelSpecs;

typedef struct
{
    MotorModelParam base;
    MotorModelRxFormat rx_format;
    uint8_t default_protocol;
    uint8_t default_control_mode;
    uint8_t caps;
    MotorModelSpecs specs;
    MotorModelMitLimits mit_limits;
    int16_t cmd_current_range_abs;
    fp32 torque_current_range_a;
    int16_t fb_current_meas_range_abs;
} MotorModelDbEntry;

const MotorModelDbEntry *MotorModelDbGet(MotorModel model);
const MotorModelRxDesc *MotorModelDbGetRxDesc(MotorModel model);

#endif
