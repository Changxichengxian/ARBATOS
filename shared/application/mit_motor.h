/*
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 */

#ifndef MIT_MOTOR_H
#define MIT_MOTOR_H

#include <stdint.h>

#include "motor_model_db.h"

typedef motor_model_mit_limits_t mit_motor_limits_t;

typedef struct
{
    fp32 position;
    fp32 velocity;
    fp32 kp;
    fp32 kd;
    fp32 torque;
} mit_motor_cmd_t;

typedef struct
{
    uint8_t online;
    uint8_t rx_dlc;
    uint16_t rx_id;
    uint32_t rx_count;
    uint32_t last_rx_tick;
    fp32 position;
    fp32 velocity;
    fp32 torque;
} mit_motor_feedback_t;

#endif
