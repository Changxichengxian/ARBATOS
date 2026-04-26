#pragma once

#include <stdint.h>

#include "motor_model_db.h"

typedef motor_model_mit_limits_t can_mit_motor_limits_t;

typedef struct
{
    fp32 position;
    fp32 velocity;
    fp32 kp;
    fp32 kd;
    fp32 torque;
} can_mit_motor_cmd_t;

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
} can_mit_motor_feedback_t;

void can_mit_motor_send_cmd(uint8_t bus,
                            uint16_t std_id,
                            const can_mit_motor_limits_t *limits,
                            const can_mit_motor_cmd_t *cmd);
void can_mit_motor_send_enable(uint8_t bus, uint16_t std_id);
void can_mit_motor_send_stop(uint8_t bus,
                             uint16_t std_id,
                             const can_mit_motor_limits_t *limits);
uint8_t can_mit_motor_update_feedback(uint16_t std_id,
                                      uint8_t motor_id,
                                      const can_mit_motor_limits_t *limits,
                                      uint8_t dlc,
                                      const uint8_t data[8],
                                      can_mit_motor_feedback_t *feedback);
