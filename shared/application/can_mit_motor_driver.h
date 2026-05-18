#pragma once

#include <stdint.h>

#include "mit_motor.h"

typedef mit_motor_limits_t can_mit_motor_limits_t;
typedef mit_motor_cmd_t can_mit_motor_cmd_t;
typedef mit_motor_feedback_t can_mit_motor_feedback_t;

int can_mit_motor_send_cmd(uint8_t bus,
                           uint16_t std_id,
                           const can_mit_motor_limits_t *limits,
                           const can_mit_motor_cmd_t *cmd);
int can_mit_motor_send_enable(uint8_t bus, uint16_t std_id);
int can_mit_motor_send_disable(uint8_t bus, uint16_t std_id);
int can_mit_motor_send_stop(uint8_t bus,
                            uint16_t std_id,
                            const can_mit_motor_limits_t *limits);
uint8_t can_mit_motor_update_feedback(uint16_t std_id,
                                      uint8_t motor_id,
                                      const can_mit_motor_limits_t *limits,
                                      uint8_t dlc,
                                      const uint8_t data[8],
                                      can_mit_motor_feedback_t *feedback);
