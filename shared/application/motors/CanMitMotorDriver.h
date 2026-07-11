#pragma once

#include <stdint.h>

#include "MitMotor.h"

struct BspCanTxTicket;

typedef mit_motor_limits_t CanMitMotorLimits;
typedef mit_motor_cmd_t CanMitMotorCmd;
typedef mit_motor_feedback_t CanMitMotorFeedback;

int CanMitMotorSendCmd(uint8_t bus,
                           uint16_t std_id,
                           const CanMitMotorLimits *limits,
                           const CanMitMotorCmd *cmd);
int CanMitMotorSendCmdTracked(uint8_t bus,
                              uint16_t std_id,
                              const CanMitMotorLimits *limits,
                              const CanMitMotorCmd *cmd,
                              const struct BspCanTxTicket *ticket,
                              uint8_t *tracked);
int CanMitMotorSendEnable(uint8_t bus, uint16_t std_id);
int CanMitMotorSendDisable(uint8_t bus, uint16_t std_id);
int CanMitMotorSendStop(uint8_t bus,
                            uint16_t std_id,
                            const CanMitMotorLimits *limits);
uint8_t CanMitMotorUpdateFeedback(uint16_t std_id,
                                      uint8_t motor_id,
                                      const CanMitMotorLimits *limits,
                                      uint8_t dlc,
                                      const uint8_t data[8],
                                      CanMitMotorFeedback *feedback);
