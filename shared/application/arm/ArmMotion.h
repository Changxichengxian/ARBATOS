#pragma once

#include <stdint.h>

#include "ArmTask.h"

void ArmMotionInit(void);
void ArmMotionStepManual(uint16_t key_mask);
const ArmMotorFeedback *ArmMotionGetFeedback(uint8_t index);
uint8_t ArmMotionProcessCanFeedback(uint8_t bus, uint16_t std_id, uint8_t dlc, const uint8_t data[8]);
const ArmJ0UnitreeState *ArmMotionGetJ0UnitreeState(void);
