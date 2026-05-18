#pragma once

#include <stdint.h>

#include "arm_task.h"

void arm_motion_init(void);
void arm_motion_step_manual(uint16_t key_mask);
const arm_motor_feedback_t *arm_motion_get_feedback(uint8_t index);
uint8_t arm_motion_process_can_feedback(uint8_t bus, uint16_t std_id, uint8_t dlc, const uint8_t data[8]);
const arm_j0_unitree_state_t *arm_motion_get_j0_unitree_state(void);
