#ifndef ROBOT_LIFECYCLE_TEST_CONTROL_INPUT_H
#define ROBOT_LIFECYCLE_TEST_CONTROL_INPUT_H

#include <stdint.h>

typedef enum
{
    INPUT_SW_GIMBAL_MODE = 0,
    INPUT_SW_COUNT
} input_switch_e;

typedef struct
{
    uint16_t sw[INPUT_SW_COUNT];
} ControlInputState;

uint8_t ControlInputSwitchIsPos(uint16_t raw, uint8_t pos);

#endif
