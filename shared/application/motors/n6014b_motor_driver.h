#pragma once

#include <stdint.h>

#include "LowCmd.h"
#include "config.h"
#include "types.h"

#define N6014B_MOTOR_RS485_PORT0 0u
#define N6014B_MOTOR_RS485_PORT1 1u
#define N6014B_MOTOR_DEFAULT_BAUDRATE 6000000u
#define N6014B_MOTOR_DEFAULT_RX_TIMEOUT_MS 50u

typedef struct
{
    uint8_t enabled;
    uint8_t online;
    uint8_t rs485_port;
    uint8_t motor_id;
    uint8_t mode;
    uint8_t timeout;
    int8_t motor_temp;
    uint8_t coil_temp;
    uint8_t last_tx_status;
    uint32_t motor_error;
    uint16_t motor_warn;
    uint32_t tx_count;
    uint32_t tx_fail_count;
    uint32_t rx_frame_count;
    uint32_t rx_crc_fail_count;
    uint32_t rx_parse_error_count;
    uint32_t last_rx_tick_ms;
    fp32 voltage;
    fp32 position_rad;
    fp32 speed_rad_s;
    fp32 torque_nm;
} n6014b_motor_state_t;

void n6014b_motor_driver_init(void);
int n6014b_motor_send_actuator(uint8_t port,
                               MotorId actuator_id,
                               const motor_node_param_t *node,
                               int16_t current);
const n6014b_motor_state_t *n6014b_motor_get_state(MotorId actuator_id);
