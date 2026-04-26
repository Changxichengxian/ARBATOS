#pragma once

#include <stdint.h>

#include "struct_typedef.h"

#define UNITREE_MOTOR_RS485_PORT0 0u
#define UNITREE_MOTOR_RS485_PORT1 1u

typedef struct
{
    uint8_t enable;
    uint8_t rs485_port;
    uint8_t motor_id;
    uint32_t baudrate;
    uint16_t rx_timeout_ms;
} unitree_motor_config_t;

typedef struct
{
    fp32 torque_nm;
    fp32 speed_rad_s;
    fp32 position_rad;
    fp32 kp;
    fp32 kd;
} unitree_motor_cmd_t;

typedef struct
{
    uint8_t enabled;
    uint8_t rs485_port;
    uint8_t motor_id;
    uint8_t online;
    uint8_t last_mode;
    uint8_t motor_error;
    int8_t motor_temp;
    uint8_t last_tx_status;
    uint32_t tx_count;
    uint32_t tx_fail_count;
    uint32_t rx_frame_count;
    uint32_t rx_crc_fail_count;
    uint32_t rx_parse_error_count;
    uint32_t last_rx_tick_ms;
    fp32 cmd_speed_rad_s;
    fp32 cmd_kd;
    fp32 torque_nm;
    fp32 joint_speed_rad_s;
    fp32 joint_position_rad;
} unitree_motor_state_t;

void unitree_motor_driver_init(void);
void unitree_motor_refresh(const unitree_motor_config_t *cfg);
uint8_t unitree_motor_configure(const unitree_motor_config_t *cfg);
int unitree_motor_send_cmd(const unitree_motor_config_t *cfg, const unitree_motor_cmd_t *cmd);
void unitree_motor_stop(void);
const unitree_motor_state_t *unitree_motor_get_state(void);
