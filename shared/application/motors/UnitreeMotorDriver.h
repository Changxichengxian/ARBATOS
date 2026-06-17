#pragma once

#include <stdint.h>

#include "LowCmd.h"
#include "RobotConfigTypes.h"
#include "types.h"

#define UNITREE_MOTOR_RS485_PORT0 0u
#define UNITREE_MOTOR_RS485_PORT1 1u

typedef struct
{
    uint8_t enable;
    uint8_t rs485_port;
    uint8_t motor_id;
    uint32_t baudrate;
    uint16_t rx_timeout_ms;
} UnitreeMotorConfig;

typedef struct
{
    fp32 torque_nm;
    fp32 speed_rad_s;
    fp32 position_rad;
    fp32 kp;
    fp32 kd;
} UnitreeMotorCmd;

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
} UnitreeMotorState;

void UnitreeMotorDriverInit(void);
void UnitreeMotorRefresh(const UnitreeMotorConfig *cfg);
uint8_t UnitreeMotorConfigure(const UnitreeMotorConfig *cfg);
int UnitreeMotorSendCmd(const UnitreeMotorConfig *cfg, const UnitreeMotorCmd *cmd);
uint8_t UnitreeMotorNodeSupported(const motor_node_param_t *node);
int UnitreeMotorSendActuator(uint8_t port, MotorId actuator_id, const motor_node_param_t *node, int16_t current);
void UnitreeMotorStop(void);
const UnitreeMotorState *UnitreeMotorGetState(void);
