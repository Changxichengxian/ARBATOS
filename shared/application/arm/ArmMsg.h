/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef ARM_MSG_H
#define ARM_MSG_H

#include "RobotMsg.h"
#include "StateStore.h"
#include "Types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ARM_JOINT_COUNT 6u

typedef enum
{
    ARM_MODE_DISABLED = 0,
    ARM_MODE_MANUAL,
    ARM_MODE_HOLD,
    ARM_MODE_FAULT,
} ArmMode;

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
} ArmMotorFeedback;

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
    fp32 cmd_output_speed_rad_s;
    fp32 cmd_output_kd;
    fp32 torque_nm;
    fp32 joint_speed_rad_s;
    fp32 joint_position_rad;
} ArmJ0UnitreeState;

typedef struct
{
    msg_header_t header;
    uint8_t mode; // ArmMode
    uint8_t enabled;
    uint16_t key_mask;
    fp32 key_speed_scale;
    fp32 key_kd;
    int16_t j0_current;
} ArmCmd;

typedef struct
{
    msg_header_t header;
    uint8_t mode; // ArmMode
    uint8_t active_joint_count;
    uint16_t key_mask;
    uint8_t deadman_hold_ctrl;
    uint8_t reserved[3];
    fp32 key_speed_scale;
    fp32 key_kd;
    int16_t j0_current;
    ArmMotorFeedback motor[ARM_JOINT_COUNT];
    ArmJ0UnitreeState j0_unitree;
} ArmStatus;

typedef char ArmStatusFitsStore[(sizeof(ArmStatus) <= STATE_STORE_MAX_BYTES) ? 1 : -1];

static inline uint8_t ArmStatusWrite(const ArmStatus *status)
{
    return StateStoreWrite(STATE_ARM_STATUS, status, (uint16_t)sizeof(*status));
}

static inline uint8_t ArmStatusRead(ArmStatus *out)
{
    return StateStoreRead(STATE_ARM_STATUS, out, (uint16_t)sizeof(*out));
}

#ifdef __cplusplus
}
#endif

#endif
