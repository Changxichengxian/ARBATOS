/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 Xie Yuhan <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef WHEELLEG_MSG_H
#define WHEELLEG_MSG_H

#include "robot_msg.h"
#include "state_store.h"
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define WHEELLEG_SIDE_COUNT 2u
#define WHEELLEG_LQR_STATE_DIM 6u
#define WHEELLEG_LQR_OUTPUT_DIM 4u

typedef enum
{
    WHEELLEG_SIDE_LEFT = 0,
    WHEELLEG_SIDE_RIGHT = 1,
} wheelleg_side_e;

typedef enum
{
    WHEELLEG_MODE_DISABLED = 0,
    WHEELLEG_MODE_CALIBRATION,
    WHEELLEG_MODE_STANDUP,
    WHEELLEG_MODE_BALANCE,
    WHEELLEG_MODE_JUMP,
    WHEELLEG_MODE_AIRBORNE,
    WHEELLEG_MODE_LAND,
    WHEELLEG_MODE_RECOVERY,
    WHEELLEG_MODE_FAULT,
    WHEELLEG_MODE_BENCH = 9, // 板凳模型：关节锁初始位置，轮端 LQR，不发 VMC 关节力矩
} wheelleg_mode_e;

typedef enum
{
    WHEELLEG_FAULT_NONE = 0,
    WHEELLEG_FAULT_MANUAL_OFFLINE = (1u << 0),
    WHEELLEG_FAULT_IMU_OFFLINE = (1u << 1),
    WHEELLEG_FAULT_LEFT_LEG_OFFLINE = (1u << 2),
    WHEELLEG_FAULT_RIGHT_LEG_OFFLINE = (1u << 3),
    WHEELLEG_FAULT_LEFT_WHEEL_OFFLINE = (1u << 4),
    WHEELLEG_FAULT_RIGHT_WHEEL_OFFLINE = (1u << 5),
    WHEELLEG_FAULT_ATTITUDE_LIMIT = (1u << 6),
    WHEELLEG_FAULT_CONTROLLER = (1u << 7),
} wheelleg_fault_flag_e;

typedef struct
{
    fp32 length_m;
    fp32 theta_rad;
    fp32 d_length_mps;
    fp32 d_theta_radps;
    fp32 support_force_n;
    fp32 hip_torque_nm;
    fp32 joint_torque_nm[2];
    uint8_t contact;
    uint8_t motor_online[2];
} wheelleg_leg_state_t;

typedef struct
{
    msg_header_t header;
    uint8_t mode; // wheelleg_mode_e
    uint8_t enable;
    uint8_t jump;
    uint8_t reserved;
    fp32 target_v_mps;
    fp32 target_yaw_rate_radps;
    fp32 target_leg_length_m;
    fp32 target_roll_rad;
} wheelleg_cmd_t;

typedef struct
{
    msg_header_t header;
    fp32 pitch_rad;
    fp32 d_pitch_radps;
    fp32 roll_rad;
    fp32 d_roll_radps;
    fp32 yaw_rad;
    fp32 d_yaw_radps;
    fp32 x_m;
    fp32 x_dot_mps;
    fp32 linear_acc_base[3];
    wheelleg_leg_state_t leg[WHEELLEG_SIDE_COUNT];
    fp32 wheel_pos_rad[WHEELLEG_SIDE_COUNT];
    fp32 wheel_vel_radps[WHEELLEG_SIDE_COUNT];
    fp32 wheel_torque_nm[WHEELLEG_SIDE_COUNT];
    uint8_t wheel_online[WHEELLEG_SIDE_COUNT];
} wheelleg_state_t;

typedef struct
{
    msg_header_t header;
    uint8_t mode; // wheelleg_mode_e
    uint8_t last_mode;
    uint16_t fault_flags;
    uint8_t health; // msg_health_e
    uint8_t controller_active;
    uint16_t active_controller_id;
    fp32 target_v_mps;
    fp32 target_leg_length_m;
    fp32 target_foot_x_m;
    fp32 target_leg_theta_rad;
    fp32 pitch_rad;
    fp32 x_dot_mps;
    fp32 leg_length_m[WHEELLEG_SIDE_COUNT];
    fp32 leg_theta_rad[WHEELLEG_SIDE_COUNT];
    fp32 support_force_n[WHEELLEG_SIDE_COUNT];
    fp32 wheel_torque_nm[WHEELLEG_SIDE_COUNT];
} wheelleg_status_t;

typedef struct
{
    fp32 ref[WHEELLEG_LQR_STATE_DIM];
    fp32 state[WHEELLEG_LQR_STATE_DIM];
    fp32 error[WHEELLEG_LQR_STATE_DIM];
    fp32 output[WHEELLEG_LQR_OUTPUT_DIM];
} wheelleg_lqr_debug_t;

typedef struct
{
    fp32 length_m;
    fp32 theta_rad;
    fp32 d_length_mps;
    fp32 d_theta_radps;
    fp32 support_force_n;
    fp32 virtual_torque_nm;
    fp32 joint_torque_nm[2];
} wheelleg_vmc_debug_leg_t;

typedef struct
{
    msg_header_t header;
    wheelleg_lqr_debug_t lqr;
    wheelleg_vmc_debug_leg_t vmc[WHEELLEG_SIDE_COUNT];
    fp32 observer_x_m;
    fp32 observer_v_mps;
    fp32 observer_acc_mps2;
    uint32_t update_us;
    uint32_t overrun_count;
} wheelleg_debug_t;

typedef char wheelleg_cmd_fits_store[(sizeof(wheelleg_cmd_t) <= STATE_STORE_MAX_BYTES) ? 1 : -1];
typedef char wheelleg_state_fits_store[(sizeof(wheelleg_state_t) <= STATE_STORE_MAX_BYTES) ? 1 : -1];
typedef char wheelleg_status_fits_store[(sizeof(wheelleg_status_t) <= STATE_STORE_MAX_BYTES) ? 1 : -1];
typedef char wheelleg_debug_fits_store[(sizeof(wheelleg_debug_t) <= STATE_STORE_MAX_BYTES) ? 1 : -1];

static inline uint8_t wheelleg_cmd_write(const wheelleg_cmd_t *cmd)
{
    return state_store_write(STATE_WHEELLEG_CMD, cmd, (uint16_t)sizeof(*cmd));
}

static inline uint8_t wheelleg_cmd_read(wheelleg_cmd_t *out)
{
    return state_store_read(STATE_WHEELLEG_CMD, out, (uint16_t)sizeof(*out));
}

static inline uint8_t wheelleg_state_write(const wheelleg_state_t *state)
{
    return state_store_write(STATE_WHEELLEG_STATE, state, (uint16_t)sizeof(*state));
}

static inline uint8_t wheelleg_state_read(wheelleg_state_t *out)
{
    return state_store_read(STATE_WHEELLEG_STATE, out, (uint16_t)sizeof(*out));
}

static inline uint8_t wheelleg_status_write(const wheelleg_status_t *status)
{
    return state_store_write(STATE_WHEELLEG_STATUS, status, (uint16_t)sizeof(*status));
}

static inline uint8_t wheelleg_status_read(wheelleg_status_t *out)
{
    return state_store_read(STATE_WHEELLEG_STATUS, out, (uint16_t)sizeof(*out));
}

static inline uint8_t wheelleg_debug_write(const wheelleg_debug_t *debug)
{
    return state_store_write(STATE_WHEELLEG_DEBUG, debug, (uint16_t)sizeof(*debug));
}

static inline uint8_t wheelleg_debug_read(wheelleg_debug_t *out)
{
    return state_store_read(STATE_WHEELLEG_DEBUG, out, (uint16_t)sizeof(*out));
}

#ifdef __cplusplus
}
#endif

#endif
