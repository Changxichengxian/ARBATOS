/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef WHEELLEG_MSG_H
#define WHEELLEG_MSG_H

#include "RobotMsg.h"
#include "StateStore.h"
#include "Types.h"

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
} WheelLegSide;

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
    WHEELLEG_MODE_LEG_POSITION = 10, // 非 VMC 腿位置模式：遥控目标腿长/足端前后，关节走位置环
    WHEELLEG_MODE_LEG_LQR = 11, // 关节位置控腿高，轮端 LQR
    WHEELLEG_MODE_VMC_HEIGHT = 12, // VMC 控腿高，轮端 LQR
    WHEELLEG_MODE_VMC_BALANCE = 13, // VMC 腿高和髋力矩辅助轮端 LQR
} WheelLegMode;

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
    WHEELLEG_FAULT_DOMAIN_RECOVERY = (1u << 8),
} WheelLegFaultFlag;

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
} WheelLegLegState;

typedef struct
{
    msg_header_t header;
    uint8_t mode; // WheelLegMode
    uint8_t enable;
    uint8_t jump;
    uint8_t reserved;
    fp32 target_v_mps;
    fp32 target_yaw_rate_radps;
    fp32 target_leg_length_m;
    fp32 target_roll_rad;
} WheelLegCmd;

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
    WheelLegLegState leg[WHEELLEG_SIDE_COUNT];
    fp32 wheel_pos_rad[WHEELLEG_SIDE_COUNT];
    fp32 wheel_vel_radps[WHEELLEG_SIDE_COUNT];
    fp32 wheel_torque_nm[WHEELLEG_SIDE_COUNT];
    uint8_t wheel_online[WHEELLEG_SIDE_COUNT];
} WheelLegState;

typedef struct
{
    msg_header_t header;
    uint8_t mode; // WheelLegMode
    uint8_t last_mode;
    uint16_t fault_flags;
    uint8_t health; // msg_health_e
    uint8_t controller_active;
    uint16_t active_controller_id;
    fp32 target_v_mps;
    fp32 target_yaw_rate_radps;
    fp32 target_leg_length_m;
    fp32 target_foot_x_m;
    fp32 target_foot_y_m;
    fp32 target_leg_theta_rad;
    fp32 pitch_rad;
    fp32 x_dot_mps;
    fp32 leg_length_m[WHEELLEG_SIDE_COUNT];
    fp32 leg_theta_rad[WHEELLEG_SIDE_COUNT];
    fp32 leg_alpha_rad[WHEELLEG_SIDE_COUNT];
    fp32 support_force_n[WHEELLEG_SIDE_COUNT];
    fp32 wheel_torque_nm[WHEELLEG_SIDE_COUNT];
    uint16_t domain_fault_flags;
    uint8_t domain_member_count;
    uint8_t domain_online_mask;
    uint8_t domain_binding_valid;
    uint8_t recovery_input_safe;
    uint8_t domain_outputs_active;
    uint8_t domain_inhibit_complete;
    uint8_t domain_action;
    uint8_t domain_recovery_pending;
    uint8_t domain_fault_active;
    uint8_t domain_ever_faulted;
    uint8_t domain_device_count;
    uint32_t domain_active_member_mask;
    uint32_t domain_blocking_member_mask;
    uint32_t domain_active_reason_mask;
    uint32_t domain_blocking_reason_mask;
    uint32_t domain_history_reason_mask;
    uint32_t domain_first_fault_ms;
    uint32_t domain_last_fault_ms;
    uint32_t domain_healthy_since_ms;
    uint32_t domain_stop_count;
    uint32_t domain_stop_fail_count;
    uint32_t domain_inhibit_fail_count;
    uint32_t domain_inhibit_release_fail_count;
    uint32_t domain_last_stop_tick_ms;
} WheelLegStatus;

typedef struct
{
    fp32 ref[WHEELLEG_LQR_STATE_DIM];
    fp32 state[WHEELLEG_LQR_STATE_DIM];
    fp32 error[WHEELLEG_LQR_STATE_DIM];
    fp32 output[WHEELLEG_LQR_OUTPUT_DIM];
} WheelLegLqrDebug;

typedef struct
{
    fp32 length_m;
    fp32 theta_rad;
    fp32 d_length_mps;
    fp32 d_theta_radps;
    fp32 support_force_n;
    fp32 virtual_torque_nm;
    fp32 joint_torque_nm[2];
} WheelLegVmcDebugLeg;

typedef struct
{
    msg_header_t header;
    WheelLegLqrDebug lqr;
    WheelLegVmcDebugLeg vmc[WHEELLEG_SIDE_COUNT];
    fp32 observer_x_m;
    fp32 observer_v_mps;
    fp32 observer_acc_mps2;
    uint32_t update_us;
    uint32_t overrun_count;
} WheelLegDebug;

typedef char WheelLegCmdFitsStore[(sizeof(WheelLegCmd) <= STATE_STORE_WHEELLEG_CMD_BYTES) ? 1 : -1];
typedef char WheelLegStateFitsStore[(sizeof(WheelLegState) <= STATE_STORE_WHEELLEG_STATE_BYTES) ? 1 : -1];
typedef char WheelLegStatusFitsStore[(sizeof(WheelLegStatus) <= STATE_STORE_WHEELLEG_STATUS_BYTES) ? 1 : -1];
typedef char WheelLegDebugFitsStore[(sizeof(WheelLegDebug) <= STATE_STORE_WHEELLEG_DEBUG_BYTES) ? 1 : -1];

static inline uint8_t WheelLegCmdWrite(const WheelLegCmd *cmd)
{
    return StateStoreWrite(STATE_WHEELLEG_CMD, cmd, (uint16_t)sizeof(*cmd));
}

static inline uint8_t WheelLegCmdRead(WheelLegCmd *out)
{
    return StateStoreRead(STATE_WHEELLEG_CMD, out, (uint16_t)sizeof(*out));
}

static inline uint8_t WheelLegStateWrite(const WheelLegState *state)
{
    return StateStoreWrite(STATE_WHEELLEG_STATE, state, (uint16_t)sizeof(*state));
}

static inline uint8_t WheelLegStateRead(WheelLegState *out)
{
    return StateStoreRead(STATE_WHEELLEG_STATE, out, (uint16_t)sizeof(*out));
}

static inline uint8_t WheelLegStatusWrite(const WheelLegStatus *status)
{
    return StateStoreWrite(STATE_WHEELLEG_STATUS, status, (uint16_t)sizeof(*status));
}

static inline uint8_t WheelLegStatusRead(WheelLegStatus *out)
{
    return StateStoreRead(STATE_WHEELLEG_STATUS, out, (uint16_t)sizeof(*out));
}

static inline uint8_t WheelLegDebugWrite(const WheelLegDebug *debug)
{
    return StateStoreWrite(STATE_WHEELLEG_DEBUG, debug, (uint16_t)sizeof(*debug));
}

static inline uint8_t WheelLegDebugRead(WheelLegDebug *out)
{
    return StateStoreRead(STATE_WHEELLEG_DEBUG, out, (uint16_t)sizeof(*out));
}

#ifdef __cplusplus
}
#endif

#endif
