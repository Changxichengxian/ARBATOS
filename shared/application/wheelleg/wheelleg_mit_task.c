/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include "config.h"
#include "robot_task_build_config.h"

#if ROBOT_TASK_BUILD_WHEELLEG_MIT

#include "wheelleg_mit_task.h"
#include "wheelleg_core.h"

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"

#include "INS_task.h"
#include "CAN_receive.h"
#include "LowCmd.h"
#include "bsp_can.h"
#include "config.h"
#include "control_input.h"
#include "detect_task.h"
#include "MotorInst.h"
#include "robot_msg.h"
#include "robot_task_profile.h"
#include "robot_mode.h"
#include "RtProf.h"
#include "ControlMgr.h"
#include "SdLog.h"
#include "watch.h"
#include "wheelleg_msg.h"

#include <math.h>
#include <string.h>

#define WHEELLEG_PI 3.14159265358979323846f
#define WHEELLEG_TWO_PI 6.28318530717958647692f
#define WHEELLEG_FEEDBACK_TIMEOUT_MS 80u
#define WHEELLEG_BENCH_CENTER_LEG_M 0.085f
#define WHEELLEG_MANUAL_FOOT_X_RANGE_M 0.060f
#define WHEELLEG_TARGET_LEG_SLEW_MPS 0.030f
#define WHEELLEG_TARGET_FOOT_X_SLEW_MPS 0.060f
#define WHEELLEG_AUTO_LEG_REACH_EPS_M 0.003f
#define WHEELLEG_AUTO_LEG_DWELL_MS 500u
#define WHEELLEG_DETACHED_JOINT_TEST_KP 2.0f
#define WHEELLEG_DETACHED_JOINT_TEST_KD 0.16f
#define WHEELLEG_SDLOG_BASE_PERIOD_MS 10u
#define WHEELLEG_LQR_WHEEL_TORQUE_SCALE_DEFAULT 0.18f
#define WHEELLEG_LQR_HIP_TORQUE_SCALE_DEFAULT 0.18f
#define WHEELLEG_LQR_X_HOLD_LIMIT_M 0.05f
#define WHEELLEG_LQR_MOTION_EPS 1.0e-4f
#define WHEELLEG_PITCH_TRIM_GAIN_DEFAULT 0.08f
#define WHEELLEG_PITCH_TRIM_MAX_RAD_DEFAULT 0.18f
#define WHEELLEG_PITCH_TRIM_RATE_RADPS_DEFAULT 0.015f
#define WHEELLEG_PITCH_TRIM_V_DEADBAND_MPS_DEFAULT 0.015f
#define WHEELLEG_PITCH_TRIM_LPF_DEFAULT 0.03f

#define WHEELLEG_CONTROL_STAGE_BENCH_LQR 0u
#define WHEELLEG_CONTROL_STAGE_POSITION_LQR 1u
#define WHEELLEG_CONTROL_STAGE_VMC_HEIGHT_LQR 2u
#define WHEELLEG_CONTROL_STAGE_VMC_FULL_LQR 3u
#define WHEELLEG_CONTROL_STAGE_MAX WHEELLEG_CONTROL_STAGE_VMC_FULL_LQR

#ifndef WHEELLEG_BENCH_TARGET_FOOT_X_M
#define WHEELLEG_BENCH_TARGET_FOOT_X_M 0.0f
#endif

#ifndef WHEELLEG_ENABLE_VMC_BALANCE
#define WHEELLEG_ENABLE_VMC_BALANCE 0
#endif

#ifndef WHEELLEG_IMU_BODY_ROLL_OFFSET
#define WHEELLEG_IMU_BODY_ROLL_OFFSET INS_ROLL_ADDRESS_OFFSET
#endif

#ifndef WHEELLEG_IMU_BODY_PITCH_OFFSET
#define WHEELLEG_IMU_BODY_PITCH_OFFSET INS_PITCH_ADDRESS_OFFSET
#endif

#ifndef WHEELLEG_IMU_BODY_YAW_OFFSET
#define WHEELLEG_IMU_BODY_YAW_OFFSET INS_YAW_ADDRESS_OFFSET
#endif

#ifndef WHEELLEG_IMU_ROLL_SIGN
#define WHEELLEG_IMU_ROLL_SIGN 1.0f
#endif

#ifndef WHEELLEG_IMU_PITCH_SIGN
#define WHEELLEG_IMU_PITCH_SIGN 1.0f
#endif

#ifndef WHEELLEG_IMU_YAW_SIGN
#define WHEELLEG_IMU_YAW_SIGN 1.0f
#endif

#ifndef WHEELLEG_IMU_BODY_GYRO_X_OFFSET
#define WHEELLEG_IMU_BODY_GYRO_X_OFFSET INS_GYRO_X_ADDRESS_OFFSET
#endif

#ifndef WHEELLEG_IMU_BODY_GYRO_Y_OFFSET
#define WHEELLEG_IMU_BODY_GYRO_Y_OFFSET INS_GYRO_Y_ADDRESS_OFFSET
#endif

#ifndef WHEELLEG_IMU_BODY_GYRO_Z_OFFSET
#define WHEELLEG_IMU_BODY_GYRO_Z_OFFSET INS_GYRO_Z_ADDRESS_OFFSET
#endif

#ifndef WHEELLEG_IMU_GYRO_X_SIGN
#define WHEELLEG_IMU_GYRO_X_SIGN WHEELLEG_IMU_ROLL_SIGN
#endif

#ifndef WHEELLEG_IMU_GYRO_Y_SIGN
#define WHEELLEG_IMU_GYRO_Y_SIGN WHEELLEG_IMU_PITCH_SIGN
#endif

#ifndef WHEELLEG_IMU_GYRO_Z_SIGN
#define WHEELLEG_IMU_GYRO_Z_SIGN WHEELLEG_IMU_YAW_SIGN
#endif

typedef struct
{
    MotorId front;
    MotorId back;
    MotorId wheel;
} wheelleg_actuator_map_t;

typedef wheelleg_core_pid_t wheelleg_pid_t;
typedef wheelleg_core_leg_calc_t wheelleg_leg_calc_t;

typedef struct
{
    fp32 x_m;
    fp32 y_m;
    fp32 length_m;
} wheelleg_foot_point_t;

static wheelleg_core_geometry_t wheelleg_core_geometry_from_config(void)
{
    wheelleg_core_geometry_t geo;
    const wheelleg_mit_config_t *cfg = &g_config.wheelleg_mit;

    geo.l1_m = cfg->l1_m;
    geo.l2_m = cfg->l2_m;
    geo.l3_m = cfg->l3_m;
    geo.l4_m = cfg->l4_m;
    geo.l5_m = cfg->l5_m;
    return geo;
}

static void wheelleg_core_point_from_local(const wheelleg_foot_point_t *src, wheelleg_core_foot_point_t *dst)
{
    if (src == NULL || dst == NULL)
    {
        return;
    }

    dst->x_m = src->x_m;
    dst->y_m = src->y_m;
    dst->length_m = src->length_m;
}

typedef struct
{
    wheelleg_actuator_map_t actuator[WHEELLEG_SIDE_COUNT];
    MotorState front_fb[WHEELLEG_SIDE_COUNT];
    MotorState back_fb[WHEELLEG_SIDE_COUNT];
    MotorState wheel_fb[WHEELLEG_SIDE_COUNT];
    wheelleg_leg_calc_t leg[WHEELLEG_SIDE_COUNT];
    wheelleg_pid_t leg_pid[WHEELLEG_SIDE_COUNT];
    wheelleg_pid_t split_pid;
    fp32 x_m;
    fp32 v_mps;
    fp32 target_leg_smooth;
    fp32 target_foot_x_smooth;
    fp32 target_foot_y_smooth;
    fp32 yaw_set;
    fp32 last_yaw;
    uint8_t target_smooth_valid;
    uint8_t yaw_inited;
    uint8_t balance_idle_latched;
    uint8_t ever_commanded;
    uint8_t control_stage;
    wheelleg_mode_e mode;
    wheelleg_mode_e last_mode;
    uint32_t overrun_count;
    uint8_t left_test_active;
    uint32_t left_test_start_ms;
    uint8_t foot_test_active;
    uint32_t foot_test_start_ms;
    uint8_t foot_test_phase;
    uint8_t foot_test_ik_ok;
    uint8_t detached_test_active;
    uint32_t detached_test_start_ms;
    uint8_t auto_leg_stage;
    uint32_t auto_leg_stage_tick_ms;
    fp32 bench_hold_front_rad[WHEELLEG_SIDE_COUNT];
    fp32 bench_hold_back_rad[WHEELLEG_SIDE_COUNT];
    uint8_t bench_hold_front_valid[WHEELLEG_SIDE_COUNT];
    uint8_t bench_hold_back_valid[WHEELLEG_SIDE_COUNT];
    uint8_t bench_hold_pose_valid;
    fp32 bench_hold_target_leg_m;
    fp32 bench_hold_target_foot_x_m;
    fp32 pitch_auto_offset_rad;
    fp32 pitch_trim_v_lpf_mps;
    wheelleg_foot_point_t foot_test_target[WHEELLEG_SIDE_COUNT];
    fp32 foot_test_wheel_zero_rad[WHEELLEG_SIDE_COUNT];
    uint8_t foot_test_wheel_zero_valid[WHEELLEG_SIDE_COUNT];
    fp32 foot_test_wheel_dx_m[WHEELLEG_SIDE_COUNT];
    fp32 foot_test_wheel_comp_rad[WHEELLEG_SIDE_COUNT];
    fp32 foot_test_wheel_target_rad[WHEELLEG_SIDE_COUNT];
    uint16_t feedback_faults;
} wheelleg_mit_ctrl_t;

typedef struct
{
    uint32_t loop_start_ms;
    uint32_t now_ms;
    uint16_t period_ms;
    fp32 dt;
    fp32 gyro_aligned[3];
    const fp32 *gyro_wheelleg;
    fp32 pitch;
    fp32 roll;
    fp32 yaw;
    int16_t vx_axis;
    int16_t yaw_axis;
    int16_t single_test_axis;
    int16_t leg_axis;
    uint8_t control_stage;
    uint8_t profile_on;
    uint8_t manual_on;
    uint8_t imu_ok;
    uint8_t single_test;
    uint8_t left_leg_test;
    uint8_t foot_test;
    uint8_t operation_test_active;
    uint8_t enabled;
    uint8_t controller_active;
    uint8_t kinematics_ok;
    uint16_t faults;
    uint16_t feedback_faults;
    fp32 target_v;
    fp32 target_yaw_rate;
    fp32 target_leg;
    fp32 target_foot_x;
    fp32 target_foot_y;
    fp32 wheel_torque[WHEELLEG_SIDE_COUNT];
} wheelleg_task_frame_t;

typedef struct
{
    fp32 theta_err;
    fp32 d_theta;
    fp32 x_err;
    fp32 v_err;
    fp32 pitch_err;
    fp32 gyro_y;
} wheelleg_lqr_side_state_t;

static wheelleg_mit_ctrl_t s_wheelleg;
static uint8_t s_wheelleg_sdlog_config_logged = 0u;
static uint32_t s_wheelleg_sdlog_last_status_ms = 0u;

static uint8_t wheelleg_feedback_fresh(const MotorState *fb, uint32_t now_ms);
static void wheelleg_SdLogWrite_motor_diag(uint32_t now_ms);

static fp32 wheelleg_axis_to_fp32(int16_t axis, fp32 max_abs, uint16_t deadband);
static fp32 wheelleg_lqr_x_error(fp32 target_v, fp32 target_yaw_rate);
static fp32 wheelleg_lqr_pitch_offset_for_side(uint8_t side);
static void wheelleg_pitch_trim_update(fp32 dt,
                                       int16_t vx_axis,
                                       int16_t yaw_axis,
                                       fp32 target_v,
                                       fp32 target_yaw_rate);
static MotorId wheelleg_single_test_actuator(void);

uint8_t wheelleg_mit_get_foot_test_phase(void)
{
    return s_wheelleg.foot_test_phase;
}

uint8_t wheelleg_mit_get_foot_test_ik_ok(void)
{
    return s_wheelleg.foot_test_ik_ok;
}

void wheelleg_mit_get_foot_test_target(uint8_t side, fp32 *x_m, fp32 *y_m, fp32 *length_m)
{
    const wheelleg_foot_point_t zero = {0.0f, 0.0f, 0.0f};
    const wheelleg_foot_point_t *target =
        (side < WHEELLEG_SIDE_COUNT) ? &s_wheelleg.foot_test_target[side] : &zero;

    if (x_m != NULL)
    {
        *x_m = target->x_m;
    }
    if (y_m != NULL)
    {
        *y_m = target->y_m;
    }
    if (length_m != NULL)
    {
        *length_m = target->length_m;
    }
}

void wheelleg_mit_get_foot_test_wheel(uint8_t side,
                                      uint8_t *zero_valid,
                                      fp32 *zero_rad,
                                      fp32 *dx_m,
                                      fp32 *comp_rad,
                                      fp32 *target_rad)
{
    if (side >= WHEELLEG_SIDE_COUNT)
    {
        if (zero_valid != NULL)
        {
            *zero_valid = 0u;
        }
        if (zero_rad != NULL)
        {
            *zero_rad = 0.0f;
        }
        if (dx_m != NULL)
        {
            *dx_m = 0.0f;
        }
        if (comp_rad != NULL)
        {
            *comp_rad = 0.0f;
        }
        if (target_rad != NULL)
        {
            *target_rad = 0.0f;
        }
        return;
    }

    if (zero_valid != NULL)
    {
        *zero_valid = s_wheelleg.foot_test_wheel_zero_valid[side];
    }
    if (zero_rad != NULL)
    {
        *zero_rad = s_wheelleg.foot_test_wheel_zero_rad[side];
    }
    if (dx_m != NULL)
    {
        *dx_m = s_wheelleg.foot_test_wheel_dx_m[side];
    }
    if (comp_rad != NULL)
    {
        *comp_rad = s_wheelleg.foot_test_wheel_comp_rad[side];
    }
    if (target_rad != NULL)
    {
        *target_rad = s_wheelleg.foot_test_wheel_target_rad[side];
    }
}

#include "wheelleg_mit_math_helpers.inc"

#include "wheelleg_mit_sdlog.inc"
#include "wheelleg_mit_io_helpers.inc"

#include "wheelleg_mit_control_helpers.inc"
#include "wheelleg_mit_runtime_helpers.inc"
#include "wheelleg_mit_position_helpers.inc"

#include "wheelleg_mit_control_loop.inc"

#include "wheelleg_mit_frame_runner.inc"

void wheelleg_mit_task(void const *pvParameters)
{
    TickType_t last_wake;

    (void)pvParameters;
    memset(&s_wheelleg, 0, sizeof(s_wheelleg));
    wheelleg_target_smooth_reset();
    osDelay(g_config.wheelleg_mit.task_init_time_ms);
    last_wake = xTaskGetTickCount();

    while (1)
    {
        const uint64_t profiler_start_us = RtProfBegin();
        wheelleg_task_frame_t frame;

        wheelleg_task_frame_init(&frame);
        if (WheellegControlMgrAllows(frame.now_ms, frame.dt) == 0u)
        {
            wheelleg_handle_disabled_frame(&frame);
            wheelleg_task_finish_frame(&frame, frame.faults, profiler_start_us, &last_wake, 0u);
            continue;
        }

        if (frame.enabled == 0u)
        {
            wheelleg_handle_disabled_frame(&frame);
            wheelleg_task_finish_frame(&frame, frame.faults, profiler_start_us, &last_wake, 0u);
            continue;
        }

        if (wheelleg_run_operation_test(&frame) != 0u)
        {
            wheelleg_task_finish_frame(&frame, frame.faults, profiler_start_us, &last_wake, 1u);
            continue;
        }

        wheelleg_run_balance_frame(&frame);
        wheelleg_task_finish_frame(&frame,
                                   (uint16_t)(frame.faults | frame.feedback_faults),
                                   profiler_start_us,
                                   &last_wake,
                                   1u);
    }
}

#endif
