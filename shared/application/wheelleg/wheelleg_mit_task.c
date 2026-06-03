/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include "wheelleg_mit_task.h"
#include "wheelleg_core.h"

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"

#include "INS_task.h"
#include "LowCmd.h"
#include "config.h"
#include "control_input.h"
#include "detect_task.h"
#include "motor_instance.h"
#include "robot_msg.h"
#include "robot_task_profile.h"
#include "robot_mode.h"
#include "rt_profiler.h"
#include "sdlog.h"
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

static void wheelleg_local_point_from_core(const wheelleg_core_foot_point_t *src, wheelleg_foot_point_t *dst)
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
    wheelleg_foot_point_t foot_test_target[WHEELLEG_SIDE_COUNT];
    fp32 foot_test_wheel_zero_rad[WHEELLEG_SIDE_COUNT];
    uint8_t foot_test_wheel_zero_valid[WHEELLEG_SIDE_COUNT];
    fp32 foot_test_wheel_dx_m[WHEELLEG_SIDE_COUNT];
    fp32 foot_test_wheel_comp_rad[WHEELLEG_SIDE_COUNT];
    fp32 foot_test_wheel_target_rad[WHEELLEG_SIDE_COUNT];
    uint16_t feedback_faults;
} wheelleg_mit_ctrl_t;

static wheelleg_mit_ctrl_t s_wheelleg;
static uint8_t s_wheelleg_sdlog_config_logged = 0u;
static uint32_t s_wheelleg_sdlog_last_status_ms = 0u;

static fp32 wheelleg_axis_to_fp32(int16_t axis, fp32 max_abs, uint16_t deadband);
static fp32 wheelleg_lqr_x_error(fp32 target_v, fp32 target_yaw_rate);
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

static const fp32 s_default_lqr_poly[12][4] = {
    {-243.932f, 105.148f, -19.1838f, -0.199759f},
    {-6.33721f, 2.6174f, -1.08798f, -0.0047227f},
    {-43.8763f, 16.3233f, -2.10154f, -0.127721f},
    {-52.0411f, 19.3719f, -2.60375f, -0.168927f},
    {-805.793f, 328.293f, -48.8092f, 2.92903f},
    {-40.1396f, 16.7832f, -2.61208f, 0.17602f},
    {-962.682f, 417.508f, -67.9889f, 4.84346f},
    {-89.4595f, 37.3246f, -5.80403f, 0.414703f},
    {-618.557f, 251.264f, -37.1404f, 2.18751f},
    {-800.904f, 324.39f, -47.7682f, 2.80481f},
    {3575.2f, -1332.91f, 172.196f, 9.48315f},
    {202.812f, -76.6035f, 10.1013f, 0.345984f},
};

static fp32 wheelleg_clamp(fp32 value, fp32 min_value, fp32 max_value)
{
    return wheelleg_core_clamp(value, min_value, max_value);
}

static fp32 wheelleg_abs(fp32 value)
{
    return wheelleg_core_abs(value);
}

static uint8_t wheelleg_axis_in_deadband(int16_t axis, uint16_t deadband)
{
    return wheelleg_core_axis_in_deadband(axis, deadband);
}

static fp32 wheelleg_target_theta_from_foot_x(fp32 foot_x_m, fp32 leg_length_m)
{
    return wheelleg_core_target_theta_from_foot_x(foot_x_m, leg_length_m);
}

static fp32 wheelleg_bench_default_leg_m(void)
{
    const wheelleg_mit_config_t *cfg = &g_config.wheelleg_mit;
    fp32 length = cfg->min_leg_length_m;

    if (length <= 0.02f)
    {
        length = WHEELLEG_BENCH_CENTER_LEG_M;
    }
    if (cfg->max_leg_length_m > 0.02f)
    {
        length = wheelleg_clamp(length, 0.02f, cfg->max_leg_length_m);
    }
    return length;
}

static uint8_t wheelleg_limit_foot_xy(fp32 *x_m, fp32 *y_m, fp32 *length_m)
{
    const wheelleg_mit_config_t *cfg = &g_config.wheelleg_mit;
    fp32 min_leg = cfg->min_leg_length_m;
    fp32 max_leg = cfg->max_leg_length_m;

    if (x_m == NULL || y_m == NULL || length_m == NULL)
    {
        return 0u;
    }

    if (min_leg <= 0.02f)
    {
        min_leg = 0.02f;
    }
    if (max_leg < min_leg)
    {
        max_leg = min_leg;
    }

    return wheelleg_core_limit_foot_xy(min_leg,
                                       max_leg,
                                       WHEELLEG_MANUAL_FOOT_X_RANGE_M,
                                       x_m,
                                       y_m,
                                       length_m);
}

static wheelleg_core_target_smooth_t wheelleg_target_smooth_from_state(void)
{
    wheelleg_core_target_smooth_t smooth;

    smooth.foot_x_m = s_wheelleg.target_foot_x_smooth;
    smooth.foot_y_m = s_wheelleg.target_foot_y_smooth;
    smooth.length_m = s_wheelleg.target_leg_smooth;
    smooth.valid = s_wheelleg.target_smooth_valid;
    return smooth;
}

static void wheelleg_target_smooth_to_state(const wheelleg_core_target_smooth_t *smooth)
{
    if (smooth == NULL)
    {
        return;
    }

    s_wheelleg.target_foot_x_smooth = smooth->foot_x_m;
    s_wheelleg.target_foot_y_smooth = smooth->foot_y_m;
    s_wheelleg.target_leg_smooth = smooth->length_m;
    s_wheelleg.target_smooth_valid = smooth->valid;
}

static void wheelleg_target_smooth_reset(void)
{
    wheelleg_core_target_smooth_t smooth;
    uint8_t side;

    wheelleg_core_target_smooth_clear(&smooth);
    wheelleg_target_smooth_to_state(&smooth);
    s_wheelleg.auto_leg_stage = 0u;
    s_wheelleg.auto_leg_stage_tick_ms = 0u;
    s_wheelleg.bench_hold_pose_valid = 0u;
    s_wheelleg.bench_hold_target_leg_m = wheelleg_bench_default_leg_m();
    s_wheelleg.bench_hold_target_foot_x_m = WHEELLEG_BENCH_TARGET_FOOT_X_M;
    for (side = 0u; side < WHEELLEG_SIDE_COUNT; side++)
    {
        s_wheelleg.bench_hold_front_valid[side] = 0u;
        s_wheelleg.bench_hold_back_valid[side] = 0u;
        s_wheelleg.bench_hold_front_rad[side] = 0.0f;
        s_wheelleg.bench_hold_back_rad[side] = 0.0f;
    }
}

static void wheelleg_balance_state_reset(void)
{
    wheelleg_target_smooth_reset();
    s_wheelleg.x_m = 0.0f;
    s_wheelleg.v_mps = 0.0f;
    s_wheelleg.balance_idle_latched = 0u;
    s_wheelleg.leg[WHEELLEG_SIDE_LEFT].first = 0u;
    s_wheelleg.leg[WHEELLEG_SIDE_RIGHT].first = 0u;
}

static void wheelleg_target_smooth_update_xy(fp32 target_foot_x, fp32 target_foot_y, fp32 dt)
{
    wheelleg_core_target_smooth_t smooth = wheelleg_target_smooth_from_state();
    const wheelleg_mit_config_t *cfg = &g_config.wheelleg_mit;
    const fp32 measured_leg =
        (s_wheelleg.leg[WHEELLEG_SIDE_RIGHT].length + s_wheelleg.leg[WHEELLEG_SIDE_LEFT].length) * 0.5f;
    const fp32 measured_alpha =
        (s_wheelleg.leg[WHEELLEG_SIDE_RIGHT].alpha + s_wheelleg.leg[WHEELLEG_SIDE_LEFT].alpha) * 0.5f;

    if (wheelleg_core_target_smooth_update_xy(&smooth,
                                              target_foot_x,
                                              target_foot_y,
                                              measured_leg,
                                              measured_alpha,
                                              cfg->min_leg_length_m,
                                              cfg->max_leg_length_m,
                                              WHEELLEG_MANUAL_FOOT_X_RANGE_M,
                                              WHEELLEG_TARGET_FOOT_X_SLEW_MPS,
                                              WHEELLEG_TARGET_LEG_SLEW_MPS,
                                              0.003f,
                                              dt) != 0u)
    {
        wheelleg_target_smooth_to_state(&smooth);
    }
}

#if WHEELLEG_ENABLE_VMC_BALANCE
static fp32 wheelleg_auto_leg_target_by_stage(uint8_t stage)
{
    switch (stage)
    {
    case 0u:
        return 0.100f;
    case 1u:
        return 0.120f;
    default:
        return 0.085f;
    }
}

static fp32 wheelleg_measured_leg_average(void)
{
    return (s_wheelleg.leg[WHEELLEG_SIDE_RIGHT].length +
            s_wheelleg.leg[WHEELLEG_SIDE_LEFT].length) *
           0.5f;
}

static fp32 wheelleg_auto_leg_target(uint32_t now_ms)
{
    fp32 target = wheelleg_auto_leg_target_by_stage(s_wheelleg.auto_leg_stage);

    if (s_wheelleg.auto_leg_stage < 2u &&
        s_wheelleg.target_smooth_valid != 0u &&
        wheelleg_abs(s_wheelleg.target_leg_smooth - target) <= WHEELLEG_AUTO_LEG_REACH_EPS_M &&
        wheelleg_abs(wheelleg_measured_leg_average() - target) <= WHEELLEG_AUTO_LEG_REACH_EPS_M)
    {
        if (s_wheelleg.auto_leg_stage_tick_ms == 0u)
        {
            s_wheelleg.auto_leg_stage_tick_ms = now_ms;
        }
        else if ((uint32_t)(now_ms - s_wheelleg.auto_leg_stage_tick_ms) >= WHEELLEG_AUTO_LEG_DWELL_MS)
        {
            s_wheelleg.auto_leg_stage++;
            s_wheelleg.auto_leg_stage_tick_ms = 0u;
            target = wheelleg_auto_leg_target_by_stage(s_wheelleg.auto_leg_stage);
        }
    }
    else
    {
        s_wheelleg.auto_leg_stage_tick_ms = 0u;
    }

    return target;
}
#endif

static fp32 wheelleg_joint_to_raw(fp32 kinematics_position, fp32 zero_position, int8_t dir)
{
    return wheelleg_core_joint_to_raw(kinematics_position, zero_position, dir);
}

static fp32 wheelleg_raw_to_kinematic(fp32 raw_position,
                                      fp32 raw_zero_position,
                                      int8_t dir,
                                      fp32 kinematic_zero_position)
{
    return wheelleg_core_raw_to_kinematic(raw_position, raw_zero_position, kinematic_zero_position, dir);
}

static fp32 wheelleg_kinematic_to_raw(fp32 kinematic_position,
                                      fp32 raw_zero_position,
                                      int8_t dir,
                                      fp32 kinematic_zero_position)
{
    return wheelleg_core_kinematic_to_raw(kinematic_position, raw_zero_position, kinematic_zero_position, dir);
}

static uint16_t wheelleg_period_ms(void)
{
    return (g_config.wheelleg_mit.control_period_ms == 0u) ? 3u : g_config.wheelleg_mit.control_period_ms;
}

static fp32 wheelleg_period_s(void)
{
    return (fp32)wheelleg_period_ms() * 0.001f;
}

static uint32_t wheelleg_tick_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static void wheelleg_pid_apply(wheelleg_pid_t *pid, const pid_param_t *cfg)
{
    if (pid == NULL || cfg == NULL)
    {
        return;
    }
    wheelleg_core_pid_configure(pid,
                                cfg->kp,
                                cfg->ki,
                                cfg->kd,
                                cfg->max_out,
                                cfg->max_iout);
}

static void wheelleg_pid_clear(wheelleg_pid_t *pid)
{
    wheelleg_core_pid_clear(pid);
}

static fp32 wheelleg_pid_calc(wheelleg_pid_t *pid, fp32 ref, fp32 set)
{
    return wheelleg_core_pid_calc(pid, ref, set);
}

static fp32 wheelleg_poly(const fp32 coe[4], fp32 len)
{
    return wheelleg_core_poly4(coe, len);
}

static uint8_t wheelleg_lqr_row_is_zero(const fp32 coe[4])
{
    return wheelleg_core_lqr_row_is_zero(coe);
}

static const fp32 *wheelleg_lqr_row(uint8_t index)
{
    const fp32 *configured;

    if (index >= 12u)
    {
        return &s_default_lqr_poly[0][0];
    }
    configured = &g_config.wheelleg_mit.lqr_poly[index][0];
    return (wheelleg_lqr_row_is_zero(configured) == 0u) ? configured : &s_default_lqr_poly[index][0];
}

static void wheelleg_eval_lqr(fp32 leg_length, fp32 out[12])
{
    uint8_t i;
    fp32 length = leg_length;

    if (out == NULL)
    {
        return;
    }

    if (g_config.wheelleg_mit.min_leg_length_m < g_config.wheelleg_mit.max_leg_length_m)
    {
        length = wheelleg_clamp(length,
                                g_config.wheelleg_mit.min_leg_length_m,
                                g_config.wheelleg_mit.max_leg_length_m);
    }

    for (i = 0u; i < 12u; i++)
    {
        out[i] = wheelleg_poly(wheelleg_lqr_row(i), length);
    }
}

static uint8_t wheelleg_manual_enabled_by_switch(void)
{
    return (uint8_t)(control_input_switch_is_pos(input_switch(INPUT_SW_CHASSIS_MODE),
                                                 g_config.manual_input.semantics.chassis_safe_pos) == 0u);
}

static fp32 wheelleg_imu_angle(const fp32 *angle, uint8_t offset, fp32 sign)
{
    return (angle != NULL) ? angle[offset] * sign : 0.0f;
}

static const fp32 *wheelleg_imu_gyro_aligned(const fp32 *gyro, fp32 aligned[3])
{
    if (gyro == NULL || aligned == NULL)
    {
        return NULL;
    }

    aligned[INS_GYRO_X_ADDRESS_OFFSET] = gyro[WHEELLEG_IMU_BODY_GYRO_X_OFFSET] * WHEELLEG_IMU_GYRO_X_SIGN;
    aligned[INS_GYRO_Y_ADDRESS_OFFSET] = gyro[WHEELLEG_IMU_BODY_GYRO_Y_OFFSET] * WHEELLEG_IMU_GYRO_Y_SIGN;
    aligned[INS_GYRO_Z_ADDRESS_OFFSET] = gyro[WHEELLEG_IMU_BODY_GYRO_Z_OFFSET] * WHEELLEG_IMU_GYRO_Z_SIGN;
    return aligned;
}

static void wheelleg_sdlog_copy_pid(sdlog_pid_param_t *out, const pid_param_t *pid)
{
    if (out == NULL || pid == NULL)
    {
        return;
    }

    out->kp = pid->kp;
    out->ki = pid->ki;
    out->kd = pid->kd;
    out->max_out = pid->max_out;
    out->max_iout = pid->max_iout;
}

static void wheelleg_sdlog_write_config_once(void)
{
    sdlog_wheelleg_mit_config_t log;
    const wheelleg_mit_config_t *cfg = &g_config.wheelleg_mit;
    uint8_t i;

    if (!sdlog_is_active())
    {
        s_wheelleg_sdlog_config_logged = 0u;
        return;
    }
    if (s_wheelleg_sdlog_config_logged != 0u)
    {
        return;
    }

    (void)memset(&log, 0, sizeof(log));
    log.version = SDLOG_WHEELLEG_MIT_CONFIG_VERSION;
    log.enable_switch_pos = cfg->enable_switch_pos;
    log.control_period_ms = cfg->control_period_ms;
    log.rc_deadband = cfg->rc_deadband;

    log.actuator_id[0] = cfg->left_front_actuator;
    log.actuator_id[1] = cfg->left_back_actuator;
    log.actuator_id[2] = cfg->left_wheel_actuator;
    log.actuator_id[3] = cfg->right_front_actuator;
    log.actuator_id[4] = cfg->right_back_actuator;
    log.actuator_id[5] = cfg->right_wheel_actuator;

    log.joint_dir[0] = cfg->left_front_dir;
    log.joint_dir[1] = cfg->left_back_dir;
    log.joint_dir[2] = cfg->right_front_dir;
    log.joint_dir[3] = cfg->right_back_dir;

    log.joint_zero_rad[0] = cfg->left_front_zero_rad;
    log.joint_zero_rad[1] = cfg->left_back_zero_rad;
    log.joint_zero_rad[2] = cfg->right_front_zero_rad;
    log.joint_zero_rad[3] = cfg->right_back_zero_rad;

    log.l1_m = cfg->l1_m;
    log.l2_m = cfg->l2_m;
    log.l3_m = cfg->l3_m;
    log.l4_m = cfg->l4_m;
    log.l5_m = cfg->l5_m;
    log.wheel_radius_m = cfg->wheel_radius_m;
    log.default_leg_length_m = cfg->default_leg_length_m;
    log.min_leg_length_m = cfg->min_leg_length_m;
    log.max_leg_length_m = cfg->max_leg_length_m;
    log.support_bias_n = cfg->support_bias_n;
    log.leg_mass_kg = cfg->leg_mass_kg;
    log.max_wheel_torque_nm = cfg->max_wheel_torque_nm;
    log.max_joint_torque_nm = cfg->max_joint_torque_nm;
    log.max_jump_joint_torque_nm = cfg->max_jump_joint_torque_nm;
    log.max_support_force_n = cfg->max_support_force_n;
    log.attitude_limit_rad = cfg->attitude_limit_rad;
    log.observer_lpf = cfg->observer_lpf;
    log.pitch_balance_offset_right_rad = cfg->pitch_balance_offset_right_rad;
    log.pitch_balance_offset_left_rad = cfg->pitch_balance_offset_left_rad;
    log.max_v_mps = cfg->max_v_mps;
    log.max_yaw_rate_radps = cfg->max_yaw_rate_radps;

    wheelleg_sdlog_copy_pid(&log.leg_length_pid, &cfg->leg_length_pid);
    wheelleg_sdlog_copy_pid(&log.leg_split_pid, &cfg->leg_split_pid);
    wheelleg_sdlog_copy_pid(&log.turn_pid, &cfg->turn_pid);
    wheelleg_sdlog_copy_pid(&log.roll_pid, &cfg->roll_pid);

    for (i = 0u; i < 12u; i++)
    {
        const fp32 *row = wheelleg_lqr_row(i);
        if (wheelleg_lqr_row_is_zero(&cfg->lqr_poly[i][0]) != 0u)
        {
            log.lqr_default_mask |= (uint16_t)(1u << i);
        }
        log.effective_lqr_poly[i][0] = row[0];
        log.effective_lqr_poly[i][1] = row[1];
        log.effective_lqr_poly[i][2] = row[2];
        log.effective_lqr_poly[i][3] = row[3];
    }

    sdlog_write(SDLOG_TAG_WHEELLEG_MIT_CONFIG, &log, (uint16_t)sizeof(log));
    s_wheelleg_sdlog_config_logged = 1u;
}

static void wheelleg_sdlog_write_status(uint16_t faults,
                                        wheelleg_mode_e mode,
                                        fp32 pitch,
                                        fp32 roll,
                                        fp32 yaw,
                                        const fp32 gyro[3],
                                        fp32 target_v,
                                        fp32 target_leg,
                                        fp32 target_foot_x,
                                        fp32 target_yaw_rate,
                                        fp32 target_theta,
                                        const fp32 wheel_torque[WHEELLEG_SIDE_COUNT],
                                        uint8_t controller_active,
                                        uint32_t now_ms)
{
    sdlog_wheelleg_mit_status_t log;
    const uint32_t log_period_ms = (uint32_t)WHEELLEG_SDLOG_BASE_PERIOD_MS *
                                   (uint32_t)sdlog_high_rate_divider();
    const fp32 lqr_x_err = wheelleg_lqr_x_error(target_v, target_yaw_rate);
    uint8_t side;

    if (!sdlog_is_active())
    {
        s_wheelleg_sdlog_config_logged = 0u;
        s_wheelleg_sdlog_last_status_ms = 0u;
        return;
    }

    wheelleg_sdlog_write_config_once();

    if (s_wheelleg_sdlog_last_status_ms != 0u &&
        (uint32_t)(now_ms - s_wheelleg_sdlog_last_status_ms) < log_period_ms)
    {
        return;
    }
    s_wheelleg_sdlog_last_status_ms = now_ms;

    (void)memset(&log, 0, sizeof(log));
    log.version = SDLOG_WHEELLEG_MIT_STATUS_VERSION;
    log.mode = (uint8_t)mode;
    log.last_mode = (uint8_t)s_wheelleg.last_mode;
    log.controller_active = controller_active;
    log.fault_flags = faults;
    log.feedback_faults = s_wheelleg.feedback_faults;
    log.test_mode = (uint8_t)robot_mode_variant();
    log.profile_on = robot_profile_is_wheelleg_mit();
    log.manual_on = wheelleg_manual_enabled_by_switch();

    log.pitch_rad = pitch;
    log.roll_rad = roll;
    log.yaw_rad = yaw;
    if (gyro != NULL)
    {
        log.gyro_radps[0] = gyro[INS_GYRO_X_ADDRESS_OFFSET];
        log.gyro_radps[1] = gyro[INS_GYRO_Y_ADDRESS_OFFSET];
        log.gyro_radps[2] = gyro[INS_GYRO_Z_ADDRESS_OFFSET];
    }

    log.target_v_mps = target_v;
    log.target_yaw_rate_radps = target_yaw_rate;
    log.target_leg_length_m = target_leg;
    log.target_foot_x_m = target_foot_x;
    log.target_leg_theta_rad = target_theta;
    log.observer_x_m = s_wheelleg.x_m;
    log.observer_v_mps = s_wheelleg.v_mps;

    for (side = 0u; side < WHEELLEG_SIDE_COUNT; side++)
    {
        const wheelleg_leg_calc_t *leg = &s_wheelleg.leg[side];
        const MotorState *wheel_fb = &s_wheelleg.wheel_fb[side];

        log.leg_length_m[side] = leg->length;
        log.leg_theta_rad[side] = leg->theta;
        log.leg_d_length_mps[side] = leg->d_length;
        log.leg_d_theta_radps[side] = leg->d_theta;
        log.support_force_n[side] = leg->f0;
        log.hip_torque_nm[side] = leg->tp;
        log.joint_torque_nm[side][0] = leg->joint_torque[0];
        log.joint_torque_nm[side][1] = leg->joint_torque[1];
        log.wheel_pos_rad[side] = wheel_fb->q;
        log.wheel_vel_radps[side] = wheel_fb->dq;
        log.wheel_torque_nm[side] = (wheel_torque != NULL) ? wheel_torque[side] : 0.0f;
        log.contact[side] = leg->contact;
    }

    log.motor_online_bits =
        (uint8_t)((s_wheelleg.front_fb[WHEELLEG_SIDE_LEFT].online ? (1u << 0) : 0u) |
                  (s_wheelleg.back_fb[WHEELLEG_SIDE_LEFT].online ? (1u << 1) : 0u) |
                  (s_wheelleg.wheel_fb[WHEELLEG_SIDE_LEFT].online ? (1u << 2) : 0u) |
                  (s_wheelleg.front_fb[WHEELLEG_SIDE_RIGHT].online ? (1u << 3) : 0u) |
                  (s_wheelleg.back_fb[WHEELLEG_SIDE_RIGHT].online ? (1u << 4) : 0u) |
                  (s_wheelleg.wheel_fb[WHEELLEG_SIDE_RIGHT].online ? (1u << 5) : 0u));

    log.lqr_error[0] = s_wheelleg.leg[WHEELLEG_SIDE_RIGHT].theta - target_theta;
    log.lqr_error[1] = s_wheelleg.leg[WHEELLEG_SIDE_RIGHT].d_theta;
    log.lqr_error[2] = lqr_x_err;
    log.lqr_error[3] = s_wheelleg.v_mps - target_v;
    log.lqr_error[4] = pitch - g_config.wheelleg_mit.pitch_balance_offset_right_rad;
    log.lqr_error[5] = (gyro != NULL) ? gyro[INS_GYRO_Y_ADDRESS_OFFSET] : 0.0f;
    log.lqr_output[0] = (wheel_torque != NULL) ? wheel_torque[WHEELLEG_SIDE_RIGHT] : 0.0f;
    log.lqr_output[1] = (wheel_torque != NULL) ? wheel_torque[WHEELLEG_SIDE_LEFT] : 0.0f;
    log.lqr_output[2] = s_wheelleg.leg[WHEELLEG_SIDE_RIGHT].tp;
    log.lqr_output[3] = s_wheelleg.leg[WHEELLEG_SIDE_LEFT].tp;
    log.overrun_count = s_wheelleg.overrun_count;

    sdlog_write(SDLOG_TAG_WHEELLEG_MIT_STATUS, &log, (uint16_t)sizeof(log));
}

static MotorId wheelleg_actuator_from_u8(uint8_t id)
{
    if ((uint32_t)id >= (uint32_t)MotorCount)
    {
        return MotorCount;
    }
    return (MotorId)id;
}

static uint8_t wheelleg_feedback_fresh(const MotorState *fb, uint32_t now_ms)
{
    if (fb == NULL || fb->online == 0u)
    {
        return 0u;
    }
    return ((uint32_t)(now_ms - fb->lastRxTick) <= WHEELLEG_FEEDBACK_TIMEOUT_MS) ? 1u : 0u;
}

static uint8_t wheelleg_read_feedback(MotorId id, MotorState *out, uint32_t now_ms)
{
    if ((uint32_t)id >= (uint32_t)MotorCount || out == NULL)
    {
        return 0u;
    }
    if (LowStateGetMotor(id, out) == 0u)
    {
        return 0u;
    }
    if (wheelleg_feedback_fresh(out, now_ms) == 0u)
    {
        out->online = 0u;
        return 0u;
    }
    return 1u;
}

static void wheelleg_configure_actuators(void)
{
    const wheelleg_mit_config_t *cfg = &g_config.wheelleg_mit;

    s_wheelleg.actuator[WHEELLEG_SIDE_RIGHT].front = wheelleg_actuator_from_u8(cfg->right_front_actuator);
    s_wheelleg.actuator[WHEELLEG_SIDE_RIGHT].back = wheelleg_actuator_from_u8(cfg->right_back_actuator);
    s_wheelleg.actuator[WHEELLEG_SIDE_RIGHT].wheel = wheelleg_actuator_from_u8(cfg->right_wheel_actuator);
    s_wheelleg.actuator[WHEELLEG_SIDE_LEFT].front = wheelleg_actuator_from_u8(cfg->left_front_actuator);
    s_wheelleg.actuator[WHEELLEG_SIDE_LEFT].back = wheelleg_actuator_from_u8(cfg->left_back_actuator);
    s_wheelleg.actuator[WHEELLEG_SIDE_LEFT].wheel = wheelleg_actuator_from_u8(cfg->left_wheel_actuator);
}

static uint16_t wheelleg_update_feedback(uint32_t now_ms)
{
    uint16_t faults = WHEELLEG_FAULT_NONE;
    uint8_t side;

    for (side = 0u; side < WHEELLEG_SIDE_COUNT; side++)
    {
        const wheelleg_actuator_map_t *map = &s_wheelleg.actuator[side];
        const uint8_t front_ok = wheelleg_read_feedback(map->front, &s_wheelleg.front_fb[side], now_ms);
        const uint8_t back_ok = wheelleg_read_feedback(map->back, &s_wheelleg.back_fb[side], now_ms);
        const uint8_t wheel_ok = wheelleg_read_feedback(map->wheel, &s_wheelleg.wheel_fb[side], now_ms);

        if (side == WHEELLEG_SIDE_LEFT)
        {
            if (front_ok == 0u || back_ok == 0u)
            {
                faults |= WHEELLEG_FAULT_LEFT_LEG_OFFLINE;
            }
            if (wheel_ok == 0u)
            {
                faults |= WHEELLEG_FAULT_LEFT_WHEEL_OFFLINE;
            }
        }
        else
        {
            if (front_ok == 0u || back_ok == 0u)
            {
                faults |= WHEELLEG_FAULT_RIGHT_LEG_OFFLINE;
            }
            if (wheel_ok == 0u)
            {
                faults |= WHEELLEG_FAULT_RIGHT_WHEEL_OFFLINE;
            }
        }
    }
    return faults;
}

static uint8_t wheelleg_forward_point(fp32 front_pos,
                                      fp32 back_pos,
                                      wheelleg_foot_point_t *out)
{
    const wheelleg_core_geometry_t geo = wheelleg_core_geometry_from_config();
    wheelleg_core_foot_point_t core_out;

    if (out == NULL)
    {
        return 0u;
    }
    if (wheelleg_core_forward_point(&geo, front_pos, back_pos, &core_out) == 0u)
    {
        return 0u;
    }

    wheelleg_local_point_from_core(&core_out, out);
    return 1u;
}

static uint8_t wheelleg_calc_kinematics(wheelleg_leg_calc_t *leg,
                                        fp32 front_pos,
                                        fp32 back_pos,
                                        fp32 pitch,
                                        fp32 gyro_y,
                                        uint8_t left_side,
                                        fp32 dt)
{
    const wheelleg_core_geometry_t geo = wheelleg_core_geometry_from_config();

    return wheelleg_core_calc_kinematics(&geo, leg, front_pos, back_pos, pitch, gyro_y, left_side, dt);
}

static uint8_t wheelleg_calc_vmc(wheelleg_leg_calc_t *leg)
{
    return wheelleg_core_calc_vmc(leg);
}

static void wheelleg_fill_torque_cmd(MotorCmd *cmd, fp32 torque)
{
    control_core_cmd_set_state_torque(cmd, 0.0f, 0.0f, 0.0f, 0.0f, torque);
}

static void wheelleg_send_torque(MotorId id, fp32 torque)
{
    MotorCmd cmd;

    if ((uint32_t)id >= (uint32_t)MotorCount)
    {
        return;
    }

    wheelleg_fill_torque_cmd(&cmd, torque);
    (void)motor_instance_cmd_set_ids(&id, &cmd, 1u);
}

static uint8_t wheelleg_send_state_cmd(MotorId id,
                                       fp32 position,
                                       fp32 velocity,
                                       fp32 kp,
                                       fp32 kd,
                                       fp32 torque)
{
    MotorCmd cmd;

    if ((uint32_t)id >= (uint32_t)MotorCount)
    {
        return 0u;
    }

    control_core_cmd_set_state_torque(&cmd, position, velocity, kp, kd, torque);
    return motor_instance_cmd_set_state_torque_id(id, &cmd);
}

static void wheelleg_clear_state_cmd(MotorId id)
{
    if ((uint32_t)id >= (uint32_t)MotorCount)
    {
        return;
    }

    (void)motor_instance_cmd_clear_id(id);
}

static void wheelleg_clear_leg_virtual_outputs(void)
{
    uint8_t side;

    for (side = 0u; side < WHEELLEG_SIDE_COUNT; side++)
    {
        s_wheelleg.leg[side].f0 = 0.0f;
        s_wheelleg.leg[side].tp = 0.0f;
        s_wheelleg.leg[side].joint_torque[0] = 0.0f;
        s_wheelleg.leg[side].joint_torque[1] = 0.0f;
        s_wheelleg.leg[side].contact = 0u;
    }
}

static void wheelleg_clear_all_control_cmds(void)
{
    uint8_t side;

    for (side = 0u; side < WHEELLEG_SIDE_COUNT; side++)
    {
        const wheelleg_actuator_map_t *map = &s_wheelleg.actuator[side];
        wheelleg_clear_state_cmd(map->front);
        wheelleg_clear_state_cmd(map->back);
        wheelleg_clear_state_cmd(map->wheel);
    }
    wheelleg_clear_state_cmd(wheelleg_single_test_actuator());
}

static uint8_t wheelleg_control_stage_from_config(void)
{
    uint8_t stage = g_config.wheelleg_mit.control_stage;

    if (stage > WHEELLEG_CONTROL_STAGE_MAX)
    {
        stage = WHEELLEG_CONTROL_STAGE_MAX;
    }
    return stage;
}

static wheelleg_mode_e wheelleg_mode_from_control_stage(uint8_t stage)
{
    switch (stage)
    {
    case WHEELLEG_CONTROL_STAGE_POSITION_LQR:
        return WHEELLEG_MODE_LEG_LQR;
    case WHEELLEG_CONTROL_STAGE_VMC_HEIGHT_LQR:
        return WHEELLEG_MODE_VMC_HEIGHT;
    case WHEELLEG_CONTROL_STAGE_VMC_FULL_LQR:
        return WHEELLEG_MODE_VMC_BALANCE;
    case WHEELLEG_CONTROL_STAGE_BENCH_LQR:
    default:
        return WHEELLEG_MODE_BENCH;
    }
}

static fp32 wheelleg_lqr_wheel_scale(void)
{
    fp32 scale = g_config.wheelleg_mit.lqr_wheel_torque_scale;

    if (scale <= 0.0f)
    {
        scale = WHEELLEG_LQR_WHEEL_TORQUE_SCALE_DEFAULT;
    }
    return wheelleg_clamp(scale, 0.0f, 2.0f);
}

static fp32 wheelleg_lqr_hip_scale(void)
{
    fp32 scale = g_config.wheelleg_mit.lqr_hip_torque_scale;

    if (scale <= 0.0f)
    {
        scale = WHEELLEG_LQR_HIP_TORQUE_SCALE_DEFAULT;
    }
    return wheelleg_clamp(scale, 0.0f, 2.0f);
}

static fp32 wheelleg_lqr_length_for_side(uint8_t side, uint8_t use_measured_leg_length)
{
    fp32 length = g_config.wheelleg_mit.min_leg_length_m;
    fp32 max_leg = g_config.wheelleg_mit.max_leg_length_m;

    if (use_measured_leg_length != 0u && side < WHEELLEG_SIDE_COUNT)
    {
        length = s_wheelleg.leg[side].length;
    }
    if (length <= 0.02f)
    {
        length = wheelleg_bench_default_leg_m();
    }
    if (max_leg < 0.02f)
    {
        max_leg = 0.02f;
    }
    return wheelleg_clamp(length, 0.02f, max_leg);
}

static fp32 wheelleg_lqr_x_error(fp32 target_v, fp32 target_yaw_rate)
{
    return wheelleg_core_lqr_x_error(s_wheelleg.x_m,
                                     target_v,
                                     target_yaw_rate,
                                     WHEELLEG_LQR_X_HOLD_LIMIT_M,
                                     WHEELLEG_LQR_MOTION_EPS);
}

static void wheelleg_control_stage_update(uint8_t stage)
{
    if (s_wheelleg.control_stage == stage)
    {
        return;
    }

    wheelleg_clear_all_control_cmds();
    wheelleg_pid_clear(&s_wheelleg.leg_pid[WHEELLEG_SIDE_LEFT]);
    wheelleg_pid_clear(&s_wheelleg.leg_pid[WHEELLEG_SIDE_RIGHT]);
    wheelleg_pid_clear(&s_wheelleg.split_pid);
    wheelleg_balance_state_reset();
    wheelleg_clear_leg_virtual_outputs();
    s_wheelleg.yaw_inited = 0u;
    s_wheelleg.control_stage = stage;
}

static void wheelleg_clear_test_runtime_flags(void)
{
    s_wheelleg.left_test_active = 0u;
    s_wheelleg.foot_test_active = 0u;
    s_wheelleg.detached_test_active = 0u;
    s_wheelleg.foot_test_phase = 0u;
    s_wheelleg.foot_test_ik_ok = 0u;
}

static void wheelleg_balance_idle_reset(int16_t vx_axis, int16_t yaw_axis, fp32 yaw)
{
    const uint8_t idle =
        (uint8_t)((wheelleg_axis_in_deadband(vx_axis, g_config.wheelleg_mit.rc_deadband) != 0u &&
                   wheelleg_axis_in_deadband(yaw_axis, g_config.wheelleg_mit.rc_deadband) != 0u)
                      ? 1u
                      : 0u);

    if (idle == 0u)
    {
        s_wheelleg.balance_idle_latched = 0u;
        return;
    }

    if (s_wheelleg.balance_idle_latched != 0u)
    {
        return;
    }

    s_wheelleg.x_m = 0.0f;
    s_wheelleg.yaw_set = yaw;
    s_wheelleg.yaw_inited = 1u;
    s_wheelleg.balance_idle_latched = 1u;
}

static MotorId wheelleg_core_actuator_to_id(wheelleg_core_actuator_e actuator)
{
    const wheelleg_actuator_map_t *right_map = &s_wheelleg.actuator[WHEELLEG_SIDE_RIGHT];
    const wheelleg_actuator_map_t *left_map = &s_wheelleg.actuator[WHEELLEG_SIDE_LEFT];

    switch (actuator)
    {
    case WHEELLEG_CORE_ACT_RIGHT_FRONT:
        return right_map->front;
    case WHEELLEG_CORE_ACT_RIGHT_BACK:
        return right_map->back;
    case WHEELLEG_CORE_ACT_RIGHT_WHEEL:
        return right_map->wheel;
    case WHEELLEG_CORE_ACT_LEFT_FRONT:
        return left_map->front;
    case WHEELLEG_CORE_ACT_LEFT_BACK:
        return left_map->back;
    case WHEELLEG_CORE_ACT_LEFT_WHEEL:
        return left_map->wheel;
    default:
        return MotorCount;
    }
}

static uint8_t wheelleg_send_core_output(const wheelleg_core_output_t *out)
{
    MotorId ids[WHEELLEG_CORE_ACTUATOR_COUNT];
    MotorCmd cmds[WHEELLEG_CORE_ACTUATOR_COUNT];
    uint8_t written = 0u;
    uint8_t i;

    if (out == NULL)
    {
        return 0u;
    }

    for (i = 0u; i < out->actuator_count && i < WHEELLEG_CORE_ACTUATOR_COUNT; i++)
    {
        MotorId id;

        if (out->actuator[i].active == 0u)
        {
            continue;
        }

        id = wheelleg_core_actuator_to_id((wheelleg_core_actuator_e)i);
        if ((uint32_t)id >= (uint32_t)MotorCount)
        {
            return 0u;
        }

        ids[written] = id;
        cmds[written] = out->actuator[i];
        written++;
    }

    if (written == 0u)
    {
        return 0u;
    }

    return motor_instance_cmd_set_state_torque_ids(ids, cmds, written);
}

static void wheelleg_send_wheel_torques(const fp32 wheel_torque[WHEELLEG_SIDE_COUNT])
{
    wheelleg_core_output_t out;

    if (wheel_torque == NULL)
    {
        return;
    }

    wheelleg_core_output_clear(&out);
    wheelleg_core_set_wheel_torques(&out, wheel_torque);
    (void)wheelleg_send_core_output(&out);
}

static void wheelleg_send_vmc_joint_torques(void)
{
    const wheelleg_mit_config_t *cfg = &g_config.wheelleg_mit;
    wheelleg_core_output_t out;

    wheelleg_core_output_clear(&out);
    wheelleg_core_set_vmc_joint_torques(&out,
                                        s_wheelleg.leg,
                                        cfg->right_front_dir,
                                        cfg->right_back_dir,
                                        cfg->left_front_dir,
                                        cfg->left_back_dir);
    (void)wheelleg_send_core_output(&out);
}

static void wheelleg_clear_joint_test_cmds(void)
{
    const wheelleg_actuator_map_t *left_map = &s_wheelleg.actuator[WHEELLEG_SIDE_LEFT];
    const wheelleg_actuator_map_t *right_map = &s_wheelleg.actuator[WHEELLEG_SIDE_RIGHT];

    wheelleg_clear_state_cmd(left_map->front);
    wheelleg_clear_state_cmd(left_map->back);
    wheelleg_clear_state_cmd(left_map->wheel);
    wheelleg_clear_state_cmd(right_map->front);
    wheelleg_clear_state_cmd(right_map->back);
    wheelleg_clear_state_cmd(right_map->wheel);
    wheelleg_clear_state_cmd(wheelleg_single_test_actuator());
}

static void wheelleg_bench_hold_prepare_default(void)
{
    const wheelleg_mit_config_t *cfg = &g_config.wheelleg_mit;

    s_wheelleg.bench_hold_front_rad[WHEELLEG_SIDE_LEFT] = cfg->left_front_zero_rad;
    s_wheelleg.bench_hold_back_rad[WHEELLEG_SIDE_LEFT] = cfg->left_back_zero_rad;
    s_wheelleg.bench_hold_front_rad[WHEELLEG_SIDE_RIGHT] = cfg->right_front_zero_rad;
    s_wheelleg.bench_hold_back_rad[WHEELLEG_SIDE_RIGHT] = cfg->right_back_zero_rad;

    s_wheelleg.bench_hold_front_valid[WHEELLEG_SIDE_LEFT] = 1u;
    s_wheelleg.bench_hold_back_valid[WHEELLEG_SIDE_LEFT] = 1u;
    s_wheelleg.bench_hold_front_valid[WHEELLEG_SIDE_RIGHT] = 1u;
    s_wheelleg.bench_hold_back_valid[WHEELLEG_SIDE_RIGHT] = 1u;

    s_wheelleg.bench_hold_target_leg_m = wheelleg_bench_default_leg_m();
    s_wheelleg.bench_hold_target_foot_x_m = WHEELLEG_BENCH_TARGET_FOOT_X_M;
    s_wheelleg.bench_hold_pose_valid = 1u;
}

static uint8_t wheelleg_bench_hold_apply_joints(void)
{
    const wheelleg_mit_config_t *cfg = &g_config.wheelleg_mit;
    const fp32 kp = (cfg->foot_test_kp > 0.0f) ? cfg->foot_test_kp : WHEELLEG_DETACHED_JOINT_TEST_KP;
    const fp32 kd = (cfg->foot_test_kd > 0.0f) ? cfg->foot_test_kd : WHEELLEG_DETACHED_JOINT_TEST_KD;
    uint8_t side;
    uint8_t sent = 0u;

    wheelleg_bench_hold_prepare_default();

    for (side = 0u; side < WHEELLEG_SIDE_COUNT; side++)
    {
        const wheelleg_actuator_map_t *map = &s_wheelleg.actuator[side];

        if (s_wheelleg.bench_hold_front_valid[side] != 0u)
        {
            sent += wheelleg_send_state_cmd(map->front,
                                            s_wheelleg.bench_hold_front_rad[side],
                                            0.0f,
                                            kp,
                                            kd,
                                            0.0f);
        }
        if (s_wheelleg.bench_hold_back_valid[side] != 0u)
        {
            sent += wheelleg_send_state_cmd(map->back,
                                            s_wheelleg.bench_hold_back_rad[side],
                                            0.0f,
                                            kp,
                                            kd,
                                            0.0f);
        }
    }

    return (sent > 0u) ? 1u : 0u;
}

static void wheelleg_bench_hold_update_pose_target(void)
{
    wheelleg_bench_hold_prepare_default();
}

static MotorId wheelleg_single_test_actuator(void)
{
    const MotorId target = robot_mode_target_motor();
    return (target != MotorCount) ? target : (MotorId)g_config.wheelleg_mit.single_test_actuator;
}

static uint8_t wheelleg_single_test_target_is_wheelleg(MotorId id)
{
    const wheelleg_mit_config_t *cfg = &g_config.wheelleg_mit;

    return (uint8_t)((id == (MotorId)cfg->left_front_actuator) ||
                     (id == (MotorId)cfg->left_back_actuator) ||
                     (id == (MotorId)cfg->left_wheel_actuator) ||
                     (id == (MotorId)cfg->right_front_actuator) ||
                     (id == (MotorId)cfg->right_back_actuator) ||
                     (id == (MotorId)cfg->right_wheel_actuator));
}

static uint8_t wheelleg_single_test_enabled(void)
{
    if (robot_mode_variant() == ROBOT_RUN_VARIANT_WHEELLEG_SINGLE_MOTOR)
    {
        return 1u;
    }

    return (uint8_t)(robot_mode_current() == ROBOT_RUN_MODE_SINGLE_MOTOR &&
                     wheelleg_single_test_target_is_wheelleg(robot_mode_target_motor()) != 0u);
}

static uint8_t wheelleg_single_test_apply(int16_t speed_axis)
{
    const wheelleg_mit_config_t *cfg = &g_config.wheelleg_mit;
    const MotorId id = wheelleg_single_test_actuator();
    const fp32 velocity = wheelleg_axis_to_fp32(speed_axis,
                                               cfg->single_test_velocity_radps,
                                               cfg->rc_deadband);
    fp32 torque_limit;
    fp32 torque;

    wheelleg_clear_joint_test_cmds();

    torque_limit = wheelleg_abs(cfg->single_test_torque_limit_nm);
    torque = (torque_limit > 0.0f)
                 ? wheelleg_clamp(cfg->single_test_torque_nm, -torque_limit, torque_limit)
                 : 0.0f;
    return wheelleg_send_state_cmd(id,
                                   cfg->single_test_position_rad,
                                   velocity,
                                   cfg->single_test_kp,
                                   cfg->single_test_kd,
                                   torque);
}

static fp32 wheelleg_lerp(fp32 start, fp32 end, uint32_t elapsed_ms, uint32_t duration_ms)
{
    return wheelleg_core_lerp(start, end, elapsed_ms, duration_ms);
}

static void wheelleg_foot_point_lerp(const wheelleg_foot_point_t *start,
                                     const wheelleg_foot_point_t *end,
                                     uint32_t elapsed_ms,
                                     uint32_t duration_ms,
                                     wheelleg_foot_point_t *out)
{
    if (start == NULL || end == NULL || out == NULL)
    {
        return;
    }

    out->x_m = wheelleg_lerp(start->x_m, end->x_m, elapsed_ms, duration_ms);
    out->y_m = wheelleg_lerp(start->y_m, end->y_m, elapsed_ms, duration_ms);
    out->length_m = sqrtf(out->x_m * out->x_m + out->y_m * out->y_m);
}

static uint8_t wheelleg_foot_point_from_length_x(fp32 length_m,
                                                 fp32 x_m,
                                                 wheelleg_foot_point_t *out)
{
    fp32 max_x;
    fp32 y_sq;

    if (out == NULL || length_m <= 0.02f)
    {
        return 0u;
    }

    max_x = length_m * 0.98f;
    x_m = wheelleg_clamp(x_m, -max_x, max_x);
    y_sq = length_m * length_m - x_m * x_m;
    if (y_sq < 0.0f)
    {
        return 0u;
    }

    out->x_m = x_m;
    out->y_m = sqrtf(y_sq);
    out->length_m = length_m;
    return 1u;
}

static uint8_t wheelleg_foot_point_from_xy(fp32 x_m,
                                           fp32 y_m,
                                           wheelleg_foot_point_t *out)
{
    fp32 length_m = 0.0f;

    if (out == NULL)
    {
        return 0u;
    }

    if (wheelleg_limit_foot_xy(&x_m, &y_m, &length_m) == 0u)
    {
        return 0u;
    }

    out->x_m = x_m;
    out->y_m = y_m;
    out->length_m = length_m;
    return 1u;
}

static uint8_t wheelleg_inverse_point(const wheelleg_foot_point_t *target,
                                      fp32 front_ref,
                                      fp32 back_ref,
                                      fp32 *front_out,
                                      fp32 *back_out)
{
    const wheelleg_core_geometry_t geo = wheelleg_core_geometry_from_config();
    wheelleg_core_foot_point_t core_target;

    if (target == NULL || front_out == NULL || back_out == NULL)
    {
        return 0u;
    }

    wheelleg_core_point_from_local(target, &core_target);
    return wheelleg_core_inverse_point(&geo, &core_target, front_ref, back_ref, front_out, back_out);
}

static uint8_t wheelleg_kinematic_zero(fp32 *front_zero, fp32 *back_zero)
{
    const wheelleg_mit_config_t *cfg = &g_config.wheelleg_mit;
    wheelleg_foot_point_t zero;
    fp32 length = cfg->min_leg_length_m;

    if (front_zero == NULL || back_zero == NULL)
    {
        return 0u;
    }
    if (length <= 0.02f)
    {
        length = cfg->default_leg_length_m;
    }
    if (cfg->max_leg_length_m > 0.02f)
    {
        length = wheelleg_clamp(length, 0.02f, cfg->max_leg_length_m);
    }

    zero.x_m = 0.0f;
    zero.y_m = length;
    zero.length_m = length;

    return wheelleg_inverse_point(&zero,
                                  -WHEELLEG_PI,
                                  -WHEELLEG_PI,
                                  front_zero,
                                  back_zero);
}

static uint8_t wheelleg_foot_test_target(const wheelleg_foot_point_t *zero,
                                         uint32_t elapsed_ms,
                                         wheelleg_foot_point_t *target,
                                         uint8_t *phase)
{
    const wheelleg_mit_config_t *cfg = &g_config.wheelleg_mit;
    const fp32 length_m = wheelleg_clamp((cfg->foot_test_length_m > 0.02f)
                                             ? cfg->foot_test_length_m
                                             : cfg->default_leg_length_m,
                                         cfg->min_leg_length_m,
                                         cfg->max_leg_length_m);
    const fp32 forward_m = wheelleg_abs(cfg->foot_test_forward_m) *
                           ((cfg->foot_test_forward_dir < 0) ? -1.0f : 1.0f);
    uint32_t zero_ms = cfg->foot_test_zero_hold_time_ms;
    uint32_t extend_ms = cfg->foot_test_extend_time_ms;
    uint32_t swing_ms = cfg->foot_test_swing_time_ms;
    uint32_t return_ms = cfg->foot_test_return_time_ms;
    uint32_t cycle_ms;
    uint32_t t;
    wheelleg_foot_point_t center;
    wheelleg_foot_point_t forward;
    wheelleg_foot_point_t backward;

    if (zero == NULL || target == NULL || phase == NULL)
    {
        return 0u;
    }
    if (extend_ms == 0u)
    {
        extend_ms = 1000u;
    }
    if (swing_ms == 0u)
    {
        swing_ms = 1000u;
    }
    if (return_ms == 0u)
    {
        return_ms = 1000u;
    }

    if (wheelleg_foot_point_from_length_x(length_m, 0.0f, &center) == 0u ||
        wheelleg_foot_point_from_length_x(length_m, forward_m, &forward) == 0u ||
        wheelleg_foot_point_from_length_x(length_m, -forward_m, &backward) == 0u)
    {
        return 0u;
    }

    cycle_ms = zero_ms + extend_ms + 3u * swing_ms + return_ms;
    if (cycle_ms == 0u)
    {
        return 0u;
    }

    t = elapsed_ms % cycle_ms;
    if (t < zero_ms)
    {
        *target = *zero;
        *phase = 0u;
        return 1u;
    }
    t -= zero_ms;
    if (t < extend_ms)
    {
        wheelleg_foot_point_lerp(zero, &center, t, extend_ms, target);
        *phase = 1u;
        return 1u;
    }
    t -= extend_ms;
    if (t < swing_ms)
    {
        wheelleg_foot_point_lerp(&center, &forward, t, swing_ms, target);
        *phase = 2u;
        return 1u;
    }
    t -= swing_ms;
    if (t < swing_ms)
    {
        wheelleg_foot_point_lerp(&forward, &backward, t, swing_ms, target);
        *phase = 3u;
        return 1u;
    }
    t -= swing_ms;
    if (t < swing_ms)
    {
        wheelleg_foot_point_lerp(&backward, &center, t, swing_ms, target);
        *phase = 4u;
        return 1u;
    }
    t -= swing_ms;
    wheelleg_foot_point_lerp(&center, zero, t, return_ms, target);
    *phase = 5u;
    return 1u;
}

static void wheelleg_foot_test_reset_wheel_zero(void)
{
    uint8_t side;

    for (side = 0u; side < WHEELLEG_SIDE_COUNT; side++)
    {
        s_wheelleg.foot_test_wheel_zero_rad[side] = 0.0f;
        s_wheelleg.foot_test_wheel_zero_valid[side] = 0u;
        s_wheelleg.foot_test_wheel_dx_m[side] = 0.0f;
        s_wheelleg.foot_test_wheel_comp_rad[side] = 0.0f;
        s_wheelleg.foot_test_wheel_target_rad[side] = 0.0f;
    }
}

static fp32 wheelleg_foot_test_ground_x(const wheelleg_foot_point_t *point,
                                        uint8_t side,
                                        fp32 pitch)
{
    const fp32 pitch_side = (side == WHEELLEG_SIDE_LEFT) ? -pitch : pitch;

    if (point == NULL)
    {
        return 0.0f;
    }
    return point->x_m * cosf(pitch_side) - point->y_m * sinf(pitch_side);
}

static uint8_t wheelleg_foot_test_apply_wheel(const wheelleg_actuator_map_t *map,
                                              uint8_t side,
                                              const wheelleg_foot_point_t *zero,
                                              const wheelleg_foot_point_t *target,
                                              fp32 pitch,
                                              fp32 kp,
                                              fp32 kd)
{
    fp32 radius;
    fp32 dx;
    fp32 wheel_target;

    if (map == NULL || zero == NULL || target == NULL || side >= WHEELLEG_SIDE_COUNT)
    {
        return 0u;
    }

    radius = g_config.wheelleg_mit.wheel_radius_m;
    if (radius <= 0.001f)
    {
        return 0u;
    }

    dx = wheelleg_foot_test_ground_x(target, side, pitch) -
         wheelleg_foot_test_ground_x(zero, side, pitch);
    s_wheelleg.foot_test_wheel_dx_m[side] = dx;
    s_wheelleg.foot_test_wheel_comp_rad[side] = dx / radius;

    if ((uint32_t)map->wheel >= (uint32_t)MotorCount)
    {
        s_wheelleg.foot_test_wheel_target_rad[side] = s_wheelleg.foot_test_wheel_comp_rad[side];
        return 1u;
    }

    if (s_wheelleg.foot_test_wheel_zero_valid[side] == 0u)
    {
        s_wheelleg.foot_test_wheel_zero_rad[side] =
            (s_wheelleg.wheel_fb[side].online != 0u) ? s_wheelleg.wheel_fb[side].q : 0.0f;
        s_wheelleg.foot_test_wheel_zero_valid[side] = 1u;
    }

    wheel_target = s_wheelleg.foot_test_wheel_zero_rad[side] +
                   s_wheelleg.foot_test_wheel_comp_rad[side];
    s_wheelleg.foot_test_wheel_target_rad[side] = wheel_target;
    return wheelleg_send_state_cmd(map->wheel, wheel_target, 0.0f, kp, kd, 0.0f);
}

static uint8_t wheelleg_foot_test_apply_leg(const wheelleg_actuator_map_t *map,
                                            uint8_t side,
                                            fp32 front_raw_zero,
                                            fp32 back_raw_zero,
                                            int8_t front_dir,
                                            int8_t back_dir,
                                            uint32_t elapsed_ms,
                                            fp32 pitch,
                                            fp32 kp,
                                            fp32 kd,
                                            fp32 torque)
{
    wheelleg_foot_point_t zero;
    wheelleg_foot_point_t target;
    fp32 front_kin_zero;
    fp32 back_kin_zero;
    fp32 front_target;
    fp32 back_target;
    uint8_t phase = 0u;

    if (map == NULL)
    {
        return 0u;
    }
    if (wheelleg_kinematic_zero(&front_kin_zero, &back_kin_zero) == 0u)
    {
        return 0u;
    }

    zero.x_m = 0.0f;
    zero.y_m = g_config.wheelleg_mit.min_leg_length_m;
    zero.length_m = zero.y_m;
    if (zero.length_m <= 0.02f)
    {
        zero.y_m = g_config.wheelleg_mit.default_leg_length_m;
        zero.length_m = zero.y_m;
    }

    if (wheelleg_foot_test_target(&zero, elapsed_ms, &target, &phase) == 0u ||
        wheelleg_inverse_point(&target, front_kin_zero, back_kin_zero, &front_target, &back_target) == 0u)
    {
        return 0u;
    }

    if (side < WHEELLEG_SIDE_COUNT)
    {
        s_wheelleg.foot_test_target[side] = target;
    }
    s_wheelleg.foot_test_phase = phase;
    if (wheelleg_send_state_cmd(map->front,
                                wheelleg_kinematic_to_raw(front_target,
                                                          front_raw_zero,
                                                          front_dir,
                                                          front_kin_zero),
                                0.0f,
                                kp,
                                kd,
                                torque) == 0u ||
        wheelleg_send_state_cmd(map->back,
                                wheelleg_kinematic_to_raw(back_target,
                                                          back_raw_zero,
                                                          back_dir,
                                                          back_kin_zero),
                                0.0f,
                                kp,
                                kd,
                                torque) == 0u)
    {
        return 0u;
    }
    return wheelleg_foot_test_apply_wheel(map, side, &zero, &target, pitch, kp, kd);
}

static uint8_t wheelleg_foot_test_apply(uint32_t now_ms, fp32 pitch)
{
    const wheelleg_mit_config_t *cfg = &g_config.wheelleg_mit;
    const wheelleg_actuator_map_t *left_map = &s_wheelleg.actuator[WHEELLEG_SIDE_LEFT];
    const wheelleg_actuator_map_t *right_map = &s_wheelleg.actuator[WHEELLEG_SIDE_RIGHT];
    fp32 torque_limit;
    fp32 torque;
    uint32_t elapsed_ms;
    uint8_t ok_left;
    uint8_t ok_right;

    if (s_wheelleg.foot_test_active == 0u)
    {
        s_wheelleg.foot_test_start_ms = now_ms;
        s_wheelleg.foot_test_active = 1u;
        wheelleg_foot_test_reset_wheel_zero();
    }

    wheelleg_clear_joint_test_cmds();

    torque_limit = wheelleg_abs(cfg->foot_test_torque_limit_nm);
    torque = (torque_limit > 0.0f)
                 ? wheelleg_clamp(cfg->foot_test_torque_ff_nm, -torque_limit, torque_limit)
                 : 0.0f;
    elapsed_ms = now_ms - s_wheelleg.foot_test_start_ms;
    s_wheelleg.foot_test_ik_ok = 0u;

    ok_left = wheelleg_foot_test_apply_leg(left_map,
                                           WHEELLEG_SIDE_LEFT,
                                           cfg->left_front_zero_rad,
                                           cfg->left_back_zero_rad,
                                           cfg->left_front_dir,
                                           cfg->left_back_dir,
                                           elapsed_ms,
                                           pitch,
                                           cfg->foot_test_kp,
                                           cfg->foot_test_kd,
                                           torque);
    ok_right = wheelleg_foot_test_apply_leg(right_map,
                                            WHEELLEG_SIDE_RIGHT,
                                            cfg->right_front_zero_rad,
                                            cfg->right_back_zero_rad,
                                            cfg->right_front_dir,
                                            cfg->right_back_dir,
                                            elapsed_ms,
                                            pitch,
                                            cfg->foot_test_kp,
                                            cfg->foot_test_kd,
                                            torque);
    if (ok_left == 0u || ok_right == 0u)
    {
        wheelleg_clear_joint_test_cmds();
        return 0u;
    }

    s_wheelleg.foot_test_ik_ok = 1u;
    return 1u;
}

static fp32 wheelleg_manual_y_target_from_axis(int16_t axis)
{
    fp32 center = g_config.wheelleg_mit.default_leg_length_m;
    fp32 min_leg = g_config.wheelleg_mit.min_leg_length_m;
    fp32 max_leg = g_config.wheelleg_mit.max_leg_length_m;
    fp32 u;

    if (min_leg <= 0.02f)
    {
        min_leg = 0.02f;
    }
    if (max_leg < min_leg)
    {
        max_leg = min_leg;
    }
    center = wheelleg_clamp(center, min_leg, max_leg);

    if (wheelleg_axis_in_deadband(axis, g_config.wheelleg_mit.rc_deadband) != 0u)
    {
        return center;
    }

    u = wheelleg_clamp(((fp32)axis) / (fp32)RC_CH_VALUE_ABS_LEGACY, -1.0f, 1.0f);
    if (u >= 0.0f)
    {
        return center + u * (max_leg - center);
    }
    return center + u * (center - min_leg);
}

static fp32 wheelleg_manual_x_target_from_axis(int16_t axis)
{
    return wheelleg_axis_to_fp32(axis,
                                WHEELLEG_MANUAL_FOOT_X_RANGE_M,
                                g_config.wheelleg_mit.rc_deadband);
}

static uint8_t wheelleg_position_apply_leg(const wheelleg_actuator_map_t *map,
                                           uint8_t side,
                                           fp32 front_raw_zero,
                                           fp32 back_raw_zero,
                                           int8_t front_dir,
                                           int8_t back_dir,
                                           fp32 target_foot_x,
                                           fp32 target_foot_y,
                                           fp32 kp,
                                           fp32 kd)
{
    wheelleg_foot_point_t target;
    fp32 front_kin_zero;
    fp32 back_kin_zero;
    fp32 front_target;
    fp32 back_target;

    if (map == NULL)
    {
        return 0u;
    }
    if (wheelleg_kinematic_zero(&front_kin_zero, &back_kin_zero) == 0u ||
        wheelleg_foot_point_from_xy(target_foot_x, target_foot_y, &target) == 0u ||
        wheelleg_inverse_point(&target, front_kin_zero, back_kin_zero, &front_target, &back_target) == 0u)
    {
        return 0u;
    }

    if (side < WHEELLEG_SIDE_COUNT)
    {
        s_wheelleg.foot_test_target[side] = target;
    }

    if (wheelleg_send_state_cmd(map->front,
                                wheelleg_kinematic_to_raw(front_target,
                                                          front_raw_zero,
                                                          front_dir,
                                                          front_kin_zero),
                                0.0f,
                                kp,
                                kd,
                                0.0f) == 0u ||
        wheelleg_send_state_cmd(map->back,
                                wheelleg_kinematic_to_raw(back_target,
                                                          back_raw_zero,
                                                          back_dir,
                                                          back_kin_zero),
                                0.0f,
                                kp,
                                kd,
                                0.0f) == 0u)
    {
        return 0u;
    }

    wheelleg_clear_state_cmd(map->wheel);
    return 1u;
}

static uint8_t wheelleg_manual_position_apply(fp32 target_foot_x, fp32 target_foot_y)
{
    const wheelleg_mit_config_t *cfg = &g_config.wheelleg_mit;
    const wheelleg_actuator_map_t *left_map = &s_wheelleg.actuator[WHEELLEG_SIDE_LEFT];
    const wheelleg_actuator_map_t *right_map = &s_wheelleg.actuator[WHEELLEG_SIDE_RIGHT];
    const fp32 kp = (cfg->foot_test_kp > 0.0f) ? cfg->foot_test_kp : WHEELLEG_DETACHED_JOINT_TEST_KP;
    const fp32 kd = (cfg->foot_test_kd > 0.0f) ? cfg->foot_test_kd : WHEELLEG_DETACHED_JOINT_TEST_KD;
    uint8_t ok_left;
    uint8_t ok_right;

    s_wheelleg.foot_test_ik_ok = 0u;

    ok_left = wheelleg_position_apply_leg(left_map,
                                          WHEELLEG_SIDE_LEFT,
                                          cfg->left_front_zero_rad,
                                          cfg->left_back_zero_rad,
                                          cfg->left_front_dir,
                                          cfg->left_back_dir,
                                          target_foot_x,
                                          target_foot_y,
                                          kp,
                                          kd);
    ok_right = wheelleg_position_apply_leg(right_map,
                                           WHEELLEG_SIDE_RIGHT,
                                           cfg->right_front_zero_rad,
                                           cfg->right_back_zero_rad,
                                           cfg->right_front_dir,
                                           cfg->right_back_dir,
                                           target_foot_x,
                                           target_foot_y,
                                           kp,
                                           kd);
    if (ok_left == 0u || ok_right == 0u)
    {
        wheelleg_clear_joint_test_cmds();
        return 0u;
    }

    s_wheelleg.foot_test_ik_ok = 1u;
    return 1u;
}

static fp32 wheelleg_left_leg_test_target(uint32_t elapsed_ms)
{
    const wheelleg_mit_config_t *cfg = &g_config.wheelleg_mit;
    const uint32_t zero_ms = cfg->left_test_zero_time_ms;
    uint32_t move_ms = cfg->left_test_move_time_ms;
    uint32_t cycle_ms;
    uint32_t t;
    const fp32 angle = cfg->left_test_angle_rad;

    if (move_ms == 0u)
    {
        move_ms = 1000u;
    }
    cycle_ms = zero_ms + 4u * move_ms;
    if (cycle_ms == 0u)
    {
        return 0.0f;
    }

    t = elapsed_ms % cycle_ms;
    if (t < zero_ms)
    {
        return 0.0f;
    }
    t -= zero_ms;
    if (t < move_ms)
    {
        return wheelleg_lerp(0.0f, angle, t, move_ms);
    }
    t -= move_ms;
    if (t < move_ms)
    {
        return wheelleg_lerp(angle, 0.0f, t, move_ms);
    }
    t -= move_ms;
    if (t < move_ms)
    {
        return wheelleg_lerp(0.0f, -angle, t, move_ms);
    }
    t -= move_ms;
    return wheelleg_lerp(-angle, 0.0f, t, move_ms);
}

static uint8_t wheelleg_left_leg_test_apply(uint32_t now_ms)
{
    const wheelleg_mit_config_t *cfg = &g_config.wheelleg_mit;
    const wheelleg_actuator_map_t *left_map = &s_wheelleg.actuator[WHEELLEG_SIDE_LEFT];
    const wheelleg_actuator_map_t *right_map = &s_wheelleg.actuator[WHEELLEG_SIDE_RIGHT];
    fp32 torque_limit;
    fp32 torque;
    fp32 target;
    uint8_t sent = 0u;

    if (s_wheelleg.left_test_active == 0u)
    {
        s_wheelleg.left_test_start_ms = now_ms;
        s_wheelleg.left_test_active = 1u;
    }

    wheelleg_clear_joint_test_cmds();

    torque_limit = wheelleg_abs(cfg->left_test_torque_limit_nm);
    torque = (torque_limit > 0.0f)
                 ? wheelleg_clamp(cfg->left_test_torque_ff_nm, -torque_limit, torque_limit)
                 : 0.0f;
    target = wheelleg_left_leg_test_target(now_ms - s_wheelleg.left_test_start_ms);

    if (cfg->left_test_front_dir != 0)
    {
        if (wheelleg_send_state_cmd(left_map->front,
                                    wheelleg_joint_to_raw(target,
                                                          cfg->left_front_zero_rad,
                                                          cfg->left_test_front_dir),
                                    0.0f,
                                    cfg->left_test_kp,
                                    cfg->left_test_kd,
                                    torque) == 0u)
        {
            return 0u;
        }
        sent = 1u;
    }
    if (cfg->left_test_back_dir != 0)
    {
        if (wheelleg_send_state_cmd(left_map->back,
                                    wheelleg_joint_to_raw(target,
                                                          cfg->left_back_zero_rad,
                                                          cfg->left_test_back_dir),
                                    0.0f,
                                    cfg->left_test_kp,
                                    cfg->left_test_kd,
                                    torque) == 0u)
        {
            return 0u;
        }
        sent = 1u;
    }
    if (cfg->right_test_front_dir != 0)
    {
        if (wheelleg_send_state_cmd(right_map->front,
                                    wheelleg_joint_to_raw(target,
                                                          cfg->right_front_zero_rad,
                                                          cfg->right_test_front_dir),
                                    0.0f,
                                    cfg->left_test_kp,
                                    cfg->left_test_kd,
                                    torque) == 0u)
        {
            return 0u;
        }
        sent = 1u;
    }
    if (cfg->right_test_back_dir != 0)
    {
        if (wheelleg_send_state_cmd(right_map->back,
                                    wheelleg_joint_to_raw(target,
                                                          cfg->right_back_zero_rad,
                                                          cfg->right_test_back_dir),
                                    0.0f,
                                    cfg->left_test_kp,
                                    cfg->left_test_kd,
                                    torque) == 0u)
        {
            return 0u;
        }
        sent = 1u;
    }

    return sent;
}

static fp32 wheelleg_axis_to_fp32(int16_t axis, fp32 max_abs, uint16_t deadband)
{
    return wheelleg_core_axis_to_fp32(axis, max_abs, deadband, RC_CH_VALUE_ABS_LEGACY);
}

static void wheelleg_update_observer(fp32 dt, const fp32 gyro[3])
{
    wheelleg_leg_calc_t *left = &s_wheelleg.leg[WHEELLEG_SIDE_LEFT];
    wheelleg_leg_calc_t *right = &s_wheelleg.leg[WHEELLEG_SIDE_RIGHT];
    wheelleg_core_observer_t observer;
    const fp32 lpf = wheelleg_clamp(g_config.wheelleg_mit.observer_lpf, 0.01f, 1.0f);
    fp32 gyro_y = (gyro != NULL) ? gyro[INS_GYRO_Y_ADDRESS_OFFSET] : 0.0f;

    observer.x_m = s_wheelleg.x_m;
    observer.v_mps = s_wheelleg.v_mps;
    wheelleg_core_observer_update(&observer,
                                  left,
                                  right,
                                  s_wheelleg.wheel_fb[WHEELLEG_SIDE_LEFT].dq,
                                  s_wheelleg.wheel_fb[WHEELLEG_SIDE_RIGHT].dq,
                                  g_config.wheelleg_mit.wheel_radius_m,
                                  gyro_y,
                                  lpf,
                                  dt);
    s_wheelleg.x_m = observer.x_m;
    s_wheelleg.v_mps = observer.v_mps;
}

static void wheelleg_publish(uint16_t faults,
                             wheelleg_mode_e mode,
                             fp32 pitch,
                             fp32 roll,
                             fp32 yaw,
                             const fp32 gyro[3],
                             fp32 target_v,
                             fp32 target_leg,
                             fp32 target_foot_x,
                             fp32 target_yaw_rate,
                             const fp32 wheel_torque[WHEELLEG_SIDE_COUNT],
                             uint8_t controller_active)
{
    wheelleg_state_t state;
    wheelleg_status_t status;
    wheelleg_debug_t debug;
    static uint32_t seq;
    const uint32_t now_ms = wheelleg_tick_ms();
    const fp32 target_theta = wheelleg_target_theta_from_foot_x(target_foot_x, target_leg);
    const fp32 target_foot_y_sq = target_leg * target_leg - target_foot_x * target_foot_x;
    const fp32 target_foot_y = (target_foot_y_sq > 0.0f) ? sqrtf(target_foot_y_sq) : 0.0f;
    const fp32 lqr_x_err = wheelleg_lqr_x_error(target_v, target_yaw_rate);
    uint8_t side;

    (void)memset(&state, 0, sizeof(state));
    (void)memset(&status, 0, sizeof(status));
    (void)memset(&debug, 0, sizeof(debug));
    msg_header_init(&state.header, MSG_SOURCE_AUTONOMY, (uint16_t)sizeof(state), now_ms, seq);
    msg_header_init(&status.header, MSG_SOURCE_AUTONOMY, (uint16_t)sizeof(status), now_ms, seq);
    msg_header_init(&debug.header, MSG_SOURCE_AUTONOMY, (uint16_t)sizeof(debug), now_ms, seq);
    seq++;

    state.pitch_rad = pitch;
    state.roll_rad = roll;
    state.yaw_rad = yaw;
    if (gyro != NULL)
    {
        state.d_roll_radps = gyro[INS_GYRO_X_ADDRESS_OFFSET];
        state.d_pitch_radps = gyro[INS_GYRO_Y_ADDRESS_OFFSET];
        state.d_yaw_radps = gyro[INS_GYRO_Z_ADDRESS_OFFSET];
    }
    state.x_m = s_wheelleg.x_m;
    state.x_dot_mps = s_wheelleg.v_mps;

    status.mode = (uint8_t)mode;
    status.last_mode = (uint8_t)s_wheelleg.last_mode;
    status.fault_flags = faults;
    status.health = (faults == WHEELLEG_FAULT_NONE) ? (uint8_t)MSG_HEALTH_OK : (uint8_t)MSG_HEALTH_FAULT;
    status.controller_active = controller_active;
    status.active_controller_id = s_wheelleg.control_stage;
    status.target_v_mps = target_v;
    status.target_yaw_rate_radps = target_yaw_rate;
    status.target_leg_length_m = target_leg;
    status.target_foot_x_m = target_foot_x;
    status.target_foot_y_m = target_foot_y;
    status.target_leg_theta_rad = target_theta;
    status.pitch_rad = pitch;
    status.x_dot_mps = s_wheelleg.v_mps;

    for (side = 0u; side < WHEELLEG_SIDE_COUNT; side++)
    {
        const wheelleg_leg_calc_t *leg = &s_wheelleg.leg[side];
        const MotorState *wheel_fb = &s_wheelleg.wheel_fb[side];

        state.leg[side].length_m = leg->length;
        state.leg[side].theta_rad = leg->theta;
        state.leg[side].d_length_mps = leg->d_length;
        state.leg[side].d_theta_radps = leg->d_theta;
        state.leg[side].support_force_n = leg->f0;
        state.leg[side].hip_torque_nm = leg->tp;
        state.leg[side].joint_torque_nm[0] = leg->joint_torque[0];
        state.leg[side].joint_torque_nm[1] = leg->joint_torque[1];
        state.leg[side].contact = leg->contact;
        state.leg[side].motor_online[0] = s_wheelleg.front_fb[side].online;
        state.leg[side].motor_online[1] = s_wheelleg.back_fb[side].online;
        state.wheel_pos_rad[side] = wheel_fb->q;
        state.wheel_vel_radps[side] = wheel_fb->dq;
        state.wheel_torque_nm[side] = wheel_torque[side];
        state.wheel_online[side] = wheel_fb->online;

        status.leg_length_m[side] = leg->length;
        status.leg_theta_rad[side] = leg->theta;
        status.leg_alpha_rad[side] = leg->alpha;
        status.support_force_n[side] = leg->f0;
        status.wheel_torque_nm[side] = wheel_torque[side];

        debug.vmc[side].length_m = leg->length;
        debug.vmc[side].theta_rad = leg->theta;
        debug.vmc[side].d_length_mps = leg->d_length;
        debug.vmc[side].d_theta_radps = leg->d_theta;
        debug.vmc[side].support_force_n = leg->f0;
        debug.vmc[side].virtual_torque_nm = leg->tp;
        debug.vmc[side].joint_torque_nm[0] = leg->joint_torque[0];
        debug.vmc[side].joint_torque_nm[1] = leg->joint_torque[1];
    }

    debug.lqr.ref[0] = target_v;
    debug.lqr.ref[1] = target_leg;
    debug.lqr.ref[2] = target_yaw_rate;
    debug.lqr.ref[3] = target_foot_x;
    debug.lqr.ref[4] = target_theta;
    debug.lqr.error[0] = s_wheelleg.leg[WHEELLEG_SIDE_RIGHT].theta - target_theta;
    debug.lqr.error[1] = s_wheelleg.leg[WHEELLEG_SIDE_RIGHT].d_theta;
    debug.lqr.error[2] = lqr_x_err;
    debug.lqr.error[3] = s_wheelleg.v_mps - target_v;
    debug.lqr.error[4] = pitch - g_config.wheelleg_mit.pitch_balance_offset_right_rad;
    debug.lqr.error[5] = (gyro != NULL) ? gyro[INS_GYRO_Y_ADDRESS_OFFSET] : 0.0f;
    debug.lqr.state[0] = s_wheelleg.leg[WHEELLEG_SIDE_RIGHT].theta;
    debug.lqr.state[1] = s_wheelleg.leg[WHEELLEG_SIDE_RIGHT].d_theta;
    debug.lqr.state[2] = s_wheelleg.x_m;
    debug.lqr.state[3] = s_wheelleg.v_mps;
    debug.lqr.state[4] = pitch;
    debug.lqr.state[5] = (gyro != NULL) ? gyro[INS_GYRO_Y_ADDRESS_OFFSET] : 0.0f;
    debug.lqr.output[0] = wheel_torque[WHEELLEG_SIDE_RIGHT];
    debug.lqr.output[1] = wheel_torque[WHEELLEG_SIDE_LEFT];
    debug.lqr.output[2] = s_wheelleg.leg[WHEELLEG_SIDE_RIGHT].tp;
    debug.lqr.output[3] = s_wheelleg.leg[WHEELLEG_SIDE_LEFT].tp;
    debug.observer_x_m = s_wheelleg.x_m;
    debug.observer_v_mps = s_wheelleg.v_mps;
    debug.overrun_count = s_wheelleg.overrun_count;

    (void)wheelleg_state_write(&state);
    (void)wheelleg_status_write(&status);
    (void)wheelleg_debug_write(&debug);

    wheelleg_sdlog_write_status(faults,
                                mode,
                                pitch,
                                roll,
                                yaw,
                                gyro,
                                target_v,
                                target_leg,
                                target_foot_x,
                                target_yaw_rate,
                                target_theta,
                                wheel_torque,
                                controller_active,
                                now_ms);
}

uint8_t wheelleg_update_leg_kinematics(fp32 pitch, const fp32 gyro[3], fp32 dt)
{
    fp32 front_kin_zero;
    fp32 back_kin_zero;
    fp32 right_front_pos;
    fp32 right_back_pos;
    fp32 left_front_pos;
    fp32 left_back_pos;

    if (wheelleg_kinematic_zero(&front_kin_zero, &back_kin_zero) == 0u)
    {
        return 0u;
    }

    right_front_pos = wheelleg_raw_to_kinematic(s_wheelleg.front_fb[WHEELLEG_SIDE_RIGHT].q,
                                                g_config.wheelleg_mit.right_front_zero_rad,
                                                g_config.wheelleg_mit.right_front_dir,
                                                front_kin_zero);
    right_back_pos = wheelleg_raw_to_kinematic(s_wheelleg.back_fb[WHEELLEG_SIDE_RIGHT].q,
                                               g_config.wheelleg_mit.right_back_zero_rad,
                                               g_config.wheelleg_mit.right_back_dir,
                                               back_kin_zero);
    left_front_pos = wheelleg_raw_to_kinematic(s_wheelleg.front_fb[WHEELLEG_SIDE_LEFT].q,
                                               g_config.wheelleg_mit.left_front_zero_rad,
                                               g_config.wheelleg_mit.left_front_dir,
                                               front_kin_zero);
    left_back_pos = wheelleg_raw_to_kinematic(s_wheelleg.back_fb[WHEELLEG_SIDE_LEFT].q,
                                              g_config.wheelleg_mit.left_back_zero_rad,
                                              g_config.wheelleg_mit.left_back_dir,
                                              back_kin_zero);

    if (wheelleg_calc_kinematics(&s_wheelleg.leg[WHEELLEG_SIDE_RIGHT],
                                 right_front_pos,
                                 right_back_pos,
                                 pitch,
                                 (gyro != NULL) ? gyro[INS_GYRO_Y_ADDRESS_OFFSET] : 0.0f,
                                 0u,
                                 dt) == 0u ||
        wheelleg_calc_kinematics(&s_wheelleg.leg[WHEELLEG_SIDE_LEFT],
                                 left_front_pos,
                                 left_back_pos,
                                 pitch,
                                 (gyro != NULL) ? gyro[INS_GYRO_Y_ADDRESS_OFFSET] : 0.0f,
                                 1u,
                                 dt) == 0u)
    {
        return 0u;
    }
    return 1u;
}

// 板凳模型：关节按位置撑住，轮端只跑 LQR，不发 VMC 关节力矩。
uint8_t wheelleg_bench_lqr_step(fp32 dt,
                                fp32 pitch,
                                fp32 yaw,
                                const fp32 gyro[3],
                                fp32 target_v,
                                fp32 target_leg,
                                fp32 target_foot_x,
                                fp32 target_yaw_rate,
                                uint8_t use_measured_leg_length,
                                fp32 wheel_torque[WHEELLEG_SIDE_COUNT])
{
    fp32 k_right[12];
    fp32 k_left[12];
    fp32 right_pitch = pitch;
    fp32 right_gyro = (gyro != NULL) ? gyro[INS_GYRO_Y_ADDRESS_OFFSET] : 0.0f;
    fp32 left_pitch = -pitch;
    fp32 left_gyro = -right_gyro;
    fp32 yaw_gyro = (gyro != NULL) ? gyro[INS_GYRO_Z_ADDRESS_OFFSET] : 0.0f;
    fp32 turn_t;
    fp32 x_err_r;
    fp32 v_err_r;
    fp32 x_err_l;
    fp32 v_err_l;
    fp32 right_theta_err;
    fp32 left_theta_err;
    const fp32 target_theta = wheelleg_target_theta_from_foot_x(target_foot_x, target_leg);
    const fp32 lqr_length_right =
        wheelleg_lqr_length_for_side(WHEELLEG_SIDE_RIGHT, use_measured_leg_length);
    const fp32 lqr_length_left =
        wheelleg_lqr_length_for_side(WHEELLEG_SIDE_LEFT, use_measured_leg_length);
    const fp32 wheel_scale = wheelleg_lqr_wheel_scale();
    uint8_t i;

    if (wheel_torque == NULL)
    {
        return 0u;
    }

    wheelleg_eval_lqr(lqr_length_right, k_right);
    wheelleg_eval_lqr(lqr_length_left, k_left);

    if (s_wheelleg.yaw_inited == 0u)
    {
        s_wheelleg.yaw_set = yaw;
        s_wheelleg.last_yaw = yaw;
        s_wheelleg.yaw_inited = 1u;
    }
    s_wheelleg.yaw_set += target_yaw_rate * dt;
    turn_t = wheelleg_core_turn_torque(s_wheelleg.yaw_set,
                                       yaw,
                                       yaw_gyro,
                                       g_config.wheelleg_mit.turn_pid.kp,
                                       g_config.wheelleg_mit.turn_pid.kd,
                                       g_config.wheelleg_mit.turn_pid.max_out);

    right_theta_err = s_wheelleg.leg[WHEELLEG_SIDE_RIGHT].theta - target_theta;
    left_theta_err = s_wheelleg.leg[WHEELLEG_SIDE_LEFT].theta - target_theta;

    x_err_r = wheelleg_lqr_x_error(target_v, target_yaw_rate);
    v_err_r = s_wheelleg.v_mps - target_v;
    wheel_torque[WHEELLEG_SIDE_RIGHT] =
        wheelleg_core_lqr_wheel_output(k_right,
                                       right_theta_err,
                                       s_wheelleg.leg[WHEELLEG_SIDE_RIGHT].d_theta,
                                       x_err_r,
                                       v_err_r,
                                       right_pitch - g_config.wheelleg_mit.pitch_balance_offset_right_rad,
                                       right_gyro);

    x_err_l = -x_err_r;
    v_err_l = target_v - s_wheelleg.v_mps;
    wheel_torque[WHEELLEG_SIDE_LEFT] =
        wheelleg_core_lqr_wheel_output(k_left,
                                       left_theta_err,
                                       s_wheelleg.leg[WHEELLEG_SIDE_LEFT].d_theta,
                                       x_err_l,
                                       v_err_l,
                                       left_pitch - g_config.wheelleg_mit.pitch_balance_offset_left_rad,
                                       left_gyro);

    for (i = 0u; i < WHEELLEG_SIDE_COUNT; i++)
    {
        wheel_torque[i] = wheel_torque[i] * wheel_scale - turn_t;
        wheel_torque[i] = wheelleg_clamp(wheel_torque[i],
                                         -g_config.wheelleg_mit.max_wheel_torque_nm,
                                         g_config.wheelleg_mit.max_wheel_torque_nm);
        s_wheelleg.leg[i].f0 = 0.0f;
        s_wheelleg.leg[i].tp = 0.0f;
        s_wheelleg.leg[i].joint_torque[0] = 0.0f;
        s_wheelleg.leg[i].joint_torque[1] = 0.0f;
        s_wheelleg.leg[i].contact = 0u;
    }
    return 1u;
}

static uint8_t wheelleg_controller_step(fp32 dt,
                                        fp32 pitch,
                                        fp32 roll,
                                        fp32 yaw,
                                        const fp32 gyro[3],
                                        fp32 target_v,
                                        fp32 target_leg,
                                        fp32 target_foot_x,
                                        fp32 target_yaw_rate,
                                        uint8_t use_lqr_hip,
                                        fp32 wheel_torque[WHEELLEG_SIDE_COUNT])
{
    fp32 k_right[12];
    fp32 k_left[12];
    fp32 right_pitch = pitch;
    fp32 right_gyro = (gyro != NULL) ? gyro[INS_GYRO_Y_ADDRESS_OFFSET] : 0.0f;
    fp32 left_pitch = -pitch;
    fp32 left_gyro = -right_gyro;
    fp32 yaw_gyro = (gyro != NULL) ? gyro[INS_GYRO_Z_ADDRESS_OFFSET] : 0.0f;
    fp32 roll_gyro = (gyro != NULL) ? gyro[INS_GYRO_X_ADDRESS_OFFSET] : 0.0f;
    fp32 turn_t;
    fp32 roll_f0;
    fp32 split_tp;
    fp32 x_err_r;
    fp32 v_err_r;
    fp32 x_err_l;
    fp32 v_err_l;
    fp32 cos_theta;
    const fp32 target_theta = wheelleg_target_theta_from_foot_x(target_foot_x, target_leg);
    const fp32 wheel_scale = wheelleg_lqr_wheel_scale();
    const fp32 hip_scale = (use_lqr_hip != 0u) ? wheelleg_lqr_hip_scale() : 0.0f;
    fp32 right_theta_err;
    fp32 left_theta_err;
    uint8_t i;

    if (wheel_torque == NULL)
    {
        return 0u;
    }

    wheelleg_eval_lqr(s_wheelleg.leg[WHEELLEG_SIDE_RIGHT].length, k_right);
    wheelleg_eval_lqr(s_wheelleg.leg[WHEELLEG_SIDE_LEFT].length, k_left);

    if (s_wheelleg.yaw_inited == 0u)
    {
        s_wheelleg.yaw_set = yaw;
        s_wheelleg.last_yaw = yaw;
        s_wheelleg.yaw_inited = 1u;
    }
    s_wheelleg.yaw_set += target_yaw_rate * dt;
    turn_t = wheelleg_core_turn_torque(s_wheelleg.yaw_set,
                                       yaw,
                                       yaw_gyro,
                                       g_config.wheelleg_mit.turn_pid.kp,
                                       g_config.wheelleg_mit.turn_pid.kd,
                                       g_config.wheelleg_mit.turn_pid.max_out);

    roll_f0 = wheelleg_core_roll_force(0.0f,
                                       roll,
                                       roll_gyro,
                                       g_config.wheelleg_mit.roll_pid.kp,
                                       g_config.wheelleg_mit.roll_pid.kd,
                                       g_config.wheelleg_mit.roll_pid.max_out);

    right_theta_err = s_wheelleg.leg[WHEELLEG_SIDE_RIGHT].theta - target_theta;
    left_theta_err = s_wheelleg.leg[WHEELLEG_SIDE_LEFT].theta - target_theta;
    if (use_lqr_hip != 0u)
    {
        split_tp = wheelleg_pid_calc(&s_wheelleg.split_pid,
                                     s_wheelleg.leg[WHEELLEG_SIDE_RIGHT].theta +
                                         s_wheelleg.leg[WHEELLEG_SIDE_LEFT].theta,
                                     2.0f * target_theta);
    }
    else
    {
        split_tp = 0.0f;
        wheelleg_pid_clear(&s_wheelleg.split_pid);
    }

    x_err_r = wheelleg_lqr_x_error(target_v, target_yaw_rate);
    v_err_r = s_wheelleg.v_mps - target_v;
    wheel_torque[WHEELLEG_SIDE_RIGHT] =
        wheelleg_core_lqr_wheel_output(k_right,
                                       right_theta_err,
                                       s_wheelleg.leg[WHEELLEG_SIDE_RIGHT].d_theta,
                                       x_err_r,
                                       v_err_r,
                                       right_pitch - g_config.wheelleg_mit.pitch_balance_offset_right_rad,
                                       right_gyro);
    s_wheelleg.leg[WHEELLEG_SIDE_RIGHT].tp = 0.0f;
    if (use_lqr_hip != 0u)
    {
        s_wheelleg.leg[WHEELLEG_SIDE_RIGHT].tp =
            hip_scale * wheelleg_core_lqr_hip_output(k_right,
                                                     right_theta_err,
                                                     s_wheelleg.leg[WHEELLEG_SIDE_RIGHT].d_theta,
                                                     x_err_r,
                                                     v_err_r,
                                                     right_pitch - g_config.wheelleg_mit.pitch_balance_offset_right_rad,
                                                     right_gyro,
                                                     split_tp);
    }

    x_err_l = -x_err_r;
    v_err_l = target_v - s_wheelleg.v_mps;
    wheel_torque[WHEELLEG_SIDE_LEFT] =
        wheelleg_core_lqr_wheel_output(k_left,
                                       left_theta_err,
                                       s_wheelleg.leg[WHEELLEG_SIDE_LEFT].d_theta,
                                       x_err_l,
                                       v_err_l,
                                       left_pitch - g_config.wheelleg_mit.pitch_balance_offset_left_rad,
                                       left_gyro);
    s_wheelleg.leg[WHEELLEG_SIDE_LEFT].tp = 0.0f;
    if (use_lqr_hip != 0u)
    {
        s_wheelleg.leg[WHEELLEG_SIDE_LEFT].tp =
            hip_scale * wheelleg_core_lqr_hip_output(k_left,
                                                     left_theta_err,
                                                     s_wheelleg.leg[WHEELLEG_SIDE_LEFT].d_theta,
                                                     x_err_l,
                                                     v_err_l,
                                                     left_pitch - g_config.wheelleg_mit.pitch_balance_offset_left_rad,
                                                     left_gyro,
                                                     split_tp);
    }

    for (i = 0u; i < WHEELLEG_SIDE_COUNT; i++)
    {
        cos_theta = cosf(s_wheelleg.leg[i].theta);
        if (wheelleg_abs(cos_theta) < 0.1f)
        {
            cos_theta = (cos_theta >= 0.0f) ? 0.1f : -0.1f;
        }
        s_wheelleg.leg[i].f0 = g_config.wheelleg_mit.support_bias_n / cos_theta +
                               wheelleg_pid_calc(&s_wheelleg.leg_pid[i],
                                                 s_wheelleg.leg[i].length,
                                                 target_leg);
        if (i == WHEELLEG_SIDE_RIGHT)
        {
            s_wheelleg.leg[i].f0 += roll_f0;
        }
        else
        {
            s_wheelleg.leg[i].f0 -= roll_f0;
        }
        s_wheelleg.leg[i].f0 = wheelleg_clamp(s_wheelleg.leg[i].f0,
                                              -g_config.wheelleg_mit.max_support_force_n,
                                              g_config.wheelleg_mit.max_support_force_n);
        s_wheelleg.leg[i].tp = wheelleg_clamp(s_wheelleg.leg[i].tp,
                                              -g_config.wheelleg_mit.max_joint_torque_nm,
                                              g_config.wheelleg_mit.max_joint_torque_nm);
        wheel_torque[i] = wheel_torque[i] * wheel_scale - turn_t;
        wheel_torque[i] = wheelleg_clamp(wheel_torque[i],
                                         -g_config.wheelleg_mit.max_wheel_torque_nm,
                                         g_config.wheelleg_mit.max_wheel_torque_nm);
        s_wheelleg.leg[i].contact = (s_wheelleg.leg[i].f0 * cosf(s_wheelleg.leg[i].theta) > 3.0f) ? 1u : 0u;
        if (wheelleg_calc_vmc(&s_wheelleg.leg[i]) == 0u)
        {
            return 0u;
        }
        s_wheelleg.leg[i].joint_torque[0] =
            wheelleg_clamp(s_wheelleg.leg[i].joint_torque[0],
                           -g_config.wheelleg_mit.max_joint_torque_nm,
                           g_config.wheelleg_mit.max_joint_torque_nm);
        s_wheelleg.leg[i].joint_torque[1] =
            wheelleg_clamp(s_wheelleg.leg[i].joint_torque[1],
                           -g_config.wheelleg_mit.max_joint_torque_nm,
                           g_config.wheelleg_mit.max_joint_torque_nm);
    }

    return 1u;
}

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
        const uint64_t profiler_start_us = rt_profiler_begin();
        watch_task_beat(WATCH_TASK_WHEELLEG_MIT);
        const uint32_t loop_start = wheelleg_tick_ms();
        const uint16_t period_ms = wheelleg_period_ms();
        const fp32 dt = wheelleg_period_s();
        const uint32_t now_ms = wheelleg_tick_ms();
        const fp32 *angle = get_INS_angle_point();
        const fp32 *gyro = get_gyro_data_point();
        fp32 gyro_aligned[3] = {0.0f, 0.0f, 0.0f};
        const fp32 *gyro_wheelleg = wheelleg_imu_gyro_aligned(gyro, gyro_aligned);
        const fp32 pitch = wheelleg_imu_angle(angle, WHEELLEG_IMU_BODY_PITCH_OFFSET, WHEELLEG_IMU_PITCH_SIGN);
        const fp32 roll = wheelleg_imu_angle(angle, WHEELLEG_IMU_BODY_ROLL_OFFSET, WHEELLEG_IMU_ROLL_SIGN);
        const fp32 yaw = wheelleg_imu_angle(angle, WHEELLEG_IMU_BODY_YAW_OFFSET, WHEELLEG_IMU_YAW_SIGN);
        const int16_t vx_axis = input_axis(INPUT_AXIS_CHASSIS_X);
        const int16_t yaw_axis = input_axis(INPUT_AXIS_CHASSIS_WZ);
        const int16_t single_test_axis = input_axis(INPUT_AXIS_CALIB_3);
        const int16_t leg_axis = input_axis(INPUT_AXIS_CHASSIS_Y);
        const uint8_t control_stage = wheelleg_control_stage_from_config();
        fp32 target_v = 0.0f;
        fp32 target_yaw_rate = 0.0f;
        fp32 target_leg = 0.100f;
        fp32 target_foot_x = 0.0f;
        fp32 target_foot_y = 0.100f;
        fp32 wheel_torque[WHEELLEG_SIDE_COUNT] = {0.0f, 0.0f};
        uint16_t faults = WHEELLEG_FAULT_NONE;
        const uint8_t profile_on = robot_profile_is_wheelleg_mit();
        const uint8_t manual_on = wheelleg_manual_enabled_by_switch();
        const uint8_t imu_ok =
            (toe_is_error(BOARD_GYRO_TOE) == 0u && toe_is_error(BOARD_ACCEL_TOE) == 0u) ? 1u : 0u;
        const uint8_t single_test = wheelleg_single_test_enabled();
        const uint8_t left_leg_test = robot_mode_wheelleg_left_leg_swing();
        const uint8_t foot_test = robot_mode_wheelleg_foot_trajectory();
        const uint8_t operation_test_active =
            (single_test != 0u || left_leg_test != 0u || foot_test != 0u) ? 1u : 0u;
        uint8_t enabled;
        uint8_t controller_active = 0u;
        uint8_t kinematics_ok = 0u;
        uint16_t feedback_faults;

        wheelleg_configure_actuators();
        wheelleg_pid_apply(&s_wheelleg.leg_pid[WHEELLEG_SIDE_LEFT], &g_config.wheelleg_mit.leg_length_pid);
        wheelleg_pid_apply(&s_wheelleg.leg_pid[WHEELLEG_SIDE_RIGHT], &g_config.wheelleg_mit.leg_length_pid);
        wheelleg_pid_apply(&s_wheelleg.split_pid, &g_config.wheelleg_mit.leg_split_pid);
        wheelleg_control_stage_update(control_stage);

        feedback_faults = wheelleg_update_feedback(now_ms);
        s_wheelleg.feedback_faults = feedback_faults;
        if (toe_is_error(DBUS_TOE))
        {
            faults |= WHEELLEG_FAULT_MANUAL_OFFLINE;
        }
        if (operation_test_active == 0u)
        {
            if (imu_ok == 0u)
            {
                faults |= WHEELLEG_FAULT_IMU_OFFLINE;
            }
            if (wheelleg_abs(pitch) > g_config.wheelleg_mit.attitude_limit_rad ||
                wheelleg_abs(roll) > g_config.wheelleg_mit.attitude_limit_rad)
            {
                faults |= WHEELLEG_FAULT_ATTITUDE_LIMIT;
            }
        }
        if (profile_on != 0u &&
            feedback_faults == WHEELLEG_FAULT_NONE)
        {
            kinematics_ok = wheelleg_update_leg_kinematics((imu_ok != 0u) ? pitch : 0.0f,
                                                           (imu_ok != 0u) ? gyro_wheelleg : NULL,
                                                           dt);
        }

        enabled = (uint8_t)(profile_on != 0u &&
                            manual_on != 0u &&
                            faults == WHEELLEG_FAULT_NONE);

        s_wheelleg.last_mode = s_wheelleg.mode;
        if (enabled == 0u)
        {
            s_wheelleg.mode = (faults == WHEELLEG_FAULT_NONE) ? WHEELLEG_MODE_DISABLED : WHEELLEG_MODE_FAULT;
            if (profile_on != 0u)
            {
                wheelleg_clear_all_control_cmds();
            }
            wheelleg_pid_clear(&s_wheelleg.leg_pid[WHEELLEG_SIDE_LEFT]);
            wheelleg_pid_clear(&s_wheelleg.leg_pid[WHEELLEG_SIDE_RIGHT]);
            wheelleg_pid_clear(&s_wheelleg.split_pid);
            wheelleg_balance_state_reset();
            s_wheelleg.yaw_inited = 0u;
            s_wheelleg.left_test_active = 0u;
            s_wheelleg.foot_test_active = 0u;
            s_wheelleg.detached_test_active = 0u;
            s_wheelleg.foot_test_phase = 0u;
            s_wheelleg.foot_test_ik_ok = 0u;
            target_leg = s_wheelleg.bench_hold_target_leg_m;
            target_foot_x = s_wheelleg.bench_hold_target_foot_x_m;
            wheelleg_publish(faults, s_wheelleg.mode, pitch, roll, yaw, gyro_wheelleg,
                             target_v, target_leg, target_foot_x, target_yaw_rate, wheel_torque, 0u);
            rt_profiler_end(RT_PROFILER_WHEELLEG_MIT_CONTROL_LOOP, profiler_start_us);
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(period_ms));
            continue;
        }

        if (single_test == 0u)
        {
            wheelleg_clear_state_cmd(wheelleg_single_test_actuator());
        }

        if (single_test != 0u)
        {
            wheelleg_balance_state_reset();
            s_wheelleg.left_test_active = 0u;
            s_wheelleg.foot_test_active = 0u;
            s_wheelleg.detached_test_active = 0u;
            s_wheelleg.foot_test_phase = 0u;
            s_wheelleg.foot_test_ik_ok = 0u;
            s_wheelleg.mode = WHEELLEG_MODE_CALIBRATION;
            if (wheelleg_single_test_apply(single_test_axis) != 0u)
            {
                controller_active = 1u;
                s_wheelleg.ever_commanded = 1u;
            }
            else
            {
                faults |= WHEELLEG_FAULT_CONTROLLER;
                s_wheelleg.mode = WHEELLEG_MODE_FAULT;
            }
            wheelleg_publish(faults,
                             s_wheelleg.mode,
                             pitch,
                             roll,
                             yaw,
                             gyro_wheelleg,
                             target_v,
                             target_leg,
                             target_foot_x,
                             target_yaw_rate,
                             wheel_torque,
                             controller_active);
            if ((uint32_t)(wheelleg_tick_ms() - loop_start) > period_ms)
            {
                s_wheelleg.overrun_count++;
            }
            rt_profiler_end(RT_PROFILER_WHEELLEG_MIT_CONTROL_LOOP, profiler_start_us);
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(period_ms));
            continue;
        }

        if (left_leg_test != 0u)
        {
            wheelleg_balance_state_reset();
            s_wheelleg.foot_test_active = 0u;
            s_wheelleg.detached_test_active = 0u;
            s_wheelleg.foot_test_phase = 0u;
            s_wheelleg.foot_test_ik_ok = 0u;
            s_wheelleg.mode = WHEELLEG_MODE_CALIBRATION;
            if (wheelleg_left_leg_test_apply(now_ms) != 0u)
            {
                controller_active = 1u;
                s_wheelleg.ever_commanded = 1u;
            }
            else
            {
                faults |= WHEELLEG_FAULT_CONTROLLER;
                s_wheelleg.mode = WHEELLEG_MODE_FAULT;
            }
            wheelleg_publish(faults,
                             s_wheelleg.mode,
                             pitch,
                             roll,
                             yaw,
                             gyro_wheelleg,
                             target_v,
                             target_leg,
                             target_foot_x,
                             target_yaw_rate,
                             wheel_torque,
                             controller_active);
            if ((uint32_t)(wheelleg_tick_ms() - loop_start) > period_ms)
            {
                s_wheelleg.overrun_count++;
            }
            rt_profiler_end(RT_PROFILER_WHEELLEG_MIT_CONTROL_LOOP, profiler_start_us);
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(period_ms));
            continue;
        }

        if (foot_test != 0u)
        {
            wheelleg_balance_state_reset();
            s_wheelleg.left_test_active = 0u;
            s_wheelleg.detached_test_active = 0u;
            s_wheelleg.mode = WHEELLEG_MODE_CALIBRATION;
            if (wheelleg_foot_test_apply(now_ms, pitch) != 0u)
            {
                controller_active = 1u;
                s_wheelleg.ever_commanded = 1u;
            }
            else
            {
                faults |= WHEELLEG_FAULT_CONTROLLER;
                s_wheelleg.mode = WHEELLEG_MODE_FAULT;
            }
            wheelleg_publish(faults,
                             s_wheelleg.mode,
                             pitch,
                             roll,
                             yaw,
                             gyro_wheelleg,
                             target_v,
                             target_leg,
                             target_foot_x,
                             target_yaw_rate,
                             wheel_torque,
                             controller_active);
            if ((uint32_t)(wheelleg_tick_ms() - loop_start) > period_ms)
            {
                s_wheelleg.overrun_count++;
            }
            rt_profiler_end(RT_PROFILER_WHEELLEG_MIT_CONTROL_LOOP, profiler_start_us);
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(period_ms));
            continue;
        }

        wheelleg_clear_test_runtime_flags();
        s_wheelleg.mode = wheelleg_mode_from_control_stage(control_stage);
        target_v = wheelleg_axis_to_fp32(vx_axis,
                                         g_config.wheelleg_mit.max_v_mps,
                                         g_config.wheelleg_mit.rc_deadband);
        target_yaw_rate = -wheelleg_axis_to_fp32(yaw_axis,
                                                 g_config.wheelleg_mit.max_yaw_rate_radps,
                                                 g_config.wheelleg_mit.rc_deadband);

        if (control_stage == WHEELLEG_CONTROL_STAGE_BENCH_LQR)
        {
            uint8_t joint_hold_ok;
            uint8_t wheel_lqr_ok = 0u;

            wheelleg_pid_clear(&s_wheelleg.leg_pid[WHEELLEG_SIDE_LEFT]);
            wheelleg_pid_clear(&s_wheelleg.leg_pid[WHEELLEG_SIDE_RIGHT]);
            wheelleg_pid_clear(&s_wheelleg.split_pid);
            joint_hold_ok = wheelleg_bench_hold_apply_joints();
            target_leg = s_wheelleg.bench_hold_target_leg_m;
            target_foot_x = s_wheelleg.bench_hold_target_foot_x_m;

            if ((faults & WHEELLEG_FAULT_CONTROLLER) == 0u &&
                feedback_faults == WHEELLEG_FAULT_NONE)
            {
                if (kinematics_ok != 0u)
                {
                    wheelleg_bench_hold_update_pose_target();
                    target_leg = s_wheelleg.bench_hold_target_leg_m;
                    target_foot_x = s_wheelleg.bench_hold_target_foot_x_m;
                    wheelleg_update_observer(dt, gyro_wheelleg);
                    wheelleg_balance_idle_reset(vx_axis, yaw_axis, yaw);
                    if (wheelleg_bench_lqr_step(dt,
                                                pitch,
                                                yaw,
                                                gyro_wheelleg,
                                                target_v,
                                                target_leg,
                                                target_foot_x,
                                                target_yaw_rate,
                                                0u,
                                                wheel_torque) != 0u)
                    {
                        wheel_lqr_ok = 1u;
                    }
                    else
                    {
                        faults |= WHEELLEG_FAULT_CONTROLLER;
                    }
                }
                else
                {
                    faults |= WHEELLEG_FAULT_CONTROLLER;
                }
            }

            if ((faults & WHEELLEG_FAULT_CONTROLLER) == 0u)
            {
                controller_active = (uint8_t)((joint_hold_ok != 0u || wheel_lqr_ok != 0u) ? 1u : 0u);
                if (wheel_lqr_ok == 0u)
                {
                    wheel_torque[WHEELLEG_SIDE_RIGHT] = 0.0f;
                    wheel_torque[WHEELLEG_SIDE_LEFT] = 0.0f;
                }
                wheelleg_send_wheel_torques(wheel_torque);
            }
        }
        else if (control_stage == WHEELLEG_CONTROL_STAGE_POSITION_LQR)
        {
            uint8_t position_ok = 0u;
            uint8_t wheel_lqr_ok = 0u;

            wheelleg_pid_clear(&s_wheelleg.leg_pid[WHEELLEG_SIDE_LEFT]);
            wheelleg_pid_clear(&s_wheelleg.leg_pid[WHEELLEG_SIDE_RIGHT]);
            wheelleg_pid_clear(&s_wheelleg.split_pid);
            wheelleg_clear_leg_virtual_outputs();
            target_foot_x = wheelleg_manual_x_target_from_axis(0);
            target_foot_y = wheelleg_manual_y_target_from_axis(leg_axis);
            if (wheelleg_limit_foot_xy(&target_foot_x, &target_foot_y, &target_leg) == 0u)
            {
                faults |= WHEELLEG_FAULT_CONTROLLER;
            }
            if (feedback_faults != WHEELLEG_FAULT_NONE)
            {
                faults |= feedback_faults;
            }
            if (faults == WHEELLEG_FAULT_NONE && kinematics_ok == 0u)
            {
                faults |= WHEELLEG_FAULT_CONTROLLER;
            }

            if (faults == WHEELLEG_FAULT_NONE)
            {
                wheelleg_target_smooth_update_xy(target_foot_x, target_foot_y, dt);
                target_leg = s_wheelleg.target_leg_smooth;
                target_foot_x = s_wheelleg.target_foot_x_smooth;
                target_foot_y = s_wheelleg.target_foot_y_smooth;
                if (wheelleg_manual_position_apply(target_foot_x, target_foot_y) != 0u)
                {
                    position_ok = 1u;
                    wheelleg_update_observer(dt, gyro_wheelleg);
                    wheelleg_balance_idle_reset(vx_axis, yaw_axis, yaw);
                    if (wheelleg_bench_lqr_step(dt,
                                                pitch,
                                                yaw,
                                                gyro_wheelleg,
                                                target_v,
                                                target_leg,
                                                target_foot_x,
                                                target_yaw_rate,
                                                1u,
                                                wheel_torque) != 0u)
                    {
                        wheel_lqr_ok = 1u;
                    }
                    else
                    {
                        faults |= WHEELLEG_FAULT_CONTROLLER;
                    }
                }
                else
                {
                    faults |= WHEELLEG_FAULT_CONTROLLER;
                }
            }

            if (faults == WHEELLEG_FAULT_NONE)
            {
                controller_active = (uint8_t)((position_ok != 0u || wheel_lqr_ok != 0u) ? 1u : 0u);
                wheelleg_send_wheel_torques(wheel_torque);
            }
        }
        else
        {
            const uint8_t use_lqr_hip =
                (control_stage == WHEELLEG_CONTROL_STAGE_VMC_FULL_LQR) ? 1u : 0u;

            target_foot_x = wheelleg_manual_x_target_from_axis(0);
            target_foot_y = wheelleg_manual_y_target_from_axis(leg_axis);
            if (wheelleg_limit_foot_xy(&target_foot_x, &target_foot_y, &target_leg) == 0u)
            {
                faults |= WHEELLEG_FAULT_CONTROLLER;
            }
            if (feedback_faults != WHEELLEG_FAULT_NONE)
            {
                faults |= feedback_faults;
            }
            if (faults == WHEELLEG_FAULT_NONE && kinematics_ok == 0u)
            {
                faults |= WHEELLEG_FAULT_CONTROLLER;
            }

            if (faults == WHEELLEG_FAULT_NONE)
            {
                wheelleg_target_smooth_update_xy(target_foot_x, target_foot_y, dt);
                target_leg = s_wheelleg.target_leg_smooth;
                target_foot_x = s_wheelleg.target_foot_x_smooth;
                target_foot_y = s_wheelleg.target_foot_y_smooth;
                wheelleg_update_observer(dt, gyro_wheelleg);
                wheelleg_balance_idle_reset(vx_axis, yaw_axis, yaw);
                if (wheelleg_controller_step(dt,
                                             pitch,
                                             roll,
                                             yaw,
                                             gyro_wheelleg,
                                             target_v,
                                             target_leg,
                                             target_foot_x,
                                             target_yaw_rate,
                                             use_lqr_hip,
                                             wheel_torque) != 0u)
                {
                    controller_active = 1u;
                    wheelleg_send_wheel_torques(wheel_torque);
                    wheelleg_send_vmc_joint_torques();
                }
                else
                {
                    faults |= WHEELLEG_FAULT_CONTROLLER;
                }
            }
        }

        if (faults != WHEELLEG_FAULT_NONE)
        {
            s_wheelleg.mode = WHEELLEG_MODE_FAULT;
            wheelleg_target_smooth_reset();
            wheelleg_clear_all_control_cmds();
            controller_active = 0u;
        }
        else if (controller_active != 0u)
        {
            s_wheelleg.ever_commanded = 1u;
        }

        wheelleg_publish((uint16_t)(faults | feedback_faults),
                         s_wheelleg.mode,
                         pitch,
                         roll,
                         yaw,
                         gyro_wheelleg,
                         target_v,
                         target_leg,
                         target_foot_x,
                         target_yaw_rate,
                         wheel_torque,
                         controller_active);

        if ((uint32_t)(wheelleg_tick_ms() - loop_start) > period_ms)
        {
            s_wheelleg.overrun_count++;
        }
        rt_profiler_end(RT_PROFILER_WHEELLEG_MIT_CONTROL_LOOP, profiler_start_us);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(period_ms));
    }
}
