/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 */

#include "wheelleg_mit_task.h"

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"

#include "INS_task.h"
#include "actuator_cmd.h"
#include "config.h"
#include "control_input.h"
#include "detect_task.h"
#include "robot_msg.h"
#include "robot_task_profile.h"
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
#define WHEELLEG_DETACHED_JOINT_STAGE_MS 2000u
#define WHEELLEG_DETACHED_JOINT_TEST_KP 2.0f
#define WHEELLEG_DETACHED_JOINT_TEST_KD 0.16f
#define WHEELLEG_BENCH_LQR_TORQUE_SCALE 0.18f

typedef struct
{
    fp32 kp;
    fp32 ki;
    fp32 kd;
    fp32 max_out;
    fp32 max_iout;
    fp32 iout;
    fp32 last_err;
} wheelleg_pid_t;

typedef struct
{
    actuator_id_e front;
    actuator_id_e back;
    actuator_id_e wheel;
} wheelleg_actuator_map_t;

typedef struct
{
    fp32 l1;
    fp32 l2;
    fp32 l3;
    fp32 l4;
    fp32 l5;
    fp32 phi1;
    fp32 phi2;
    fp32 phi3;
    fp32 phi4;
    fp32 phi0;
    fp32 alpha;
    fp32 d_alpha;
    fp32 length;
    fp32 d_length;
    fp32 dd_length;
    fp32 theta;
    fp32 d_theta;
    fp32 dd_theta;
    fp32 f0;
    fp32 tp;
    fp32 fn;
    fp32 joint_torque[2];
    fp32 last_phi0;
    fp32 last_length;
    fp32 last_d_length;
    fp32 last_d_theta;
    uint8_t first;
    uint8_t contact;
} wheelleg_leg_calc_t;

typedef struct
{
    fp32 x_m;
    fp32 y_m;
    fp32 length_m;
} wheelleg_foot_point_t;

typedef struct
{
    wheelleg_actuator_map_t actuator[WHEELLEG_SIDE_COUNT];
    actuator_feedback_t front_fb[WHEELLEG_SIDE_COUNT];
    actuator_feedback_t back_fb[WHEELLEG_SIDE_COUNT];
    actuator_feedback_t wheel_fb[WHEELLEG_SIDE_COUNT];
    wheelleg_leg_calc_t leg[WHEELLEG_SIDE_COUNT];
    wheelleg_pid_t leg_pid[WHEELLEG_SIDE_COUNT];
    wheelleg_pid_t split_pid;
    fp32 x_m;
    fp32 v_mps;
    fp32 target_leg_smooth;
    fp32 target_foot_x_smooth;
    fp32 yaw_set;
    fp32 last_yaw;
    uint8_t target_smooth_valid;
    uint8_t yaw_inited;
    uint8_t ever_commanded;
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
} wheelleg_mit_ctrl_t;

static wheelleg_mit_ctrl_t s_wheelleg;

static fp32 wheelleg_axis_to_fp32(int16_t axis, fp32 max_abs, uint16_t deadband);

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
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

static fp32 wheelleg_abs(fp32 value)
{
    return (value >= 0.0f) ? value : -value;
}

static uint8_t wheelleg_axis_in_deadband(int16_t axis, uint16_t deadband)
{
    return (axis > -(int16_t)deadband && axis < (int16_t)deadband) ? 1u : 0u;
}

static fp32 wheelleg_target_theta_from_foot_x(fp32 foot_x_m, fp32 leg_length_m)
{
    if (leg_length_m < 0.02f)
    {
        leg_length_m = 0.02f;
    }
    return atan2f(foot_x_m, leg_length_m);
}

static fp32 wheelleg_slew_fp32(fp32 current, fp32 target, fp32 max_delta)
{
    const fp32 delta = target - current;

    if (delta > max_delta)
    {
        return current + max_delta;
    }
    if (delta < -max_delta)
    {
        return current - max_delta;
    }
    return target;
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

static void wheelleg_target_smooth_reset(void)
{
    uint8_t side;

    s_wheelleg.target_smooth_valid = 0u;
    s_wheelleg.target_leg_smooth = 0.0f;
    s_wheelleg.target_foot_x_smooth = 0.0f;
    s_wheelleg.auto_leg_stage = 0u;
    s_wheelleg.auto_leg_stage_tick_ms = 0u;
    s_wheelleg.bench_hold_pose_valid = 0u;
    s_wheelleg.bench_hold_target_leg_m = wheelleg_bench_default_leg_m();
    s_wheelleg.bench_hold_target_foot_x_m = 0.0f;
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
    s_wheelleg.leg[WHEELLEG_SIDE_LEFT].first = 0u;
    s_wheelleg.leg[WHEELLEG_SIDE_RIGHT].first = 0u;
}

static void wheelleg_target_smooth_update(fp32 target_leg, fp32 target_foot_x, fp32 dt)
{
    const fp32 measured_leg =
        (s_wheelleg.leg[WHEELLEG_SIDE_RIGHT].length + s_wheelleg.leg[WHEELLEG_SIDE_LEFT].length) * 0.5f;
    fp32 measured_theta =
        (s_wheelleg.leg[WHEELLEG_SIDE_RIGHT].theta + s_wheelleg.leg[WHEELLEG_SIDE_LEFT].theta) * 0.5f;

    if (dt <= 0.0f)
    {
        dt = 0.003f;
    }

    if (s_wheelleg.target_smooth_valid == 0u)
    {
        measured_theta = wheelleg_clamp(measured_theta, -1.2f, 1.2f);
        s_wheelleg.target_leg_smooth = wheelleg_clamp(measured_leg,
                                                      g_config.wheelleg_mit.min_leg_length_m,
                                                      g_config.wheelleg_mit.max_leg_length_m);
        s_wheelleg.target_foot_x_smooth = wheelleg_clamp(tanf(measured_theta) * s_wheelleg.target_leg_smooth,
                                                         -WHEELLEG_MANUAL_FOOT_X_RANGE_M,
                                                         WHEELLEG_MANUAL_FOOT_X_RANGE_M);
        s_wheelleg.target_smooth_valid = 1u;
    }

    s_wheelleg.target_leg_smooth =
        wheelleg_slew_fp32(s_wheelleg.target_leg_smooth,
                           target_leg,
                           WHEELLEG_TARGET_LEG_SLEW_MPS * dt);
    s_wheelleg.target_foot_x_smooth =
        wheelleg_slew_fp32(s_wheelleg.target_foot_x_smooth,
                           target_foot_x,
                           WHEELLEG_TARGET_FOOT_X_SLEW_MPS * dt);
}

static void wheelleg_target_smooth_seed(fp32 target_leg, fp32 target_foot_x)
{
    if (s_wheelleg.target_smooth_valid == 0u)
    {
        s_wheelleg.target_leg_smooth = target_leg;
        s_wheelleg.target_foot_x_smooth = target_foot_x;
        s_wheelleg.target_smooth_valid = 1u;
    }
}

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

static fp32 wheelleg_detached_joint_leg_target(uint32_t now_ms)
{
    uint32_t elapsed_ms;
    uint32_t phase_ms;
    uint8_t start_stage;
    uint8_t end_stage;
    fp32 start_leg;
    fp32 end_leg;
    fp32 u;

    if (s_wheelleg.detached_test_active == 0u)
    {
        s_wheelleg.detached_test_active = 1u;
        s_wheelleg.detached_test_start_ms = now_ms;
    }

    elapsed_ms = now_ms - s_wheelleg.detached_test_start_ms;
    start_stage = (uint8_t)((elapsed_ms / WHEELLEG_DETACHED_JOINT_STAGE_MS) % 3u);
    end_stage = (uint8_t)((start_stage + 1u) % 3u);
    phase_ms = elapsed_ms % WHEELLEG_DETACHED_JOINT_STAGE_MS;
    u = (fp32)phase_ms / (fp32)WHEELLEG_DETACHED_JOINT_STAGE_MS;
    start_leg = wheelleg_auto_leg_target_by_stage(start_stage);
    end_leg = wheelleg_auto_leg_target_by_stage(end_stage);
    return start_leg + (end_leg - start_leg) * u;
}

static fp32 wheelleg_dir_sign(int8_t dir)
{
    return (dir < 0) ? -1.0f : 1.0f;
}

static fp32 wheelleg_joint_to_raw(fp32 kinematics_position, fp32 zero_position, int8_t dir)
{
    return zero_position + kinematics_position * wheelleg_dir_sign(dir);
}

static fp32 wheelleg_raw_to_kinematic(fp32 raw_position,
                                      fp32 raw_zero_position,
                                      int8_t dir,
                                      fp32 kinematic_zero_position)
{
    return kinematic_zero_position +
           (raw_position - raw_zero_position) * wheelleg_dir_sign(dir);
}

static fp32 wheelleg_kinematic_to_raw(fp32 kinematic_position,
                                      fp32 raw_zero_position,
                                      int8_t dir,
                                      fp32 kinematic_zero_position)
{
    return raw_zero_position +
           (kinematic_position - kinematic_zero_position) * wheelleg_dir_sign(dir);
}

static fp32 wheelleg_wrap_pi(fp32 value)
{
    while (value > WHEELLEG_PI)
    {
        value -= WHEELLEG_TWO_PI;
    }
    while (value < -WHEELLEG_PI)
    {
        value += WHEELLEG_TWO_PI;
    }
    return value;
}

static fp32 wheelleg_near_angle(fp32 value, fp32 reference)
{
    return reference + wheelleg_wrap_pi(value - reference);
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
    pid->kp = cfg->kp;
    pid->ki = cfg->ki;
    pid->kd = cfg->kd;
    pid->max_out = cfg->max_out;
    pid->max_iout = cfg->max_iout;
}

static void wheelleg_pid_clear(wheelleg_pid_t *pid)
{
    if (pid == NULL)
    {
        return;
    }
    pid->iout = 0.0f;
    pid->last_err = 0.0f;
}

static fp32 wheelleg_pid_calc(wheelleg_pid_t *pid, fp32 ref, fp32 set)
{
    fp32 err;
    fp32 out;

    if (pid == NULL)
    {
        return 0.0f;
    }

    err = set - ref;
    pid->iout += pid->ki * err;
    pid->iout = wheelleg_clamp(pid->iout, -pid->max_iout, pid->max_iout);
    out = pid->kp * err + pid->iout + pid->kd * (err - pid->last_err);
    pid->last_err = err;
    return wheelleg_clamp(out, -pid->max_out, pid->max_out);
}

static fp32 wheelleg_poly(const fp32 coe[4], fp32 len)
{
    return ((coe[0] * len + coe[1]) * len + coe[2]) * len + coe[3];
}

static uint8_t wheelleg_lqr_row_is_zero(const fp32 coe[4])
{
    return (coe[0] == 0.0f && coe[1] == 0.0f && coe[2] == 0.0f && coe[3] == 0.0f) ? 1u : 0u;
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

static actuator_id_e wheelleg_actuator_from_u8(uint8_t id)
{
    if ((uint32_t)id >= (uint32_t)ACTUATOR_ID__COUNT)
    {
        return ACTUATOR_ID__COUNT;
    }
    return (actuator_id_e)id;
}

static uint8_t wheelleg_feedback_fresh(const actuator_feedback_t *fb, uint32_t now_ms)
{
    if (fb == NULL || fb->online == 0u)
    {
        return 0u;
    }
    return ((uint32_t)(now_ms - fb->last_rx_tick) <= WHEELLEG_FEEDBACK_TIMEOUT_MS) ? 1u : 0u;
}

static uint8_t wheelleg_read_feedback(actuator_id_e id, actuator_feedback_t *out, uint32_t now_ms)
{
    if ((uint32_t)id >= (uint32_t)ACTUATOR_ID__COUNT || out == NULL)
    {
        return 0u;
    }
    if (actuator_feedback_get_copy(id, out) == 0u)
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
    const wheelleg_mit_config_t *cfg = &g_config.wheelleg_mit;
    const fp32 l1 = cfg->l1_m;
    const fp32 l2 = cfg->l2_m;
    const fp32 l3 = cfg->l3_m;
    const fp32 l4 = cfg->l4_m;
    const fp32 l5 = cfg->l5_m;
    const fp32 phi1 = WHEELLEG_PI * 0.5f + front_pos;
    const fp32 phi4 = WHEELLEG_PI * 0.5f + back_pos;
    const fp32 xb = l1 * cosf(phi1);
    const fp32 yb = l1 * sinf(phi1);
    const fp32 xd = l5 + l4 * cosf(phi4);
    const fp32 yd = l4 * sinf(phi4);
    const fp32 lbd = sqrtf((xd - xb) * (xd - xb) + (yd - yb) * (yd - yb));
    fp32 a0;
    fp32 b0;
    fp32 c0;
    fp32 discr;
    fp32 phi2;
    fp32 xc;
    fp32 yc;

    if (out == NULL || l1 <= 0.0f || l2 <= 0.0f || l3 <= 0.0f || l4 <= 0.0f || l5 <= 0.0f)
    {
        return 0u;
    }
    if (lbd <= 0.001f)
    {
        return 0u;
    }

    a0 = 2.0f * l2 * (xd - xb);
    b0 = 2.0f * l2 * (yd - yb);
    c0 = l2 * l2 + lbd * lbd - l3 * l3;
    discr = a0 * a0 + b0 * b0 - c0 * c0;
    if (discr < -0.000001f || wheelleg_abs(a0 + c0) < 0.000001f)
    {
        return 0u;
    }
    if (discr < 0.0f)
    {
        discr = 0.0f;
    }

    phi2 = 2.0f * atan2f(b0 + sqrtf(discr), a0 + c0);
    xc = xb + l2 * cosf(phi2);
    yc = yb + l2 * sinf(phi2);
    out->x_m = xc - l5 * 0.5f;
    out->y_m = yc;
    out->length_m = sqrtf(out->x_m * out->x_m + out->y_m * out->y_m);
    return (out->length_m > 0.01f) ? 1u : 0u;
}

static uint8_t wheelleg_calc_kinematics(wheelleg_leg_calc_t *leg,
                                        fp32 front_pos,
                                        fp32 back_pos,
                                        fp32 pitch,
                                        fp32 gyro_y,
                                        uint8_t left_side,
                                        fp32 dt)
{
    fp32 xb;
    fp32 yb;
    fp32 xd;
    fp32 yd;
    fp32 lbd;
    fp32 a0;
    fp32 b0;
    fp32 c0;
    fp32 discr;
    fp32 xc;
    fp32 yc;
    fp32 length;
    fp32 pitch_side = pitch;
    fp32 gyro_side = gyro_y;

    if (leg == NULL || dt <= 0.0f)
    {
        return 0u;
    }

    if (left_side != 0u)
    {
        pitch_side = -pitch;
        gyro_side = -gyro_y;
    }

    leg->l1 = g_config.wheelleg_mit.l1_m;
    leg->l2 = g_config.wheelleg_mit.l2_m;
    leg->l3 = g_config.wheelleg_mit.l3_m;
    leg->l4 = g_config.wheelleg_mit.l4_m;
    leg->l5 = g_config.wheelleg_mit.l5_m;
    leg->phi1 = WHEELLEG_PI * 0.5f + front_pos;
    leg->phi4 = WHEELLEG_PI * 0.5f + back_pos;

    xb = leg->l1 * cosf(leg->phi1);
    yb = leg->l1 * sinf(leg->phi1);
    xd = leg->l5 + leg->l4 * cosf(leg->phi4);
    yd = leg->l4 * sinf(leg->phi4);
    lbd = sqrtf((xd - xb) * (xd - xb) + (yd - yb) * (yd - yb));
    a0 = 2.0f * leg->l2 * (xd - xb);
    b0 = 2.0f * leg->l2 * (yd - yb);
    c0 = leg->l2 * leg->l2 + lbd * lbd - leg->l3 * leg->l3;
    discr = a0 * a0 + b0 * b0 - c0 * c0;
    if (discr < 0.0f || (a0 + c0) == 0.0f)
    {
        return 0u;
    }

    leg->phi2 = 2.0f * atan2f(b0 + sqrtf(discr), a0 + c0);
    leg->phi3 = atan2f(yb - yd + leg->l2 * sinf(leg->phi2),
                       xb - xd + leg->l2 * cosf(leg->phi2));
    xc = xb + leg->l2 * cosf(leg->phi2);
    yc = yb + leg->l2 * sinf(leg->phi2);
    length = sqrtf((xc - leg->l5 * 0.5f) * (xc - leg->l5 * 0.5f) + yc * yc);
    if (length <= 0.01f)
    {
        return 0u;
    }

    leg->phi0 = atan2f(yc, xc - leg->l5 * 0.5f);
    leg->alpha = WHEELLEG_PI * 0.5f - leg->phi0;
    if (leg->first == 0u)
    {
        leg->last_phi0 = leg->phi0;
        leg->last_length = length;
        leg->last_d_length = 0.0f;
        leg->last_d_theta = 0.0f;
        leg->first = 1u;
    }

    leg->d_alpha = -(leg->phi0 - leg->last_phi0) / dt;
    leg->theta = WHEELLEG_PI * 0.5f - pitch_side - leg->phi0;
    leg->d_theta = -gyro_side + leg->d_alpha;
    leg->length = length;
    leg->d_length = (leg->length - leg->last_length) / dt;
    leg->dd_length = (leg->d_length - leg->last_d_length) / dt;
    leg->dd_theta = (leg->d_theta - leg->last_d_theta) / dt;
    leg->last_phi0 = leg->phi0;
    leg->last_length = leg->length;
    leg->last_d_length = leg->d_length;
    leg->last_d_theta = leg->d_theta;
    return 1u;
}

static uint8_t wheelleg_calc_vmc(wheelleg_leg_calc_t *leg)
{
    fp32 sin32;
    fp32 j11;
    fp32 j12;
    fp32 j21;
    fp32 j22;

    if (leg == NULL || leg->length <= 0.01f)
    {
        return 0u;
    }

    sin32 = sinf(leg->phi3 - leg->phi2);
    if (wheelleg_abs(sin32) < 0.0001f)
    {
        return 0u;
    }

    j11 = leg->l1 * sinf(leg->phi0 - leg->phi3) * sinf(leg->phi1 - leg->phi2) / sin32;
    j12 = leg->l1 * cosf(leg->phi0 - leg->phi3) * sinf(leg->phi1 - leg->phi2) / (leg->length * sin32);
    j21 = leg->l4 * sinf(leg->phi0 - leg->phi2) * sinf(leg->phi3 - leg->phi4) / sin32;
    j22 = leg->l4 * cosf(leg->phi0 - leg->phi2) * sinf(leg->phi3 - leg->phi4) / (leg->length * sin32);
    leg->joint_torque[0] = j11 * leg->f0 + j12 * leg->tp;
    leg->joint_torque[1] = j21 * leg->f0 + j22 * leg->tp;
    return 1u;
}

static void wheelleg_send_torque(actuator_id_e id, fp32 torque)
{
    actuator_cmd_t cmd;

    if ((uint32_t)id >= (uint32_t)ACTUATOR_ID__COUNT)
    {
        return;
    }

    (void)memset(&cmd, 0, sizeof(cmd));
    cmd.torque = torque;
    actuator_cmd_set_state_torque(id, &cmd);
}

static void wheelleg_send_joint_torque(actuator_id_e id, fp32 kinematic_torque, int8_t dir)
{
    wheelleg_send_torque(id, kinematic_torque * wheelleg_dir_sign(dir));
}

static uint8_t wheelleg_send_state_cmd(actuator_id_e id,
                                       fp32 position,
                                       fp32 velocity,
                                       fp32 kp,
                                       fp32 kd,
                                       fp32 torque)
{
    actuator_cmd_t cmd;

    if ((uint32_t)id >= (uint32_t)ACTUATOR_ID__COUNT)
    {
        return 0u;
    }

    (void)memset(&cmd, 0, sizeof(cmd));
    cmd.position = position;
    cmd.velocity = velocity;
    cmd.kp = kp;
    cmd.kd = kd;
    cmd.torque = torque;
    actuator_cmd_set_state_torque(id, &cmd);
    return 1u;
}

static void wheelleg_clear_state_cmd(actuator_id_e id)
{
    if ((uint32_t)id >= (uint32_t)ACTUATOR_ID__COUNT)
    {
        return;
    }

    actuator_cmd_clear(id);
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
    wheelleg_clear_state_cmd((actuator_id_e)g_config.wheelleg_mit.single_test_actuator);
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
    wheelleg_clear_state_cmd((actuator_id_e)g_config.wheelleg_mit.single_test_actuator);
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
    s_wheelleg.bench_hold_target_foot_x_m = 0.0f;
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

static uint8_t wheelleg_single_test_apply(int16_t speed_axis)
{
    const wheelleg_mit_config_t *cfg = &g_config.wheelleg_mit;
    const actuator_id_e id = (actuator_id_e)cfg->single_test_actuator;
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
    fp32 u;

    if (duration_ms == 0u || elapsed_ms >= duration_ms)
    {
        return end;
    }
    u = (fp32)elapsed_ms / (fp32)duration_ms;
    return start + (end - start) * u;
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

static uint8_t wheelleg_foot_point_from_y_x(fp32 y_m,
                                            fp32 x_m,
                                            wheelleg_foot_point_t *out)
{
    if (out == NULL || y_m <= 0.02f)
    {
        return 0u;
    }

    out->x_m = x_m;
    out->y_m = y_m;
    out->length_m = sqrtf(x_m * x_m + y_m * y_m);
    return 1u;
}

static uint8_t wheelleg_inverse_point(const wheelleg_foot_point_t *target,
                                      fp32 front_ref,
                                      fp32 back_ref,
                                      fp32 *front_out,
                                      fp32 *back_out)
{
    const wheelleg_mit_config_t *cfg = &g_config.wheelleg_mit;
    const fp32 l1 = cfg->l1_m;
    const fp32 l2 = cfg->l2_m;
    const fp32 l3 = cfg->l3_m;
    const fp32 l4 = cfg->l4_m;
    const fp32 l5 = cfg->l5_m;
    const fp32 cx = l5 * 0.5f + ((target != NULL) ? target->x_m : 0.0f);
    const fp32 cy = (target != NULL) ? target->y_m : 0.0f;
    const fp32 front_r = sqrtf(cx * cx + cy * cy);
    const fp32 back_dx = cx - l5;
    const fp32 back_r = sqrtf(back_dx * back_dx + cy * cy);
    fp32 front_cos;
    fp32 back_cos;
    fp32 front_base;
    fp32 back_base;
    fp32 front_q;
    fp32 back_q;
    fp32 front_raw[2];
    fp32 back_raw[2];
    fp32 best_front = 0.0f;
    fp32 best_back = 0.0f;
    fp32 best_score = 1000000.0f;
    uint8_t found = 0u;
    uint8_t i;
    uint8_t j;

    if (target == NULL || front_out == NULL || back_out == NULL ||
        l1 <= 0.0f || l2 <= 0.0f || l3 <= 0.0f || l4 <= 0.0f ||
        front_r <= 0.001f || back_r <= 0.001f)
    {
        return 0u;
    }

    front_cos = (l1 * l1 + front_r * front_r - l2 * l2) / (2.0f * l1 * front_r);
    back_cos = (l4 * l4 + back_r * back_r - l3 * l3) / (2.0f * l4 * back_r);
    if (front_cos > 1.0001f || front_cos < -1.0001f ||
        back_cos > 1.0001f || back_cos < -1.0001f)
    {
        return 0u;
    }

    front_cos = wheelleg_clamp(front_cos, -1.0f, 1.0f);
    back_cos = wheelleg_clamp(back_cos, -1.0f, 1.0f);
    front_base = atan2f(cy, cx);
    back_base = atan2f(cy, back_dx);
    front_q = acosf(front_cos);
    back_q = acosf(back_cos);

    front_raw[0] = wheelleg_near_angle(front_base + front_q - WHEELLEG_PI * 0.5f, front_ref);
    front_raw[1] = wheelleg_near_angle(front_base - front_q - WHEELLEG_PI * 0.5f, front_ref);
    back_raw[0] = wheelleg_near_angle(back_base + back_q - WHEELLEG_PI * 0.5f, back_ref);
    back_raw[1] = wheelleg_near_angle(back_base - back_q - WHEELLEG_PI * 0.5f, back_ref);

    for (i = 0u; i < 2u; i++)
    {
        for (j = 0u; j < 2u; j++)
        {
            wheelleg_foot_point_t check;
            fp32 err;
            fp32 score;

            if (wheelleg_forward_point(front_raw[i], back_raw[j], &check) == 0u)
            {
                continue;
            }
            err = wheelleg_abs(check.x_m - target->x_m) + wheelleg_abs(check.y_m - target->y_m);
            if (err > 0.003f)
            {
                continue;
            }

            score = wheelleg_abs(wheelleg_wrap_pi(front_raw[i] - front_ref)) +
                    wheelleg_abs(wheelleg_wrap_pi(back_raw[j] - back_ref)) +
                    err * 1000.0f;
            if (found == 0u || score < best_score)
            {
                best_score = score;
                best_front = front_raw[i];
                best_back = back_raw[j];
                found = 1u;
            }
        }
    }

    if (found == 0u)
    {
        return 0u;
    }

    *front_out = best_front;
    *back_out = best_back;
    return 1u;
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

    if ((uint32_t)map->wheel >= (uint32_t)ACTUATOR_ID__COUNT)
    {
        s_wheelleg.foot_test_wheel_target_rad[side] = s_wheelleg.foot_test_wheel_comp_rad[side];
        return 1u;
    }

    if (s_wheelleg.foot_test_wheel_zero_valid[side] == 0u)
    {
        s_wheelleg.foot_test_wheel_zero_rad[side] =
            (s_wheelleg.wheel_fb[side].online != 0u) ? s_wheelleg.wheel_fb[side].position : 0.0f;
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

static uint8_t wheelleg_joint_position_apply_side(const wheelleg_actuator_map_t *map,
                                                  uint8_t side,
                                                  fp32 front_kin_target,
                                                  fp32 back_kin_target,
                                                  fp32 front_kin_zero,
                                                  fp32 back_kin_zero,
                                                  fp32 front_raw_zero,
                                                  fp32 back_raw_zero,
                                                  int8_t front_dir,
                                                  int8_t back_dir,
                                                  fp32 kp,
                                                  fp32 kd,
                                                  uint8_t clear_wheel)
{
    uint8_t sent = 0u;

    if (map == NULL || side >= WHEELLEG_SIDE_COUNT)
    {
        return 0u;
    }

    sent += wheelleg_send_state_cmd(map->front,
                                    wheelleg_kinematic_to_raw(front_kin_target,
                                                              front_raw_zero,
                                                              front_dir,
                                                              front_kin_zero),
                                    0.0f,
                                    kp,
                                    kd,
                                    0.0f);
    sent += wheelleg_send_state_cmd(map->back,
                                    wheelleg_kinematic_to_raw(back_kin_target,
                                                              back_raw_zero,
                                                              back_dir,
                                                              back_kin_zero),
                                    0.0f,
                                    kp,
                                    kd,
                                    0.0f);
    if (clear_wheel != 0u)
    {
        wheelleg_clear_state_cmd(map->wheel);
    }

    return sent;
}

static uint8_t wheelleg_joint_position_target_apply(fp32 target_leg,
                                                    fp32 target_foot_x,
                                                    uint8_t clear_wheels,
                                                    fp32 *target_leg_out,
                                                    fp32 *target_foot_x_out)
{
    const wheelleg_mit_config_t *cfg = &g_config.wheelleg_mit;
    const wheelleg_actuator_map_t *left_map = &s_wheelleg.actuator[WHEELLEG_SIDE_LEFT];
    const wheelleg_actuator_map_t *right_map = &s_wheelleg.actuator[WHEELLEG_SIDE_RIGHT];
    wheelleg_foot_point_t target;
    fp32 front_kin_zero;
    fp32 back_kin_zero;
    fp32 front_kin_target;
    fp32 back_kin_target;
    const fp32 kp = (cfg->foot_test_kp > 0.0f) ? cfg->foot_test_kp : WHEELLEG_DETACHED_JOINT_TEST_KP;
    const fp32 kd = (cfg->foot_test_kd > 0.0f) ? cfg->foot_test_kd : WHEELLEG_DETACHED_JOINT_TEST_KD;
    uint8_t sent = 0u;

    target_leg = wheelleg_clamp(target_leg, cfg->min_leg_length_m, cfg->max_leg_length_m);
    if (target_leg_out != NULL)
    {
        *target_leg_out = target_leg;
    }
    if (target_foot_x_out != NULL)
    {
        *target_foot_x_out = target_foot_x;
    }

    if (wheelleg_foot_point_from_y_x(target_leg, target_foot_x, &target) == 0u ||
        wheelleg_kinematic_zero(&front_kin_zero, &back_kin_zero) == 0u ||
        wheelleg_inverse_point(&target,
                               front_kin_zero,
                               back_kin_zero,
                               &front_kin_target,
                               &back_kin_target) == 0u)
    {
        return 0u;
    }

    s_wheelleg.foot_test_target[WHEELLEG_SIDE_LEFT] = target;
    s_wheelleg.foot_test_target[WHEELLEG_SIDE_RIGHT] = target;
    s_wheelleg.foot_test_ik_ok = 1u;

    sent += wheelleg_joint_position_apply_side(left_map,
                                               WHEELLEG_SIDE_LEFT,
                                               front_kin_target,
                                               back_kin_target,
                                               front_kin_zero,
                                               back_kin_zero,
                                               cfg->left_front_zero_rad,
                                               cfg->left_back_zero_rad,
                                               cfg->left_front_dir,
                                               cfg->left_back_dir,
                                               kp,
                                               kd,
                                               clear_wheels);
    sent += wheelleg_joint_position_apply_side(right_map,
                                               WHEELLEG_SIDE_RIGHT,
                                               front_kin_target,
                                               back_kin_target,
                                               front_kin_zero,
                                               back_kin_zero,
                                               cfg->right_front_zero_rad,
                                               cfg->right_back_zero_rad,
                                               cfg->right_front_dir,
                                               cfg->right_back_dir,
                                               kp,
                                               kd,
                                               clear_wheels);
    return sent;
}

static uint8_t wheelleg_detached_joint_test_apply(uint32_t now_ms,
                                                  fp32 *target_leg_out,
                                                  fp32 *target_foot_x_out)
{
    return wheelleg_joint_position_target_apply(wheelleg_detached_joint_leg_target(now_ms),
                                                0.0f,
                                                1u,
                                                target_leg_out,
                                                target_foot_x_out);
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
    if (wheelleg_axis_in_deadband(axis, deadband) != 0u)
    {
        return 0.0f;
    }
    return wheelleg_clamp(((fp32)axis) / (fp32)RC_CH_VALUE_ABS_LEGACY, -1.0f, 1.0f) * max_abs;
}

static void wheelleg_update_observer(fp32 dt, const fp32 gyro[3])
{
    wheelleg_leg_calc_t *left = &s_wheelleg.leg[WHEELLEG_SIDE_LEFT];
    wheelleg_leg_calc_t *right = &s_wheelleg.leg[WHEELLEG_SIDE_RIGHT];
    const fp32 lpf = wheelleg_clamp(g_config.wheelleg_mit.observer_lpf, 0.01f, 1.0f);
    fp32 gyro_y = (gyro != NULL) ? gyro[INS_GYRO_Y_ADDRESS_OFFSET] : 0.0f;
    fp32 wr = -s_wheelleg.wheel_fb[WHEELLEG_SIDE_RIGHT].velocity - gyro_y + right->d_alpha;
    fp32 wl = -s_wheelleg.wheel_fb[WHEELLEG_SIDE_LEFT].velocity + gyro_y + left->d_alpha;
    fp32 vr = wr * g_config.wheelleg_mit.wheel_radius_m +
              right->length * right->d_theta * cosf(right->theta) +
              right->d_length * sinf(right->theta);
    fp32 vl = wl * g_config.wheelleg_mit.wheel_radius_m +
              left->length * left->d_theta * cosf(left->theta) +
              left->d_length * sinf(left->theta);
    fp32 v_meas = (vr - vl) * 0.5f;

    s_wheelleg.v_mps += lpf * (v_meas - s_wheelleg.v_mps);
    s_wheelleg.x_m += s_wheelleg.v_mps * dt;
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
    status.target_v_mps = target_v;
    status.target_leg_length_m = target_leg;
    status.target_foot_x_m = target_foot_x;
    status.target_leg_theta_rad = target_theta;
    status.pitch_rad = pitch;
    status.x_dot_mps = s_wheelleg.v_mps;

    for (side = 0u; side < WHEELLEG_SIDE_COUNT; side++)
    {
        const wheelleg_leg_calc_t *leg = &s_wheelleg.leg[side];
        const actuator_feedback_t *wheel_fb = &s_wheelleg.wheel_fb[side];

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
        state.wheel_pos_rad[side] = wheel_fb->position;
        state.wheel_vel_radps[side] = wheel_fb->velocity;
        state.wheel_torque_nm[side] = wheel_torque[side];
        state.wheel_online[side] = wheel_fb->online;

        status.leg_length_m[side] = leg->length;
        status.leg_theta_rad[side] = leg->theta;
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
    debug.lqr.error[2] = s_wheelleg.x_m;
    debug.lqr.error[3] = s_wheelleg.v_mps - 0.4f * target_v;
    debug.lqr.error[4] = pitch;
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

    right_front_pos = wheelleg_raw_to_kinematic(s_wheelleg.front_fb[WHEELLEG_SIDE_RIGHT].position,
                                                g_config.wheelleg_mit.right_front_zero_rad,
                                                g_config.wheelleg_mit.right_front_dir,
                                                front_kin_zero);
    right_back_pos = wheelleg_raw_to_kinematic(s_wheelleg.back_fb[WHEELLEG_SIDE_RIGHT].position,
                                               g_config.wheelleg_mit.right_back_zero_rad,
                                               g_config.wheelleg_mit.right_back_dir,
                                               back_kin_zero);
    left_front_pos = wheelleg_raw_to_kinematic(s_wheelleg.front_fb[WHEELLEG_SIDE_LEFT].position,
                                               g_config.wheelleg_mit.left_front_zero_rad,
                                               g_config.wheelleg_mit.left_front_dir,
                                               front_kin_zero);
    left_back_pos = wheelleg_raw_to_kinematic(s_wheelleg.back_fb[WHEELLEG_SIDE_LEFT].position,
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
    const fp32 lqr_length =
        wheelleg_clamp(g_config.wheelleg_mit.min_leg_length_m, 0.02f, g_config.wheelleg_mit.max_leg_length_m);
    uint8_t i;

    if (wheel_torque == NULL)
    {
        return 0u;
    }

    wheelleg_eval_lqr(lqr_length, k_right);
    wheelleg_eval_lqr(lqr_length, k_left);

    if (s_wheelleg.yaw_inited == 0u)
    {
        s_wheelleg.yaw_set = yaw;
        s_wheelleg.last_yaw = yaw;
        s_wheelleg.yaw_inited = 1u;
    }
    s_wheelleg.yaw_set += target_yaw_rate * dt;
    turn_t = g_config.wheelleg_mit.turn_pid.kp * wheelleg_wrap_pi(s_wheelleg.yaw_set - yaw) -
             g_config.wheelleg_mit.turn_pid.kd * yaw_gyro;
    turn_t = wheelleg_clamp(turn_t,
                            -g_config.wheelleg_mit.turn_pid.max_out,
                            g_config.wheelleg_mit.turn_pid.max_out);

    right_theta_err = s_wheelleg.leg[WHEELLEG_SIDE_RIGHT].theta - target_theta;
    left_theta_err = s_wheelleg.leg[WHEELLEG_SIDE_LEFT].theta - target_theta;

    x_err_r = s_wheelleg.x_m;
    v_err_r = s_wheelleg.v_mps - 0.4f * target_v;
    wheel_torque[WHEELLEG_SIDE_RIGHT] =
        k_right[0] * right_theta_err +
        k_right[1] * s_wheelleg.leg[WHEELLEG_SIDE_RIGHT].d_theta +
        k_right[2] * x_err_r +
        k_right[3] * v_err_r +
        k_right[4] * (right_pitch - g_config.wheelleg_mit.pitch_balance_offset_right_rad) +
        k_right[5] * right_gyro -
        turn_t;

    x_err_l = -s_wheelleg.x_m;
    v_err_l = 0.4f * target_v - s_wheelleg.v_mps;
    wheel_torque[WHEELLEG_SIDE_LEFT] =
        k_left[0] * left_theta_err +
        k_left[1] * s_wheelleg.leg[WHEELLEG_SIDE_LEFT].d_theta +
        k_left[2] * x_err_l +
        k_left[3] * v_err_l +
        k_left[4] * (left_pitch - g_config.wheelleg_mit.pitch_balance_offset_left_rad) +
        k_left[5] * left_gyro -
        turn_t;

    for (i = 0u; i < WHEELLEG_SIDE_COUNT; i++)
    {
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
    fp32 right_theta_err;
    fp32 left_theta_err;
    uint8_t i;

    wheelleg_eval_lqr(s_wheelleg.leg[WHEELLEG_SIDE_RIGHT].length, k_right);
    wheelleg_eval_lqr(s_wheelleg.leg[WHEELLEG_SIDE_LEFT].length, k_left);

    if (s_wheelleg.yaw_inited == 0u)
    {
        s_wheelleg.yaw_set = yaw;
        s_wheelleg.last_yaw = yaw;
        s_wheelleg.yaw_inited = 1u;
    }
    s_wheelleg.yaw_set += target_yaw_rate * dt;
    turn_t = g_config.wheelleg_mit.turn_pid.kp * wheelleg_wrap_pi(s_wheelleg.yaw_set - yaw) -
             g_config.wheelleg_mit.turn_pid.kd * yaw_gyro;
    turn_t = wheelleg_clamp(turn_t,
                            -g_config.wheelleg_mit.turn_pid.max_out,
                            g_config.wheelleg_mit.turn_pid.max_out);

    roll_f0 = g_config.wheelleg_mit.roll_pid.kp * (0.0f - roll) -
              g_config.wheelleg_mit.roll_pid.kd * roll_gyro;
    roll_f0 = wheelleg_clamp(roll_f0,
                             -g_config.wheelleg_mit.roll_pid.max_out,
                             g_config.wheelleg_mit.roll_pid.max_out);

    right_theta_err = s_wheelleg.leg[WHEELLEG_SIDE_RIGHT].theta - target_theta;
    left_theta_err = s_wheelleg.leg[WHEELLEG_SIDE_LEFT].theta - target_theta;
    split_tp = wheelleg_pid_calc(&s_wheelleg.split_pid,
                                 s_wheelleg.leg[WHEELLEG_SIDE_RIGHT].theta +
                                     s_wheelleg.leg[WHEELLEG_SIDE_LEFT].theta,
                                 2.0f * target_theta);

    x_err_r = s_wheelleg.x_m;
    v_err_r = s_wheelleg.v_mps - 0.4f * target_v;
    wheel_torque[WHEELLEG_SIDE_RIGHT] =
        k_right[0] * right_theta_err +
        k_right[1] * s_wheelleg.leg[WHEELLEG_SIDE_RIGHT].d_theta +
        k_right[2] * x_err_r +
        k_right[3] * v_err_r +
        k_right[4] * (right_pitch - g_config.wheelleg_mit.pitch_balance_offset_right_rad) +
        k_right[5] * right_gyro -
        turn_t;
    s_wheelleg.leg[WHEELLEG_SIDE_RIGHT].tp =
        k_right[6] * right_theta_err +
        k_right[7] * s_wheelleg.leg[WHEELLEG_SIDE_RIGHT].d_theta +
        k_right[8] * x_err_r +
        k_right[9] * v_err_r +
        k_right[10] * (right_pitch - g_config.wheelleg_mit.pitch_balance_offset_right_rad) +
        k_right[11] * right_gyro +
        split_tp;

    x_err_l = -s_wheelleg.x_m;
    v_err_l = 0.4f * target_v - s_wheelleg.v_mps;
    wheel_torque[WHEELLEG_SIDE_LEFT] =
        k_left[0] * left_theta_err +
        k_left[1] * s_wheelleg.leg[WHEELLEG_SIDE_LEFT].d_theta +
        k_left[2] * x_err_l +
        k_left[3] * v_err_l +
        k_left[4] * (left_pitch - g_config.wheelleg_mit.pitch_balance_offset_left_rad) +
        k_left[5] * left_gyro -
        turn_t;
    s_wheelleg.leg[WHEELLEG_SIDE_LEFT].tp =
        k_left[6] * left_theta_err +
        k_left[7] * s_wheelleg.leg[WHEELLEG_SIDE_LEFT].d_theta +
        k_left[8] * x_err_l +
        k_left[9] * v_err_l +
        k_left[10] * (left_pitch - g_config.wheelleg_mit.pitch_balance_offset_left_rad) +
        k_left[11] * left_gyro +
        split_tp;

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
        watch_task_beat(WATCH_TASK_WHEELLEG_MIT);
        const uint32_t loop_start = wheelleg_tick_ms();
        const uint16_t period_ms = wheelleg_period_ms();
        const fp32 dt = wheelleg_period_s();
        const uint32_t now_ms = wheelleg_tick_ms();
        const fp32 *angle = get_INS_angle_point();
        const fp32 *gyro = get_gyro_data_point();
        const fp32 pitch = (angle != NULL) ? angle[INS_PITCH_ADDRESS_OFFSET] : 0.0f;
        const fp32 roll = (angle != NULL) ? angle[INS_ROLL_ADDRESS_OFFSET] : 0.0f;
        const fp32 yaw = (angle != NULL) ? angle[INS_YAW_ADDRESS_OFFSET] : 0.0f;
        const int16_t vx_axis = input_axis(INPUT_AXIS_CHASSIS_X);
        const int16_t yaw_axis = input_axis(INPUT_AXIS_CHASSIS_WZ);
        const int16_t single_test_axis = input_axis(INPUT_AXIS_CALIB_3);
        const test_mode_e test_mode = (test_mode_e)g_config.test.mode;
        fp32 target_v = 0.0f;
        fp32 target_yaw_rate = 0.0f;
        fp32 target_leg = 0.100f;
        fp32 target_foot_x = 0.0f;
        fp32 wheel_torque[WHEELLEG_SIDE_COUNT] = {0.0f, 0.0f};
        uint16_t faults = WHEELLEG_FAULT_NONE;
        const uint8_t profile_on = robot_profile_is_wheelleg_mit();
        const uint8_t manual_on =
            control_input_switch_is_pos(input_switch(INPUT_SW_CHASSIS_MODE),
                                        g_config.wheelleg_mit.enable_switch_pos);
        const uint8_t single_test =
            (test_mode == TEST_MODE_WHEELLEG_SINGLE_MOTOR) ? 1u : 0u;
        const uint8_t left_leg_test =
            (test_mode == TEST_MODE_WHEELLEG_LEFT_LEG_SWING) ? 1u : 0u;
        const uint8_t foot_test =
            (test_mode == TEST_MODE_WHEELLEG_FOOT_TRAJECTORY) ? 1u : 0u;
        const uint8_t test_mode_active =
            (single_test != 0u || left_leg_test != 0u || foot_test != 0u) ? 1u : 0u;
        uint8_t enabled;
        uint8_t controller_active = 0u;
        uint16_t feedback_faults;

        wheelleg_configure_actuators();
        wheelleg_pid_apply(&s_wheelleg.leg_pid[WHEELLEG_SIDE_LEFT], &g_config.wheelleg_mit.leg_length_pid);
        wheelleg_pid_apply(&s_wheelleg.leg_pid[WHEELLEG_SIDE_RIGHT], &g_config.wheelleg_mit.leg_length_pid);
        wheelleg_pid_apply(&s_wheelleg.split_pid, &g_config.wheelleg_mit.leg_split_pid);

        feedback_faults = wheelleg_update_feedback(now_ms);
        if (toe_is_error(DBUS_TOE))
        {
            faults |= WHEELLEG_FAULT_MANUAL_OFFLINE;
        }
        if (test_mode_active == 0u)
        {
            if (toe_is_error(BOARD_GYRO_TOE) || toe_is_error(BOARD_ACCEL_TOE))
            {
                faults |= WHEELLEG_FAULT_IMU_OFFLINE;
            }
            if (wheelleg_abs(pitch) > g_config.wheelleg_mit.attitude_limit_rad ||
                wheelleg_abs(roll) > g_config.wheelleg_mit.attitude_limit_rad)
            {
                faults |= WHEELLEG_FAULT_ATTITUDE_LIMIT;
            }
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
            wheelleg_publish(faults, s_wheelleg.mode, pitch, roll, yaw, gyro,
                             target_v, target_leg, target_foot_x, target_yaw_rate, wheel_torque, 0u);
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(period_ms));
            continue;
        }

        if (single_test == 0u)
        {
            wheelleg_clear_state_cmd((actuator_id_e)g_config.wheelleg_mit.single_test_actuator);
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
                             gyro,
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
                             gyro,
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
                             gyro,
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
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(period_ms));
            continue;
        }

        if (test_mode_active == 0u)
        {
            const wheelleg_actuator_map_t *right_map = &s_wheelleg.actuator[WHEELLEG_SIDE_RIGHT];
            const wheelleg_actuator_map_t *left_map = &s_wheelleg.actuator[WHEELLEG_SIDE_LEFT];
            uint8_t joint_hold_ok;
            uint8_t wheel_lqr_ok = 0u;

            s_wheelleg.mode = WHEELLEG_MODE_BENCH;
            wheelleg_pid_clear(&s_wheelleg.leg_pid[WHEELLEG_SIDE_LEFT]);
            wheelleg_pid_clear(&s_wheelleg.leg_pid[WHEELLEG_SIDE_RIGHT]);
            wheelleg_pid_clear(&s_wheelleg.split_pid);
            s_wheelleg.left_test_active = 0u;
            s_wheelleg.foot_test_active = 0u;
            s_wheelleg.detached_test_active = 0u;
            s_wheelleg.foot_test_phase = 0u;
            s_wheelleg.foot_test_ik_ok = 0u;

            target_v = wheelleg_axis_to_fp32(vx_axis,
                                             g_config.wheelleg_mit.max_v_mps,
                                             g_config.wheelleg_mit.rc_deadband);
            target_yaw_rate = -wheelleg_axis_to_fp32(yaw_axis,
                                                     g_config.wheelleg_mit.max_yaw_rate_radps,
                                                     g_config.wheelleg_mit.rc_deadband);
            joint_hold_ok = wheelleg_bench_hold_apply_joints();
            target_leg = s_wheelleg.bench_hold_target_leg_m;
            target_foot_x = s_wheelleg.bench_hold_target_foot_x_m;

            if ((faults & WHEELLEG_FAULT_CONTROLLER) == 0u &&
                feedback_faults == WHEELLEG_FAULT_NONE)
            {
                if (wheelleg_update_leg_kinematics(pitch, gyro, dt) != 0u)
                {
                    wheelleg_bench_hold_update_pose_target();
                    target_leg = s_wheelleg.bench_hold_target_leg_m;
                    target_foot_x = s_wheelleg.bench_hold_target_foot_x_m;
                    wheelleg_update_observer(dt, gyro);
                    if (wheelleg_axis_in_deadband(vx_axis, g_config.wheelleg_mit.rc_deadband) != 0u &&
                        wheelleg_axis_in_deadband(yaw_axis, g_config.wheelleg_mit.rc_deadband) != 0u)
                    {
                        s_wheelleg.x_m = 0.0f;
                        s_wheelleg.yaw_set = yaw;
                    }
                    if (wheelleg_bench_lqr_step(dt,
                                                pitch,
                                                yaw,
                                                gyro,
                                                target_v,
                                                target_leg,
                                                target_foot_x,
                                                target_yaw_rate,
                                                wheel_torque) != 0u)
                    {
                        wheel_torque[WHEELLEG_SIDE_RIGHT] *= WHEELLEG_BENCH_LQR_TORQUE_SCALE;
                        wheel_torque[WHEELLEG_SIDE_LEFT] *= WHEELLEG_BENCH_LQR_TORQUE_SCALE;
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
                s_wheelleg.ever_commanded = 1u;
                if (wheel_lqr_ok == 0u)
                {
                    wheel_torque[WHEELLEG_SIDE_RIGHT] = 0.0f;
                    wheel_torque[WHEELLEG_SIDE_LEFT] = 0.0f;
                }
                wheelleg_send_torque(right_map->wheel, wheel_torque[WHEELLEG_SIDE_RIGHT]);
                wheelleg_send_torque(left_map->wheel, wheel_torque[WHEELLEG_SIDE_LEFT]);
            }
            else
            {
                s_wheelleg.mode = WHEELLEG_MODE_FAULT;
                wheelleg_clear_all_control_cmds();
            }
            wheelleg_publish((uint16_t)(faults | feedback_faults),
                             s_wheelleg.mode,
                             pitch,
                             roll,
                             yaw,
                             gyro,
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
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(period_ms));
            continue;
        }

        s_wheelleg.left_test_active = 0u;
        s_wheelleg.foot_test_active = 0u;
        s_wheelleg.detached_test_active = 0u;
        s_wheelleg.foot_test_phase = 0u;
        s_wheelleg.foot_test_ik_ok = 0u;
        s_wheelleg.mode = WHEELLEG_MODE_BALANCE;
        {
            fp32 front_kin_zero;
            fp32 back_kin_zero;
            fp32 right_front_pos;
            fp32 right_back_pos;
            fp32 left_front_pos;
            fp32 left_back_pos;

            if (wheelleg_kinematic_zero(&front_kin_zero, &back_kin_zero) == 0u)
            {
                faults |= WHEELLEG_FAULT_CONTROLLER;
            }
            else
            {
                right_front_pos = wheelleg_raw_to_kinematic(s_wheelleg.front_fb[WHEELLEG_SIDE_RIGHT].position,
                                                            g_config.wheelleg_mit.right_front_zero_rad,
                                                            g_config.wheelleg_mit.right_front_dir,
                                                            front_kin_zero);
                right_back_pos = wheelleg_raw_to_kinematic(s_wheelleg.back_fb[WHEELLEG_SIDE_RIGHT].position,
                                                           g_config.wheelleg_mit.right_back_zero_rad,
                                                           g_config.wheelleg_mit.right_back_dir,
                                                           back_kin_zero);
                left_front_pos = wheelleg_raw_to_kinematic(s_wheelleg.front_fb[WHEELLEG_SIDE_LEFT].position,
                                                           g_config.wheelleg_mit.left_front_zero_rad,
                                                           g_config.wheelleg_mit.left_front_dir,
                                                           front_kin_zero);
                left_back_pos = wheelleg_raw_to_kinematic(s_wheelleg.back_fb[WHEELLEG_SIDE_LEFT].position,
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
                    faults |= WHEELLEG_FAULT_CONTROLLER;
                }
            }
        }

        if ((faults & WHEELLEG_FAULT_CONTROLLER) == 0u)
        {
            target_v = 0.0f;
            target_yaw_rate = 0.0f;
            target_leg = wheelleg_auto_leg_target(now_ms);
            target_foot_x = 0.0f;
            wheelleg_target_smooth_update(target_leg, target_foot_x, dt);
            target_leg = s_wheelleg.target_leg_smooth;
            target_foot_x = s_wheelleg.target_foot_x_smooth;
            wheelleg_update_observer(dt, gyro);
            if (wheelleg_controller_step(dt,
                                         pitch,
                                         roll,
                                         yaw,
                                         gyro,
                                         target_v,
                                         target_leg,
                                         target_foot_x,
                                         target_yaw_rate,
                                         wheel_torque) == 0u)
            {
                faults |= WHEELLEG_FAULT_CONTROLLER;
            }
        }

        if (faults != WHEELLEG_FAULT_NONE)
        {
            s_wheelleg.mode = WHEELLEG_MODE_FAULT;
            wheelleg_target_smooth_reset();
            wheelleg_clear_all_control_cmds();
        }
        else
        {
            const wheelleg_actuator_map_t *right_map = &s_wheelleg.actuator[WHEELLEG_SIDE_RIGHT];
            const wheelleg_actuator_map_t *left_map = &s_wheelleg.actuator[WHEELLEG_SIDE_LEFT];

            wheelleg_send_joint_torque(right_map->front,
                                       s_wheelleg.leg[WHEELLEG_SIDE_RIGHT].joint_torque[0],
                                       g_config.wheelleg_mit.right_front_dir);
            wheelleg_send_joint_torque(right_map->back,
                                       s_wheelleg.leg[WHEELLEG_SIDE_RIGHT].joint_torque[1],
                                       g_config.wheelleg_mit.right_back_dir);
            wheelleg_send_torque(right_map->wheel, wheel_torque[WHEELLEG_SIDE_RIGHT]);
            wheelleg_send_joint_torque(left_map->front,
                                       s_wheelleg.leg[WHEELLEG_SIDE_LEFT].joint_torque[0],
                                       g_config.wheelleg_mit.left_front_dir);
            wheelleg_send_joint_torque(left_map->back,
                                       s_wheelleg.leg[WHEELLEG_SIDE_LEFT].joint_torque[1],
                                       g_config.wheelleg_mit.left_back_dir);
            wheelleg_send_torque(left_map->wheel, wheel_torque[WHEELLEG_SIDE_LEFT]);
            controller_active = 1u;
            s_wheelleg.ever_commanded = 1u;
        }

        wheelleg_publish(faults,
                         s_wheelleg.mode,
                         pitch,
                         roll,
                         yaw,
                         gyro,
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
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(period_ms));
    }
}
