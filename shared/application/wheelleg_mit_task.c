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
#include "wheelleg_msg.h"

#include <math.h>
#include <string.h>

#define WHEELLEG_PI 3.14159265358979323846f
#define WHEELLEG_TWO_PI 6.28318530717958647692f
#define WHEELLEG_FEEDBACK_TIMEOUT_MS 80u

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
    wheelleg_actuator_map_t actuator[WHEELLEG_SIDE_COUNT];
    actuator_feedback_t front_fb[WHEELLEG_SIDE_COUNT];
    actuator_feedback_t back_fb[WHEELLEG_SIDE_COUNT];
    actuator_feedback_t wheel_fb[WHEELLEG_SIDE_COUNT];
    wheelleg_leg_calc_t leg[WHEELLEG_SIDE_COUNT];
    wheelleg_pid_t leg_pid[WHEELLEG_SIDE_COUNT];
    wheelleg_pid_t split_pid;
    fp32 x_m;
    fp32 v_mps;
    fp32 yaw_set;
    fp32 last_yaw;
    uint8_t yaw_inited;
    uint8_t ever_commanded;
    wheelleg_mode_e mode;
    wheelleg_mode_e last_mode;
    uint32_t overrun_count;
} wheelleg_mit_ctrl_t;

static wheelleg_mit_ctrl_t s_wheelleg;

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

static void wheelleg_send_all_zero(void)
{
    uint8_t side;

    for (side = 0u; side < WHEELLEG_SIDE_COUNT; side++)
    {
        const wheelleg_actuator_map_t *map = &s_wheelleg.actuator[side];
        wheelleg_send_torque(map->front, 0.0f);
        wheelleg_send_torque(map->back, 0.0f);
        wheelleg_send_torque(map->wheel, 0.0f);
    }
}

static fp32 wheelleg_axis_to_fp32(int16_t axis, fp32 max_abs, uint16_t deadband)
{
    if (axis > -(int16_t)deadband && axis < (int16_t)deadband)
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
                             fp32 target_yaw_rate,
                             const fp32 wheel_torque[WHEELLEG_SIDE_COUNT],
                             uint8_t controller_active)
{
    wheelleg_state_t state;
    wheelleg_status_t status;
    wheelleg_debug_t debug;
    static uint32_t seq;
    const uint32_t now_ms = wheelleg_tick_ms();
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

static uint8_t wheelleg_controller_step(fp32 dt,
                                        fp32 pitch,
                                        fp32 roll,
                                        fp32 yaw,
                                        const fp32 gyro[3],
                                        fp32 target_v,
                                        fp32 target_leg,
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

    split_tp = wheelleg_pid_calc(&s_wheelleg.split_pid,
                                 s_wheelleg.leg[WHEELLEG_SIDE_RIGHT].theta +
                                     s_wheelleg.leg[WHEELLEG_SIDE_LEFT].theta,
                                 0.0f);

    x_err_r = s_wheelleg.x_m;
    v_err_r = s_wheelleg.v_mps - 0.4f * target_v;
    wheel_torque[WHEELLEG_SIDE_RIGHT] =
        k_right[0] * s_wheelleg.leg[WHEELLEG_SIDE_RIGHT].theta +
        k_right[1] * s_wheelleg.leg[WHEELLEG_SIDE_RIGHT].d_theta +
        k_right[2] * x_err_r +
        k_right[3] * v_err_r +
        k_right[4] * (right_pitch - g_config.wheelleg_mit.pitch_balance_offset_right_rad) +
        k_right[5] * right_gyro -
        turn_t;
    s_wheelleg.leg[WHEELLEG_SIDE_RIGHT].tp =
        k_right[6] * s_wheelleg.leg[WHEELLEG_SIDE_RIGHT].theta +
        k_right[7] * s_wheelleg.leg[WHEELLEG_SIDE_RIGHT].d_theta +
        k_right[8] * x_err_r +
        k_right[9] * v_err_r +
        k_right[10] * (right_pitch - g_config.wheelleg_mit.pitch_balance_offset_right_rad) +
        k_right[11] * right_gyro +
        split_tp;

    x_err_l = -s_wheelleg.x_m;
    v_err_l = 0.4f * target_v - s_wheelleg.v_mps;
    wheel_torque[WHEELLEG_SIDE_LEFT] =
        k_left[0] * s_wheelleg.leg[WHEELLEG_SIDE_LEFT].theta +
        k_left[1] * s_wheelleg.leg[WHEELLEG_SIDE_LEFT].d_theta +
        k_left[2] * x_err_l +
        k_left[3] * v_err_l +
        k_left[4] * (left_pitch - g_config.wheelleg_mit.pitch_balance_offset_left_rad) +
        k_left[5] * left_gyro -
        turn_t;
    s_wheelleg.leg[WHEELLEG_SIDE_LEFT].tp =
        k_left[6] * s_wheelleg.leg[WHEELLEG_SIDE_LEFT].theta +
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
    osDelay(g_config.wheelleg_mit.task_init_time_ms);
    last_wake = xTaskGetTickCount();

    while (1)
    {
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
        const int16_t leg_axis = input_axis(INPUT_AXIS_CHASSIS_Y);
        const int16_t yaw_axis = input_axis(INPUT_AXIS_CHASSIS_WZ);
        const fp32 target_v = wheelleg_axis_to_fp32(vx_axis,
                                                    g_config.wheelleg_mit.max_v_mps,
                                                    g_config.wheelleg_mit.rc_deadband);
        const fp32 target_yaw_rate = wheelleg_axis_to_fp32(yaw_axis,
                                                           g_config.wheelleg_mit.max_yaw_rate_radps,
                                                           g_config.wheelleg_mit.rc_deadband);
        const fp32 leg_span = g_config.wheelleg_mit.max_leg_length_m - g_config.wheelleg_mit.min_leg_length_m;
        fp32 target_leg = g_config.wheelleg_mit.default_leg_length_m +
                          wheelleg_axis_to_fp32(leg_axis,
                                                leg_span * 0.5f,
                                                g_config.wheelleg_mit.rc_deadband);
        fp32 wheel_torque[WHEELLEG_SIDE_COUNT] = {0.0f, 0.0f};
        uint16_t faults = WHEELLEG_FAULT_NONE;
        const uint8_t profile_on = robot_profile_is_wheelleg_mit();
        const uint8_t manual_on =
            control_input_switch_is_pos(input_switch(INPUT_SW_CHASSIS_MODE),
                                        g_config.wheelleg_mit.enable_switch_pos);
        uint8_t enabled;
        uint8_t controller_active = 0u;

        target_leg = wheelleg_clamp(target_leg,
                                    g_config.wheelleg_mit.min_leg_length_m,
                                    g_config.wheelleg_mit.max_leg_length_m);
        wheelleg_configure_actuators();
        wheelleg_pid_apply(&s_wheelleg.leg_pid[WHEELLEG_SIDE_LEFT], &g_config.wheelleg_mit.leg_length_pid);
        wheelleg_pid_apply(&s_wheelleg.leg_pid[WHEELLEG_SIDE_RIGHT], &g_config.wheelleg_mit.leg_length_pid);
        wheelleg_pid_apply(&s_wheelleg.split_pid, &g_config.wheelleg_mit.leg_split_pid);

        faults |= wheelleg_update_feedback(now_ms);
        if (toe_is_error(DBUS_TOE))
        {
            faults |= WHEELLEG_FAULT_MANUAL_OFFLINE;
        }
        if (toe_is_error(BOARD_GYRO_TOE) || toe_is_error(BOARD_ACCEL_TOE))
        {
            faults |= WHEELLEG_FAULT_IMU_OFFLINE;
        }
        if (wheelleg_abs(pitch) > g_config.wheelleg_mit.attitude_limit_rad ||
            wheelleg_abs(roll) > g_config.wheelleg_mit.attitude_limit_rad)
        {
            faults |= WHEELLEG_FAULT_ATTITUDE_LIMIT;
        }

        enabled = (uint8_t)(profile_on != 0u && manual_on != 0u && faults == WHEELLEG_FAULT_NONE);

        s_wheelleg.last_mode = s_wheelleg.mode;
        if (enabled == 0u)
        {
            s_wheelleg.mode = (faults == WHEELLEG_FAULT_NONE) ? WHEELLEG_MODE_DISABLED : WHEELLEG_MODE_FAULT;
            if (profile_on != 0u && (manual_on != 0u || s_wheelleg.ever_commanded != 0u))
            {
                wheelleg_send_all_zero();
            }
            wheelleg_pid_clear(&s_wheelleg.leg_pid[WHEELLEG_SIDE_LEFT]);
            wheelleg_pid_clear(&s_wheelleg.leg_pid[WHEELLEG_SIDE_RIGHT]);
            wheelleg_pid_clear(&s_wheelleg.split_pid);
            s_wheelleg.yaw_inited = 0u;
            wheelleg_publish(faults, s_wheelleg.mode, pitch, roll, yaw, gyro,
                             target_v, target_leg, target_yaw_rate, wheel_torque, 0u);
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(period_ms));
            continue;
        }

        s_wheelleg.mode = WHEELLEG_MODE_BALANCE;
        if (wheelleg_calc_kinematics(&s_wheelleg.leg[WHEELLEG_SIDE_RIGHT],
                                     s_wheelleg.front_fb[WHEELLEG_SIDE_RIGHT].position,
                                     s_wheelleg.back_fb[WHEELLEG_SIDE_RIGHT].position,
                                     pitch,
                                     (gyro != NULL) ? gyro[INS_GYRO_Y_ADDRESS_OFFSET] : 0.0f,
                                     0u,
                                     dt) == 0u ||
            wheelleg_calc_kinematics(&s_wheelleg.leg[WHEELLEG_SIDE_LEFT],
                                     s_wheelleg.front_fb[WHEELLEG_SIDE_LEFT].position,
                                     s_wheelleg.back_fb[WHEELLEG_SIDE_LEFT].position,
                                     pitch,
                                     (gyro != NULL) ? gyro[INS_GYRO_Y_ADDRESS_OFFSET] : 0.0f,
                                     1u,
                                     dt) == 0u)
        {
            faults |= WHEELLEG_FAULT_CONTROLLER;
        }
        else
        {
            wheelleg_update_observer(dt, gyro);
            if (wheelleg_controller_step(dt,
                                         pitch,
                                         roll,
                                         yaw,
                                         gyro,
                                         target_v,
                                         target_leg,
                                         target_yaw_rate,
                                         wheel_torque) == 0u)
            {
                faults |= WHEELLEG_FAULT_CONTROLLER;
            }
        }

        if (faults != WHEELLEG_FAULT_NONE)
        {
            s_wheelleg.mode = WHEELLEG_MODE_FAULT;
            wheelleg_send_all_zero();
        }
        else
        {
            const wheelleg_actuator_map_t *right_map = &s_wheelleg.actuator[WHEELLEG_SIDE_RIGHT];
            const wheelleg_actuator_map_t *left_map = &s_wheelleg.actuator[WHEELLEG_SIDE_LEFT];

            wheelleg_send_torque(right_map->front, s_wheelleg.leg[WHEELLEG_SIDE_RIGHT].joint_torque[0]);
            wheelleg_send_torque(right_map->back, s_wheelleg.leg[WHEELLEG_SIDE_RIGHT].joint_torque[1]);
            wheelleg_send_torque(right_map->wheel, wheel_torque[WHEELLEG_SIDE_RIGHT]);
            wheelleg_send_torque(left_map->front, s_wheelleg.leg[WHEELLEG_SIDE_LEFT].joint_torque[0]);
            wheelleg_send_torque(left_map->back, s_wheelleg.leg[WHEELLEG_SIDE_LEFT].joint_torque[1]);
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
