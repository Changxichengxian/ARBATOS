/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef WHEELLEG_CORE_H
#define WHEELLEG_CORE_H

#include <stdint.h>

#include <math.h>

#include "control_core.h"
#include "wheelleg_msg.h"

#ifdef __cplusplus
extern "C" {
#endif

#define WHEELLEG_CORE_ACTUATOR_COUNT 6u
#define WHEELLEG_CORE_JOINT_COUNT 2u
#define WHEELLEG_CORE_LQR_ROW_COUNT 12u
#define WHEELLEG_CORE_LQR_COEFF_COUNT 4u
#define WHEELLEG_CORE_PI 3.14159265358979323846f
#define WHEELLEG_CORE_TWO_PI 6.28318530717958647692f

typedef enum
{
    WHEELLEG_CORE_ACT_RIGHT_FRONT = 0u,
    WHEELLEG_CORE_ACT_RIGHT_BACK,
    WHEELLEG_CORE_ACT_RIGHT_WHEEL,
    WHEELLEG_CORE_ACT_LEFT_FRONT,
    WHEELLEG_CORE_ACT_LEFT_BACK,
    WHEELLEG_CORE_ACT_LEFT_WHEEL,
} wheelleg_core_actuator_e;

typedef enum
{
    WHEELLEG_CORE_JOINT_FRONT = 0u,
    WHEELLEG_CORE_JOINT_BACK,
} wheelleg_core_joint_e;

typedef struct
{
    fp32 l1_m;
    fp32 l2_m;
    fp32 l3_m;
    fp32 l4_m;
    fp32 l5_m;
} wheelleg_core_geometry_t;

typedef struct
{
    fp32 x_m;
    fp32 y_m;
    fp32 length_m;
} wheelleg_core_foot_point_t;

typedef struct
{
    fp32 x_m;
    fp32 v_mps;
} wheelleg_core_observer_t;

typedef struct
{
    fp32 foot_x_m;
    fp32 foot_y_m;
    fp32 length_m;
    uint8_t valid;
} wheelleg_core_target_smooth_t;

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
} wheelleg_core_leg_calc_t;

typedef struct
{
    wheelleg_cmd_t cmd;
    wheelleg_state_t state;
    fp32 dt_s;
    uint32_t tick_ms;
    uint8_t manual_enabled;
    uint8_t profile_enabled;
    uint8_t test_mode;
} wheelleg_core_input_t;

typedef struct
{
    wheelleg_status_t status;
    wheelleg_debug_t debug;
    actuator_cmd_t actuator[WHEELLEG_CORE_ACTUATOR_COUNT];
    uint8_t fault_flags;
    uint8_t actuator_count;
} wheelleg_core_output_t;

static inline fp32 wheelleg_core_clamp(fp32 value, fp32 min_value, fp32 max_value)
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

static inline fp32 wheelleg_core_abs(fp32 value)
{
    return (value >= 0.0f) ? value : -value;
}

static inline uint8_t wheelleg_core_axis_in_deadband(int16_t axis, uint16_t deadband)
{
    return (axis > -(int16_t)deadband && axis < (int16_t)deadband) ? 1u : 0u;
}

static inline fp32 wheelleg_core_axis_to_fp32(int16_t axis,
                                             fp32 max_abs,
                                             uint16_t deadband,
                                             int16_t axis_abs)
{
    if (axis_abs <= 0 || wheelleg_core_axis_in_deadband(axis, deadband) != 0u)
    {
        return 0.0f;
    }
    return wheelleg_core_clamp(((fp32)axis) / (fp32)axis_abs, -1.0f, 1.0f) * max_abs;
}

static inline fp32 wheelleg_core_target_theta_from_foot_x(fp32 foot_x_m, fp32 leg_length_m)
{
    fp32 ratio;

    if (leg_length_m < 0.02f)
    {
        leg_length_m = 0.02f;
    }
    ratio = wheelleg_core_clamp(foot_x_m / leg_length_m, -0.98f, 0.98f);
    return asinf(ratio);
}

static inline fp32 wheelleg_core_slew_fp32(fp32 current, fp32 target, fp32 max_delta)
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

static inline fp32 wheelleg_core_dir_sign(int8_t dir)
{
    return (dir < 0) ? -1.0f : 1.0f;
}

static inline fp32 wheelleg_core_joint_to_raw(fp32 kinematic_position, fp32 zero_position, int8_t dir)
{
    return zero_position + kinematic_position * wheelleg_core_dir_sign(dir);
}

static inline fp32 wheelleg_core_raw_to_kinematic(fp32 raw_position,
                                                 fp32 raw_zero_position,
                                                 fp32 kinematic_zero_position,
                                                 int8_t dir)
{
    return kinematic_zero_position +
           (raw_position - raw_zero_position) * wheelleg_core_dir_sign(dir);
}

static inline fp32 wheelleg_core_kinematic_to_raw(fp32 kinematic_position,
                                                 fp32 raw_zero_position,
                                                 fp32 kinematic_zero_position,
                                                 int8_t dir)
{
    return raw_zero_position +
           (kinematic_position - kinematic_zero_position) * wheelleg_core_dir_sign(dir);
}

static inline fp32 wheelleg_core_wrap_pi(fp32 value)
{
    while (value > WHEELLEG_CORE_PI)
    {
        value -= WHEELLEG_CORE_TWO_PI;
    }
    while (value < -WHEELLEG_CORE_PI)
    {
        value += WHEELLEG_CORE_TWO_PI;
    }
    return value;
}

static inline fp32 wheelleg_core_near_angle(fp32 value, fp32 reference)
{
    return reference + wheelleg_core_wrap_pi(value - reference);
}

static inline fp32 wheelleg_core_lerp(fp32 start, fp32 end, uint32_t elapsed_ms, uint32_t duration_ms)
{
    fp32 u;

    if (duration_ms == 0u || elapsed_ms >= duration_ms)
    {
        return end;
    }
    u = (fp32)elapsed_ms / (fp32)duration_ms;
    return start + (end - start) * u;
}

static inline fp32 wheelleg_core_poly4(const fp32 coe[WHEELLEG_CORE_LQR_COEFF_COUNT], fp32 x)
{
    if (coe == NULL)
    {
        return 0.0f;
    }
    return ((coe[0] * x + coe[1]) * x + coe[2]) * x + coe[3];
}

static inline uint8_t wheelleg_core_lqr_row_is_zero(const fp32 coe[WHEELLEG_CORE_LQR_COEFF_COUNT])
{
    return (coe == NULL ||
            (coe[0] == 0.0f && coe[1] == 0.0f && coe[2] == 0.0f && coe[3] == 0.0f))
               ? 1u
               : 0u;
}

static inline fp32 wheelleg_core_lqr_x_error(fp32 observer_x_m,
                                            fp32 target_v_mps,
                                            fp32 target_yaw_rate_radps,
                                            fp32 hold_limit_m,
                                            fp32 motion_eps)
{
    if (wheelleg_core_abs(target_v_mps) > motion_eps ||
        wheelleg_core_abs(target_yaw_rate_radps) > motion_eps)
    {
        return 0.0f;
    }
    return wheelleg_core_clamp(observer_x_m, -hold_limit_m, hold_limit_m);
}

static inline fp32 wheelleg_core_lqr_wheel_output(const fp32 k[WHEELLEG_CORE_LQR_ROW_COUNT],
                                                  fp32 theta_err,
                                                  fp32 d_theta,
                                                  fp32 x_err,
                                                  fp32 v_err,
                                                  fp32 pitch_err,
                                                  fp32 gyro_y)
{
    if (k == NULL)
    {
        return 0.0f;
    }

    return k[0] * theta_err +
           k[1] * d_theta +
           k[2] * x_err +
           k[3] * v_err +
           k[4] * pitch_err +
           k[5] * gyro_y;
}

static inline fp32 wheelleg_core_lqr_hip_output(const fp32 k[WHEELLEG_CORE_LQR_ROW_COUNT],
                                                fp32 theta_err,
                                                fp32 d_theta,
                                                fp32 x_err,
                                                fp32 v_err,
                                                fp32 pitch_err,
                                                fp32 gyro_y,
                                                fp32 split_tp)
{
    if (k == NULL)
    {
        return 0.0f;
    }

    return k[6] * theta_err +
           k[7] * d_theta +
           k[8] * x_err +
           k[9] * v_err +
           k[10] * pitch_err +
           k[11] * gyro_y +
           split_tp;
}

static inline fp32 wheelleg_core_turn_torque(fp32 yaw_set,
                                             fp32 yaw,
                                             fp32 yaw_gyro,
                                             fp32 kp,
                                             fp32 kd,
                                             fp32 max_out)
{
    const fp32 out = kp * wheelleg_core_wrap_pi(yaw_set - yaw) - kd * yaw_gyro;

    return wheelleg_core_clamp(out, -max_out, max_out);
}

static inline fp32 wheelleg_core_roll_force(fp32 roll_set,
                                            fp32 roll,
                                            fp32 roll_gyro,
                                            fp32 kp,
                                            fp32 kd,
                                            fp32 max_out)
{
    const fp32 out = kp * (roll_set - roll) - kd * roll_gyro;

    return wheelleg_core_clamp(out, -max_out, max_out);
}

static inline void wheelleg_core_observer_update(wheelleg_core_observer_t *observer,
                                                 const wheelleg_core_leg_calc_t *left_leg,
                                                 const wheelleg_core_leg_calc_t *right_leg,
                                                 fp32 left_wheel_velocity_radps,
                                                 fp32 right_wheel_velocity_radps,
                                                 fp32 wheel_radius_m,
                                                 fp32 gyro_y,
                                                 fp32 lpf,
                                                 fp32 dt)
{
    fp32 wr;
    fp32 wl;
    fp32 vr;
    fp32 vl;
    fp32 v_meas;

    if (observer == NULL || left_leg == NULL || right_leg == NULL || dt <= 0.0f)
    {
        return;
    }

    lpf = wheelleg_core_clamp(lpf, 0.01f, 1.0f);
    wr = -right_wheel_velocity_radps + right_leg->d_alpha - gyro_y;
    wl = left_wheel_velocity_radps + left_leg->d_alpha - gyro_y;
    vr = wr * wheel_radius_m +
         right_leg->length * right_leg->d_theta * cosf(right_leg->theta) +
         right_leg->d_length * sinf(right_leg->theta);
    vl = wl * wheel_radius_m +
         left_leg->length * left_leg->d_theta * cosf(left_leg->theta) +
         left_leg->d_length * sinf(left_leg->theta);
    v_meas = (vr + vl) * 0.5f;

    observer->v_mps += lpf * (v_meas - observer->v_mps);
    observer->x_m += observer->v_mps * dt;
}

static inline uint8_t wheelleg_core_limit_foot_xy(fp32 min_leg_m,
                                                  fp32 max_leg_m,
                                                  fp32 max_foot_x_range_m,
                                                  fp32 *x_m,
                                                  fp32 *y_m,
                                                  fp32 *length_m)
{
    fp32 x;
    fp32 y;
    fp32 max_x_sq;
    fp32 max_abs_x;
    fp32 length;

    if (x_m == NULL || y_m == NULL || length_m == NULL)
    {
        return 0u;
    }

    if (min_leg_m <= 0.02f)
    {
        min_leg_m = 0.02f;
    }
    if (max_leg_m < min_leg_m)
    {
        max_leg_m = min_leg_m;
    }

    y = wheelleg_core_clamp(*y_m, min_leg_m, max_leg_m);
    max_x_sq = max_leg_m * max_leg_m - y * y;
    max_abs_x = (max_x_sq > 0.0f) ? sqrtf(max_x_sq) : 0.0f;
    if (max_abs_x > max_foot_x_range_m)
    {
        max_abs_x = max_foot_x_range_m;
    }

    x = wheelleg_core_clamp(*x_m, -max_abs_x, max_abs_x);
    length = sqrtf(x * x + y * y);
    if (length <= 0.02f || length > max_leg_m + 0.0001f)
    {
        return 0u;
    }

    *x_m = x;
    *y_m = y;
    *length_m = length;
    return 1u;
}

static inline void wheelleg_core_target_smooth_clear(wheelleg_core_target_smooth_t *smooth)
{
    if (smooth == NULL)
    {
        return;
    }

    smooth->foot_x_m = 0.0f;
    smooth->foot_y_m = 0.0f;
    smooth->length_m = 0.0f;
    smooth->valid = 0u;
}

static inline uint8_t wheelleg_core_target_smooth_update_xy(wheelleg_core_target_smooth_t *smooth,
                                                            fp32 target_foot_x_m,
                                                            fp32 target_foot_y_m,
                                                            fp32 measured_leg_m,
                                                            fp32 measured_alpha_rad,
                                                            fp32 min_leg_m,
                                                            fp32 max_leg_m,
                                                            fp32 max_foot_x_range_m,
                                                            fp32 foot_x_slew_mps,
                                                            fp32 foot_y_slew_mps,
                                                            fp32 default_dt,
                                                            fp32 dt)
{
    fp32 measured_x;
    fp32 measured_y;
    fp32 measured_length;
    fp32 target_length;

    if (smooth == NULL)
    {
        return 0u;
    }
    if (dt <= 0.0f)
    {
        dt = default_dt;
    }

    if (wheelleg_core_limit_foot_xy(min_leg_m,
                                    max_leg_m,
                                    max_foot_x_range_m,
                                    &target_foot_x_m,
                                    &target_foot_y_m,
                                    &target_length) == 0u)
    {
        return 0u;
    }

    if (smooth->valid == 0u)
    {
        measured_alpha_rad = wheelleg_core_clamp(measured_alpha_rad, -1.2f, 1.2f);
        measured_length = wheelleg_core_clamp(measured_leg_m, min_leg_m, max_leg_m);
        measured_x = measured_length * sinf(measured_alpha_rad);
        measured_y = measured_length * cosf(measured_alpha_rad);
        if (wheelleg_core_limit_foot_xy(min_leg_m,
                                        max_leg_m,
                                        max_foot_x_range_m,
                                        &measured_x,
                                        &measured_y,
                                        &measured_length) == 0u)
        {
            measured_x = target_foot_x_m;
            measured_y = target_foot_y_m;
            measured_length = target_length;
        }

        smooth->foot_x_m = measured_x;
        smooth->foot_y_m = measured_y;
        smooth->length_m = measured_length;
        smooth->valid = 1u;
    }

    smooth->foot_x_m = wheelleg_core_slew_fp32(smooth->foot_x_m,
                                               target_foot_x_m,
                                               foot_x_slew_mps * dt);
    smooth->foot_y_m = wheelleg_core_slew_fp32(smooth->foot_y_m,
                                               target_foot_y_m,
                                               foot_y_slew_mps * dt);
    smooth->length_m = target_length;
    return wheelleg_core_limit_foot_xy(min_leg_m,
                                       max_leg_m,
                                       max_foot_x_range_m,
                                       &smooth->foot_x_m,
                                       &smooth->foot_y_m,
                                       &smooth->length_m);
}

static inline uint8_t wheelleg_core_forward_point(const wheelleg_core_geometry_t *geo,
                                                  fp32 front_pos,
                                                  fp32 back_pos,
                                                  wheelleg_core_foot_point_t *out)
{
    const fp32 l1 = (geo != NULL) ? geo->l1_m : 0.0f;
    const fp32 l2 = (geo != NULL) ? geo->l2_m : 0.0f;
    const fp32 l3 = (geo != NULL) ? geo->l3_m : 0.0f;
    const fp32 l4 = (geo != NULL) ? geo->l4_m : 0.0f;
    const fp32 l5 = (geo != NULL) ? geo->l5_m : 0.0f;
    const fp32 phi1 = WHEELLEG_CORE_PI * 0.5f + front_pos;
    const fp32 phi4 = WHEELLEG_CORE_PI * 0.5f + back_pos;
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
    if (discr < -0.000001f || wheelleg_core_abs(a0 + c0) < 0.000001f)
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

static inline uint8_t wheelleg_core_calc_kinematics(const wheelleg_core_geometry_t *geo,
                                                    wheelleg_core_leg_calc_t *leg,
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

    if (geo == NULL || leg == NULL || dt <= 0.0f)
    {
        return 0u;
    }

    if (left_side != 0u)
    {
        pitch_side = -pitch;
        gyro_side = -gyro_y;
    }

    leg->l1 = geo->l1_m;
    leg->l2 = geo->l2_m;
    leg->l3 = geo->l3_m;
    leg->l4 = geo->l4_m;
    leg->l5 = geo->l5_m;
    leg->phi1 = WHEELLEG_CORE_PI * 0.5f + front_pos;
    leg->phi4 = WHEELLEG_CORE_PI * 0.5f + back_pos;

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
    leg->alpha = WHEELLEG_CORE_PI * 0.5f - leg->phi0;
    if (leg->first == 0u)
    {
        leg->last_phi0 = leg->phi0;
        leg->last_length = length;
        leg->last_d_length = 0.0f;
        leg->last_d_theta = 0.0f;
        leg->first = 1u;
    }

    leg->d_alpha = -(leg->phi0 - leg->last_phi0) / dt;
    leg->theta = leg->alpha - pitch_side;
    leg->d_theta = leg->d_alpha - gyro_side;
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

static inline uint8_t wheelleg_core_calc_vmc(wheelleg_core_leg_calc_t *leg)
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
    if (wheelleg_core_abs(sin32) < 0.0001f)
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

static inline uint8_t wheelleg_core_inverse_point(const wheelleg_core_geometry_t *geo,
                                                  const wheelleg_core_foot_point_t *target,
                                                  fp32 front_ref,
                                                  fp32 back_ref,
                                                  fp32 *front_out,
                                                  fp32 *back_out)
{
    const fp32 l1 = (geo != NULL) ? geo->l1_m : 0.0f;
    const fp32 l2 = (geo != NULL) ? geo->l2_m : 0.0f;
    const fp32 l3 = (geo != NULL) ? geo->l3_m : 0.0f;
    const fp32 l4 = (geo != NULL) ? geo->l4_m : 0.0f;
    const fp32 l5 = (geo != NULL) ? geo->l5_m : 0.0f;
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

    front_cos = wheelleg_core_clamp(front_cos, -1.0f, 1.0f);
    back_cos = wheelleg_core_clamp(back_cos, -1.0f, 1.0f);
    front_base = atan2f(cy, cx);
    back_base = atan2f(cy, back_dx);
    front_q = acosf(front_cos);
    back_q = acosf(back_cos);

    front_raw[0] = wheelleg_core_near_angle(front_base + front_q - WHEELLEG_CORE_PI * 0.5f, front_ref);
    front_raw[1] = wheelleg_core_near_angle(front_base - front_q - WHEELLEG_CORE_PI * 0.5f, front_ref);
    back_raw[0] = wheelleg_core_near_angle(back_base + back_q - WHEELLEG_CORE_PI * 0.5f, back_ref);
    back_raw[1] = wheelleg_core_near_angle(back_base - back_q - WHEELLEG_CORE_PI * 0.5f, back_ref);

    for (i = 0u; i < 2u; i++)
    {
        for (j = 0u; j < 2u; j++)
        {
            wheelleg_core_foot_point_t check;
            fp32 err;
            fp32 score;

            if (wheelleg_core_forward_point(geo, front_raw[i], back_raw[j], &check) == 0u)
            {
                continue;
            }
            err = wheelleg_core_abs(check.x_m - target->x_m) + wheelleg_core_abs(check.y_m - target->y_m);
            if (err > 0.003f)
            {
                continue;
            }

            score = wheelleg_core_abs(wheelleg_core_wrap_pi(front_raw[i] - front_ref)) +
                    wheelleg_core_abs(wheelleg_core_wrap_pi(back_raw[j] - back_ref)) +
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

static inline void wheelleg_core_set_joint_torque(wheelleg_core_output_t *out,
                                                  wheelleg_core_actuator_e actuator,
                                                  fp32 torque_nm)
{
    if (out == NULL || (uint8_t)actuator >= WHEELLEG_CORE_ACTUATOR_COUNT)
    {
        return;
    }

    control_core_cmd_set_state_torque(&out->actuator[(uint8_t)actuator],
                                      0.0f,
                                      0.0f,
                                      0.0f,
                                      0.0f,
                                      torque_nm);
}

static inline void wheelleg_core_set_state_torque(wheelleg_core_output_t *out,
                                                  wheelleg_core_actuator_e actuator,
                                                  fp32 position_rad,
                                                  fp32 velocity_radps,
                                                  fp32 kp,
                                                  fp32 kd,
                                                  fp32 torque_nm)
{
    if (out == NULL || (uint8_t)actuator >= WHEELLEG_CORE_ACTUATOR_COUNT)
    {
        return;
    }

    control_core_cmd_set_state_torque(&out->actuator[(uint8_t)actuator],
                                      position_rad,
                                      velocity_radps,
                                      kp,
                                      kd,
                                      torque_nm);
}

static inline void wheelleg_core_set_wheel_torques(wheelleg_core_output_t *out,
                                                   const fp32 wheel_torque_nm[WHEELLEG_SIDE_COUNT])
{
    if (out == NULL || wheel_torque_nm == NULL)
    {
        return;
    }

    wheelleg_core_set_joint_torque(out,
                                   WHEELLEG_CORE_ACT_RIGHT_WHEEL,
                                   wheel_torque_nm[WHEELLEG_SIDE_RIGHT]);
    wheelleg_core_set_joint_torque(out,
                                   WHEELLEG_CORE_ACT_LEFT_WHEEL,
                                   wheel_torque_nm[WHEELLEG_SIDE_LEFT]);
}

static inline void wheelleg_core_set_vmc_joint_torques(wheelleg_core_output_t *out,
                                                       const wheelleg_core_leg_calc_t leg[WHEELLEG_SIDE_COUNT],
                                                       int8_t right_front_dir,
                                                       int8_t right_back_dir,
                                                       int8_t left_front_dir,
                                                       int8_t left_back_dir)
{
    const wheelleg_core_leg_calc_t *right_leg;
    const wheelleg_core_leg_calc_t *left_leg;

    if (out == NULL || leg == NULL)
    {
        return;
    }

    right_leg = &leg[WHEELLEG_SIDE_RIGHT];
    left_leg = &leg[WHEELLEG_SIDE_LEFT];

    wheelleg_core_set_joint_torque(out,
                                   WHEELLEG_CORE_ACT_RIGHT_FRONT,
                                   right_leg->joint_torque[(uint8_t)WHEELLEG_CORE_JOINT_FRONT] *
                                       wheelleg_core_dir_sign(right_front_dir));
    wheelleg_core_set_joint_torque(out,
                                   WHEELLEG_CORE_ACT_RIGHT_BACK,
                                   right_leg->joint_torque[(uint8_t)WHEELLEG_CORE_JOINT_BACK] *
                                       wheelleg_core_dir_sign(right_back_dir));
    wheelleg_core_set_joint_torque(out,
                                   WHEELLEG_CORE_ACT_LEFT_FRONT,
                                   left_leg->joint_torque[(uint8_t)WHEELLEG_CORE_JOINT_FRONT] *
                                       wheelleg_core_dir_sign(left_front_dir));
    wheelleg_core_set_joint_torque(out,
                                   WHEELLEG_CORE_ACT_LEFT_BACK,
                                   left_leg->joint_torque[(uint8_t)WHEELLEG_CORE_JOINT_BACK] *
                                       wheelleg_core_dir_sign(left_back_dir));
}

static inline void wheelleg_core_output_clear(wheelleg_core_output_t *out)
{
    if (out == NULL)
    {
        return;
    }

    out->fault_flags = 0u;
    out->actuator_count = WHEELLEG_CORE_ACTUATOR_COUNT;
    control_core_cmd_clear_many(out->actuator, WHEELLEG_CORE_ACTUATOR_COUNT);
}

#ifdef __cplusplus
}
#endif

#endif
