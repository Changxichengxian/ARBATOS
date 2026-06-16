/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Private implementation chunk included by wheelleg_mit_task.c.
 */

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

static fp32 wheelleg_lqr_side_sign(uint8_t side)
{
    return (side == WHEELLEG_SIDE_LEFT) ? -1.0f : 1.0f;
}

static fp32 wheelleg_cfg_positive_or_default(fp32 value, fp32 default_value, fp32 max_value)
{
    if (value <= 0.0f)
    {
        value = default_value;
    }
    return wheelleg_clamp(value, 0.0f, max_value);
}

static fp32 wheelleg_lqr_pitch_offset_for_side(uint8_t side)
{
    const wheelleg_mit_config_t *cfg = &g_config.wheelleg_mit;
    const fp32 base_offset =
        (side == WHEELLEG_SIDE_LEFT) ? cfg->pitch_balance_offset_left_rad
                                     : cfg->pitch_balance_offset_right_rad;

    return base_offset + wheelleg_lqr_side_sign(side) * s_wheelleg.pitch_auto_offset_rad;
}

static uint8_t wheelleg_pitch_trim_idle(int16_t vx_axis,
                                        int16_t yaw_axis,
                                        fp32 target_v,
                                        fp32 target_yaw_rate)
{
    if (wheelleg_axis_in_deadband(vx_axis, g_config.wheelleg_mit.rc_deadband) == 0u ||
        wheelleg_axis_in_deadband(yaw_axis, g_config.wheelleg_mit.rc_deadband) == 0u)
    {
        return 0u;
    }
    if (wheelleg_abs(target_v) > WHEELLEG_LQR_MOTION_EPS ||
        wheelleg_abs(target_yaw_rate) > WHEELLEG_LQR_MOTION_EPS)
    {
        return 0u;
    }
    return 1u;
}

static fp32 wheelleg_pitch_trim_velocity_error(fp32 v_mps, fp32 deadband_mps)
{
    if (wheelleg_abs(v_mps) <= deadband_mps)
    {
        return 0.0f;
    }
    return (v_mps > 0.0f) ? (v_mps - deadband_mps) : (v_mps + deadband_mps);
}

static void wheelleg_pitch_trim_update(fp32 dt,
                                       int16_t vx_axis,
                                       int16_t yaw_axis,
                                       fp32 target_v,
                                       fp32 target_yaw_rate)
{
    const wheelleg_mit_config_t *cfg = &g_config.wheelleg_mit;
    const fp32 max_offset =
        wheelleg_cfg_positive_or_default(cfg->pitch_trim_max_rad,
                                         WHEELLEG_PITCH_TRIM_MAX_RAD_DEFAULT,
                                         0.35f);
    const fp32 max_rate =
        wheelleg_cfg_positive_or_default(cfg->pitch_trim_rate_radps,
                                         WHEELLEG_PITCH_TRIM_RATE_RADPS_DEFAULT,
                                         0.20f);
    const fp32 deadband =
        wheelleg_cfg_positive_or_default(cfg->pitch_trim_v_deadband_mps,
                                         WHEELLEG_PITCH_TRIM_V_DEADBAND_MPS_DEFAULT,
                                         0.20f);
    fp32 lpf;
    fp32 gain;
    fp32 v_err;
    fp32 rate;

    if (dt <= 0.0f || cfg->pitch_trim_enable == 0u)
    {
        s_wheelleg.pitch_auto_offset_rad = 0.0f;
        s_wheelleg.pitch_trim_v_lpf_mps = 0.0f;
        return;
    }

    if (wheelleg_pitch_trim_idle(vx_axis, yaw_axis, target_v, target_yaw_rate) == 0u)
    {
        s_wheelleg.pitch_trim_v_lpf_mps = 0.0f;
        return;
    }

    lpf = cfg->pitch_trim_lpf;
    if (lpf <= 0.0f)
    {
        lpf = WHEELLEG_PITCH_TRIM_LPF_DEFAULT;
    }
    lpf = wheelleg_clamp(lpf, 0.001f, 1.0f);
    s_wheelleg.pitch_trim_v_lpf_mps += lpf * (s_wheelleg.v_mps - s_wheelleg.pitch_trim_v_lpf_mps);

    gain = cfg->pitch_trim_gain;
    if (gain == 0.0f)
    {
        gain = WHEELLEG_PITCH_TRIM_GAIN_DEFAULT;
    }

    v_err = wheelleg_pitch_trim_velocity_error(s_wheelleg.pitch_trim_v_lpf_mps, deadband);
    rate = wheelleg_clamp(gain * v_err, -max_rate, max_rate);
    s_wheelleg.pitch_auto_offset_rad =
        wheelleg_clamp(s_wheelleg.pitch_auto_offset_rad + rate * dt, -max_offset, max_offset);
}

static void wheelleg_lqr_side_state(uint8_t side,
                                    fp32 pitch,
                                    const fp32 gyro[3],
                                    fp32 target_theta,
                                    fp32 target_v,
                                    fp32 target_yaw_rate,
                                    wheelleg_lqr_side_state_t *out)
{
    const fp32 side_sign = wheelleg_lqr_side_sign(side);
    const fp32 gyro_y = (gyro != NULL) ? gyro[INS_GYRO_Y_ADDRESS_OFFSET] : 0.0f;
    const fp32 x_err = wheelleg_lqr_x_error(target_v, target_yaw_rate);
    const fp32 v_err = s_wheelleg.v_mps - target_v;

    if (out == NULL || side >= WHEELLEG_SIDE_COUNT)
    {
        return;
    }

    out->theta_err = s_wheelleg.leg[side].theta - target_theta;
    out->d_theta = s_wheelleg.leg[side].d_theta;
    out->x_err = side_sign * x_err;
    out->v_err = side_sign * v_err;
    out->pitch_err = side_sign * pitch - wheelleg_lqr_pitch_offset_for_side(side);
    out->gyro_y = side_sign * gyro_y;
}

static fp32 wheelleg_lqr_wheel_torque_to_motor(uint8_t side,
                                               fp32 lqr_torque,
                                               fp32 wheel_scale,
                                               fp32 turn_t)
{
    (void)side;
    return wheelleg_clamp(lqr_torque * wheel_scale - turn_t,
                          -g_config.wheelleg_mit.max_wheel_torque_nm,
                          g_config.wheelleg_mit.max_wheel_torque_nm);
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
