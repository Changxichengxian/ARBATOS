/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Private implementation chunk included by wheelleg_mit_task.c.
 */

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
    s_wheelleg.pitch_auto_offset_rad = 0.0f;
    s_wheelleg.pitch_trim_v_lpf_mps = 0.0f;
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

static uint8_t wheelleg_control_manager_allows(uint32_t tick_ms, fp32 dt)
{
    control_context_t context = {0};

    context.tick_ms = tick_ms;
    context.dt_s = dt;

    if (control_manager_update_domain(CONTROL_DOMAIN_WHEELLEG, &context) != CONTROL_RESULT_OK)
    {
        return 0u;
    }

    return (control_manager_active_id(CONTROL_DOMAIN_WHEELLEG) == CONTROL_CONTROLLER_WHEELLEG_MIT_BALANCE) ? 1u : 0u;
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
