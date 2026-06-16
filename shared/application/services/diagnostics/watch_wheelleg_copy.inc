/*
 * SPDX-FileCopyrightText: 2026 ?? <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Private implementation chunk included by watch.c.
 */

#if WATCH_ENABLE_LOCOMOTION_WHEELLEG_MIT
static fp32 watch_wheelleg_dir_sign(int8_t dir)
{
    return (dir < 0) ? -1.0f : 1.0f;
}

static void watch_copy_wheelleg_mit_motor(watch_wheelleg_mit_motor_t *out,
                                          uint8_t raw_id,
                                          fp32 zero_rad,
                                          int8_t dir,
                                          uint8_t use_rel_position)
{
    MotorCmd cmd = {0};
    MotorState fb = {0};
    MotorApplied applied = {0};
    MotorId id;

    if (out == NULL)
    {
        return;
    }
    out->id = raw_id;
    if ((uint32_t)raw_id >= (uint32_t)MotorCount)
    {
        return;
    }

    id = (MotorId)raw_id;
    if (LowCmdGetMotor(id, &cmd) != 0u)
    {
        out->cmd_active = cmd.active;
        out->cmd_mode = cmd.mode;
        out->cmd_writer = cmd.writer;
        out->cmd_timeout_ms = cmd.timeoutMs;
        out->cmd_seq = cmd.seq;
        out->cmd_tick_ms = cmd.tick;
        out->cmd_position_deg = watch_rad_to_deg(cmd.q);
        out->cmd_velocity_deg_s = watch_rad_to_deg(cmd.dq);
        out->cmd_kp = cmd.kp;
        out->cmd_kd = cmd.kd;
        out->cmd_torque_nm = cmd.tau;
    }

    if (LowStateGetApplied(id, &applied) != 0u)
    {
        out->applied_active = applied.active;
        out->applied_mode = applied.mode;
        out->applied_drive_state = applied.driveState;
        out->applied_flags = applied.flags;
        out->applied_bus = applied.bus;
        out->applied_transport = applied.transport;
        out->applied_protocol = applied.protocol;
        out->applied_tx_id = applied.txId;
        out->applied_tick_ms = applied.tick;
        out->applied_current = applied.current;
        out->applied_position_deg = watch_rad_to_deg(applied.q);
        out->applied_velocity_deg_s = watch_rad_to_deg(applied.dq);
        out->applied_kp = applied.kp;
        out->applied_kd = applied.kd;
        out->applied_torque_nm = applied.tau;
    }

    if (LowStateGetMotor(id, &fb) != 0u)
    {
        out->online = fb.online;
        out->fb_bus = fb.bus;
        out->fb_rx_dlc = fb.rxDlc;
        out->fb_rx_data0 = fb.rxData0;
        out->fb_rx_data0_low4 = (uint8_t)(fb.rxData0 & 0x0Fu);
        out->fb_rx_data0_high4 = (uint8_t)(fb.rxData0 >> 4);
        out->fb_motor_id = fb.motorId;
        out->fb_state = fb.state;
        out->fb_rx_id = fb.rxId;
        out->fb_rx_count = fb.rxCount;
        out->fb_last_rx_tick_ms = fb.lastRxTick;
        out->fb_position_deg = watch_rad_to_deg(fb.q);
        out->fb_velocity_deg_s = watch_rad_to_deg(fb.dq);
        out->fb_torque_nm = fb.tauEst;
        if (use_rel_position != 0u)
        {
            const fp32 rel_position_rad = (fb.q - zero_rad) * watch_wheelleg_dir_sign(dir);
            out->rel_position_deg = watch_rad_to_deg(rel_position_rad);
        }
        if (out->cmd_active != 0u)
        {
            out->cmd_minus_fb_deg = watch_rad_to_deg(cmd.q - fb.q);
        }
    }
}

static void watch_copy_wheelleg_mit_foot_test(uint8_t side)
{
    watch_wheelleg_mit_foot_test_t *out;
    fp32 wheel_zero_rad = 0.0f;
    fp32 wheel_comp_rad = 0.0f;
    fp32 wheel_target_rad = 0.0f;

    if (side >= (uint8_t)WHEELLEG_SIDE_COUNT)
    {
        return;
    }

    out = &g_watch.wheelleg_mit.foot_test[side];
    wheelleg_mit_get_foot_test_target(side,
                                      &out->target_x_m,
                                      &out->target_y_m,
                                      &out->target_length_m);
    wheelleg_mit_get_foot_test_wheel(side,
                                     &out->wheel_zero_valid,
                                     &wheel_zero_rad,
                                     &out->wheel_dx_m,
                                     &wheel_comp_rad,
                                     &wheel_target_rad);
    out->wheel_zero_deg = watch_rad_to_deg(wheel_zero_rad);
    out->wheel_comp_deg = watch_rad_to_deg(wheel_comp_rad);
    out->wheel_target_deg = watch_rad_to_deg(wheel_target_rad);
}

static uint8_t watch_wheelleg_manual_enabled_by_switch(uint8_t chassis_sw)
{
    return (uint8_t)(control_input_switch_is_pos(chassis_sw,
                                                 g_config.manual_input.semantics.chassis_safe_pos) == 0u);
}

static void watch_copy_wheelleg_mit(void)
{
    const uint8_t profile_on = watch_block_active_wheelleg_mit();
    const uint8_t chassis_sw = input_switch(INPUT_SW_CHASSIS_MODE);
    const uint8_t run_variant = (uint8_t)robot_mode_variant();
    wheelleg_status_t status;
    wheelleg_state_t state;
    LowCmdDiag lowcmd_diag = {0};
    uint8_t status_valid;
    uint8_t state_valid;

    memset(&g_watch.wheelleg_mit, 0, sizeof(g_watch.wheelleg_mit));
    if (profile_on == 0u)
    {
        return;
    }

    status_valid = wheelleg_status_read(&status);
    state_valid = wheelleg_state_read(&state);
    (void)LowCmdGetDiag(&lowcmd_diag);

    g_watch.wheelleg_mit.status_valid = status_valid;
    g_watch.wheelleg_mit.state_valid = state_valid;
    g_watch.wheelleg_mit.profile_on = profile_on;
    g_watch.wheelleg_mit.lowcmd_seq = lowcmd_diag.seq;
    g_watch.wheelleg_mit.lowcmd_rejected_count = lowcmd_diag.rejected_count;
    g_watch.wheelleg_mit.lowcmd_emergency_stop_count = lowcmd_diag.emergency_stop_count;
    g_watch.wheelleg_mit.lowcmd_last_reject_tick_ms = lowcmd_diag.last_reject_tick;
    g_watch.wheelleg_mit.lowcmd_last_reject_writer = lowcmd_diag.last_reject_writer;
    g_watch.wheelleg_mit.lowcmd_last_reject_owner = lowcmd_diag.last_reject_owner;
    g_watch.wheelleg_mit.lowcmd_emergency_writer = lowcmd_diag.emergency_writer;
    g_watch.wheelleg_mit.lowcmd_emergency_active = lowcmd_diag.emergency_active;
    g_watch.wheelleg_mit.input_chassis_switch = chassis_sw;
    g_watch.wheelleg_mit.enable_switch_pos = g_config.wheelleg_mit.enable_switch_pos;
    g_watch.wheelleg_mit.manual_on = watch_wheelleg_manual_enabled_by_switch(chassis_sw);
    g_watch.wheelleg_mit.test_mode = run_variant;
    g_watch.wheelleg_mit.foot_test_phase = wheelleg_mit_get_foot_test_phase();
    g_watch.wheelleg_mit.foot_test_ik_ok = wheelleg_mit_get_foot_test_ik_ok();
    watch_copy_wheelleg_mit_foot_test((uint8_t)WHEELLEG_SIDE_LEFT);
    watch_copy_wheelleg_mit_foot_test((uint8_t)WHEELLEG_SIDE_RIGHT);

    if (status_valid != 0u)
    {
        g_watch.wheelleg_mit.mode = status.mode;
        g_watch.wheelleg_mit.last_mode = status.last_mode;
        g_watch.wheelleg_mit.fault_flags = status.fault_flags;
        g_watch.wheelleg_mit.health = status.health;
        g_watch.wheelleg_mit.controller_active = status.controller_active;
        g_watch.wheelleg_mit.active_controller_id = status.active_controller_id;
        g_watch.wheelleg_mit.target_v_mps = status.target_v_mps;
        g_watch.wheelleg_mit.target_yaw_rate_radps = status.target_yaw_rate_radps;
        g_watch.wheelleg_mit.target_leg_length_m = status.target_leg_length_m;
        g_watch.wheelleg_mit.target_foot_x_m = status.target_foot_x_m;
        g_watch.wheelleg_mit.target_foot_y_m = status.target_foot_y_m;
        g_watch.wheelleg_mit.target_leg_theta_deg = watch_rad_to_deg(status.target_leg_theta_rad);
        g_watch.wheelleg_mit.pitch_deg = watch_rad_to_deg(status.pitch_rad);
        g_watch.wheelleg_mit.x_dot_mps = status.x_dot_mps;
        g_watch.wheelleg_mit.enabled =
            (uint8_t)(g_watch.wheelleg_mit.manual_on != 0u &&
                      (status.fault_flags == WHEELLEG_FAULT_NONE ||
                       status.controller_active != 0u));

        g_watch.wheelleg_mit.leg[0].length_m = status.leg_length_m[WHEELLEG_SIDE_LEFT];
        g_watch.wheelleg_mit.leg[1].length_m = status.leg_length_m[WHEELLEG_SIDE_RIGHT];
        g_watch.wheelleg_mit.leg[0].theta_deg = watch_rad_to_deg(status.leg_theta_rad[WHEELLEG_SIDE_LEFT]);
        g_watch.wheelleg_mit.leg[1].theta_deg = watch_rad_to_deg(status.leg_theta_rad[WHEELLEG_SIDE_RIGHT]);
        g_watch.wheelleg_mit.leg[0].alpha_deg = watch_rad_to_deg(status.leg_alpha_rad[WHEELLEG_SIDE_LEFT]);
        g_watch.wheelleg_mit.leg[1].alpha_deg = watch_rad_to_deg(status.leg_alpha_rad[WHEELLEG_SIDE_RIGHT]);
        g_watch.wheelleg_mit.leg[0].support_force_n = status.support_force_n[WHEELLEG_SIDE_LEFT];
        g_watch.wheelleg_mit.leg[1].support_force_n = status.support_force_n[WHEELLEG_SIDE_RIGHT];
        g_watch.wheelleg_mit.leg[0].wheel_torque_nm = status.wheel_torque_nm[WHEELLEG_SIDE_LEFT];
        g_watch.wheelleg_mit.leg[1].wheel_torque_nm = status.wheel_torque_nm[WHEELLEG_SIDE_RIGHT];
    }

    if (state_valid != 0u)
    {
        g_watch.wheelleg_mit.leg[0].front_online =
            state.leg[WHEELLEG_SIDE_LEFT].motor_online[0];
        g_watch.wheelleg_mit.leg[0].back_online =
            state.leg[WHEELLEG_SIDE_LEFT].motor_online[1];
        g_watch.wheelleg_mit.leg[1].front_online =
            state.leg[WHEELLEG_SIDE_RIGHT].motor_online[0];
        g_watch.wheelleg_mit.leg[1].back_online =
            state.leg[WHEELLEG_SIDE_RIGHT].motor_online[1];
        g_watch.wheelleg_mit.leg[0].wheel_online = state.wheel_online[WHEELLEG_SIDE_LEFT];
        g_watch.wheelleg_mit.leg[1].wheel_online = state.wheel_online[WHEELLEG_SIDE_RIGHT];
    }

    watch_copy_wheelleg_mit_motor(&g_watch.wheelleg_mit.joint[0],
                                  g_config.wheelleg_mit.left_front_actuator,
                                  g_config.wheelleg_mit.left_front_zero_rad,
                                  g_config.wheelleg_mit.left_front_dir,
                                  1u);
    watch_copy_wheelleg_mit_motor(&g_watch.wheelleg_mit.joint[1],
                                  g_config.wheelleg_mit.left_back_actuator,
                                  g_config.wheelleg_mit.left_back_zero_rad,
                                  g_config.wheelleg_mit.left_back_dir,
                                  1u);
    watch_copy_wheelleg_mit_motor(&g_watch.wheelleg_mit.joint[2],
                                  g_config.wheelleg_mit.right_front_actuator,
                                  g_config.wheelleg_mit.right_front_zero_rad,
                                  g_config.wheelleg_mit.right_front_dir,
                                  1u);
    watch_copy_wheelleg_mit_motor(&g_watch.wheelleg_mit.joint[3],
                                  g_config.wheelleg_mit.right_back_actuator,
                                  g_config.wheelleg_mit.right_back_zero_rad,
                                  g_config.wheelleg_mit.right_back_dir,
                                  1u);
    watch_copy_wheelleg_mit_motor(&g_watch.wheelleg_mit.wheel[0],
                                  g_config.wheelleg_mit.left_wheel_actuator,
                                  0.0f,
                                  1,
                                  0u);
    watch_copy_wheelleg_mit_motor(&g_watch.wheelleg_mit.wheel[1],
                                  g_config.wheelleg_mit.right_wheel_actuator,
                                  0.0f,
                                  1,
                                  0u);
}

static void watch_wheelleg_run_capture_reset(void)
{
    memset(&s_wheelleg_run_capture, 0, sizeof(s_wheelleg_run_capture));
}

static void watch_wheelleg_run_capture_range(fp32 value, fp32 *min_value, fp32 *max_value, uint8_t first)
{
    if (first != 0u)
    {
        *min_value = value;
        *max_value = value;
        return;
    }
    if (value < *min_value)
    {
        *min_value = value;
    }
    if (value > *max_value)
    {
        *max_value = value;
    }
}

static void watch_wheelleg_run_capture_update(void)
{
    const uint8_t active =
        (uint8_t)((g_watch.wheelleg_mit.profile_on != 0u &&
                   g_watch.wheelleg_mit.status_valid != 0u &&
                   g_watch.wheelleg_mit.controller_active != 0u)
                      ? 1u
                      : 0u);
    watch_wheelleg_run_capture_t *capture = &s_wheelleg_run_capture;
    uint8_t first_sample;
    fp32 target_v;
    fp32 target_yaw_rate;
    fp32 x_dot;
    fp32 pitch;
    fp32 lqr_pitch_gyro;
    fp32 yaw_gyro;
    fp32 wheel_sum;
    fp32 wheel_diff;
    fp32 lqr_v_err;
    fp32 lqr_x;
    fp32 lqr_pitch_err;
    fp32 lqr_left_pitch_err;

    if (active == 0u)
    {
        capture->active = 0u;
        return;
    }

    if (capture->active == 0u)
    {
        watch_wheelleg_run_capture_reset();
        capture->active = 1u;
    }

    first_sample = (capture->sample_count == 0u) ? 1u : 0u;
    target_v = g_watch.diag.target_v_mps;
    target_yaw_rate = g_watch.diag.target_yaw_rate_radps;
    x_dot = g_watch.diag.x_dot_mps;
    pitch = g_watch.diag.pitch_deg;
    lqr_pitch_gyro = g_watch.diag.lqr_pitch_gyro_deg_s;
    yaw_gyro = g_watch.imu.gyro_dps[INS_GYRO_Z_ADDRESS_OFFSET];
    wheel_sum = g_watch.diag.left_wheel_cmd_torque_nm + g_watch.diag.right_wheel_cmd_torque_nm;
    wheel_diff = g_watch.diag.right_wheel_cmd_torque_nm - g_watch.diag.left_wheel_cmd_torque_nm;
    lqr_v_err = g_watch.diag.lqr_v_err_mps;
    lqr_x = g_watch.diag.lqr_x_m;
    lqr_pitch_err = g_watch.diag.lqr_pitch_err_deg;
    lqr_left_pitch_err = g_watch.diag.lqr_left_pitch_err_deg;

    capture->sample_count++;
    capture->valid = 1u;
    capture->target_v_sum += target_v;
    if (watch_abs_fp32(target_v) > capture->target_v_abs_max)
    {
        capture->target_v_abs_max = watch_abs_fp32(target_v);
    }
    capture->target_yaw_rate_sum += target_yaw_rate;
    if (watch_abs_fp32(target_yaw_rate) > capture->target_yaw_rate_abs_max)
    {
        capture->target_yaw_rate_abs_max = watch_abs_fp32(target_yaw_rate);
    }
    capture->x_dot_sum += x_dot;
    watch_wheelleg_run_capture_range(x_dot, &capture->x_dot_min, &capture->x_dot_max, first_sample);
    capture->pitch_sum += pitch;
    watch_wheelleg_run_capture_range(pitch, &capture->pitch_min, &capture->pitch_max, first_sample);
    capture->lqr_pitch_gyro_sum += lqr_pitch_gyro;
    watch_wheelleg_run_capture_range(lqr_pitch_gyro,
                                     &capture->lqr_pitch_gyro_min,
                                     &capture->lqr_pitch_gyro_max,
                                     first_sample);
    capture->yaw_gyro_sum += yaw_gyro;
    if (watch_abs_fp32(yaw_gyro) > capture->yaw_gyro_abs_max)
    {
        capture->yaw_gyro_abs_max = watch_abs_fp32(yaw_gyro);
    }
    capture->wheel_sum_sum += wheel_sum;
    if (watch_abs_fp32(wheel_sum) > capture->wheel_sum_abs_max)
    {
        capture->wheel_sum_abs_max = watch_abs_fp32(wheel_sum);
    }
    capture->wheel_diff_sum += wheel_diff;
    if (watch_abs_fp32(wheel_diff) > capture->wheel_diff_abs_max)
    {
        capture->wheel_diff_abs_max = watch_abs_fp32(wheel_diff);
    }
    capture->lqr_v_err_sum += lqr_v_err;
    watch_wheelleg_run_capture_range(lqr_v_err,
                                     &capture->lqr_v_err_min,
                                     &capture->lqr_v_err_max,
                                     first_sample);
    capture->lqr_x_sum += lqr_x;
    watch_wheelleg_run_capture_range(lqr_x,
                                     &capture->lqr_x_min,
                                     &capture->lqr_x_max,
                                     first_sample);
    capture->lqr_pitch_err_sum += lqr_pitch_err;
    watch_wheelleg_run_capture_range(lqr_pitch_err,
                                     &capture->lqr_pitch_err_min,
                                     &capture->lqr_pitch_err_max,
                                     first_sample);
    capture->lqr_left_pitch_err_sum += lqr_left_pitch_err;
    watch_wheelleg_run_capture_range(lqr_left_pitch_err,
                                     &capture->lqr_left_pitch_err_min,
                                     &capture->lqr_left_pitch_err_max,
                                     first_sample);
}

static void watch_wheelleg_run_capture_copy(void)
{
    const watch_wheelleg_run_capture_t *capture = &s_wheelleg_run_capture;
    fp32 inv_count;

    g_watch.diag.run_capture_active = capture->active;
    g_watch.diag.run_capture_valid = capture->valid;
    g_watch.diag.run_sample_count = capture->sample_count;

    if (capture->valid == 0u || capture->sample_count == 0u)
    {
        return;
    }

    inv_count = 1.0f / (fp32)capture->sample_count;
    g_watch.diag.run_target_v_avg_mps = capture->target_v_sum * inv_count;
    g_watch.diag.run_target_v_abs_max_mps = capture->target_v_abs_max;
    g_watch.diag.run_target_yaw_rate_avg_radps = capture->target_yaw_rate_sum * inv_count;
    g_watch.diag.run_target_yaw_rate_abs_max_radps = capture->target_yaw_rate_abs_max;
    g_watch.diag.run_x_dot_avg_mps = capture->x_dot_sum * inv_count;
    g_watch.diag.run_x_dot_min_mps = capture->x_dot_min;
    g_watch.diag.run_x_dot_max_mps = capture->x_dot_max;
    g_watch.diag.run_pitch_avg_deg = capture->pitch_sum * inv_count;
    g_watch.diag.run_pitch_min_deg = capture->pitch_min;
    g_watch.diag.run_pitch_max_deg = capture->pitch_max;
    g_watch.diag.run_lqr_pitch_gyro_avg_deg_s = capture->lqr_pitch_gyro_sum * inv_count;
    g_watch.diag.run_lqr_pitch_gyro_min_deg_s = capture->lqr_pitch_gyro_min;
    g_watch.diag.run_lqr_pitch_gyro_max_deg_s = capture->lqr_pitch_gyro_max;
    g_watch.diag.run_yaw_gyro_avg_deg_s = capture->yaw_gyro_sum * inv_count;
    g_watch.diag.run_yaw_gyro_abs_max_deg_s = capture->yaw_gyro_abs_max;
    g_watch.diag.run_wheel_sum_avg_nm = capture->wheel_sum_sum * inv_count;
    g_watch.diag.run_wheel_sum_abs_max_nm = capture->wheel_sum_abs_max;
    g_watch.diag.run_wheel_diff_avg_nm = capture->wheel_diff_sum * inv_count;
    g_watch.diag.run_wheel_diff_abs_max_nm = capture->wheel_diff_abs_max;
    g_watch.diag.run_wheel_balance_avg_nm = g_watch.diag.run_wheel_diff_avg_nm;
    g_watch.diag.run_wheel_balance_abs_max_nm = g_watch.diag.run_wheel_diff_abs_max_nm;
    g_watch.diag.run_wheel_turn_avg_nm = g_watch.diag.run_wheel_sum_avg_nm;
    g_watch.diag.run_wheel_turn_abs_max_nm = g_watch.diag.run_wheel_sum_abs_max_nm;
    g_watch.diag.run_lqr_v_err_avg_mps = capture->lqr_v_err_sum * inv_count;
    g_watch.diag.run_lqr_v_err_min_mps = capture->lqr_v_err_min;
    g_watch.diag.run_lqr_v_err_max_mps = capture->lqr_v_err_max;
    g_watch.diag.run_lqr_x_avg_m = capture->lqr_x_sum * inv_count;
    g_watch.diag.run_lqr_x_min_m = capture->lqr_x_min;
    g_watch.diag.run_lqr_x_max_m = capture->lqr_x_max;
    g_watch.diag.run_lqr_pitch_err_avg_deg = capture->lqr_pitch_err_sum * inv_count;
    g_watch.diag.run_lqr_pitch_err_min_deg = capture->lqr_pitch_err_min;
    g_watch.diag.run_lqr_pitch_err_max_deg = capture->lqr_pitch_err_max;
    g_watch.diag.run_lqr_left_pitch_err_avg_deg = capture->lqr_left_pitch_err_sum * inv_count;
    g_watch.diag.run_lqr_left_pitch_err_min_deg = capture->lqr_left_pitch_err_min;
    g_watch.diag.run_lqr_left_pitch_err_max_deg = capture->lqr_left_pitch_err_max;
}
#endif
