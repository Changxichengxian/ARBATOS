/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Private implementation chunk included by wheelleg_mit_task.c.
 */

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

static void wheelleg_SdLogWrite_config_once(void)
{
    sdlog_wheelleg_mit_config_t log;
    const wheelleg_mit_config_t *cfg = &g_config.wheelleg_mit;
    uint8_t i;

    if (!SdLogIsActive())
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

    SdLogWrite(SDLOG_TAG_WHEELLEG_MIT_CONFIG, &log, (uint16_t)sizeof(log));
    s_wheelleg_sdlog_config_logged = 1u;
}

static void wheelleg_SdLogWrite_status(uint16_t faults,
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
                                   (uint32_t)SdLogHighRateDiv();
    const fp32 lqr_x_err = wheelleg_lqr_x_error(target_v, target_yaw_rate);
    uint8_t side;

    if (!SdLogIsActive())
    {
        s_wheelleg_sdlog_config_logged = 0u;
        s_wheelleg_sdlog_last_status_ms = 0u;
        return;
    }

    wheelleg_SdLogWrite_config_once();

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
    log.lqr_error[4] = pitch - wheelleg_lqr_pitch_offset_for_side(WHEELLEG_SIDE_RIGHT);
    log.lqr_error[5] = (gyro != NULL) ? gyro[INS_GYRO_Y_ADDRESS_OFFSET] : 0.0f;
    log.lqr_output[0] = (wheel_torque != NULL) ? wheel_torque[WHEELLEG_SIDE_RIGHT] : 0.0f;
    log.lqr_output[1] = (wheel_torque != NULL) ? wheel_torque[WHEELLEG_SIDE_LEFT] : 0.0f;
    log.lqr_output[2] = s_wheelleg.leg[WHEELLEG_SIDE_RIGHT].tp;
    log.lqr_output[3] = s_wheelleg.leg[WHEELLEG_SIDE_LEFT].tp;
    log.overrun_count = s_wheelleg.overrun_count;

    SdLogWrite(SDLOG_TAG_WHEELLEG_MIT_STATUS, &log, (uint16_t)sizeof(log));
    wheelleg_SdLogWrite_motor_diag(now_ms);
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

static void wheelleg_sdlog_fill_motor_diag(sdlog_wheelleg_mit_motor_diag_entry_t *entry,
                                           uint8_t role,
                                           MotorId id,
                                           const MotorState *fb,
                                           uint32_t now_ms)
{
    MotorCmd cmd;
    MotorApplied applied;

    if (entry == NULL)
    {
        return;
    }

    (void)memset(entry, 0, sizeof(*entry));
    entry->role = role;
    entry->actuator_id = (uint8_t)id;
    if ((uint32_t)id >= (uint32_t)MotorCount)
    {
        return;
    }

    if (fb != NULL)
    {
        entry->fresh = wheelleg_feedback_fresh(fb, now_ms);
        entry->fb_online = fb->online;
        entry->fb_bus = fb->bus;
        entry->fb_rx_dlc = fb->rxDlc;
        entry->fb_rx_data0 = fb->rxData0;
        entry->fb_rx_data0_low4 = (uint8_t)(fb->rxData0 & 0x0Fu);
        entry->fb_rx_data0_high4 = (uint8_t)(fb->rxData0 >> 4);
        entry->fb_motor_id = fb->motorId;
        entry->fb_state = fb->state;
        entry->fb_rx_id = fb->rxId;
        entry->fb_rx_count = fb->rxCount;
        entry->fb_last_rx_tick_ms = fb->lastRxTick;
        entry->fb_position_rad = fb->q;
        entry->fb_velocity_radps = fb->dq;
        entry->fb_torque_nm = fb->tauEst;
    }

    if (LowCmdGetMotor(id, &cmd) != 0u)
    {
        entry->cmd_active = cmd.active;
        entry->cmd_mode = cmd.mode;
        entry->cmd_writer = cmd.writer;
        entry->cmd_timeout_ms = cmd.timeoutMs;
        entry->cmd_seq = cmd.seq;
        entry->cmd_tick_ms = cmd.tick;
        entry->cmd_position_rad = cmd.q;
        entry->cmd_velocity_radps = cmd.dq;
        entry->cmd_kp = cmd.kp;
        entry->cmd_kd = cmd.kd;
        entry->cmd_torque_nm = cmd.tau;
    }

    if (LowStateGetApplied(id, &applied) != 0u)
    {
        entry->applied_active = applied.active;
        entry->applied_mode = applied.mode;
        entry->applied_drive_state = applied.driveState;
        entry->applied_flags = applied.flags;
        entry->applied_tx_id = applied.txId;
        entry->applied_tick_ms = applied.tick;
        entry->applied_torque_nm = applied.tau;
        entry->tx_id_count = bsp_can_get_tx_std_id_count(applied.bus, applied.txId);
    }
}

static void wheelleg_SdLogWrite_motor_diag(uint32_t now_ms)
{
    sdlog_wheelleg_mit_motor_diag_t log;
    const wheelleg_mit_config_t *cfg = &g_config.wheelleg_mit;

    (void)memset(&log, 0, sizeof(log));
    log.version = SDLOG_WHEELLEG_MIT_MOTOR_DIAG_VERSION;
    log.count = SDLOG_WHEELLEG_MIT_MOTOR_DIAG_COUNT;
    log.can1_rx_count = CAN_get_can1_rx_count();
    log.can1_rx_drop = CAN_get_can1_rx_drop_count();
    log.can1_tx_count = CAN_get_can1_tx_count();
    log.can1_tx_fail = CAN_get_can1_tx_fail_count();
    log.can2_rx_count = CAN_get_can2_rx_count();
    log.can2_rx_drop = CAN_get_can2_rx_drop_count();
    log.can2_tx_count = CAN_get_can2_tx_count();
    log.can2_tx_fail = CAN_get_can2_tx_fail_count();

    wheelleg_sdlog_fill_motor_diag(&log.motor[0], 0u, (MotorId)cfg->left_front_actuator, &s_wheelleg.front_fb[WHEELLEG_SIDE_LEFT], now_ms);
    wheelleg_sdlog_fill_motor_diag(&log.motor[1], 1u, (MotorId)cfg->left_back_actuator, &s_wheelleg.back_fb[WHEELLEG_SIDE_LEFT], now_ms);
    wheelleg_sdlog_fill_motor_diag(&log.motor[2], 2u, (MotorId)cfg->left_wheel_actuator, &s_wheelleg.wheel_fb[WHEELLEG_SIDE_LEFT], now_ms);
    wheelleg_sdlog_fill_motor_diag(&log.motor[3], 3u, (MotorId)cfg->right_front_actuator, &s_wheelleg.front_fb[WHEELLEG_SIDE_RIGHT], now_ms);
    wheelleg_sdlog_fill_motor_diag(&log.motor[4], 4u, (MotorId)cfg->right_back_actuator, &s_wheelleg.back_fb[WHEELLEG_SIDE_RIGHT], now_ms);
    wheelleg_sdlog_fill_motor_diag(&log.motor[5], 5u, (MotorId)cfg->right_wheel_actuator, &s_wheelleg.wheel_fb[WHEELLEG_SIDE_RIGHT], now_ms);

    SdLogWrite(SDLOG_TAG_WHEELLEG_MIT_MOTOR_DIAG, &log, (uint16_t)sizeof(log));
}
