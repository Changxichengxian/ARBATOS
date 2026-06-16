/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Private implementation chunk included by wheelleg_mit_task.c.
 */

static fp32 wheelleg_axis_to_fp32(int16_t axis, fp32 max_abs, uint16_t deadband)
{
    return wheelleg_core_axis_to_fp32(axis, max_abs, deadband, RC_CH_VALUE_ABS_LEGACY);
}

static fp32 wheelleg_observer_wheel_velocity(uint8_t side)
{
    const fp32 raw_velocity =
        (side < WHEELLEG_SIDE_COUNT) ? s_wheelleg.wheel_fb[side].dq : 0.0f;

    return (side == WHEELLEG_SIDE_RIGHT) ? -raw_velocity : raw_velocity;
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
                                  wheelleg_observer_wheel_velocity(WHEELLEG_SIDE_LEFT),
                                  wheelleg_observer_wheel_velocity(WHEELLEG_SIDE_RIGHT),
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
    debug.lqr.ref[5] = s_wheelleg.pitch_auto_offset_rad;
    debug.lqr.error[0] = s_wheelleg.leg[WHEELLEG_SIDE_RIGHT].theta - target_theta;
    debug.lqr.error[1] = s_wheelleg.leg[WHEELLEG_SIDE_RIGHT].d_theta;
    debug.lqr.error[2] = lqr_x_err;
    debug.lqr.error[3] = s_wheelleg.v_mps - target_v;
    debug.lqr.error[4] = pitch - wheelleg_lqr_pitch_offset_for_side(WHEELLEG_SIDE_RIGHT);
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

    wheelleg_SdLogWrite_status(faults,
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
    wheelleg_lqr_side_state_t right_state;
    wheelleg_lqr_side_state_t left_state;
    fp32 yaw_gyro = (gyro != NULL) ? gyro[INS_GYRO_Z_ADDRESS_OFFSET] : 0.0f;
    fp32 turn_t;
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

    wheelleg_lqr_side_state(WHEELLEG_SIDE_RIGHT,
                            pitch,
                            gyro,
                            target_theta,
                            target_v,
                            target_yaw_rate,
                            &right_state);
    wheelleg_lqr_side_state(WHEELLEG_SIDE_LEFT,
                            pitch,
                            gyro,
                            target_theta,
                            target_v,
                            target_yaw_rate,
                            &left_state);

    wheel_torque[WHEELLEG_SIDE_RIGHT] =
        wheelleg_core_lqr_wheel_output(k_right,
                                       right_state.theta_err,
                                       right_state.d_theta,
                                       right_state.x_err,
                                       right_state.v_err,
                                       right_state.pitch_err,
                                       right_state.gyro_y);
    wheel_torque[WHEELLEG_SIDE_LEFT] =
        wheelleg_core_lqr_wheel_output(k_left,
                                       left_state.theta_err,
                                       left_state.d_theta,
                                       left_state.x_err,
                                       left_state.v_err,
                                       left_state.pitch_err,
                                       left_state.gyro_y);

    for (i = 0u; i < WHEELLEG_SIDE_COUNT; i++)
    {
        wheel_torque[i] = wheelleg_lqr_wheel_torque_to_motor(i, wheel_torque[i], wheel_scale, turn_t);
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
    wheelleg_lqr_side_state_t right_state;
    wheelleg_lqr_side_state_t left_state;
    fp32 yaw_gyro = (gyro != NULL) ? gyro[INS_GYRO_Z_ADDRESS_OFFSET] : 0.0f;
    fp32 roll_gyro = (gyro != NULL) ? gyro[INS_GYRO_X_ADDRESS_OFFSET] : 0.0f;
    fp32 turn_t;
    fp32 roll_f0;
    fp32 split_tp;
    fp32 cos_theta;
    const fp32 target_theta = wheelleg_target_theta_from_foot_x(target_foot_x, target_leg);
    const fp32 wheel_scale = wheelleg_lqr_wheel_scale();
    const fp32 hip_scale = (use_lqr_hip != 0u) ? wheelleg_lqr_hip_scale() : 0.0f;
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

    wheelleg_lqr_side_state(WHEELLEG_SIDE_RIGHT,
                            pitch,
                            gyro,
                            target_theta,
                            target_v,
                            target_yaw_rate,
                            &right_state);
    wheelleg_lqr_side_state(WHEELLEG_SIDE_LEFT,
                            pitch,
                            gyro,
                            target_theta,
                            target_v,
                            target_yaw_rate,
                            &left_state);

    wheel_torque[WHEELLEG_SIDE_RIGHT] =
        wheelleg_core_lqr_wheel_output(k_right,
                                       right_state.theta_err,
                                       right_state.d_theta,
                                       right_state.x_err,
                                       right_state.v_err,
                                       right_state.pitch_err,
                                       right_state.gyro_y);
    s_wheelleg.leg[WHEELLEG_SIDE_RIGHT].tp = 0.0f;
    if (use_lqr_hip != 0u)
    {
        s_wheelleg.leg[WHEELLEG_SIDE_RIGHT].tp =
            hip_scale * wheelleg_core_lqr_hip_output(k_right,
                                                     right_state.theta_err,
                                                     right_state.d_theta,
                                                     right_state.x_err,
                                                     right_state.v_err,
                                                     right_state.pitch_err,
                                                     right_state.gyro_y,
                                                     split_tp);
    }

    wheel_torque[WHEELLEG_SIDE_LEFT] =
        wheelleg_core_lqr_wheel_output(k_left,
                                       left_state.theta_err,
                                       left_state.d_theta,
                                       left_state.x_err,
                                       left_state.v_err,
                                       left_state.pitch_err,
                                       left_state.gyro_y);
    s_wheelleg.leg[WHEELLEG_SIDE_LEFT].tp = 0.0f;
    if (use_lqr_hip != 0u)
    {
        s_wheelleg.leg[WHEELLEG_SIDE_LEFT].tp =
            hip_scale * wheelleg_core_lqr_hip_output(k_left,
                                                     left_state.theta_err,
                                                     left_state.d_theta,
                                                     left_state.x_err,
                                                     left_state.v_err,
                                                     left_state.pitch_err,
                                                     left_state.gyro_y,
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
        wheel_torque[i] = wheelleg_lqr_wheel_torque_to_motor(i, wheel_torque[i], wheel_scale, turn_t);
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
