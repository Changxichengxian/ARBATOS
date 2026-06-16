/*
 * SPDX-FileCopyrightText: 2026 ?? <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Private implementation chunk included by chassis_control_task.c.
 */

static fp32 chassis_get_gimbal_yaw_relative_angle(const gimbal_motor_state_t *yaw_motor)
{
    if (yaw_motor == NULL || yaw_motor->valid == 0u || yaw_motor->measure.valid == 0u)
    {
        return 0.0f;
    }

    int32_t relative_ecd = (int32_t)yaw_motor->measure.ecd - (int32_t)yaw_motor->offset_ecd;
    if (relative_ecd > HALF_ECD_RANGE)
    {
        relative_ecd -= ECD_RANGE;
    }
    else if (relative_ecd < -HALF_ECD_RANGE)
    {
        relative_ecd += ECD_RANGE;
    }

    fp32 angle = (fp32)relative_ecd * MOTOR_ECD_TO_RAD;
    return YAW_TURN ? -angle : angle;
}

static fp32 chassis_get_gimbal_yaw_relative_rate(const gimbal_motor_state_t *yaw_motor)
{
    if (yaw_motor == NULL || yaw_motor->valid == 0u || yaw_motor->measure.valid == 0u)
    {
        return 0.0f;
    }

    fp32 w = (fp32)yaw_motor->measure.speed_rpm * CHASSIS_GIMBAL_MOTOR_RPM_TO_RADPS;
    return YAW_TURN ? -w : w;
}

static bool_t chassis_get_turnaround_frame_yaw(fp32 *out_yaw_relative)
{
    gimbal_state_t gimbal_state;
    if (out_yaw_relative == NULL ||
        gimbal_state_read(&gimbal_state) == 0u ||
        gimbal_state.valid == 0u ||
        gimbal_state.turnaround_frame_valid == 0u)
    {
        return 0;
    }

    *out_yaw_relative = gimbal_state.turnaround_frame_yaw_relative;
    return 1;
}

static void chassis_fill_motor_measure_state(motor_measure_state_t *out, const motor_measure_t *measure)
{
    if (out == NULL)
    {
        return;
    }

    memset(out, 0, sizeof(*out));
    if (measure == NULL)
    {
        return;
    }

    out->valid = 1u;
    out->ecd = measure->ecd;
    out->speed_rpm = measure->speed_rpm;
    out->given_current = measure->given_current;
    out->temperature = measure->temperate;
    out->last_ecd = measure->last_ecd;
}

static void chassis_wz_kf_reset(fp32 wz0)
{
    kalman_2x1_init(&chassis_wz_kf,
                    wz0,
                    0.0f,
                    1.0f,
                    0.0f,
                    1.0f,
                    CHASSIS_WZ_KF_Q_WZ,
                    0.0f,
                    CHASSIS_WZ_KF_Q_GYRO_BIAS,
                    CHASSIS_WZ_KF_R_WHEEL,
                    1.0f,
                    0.0f);
    chassis_wz_kf_inited = 1;
}

static fp32 chassis_wz_kf_step(fp32 wz_wheel, bool_t imu_valid, fp32 wz_imu)
{
    if (!chassis_wz_kf_inited)
    {
        chassis_wz_kf_reset(wz_wheel);
    }

    // random-walk model: x(k+1) = x(k) + w
    kalman_2x1_predict(&chassis_wz_kf, 1.0f, 0.0f, 0.0f, 1.0f);

    // wheel measurement: z = wz + v
    kalman_2x1_set_H(&chassis_wz_kf, 1.0f, 0.0f);
    kalman_2x1_set_R(&chassis_wz_kf, CHASSIS_WZ_KF_R_WHEEL);
    kalman_2x1_update(&chassis_wz_kf, wz_wheel);

    // IMU measurement (after gimbal-yaw rate removal): z = wz + bias + v
    if (imu_valid)
    {
        kalman_2x1_set_H(&chassis_wz_kf, 1.0f, 1.0f);
        kalman_2x1_set_R(&chassis_wz_kf, CHASSIS_WZ_KF_R_IMU);
        kalman_2x1_update(&chassis_wz_kf, wz_imu);
    }

    return chassis_wz_kf.x0;
}

static void chassis_snapshot_capture(chassis_runtime_snapshot_t *snapshot, chassis_move_t *control);
static bool_t operation_mode_allow_chassis(const chassis_runtime_snapshot_t *snapshot)
{
    (void)snapshot;
    return (robot_mode_allow_chassis() != 0u) ? 1 : 0;
}

static void chassis_snapshot_capture(chassis_runtime_snapshot_t *snapshot, chassis_move_t *control)
{
    if (snapshot == NULL)
    {
        return;
    }

    memset(snapshot, 0, sizeof(*snapshot));
    if (manual_input_get_current_copy(&snapshot->manual_input_copy) != 0u)
    {
        snapshot->manual_input = &snapshot->manual_input_copy;
    }
    {
        gimbal_state_t gimbal_state;
        if (gimbal_state_read(&gimbal_state) != 0u && gimbal_state.valid != 0u)
        {
            snapshot->gimbal_state_valid = 1u;
            snapshot->gimbal_online = gimbal_state.online;
            snapshot->yaw_motor = gimbal_state.yaw;
            snapshot->pitch_motor = gimbal_state.pitch;
        }
        else if (control != NULL && control->gimbal_state_valid != 0u)
        {
            snapshot->gimbal_state_valid = 1u;
            snapshot->gimbal_online = control->gimbal_online;
            snapshot->yaw_motor = control->chassis_yaw_motor;
            snapshot->pitch_motor = control->chassis_pitch_motor;
        }
    }
    snapshot->ins_angle = (control != NULL) ? control->chassis_INS_angle : get_INS_angle_point();
    snapshot->gyro = chassis_INT_gyro_point;
    snapshot->chassis_only_mode = robot_mode_force_chassis_only();
    snapshot->period_ms = robot_profile_chassis_control_period_ms();
    snapshot->period_us = (uint32_t)snapshot->period_ms * 1000u;
    snapshot->period_s = (fp32)robot_profile_chassis_control_period_s();
    snapshot->control_hz = (snapshot->period_s > 0.0f) ? (1.0f / snapshot->period_s) : CHASSIS_CONTROL_FREQUENCE;
    snapshot->motor_rpm_to_vector = M3508_MOTOR_RPM_TO_VECTOR;
    snapshot->motor_speed_to_vx = MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
    snapshot->motor_speed_to_vy = MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
    snapshot->motor_speed_to_wz = MOTOR_SPEED_TO_CHASSIS_SPEED_WZ;
    snapshot->motor_distance_to_center = MOTOR_DISTANCE_TO_CENTER;
    snapshot->max_wheel_speed = MAX_WHEEL_SPEED;
    snapshot->wheel_type = g_config.chassis.wheel_type;

    for (uint8_t i = 0u; i < CHASSIS_MOTOR_COUNT; i++)
    {
        snapshot->motor_measure[i] = (control != NULL) ? control->motor_chassis[i].chassis_motor_measure : get_chassis_motor_measure_point(i);
        snapshot->motor_cfg[i] = &g_config.motor.chassis[i];
        snapshot->motor_dir[i] = g_config.chassis.motor_dir[i];
    }

    if (control != NULL)
    {
        chassis_control_snapshot_t fast = {0};
        const manual_input_state_t *manual_input = snapshot->manual_input;

        if (snapshot->gimbal_state_valid != 0u)
        {
            control->gimbal_state_valid = 1u;
            control->gimbal_online = snapshot->gimbal_online;
            control->chassis_yaw_motor = snapshot->yaw_motor;
            control->chassis_pitch_motor = snapshot->pitch_motor;
        }

        fast.manual_online = toe_is_error(DBUS_TOE) ? 0u : 1u;
        fast.chassis_only_mode = snapshot->chassis_only_mode;
        fast.mode_sw = input_switch(INPUT_SW_CHASSIS_MODE);
        fast.safe_pos = g_config.manual_input.semantics.chassis_safe_pos;
        fast.follow_pos = g_config.manual_input.semantics.chassis_follow_pos;
        fast.spin_pos = g_config.manual_input.semantics.chassis_spin_pos;
        fast.axis_x = input_axis(INPUT_AXIS_CHASSIS_X);
        fast.axis_y = input_axis(INPUT_AXIS_CHASSIS_Y);
        fast.axis_wz = input_axis(INPUT_AXIS_CHASSIS_WZ);
        fast.rc_deadband = CHASSIS_RC_DEADLINE;
        fast.key_mask = (manual_input != NULL) ? manual_input->key.v : 0u;
        fast.front_key_mask = CHASSIS_FRONT_KEY;
        fast.back_key_mask = CHASSIS_BACK_KEY;
        fast.left_key_mask = CHASSIS_LEFT_KEY;
        fast.right_key_mask = CHASSIS_RIGHT_KEY;
        fast.gyro_spin_key_mask = SWING_KEY;
        fast.gyro_spin_var_key_mask = CHASSIS_GYRO_SPIN_VAR_KEY;
        fast.swing_key_mask = CHASSIS_SWING_KEY;
        fast.vx_rc_sen = CHASSIS_VX_RC_SEN;
        fast.vy_rc_sen = CHASSIS_VY_RC_SEN;
        fast.angle_z_rc_sen = CHASSIS_ANGLE_Z_RC_SEN;
        fast.wz_rc_sen = CHASSIS_WZ_RC_SEN;
        fast.open_rc_scale = (fp32)CHASSIS_OPEN_RC_SCALE;
        fast.swing_no_move_angle = SWING_NO_MOVE_ANGLE;
        fast.swing_move_angle = SWING_MOVE_ANGLE;
        fast.swing_amp_rad = CHASSIS_SWING_AMP_RAD;
        fast.swing_half_period_ms = (uint32_t)CHASSIS_SWING_HALF_PERIOD_MS;
        fast.swing_center_hold_min_ms = (uint32_t)CHASSIS_SWING_CENTER_HOLD_MIN_MS;
        fast.swing_center_hold_max_ms = (uint32_t)CHASSIS_SWING_CENTER_HOLD_MAX_MS;

        control->fast = fast;
    }
}
