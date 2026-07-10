/*
 * SPDX-FileCopyrightText: 2026 ?? <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Private implementation chunk included by ChassisControlTask.c.
 */

static fp32 ChassisGetGimbalYawRelativeAngle(const GimbalMotorState *yaw_motor)
{
    if (yaw_motor == NULL || yaw_motor->valid == 0u || yaw_motor->measure.valid == 0u)
    {
        return 0.0f;
    }

    const uint32_t ecd_range = MotorCfgEncoderRange(g_config.motor.yaw.model);
    const int32_t half_ecd_range = (int32_t)(ecd_range / 2u);
    const int32_t full_ecd_range = (int32_t)ecd_range;
    int32_t relative_ecd = (int32_t)yaw_motor->measure.ecd - (int32_t)yaw_motor->offset_ecd;
    if (relative_ecd > half_ecd_range)
    {
        relative_ecd -= full_ecd_range;
    }
    else if (relative_ecd < -half_ecd_range)
    {
        relative_ecd += full_ecd_range;
    }

    fp32 angle = (fp32)relative_ecd * (6.28318530718f / (fp32)ecd_range);
    return CHASSIS_GIMBAL_YAW_RELATIVE_TURN ? -angle : angle;
}

static fp32 ChassisGetGimbalYawRelativeRate(const GimbalMotorState *yaw_motor)
{
    if (yaw_motor == NULL || yaw_motor->valid == 0u || yaw_motor->measure.valid == 0u)
    {
        return 0.0f;
    }

    fp32 w = (fp32)yaw_motor->measure.speed_rpm * CHASSIS_GIMBAL_MOTOR_RPM_TO_RADPS;
    return CHASSIS_GIMBAL_YAW_RELATIVE_TURN ? -w : w;
}

static bool_t ChassisGetTurnaroundFrameYaw(fp32 *out_yaw_relative)
{
    GimbalState state;
    if (out_yaw_relative == NULL ||
        GimbalStateRead(&state) == 0u ||
        state.valid == 0u ||
        state.turnaround_frame_valid == 0u)
    {
        return 0;
    }

    *out_yaw_relative = state.turnaround_frame_yaw_relative;
    return 1;
}

static void ChassisFillMotorMeasureState(motor_measure_state_t *out, const motor_measure_t *measure)
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

static void ChassisWzKfReset(fp32 wz0)
{
    kalman_2x1_init(&ChassisWzKf,
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
    ChassisWzKfInited = 1;
}

static fp32 ChassisWzKfStep(fp32 wz_wheel, bool_t imu_valid, fp32 wz_imu)
{
    if (!ChassisWzKfInited)
    {
        ChassisWzKfReset(wz_wheel);
    }

    // random-walk model: x(k+1) = x(k) + w
    kalman_2x1_predict(&ChassisWzKf, 1.0f, 0.0f, 0.0f, 1.0f);

    // wheel measurement: z = wz + v
    kalman_2x1_set_H(&ChassisWzKf, 1.0f, 0.0f);
    kalman_2x1_set_R(&ChassisWzKf, CHASSIS_WZ_KF_R_WHEEL);
    kalman_2x1_update(&ChassisWzKf, wz_wheel);

    // IMU measurement (after gimbal-yaw rate removal): z = wz + bias + v
    if (imu_valid)
    {
        kalman_2x1_set_H(&ChassisWzKf, 1.0f, 1.0f);
        kalman_2x1_set_R(&ChassisWzKf, CHASSIS_WZ_KF_R_IMU);
        kalman_2x1_update(&ChassisWzKf, wz_imu);
    }

    return ChassisWzKf.x0;
}

static void ChassisSnapshotCapture(ChassisRuntimeSnapshot *snapshot, ChassisMove *control);
static bool_t operation_mode_allow_chassis(const ChassisRuntimeSnapshot *snapshot)
{
    (void)snapshot;
    return (robot_mode_allow_chassis() != 0u) ? 1 : 0;
}

static void ChassisSnapshotCapture(ChassisRuntimeSnapshot *snapshot, ChassisMove *control)
{
    if (snapshot == NULL)
    {
        return;
    }

    memset(snapshot, 0, sizeof(*snapshot));
    if (ManualInputGetCurrentCopy(&snapshot->ManualInputCopy) != 0u)
    {
        snapshot->manual_input = &snapshot->ManualInputCopy;
    }
    {
        GimbalState state;
        if (GimbalStateRead(&state) != 0u && state.valid != 0u)
        {
            snapshot->GimbalStateValid = 1u;
            snapshot->GimbalOnline = state.online;
            snapshot->yaw_motor = state.yaw;
            snapshot->pitch_motor = state.pitch;
        }
        else if (control != NULL && control->GimbalStateValid != 0u)
        {
            snapshot->GimbalStateValid = 1u;
            snapshot->GimbalOnline = control->GimbalOnline;
            snapshot->yaw_motor = control->ChassisYawMotor;
            snapshot->pitch_motor = control->ChassisPitchMotor;
        }
    }
    snapshot->period_ms = RobotProfileChassisControlPeriodMs();
    snapshot->period_us = (uint32_t)snapshot->period_ms * 1000u;
    snapshot->period_s = (fp32)RobotProfileChassisControlPeriodS();
    if (InsSnapshotRead(&snapshot->ImuSnapshot) != 0 &&
        snapshot->ImuSnapshot.age_ms <= (uint32_t)snapshot->period_ms * 3u + 2u)
    {
        snapshot->ins_angle = snapshot->ImuSnapshot.angle;
        snapshot->gyro = snapshot->ImuSnapshot.gyro;
    }
    snapshot->ChassisOnlyMode = robot_mode_force_chassis_only();
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
        snapshot->motor_measure[i] = (control != NULL) ? control->motor_chassis[i].ChassisMotorMeasure : get_chassis_motor_measure_point(i);
        snapshot->motor_cfg[i] = &g_config.motor.chassis[i];
        snapshot->motor_dir[i] = g_config.chassis.motor_dir[i];
    }

    if (control != NULL)
    {
        ChassisControlSnapshot fast = {0};
        const ManualInputState *manual_input = snapshot->manual_input;

        if (snapshot->GimbalStateValid != 0u)
        {
            control->GimbalStateValid = 1u;
            control->GimbalOnline = snapshot->GimbalOnline;
            control->ChassisYawMotor = snapshot->yaw_motor;
            control->ChassisPitchMotor = snapshot->pitch_motor;
        }

        fast.manual_online = toe_is_error(DBUS_TOE) ? 0u : 1u;
        fast.ChassisOnlyMode = snapshot->ChassisOnlyMode;
        fast.mode_sw = ControlInputSwitch(INPUT_SW_CHASSIS_MODE);
        fast.safe_pos = g_config.manual_input.semantics.ChassisSafePos;
        fast.follow_pos = g_config.manual_input.semantics.ChassisFollowPos;
        fast.spin_pos = g_config.manual_input.semantics.ChassisSpinPos;
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
