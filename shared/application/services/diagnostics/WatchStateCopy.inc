/*
 * SPDX-FileCopyrightText: 2026 ?? <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Private implementation chunk included by Watch.c.
 */

static void WatchCopyImu(void)
{
    const fp32 rad2deg = 57.29577951308232f;

    if (ins_quat_src != NULL)
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            g_watch.imu.quat[i] = ins_quat_src[i];
        }
    }

    if (ins_angle_src != NULL)
    {
        for (uint8_t i = 0; i < 3; i++)
        {
            g_watch.imu.angle_deg[i] = ins_angle_src[i] * rad2deg;
        }
    }

    if (ins_gyro_src != NULL)
    {
        for (uint8_t i = 0; i < 3; i++)
        {
            g_watch.imu.gyro_dps[i] = ins_gyro_src[i] * rad2deg;
        }
    }

    if (ins_accel_src != NULL)
    {
        for (uint8_t i = 0; i < 3; i++)
        {
            g_watch.imu.accel[i] = ins_accel_src[i];
        }
    }
}

#if WATCH_ENABLE_LOCOMOTION_CLASSIC
static void WatchCopyChassis(void)
{
    const fp32 rad2deg = 57.29577951308232f;

    if (!WatchBlockActiveLocomotionClassic())
    {
        memset(&g_watch.chassis, 0, sizeof(g_watch.chassis));
        return;
    }

    ChassisState chassis;
    if (ChassisStateRead(&chassis) == 0u || chassis.valid == 0u)
    {
        memset(&g_watch.chassis, 0, sizeof(g_watch.chassis));
        return;
    }

    g_watch.chassis.mode = (WatchChassisMode)chassis.mode;
    g_watch.chassis.last_mode = (WatchChassisMode)chassis.last_mode;

    g_watch.chassis.vx_set = chassis.vx_set;
    g_watch.chassis.vy_set = chassis.vy_set;
    g_watch.chassis.wz_set = chassis.wz_set;
    g_watch.chassis.vx = chassis.vx;
    g_watch.chassis.vy = chassis.vy;
    g_watch.chassis.wz = chassis.wz;
    g_watch.chassis.yaw_deg = chassis.ChassisYaw * rad2deg;
    g_watch.chassis.fault_configured_mask = chassis.fault_configured_mask;
    g_watch.chassis.fault_active_mask = chassis.fault_active_mask;
    g_watch.chassis.fault_blocking_mask = chassis.fault_blocking_mask;
    g_watch.chassis.fault_recovery_mask = chassis.fault_recovery_mask;
    g_watch.chassis.fault_inhibit_mask = chassis.fault_inhibit_mask;
    g_watch.chassis.fault_hold_zero_mask = chassis.fault_hold_zero_mask;
    g_watch.chassis.fault_recovery_input_safe = chassis.fault_recovery_input_safe;
    g_watch.chassis.fault_inhibit_fail_count = chassis.fault_inhibit_fail_count;
    g_watch.chassis.fault_release_fail_count = chassis.fault_release_fail_count;

    for (uint8_t i = 0; i < 4u && i < CHASSIS_STATE_MOTOR_COUNT; i++)
    {
        const ChassisMotorState *m = &chassis.motor[i];
        g_watch.chassis.motor_rpm[i] = (m->measure.valid != 0u) ? m->measure.speed_rpm : 0;
        g_watch.chassis.motor_current[i] = m->give_current;
        g_watch.chassis.motor_speed_set[i] = m->speed_set;
        g_watch.chassis.motor_ecd[i] = (m->measure.valid != 0u) ? m->measure.ecd : 0;
        g_watch.chassis.motor_temp[i] = (m->measure.valid != 0u) ? m->measure.temperature : 0;
        g_watch.chassis.motor_reason_mask[i] = chassis.motor_reason_mask[i];
        g_watch.chassis.motor_feedback_age_ms[i] = chassis.motor_feedback_age_ms[i];
    }
}
#endif

#if WATCH_ENABLE_GIMBAL_SINGLE
static void WatchCopyGimbal(void)
{
    if (!WatchBlockActiveGimbalSingle())
    {
        memset(&g_watch.gimbal, 0, sizeof(g_watch.gimbal));
        return;
    }

    GimbalState gimbal;
    if (GimbalStateRead(&gimbal) == 0u || gimbal.valid == 0u)
    {
        memset(&g_watch.gimbal, 0, sizeof(g_watch.gimbal));
        return;
    }

    const GimbalMotorState *yaw = &gimbal.yaw;
    const GimbalMotorState *pitch = &gimbal.pitch;

    g_watch.gimbal.yaw_online = gimbal.yaw_online;
    g_watch.gimbal.pitch_online = gimbal.pitch_online;
    g_watch.gimbal.imu_online = gimbal.imu_online;
    g_watch.gimbal.follow_available = gimbal.follow_available;
    g_watch.gimbal.fault_configured_mask = gimbal.fault_configured_mask;
    g_watch.gimbal.fault_active_mask = gimbal.fault_active_mask;
    g_watch.gimbal.fault_blocking_mask = gimbal.fault_blocking_mask;
    g_watch.gimbal.fault_recovery_mask = gimbal.fault_recovery_mask;
    g_watch.gimbal.fault_inhibit_mask = gimbal.fault_inhibit_mask;
    g_watch.gimbal.fault_hold_zero_mask = gimbal.fault_hold_zero_mask;
    g_watch.gimbal.fault_imu_required_mask = gimbal.fault_imu_required_mask;
    g_watch.gimbal.fault_recovery_input_safe = gimbal.fault_recovery_input_safe;
    g_watch.gimbal.offline_mask = gimbal.offline_mask;
    g_watch.gimbal.required_offline_mask = gimbal.required_offline_mask;
    g_watch.gimbal.yaw_reason_mask = gimbal.yaw_reason_mask;
    g_watch.gimbal.pitch_reason_mask = gimbal.pitch_reason_mask;
    g_watch.gimbal.yaw_feedback_age_ms = gimbal.yaw_feedback_age_ms;
    g_watch.gimbal.pitch_feedback_age_ms = gimbal.pitch_feedback_age_ms;
    g_watch.gimbal.imu_age_ms = gimbal.imu_age_ms;
    g_watch.gimbal.fault_inhibit_fail_count = gimbal.fault_inhibit_fail_count;
    g_watch.gimbal.fault_release_fail_count = gimbal.fault_release_fail_count;

    if (yaw->valid != 0u)
    {
        g_watch.gimbal.yaw_mode = (WatchGimbalMotorMode)yaw->motor_mode;
        g_watch.gimbal.yaw_angle_deg = WatchRadToDeg(yaw->angle);
        g_watch.gimbal.yaw_set_deg = WatchRadToDeg(yaw->angle_set);
        g_watch.gimbal.yaw_gyro_dps = WatchRadToDeg(yaw->motor_gyro);
        g_watch.gimbal.yaw_current = yaw->given_current;

        g_watch.gimbal.yaw_rpm = (yaw->measure.valid != 0u) ? yaw->measure.speed_rpm : 0;
        g_watch.gimbal.yaw_current_fb = (yaw->measure.valid != 0u) ? yaw->measure.given_current : 0;
        g_watch.gimbal.yaw_ecd = (yaw->measure.valid != 0u) ? yaw->measure.ecd : 0;
        g_watch.gimbal.yaw_ecd_deg = (yaw->measure.valid != 0u) ? WatchEcdToDeg(g_config.motor.yaw.model, yaw->measure.ecd) : 0.0f;
        g_watch.gimbal.yaw_temp = (yaw->measure.valid != 0u) ? yaw->measure.temperature : 0;
    }
    else
    {
        g_watch.gimbal.yaw_mode = WATCH_GIMBAL_MOTOR_RAW;
        g_watch.gimbal.yaw_angle_deg = 0.0f;
        g_watch.gimbal.yaw_set_deg = 0.0f;
        g_watch.gimbal.yaw_gyro_dps = 0.0f;
        g_watch.gimbal.yaw_current = 0;
        g_watch.gimbal.yaw_rpm = 0;
        g_watch.gimbal.yaw_current_fb = 0;
        g_watch.gimbal.yaw_ecd = 0;
        g_watch.gimbal.yaw_ecd_deg = 0.0f;
        g_watch.gimbal.yaw_temp = 0;
    }

    if (pitch->valid != 0u)
    {
        g_watch.gimbal.pitch_mode = (WatchGimbalMotorMode)pitch->motor_mode;
        g_watch.gimbal.pitch_angle_deg = WatchRadToDeg(pitch->angle);
        g_watch.gimbal.pitch_set_deg = WatchRadToDeg(pitch->angle_set);
        g_watch.gimbal.pitch_gyro_dps = WatchRadToDeg(pitch->motor_gyro);
        g_watch.gimbal.pitch_current = pitch->given_current;

        g_watch.gimbal.pitch_rpm = (pitch->measure.valid != 0u) ? pitch->measure.speed_rpm : 0;
        g_watch.gimbal.pitch_current_fb = (pitch->measure.valid != 0u) ? pitch->measure.given_current : 0;
        g_watch.gimbal.pitch_ecd = (pitch->measure.valid != 0u) ? pitch->measure.ecd : 0;
        g_watch.gimbal.pitch_ecd_deg = (pitch->measure.valid != 0u) ? WatchEcdToDeg(g_config.motor.pitch.model, pitch->measure.ecd) : 0.0f;
        g_watch.gimbal.pitch_temp = (pitch->measure.valid != 0u) ? pitch->measure.temperature : 0;
    }
    else
    {
        g_watch.gimbal.pitch_mode = WATCH_GIMBAL_MOTOR_RAW;
        g_watch.gimbal.pitch_angle_deg = 0.0f;
        g_watch.gimbal.pitch_set_deg = 0.0f;
        g_watch.gimbal.pitch_gyro_dps = 0.0f;
        g_watch.gimbal.pitch_current = 0;
        g_watch.gimbal.pitch_rpm = 0;
        g_watch.gimbal.pitch_current_fb = 0;
        g_watch.gimbal.pitch_ecd = 0;
        g_watch.gimbal.pitch_ecd_deg = 0.0f;
        g_watch.gimbal.pitch_temp = 0;
    }
}
#endif

#if WATCH_ENABLE_GIMBAL_DUAL
static void WatchCopyDualGimbal(void)
{
    GimbalDualYawDebugState debug;
    GimbalState state = {0};
    const uint8_t state_valid =
        (GimbalStateRead(&state) != 0u && state.valid != 0u) ? 1u : 0u;

    if (!WatchBlockActiveGimbalDual())
    {
        memset(&g_watch.dual_gimbal, 0, sizeof(g_watch.dual_gimbal));
        return;
    }

    memset(&debug, 0, sizeof(debug));
#if ROBOT_TASK_BUILD_ANY_GIMBAL
    (void)GimbalDualYawDebugRead(&debug);
#endif

    g_watch.dual_gimbal.active = debug.active;
    g_watch.dual_gimbal.upper_online = (state_valid != 0u) ? state.yaw_upper_online : debug.upper_online;
    g_watch.dual_gimbal.upper_limited = debug.upper_limited;
    g_watch.dual_gimbal.yaw_online = state.yaw_online;
    g_watch.dual_gimbal.pitch_online = state.pitch_online;
    g_watch.dual_gimbal.imu_online = state.imu_online;
    g_watch.dual_gimbal.follow_available = state.follow_available;
    g_watch.dual_gimbal.fault_configured_mask = state.fault_configured_mask;
    g_watch.dual_gimbal.fault_active_mask = state.fault_active_mask;
    g_watch.dual_gimbal.fault_blocking_mask = state.fault_blocking_mask;
    g_watch.dual_gimbal.fault_recovery_mask = state.fault_recovery_mask;
    g_watch.dual_gimbal.fault_inhibit_mask = state.fault_inhibit_mask;
    g_watch.dual_gimbal.fault_hold_zero_mask = state.fault_hold_zero_mask;
    g_watch.dual_gimbal.fault_imu_required_mask = state.fault_imu_required_mask;
    g_watch.dual_gimbal.fault_recovery_input_safe = state.fault_recovery_input_safe;
    g_watch.dual_gimbal.offline_mask = state.offline_mask;
    g_watch.dual_gimbal.required_offline_mask = state.required_offline_mask;
    g_watch.dual_gimbal.yaw_reason_mask = state.yaw_reason_mask;
    g_watch.dual_gimbal.yaw_upper_reason_mask = state.yaw_upper_reason_mask;
    g_watch.dual_gimbal.pitch_reason_mask = state.pitch_reason_mask;
    g_watch.dual_gimbal.yaw_feedback_age_ms = state.yaw_feedback_age_ms;
    g_watch.dual_gimbal.yaw_upper_feedback_age_ms = state.yaw_upper_feedback_age_ms;
    g_watch.dual_gimbal.pitch_feedback_age_ms = state.pitch_feedback_age_ms;
    g_watch.dual_gimbal.imu_age_ms = state.imu_age_ms;
    g_watch.dual_gimbal.fault_inhibit_fail_count = state.fault_inhibit_fail_count;
    g_watch.dual_gimbal.fault_release_fail_count = state.fault_release_fail_count;
    g_watch.dual_gimbal.total_set_deg = WatchRadToDeg(debug.total_set_rad);
    g_watch.dual_gimbal.big_angle_deg = WatchRadToDeg(debug.big_angle_rad);
    g_watch.dual_gimbal.big_set_deg = WatchRadToDeg(debug.big_set_rad);
    g_watch.dual_gimbal.big_error_deg = WatchRadToDeg(debug.big_error_rad);
    g_watch.dual_gimbal.upper_angle_deg = WatchRadToDeg(debug.upper_angle_rad);
    g_watch.dual_gimbal.upper_set_deg = WatchRadToDeg(debug.upper_set_rad);
    g_watch.dual_gimbal.upper_error_deg = WatchRadToDeg(debug.upper_error_rad);
    g_watch.dual_gimbal.upper_gyro_dps = WatchRadToDeg(debug.upper_gyro_radps);
    g_watch.dual_gimbal.yaw_current = debug.yaw_current;
    g_watch.dual_gimbal.yaw_upper_current = debug.yaw_upper_current;
    g_watch.dual_gimbal.yaw_upper_ecd = debug.yaw_upper_ecd;
}
#endif

#if WATCH_ENABLE_SHOOT_RM
static void WatchCopyShoot(void)
{
    const fp32 rad2deg = 57.29577951308232f;

    ShootState shoot;
    if (ShootStateRead(&shoot) == 0u || shoot.valid == 0u)
    {
        memset(&g_watch.shoot, 0, sizeof(g_watch.shoot));
        return;
    }

    g_watch.shoot.mode = (WatchShootMode)shoot.mode;
    g_watch.shoot.fric_speed_set_rpm = (int16_t)shoot.fric_speed_set;
    for (uint8_t i = 0; i < 4; i++)
    {
        g_watch.shoot.fric_current_cmd[i] =
            LowCmdGetCurrent(WatchFrictionMotorId(i));
    }

    g_watch.shoot.trigger_angle_deg = shoot.angle * rad2deg;
    g_watch.shoot.trigger_set_deg = shoot.set_angle * rad2deg;
    g_watch.shoot.trigger_speed = shoot.speed;
    g_watch.shoot.trigger_speed_set = shoot.speed_set;
    g_watch.shoot.trigger_current = shoot.given_current;

    const motor_measure_t *trigger_meas = get_trigger_motor_measure_point();
    if (trigger_meas)
    {
        g_watch.shoot.trigger_rpm = trigger_meas->speed_rpm;
        g_watch.shoot.trigger_ecd = trigger_meas->ecd;
        g_watch.shoot.trigger_temp = trigger_meas->temperate;
    }
    else
    {
        g_watch.shoot.trigger_rpm = 0;
        g_watch.shoot.trigger_ecd = 0;
        g_watch.shoot.trigger_temp = 0;
    }

    g_watch.shoot.heat_limit = shoot.heat_limit;
    g_watch.shoot.heat = shoot.heat;

    for (uint8_t i = 0; i < 4; i++)
    {
        const motor_measure_t *fm = get_friction_motor_measure_point(i);
        if (fm)
        {
            g_watch.shoot.fric_rpm[i] = fm->speed_rpm;
            g_watch.shoot.fric_current[i] = fm->given_current;
            g_watch.shoot.fric_temp[i] = fm->temperate;
        }
        else
        {
            g_watch.shoot.fric_rpm[i] = 0;
            g_watch.shoot.fric_current[i] = 0;
            g_watch.shoot.fric_temp[i] = 0;
        }
    }
}
#endif

#if WATCH_ENABLE_ARM_J0_UNITREE
static void WatchCopyArmJ0Unitree(void)
{
    const ArmJ0UnitreeState *state = NULL;

    if (!WatchBlockActiveArm())
    {
        memset(&g_watch.ArmJ0Unitree, 0, sizeof(g_watch.ArmJ0Unitree));
        return;
    }

    state = ArmJ0UnitreeGetState();
    if (state == NULL)
    {
        memset(&g_watch.ArmJ0Unitree, 0, sizeof(g_watch.ArmJ0Unitree));
        return;
    }

    g_watch.ArmJ0Unitree.enabled = state->enabled;
    g_watch.ArmJ0Unitree.rs485_port = state->rs485_port;
    g_watch.ArmJ0Unitree.motor_id = state->motor_id;
    g_watch.ArmJ0Unitree.online = state->online;
    g_watch.ArmJ0Unitree.last_mode = state->last_mode;
    g_watch.ArmJ0Unitree.motor_error = state->motor_error;
    g_watch.ArmJ0Unitree.motor_temp = state->motor_temp;
    g_watch.ArmJ0Unitree.last_tx_status = state->last_tx_status;
    g_watch.ArmJ0Unitree.tx_count = state->tx_count;
    g_watch.ArmJ0Unitree.tx_fail_count = state->tx_fail_count;
    g_watch.ArmJ0Unitree.rx_frame_count = state->rx_frame_count;
    g_watch.ArmJ0Unitree.rx_crc_fail_count = state->rx_crc_fail_count;
    g_watch.ArmJ0Unitree.rx_parse_error_count = state->rx_parse_error_count;
    g_watch.ArmJ0Unitree.last_rx_tick_ms = state->last_rx_tick_ms;
    g_watch.ArmJ0Unitree.cmd_speed_deg_s = WatchRadToDeg(state->cmd_output_speed_rad_s);
    g_watch.ArmJ0Unitree.cmd_kd = state->cmd_output_kd;
    g_watch.ArmJ0Unitree.torque_nm = state->torque_nm;
    g_watch.ArmJ0Unitree.joint_speed_deg_s = WatchRadToDeg(state->joint_speed_rad_s);
    g_watch.ArmJ0Unitree.joint_position_deg = WatchRadToDeg(state->joint_position_rad);
}
#endif
