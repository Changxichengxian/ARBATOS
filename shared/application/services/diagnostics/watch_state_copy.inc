/*
 * SPDX-FileCopyrightText: 2026 ?? <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Private implementation chunk included by watch.c.
 */

static void watch_copy_imu(void)
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
static void watch_copy_chassis(void)
{
    const fp32 rad2deg = 57.29577951308232f;

    if (!watch_block_active_locomotion_classic())
    {
        memset(&g_watch.chassis, 0, sizeof(g_watch.chassis));
        return;
    }

    chassis_state_t chassis;
    if (chassis_state_read(&chassis) == 0u || chassis.valid == 0u)
    {
        memset(&g_watch.chassis, 0, sizeof(g_watch.chassis));
        return;
    }

    g_watch.chassis.mode = (watch_chassis_mode_e)chassis.mode;
    g_watch.chassis.last_mode = (watch_chassis_mode_e)chassis.last_mode;

    g_watch.chassis.vx_set = chassis.vx_set;
    g_watch.chassis.vy_set = chassis.vy_set;
    g_watch.chassis.wz_set = chassis.wz_set;
    g_watch.chassis.vx = chassis.vx;
    g_watch.chassis.vy = chassis.vy;
    g_watch.chassis.wz = chassis.wz;
    g_watch.chassis.yaw_deg = chassis.chassis_yaw * rad2deg;

    for (uint8_t i = 0; i < 4u && i < CHASSIS_STATE_MOTOR_COUNT; i++)
    {
        const chassis_motor_state_t *m = &chassis.motor[i];
        g_watch.chassis.motor_rpm[i] = (m->measure.valid != 0u) ? m->measure.speed_rpm : 0;
        g_watch.chassis.motor_current[i] = m->give_current;
        g_watch.chassis.motor_speed_set[i] = m->speed_set;
        g_watch.chassis.motor_ecd[i] = (m->measure.valid != 0u) ? m->measure.ecd : 0;
        g_watch.chassis.motor_temp[i] = (m->measure.valid != 0u) ? m->measure.temperature : 0;
    }
}
#endif

#if WATCH_ENABLE_GIMBAL_SINGLE
static void watch_copy_gimbal(void)
{
    if (!watch_block_active_gimbal_single())
    {
        memset(&g_watch.gimbal, 0, sizeof(g_watch.gimbal));
        return;
    }

    gimbal_state_t gimbal;
    if (gimbal_state_read(&gimbal) == 0u || gimbal.valid == 0u)
    {
        memset(&g_watch.gimbal, 0, sizeof(g_watch.gimbal));
        return;
    }

    const gimbal_motor_state_t *yaw = &gimbal.yaw;
    const gimbal_motor_state_t *pitch = &gimbal.pitch;

    if (yaw->valid != 0u)
    {
        g_watch.gimbal.yaw_mode = (watch_gimbal_motor_mode_e)yaw->motor_mode;
        g_watch.gimbal.yaw_angle_deg = watch_rad_to_deg(yaw->angle);
        g_watch.gimbal.yaw_set_deg = watch_rad_to_deg(yaw->angle_set);
        g_watch.gimbal.yaw_gyro_dps = watch_rad_to_deg(yaw->motor_gyro);
        g_watch.gimbal.yaw_current = yaw->given_current;

        g_watch.gimbal.yaw_rpm = (yaw->measure.valid != 0u) ? yaw->measure.speed_rpm : 0;
        g_watch.gimbal.yaw_current_fb = (yaw->measure.valid != 0u) ? yaw->measure.given_current : 0;
        g_watch.gimbal.yaw_ecd = (yaw->measure.valid != 0u) ? yaw->measure.ecd : 0;
        g_watch.gimbal.yaw_ecd_deg = (yaw->measure.valid != 0u) ? watch_ecd_to_deg(yaw->measure.ecd) : 0.0f;
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
        g_watch.gimbal.pitch_mode = (watch_gimbal_motor_mode_e)pitch->motor_mode;
        g_watch.gimbal.pitch_angle_deg = watch_rad_to_deg(pitch->angle);
        g_watch.gimbal.pitch_set_deg = watch_rad_to_deg(pitch->angle_set);
        g_watch.gimbal.pitch_gyro_dps = watch_rad_to_deg(pitch->motor_gyro);
        g_watch.gimbal.pitch_current = pitch->given_current;

        g_watch.gimbal.pitch_rpm = (pitch->measure.valid != 0u) ? pitch->measure.speed_rpm : 0;
        g_watch.gimbal.pitch_current_fb = (pitch->measure.valid != 0u) ? pitch->measure.given_current : 0;
        g_watch.gimbal.pitch_ecd = (pitch->measure.valid != 0u) ? pitch->measure.ecd : 0;
        g_watch.gimbal.pitch_ecd_deg = (pitch->measure.valid != 0u) ? watch_ecd_to_deg(pitch->measure.ecd) : 0.0f;
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

#if WATCH_ENABLE_SHOOT_RM
static void watch_copy_shoot(void)
{
    const fp32 rad2deg = 57.29577951308232f;

    shoot_state_t shoot;
    if (shoot_state_read(&shoot) == 0u || shoot.valid == 0u)
    {
        memset(&g_watch.shoot, 0, sizeof(g_watch.shoot));
        return;
    }

    g_watch.shoot.mode = (watch_shoot_mode_e)shoot.mode;
    g_watch.shoot.fric_speed_set_rpm = (int16_t)shoot.fric_speed_set;
    for (uint8_t i = 0; i < 4; i++)
    {
        g_watch.shoot.fric_current_cmd[i] =
            LowCmdGetCurrent(watch_friction_motor_id(i));
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
static void watch_copy_arm_j0_unitree(void)
{
    const arm_j0_unitree_state_t *state = NULL;

    if (!watch_block_active_arm())
    {
        memset(&g_watch.arm_j0_unitree, 0, sizeof(g_watch.arm_j0_unitree));
        return;
    }

    state = arm_j0_unitree_get_state();
    if (state == NULL)
    {
        memset(&g_watch.arm_j0_unitree, 0, sizeof(g_watch.arm_j0_unitree));
        return;
    }

    g_watch.arm_j0_unitree.enabled = state->enabled;
    g_watch.arm_j0_unitree.rs485_port = state->rs485_port;
    g_watch.arm_j0_unitree.motor_id = state->motor_id;
    g_watch.arm_j0_unitree.online = state->online;
    g_watch.arm_j0_unitree.last_mode = state->last_mode;
    g_watch.arm_j0_unitree.motor_error = state->motor_error;
    g_watch.arm_j0_unitree.motor_temp = state->motor_temp;
    g_watch.arm_j0_unitree.last_tx_status = state->last_tx_status;
    g_watch.arm_j0_unitree.tx_count = state->tx_count;
    g_watch.arm_j0_unitree.tx_fail_count = state->tx_fail_count;
    g_watch.arm_j0_unitree.rx_frame_count = state->rx_frame_count;
    g_watch.arm_j0_unitree.rx_crc_fail_count = state->rx_crc_fail_count;
    g_watch.arm_j0_unitree.rx_parse_error_count = state->rx_parse_error_count;
    g_watch.arm_j0_unitree.last_rx_tick_ms = state->last_rx_tick_ms;
    g_watch.arm_j0_unitree.cmd_speed_deg_s = watch_rad_to_deg(state->cmd_output_speed_rad_s);
    g_watch.arm_j0_unitree.cmd_kd = state->cmd_output_kd;
    g_watch.arm_j0_unitree.torque_nm = state->torque_nm;
    g_watch.arm_j0_unitree.joint_speed_deg_s = watch_rad_to_deg(state->joint_speed_rad_s);
    g_watch.arm_j0_unitree.joint_position_deg = watch_rad_to_deg(state->joint_position_rad);
}
#endif
