/*
 * SPDX-FileCopyrightText: 2026 ?? <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Private implementation chunk included by gimbal_control_task.c.
 */

static fp32 gimbal_pitch_kick_scale(fp32 angle_err)
{
    const fp32 err_abs = fabsf(angle_err);

    if (err_abs <= PITCH_KICK_ERR_OFF_RAD)
    {
        return 0.0f;
    }
    if (err_abs >= PITCH_KICK_ERR_ON_RAD)
    {
        return 1.0f;
    }

    return (err_abs - PITCH_KICK_ERR_OFF_RAD) / (PITCH_KICK_ERR_ON_RAD - PITCH_KICK_ERR_OFF_RAD);
}

static int16_t gimbal_apply_output_turn(int16_t current, uint8_t turn)
{
    return (turn != 0u) ? (int16_t)-current : current;
}

static uint8_t gimbal_dual_yaw_use_upper_control(void)
{
    return (uint8_t)(robot_profile_need_dual_gimbal_control_task() != 0u &&
                     motor_cfg_node_id(&g_config.motor.yaw) == 0u &&
                     motor_cfg_node_id(&g_config.motor.yaw_upper) != 0u);
}

static const motor_measure_t *gimbal_yaw_control_measure_point(void)
{
    return (gimbal_dual_yaw_use_upper_control() != 0u) ? get_yaw_upper_gimbal_motor_measure_point() :
                                                         get_yaw_gimbal_motor_measure_point();
}

static const motor_node_param_t *gimbal_yaw_control_motor_cfg(void)
{
    return (gimbal_dual_yaw_use_upper_control() != 0u) ? &g_config.motor.yaw_upper : &g_config.motor.yaw;
}

static uint8_t gimbal_yaw_soft_limit_enabled(void)
{
    return (uint8_t)(((YAW_SOFT_LIMIT_MAX != 0.0f) || (YAW_SOFT_LIMIT_MIN != 0.0f)) &&
                     (YAW_SOFT_LIMIT_MAX != YAW_SOFT_LIMIT_MIN) &&
                     (fabsf(YAW_SOFT_LIMIT_MAX) < 100.0f) &&
                     (fabsf(YAW_SOFT_LIMIT_MIN) < 100.0f));
}

static void gimbal_snapshot_capture(gimbal_runtime_snapshot_t *snapshot, gimbal_control_t *control)
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
    snapshot->gyro = (control != NULL) ? control->gimbal_INT_gyro_point : get_gyro_data_point();
    snapshot->ins_angle = (control != NULL) ? control->gimbal_INS_angle_point : get_INS_angle_point();
    snapshot->yaw_control_is_upper = gimbal_dual_yaw_use_upper_control();
    snapshot->yaw_measure = (control != NULL) ? control->gimbal_yaw_motor.gimbal_motor_measure : gimbal_yaw_control_measure_point();
    snapshot->pitch_measure = (control != NULL) ? control->gimbal_pitch_motor.gimbal_motor_measure : get_pitch_gimbal_motor_measure_point();
    snapshot->yaw_motor_cfg = gimbal_yaw_control_motor_cfg();
    snapshot->pitch_motor_cfg = &g_config.motor.pitch;
    snapshot->trigger_motor_cfg = &g_config.motor.trigger;
    snapshot->run_variant = robot_mode_variant();
    snapshot->pitch_cali_mode = robot_mode_gimbal_pitch_cali();
    snapshot->period_ms = robot_profile_gimbal_control_period_ms();
    snapshot->period_us = (uint32_t)snapshot->period_ms * 1000u;
    snapshot->yaw_turn = YAW_TURN ? 1u : 0u;
    snapshot->pitch_turn = PITCH_TURN ? 1u : 0u;

    if (control != NULL)
    {
        gimbal_control_snapshot_t fast = {0};
        const manual_input_state_t *manual_input = snapshot->manual_input;

        fast.dbus_offline = toe_is_error(DBUS_TOE);
        fast.run_variant = snapshot->run_variant;
        fast.pitch_cali_mode = snapshot->pitch_cali_mode;
        fast.mode_sw = input_switch(INPUT_SW_GIMBAL_MODE);
        fast.safe_pos = g_config.manual_input.semantics.gimbal_safe_pos;
        fast.active_source = remote_control_get_active_source();
        fast.image_auto_aim_requested = image_remote_auto_aim_requested() ? 1u : 0u;
        fast.yaw_axis = input_axis(INPUT_AXIS_GIMBAL_YAW);
        fast.pitch_axis = input_axis(INPUT_AXIS_GIMBAL_PITCH);
        fast.mouse_x = (manual_input != NULL) ? manual_input->mouse.x : 0;
        fast.mouse_y = (manual_input != NULL) ? manual_input->mouse.y : 0;
        fast.rc_deadband = RC_DEADBAND;
        fast.key_mask = (manual_input != NULL) ? manual_input->key.v : 0u;
        fast.turn_key_mask = TURN_KEYBOARD;
        fast.turn_speed = TURN_SPEED;
        fast.yaw_rc_sen = YAW_RC_SEN;
        fast.pitch_rc_sen = PITCH_RC_SEN;
        fast.yaw_mouse_sen = YAW_MOUSE_SEN;
        fast.pitch_mouse_sen = PITCH_MOUSE_SEN;

        control->fast = fast;
    }
}

static void gimbal_apply_operation_mode(const gimbal_runtime_snapshot_t *snapshot, int16_t *yaw_current, int16_t *pitch_current)
{
    const robot_run_variant_e variant = (snapshot != NULL) ? snapshot->run_variant : robot_mode_variant();
    const uint8_t pitch_cali_mode = (snapshot != NULL) ? snapshot->pitch_cali_mode : robot_mode_gimbal_pitch_cali();

    if (gimbal_behaviour_watch == GIMBAL_ZERO_FORCE)
    {
        gimbal_total_pid_clear(&gimbal_control);
        *yaw_current = 0;
        *pitch_current = 0;
        return;
    }

    if (pitch_cali_mode != 0u)
    {
        return;
    }

    switch (variant)
    {
    case ROBOT_RUN_VARIANT_NORMAL:
    case ROBOT_RUN_VARIANT_GIMBAL_DUAL:
        return;
    case ROBOT_RUN_VARIANT_GIMBAL_YAW_ONLY:
        gimbal_PID_clear(&gimbal_control.gimbal_pitch_motor.gimbal_motor_angle_pid);
        PID_clear(&gimbal_control.gimbal_pitch_motor.gimbal_motor_gyro_pid);
        *pitch_current = 0;
        return;
    case ROBOT_RUN_VARIANT_GIMBAL_PITCH_ONLY:
        gimbal_PID_clear(&gimbal_control.gimbal_yaw_motor.gimbal_motor_angle_pid);
        PID_clear(&gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid);
        *yaw_current = 0;
        return;
    case ROBOT_RUN_VARIANT_GIMBAL_YAW_EASY:
        gimbal_total_pid_clear(&gimbal_control);
        *yaw_current = gimbal_yaw_easytest_current;
        *pitch_current = 0;
        return;
    default:
        gimbal_total_pid_clear(&gimbal_control);
        *yaw_current = 0;
        *pitch_current = 0;
        return;
    }
}

static bool_t GimbalControlMgrAllows(uint16_t expected_controller_id,
                                            const gimbal_runtime_snapshot_t *snapshot)
{
    ControlCtx context = {0};
    const uint16_t period_ms =
        (snapshot != NULL && snapshot->period_ms != 0u) ? snapshot->period_ms : robot_profile_gimbal_control_period_ms();

    context.tick_ms = bsp_time_get_tick_ms();
    context.dt_s = (fp32)period_ms * 0.001f;

    if (ControlMgrUpdateDomain(ControlDomainGimbal, &context) != ControlResultOk)
    {
        return 0;
    }

    return (ControlMgrActiveId(ControlDomainGimbal) == expected_controller_id) ? 1 : 0;
}

static bool_t ShootControlMgrAllows(const gimbal_runtime_snapshot_t *snapshot)
{
    ControlCtx context = {0};
    const uint16_t period_ms =
        (snapshot != NULL && snapshot->period_ms != 0u) ? snapshot->period_ms : robot_profile_gimbal_control_period_ms();

    context.tick_ms = bsp_time_get_tick_ms();
    context.dt_s = (fp32)period_ms * 0.001f;

    if (ControlMgrUpdateDomain(ControlDomainShoot, &context) != ControlResultOk)
    {
        return 0;
    }

    return (ControlMgrActiveId(ControlDomainShoot) == ControlIdShoot) ? 1 : 0;
}

static int16_t gimbal_run_shoot_control(const gimbal_runtime_snapshot_t *snapshot)
{
    if (!ShootControlMgrAllows(snapshot))
    {
        shoot_stop_outputs();
        return 0;
    }

    return shoot_control_loop();
}

static void gimbal_stop_outputs(const MotorCurrentBind *bindings, uint8_t count)
{
    int16_t zero_current[DUAL_YAW_GIMBAL_OUTPUT_MOTOR_COUNT] = {0};

    gimbal_total_pid_clear(&gimbal_control);
    gimbal_control.gimbal_yaw_motor.current_set = 0;
    gimbal_control.gimbal_yaw_motor.given_current = 0;
    gimbal_control.gimbal_pitch_motor.current_set = 0;
    gimbal_control.gimbal_pitch_motor.given_current = 0;
    shoot_can_set_current = 0;
    yaw_can_set_current = 0;
    pitch_can_set_current = 0;
    gimbal_watch_yaw_current = 0;
    gimbal_watch_yaw_upper_current = 0;
    gimbal_watch_pitch_current = 0;

    if (count > DUAL_YAW_GIMBAL_OUTPUT_MOTOR_COUNT)
    {
        count = DUAL_YAW_GIMBAL_OUTPUT_MOTOR_COUNT;
    }
    (void)MotorInstSetCurrentBindsBestEffort(bindings, zero_current, count);
    shoot_stop_outputs();
    gimbal_write_state();
}

static void gimbal_control_delay(TickType_t *last_wake, uint16_t period_ms)
{
    const TickType_t delay_start = xTaskGetTickCount();

    if (last_wake == NULL)
    {
        return;
    }

    vTaskDelayUntil(last_wake, pdMS_TO_TICKS(period_ms));
    if (xTaskGetTickCount() == delay_start)
    {
        vTaskDelay(1u);
        *last_wake = xTaskGetTickCount();
    }
}
