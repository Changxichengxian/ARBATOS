/*
 * SPDX-FileCopyrightText: 2026 ?? <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Private implementation chunk included by GimbalControlTask.c.
 */

static fp32 GimbalPitchKickScale(fp32 angle_err)
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

static int16_t GimbalApplyOutputTurn(int16_t current, uint8_t turn)
{
    return (turn != 0u) ? (int16_t)-current : current;
}

static uint8_t GimbalSingleMitYawTestActive(const GimbalRuntimeSnapshot *snapshot)
{
    const MotorId target = robot_mode_target_motor();
    const motor_node_param_t *node = MotorInstNode(target);
    const robot_run_variant_e variant = (snapshot != NULL) ? snapshot->run_variant : robot_mode_variant();

    if (robot_mode_current() != ROBOT_RUN_MODE_SINGLE_MOTOR ||
        variant != ROBOT_RUN_VARIANT_GIMBAL_YAW_ONLY ||
        target != Motor4 ||
        node == NULL)
    {
        return 0u;
    }

    if (MotorCfgTransport(node) != MOTOR_TRANSPORT_CAN ||
        MotorCfgControlMode(node) != MOTOR_CONTROL_MODE_MIT ||
        MotorCfgProtocol(node) == MOTOR_PROTOCOL_RM_GROUP)
    {
        return 0u;
    }

    return 1u;
}

static fp32 GimbalSingleMitYawTestTorque(const GimbalRuntimeSnapshot *snapshot)
{
    int16_t axis = input_axis(INPUT_AXIS_GIMBAL_YAW);
    const int16_t deadband = (int16_t)RC_DEADBAND;
    const int16_t max_axis = (int16_t)RC_CH_VALUE_ABS_MAX;
    fp32 torque;

    if (axis > -deadband && axis < deadband)
    {
        return 0.0f;
    }
    if (axis > max_axis)
    {
        axis = max_axis;
    }
    else if (axis < -max_axis)
    {
        axis = (int16_t)-max_axis;
    }

    {
        const fp32 yaw_input_dir = (YAW_RC_SEN >= 0.0f) ? -1.0f : 1.0f;
        torque = ((fp32)axis / (fp32)max_axis) * yaw_input_dir * GIMBAL_SINGLE_MIT_TEST_MAX_TORQUE_NM;
    }
    if (snapshot != NULL && snapshot->yaw_turn != 0u)
    {
        torque = -torque;
    }
    return torque;
}

static void GimbalApplySingleMitYawTest(const GimbalRuntimeSnapshot *snapshot)
{
    const fp32 torque = GimbalSingleMitYawTestTorque(snapshot);

    GimbalTotalPidClear(&g_gimbal);
    g_gimbal.GimbalYawMotor.current_set = 0.0f;
    g_gimbal.GimbalYawMotor.given_current = 0;
    g_gimbal.GimbalPitchMotor.current_set = 0.0f;
    g_gimbal.GimbalPitchMotor.given_current = 0;
    (void)MotorInstSetDampingId(Motor4, GIMBAL_SINGLE_MIT_TEST_DAMPING_KD, torque);
}

static uint8_t GimbalDualYawUseUpperControl(void)
{
    return (uint8_t)(RobotProfileNeedDualGimbalControlTask() != 0u &&
                     MotorCfgNodeId(&g_config.motor.yaw) == 0u &&
                     MotorCfgNodeId(&g_config.motor.yaw_upper) != 0u);
}

static const motor_measure_t *GimbalYawControlMeasurePoint(void)
{
    return (GimbalDualYawUseUpperControl() != 0u) ? get_yaw_upper_gimbal_motor_measure_point() :
                                                         get_yaw_gimbal_motor_measure_point();
}

static const motor_node_param_t *GimbalYawControlMotorCfg(void)
{
    return (GimbalDualYawUseUpperControl() != 0u) ? &g_config.motor.yaw_upper : &g_config.motor.yaw;
}

static uint8_t GimbalYawSoftLimitEnabled(void)
{
    return (uint8_t)(((YAW_SOFT_LIMIT_MAX != 0.0f) || (YAW_SOFT_LIMIT_MIN != 0.0f)) &&
                     (YAW_SOFT_LIMIT_MAX != YAW_SOFT_LIMIT_MIN) &&
                     (fabsf(YAW_SOFT_LIMIT_MAX) < 100.0f) &&
                     (fabsf(YAW_SOFT_LIMIT_MIN) < 100.0f));
}

static void GimbalSnapshotCapture(GimbalRuntimeSnapshot *snapshot, GimbalControl *control)
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
    snapshot->gyro = (control != NULL) ? control->GimbalINTGyroPoint : get_gyro_data_point();
    snapshot->ins_angle = (control != NULL) ? control->GimbalInsAnglePoint : get_INS_angle_point();
    snapshot->yaw_control_is_upper = GimbalDualYawUseUpperControl();
    snapshot->yaw_measure = (control != NULL) ? control->GimbalYawMotor.GimbalMotorMeasure : GimbalYawControlMeasurePoint();
    snapshot->pitch_measure = (control != NULL) ? control->GimbalPitchMotor.GimbalMotorMeasure : get_pitch_gimbal_motor_measure_point();
    snapshot->yaw_motor_cfg = GimbalYawControlMotorCfg();
    snapshot->pitch_motor_cfg = &g_config.motor.pitch;
    snapshot->trigger_motor_cfg = &g_config.motor.trigger;
    snapshot->run_variant = robot_mode_variant();
    snapshot->PitchCaliMode = robot_mode_gimbal_pitch_cali();
    snapshot->period_ms = RobotProfileGimbalControlPeriodMs();
    snapshot->period_us = (uint32_t)snapshot->period_ms * 1000u;
    snapshot->yaw_turn = YAW_TURN ? 1u : 0u;
    snapshot->pitch_turn = PITCH_TURN ? 1u : 0u;

    if (control != NULL)
    {
        GimbalControlSnapshot fast = {0};
        const ManualInputState *manual_input = snapshot->manual_input;

        fast.dbus_offline = toe_is_error(DBUS_TOE);
        fast.run_variant = snapshot->run_variant;
        fast.PitchCaliMode = snapshot->PitchCaliMode;
        fast.mode_sw = ControlInputSwitch(INPUT_SW_GIMBAL_MODE);
        fast.safe_pos = g_config.manual_input.semantics.GimbalSafePos;
        fast.active_source = remote_control_get_active_source();
        fast.image_auto_aim_requested = ImageRemoteAutoAimRequested() ? 1u : 0u;
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

static void GimbalApplyOperationMode(const GimbalRuntimeSnapshot *snapshot, int16_t *yaw_current, int16_t *pitch_current)
{
    const robot_run_variant_e variant = (snapshot != NULL) ? snapshot->run_variant : robot_mode_variant();
    const uint8_t PitchCaliMode = (snapshot != NULL) ? snapshot->PitchCaliMode : robot_mode_gimbal_pitch_cali();

    if (GimbalBehaviourWatch == GIMBAL_ZERO_FORCE)
    {
        GimbalTotalPidClear(&g_gimbal);
        *yaw_current = 0;
        *pitch_current = 0;
        return;
    }

    if (PitchCaliMode != 0u)
    {
        return;
    }

    switch (variant)
    {
    case ROBOT_RUN_VARIANT_NORMAL:
    case ROBOT_RUN_VARIANT_GIMBAL_DUAL:
        return;
    case ROBOT_RUN_VARIANT_GIMBAL_YAW_ONLY:
        GimbalPidClear(&g_gimbal.GimbalPitchMotor.GimbalMotorAnglePid);
        PID_clear(&g_gimbal.GimbalPitchMotor.GimbalMotorGyroPid);
        *pitch_current = 0;
        return;
    case ROBOT_RUN_VARIANT_GIMBAL_PITCH_ONLY:
        GimbalPidClear(&g_gimbal.GimbalYawMotor.GimbalMotorAnglePid);
        PID_clear(&g_gimbal.GimbalYawMotor.GimbalMotorGyroPid);
        *yaw_current = 0;
        return;
    case ROBOT_RUN_VARIANT_GIMBAL_YAW_EASY:
        GimbalTotalPidClear(&g_gimbal);
        *yaw_current = GimbalYawEasytestCurrent;
        *pitch_current = 0;
        return;
    default:
        GimbalTotalPidClear(&g_gimbal);
        *yaw_current = 0;
        *pitch_current = 0;
        return;
    }
}

static bool_t GimbalControlMgrAllows(uint16_t expected_controller_id,
                                            const GimbalRuntimeSnapshot *snapshot)
{
    ControlCtx context = {0};
    const uint16_t period_ms =
        (snapshot != NULL && snapshot->period_ms != 0u) ? snapshot->period_ms : RobotProfileGimbalControlPeriodMs();

    context.tick_ms = BspTimeGetTickMs();
    context.dt_s = (fp32)period_ms * 0.001f;

    if (ControlMgrUpdateDomain(ControlDomainGimbal, &context) != ControlResultOk)
    {
        return 0;
    }

    return (ControlMgrActiveId(ControlDomainGimbal) == expected_controller_id) ? 1 : 0;
}

static bool_t ShootControlMgrAllows(const GimbalRuntimeSnapshot *snapshot)
{
    ControlCtx context = {0};
    const uint16_t period_ms =
        (snapshot != NULL && snapshot->period_ms != 0u) ? snapshot->period_ms : RobotProfileGimbalControlPeriodMs();

    context.tick_ms = BspTimeGetTickMs();
    context.dt_s = (fp32)period_ms * 0.001f;

    if (ControlMgrUpdateDomain(ControlDomainShoot, &context) != ControlResultOk)
    {
        return 0;
    }

    return (ControlMgrActiveId(ControlDomainShoot) == ControlIdShoot) ? 1 : 0;
}

static int16_t GimbalRunShootControl(const GimbalRuntimeSnapshot *snapshot)
{
    if (!ShootControlMgrAllows(snapshot))
    {
        ShootStopOutputs();
        return 0;
    }

    return ShootControlLoop();
}

static void GimbalStopOutputs(const MotorCurrentBind *bindings, uint8_t count)
{
    int16_t zero_current[DUAL_YAW_GIMBAL_OUTPUT_MOTOR_COUNT] = {0};

    GimbalTotalPidClear(&g_gimbal);
    g_gimbal.GimbalYawMotor.current_set = 0;
    g_gimbal.GimbalYawMotor.given_current = 0;
    g_gimbal.GimbalPitchMotor.current_set = 0;
    g_gimbal.GimbalPitchMotor.given_current = 0;
    ShootCanSetCurrent = 0;
    yaw_can_set_current = 0;
    pitch_can_set_current = 0;
    GimbalWatchYawCurrent = 0;
    GimbalWatchYawUpperCurrent = 0;
    GimbalWatchPitchCurrent = 0;

    if (count > DUAL_YAW_GIMBAL_OUTPUT_MOTOR_COUNT)
    {
        count = DUAL_YAW_GIMBAL_OUTPUT_MOTOR_COUNT;
    }
    (void)MotorInstSetCurrentBindsBestEffort(bindings, zero_current, count);
    ShootStopOutputs();
    GimbalWriteState();
}

static void GimbalControlDelay(TickType_t *last_wake, uint16_t period_ms)
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
