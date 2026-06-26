/*
 * SPDX-FileCopyrightText: 2026 ?? <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Private implementation chunk included by GimbalControlTask.c.
 */

/**
  * @brief          "GimbalControl" variable initialization, include pid initialization, remote control data point initialization, gimbal motors
  *                 data point initialization, and gyro sensor angle point initialization.
  * @param[out]     init: "GimbalControl" variable point
  * @retval         none
  */
static uint8_t GimbalDualYawImuCenterEnabled(const GimbalRuntimeSnapshot *snapshot)
{
#if GIMBAL_USE_ENCODER_FEEDBACK
    (void)snapshot;
    return 0u;
#else
    (void)snapshot;
    return (uint8_t)(RobotProfileNeedDualGimbalControlTask() != 0u &&
                     MotorCfgNodeId(&g_config.motor.yaw) != 0u);
#endif
}

static uint8_t GimbalYawImuFeedbackTurn(const GimbalRuntimeSnapshot *snapshot)
{
    if (GimbalDualYawImuCenterEnabled(snapshot) != 0u)
    {
        return (uint8_t)((GIMBAL_DUAL_YAW_IMU_TURN != 0u) ? 1u : 0u);
    }

    return (snapshot != NULL) ? snapshot->yaw_turn : (YAW_TURN ? 1u : 0u);
}

static uint8_t GimbalDualYawImuAngleRead(const GimbalRuntimeSnapshot *snapshot,
                                             const GimbalControl *control,
                                             fp32 *out)
{
    const fp32 *ins_angle = (snapshot != NULL) ? snapshot->ins_angle :
                                                   ((control != NULL) ? control->GimbalInsAnglePoint : NULL);
    const uint8_t yaw_turn = GimbalYawImuFeedbackTurn(snapshot);

    if (GimbalDualYawImuCenterEnabled(snapshot) == 0u ||
        ins_angle == NULL ||
        out == NULL)
    {
        return 0u;
    }

    *out = (yaw_turn != 0u) ? -ins_angle[INS_YAW_ADDRESS_OFFSET] : ins_angle[INS_YAW_ADDRESS_OFFSET];
    return 1u;
}

static uint8_t GimbalDualYawImuCenterCapture(GimbalControl *control, const GimbalRuntimeSnapshot *snapshot)
{
    fp32 yaw_angle = 0.0f;

    if (control == NULL || GimbalDualYawImuAngleRead(snapshot, control, &yaw_angle) == 0u)
    {
        return 0u;
    }

    s_dual_yaw_imu_center = yaw_angle;
    s_dual_yaw_imu_center_ready = 1u;
    control->GimbalYawMotor.angle = 0.0f;
    control->GimbalYawMotor.angle_set = 0.0f;
    control->GimbalYawMotor.motor_gyro_set = control->GimbalYawMotor.motor_gyro;
    return 1u;
}

static void GimbalDualYawImuCenterUpdateOnOutput(GimbalControl *control,
                                                    const GimbalRuntimeSnapshot *snapshot,
                                                    uint8_t output_allowed)
{
    if (GimbalDualYawImuCenterEnabled(snapshot) == 0u)
    {
        return;
    }

    if (output_allowed == 0u)
    {
        s_dual_yaw_output_allowed_prev = 0u;
        s_dual_yaw_imu_center_ready = 0u;
        return;
    }

    if (s_dual_yaw_output_allowed_prev == 0u || s_dual_yaw_imu_center_ready == 0u)
    {
        if (GimbalDualYawImuCenterCapture(control, snapshot) != 0u)
        {
            GimbalTotalPidClear(control);
        }
    }

    s_dual_yaw_output_allowed_prev = s_dual_yaw_imu_center_ready;
}

static void GimbalInit(GimbalControl *init)
{

    const fp32 Pitch_speed_pid[3] = {PITCH_SPEED_PID_KP, PITCH_SPEED_PID_KI, PITCH_SPEED_PID_KD};
    const fp32 Yaw_speed_pid[3] = {YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD};
    (void)MotorInstBindCurrent(GimbalOutputMotorNames,
                                              GIMBAL_OUTPUT_MOTOR_COUNT,
                                              GimbalOutputCurrentBindings,
                                              GIMBAL_OUTPUT_MOTOR_COUNT);
    (void)MotorInstBindCurrent(DualYawGimbalOutputMotorNames,
                                              DUAL_YAW_GIMBAL_OUTPUT_MOTOR_COUNT,
                                              DualYawGimbalOutputCurrentBindings,
                                              DUAL_YAW_GIMBAL_OUTPUT_MOTOR_COUNT);
    //电机数据指针获取
    init->GimbalYawMotor.GimbalMotorMeasure = GimbalYawControlMeasurePoint();
    init->GimbalPitchMotor.GimbalMotorMeasure = get_pitch_gimbal_motor_measure_point();
    init->GimbalINTGyroPoint = get_gyro_data_point();
    init->GimbalInsAnglePoint = get_INS_angle_point();
    init->GimbalRcCtrl = get_remote_control_point();
    init->GimbalPitchMotor.mode = init->GimbalPitchMotor.last_mode = GIMBAL_MOTOR_RAW;
    init->GimbalYawMotor.mode = init->GimbalYawMotor.last_mode = GIMBAL_MOTOR_RAW;
    //初始化yaw电机pid
    GimbalPidInit(&init->GimbalYawMotor.GimbalMotorAnglePid, YAW_ENCODE_ANGLE_PID_MAX_OUT, YAW_ENCODE_ANGLE_PID_MAX_IOUT, YAW_ENCODE_ANGLE_PID_KP, YAW_ENCODE_ANGLE_PID_KI, YAW_ENCODE_ANGLE_PID_KD);
    PID_init(&init->GimbalYawMotor.GimbalMotorGyroPid, PID_POSITION, Yaw_speed_pid, YAW_SPEED_PID_MAX_OUT, YAW_SPEED_PID_MAX_IOUT);
    //初始化pitch电机pid
    GimbalPidInit(&init->GimbalPitchMotor.GimbalMotorAnglePid, PITCH_ENCODE_ANGLE_PID_MAX_OUT, PITCH_ENCODE_ANGLE_PID_MAX_IOUT, PITCH_ENCODE_ANGLE_PID_KP, PITCH_ENCODE_ANGLE_PID_KI, PITCH_ENCODE_ANGLE_PID_KD);
    PID_init(&init->GimbalPitchMotor.GimbalMotorGyroPid, PID_POSITION, Pitch_speed_pid, PITCH_SPEED_PID_MAX_OUT, PITCH_SPEED_PID_MAX_IOUT);
    {
        const fp32 limit = fabsf(PITCH_CURRENT_LIMIT);
        if (limit > 0.0f)
        {
            if (init->GimbalPitchMotor.GimbalMotorGyroPid.max_out > limit)
            {
                init->GimbalPitchMotor.GimbalMotorGyroPid.max_out = limit;
            }
            if (init->GimbalPitchMotor.GimbalMotorGyroPid.max_iout > limit)
            {
                init->GimbalPitchMotor.GimbalMotorGyroPid.max_iout = limit;
            }
        }
    }

    //清除所有PID
    GimbalTotalPidClear(init);
    s_dual_yaw_imu_center_ready = 0u;
    s_dual_yaw_imu_center = 0.0f;
    s_dual_yaw_output_allowed_prev = 0u;
    init->GimbalYawMotor.offset_ecd = g_config.gimbal.yaw_middle_ecd;
    init->GimbalPitchMotor.offset_ecd = GIMBAL_PITCH_MIDDLE_ECD;

    {
        GimbalRuntimeSnapshot snapshot;
        GimbalSnapshotCapture(&snapshot, init);
        GimbalFeedbackUpdate(init, &snapshot);
    }

    init->GimbalYawMotor.angle_set = init->GimbalYawMotor.angle;
    init->GimbalYawMotor.motor_gyro_set = init->GimbalYawMotor.motor_gyro;

    init->GimbalPitchMotor.angle_set = init->GimbalPitchMotor.angle;
    init->GimbalPitchMotor.motor_gyro_set = init->GimbalPitchMotor.motor_gyro;
    {
        fp32 cfg_max = YAW_SOFT_LIMIT_MAX;
        fp32 cfg_min = YAW_SOFT_LIMIT_MIN;
        if (cfg_max < cfg_min)
        {
            const fp32 t = cfg_max;
            cfg_max = cfg_min;
            cfg_min = t;
        }
        if ((cfg_max > cfg_min) && (fabsf(cfg_max) < 100.0f) && (fabsf(cfg_min) < 100.0f))
        {
            init->GimbalYawMotor.max_angle = cfg_max;
            init->GimbalYawMotor.min_angle = cfg_min;
        }
        else if ((init->GimbalYawMotor.max_angle <= init->GimbalYawMotor.min_angle) ||
                 ((init->GimbalYawMotor.max_angle == 0.0f) && (init->GimbalYawMotor.min_angle == 0.0f)) ||
                 (fabsf(init->GimbalYawMotor.max_angle) > 100.0f) ||
                 (fabsf(init->GimbalYawMotor.min_angle) > 100.0f))
        {
            init->GimbalYawMotor.max_angle = PI;
            init->GimbalYawMotor.min_angle = -PI;
        }
    }

    {
        const fp32 cfg_up = PITCH_SOFT_LIMIT_UP;
        const fp32 cfg_down = PITCH_SOFT_LIMIT_DOWN;
        fp32 cfg_min = cfg_up;
        fp32 cfg_max = cfg_down;
        if (cfg_max < cfg_min)
        {
            const fp32 t = cfg_max;
            cfg_max = cfg_min;
            cfg_min = t;
        }
        if ((cfg_max > cfg_min) && (fabsf(cfg_max) < 100.0f) && (fabsf(cfg_min) < 100.0f))
        {
            init->GimbalPitchMotor.max_angle = cfg_max;
            init->GimbalPitchMotor.min_angle = cfg_min;
        }
        else
        {
            if ((init->GimbalPitchMotor.max_angle <= init->GimbalPitchMotor.min_angle) ||
                ((init->GimbalPitchMotor.max_angle == 0.0f) && (init->GimbalPitchMotor.min_angle == 0.0f)) ||
                (fabsf(init->GimbalPitchMotor.max_angle) > 100.0f) ||
                (fabsf(init->GimbalPitchMotor.min_angle) > 100.0f))
            {
                init->GimbalPitchMotor.max_angle = 4.0f * PI;
                init->GimbalPitchMotor.min_angle = -4.0f * PI;
            }
        }
    }

}

/**
  * @brief          set gimbal control mode, mainly call 'GimbalBehaviourModeSet' function
  * @param[out]     GimbalSetMode: "GimbalControl" variable point
  * @retval         none
  */
static void GimbalSetMode(GimbalControl *set_mode)
{
    if (set_mode == NULL)
    {
        return;
    }
    GimbalBehaviourModeSet(set_mode);
}
/**
  * @brief          gimbal some measure data update, such as motor encoder, euler angle, gyro
  * @param[out]     GimbalFeedbackUpdate: "GimbalControl" variable point
  * @retval         none
  */
/**
  * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
  * @param[out]     GimbalFeedbackUpdate:"GimbalControl"变量指针.
  * @retval         none
  */
static void GimbalFeedbackUpdate(GimbalControl *feedback_update, const GimbalRuntimeSnapshot *snapshot)
{
    if (feedback_update == NULL)
    {
        return;
    }

#if GIMBAL_USE_ENCODER_FEEDBACK
    {
        const motor_measure_t *pitch_measure = (snapshot != NULL) ?
                                                   snapshot->pitch_measure :
                                                   feedback_update->GimbalPitchMotor.GimbalMotorMeasure;
        const motor_measure_t *yaw_measure = (snapshot != NULL) ?
                                                 snapshot->yaw_measure :
                                                 feedback_update->GimbalYawMotor.GimbalMotorMeasure;
        const uint8_t pitch_turn = (snapshot != NULL) ? snapshot->pitch_turn : (PITCH_TURN ? 1u : 0u);
        const uint8_t yaw_turn = (snapshot != NULL) ? snapshot->yaw_turn : (YAW_TURN ? 1u : 0u);

        GimbalFeedbackFromEncoder(&feedback_update->GimbalPitchMotor, pitch_measure, pitch_turn);
        GimbalFeedbackFromEncoder(&feedback_update->GimbalYawMotor, yaw_measure, yaw_turn);
        return;
    }
#else
    {
        fp32 pitch_angle_raw = 0.0f;
        fp32 pitch_speed_raw = 0.0f;

        const fp32 *ins_angle = (snapshot != NULL) ? snapshot->ins_angle : feedback_update->GimbalInsAnglePoint;
        const fp32 *gyro = (snapshot != NULL) ? snapshot->gyro : feedback_update->GimbalINTGyroPoint;
        const uint8_t pitch_turn = (snapshot != NULL) ? snapshot->pitch_turn : (PITCH_TURN ? 1u : 0u);

        if (ins_angle != NULL)
        {
            pitch_angle_raw = ins_angle[INS_ROLL_ADDRESS_OFFSET];
        }
        if (gyro != NULL)
        {
            pitch_speed_raw = gyro[INS_GYRO_X_ADDRESS_OFFSET];
        }

        feedback_update->GimbalPitchMotor.angle = (pitch_turn != 0u) ? -pitch_angle_raw : pitch_angle_raw;
        feedback_update->GimbalPitchMotor.motor_gyro = (pitch_turn != 0u) ? -pitch_speed_raw : pitch_speed_raw;
    }

    {
        fp32 yaw_angle_raw = 0.0f;
        fp32 yaw_speed_raw = 0.0f;
        fp32 yaw_angle = 0.0f;
        const fp32 *ins_angle = (snapshot != NULL) ? snapshot->ins_angle : feedback_update->GimbalInsAnglePoint;
        const fp32 *gyro = (snapshot != NULL) ? snapshot->gyro : feedback_update->GimbalINTGyroPoint;
        const uint8_t yaw_turn = GimbalYawImuFeedbackTurn(snapshot);

        if (ins_angle != NULL)
        {
            yaw_angle_raw = ins_angle[INS_YAW_ADDRESS_OFFSET];
        }

        if (gyro != NULL)
        {
            yaw_speed_raw = gyro[INS_GYRO_Z_ADDRESS_OFFSET];
        }

        yaw_angle = (yaw_turn != 0u) ? -yaw_angle_raw : yaw_angle_raw;
        if (GimbalDualYawImuCenterEnabled(snapshot) != 0u)
        {
            feedback_update->GimbalYawMotor.angle = (s_dual_yaw_imu_center_ready != 0u) ?
                                                        rad_format(yaw_angle - s_dual_yaw_imu_center) :
                                                        0.0f;
        }
        else
        {
            feedback_update->GimbalYawMotor.angle = yaw_angle;
        }
        feedback_update->GimbalYawMotor.motor_gyro = (yaw_turn != 0u) ? -yaw_speed_raw : yaw_speed_raw;
    }
#endif
}

/**
  * @brief          calculate the encoder angle between ecd and offset_ecd
  * @param[in]      ecd: motor now encode
  * @param[in]      offset_ecd: gimbal offset encode
  * @retval         angle, unit rad
  */
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
    int32_t relative_ecd = ecd - offset_ecd;
    if (relative_ecd > HALF_ECD_RANGE)
    {
        relative_ecd -= ECD_RANGE;
    }
    else if (relative_ecd < -HALF_ECD_RANGE)
    {
        relative_ecd += ECD_RANGE;
    }

    return relative_ecd * MOTOR_ECD_TO_RAD;
}

#if GIMBAL_USE_ENCODER_FEEDBACK
static void GimbalFeedbackFromEncoder(GimbalMotor *motor,
                                         const motor_measure_t *measure,
                                         uint8_t turn)
{
    fp32 angle;
    fp32 speed;

    if (motor == NULL || measure == NULL)
    {
        return;
    }

    angle = motor_ecd_to_angle_change(measure->ecd, motor->offset_ecd);
    speed = (fp32)measure->speed_rpm * 0.104719755f;

    if (turn != 0u)
    {
        angle = -angle;
        speed = -speed;
    }

    motor->angle = angle;
    motor->motor_gyro = speed;
    motor->motor_speed = speed;
}
#endif

/**
  * @brief          when gimbal mode change, some param should be changed, such as  yaw_set should be new yaw
  * @param[out]     GimbalModeChange: "GimbalControl" variable point
  * @retval         none
  */
/**
  * @brief          云台模式改变，有些参数需要改变，例如控制yaw角度设定值应该变成当前yaw角度
  * @param[out]     GimbalModeChange:"GimbalControl"变量指针.
  * @retval         none
  */
static void GimbalModeChangeControlTransit(GimbalControl *GimbalModeChange)
{
    if (GimbalModeChange == NULL)
    {
        return;
    }
    //yaw电机状态机切换保存数据
    if (GimbalModeChange->GimbalYawMotor.last_mode != GIMBAL_MOTOR_RAW && GimbalModeChange->GimbalYawMotor.mode == GIMBAL_MOTOR_RAW)
    {
        GimbalModeChange->GimbalYawMotor.raw_cmd_current = GimbalModeChange->GimbalYawMotor.current_set = GimbalModeChange->GimbalYawMotor.given_current;
    }
    else if (GimbalModeChange->GimbalYawMotor.last_mode != GIMBAL_MOTOR_ENCODER && GimbalModeChange->GimbalYawMotor.mode == GIMBAL_MOTOR_ENCODER)
    {
        GimbalModeChange->GimbalYawMotor.angle_set = GimbalModeChange->GimbalYawMotor.angle;
    }
    GimbalModeChange->GimbalYawMotor.last_mode = GimbalModeChange->GimbalYawMotor.mode;

    //pitch电机状态机切换保存数据
    if (GimbalModeChange->GimbalPitchMotor.last_mode != GIMBAL_MOTOR_RAW && GimbalModeChange->GimbalPitchMotor.mode == GIMBAL_MOTOR_RAW)
    {
        GimbalModeChange->GimbalPitchMotor.raw_cmd_current = GimbalModeChange->GimbalPitchMotor.current_set = GimbalModeChange->GimbalPitchMotor.given_current;
    }
    else if (GimbalModeChange->GimbalPitchMotor.last_mode != GIMBAL_MOTOR_ENCODER && GimbalModeChange->GimbalPitchMotor.mode == GIMBAL_MOTOR_ENCODER)
    {
        GimbalModeChange->GimbalPitchMotor.angle_set = GimbalModeChange->GimbalPitchMotor.angle;
    }

    GimbalModeChange->GimbalPitchMotor.last_mode = GimbalModeChange->GimbalPitchMotor.mode;
}
/**
  * @brief          set gimbal control set-point, control set-point is set by "GimbalBehaviourControlSet".
  * @param[out]     GimbalSetControl: "GimbalControl" variable point
  * @retval         none
  */
static void GimbalSetControl(GimbalControl *set_control)
{
    fp32 add_yaw_angle = 0.0f;
    fp32 add_pitch_angle = 0.0f;
    VisionToGimbal VisionCmd;
    uint8_t image_manual_active;
    uint8_t image_auto_aim_on;

    if (set_control == NULL)
    {
        return;
    }

    GimbalBehaviourControlSet(&add_yaw_angle, &add_pitch_angle, set_control);

    image_manual_active = (set_control->fast.active_source == MANUAL_INPUT_SRC_IMAGE) ? 1u : 0u;
    image_auto_aim_on = set_control->fast.image_auto_aim_requested;
    if (VisionTakeLatest(&VisionCmd) && (VisionCmd.mode == 1U || VisionCmd.mode == 2U) &&
        (image_manual_active == 0u || image_auto_aim_on != 0u) &&
        set_control->GimbalYawMotor.mode == GIMBAL_MOTOR_ENCODER &&
        set_control->GimbalPitchMotor.mode == GIMBAL_MOTOR_ENCODER &&
        GimbalBehaviourWatch == GIMBAL_ANGLE &&
        !GimbalTurnaroundIsActive())
    {
        set_control->GimbalYawMotor.angle_set = rad_format(VisionCmd.yaw);
        set_control->GimbalPitchMotor.angle_set = fp32_constrain(-VisionCmd.pitch,
                                                                   set_control->GimbalPitchMotor.min_angle,
                                                                   set_control->GimbalPitchMotor.max_angle);
        add_yaw_angle = 0.0f;
        add_pitch_angle = 0.0f;
    }

    if (set_control->GimbalYawMotor.mode == GIMBAL_MOTOR_RAW)
    {
        set_control->GimbalYawMotor.raw_cmd_current = add_yaw_angle;
    }
    else if (set_control->GimbalYawMotor.mode == GIMBAL_MOTOR_ENCODER)
    {
        GimbalAngleLimit(&set_control->GimbalYawMotor, add_yaw_angle);
    }

    if (set_control->GimbalPitchMotor.mode == GIMBAL_MOTOR_RAW)
    {
        set_control->GimbalPitchMotor.raw_cmd_current = add_pitch_angle;
    }
    else if (set_control->GimbalPitchMotor.mode == GIMBAL_MOTOR_ENCODER)
    {
        GimbalAngleLimit(&set_control->GimbalPitchMotor, add_pitch_angle);
    }
}

static void GimbalAngleLimit(GimbalMotor *GimbalMotor, fp32 add)
{
    fp32 angle_set;

    if (GimbalMotor == NULL)
    {
        return;
    }
    angle_set = GimbalMotor->angle_set + add;

    // Yaw is clamped when a soft limit is configured; otherwise keep the shared wrap-around behavior.
    if (GimbalMotor == &g_gimbal.GimbalYawMotor)
    {
        if (GimbalYawSoftLimitEnabled() != 0u)
        {
            if (angle_set > GimbalMotor->max_angle)
            {
                angle_set = GimbalMotor->max_angle;
            }
            else if (angle_set < GimbalMotor->min_angle)
            {
                angle_set = GimbalMotor->min_angle;
            }
            GimbalMotor->angle_set = angle_set;
        }
        else
        {
            GimbalMotor->angle_set = rad_format(angle_set);
        }
        return;
    }
    if (angle_set > GimbalMotor->max_angle)
    {
        angle_set = GimbalMotor->max_angle;
    }
    else if (angle_set < GimbalMotor->min_angle)
    {
        angle_set = GimbalMotor->min_angle;
    }
    GimbalMotor->angle_set = angle_set;
}

static fp32 GimbalYawChassisSpinFfCurrent(const GimbalMotor *yaw_motor)
{
    ChassisState chassis;
    static uint8_t was_spin_active = 0u;
    static uint32_t exit_start_ms = 0u;
    static fp32 exit_yaw_err = 0.0f;

    if (yaw_motor == NULL ||
        ChassisStateRead(&chassis) == 0u ||
        chassis.valid == 0u)
    {
        return 0.0f;
    }

    const fp32 wz_set = chassis.wz_set;
    const fp32 wz_meas = chassis.wz;
    const fp32 yaw_dir = YAW_TURN ? 1.0f : -1.0f;
    const uint32_t now_ms = BspTimeGetTickMs();
    const uint8_t spin_active =
        (chassis.mode == CHASSIS_STATE_MODE_NO_FOLLOW_YAW &&
         fabsf(wz_set) >= GIMBAL_YAW_CHASSIS_SPIN_FF_MIN_WZ_RADPS) ?
            1u :
            0u;

    if (spin_active != 0u)
    {
        was_spin_active = 1u;
        exit_start_ms = 0u;

        const fp32 ff = yaw_dir * wz_set * GIMBAL_YAW_CHASSIS_SPIN_FF_CURRENT_PER_RADPS;
        return fp32_constrain(ff,
                              -GIMBAL_YAW_CHASSIS_SPIN_FF_MAX_CURRENT,
                              GIMBAL_YAW_CHASSIS_SPIN_FF_MAX_CURRENT);
    }

    if (was_spin_active != 0u)
    {
        was_spin_active = 0u;
        exit_start_ms = now_ms;
        exit_yaw_err = rad_format(yaw_motor->angle_set - yaw_motor->angle);
    }

    fp32 ff = 0.0f;
    if (fabsf(wz_meas) >= GIMBAL_YAW_CHASSIS_SPIN_FF_MIN_WZ_RADPS)
    {
        ff += yaw_dir * wz_meas * GIMBAL_YAW_CHASSIS_SPIN_EXIT_RATE_CURRENT_PER_RADPS;
    }

    if (exit_start_ms != 0u)
    {
        const uint32_t elapsed_ms = now_ms - exit_start_ms;
        if (elapsed_ms < GIMBAL_YAW_CHASSIS_SPIN_EXIT_FF_TIME_MS)
        {
            const fp32 decay = 1.0f - ((fp32)elapsed_ms / (fp32)GIMBAL_YAW_CHASSIS_SPIN_EXIT_FF_TIME_MS);
            const fp32 yaw_err_now = rad_format(yaw_motor->angle_set - yaw_motor->angle);
            const fp32 yaw_err_blend = decay * exit_yaw_err + (1.0f - decay) * yaw_err_now;
            ff += yaw_err_blend * GIMBAL_YAW_CHASSIS_SPIN_EXIT_ERR_CURRENT_PER_RAD * decay;
        }
        else
        {
            exit_start_ms = 0u;
        }
    }

    return fp32_constrain(ff,
                          -GIMBAL_YAW_CHASSIS_SPIN_EXIT_FF_MAX_CURRENT,
                          GIMBAL_YAW_CHASSIS_SPIN_EXIT_FF_MAX_CURRENT);
}

static fp32 GimbalYawKickCurrent(const GimbalMotor *yaw_motor)
{
    const fp32 kick = fabsf(YAW_KICK_CURRENT);

    if (yaw_motor == NULL || kick <= 0.0f)
    {
        return 0.0f;
    }
    if (yaw_motor->motor_gyro_set > 0.0f)
    {
        return kick;
    }
    if (yaw_motor->motor_gyro_set < 0.0f)
    {
        return -kick;
    }
    return 0.0f;
}

static void GimbalControlLoop(GimbalControl *control_loop)
{
    if (control_loop == NULL)
    {
        return;
    }

    if (control_loop->GimbalYawMotor.mode == GIMBAL_MOTOR_RAW)
    {
        GimbalMotorRawAngleControl(&control_loop->GimbalYawMotor);
    }
    else if (control_loop->GimbalYawMotor.mode == GIMBAL_MOTOR_ENCODER)
    {
        GimbalMotorAngleControl(&control_loop->GimbalYawMotor);
    }

    if (control_loop->GimbalPitchMotor.mode == GIMBAL_MOTOR_RAW)
    {
        GimbalMotorRawAngleControl(&control_loop->GimbalPitchMotor);
    }
    else if (control_loop->GimbalPitchMotor.mode == GIMBAL_MOTOR_ENCODER)
    {
        GimbalMotorAngleControl(&control_loop->GimbalPitchMotor);
    }
}

/**
  * @brief          gimbal control mode :GIMBAL_MOTOR_ENCODER, use the encoder angle to control.
  * @param[out]     GimbalMotor: yaw motor or pitch motor
  * @retval         none
  */
/**
  * @brief          云台控制模式:GIMBAL_MOTOR_ENCODER，使用编码角进行控制
  * @param[out]     GimbalMotor:yaw电机或者pitch电机
  * @retval         none
  */
static void GimbalMotorAngleControl(GimbalMotor *GimbalMotor)
{
    GimbalCoreAxisInput core_input;
    GimbalCoreAxisOutput core_output;
    fp32 pitch_err;
    fp32 kick_up;
    fp32 kick_down;
    fp32 ff_hold;
    fp32 ff_table_hold;
    fp32 ff_kick_up;
    fp32 ff_kick_down;
    fp32 kick_scale;
    fp32 current_limit;
    axis_current_conditioner_info_t limit_info;

    if (GimbalMotor == NULL)
    {
        return;
    }

    memset(&core_input, 0, sizeof(core_input));
    memset(&core_output, 0, sizeof(core_output));
    core_input.mode = (uint8_t)GIMBAL_CORE_MODE_ANGLE;
    core_input.angle_rad = GimbalMotor->angle;
    core_input.angle_set_rad = GimbalMotor->angle_set;
    core_input.gyro_radps = GimbalMotor->motor_gyro;
    GimbalCoreStepAxisBase(&GimbalMotor->GimbalMotorAnglePid,
                               &GimbalMotor->GimbalMotorGyroPid,
                               &core_input,
                               &core_output);
    GimbalMotor->motor_gyro_set = core_output.gyro_set_radps;
    GimbalMotor->current_set = core_output.current_set;
    GimbalMotor->given_current = core_output.given_current;

    if (GimbalMotor == &g_gimbal.GimbalYawMotor)
    {
        const fp32 yaw_current_limit = fabsf(GimbalMotor->GimbalMotorGyroPid.max_out);
        GimbalMotor->current_set += GimbalYawChassisSpinFfCurrent(GimbalMotor);
        GimbalMotor->current_set += GimbalYawKickCurrent(GimbalMotor);
        if (GimbalYawSoftLimitEnabled() != 0u)
        {
            if ((GimbalMotor->angle >= GimbalMotor->max_angle && GimbalMotor->motor_gyro_set > 0.0f) ||
                (GimbalMotor->angle <= GimbalMotor->min_angle && GimbalMotor->motor_gyro_set < 0.0f))
            {
                GimbalMotor->current_set = 0.0f;
                GimbalMotor->GimbalMotorAnglePid.Iout = 0.0f;
                GimbalMotor->GimbalMotorGyroPid.Iout = 0.0f;
            }
        }
        if (yaw_current_limit > 0.0f)
        {
            GimbalMotor->current_set = fp32_constrain(GimbalMotor->current_set,
                                                       -yaw_current_limit,
                                                       yaw_current_limit);
        }
    }

    GimbalMotor->given_current = (int16_t)(GimbalMotor->current_set);

    if (GimbalMotor == &g_gimbal.GimbalPitchMotor)
    {
        pitch_err = GimbalMotor->angle_set - GimbalMotor->angle;
        kick_up = fabsf(PITCH_KICK_UP_CURRENT);
        kick_down = fabsf(PITCH_KICK_DOWN_CURRENT);
        ff_hold = GIMBAL_PITCH_HOLD_CURRENT;
        ff_table_hold = 0.0f;
        ff_kick_up = 0.0f;
        ff_kick_down = 0.0f;
        kick_scale = 0.0f;
        current_limit = fabsf(PITCH_CURRENT_LIMIT);
        memset(&limit_info, 0, sizeof(limit_info));

        if (PitchCaliGetComp(GimbalMotor->angle, &ff_table_hold, &ff_kick_up, &ff_kick_down))
        {
            ff_hold = ff_table_hold;
            kick_up = fabsf(ff_kick_up);
            kick_down = fabsf(ff_kick_down);
        }
        GimbalMotor->current_set += ff_hold;

        kick_scale = GimbalPitchKickScale(pitch_err);
        kick_up *= kick_scale;
        kick_down *= kick_scale;

        GimbalMotor->current_set = axis_current_conditioner_apply(
            GimbalMotor->current_set,
            GimbalMotor->motor_gyro_set,
            kick_up,
            kick_down,
            GimbalMotor->angle,
            GimbalMotor->min_angle,
            GimbalMotor->max_angle,
            current_limit,
            &GimbalMotor->GimbalMotorAnglePid.Iout,
            &GimbalMotor->GimbalMotorGyroPid.Iout,
            &limit_info);

        GimbalMotor->given_current = (int16_t)(GimbalMotor->current_set);
    }
}

static void GimbalMotorRawAngleControl(GimbalMotor *GimbalMotor)
{
    GimbalCoreAxisInput core_input;
    GimbalCoreAxisOutput core_output;

    if (GimbalMotor == NULL)
    {
        return;
    }

    memset(&core_input, 0, sizeof(core_input));
    memset(&core_output, 0, sizeof(core_output));
    core_input.mode = (uint8_t)GIMBAL_CORE_MODE_RAW;
    core_input.gyro_set_radps = GimbalMotor->motor_gyro_set;
    core_input.raw_current = GimbalMotor->raw_cmd_current;
    GimbalCoreStepAxisBase(&GimbalMotor->GimbalMotorAnglePid,
                               &GimbalMotor->GimbalMotorGyroPid,
                               &core_input,
                               &core_output);
    GimbalMotor->motor_gyro_set = core_output.gyro_set_radps;
    GimbalMotor->current_set = core_output.current_set;
    GimbalMotor->given_current = core_output.given_current;
}

#if GIMBAL_TEST_MODE
int32_t yaw_angle_int_1000, pitch_angle_int_1000;
int32_t yaw_angle_set_1000, pitch_angle_set_1000;
int32_t yaw_speed_int_1000, pitch_speed_int_1000;
int32_t yaw_speed_set_int_1000, pitch_speed_set_int_1000;
static void J_scope_gimbal_test(void)
{
    yaw_angle_int_1000 = (int32_t)(g_gimbal.GimbalYawMotor.angle * 1000);
    yaw_angle_set_1000 = (int32_t)(g_gimbal.GimbalYawMotor.angle_set * 1000);
    yaw_speed_int_1000 = (int32_t)(g_gimbal.GimbalYawMotor.motor_gyro * 1000);
    yaw_speed_set_int_1000 = (int32_t)(g_gimbal.GimbalYawMotor.motor_gyro_set * 1000);

    pitch_angle_int_1000 = (int32_t)(g_gimbal.GimbalPitchMotor.angle * 1000);
    pitch_angle_set_1000 = (int32_t)(g_gimbal.GimbalPitchMotor.angle_set * 1000);
    pitch_speed_int_1000 = (int32_t)(g_gimbal.GimbalPitchMotor.motor_gyro * 1000);
    pitch_speed_set_int_1000 = (int32_t)(g_gimbal.GimbalPitchMotor.motor_gyro_set * 1000);
}

#endif
