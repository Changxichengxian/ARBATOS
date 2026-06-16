/*
 * SPDX-FileCopyrightText: 2026 ?? <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Private implementation chunk included by gimbal_control_task.c.
 */

/**
  * @brief          "gimbal_control" variable initialization, include pid initialization, remote control data point initialization, gimbal motors
  *                 data point initialization, and gyro sensor angle point initialization.
  * @param[out]     init: "gimbal_control" variable point
  * @retval         none
  */
static void gimbal_init(gimbal_control_t *init)
{

    const fp32 Pitch_speed_pid[3] = {PITCH_SPEED_PID_KP, PITCH_SPEED_PID_KI, PITCH_SPEED_PID_KD};
    const fp32 Yaw_speed_pid[3] = {YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD};
    (void)MotorInstBindCurrent(gimbal_output_motor_names,
                                              GIMBAL_OUTPUT_MOTOR_COUNT,
                                              gimbal_output_current_bindings,
                                              GIMBAL_OUTPUT_MOTOR_COUNT);
    (void)MotorInstBindCurrent(dual_yaw_gimbal_output_motor_names,
                                              DUAL_YAW_GIMBAL_OUTPUT_MOTOR_COUNT,
                                              dual_yaw_gimbal_output_current_bindings,
                                              DUAL_YAW_GIMBAL_OUTPUT_MOTOR_COUNT);
    //电机数据指针获取
    init->gimbal_yaw_motor.gimbal_motor_measure = gimbal_yaw_control_measure_point();
    init->gimbal_pitch_motor.gimbal_motor_measure = get_pitch_gimbal_motor_measure_point();
    init->gimbal_INT_gyro_point = get_gyro_data_point();
    init->gimbal_INS_angle_point = get_INS_angle_point();
    init->gimbal_rc_ctrl = get_remote_control_point();
    init->gimbal_pitch_motor.gimbal_motor_mode = init->gimbal_pitch_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    init->gimbal_yaw_motor.gimbal_motor_mode = init->gimbal_yaw_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    //初始化yaw电机pid
    gimbal_PID_init(&init->gimbal_yaw_motor.gimbal_motor_angle_pid, YAW_ENCODE_ANGLE_PID_MAX_OUT, YAW_ENCODE_ANGLE_PID_MAX_IOUT, YAW_ENCODE_ANGLE_PID_KP, YAW_ENCODE_ANGLE_PID_KI, YAW_ENCODE_ANGLE_PID_KD);
    PID_init(&init->gimbal_yaw_motor.gimbal_motor_gyro_pid, PID_POSITION, Yaw_speed_pid, YAW_SPEED_PID_MAX_OUT, YAW_SPEED_PID_MAX_IOUT);
    //初始化pitch电机pid
    gimbal_PID_init(&init->gimbal_pitch_motor.gimbal_motor_angle_pid, PITCH_ENCODE_ANGLE_PID_MAX_OUT, PITCH_ENCODE_ANGLE_PID_MAX_IOUT, PITCH_ENCODE_ANGLE_PID_KP, PITCH_ENCODE_ANGLE_PID_KI, PITCH_ENCODE_ANGLE_PID_KD);
    PID_init(&init->gimbal_pitch_motor.gimbal_motor_gyro_pid, PID_POSITION, Pitch_speed_pid, PITCH_SPEED_PID_MAX_OUT, PITCH_SPEED_PID_MAX_IOUT);
    {
        const fp32 limit = fabsf(PITCH_CURRENT_LIMIT);
        if (limit > 0.0f)
        {
            if (init->gimbal_pitch_motor.gimbal_motor_gyro_pid.max_out > limit)
            {
                init->gimbal_pitch_motor.gimbal_motor_gyro_pid.max_out = limit;
            }
            if (init->gimbal_pitch_motor.gimbal_motor_gyro_pid.max_iout > limit)
            {
                init->gimbal_pitch_motor.gimbal_motor_gyro_pid.max_iout = limit;
            }
        }
    }

    //清除所有PID
    gimbal_total_pid_clear(init);
    init->gimbal_yaw_motor.offset_ecd = g_config.gimbal.yaw_middle_ecd;
    init->gimbal_pitch_motor.offset_ecd = GIMBAL_PITCH_MIDDLE_ECD;

    {
        gimbal_runtime_snapshot_t snapshot;
        gimbal_snapshot_capture(&snapshot, init);
        gimbal_feedback_update(init, &snapshot);
    }

    init->gimbal_yaw_motor.angle_set = init->gimbal_yaw_motor.angle;
    init->gimbal_yaw_motor.motor_gyro_set = init->gimbal_yaw_motor.motor_gyro;

    init->gimbal_pitch_motor.angle_set = init->gimbal_pitch_motor.angle;
    init->gimbal_pitch_motor.motor_gyro_set = init->gimbal_pitch_motor.motor_gyro;
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
            init->gimbal_yaw_motor.max_angle = cfg_max;
            init->gimbal_yaw_motor.min_angle = cfg_min;
        }
        else if ((init->gimbal_yaw_motor.max_angle <= init->gimbal_yaw_motor.min_angle) ||
                 ((init->gimbal_yaw_motor.max_angle == 0.0f) && (init->gimbal_yaw_motor.min_angle == 0.0f)) ||
                 (fabsf(init->gimbal_yaw_motor.max_angle) > 100.0f) ||
                 (fabsf(init->gimbal_yaw_motor.min_angle) > 100.0f))
        {
            init->gimbal_yaw_motor.max_angle = PI;
            init->gimbal_yaw_motor.min_angle = -PI;
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
            init->gimbal_pitch_motor.max_angle = cfg_max;
            init->gimbal_pitch_motor.min_angle = cfg_min;
        }
        else
        {
            if ((init->gimbal_pitch_motor.max_angle <= init->gimbal_pitch_motor.min_angle) ||
                ((init->gimbal_pitch_motor.max_angle == 0.0f) && (init->gimbal_pitch_motor.min_angle == 0.0f)) ||
                (fabsf(init->gimbal_pitch_motor.max_angle) > 100.0f) ||
                (fabsf(init->gimbal_pitch_motor.min_angle) > 100.0f))
            {
                init->gimbal_pitch_motor.max_angle = 4.0f * PI;
                init->gimbal_pitch_motor.min_angle = -4.0f * PI;
            }
        }
    }

}

/**
  * @brief          set gimbal control mode, mainly call 'gimbal_behaviour_mode_set' function
  * @param[out]     gimbal_set_mode: "gimbal_control" variable point
  * @retval         none
  */
static void gimbal_set_mode(gimbal_control_t *set_mode)
{
    if (set_mode == NULL)
    {
        return;
    }
    gimbal_behaviour_mode_set(set_mode);
}
/**
  * @brief          gimbal some measure data update, such as motor encoder, euler angle, gyro
  * @param[out]     gimbal_feedback_update: "gimbal_control" variable point
  * @retval         none
  */
/**
  * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
  * @param[out]     gimbal_feedback_update:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_feedback_update(gimbal_control_t *feedback_update, const gimbal_runtime_snapshot_t *snapshot)
{
    if (feedback_update == NULL)
    {
        return;
    }

#if GIMBAL_USE_ENCODER_FEEDBACK
    {
        const motor_measure_t *pitch_measure = (snapshot != NULL) ?
                                                   snapshot->pitch_measure :
                                                   feedback_update->gimbal_pitch_motor.gimbal_motor_measure;
        const motor_measure_t *yaw_measure = (snapshot != NULL) ?
                                                 snapshot->yaw_measure :
                                                 feedback_update->gimbal_yaw_motor.gimbal_motor_measure;
        const uint8_t pitch_turn = (snapshot != NULL) ? snapshot->pitch_turn : (PITCH_TURN ? 1u : 0u);
        const uint8_t yaw_turn = (snapshot != NULL) ? snapshot->yaw_turn : (YAW_TURN ? 1u : 0u);

        gimbal_feedback_from_encoder(&feedback_update->gimbal_pitch_motor, pitch_measure, pitch_turn);
        gimbal_feedback_from_encoder(&feedback_update->gimbal_yaw_motor, yaw_measure, yaw_turn);
        return;
    }
#else
    {
        fp32 pitch_angle_raw = 0.0f;
        fp32 pitch_speed_raw = 0.0f;

        const fp32 *ins_angle = (snapshot != NULL) ? snapshot->ins_angle : feedback_update->gimbal_INS_angle_point;
        const fp32 *gyro = (snapshot != NULL) ? snapshot->gyro : feedback_update->gimbal_INT_gyro_point;
        const uint8_t pitch_turn = (snapshot != NULL) ? snapshot->pitch_turn : (PITCH_TURN ? 1u : 0u);

        if (ins_angle != NULL)
        {
            pitch_angle_raw = ins_angle[INS_ROLL_ADDRESS_OFFSET];
        }
        if (gyro != NULL)
        {
            pitch_speed_raw = gyro[INS_GYRO_X_ADDRESS_OFFSET];
        }

        feedback_update->gimbal_pitch_motor.angle = (pitch_turn != 0u) ? -pitch_angle_raw : pitch_angle_raw;
        feedback_update->gimbal_pitch_motor.motor_gyro = (pitch_turn != 0u) ? -pitch_speed_raw : pitch_speed_raw;
    }

    {
        fp32 yaw_angle_raw = 0.0f;
        fp32 yaw_speed_raw = 0.0f;
        const fp32 *ins_angle = (snapshot != NULL) ? snapshot->ins_angle : feedback_update->gimbal_INS_angle_point;
        const fp32 *gyro = (snapshot != NULL) ? snapshot->gyro : feedback_update->gimbal_INT_gyro_point;
        const uint8_t yaw_turn = (snapshot != NULL) ? snapshot->yaw_turn : (YAW_TURN ? 1u : 0u);

        if (ins_angle != NULL)
        {
            yaw_angle_raw = ins_angle[INS_YAW_ADDRESS_OFFSET];
        }

        if (gyro != NULL)
        {
            yaw_speed_raw = gyro[INS_GYRO_Z_ADDRESS_OFFSET];
        }

        feedback_update->gimbal_yaw_motor.angle = (yaw_turn != 0u) ? -yaw_angle_raw : yaw_angle_raw;
        feedback_update->gimbal_yaw_motor.motor_gyro = (yaw_turn != 0u) ? -yaw_speed_raw : yaw_speed_raw;
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
static void gimbal_feedback_from_encoder(gimbal_motor_t *motor,
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
  * @param[out]     gimbal_mode_change: "gimbal_control" variable point
  * @retval         none
  */
/**
  * @brief          云台模式改变，有些参数需要改变，例如控制yaw角度设定值应该变成当前yaw角度
  * @param[out]     gimbal_mode_change:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_mode_change_control_transit(gimbal_control_t *gimbal_mode_change)
{
    if (gimbal_mode_change == NULL)
    {
        return;
    }
    //yaw电机状态机切换保存数据
    if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_mode_change->gimbal_yaw_motor.raw_cmd_current = gimbal_mode_change->gimbal_yaw_motor.current_set = gimbal_mode_change->gimbal_yaw_motor.given_current;
    }
    else if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCODER && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCODER)
    {
        gimbal_mode_change->gimbal_yaw_motor.angle_set = gimbal_mode_change->gimbal_yaw_motor.angle;
    }
    gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode;

    //pitch电机状态机切换保存数据
    if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_mode_change->gimbal_pitch_motor.raw_cmd_current = gimbal_mode_change->gimbal_pitch_motor.current_set = gimbal_mode_change->gimbal_pitch_motor.given_current;
    }
    else if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCODER && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCODER)
    {
        gimbal_mode_change->gimbal_pitch_motor.angle_set = gimbal_mode_change->gimbal_pitch_motor.angle;
    }

    gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode;
}
/**
  * @brief          set gimbal control set-point, control set-point is set by "gimbal_behaviour_control_set".
  * @param[out]     gimbal_set_control: "gimbal_control" variable point
  * @retval         none
  */
static void gimbal_set_control(gimbal_control_t *set_control)
{
    fp32 add_yaw_angle = 0.0f;
    fp32 add_pitch_angle = 0.0f;
    VisionToGimbal vision_cmd;
    uint8_t image_manual_active;
    uint8_t image_auto_aim_on;

    if (set_control == NULL)
    {
        return;
    }

    gimbal_behaviour_control_set(&add_yaw_angle, &add_pitch_angle, set_control);

    image_manual_active = (set_control->fast.active_source == MANUAL_INPUT_SRC_IMAGE) ? 1u : 0u;
    image_auto_aim_on = set_control->fast.image_auto_aim_requested;
    if (vision_take_latest(&vision_cmd) && (vision_cmd.mode == 1U || vision_cmd.mode == 2U) &&
        (image_manual_active == 0u || image_auto_aim_on != 0u) &&
        set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCODER &&
        set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCODER &&
        gimbal_behaviour_watch == GIMBAL_ANGLE &&
        !gimbal_turnaround_is_active())
    {
        set_control->gimbal_yaw_motor.angle_set = rad_format(vision_cmd.yaw);
        set_control->gimbal_pitch_motor.angle_set = fp32_constrain(-vision_cmd.pitch,
                                                                   set_control->gimbal_pitch_motor.min_angle,
                                                                   set_control->gimbal_pitch_motor.max_angle);
        add_yaw_angle = 0.0f;
        add_pitch_angle = 0.0f;
    }

    if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        set_control->gimbal_yaw_motor.raw_cmd_current = add_yaw_angle;
    }
    else if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCODER)
    {
        gimbal_angle_limit(&set_control->gimbal_yaw_motor, add_yaw_angle);
    }

    if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        set_control->gimbal_pitch_motor.raw_cmd_current = add_pitch_angle;
    }
    else if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCODER)
    {
        gimbal_angle_limit(&set_control->gimbal_pitch_motor, add_pitch_angle);
    }
}

static void gimbal_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add)
{
    fp32 angle_set;

    if (gimbal_motor == NULL)
    {
        return;
    }
    angle_set = gimbal_motor->angle_set + add;

    // Yaw is clamped when a soft limit is configured; otherwise keep the shared wrap-around behavior.
    if (gimbal_motor == &gimbal_control.gimbal_yaw_motor)
    {
        if (gimbal_yaw_soft_limit_enabled() != 0u)
        {
            if (angle_set > gimbal_motor->max_angle)
            {
                angle_set = gimbal_motor->max_angle;
            }
            else if (angle_set < gimbal_motor->min_angle)
            {
                angle_set = gimbal_motor->min_angle;
            }
            gimbal_motor->angle_set = angle_set;
        }
        else
        {
            gimbal_motor->angle_set = rad_format(angle_set);
        }
        return;
    }
    if (angle_set > gimbal_motor->max_angle)
    {
        angle_set = gimbal_motor->max_angle;
    }
    else if (angle_set < gimbal_motor->min_angle)
    {
        angle_set = gimbal_motor->min_angle;
    }
    gimbal_motor->angle_set = angle_set;
}

static fp32 gimbal_yaw_chassis_spin_ff_current(const gimbal_motor_t *yaw_motor)
{
    chassis_state_t chassis;
    static uint8_t was_spin_active = 0u;
    static uint32_t exit_start_ms = 0u;
    static fp32 exit_yaw_err = 0.0f;

    if (yaw_motor == NULL ||
        chassis_state_read(&chassis) == 0u ||
        chassis.valid == 0u)
    {
        return 0.0f;
    }

    const fp32 wz_set = chassis.wz_set;
    const fp32 wz_meas = chassis.wz;
    const fp32 yaw_dir = YAW_TURN ? 1.0f : -1.0f;
    const uint32_t now_ms = bsp_time_get_tick_ms();
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

static fp32 gimbal_yaw_kick_current(const gimbal_motor_t *yaw_motor)
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

static void gimbal_control_loop(gimbal_control_t *control_loop)
{
    if (control_loop == NULL)
    {
        return;
    }

    if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_motor_raw_angle_control(&control_loop->gimbal_yaw_motor);
    }
    else if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCODER)
    {
        gimbal_motor_angle_control(&control_loop->gimbal_yaw_motor);
    }

    if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_motor_raw_angle_control(&control_loop->gimbal_pitch_motor);
    }
    else if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCODER)
    {
        gimbal_motor_angle_control(&control_loop->gimbal_pitch_motor);
    }
}

/**
  * @brief          gimbal control mode :GIMBAL_MOTOR_ENCODER, use the encoder angle to control.
  * @param[out]     gimbal_motor: yaw motor or pitch motor
  * @retval         none
  */
/**
  * @brief          云台控制模式:GIMBAL_MOTOR_ENCODER，使用编码角进行控制
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
  */
static void gimbal_motor_angle_control(gimbal_motor_t *gimbal_motor)
{
    gimbal_core_axis_input_t core_input;
    gimbal_core_axis_output_t core_output;
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

    if (gimbal_motor == NULL)
    {
        return;
    }

    memset(&core_input, 0, sizeof(core_input));
    memset(&core_output, 0, sizeof(core_output));
    core_input.mode = (uint8_t)GIMBAL_CORE_MODE_ANGLE;
    core_input.angle_rad = gimbal_motor->angle;
    core_input.angle_set_rad = gimbal_motor->angle_set;
    core_input.gyro_radps = gimbal_motor->motor_gyro;
    gimbal_core_step_axis_base(&gimbal_motor->gimbal_motor_angle_pid,
                               &gimbal_motor->gimbal_motor_gyro_pid,
                               &core_input,
                               &core_output);
    gimbal_motor->motor_gyro_set = core_output.gyro_set_radps;
    gimbal_motor->current_set = core_output.current_set;
    gimbal_motor->given_current = core_output.given_current;

    if (gimbal_motor == &gimbal_control.gimbal_yaw_motor)
    {
        const fp32 yaw_current_limit = fabsf(gimbal_motor->gimbal_motor_gyro_pid.max_out);
        gimbal_motor->current_set += gimbal_yaw_chassis_spin_ff_current(gimbal_motor);
        gimbal_motor->current_set += gimbal_yaw_kick_current(gimbal_motor);
        if (gimbal_yaw_soft_limit_enabled() != 0u)
        {
            if ((gimbal_motor->angle >= gimbal_motor->max_angle && gimbal_motor->motor_gyro_set > 0.0f) ||
                (gimbal_motor->angle <= gimbal_motor->min_angle && gimbal_motor->motor_gyro_set < 0.0f))
            {
                gimbal_motor->current_set = 0.0f;
                gimbal_motor->gimbal_motor_angle_pid.Iout = 0.0f;
                gimbal_motor->gimbal_motor_gyro_pid.Iout = 0.0f;
            }
        }
        if (yaw_current_limit > 0.0f)
        {
            gimbal_motor->current_set = fp32_constrain(gimbal_motor->current_set,
                                                       -yaw_current_limit,
                                                       yaw_current_limit);
        }
    }

    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);

    if (gimbal_motor == &gimbal_control.gimbal_pitch_motor)
    {
        pitch_err = gimbal_motor->angle_set - gimbal_motor->angle;
        kick_up = fabsf(PITCH_KICK_UP_CURRENT);
        kick_down = fabsf(PITCH_KICK_DOWN_CURRENT);
        ff_hold = GIMBAL_PITCH_HOLD_CURRENT;
        ff_table_hold = 0.0f;
        ff_kick_up = 0.0f;
        ff_kick_down = 0.0f;
        kick_scale = 0.0f;
        current_limit = fabsf(PITCH_CURRENT_LIMIT);
        memset(&limit_info, 0, sizeof(limit_info));

        if (pitch_cali_get_comp(gimbal_motor->angle, &ff_table_hold, &ff_kick_up, &ff_kick_down))
        {
            ff_hold = ff_table_hold;
            kick_up = fabsf(ff_kick_up);
            kick_down = fabsf(ff_kick_down);
        }
        gimbal_motor->current_set += ff_hold;

        kick_scale = gimbal_pitch_kick_scale(pitch_err);
        kick_up *= kick_scale;
        kick_down *= kick_scale;

        gimbal_motor->current_set = axis_current_conditioner_apply(
            gimbal_motor->current_set,
            gimbal_motor->motor_gyro_set,
            kick_up,
            kick_down,
            gimbal_motor->angle,
            gimbal_motor->min_angle,
            gimbal_motor->max_angle,
            current_limit,
            &gimbal_motor->gimbal_motor_angle_pid.Iout,
            &gimbal_motor->gimbal_motor_gyro_pid.Iout,
            &limit_info);

        gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
    }
}

static void gimbal_motor_raw_angle_control(gimbal_motor_t *gimbal_motor)
{
    gimbal_core_axis_input_t core_input;
    gimbal_core_axis_output_t core_output;

    if (gimbal_motor == NULL)
    {
        return;
    }

    memset(&core_input, 0, sizeof(core_input));
    memset(&core_output, 0, sizeof(core_output));
    core_input.mode = (uint8_t)GIMBAL_CORE_MODE_RAW;
    core_input.gyro_set_radps = gimbal_motor->motor_gyro_set;
    core_input.raw_current = gimbal_motor->raw_cmd_current;
    gimbal_core_step_axis_base(&gimbal_motor->gimbal_motor_angle_pid,
                               &gimbal_motor->gimbal_motor_gyro_pid,
                               &core_input,
                               &core_output);
    gimbal_motor->motor_gyro_set = core_output.gyro_set_radps;
    gimbal_motor->current_set = core_output.current_set;
    gimbal_motor->given_current = core_output.given_current;
}

#if GIMBAL_TEST_MODE
int32_t yaw_angle_int_1000, pitch_angle_int_1000;
int32_t yaw_angle_set_1000, pitch_angle_set_1000;
int32_t yaw_speed_int_1000, pitch_speed_int_1000;
int32_t yaw_speed_set_int_1000, pitch_speed_set_int_1000;
static void J_scope_gimbal_test(void)
{
    yaw_angle_int_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.angle * 1000);
    yaw_angle_set_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.angle_set * 1000);
    yaw_speed_int_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.motor_gyro * 1000);
    yaw_speed_set_int_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.motor_gyro_set * 1000);

    pitch_angle_int_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.angle * 1000);
    pitch_angle_set_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.angle_set * 1000);
    pitch_speed_int_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.motor_gyro * 1000);
    pitch_speed_set_int_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.motor_gyro_set * 1000);
}

#endif
