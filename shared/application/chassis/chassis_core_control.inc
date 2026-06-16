/*
 * SPDX-FileCopyrightText: 2026 ?? <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Private implementation chunk included by chassis_control_task.c.
 */

/**
  * @brief          "chassis_move" valiable initialization, include pid initialization, remote control data point initialization, 3508 chassis motors
  *                 data point initialization, gimbal motor data point initialization, and gyro sensor angle point initialization.
  * @param[out]     chassis_move_init: "chassis_move" valiable point
  * @retval         none
  */
static void chassis_init(chassis_move_t *chassis_move_init)
{
    if (chassis_move_init == NULL)
    {
        return;
    }

    //chassis motor speed PID
    const fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};

    //chassis angle PID
    const fp32 chassis_yaw_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};

    const fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    const fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
    uint8_t i;

    chassis_move_init->chassis_mode = CHASSIS_VECTOR_RAW;
    (void)motor_instance_bind_current_outputs(chassis_motor_instance_names,
                                              CHASSIS_MOTOR_COUNT,
                                              chassis_motor_current_bindings,
                                              CHASSIS_MOTOR_COUNT);
    //get remote control point
    chassis_move_init->chassis_RC = get_remote_control_point();
    //get gyro sensor euler angle point
    chassis_move_init->chassis_INS_angle = get_INS_angle_point();
    chassis_INT_gyro_point = get_gyro_data_point();
    // Cache gimbal state once it becomes available.
    chassis_move_init->gimbal_state_valid = 0u;
    chassis_move_init->gimbal_online = 0u;
    memset(&chassis_move_init->chassis_yaw_motor, 0, sizeof(chassis_move_init->chassis_yaw_motor));
    memset(&chassis_move_init->chassis_pitch_motor, 0, sizeof(chassis_move_init->chassis_pitch_motor));

    //get chassis motor data point,  initialize motor speed PID
    //获取底盘电机数据指针，初始化PID
    for (i = 0; i < 4; i++)
    {
        chassis_move_init->motor_chassis[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
        PID_init(&chassis_move_init->motor_speed_pid[i], PID_POSITION, motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
    }
    //initialize angle PID
    //初始化角度PID
    PID_init(&chassis_move_init->chassis_angle_pid, PID_POSITION, chassis_yaw_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);

    //first order low-pass filter  replace ramp function
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, robot_profile_chassis_control_period_s(), chassis_x_order_filter);
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, robot_profile_chassis_control_period_s(), chassis_y_order_filter);

    //max and min speed
    chassis_move_init->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
    chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;

    chassis_move_init->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
    chassis_move_init->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;

    //update data
    {
        chassis_runtime_snapshot_t snapshot;
        chassis_snapshot_capture(&snapshot, chassis_move_init);
        chassis_feedback_update(chassis_move_init, &snapshot);
        chassis_write_state(chassis_move_init);
    }
}

/**
  * @brief          set chassis control mode, mainly call 'chassis_behaviour_mode_set' function
  * @param[out]     chassis_move_mode: "chassis_move" valiable point
  * @retval         none
  */
static void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }
    //in file "chassis_behaviour.c"
    chassis_behaviour_mode_set(chassis_move_mode);
}

/**
  * @brief          when chassis mode change, some param should be changed, suan as chassis yaw_set should be now chassis yaw
  * @param[out]     chassis_move_transit: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          底盘模式改变，有些参数需要改变，例如底盘控制yaw角度设定值应该变成当前底盘yaw角度
  * @param[out]     chassis_move_transit:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit)
{
    if (chassis_move_transit == NULL)
    {
        return;
    }

    if (chassis_move_transit->last_chassis_mode == chassis_move_transit->chassis_mode)
    {
        return;
    }

    // 当从安全/原始模式（例如零力）切回使能模式时，清理 PID 积分和指令，防止 I 项遗留导致复位后猛冲
    if (chassis_move_transit->last_chassis_mode == CHASSIS_VECTOR_RAW &&
        chassis_move_transit->chassis_mode != CHASSIS_VECTOR_RAW)
    {
        for (uint8_t i = 0; i < CHASSIS_MOTOR_COUNT; i++)
        {
            PID_clear(&chassis_move_transit->motor_speed_pid[i]);
            chassis_move_transit->motor_chassis[i].speed_set = 0.0f;
            chassis_move_transit->motor_chassis[i].give_current = 0;
        }
        chassis_move_transit->vx_set = 0.0f;
        chassis_move_transit->vy_set = 0.0f;
        chassis_move_transit->wz_set = 0.0f;
        chassis_move_transit->chassis_cmd_slow_set_vx.out = 0.0f;
        chassis_move_transit->chassis_cmd_slow_set_vy.out = 0.0f;
    }

    //change to follow gimbal angle mode
    //切入跟随云台模式
    if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
        chassis_move_transit->chassis_yaw_offset_set = 0.0f;
    }
    //change to follow chassis yaw angle
    //切入跟随底盘角度模式
    else if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW)
    {
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
    }
    //change to no follow angle
    else if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_NO_FOLLOW_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
    {
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
    }

    chassis_move_transit->last_chassis_mode = chassis_move_transit->chassis_mode;
}

/**
  * @param[out]     chassis_move_update: "chassis_move" valiable point
  * @retval         none
  */
static void chassis_feedback_update(chassis_move_t *chassis_move_update, const chassis_runtime_snapshot_t *snapshot)
{
    if (chassis_move_update == NULL)
    {
        return;
    }

    uint8_t i = 0;
    for (i = 0; i < 4; i++)
    {
        //update motor speed, accel is differential of speed PID
        //更新电机速度，加速度是速度的PID微分
        const motor_measure_t *measure = (snapshot != NULL) ? snapshot->motor_measure[i] : chassis_move_update->motor_chassis[i].chassis_motor_measure;
        const int8_t motor_dir = (snapshot != NULL) ? snapshot->motor_dir[i] : g_config.chassis.motor_dir[i];
        const fp32 rpm_to_vector = (snapshot != NULL) ? snapshot->motor_rpm_to_vector : CHASSIS_MOTOR_RPM_TO_VECTOR_SEN;
        const fp32 control_hz = (snapshot != NULL) ? snapshot->control_hz : CHASSIS_CONTROL_FREQUENCE;
        const int16_t speed_rpm = (measure != NULL) ? measure->speed_rpm : 0;
        chassis_move_update->motor_chassis[i].speed = rpm_to_vector * speed_rpm * (fp32)motor_dir;
        chassis_move_update->motor_chassis[i].accel = chassis_move_update->motor_speed_pid[i].Dbuf[0] * control_hz;
    }

    //calculate chassis speeds (robot frame, right-hand):
    // - vx: forward is positive
    // - vy: left is positive
    // - wz: CCW is positive
    // NOTE: keep consistent with chassis_core_vector_to_wheel_speed() and g_config.chassis.wheel_type.
    const fp32 w0 = chassis_move_update->motor_chassis[0].speed;
    const fp32 w1 = chassis_move_update->motor_chassis[1].speed;
    const fp32 w2 = chassis_move_update->motor_chassis[2].speed;
    const fp32 w3 = chassis_move_update->motor_chassis[3].speed;

    fp32 wz_wheel = 0.0f;
    const uint8_t wheel_type = (snapshot != NULL) ? snapshot->wheel_type : g_config.chassis.wheel_type;
    const fp32 motor_speed_to_vx = (snapshot != NULL) ? snapshot->motor_speed_to_vx : MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
    const fp32 motor_speed_to_vy = (snapshot != NULL) ? snapshot->motor_speed_to_vy : MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
    const fp32 motor_speed_to_wz = (snapshot != NULL) ? snapshot->motor_speed_to_wz : MOTOR_SPEED_TO_CHASSIS_SPEED_WZ;
    const fp32 motor_distance_to_center = (snapshot != NULL) ? snapshot->motor_distance_to_center : MOTOR_DISTANCE_TO_CENTER;
    if (wheel_type == (uint8_t)CHASSIS_WHEEL_TYPE_XDRIVE)
    {
        // X-drive (45° omni) inverse kinematics:
        // w0 = +vx +vy +yaw, w1 = -vx +vy +yaw, w2 = -vx -vy +yaw, w3 = +vx -vy +yaw
        chassis_move_update->vx = (w0 - w1 - w2 + w3) * motor_speed_to_vx;
        chassis_move_update->vy = (w0 + w1 - w2 - w3) * motor_speed_to_vy;
        wz_wheel = (w0 + w1 + w2 + w3) * motor_speed_to_wz / motor_distance_to_center;
    }
    else
    {
        // Mecanum inverse kinematics (DJI legacy mapping):
        // w0 = +vx -vy -yaw, w1 = +vx +vy +yaw, w2 = +vx +vy -yaw, w3 = +vx -vy +yaw
        chassis_move_update->vx = (w0 + w1 + w2 + w3) * motor_speed_to_vx;
        chassis_move_update->vy = (-w0 + w1 + w2 - w3) * motor_speed_to_vy;
        wz_wheel = (-w0 + w1 - w2 + w3) * motor_speed_to_wz / motor_distance_to_center;
    }
    chassis_move_update->wz = wz_wheel;

    // Fuse yaw-rate using IMU (on gimbal) if available.
    // - 底盘单跑时只用轮速里程计，避免依赖 IMU/云台信号。
    bool_t imu_valid = 0;
    fp32 wz_imu = 0.0f;
    const uint8_t chassis_only_mode = (snapshot != NULL) ? snapshot->chassis_only_mode : robot_mode_force_chassis_only();
    const fp32 *gyro = (snapshot != NULL) ? snapshot->gyro : chassis_INT_gyro_point;
    const gimbal_motor_state_t *yaw_motor = NULL;
    const gimbal_motor_state_t *pitch_motor = NULL;
    uint8_t gimbal_online = 0u;
    if (snapshot != NULL && snapshot->gimbal_state_valid != 0u)
    {
        yaw_motor = &snapshot->yaw_motor;
        pitch_motor = &snapshot->pitch_motor;
        gimbal_online = snapshot->gimbal_online;
    }
    else if (chassis_move_update->gimbal_state_valid != 0u)
    {
        yaw_motor = &chassis_move_update->chassis_yaw_motor;
        pitch_motor = &chassis_move_update->chassis_pitch_motor;
        gimbal_online = chassis_move_update->gimbal_online;
    }
    const fp32 *ins_angle = (snapshot != NULL) ? snapshot->ins_angle : chassis_move_update->chassis_INS_angle;
    if (CHASSIS_USE_IMU_YAW_FEEDBACK != 0u &&
        chassis_only_mode == 0u &&
        gimbal_online != 0u &&
        gyro != NULL &&
        yaw_motor != NULL)
    {
        const fp32 gyro_z = gyro[INS_GYRO_Z_ADDRESS_OFFSET];
        const fp32 gimbal_wz = chassis_get_gimbal_yaw_relative_rate(yaw_motor);
        wz_imu = gyro_z - gimbal_wz;
        imu_valid = 1;
    }
    chassis_move_update->wz = chassis_wz_kf_step(wz_wheel, imu_valid, wz_imu);

    //calculate chassis euler angle, if chassis add a new gyro sensor,please change this code
    if (chassis_only_mode != 0u)
    {
        chassis_move_update->chassis_yaw = 0.0f;
        chassis_move_update->chassis_pitch = 0.0f;
        chassis_move_update->chassis_roll = 0.0f;
    }
    else
    {
        const fp32 yaw_relative = chassis_get_gimbal_yaw_relative_angle(yaw_motor);
        const fp32 yaw = (ins_angle != NULL) ? ins_angle[INS_YAW_ADDRESS_OFFSET] : 0.0f;
        const fp32 pitch = (ins_angle != NULL) ? ins_angle[INS_PITCH_ADDRESS_OFFSET] : 0.0f;
        const fp32 roll = (ins_angle != NULL) ? ins_angle[INS_ROLL_ADDRESS_OFFSET] : 0.0f;
        const fp32 gimbal_pitch = (pitch_motor != NULL && pitch_motor->valid != 0u) ? pitch_motor->angle : 0.0f;
        chassis_move_update->chassis_yaw = rad_format(yaw - yaw_relative);
        chassis_move_update->chassis_pitch = rad_format(pitch - gimbal_pitch);
        chassis_move_update->chassis_roll = roll;
    }
}

static fp32 chassis_follow_yaw_pd_calc(pid_type_def *pid, fp32 yaw_error, fp32 chassis_wz)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    // Reuse pid_type_def fields for telemetry/debug:
    // - P/I acts on yaw_error (rad)
    // - D term is damping on chassis_wz (rad/s), i.e. derivative on measurement
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->error[0] = yaw_error;
    pid->set = yaw_error;
    pid->fdb = 0.0f;

    pid->Pout = pid->Kp * yaw_error;
    pid->Iout += pid->Ki * yaw_error;
    pid->Iout = fp32_constrain(pid->Iout, -pid->max_iout, pid->max_iout);
    pid->Dout = -pid->Kd * chassis_wz;

    pid->out = pid->Pout + pid->Iout + pid->Dout;
    pid->out = fp32_constrain(pid->out, -pid->max_out, pid->max_out);
    return pid->out;
}
/**
  * @brief          accroding to the channel value of remote control, calculate chassis vertical and horizontal speed set-point
  *
  * @param[out]     vx_set: vertical speed set-point
  * @param[out]     vy_set: horizontal speed set-point
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          根据遥控器通道值，计算纵向和横移速度
  *
  * @param[out]     vx_set: 纵向速度指针
  * @param[out]     vy_set: 横向速度指针
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" 变量指针
  * @retval         none
  */
void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL)
    {
        return;
    }

    int16_t vx_channel, vy_channel;
    fp32 vx_set_channel, vy_set_channel;
    const chassis_control_snapshot_t *fast = &chassis_move_rc_to_vector->fast;
    const uint16_t key_mask = fast->key_mask;
    //deadline, because some remote control need be calibrated,  the value of rocker is not zero in middle place,
    rc_deadband_limit(fast->axis_x, vx_channel, fast->rc_deadband);
    rc_deadband_limit(fast->axis_y, vy_channel, fast->rc_deadband);

    vx_set_channel = vx_channel * fast->vx_rc_sen;
    // vy: positive means left, negative means right.
    vy_set_channel = vy_channel * -fast->vy_rc_sen;

    //keyboard set speed set-point
    //键盘控制
    if (key_mask & fast->front_key_mask)
    {
        vx_set_channel = chassis_move_rc_to_vector->vx_max_speed;
    }
    else if (key_mask & fast->back_key_mask)
    {
        vx_set_channel = chassis_move_rc_to_vector->vx_min_speed;
    }

    if (key_mask & fast->left_key_mask)
    {
        vy_set_channel = chassis_move_rc_to_vector->vy_max_speed;
    }
    else if (key_mask & fast->right_key_mask)
    {
        vy_set_channel = chassis_move_rc_to_vector->vy_min_speed;
    }

    //first order low-pass replace ramp function, calculate chassis speed set-point to improve control performance
    //一阶低通滤波代替斜波作为底盘速度输入
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, vy_set_channel);
    //stop command, need not slow change, set zero derectly
    const fp32 vx_stop_threshold = fabsf((fp32)fast->rc_deadband * fast->vx_rc_sen);
    const fp32 vy_stop_threshold = fabsf((fp32)fast->rc_deadband * fast->vy_rc_sen);
    if (vx_set_channel < vx_stop_threshold && vx_set_channel > -vx_stop_threshold)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
    }

    if (vy_set_channel < vy_stop_threshold && vy_set_channel > -vy_stop_threshold)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out = 0.0f;
    }

    *vx_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
    *vy_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out;
}
/**
  * @brief          set chassis control set-point, three movement control value is set by "chassis_behaviour_control_set".
  * @param[out]     chassis_move_update: "chassis_move" valiable point
  * @retval         none
  */
static void chassis_set_contorl(chassis_move_t *chassis_move_control)
{

    if (chassis_move_control == NULL)
    {
        return;
    }

    fp32 vx_set = 0.0f, vy_set = 0.0f, angle_set = 0.0f;
    chassis_behaviour_control_set(&vx_set, &vy_set, &angle_set, chassis_move_control);

    //follow gimbal mode
    if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
        const fp32 yaw_relative_meas = chassis_get_gimbal_yaw_relative_angle(&chassis_move_control->chassis_yaw_motor);
        fp32 yaw_frame = yaw_relative_meas;
        fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
        (void)chassis_get_turnaround_frame_yaw(&yaw_frame);
        //rotate chassis direction, make sure vertial direction follow gimbal
        //旋转控制底盘速度方向，保证前进方向是云台方向，有利于运动平稳
        sin_yaw = arm_sin_f32(-yaw_frame);
        cos_yaw = arm_cos_f32(-yaw_frame);
        chassis_move_control->vx_set = cos_yaw * vx_set + sin_yaw * vy_set;
        chassis_move_control->vy_set = -sin_yaw * vx_set + cos_yaw * vy_set;
        //set control gimbal-chassis angle set-point
        //设置控制云台-底盘角度
        chassis_move_control->chassis_yaw_offset_set = rad_format(angle_set);
        // Follow gimbal yaw:
        // - yaw_relative is the gimbal-chassis relative angle (rad)
        // - yaw_error keeps the same sign as wz_set, so the chassis rotates to reduce yaw_error
        const fp32 yaw_error = rad_format(yaw_relative_meas - chassis_move_control->chassis_yaw_offset_set);
        if (fabsf(yaw_error) < CHASSIS_FOLLOW_YAW_STOP_ERR_RAD && fabsf(chassis_move_control->wz) < CHASSIS_FOLLOW_YAW_STOP_WZ_RADPS)
        {
            PID_clear(&chassis_move_control->chassis_angle_pid);
            chassis_move_control->wz_set = 0.0f;
        }
        else
        {
            chassis_move_control->wz_set = chassis_follow_yaw_pd_calc(&chassis_move_control->chassis_angle_pid, yaw_error, chassis_move_control->wz);
        }
        //speed limit
        //速度限幅
        chassis_move_control->vx_set = fp32_constrain(chassis_move_control->vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(chassis_move_control->vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW)
    {
        //set chassis yaw angle set-point
        chassis_move_control->chassis_yaw_set = rad_format(angle_set);
        const fp32 yaw_error = rad_format(chassis_move_control->chassis_yaw_set - chassis_move_control->chassis_yaw);
        if (fabsf(yaw_error) < CHASSIS_FOLLOW_YAW_STOP_ERR_RAD && fabsf(chassis_move_control->wz) < CHASSIS_FOLLOW_YAW_STOP_WZ_RADPS)
        {
            PID_clear(&chassis_move_control->chassis_angle_pid);
            chassis_move_control->wz_set = 0.0f;
        }
        else
        {
            chassis_move_control->wz_set = chassis_follow_yaw_pd_calc(&chassis_move_control->chassis_angle_pid, yaw_error, chassis_move_control->wz);
        }
        //speed limit
        //速度限幅
        chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
    {
        //"angle_set" is rotation speed set-point
        chassis_move_control->wz_set = angle_set;
        chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_RAW)
    {
        //in raw mode, set-point is sent to CAN bus
        //在原始模式，设置值是发送到CAN总线
        chassis_move_control->vx_set = vx_set;
        chassis_move_control->vy_set = vy_set;
        chassis_move_control->wz_set = angle_set;
        chassis_move_control->chassis_cmd_slow_set_vx.out = 0.0f;
        chassis_move_control->chassis_cmd_slow_set_vy.out = 0.0f;
    }
}

/**
  * @brief          control loop, according to control set-point, calculate motor current,
  *                 motor current will be sentto motor
  * @param[out]     chassis_move_control_loop: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
  * @param[out]     chassis_move_control_loop:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop,
                                 const chassis_runtime_snapshot_t *snapshot,
                                 int16_t pre_power_current[CHASSIS_MOTOR_COUNT])
{
    chassis_core_kinematic_config_t core_cfg;
    chassis_core_cmd_t core_cmd;
    chassis_core_output_t core_output;
    uint8_t i = 0;

    chassis_power_control_apply_speed_limit(chassis_move_control_loop);

    //mecanum wheel speed calculation
    //麦轮运动分解
    (void)memset(&core_cfg, 0, sizeof(core_cfg));
    (void)memset(&core_cmd, 0, sizeof(core_cmd));
    (void)memset(&core_output, 0, sizeof(core_output));
    core_cfg.wheel_type = (snapshot != NULL) ? snapshot->wheel_type : g_config.chassis.wheel_type;
    core_cfg.motor_distance_to_center = (snapshot != NULL) ? snapshot->motor_distance_to_center : MOTOR_DISTANCE_TO_CENTER;
    core_cfg.max_wheel_speed = (snapshot != NULL) ? snapshot->max_wheel_speed : MAX_WHEEL_SPEED;
    core_cmd.enabled = 1u;
    core_cmd.raw_output = (chassis_move_control_loop->chassis_mode == CHASSIS_VECTOR_RAW) ? 1u : 0u;
    core_cmd.vx_mps = chassis_move_control_loop->vx_set;
    core_cmd.vy_mps = chassis_move_control_loop->vy_set;
    core_cmd.wz_radps = chassis_move_control_loop->wz_set;
    chassis_core_step_velocity(&core_cfg, &core_cmd, &core_output);

    if (chassis_move_control_loop->chassis_mode == CHASSIS_VECTOR_RAW)
    {

        for (i = 0; i < 4; i++)
        {
            const int8_t motor_dir = (snapshot != NULL) ? snapshot->motor_dir[i] : g_config.chassis.motor_dir[i];
            chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(core_output.wheel_speed_set[i] * (fp32)motor_dir);
            if (pre_power_current != NULL)
            {
                pre_power_current[i] = chassis_move_control_loop->motor_chassis[i].give_current;
            }
        }
        //in raw mode, derectly return
        //raw控制直接返回
        return;
    }

    //calculate the max speed in four wheels, limit the max speed
    //计算轮子控制最大速度，并限制其最大速度
    for (i = 0; i < 4; i++)
    {
        chassis_move_control_loop->motor_chassis[i].speed_set = core_output.wheel_speed_set[i];
    }

    //calculate pid
    //计算pid
    for (i = 0; i < 4; i++)
    {
        PID_calc(&chassis_move_control_loop->motor_speed_pid[i], chassis_move_control_loop->motor_chassis[i].speed, chassis_move_control_loop->motor_chassis[i].speed_set);
        if (pre_power_current != NULL)
        {
            pre_power_current[i] = chassis_sdlog_clamp_current(chassis_move_control_loop->motor_speed_pid[i].out *
                                                               (fp32)((snapshot != NULL) ? snapshot->motor_dir[i] : g_config.chassis.motor_dir[i]));
        }
    }

    //功率控制
    chassis_power_control(chassis_move_control_loop);

    for (i = 0; i < 4; i++)
    {
        const int8_t motor_dir = (snapshot != NULL) ? snapshot->motor_dir[i] : g_config.chassis.motor_dir[i];
        chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(chassis_move_control_loop->motor_speed_pid[i].out * (fp32)motor_dir);
    }
}
