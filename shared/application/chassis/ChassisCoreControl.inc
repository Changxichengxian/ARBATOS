/*
 * SPDX-FileCopyrightText: 2026 ?? <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Private implementation chunk included by ChassisControlTask.c.
 */

/**
  * @brief          "ChassisMove" variable initialization, include pid initialization, remote control data point initialization, 3508 chassis motors
  *                 data point initialization, gimbal motor data point initialization, and gyro sensor angle point initialization.
  * @param[out]     ChassisMoveInit: "ChassisMove" variable point
  * @retval         none
  */
static void ChassisInit(ChassisMove *ChassisMoveInit)
{
    if (ChassisMoveInit == NULL)
    {
        return;
    }

    //chassis motor speed PID
    const fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};

    //chassis angle PID
    const fp32 ChassisYawPid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};

    const fp32 ChassisXOrderFilter[1] = {CHASSIS_ACCEL_X_NUM};
    const fp32 ChassisYOrderFilter[1] = {CHASSIS_ACCEL_Y_NUM};
    uint8_t i;

    ChassisMoveInit->mode = CHASSIS_VECTOR_RAW;
    (void)MotorInstBindCurrent(ChassisMotorInstNames,
                                              CHASSIS_MOTOR_COUNT,
                                              ChassisMotorCurrentBindings,
                                              CHASSIS_MOTOR_COUNT);
    //get gyro sensor euler angle point
    ChassisMoveInit->ChassisInsAngle = get_INS_angle_point();
    ChassisINTGyroPoint = get_gyro_data_point();
    //get chassis motor data point,  initialize motor speed PID
    //获取底盘电机数据指针，初始化PID
    for (i = 0; i < 4; i++)
    {
        (void)memset(&ChassisMoveInit->motor_chassis[i].measure,
                     0,
                     sizeof(ChassisMoveInit->motor_chassis[i].measure));
        ChassisMoveInit->motor_chassis[i].measureValid = 0u;
        PID_init(&ChassisMoveInit->motor_speed_pid[i], PID_POSITION, motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
    }
    //initialize angle PID
    //初始化角度PID
    PID_init(&ChassisMoveInit->ChassisAnglePid, PID_POSITION, ChassisYawPid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);

    //first order low-pass filter  replace ramp function
    first_order_filter_init(&ChassisMoveInit->ChassisCmdSlowSetVx, RobotProfileChassisControlPeriodS(), ChassisXOrderFilter);
    first_order_filter_init(&ChassisMoveInit->ChassisCmdSlowSetVy, RobotProfileChassisControlPeriodS(), ChassisYOrderFilter);

    //max and min speed
    ChassisMoveInit->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
    ChassisMoveInit->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;

    ChassisMoveInit->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
    ChassisMoveInit->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;

    /* 首次反馈只在 enter 后同帧 update 抓取，初始化阶段不重复读共享状态。 */
    ChassisWriteState(ChassisMoveInit);
}

/**
  * @brief          set chassis control mode, mainly call 'ChassisBehaviourModeSet' function
  * @param[out]     ChassisMoveMode: "ChassisMove" variable point
  * @retval         none
  */
static void ChassisSetMode(ChassisMove *ChassisMoveMode)
{
    if (ChassisMoveMode == NULL)
    {
        return;
    }
    //in file "ChassisBehaviour.c"
    ChassisBehaviourModeSet(ChassisMoveMode);
}

/**
  * @brief          when chassis mode change, some param should be changed, such as chassis yaw_set should be now chassis yaw
  * @param[out]     ChassisMoveTransit: "ChassisMove" variable point
  * @retval         none
  */
/**
  * @brief          底盘模式改变，有些参数需要改变，例如底盘控制yaw角度设定值应该变成当前底盘yaw角度
  * @param[out]     ChassisMoveTransit:"ChassisMove"变量指针.
  * @retval         none
  */
static void ChassisModeChangeControlTransit(ChassisMove *ChassisMoveTransit)
{
    if (ChassisMoveTransit == NULL)
    {
        return;
    }

    if (ChassisMoveTransit->last_mode == ChassisMoveTransit->mode)
    {
        return;
    }

    // 当从安全/原始模式（例如零力）切回使能模式时，清理 PID 积分和指令，防止 I 项遗留导致复位后猛冲
    if (ChassisMoveTransit->last_mode == CHASSIS_VECTOR_RAW &&
        ChassisMoveTransit->mode != CHASSIS_VECTOR_RAW)
    {
        for (uint8_t i = 0; i < CHASSIS_MOTOR_COUNT; i++)
        {
            PID_clear(&ChassisMoveTransit->motor_speed_pid[i]);
            ChassisMoveTransit->motor_chassis[i].speed_set = 0.0f;
            ChassisMoveTransit->motor_chassis[i].give_current = 0;
        }
        ChassisMoveTransit->vx_set = 0.0f;
        ChassisMoveTransit->vy_set = 0.0f;
        ChassisMoveTransit->wz_set = 0.0f;
        ChassisMoveTransit->ChassisCmdSlowSetVx.out = 0.0f;
        ChassisMoveTransit->ChassisCmdSlowSetVy.out = 0.0f;
    }

    //change to follow gimbal angle mode
    //切入跟随云台模式
    if ((ChassisMoveTransit->last_mode != CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW) && ChassisMoveTransit->mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
        ChassisMoveTransit->ChassisYawOffsetSet = 0.0f;
    }
    //change to follow chassis yaw angle
    //切入跟随底盘角度模式
    else if ((ChassisMoveTransit->last_mode != CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW) && ChassisMoveTransit->mode == CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW)
    {
        ChassisMoveTransit->ChassisYawSet = ChassisMoveTransit->ChassisYaw;
    }
    //change to no follow angle
    else if ((ChassisMoveTransit->last_mode != CHASSIS_VECTOR_NO_FOLLOW_YAW) && ChassisMoveTransit->mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
    {
        ChassisMoveTransit->ChassisYawSet = ChassisMoveTransit->ChassisYaw;
    }

    ChassisMoveTransit->last_mode = ChassisMoveTransit->mode;
}

/**
  * @param[out]     ChassisMoveUpdate: "ChassisMove" variable point
  * @retval         none
  */
static void ChassisFeedbackUpdate(ChassisMove *ChassisMoveUpdate, const ChassisRuntimeSnapshot *snapshot)
{
    if (ChassisMoveUpdate == NULL)
    {
        return;
    }

    uint8_t i = 0;
    for (i = 0; i < 4; i++)
    {
        //update motor speed, accel is differential of speed PID
        //更新电机速度，加速度是速度的PID微分
        ChassisMotor *motor = &ChassisMoveUpdate->motor_chassis[i];
        if (snapshot != NULL)
        {
            motor->measure = snapshot->motor_measure[i];
            motor->measureValid = snapshot->motor_health[i].feedbackValid;
        }
        else
        {
            (void)memset(&motor->measure, 0, sizeof(motor->measure));
            motor->measureValid = 0u;
        }
        const int8_t motor_dir = (snapshot != NULL) ? snapshot->motor_dir[i] : g_config.chassis.motor_dir[i];
        const fp32 rpm_to_vector = (snapshot != NULL) ? snapshot->motor_rpm_to_vector : CHASSIS_MOTOR_RPM_TO_VECTOR_SEN;
        const fp32 control_hz = (snapshot != NULL) ? snapshot->control_hz : CHASSIS_CONTROL_FREQUENCE;
        const int16_t speed_rpm = (motor->measureValid != 0u &&
                                   snapshot != NULL &&
                                   snapshot->motor_health[i].healthy != 0u) ?
                                      motor->measure.speed_rpm :
                                      0;
        ChassisMoveUpdate->motor_chassis[i].speed = rpm_to_vector * speed_rpm * (fp32)motor_dir;
        ChassisMoveUpdate->motor_chassis[i].accel = ChassisMoveUpdate->motor_speed_pid[i].Dbuf[0] * control_hz;
    }

    //calculate chassis speeds (robot frame, right-hand):
    // - vx: forward is positive
    // - vy: left is positive
    // - wz: CCW is positive
    // NOTE: keep consistent with ChassisCoreVectorToWheelSpeed() and g_config.chassis.wheel_type.
    const fp32 w0 = ChassisMoveUpdate->motor_chassis[0].speed;
    const fp32 w1 = ChassisMoveUpdate->motor_chassis[1].speed;
    const fp32 w2 = ChassisMoveUpdate->motor_chassis[2].speed;
    const fp32 w3 = ChassisMoveUpdate->motor_chassis[3].speed;

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
        ChassisMoveUpdate->vx = (w0 - w1 - w2 + w3) * motor_speed_to_vx;
        ChassisMoveUpdate->vy = (w0 + w1 - w2 - w3) * motor_speed_to_vy;
        wz_wheel = (w0 + w1 + w2 + w3) * motor_speed_to_wz / motor_distance_to_center;
    }
    else
    {
        // Mecanum inverse kinematics (DJI legacy mapping):
        // w0 = +vx -vy -yaw, w1 = +vx +vy +yaw, w2 = +vx +vy -yaw, w3 = +vx -vy +yaw
        ChassisMoveUpdate->vx = (w0 + w1 + w2 + w3) * motor_speed_to_vx;
        ChassisMoveUpdate->vy = (-w0 + w1 + w2 - w3) * motor_speed_to_vy;
        wz_wheel = (-w0 + w1 - w2 + w3) * motor_speed_to_wz / motor_distance_to_center;
    }
    ChassisMoveUpdate->wz = wz_wheel;

    // Fuse yaw-rate using IMU (on gimbal) if available.
    // - 底盘单跑时只用轮速里程计，避免依赖 IMU/云台信号。
    bool_t imu_valid = 0;
    fp32 wz_imu = 0.0f;
    const uint8_t ChassisOnlyMode = (snapshot != NULL) ? snapshot->ChassisOnlyMode : robot_mode_force_chassis_only();
    const fp32 *gyro = (snapshot != NULL) ? snapshot->gyro : ChassisINTGyroPoint;
    const GimbalMotorState *yaw_motor = NULL;
    const GimbalMotorState *pitch_motor = NULL;
    uint8_t GimbalOnline = 0u;
    if (snapshot != NULL && snapshot->gimbal.valid != 0u)
    {
        yaw_motor = &snapshot->gimbal.yaw;
        pitch_motor = &snapshot->gimbal.pitch;
        GimbalOnline = snapshot->gimbal.online;
    }
    const fp32 *ins_angle = (snapshot != NULL) ? snapshot->ins_angle : ChassisMoveUpdate->ChassisInsAngle;
    if (CHASSIS_USE_IMU_YAW_FEEDBACK != 0u &&
        ChassisOnlyMode == 0u &&
        GimbalOnline != 0u &&
        gyro != NULL &&
        yaw_motor != NULL)
    {
        const fp32 gyro_z = gyro[INS_GYRO_Z_ADDRESS_OFFSET];
        const fp32 GimbalWz = ChassisGetGimbalYawRelativeRate(yaw_motor);
        wz_imu = gyro_z - GimbalWz;
        imu_valid = 1;
    }
    ChassisMoveUpdate->wz = ChassisWzKfStep(wz_wheel, imu_valid, wz_imu);

    //calculate chassis euler angle, if chassis add a new gyro sensor,please change this code
    if (ChassisOnlyMode != 0u)
    {
        ChassisMoveUpdate->ChassisYaw = 0.0f;
        ChassisMoveUpdate->ChassisPitch = 0.0f;
        ChassisMoveUpdate->ChassisRoll = 0.0f;
    }
    else
    {
        const fp32 yaw_relative = ChassisGetGimbalYawRelativeAngle(yaw_motor);
        const fp32 yaw = (ins_angle != NULL) ? ins_angle[INS_YAW_ADDRESS_OFFSET] : 0.0f;
        const fp32 pitch = (ins_angle != NULL) ? ins_angle[INS_PITCH_ADDRESS_OFFSET] : 0.0f;
        const fp32 roll = (ins_angle != NULL) ? ins_angle[INS_ROLL_ADDRESS_OFFSET] : 0.0f;
        const fp32 GimbalPitch = (pitch_motor != NULL && pitch_motor->valid != 0u) ? pitch_motor->angle : 0.0f;
        ChassisMoveUpdate->ChassisYaw = rad_format(yaw - yaw_relative);
        ChassisMoveUpdate->ChassisPitch = rad_format(pitch - GimbalPitch);
        ChassisMoveUpdate->ChassisRoll = roll;
    }
}

static fp32 ChassisFollowYawPdCalc(PidTypeDef *pid, fp32 yaw_error, fp32 ChassisWz)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    // Reuse PidTypeDef fields for telemetry/debug:
    // - P/I acts on yaw_error (rad)
    // - D term is damping on ChassisWz (rad/s), i.e. derivative on measurement
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->error[0] = yaw_error;
    pid->set = yaw_error;
    pid->fdb = 0.0f;

    pid->Pout = pid->Kp * yaw_error;
    pid->Iout += pid->Ki * yaw_error;
    pid->Iout = fp32_constrain(pid->Iout, -pid->max_iout, pid->max_iout);
    pid->Dout = -pid->Kd * ChassisWz;

    pid->out = pid->Pout + pid->Iout + pid->Dout;
    pid->out = fp32_constrain(pid->out, -pid->max_out, pid->max_out);
    return pid->out;
}
/**
  * @brief          accroding to the channel value of remote control, calculate chassis vertical and horizontal speed set-point
  *
  * @param[out]     vx_set: vertical speed set-point
  * @param[out]     vy_set: horizontal speed set-point
  * @param[out]     ChassisMoveRcToVector: "ChassisMove" variable point
  * @retval         none
  */
/**
  * @brief          根据遥控器通道值，计算纵向和横移速度
  *
  * @param[out]     vx_set: 纵向速度指针
  * @param[out]     vy_set: 横向速度指针
  * @param[out]     ChassisMoveRcToVector: "ChassisMove" 变量指针
  * @retval         none
  */
void ChassisRcToControlVector(fp32 *vx_set, fp32 *vy_set, ChassisMove *ChassisMoveRcToVector)
{
    if (ChassisMoveRcToVector == NULL || vx_set == NULL || vy_set == NULL)
    {
        return;
    }

    int16_t vx_channel, vy_channel;
    fp32 vx_set_channel, vy_set_channel;
    const ChassisControlSnapshot *fast = &ChassisMoveRcToVector->fast;
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
        vx_set_channel = ChassisMoveRcToVector->vx_max_speed;
    }
    else if (key_mask & fast->back_key_mask)
    {
        vx_set_channel = ChassisMoveRcToVector->vx_min_speed;
    }

    if (key_mask & fast->left_key_mask)
    {
        vy_set_channel = ChassisMoveRcToVector->vy_max_speed;
    }
    else if (key_mask & fast->right_key_mask)
    {
        vy_set_channel = ChassisMoveRcToVector->vy_min_speed;
    }

    //first order low-pass replace ramp function, calculate chassis speed set-point to improve control performance
    //一阶低通滤波代替斜波作为底盘速度输入
    first_order_filter_cali(&ChassisMoveRcToVector->ChassisCmdSlowSetVx, vx_set_channel);
    first_order_filter_cali(&ChassisMoveRcToVector->ChassisCmdSlowSetVy, vy_set_channel);
    //stop command, need not slow change, set zero derectly
    const fp32 vx_stop_threshold = fabsf((fp32)fast->rc_deadband * fast->vx_rc_sen);
    const fp32 vy_stop_threshold = fabsf((fp32)fast->rc_deadband * fast->vy_rc_sen);
    if (vx_set_channel < vx_stop_threshold && vx_set_channel > -vx_stop_threshold)
    {
        ChassisMoveRcToVector->ChassisCmdSlowSetVx.out = 0.0f;
    }

    if (vy_set_channel < vy_stop_threshold && vy_set_channel > -vy_stop_threshold)
    {
        ChassisMoveRcToVector->ChassisCmdSlowSetVy.out = 0.0f;
    }

    *vx_set = ChassisMoveRcToVector->ChassisCmdSlowSetVx.out;
    *vy_set = ChassisMoveRcToVector->ChassisCmdSlowSetVy.out;
}
/**
  * @brief          set chassis control set-point, three movement control value is set by "ChassisBehaviourControlSet".
  * @param[out]     ChassisMoveUpdate: "ChassisMove" variable point
  * @retval         none
  */
static void ChassisSetControl(ChassisMove *ChassisMoveControl,
                              const ChassisRuntimeSnapshot *snapshot)
{

    if (ChassisMoveControl == NULL)
    {
        return;
    }

    fp32 vx_set = 0.0f, vy_set = 0.0f, angle_set = 0.0f;
    ChassisBehaviourControlSet(&vx_set, &vy_set, &angle_set, ChassisMoveControl);

    //follow gimbal mode
    if (ChassisMoveControl->mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
        const GimbalMotorState *yawMotor = (snapshot != NULL) ? &snapshot->gimbal.yaw : NULL;
        const fp32 yaw_relative_meas = ChassisGetGimbalYawRelativeAngle(yawMotor);
        fp32 yaw_frame = yaw_relative_meas;
        fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
        (void)ChassisGetTurnaroundFrameYaw(snapshot, &yaw_frame);
        //rotate chassis direction, make sure vertial direction follow gimbal
        //旋转控制底盘速度方向，保证前进方向是云台方向，有利于运动平稳
        sin_yaw = arm_sin_f32(-yaw_frame);
        cos_yaw = arm_cos_f32(-yaw_frame);
        ChassisMoveControl->vx_set = cos_yaw * vx_set + sin_yaw * vy_set;
        ChassisMoveControl->vy_set = -sin_yaw * vx_set + cos_yaw * vy_set;
        //set control gimbal-chassis angle set-point
        //设置控制云台-底盘角度
        ChassisMoveControl->ChassisYawOffsetSet = rad_format(angle_set);
        // Follow gimbal yaw:
        // - yaw_relative is the gimbal-chassis relative angle (rad)
        // - yaw_error keeps the same sign as wz_set, so the chassis rotates to reduce yaw_error
        const fp32 yaw_error = rad_format(yaw_relative_meas - ChassisMoveControl->ChassisYawOffsetSet);
        if (fabsf(yaw_error) < CHASSIS_FOLLOW_YAW_STOP_ERR_RAD && fabsf(ChassisMoveControl->wz) < CHASSIS_FOLLOW_YAW_STOP_WZ_RADPS)
        {
            PID_clear(&ChassisMoveControl->ChassisAnglePid);
            ChassisMoveControl->wz_set = 0.0f;
        }
        else
        {
            ChassisMoveControl->wz_set = ChassisFollowYawPdCalc(&ChassisMoveControl->ChassisAnglePid, yaw_error, ChassisMoveControl->wz);
        }
        //speed limit
        //速度限幅
        ChassisMoveControl->vx_set = fp32_constrain(ChassisMoveControl->vx_set, ChassisMoveControl->vx_min_speed, ChassisMoveControl->vx_max_speed);
        ChassisMoveControl->vy_set = fp32_constrain(ChassisMoveControl->vy_set, ChassisMoveControl->vy_min_speed, ChassisMoveControl->vy_max_speed);
    }
    else if (ChassisMoveControl->mode == CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW)
    {
        //set chassis yaw angle set-point
        ChassisMoveControl->ChassisYawSet = rad_format(angle_set);
        const fp32 yaw_error = rad_format(ChassisMoveControl->ChassisYawSet - ChassisMoveControl->ChassisYaw);
        if (fabsf(yaw_error) < CHASSIS_FOLLOW_YAW_STOP_ERR_RAD && fabsf(ChassisMoveControl->wz) < CHASSIS_FOLLOW_YAW_STOP_WZ_RADPS)
        {
            PID_clear(&ChassisMoveControl->ChassisAnglePid);
            ChassisMoveControl->wz_set = 0.0f;
        }
        else
        {
            ChassisMoveControl->wz_set = ChassisFollowYawPdCalc(&ChassisMoveControl->ChassisAnglePid, yaw_error, ChassisMoveControl->wz);
        }
        //speed limit
        //速度限幅
        ChassisMoveControl->vx_set = fp32_constrain(vx_set, ChassisMoveControl->vx_min_speed, ChassisMoveControl->vx_max_speed);
        ChassisMoveControl->vy_set = fp32_constrain(vy_set, ChassisMoveControl->vy_min_speed, ChassisMoveControl->vy_max_speed);
    }
    else if (ChassisMoveControl->mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
    {
        //"angle_set" is rotation speed set-point
        ChassisMoveControl->wz_set = angle_set;
        ChassisMoveControl->vx_set = fp32_constrain(vx_set, ChassisMoveControl->vx_min_speed, ChassisMoveControl->vx_max_speed);
        ChassisMoveControl->vy_set = fp32_constrain(vy_set, ChassisMoveControl->vy_min_speed, ChassisMoveControl->vy_max_speed);
    }
    else if (ChassisMoveControl->mode == CHASSIS_VECTOR_RAW)
    {
        //in raw mode, set-point is sent to CAN bus
        //在原始模式，设置值是发送到CAN总线
        ChassisMoveControl->vx_set = vx_set;
        ChassisMoveControl->vy_set = vy_set;
        ChassisMoveControl->wz_set = angle_set;
        ChassisMoveControl->ChassisCmdSlowSetVx.out = 0.0f;
        ChassisMoveControl->ChassisCmdSlowSetVy.out = 0.0f;
    }
}

/**
  * @brief          control loop, according to control set-point, calculate motor current,
  *                 motor current will be sent to motor
  * @param[out]     ChassisMoveControlLoop: "ChassisMove" variable point
  * @retval         none
  */
/**
  * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
  * @param[out]     ChassisMoveControlLoop:"ChassisMove"变量指针.
  * @retval         none
  */
static void ChassisControlLoop(ChassisMove *ChassisMoveControlLoop,
                                 const ChassisRuntimeSnapshot *snapshot,
                                 int16_t pre_power_current[CHASSIS_MOTOR_COUNT])
{
    ChassisCoreKinematicConfig core_cfg;
    ChassisCoreCmd core_cmd;
    ChassisCoreOutput core_output;
    uint8_t i = 0;

    ChassisPowerControlApplySpeedLimit(ChassisMoveControlLoop);

    //mecanum wheel speed calculation
    //麦轮运动分解
    (void)memset(&core_cfg, 0, sizeof(core_cfg));
    (void)memset(&core_cmd, 0, sizeof(core_cmd));
    (void)memset(&core_output, 0, sizeof(core_output));
    core_cfg.wheel_type = (snapshot != NULL) ? snapshot->wheel_type : g_config.chassis.wheel_type;
    core_cfg.motor_distance_to_center = (snapshot != NULL) ? snapshot->motor_distance_to_center : MOTOR_DISTANCE_TO_CENTER;
    core_cfg.max_wheel_speed = (snapshot != NULL) ? snapshot->max_wheel_speed : MAX_WHEEL_SPEED;
    core_cmd.enabled = 1u;
    core_cmd.raw_output = (ChassisMoveControlLoop->mode == CHASSIS_VECTOR_RAW) ? 1u : 0u;
    core_cmd.vx_mps = ChassisMoveControlLoop->vx_set;
    core_cmd.vy_mps = ChassisMoveControlLoop->vy_set;
    core_cmd.wz_radps = ChassisMoveControlLoop->wz_set;
    ChassisCoreStepVelocity(&core_cfg, &core_cmd, &core_output);

    if (ChassisMoveControlLoop->mode == CHASSIS_VECTOR_RAW)
    {

        for (i = 0; i < 4; i++)
        {
            const int8_t motor_dir = (snapshot != NULL) ? snapshot->motor_dir[i] : g_config.chassis.motor_dir[i];
            ChassisMoveControlLoop->motor_chassis[i].give_current = (int16_t)(core_output.wheel_speed_set[i] * (fp32)motor_dir);
        }
        ChassisFaultApplyControl(ChassisMoveControlLoop);
        if (pre_power_current != NULL)
        {
            for (i = 0; i < 4; i++)
            {
                pre_power_current[i] = ChassisMoveControlLoop->motor_chassis[i].give_current;
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
        ChassisMoveControlLoop->motor_chassis[i].speed_set = core_output.wheel_speed_set[i];
    }

    //calculate pid
    //计算pid
    for (i = 0; i < 4; i++)
    {
        PID_calc(&ChassisMoveControlLoop->motor_speed_pid[i], ChassisMoveControlLoop->motor_chassis[i].speed, ChassisMoveControlLoop->motor_chassis[i].speed_set);
    }

    /* 故障轴不能先占用四轮共享功率额度，再在发送前才被清零。 */
    ChassisFaultApplyControl(ChassisMoveControlLoop);
    if (pre_power_current != NULL)
    {
        for (i = 0; i < 4; i++)
        {
            pre_power_current[i] = ChassisSdLogClampCurrent(ChassisMoveControlLoop->motor_speed_pid[i].out *
                                                               (fp32)((snapshot != NULL) ? snapshot->motor_dir[i] : g_config.chassis.motor_dir[i]));
        }
    }

    //功率控制
    ChassisPowerControl(ChassisMoveControlLoop, ChassisFaultPowerEligibleMask());

    for (i = 0; i < 4; i++)
    {
        const int8_t motor_dir = (snapshot != NULL) ? snapshot->motor_dir[i] : g_config.chassis.motor_dir[i];
        ChassisMoveControlLoop->motor_chassis[i].give_current = (int16_t)(ChassisMoveControlLoop->motor_speed_pid[i].out * (fp32)motor_dir);
    }
}
