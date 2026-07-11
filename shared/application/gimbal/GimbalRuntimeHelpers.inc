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

static MotorId s_gimbal_yaw_id = MotorCount;
static MotorId s_gimbal_yaw_upper_id = MotorCount;
static MotorId s_gimbal_pitch_id = MotorCount;
static uint8_t s_gimbal_motor_ids_ready = 0u;

static void GimbalMotorIdsEnsure(void)
{
    if (s_gimbal_motor_ids_ready != 0u)
    {
        return;
    }

    s_gimbal_yaw_id = MotorInstIdByName("motor.yaw");
    s_gimbal_yaw_upper_id = MotorInstIdByName("motor.yaw_upper");
    s_gimbal_pitch_id = MotorInstIdByName("motor.pitch");
    s_gimbal_motor_ids_ready = 1u;
}

static void GimbalMotorHealthRead(uint8_t configured,
                                  MotorId id,
                                  uint32_t now_ms,
                                  MotorHealthResult *out)
{
    if (out == NULL)
    {
        return;
    }

    memset(out, 0, sizeof(*out));
    out->motorId = id;
    out->ageMs = MOTOR_HEALTH_AGE_UNKNOWN;
    if (configured == 0u || id == MotorCount)
    {
        return;
    }

    (void)MotorHealthRead(id, now_ms, GIMBAL_MOTOR_FEEDBACK_TIMEOUT_MS, out);
}

static uint8_t GimbalImuFeedbackReady(const GimbalRuntimeSnapshot *snapshot)
{
    return (uint8_t)(snapshot != NULL &&
                     snapshot->feedback_mode == (uint8_t)GIMBAL_FEEDBACK_IMU_NORMAL &&
                     snapshot->feedback_transition_pending == 0u &&
                     snapshot->imu_online != 0u);
}

static uint8_t GimbalYawControlHealthy(const GimbalRuntimeSnapshot *snapshot)
{
    if (snapshot == NULL)
    {
        return 0u;
    }
    if (snapshot->yaw_control_is_upper != 0u)
    {
        return (uint8_t)(snapshot->yaw_upper_configured != 0u &&
                         snapshot->yaw_upper_health.healthy != 0u);
    }
    return (uint8_t)(snapshot->yaw_configured != 0u && snapshot->yaw_health.healthy != 0u);
}

static void GimbalAxisClear(GimbalMotor *motor)
{
    if (motor == NULL)
    {
        return;
    }

    GimbalPidClear(&motor->GimbalMotorAnglePid);
    PID_clear(&motor->GimbalMotorGyroPid);
    motor->angle_set = motor->angle;
    motor->motor_gyro_set = motor->motor_gyro;
    motor->raw_cmd_current = 0.0f;
    motor->current_set = 0.0f;
    motor->given_current = 0;
}

static uint8_t GimbalFeedbackAxisReady(const GimbalRuntimeSnapshot *snapshot,
                                       uint32_t axisMask)
{
    return (uint8_t)(snapshot != NULL &&
                     (snapshot->feedback_block_mask & axisMask) == 0u);
}

static void GimbalFeedbackDiscardVision(const GimbalRuntimeSnapshot *snapshot)
{
    VisionToGimbal discardedVision;

    if (snapshot != NULL &&
        (snapshot->feedback_mode == (uint8_t)GIMBAL_FEEDBACK_ENCODER_DEGRADED ||
         snapshot->feedback_transition_pending != 0u))
    {
        /* 降级和反馈切换等待期间都消费最新视觉帧，避免恢复后执行积压目标。 */
        (void)VisionTakeLatest(&discardedVision);
    }
}

static void GimbalFeedbackTransitionReset(GimbalControl *control,
                                          const GimbalRuntimeSnapshot *snapshot)
{
    const uint32_t yawMask = GIMBAL_FAULT_MASK_YAW |
                             GIMBAL_FAULT_MASK_YAW_UPPER;

    if (control == NULL || snapshot == NULL ||
        snapshot->feedback_transition_pending == 0u)
    {
        return;
    }

    /* 反馈基准切换时先取消掉头并对齐目标，当前帧再由 holdZeroMask 保证零输出。 */
    GimbalBehaviourInputGateBlock();
    if ((snapshot->feedback_transition_zero_mask & yawMask) != 0u)
    {
        GimbalAxisClear(&control->GimbalYawMotor);
    }
    if ((snapshot->feedback_transition_zero_mask & GIMBAL_FAULT_MASK_PITCH) != 0u)
    {
        GimbalAxisClear(&control->GimbalPitchMotor);
    }
    s_dual_yaw_imu_center_ready = 0u;
    s_dual_yaw_output_allowed_prev = 0u;
}

static void GimbalApplyHealthToControl(GimbalControl *control, const GimbalRuntimeSnapshot *snapshot)
{
    uint32_t yaw_mask;
    uint8_t yaw_feedback_ready;
    uint8_t pitch_feedback_ready;

    if (control == NULL || snapshot == NULL)
    {
        return;
    }

    yaw_mask = (snapshot->yaw_control_is_upper != 0u) ?
                   GIMBAL_FAULT_MASK_YAW_UPPER :
                   GIMBAL_FAULT_MASK_YAW;
    yaw_feedback_ready = GimbalFeedbackAxisReady(snapshot, yaw_mask);
    pitch_feedback_ready = GimbalFeedbackAxisReady(snapshot, GIMBAL_FAULT_MASK_PITCH);

    if (yaw_feedback_ready == 0u || GimbalYawControlHealthy(snapshot) == 0u)
    {
        GimbalAxisClear(&control->GimbalYawMotor);
    }
    if (pitch_feedback_ready == 0u ||
        snapshot->pitch_configured == 0u ||
        snapshot->pitch_health.healthy == 0u)
    {
        GimbalAxisClear(&control->GimbalPitchMotor);
    }
}

static void GimbalApplyOutputHealth(const GimbalRuntimeSnapshot *snapshot,
                                    int16_t *yaw_current,
                                    int16_t *yaw_upper_current,
                                    int16_t *pitch_current)
{
    if (snapshot == NULL)
    {
        return;
    }

    if (yaw_current != NULL &&
        ((snapshot->feedback_block_mask & GIMBAL_FAULT_MASK_YAW) != 0u ||
         snapshot->yaw_configured == 0u ||
         snapshot->yaw_health.healthy == 0u))
    {
        *yaw_current = 0;
    }
    if (yaw_upper_current != NULL &&
        ((snapshot->feedback_block_mask & GIMBAL_FAULT_MASK_YAW_UPPER) != 0u ||
         snapshot->yaw_upper_configured == 0u ||
         snapshot->yaw_upper_health.healthy == 0u))
    {
        *yaw_upper_current = 0;
    }
    if (pitch_current != NULL &&
        ((snapshot->feedback_block_mask & GIMBAL_FAULT_MASK_PITCH) != 0u ||
         snapshot->pitch_configured == 0u ||
         snapshot->pitch_health.healthy == 0u))
    {
        *pitch_current = 0;
    }
}

static int16_t GimbalApplyOutputTurn(int16_t current, uint8_t turn)
{
    return (turn != 0u) ? (int16_t)-current : current;
}

static uint8_t GimbalRuntimeAllowsMotor(const GimbalRuntimeSnapshot *snapshot,
                                        MotorId id)
{
    if (snapshot == NULL)
    {
        return 0u;
    }
    return robot_mode_snapshot_allow_motor(snapshot->run_mode,
                                           snapshot->target_motor,
                                           id);
}

static uint8_t GimbalSingleMitYawTestActive(const GimbalRuntimeSnapshot *snapshot)
{
    MotorId target;
    robot_run_mode_e mode;
    MotorId expected;
    const motor_node_param_t *node;
    robot_run_variant_e variant;

    if (snapshot == NULL)
    {
        return 0u;
    }
    target = snapshot->target_motor;
    mode = snapshot->run_mode;
    node = MotorInstNode(target);
    variant = snapshot->run_variant;

    GimbalMotorIdsEnsure();
    expected = (snapshot->yaw_control_is_upper != 0u) ?
                   s_gimbal_yaw_upper_id :
                   s_gimbal_yaw_id;
    if (mode != ROBOT_RUN_MODE_SINGLE_MOTOR ||
        variant != ROBOT_RUN_VARIANT_GIMBAL_YAW_ONLY ||
        target != expected ||
        (uint32_t)target >= (uint32_t)MotorCount ||
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
    int16_t axis = (snapshot != NULL &&
                    snapshot->manual_input != NULL &&
                    snapshot->manual_online != 0u) ?
                       snapshot->manual_input->control.axis[INPUT_AXIS_GIMBAL_YAW] :
                       0;
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

static uint8_t GimbalApplySingleMitYawTest(
    const GimbalRuntimeSnapshot *snapshot,
    const MotorCurrentBind *bindings,
    const uint32_t *axisMasks,
    uint8_t bindingCount,
    const ControlOutputPermit *permit,
    GimbalPublishResult *result)
{
    const MotorId target = (snapshot != NULL) ?
                               snapshot->target_motor :
                               MotorCount;
    const fp32 torque = GimbalSingleMitYawTestTorque(snapshot);
    MotorId ids[DUAL_YAW_GIMBAL_OUTPUT_MOTOR_COUNT];
    MotorCmd cmds[DUAL_YAW_GIMBAL_OUTPUT_MOTOR_COUNT];
    MotorCmd before[DUAL_YAW_GIMBAL_OUTPUT_MOTOR_COUNT];
    uint32_t selectedAxisMasks[DUAL_YAW_GIMBAL_OUTPUT_MOTOR_COUNT];
    uint8_t beforeValid[DUAL_YAW_GIMBAL_OUTPUT_MOTOR_COUNT] = {0};
    uint8_t count = 0u;
    uint8_t targetFound = 0u;
    uint8_t written;

    if (result != NULL)
    {
        (void)memset(result, 0, sizeof(*result));
    }

    GimbalTotalPidClear(&g_gimbal);
    g_gimbal.GimbalYawMotor.current_set = 0.0f;
    g_gimbal.GimbalYawMotor.given_current = 0;
    g_gimbal.GimbalPitchMotor.current_set = 0.0f;
    g_gimbal.GimbalPitchMotor.given_current = 0;
    if (bindings == NULL || axisMasks == NULL || permit == NULL ||
        (uint32_t)target >= (uint32_t)MotorCount ||
        bindingCount > DUAL_YAW_GIMBAL_OUTPUT_MOTOR_COUNT)
    {
        return 0u;
    }
    for (uint8_t i = 0u; i < bindingCount; i++)
    {
        if (bindings[i].enabled == 0u ||
            (uint32_t)bindings[i].actuator_id >= (uint32_t)MotorCount)
        {
            continue;
        }
        if (bindings[i].actuator_id != target &&
            ((s_gimbalFeedbackRouteDebt.blockedMask |
              s_gimbalFeedbackRouteDebt.waitMask) & axisMasks[i]) != 0u)
        {
            continue;
        }
        ids[count] = bindings[i].actuator_id;
        selectedAxisMasks[count] = axisMasks[i];
        beforeValid[count] = LowCmdGetMotor(ids[count], &before[count]);
        if (ids[count] == target)
        {
            control_core_cmd_set_damping(&cmds[count],
                                         GIMBAL_SINGLE_MIT_TEST_DAMPING_KD,
                                         torque);
            targetFound = 1u;
        }
        else
        {
            control_core_cmd_set_current(&cmds[count], 0);
        }
        count++;
    }
    if (targetFound != 0u && count != 0u)
    {
        /* 已持有精确零命令的非目标轴不再改写；待补零轴与测试轴仍一次发布。 */
        written = MotorInstSetIdsBestEffortWithPermit(ids, cmds, count, permit);
        if (written == 0u)
        {
            return 0u;
        }
        if (result != NULL)
        {
            for (uint8_t i = 0u; i < count; i++)
            {
                MotorCmd after;

                if (ids[i] == target ||
                    LowCmdGetMotor(ids[i], &after) == 0u ||
                    after.active == 0u ||
                    after.mode != (uint8_t)MotorModeCurrent ||
                    after.writer != (uint16_t)LOWCMD_WRITER_CONTROL ||
                    after.current != 0 ||
                    (beforeValid[i] != 0u &&
                     after.seq == before[i].seq &&
                     after.seqEpoch == before[i].seqEpoch &&
                     after.tick == before[i].tick))
                {
                    continue;
                }
                result->ids[result->count] = ids[i];
                result->axisMasks[result->count] = selectedAxisMasks[i];
                result->count++;
            }
        }
        return 1u;
    }
    return 0u;
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

static uint8_t GimbalEncoderFallbackNodeReliable(
    uint8_t axis,
    const motor_node_param_t *node,
    const motor_measure_t *measure)
{
    uint32_t encoderRange;

    if (node == NULL || measure == NULL ||
        MotorCfgNodeId(node) == 0u ||
        MotorCfgModelDb(node->model) == NULL)
    {
        return 0u;
    }
    /* 经典 3510 pitch 的减速器间隙和零点不满足失姿态后的闭环要求。 */
    if (axis == GIMBAL_FAULT_AXIS_PITCH && node->model == MOTOR_MODEL_3510)
    {
        return 0u;
    }

    encoderRange = MotorCfgEncoderRange(node->model);
    return (uint8_t)(encoderRange >= 2u && encoderRange <= 65536u);
}

static uint32_t GimbalEncoderFallbackReliableMask(
    const GimbalRuntimeSnapshot *snapshot)
{
    uint32_t mask;

    if (snapshot == NULL)
    {
        return 0u;
    }

    mask = RobotProfileGimbalEncoderFallbackMask();
    if (GimbalEncoderFallbackNodeReliable(GIMBAL_FAULT_AXIS_YAW,
                                           &g_config.motor.yaw,
                                           get_yaw_gimbal_motor_measure_point()) == 0u)
    {
        mask &= ~GIMBAL_FAULT_MASK_YAW;
    }
    if (GimbalEncoderFallbackNodeReliable(GIMBAL_FAULT_AXIS_YAW_UPPER,
                                           &g_config.motor.yaw_upper,
                                           get_yaw_upper_gimbal_motor_measure_point()) == 0u)
    {
        mask &= ~GIMBAL_FAULT_MASK_YAW_UPPER;
    }
    if (GimbalEncoderFallbackNodeReliable(GIMBAL_FAULT_AXIS_PITCH,
                                           &g_config.motor.pitch,
                                           get_pitch_gimbal_motor_measure_point()) == 0u)
    {
        mask &= ~GIMBAL_FAULT_MASK_PITCH;
    }
    return mask;
}

static void GimbalSnapshotCapture(GimbalRuntimeSnapshot *snapshot,
                                  GimbalControl *control,
                                  const ManualInputSnapshot *manual_input)
{
    uint32_t now_ms;

    if (snapshot == NULL)
    {
        return;
    }

    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->ImuSnapshot.age_ms = GIMBAL_STATE_AGE_UNKNOWN;
    snapshot->manual_input = manual_input;
    snapshot->manual_online = (uint8_t)(snapshot->manual_input != NULL &&
                                        snapshot->manual_input->online != 0u);
    if (snapshot->manual_online != 0u)
    {
        snapshot->recovery_input_safe = ControlInputSwitchIsPos(
            snapshot->manual_input->control.sw[INPUT_SW_GIMBAL_MODE],
            snapshot->manual_input->semantics.GimbalSafePos);
    }
    snapshot->period_ms = RobotProfileGimbalControlPeriodMs();
    snapshot->period_us = (uint32_t)snapshot->period_ms * 1000u;
    if (InsSnapshotRead(&snapshot->ImuSnapshot) != 0 &&
        snapshot->ImuSnapshot.age_ms <= GIMBAL_IMU_SNAPSHOT_TIMEOUT_MS)
    {
        snapshot->imu_online = 1u;
        snapshot->gyro = snapshot->ImuSnapshot.gyro;
        snapshot->ins_angle = snapshot->ImuSnapshot.angle;
    }
    snapshot->yaw_control_is_upper = GimbalDualYawUseUpperControl();
    snapshot->yaw_configured = (MotorCfgNodeId(&g_config.motor.yaw) != 0u) ? 1u : 0u;
    snapshot->yaw_upper_configured = (MotorCfgNodeId(&g_config.motor.yaw_upper) != 0u) ? 1u : 0u;
    snapshot->pitch_configured = (MotorCfgNodeId(&g_config.motor.pitch) != 0u) ? 1u : 0u;
    GimbalMotorIdsEnsure();
    now_ms = BspTimeGetTickMs();
    GimbalMotorHealthRead(snapshot->yaw_configured, s_gimbal_yaw_id, now_ms, &snapshot->yaw_health);
    GimbalMotorHealthRead(snapshot->yaw_upper_configured,
                          s_gimbal_yaw_upper_id,
                          now_ms,
                          &snapshot->yaw_upper_health);
    GimbalMotorHealthRead(snapshot->pitch_configured, s_gimbal_pitch_id, now_ms, &snapshot->pitch_health);
    snapshot->yaw_measure = (control != NULL) ? control->GimbalYawMotor.GimbalMotorMeasure : GimbalYawControlMeasurePoint();
    snapshot->pitch_measure = (control != NULL) ? control->GimbalPitchMotor.GimbalMotorMeasure : get_pitch_gimbal_motor_measure_point();
    snapshot->yaw_motor_cfg = GimbalYawControlMotorCfg();
    snapshot->pitch_motor_cfg = &g_config.motor.pitch;
    snapshot->run_mode = robot_mode_current();
    snapshot->run_variant = robot_mode_variant();
    snapshot->target_motor = robot_mode_target_motor();
    snapshot->imu_required_mask = GimbalFaultImuAxisMask(
        snapshot->run_variant,
        (snapshot->yaw_configured != 0u ? GIMBAL_FAULT_MASK_YAW : 0u) |
            (snapshot->yaw_upper_configured != 0u ? GIMBAL_FAULT_MASK_YAW_UPPER : 0u) |
            (snapshot->pitch_configured != 0u ? GIMBAL_FAULT_MASK_PITCH : 0u),
        snapshot->yaw_control_is_upper,
        (uint8_t)(RobotProfileNeedDualGimbalControlTask() != 0u &&
                  g_config.dual_gimbal.enable != 0u));
    snapshot->encoder_fallback_mask = GimbalEncoderFallbackReliableMask(snapshot);
    {
        const GimbalFeedbackPolicyOutput feedback = GimbalFeedbackPolicyStep(
            &s_gimbalFeedbackPolicy,
            now_ms,
            snapshot->imu_online,
            snapshot->recovery_input_safe,
            snapshot->imu_required_mask,
            snapshot->encoder_fallback_mask);

        snapshot->feedback_mode = feedback.mode;
        snapshot->encoder_degraded_mask = feedback.encoderDegradedMask;
        snapshot->feedback_block_mask = feedback.feedbackBlockMask;
        snapshot->feedback_transition_zero_mask = feedback.transitionZeroMask;
        snapshot->feedback_transition_seq = feedback.transitionSeq;
        snapshot->feedback_recovery_pending = feedback.recoveryPending;
        snapshot->feedback_transition_pending = feedback.transitionPending;
    }
    snapshot->PitchCaliMode =
        (snapshot->feedback_mode == (uint8_t)GIMBAL_FEEDBACK_IMU_NORMAL &&
         snapshot->feedback_transition_pending == 0u) ?
            robot_mode_gimbal_pitch_cali() :
            0u;
    snapshot->yaw_turn = YAW_TURN ? 1u : 0u;
    snapshot->pitch_turn = PITCH_TURN ? 1u : 0u;

    if (control != NULL)
    {
        GimbalControlSnapshot fast = {0};
        const ManualInputState *manual_state = (snapshot->manual_online != 0u) ?
                                                   &snapshot->manual_input->manual :
                                                   NULL;
        const ControlInputState *control_input = (snapshot->manual_online != 0u) ?
                                                     &snapshot->manual_input->control :
                                                     NULL;

        fast.manual_offline = (snapshot->manual_online != 0u) ? 0u : 1u;
        fast.control_allowed = (uint8_t)(snapshot->manual_online != 0u &&
                                         RobotLifecycleOutputAllowed() != 0u);
        fast.authority_seq = (snapshot->manual_input != NULL) ?
                                 snapshot->manual_input->authoritySeq :
                                 0u;
        fast.semantics_seq = (snapshot->manual_input != NULL) ?
                                 snapshot->manual_input->semanticsSeq :
                                 0u;
        fast.run_variant = snapshot->run_variant;
        fast.PitchCaliMode = snapshot->PitchCaliMode;
        fast.safe_pos = (snapshot->manual_input != NULL) ?
                            snapshot->manual_input->semantics.GimbalSafePos :
                            MANUAL_INPUT_SWITCH_POS_UP;
        fast.mode_sw = (control_input != NULL) ?
                           control_input->sw[INPUT_SW_GIMBAL_MODE] :
                           ControlInputSwitchPosToRaw(fast.safe_pos);
        fast.active_source = (snapshot->manual_online != 0u) ?
                                 snapshot->manual_input->activeSource :
                                 MANUAL_INPUT_SRC_AUTO;
        fast.image_auto_aim_requested =
            (snapshot->manual_online != 0u &&
              (snapshot->manual_input->sourceFlags & MANUAL_INPUT_SOURCE_FLAG_AUTO_AIM) != 0u) ? 1u : 0u;
        fast.encoder_feedback_degraded =
            (snapshot->feedback_mode == (uint8_t)GIMBAL_FEEDBACK_ENCODER_DEGRADED ||
             snapshot->feedback_transition_pending != 0u) ? 1u : 0u;
        fast.yaw_axis = (control_input != NULL) ? control_input->axis[INPUT_AXIS_GIMBAL_YAW] : 0;
        fast.pitch_axis = (control_input != NULL) ? control_input->axis[INPUT_AXIS_GIMBAL_PITCH] : 0;
        fast.mouse_x = (manual_state != NULL) ? manual_state->mouse.x : 0;
        fast.mouse_y = (manual_state != NULL) ? manual_state->mouse.y : 0;
        fast.rc_deadband = RC_DEADBAND;
        fast.key_mask = (manual_state != NULL) ? manual_state->key.v : 0u;
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
                                     const GimbalRuntimeSnapshot *snapshot,
                                     ControlOutputPermit *permit)
{
    ControlCtx context = {0};
    const uint16_t period_ms =
        (snapshot != NULL && snapshot->period_ms != 0u) ? snapshot->period_ms : RobotProfileGimbalControlPeriodMs();

    context.tick_ms = BspTimeGetTickMs();
    context.dt_s = (fp32)period_ms * 0.001f;

    if (permit == NULL ||
        ControlMgrUpdateDomain(ControlDomainGimbal, &context) != ControlResultOk)
    {
        return 0;
    }

    if (ControlMgrActiveId(ControlDomainGimbal) != expected_controller_id ||
        ControlMgrOutputPermitValid(&context.outputPermit, 0u) == 0u)
    {
        return 0;
    }

    *permit = context.outputPermit;
    return 1;
}

static void GimbalRunShootControl(const GimbalRuntimeSnapshot *snapshot)
{
    ShootCtrlInput input = {0};
    ShootCtrlOutput output = {0};

    input.manualInput = (snapshot != NULL) ? snapshot->manual_input : NULL;
    input.tickMs = BspTimeGetTickMs();
    input.periodMs = (snapshot != NULL && snapshot->period_ms != 0u) ?
                         snapshot->period_ms :
                         RobotProfileGimbalControlPeriodMs();
    input.forceSafe = (uint8_t)(snapshot == NULL ||
                                snapshot->manual_online == 0u ||
                                snapshot->recovery_input_safe != 0u ||
                                RobotLifecycleOutputAllowed() == 0u);
    (void)ShootCtrlStep(&input, &output);
}

static void GimbalStopOutputs(const GimbalRuntimeSnapshot *snapshot)
{
    GimbalTotalPidClear(&g_gimbal);
    g_gimbal.GimbalYawMotor.current_set = 0;
    g_gimbal.GimbalYawMotor.given_current = 0;
    g_gimbal.GimbalPitchMotor.current_set = 0;
    g_gimbal.GimbalPitchMotor.given_current = 0;
    yaw_can_set_current = 0;
    pitch_can_set_current = 0;
    GimbalWatchYawCurrent = 0;
    GimbalWatchYawUpperCurrent = 0;
    GimbalWatchPitchCurrent = 0;

    GimbalWriteState(snapshot);
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
