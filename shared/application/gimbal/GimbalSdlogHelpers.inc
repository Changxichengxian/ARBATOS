/*
 * SPDX-FileCopyrightText: 2026 ?? <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Private implementation chunk included by GimbalControlTask.c.
 */

static int16_t GimbalSdLogClampCurrent(fp32 current)
{
    if (current > 32767.0f)
    {
        return 32767;
    }
    if (current < -32768.0f)
    {
        return -32768;
    }
    return (int16_t)current;
}

static void GimbalFillMotorMeasureState(motor_measure_state_t *out, const motor_measure_t *measure)
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

static void GimbalFillMotorState(GimbalMotorState *out, const GimbalMotor *motor)
{
    if (out == NULL)
    {
        return;
    }

    memset(out, 0, sizeof(*out));
    if (motor == NULL)
    {
        return;
    }

    out->valid = 1u;
    out->motor_mode = (uint8_t)motor->mode;
    out->last_motor_mode = (uint8_t)motor->last_mode;
    out->offset_ecd = motor->offset_ecd;
    out->max_angle = motor->max_angle;
    out->min_angle = motor->min_angle;
    GimbalFillMotorMeasureState(&out->measure, motor->GimbalMotorMeasure);
    out->angle_pid = motor->GimbalMotorAnglePid;
    out->gyro_pid = motor->GimbalMotorGyroPid;
    out->angle = motor->angle;
    out->angle_set = motor->angle_set;
    out->motor_gyro = motor->motor_gyro;
    out->motor_gyro_set = motor->motor_gyro_set;
    out->motor_speed = motor->motor_speed;
    out->raw_cmd_current = motor->raw_cmd_current;
    out->current_set = motor->current_set;
    out->given_current = motor->given_current;
}

static void GimbalAxisRequirements(const GimbalRuntimeSnapshot *snapshot,
                                   uint8_t *yaw_required,
                                   uint8_t *yaw_upper_required,
                                   uint8_t *pitch_required)
{
    const uint8_t yaw_configured = (MotorCfgNodeId(&g_config.motor.yaw) != 0u) ? 1u : 0u;
    const uint8_t yaw_upper_configured = (MotorCfgNodeId(&g_config.motor.yaw_upper) != 0u) ? 1u : 0u;
    const uint8_t pitch_configured = (MotorCfgNodeId(&g_config.motor.pitch) != 0u) ? 1u : 0u;
    const uint8_t yaw_control_is_upper = (snapshot != NULL) ?
                                             snapshot->yaw_control_is_upper :
                                             GimbalDualYawUseUpperControl();
    const robot_run_variant_e variant = (snapshot != NULL) ? snapshot->run_variant : robot_mode_variant();

    if (yaw_required == NULL || yaw_upper_required == NULL || pitch_required == NULL)
    {
        return;
    }

    *yaw_required = 0u;
    *yaw_upper_required = 0u;
    *pitch_required = 0u;
    switch (variant)
    {
    case ROBOT_RUN_VARIANT_NORMAL:
        if (yaw_control_is_upper != 0u)
        {
            *yaw_upper_required = yaw_upper_configured;
        }
        else
        {
            *yaw_required = yaw_configured;
        }
        *pitch_required = pitch_configured;
        return;
    case ROBOT_RUN_VARIANT_GIMBAL_DUAL:
        if (yaw_control_is_upper != 0u)
        {
            *yaw_upper_required = yaw_upper_configured;
        }
        else
        {
            *yaw_required = yaw_configured;
        }
        *pitch_required = pitch_configured;
        if (RobotProfileNeedDualGimbalControlTask() != 0u &&
            g_config.dual_gimbal.enable != 0u)
        {
            *yaw_upper_required = yaw_upper_configured;
        }
        return;
    case ROBOT_RUN_VARIANT_GIMBAL_YAW_ONLY:
    case ROBOT_RUN_VARIANT_GIMBAL_YAW_EASY:
        if (yaw_control_is_upper != 0u)
        {
            *yaw_upper_required = yaw_upper_configured;
        }
        else
        {
            *yaw_required = yaw_configured;
        }
        return;
    case ROBOT_RUN_VARIANT_GIMBAL_PITCH_ONLY:
        *pitch_required = pitch_configured;
        return;
    default:
        return;
    }
}

static uint8_t GimbalFireAimReady(const GimbalRuntimeSnapshot *snapshot)
{
    const robot_run_variant_e variant = (snapshot != NULL) ? snapshot->run_variant : robot_mode_variant();

    if (snapshot == NULL || GimbalImuFeedbackReady(snapshot) == 0u)
    {
        return 0u;
    }

    switch (variant)
    {
    case ROBOT_RUN_VARIANT_NORMAL:
    case ROBOT_RUN_VARIANT_GIMBAL_DUAL:
        return (uint8_t)(GimbalYawControlHealthy(snapshot) != 0u &&
                         snapshot->pitch_configured != 0u &&
                         snapshot->pitch_health.healthy != 0u);
    case ROBOT_RUN_VARIANT_GIMBAL_YAW_ONLY:
        return GimbalYawControlHealthy(snapshot);
    case ROBOT_RUN_VARIANT_GIMBAL_PITCH_ONLY:
        return (uint8_t)(snapshot->pitch_configured != 0u &&
                         snapshot->pitch_health.healthy != 0u);
    default:
        return 0u;
    }
}

static void GimbalWriteState(const GimbalRuntimeSnapshot *snapshot)
{
    GimbalState state = {0};
    uint8_t yaw_required = 0u;
    uint8_t yaw_upper_required = 0u;
    uint8_t pitch_required = 0u;
    const uint8_t yaw_configured = (MotorCfgNodeId(&g_config.motor.yaw) != 0u) ? 1u : 0u;
    const uint8_t yaw_upper_configured = (MotorCfgNodeId(&g_config.motor.yaw_upper) != 0u) ? 1u : 0u;
    const uint8_t pitch_configured = (MotorCfgNodeId(&g_config.motor.pitch) != 0u) ? 1u : 0u;
    const uint8_t manual_offline = (snapshot == NULL || snapshot->manual_online == 0u) ? 1u : 0u;
    const uint8_t imu_required = (snapshot != NULL && snapshot->feedback_block_mask != 0u) ? 1u : 0u;
    const uint8_t imu_offline = (snapshot == NULL || snapshot->imu_online == 0u) ? 1u : 0u;
    const uint8_t yaw_offline = (yaw_configured != 0u &&
                                 (snapshot == NULL || snapshot->yaw_health.healthy == 0u)) ?
                                    1u :
                                    0u;
    const uint8_t yaw_upper_offline = (yaw_upper_configured != 0u &&
                                       (snapshot == NULL || snapshot->yaw_upper_health.healthy == 0u)) ?
                                          1u :
                                          0u;
    const uint8_t pitch_offline = (pitch_configured != 0u &&
                                   (snapshot == NULL || snapshot->pitch_health.healthy == 0u)) ?
                                      1u :
                                      0u;
    const uint32_t aim_axis_mask = (snapshot != NULL) ?
        GimbalFaultAimAxisMask(snapshot->run_variant,
                               s_gimbalFault.configuredMask,
                               snapshot->yaw_control_is_upper) :
        0u;

    GimbalAxisRequirements(snapshot, &yaw_required, &yaw_upper_required, &pitch_required);

    state.valid = 1u;
    state.behaviour = (uint8_t)GimbalBehaviourWatch;
    state.ChassisStop = (uint8_t)GimbalCmdToChassisStop();
    state.ShootStop = (uint8_t)GimbalCmdToShootStop();
    state.manual_online = (manual_offline == 0u) ? 1u : 0u;
    state.imu_online = (imu_offline == 0u) ? 1u : 0u;
    state.feedback_mode = (snapshot != NULL) ?
                              snapshot->feedback_mode :
                              (uint8_t)GIMBAL_FEEDBACK_ENCODER_DEGRADED;
    state.encoder_fallback_mask = (snapshot != NULL) ?
                                      (uint8_t)snapshot->encoder_fallback_mask :
                                      0u;
    state.feedback_degraded_mask = (snapshot != NULL) ?
                                       (uint8_t)snapshot->encoder_degraded_mask :
                                       0u;
    state.feedback_block_mask = (snapshot != NULL) ?
                                    (uint8_t)snapshot->feedback_block_mask :
                                    (uint8_t)s_gimbalFault.configuredMask;
    state.feedback_recovery_pending = (snapshot != NULL) ?
                                          snapshot->feedback_recovery_pending :
                                          0u;
    state.feedback_transition_zero_mask = (snapshot != NULL) ?
                                              (uint8_t)snapshot->feedback_transition_zero_mask :
                                              0u;
    state.yaw_required = yaw_required;
    state.yaw_online = (yaw_configured != 0u && yaw_offline == 0u) ? 1u : 0u;
    state.yaw_upper_required = yaw_upper_required;
    state.yaw_upper_online = (yaw_upper_configured != 0u && yaw_upper_offline == 0u) ? 1u : 0u;
    state.pitch_required = pitch_required;
    state.pitch_online = (pitch_configured != 0u && pitch_offline == 0u) ? 1u : 0u;
    state.yaw_control_is_upper = (snapshot != NULL) ? snapshot->yaw_control_is_upper : GimbalDualYawUseUpperControl();
    state.fault_configured_mask = (uint8_t)s_gimbalFault.configuredMask;
    state.fault_active_mask = (uint8_t)s_gimbalFault.activeMask;
    state.fault_blocking_mask = (uint8_t)s_gimbalFault.blockingMask;
    state.fault_recovery_mask = (uint8_t)s_gimbalFault.recoveryMask;
    state.fault_inhibit_mask = (uint8_t)s_gimbalFault.inhibitMask;
    state.fault_hold_zero_mask = (uint8_t)s_gimbalFault.holdZeroMask;
    state.fault_imu_required_mask = (uint8_t)s_gimbalFault.imuRequiredMask;
    state.fault_recovery_input_safe = s_gimbalFault.recoveryInputSafe;
    state.fault_inhibit_fail_count = s_gimbalFault.inhibitFailCount;
    state.fault_release_fail_count = s_gimbalFault.releaseFailCount;
    state.follow_available = (uint8_t)(snapshot != NULL &&
                                       GimbalYawControlHealthy(snapshot) != 0u &&
                                       GimbalFeedbackAxisReady(
                                           snapshot,
                                           snapshot->yaw_control_is_upper != 0u ?
                                               GIMBAL_FAULT_MASK_YAW_UPPER :
                                               GIMBAL_FAULT_MASK_YAW) != 0u &&
                                       (s_gimbalFault.blockingMask &
                                        (snapshot->yaw_control_is_upper != 0u ?
                                             GIMBAL_FAULT_MASK_YAW_UPPER :
                                             GIMBAL_FAULT_MASK_YAW)) == 0u);
    state.yaw_reason_mask = (snapshot != NULL) ?
                                snapshot->yaw_health.reasonMask :
                                (yaw_configured != 0u ? (uint16_t)MOTOR_HEALTH_REASON_NO_FEEDBACK : 0u);
    state.yaw_upper_reason_mask = (snapshot != NULL) ?
                                      snapshot->yaw_upper_health.reasonMask :
                                      (yaw_upper_configured != 0u ?
                                           (uint16_t)MOTOR_HEALTH_REASON_NO_FEEDBACK :
                                           0u);
    state.pitch_reason_mask = (snapshot != NULL) ?
                                  snapshot->pitch_health.reasonMask :
                                  (pitch_configured != 0u ? (uint16_t)MOTOR_HEALTH_REASON_NO_FEEDBACK : 0u);
    state.yaw_feedback_age_ms = (snapshot != NULL) ? snapshot->yaw_health.ageMs : GIMBAL_STATE_AGE_UNKNOWN;
    state.yaw_upper_feedback_age_ms = (snapshot != NULL) ?
                                          snapshot->yaw_upper_health.ageMs :
                                          GIMBAL_STATE_AGE_UNKNOWN;
    state.pitch_feedback_age_ms = (snapshot != NULL) ? snapshot->pitch_health.ageMs : GIMBAL_STATE_AGE_UNKNOWN;
    state.imu_age_ms = (snapshot != NULL) ? snapshot->ImuSnapshot.age_ms : GIMBAL_STATE_AGE_UNKNOWN;
    if (manual_offline != 0u)
    {
        state.offline_mask |= GIMBAL_STATE_OFFLINE_MANUAL;
    }
    if (yaw_configured != 0u && yaw_offline != 0u)
    {
        state.offline_mask |= GIMBAL_STATE_OFFLINE_YAW;
        if (yaw_required != 0u)
        {
            state.required_offline_mask |= GIMBAL_STATE_OFFLINE_YAW;
        }
    }
    if (yaw_upper_configured != 0u && yaw_upper_offline != 0u)
    {
        state.offline_mask |= GIMBAL_STATE_OFFLINE_YAW_UPPER;
        if (yaw_upper_required != 0u)
        {
            state.required_offline_mask |= GIMBAL_STATE_OFFLINE_YAW_UPPER;
        }
    }
    if (pitch_configured != 0u && pitch_offline != 0u)
    {
        state.offline_mask |= GIMBAL_STATE_OFFLINE_PITCH;
        if (pitch_required != 0u)
        {
            state.required_offline_mask |= GIMBAL_STATE_OFFLINE_PITCH;
        }
    }
    if (imu_offline != 0u)
    {
        state.offline_mask |= GIMBAL_STATE_OFFLINE_IMU;
        if (imu_required != 0u)
        {
            state.required_offline_mask |= GIMBAL_STATE_OFFLINE_IMU;
        }
    }
    {
        uint32_t required_axis_mask = 0u;

        if (yaw_required != 0u)
        {
            required_axis_mask |= GIMBAL_FAULT_MASK_YAW;
        }
        if (yaw_upper_required != 0u)
        {
            required_axis_mask |= GIMBAL_FAULT_MASK_YAW_UPPER;
        }
        if (pitch_required != 0u)
        {
            required_axis_mask |= GIMBAL_FAULT_MASK_PITCH;
        }
        state.online = (uint8_t)(state.required_offline_mask == 0u &&
                                 (s_gimbalFault.blockingMask & required_axis_mask) == 0u);
    }
    state.controllable = (uint8_t)(state.online != 0u &&
                                   state.manual_online != 0u &&
                                   state.ChassisStop == 0u &&
                                   snapshot != NULL &&
                                   snapshot->control_allowed != 0u);
    state.fire_allowed = (GimbalFireAimReady(snapshot) != 0u &&
                          (s_gimbalFault.blockingMask & aim_axis_mask) == 0u &&
                          state.manual_online != 0u &&
                          snapshot != NULL &&
                          snapshot->control_allowed != 0u &&
                          state.ShootStop == 0u) ?
                             1u :
                             0u;
    state.turnaround_active = (uint8_t)GimbalTurnaroundIsActive();
    state.turnaround_follow_offset_rad = GimbalTurnaroundChassisFollowOffsetRad();
    state.turnaround_frame_valid = (uint8_t)GimbalTurnaroundGetFrameYawRelative(&state.turnaround_frame_yaw_relative);
    GimbalFillMotorState(&state.yaw, &g_gimbal.GimbalYawMotor);
    if (pitch_configured != 0u)
    {
        GimbalFillMotorState(&state.pitch, &g_gimbal.GimbalPitchMotor);
    }

    (void)GimbalStateWrite(&state);
}

static void GimbalSdLogBeginBaseStream(uint32_t now_ms, uint32_t period_us)
{
    s_gimbal_sdlog_base_stream.start_tick_ms = now_ms;
    s_gimbal_sdlog_base_stream.last_tick_ms = now_ms;
    s_gimbal_sdlog_base_stream.period_us = period_us;
    s_gimbal_sdlog_base_stream.sample_count = 0u;
}

static void GimbalSdLogFlushBaseStream(void)
{
    if (s_gimbal_sdlog_base_stream.sample_count == 0u)
    {
        return;
    }

    sdlog_gimbal_base_stream_header_t header = {0};
    uint8_t payload[sizeof(header) + sizeof(s_gimbal_sdlog_base_stream.samples)];
    const uint16_t payload_len =
        (uint16_t)(sizeof(header) + (uint32_t)s_gimbal_sdlog_base_stream.sample_count * sizeof(sdlog_gimbal_base_sample_t));

    header.start_tick_ms = s_gimbal_sdlog_base_stream.start_tick_ms;
    header.period_us = (s_gimbal_sdlog_base_stream.period_us > 0xFFFFu) ? 0xFFFFu : (uint16_t)s_gimbal_sdlog_base_stream.period_us;
    header.sample_count = (uint8_t)s_gimbal_sdlog_base_stream.sample_count;
    header.version = SDLOG_GIMBAL_BASE_STREAM_VERSION;

    memcpy(payload, &header, sizeof(header));
    memcpy(&payload[sizeof(header)],
           s_gimbal_sdlog_base_stream.samples,
           (uint32_t)s_gimbal_sdlog_base_stream.sample_count * sizeof(sdlog_gimbal_base_sample_t));
    SdLogWrite(SDLOG_TAG_GIMBAL_BASE_STREAM, payload, payload_len);
    s_gimbal_sdlog_base_stream.sample_count = 0u;
}

static void GimbalSdLogAppendBaseSample(const sdlog_gimbal_base_sample_t *sample,
                                            uint32_t now_ms,
                                            uint32_t period_us)
{
    if (sample == NULL)
    {
        return;
    }

    if (period_us == 0u)
    {
        period_us = 1000u;
    }

    const uint8_t div = SdLogHighRateDiv();
    if (div > 1u)
    {
        const uint16_t slot = s_gimbal_sdlog_base_stream.sample_div_counter++;
        if ((slot % (uint16_t)div) != 0u)
        {
            return;
        }
        period_us *= (uint32_t)div;
    }
    else
    {
        s_gimbal_sdlog_base_stream.sample_div_counter = 0u;
    }

    if (s_gimbal_sdlog_base_stream.sample_count == 0u)
    {
        GimbalSdLogBeginBaseStream(now_ms, period_us);
    }
    else
    {
        const uint32_t expected_dt_ms = (s_gimbal_sdlog_base_stream.period_us + 999u) / 1000u;
        const uint32_t actual_dt_ms = now_ms - s_gimbal_sdlog_base_stream.last_tick_ms;
        if (s_gimbal_sdlog_base_stream.period_us != period_us || actual_dt_ms != expected_dt_ms)
        {
            GimbalSdLogFlushBaseStream();
            GimbalSdLogBeginBaseStream(now_ms, period_us);
        }
    }

    s_gimbal_sdlog_base_stream.samples[s_gimbal_sdlog_base_stream.sample_count++] = *sample;
    s_gimbal_sdlog_base_stream.last_tick_ms = now_ms;

    if (s_gimbal_sdlog_base_stream.sample_count >= GIMBAL_SDLOG_BASE_STREAM_MAX_SAMPLES)
    {
        GimbalSdLogFlushBaseStream();
    }
}
