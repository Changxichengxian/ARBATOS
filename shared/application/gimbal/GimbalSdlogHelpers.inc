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

static void GimbalWriteState(void)
{
    GimbalState state = {0};
    const uint8_t yaw_required = (MotorCfgNodeId(&g_config.motor.yaw) != 0u) ? 1u : 0u;
    const uint8_t yaw_upper_required =
        (RobotProfileNeedDualGimbalControlTask() != 0u && MotorCfgNodeId(&g_config.motor.yaw_upper) != 0u) ? 1u : 0u;
    const uint8_t pitch_required = (MotorCfgNodeId(&g_config.motor.pitch) != 0u) ? 1u : 0u;
    const uint8_t manual_offline = toe_is_error(DBUS_TOE) ? 1u : 0u;
    const uint8_t imu_required = (GIMBAL_USE_ENCODER_FEEDBACK == 0u) ? 1u : 0u;
    const uint8_t imu_offline = toe_is_error(RM_IMU_TOE) ? 1u : 0u;
    const uint8_t yaw_offline = (yaw_required != 0u && toe_is_error(YAW_GIMBAL_MOTOR_TOE)) ? 1u : 0u;
    const uint8_t pitch_offline = (pitch_required != 0u && toe_is_error(PITCH_GIMBAL_MOTOR_TOE)) ? 1u : 0u;
    const uint8_t yaw_upper_offline = 0u;

    state.valid = 1u;
    state.behaviour = (uint8_t)GimbalBehaviourWatch;
    state.ChassisStop = (uint8_t)GimbalCmdToChassisStop();
    state.ShootStop = (uint8_t)GimbalCmdToShootStop();
    state.manual_online = (manual_offline == 0u) ? 1u : 0u;
    state.imu_online = (imu_required == 0u || imu_offline == 0u) ? 1u : 0u;
    state.yaw_required = yaw_required;
    state.yaw_online = (yaw_required != 0u && yaw_offline == 0u) ? 1u : 0u;
    state.yaw_upper_required = yaw_upper_required;
    state.yaw_upper_online = (yaw_upper_required != 0u && yaw_upper_offline == 0u) ? 1u : 0u;
    state.pitch_required = pitch_required;
    state.pitch_online = (pitch_required != 0u && pitch_offline == 0u) ? 1u : 0u;
    if (manual_offline != 0u)
    {
        state.offline_mask |= GIMBAL_STATE_OFFLINE_DBUS;
    }
    if (yaw_offline != 0u)
    {
        state.offline_mask |= GIMBAL_STATE_OFFLINE_YAW;
        state.required_offline_mask |= GIMBAL_STATE_OFFLINE_YAW;
    }
    if (yaw_upper_offline != 0u)
    {
        state.offline_mask |= GIMBAL_STATE_OFFLINE_YAW_UPPER;
        state.required_offline_mask |= GIMBAL_STATE_OFFLINE_YAW_UPPER;
    }
    if (pitch_offline != 0u)
    {
        state.offline_mask |= GIMBAL_STATE_OFFLINE_PITCH;
        state.required_offline_mask |= GIMBAL_STATE_OFFLINE_PITCH;
    }
    if (imu_offline != 0u)
    {
        state.offline_mask |= GIMBAL_STATE_OFFLINE_IMU;
        if (imu_required != 0u)
        {
            state.required_offline_mask |= GIMBAL_STATE_OFFLINE_IMU;
        }
    }
    state.online = (state.required_offline_mask == 0u) ? 1u : 0u;
    state.controllable = (state.online != 0u && state.manual_online != 0u && state.ChassisStop == 0u) ? 1u : 0u;
    state.fire_allowed = (state.online != 0u && state.manual_online != 0u && state.ShootStop == 0u) ? 1u : 0u;
    state.turnaround_active = (uint8_t)GimbalTurnaroundIsActive();
    state.turnaround_follow_offset_rad = GimbalTurnaroundChassisFollowOffsetRad();
    state.turnaround_frame_valid = (uint8_t)GimbalTurnaroundGetFrameYawRelative(&state.turnaround_frame_yaw_relative);
    GimbalFillMotorState(&state.yaw, &g_gimbal.GimbalYawMotor);
    if (pitch_required != 0u)
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
