/*
 * SPDX-FileCopyrightText: 2026 ?? <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Private implementation chunk included by ChassisControlTask.c.
 */

static int16_t ChassisSdLogClampCurrent(fp32 current)
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

static void ChassisSdLogBeginBaseStream(uint32_t now_ms, uint32_t period_us)
{
    s_chassis_sdlog_base_stream.start_tick_ms = now_ms;
    s_chassis_sdlog_base_stream.last_tick_ms = now_ms;
    s_chassis_sdlog_base_stream.period_us = period_us;
    s_chassis_sdlog_base_stream.sample_count = 0u;
}

static void ChassisSdLogFlushBaseStream(void)
{
    if (s_chassis_sdlog_base_stream.sample_count == 0u)
    {
        return;
    }

    sdlog_chassis_base_stream_header_t header = {0};
    uint8_t payload[sizeof(header) + sizeof(s_chassis_sdlog_base_stream.samples)];
    const uint16_t payload_len =
        (uint16_t)(sizeof(header) + (uint32_t)s_chassis_sdlog_base_stream.sample_count * sizeof(sdlog_chassis_base_sample_t));

    header.start_tick_ms = s_chassis_sdlog_base_stream.start_tick_ms;
    header.period_us = (s_chassis_sdlog_base_stream.period_us > 0xFFFFu) ? 0xFFFFu : (uint16_t)s_chassis_sdlog_base_stream.period_us;
    header.sample_count = (uint8_t)s_chassis_sdlog_base_stream.sample_count;
    header.version = SDLOG_CHASSIS_BASE_STREAM_VERSION;

    memcpy(payload, &header, sizeof(header));
    memcpy(&payload[sizeof(header)],
           s_chassis_sdlog_base_stream.samples,
           (uint32_t)s_chassis_sdlog_base_stream.sample_count * sizeof(sdlog_chassis_base_sample_t));
    SdLogWrite(SDLOG_TAG_CHASSIS_BASE_STREAM, payload, payload_len);
    s_chassis_sdlog_base_stream.sample_count = 0u;
}

static void ChassisControlClearOutputs(ChassisMove *control)
{
    if (control == NULL)
    {
        return;
    }

    ChassisSdLogFlushBaseStream();
    for (uint8_t i = 0; i < CHASSIS_MOTOR_COUNT; i++)
    {
        PID_clear(&control->motor_speed_pid[i]);
        control->motor_chassis[i].speed_set = 0.0f;
        control->motor_chassis[i].give_current = 0;
    }
    control->vx_set = 0.0f;
    control->vy_set = 0.0f;
    control->wz_set = 0.0f;
}

static void ChassisControlStopOutputs(ChassisMove *control)
{
    ChassisControlClearOutputs(control);

    (void)MotorInstSetCurrentBindsBestEffort(ChassisMotorCurrentBindings,
                                             ChassisZeroCurrentCmd,
                                             CHASSIS_MOTOR_COUNT);
}

static void ChassisControlDelay(TickType_t *last_wake, uint16_t period_ms)
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

static void ChassisSdLogAppendBaseSample(const sdlog_chassis_base_sample_t *sample,
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
        const uint16_t slot = s_chassis_sdlog_base_stream.sample_div_counter++;
        if ((slot % (uint16_t)div) != 0u)
        {
            return;
        }
        period_us *= (uint32_t)div;
    }
    else
    {
        s_chassis_sdlog_base_stream.sample_div_counter = 0u;
    }

    if (s_chassis_sdlog_base_stream.sample_count == 0u)
    {
        ChassisSdLogBeginBaseStream(now_ms, period_us);
    }
    else
    {
        const uint32_t expected_dt_ms = (s_chassis_sdlog_base_stream.period_us + 999u) / 1000u;
        const uint32_t actual_dt_ms = now_ms - s_chassis_sdlog_base_stream.last_tick_ms;
        if (s_chassis_sdlog_base_stream.period_us != period_us || actual_dt_ms != expected_dt_ms)
        {
            ChassisSdLogFlushBaseStream();
            ChassisSdLogBeginBaseStream(now_ms, period_us);
        }
    }

    s_chassis_sdlog_base_stream.samples[s_chassis_sdlog_base_stream.sample_count++] = *sample;
    s_chassis_sdlog_base_stream.last_tick_ms = now_ms;

    if (s_chassis_sdlog_base_stream.sample_count >= CHASSIS_SDLOG_BASE_STREAM_MAX_SAMPLES)
    {
        ChassisSdLogFlushBaseStream();
    }
}
