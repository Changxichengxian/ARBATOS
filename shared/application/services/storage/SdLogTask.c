/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "SdLogTask.h"

#include "cmsis_os.h"

#include "RobotConfig.h"
#include "BspTime.h"
#include "SdCard.h"
#include "SdLog.h"
#include "RtProf.h"
#include "RobotTaskProfile.h"
#include "RobotMode.h"
#include "PowerMeter.h"
#include "ChassisPowerControl.h"
#if defined(__ZEPHYR__)
#include "RobotFaultZephyr.h"
#endif

#define SDLOG_TASK_IDLE_DELAY_MS 10u
#define SDLOG_TASK_BACKLOG_YIELD_POLLS 8u
#define SDLOG_TASK_RT_PROFILER_PERIOD_MS 500u
#define SDLOG_TASK_BOOT_DELAY_MS 2000u
#define SDLOG_TASK_REMOUNT_SETTLE_MS 2000u
#define SDLOG_TASK_REOPEN_RETRY_MS 1000u
#define SDLOG_TASK_MOUNT_RETRY_START_MS 200u
#define SDLOG_TASK_MOUNT_RETRY_MAX_MS 2000u

static uint32_t s_power_meter_log_drop_count = 0u;
static uint32_t s_chassis_power_model_sequence = 0u;

static uint8_t SdLogEntertainPause(void)
{
#if defined(__ZEPHYR__)
    /* 异常重启时优先保存证据，不受娱乐模式停日志的规则影响。 */
    if (RobotFaultZephyrBootLocked() != 0u) { return 0u; }
#endif
    return robot_mode_is_entertain();
}

static uint16_t SdLogWritePowerMeterBatch(void)
{
    PowerMeterSample samples[SDLOG_POWER_METER_MAX_SAMPLES];
    sdlog_power_meter_batch_t batch = {0};
    PowerMeterStats stats = {0};
    const uint16_t count = PowerMeterPopBatch(samples, SDLOG_POWER_METER_MAX_SAMPLES);

    if (count == 0u)
    {
        return 0u;
    }
    PowerMeterGetStats(&stats);
    batch.version = SDLOG_POWER_METER_VERSION;
    batch.count = (uint8_t)count;
    batch.canBus = g_config.powerMeter.canBus;
    batch.canId = g_config.powerMeter.canId;
    batch.canQueueDropCount = stats.queueDropCount;
    batch.logDropCount = s_power_meter_log_drop_count;
    for (uint16_t i = 0u; i < count; i++)
    {
        batch.sample[i].rxTickMs = samples[i].rxTickMs;
        batch.sample[i].sequence = samples[i].sequence;
        batch.sample[i].rawVoltage = samples[i].rawVoltage;
        batch.sample[i].rawCurrent = samples[i].rawCurrent;
        batch.sample[i].voltageV = samples[i].voltageV;
        batch.sample[i].currentA = samples[i].currentA;
        batch.sample[i].powerW = samples[i].powerW;
    }
    if (SdLogTryWrite(SDLOG_TAG_POWER_METER, &batch,
                      (uint16_t)(16u + count * sizeof(sdlog_power_meter_sample_t))) == 0u)
    {
        s_power_meter_log_drop_count += count;
    }
    return count;
}

static void SdLogWriteChassisPowerModel(void)
{
    ChassisPowerModelSnapshot snapshot = {0};
    sdlog_chassis_power_model_t record = {0};

    if (ChassisPowerModelReadSnapshot(&snapshot) == 0u || snapshot.sequence == s_chassis_power_model_sequence)
    {
        return;
    }
    record.version = SDLOG_CHASSIS_POWER_MODEL_VERSION;
    record.modelValid = snapshot.modelValid;
    record.reserved16 = (uint16_t)snapshot.activeMotorMask;
    record.tickMs = snapshot.tickMs;
    record.sequence = snapshot.sequence;
    for (uint8_t i = 0u; i < 4u; i++)
    {
        record.currentCmd[i] = snapshot.currentCmd[i];
        record.wheelRpm[i] = snapshot.wheelRpm[i];
    }
    record.estimatedPowerW = snapshot.estimatedPowerW;
    if (SdLogTryWrite(SDLOG_TAG_CHASSIS_POWER_MODEL, &record, (uint16_t)sizeof(record)) != 0u)
    {
        s_chassis_power_model_sequence = snapshot.sequence;
    }
}

static uint8_t sdlog_time_reached(uint32_t now_ms, uint32_t deadline_ms)
{
    return ((int32_t)(now_ms - deadline_ms) >= 0) ? 1u : 0u;
}

static void sdlog_grow_mount_retry(uint32_t *retry_ms)
{
    if (retry_ms == NULL || *retry_ms >= SDLOG_TASK_MOUNT_RETRY_MAX_MS)
    {
        return;
    }

    *retry_ms *= 2u;
    if (*retry_ms > SDLOG_TASK_MOUNT_RETRY_MAX_MS)
    {
        *retry_ms = SDLOG_TASK_MOUNT_RETRY_MAX_MS;
    }
}

static uint8_t SdLogRtProfActive(RtProfId id)
{
    return RtProfActive(id);
}

static void SdLogWriteRtProfSample(void)
{
    SdLogRtProf sample = {0};
    uint8_t profiler_count = 0u;
    (void)RtProfDescs(&profiler_count);

    for (uint8_t i = 0u; i < profiler_count; i++)
    {
        if (sample.count >= (uint8_t)SDLOG_RT_PROFILER_MAX)
        {
            break;
        }
        if (!SdLogRtProfActive((RtProfId)i))
        {
            continue;
        }

        RtProfStats stats = {0};
        RtProfGet((RtProfId)i, &stats);

        sample.entry[sample.count].id = (uint8_t)i;
        sample.entry[sample.count].count = stats.count;
        sample.entry[sample.count].last_us = stats.last_us;
        sample.entry[sample.count].max_us = stats.max_us;
        sample.entry[sample.count].avg_us = stats.avg_us;
        sample.entry[sample.count].budget_us = stats.budget_us;
        sample.entry[sample.count].overrun_count = stats.overrun_count;
        sample.count++;
    }

    SdLogWrite(SDLOG_TAG_RT_PROFILER, &sample, (uint16_t)sizeof(sample));
}

static void sdlog_wait_boot_delay_ms(uint32_t delay_ms)
{
    const uint32_t now_ms = BspTimeGetTickMs();
    if (now_ms >= delay_ms)
    {
        return;
    }
    osDelay(delay_ms - now_ms);
}

void SdLogTask(void const *argument)
{
    (void)argument;

    // Wait for TF/SD ready (mount may be done by StartupServiceTask).
    uint32_t retry_ms = SDLOG_TASK_MOUNT_RETRY_START_MS;
    uint32_t next_start_ms = 0u;
    while (!SdcardIsMounted())
    {
        const int m = SdcardMount();
        if (m == 0)
        {
            break;
        }

        osDelay(retry_ms);
        sdlog_grow_mount_retry(&retry_ms);
    }
    retry_ms = SDLOG_TASK_MOUNT_RETRY_START_MS;

    if (SdLogEntertainPause() == 0u)
    {
        sdlog_wait_boot_delay_ms(SDLOG_TASK_BOOT_DELAY_MS);
        if (SdLogStart() != 0)
        {
            next_start_ms = BspTimeGetTickMs() + SDLOG_TASK_REOPEN_RETRY_MS;
        }
    }

    while (1)
    {
        if (SdLogEntertainPause() != 0u)
        {
            SdLogStop();
            osDelay(SDLOG_TASK_IDLE_DELAY_MS);
            continue;
        }

        if (!SdcardIsMounted())
        {
            const int m = SdcardMount();
            if (m != 0)
            {
                osDelay(retry_ms);
                sdlog_grow_mount_retry(&retry_ms);
                continue;
            }
            retry_ms = SDLOG_TASK_MOUNT_RETRY_START_MS;
            next_start_ms = BspTimeGetTickMs() + SDLOG_TASK_REMOUNT_SETTLE_MS;
        }

        // If the log file was closed due to an error, try to reopen it.
        if (!SdLogIsActive() && SdcardIsMounted())
        {
            const uint32_t now_ms = BspTimeGetTickMs();
            if (!sdlog_time_reached(now_ms, next_start_ms))
            {
                osDelay(SDLOG_TASK_IDLE_DELAY_MS);
                continue;
            }

            sdlog_wait_boot_delay_ms(SDLOG_TASK_BOOT_DELAY_MS);
            if (SdLogStart() != 0)
            {
                next_start_ms = BspTimeGetTickMs() + SDLOG_TASK_REOPEN_RETRY_MS;
            }
        }

        if (!SdLogIsActive())
        {
            osDelay(SDLOG_TASK_IDLE_DELAY_MS);
            continue;
        }

        static uint32_t lastRtProfLogMs = 0u;
        const uint32_t now_ms = BspTimeGetTickMs();
        if ((uint32_t)(now_ms - lastRtProfLogMs) >= SDLOG_TASK_RT_PROFILER_PERIOD_MS)
        {
            lastRtProfLogMs = now_ms;
            SdLogWriteRtProfSample();
        }

        if (g_config.powerMeter.enable != 0u)
        {
            for (uint8_t batch = 0u; batch < 4u; batch++)
            {
                if (SdLogWritePowerMeterBatch() == 0u)
                {
                    break;
                }
                SdLogPoll();
            }
            SdLogWriteChassisPowerModel();
        }

        uint32_t backlog_polls = 0u;
        while (1)
        {
            SdLogStats stats = {0};

            SdLogPoll();
            SdLogGetStats(&stats);

            if (stats.active == 0u || stats.ring_used == 0u)
            {
                break;
            }

            backlog_polls++;
            if (backlog_polls >= SDLOG_TASK_BACKLOG_YIELD_POLLS)
            {
                backlog_polls = 0u;
                (void)osThreadYield();
            }
        }

        osDelay(SDLOG_TASK_IDLE_DELAY_MS);
    }
}
