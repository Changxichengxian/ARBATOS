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
#if defined(ARBATOS_BUILD_INFO_AUTOGEN_AVAILABLE)
#include "build_info_autogen.h"
#endif
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

static uint8_t s_flash_pause_latched = 0u;

const volatile uint32_t g_sdLogFlashMailboxGuard = SDLOG_FLASH_MAILBOX_GUARD;
#if defined(ARBATOS_BUILD_INFO_AUTOGEN_AVAILABLE)
const uint8_t g_sdLogFlashBuildFingerprint[SDLOG_FLASH_BUILD_FINGERPRINT_SIZE]
    __attribute__((used, aligned(4))) = ARBATOS_BUILD_FINGERPRINT_BYTES;
#endif
/* DTCM 不经过数据缓存，调试器和任务能直接看到彼此写入的状态。 */
__attribute__((section("DTCM"))) volatile uint32_t g_sdLogFlashPauseRequest;
__attribute__((section("DTCM"))) volatile uint32_t g_sdLogFlashPauseReady;

#if defined(CONFIG_ARBATOS_MINI_BENCH)
#include "WheelLegBenchGate.h"
#include "ff.h"
#include <stdio.h>
#include <string.h>

/* 仅停机、关日志后读取原始证据；单次底层读取不超过半扇区。 */
volatile struct {
    uint32_t request, operation, offset, length, result, count, size;
    char path[96];
    uint8_t data[4096];
} MiniBenchRead;

static void SdLogBenchRead(void)
{
    if (MiniBenchRead.request != 0x53445251u) return;
    MiniBenchRead.count = 0;
    MiniBenchRead.size = 0;
    MiniBenchRead.result = FR_DENIED;
    if (WheelLegBenchTest != 0u || WheelLegBenchArm != 0u ||
        g_sdLogFlashPauseReady != SDLOG_FLASH_PAUSE_READY || SdLogIsActive()) {
        MiniBenchRead.request = 0;
        return;
    }
    char path[96];
    for (unsigned i = 0; i < sizeof(path); i++) path[i] = MiniBenchRead.path[i];
    path[sizeof(path)-1] = 0;
    uint32_t length = MiniBenchRead.length;
    if (length > sizeof(MiniBenchRead.data)) length = sizeof(MiniBenchRead.data);
    if (MiniBenchRead.operation == 1u) {
        FIL file;
        FRESULT result = f_open(&file, path, FA_READ);
        if (result == FR_OK) {
            MiniBenchRead.size = (uint32_t)f_size(&file);
            result = f_lseek(&file, MiniBenchRead.offset);
            while (result == FR_OK && MiniBenchRead.count < length) {
                uint8_t chunk[256];
                UINT count = 0;
                UINT wanted = (UINT)(length - MiniBenchRead.count);
                if (wanted > sizeof(chunk)) wanted = sizeof(chunk);
                result = f_read(&file, chunk, wanted, &count);
                for (UINT i = 0; i < count; i++) MiniBenchRead.data[MiniBenchRead.count+i] = chunk[i];
                MiniBenchRead.count += count;
                if (count < wanted) break;
            }
            (void)f_close(&file);
        }
        MiniBenchRead.result = (uint32_t)result;
    } else if (MiniBenchRead.operation == 2u) {
        DIR dir;
        FILINFO info;
        FRESULT result = f_opendir(&dir, "0:/");
        if (result == FR_OK) {
            uint32_t index = 0;
            while ((result = f_readdir(&dir, &info)) == FR_OK && info.fname[0] != 0) {
                if (index++ < MiniBenchRead.offset) continue;
                char line[128];
                int count = snprintf(line, sizeof(line), "%s,%lu\n", info.fname, (unsigned long)info.fsize);
                if (count <= 0 || (unsigned)count >= sizeof(line) ||
                    MiniBenchRead.count + (unsigned)count > length) break;
                for (int i = 0; i < count; i++) MiniBenchRead.data[MiniBenchRead.count+(unsigned)i] = (uint8_t)line[i];
                MiniBenchRead.count += (unsigned)count;
                MiniBenchRead.size = index;
            }
            (void)f_closedir(&dir);
        }
        MiniBenchRead.result = (uint32_t)result;
    }
    MiniBenchRead.request = 0;
}
#endif

static uint8_t SdLogHandleFlashPause(void)
{
#if defined(ARBATOS_BUILD_INFO_AUTOGEN_AVAILABLE)
    const volatile uint8_t *fingerprint = g_sdLogFlashBuildFingerprint;
    if (fingerprint[0] != ARBATOS_BUILD_FINGERPRINT_FIRST_BYTE)
    {
        return 0u;
    }
#endif
    if (g_sdLogFlashMailboxGuard != SDLOG_FLASH_MAILBOX_GUARD)
    {
        return 0u;
    }

    if (g_sdLogFlashPauseRequest == SDLOG_FLASH_PAUSE_REQUEST)
    {
        s_flash_pause_latched = 1u;
        if (g_sdLogFlashPauseReady == 0u)
        {
            g_sdLogFlashPauseReady = (SdLogQuiesce() == 0) ?
                SDLOG_FLASH_PAUSE_READY : SDLOG_FLASH_PAUSE_FAILED;
        }
    }

#if defined(CONFIG_ARBATOS_MINI_BENCH)
    if (s_flash_pause_latched && SdcardIsMounted()) SdLogBenchRead();
#endif
    return s_flash_pause_latched;
}

static uint8_t SdLogEntertainPause(void)
{
#if defined(__ZEPHYR__)
    /* 异常重启时优先保存证据，不受娱乐模式停日志的规则影响。 */
    if (RobotFaultZephyrBootLocked() != 0u) { return 0u; }
#endif
    return robot_mode_is_entertain();
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
    g_sdLogFlashPauseRequest = 0u;
    g_sdLogFlashPauseReady = 0u;

    // Wait for TF/SD ready (mount may be done by StartupServiceTask).
    uint32_t retry_ms = SDLOG_TASK_MOUNT_RETRY_START_MS;
    uint32_t next_start_ms = 0u;
    while (!SdcardIsMounted())
    {
        if (SdLogHandleFlashPause() != 0u)
        {
            osDelay(SDLOG_TASK_IDLE_DELAY_MS);
            continue;
        }

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
        if (SdLogHandleFlashPause() != 0u)
        {
            osDelay(SDLOG_TASK_IDLE_DELAY_MS);
            continue;
        }

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

        uint32_t backlog_polls = 0u;
        while (1)
        {
            SdLogStats stats = {0};

            if (SdLogHandleFlashPause() != 0u)
            {
                break;
            }

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
