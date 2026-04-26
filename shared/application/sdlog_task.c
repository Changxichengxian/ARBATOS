/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "sdlog_task.h"

#include "cmsis_os2.h"

#include "config.h"
#include "bsp_time.h"
#include "sdcard.h"
#include "sdlog.h"
#include "rt_profiler.h"

#define SDLOG_TASK_IDLE_DELAY_MS 10u
#define SDLOG_TASK_BACKLOG_YIELD_POLLS 8u
#define SDLOG_TASK_RT_PROFILER_PERIOD_MS 500u

static void sdlog_write_rt_profiler_sample(void)
{
    sdlog_rt_profiler_t sample = {0};
    const uint32_t count = ((uint32_t)RT_PROFILER_COUNT < (uint32_t)SDLOG_RT_PROFILER_MAX) ?
                               (uint32_t)RT_PROFILER_COUNT :
                               (uint32_t)SDLOG_RT_PROFILER_MAX;

    sample.count = (uint8_t)count;
    for (uint32_t i = 0u; i < count; i++)
    {
        rt_profiler_stats_t stats = {0};
        rt_profiler_get((rt_profiler_id_e)i, &stats);

        sample.entry[i].id = (uint8_t)i;
        sample.entry[i].count = stats.count;
        sample.entry[i].last_us = stats.last_us;
        sample.entry[i].max_us = stats.max_us;
        sample.entry[i].avg_us = stats.avg_us;
        sample.entry[i].budget_us = stats.budget_us;
        sample.entry[i].overrun_count = stats.overrun_count;
    }

    sdlog_write(SDLOG_TAG_RT_PROFILER, &sample, (uint16_t)sizeof(sample));
}

static void sdlog_wait_boot_delay_ms(uint32_t delay_ms)
{
    const uint32_t now_ms = bsp_time_get_tick_ms();
    if (now_ms >= delay_ms)
    {
        return;
    }
    osDelay(delay_ms - now_ms);
}

void sdlog_task(void const *argument)
{
    (void)argument;

    // Wait for TF/SD ready (mount may be done by startup_service_task).
    uint32_t retry_ms = 200u;
    while (!sdcard_is_mounted())
    {
        const int m = sdcard_mount();
        if (m == 0)
        {
            break;
        }

        osDelay(retry_ms);
        if (retry_ms < 2000u)
        {
            retry_ms *= 2u;
            if (retry_ms > 2000u)
            {
                retry_ms = 2000u;
            }
        }
    }

    if ((test_mode_e)g_config.test.mode != TEST_MODE_ENTERTAIN)
    {
        sdlog_wait_boot_delay_ms(2000u);
        (void)sdlog_start();
    }

    while (1)
    {
        if ((test_mode_e)g_config.test.mode == TEST_MODE_ENTERTAIN)
        {
            sdlog_stop();
            osDelay(SDLOG_TASK_IDLE_DELAY_MS);
            continue;
        }

        // If the log file was closed due to an error, try to reopen it.
        if (!sdlog_is_active() && sdcard_is_mounted())
        {
            sdlog_wait_boot_delay_ms(2000u);
            (void)sdlog_start();
        }

        if (!sdlog_is_active())
        {
            osDelay(SDLOG_TASK_IDLE_DELAY_MS);
            continue;
        }

        static uint32_t last_rt_profiler_log_ms = 0u;
        const uint32_t now_ms = bsp_time_get_tick_ms();
        if ((uint32_t)(now_ms - last_rt_profiler_log_ms) >= SDLOG_TASK_RT_PROFILER_PERIOD_MS)
        {
            last_rt_profiler_log_ms = now_ms;
            sdlog_write_rt_profiler_sample();
        }

        uint32_t backlog_polls = 0u;
        while (1)
        {
            sdlog_stats_t stats = {0};

            sdlog_poll();
            sdlog_get_stats(&stats);

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
