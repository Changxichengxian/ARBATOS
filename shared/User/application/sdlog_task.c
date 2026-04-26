/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "sdlog_task.h"

#include "cmsis_os.h"

#include "app_config.h"
#include "bsp_time.h"
#include "sdcard.h"
#include "sdlog.h"

#define SDLOG_TASK_IDLE_DELAY_MS 10u
#define SDLOG_TASK_BACKLOG_YIELD_POLLS 8u

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

    // Wait for TF/SD ready (mount may be done by test_task).
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

    if ((test_mode_e)g_app_config.test.mode != TEST_MODE_ENTERTAIN)
    {
        sdlog_wait_boot_delay_ms(2000u);
        (void)sdlog_start();
    }

    while (1)
    {
        if ((test_mode_e)g_app_config.test.mode == TEST_MODE_ENTERTAIN)
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
