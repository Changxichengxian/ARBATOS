/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "sdcard.h"

#include <stdio.h>

#include "FreeRTOS.h"
#include "semphr.h"

#include "cmsis_os.h"
#include "bsp_time.h"
#include "fatfs/ff.h"

static FATFS sd_fs;
static volatile uint8_t sd_mounted = 0u;

static SemaphoreHandle_t sdcard_mutex = NULL;
static StaticSemaphore_t sdcard_mutex_buf;

static void sdcard_lock(void)
{
    taskENTER_CRITICAL();
    if (sdcard_mutex == NULL)
    {
        sdcard_mutex = xSemaphoreCreateMutexStatic(&sdcard_mutex_buf);
    }
    taskEXIT_CRITICAL();

    if (sdcard_mutex != NULL)
    {
        (void)xSemaphoreTake(sdcard_mutex, portMAX_DELAY);
    }
}

static void sdcard_unlock(void)
{
    if (sdcard_mutex != NULL)
    {
        (void)xSemaphoreGive(sdcard_mutex);
    }
}

int sdcard_mount(void)
{
    sdcard_lock();

    if (sd_mounted)
    {
        sdcard_unlock();
        return 0;
    }

    const FRESULT res = f_mount(&sd_fs, "0:", 1);
    if (res == FR_OK)
    {
        sd_mounted = 1u;
        sdcard_unlock();
        return 0;
    }

    sdcard_unlock();
    return (int)res;
}

int sdcard_is_mounted(void)
{
    return (sd_mounted != 0u) ? 1 : 0;
}

int sdcard_boot_mark(void)
{
    sdcard_lock();

    if (!sd_mounted)
    {
        sdcard_unlock();
        return -1;
    }

    FIL fp;
    const FRESULT open_res = f_open(&fp, "0:/boot.txt", FA_OPEN_APPEND | FA_WRITE);
    if (open_res != FR_OK)
    {
        sdcard_unlock();
        return (int)open_res;
    }

    char line[96];
    const uint32_t now = bsp_time_get_tick_ms();
    const int n = snprintf(line,
                           sizeof(line),
                           "boot tick=%lu heap_free=%lu heap_min=%lu\r\n",
                           (unsigned long)now,
                           (unsigned long)xPortGetFreeHeapSize(),
                           (unsigned long)xPortGetMinimumEverFreeHeapSize());
    const UINT to_write = (n > 0) ? (UINT)((n < (int)sizeof(line)) ? n : (int)sizeof(line) - 1) : 0u;

    UINT bw = 0u;
    const FRESULT wr_res = (to_write != 0u) ? f_write(&fp, line, to_write, &bw) : FR_OK;
    (void)f_sync(&fp);
    (void)f_close(&fp);

    const int ret = (wr_res == FR_OK && bw == to_write) ? 0 : -2;
    sdcard_unlock();
    return ret;
}
