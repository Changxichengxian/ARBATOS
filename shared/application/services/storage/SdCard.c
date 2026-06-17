/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "SdCard.h"

#include <stdio.h>

#include "FreeRTOS.h"
#include "semphr.h"

#include "cmsis_os.h"
#include "BspTime.h"
#include "fatfs/ff.h"

static FATFS sd_fs;
static volatile uint8_t sd_mounted = 0u;

static SemaphoreHandle_t SdcardMutex = NULL;
static StaticSemaphore_t SdcardMutexBuf;

static void SdcardLock(void)
{
    taskENTER_CRITICAL();
    if (SdcardMutex == NULL)
    {
        SdcardMutex = xSemaphoreCreateMutexStatic(&SdcardMutexBuf);
    }
    taskEXIT_CRITICAL();

    if (SdcardMutex != NULL)
    {
        (void)xSemaphoreTake(SdcardMutex, portMAX_DELAY);
    }
}

static void SdcardUnlock(void)
{
    if (SdcardMutex != NULL)
    {
        (void)xSemaphoreGive(SdcardMutex);
    }
}

int SdcardMount(void)
{
    SdcardLock();

    if (sd_mounted)
    {
        SdcardUnlock();
        return 0;
    }

    const FRESULT res = f_mount(&sd_fs, "0:", 1);
    if (res == FR_OK)
    {
        sd_mounted = 1u;
        SdcardUnlock();
        return 0;
    }

    SdcardUnlock();
    return (int)res;
}

void SdcardUnmount(void)
{
    SdcardLock();

    sd_mounted = 0u;
    (void)f_mount(NULL, "0:", 0);

    SdcardUnlock();
}

int SdcardIsMounted(void)
{
    return (sd_mounted != 0u) ? 1 : 0;
}

int SdcardBootMark(void)
{
    SdcardLock();

    if (!sd_mounted)
    {
        SdcardUnlock();
        return -1;
    }

    FIL fp;
    const FRESULT open_res = f_open(&fp, "0:/boot.txt", FA_OPEN_APPEND | FA_WRITE);
    if (open_res != FR_OK)
    {
        SdcardUnlock();
        return (int)open_res;
    }

    char line[96];
    const uint32_t now = BspTimeGetTickMs();
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
    SdcardUnlock();
    return ret;
}
