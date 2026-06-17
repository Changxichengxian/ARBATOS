/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "BuzzerFilePlayer.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "fatfs/ff.h"

#include "BspBuzzer.h"
#include "SdCard.h"

#define BUZZER_PCM_FILE_TASK_STACK_WORDS 512u
#ifndef BUZZER_PCM_FILE_READ_CHUNK
#if defined(STM32F407xx)
#define BUZZER_PCM_FILE_READ_CHUNK 2048u
#else
#define BUZZER_PCM_FILE_READ_CHUNK 4096u
#endif
#endif
#define BUZZER_PCM_FILE_PATH_MAX 256u
#define BUZZER_PCM_FILE_NOTIFY_PLAY (1u << 0)
#define BUZZER_PCM_FILE_NOTIFY_STOP (1u << 1)
#define BUZZER_MUSIC_LP_SHIFT_12K 3u
#define BUZZER_MUSIC_ENV_SHIFT_12K 7u

typedef struct
{
    char path[BUZZER_PCM_FILE_PATH_MAX];
    uint32_t sample_rate_hz;
    uint8_t loop;
    uint8_t volume;
} BuzzerPcmFileCmd;

static TaskHandle_t g_buzzer_pcm_file_task_handle = NULL;
static StaticTask_t g_buzzer_pcm_file_task_tcb;
static StackType_t g_buzzer_pcm_file_task_stack[BUZZER_PCM_FILE_TASK_STACK_WORDS];
static volatile int32_t g_buzzer_pcm_file_last_error = 0;
static BuzzerPcmFileCmd g_buzzer_pcm_file_cmd;
static uint8_t g_buzzer_pcm_file_read_buf[BUZZER_PCM_FILE_READ_CHUNK];
static uint8_t g_music_lp_shift = BUZZER_MUSIC_LP_SHIFT_12K;
static uint8_t g_music_env_shift = BUZZER_MUSIC_ENV_SHIFT_12K;
static int32_t g_music_lp = 0;
static int32_t g_music_env = 0;
static volatile uint8_t g_music_env_u8 = 0;

static void BuzzerMusicFilterConfig(uint32_t sample_rate_hz)
{
    if (sample_rate_hz >= 12000u)
    {
        g_music_lp_shift = 3u;
        g_music_env_shift = 7u;
    }
    else if (sample_rate_hz >= 8000u)
    {
        g_music_lp_shift = 2u;
        g_music_env_shift = 6u;
    }
    else
    {
        g_music_lp_shift = 1u;
        g_music_env_shift = 5u;
    }
}

void BuzzerPcmResetMusicEnv(void)
{
    taskENTER_CRITICAL();
    g_music_lp = 0;
    g_music_env = 0;
    g_music_env_u8 = 0u;
    taskEXIT_CRITICAL();
}

uint8_t BuzzerPcmGetMusicEnvU8(void)
{
    return g_music_env_u8;
}

static void BuzzerMusicProcessBuf(uint8_t *buf, uint32_t len)
{
    if (buf == NULL || len == 0u)
    {
        return;
    }

    for (uint32_t i = 0; i < len; i++)
    {
        const int32_t x = (int32_t)buf[i] - 128;
        g_music_lp += (x - g_music_lp) >> g_music_lp_shift;

        const int32_t hp = x - g_music_lp;
        int32_t y = hp + 128;
        if (y < 0)
        {
            y = 0;
        }
        else if (y > 255)
        {
            y = 255;
        }
        buf[i] = (uint8_t)y;

        const int32_t abs_lp = (g_music_lp >= 0) ? g_music_lp : -g_music_lp;
        g_music_env += (abs_lp - g_music_env) >> g_music_env_shift;
        if (g_music_env < 0)
        {
            g_music_env = 0;
        }

        uint32_t env_u32 = 0u;
        if (g_music_env > 127)
        {
            env_u32 = 127u;
        }
        else if (g_music_env > 0)
        {
            env_u32 = (uint32_t)g_music_env;
        }
        g_music_env_u8 = (uint8_t)(env_u32 << 1);
    }
}

static void BuzzerPcmFileTask(void *argument)
{
    (void)argument;

    for (;;)
    {
        uint32_t notify = 0u;
        if (xTaskNotifyWait(0u, 0xFFFFFFFFu, &notify, portMAX_DELAY) != pdPASS)
        {
            continue;
        }

        if ((notify & BUZZER_PCM_FILE_NOTIFY_STOP) != 0u)
        {
            BuzzerPcmStop();
        }
        if ((notify & BUZZER_PCM_FILE_NOTIFY_PLAY) == 0u)
        {
            continue;
        }

        for (;;)
        {
            BuzzerPcmFileCmd cmd;
            taskENTER_CRITICAL();
            memcpy(&cmd, &g_buzzer_pcm_file_cmd, sizeof(cmd));
            taskEXIT_CRITICAL();

            g_buzzer_pcm_file_last_error = 0;

            if (SdcardIsMounted() == 0)
            {
                const int m = SdcardMount();
                if (m != 0)
                {
                    g_buzzer_pcm_file_last_error = (int32_t)m;
                    break;
                }
            }

            FIL fp;
            FRESULT fr = f_open(&fp, cmd.path, FA_READ);
            if (fr != FR_OK)
            {
                g_buzzer_pcm_file_last_error = (int32_t)fr;
                break;
            }

            BuzzerMusicFilterConfig(cmd.sample_rate_hz);
            BuzzerPcmResetMusicEnv();

            const int start_res = BuzzerPcmStartStreamU8(cmd.sample_rate_hz, cmd.volume);
            if (start_res != 0)
            {
                g_buzzer_pcm_file_last_error = (int32_t)start_res;
                (void)f_close(&fp);
                break;
            }

            uint8_t want_restart = 0u;
            uint8_t want_stop = 0u;

            for (;;)
            {
                uint32_t ev = 0u;
                if (xTaskNotifyWait(0u, 0xFFFFFFFFu, &ev, 0u) == pdPASS)
                {
                    if ((ev & BUZZER_PCM_FILE_NOTIFY_STOP) != 0u)
                    {
                        want_stop = 1u;
                        break;
                    }
                    if ((ev & BUZZER_PCM_FILE_NOTIFY_PLAY) != 0u)
                    {
                        want_restart = 1u;
                        break;
                    }
                }

                if (BuzzerPcmIsRunning() == 0u || BuzzerPcmIsStreamMode() == 0u)
                {
                    break;
                }

                uint32_t free = BuzzerPcmStreamGetFree();
                if (free < 64u)
                {
                    vTaskDelay(pdMS_TO_TICKS(2));
                    continue;
                }

                uint32_t to_read = free;
                if (to_read > BUZZER_PCM_FILE_READ_CHUNK)
                {
                    to_read = BUZZER_PCM_FILE_READ_CHUNK;
                }

                UINT br = 0u;
                fr = f_read(&fp, g_buzzer_pcm_file_read_buf, (UINT)to_read, &br);
                if (fr != FR_OK)
                {
                    g_buzzer_pcm_file_last_error = (int32_t)fr;
                    break;
                }

                if (br == 0u)
                {
                    if (cmd.loop != 0u)
                    {
                        fr = f_lseek(&fp, 0u);
                        if (fr != FR_OK)
                        {
                            g_buzzer_pcm_file_last_error = (int32_t)fr;
                            break;
                        }
                        continue;
                    }

                    // Drain buffered samples, then stop.
                    while (BuzzerPcmIsRunning() != 0u && BuzzerPcmIsStreamMode() != 0u && BuzzerPcmStreamGetUsed() != 0u)
                    {
                        uint32_t ev2 = 0u;
                        if (xTaskNotifyWait(0u, 0xFFFFFFFFu, &ev2, 0u) == pdPASS)
                        {
                            if ((ev2 & BUZZER_PCM_FILE_NOTIFY_STOP) != 0u)
                            {
                                want_stop = 1u;
                                break;
                            }
                            if ((ev2 & BUZZER_PCM_FILE_NOTIFY_PLAY) != 0u)
                            {
                                want_restart = 1u;
                                break;
                            }
                        }
                        vTaskDelay(pdMS_TO_TICKS(10));
                    }
                    break;
                }

                BuzzerMusicProcessBuf(g_buzzer_pcm_file_read_buf, (uint32_t)br);

                uint32_t wrote = 0u;
                while (wrote < (uint32_t)br)
                {
                    if (BuzzerPcmIsRunning() == 0u || BuzzerPcmIsStreamMode() == 0u)
                    {
                        break;
                    }
                    uint32_t free2 = BuzzerPcmStreamGetFree();
                    if (free2 == 0u)
                    {
                        uint32_t ev3 = 0u;
                        if (xTaskNotifyWait(0u, 0xFFFFFFFFu, &ev3, 0u) == pdPASS)
                        {
                            if ((ev3 & BUZZER_PCM_FILE_NOTIFY_STOP) != 0u)
                            {
                                want_stop = 1u;
                                break;
                            }
                            if ((ev3 & BUZZER_PCM_FILE_NOTIFY_PLAY) != 0u)
                            {
                                want_restart = 1u;
                                break;
                            }
                        }

                        vTaskDelay(pdMS_TO_TICKS(1));
                        continue;
                    }

                    const uint32_t w = BuzzerPcmStreamWriteU8(&g_buzzer_pcm_file_read_buf[wrote], (uint32_t)br - wrote);
                    wrote += w;
                }

                if (want_restart != 0u || want_stop != 0u)
                {
                    break;
                }
            }

            (void)f_close(&fp);
            BuzzerPcmStop();
            BuzzerPcmResetMusicEnv();

            if (want_stop != 0u)
            {
                break;
            }
            if (want_restart != 0u)
            {
                continue;
            }

            break;
        }
    }
}

int BuzzerPcmPlayFileU8(const char *path, uint32_t sample_rate_hz, uint8_t loop, uint8_t volume)
{
    if (path == NULL || path[0] == '\0')
    {
        return -1;
    }
    if (sample_rate_hz == 0u)
    {
        return -1;
    }

    // Lazy-create file playback task so we don't need CubeMX to register a new thread.
    if (g_buzzer_pcm_file_task_handle == NULL)
    {
        g_buzzer_pcm_file_task_handle =
            xTaskCreateStatic(BuzzerPcmFileTask,
                              "BuzzerFile",
                              BUZZER_PCM_FILE_TASK_STACK_WORDS,
                              NULL,
                              tskIDLE_PRIORITY + 1,
                              g_buzzer_pcm_file_task_stack,
                              &g_buzzer_pcm_file_task_tcb);
        if (g_buzzer_pcm_file_task_handle == NULL)
        {
            g_buzzer_pcm_file_task_handle = NULL;
            return -2;
        }
    }

    // Copy command (keep it simple; ASCII path recommended by ffconf.h).
    BuzzerPcmFileCmd cmd = {0};
    (void)strncpy(cmd.path, path, BUZZER_PCM_FILE_PATH_MAX - 1u);
    cmd.sample_rate_hz = sample_rate_hz;
    cmd.loop = (loop != 0u) ? 1u : 0u;
    cmd.volume = volume;

    taskENTER_CRITICAL();
    memcpy(&g_buzzer_pcm_file_cmd, &cmd, sizeof(cmd));
    taskEXIT_CRITICAL();

    g_buzzer_pcm_file_last_error = 0;
    (void)xTaskNotify(g_buzzer_pcm_file_task_handle, BUZZER_PCM_FILE_NOTIFY_PLAY, eSetBits);
    return 0;
}

void BuzzerPcmPlayFileStop(void)
{
    if (g_buzzer_pcm_file_task_handle != NULL)
    {
        (void)xTaskNotify(g_buzzer_pcm_file_task_handle, BUZZER_PCM_FILE_NOTIFY_STOP, eSetBits);
    }
    BuzzerPcmStop();
    BuzzerPcmResetMusicEnv();
}

int32_t BuzzerPcmPlayFileLastError(void)
{
    return g_buzzer_pcm_file_last_error;
}
