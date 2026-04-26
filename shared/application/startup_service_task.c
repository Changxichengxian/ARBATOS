/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */


#include "startup_service_task.h"
#include "cmsis_os2.h"
#include "bsp_buzzer.h"
#include "bsp_usb.h"
#include "config.h"
#include "detect_task.h"
#include "INS_task.h"
#include "referee.h"
#include "sdcard.h"

#define TEST_TASK_PERIOD_MS 2U
#define SOFT_BEEP_GAP_MS 100U
#define SOFT_BEEP_GAP_TICKS ((SOFT_BEEP_GAP_MS + TEST_TASK_PERIOD_MS - 1U) / TEST_TASK_PERIOD_MS)
#define SOFT_BEEP_PSC (g_config.buzzer.soft_beep_psc)
#define SOFT_BEEP_DURATION_MS (g_config.buzzer.soft_beep_duration_ms)

static void buzzer_schedule(uint8_t times);
static void buzzer_tick(void);
static uint8_t buzzer_is_idle(void);

const error_t *error_list_test_local;
static uint8_t beep_times_pending = 0;
static uint16_t beep_on_ticks_left = 0;
static uint16_t beep_gap_ticks_left = 0;

/**
  * @brief          startup service task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          test任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void startup_service_task(void const * argument)
{
    static uint8_t error, last_error;
    static uint8_t error_num;
    static uint8_t ever_all_online = 0;
    static uint8_t startup_beep_done = 0;
    static bool_t gyro_boot_cali_seen = 0;
    static bool_t gyro_boot_beep_done = 0;
    (void)argument;
    error_list_test_local = get_error_list_point();

    /* Start USB CDC device once at boot so the PC can enumerate */
    bsp_usb_device_init();

    /* Optional TF/SD card bring-up (SPI2 + FatFs) */
    (void)sdcard_mount();
    (void)sdcard_boot_mark();

    while(1)
    {
        error = 0;

        // find error
        for(error_num = 0; error_num < REFEREE_TOE; error_num++)
        {
            if(error_list_test_local[error_num].error_exist)
            {
                error = 1;
                break;
            }
        }

        if(error == 0)
        {
            ever_all_online = 1;
            if(last_error != 0 && buzzer_is_idle() != 0u)
            {
                buzzer_tone_stop();
            }
        }

        {
            uint8_t boot_missing = (error != 0 && ever_all_online == 0 && startup_beep_done == 0);
            uint8_t drop_when_running = (error != 0 && last_error == 0 && ever_all_online != 0);

            if((boot_missing || drop_when_running) && buzzer_is_idle() != 0u)
            {
                buzzer_schedule(1u);
                startup_beep_done = 1;
            }
        }

        if (gyro_boot_beep_done == 0)
        {
            const bool_t calibrating = ins_is_gyro_boot_calibrating();
            if (calibrating != 0)
            {
                gyro_boot_cali_seen = 1;
            }
            else if (gyro_boot_cali_seen != 0 && buzzer_is_idle() != 0u)
            {
                buzzer_schedule(ins_is_gyro_boot_calibrated() ? 1u : 2u);
                gyro_boot_beep_done = 1;
            }
        }

        buzzer_tick();
        referee_ui_demo_tick();
        last_error = error;
        osDelay(TEST_TASK_PERIOD_MS);
    }
}

/**
  * @brief          安排提示音次数
  */
static void buzzer_schedule(uint8_t times)
{
    if (times == 0u)
    {
        return;
    }

    if (beep_times_pending != 0u || beep_on_ticks_left != 0u || beep_gap_ticks_left != 0u)
    {
        return;
    }

    beep_times_pending = times;
}

static uint8_t buzzer_is_idle(void)
{
    return (beep_times_pending == 0u && beep_on_ticks_left == 0u && beep_gap_ticks_left == 0u) ? 1u : 0u;
}

/**
  * @brief          提示音序列推进
  */
static void buzzer_tick(void)
{
    if (beep_on_ticks_left != 0u)
    {
        beep_on_ticks_left--;
        if (beep_on_ticks_left == 0u)
        {
            buzzer_tone_stop();
            if (beep_times_pending != 0u)
            {
                beep_gap_ticks_left = SOFT_BEEP_GAP_TICKS;
                if (beep_gap_ticks_left == 0u)
                {
                    beep_gap_ticks_left = 1u;
                }
            }
        }
        return;
    }

    if (beep_gap_ticks_left != 0u)
    {
        beep_gap_ticks_left--;
        return;
    }

    if (beep_times_pending == 0u)
    {
        return;
    }

    buzzer_tone_start_legacy(SOFT_BEEP_PSC, buzzer_legacy_pwm_half());
    beep_times_pending--;

    {
        uint16_t duration_ticks = (uint16_t)((SOFT_BEEP_DURATION_MS + TEST_TASK_PERIOD_MS - 1U) / TEST_TASK_PERIOD_MS);
        if (duration_ticks == 0u)
        {
            duration_ticks = 1u;
        }
        beep_on_ticks_left = duration_ticks;
    }
}
