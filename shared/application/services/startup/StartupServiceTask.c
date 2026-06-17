/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */


#include "StartupServiceTask.h"

#include <stdio.h>
#include <string.h>

#include "cmsis_os.h"
#include "fatfs/ff.h"
#include "BspBuzzer.h"
#include "BspTime.h"
#include "BspUsb.h"
#include "BuzzerFilePlayer.h"
#include "config.h"
#include "ControlInput.h"
#include "DetectTask.h"
#include "ImageRemoteLink.h"
#include "ManualInput.h"
#include "Referee.h"
#include "SdCard.h"
#include "RobotMode.h"

#define TEST_TASK_PERIOD_MS 2U
#define SOFT_BEEP_GAP_MS 100U
#define SOFT_BEEP_GAP_TICKS ((SOFT_BEEP_GAP_MS + TEST_TASK_PERIOD_MS - 1U) / TEST_TASK_PERIOD_MS)
#define SOFT_BEEP_PSC (g_config.buzzer.soft_beep_psc)
#define SOFT_BEEP_DURATION_MS (g_config.buzzer.soft_beep_duration_ms)
#define LOST_BEEP_CONFIRM_COUNT 5u
#define LOST_BEEP_CONFIRM_STEP_TICKS ((DETECT_CONTROL_TIME + TEST_TASK_PERIOD_MS - 1U) / TEST_TASK_PERIOD_MS)
#define STARTUP_LOST_BEEP_MUTE_MS 3500U
#define STARTUP_LOST_BEEP_MUTE_TICKS ((STARTUP_LOST_BEEP_MUTE_MS + TEST_TASK_PERIOD_MS - 1U) / TEST_TASK_PERIOD_MS)
#define ENTERTAIN_MUSIC_PATH_MAX 256u
#define ENTERTAIN_MUSIC_ROOT "0:/"

static uint8_t lost_beep_confirm_due(void);
static void BuzzerSchedule(uint8_t times);
static void BuzzerTick(void);
static uint8_t BuzzerIsIdle(void);
static void entertainment_music_tick(void);
static uint8_t entertainment_manual_connected(void);
static uint8_t entertainment_switch_is_ready(uint16_t raw_sw);
static uint8_t entertainment_music_name_is_u8(const char *name);
static int entertainment_find_music_by_index(int32_t *index_io,
                                             char *out,
                                             uint32_t out_size,
                                             uint32_t *count_out);

const DetectError *error_list_test_local;
static uint8_t beep_times_pending = 0;
static uint16_t beep_on_ticks_left = 0;
static uint16_t beep_gap_ticks_left = 0;
static uint8_t lost_confirm_step_ticks[DETECT_ERROR_COUNT];
static uint8_t lost_confirm_count[DETECT_ERROR_COUNT];
static uint8_t lost_beeped[DETECT_ERROR_COUNT];

static uint16_t entertainment_switch_raw_from_pos(uint8_t pos)
{
    return (uint16_t)input_switch_pos_to_raw(pos);
}

static uint8_t entertainment_switch_is_stop(uint16_t raw_sw)
{
    return input_switch_is_pos(raw_sw, g_config.manual_input.semantics.ShootStopPos);
}

static uint8_t entertainment_switch_is_ready(uint16_t raw_sw)
{
    return input_switch_is_pos(raw_sw, g_config.manual_input.semantics.ShootReadyPos);
}

static input_switch_e entertainment_get_image_switch_input(void)
{
    switch (g_config.manual_input.semantics.image_vt13_shoot_switch_input)
    {
    case MANUAL_INPUT_IMAGE_SWITCH_CHASSIS:
        return INPUT_SW_CHASSIS_MODE;
    case MANUAL_INPUT_IMAGE_SWITCH_GIMBAL:
        return INPUT_SW_GIMBAL_MODE;
    case MANUAL_INPUT_IMAGE_SWITCH_SHOOT:
    default:
        return INPUT_SW_SHOOT_MODE;
    }
}

static uint16_t entertainment_get_raw_switch(void)
{
    uint16_t raw_sw = (uint16_t)input_switch(INPUT_SW_SHOOT_MODE);

    if (remote_control_get_active_source() != MANUAL_INPUT_SRC_IMAGE)
    {
        return raw_sw;
    }

    ImageRemoteState image_state;
    if (!ImageRemoteGetState(&image_state) ||
        image_state.proto != SDLOG_MANUAL_INPUT_PROTO_IMAGE_VT13)
    {
        return raw_sw;
    }

    raw_sw = (uint16_t)input_switch(entertainment_get_image_switch_input());
    if (entertainment_switch_is_stop(raw_sw))
    {
        return entertainment_switch_raw_from_pos(g_config.manual_input.semantics.ShootStopPos);
    }
    if (entertainment_switch_is_ready(raw_sw))
    {
        return entertainment_switch_raw_from_pos(g_config.manual_input.semantics.ShootReadyPos);
    }
    return entertainment_switch_raw_from_pos(g_config.manual_input.semantics.ShootFirePos);
}

static uint8_t entertainment_manual_connected(void)
{
    return (remote_control_get_active_source() != MANUAL_INPUT_SRC_AUTO) ? 1u : 0u;
}

static char entertainment_ascii_upper(char c)
{
    if (c >= 'a' && c <= 'z')
    {
        return (char)(c - ('a' - 'A'));
    }
    return c;
}

static uint8_t entertainment_music_name_is_u8(const char *name)
{
    const uint32_t len = (name != NULL) ? (uint32_t)strlen(name) : 0u;

    if (len < 4u)
    {
        return 0u;
    }

    return (uint8_t)(name[len - 3u] == '.' &&
                     entertainment_ascii_upper(name[len - 2u]) == 'U' &&
                     entertainment_ascii_upper(name[len - 1u]) == '8');
}

static int entertainment_music_scan_count(uint32_t *count_out)
{
    DIR dir;
    FILINFO info;
    FRESULT fr;
    uint32_t count = 0u;

    if (count_out == NULL)
    {
        return -1;
    }

    if (SdcardIsMounted() == 0)
    {
        const int m = SdcardMount();
        if (m != 0)
        {
            return m;
        }
    }

    fr = f_opendir(&dir, ENTERTAIN_MUSIC_ROOT);
    if (fr != FR_OK)
    {
        return -(int)fr;
    }

    for (;;)
    {
        fr = f_readdir(&dir, &info);
        if (fr != FR_OK)
        {
            (void)f_closedir(&dir);
            return -(int)fr;
        }
        if (info.fname[0] == '\0')
        {
            break;
        }
        if ((info.fattrib & AM_DIR) == 0u && entertainment_music_name_is_u8(info.fname) != 0u)
        {
            count++;
        }
    }

    (void)f_closedir(&dir);
    *count_out = count;
    return (count != 0u) ? 0 : -2;
}

static int entertainment_find_music_by_index(int32_t *index_io,
                                             char *out,
                                             uint32_t out_size,
                                             uint32_t *count_out)
{
    DIR dir;
    FILINFO info;
    FRESULT fr;
    uint32_t count = 0u;
    uint32_t seen = 0u;
    int32_t index;

    if (index_io == NULL || out == NULL || out_size == 0u)
    {
        return -1;
    }

    const int count_res = entertainment_music_scan_count(&count);
    if (count_res != 0)
    {
        if (count_out != NULL)
        {
            *count_out = 0u;
        }
        return count_res;
    }

    index = *index_io;
    while (index < 0)
    {
        index += (int32_t)count;
    }
    index %= (int32_t)count;

    fr = f_opendir(&dir, ENTERTAIN_MUSIC_ROOT);
    if (fr != FR_OK)
    {
        return -(int)fr;
    }

    for (;;)
    {
        fr = f_readdir(&dir, &info);
        if (fr != FR_OK)
        {
            (void)f_closedir(&dir);
            return -(int)fr;
        }
        if (info.fname[0] == '\0')
        {
            break;
        }
        if ((info.fattrib & AM_DIR) != 0u || entertainment_music_name_is_u8(info.fname) == 0u)
        {
            continue;
        }
        if (seen == (uint32_t)index)
        {
            const int n = snprintf(out, (size_t)out_size, "%s%s", ENTERTAIN_MUSIC_ROOT, info.fname);
            (void)f_closedir(&dir);
            if (n <= 0 || (uint32_t)n >= out_size)
            {
                return -3;
            }
            *index_io = index;
            if (count_out != NULL)
            {
                *count_out = count;
            }
            return 0;
        }
        seen++;
    }

    (void)f_closedir(&dir);
    return -4;
}

static void entertainment_music_tick(void)
{
    static uint8_t last_mode_entertain = 0u;
    static uint8_t last_manual_connected = 0u;
    static uint16_t last_music_sw = 0u;
    static uint32_t last_start_ms = 0u;
    static int32_t selected_index = 0;
    static char want_path[ENTERTAIN_MUSIC_PATH_MAX] = {0};
    static char last_cmd_path[ENTERTAIN_MUSIC_PATH_MAX] = {0};
    uint8_t reload_path = 0u;

    if (robot_mode_is_entertain() == 0u)
    {
        if (last_mode_entertain != 0u && BuzzerPcmIsStreamMode() != 0u)
        {
            BuzzerPcmPlayFileStop();
        }
        last_mode_entertain = 0u;
        last_manual_connected = 0u;
        last_music_sw = 0u;
        selected_index = 0;
        want_path[0] = '\0';
        last_cmd_path[0] = '\0';
        last_start_ms = 0u;
        return;
    }

    last_mode_entertain = 1u;

    const uint16_t music_sw = entertainment_get_raw_switch();
    const uint8_t manual_connected = entertainment_manual_connected();
    const BuzzerPcmConfig *pcm_cfg = &g_config.buzzer.pcm;
    const uint32_t sample_rate_hz = (pcm_cfg->sample_rate_hz != 0u) ? pcm_cfg->sample_rate_hz : 12000u;
    const uint8_t volume = pcm_cfg->volume;
    const uint8_t loop = (pcm_cfg->loop != 0u) ? 1u : 0u;
    const uint16_t retry_ms = (pcm_cfg->retry_ms != 0u) ? pcm_cfg->retry_ms : 500u;
    const uint32_t now_ms = BspTimeGetTickMs();

    if (manual_connected == 0u)
    {
        if (BuzzerPcmIsStreamMode() != 0u)
        {
            BuzzerPcmPlayFileStop();
        }
        last_manual_connected = 0u;
        last_music_sw = 0u;
        want_path[0] = '\0';
        last_cmd_path[0] = '\0';
        last_start_ms = 0u;
        return;
    }

    if (last_manual_connected == 0u)
    {
        reload_path = 1u;
    }
    if (last_music_sw == RC_SW_MID && music_sw == RC_SW_DOWN)
    {
        selected_index++;
        reload_path = 1u;
    }
    else if (last_music_sw == RC_SW_MID && music_sw == RC_SW_UP)
    {
        selected_index--;
        reload_path = 1u;
    }
    if (want_path[0] == '\0' && (uint32_t)(now_ms - last_start_ms) >= (uint32_t)retry_ms)
    {
        reload_path = 1u;
    }

    last_manual_connected = 1u;
    last_music_sw = music_sw;

    if (reload_path != 0u)
    {
        char tmp[ENTERTAIN_MUSIC_PATH_MAX] = {0};
        if (entertainment_find_music_by_index(&selected_index, tmp, (uint32_t)sizeof(tmp), NULL) != 0)
        {
            want_path[0] = '\0';
            last_cmd_path[0] = '\0';
            last_start_ms = now_ms;
            if (BuzzerPcmIsStreamMode() != 0u)
            {
                BuzzerPcmPlayFileStop();
            }
            return;
        }

        (void)strncpy(want_path, tmp, sizeof(want_path) - 1u);
        want_path[sizeof(want_path) - 1u] = '\0';
    }

    if (BuzzerPcmIsStreamMode() != 0u)
    {
        if (strcmp(last_cmd_path, want_path) != 0)
        {
            (void)BuzzerPcmPlayFileU8(want_path, sample_rate_hz, loop, volume);
            (void)strncpy(last_cmd_path, want_path, sizeof(last_cmd_path) - 1u);
            last_cmd_path[sizeof(last_cmd_path) - 1u] = '\0';
            last_start_ms = BspTimeGetTickMs();
        }
        return;
    }

    if (BuzzerPcmIsRunning() != 0u)
    {
        return;
    }

    if (strcmp(last_cmd_path, want_path) != 0 || (uint32_t)(now_ms - last_start_ms) >= (uint32_t)retry_ms)
    {
        (void)BuzzerPcmPlayFileU8(want_path, sample_rate_hz, loop, volume);
        (void)strncpy(last_cmd_path, want_path, sizeof(last_cmd_path) - 1u);
        last_cmd_path[sizeof(last_cmd_path) - 1u] = '\0';
        last_start_ms = now_ms;
    }
}

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
void StartupServiceTask(void const * argument)
{
    static uint8_t error, last_error;
    static uint8_t error_num;
    static uint8_t ever_all_online = 0;
    static uint8_t startup_beep_done = 0;
    static uint16_t startup_lost_beep_mute_ticks = STARTUP_LOST_BEEP_MUTE_TICKS;
    (void)argument;
    error_list_test_local = get_error_list_point();

    /* Start USB CDC device once at boot so the PC can enumerate */
    BspUsbDeviceInit();

    /* Optional TF/SD card bring-up (SPI2 + FatFs) */
    (void)SdcardMount();
    (void)SdcardBootMark();

    while(1)
    {
        error = 0;

        // find error
        for(error_num = 0; error_num < REFEREE_TOE; error_num++)
        {
            if(error_list_test_local[error_num].is_lost)
            {
                error = 1;
                break;
            }
        }

        const uint8_t confirmed_lost = lost_beep_confirm_due();
        if (startup_lost_beep_mute_ticks != 0u)
        {
            startup_lost_beep_mute_ticks--;
        }

        const uint8_t entertainment_mode = robot_mode_is_entertain();
        entertainment_music_tick();

        if(error == 0)
        {
            ever_all_online = 1;
            if(last_error != 0 && BuzzerIsIdle() != 0u)
            {
                BuzzerToneStop();
            }
        }

        {
            uint8_t boot_missing =
                (confirmed_lost != 0u && error != 0 && ever_all_online == 0 && startup_beep_done == 0);
            uint8_t drop_when_running =
                (confirmed_lost != 0u && error != 0 && ever_all_online != 0);

            if(entertainment_mode == 0u &&
               startup_lost_beep_mute_ticks == 0u &&
               (boot_missing || drop_when_running) &&
               BuzzerIsIdle() != 0u)
            {
                BuzzerSchedule(1u);
                startup_beep_done = 1;
            }
        }

        if (entertainment_mode != 0u)
        {
            beep_times_pending = 0u;
            beep_gap_ticks_left = 0u;
            if (beep_on_ticks_left != 0u && BuzzerPcmIsRunning() == 0u)
            {
                BuzzerToneStop();
            }
            beep_on_ticks_left = 0u;
        }
        else
        {
            BuzzerTick();
        }
        RefereeUiDemoTick();
        last_error = error;
        osDelay(TEST_TASK_PERIOD_MS);
    }
}

/**
  * @brief          安排提示音次数
  */
static void BuzzerSchedule(uint8_t times)
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

static uint8_t BuzzerIsIdle(void)
{
    return (beep_times_pending == 0u &&
            beep_on_ticks_left == 0u &&
            beep_gap_ticks_left == 0u &&
            BuzzerPcmIsRunning() == 0u) ? 1u : 0u;
}

static uint8_t lost_beep_confirm_due(void)
{
    uint8_t due = 0u;
    const uint8_t toe_limit = ((uint8_t)REFEREE_TOE < (uint8_t)DETECT_ERROR_COUNT) ?
        (uint8_t)REFEREE_TOE :
        (uint8_t)DETECT_ERROR_COUNT;

    if (error_list_test_local == NULL)
    {
        return 0u;
    }

    for (uint8_t toe = 0u; toe < toe_limit; toe++)
    {
        if (error_list_test_local[toe].is_lost != 0u)
        {
            const uint8_t step_ticks =
                (LOST_BEEP_CONFIRM_STEP_TICKS > 0u) ? (uint8_t)LOST_BEEP_CONFIRM_STEP_TICKS : 1u;

            if (lost_confirm_step_ticks[toe] < step_ticks)
            {
                lost_confirm_step_ticks[toe]++;
            }

            if (lost_confirm_step_ticks[toe] >= step_ticks)
            {
                lost_confirm_step_ticks[toe] = 0u;
                if (lost_confirm_count[toe] < LOST_BEEP_CONFIRM_COUNT)
                {
                    lost_confirm_count[toe]++;
                }
            }

            if (lost_confirm_count[toe] >= LOST_BEEP_CONFIRM_COUNT && lost_beeped[toe] == 0u)
            {
                lost_beeped[toe] = 1u;
                due = 1u;
            }
        }
        else
        {
            lost_confirm_step_ticks[toe] = 0u;
            lost_confirm_count[toe] = 0u;
            lost_beeped[toe] = 0u;
        }
    }

    return due;
}

/**
  * @brief          提示音序列推进
  */
static void BuzzerTick(void)
{
    if (beep_on_ticks_left != 0u)
    {
        beep_on_ticks_left--;
        if (beep_on_ticks_left == 0u)
        {
            BuzzerToneStop();
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

    if (BuzzerPcmIsRunning() != 0u)
    {
        return;
    }

    BuzzerToneStartLegacy(SOFT_BEEP_PSC, BuzzerLegacyPwmHalf());
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
