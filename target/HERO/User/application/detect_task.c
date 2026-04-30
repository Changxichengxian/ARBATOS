/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */



#include "detect_task.h"
#include "cmsis_os.h"
#include "config.h"
#include "watch.h"
#include "sdlog.h"
#include "sdcard.h"
#include "bsp_buzzer.h"
#include "gimbal_behaviour.h"
#include "chassis_behaviour.h"
#include "cpu_usage.h"
#include "shoot.h"
#include "host_link_task.h"
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#if INCLUDE_uxTaskGetStackHighWaterMark
extern uint32_t gimbal_high_water;
extern uint32_t chassis_high_water;
extern uint32_t calibrate_task_stack;
#endif

extern volatile uint32_t gimbal_loop_counter;
extern volatile uint32_t chassis_loop_counter;
extern chassis_behaviour_e chassis_behaviour_mode;

#define WATCH_UPDATE_PERIOD_MS 100u


/**
  * @brief          init error_list, assign  offline_time, online_time, priority.
  * @param[in]      time: system time
  * @retval         none
  */
/**
  * @brief          初始化error_list,赋值 offline_time, online_time, priority
  * @param[in]      time:系统时间
  * @retval         none
  */
static void detect_init(uint32_t time);




error_t error_list[ERROR_LIST_LENGHT + 1];
static const detect_config_t *const detect_cfg = &g_config.detect;


#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t detect_task_stack;
#endif

static void sdlog_pack_detect_summary(sdlog_detect_summary_t *out, uint8_t display_toe)
{
    if (out == NULL)
    {
        return;
    }

    memset(out, 0, sizeof(*out));
    out->display_toe = (display_toe < ERROR_LIST_LENGHT) ? display_toe : 0xFFu;

    const uint8_t n = (uint8_t)ERROR_LIST_LENGHT;
    for (uint8_t i = 0u; i < n; i++)
    {
        const error_t *e = &error_list[i];
        const uint16_t bit = (uint16_t)(1u << i);

        if (e->enable != 0u)
        {
            out->enable_mask |= bit;
        }
        if (e->is_lost != 0u)
        {
            out->lost_mask |= bit;
            if (out->lost_count < 0xFFu)
            {
                out->lost_count++;
            }
        }
        if (e->data_is_error != 0u)
        {
            out->data_error_mask |= bit;
            if (out->data_error_count < 0xFFu)
            {
                out->data_error_count++;
            }
        }
        if (e->error_exist != 0u)
        {
            out->error_exist_mask |= bit;
            if (out->error_count < 0xFFu)
            {
                out->error_count++;
            }
        }
    }
}

void health_monitor_task(void const *pvParameters)
{
    detect_task(pvParameters);
}

static void sdlog_pack_control_summary(sdlog_control_summary_t *out)
{
    if (out == NULL)
    {
        return;
    }

    memset(out, 0, sizeof(*out));
    out->manual_source = g_watch.diag.manual_active_source;
    out->chassis_mode = (uint8_t)g_watch.chassis.mode;
    out->yaw_mode = (uint8_t)g_watch.gimbal.yaw_mode;
    out->pitch_mode = (uint8_t)g_watch.gimbal.pitch_mode;
    out->shoot_mode = (uint8_t)g_watch.shoot.mode;
    out->rc_s0 = (int8_t)g_watch.rc.s[0];
    out->rc_s1 = (int8_t)g_watch.rc.s[1];

    for (uint8_t i = 0u; i < 4u; i++)
    {
        out->rc_ch[i] = g_watch.rc.ch[i];
    }

    out->chassis_vx_set = g_watch.chassis.vx_set;
    out->chassis_vy_set = g_watch.chassis.vy_set;
    out->chassis_wz_set = g_watch.chassis.wz_set;
    out->chassis_vx = g_watch.chassis.vx;
    out->chassis_vy = g_watch.chassis.vy;
    out->chassis_wz = g_watch.chassis.wz;
    out->yaw_set_deg = g_watch.gimbal.yaw_set_deg;
    out->yaw_deg = g_watch.gimbal.yaw_angle_deg;
    out->pitch_set_deg = g_watch.gimbal.pitch_set_deg;
    out->pitch_deg = g_watch.gimbal.pitch_angle_deg;
    out->yaw_current = g_watch.gimbal.yaw_current;
    out->pitch_current = g_watch.gimbal.pitch_current;
    out->trigger_current = g_watch.shoot.trigger_current;
    out->fric_speed_set_rpm = g_watch.shoot.fric_speed_set_rpm;
}

static void sdlog_emit_event(uint16_t event_id, uint16_t arg0_u16, uint32_t arg1_u32, uint32_t arg2_u32)
{
    sdlog_event_t evt = {0};
    evt.event_id = event_id;
    evt.arg0_u16 = arg0_u16;
    evt.arg1_u32 = arg1_u32;
    evt.arg2_u32 = arg2_u32;
    sdlog_write(SDLOG_TAG_EVENT, &evt, (uint16_t)sizeof(evt));
}

/**
  * @brief          detect task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          检测任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void detect_task(void const *pvParameters)
{
    static uint32_t system_time;
    system_time = xTaskGetTickCount();
    //init,初始化
    detect_init(system_time);
    // 初始化调试观察指针（一次即可）
    watch_init();
    //wait a time.空闲一段时间
    vTaskDelay(DETECT_TASK_INIT_TIME);
    cpu_usage_init();
    (void)cpu_usage_get_permille();

    static sdlog_detect_summary_t detect_log;
    static uint8_t toe_last_lost[ERROR_LIST_LENGHT] = {0};
    static uint8_t toe_last_data_err[ERROR_LIST_LENGHT] = {0};
    static uint8_t config_buf[sizeof(sdlog_config_header_t) + sizeof(g_config)];

    uint8_t sd_mounted_last = (uint8_t)sdcard_is_mounted();
    uint8_t config_logged = 0u;
    uint32_t sdlog_dropped_last = 0u;
    uint32_t last_sys_stats_tick = system_time;
    uint32_t last_bsp_cfg_tick = system_time;
    uint32_t last_watch_snapshot_tick = system_time;

    uint8_t test_mode_last = (uint8_t)g_config.test.mode;
    uint8_t gimbal_behaviour_last = (uint8_t)gimbal_behaviour_watch;
    uint8_t chassis_behaviour_last = (uint8_t)chassis_behaviour_mode;
    const shoot_control_t *shoot_watch = get_shoot_control_point();
    uint8_t shoot_mode_last = (shoot_watch != NULL) ? (uint8_t)shoot_watch->shoot_mode : 0u;

    for (uint8_t i = 0u; i < (uint8_t)(sizeof(toe_last_lost) / sizeof(toe_last_lost[0])); i++)
    {
        toe_last_lost[i] = (uint8_t)error_list[i].is_lost;
        toe_last_data_err[i] = (uint8_t)error_list[i].data_is_error;
    }

    while (1)
    {
        static uint8_t error_num_display = 0;
        system_time = xTaskGetTickCount();

        error_num_display = ERROR_LIST_LENGHT;
        error_list[ERROR_LIST_LENGHT].is_lost = 0;
        error_list[ERROR_LIST_LENGHT].error_exist = 0;

        for (int i = 0; i < ERROR_LIST_LENGHT; i++)
        {
            //disable, continue
            //未使能，跳过
            if (error_list[i].enable == 0)
            {
                continue;
            }

            //judge offline.判断掉线
            if (system_time - error_list[i].new_time > error_list[i].set_offline_time)
            {
                //record error and time
                // mark as lost so LED shows red
                error_list[i].is_lost = 1;
                error_list[i].error_exist = 1;
                error_list[i].lost_time = system_time;
                //judge the priority,save the highest priority ,
                //判断错误优先级， 保存优先级最高的错误码
                if (error_list[i].priority > error_list[error_num_display].priority)
                {
                    error_num_display = i;
                }


                error_list[ERROR_LIST_LENGHT].is_lost = 1;
                error_list[ERROR_LIST_LENGHT].error_exist = 1;
                //if solve_lost_fun != NULL, run it
                //如果提供解决函数，运行解决函数
                if (error_list[i].solve_lost_fun != NULL)
                {
                    error_list[i].solve_lost_fun();
                }
            }
            else if (system_time - error_list[i].work_time < error_list[i].set_online_time)
            {
                //just online, maybe unstable, only record
                //刚刚上线，可能存在数据不稳定，只记录不丢失，
                error_list[i].is_lost = 0;
                error_list[i].error_exist = 1;
            }
            else
            {
                error_list[i].is_lost = 0;
                //判断是否存在数据错误
                //judge if exist data error
                if (error_list[i].data_is_error_fun != NULL)
                {
                    error_list[i].error_exist = 1;
                }
                else
                {
                    error_list[i].error_exist = 0;
                }
                //calc frequency
                //计算频率
                if (error_list[i].new_time > error_list[i].last_time)
                {
                    error_list[i].frequency = configTICK_RATE_HZ / (fp32)(error_list[i].new_time - error_list[i].last_time);
                }
            }
        }

        // Edge-triggered events for link/bus health.
        for (uint8_t i = 0u; i < (uint8_t)(sizeof(toe_last_lost) / sizeof(toe_last_lost[0])); i++)
        {
            const uint8_t lost_now = (uint8_t)error_list[i].is_lost;
            const uint8_t err_now = (uint8_t)error_list[i].data_is_error;

            if (lost_now && !toe_last_lost[i])
            {
                sdlog_emit_event(SDLOG_EVT_TOE_LOST, i, error_list[i].new_time, error_list[i].set_offline_time);
            }
            else if (!lost_now && toe_last_lost[i])
            {
                sdlog_emit_event(SDLOG_EVT_TOE_RECOVER, i, error_list[i].new_time, 0u);
            }

            if (err_now && !toe_last_data_err[i])
            {
                sdlog_emit_event(SDLOG_EVT_TOE_DATA_ERROR, i, error_list[i].new_time, 0u);
            }

            toe_last_lost[i] = lost_now;
            toe_last_data_err[i] = err_now;
        }

        if ((uint32_t)(system_time - last_watch_snapshot_tick) >= (uint32_t)pdMS_TO_TICKS(WATCH_UPDATE_PERIOD_MS))
        {
            last_watch_snapshot_tick = system_time;
            watch_update();

            sdlog_control_summary_t control_log = {0};
            sdlog_pack_control_summary(&control_log);
            sdlog_write(SDLOG_TAG_CONTROL_SUMMARY, &control_log, (uint16_t)sizeof(control_log));

            sdlog_pack_detect_summary(&detect_log, error_num_display);
            sdlog_write(SDLOG_TAG_DETECT_STATUS, &detect_log, (uint16_t)sizeof(detect_log));
        }

        // Periodic "application -> BSP" config sync (do not rely on SD log state).
        if ((uint32_t)(system_time - last_bsp_cfg_tick) >= (uint32_t)pdMS_TO_TICKS(1000u))
        {
            last_bsp_cfg_tick = system_time;

            uint8_t buz_en = 0u;
            uint32_t carrier_min_hz = 0u;
            uint16_t gain_q8 = 0u;
            taskENTER_CRITICAL();
            buz_en = g_config.buzzer.enable;
            carrier_min_hz = g_config.buzzer.pcm.carrier_min_hz;
            gain_q8 = g_config.buzzer.pcm.gain_q8;
            taskEXIT_CRITICAL();

            buzzer_set_enable(buz_en);
            buzzer_pcm_set_carrier_min_hz(carrier_min_hz);
            buzzer_pcm_set_stream_gain_q8(gain_q8);
        }

        const uint8_t test_mode_now = (uint8_t)g_config.test.mode;
        if (test_mode_now != test_mode_last)
        {
            sdlog_emit_event(SDLOG_EVT_TEST_MODE, test_mode_now, test_mode_last, 0u);
            test_mode_last = test_mode_now;
        }

        const uint8_t gimbal_behaviour_now = (uint8_t)gimbal_behaviour_watch;
        if (gimbal_behaviour_now != gimbal_behaviour_last)
        {
            sdlog_emit_event(SDLOG_EVT_GIMBAL_BEHAVIOUR, gimbal_behaviour_now, gimbal_behaviour_last, 0u);
            gimbal_behaviour_last = gimbal_behaviour_now;
        }

        const uint8_t chassis_behaviour_now = (uint8_t)chassis_behaviour_mode;
        if (chassis_behaviour_now != chassis_behaviour_last)
        {
            sdlog_emit_event(SDLOG_EVT_CHASSIS_BEHAVIOUR, chassis_behaviour_now, chassis_behaviour_last, 0u);
            chassis_behaviour_last = chassis_behaviour_now;
        }

        shoot_watch = get_shoot_control_point();
        const uint8_t shoot_mode_now = (shoot_watch != NULL) ? (uint8_t)shoot_watch->shoot_mode : 0u;
        if (shoot_mode_now != shoot_mode_last)
        {
            sdlog_emit_event(SDLOG_EVT_SHOOT_MODE, shoot_mode_now, shoot_mode_last, 0u);
            shoot_mode_last = shoot_mode_now;
        }

        // Log configuration snapshot once after boot (when SD log is active).
        if (!config_logged && sdlog_is_active())
        {
            const uint16_t cfg_size = (uint16_t)sizeof(config_buf);
            taskENTER_CRITICAL();
            sdlog_config_header_t *cfg_hdr = (sdlog_config_header_t *)config_buf;
            cfg_hdr->version = SDLOG_CONFIG_VERSION;
            cfg_hdr->header_size = (uint16_t)sizeof(*cfg_hdr);
            cfg_hdr->config_size = (uint16_t)sizeof(g_config);
            cfg_hdr->flags = 0u;
            memcpy(config_buf + sizeof(*cfg_hdr), &g_config, sizeof(g_config));
            taskEXIT_CRITICAL();
            sdlog_write(SDLOG_TAG_CONFIG, config_buf, cfg_size);
            config_logged = 1u;
        }

        // Periodic system resource / realtime stats.
        if ((uint32_t)(system_time - last_sys_stats_tick) >= (uint32_t)pdMS_TO_TICKS(100u))
        {
            last_sys_stats_tick = system_time;

            const uint8_t sd_mounted_now = (uint8_t)sdcard_is_mounted();
            if (sd_mounted_now != sd_mounted_last)
            {
                sd_mounted_last = sd_mounted_now;
                sdlog_emit_event(SDLOG_EVT_SD_CARD_MOUNT, sd_mounted_now, system_time, 0u);
            }

            sdlog_stats_t s = {0};
            sdlog_get_stats(&s);
            sdlog_image_link_stats_t image_stats = {0};
            image_remote_link_get_stats(&image_stats);

            sdlog_sys_stats_t sys = {0};
            sys.sd_mounted = sd_mounted_now;
            sys.sdlog_active = s.active;
            sys.sdlog_dropped = s.dropped;
            sys.sdlog_ring_used = s.ring_used;
            sys.sdlog_ring_free = s.ring_free;
            sys.sdlog_bytes_flushed = s.bytes_flushed;
            sys.sdlog_last_sync_ms = s.last_sync_ms;
            sys.sdlog_last_error = s.last_error;

            sys.heap_free = g_watch.rtos.heap_free;
            sys.heap_ever_free = g_watch.rtos.heap_ever_free;

#if INCLUDE_uxTaskGetStackHighWaterMark
            sys.stack_gimbal = gimbal_high_water;
            sys.stack_chassis = chassis_high_water;
            sys.stack_detect = detect_task_stack;
            sys.stack_calibrate = calibrate_task_stack;
#endif

            sys.gimbal_loop_cnt = gimbal_loop_counter;
            sys.chassis_loop_cnt = chassis_loop_counter;
            sys.cpu_load_permille = cpu_usage_get_permille();

            sdlog_write(SDLOG_TAG_SYS_STATS, &sys, (uint16_t)sizeof(sys));
            sdlog_write(SDLOG_TAG_IMAGE_LINK_STATS, &image_stats, (uint16_t)sizeof(image_stats));

            if (s.dropped != sdlog_dropped_last)
            {
                const uint32_t delta = s.dropped - sdlog_dropped_last;
                sdlog_dropped_last = s.dropped;
                sdlog_emit_event(SDLOG_EVT_SDLOG_DROPPED, 0u, s.dropped, delta);
            }
        }

        vTaskDelay(DETECT_CONTROL_TIME);
#if INCLUDE_uxTaskGetStackHighWaterMark
        detect_task_stack = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}


/**
  * @brief          get toe error status
  * @param[in]      toe: table of equipment
  * @retval         true (eror) or false (no error)
  */
/**
  * @brief          获取设备对应的错误状态
  * @param[in]      toe:设备目录
  * @retval         true(错误) 或者false(没错误)
  */
bool_t toe_is_error(uint8_t toe)
{
    return (error_list[toe].error_exist == 1);
}

/**
  * @brief          record the time
  * @param[in]      toe: table of equipment
  * @retval         none
  */
/**
  * @brief          记录时间
  * @param[in]      toe:设备目录
  * @retval         none
  */
void detect_hook(uint8_t toe)
{
    error_list[toe].last_time = error_list[toe].new_time;
    error_list[toe].new_time = xTaskGetTickCount();

    if (error_list[toe].is_lost)
    {
        error_list[toe].is_lost = 0;
        error_list[toe].work_time = error_list[toe].new_time;
    }

    if (error_list[toe].data_is_error_fun != NULL)
    {
        if (error_list[toe].data_is_error_fun())
        {
            error_list[toe].error_exist = 1;
            error_list[toe].data_is_error = 1;

            if (error_list[toe].solve_data_error_fun != NULL)
            {
                error_list[toe].solve_data_error_fun();
            }
        }
        else
        {
            error_list[toe].data_is_error = 0;
        }
    }
    else
    {
        error_list[toe].data_is_error = 0;
    }
}

/**
  * @brief          get error list
  * @param[in]      none
  * @retval         the point of error_list
  */
/**
  * @brief          得到错误列表
  * @param[in]      none
  * @retval         error_list的指针
  */
const error_t *get_error_list_point(void)
{
    return error_list;
}

static void detect_init(uint32_t time)
{
    //设置离线时间，上线稳定工作时间，优先级 offlineTime onlinetime priority
    for (uint8_t i = 0; i < ERROR_LIST_LENGHT; i++)
    {
        error_list[i].set_offline_time = detect_cfg->items[i].offline_time_ms;
        error_list[i].set_online_time = detect_cfg->items[i].online_time_ms;
        error_list[i].priority = detect_cfg->items[i].priority;
        error_list[i].data_is_error_fun = NULL;
        error_list[i].solve_lost_fun = NULL;
        error_list[i].solve_data_error_fun = NULL;

        error_list[i].enable = (detect_cfg->enable_mask >> i) & 0x1U;
        error_list[i].error_exist = error_list[i].enable ? 1 : 0;
        error_list[i].is_lost = error_list[i].enable ? 1 : 0;
        error_list[i].data_is_error = error_list[i].enable ? 1 : 0;
        error_list[i].frequency = 0.0f;
        error_list[i].new_time = time;
        error_list[i].last_time = time;
        error_list[i].lost_time = time;
        error_list[i].work_time = time;
    }

//    error_list[DBUSTOE].dataIsErrorFun = RC_data_is_error;
//    error_list[DBUSTOE].solveLostFun = slove_RC_lost;
//    error_list[DBUSTOE].solveDataErrorFun = slove_data_error;

}
