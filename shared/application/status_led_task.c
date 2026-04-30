/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */


#include "status_led_task.h"
#include "bsp_led.h"
#include "cmsis_os.h"
#include "detect_task.h"
#include "config.h"
#include "INS_task.h"

// 0xAA_RR_GG_BB: AA 为整体亮度，必须非 0 才能真正点亮
#define LED_COLOR_RED    0xFFFF0000U
#define LED_COLOR_GREEN  0xFF00FF00U
#define LED_COLOR_BLUE   0xFF0000FFU
#define LED_COLOR_WHITE  0xFFFFFFFFU
#define LED_COLOR_OFF    0x00000000U

/**
  * @brief          LED 状态巡视：依次对每个模块闪烁，颜色表示状态
  *                  - 绿：在线且无错误
  *                  - 蓝：在线但存在数据/子状态错误
  *                  - 红：离线/超时
  *                  每个模块占用 0.5s 时隙：亮 0.3s、灭 0.2s，然后立即切换下一个模块。
  */
void status_led_task(void const *argument)
{
    const error_t *errors = get_error_list_point();
    const uint16_t on_ms = g_config.led.slot_on_ms;
    const uint16_t off_ms = g_config.led.slot_off_ms;
    const uint16_t gap_ms = g_config.led.slot_gap_ms;

    while (1)
    {
        // 上电陀螺零偏采集中：三色全亮为白
        if (ins_is_gyro_boot_calibrating())
        {
            aRGB_led_show(LED_COLOR_WHITE);
            osDelay(on_ms);
            aRGB_led_show(LED_COLOR_OFF);
            osDelay(off_ms);
            continue;
        }

        // 额外指示：上电 2s 陀螺零偏采集状态（成功绿，失败/未完成红）
        {
            const uint32_t calib_color = ins_is_gyro_boot_calibrated() ? LED_COLOR_GREEN : LED_COLOR_RED;
            aRGB_led_show(calib_color);
            osDelay(on_ms);
            aRGB_led_show(LED_COLOR_OFF);
            osDelay(off_ms);
        }

        for (uint8_t toe = 0; toe < ERROR_LIST_LENGHT; toe++)
        {
            const error_t *e = &errors[toe];
            if (e->enable == 0)
            {
                continue;
            }

            uint32_t color = LED_COLOR_GREEN;

            if (e->is_lost)
            {
                color = LED_COLOR_RED;
            }
            else if (e->error_exist)
            {
                color = LED_COLOR_BLUE;
            }

            aRGB_led_show(color);
            osDelay(on_ms);
            aRGB_led_show(LED_COLOR_OFF);
            osDelay(off_ms);
        }

        // add a configurable dark gap between sweeps，便于分辨两轮
        aRGB_led_show(LED_COLOR_OFF);
        osDelay(gap_ms);
    }
}

