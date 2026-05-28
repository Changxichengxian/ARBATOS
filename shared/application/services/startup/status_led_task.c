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
#include "INS_task.h"

// 0xAA_RR_GG_BB: AA must be non-zero for the LED to light.
#define LED_COLOR_GREEN  0xFF00FF00U
#define LED_COLOR_WHITE  0xFFFFFFFFU

void status_led_task(void const *argument)
{
    (void)argument;

    while (1)
    {
        aRGB_led_show(ins_is_gyro_boot_calibrated() ? LED_COLOR_GREEN : LED_COLOR_WHITE);
        osDelay(100);
    }
}
