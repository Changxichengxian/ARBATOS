/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "bsp_servo_pwm.h"
#include "main.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;

void servo_pwm_set(uint16_t pwm, uint8_t i)
{
    switch(i)
    {
        case 0:
        {
            __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwm);
        }break;
        case 1:
        {
            __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwm);
        }break;
        case 2:
        {
            __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, pwm);
        }break;
        case 3:
        {
            __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, pwm);
        }break;
    }
}
