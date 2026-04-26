/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "bsp_laser.h"
#include "main.h"

#if defined(LASER_GPIO_Port) && defined(LASER_Pin)

void laser_on(void)
{
    HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
}

void laser_off(void)
{
    HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_RESET);
}

#else

extern TIM_HandleTypeDef htim3;
void laser_on(void)
{
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, 8399);
}
void laser_off(void)
{
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, 0);
}

#endif
