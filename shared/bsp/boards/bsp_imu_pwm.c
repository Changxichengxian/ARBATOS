/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

/**
  **************************(C) COPYRIGHT 2026 ARBATOS**************************
  * ARBATOS
  * Copyright (c) 2024-2026 陈轩 <2811158416@qq.com>
  * @brief      BSP：bsp_imu_pwm 源文件
  *
  * Repo: https://github.com/Changxichengxian/ARBATOS.git
  **************************(C) COPYRIGHT 2026 ARBATOS**************************
  */
#include "bsp_imu_pwm.h"
#include "main.h"
#include "bsp_imu_pwm_cfg.h"

#ifndef BSP_IMU_PWM_TIM_HANDLE
#error "BSP_IMU_PWM_TIM_HANDLE is not defined in bsp_imu_pwm_cfg.h"
#endif
#ifndef BSP_IMU_PWM_TIM_CHANNEL
#define BSP_IMU_PWM_TIM_CHANNEL TIM_CHANNEL_1
#endif

extern TIM_HandleTypeDef BSP_IMU_PWM_TIM_HANDLE;

void imu_pwm_set(uint16_t pwm)
{
    __HAL_TIM_SetCompare(&BSP_IMU_PWM_TIM_HANDLE, BSP_IMU_PWM_TIM_CHANNEL, pwm);
}
