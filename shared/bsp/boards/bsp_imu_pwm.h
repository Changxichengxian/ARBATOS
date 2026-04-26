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
  * @brief      BSP：bsp_imu_pwm 头文件
  *
  * Repo: https://github.com/Changxichengxian/ARBATOS.git
  **************************(C) COPYRIGHT 2026 ARBATOS**************************
  */
#ifndef BSP_IMU_PWM_H
#define BSP_IMU_PWM_H
#include "struct_typedef.h"

// Board-specific timer mapping is configured in bsp_imu_pwm_cfg.h.

extern void imu_pwm_set(uint16_t pwm);

#endif
