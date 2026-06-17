/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

/**
  **************************(C) COPYRIGHT 2026 ARBATOS**************************
  * ARBATOS
  * Copyright (c) 2024-2026 陈轩 <2811158416@qq.com>
  * @brief      BSP：时间基准（tick ms）实现
  *
  * Repo: https://github.com/Changxichengxian/ARBATOS.git
  **************************(C) COPYRIGHT 2026 ARBATOS**************************
  */
#include "BspTime.h"

#include "BspDwt.h"
#include "main.h"

uint32_t BspTimeGetTickMs(void)
{
    return HAL_GetTick();
}

uint32_t BspTimeGetTickUs(void)
{
    return (uint32_t)BSP_DWT_GetUs();
}
