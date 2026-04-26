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
  * @brief      BSP：射击微动开关（触发开关）输入实现
  *
  * Repo: https://github.com/Changxichengxian/ARBATOS.git
  **************************(C) COPYRIGHT 2026 ARBATOS**************************
  */
#include "bsp_shoot_trig.h"

#include "main.h"

bool_t bsp_shoot_trig_read_raw(uint8_t *out_level)
{
    if (out_level == NULL)
    {
        return 0u;
    }

#if defined(BUTTON_TRIG_GPIO_Port) && defined(BUTTON_TRIG_Pin)
    *out_level = (uint8_t)HAL_GPIO_ReadPin(BUTTON_TRIG_GPIO_Port, BUTTON_TRIG_Pin);
    return 1u;
#else
    // This board/target does not expose the trigger micro-switch GPIO.
    *out_level = 0u;
    return 0u;
#endif
}
