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
  * @brief      BSP：USB 设备初始化（MX_USB_DEVICE_Init 封装）
  *
  * Repo: https://github.com/Changxichengxian/ARBATOS.git
  **************************(C) COPYRIGHT 2026 ARBATOS**************************
  */
#include "bsp_usb.h"

#include <stdint.h>

#include "usb_device.h"

static uint8_t s_usb_inited = 0u;

void bsp_usb_device_init(void)
{
    if (s_usb_inited != 0u)
    {
        return;
    }
    s_usb_inited = 1u;
    MX_USB_DEVICE_Init();
}
