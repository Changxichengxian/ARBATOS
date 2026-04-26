/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "usb_task.h"

#include "cmsis_os.h"

void usb_task(void const *argument)
{
    (void)argument;
    for (;;)
    {
        osDelay(1000);
    }
}

bool vision_take_latest(VisionToGimbal *out)
{
    (void)out;
    return false;
}

void vision_usb_rx_callback(uint8_t *buf, uint32_t len)
{
    (void)buf;
    (void)len;
}

