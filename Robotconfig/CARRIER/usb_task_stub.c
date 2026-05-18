/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "host_link_task.h"

#include <string.h>

#include "cmsis_os.h"

void host_link_task(void const *argument)
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

bool image_remote_get_state(image_remote_state_t *out)
{
    if (out != NULL)
    {
        memset(out, 0, sizeof(*out));
    }
    return false;
}

bool image_remote_auto_aim_requested(void)
{
    return false;
}

bool image_remote_aux_fire_requested(void)
{
    return false;
}

void vision_link_rx_callback(uint8_t *buf, uint32_t len)
{
    (void)buf;
    (void)len;
}

void image_remote_link_get_stats(sdlog_image_link_stats_t *out)
{
    if (out != NULL)
    {
        memset(out, 0, sizeof(*out));
    }
}
