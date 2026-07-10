/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "HostLinkTask.h"

#include <string.h>

#include "cmsis_os.h"

void HostLinkTask(void const *argument)
{
    (void)argument;
    for (;;)
    {
        osDelay(1000);
    }
}

bool VisionTakeLatest(VisionToGimbal *out)
{
    (void)out;
    return false;
}

bool ImageRemoteGetState(ImageRemoteState *out)
{
    if (out != NULL)
    {
        memset(out, 0, sizeof(*out));
    }
    return false;
}

void VisionLinkRxCallback(uint8_t *buf, uint32_t len)
{
    (void)buf;
    (void)len;
}

void ImageRemoteLinkGetStats(sdlog_image_link_stats_t *out)
{
    if (out != NULL)
    {
        memset(out, 0, sizeof(*out));
    }
}
