/*
 * SPDX-FileCopyrightText: 2026 Chen Xuan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-06-25
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "HostLinkTask.h"

#include <string.h>

#include "cmsis_os.h"

#include "InsTask.h"
#include "SubBoardBringup.h"
#include "VisionLink.h"
#include "Watch.h"

void HostLinkTask(void const *argument)
{
    (void)argument;

    const fp32 *ins_quat = get_INS_quat_point();
    const fp32 *ins_angle = get_INS_angle_point();
    const fp32 *ins_gyro = get_gyro_data_point();

    VisionLinkInit(ins_quat, ins_angle, ins_gyro);
    SubBoardBringupRunOnce();

    for (;;)
    {
        WatchTaskBeat(WATCH_TASK_HOST_LINK);
        VisionLinkPollTx();
        SubBoardBringupPoll();
        osDelay(2);
    }
}

bool ImageRemoteGetState(ImageRemoteState *out)
{
    if (out != NULL)
    {
        memset(out, 0, sizeof(*out));
    }
    return false;
}

void ImageRemoteLinkGetStats(sdlog_image_link_stats_t *out)
{
    if (out != NULL)
    {
        memset(out, 0, sizeof(*out));
    }
}
