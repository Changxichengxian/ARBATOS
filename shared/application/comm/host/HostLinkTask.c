/*
 * SPDX-FileCopyrightText: 2026 Chen Xuan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "HostLinkTask.h"

#include "cmsis_os.h"
#if defined(__ZEPHYR__)
#include "RobotTargetConfig.h"
#endif
#include "AuxPort.h"
#include "AuxTelem.h"
#include "InsTask.h"
#if defined(ROBOT_SUBBOARD_RTC_SERVICE) && ROBOT_SUBBOARD_RTC_SERVICE
#include "SubBoardBringup.h"
#endif
#include "VisionLink.h"
#include "Watch.h"

void HostLinkTask(void const * argument)
{
    (void)argument;

    const fp32 *ins_quat = get_INS_quat_point();
    const fp32 *ins_angle = get_INS_angle_point();
    const fp32 *ins_gyro = get_gyro_data_point();
    const fp32 *ins_accel = get_accel_data_point();

    AuxTelemSetInsSources(ins_quat, ins_angle, ins_gyro, ins_accel);
    VisionLinkInit(ins_quat, ins_angle, ins_gyro);
    AuxPortInit();
#if defined(ROBOT_SUBBOARD_RTC_SERVICE) && ROBOT_SUBBOARD_RTC_SERVICE
    SubBoardBringupRunOnce();
#endif

    while (1)
    {
        WatchTaskBeat(WATCH_TASK_HOST_LINK);
        VisionLinkPollTx();
        AuxPortPoll();
#if defined(ROBOT_SUBBOARD_RTC_SERVICE) && ROBOT_SUBBOARD_RTC_SERVICE
        SubBoardBringupPoll();
#endif

        osDelay(2);
    }
}
