/*
 * SPDX-FileCopyrightText: 2026 Chen Xuan <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 Chen Xuan <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "host_link_task.h"

#include "cmsis_os.h"
#include "aux_port.h"
#include "aux_telem.h"
#include "INS_task.h"
#include "vision_link.h"
#include "watch.h"

void host_link_task(void const * argument)
{
    (void)argument;

    const fp32 *ins_quat = get_INS_quat_point();
    const fp32 *ins_angle = get_INS_angle_point();
    const fp32 *ins_gyro = get_gyro_data_point();
    const fp32 *ins_accel = get_accel_data_point();

    aux_telem_set_ins_sources(ins_quat, ins_angle, ins_gyro, ins_accel);
    vision_link_init(ins_quat, ins_angle, ins_gyro);
    aux_port_init();

    while (1)
    {
        watch_task_beat(WATCH_TASK_HOST_LINK);
        vision_link_poll_tx();
        aux_port_poll();

        osDelay(2);
    }
}
