/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef BSP_SERVO_PWM_H
#define BSP_SERVO_PWM_H
#include "Types.h"
#include "LowCmd.h"
#include "ServoConfig.h"

/*
 * cmd 的 PosVel.q 为 [-1,1] 归一化位置，0 为中点；物理角度由舵机行程标定。
 * timeoutMs 在本函数被任务调用时复核，不是独立于任务调度的硬件超时器。
 */
uint16_t ServoPwmApply(const ServoChannelConfig *channel,
                       MotorId id,
                       const MotorCmd *cmd,
                       const ControlOutputStamp *owner,
                       uint32_t nowMs);
void ServoPwmDisable(uint8_t port);

#endif
