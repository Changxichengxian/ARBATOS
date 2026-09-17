/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#pragma once

#include "RobotTargetConfig.h"
#define ROBOT_PROFILE_GIMBAL_ENCODER_FALLBACK_MASK 0x01u

// 实车顺时针转时 yaw 编码器增加；普通跟随、小陀螺及云台坐标转换统一反向。
#define CHASSIS_TRANSLATION_FRAME_REVERSE 1u

// A 板接收 9 路约 1 kHz 反馈，默认 200 us 预算实测会积压；沿用现有任务预算入口。
#define ROBOT_PROFILE_CAN_FEEDBACK_RX_BUDGET_US 500u
#define ROBOT_PROFILE_CAN_FEEDBACK_RX_PROFILE_BUDGET_US 600u

#define MOTOR_ARM_JOINT_COUNT 6u

#include "RobotConfigTypes.h"
