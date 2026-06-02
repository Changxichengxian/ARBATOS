/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#pragma once

#define ARBATOS_TARGET_NAME "HERO-M"
#define ARBATOS_BOARD_NAME "DM_MC02_H7"
#define ROBOT_PROFILE_KIND ROBOT_PROFILE_KIND_HERO
#define ROBOT_BOARD_KIND ROBOT_BOARD_KIND_STM32H7
#define ROBOT_BOARD_CPU_HZ 500000000u
#define ROBOT_BOARD_CAN_BUS_COUNT 2u
#define ROBOT_BOARD_HAS_FPU 1u

#define MOTOR_ARM_JOINT_COUNT 6u

#include "robot_config_types.h"
