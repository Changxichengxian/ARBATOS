/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */


#ifndef CHASSIS_POWER_CONTROL_H
#define CHASSIS_POWER_CONTROL_H
#include "ChassisControlTask.h"
#include "main.h"

/**
  * @brief          limit the power, mainly limit motor current
  * @param[in]      ChassisPowerControl: chassis data
  * @retval         none
  */
/**
  * @brief          限制功率，主要限制电机电流
  * @param[in]      ChassisPowerControl: 底盘数据
 * @retval         none
  */
extern void ChassisPowerControl(ChassisMove *ChassisPowerControl);
extern void ChassisPowerControlApplySpeedLimit(ChassisMove *ChassisPowerControl);

#endif
