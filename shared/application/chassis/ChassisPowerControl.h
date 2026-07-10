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
  * @param[in]      activeMotorMask: axes allowed to share the power budget
  * @retval         none
  */
/**
  * @brief          限制功率，主要限制电机电流
  * @param[in]      ChassisPowerControl: 底盘数据
  * @param[in]      activeMotorMask: 允许参与共享功率预算的电机轴
  * @retval         none
  */
extern void ChassisPowerControl(ChassisMove *ChassisPowerControl, uint32_t activeMotorMask);
extern void ChassisPowerControlApplySpeedLimit(ChassisMove *ChassisPowerControl);

#endif
