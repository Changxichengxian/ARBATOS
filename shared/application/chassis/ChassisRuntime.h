/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-07-10
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef CHASSIS_RUNTIME_H
#define CHASSIS_RUNTIME_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 这些入口只供 ChassisCtrl 适配层使用。任务和注册表不得绕过
 * ControlMgr 直接调用，避免底盘同时存在两套生命周期。
 */
void ChassisRuntimeInit(void);
void ChassisRuntimeStep(uint32_t tickMs, uint16_t periodMs, int16_t motorCurrent[4]);
void ChassisRuntimeSafeStep(uint32_t tickMs, uint16_t periodMs);
void ChassisRuntimeStop(void);

#ifdef __cplusplus
}
#endif

#endif
