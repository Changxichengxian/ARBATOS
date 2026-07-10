/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-07-10
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef SHOOT_RUNTIME_H
#define SHOOT_RUNTIME_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct ManualInputSnapshot;

/*
 * 这些入口只供 ShootCtrl 适配层使用。任务和注册表不得绕过
 * ControlMgr 直接调用，避免出现两套生命周期。
 */
void ShootRuntimeInit(void);
int16_t ShootRuntimeStep(const struct ManualInputSnapshot *manualInput);
void ShootRuntimeSafeStep(const struct ManualInputSnapshot *manualInput);
void ShootRuntimeStop(void);

#ifdef __cplusplus
}
#endif

#endif
