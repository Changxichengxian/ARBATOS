/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CONTROL_RUNTIME_TASK_H
#define CONTROL_RUNTIME_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

void ControlChassisTask(void const *argument);
void ControlGimbalTask(void const *argument);

#ifdef __cplusplus
}
#endif

#endif
