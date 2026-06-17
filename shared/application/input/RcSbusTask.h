/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef RC_SBUS_TASK_H
#define RC_SBUS_TASK_H

/*
 * This task only drains board-level SBUS/DBUS receive buffers and forwards
 * raw frames to the manual-input merge layer in `ManualInput.c`.
 */
extern void ManualInputSbusRxTask(void const *pvParameters);
extern void RcSbusTask(void const *pvParameters);

#endif
