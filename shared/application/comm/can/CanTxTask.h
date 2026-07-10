/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */


#ifndef CAN_TX_TASK_H
#define CAN_TX_TASK_H

#include <stdint.h>

void CanTxTask(void const *pvParameters);
uint32_t CanTxMitEnableTxCount(uint8_t actuator_id);
uint32_t CanTxMitCmdTxCount(uint8_t actuator_id);

/* 任务创建前准备紧急发送表；可重复调用，表完整时不会重建。 */
void CanTxEmergencyPrepare(void);
/* 致命故障专用：只读取完整性校验通过的缓存，锁住普通 CAN 后直发安全帧。 */
void CanTxEmergencyStopNow(void);

#endif
