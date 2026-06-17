/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */


#ifndef ELRS_TASK_H
#define ELRS_TASK_H

#include "BspUsart.h"

// ELRS/CRSF baud rate on the aux link port.
#define ELRS_LINK_BAUD 420000u

// FreeRTOS task entry (created in freertos.c).
void ElrsLinkTask(void const *argument);

// Start/stop ELRS RX on the aux link port.
void ElrsLinkRxStart(void);
void ElrsLinkStop(void);

// Callbacks from the aux link ISR context.
void ElrsLinkOnRxEvent(uint16_t size, BspAuxLinkRxEvent evt);
void ElrsLinkOnItByte(uint8_t b);
bool_t ElrsLinkOnUartError(void);

#endif
