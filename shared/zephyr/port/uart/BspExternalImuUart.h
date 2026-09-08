/* SPDX-License-Identifier: Apache-2.0 */
#ifndef BSP_EXTERNAL_IMU_UART_H
#define BSP_EXTERNAL_IMU_UART_H

#include "BspUsart.h"

void BspExternalImuLinkSetRxEventCb(BspAuxLinkRxEventCb cb);
void BspExternalImuLinkSetRxByteCb(BspAuxLinkRxByteCb cb);
void BspExternalImuLinkSetErrorCb(BspAuxLinkErrorCb cb);
uint32_t BspExternalImuLinkGetBaudrate(void);
uint8_t BspExternalImuLinkRxHasDma(void);
int BspExternalImuLinkRxItStart(void);
void BspExternalImuLinkRxItStop(void);
int BspExternalImuLinkRxToIdleDmaStart(uint8_t *buf, uint16_t len);

#endif
