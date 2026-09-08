/* SPDX-License-Identifier: Apache-2.0 */
#ifndef BSP_ELRS_UART_H
#define BSP_ELRS_UART_H

#include "BspUsart.h"

#if defined(__ZEPHYR__)
void BspElrsLinkSetRxEventCb(BspAuxLinkRxEventCb cb);
void BspElrsLinkSetRxByteCb(BspAuxLinkRxByteCb cb);
void BspElrsLinkSetErrorCb(BspAuxLinkErrorCb cb);
uint32_t BspElrsLinkGetBaudrate(void);
uint8_t BspElrsLinkRxHasDma(void);
int BspElrsLinkRxItStart(void);
void BspElrsLinkRxItStop(void);
int BspElrsLinkRxToIdleDmaStart(uint8_t *buf, uint16_t len);
#else
/* 旧 HAL 工程继续让 ELRS 复用 AUX，不改变现有板级实现。 */
#define BspElrsLinkSetRxEventCb       BspAuxLinkSetRxEventCb
#define BspElrsLinkSetRxByteCb        BspAuxLinkSetRxByteCb
#define BspElrsLinkSetErrorCb         BspAuxLinkSetErrorCb
#define BspElrsLinkGetBaudrate        BspAuxLinkGetBaudrate
#define BspElrsLinkRxHasDma           BspAuxLinkRxHasDma
#define BspElrsLinkRxItStart          BspAuxLinkRxItStart
#define BspElrsLinkRxItStop           BspAuxLinkRxItStop
#define BspElrsLinkRxToIdleDmaStart   BspAuxLinkRxToIdleDmaStart
#endif

#endif
