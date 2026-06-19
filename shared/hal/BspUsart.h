/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef BSP_USART_H
#define BSP_USART_H
#include "Types.h"

#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

extern void usart6_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);

// Referee system (USART6) RX is double-buffered DMA + IDLE IRQ.
#define BSP_USART6_RX_BUF_LENGTH 512u
#define BSP_REFEREE_RX_BUF_LENGTH BSP_USART6_RX_BUF_LENGTH

extern void BspUsart6RefereeInit(void);

// ===== RX (ISR -> task) =====
void BspUsart6RxAttachTask(TaskHandle_t task);
int BspUsart6RxPop(uint8_t *out, uint16_t *out_len);
uint32_t BspUsart6RxGetDropCount(void);

// ===== Referee UART (board-specific) =====
// Default implementation maps to the legacy USART6 path.
void BspRefereeUartInit(void);
void BspRefereeRxAttachTask(TaskHandle_t task);
int BspRefereeRxPop(uint8_t *out, uint16_t *out_len);
uint32_t BspRefereeRxGetDropCount(void);
int BspRefereeTx(const uint8_t *data, uint16_t len);
uint8_t BspRefereeTxReady(void);

extern void usart1_tx_dma_init(void);
extern void usart1_tx_dma_enable(uint8_t *data, uint16_t len);

// ===== Aux link port =====
// Legacy name: UART1.
typedef enum
{
    BSP_AUX_LINK_RXEVENT_UNKNOWN = 0,
    BSP_AUX_LINK_RXEVENT_IDLE,
    BSP_AUX_LINK_RXEVENT_HT,
    BSP_AUX_LINK_RXEVENT_TC,
} BspAuxLinkRxEvent;

typedef void (*BspAuxLinkRxEventCb)(uint16_t size, BspAuxLinkRxEvent evt);
typedef void (*BspAuxLinkRxByteCb)(uint8_t b);
// Return: 1 if handled (no default restart), 0 to apply default handling.
typedef uint8_t (*BspAuxLinkErrorCb)(void);
typedef void (*BspUsartRxByteCb)(uint8_t b);
typedef uint8_t (*BspUsartErrorCb)(void);

typedef BspAuxLinkRxEvent BspUart1RxEvent;
typedef BspAuxLinkRxEventCb BspUart1RxEventCb;
typedef BspAuxLinkRxByteCb BspUart1RxByteCb;
typedef BspAuxLinkErrorCb BspUart1ErrorCb;

#define BSP_UART1_RXEVENT_UNKNOWN BSP_AUX_LINK_RXEVENT_UNKNOWN
#define BSP_UART1_RXEVENT_IDLE    BSP_AUX_LINK_RXEVENT_IDLE
#define BSP_UART1_RXEVENT_HT      BSP_AUX_LINK_RXEVENT_HT
#define BSP_UART1_RXEVENT_TC      BSP_AUX_LINK_RXEVENT_TC

void BspAuxLinkSetRxEventCb(BspAuxLinkRxEventCb cb);
void BspAuxLinkSetRxByteCb(BspAuxLinkRxByteCb cb);
void BspAuxLinkSetErrorCb(BspAuxLinkErrorCb cb);

uint32_t BspAuxLinkGetBaudrate(void);
int BspAuxLinkSetBaudrate(uint32_t baudrate);

// Return: 0 on success, else HAL_StatusTypeDef value (1: ERROR, 2: BUSY, 3: TIMEOUT)
int BspAuxLinkTxDma(const uint8_t *data, uint16_t len);
uint8_t BspAuxLinkTxReady(void);

// Return: 0 on success, else HAL_StatusTypeDef value (1: ERROR, 2: BUSY, 3: TIMEOUT)
int BspAuxLinkRxItStart(void);
void BspAuxLinkRxItStop(void);

uint8_t BspAuxLinkRxHasDma(void);
// Return: 0 on success, else HAL_StatusTypeDef value (1: ERROR, 2: BUSY, 3: TIMEOUT)
int BspAuxLinkRxToIdleDmaStart(uint8_t *buf, uint16_t len);

#define BspUsart1SetRxEventCb      BspAuxLinkSetRxEventCb
#define BspUsart1SetRxByteCb       BspAuxLinkSetRxByteCb
#define BspUsart1SetErrorCb         BspAuxLinkSetErrorCb
#define BspUsart1GetBaudrate         BspAuxLinkGetBaudrate
#define BspUsart1SetBaudrate         BspAuxLinkSetBaudrate
#define BspUsart1TxDma               BspAuxLinkTxDma
#define BspUsart1TxReady             BspAuxLinkTxReady
#define BspUsart1RxItStart          BspAuxLinkRxItStart
#define BspUsart1RxItStop           BspAuxLinkRxItStop
#define BspUsart1RxHasDma           BspAuxLinkRxHasDma
#define BspUsart1RxToIdleDmaStart BspAuxLinkRxToIdleDmaStart

// ===== RS485 ports (USART2 / USART3 on MC02 H7) =====
void BspUsart2SetRxByteCb(BspUsartRxByteCb cb);
void BspUsart2SetErrorCb(BspUsartErrorCb cb);
uint32_t BspUsart2GetBaudrate(void);
int BspUsart2SetBaudrate(uint32_t baudrate);
int BspUsart2Tx(const uint8_t *data, uint16_t len, uint32_t timeout_ms);
int BspUsart2RxItStart(void);
void BspUsart2RxItStop(void);

void BspUsart3SetRxByteCb(BspUsartRxByteCb cb);
void BspUsart3SetErrorCb(BspUsartErrorCb cb);
uint32_t BspUsart3GetBaudrate(void);
int BspUsart3SetBaudrate(uint32_t baudrate);
int BspUsart3Tx(const uint8_t *data, uint16_t len, uint32_t timeout_ms);
int BspUsart3RxItStart(void);
void BspUsart3RxItStop(void);
#endif
