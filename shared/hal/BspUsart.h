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
#define BSP_RS485_TX_IT_MAX_LEN 40u

#if defined(STM32H723xx)
/*
 * 致命故障专用：Prepare 只在正常启动阶段计算 BRR；Lock 后普通 IT 发送永久拒绝。
 * FaultTx 不使用 RTOS、HAL 超时和中断，只按预计算 BRR 直接轮询寄存器。
 */
uint8_t BspRs485FaultBaudPrepare(uint8_t port, uint32_t baudrate, uint32_t *out_brr);
void BspRs485FaultLock(void);
int BspRs485FaultTx(uint8_t port,
                    uint32_t baudrate,
                    uint32_t brr,
                    const uint8_t *data,
                    uint16_t len);
uint8_t BspRs485FaultLocked(void);
#endif

void BspUsart2SetRxByteCb(BspUsartRxByteCb cb);
void BspUsart2SetErrorCb(BspUsartErrorCb cb);
uint32_t BspUsart2GetBaudrate(void);
int BspUsart2SetBaudrate(uint32_t baudrate);
int BspUsart2TxItPrepare(void);
/* Start 只复制并启动中断发送，须与最终安全裁决放在同一任务临界区；Wait 必须在退出临界区后调用。 */
int BspUsart2TxItStart(const uint8_t *data, uint16_t len);
int BspUsart2TxItWait(uint32_t timeout_ms);
int BspUsart2RxItStart(void);
void BspUsart2RxItStop(void);

void BspUsart3SetRxByteCb(BspUsartRxByteCb cb);
void BspUsart3SetErrorCb(BspUsartErrorCb cb);
uint32_t BspUsart3GetBaudrate(void);
int BspUsart3SetBaudrate(uint32_t baudrate);
int BspUsart3TxItPrepare(void);
/* 与 USART2 相同：Start 在短临界区内提交，Wait 在临界区外等待物理发送完成。 */
int BspUsart3TxItStart(const uint8_t *data, uint16_t len);
int BspUsart3TxItWait(uint32_t timeout_ms);
int BspUsart3RxItStart(void);
void BspUsart3RxItStop(void);
#endif
