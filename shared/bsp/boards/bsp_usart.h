/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef BSP_USART_H
#define BSP_USART_H
#include "struct_typedef.h"

#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

extern void usart6_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);

// Referee system (USART6) RX is double-buffered DMA + IDLE IRQ.
#define BSP_USART6_RX_BUF_LENGTH 512u
#define BSP_REFEREE_RX_BUF_LENGTH BSP_USART6_RX_BUF_LENGTH

extern void bsp_usart6_referee_init(void);

// ===== RX (ISR -> task) =====
void bsp_usart6_rx_attach_task(TaskHandle_t task);
int bsp_usart6_rx_pop(uint8_t *out, uint16_t *out_len);
uint32_t bsp_usart6_rx_get_drop_count(void);

// ===== Referee UART (board-specific) =====
// Default implementation maps to the legacy USART6 path.
void bsp_referee_uart_init(void);
void bsp_referee_rx_attach_task(TaskHandle_t task);
int bsp_referee_rx_pop(uint8_t *out, uint16_t *out_len);
uint32_t bsp_referee_rx_get_drop_count(void);
int bsp_referee_tx(const uint8_t *data, uint16_t len);
uint8_t bsp_referee_tx_ready(void);

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
} bsp_aux_link_rx_event_e;

typedef void (*bsp_aux_link_rx_event_cb_t)(uint16_t size, bsp_aux_link_rx_event_e evt);
typedef void (*bsp_aux_link_rx_byte_cb_t)(uint8_t b);
// Return: 1 if handled (no default restart), 0 to apply default handling.
typedef uint8_t (*bsp_aux_link_error_cb_t)(void);
typedef void (*bsp_usart_rx_byte_cb_t)(uint8_t b);
typedef uint8_t (*bsp_usart_error_cb_t)(void);

typedef bsp_aux_link_rx_event_e bsp_uart1_rx_event_e;
typedef bsp_aux_link_rx_event_cb_t bsp_uart1_rx_event_cb_t;
typedef bsp_aux_link_rx_byte_cb_t bsp_uart1_rx_byte_cb_t;
typedef bsp_aux_link_error_cb_t bsp_uart1_error_cb_t;

#define BSP_UART1_RXEVENT_UNKNOWN BSP_AUX_LINK_RXEVENT_UNKNOWN
#define BSP_UART1_RXEVENT_IDLE    BSP_AUX_LINK_RXEVENT_IDLE
#define BSP_UART1_RXEVENT_HT      BSP_AUX_LINK_RXEVENT_HT
#define BSP_UART1_RXEVENT_TC      BSP_AUX_LINK_RXEVENT_TC

void bsp_aux_link_set_rx_event_cb(bsp_aux_link_rx_event_cb_t cb);
void bsp_aux_link_set_rx_byte_cb(bsp_aux_link_rx_byte_cb_t cb);
void bsp_aux_link_set_error_cb(bsp_aux_link_error_cb_t cb);

uint32_t bsp_aux_link_get_baudrate(void);
int bsp_aux_link_set_baudrate(uint32_t baudrate);

// Return: 0 on success, else HAL_StatusTypeDef value (1: ERROR, 2: BUSY, 3: TIMEOUT)
int bsp_aux_link_tx_dma(const uint8_t *data, uint16_t len);
uint8_t bsp_aux_link_tx_ready(void);

// Return: 0 on success, else HAL_StatusTypeDef value (1: ERROR, 2: BUSY, 3: TIMEOUT)
int bsp_aux_link_rx_it_start(void);
void bsp_aux_link_rx_it_stop(void);

uint8_t bsp_aux_link_rx_has_dma(void);
// Return: 0 on success, else HAL_StatusTypeDef value (1: ERROR, 2: BUSY, 3: TIMEOUT)
int bsp_aux_link_rx_to_idle_dma_start(uint8_t *buf, uint16_t len);

#define bsp_usart1_set_rx_event_cb      bsp_aux_link_set_rx_event_cb
#define bsp_usart1_set_rx_byte_cb       bsp_aux_link_set_rx_byte_cb
#define bsp_usart1_set_error_cb         bsp_aux_link_set_error_cb
#define bsp_usart1_get_baudrate         bsp_aux_link_get_baudrate
#define bsp_usart1_set_baudrate         bsp_aux_link_set_baudrate
#define bsp_usart1_tx_dma               bsp_aux_link_tx_dma
#define bsp_usart1_tx_ready             bsp_aux_link_tx_ready
#define bsp_usart1_rx_it_start          bsp_aux_link_rx_it_start
#define bsp_usart1_rx_it_stop           bsp_aux_link_rx_it_stop
#define bsp_usart1_rx_has_dma           bsp_aux_link_rx_has_dma
#define bsp_usart1_rx_to_idle_dma_start bsp_aux_link_rx_to_idle_dma_start

// ===== RS485 ports (USART2 / USART3 on MC02 H7) =====
void bsp_usart2_set_rx_byte_cb(bsp_usart_rx_byte_cb_t cb);
void bsp_usart2_set_error_cb(bsp_usart_error_cb_t cb);
uint32_t bsp_usart2_get_baudrate(void);
int bsp_usart2_set_baudrate(uint32_t baudrate);
int bsp_usart2_tx(const uint8_t *data, uint16_t len, uint32_t timeout_ms);
int bsp_usart2_rx_it_start(void);
void bsp_usart2_rx_it_stop(void);

void bsp_usart3_set_rx_byte_cb(bsp_usart_rx_byte_cb_t cb);
void bsp_usart3_set_error_cb(bsp_usart_error_cb_t cb);
uint32_t bsp_usart3_get_baudrate(void);
int bsp_usart3_set_baudrate(uint32_t baudrate);
int bsp_usart3_tx(const uint8_t *data, uint16_t len, uint32_t timeout_ms);
int bsp_usart3_rx_it_start(void);
void bsp_usart3_rx_it_stop(void);
#endif
