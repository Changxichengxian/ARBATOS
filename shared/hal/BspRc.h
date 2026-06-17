/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

/**
  **************************(C) COPYRIGHT 2026 ARBATOS**************************
  * ARBATOS
  * Copyright (c) 2024-2026 陈轩 <2811158416@qq.com>
  * @brief      BSP：BspRc 头文件
  *
  * Repo: https://github.com/Changxichengxian/ARBATOS.git
  **************************(C) COPYRIGHT 2026 ARBATOS**************************
  */
#ifndef BSP_RC_H
#define BSP_RC_H
#include "types.h"

#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#define BSP_RC_SBUS_FRAME_LENGTH 18u
#define BSP_RC_SBUS_RX_BUF_NUM 36u

typedef struct
{
    uint32_t rx_event_cnt;
    uint32_t rx_bad_size_cnt;
    uint32_t uart_error_cnt;
    uint32_t uart_last_error;
    uint32_t restart_cnt;
    uint32_t drop_cnt;
    uint32_t uart_sr;
    uint32_t uart_cr1;
    uint32_t dma_ndtr;
    uint32_t dma_cr;
    uint16_t rx_last_size;
    uint16_t rx_last_event;
} BspRcDiag;

extern void BspRcSbusInit(void);

// ===== RX (ISR -> task) =====
void BspRcSbusRxAttachTask(TaskHandle_t task);
uint8_t BspRcSbusRxPop(uint8_t frame[BSP_RC_SBUS_FRAME_LENGTH]);
uint32_t BspRcSbusRxGetDropCount(void);
// Board port calls this from UART/DMA ISR or RX callbacks.
void BspRcSbusOnFrameIsr(const uint8_t *frame, uint16_t size);

extern void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
extern void RC_unable(void);
extern void RC_restart(uint16_t dma_buf_num);
void BspRcGetDiag(BspRcDiag *out);
#endif
