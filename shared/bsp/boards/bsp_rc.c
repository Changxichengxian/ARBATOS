/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

/**
  **************************(C) COPYRIGHT 2026 ARBATOS**************************
  * ARBATOS
  * @brief      BSP: SBUS RX ring buffer and task notification
  **************************(C) COPYRIGHT 2026 ARBATOS**************************
  */
#include "bsp_rc.h"

#include <string.h>

#define BSP_RC_SBUS_RX_RING_SIZE 8u
typedef char _check_sbus_rx_ring_pow2[(BSP_RC_SBUS_RX_RING_SIZE & (BSP_RC_SBUS_RX_RING_SIZE - 1u)) == 0u ? 1 : -1];

typedef struct
{
    uint8_t frame[BSP_RC_SBUS_FRAME_LENGTH];
} bsp_rc_sbus_frame_t;

static bsp_rc_sbus_frame_t sbus_ring[BSP_RC_SBUS_RX_RING_SIZE];
static volatile uint16_t sbus_head = 0u;
static volatile uint16_t sbus_tail = 0u;
static volatile uint32_t sbus_drop = 0u;
static TaskHandle_t sbus_task_handle = NULL;

extern void bsp_rc_port_init(void);

static uint8_t bsp_rc_sbus_push_from_isr(const uint8_t frame[BSP_RC_SBUS_FRAME_LENGTH])
{
    const uint16_t h = sbus_head;
    const uint16_t next = (uint16_t)((h + 1u) & (BSP_RC_SBUS_RX_RING_SIZE - 1u));
    if (next == sbus_tail)
    {
        sbus_drop++;
        return 0u;
    }

    memcpy(sbus_ring[h].frame, frame, BSP_RC_SBUS_FRAME_LENGTH);
    sbus_head = next;
    return 1u;
}

static void bsp_rc_sbus_notify_from_isr(void)
{
    if (sbus_task_handle == NULL)
    {
        return;
    }
    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED)
    {
        return;
    }

    BaseType_t hpw = pdFALSE;
    vTaskNotifyGiveFromISR(sbus_task_handle, &hpw);
    portYIELD_FROM_ISR(hpw);
}

void bsp_rc_sbus_init(void)
{
    sbus_head = 0u;
    sbus_tail = 0u;
    sbus_drop = 0u;

    bsp_rc_port_init();
}

void bsp_rc_sbus_rx_attach_task(TaskHandle_t task)
{
    sbus_task_handle = task;
}

uint8_t bsp_rc_sbus_rx_pop(uint8_t frame[BSP_RC_SBUS_FRAME_LENGTH])
{
    if (frame == NULL)
    {
        return 0u;
    }

    const uint16_t t = sbus_tail;
    if (t == sbus_head)
    {
        return 0u;
    }

    memcpy(frame, sbus_ring[t].frame, BSP_RC_SBUS_FRAME_LENGTH);
    sbus_tail = (uint16_t)((t + 1u) & (BSP_RC_SBUS_RX_RING_SIZE - 1u));
    return 1u;
}

uint32_t bsp_rc_sbus_rx_get_drop_count(void)
{
    return sbus_drop;
}

void bsp_rc_sbus_on_frame_isr(const uint8_t *frame, uint16_t size)
{
    if (frame == NULL)
    {
        sbus_drop++;
        return;
    }
    if (size != BSP_RC_SBUS_FRAME_LENGTH)
    {
        sbus_drop++;
        return;
    }

    if (bsp_rc_sbus_push_from_isr(frame) != 0u)
    {
        bsp_rc_sbus_notify_from_isr();
    }
}
