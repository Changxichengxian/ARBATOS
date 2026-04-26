/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "elrs_task.h"

#include <string.h>

#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"

#include "detect_task.h"
#include "app_config.h"
#include "app_watch.h"
#include "remote_control.h"
#include "sdlog.h"

extern void Error_Handler(void);

// ===== UART1 ELRS(CRSF) RX =====
#define CRSF_FRAME_LEN_MAX 64u // length byte: [type + payload + crc]
#define CRSF_FRAME_SIZE_MAX (CRSF_FRAME_LEN_MAX + 2u) // [addr + len] + len bytes
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16u
#define CRSF_RC_FRAME_LEN 24u // length byte for RC_CHANNELS_PACKED
#define CRSF_RC_PAYLOAD_LEN 22u
#define CRSF_CHANNEL_VALUE_MIN 172u
#define CRSF_CHANNEL_VALUE_MID 992u
#define CRSF_CHANNEL_VALUE_MAX 1811u

#define UART1_ELRS_DMA_RX_BUF_SIZE 4096u
typedef char _check_uart1_elrs_dma_buf_size_u16[(UART1_ELRS_DMA_RX_BUF_SIZE <= 65535u) ? 1 : -1];

#define UART1_ELRS_NOTIFY_RX (1u << 0)
#define UART1_ELRS_NOTIFY_RESTART (1u << 1)

static uint8_t uart1_crsf_buf[CRSF_FRAME_SIZE_MAX];
static uint8_t uart1_crsf_pos = 0u;
static uint8_t uart1_crsf_expected = 0u;
static volatile uint32_t uart1_elrs_last_frame_tick_ms = 0u;
static volatile uint32_t uart1_elrs_frame_cnt = 0u;

static uint8_t uart1_elrs_dma_rx_buf[UART1_ELRS_DMA_RX_BUF_SIZE];
static volatile uint16_t uart1_elrs_dma_pos = 0u;
static uint16_t uart1_elrs_dma_last_pos = 0u;
static volatile uint32_t uart1_elrs_dma_wrap_cnt = 0u;
static uint32_t uart1_elrs_dma_last_wrap_cnt = 0u;
static volatile uint8_t uart1_elrs_dma_active = 0u;
static volatile uint8_t uart1_elrs_dma_restart_req = 0u;

static TaskHandle_t uart1_elrs_task_handle = NULL;
static volatile uint32_t uart1_elrs_notify_pending = 0u;

static void uart1_elrs_rx_start_internal(void);
static void uart1_elrs_reset(void);
static void uart1_elrs_dma_process_to(uint16_t pos);
static void uart1_elrs_notify_from_isr(uint32_t notify_bits);
static void uart1_elrs_on_byte(uint8_t b);
static void uart1_elrs_handle_frame(const uint8_t *frame, uint8_t total_len);
static void uart1_elrs_decode_rc_channels(const uint8_t *payload, uint8_t payload_len);
static uint8_t uart1_elrs_crc8_dvb_s2(const uint8_t *data, uint8_t len);
static int16_t uart1_elrs_map_axis(uint16_t v);
static uint8_t uart1_elrs_map_switch(uint16_t v);

void uart1_elrs_task(void const *argument)
{
    (void)argument;

    uart1_elrs_task_handle = xTaskGetCurrentTaskHandle();

    // Drain any notifications posted before the task handle is ready.
    taskENTER_CRITICAL();
    const uint32_t pending = uart1_elrs_notify_pending;
    uart1_elrs_notify_pending = 0u;
    taskEXIT_CRITICAL();
    if (pending != 0u)
    {
        (void)xTaskNotify(uart1_elrs_task_handle, pending, eSetBits);
    }

    while (1)
    {
        uint32_t bits = 0u;
        app_watch_task_wait(APP_WATCH_TASK_UART1_ELRS);
        (void)xTaskNotifyWait(0u, 0xFFFFFFFFu, &bits, portMAX_DELAY);
        app_watch_task_beat(APP_WATCH_TASK_UART1_ELRS);

        if (bsp_usart1_get_baudrate() != UART1_ELRS_BAUD)
        {
            continue;
        }

        if (uart1_elrs_dma_restart_req || (bits & UART1_ELRS_NOTIFY_RESTART) != 0u)
        {
            uart1_elrs_dma_restart_req = 0u;
            uart1_elrs_rx_start_internal();
            continue;
        }

        if (!uart1_elrs_dma_active)
        {
            continue;
        }

        if ((bits & UART1_ELRS_NOTIFY_RX) != 0u)
        {
            const uint32_t wrap = uart1_elrs_dma_wrap_cnt;
            const uint16_t pos = uart1_elrs_dma_pos;
            const uint32_t wraps = wrap - uart1_elrs_dma_last_wrap_cnt;
            if (wraps > 1u)
            {
                // Too late: DMA has wrapped multiple times before we could process.
                app_watch_task_timeout(APP_WATCH_TASK_UART1_ELRS);
                uart1_elrs_reset();
                uart1_elrs_dma_last_wrap_cnt = wrap;
                uart1_elrs_dma_last_pos = pos;
                continue;
            }
            if (wraps == 1u)
            {
                // Process tail of the previous cycle up to end-of-buffer.
                uart1_elrs_dma_process_to((uint16_t)UART1_ELRS_DMA_RX_BUF_SIZE);
                uart1_elrs_dma_last_wrap_cnt = wrap;
            }

            uart1_elrs_dma_process_to(pos);
        }
    }
}

void uart1_elrs_stop(void)
{
    uart1_elrs_dma_active = 0u;
    uart1_elrs_dma_restart_req = 0u;
    uart1_elrs_reset();
    uart1_elrs_dma_pos = 0u;
    uart1_elrs_dma_last_pos = 0u;
    uart1_elrs_dma_wrap_cnt = 0u;
    uart1_elrs_dma_last_wrap_cnt = 0u;
}

void uart1_elrs_rx_start(void)
{
    uart1_elrs_rx_start_internal();
}

void uart1_elrs_on_rx_event(uint16_t Size, bsp_uart1_rx_event_e evt)
{
    if (bsp_usart1_get_baudrate() != UART1_ELRS_BAUD || !uart1_elrs_dma_active)
    {
        return;
    }

    // In DMA circular mode, HAL reports an extra IDLE event with Size==RxXferSize
    // after a Transfer Complete; ignore it to avoid duplicate processing.
    if (evt == BSP_UART1_RXEVENT_IDLE && Size >= (uint16_t)UART1_ELRS_DMA_RX_BUF_SIZE)
    {
        return;
    }

    if (evt == BSP_UART1_RXEVENT_TC)
    {
        uart1_elrs_dma_wrap_cnt++;
    }

    // Size is the DMA write index within the current cycle (0..RxXferSize).
    // Use 0 to represent "wrapped to start".
    uart1_elrs_dma_pos = (Size >= (uint16_t)UART1_ELRS_DMA_RX_BUF_SIZE) ? 0u : Size;
    uart1_elrs_notify_from_isr(UART1_ELRS_NOTIFY_RX);
}

void uart1_elrs_on_it_byte(uint8_t b)
{
    uart1_elrs_on_byte(b);
}

bool_t uart1_elrs_on_uart_error(void)
{
    app_watch_task_error(APP_WATCH_TASK_UART1_ELRS);
    uart1_elrs_reset();
    if (!uart1_elrs_dma_active)
    {
        return 0;
    }

    uart1_elrs_dma_restart_req = 1u;
    uart1_elrs_notify_from_isr(UART1_ELRS_NOTIFY_RESTART);
    return 1;
}

static void uart1_elrs_rx_start_internal(void)
{
    uart1_elrs_stop();

    if (!bsp_usart1_rx_has_dma())
    {
        // Fallback (shouldn't happen): IT per byte.
        (void)bsp_usart1_rx_it_start();
        return;
    }

    if (bsp_usart1_rx_to_idle_dma_start(uart1_elrs_dma_rx_buf, (uint16_t)UART1_ELRS_DMA_RX_BUF_SIZE) != 0)
    {
        app_watch_task_error(APP_WATCH_TASK_UART1_ELRS);
        Error_Handler();
    }
    uart1_elrs_dma_active = 1u;
}

static void uart1_elrs_reset(void)
{
    uart1_crsf_pos = 0u;
    uart1_crsf_expected = 0u;
}

static void uart1_elrs_dma_process_to(uint16_t pos)
{
    const uint16_t size = (uint16_t)UART1_ELRS_DMA_RX_BUF_SIZE;
    uint16_t last = uart1_elrs_dma_last_pos;

    if (pos > size)
    {
        pos = (uint16_t)(pos % size);
    }
    if (last >= size)
    {
        last = 0u;
    }

    if (pos == last)
    {
        return;
    }

    if (pos > last)
    {
        for (uint16_t i = last; i < pos; i++)
        {
            uart1_elrs_on_byte(uart1_elrs_dma_rx_buf[i]);
        }
    }
    else
    {
        for (uint16_t i = last; i < size; i++)
        {
            uart1_elrs_on_byte(uart1_elrs_dma_rx_buf[i]);
        }
        for (uint16_t i = 0u; i < pos; i++)
        {
            uart1_elrs_on_byte(uart1_elrs_dma_rx_buf[i]);
        }
    }

    uart1_elrs_dma_last_pos = (pos == size) ? 0u : pos;
}

static void uart1_elrs_notify_from_isr(uint32_t notify_bits)
{
    if (uart1_elrs_task_handle == NULL)
    {
        uart1_elrs_notify_pending |= notify_bits;
        return;
    }

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    (void)xTaskNotifyFromISR(uart1_elrs_task_handle, notify_bits, eSetBits, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static uint8_t uart1_elrs_crc8_dvb_s2(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0u;
    for (uint8_t i = 0u; i < len; i++)
    {
        crc ^= data[i];
        for (uint8_t b = 0u; b < 8u; b++)
        {
            crc = (crc & 0x80u) ? (uint8_t)((crc << 1u) ^ 0xD5u) : (uint8_t)(crc << 1u);
        }
    }
    return crc;
}

static int16_t uart1_elrs_map_axis(uint16_t v)
{
    if (v < CRSF_CHANNEL_VALUE_MIN)
    {
        v = CRSF_CHANNEL_VALUE_MIN;
    }
    if (v > CRSF_CHANNEL_VALUE_MAX)
    {
        v = CRSF_CHANNEL_VALUE_MAX;
    }

    if (v >= CRSF_CHANNEL_VALUE_MID)
    {
        const uint16_t delta = (uint16_t)(v - CRSF_CHANNEL_VALUE_MID);
        const uint16_t denom = (uint16_t)(CRSF_CHANNEL_VALUE_MAX - CRSF_CHANNEL_VALUE_MID);
        return (int16_t)((((uint32_t)delta * (uint32_t)RC_CH_VALUE_ABS_MAX) + (denom / 2u)) / denom);
    }

    const uint16_t delta = (uint16_t)(CRSF_CHANNEL_VALUE_MID - v);
    const uint16_t denom = (uint16_t)(CRSF_CHANNEL_VALUE_MID - CRSF_CHANNEL_VALUE_MIN);
    return (int16_t)(-((int16_t)((((uint32_t)delta * (uint32_t)RC_CH_VALUE_ABS_MAX) + (denom / 2u)) / denom)));
}

static uint8_t uart1_elrs_map_switch(uint16_t v)
{
    if (v < CRSF_CHANNEL_VALUE_MIN)
    {
        v = CRSF_CHANNEL_VALUE_MIN;
    }
    if (v > CRSF_CHANNEL_VALUE_MAX)
    {
        v = CRSF_CHANNEL_VALUE_MAX;
    }

    const uint16_t t_down = (uint16_t)((CRSF_CHANNEL_VALUE_MIN + CRSF_CHANNEL_VALUE_MID) / 2u);
    const uint16_t t_up = (uint16_t)((CRSF_CHANNEL_VALUE_MID + CRSF_CHANNEL_VALUE_MAX) / 2u);

    if (v <= t_down)
    {
        return RC_SW_DOWN;
    }
    if (v >= t_up)
    {
        return RC_SW_UP;
    }
    return RC_SW_MID;
}

static void uart1_elrs_decode_rc_channels(const uint8_t *payload, uint8_t payload_len)
{
    if (payload == NULL || payload_len < CRSF_RC_PAYLOAD_LEN)
    {
        return;
    }

    uint16_t ch[16] = {0};
    uint32_t bitbuf = 0u;
    uint8_t bitcnt = 0u;
    uint8_t idx = 0u;

    for (uint8_t i = 0u; i < CRSF_RC_PAYLOAD_LEN; i++)
    {
        bitbuf |= ((uint32_t)payload[i]) << bitcnt;
        bitcnt = (uint8_t)(bitcnt + 8u);
        while (bitcnt >= 11u && idx < 16u)
        {
            ch[idx++] = (uint16_t)(bitbuf & 0x07FFu);
            bitbuf >>= 11u;
            bitcnt = (uint8_t)(bitcnt - 11u);
        }
    }

    if (idx < 4u)
    {
        return;
    }

    const app_input_config_t *cfg = &g_app_config.input;
    int16_t ch_raw[16] = {0};
    for (uint8_t i = 0u; i < 16u; i++)
    {
        ch_raw[i] = (int16_t)ch[i];
    }

    RC_ctrl_t rc = {0};
    for (uint8_t i = 0u; i < 5u; i++)
    {
        uint8_t idx_ch = cfg->elrs_ch_map[i];
        if (idx_ch >= 16u)
        {
            idx_ch = (i < 4u) ? i : 6u;
        }
        rc.rc.ch[i] = uart1_elrs_map_axis(ch[idx_ch]);
    }

    for (uint8_t i = 0u; i < 2u; i++)
    {
        uint8_t idx_sw = cfg->elrs_sw_map[i];
        if (idx_sw >= 16u)
        {
            idx_sw = (uint8_t)(4u + i);
        }
        rc.rc.s[i] = (char)uart1_elrs_map_switch(ch[idx_sw]);
    }

    remote_control_log_raw_source(MANUAL_INPUT_SRC_ELRS,
                                  SDLOG_MANUAL_INPUT_PROTO_CRSF,
                                  SDLOG_MANUAL_INPUT_RANGE_RAW_11BIT,
                                  16u,
                                  ch_raw,
                                  NULL,
                                  &rc);
    remote_control_set_rc(&rc);

    sdlog_rc_crsf_t pkt = {0};
    for (uint8_t i = 0u; i < 16u; i++)
    {
        pkt.ch_raw[i] = ch[i];
    }
    const uint32_t rc_len = (uint32_t)sizeof(RC_ctrl_t);
    const uint32_t copy_len = (rc_len < (uint32_t)sizeof(pkt.rc_ctrl)) ? rc_len : (uint32_t)sizeof(pkt.rc_ctrl);
    const uint8_t *rc_bytes = (const uint8_t *)&rc;
    for (uint32_t i = 0u; i < copy_len; i++)
    {
        pkt.rc_ctrl[i] = rc_bytes[i];
    }
    sdlog_write(SDLOG_TAG_RC_CRSF, &pkt, (uint16_t)sizeof(pkt));

    uart1_elrs_last_frame_tick_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    uart1_elrs_frame_cnt++;
}

static void uart1_elrs_handle_frame(const uint8_t *frame, uint8_t total_len)
{
    if (frame == NULL || total_len < 4u)
    {
        return;
    }

    const uint8_t len = frame[1];
    if (len < 2u || (uint8_t)(len + 2u) != total_len)
    {
        return;
    }

    const uint8_t crc = frame[total_len - 1u];
    const uint8_t calc = uart1_elrs_crc8_dvb_s2(&frame[2], (uint8_t)(len - 1u));
    if (calc != crc)
    {
        return;
    }

    const uint8_t type = frame[2];
    if (type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED && len == CRSF_RC_FRAME_LEN)
    {
        uart1_elrs_decode_rc_channels(&frame[3], CRSF_RC_PAYLOAD_LEN);
    }
}

static void uart1_elrs_on_byte(uint8_t b)
{
    if (uart1_crsf_pos == 0u)
    {
        uart1_crsf_buf[0] = b;
        uart1_crsf_pos = 1u;
        return;
    }

    if (uart1_crsf_pos == 1u)
    {
        if (b < 2u || b > CRSF_FRAME_LEN_MAX)
        {
            uart1_elrs_reset();
            return;
        }

        uart1_crsf_buf[1] = b;
        uart1_crsf_expected = (uint8_t)(b + 2u);
        uart1_crsf_pos = 2u;
        return;
    }

    if (uart1_crsf_expected < 4u || uart1_crsf_expected > CRSF_FRAME_SIZE_MAX)
    {
        uart1_elrs_reset();
        return;
    }

    uart1_crsf_buf[uart1_crsf_pos++] = b;
    if (uart1_crsf_pos >= uart1_crsf_expected)
    {
        uart1_elrs_handle_frame(uart1_crsf_buf, uart1_crsf_expected);
        uart1_elrs_reset();
    }
}
