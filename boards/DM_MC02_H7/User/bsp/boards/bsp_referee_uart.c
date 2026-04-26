#include "bsp_usart.h"
#include "bsp_board_ports.h"

#include "bsp_uart_dispatch.h"

#include <string.h>

#define BSP_REFEREE_UART_HANDLE BSP_BOARD_REFEREE_UART_HANDLE

static uint8_t referee_rx_buf[BSP_USART6_RX_BUF_LENGTH];

#define BSP_REFEREE_RX_RING_SIZE 4u
typedef char _check_referee_rx_ring_pow2[(BSP_REFEREE_RX_RING_SIZE & (BSP_REFEREE_RX_RING_SIZE - 1u)) == 0u ? 1 : -1];

typedef struct
{
    uint16_t len;
    uint8_t data[BSP_USART6_RX_BUF_LENGTH];
} bsp_referee_rx_chunk_t;

static bsp_referee_rx_chunk_t referee_rx_ring[BSP_REFEREE_RX_RING_SIZE];
static volatile uint16_t referee_rx_head = 0u;
static volatile uint16_t referee_rx_tail = 0u;
static volatile uint32_t referee_rx_drop = 0u;
static TaskHandle_t referee_rx_task_handle = NULL;
static uint8_t referee_tx_dma_buf[BSP_USART6_RX_BUF_LENGTH];

static void bsp_referee_uart7_start_rx(void)
{
    (void)HAL_UART_AbortReceive(&BSP_REFEREE_UART_HANDLE);
    (void)HAL_UARTEx_ReceiveToIdle_IT(&BSP_REFEREE_UART_HANDLE, referee_rx_buf, (uint16_t)sizeof(referee_rx_buf));
}

static uint8_t bsp_referee_push_from_isr(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0u || len > BSP_USART6_RX_BUF_LENGTH)
    {
        referee_rx_drop++;
        return 0u;
    }

    const uint16_t h = referee_rx_head;
    const uint16_t next = (uint16_t)((h + 1u) & (BSP_REFEREE_RX_RING_SIZE - 1u));
    if (next == referee_rx_tail)
    {
        referee_rx_drop++;
        return 0u;
    }

    referee_rx_ring[h].len = len;
    memcpy(referee_rx_ring[h].data, data, len);
    referee_rx_head = next;
    return 1u;
}

static void bsp_referee_notify_from_isr(void)
{
    if (referee_rx_task_handle == NULL)
    {
        return;
    }
    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED)
    {
        return;
    }

    BaseType_t hpw = pdFALSE;
    vTaskNotifyGiveFromISR(referee_rx_task_handle, &hpw);
    portYIELD_FROM_ISR(hpw);
}

static void bsp_referee_uart7_rx_event(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart != &BSP_REFEREE_UART_HANDLE)
    {
        return;
    }

    if (size != 0u)
    {
        if (bsp_referee_push_from_isr(referee_rx_buf, size) != 0u)
        {
            bsp_referee_notify_from_isr();
        }
    }

    bsp_referee_uart7_start_rx();
}

static void bsp_referee_uart7_error(UART_HandleTypeDef *huart)
{
    if (huart != &BSP_REFEREE_UART_HANDLE)
    {
        return;
    }

    bsp_referee_uart7_start_rx();
}

void bsp_referee_uart_init(void)
{
    referee_rx_head = 0u;
    referee_rx_tail = 0u;
    referee_rx_drop = 0u;

    (void)bsp_uart_dispatch_register(&BSP_REFEREE_UART_HANDLE, bsp_referee_uart7_rx_event, bsp_referee_uart7_error);
    bsp_referee_uart7_start_rx();
}

void bsp_referee_rx_attach_task(TaskHandle_t task)
{
    referee_rx_task_handle = task;
}

int bsp_referee_rx_pop(uint8_t *out, uint16_t *out_len)
{
    if (out == NULL || out_len == NULL)
    {
        return 0;
    }

    const uint16_t t = referee_rx_tail;
    if (t == referee_rx_head)
    {
        return 0;
    }

    const uint16_t len = referee_rx_ring[t].len;
    if (len == 0u || len > BSP_USART6_RX_BUF_LENGTH)
    {
        referee_rx_tail = (uint16_t)((t + 1u) & (BSP_REFEREE_RX_RING_SIZE - 1u));
        referee_rx_drop++;
        return 0;
    }

    memcpy(out, referee_rx_ring[t].data, len);
    *out_len = len;
    referee_rx_tail = (uint16_t)((t + 1u) & (BSP_REFEREE_RX_RING_SIZE - 1u));
    return 1;
}

uint32_t bsp_referee_rx_get_drop_count(void)
{
    return referee_rx_drop;
}

void bsp_usart6_referee_init(void)
{
    bsp_referee_uart_init();
}

void bsp_usart6_rx_attach_task(TaskHandle_t task)
{
    bsp_referee_rx_attach_task(task);
}

int bsp_usart6_rx_pop(uint8_t *out, uint16_t *out_len)
{
    return bsp_referee_rx_pop(out, out_len);
}

uint32_t bsp_usart6_rx_get_drop_count(void)
{
    return bsp_referee_rx_get_drop_count();
}

int bsp_referee_tx(const uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef ret = HAL_ERROR;

    if (data == NULL || len == 0u || len > (uint16_t)sizeof(referee_tx_dma_buf))
    {
        return (int)HAL_ERROR;
    }

    if (BSP_REFEREE_UART_HANDLE.hdmatx == NULL)
    {
        if (BSP_REFEREE_UART_HANDLE.gState != HAL_UART_STATE_READY)
        {
            return (int)HAL_BUSY;
        }
        return (int)HAL_UART_Transmit(&BSP_REFEREE_UART_HANDLE, (uint8_t *)data, len, 10u);
    }

    taskENTER_CRITICAL();
    if (BSP_REFEREE_UART_HANDLE.gState != HAL_UART_STATE_READY)
    {
        taskEXIT_CRITICAL();
        return (int)HAL_BUSY;
    }

    memcpy(referee_tx_dma_buf, data, len);
    ret = HAL_UART_Transmit_DMA(&BSP_REFEREE_UART_HANDLE, referee_tx_dma_buf, len);
    taskEXIT_CRITICAL();
    return (int)ret;
}

uint8_t bsp_referee_tx_ready(void)
{
    return (BSP_REFEREE_UART_HANDLE.gState == HAL_UART_STATE_READY) ? 1u : 0u;
}
