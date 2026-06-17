#include "BspUsart.h"
#include "BspBoardPorts.h"

#include "BspUartDispatch.h"

#include <string.h>

#define BSP_REFEREE_UART_HANDLE BSP_BOARD_REFEREE_UART_HANDLE

static uint8_t RefereeRxBuf[BSP_USART6_RX_BUF_LENGTH];

#define BSP_REFEREE_RX_RING_SIZE 4u
typedef char _check_referee_rx_ring_pow2[(BSP_REFEREE_RX_RING_SIZE & (BSP_REFEREE_RX_RING_SIZE - 1u)) == 0u ? 1 : -1];

typedef struct
{
    uint16_t len;
    uint8_t data[BSP_USART6_RX_BUF_LENGTH];
} BspRefereeRxChunk;

static BspRefereeRxChunk RefereeRxRing[BSP_REFEREE_RX_RING_SIZE];
static volatile uint16_t RefereeRxHead = 0u;
static volatile uint16_t RefereeRxTail = 0u;
static volatile uint32_t RefereeRxDrop = 0u;
static TaskHandle_t RefereeRxTaskHandle = NULL;
static uint8_t RefereeTxDmaBuf[BSP_USART6_RX_BUF_LENGTH];

static void BspRefereeUart7StartRx(void)
{
    (void)HAL_UART_AbortReceive(&BSP_REFEREE_UART_HANDLE);
    (void)HAL_UARTEx_ReceiveToIdle_IT(&BSP_REFEREE_UART_HANDLE, RefereeRxBuf, (uint16_t)sizeof(RefereeRxBuf));
}

static uint8_t BspRefereePushFromIsr(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0u || len > BSP_USART6_RX_BUF_LENGTH)
    {
        RefereeRxDrop++;
        return 0u;
    }

    const uint16_t h = RefereeRxHead;
    const uint16_t next = (uint16_t)((h + 1u) & (BSP_REFEREE_RX_RING_SIZE - 1u));
    if (next == RefereeRxTail)
    {
        RefereeRxDrop++;
        return 0u;
    }

    RefereeRxRing[h].len = len;
    memcpy(RefereeRxRing[h].data, data, len);
    RefereeRxHead = next;
    return 1u;
}

static void BspRefereeNotifyFromIsr(void)
{
    if (RefereeRxTaskHandle == NULL)
    {
        return;
    }
    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED)
    {
        return;
    }

    BaseType_t hpw = pdFALSE;
    vTaskNotifyGiveFromISR(RefereeRxTaskHandle, &hpw);
    portYIELD_FROM_ISR(hpw);
}

static void BspRefereeUart7RxEvent(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart != &BSP_REFEREE_UART_HANDLE)
    {
        return;
    }

    if (size != 0u)
    {
        if (BspRefereePushFromIsr(RefereeRxBuf, size) != 0u)
        {
            BspRefereeNotifyFromIsr();
        }
    }

    BspRefereeUart7StartRx();
}

static void BspRefereeUart7Error(UART_HandleTypeDef *huart)
{
    if (huart != &BSP_REFEREE_UART_HANDLE)
    {
        return;
    }

    BspRefereeUart7StartRx();
}

void BspRefereeUartInit(void)
{
    RefereeRxHead = 0u;
    RefereeRxTail = 0u;
    RefereeRxDrop = 0u;

    (void)BspUartDispatchRegister(&BSP_REFEREE_UART_HANDLE, BspRefereeUart7RxEvent, BspRefereeUart7Error);
    BspRefereeUart7StartRx();
}

void BspRefereeRxAttachTask(TaskHandle_t task)
{
    RefereeRxTaskHandle = task;
}

int BspRefereeRxPop(uint8_t *out, uint16_t *out_len)
{
    if (out == NULL || out_len == NULL)
    {
        return 0;
    }

    const uint16_t t = RefereeRxTail;
    if (t == RefereeRxHead)
    {
        return 0;
    }

    const uint16_t len = RefereeRxRing[t].len;
    if (len == 0u || len > BSP_USART6_RX_BUF_LENGTH)
    {
        RefereeRxTail = (uint16_t)((t + 1u) & (BSP_REFEREE_RX_RING_SIZE - 1u));
        RefereeRxDrop++;
        return 0;
    }

    memcpy(out, RefereeRxRing[t].data, len);
    *out_len = len;
    RefereeRxTail = (uint16_t)((t + 1u) & (BSP_REFEREE_RX_RING_SIZE - 1u));
    return 1;
}

uint32_t BspRefereeRxGetDropCount(void)
{
    return RefereeRxDrop;
}

void BspUsart6RefereeInit(void)
{
    BspRefereeUartInit();
}

void BspUsart6RxAttachTask(TaskHandle_t task)
{
    BspRefereeRxAttachTask(task);
}

int BspUsart6RxPop(uint8_t *out, uint16_t *out_len)
{
    return BspRefereeRxPop(out, out_len);
}

uint32_t BspUsart6RxGetDropCount(void)
{
    return BspRefereeRxGetDropCount();
}

int BspRefereeTx(const uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef ret = HAL_ERROR;

    if (data == NULL || len == 0u || len > (uint16_t)sizeof(RefereeTxDmaBuf))
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

    memcpy(RefereeTxDmaBuf, data, len);
    ret = HAL_UART_Transmit_DMA(&BSP_REFEREE_UART_HANDLE, RefereeTxDmaBuf, len);
    taskEXIT_CRITICAL();
    return (int)ret;
}

uint8_t BspRefereeTxReady(void)
{
    return (BSP_REFEREE_UART_HANDLE.gState == HAL_UART_STATE_READY) ? 1u : 0u;
}
