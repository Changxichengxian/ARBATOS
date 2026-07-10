#include "BspUsart.h"
#include "BspBoardPorts.h"

#include "BspUartDispatch.h"
#include "usart.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include <string.h>

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

#define BSP_AUX_UART_HANDLE           BSP_BOARD_AUX_UART_HANDLE
#define BSP_RS485_PORT0_UART_HANDLE   BSP_BOARD_RS485_PORT0_UART_HANDLE
#define BSP_RS485_PORT1_UART_HANDLE   BSP_BOARD_RS485_PORT1_UART_HANDLE

static BspAuxLinkRxEventCb g_aux_rx_event_cb = NULL;
static BspAuxLinkRxByteCb g_aux_rx_byte_cb = NULL;
static BspAuxLinkErrorCb g_aux_error_cb = NULL;
static volatile uint8_t g_aux_it_rx_active = 0u;
static uint8_t g_aux_it_rx_byte = 0u;
static uint8_t g_aux_dispatch_registered = 0u;

static BspUsartRxByteCb usart2_rx_byte_cb = NULL;
static BspUsartErrorCb usart2_error_cb = NULL;
static volatile uint8_t usart2_it_rx_active = 0u;
static uint8_t usart2_it_rx_byte = 0u;
static uint8_t usart2_dispatch_registered = 0u;
static SemaphoreHandle_t usart2_tx_sem = NULL;
static StaticSemaphore_t usart2_tx_sem_buf;
static uint8_t usart2_tx_buf[BSP_RS485_TX_IT_MAX_LEN];

static BspUsartRxByteCb usart3_rx_byte_cb = NULL;
static BspUsartErrorCb usart3_error_cb = NULL;
static volatile uint8_t usart3_it_rx_active = 0u;
static uint8_t usart3_it_rx_byte = 0u;
static uint8_t usart3_dispatch_registered = 0u;
static SemaphoreHandle_t usart3_tx_sem = NULL;
static StaticSemaphore_t usart3_tx_sem_buf;
static uint8_t usart3_tx_buf[BSP_RS485_TX_IT_MAX_LEN];

static int BspRs485TxSemPrepare(SemaphoreHandle_t *sem, StaticSemaphore_t *storage)
{
    if (sem == NULL || storage == NULL)
    {
        return (int)HAL_ERROR;
    }

    taskENTER_CRITICAL();
    if (*sem == NULL)
    {
        *sem = xSemaphoreCreateBinaryStatic(storage);
    }
    taskEXIT_CRITICAL();
    return (*sem != NULL) ? (int)HAL_OK : (int)HAL_ERROR;
}

static int BspRs485TxItStart(UART_HandleTypeDef *huart,
                             SemaphoreHandle_t sem,
                             uint8_t *storage,
                             const uint8_t *data,
                             uint16_t len)
{
    if (huart == NULL || sem == NULL || storage == NULL || data == NULL ||
        len == 0u || len > (uint16_t)BSP_RS485_TX_IT_MAX_LEN)
    {
        return (int)HAL_ERROR;
    }
    if (huart->gState != HAL_UART_STATE_READY)
    {
        return (int)HAL_BUSY;
    }

    (void)xSemaphoreTake(sem, 0u);
    (void)memcpy(storage, data, len);
    __DMB();
    return (int)HAL_UART_Transmit_IT(huart, storage, len);
}

static int BspRs485TxItWait(UART_HandleTypeDef *huart,
                            SemaphoreHandle_t sem,
                            uint32_t timeout_ms)
{
    TickType_t timeout = pdMS_TO_TICKS((timeout_ms == 0u) ? 10u : timeout_ms);

    if (huart == NULL || sem == NULL)
    {
        return (int)HAL_ERROR;
    }
    if (timeout == 0u)
    {
        timeout = 1u;
    }
    if (xSemaphoreTake(sem, timeout) != pdTRUE)
    {
        (void)HAL_UART_AbortTransmit(huart);
        return (int)HAL_TIMEOUT;
    }
    /* 收到 TC 即表示最后停止位已经完成；并行 RX 的错误码不应污染 TX 结果。 */
    return (int)HAL_OK;
}

static void BspRs485TxSignalFromIsr(SemaphoreHandle_t sem)
{
    BaseType_t higher_priority_task_woken = pdFALSE;

    if (sem == NULL)
    {
        return;
    }
    (void)xSemaphoreGiveFromISR(sem, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

static int BspRs485SetBaudrate(UART_HandleTypeDef *huart, volatile uint8_t *it_rx_active, uint32_t baudrate)
{
    HAL_StatusTypeDef ret = HAL_OK;

    if (huart == NULL || it_rx_active == NULL || baudrate == 0u)
    {
        return (int)HAL_ERROR;
    }

    if (huart->Init.BaudRate == baudrate)
    {
        return (int)HAL_OK;
    }

    /* 不能用改波特率的 HAL_Abort 截断已经完成安全裁决并启动的帧。 */
    if (huart->gState != HAL_UART_STATE_READY)
    {
        return (int)HAL_BUSY;
    }

    *it_rx_active = 0u;
    (void)HAL_UART_Abort(huart);

    huart->Init.BaudRate = baudrate;
    ret = HAL_RS485Ex_Init(huart, UART_DE_POLARITY_HIGH, 0u, 0u);
    if (ret != HAL_OK)
    {
        return (int)ret;
    }

    ret = HAL_UARTEx_SetTxFifoThreshold(huart, UART_TXFIFO_THRESHOLD_1_8);
    if (ret != HAL_OK)
    {
        return (int)ret;
    }

    ret = HAL_UARTEx_SetRxFifoThreshold(huart, UART_RXFIFO_THRESHOLD_1_8);
    if (ret != HAL_OK)
    {
        return (int)ret;
    }

    ret = HAL_UARTEx_DisableFifoMode(huart);
    return (int)ret;
}

static void BspAuxLinkDispatchError(UART_HandleTypeDef *huart)
{
    if (huart != &BSP_AUX_UART_HANDLE)
    {
        return;
    }

    uint8_t handled = 0u;
    if (g_aux_error_cb != NULL)
    {
        handled = g_aux_error_cb();
    }

    if (!handled && g_aux_it_rx_active != 0u)
    {
        (void)HAL_UART_Receive_IT(&BSP_AUX_UART_HANDLE, &g_aux_it_rx_byte, 1u);
    }
}

static void BspAuxLinkRegisterDispatchOnce(void)
{
    if (g_aux_dispatch_registered != 0u)
    {
        return;
    }

    if (BspUartDispatchRegister(&BSP_AUX_UART_HANDLE, NULL, BspAuxLinkDispatchError) == 0)
    {
        g_aux_dispatch_registered = 1u;
    }
}

static void BspUsart2DispatchError(UART_HandleTypeDef *huart)
{
    if (huart != &BSP_RS485_PORT0_UART_HANDLE)
    {
        return;
    }

    uint8_t handled = 0u;
    if (usart2_error_cb != NULL)
    {
        handled = usart2_error_cb();
    }

    if (!handled && usart2_it_rx_active != 0u)
    {
        (void)HAL_UART_Receive_IT(&BSP_RS485_PORT0_UART_HANDLE, &usart2_it_rx_byte, 1u);
    }
}

static void BspUsart2RegisterDispatchOnce(void)
{
    if (usart2_dispatch_registered != 0u)
    {
        return;
    }

    if (BspUartDispatchRegister(&BSP_RS485_PORT0_UART_HANDLE, NULL, BspUsart2DispatchError) == 0)
    {
        usart2_dispatch_registered = 1u;
    }
}

static void BspUsart3DispatchError(UART_HandleTypeDef *huart)
{
    if (huart != &BSP_RS485_PORT1_UART_HANDLE)
    {
        return;
    }

    uint8_t handled = 0u;
    if (usart3_error_cb != NULL)
    {
        handled = usart3_error_cb();
    }

    if (!handled && usart3_it_rx_active != 0u)
    {
        (void)HAL_UART_Receive_IT(&BSP_RS485_PORT1_UART_HANDLE, &usart3_it_rx_byte, 1u);
    }
}

static void BspUsart3RegisterDispatchOnce(void)
{
    if (usart3_dispatch_registered != 0u)
    {
        return;
    }

    if (BspUartDispatchRegister(&BSP_RS485_PORT1_UART_HANDLE, NULL, BspUsart3DispatchError) == 0)
    {
        usart3_dispatch_registered = 1u;
    }
}

void BspAuxLinkSetRxEventCb(BspAuxLinkRxEventCb cb)
{
    g_aux_rx_event_cb = cb;
    (void)g_aux_rx_event_cb;
}

void BspAuxLinkSetRxByteCb(BspAuxLinkRxByteCb cb)
{
    g_aux_rx_byte_cb = cb;
}

void BspAuxLinkSetErrorCb(BspAuxLinkErrorCb cb)
{
    g_aux_error_cb = cb;
    BspAuxLinkRegisterDispatchOnce();
}

uint32_t BspAuxLinkGetBaudrate(void)
{
    return BSP_AUX_UART_HANDLE.Init.BaudRate;
}

int BspAuxLinkSetBaudrate(uint32_t baudrate)
{
    if (baudrate == 0u)
    {
        return (int)HAL_ERROR;
    }
    if (BSP_AUX_UART_HANDLE.Init.BaudRate == baudrate)
    {
        return (int)HAL_OK;
    }

    g_aux_it_rx_active = 0u;
    (void)HAL_UART_Abort(&BSP_AUX_UART_HANDLE);

    BSP_AUX_UART_HANDLE.Init.BaudRate = baudrate;
    return (int)HAL_UART_Init(&BSP_AUX_UART_HANDLE);
}

int BspAuxLinkTxDma(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0u)
    {
        return (int)HAL_ERROR;
    }

    if (BSP_AUX_UART_HANDLE.gState != HAL_UART_STATE_READY)
    {
        return (int)HAL_BUSY;
    }

    return (int)HAL_UART_Transmit(&BSP_AUX_UART_HANDLE, (uint8_t *)data, len, 10u);
}

uint8_t BspAuxLinkTxReady(void)
{
    return (BSP_AUX_UART_HANDLE.gState == HAL_UART_STATE_READY) ? 1u : 0u;
}

int BspAuxLinkRxItStart(void)
{
    BspAuxLinkRegisterDispatchOnce();

    g_aux_it_rx_active = 1u;
    (void)HAL_UART_AbortReceive(&BSP_AUX_UART_HANDLE);
    const HAL_StatusTypeDef ret = HAL_UART_Receive_IT(&BSP_AUX_UART_HANDLE, &g_aux_it_rx_byte, 1u);
    if (ret != HAL_OK)
    {
        g_aux_it_rx_active = 0u;
    }
    return (int)ret;
}

void BspAuxLinkRxItStop(void)
{
    g_aux_it_rx_active = 0u;
    (void)HAL_UART_AbortReceive(&BSP_AUX_UART_HANDLE);
}

void BspUsart2SetRxByteCb(BspUsartRxByteCb cb)
{
    usart2_rx_byte_cb = cb;
}

void BspUsart2SetErrorCb(BspUsartErrorCb cb)
{
    usart2_error_cb = cb;
    BspUsart2RegisterDispatchOnce();
}

uint32_t BspUsart2GetBaudrate(void)
{
    return BSP_RS485_PORT0_UART_HANDLE.Init.BaudRate;
}

int BspUsart2SetBaudrate(uint32_t baudrate)
{
    return BspRs485SetBaudrate(&BSP_RS485_PORT0_UART_HANDLE, &usart2_it_rx_active, baudrate);
}

int BspUsart2TxItPrepare(void)
{
    return BspRs485TxSemPrepare(&usart2_tx_sem, &usart2_tx_sem_buf);
}

int BspUsart2TxItStart(const uint8_t *data, uint16_t len)
{
    return BspRs485TxItStart(&BSP_RS485_PORT0_UART_HANDLE,
                             usart2_tx_sem,
                             usart2_tx_buf,
                             data,
                             len);
}

int BspUsart2TxItWait(uint32_t timeout_ms)
{
    return BspRs485TxItWait(&BSP_RS485_PORT0_UART_HANDLE, usart2_tx_sem, timeout_ms);
}

int BspUsart2RxItStart(void)
{
    BspUsart2RegisterDispatchOnce();

    usart2_it_rx_active = 1u;
    (void)HAL_UART_AbortReceive(&BSP_RS485_PORT0_UART_HANDLE);
    const HAL_StatusTypeDef ret = HAL_UART_Receive_IT(&BSP_RS485_PORT0_UART_HANDLE, &usart2_it_rx_byte, 1u);
    if (ret != HAL_OK)
    {
        usart2_it_rx_active = 0u;
    }
    return (int)ret;
}

void BspUsart2RxItStop(void)
{
    usart2_it_rx_active = 0u;
    (void)HAL_UART_AbortReceive(&BSP_RS485_PORT0_UART_HANDLE);
}

void BspUsart3SetRxByteCb(BspUsartRxByteCb cb)
{
    usart3_rx_byte_cb = cb;
}

void BspUsart3SetErrorCb(BspUsartErrorCb cb)
{
    usart3_error_cb = cb;
    BspUsart3RegisterDispatchOnce();
}

uint32_t BspUsart3GetBaudrate(void)
{
    return BSP_RS485_PORT1_UART_HANDLE.Init.BaudRate;
}

int BspUsart3SetBaudrate(uint32_t baudrate)
{
    return BspRs485SetBaudrate(&BSP_RS485_PORT1_UART_HANDLE, &usart3_it_rx_active, baudrate);
}

int BspUsart3TxItPrepare(void)
{
    return BspRs485TxSemPrepare(&usart3_tx_sem, &usart3_tx_sem_buf);
}

int BspUsart3TxItStart(const uint8_t *data, uint16_t len)
{
    return BspRs485TxItStart(&BSP_RS485_PORT1_UART_HANDLE,
                             usart3_tx_sem,
                             usart3_tx_buf,
                             data,
                             len);
}

int BspUsart3TxItWait(uint32_t timeout_ms)
{
    return BspRs485TxItWait(&BSP_RS485_PORT1_UART_HANDLE, usart3_tx_sem, timeout_ms);
}

int BspUsart3RxItStart(void)
{
    BspUsart3RegisterDispatchOnce();

    usart3_it_rx_active = 1u;
    (void)HAL_UART_AbortReceive(&BSP_RS485_PORT1_UART_HANDLE);
    const HAL_StatusTypeDef ret = HAL_UART_Receive_IT(&BSP_RS485_PORT1_UART_HANDLE, &usart3_it_rx_byte, 1u);
    if (ret != HAL_OK)
    {
        usart3_it_rx_active = 0u;
    }
    return (int)ret;
}

void BspUsart3RxItStop(void)
{
    usart3_it_rx_active = 0u;
    (void)HAL_UART_AbortReceive(&BSP_RS485_PORT1_UART_HANDLE);
}

uint8_t BspAuxLinkRxHasDma(void)
{
    return 0u;
}

int BspAuxLinkRxToIdleDmaStart(uint8_t *buf, uint16_t len)
{
    (void)buf;
    (void)len;
    return (int)HAL_ERROR;
}

void usart1_tx_dma_init(void)
{
}

void usart1_tx_dma_enable(uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0u)
    {
        return;
    }

    (void)HAL_UART_Transmit(&BSP_AUX_UART_HANDLE, data, len, 10u);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &BSP_AUX_UART_HANDLE)
    {
        const uint8_t b = g_aux_it_rx_byte;
        if (g_aux_rx_byte_cb != NULL)
        {
            g_aux_rx_byte_cb(b);
        }

        if (g_aux_it_rx_active != 0u)
        {
            (void)HAL_UART_Receive_IT(&BSP_AUX_UART_HANDLE, &g_aux_it_rx_byte, 1u);
        }
        return;
    }

    if (huart == &BSP_RS485_PORT0_UART_HANDLE)
    {
        const uint8_t b = usart2_it_rx_byte;
        if (usart2_rx_byte_cb != NULL)
        {
            usart2_rx_byte_cb(b);
        }

        if (usart2_it_rx_active != 0u)
        {
            (void)HAL_UART_Receive_IT(&BSP_RS485_PORT0_UART_HANDLE, &usart2_it_rx_byte, 1u);
        }
        return;
    }

    if (huart == &BSP_RS485_PORT1_UART_HANDLE)
    {
        const uint8_t b = usart3_it_rx_byte;
        if (usart3_rx_byte_cb != NULL)
        {
            usart3_rx_byte_cb(b);
        }

        if (usart3_it_rx_active != 0u)
        {
            (void)HAL_UART_Receive_IT(&BSP_RS485_PORT1_UART_HANDLE, &usart3_it_rx_byte, 1u);
        }
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &BSP_RS485_PORT0_UART_HANDLE)
    {
        BspRs485TxSignalFromIsr(usart2_tx_sem);
    }
    else if (huart == &BSP_RS485_PORT1_UART_HANDLE)
    {
        BspRs485TxSignalFromIsr(usart3_tx_sem);
    }
}
