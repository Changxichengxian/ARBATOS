#include "BspUsart.h"
#include "BspBoardPorts.h"

#include "BspUartDispatch.h"
#include "usart.h"

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

static BspUsartRxByteCb usart3_rx_byte_cb = NULL;
static BspUsartErrorCb usart3_error_cb = NULL;
static volatile uint8_t usart3_it_rx_active = 0u;
static uint8_t usart3_it_rx_byte = 0u;
static uint8_t usart3_dispatch_registered = 0u;

static uint32_t BspUsartBlockingTimeout(uint32_t timeout_ms)
{
    return (timeout_ms == 0u) ? 10u : timeout_ms;
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

int BspUsart2Tx(const uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
    if (data == NULL || len == 0u)
    {
        return (int)HAL_ERROR;
    }

    if (BSP_RS485_PORT0_UART_HANDLE.gState != HAL_UART_STATE_READY)
    {
        return (int)HAL_BUSY;
    }

    return (int)HAL_UART_Transmit(&BSP_RS485_PORT0_UART_HANDLE, (uint8_t *)data, len, BspUsartBlockingTimeout(timeout_ms));
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

int BspUsart3Tx(const uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
    if (data == NULL || len == 0u)
    {
        return (int)HAL_ERROR;
    }

    if (BSP_RS485_PORT1_UART_HANDLE.gState != HAL_UART_STATE_READY)
    {
        return (int)HAL_BUSY;
    }

    return (int)HAL_UART_Transmit(&BSP_RS485_PORT1_UART_HANDLE, (uint8_t *)data, len, BspUsartBlockingTimeout(timeout_ms));
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
