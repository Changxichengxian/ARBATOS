#include "bsp_usart.h"
#include "bsp_board_ports.h"

#include "bsp_uart_dispatch.h"
#include "usart.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

#define BSP_AUX_UART_HANDLE           BSP_BOARD_AUX_UART_HANDLE
#define BSP_RS485_PORT0_UART_HANDLE   BSP_BOARD_RS485_PORT0_UART_HANDLE
#define BSP_RS485_PORT1_UART_HANDLE   BSP_BOARD_RS485_PORT1_UART_HANDLE

static bsp_aux_link_rx_event_cb_t g_aux_rx_event_cb = NULL;
static bsp_aux_link_rx_byte_cb_t g_aux_rx_byte_cb = NULL;
static bsp_aux_link_error_cb_t g_aux_error_cb = NULL;
static volatile uint8_t g_aux_it_rx_active = 0u;
static uint8_t g_aux_it_rx_byte = 0u;
static uint8_t g_aux_dispatch_registered = 0u;

static bsp_usart_rx_byte_cb_t usart2_rx_byte_cb = NULL;
static bsp_usart_error_cb_t usart2_error_cb = NULL;
static volatile uint8_t usart2_it_rx_active = 0u;
static uint8_t usart2_it_rx_byte = 0u;
static uint8_t usart2_dispatch_registered = 0u;

static bsp_usart_rx_byte_cb_t usart3_rx_byte_cb = NULL;
static bsp_usart_error_cb_t usart3_error_cb = NULL;
static volatile uint8_t usart3_it_rx_active = 0u;
static uint8_t usart3_it_rx_byte = 0u;
static uint8_t usart3_dispatch_registered = 0u;

static uint32_t bsp_usart_blocking_timeout(uint32_t timeout_ms)
{
    return (timeout_ms == 0u) ? 10u : timeout_ms;
}

static int bsp_rs485_set_baudrate(UART_HandleTypeDef *huart, volatile uint8_t *it_rx_active, uint32_t baudrate)
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

static void bsp_aux_link_dispatch_error(UART_HandleTypeDef *huart)
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

static void bsp_aux_link_register_dispatch_once(void)
{
    if (g_aux_dispatch_registered != 0u)
    {
        return;
    }

    if (bsp_uart_dispatch_register(&BSP_AUX_UART_HANDLE, NULL, bsp_aux_link_dispatch_error) == 0)
    {
        g_aux_dispatch_registered = 1u;
    }
}

static void bsp_usart2_dispatch_error(UART_HandleTypeDef *huart)
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

static void bsp_usart2_register_dispatch_once(void)
{
    if (usart2_dispatch_registered != 0u)
    {
        return;
    }

    if (bsp_uart_dispatch_register(&BSP_RS485_PORT0_UART_HANDLE, NULL, bsp_usart2_dispatch_error) == 0)
    {
        usart2_dispatch_registered = 1u;
    }
}

static void bsp_usart3_dispatch_error(UART_HandleTypeDef *huart)
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

static void bsp_usart3_register_dispatch_once(void)
{
    if (usart3_dispatch_registered != 0u)
    {
        return;
    }

    if (bsp_uart_dispatch_register(&BSP_RS485_PORT1_UART_HANDLE, NULL, bsp_usart3_dispatch_error) == 0)
    {
        usart3_dispatch_registered = 1u;
    }
}

void bsp_aux_link_set_rx_event_cb(bsp_aux_link_rx_event_cb_t cb)
{
    g_aux_rx_event_cb = cb;
    (void)g_aux_rx_event_cb;
}

void bsp_aux_link_set_rx_byte_cb(bsp_aux_link_rx_byte_cb_t cb)
{
    g_aux_rx_byte_cb = cb;
}

void bsp_aux_link_set_error_cb(bsp_aux_link_error_cb_t cb)
{
    g_aux_error_cb = cb;
    bsp_aux_link_register_dispatch_once();
}

uint32_t bsp_aux_link_get_baudrate(void)
{
    return BSP_AUX_UART_HANDLE.Init.BaudRate;
}

int bsp_aux_link_set_baudrate(uint32_t baudrate)
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

int bsp_aux_link_tx_dma(const uint8_t *data, uint16_t len)
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

uint8_t bsp_aux_link_tx_ready(void)
{
    return (BSP_AUX_UART_HANDLE.gState == HAL_UART_STATE_READY) ? 1u : 0u;
}

int bsp_aux_link_rx_it_start(void)
{
    bsp_aux_link_register_dispatch_once();

    g_aux_it_rx_active = 1u;
    (void)HAL_UART_AbortReceive(&BSP_AUX_UART_HANDLE);
    const HAL_StatusTypeDef ret = HAL_UART_Receive_IT(&BSP_AUX_UART_HANDLE, &g_aux_it_rx_byte, 1u);
    if (ret != HAL_OK)
    {
        g_aux_it_rx_active = 0u;
    }
    return (int)ret;
}

void bsp_aux_link_rx_it_stop(void)
{
    g_aux_it_rx_active = 0u;
    (void)HAL_UART_AbortReceive(&BSP_AUX_UART_HANDLE);
}

void bsp_usart2_set_rx_byte_cb(bsp_usart_rx_byte_cb_t cb)
{
    usart2_rx_byte_cb = cb;
}

void bsp_usart2_set_error_cb(bsp_usart_error_cb_t cb)
{
    usart2_error_cb = cb;
    bsp_usart2_register_dispatch_once();
}

uint32_t bsp_usart2_get_baudrate(void)
{
    return BSP_RS485_PORT0_UART_HANDLE.Init.BaudRate;
}

int bsp_usart2_set_baudrate(uint32_t baudrate)
{
    return bsp_rs485_set_baudrate(&BSP_RS485_PORT0_UART_HANDLE, &usart2_it_rx_active, baudrate);
}

int bsp_usart2_tx(const uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
    if (data == NULL || len == 0u)
    {
        return (int)HAL_ERROR;
    }

    if (BSP_RS485_PORT0_UART_HANDLE.gState != HAL_UART_STATE_READY)
    {
        return (int)HAL_BUSY;
    }

    return (int)HAL_UART_Transmit(&BSP_RS485_PORT0_UART_HANDLE, (uint8_t *)data, len, bsp_usart_blocking_timeout(timeout_ms));
}

int bsp_usart2_rx_it_start(void)
{
    bsp_usart2_register_dispatch_once();

    usart2_it_rx_active = 1u;
    (void)HAL_UART_AbortReceive(&BSP_RS485_PORT0_UART_HANDLE);
    const HAL_StatusTypeDef ret = HAL_UART_Receive_IT(&BSP_RS485_PORT0_UART_HANDLE, &usart2_it_rx_byte, 1u);
    if (ret != HAL_OK)
    {
        usart2_it_rx_active = 0u;
    }
    return (int)ret;
}

void bsp_usart2_rx_it_stop(void)
{
    usart2_it_rx_active = 0u;
    (void)HAL_UART_AbortReceive(&BSP_RS485_PORT0_UART_HANDLE);
}

void bsp_usart3_set_rx_byte_cb(bsp_usart_rx_byte_cb_t cb)
{
    usart3_rx_byte_cb = cb;
}

void bsp_usart3_set_error_cb(bsp_usart_error_cb_t cb)
{
    usart3_error_cb = cb;
    bsp_usart3_register_dispatch_once();
}

uint32_t bsp_usart3_get_baudrate(void)
{
    return BSP_RS485_PORT1_UART_HANDLE.Init.BaudRate;
}

int bsp_usart3_set_baudrate(uint32_t baudrate)
{
    return bsp_rs485_set_baudrate(&BSP_RS485_PORT1_UART_HANDLE, &usart3_it_rx_active, baudrate);
}

int bsp_usart3_tx(const uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
    if (data == NULL || len == 0u)
    {
        return (int)HAL_ERROR;
    }

    if (BSP_RS485_PORT1_UART_HANDLE.gState != HAL_UART_STATE_READY)
    {
        return (int)HAL_BUSY;
    }

    return (int)HAL_UART_Transmit(&BSP_RS485_PORT1_UART_HANDLE, (uint8_t *)data, len, bsp_usart_blocking_timeout(timeout_ms));
}

int bsp_usart3_rx_it_start(void)
{
    bsp_usart3_register_dispatch_once();

    usart3_it_rx_active = 1u;
    (void)HAL_UART_AbortReceive(&BSP_RS485_PORT1_UART_HANDLE);
    const HAL_StatusTypeDef ret = HAL_UART_Receive_IT(&BSP_RS485_PORT1_UART_HANDLE, &usart3_it_rx_byte, 1u);
    if (ret != HAL_OK)
    {
        usart3_it_rx_active = 0u;
    }
    return (int)ret;
}

void bsp_usart3_rx_it_stop(void)
{
    usart3_it_rx_active = 0u;
    (void)HAL_UART_AbortReceive(&BSP_RS485_PORT1_UART_HANDLE);
}

uint8_t bsp_aux_link_rx_has_dma(void)
{
    return 0u;
}

int bsp_aux_link_rx_to_idle_dma_start(uint8_t *buf, uint16_t len)
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
