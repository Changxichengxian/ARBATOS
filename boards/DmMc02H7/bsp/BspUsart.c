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
#define BSP_RS485_FAULT_ACK_SPIN_LIMIT 1000000u
#define BSP_RS485_FAULT_TX_SPIN_LIMIT  24000000u
#define BSP_RS485_FAULT_BRR_MIN        0x10u
#define BSP_RS485_FAULT_BRR_MAX        0xFFFFu
#define BSP_RS485_FAULT_CR1_IRQ_MASK   (USART_CR1_IDLEIE | USART_CR1_RXNEIE_RXFNEIE | \
                                        USART_CR1_TCIE | USART_CR1_TXEIE_TXFNFIE | \
                                        USART_CR1_PEIE | USART_CR1_CMIE | USART_CR1_RTOIE | \
                                        USART_CR1_EOBIE | USART_CR1_TXFEIE | USART_CR1_RXFFIE)
#define BSP_RS485_FAULT_CR3_IRQ_MASK   (USART_CR3_EIE | USART_CR3_CTSIE | USART_CR3_WUFIE | \
                                        USART_CR3_TXFTIE | USART_CR3_TCBGTIE | USART_CR3_RXFTIE)
#define BSP_RS485_FAULT_ICR_MASK       (USART_ICR_PECF | USART_ICR_FECF | USART_ICR_NECF | \
                                        USART_ICR_ORECF | USART_ICR_TCCF)

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
static volatile uint8_t rs485_fault_locked = 0u;

static UART_HandleTypeDef *BspRs485Handle(uint8_t port)
{
    if (port == 0u)
    {
        return &BSP_RS485_PORT0_UART_HANDLE;
    }
    if (port == 1u)
    {
        return &BSP_RS485_PORT1_UART_HANDLE;
    }
    return NULL;
}

static uint8_t BspRs485FaultWaitFlag(USART_TypeDef *uart,
                                     uint32_t mask,
                                     uint32_t expected,
                                     uint32_t spin_limit)
{
    if (uart == NULL)
    {
        return 0u;
    }

    while (spin_limit > 0u)
    {
        if ((uart->ISR & mask) == expected)
        {
            return 1u;
        }
        spin_limit--;
        __NOP();
    }
    return 0u;
}

static void BspRs485FaultAbortUart(UART_HandleTypeDef *huart)
{
    USART_TypeDef *uart;

    if (huart == NULL || huart->Instance == NULL)
    {
        return;
    }

    uart = huart->Instance;
    uart->CR1 &= ~BSP_RS485_FAULT_CR1_IRQ_MASK;
    uart->CR3 &= ~(BSP_RS485_FAULT_CR3_IRQ_MASK | USART_CR3_DMAT | USART_CR3_DMAR);
    uart->CR1 &= ~USART_CR1_TE;
    (void)BspRs485FaultWaitFlag(uart,
                               USART_ISR_TEACK,
                               0u,
                               BSP_RS485_FAULT_ACK_SPIN_LIMIT);
    uart->RQR = USART_RQR_TXFRQ | USART_RQR_RXFRQ;
    uart->ICR = BSP_RS485_FAULT_ICR_MASK;
}

static uint8_t BspRs485FaultConfigure(UART_HandleTypeDef *huart,
                                      uint32_t baudrate,
                                      uint32_t brr)
{
    USART_TypeDef *uart;
    uint32_t cr1;

    if (huart == NULL || huart->Instance == NULL || baudrate == 0u ||
        brr < BSP_RS485_FAULT_BRR_MIN || brr > BSP_RS485_FAULT_BRR_MAX)
    {
        return 0u;
    }

    uart = huart->Instance;
    BspRs485FaultAbortUart(huart);
    cr1 = uart->CR1;
    uart->CR1 = cr1 & ~(USART_CR1_UE | BSP_RS485_FAULT_CR1_IRQ_MASK);
    if (BspRs485FaultWaitFlag(uart,
                              USART_ISR_TEACK | USART_ISR_REACK,
                              0u,
                              BSP_RS485_FAULT_ACK_SPIN_LIMIT) == 0u)
    {
        return 0u;
    }

    uart->BRR = brr;
    uart->CR3 = (uart->CR3 & ~(BSP_RS485_FAULT_CR3_IRQ_MASK | USART_CR3_DMAT | USART_CR3_DMAR)) |
                USART_CR3_DEM;
    uart->ICR = BSP_RS485_FAULT_ICR_MASK;
    uart->CR1 = (cr1 | USART_CR1_UE | USART_CR1_TE | USART_CR1_RE) &
                ~BSP_RS485_FAULT_CR1_IRQ_MASK;
    huart->Init.BaudRate = baudrate;
    return BspRs485FaultWaitFlag(uart,
                                 USART_ISR_TEACK,
                                 USART_ISR_TEACK,
                                 BSP_RS485_FAULT_ACK_SPIN_LIMIT);
}

static int BspRs485TxSemPrepare(SemaphoreHandle_t *sem, StaticSemaphore_t *storage)
{
    if (rs485_fault_locked != 0u)
    {
        return (int)HAL_ERROR;
    }
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
    if (rs485_fault_locked != 0u)
    {
        return (int)HAL_ERROR;
    }
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

    if (rs485_fault_locked != 0u)
    {
        return (int)HAL_ERROR;
    }
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

uint8_t BspRs485FaultBaudPrepare(uint8_t port, uint32_t baudrate, uint32_t *out_brr)
{
    UART_HandleTypeDef *huart = BspRs485Handle(port);
    UART_ClockSourceTypeDef clocksource;
    uint32_t pclk;
    uint32_t brr;

    if (huart == NULL || out_brr == NULL || baudrate == 0u ||
        huart->Init.OverSampling != UART_OVERSAMPLING_16)
    {
        return 0u;
    }

    UART_GETCLOCKSOURCE(huart, clocksource);
    if (clocksource != UART_CLOCKSOURCE_D2PCLK1)
    {
        /* MC02 H7 的 USART2/3 固定走 D2PCLK1；时钟源变化时不能沿用错误 BRR。 */
        return 0u;
    }

    pclk = HAL_RCC_GetPCLK1Freq();
    if (pclk == 0u)
    {
        return 0u;
    }
    brr = UART_DIV_SAMPLING16(pclk, baudrate, huart->Init.ClockPrescaler);
    if (brr < BSP_RS485_FAULT_BRR_MIN || brr > BSP_RS485_FAULT_BRR_MAX)
    {
        return 0u;
    }

    *out_brr = brr;
    return 1u;
}

void BspRs485FaultLock(void)
{
    const uint32_t primask = __get_PRIMASK();

    __disable_irq();
    if (rs485_fault_locked == 0u)
    {
        rs485_fault_locked = 1u;
        __DMB();
        NVIC_DisableIRQ(USART2_IRQn);
        NVIC_DisableIRQ(USART3_IRQn);
        NVIC_ClearPendingIRQ(USART2_IRQn);
        NVIC_ClearPendingIRQ(USART3_IRQn);
        BspRs485FaultAbortUart(&BSP_RS485_PORT0_UART_HANDLE);
        BspRs485FaultAbortUart(&BSP_RS485_PORT1_UART_HANDLE);
    }
    if (primask == 0u)
    {
        __enable_irq();
    }
}

int BspRs485FaultTx(uint8_t port,
                    uint32_t baudrate,
                    uint32_t brr,
                    const uint8_t *data,
                    uint16_t len)
{
    UART_HandleTypeDef *huart = BspRs485Handle(port);
    USART_TypeDef *uart;
    uint32_t primask;
    int ret = (int)HAL_ERROR;

    if (rs485_fault_locked == 0u || huart == NULL || data == NULL ||
        len == 0u || len > (uint16_t)BSP_RS485_TX_IT_MAX_LEN)
    {
        return (int)HAL_ERROR;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    uart = huart->Instance;
    if (BspRs485FaultConfigure(huart, baudrate, brr) == 0u)
    {
        goto out;
    }

    for (uint16_t i = 0u; i < len; i++)
    {
        if (BspRs485FaultWaitFlag(uart,
                                  USART_ISR_TXE_TXFNF,
                                  USART_ISR_TXE_TXFNF,
                                  BSP_RS485_FAULT_TX_SPIN_LIMIT) == 0u)
        {
            ret = (int)HAL_TIMEOUT;
            goto out;
        }
        uart->TDR = data[i];
    }
    if (BspRs485FaultWaitFlag(uart,
                              USART_ISR_TC,
                              USART_ISR_TC,
                              BSP_RS485_FAULT_TX_SPIN_LIMIT) == 0u)
    {
        ret = (int)HAL_TIMEOUT;
        goto out;
    }
    ret = (int)HAL_OK;

out:
    if (ret != (int)HAL_OK)
    {
        BspRs485FaultAbortUart(huart);
    }
    if (primask == 0u)
    {
        __enable_irq();
    }
    return ret;
}

uint8_t BspRs485FaultLocked(void)
{
    return rs485_fault_locked;
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
