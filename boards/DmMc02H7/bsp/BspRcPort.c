#include "BspRc.h"
#include "BspRcCfg.h"

#include "BspUartDispatch.h"
#include "usart.h"

static uint8_t sbus_rx_buf[BSP_RC_SBUS_FRAME_LENGTH];
static volatile BspRcDiag rc_diag;

static void BspRcUartStartRx(void);
static void BspRcUartRxEvent(UART_HandleTypeDef *huart, uint16_t size);
static void BspRcUartError(UART_HandleTypeDef *huart);
static uint16_t BspRcUartEventCode(UART_HandleTypeDef *huart);

void BspRcPortInit(void)
{
    rc_diag.rx_event_cnt = 0u;
    rc_diag.rx_bad_size_cnt = 0u;
    rc_diag.uart_error_cnt = 0u;
    rc_diag.uart_last_error = 0u;
    rc_diag.restart_cnt = 0u;
    rc_diag.drop_cnt = 0u;
    rc_diag.rx_last_size = 0u;
    rc_diag.rx_last_event = 0u;

    (void)BspUartDispatchRegister(&BSP_RC_UART_HANDLE, BspRcUartRxEvent, BspRcUartError);
    RC_Init(NULL, NULL, 0u);
}

void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    (void)rx1_buf;
    (void)rx2_buf;
    (void)dma_buf_num;

    (void)HAL_UART_AbortReceive(&BSP_RC_UART_HANDLE);
    BspRcUartStartRx();
}

void RC_unable(void)
{
    (void)HAL_UART_AbortReceive(&BSP_RC_UART_HANDLE);
    __HAL_UART_DISABLE(&BSP_RC_UART_HANDLE);
}

void RC_restart(uint16_t dma_buf_num)
{
    (void)dma_buf_num;

    rc_diag.restart_cnt++;

    __HAL_UART_ENABLE(&BSP_RC_UART_HANDLE);
    (void)HAL_UART_AbortReceive(&BSP_RC_UART_HANDLE);
    BspRcUartStartRx();
}

static void BspRcUartStartRx(void)
{
    (void)HAL_UARTEx_ReceiveToIdle_IT(&BSP_RC_UART_HANDLE, sbus_rx_buf, (uint16_t)sizeof(sbus_rx_buf));
}

static void BspRcUartRxEvent(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart != &BSP_RC_UART_HANDLE)
    {
        return;
    }

    rc_diag.rx_event_cnt++;
    rc_diag.rx_last_size = size;
    rc_diag.rx_last_event = BspRcUartEventCode(huart);
    if (size != BSP_RC_SBUS_FRAME_LENGTH)
    {
        rc_diag.rx_bad_size_cnt++;
    }

    BspRcSbusOnFrameIsr(sbus_rx_buf, size);
    BspRcUartStartRx();
}

static void BspRcUartError(UART_HandleTypeDef *huart)
{
    if (huart != &BSP_RC_UART_HANDLE)
    {
        return;
    }

    rc_diag.uart_error_cnt++;
    rc_diag.uart_last_error = huart->ErrorCode;
    RC_restart(0u);
}

static uint16_t BspRcUartEventCode(UART_HandleTypeDef *huart)
{
    if (huart == NULL)
    {
        return 0u;
    }

    const HAL_UART_RxEventTypeTypeDef evt = HAL_UARTEx_GetRxEventType(huart);
    if (evt == HAL_UART_RXEVENT_IDLE)
    {
        return 1u;
    }
    if (evt == HAL_UART_RXEVENT_TC)
    {
        return 3u;
    }
    return 0u;
}

void BspRcGetDiag(BspRcDiag *out)
{
    if (out == NULL)
    {
        return;
    }

    out->rx_event_cnt = rc_diag.rx_event_cnt;
    out->rx_bad_size_cnt = rc_diag.rx_bad_size_cnt;
    out->uart_error_cnt = rc_diag.uart_error_cnt;
    out->uart_last_error = rc_diag.uart_last_error;
    out->restart_cnt = rc_diag.restart_cnt;
    out->drop_cnt = BspRcSbusRxGetDropCount();
    out->rx_last_size = rc_diag.rx_last_size;
    out->rx_last_event = rc_diag.rx_last_event;
}
