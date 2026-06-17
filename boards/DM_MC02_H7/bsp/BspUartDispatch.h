#ifndef BSP_UART_DISPATCH_H
#define BSP_UART_DISPATCH_H

#include "usart.h"

#include <stdint.h>

typedef void (*BspUartRxEventCb)(UART_HandleTypeDef *huart, uint16_t size);
typedef void (*BspUartErrorCb)(UART_HandleTypeDef *huart);

int BspUartDispatchRegister(UART_HandleTypeDef *huart, BspUartRxEventCb rx_event_cb, BspUartErrorCb error_cb);

#endif
