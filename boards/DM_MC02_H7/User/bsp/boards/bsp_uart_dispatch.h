#ifndef BSP_UART_DISPATCH_H
#define BSP_UART_DISPATCH_H

#include "usart.h"

#include <stdint.h>

typedef void (*bsp_uart_rx_event_cb_t)(UART_HandleTypeDef *huart, uint16_t size);
typedef void (*bsp_uart_error_cb_t)(UART_HandleTypeDef *huart);

int bsp_uart_dispatch_register(UART_HandleTypeDef *huart, bsp_uart_rx_event_cb_t rx_event_cb, bsp_uart_error_cb_t error_cb);

#endif
