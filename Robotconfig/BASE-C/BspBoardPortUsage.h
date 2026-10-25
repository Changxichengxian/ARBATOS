#ifndef BSP_BOARD_PORT_USAGE_H
#define BSP_BOARD_PORT_USAGE_H

#include "BspBoardLayout.h"

// 旧头文件兼容入口；Zephyr 运行时使用 TOML 生成的端口绑定。
#define BSP_BOARD_RC_UART_HANDLE       huart3
#define BSP_BOARD_RC_UART_IRQn         USART3_IRQn
#define BSP_BOARD_AUX_UART_HANDLE      huart1
#define BSP_BOARD_REFEREE_UART_HANDLE  huart6

#endif
