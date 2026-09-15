#ifndef BSP_BOARD_PORT_USAGE_H
#define BSP_BOARD_PORT_USAGE_H

#include "BspBoardLayout.h"

// 保留旧头文件接口，实际 Zephyr 引脚与串口格式由 A 板设备树决定。
// 参考工程 USART6 为自瞄；此车型未选择主机、自瞄或裁判服务。
#define BSP_BOARD_RC_UART_HANDLE       huart1
#define BSP_BOARD_RC_UART_IRQn         USART1_IRQn
#define BSP_BOARD_AUX_UART_HANDLE      huart8
#define BSP_BOARD_REFEREE_UART_HANDLE  huart6

#endif
