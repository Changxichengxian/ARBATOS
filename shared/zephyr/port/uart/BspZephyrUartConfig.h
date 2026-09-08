/*
 * Zephyr UART role binding.
 *
 * A target selects a role by defining one of these macros to a devicetree
 * node identifier, for example:
 *   #define ARB_UART_AUX_NODE DT_NODELABEL(usart1)
 *
 * Keep role selection outside this port source: one physical UART must not
 * be assigned to two active roles.
 */
#ifndef ARB_ZEPHYR_UART_CONFIG_H
#define ARB_ZEPHYR_UART_CONFIG_H

#include <zephyr/devicetree.h>

#include "RobotTargetConfig.h"

/*
 * Board DTS should declare these aliases.  An externally supplied role macro
 * always wins, which keeps one target free to use a nonstandard UART node.
 */
#if !defined(ARB_UART_RC_NODE) && !defined(ARB_UART_RC_DISABLED)
#if DT_HAS_ALIAS(uart_rc)
#define ARB_UART_RC_NODE DT_ALIAS(uart_rc)
#endif
#endif

#if !defined(ARB_UART_AUX_NODE) && !defined(ARB_UART_AUX_DISABLED)
#if DT_HAS_ALIAS(uart_aux)
#define ARB_UART_AUX_NODE DT_ALIAS(uart_aux)
#endif
#endif

/* ELRS 未显式选口时保持关闭；旧固件仍可通过 AUX 兼容路径使用串口 CRSF。 */
#if !defined(ARB_UART_ELRS_NODE) && !defined(ARB_UART_ELRS_DISABLED)
#if DT_HAS_ALIAS(uart_elrs)
#define ARB_UART_ELRS_NODE DT_ALIAS(uart_elrs)
#endif
#endif

#if !defined(ARB_UART_IMU_NODE) && !defined(ARB_UART_IMU_DISABLED)
#if DT_HAS_ALIAS(uart_imu)
#define ARB_UART_IMU_NODE DT_ALIAS(uart_imu)
#endif
#endif

#if !defined(ARB_UART_REFEREE_NODE) && !defined(ARB_UART_REFEREE_DISABLED)
#if DT_HAS_ALIAS(uart_referee)
#define ARB_UART_REFEREE_NODE DT_ALIAS(uart_referee)
#endif
#endif

#if !defined(ARB_UART_RS485_0_NODE) && !defined(ARB_UART_RS485_0_DISABLED)
#if DT_HAS_ALIAS(uart_rs485_0)
#define ARB_UART_RS485_0_NODE DT_ALIAS(uart_rs485_0)
#endif
#endif

#if !defined(ARB_UART_RS485_1_NODE) && !defined(ARB_UART_RS485_1_DISABLED)
#if DT_HAS_ALIAS(uart_rs485_1)
#define ARB_UART_RS485_1_NODE DT_ALIAS(uart_rs485_1)
#endif
#endif

/* RX inactivity period used to emulate the legacy IDLE interrupt. */
#ifndef ARB_UART_IDLE_TIMEOUT_US
#define ARB_UART_IDLE_TIMEOUT_US 1000
#endif

#endif
