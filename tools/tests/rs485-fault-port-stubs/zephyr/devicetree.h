#ifndef RS485_FAULT_PORT_TEST_DT_H
#define RS485_FAULT_PORT_TEST_DT_H

#include <stdint.h>
#include "soc.h"

#define DT_ALIAS(name) DT_ALIAS_EXPAND(name)
#define DT_ALIAS_EXPAND(name) DT_ALIAS_##name
#define DT_HAS_ALIAS(name) 1
#define DT_ALIAS_uart_rs485_0 0
#define DT_ALIAS_uart_rs485_1 1
#define DT_REG_ADDR(node) ((uintptr_t)&Rs485TestUarts[(node)])
#define STM32_DT_CLOCKS(node) { { (uint32_t)(node) } }
#define ARRAY_SIZE(array) (sizeof(array) / sizeof((array)[0]))

#endif
