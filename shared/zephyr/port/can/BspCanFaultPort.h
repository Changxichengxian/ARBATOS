/* SPDX-License-Identifier: Apache-2.0 */
#ifndef BSP_CAN_FAULT_PORT_H
#define BSP_CAN_FAULT_PORT_H

#include <stdint.h>

/* 只允许在不可恢复故障、全局中断已关闭且不再返回业务代码时使用。 */
void BspCanFaultPortAbort(void);
int BspCanFaultPortSend(uint8_t bus, uint16_t id, const uint8_t *data, uint8_t dlc);
uint8_t BspCanFaultPortSupported(void);

#endif
