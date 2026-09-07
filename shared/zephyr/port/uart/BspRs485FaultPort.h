/* SPDX-License-Identifier: Apache-2.0 */
#ifndef BSP_RS485_FAULT_PORT_H
#define BSP_RS485_FAULT_PORT_H
#include <stdint.h>
uint8_t BspRs485FaultPortPrepare(uint8_t port, uint32_t baud, uint32_t *brr);
int BspRs485FaultPortSend(uint8_t port, uint32_t brr, const uint8_t *data, uint16_t len);
#endif
