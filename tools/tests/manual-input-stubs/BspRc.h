#ifndef MANUAL_INPUT_TEST_BSP_RC_H
#define MANUAL_INPUT_TEST_BSP_RC_H

#include <stdint.h>

#define BSP_RC_SBUS_RX_BUF_NUM      36u
#define BSP_RC_SBUS_FRAME_LENGTH    18u

void BspRcSbusInit(void);
void RC_restart(uint16_t dma_buf_num);

#endif
