#ifndef TEST_CAN_MCAN_H
#define TEST_CAN_MCAN_H

#include <stdint.h>

#define CAN_MCAN_CCCR 0x18u
#define CAN_MCAN_PSR 0x44u
#define CAN_MCAN_ILE 0x54u
#define CAN_MCAN_TXBC 0xc0u
#define CAN_MCAN_TXFQS 0xc4u
#define CAN_MCAN_TXESC 0xc8u
#define CAN_MCAN_TXBRP 0xccu
#define CAN_MCAN_TXBAR 0xd0u
#define CAN_MCAN_TXBCR 0xd4u
#define CAN_MCAN_TXBTO 0xd8u

#define CAN_MCAN_CCCR_INIT (1u << 0)
#define CAN_MCAN_CCCR_CSR (1u << 4)
#define CAN_MCAN_PSR_BO (1u << 7)
#define CAN_MCAN_TXFQS_TFQPI 0x1fu
#define CAN_MCAN_TXFQS_TFQF (1u << 21)
#define CAN_MCAN_TXBC_TBSA 0x0000ffffu
#define CAN_MCAN_TXBC_NDTB 0x003f0000u
#define CAN_MCAN_TXBC_TFQS 0x3f000000u
#define CAN_MCAN_TXESC_TBDS 0x00000007u

struct can_mcan_tx_buffer
{
    uint32_t header0;
    uint32_t header1;
    uint32_t data0;
    uint32_t data1;
};

#define CAN_MCAN_DT_MCAN_ADDR(node) (0x1000u * (node))
#define CAN_MCAN_DT_MRAM_ADDR(node) (0x10000u * (node))
#define CAN_MCAN_DT_MRBA(node) (0x10000u * (node))
#define CAN_MCAN_DT_MRAM_TX_BUFFER_OFFSET(node) 0x100u
#define CAN_MCAN_DT_MRAM_TX_BUFFER_ELEMENTS(node) 3u

#endif
