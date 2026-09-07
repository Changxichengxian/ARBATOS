/* SPDX-License-Identifier: Apache-2.0 */
#include "BspCanFaultPort.h"

#include <soc.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can/can_mcan.h>
#include <zephyr/sys/sys_io.h>

#if defined(CONFIG_SOC_STM32H723XX) && defined(CONFIG_CAN_STM32H7_FDCAN)
/* 以本机实际启用的 H7 M_CAN 驱动布局为准，禁止混用 G4 小布局寄存器。 */
typedef struct
{
    uintptr_t regs;
    uintptr_t ram;
    uintptr_t ramBase;
    uint32_t txOffset;
    uint32_t txCount;
} CanFaultPort;

#define CAN_FAULT_PORT(node) {CAN_MCAN_DT_MCAN_ADDR(node), CAN_MCAN_DT_MRAM_ADDR(node), CAN_MCAN_DT_MRBA(node), \
    CAN_MCAN_DT_MRAM_TX_BUFFER_OFFSET(node), CAN_MCAN_DT_MRAM_TX_BUFFER_ELEMENTS(node)}

static const CanFaultPort FaultPorts[] = {
    CAN_FAULT_PORT(DT_ALIAS(can_primary)),
    CAN_FAULT_PORT(DT_ALIAS(can_secondary)),
    CAN_FAULT_PORT(DT_ALIAS(can_tertiary)),
};

/* 固定上限，即使总线断线或外设不响应，也不会阻止最终复位。 */
#define CAN_FAULT_POLL_LIMIT 60000u

static uint8_t CanFaultPortReady(const CanFaultPort *port)
{
    if ((RCC->APB1HENR & RCC_APB1HENR_FDCANEN) == 0u) {
        return 0u;
    }
    return (uint8_t)((sys_read32(port->regs + CAN_MCAN_CCCR) &
                     (CAN_MCAN_CCCR_INIT | CAN_MCAN_CCCR_CSR)) == 0u &&
                    (sys_read32(port->regs + CAN_MCAN_PSR) & CAN_MCAN_PSR_BO) == 0u);
}

void BspCanFaultPortAbort(void)
{
    for (uint32_t i = 0u; i < ARRAY_SIZE(FaultPorts); i++) {
        const CanFaultPort *port = &FaultPorts[i];
        if (CanFaultPortReady(port) == 0u) {
            continue;
        }
        /* 不再返回驱动，因此可接管全部发送槽；先取消旧的非零输出帧。 */
        sys_write32(0u, port->regs + CAN_MCAN_ILE);
        uint32_t pending = sys_read32(port->regs + CAN_MCAN_TXBRP);
        sys_write32(pending, port->regs + CAN_MCAN_TXBCR);
        for (uint32_t n = 0u; n < CAN_FAULT_POLL_LIMIT; n++) {
            if ((sys_read32(port->regs + CAN_MCAN_TXBRP) & pending) == 0u) {
                break;
            }
        }
    }
    __DSB();
}

int BspCanFaultPortSend(uint8_t bus, uint16_t id, const uint8_t *data, uint8_t dlc)
{
    if (__get_PRIMASK() == 0u || bus < 1u || bus > ARRAY_SIZE(FaultPorts) ||
        id > 0x7ffu || data == NULL || dlc > 8u) {
        return 1;
    }
    const CanFaultPort *port = &FaultPorts[bus - 1u];
    if (CanFaultPortReady(port) == 0u ||
        sys_read32(port->regs + CAN_MCAN_TXBRP) != 0u) {
        return 1;
    }
    uint32_t status = sys_read32(port->regs + CAN_MCAN_TXFQS);
    uint32_t slot = FIELD_GET(CAN_MCAN_TXFQS_TFQPI, status);
    uint32_t config = sys_read32(port->regs + CAN_MCAN_TXBC);
    uint32_t expectedBase = (uint32_t)(port->ram - port->ramBase + port->txOffset);
    if ((status & CAN_MCAN_TXFQS_TFQF) != 0u || slot >= port->txCount ||
        (config & CAN_MCAN_TXBC_TBSA) != expectedBase ||
        FIELD_GET(CAN_MCAN_TXBC_NDTB, config) != 0u ||
        FIELD_GET(CAN_MCAN_TXBC_TFQS, config) != port->txCount ||
        FIELD_GET(CAN_MCAN_TXESC_TBDS, sys_read32(port->regs + CAN_MCAN_TXESC)) != 7u) {
        return 1;
    }
    uintptr_t frame = port->ram + port->txOffset + slot * sizeof(struct can_mcan_tx_buffer);
    uint32_t words[2] = {0u, 0u};
    for (uint8_t i = 0u; i < dlc; i++) {
        words[i / 4u] |= (uint32_t)data[i] << ((i % 4u) * 8u);
    }
    sys_write32((uint32_t)id << 18u, frame);
    sys_write32((uint32_t)dlc << 16u, frame + 4u);
    sys_write32(words[0], frame + 8u);
    sys_write32(words[1], frame + 12u);
    __DSB();
    const uint32_t mask = BIT(slot);
    sys_write32(mask, port->regs + CAN_MCAN_TXBAR);
    __DSB();
    for (uint32_t n = 0u; n < CAN_FAULT_POLL_LIMIT; n++) {
        /* TXBTO 由新请求清除；同时等请求退出，避免误用上一帧的完成位。 */
        if ((sys_read32(port->regs + CAN_MCAN_TXBRP) & mask) == 0u &&
            (sys_read32(port->regs + CAN_MCAN_TXBTO) & mask) != 0u) {
            return 0;
        }
    }
    sys_write32(mask, port->regs + CAN_MCAN_TXBCR);
    return 3;
}

uint8_t BspCanFaultPortSupported(void)
{
    return 1u;
}
#else
void BspCanFaultPortAbort(void) {}
int BspCanFaultPortSend(uint8_t bus, uint16_t id, const uint8_t *data, uint8_t dlc)
{
    (void)bus; (void)id; (void)data; (void)dlc;
    return 1;
}
uint8_t BspCanFaultPortSupported(void) { return 0u; }
#endif
