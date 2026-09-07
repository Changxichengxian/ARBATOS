/* SPDX-License-Identifier: Apache-2.0 */
#include "BspRs485FaultPort.h"
#include "BspZephyrUartConfig.h"
#include <soc.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/kernel.h>

#if defined(CONFIG_SOC_STM32H723XX)
static const struct stm32_pclken Rs485Clocks0[] = STM32_DT_CLOCKS(ARB_UART_RS485_0_NODE);
static const struct stm32_pclken Rs485Clocks1[] = STM32_DT_CLOCKS(ARB_UART_RS485_1_NODE);
static USART_TypeDef *const Rs485Registers[] = {
    (USART_TypeDef *)DT_REG_ADDR(ARB_UART_RS485_0_NODE),
    (USART_TypeDef *)DT_REG_ADDR(ARB_UART_RS485_1_NODE),
};

static uint8_t Rs485FaultClockOn(uint8_t port)
{
    const uint32_t mask = (port == 0u) ? RCC_APB1LENR_USART2EN : RCC_APB1LENR_USART3EN;
    return (uint8_t)((RCC->APB1LENR & mask) != 0u);
}

uint8_t BspRs485FaultPortPrepare(uint8_t port, uint32_t baud, uint32_t *brr)
{
    if (brr == NULL || port > 1u || baud == 0u || k_is_in_isr()) {
        return 0u;
    }
    *brr = 0u;
    /* 只在正常初始化时查询时钟，异常时仅使用预先计算并纳入路由校验的 BRR。 */
    const struct stm32_pclken *clock = (port == 0u) ?
        &Rs485Clocks0[ARRAY_SIZE(Rs485Clocks0) - 1u] :
        &Rs485Clocks1[ARRAY_SIZE(Rs485Clocks1) - 1u];
    uint32_t rate = 0u;
    if (clock_control_get_rate(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE),
                               (clock_control_subsys_t)clock, &rate) != 0 || rate == 0u) {
        return 0u;
    }
    /* 固定为 16 倍采样、PRESC=1，舍入后还要求误差小于 2%。 */
    const uint32_t divisor = (uint32_t)(((uint64_t)rate + baud / 2u) / baud);
    if (divisor < 16u || divisor > 65535u) {
        return 0u;
    }
    const uint32_t actual = rate / divisor;
    const uint32_t error = (actual > baud) ? actual - baud : baud - actual;
    if ((uint64_t)error * 100u > (uint64_t)baud * 2u) {
        return 0u;
    }
    *brr = divisor;
    return 1u;
}

int BspRs485FaultPortSend(uint8_t port, uint32_t brr, const uint8_t *data, uint16_t len)
{
    if (__get_PRIMASK() == 0u || port > 1u || brr < 16u || brr > 65535u ||
        data == NULL || len == 0u || len > 64u || Rs485FaultClockOn(port) == 0u) {
        return -1;
    }
    USART_TypeDef *const uart = Rs485Registers[port];
    /* 接管端口后不再返回驱动：关 UE 丢弃旧 FIFO，以预制 8N1 安全帧重启发送。 */
    uart->CR1 = 0u;
    uart->CR2 = 0u;
    uart->CR3 = USART_CR3_DEM;
    uart->PRESC = 0u;
    uart->BRR = brr;
    uart->ICR = USART_ICR_TCCF;
    uart->CR1 = USART_CR1_UE | USART_CR1_TE;
    __DSB();
    /* 全帧共用总预算；断线、时钟故障和发送器异常都不会形成无限等待。 */
    uint32_t remaining = 300000u;
    for (uint16_t i = 0u; i < len; i++) {
        while ((uart->ISR & USART_ISR_TXE_TXFNF) == 0u && remaining != 0u) {
            remaining--;
        }
        if (remaining == 0u) {
            uart->CR1 = 0u;
            return -1;
        }
        uart->TDR = data[i];
        __DSB();
    }
    while ((uart->ISR & USART_ISR_TC) == 0u && remaining != 0u) {
        remaining--;
    }
    uart->CR1 = 0u;
    return (remaining != 0u) ? 0 : -1;
}
#else
uint8_t BspRs485FaultPortPrepare(uint8_t port, uint32_t baud, uint32_t *brr)
{
    (void)port; (void)baud;
    if (brr != NULL) { *brr = 0u; }
    return 0u;
}
int BspRs485FaultPortSend(uint8_t port, uint32_t brr, const uint8_t *data, uint16_t len)
{
    (void)port; (void)brr; (void)data; (void)len;
    return -1;
}
#endif
