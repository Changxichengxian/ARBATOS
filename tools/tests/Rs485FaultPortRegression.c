/* Direct host regression of the RS485 fatal raw-register port. */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define CONFIG_SOC_STM32H723XX 1
#include "soc.h"
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>

Rs485TestRcc Rs485TestRccRegs;
USART_TypeDef Rs485TestUarts[2];
uint32_t Rs485TestPrimask;
int Rs485TestInIsr;
int Rs485TestClockResult;
uint32_t Rs485TestClockRate;
struct device Rs485TestClockDevice;

int clock_control_get_rate(const struct device *dev, clock_control_subsys_t subsys, uint32_t *rate)
{
    (void)dev;
    (void)subsys;
    if (Rs485TestClockResult != 0 || rate == NULL) return -1;
    *rate = Rs485TestClockRate;
    return 0;
}

#include "../../shared/zephyr/port/uart/BspRs485FaultPort.c"

static int Check(int condition, const char *message)
{
    if (condition != 0) return 1;
    (void)fprintf(stderr, "FAIL: %s\n", message);
    return 0;
}

static void Reset(void)
{
    (void)memset(&Rs485TestRccRegs, 0, sizeof(Rs485TestRccRegs));
    (void)memset(Rs485TestUarts, 0, sizeof(Rs485TestUarts));
    Rs485TestPrimask = 1u;
    Rs485TestInIsr = 0;
    Rs485TestClockResult = 0;
    Rs485TestClockRate = 80000000u;
}

static int TestPrepare(void)
{
    uint32_t brr = 99u;
    Reset();
    if (!Check(BspRs485FaultPortPrepare(0u, 115200u, &brr) != 0u && brr == 694u,
               "80MHz/115200 must produce rounded BRR 694")) return 0;
    Rs485TestInIsr = 1;
    if (!Check(BspRs485FaultPortPrepare(0u, 115200u, &brr) == 0u && brr == 694u,
               "prepare must reject ISR context")) return 0;
    Reset();
    Rs485TestClockResult = -1;
    if (!Check(BspRs485FaultPortPrepare(0u, 115200u, &brr) == 0u,
               "failed kernel clock query must reject prepare")) return 0;
    Reset();
    return Check(BspRs485FaultPortPrepare(2u, 115200u, &brr) == 0u &&
                 BspRs485FaultPortPrepare(0u, 0u, &brr) == 0u &&
                 BspRs485FaultPortPrepare(0u, 115200u, NULL) == 0u,
                 "bad prepare arguments must reject");
}

static int TestSendBoundariesAndSuccess(void)
{
    const uint8_t bytes[] = {0x12u, 0x34u};
    Reset();
    Rs485TestRccRegs.APB1LENR = RCC_APB1LENR_USART2EN;
    Rs485TestPrimask = 0u;
    if (!Check(BspRs485FaultPortSend(0u, 694u, bytes, 2u) != 0,
               "fatal send without irq lock must reject")) return 0;
    Rs485TestPrimask = 1u;
    Rs485TestRccRegs.APB1LENR = 0u;
    if (!Check(BspRs485FaultPortSend(0u, 694u, bytes, 2u) != 0,
               "clock-off port must reject")) return 0;
    Rs485TestRccRegs.APB1LENR = RCC_APB1LENR_USART2EN;
    if (!Check(BspRs485FaultPortSend(0u, 15u, bytes, 2u) != 0 &&
               BspRs485FaultPortSend(2u, 694u, bytes, 2u) != 0 &&
               BspRs485FaultPortSend(0u, 694u, NULL, 2u) != 0 &&
               BspRs485FaultPortSend(0u, 694u, bytes, 0u) != 0,
               "bad send arguments must reject")) return 0;
    Rs485TestUarts[0].ISR = USART_ISR_TXE_TXFNF | USART_ISR_TC;
    if (!Check(BspRs485FaultPortSend(0u, 694u, bytes, 2u) == 0,
               "ready TXE/TC must send bounded safety frame")) return 0;
    return Check(Rs485TestUarts[0].BRR == 694u && Rs485TestUarts[0].TDR == 0x34u &&
                 Rs485TestUarts[0].CR1 == 0u && Rs485TestUarts[0].CR3 == USART_CR3_DEM,
                 "success must configure frame then disable transmitter");
}

static int TestTimeoutsDisableTransmitter(void)
{
    const uint8_t byte = 0x7eu;
    Reset();
    Rs485TestRccRegs.APB1LENR = RCC_APB1LENR_USART2EN;
    if (!Check(BspRs485FaultPortSend(0u, 694u, &byte, 1u) != 0 &&
               Rs485TestUarts[0].CR1 == 0u,
               "TXE timeout must disable transmitter")) return 0;
    Reset();
    Rs485TestRccRegs.APB1LENR = RCC_APB1LENR_USART2EN;
    Rs485TestUarts[0].ISR = USART_ISR_TXE_TXFNF;
    return Check(BspRs485FaultPortSend(0u, 694u, &byte, 1u) != 0 &&
                 Rs485TestUarts[0].CR1 == 0u && Rs485TestUarts[0].TDR == byte,
                 "TC timeout must disable transmitter after writing byte");
}

int main(void)
{
    if (!TestPrepare() || !TestSendBoundariesAndSuccess() || !TestTimeoutsDisableTransmitter()) return 1;
    (void)puts("PASS: RS485 fault port host regression");
    return 0;
}
