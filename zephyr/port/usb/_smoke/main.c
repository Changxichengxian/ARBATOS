#include "BspZephyrUsbCdc.h"
#include "usbd_cdc_if.h"

#include <zephyr/kernel.h>

static volatile uint32_t SmokeRxBytes;

void VisionLinkRxCallback(uint8_t *buf, uint32_t len)
{
    ARG_UNUSED(buf);
    SmokeRxBytes += len;
}

int main(void)
{
    static uint8_t sample[] = "arbatos usb cdc\r\n";
    BspUsbCdcDiag diag;

    (void)BspUsbCdcInit();
    (void)CDC_Transmit_FS(sample, sizeof(sample) - 1u);
    BspUsbCdcGetDiag(&diag);
    return (int)(diag.rx_bytes + SmokeRxBytes);
}
