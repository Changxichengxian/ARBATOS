/*
 * ARBATOS USB CDC backend for Zephyr 4.4's current USB device stack.
 */
#ifndef BSP_ZEPHYR_USB_CDC_H
#define BSP_ZEPHYR_USB_CDC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    uint32_t init_call_count;
    uint32_t init_fail_count;
    uint32_t connect_count;
    uint32_t configure_count;
    uint32_t disconnect_count;
    uint32_t reset_count;
    uint32_t suspend_count;
    uint32_t resume_count;
    uint32_t usb_error_count;

    uint32_t tx_request_count;
    uint32_t tx_accept_count;
    uint32_t tx_request_bytes;
    uint32_t tx_accept_bytes;
    uint32_t tx_sent_bytes;
    uint32_t tx_busy_count;
    uint32_t tx_not_ready_count;
    uint32_t tx_fail_count;
    uint32_t tx_flush_bytes;
    uint32_t tx_queue_high_water;

    uint32_t rx_callback_count;
    uint32_t rx_bytes;
    uint32_t rx_read_error_count;

    uint16_t tx_queue_used;
    uint8_t initialized;
    uint8_t connected;
    uint8_t configured;
    uint8_t suspended;
    uint8_t dtr;
} BspUsbCdcDiag;

/*
 * Called once by BspUsbDeviceInit().  Returns zero on success and a negative
 * errno value when the CDC ACM device or USB stack cannot be started.
 */
int BspUsbCdcInit(void);

void BspUsbCdcGetDiag(BspUsbCdcDiag *out);

#ifdef __cplusplus
}
#endif

#endif
