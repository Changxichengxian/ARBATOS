/*
 * Zephyr 4.4 compatibility interface for the CubeMX USB CDC API.
 */
#ifndef ARBATOS_ZEPHYR_USBD_CDC_IF_H
#define ARBATOS_ZEPHYR_USBD_CDC_IF_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Keep the values used by ST's USBD_StatusTypeDef. */
#define USBD_OK   0u
#define USBD_BUSY 1u
#define USBD_FAIL 3u

/*
 * The buffer is copied before this function returns.  The call never waits
 * for the USB host.  A full queue or a host that is not ready returns BUSY.
 */
uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len);

#ifdef __cplusplus
}
#endif

#endif
