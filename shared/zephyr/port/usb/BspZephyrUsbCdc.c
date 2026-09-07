/*
 * Zephyr 4.4 CDC ACM backend for the historical CubeMX USB API.
 */
#include "BspZephyrUsbCdc.h"
#include "usbd_cdc_if.h"

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/usb/usbd.h>

#define ARB_USB_TX_QUEUE_SIZE 2048u
#define ARB_USB_RX_CHUNK_SIZE   64u

/*
 * VisionLinkRxCallback consumes the buffer synchronously.  Keeping the
 * declaration here avoids pulling the application dependency tree into this
 * low-level port.
 */
extern void VisionLinkRxCallback(uint8_t *buf, uint32_t len);

typedef struct
{
    struct k_spinlock lock;
    BspUsbCdcDiag diag;
    uint16_t tx_head;
    uint16_t tx_tail;
    uint16_t tx_used;
    uint8_t init_started;
    uint8_t tx_data[ARB_USB_TX_QUEUE_SIZE];
} ArbUsbCdcState;

static ArbUsbCdcState ArbUsbCdc;

#if defined(CONFIG_USB_DEVICE_STACK_NEXT) && \
    DT_HAS_COMPAT_STATUS_OKAY(zephyr_cdc_acm_uart)

#define ARB_USB_CDC_AVAILABLE 1
#define ARB_USB_VID           0x0483
#define ARB_USB_PID           0x5740
#define ARB_USB_MAX_POWER     0x32

static const struct device *const ArbUsbCdcDev =
    DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);

USBD_DEVICE_DEFINE(ArbUsbDevice,
                   DEVICE_DT_GET(DT_NODELABEL(zephyr_udc0)),
                   ARB_USB_VID,
                   ARB_USB_PID);
USBD_DESC_LANG_DEFINE(ArbUsbLang);
USBD_DESC_MANUFACTURER_DEFINE(ArbUsbManufacturer, "STMicroelectronics");
USBD_DESC_PRODUCT_DEFINE(ArbUsbProduct, "STM32 Virtual ComPort");
IF_ENABLED(CONFIG_HWINFO,
           (USBD_DESC_SERIAL_NUMBER_DEFINE(ArbUsbSerialNumber)));
USBD_DESC_CONFIG_DEFINE(ArbUsbFsDescription, "FS Configuration");
USBD_CONFIGURATION_DEFINE(ArbUsbFsConfiguration,
                          0,
                          ARB_USB_MAX_POWER,
                          &ArbUsbFsDescription);
USBD_DESC_CONFIG_DEFINE(ArbUsbHsDescription, "HS Configuration");
USBD_CONFIGURATION_DEFINE(ArbUsbHsConfiguration,
                          0,
                          ARB_USB_MAX_POWER,
                          &ArbUsbHsDescription);

static void ArbUsbCdcIrq(const struct device *dev, void *user_data);
static void ArbUsbCdcMessage(struct usbd_context *const context,
                             const struct usbd_msg *message);

static void ArbUsbCdcQueueResetLocked(void)
{
    ArbUsbCdc.diag.tx_flush_bytes += ArbUsbCdc.tx_used;
    ArbUsbCdc.tx_head = 0u;
    ArbUsbCdc.tx_tail = 0u;
    ArbUsbCdc.tx_used = 0u;
}

static void ArbUsbCdcSetDtr(uint8_t dtr)
{
    k_spinlock_key_t key = k_spin_lock(&ArbUsbCdc.lock);

    ArbUsbCdc.diag.dtr = dtr;
    k_spin_unlock(&ArbUsbCdc.lock, key);
}

static uint8_t ArbUsbCdcReadDtr(void)
{
    uint32_t dtr = 0u;

    if (uart_line_ctrl_get(ArbUsbCdcDev, UART_LINE_CTRL_DTR, &dtr) != 0)
    {
        ArbUsbCdcSetDtr(0u);
        return 0u;
    }

    ArbUsbCdcSetDtr((dtr != 0u) ? 1u : 0u);
    return (dtr != 0u) ? 1u : 0u;
}

static void ArbUsbCdcQueuePutLocked(const uint8_t *data, uint16_t len)
{
    uint16_t first = MIN(len,
                         (uint16_t)(ARB_USB_TX_QUEUE_SIZE -
                                    ArbUsbCdc.tx_tail));
    uint16_t second = (uint16_t)(len - first);

    memcpy(&ArbUsbCdc.tx_data[ArbUsbCdc.tx_tail], data, first);
    if (second != 0u)
    {
        memcpy(ArbUsbCdc.tx_data, &data[first], second);
    }

    ArbUsbCdc.tx_tail =
        (uint16_t)((ArbUsbCdc.tx_tail + len) % ARB_USB_TX_QUEUE_SIZE);
    ArbUsbCdc.tx_used = (uint16_t)(ArbUsbCdc.tx_used + len);
    if (ArbUsbCdc.tx_used > ArbUsbCdc.diag.tx_queue_high_water)
    {
        ArbUsbCdc.diag.tx_queue_high_water = ArbUsbCdc.tx_used;
    }
}

static uint8_t ArbUsbCdcTxReadyLocked(uint8_t dtr)
{
    return (ArbUsbCdc.diag.initialized != 0u &&
            ArbUsbCdc.diag.configured != 0u &&
            ArbUsbCdc.diag.suspended == 0u &&
            dtr != 0u) ? 1u : 0u;
}

static void ArbUsbCdcErrorAdd(void)
{
    k_spinlock_key_t key = k_spin_lock(&ArbUsbCdc.lock);

    ArbUsbCdc.diag.usb_error_count++;
    k_spin_unlock(&ArbUsbCdc.lock, key);
}

static void ArbUsbCdcMessage(struct usbd_context *const context,
                             const struct usbd_msg *message)
{
    k_spinlock_key_t key;
    int ret;

    if (message->type == USBD_MSG_VBUS_READY &&
        usbd_can_detect_vbus(context))
    {
        ret = usbd_enable(context);
        if (ret != 0 && ret != -EALREADY)
        {
            ArbUsbCdcErrorAdd();
        }
    }
    else if (message->type == USBD_MSG_VBUS_REMOVED &&
             usbd_can_detect_vbus(context))
    {
        ret = usbd_disable(context);
        if (ret != 0 && ret != -EALREADY)
        {
            ArbUsbCdcErrorAdd();
        }
    }

    key = k_spin_lock(&ArbUsbCdc.lock);
    switch (message->type)
    {
        case USBD_MSG_UDC_ERROR:
        case USBD_MSG_STACK_ERROR:
            ArbUsbCdc.diag.usb_error_count++;
            break;

        case USBD_MSG_RESET:
            ArbUsbCdc.diag.reset_count++;
            ArbUsbCdc.diag.connected = 0u;
            ArbUsbCdc.diag.configured = 0u;
            ArbUsbCdc.diag.suspended = 0u;
            ArbUsbCdc.diag.dtr = 0u;
            ArbUsbCdcQueueResetLocked();
            break;

        case USBD_MSG_VBUS_READY:
            ArbUsbCdc.diag.connect_count++;
            ArbUsbCdc.diag.connected = 1u;
            break;

        case USBD_MSG_CONFIGURATION:
            if (message->status != 0)
            {
                ArbUsbCdc.diag.configure_count++;
                ArbUsbCdc.diag.connected = 1u;
                ArbUsbCdc.diag.configured = 1u;
                ArbUsbCdc.diag.suspended = 0u;
            }
            else
            {
                ArbUsbCdc.diag.configured = 0u;
                ArbUsbCdc.diag.dtr = 0u;
                ArbUsbCdcQueueResetLocked();
            }
            break;

        case USBD_MSG_VBUS_REMOVED:
            ArbUsbCdc.diag.disconnect_count++;
            ArbUsbCdc.diag.connected = 0u;
            ArbUsbCdc.diag.configured = 0u;
            ArbUsbCdc.diag.suspended = 0u;
            ArbUsbCdc.diag.dtr = 0u;
            ArbUsbCdcQueueResetLocked();
            break;

        case USBD_MSG_SUSPEND:
            ArbUsbCdc.diag.suspend_count++;
            ArbUsbCdc.diag.suspended = 1u;
            break;

        case USBD_MSG_RESUME:
            ArbUsbCdc.diag.resume_count++;
            ArbUsbCdc.diag.suspended = 0u;
            break;

        case USBD_MSG_CDC_ACM_CONTROL_LINE_STATE:
            if (message->dev == ArbUsbCdcDev)
            {
                uint32_t dtr = 0u;

                if (uart_line_ctrl_get(ArbUsbCdcDev,
                                       UART_LINE_CTRL_DTR,
                                       &dtr) == 0)
                {
                    ArbUsbCdc.diag.dtr = (dtr != 0u) ? 1u : 0u;
                }
            }
            break;

        default:
            break;
    }
    k_spin_unlock(&ArbUsbCdc.lock, key);
}

static int ArbUsbCdcStackInit(void)
{
    int ret;

    ret = usbd_add_descriptor(&ArbUsbDevice, &ArbUsbLang);
    if (ret != 0)
    {
        return ret;
    }
    ret = usbd_add_descriptor(&ArbUsbDevice, &ArbUsbManufacturer);
    if (ret != 0)
    {
        return ret;
    }
    ret = usbd_add_descriptor(&ArbUsbDevice, &ArbUsbProduct);
    if (ret != 0)
    {
        return ret;
    }
#if defined(CONFIG_HWINFO)
    ret = usbd_add_descriptor(&ArbUsbDevice, &ArbUsbSerialNumber);
    if (ret != 0)
    {
        return ret;
    }
#endif
    if (USBD_SUPPORTS_HIGH_SPEED &&
        usbd_caps_speed(&ArbUsbDevice) == USBD_SPEED_HS)
    {
        ret = usbd_add_configuration(&ArbUsbDevice,
                                     USBD_SPEED_HS,
                                     &ArbUsbHsConfiguration);
        if (ret != 0)
        {
            return ret;
        }
        ret = usbd_register_class(&ArbUsbDevice,
                                  "cdc_acm_0",
                                  USBD_SPEED_HS,
                                  1);
        if (ret != 0)
        {
            return ret;
        }
        ret = usbd_device_set_code_triple(&ArbUsbDevice,
                                          USBD_SPEED_HS,
                                          USB_BCC_MISCELLANEOUS,
                                          0x02,
                                          0x01);
        if (ret != 0)
        {
            return ret;
        }
    }
    ret = usbd_add_configuration(&ArbUsbDevice,
                                 USBD_SPEED_FS,
                                 &ArbUsbFsConfiguration);
    if (ret != 0)
    {
        return ret;
    }
    ret = usbd_register_class(&ArbUsbDevice,
                              "cdc_acm_0",
                              USBD_SPEED_FS,
                              1);
    if (ret != 0)
    {
        return ret;
    }
    ret = usbd_device_set_code_triple(&ArbUsbDevice,
                                      USBD_SPEED_FS,
                                      USB_BCC_MISCELLANEOUS,
                                      0x02,
                                      0x01);
    if (ret != 0)
    {
        return ret;
    }
    ret = usbd_msg_register_cb(&ArbUsbDevice, ArbUsbCdcMessage);
    if (ret != 0)
    {
        return ret;
    }
    ret = usbd_init(&ArbUsbDevice);
    if (ret != 0)
    {
        return ret;
    }
    if (!usbd_can_detect_vbus(&ArbUsbDevice))
    {
        ret = usbd_enable(&ArbUsbDevice);
    }
    return ret;
}

static void ArbUsbCdcRx(const struct device *dev)
{
    uint8_t chunk[ARB_USB_RX_CHUNK_SIZE];

    while (uart_irq_rx_ready(dev) != 0)
    {
        int received = uart_fifo_read(dev, chunk, sizeof(chunk));
        k_spinlock_key_t key;

        if (received <= 0)
        {
            if (received < 0)
            {
                key = k_spin_lock(&ArbUsbCdc.lock);
                ArbUsbCdc.diag.rx_read_error_count++;
                k_spin_unlock(&ArbUsbCdc.lock, key);
            }
            break;
        }

        key = k_spin_lock(&ArbUsbCdc.lock);
        ArbUsbCdc.diag.rx_callback_count++;
        ArbUsbCdc.diag.rx_bytes += (uint32_t)received;
        k_spin_unlock(&ArbUsbCdc.lock, key);
        VisionLinkRxCallback(chunk, (uint32_t)received);
    }
}

static uint8_t ArbUsbCdcTx(const struct device *dev)
{
    uint16_t contiguous;
    int sent;
    k_spinlock_key_t key = k_spin_lock(&ArbUsbCdc.lock);

    if (ArbUsbCdc.tx_used == 0u)
    {
        k_spin_unlock(&ArbUsbCdc.lock, key);
        uart_irq_tx_disable(dev);
        return 0u;
    }

    contiguous = MIN(ArbUsbCdc.tx_used,
                     (uint16_t)(ARB_USB_TX_QUEUE_SIZE -
                                ArbUsbCdc.tx_head));
    sent = uart_fifo_fill(dev, &ArbUsbCdc.tx_data[ArbUsbCdc.tx_head],
                          contiguous);
    if (sent > 0)
    {
        ArbUsbCdc.tx_head =
            (uint16_t)((ArbUsbCdc.tx_head + (uint16_t)sent) %
                       ARB_USB_TX_QUEUE_SIZE);
        ArbUsbCdc.tx_used =
            (uint16_t)(ArbUsbCdc.tx_used - (uint16_t)sent);
        ArbUsbCdc.diag.tx_sent_bytes += (uint32_t)sent;
    }
    k_spin_unlock(&ArbUsbCdc.lock, key);

    return (sent > 0) ? 1u : 0u;
}

static void ArbUsbCdcIrq(const struct device *dev, void *user_data)
{
    ARG_UNUSED(user_data);

    while (uart_irq_update(dev) != 0 && uart_irq_is_pending(dev) != 0)
    {
        uint8_t progressed = 0u;

        if (uart_irq_rx_ready(dev) != 0)
        {
            ArbUsbCdcRx(dev);
            progressed = 1u;
        }
        if (uart_irq_tx_ready(dev) != 0)
        {
            progressed |= ArbUsbCdcTx(dev);
        }
        if (progressed == 0u)
        {
            break;
        }
    }
}

#endif

int BspUsbCdcInit(void)
{
#if defined(ARB_USB_CDC_AVAILABLE)
    int ret;
    k_spinlock_key_t key = k_spin_lock(&ArbUsbCdc.lock);

    ArbUsbCdc.diag.init_call_count++;
    if (ArbUsbCdc.diag.initialized != 0u)
    {
        k_spin_unlock(&ArbUsbCdc.lock, key);
        return 0;
    }
    if (ArbUsbCdc.init_started != 0u)
    {
        k_spin_unlock(&ArbUsbCdc.lock, key);
        return -EALREADY;
    }
    ArbUsbCdc.init_started = 1u;
    ArbUsbCdcQueueResetLocked();
    k_spin_unlock(&ArbUsbCdc.lock, key);

    if (!device_is_ready(ArbUsbCdcDev))
    {
        ret = -ENODEV;
        goto fail;
    }

    ret = uart_irq_callback_user_data_set(ArbUsbCdcDev, ArbUsbCdcIrq,
                                           NULL);
    if (ret != 0)
    {
        goto fail;
    }
    uart_irq_rx_disable(ArbUsbCdcDev);
    uart_irq_tx_disable(ArbUsbCdcDev);

    ret = ArbUsbCdcStackInit();
    if (ret != 0)
    {
        goto fail;
    }

    key = k_spin_lock(&ArbUsbCdc.lock);
    ArbUsbCdc.diag.initialized = 1u;
    ArbUsbCdc.init_started = 0u;
    k_spin_unlock(&ArbUsbCdc.lock, key);
    uart_irq_rx_enable(ArbUsbCdcDev);
    return 0;

fail:
    key = k_spin_lock(&ArbUsbCdc.lock);
    ArbUsbCdc.diag.init_fail_count++;
    ArbUsbCdc.init_started = 0u;
    k_spin_unlock(&ArbUsbCdc.lock, key);
    return ret;
#else
    k_spinlock_key_t key = k_spin_lock(&ArbUsbCdc.lock);

    ArbUsbCdc.diag.init_call_count++;
    ArbUsbCdc.diag.init_fail_count++;
    k_spin_unlock(&ArbUsbCdc.lock, key);
    return -ENOTSUP;
#endif
}

uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len)
{
#if defined(ARB_USB_CDC_AVAILABLE)
    uint8_t dtr;
    uint8_t result = USBD_OK;
    k_spinlock_key_t key;

    if (Buf == NULL && Len != 0u)
    {
        key = k_spin_lock(&ArbUsbCdc.lock);
        ArbUsbCdc.diag.tx_request_count++;
        ArbUsbCdc.diag.tx_request_bytes += Len;
        ArbUsbCdc.diag.tx_fail_count++;
        k_spin_unlock(&ArbUsbCdc.lock, key);
        return USBD_FAIL;
    }

    dtr = ArbUsbCdcReadDtr();
    key = k_spin_lock(&ArbUsbCdc.lock);
    ArbUsbCdc.diag.tx_request_count++;
    ArbUsbCdc.diag.tx_request_bytes += Len;

    if (Len > ARB_USB_TX_QUEUE_SIZE)
    {
        ArbUsbCdc.diag.tx_fail_count++;
        result = USBD_FAIL;
    }
    else if (ArbUsbCdcTxReadyLocked(dtr) == 0u)
    {
        ArbUsbCdc.diag.tx_busy_count++;
        ArbUsbCdc.diag.tx_not_ready_count++;
        result = USBD_BUSY;
    }
    else if (Len > (uint16_t)(ARB_USB_TX_QUEUE_SIZE -
                              ArbUsbCdc.tx_used))
    {
        ArbUsbCdc.diag.tx_busy_count++;
        result = USBD_BUSY;
    }
    else
    {
        if (Len != 0u)
        {
            ArbUsbCdcQueuePutLocked(Buf, Len);
        }
        ArbUsbCdc.diag.tx_accept_count++;
        ArbUsbCdc.diag.tx_accept_bytes += Len;
    }
    k_spin_unlock(&ArbUsbCdc.lock, key);

    if (result == USBD_OK && Len != 0u)
    {
        uart_irq_tx_enable(ArbUsbCdcDev);
    }
    return result;
#else
    k_spinlock_key_t key = k_spin_lock(&ArbUsbCdc.lock);

    ArbUsbCdc.diag.tx_request_count++;
    ArbUsbCdc.diag.tx_request_bytes += Len;
    ArbUsbCdc.diag.tx_fail_count++;
    k_spin_unlock(&ArbUsbCdc.lock, key);
    ARG_UNUSED(Buf);
    return USBD_FAIL;
#endif
}

void BspUsbCdcGetDiag(BspUsbCdcDiag *out)
{
    k_spinlock_key_t key;

    if (out == NULL)
    {
        return;
    }

    key = k_spin_lock(&ArbUsbCdc.lock);
    ArbUsbCdc.diag.tx_queue_used = ArbUsbCdc.tx_used;
    *out = ArbUsbCdc.diag;
    k_spin_unlock(&ArbUsbCdc.lock, key);
}
