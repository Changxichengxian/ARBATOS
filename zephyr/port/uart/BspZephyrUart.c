/* Zephyr 4.4 implementation of the legacy ARBATOS serial BSP. */
#include "BspUsart.h"
#include "BspRc.h"
#include "BspZephyrUartConfig.h"

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/time_units.h>
#include <zephyr/sys/util.h>

#define ARB_UART_TX_MAX_LEN       BSP_USART6_RX_BUF_LENGTH
#define ARB_UART_RS485_TX_MAX_LEN BSP_RS485_TX_IT_MAX_LEN
#define ARB_UART_REFEREE_RING_SIZE 4u
#define ARB_UART_IRQ_FRAME_MAX_LEN BSP_USART6_RX_BUF_LENGTH
#define ARB_RC_RX_RING_SIZE 8u

typedef enum
{
    ArbUartRxOff = 0,
    ArbUartRxAuxByte,
    ArbUartRxAuxStream,
    ArbUartRxRc,
    ArbUartRxReferee,
    ArbUartRxRs485Byte,
} ArbUartRxMode;

typedef struct
{
    const struct device *dev;
    struct k_spinlock lock;
    struct k_sem tx_done;
    uint8_t prepared;
    uint8_t async_ok;
    uint8_t tx_busy;
    uint8_t rx_mode;
    uint8_t rx_restart;
    uint8_t rx_disable_pending;
    uint16_t rx_len;
    uint8_t *rx_buf;
    uint8_t rx_store[BSP_USART6_RX_BUF_LENGTH];
    uint8_t rx_byte;
    struct k_work_delayable irq_idle_work;
    uint8_t irq_work_ready;
    uint8_t irq_active_buf;
    uint8_t irq_overflow[2];
    uint16_t irq_len[2];
    uint32_t irq_last_cycle;
    uint8_t irq_buf[2][ARB_UART_IRQ_FRAME_MAX_LEN];
    uint8_t tx_buf[ARB_UART_TX_MAX_LEN];
    uint16_t tx_len;
    uint16_t tx_pos;
    uint32_t baudrate;
    BspAuxLinkRxEventCb aux_event_cb;
    BspAuxLinkRxByteCb aux_byte_cb;
    BspAuxLinkErrorCb aux_error_cb;
    BspUsartRxByteCb byte_cb;
    BspUsartErrorCb error_cb;
} ArbUartPort;

typedef struct
{
    uint16_t len;
    uint8_t data[BSP_USART6_RX_BUF_LENGTH];
} ArbRefereeChunk;

typedef struct
{
    uint8_t frame[BSP_RC_SBUS_FRAME_LENGTH];
} ArbRcFrame;

#ifdef ARB_UART_RC_NODE
static const struct device *const ArbRcDev = DEVICE_DT_GET(ARB_UART_RC_NODE);
#else
static const struct device *const ArbRcDev;
#endif
#ifdef ARB_UART_AUX_NODE
static const struct device *const ArbAuxDev = DEVICE_DT_GET(ARB_UART_AUX_NODE);
#else
static const struct device *const ArbAuxDev;
#endif
#ifdef ARB_UART_REFEREE_NODE
static const struct device *const ArbRefereeDev = DEVICE_DT_GET(ARB_UART_REFEREE_NODE);
#else
static const struct device *const ArbRefereeDev;
#endif
static ArbUartPort ArbRcPort = {.dev = ArbRcDev};
static ArbUartPort ArbAuxPort = {.dev = ArbAuxDev};
static ArbUartPort ArbRefereePort = {.dev = ArbRefereeDev};
static ArbUartPort ArbRs485Ports[2] = {
#ifdef ARB_UART_RS485_0_NODE
    {.dev = DEVICE_DT_GET(ARB_UART_RS485_0_NODE)},
#else
    {.dev = NULL},
#endif
#ifdef ARB_UART_RS485_1_NODE
    {.dev = DEVICE_DT_GET(ARB_UART_RS485_1_NODE)},
#else
    {.dev = NULL},
#endif
};

static uint8_t ArbRcRxBuf[BSP_RC_SBUS_FRAME_LENGTH];
static volatile BspRcDiag ArbRcDiagState;
static ArbRcFrame ArbRcRing[ARB_RC_RX_RING_SIZE];
static struct k_spinlock ArbRcRingLock;
static uint16_t ArbRcHead;
static uint16_t ArbRcTail;
static uint32_t ArbRcDrop;
static TaskHandle_t ArbRcTask;
static ArbRefereeChunk ArbRefereeRing[ARB_UART_REFEREE_RING_SIZE];
static volatile uint16_t ArbRefereeHead;
static volatile uint16_t ArbRefereeTail;
static volatile uint32_t ArbRefereeDrop;
static TaskHandle_t ArbRefereeTask;
#if defined(STM32H723xx)
static atomic_t ArbRs485FaultLockedState;
K_MUTEX_DEFINE(ArbRs485SubmitLock);
#endif

static void ArbUartAsyncCallback(const struct device *dev, struct uart_event *evt, void *user_data);
static void ArbUartIrqCallback(const struct device *dev, void *user_data);
static int ArbUartStartRx(ArbUartPort *port);
static void ArbUartIrqIdleWork(struct k_work *work);
static void BspRcPortInit(void);

static void ArbRcNotifyTask(uint8_t from_isr)
{
    if (ArbRcTask == NULL || xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED)
    {
        return;
    }
    if (from_isr != 0u)
    {
        BaseType_t higher = pdFALSE;
        vTaskNotifyGiveFromISR(ArbRcTask, &higher);
        portYIELD_FROM_ISR(higher);
    }
    else
    {
        (void)xTaskNotify(ArbRcTask, 0u, eIncrement);
    }
}

static void ArbRcSubmitFrame(const uint8_t *frame, uint16_t size, uint8_t from_isr)
{
    k_spinlock_key_t key;
    uint16_t next;

    if (frame == NULL || size != BSP_RC_SBUS_FRAME_LENGTH)
    {
        key = k_spin_lock(&ArbRcRingLock);
        ArbRcDrop++;
        k_spin_unlock(&ArbRcRingLock, key);
        return;
    }
    key = k_spin_lock(&ArbRcRingLock);
    next = (uint16_t)((ArbRcHead + 1u) & (ARB_RC_RX_RING_SIZE - 1u));
    if (next == ArbRcTail)
    {
        ArbRcDrop++;
        k_spin_unlock(&ArbRcRingLock, key);
        return;
    }
    memcpy(ArbRcRing[ArbRcHead].frame, frame, BSP_RC_SBUS_FRAME_LENGTH);
    ArbRcHead = next;
    k_spin_unlock(&ArbRcRingLock, key);
    ArbRcNotifyTask(from_isr);
}

static int ArbUartPrepare(ArbUartPort *port)
{
    int ret;
    struct uart_config config;

    if (port == NULL || port->dev == NULL || !device_is_ready(port->dev))
    {
        return -ENODEV;
    }
    if (port->prepared != 0u)
    {
        return 0;
    }

    k_sem_init(&port->tx_done, 0u, 1u);
    k_work_init_delayable(&port->irq_idle_work, ArbUartIrqIdleWork);
    port->irq_work_ready = 1u;
    if (uart_config_get(port->dev, &config) == 0)
    {
        port->baudrate = config.baudrate;
    }
    ret = uart_callback_set(port->dev, ArbUartAsyncCallback, port);
    if (ret == 0)
    {
        port->async_ok = 1u;
    }
    else
    {
        /* IRQ API is the only portable fallback when async DMA is absent. */
        port->async_ok = 0u;
        uart_irq_callback_user_data_set(port->dev, ArbUartIrqCallback, port);
        uart_irq_rx_disable(port->dev);
        uart_irq_tx_disable(port->dev);
    }
    port->prepared = 1u;
    return 0;
}

static void ArbUartNotifyReferee(void)
{
    BaseType_t higher = pdFALSE;

    if (ArbRefereeTask == NULL)
    {
        return;
    }
    vTaskNotifyGiveFromISR(ArbRefereeTask, &higher);
    portYIELD_FROM_ISR(higher);
}

static void ArbUartPushReferee(const uint8_t *data, uint16_t len)
{
    uint16_t head;
    uint16_t next;

    if (data == NULL || len == 0u || len > BSP_USART6_RX_BUF_LENGTH)
    {
        ArbRefereeDrop++;
        return;
    }
    head = ArbRefereeHead;
    next = (uint16_t)((head + 1u) & (ARB_UART_REFEREE_RING_SIZE - 1u));
    if (next == ArbRefereeTail)
    {
        ArbRefereeDrop++;
        return;
    }
    memcpy(ArbRefereeRing[head].data, data, len);
    ArbRefereeRing[head].len = len;
    ArbRefereeHead = next;
    ArbUartNotifyReferee();
}

static uint8_t ArbUartCallError(ArbUartPort *port)
{
    BspAuxLinkErrorCb aux_cb;
    BspUsartErrorCb byte_cb;
    k_spinlock_key_t key = k_spin_lock(&port->lock);
    aux_cb = port->aux_error_cb;
    byte_cb = port->error_cb;
    k_spin_unlock(&port->lock, key);

    if (port == &ArbAuxPort)
    {
        return (aux_cb != NULL && aux_cb() != 0u) ? 1u : 0u;
    }
    else if (byte_cb != NULL)
    {
        return (byte_cb() != 0u) ? 1u : 0u;
    }
    return 0u;
}

static void ArbUartDeliverByte(ArbUartPort *port, uint8_t byte)
{
    BspAuxLinkRxByteCb aux_cb;
    BspUsartRxByteCb rs485_cb;
    k_spinlock_key_t key = k_spin_lock(&port->lock);
    aux_cb = port->aux_byte_cb;
    rs485_cb = port->byte_cb;
    k_spin_unlock(&port->lock, key);

    if (port == &ArbAuxPort)
    {
        if (aux_cb != NULL)
        {
            aux_cb(byte);
        }
    }
    else if (rs485_cb != NULL)
    {
        rs485_cb(byte);
    }
}

static void ArbUartDeliverAuxEvent(ArbUartPort *port, uint16_t size, BspAuxLinkRxEvent event)
{
    BspAuxLinkRxEventCb cb;
    k_spinlock_key_t key = k_spin_lock(&port->lock);
    cb = port->aux_event_cb;
    k_spin_unlock(&port->lock, key);
    if (cb != NULL)
    {
        cb(size, event);
    }
}

static void ArbUartIrqDiscardFrame(ArbUartPort *port)
{
    k_spinlock_key_t key;
    uint8_t index;

    if (port == NULL || port->irq_work_ready == 0u)
    {
        return;
    }
    key = k_spin_lock(&port->lock);
    index = port->irq_active_buf;
    port->irq_len[index] = 0u;
    port->irq_overflow[index] = 0u;
    k_spin_unlock(&port->lock, key);
}

static void ArbUartIrqResetFrames(ArbUartPort *port)
{
    k_spinlock_key_t key;

    if (port == NULL || port->irq_work_ready == 0u)
    {
        return;
    }
    (void)k_work_cancel_delayable(&port->irq_idle_work);
    key = k_spin_lock(&port->lock);
    port->irq_active_buf = 0u;
    port->irq_len[0] = 0u;
    port->irq_len[1] = 0u;
    port->irq_overflow[0] = 0u;
    port->irq_overflow[1] = 0u;
    port->irq_last_cycle = 0u;
    k_spin_unlock(&port->lock, key);
}

static void ArbUartIrqStoreByte(ArbUartPort *port, uint8_t byte)
{
    k_spinlock_key_t key;
    uint8_t index;

    key = k_spin_lock(&port->lock);
    index = port->irq_active_buf;
    if (port->irq_len[index] < ARB_UART_IRQ_FRAME_MAX_LEN)
    {
        port->irq_buf[index][port->irq_len[index]++] = byte;
    }
    else
    {
        port->irq_overflow[index] = 1u;
    }
    port->irq_last_cycle = k_cycle_get_32();
    k_spin_unlock(&port->lock, key);
    (void)k_work_reschedule(&port->irq_idle_work, K_USEC(ARB_UART_IDLE_TIMEOUT_US));
}

static void ArbUartIrqIdleWork(struct k_work *work)
{
    struct k_work_delayable *delayable = k_work_delayable_from_work(work);
    ArbUartPort *port = CONTAINER_OF(delayable, ArbUartPort, irq_idle_work);
    k_spinlock_key_t key;
    uint8_t complete_index;
    uint16_t complete_len;
    uint8_t overflow;
    uint32_t last_cycle;
    uint32_t elapsed_us;

    key = k_spin_lock(&port->lock);
    last_cycle = port->irq_last_cycle;
    k_spin_unlock(&port->lock, key);
    elapsed_us = k_cyc_to_us_floor32(k_cycle_get_32() - last_cycle);
    if (elapsed_us < ARB_UART_IDLE_TIMEOUT_US)
    {
        (void)k_work_reschedule(&port->irq_idle_work,
                                K_USEC(ARB_UART_IDLE_TIMEOUT_US - elapsed_us));
        return;
    }

    key = k_spin_lock(&port->lock);
    if (last_cycle != port->irq_last_cycle)
    {
        k_spin_unlock(&port->lock, key);
        (void)k_work_reschedule(&port->irq_idle_work, K_USEC(ARB_UART_IDLE_TIMEOUT_US));
        return;
    }
    complete_index = port->irq_active_buf;
    complete_len = port->irq_len[complete_index];
    overflow = port->irq_overflow[complete_index];
    port->irq_active_buf ^= 1u;
    port->irq_len[port->irq_active_buf] = 0u;
    port->irq_overflow[port->irq_active_buf] = 0u;
    k_spin_unlock(&port->lock, key);

    if (complete_len == 0u)
    {
        return;
    }
    if (port->rx_mode == ArbUartRxRc)
    {
        ArbRcDiagState.rx_event_cnt++;
        ArbRcDiagState.rx_last_size = complete_len;
        ArbRcDiagState.rx_last_event = 1u;
        if (overflow != 0u || complete_len != BSP_RC_SBUS_FRAME_LENGTH)
        {
            ArbRcDiagState.rx_bad_size_cnt++;
            return;
        }
        BspRcSbusOnFrameIsr(port->irq_buf[complete_index], complete_len);
    }
    else if (port->rx_mode == ArbUartRxReferee)
    {
        if (overflow != 0u)
        {
            ArbRefereeDrop++;
            return;
        }
        ArbUartPushReferee(port->irq_buf[complete_index], complete_len);
    }
}

static int ArbUartStartRx(ArbUartPort *port)
{
    int ret;

    if (port == NULL || port->rx_mode == ArbUartRxOff)
    {
        return -EINVAL;
    }
    ret = ArbUartPrepare(port);
    if (ret != 0)
    {
        return ret;
    }
    if (port->async_ok != 0u)
    {
        return uart_rx_enable(port->dev, port->rx_buf, port->rx_len, ARB_UART_IDLE_TIMEOUT_US);
    }
    if (port->rx_mode == ArbUartRxAuxByte || port->rx_mode == ArbUartRxRs485Byte ||
        port->rx_mode == ArbUartRxRc || port->rx_mode == ArbUartRxReferee)
    {
        ArbUartIrqDiscardFrame(port);
        uart_irq_rx_enable(port->dev);
        return 0;
    }
    return -ENOTSUP;
}

static void ArbUartFinishRx(ArbUartPort *port, uint16_t size)
{
    if (port->rx_mode == ArbUartRxAuxStream)
    {
        ArbUartDeliverAuxEvent(port, size, (size >= port->rx_len) ? BSP_AUX_LINK_RXEVENT_TC : BSP_AUX_LINK_RXEVENT_IDLE);
    }
    else if (port->rx_mode == ArbUartRxRc)
    {
        ArbRcDiagState.rx_event_cnt++;
        ArbRcDiagState.rx_last_size = size;
        ArbRcDiagState.rx_last_event = (size >= port->rx_len) ? 3u : 1u;
        if (size != BSP_RC_SBUS_FRAME_LENGTH)
        {
            ArbRcDiagState.rx_bad_size_cnt++;
        }
        BspRcSbusOnFrameIsr(ArbRcRxBuf, size);
    }
    else if (port->rx_mode == ArbUartRxReferee)
    {
        ArbUartPushReferee(port->rx_buf, size);
    }
}

static void ArbUartAsyncCallback(const struct device *dev, struct uart_event *evt, void *user_data)
{
    ArbUartPort *port = user_data;
    uint16_t size;

    ARG_UNUSED(dev);
    if (port == NULL || evt == NULL)
    {
        return;
    }
    switch (evt->type)
    {
    case UART_TX_DONE:
    case UART_TX_ABORTED:
        port->tx_busy = 0u;
        k_sem_give(&port->tx_done);
        break;
    case UART_RX_RDY:
        if (evt->data.rx.buf != port->rx_buf || port->rx_mode == ArbUartRxOff ||
            port->rx_disable_pending != 0u)
        {
            break;
        }
        size = (uint16_t)(evt->data.rx.offset + evt->data.rx.len);
        if (port->rx_mode == ArbUartRxAuxStream)
        {
            ArbUartDeliverAuxEvent(port, size,
                                   (size >= port->rx_len) ? BSP_AUX_LINK_RXEVENT_TC : BSP_AUX_LINK_RXEVENT_IDLE);
            if (size >= port->rx_len)
            {
                port->rx_restart = 1u;
                port->rx_disable_pending = 1u;
                (void)uart_rx_disable(port->dev);
            }
        }
        else if (port->rx_mode == ArbUartRxRc || port->rx_mode == ArbUartRxReferee)
        {
            ArbUartFinishRx(port, size);
            port->rx_restart = 1u;
            port->rx_disable_pending = 1u;
            (void)uart_rx_disable(port->dev);
        }
        else
        {
            ArbUartDeliverByte(port, port->rx_byte);
            port->rx_restart = 1u;
            port->rx_disable_pending = 1u;
            (void)uart_rx_disable(port->dev);
        }
        break;
    case UART_RX_STOPPED:
        if (port == &ArbRcPort)
        {
            ArbRcDiagState.uart_error_cnt++;
            ArbRcDiagState.uart_last_error = (uint32_t)evt->data.rx_stop.reason;
        }
        /* RX_STOPPED may be followed by RX_RDY for stale partial bytes. */
        port->rx_disable_pending = 1u;
        port->rx_restart = (ArbUartCallError(port) == 0u) ? 1u : 0u;
        break;
    case UART_RX_DISABLED:
        port->rx_disable_pending = 0u;
        if (port->rx_restart != 0u && port->rx_mode != ArbUartRxOff)
        {
            port->rx_restart = 0u;
            (void)ArbUartStartRx(port);
        }
        break;
    default:
        break;
    }
}

static void ArbUartIrqCallback(const struct device *dev, void *user_data)
{
    ArbUartPort *port = user_data;
    uint8_t byte;
    int count;
    int error;

    if (port == NULL || uart_irq_update(dev) == 0)
    {
        return;
    }
    error = uart_err_check(dev);
    if (error != 0)
    {
        if (port == &ArbRcPort)
        {
            ArbRcDiagState.uart_error_cnt++;
            ArbRcDiagState.uart_last_error = (uint32_t)error;
        }
        ArbUartIrqDiscardFrame(port);
        (void)ArbUartCallError(port);
        /* Error destroys the byte boundary; drain stale FIFO content. */
        while (uart_irq_rx_ready(dev) != 0 && uart_fifo_read(dev, &byte, 1) > 0)
        {
        }
        return;
    }
    if (uart_irq_rx_ready(dev) != 0)
    {
        while ((count = uart_fifo_read(dev, &byte, 1)) > 0)
        {
            if (port->rx_mode == ArbUartRxRc || port->rx_mode == ArbUartRxReferee)
            {
                ArbUartIrqStoreByte(port, byte);
            }
            else
            {
                ArbUartDeliverByte(port, byte);
            }
        }
    }
    if (uart_irq_tx_ready(dev) != 0)
    {
        while (port->tx_pos < port->tx_len)
        {
            count = uart_fifo_fill(dev, &port->tx_buf[port->tx_pos], (int)(port->tx_len - port->tx_pos));
            if (count <= 0)
            {
                break;
            }
            port->tx_pos = (uint16_t)(port->tx_pos + (uint16_t)count);
        }
        if (port->tx_pos >= port->tx_len)
        {
            uart_irq_tx_disable(dev);
            port->tx_busy = 0u;
            k_sem_give(&port->tx_done);
        }
    }
}

static int ArbUartSetBaudrate(ArbUartPort *port, uint32_t baudrate)
{
    struct uart_config config;
    int ret;

    if (baudrate == 0u)
    {
        return -EINVAL;
    }
    ret = ArbUartPrepare(port);
    if (ret != 0)
    {
        return ret;
    }
    if (port->baudrate == baudrate)
    {
        return 0;
    }
    ret = uart_config_get(port->dev, &config);
    if (ret != 0)
    {
        return ret;
    }
    (void)uart_rx_disable(port->dev);
    config.baudrate = baudrate;
    ret = uart_configure(port->dev, &config);
    if (ret == 0)
    {
        port->baudrate = baudrate;
        if (port->rx_mode != ArbUartRxOff)
        {
            ret = ArbUartStartRx(port);
        }
    }
    return ret;
}

static int ArbRcConfigureSbus(void)
{
    struct uart_config config;
    int ret;

    ret = ArbUartPrepare(&ArbRcPort);
    if (ret != 0)
    {
        return ret;
    }
    ret = uart_config_get(ArbRcPort.dev, &config);
    if (ret != 0)
    {
        return ret;
    }
    config.baudrate = 100000u;
    config.parity = UART_CFG_PARITY_EVEN;
    config.stop_bits = UART_CFG_STOP_BITS_2;
    config.data_bits = UART_CFG_DATA_BITS_8;
    config.flow_ctrl = UART_CFG_FLOW_CTRL_NONE;
    ret = uart_configure(ArbRcPort.dev, &config);
    if (ret == 0)
    {
        ArbRcPort.baudrate = config.baudrate;
    }
    return ret;
}

static int ArbUartTxStart(ArbUartPort *port, const uint8_t *data, uint16_t len, uint16_t max_len)
{
    int ret;
    k_spinlock_key_t key;

    if (data == NULL || len == 0u || len > max_len)
    {
        return -EMSGSIZE;
    }
    ret = ArbUartPrepare(port);
    if (ret != 0)
    {
        return ret;
    }
    key = k_spin_lock(&port->lock);
    if (port->tx_busy != 0u)
    {
        k_spin_unlock(&port->lock, key);
        return -EBUSY;
    }
    memcpy(port->tx_buf, data, len);
    port->tx_len = len;
    port->tx_pos = 0u;
    port->tx_busy = 1u;
    k_sem_reset(&port->tx_done);
    k_spin_unlock(&port->lock, key);

    if (port->async_ok != 0u)
    {
        ret = uart_tx(port->dev, port->tx_buf, len, SYS_FOREVER_US);
    }
    else
    {
        uart_irq_tx_enable(port->dev);
        ret = 0;
    }
    if (ret != 0)
    {
        port->tx_busy = 0u;
    }
    return ret;
}

/* ===== RC / SBUS ===== */
void BspRcSbusInit(void)
{
    k_spinlock_key_t key = k_spin_lock(&ArbRcRingLock);
    ArbRcHead = 0u;
    ArbRcTail = 0u;
    ArbRcDrop = 0u;
    k_spin_unlock(&ArbRcRingLock, key);
    BspRcPortInit();
}

void BspRcSbusRxAttachTask(TaskHandle_t task)
{
    ArbRcTask = task;
}

uint8_t BspRcSbusRxPop(uint8_t frame[BSP_RC_SBUS_FRAME_LENGTH])
{
    k_spinlock_key_t key;
    uint16_t tail;

    if (frame == NULL)
    {
        return 0u;
    }
    key = k_spin_lock(&ArbRcRingLock);
    if (ArbRcTail == ArbRcHead)
    {
        k_spin_unlock(&ArbRcRingLock, key);
        return 0u;
    }
    tail = ArbRcTail;
    memcpy(frame, ArbRcRing[tail].frame, BSP_RC_SBUS_FRAME_LENGTH);
    ArbRcTail = (uint16_t)((tail + 1u) & (ARB_RC_RX_RING_SIZE - 1u));
    k_spin_unlock(&ArbRcRingLock, key);
    return 1u;
}

uint32_t BspRcSbusRxGetDropCount(void)
{
    k_spinlock_key_t key = k_spin_lock(&ArbRcRingLock);
    const uint32_t drop = ArbRcDrop;
    k_spin_unlock(&ArbRcRingLock, key);
    return drop;
}

void BspRcSbusOnFrameIsr(const uint8_t *frame, uint16_t size)
{
    ArbRcSubmitFrame(frame, size, k_is_in_isr() ? 1u : 0u);
}

static void BspRcPortInit(void)
{
    memset((void *)&ArbRcDiagState, 0, sizeof(ArbRcDiagState));
    (void)RC_Init(NULL, NULL, 0u);
}

void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    int ret;
    ARG_UNUSED(rx1_buf); ARG_UNUSED(rx2_buf); ARG_UNUSED(dma_buf_num);
    ArbRcPort.rx_mode = ArbUartRxRc;
    ArbRcPort.rx_buf = ArbRcRxBuf;
    ArbRcPort.rx_len = sizeof(ArbRcRxBuf);
    if (ArbRcPort.dev != NULL)
    {
        (void)uart_rx_disable(ArbRcPort.dev);
    }
    ArbUartIrqResetFrames(&ArbRcPort);
    ret = ArbRcConfigureSbus();
    if (ret == 0)
    {
        ret = ArbUartStartRx(&ArbRcPort);
    }
    if (ret != 0)
    {
        ArbRcDiagState.uart_error_cnt++;
        ArbRcDiagState.uart_last_error = (uint32_t)ret;
    }
}

void RC_unable(void)
{
    ArbRcPort.rx_mode = ArbUartRxOff;
    ArbUartIrqResetFrames(&ArbRcPort);
    if (ArbRcPort.dev != NULL)
    {
        (void)uart_rx_disable(ArbRcPort.dev);
        uart_irq_rx_disable(ArbRcPort.dev);
    }
}

void RC_restart(uint16_t dma_buf_num)
{
    ARG_UNUSED(dma_buf_num);
    ArbRcDiagState.restart_cnt++;
    RC_Init(NULL, NULL, 0u);
}

void BspRcGetDiag(BspRcDiag *out)
{
    if (out != NULL)
    {
        *out = ArbRcDiagState;
        out->drop_cnt = BspRcSbusRxGetDropCount();
    }
}

/* ===== referee UART ===== */
void BspRefereeUartInit(void)
{
    int ret;
    ArbRefereeHead = 0u; ArbRefereeTail = 0u; ArbRefereeDrop = 0u;
    ArbRefereePort.rx_mode = ArbUartRxReferee;
    ArbRefereePort.rx_buf = ArbRefereePort.rx_store;
    ArbRefereePort.rx_len = BSP_USART6_RX_BUF_LENGTH;
    if (ArbRefereePort.dev != NULL)
    {
        (void)uart_rx_disable(ArbRefereePort.dev);
    }
    ArbUartIrqResetFrames(&ArbRefereePort);
    ret = ArbUartStartRx(&ArbRefereePort);
    if (ret != 0)
    {
        printk("ARBATOS referee UART unavailable: %d\n", ret);
    }
}

void BspRefereeRxAttachTask(TaskHandle_t task) { ArbRefereeTask = task; }
int BspRefereeRxPop(uint8_t *out, uint16_t *out_len)
{
    uint16_t tail;
    uint16_t len;
    if (out == NULL || out_len == NULL || ArbRefereeTail == ArbRefereeHead) { return 0; }
    tail = ArbRefereeTail; len = ArbRefereeRing[tail].len;
    if (len == 0u || len > BSP_USART6_RX_BUF_LENGTH) { ArbRefereeTail = (uint16_t)((tail + 1u) & 3u); ArbRefereeDrop++; return 0; }
    memcpy(out, ArbRefereeRing[tail].data, len); *out_len = len;
    ArbRefereeTail = (uint16_t)((tail + 1u) & 3u); return 1;
}
uint32_t BspRefereeRxGetDropCount(void) { return ArbRefereeDrop; }
int BspRefereeTx(const uint8_t *data, uint16_t len) { return ArbUartTxStart(&ArbRefereePort, data, len, ARB_UART_TX_MAX_LEN); }
uint8_t BspRefereeTxReady(void) { return (ArbRefereePort.tx_busy == 0u) ? 1u : 0u; }
void BspUsart6RefereeInit(void) { BspRefereeUartInit(); }
void BspUsart6RxAttachTask(TaskHandle_t task) { BspRefereeRxAttachTask(task); }
int BspUsart6RxPop(uint8_t *out, uint16_t *out_len) { return BspRefereeRxPop(out, out_len); }
uint32_t BspUsart6RxGetDropCount(void) { return BspRefereeRxGetDropCount(); }
void usart6_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    ARG_UNUSED(rx1_buf); ARG_UNUSED(rx2_buf); ARG_UNUSED(dma_buf_num);
    BspRefereeUartInit();
}
void usart6_tx_dma_enable(uint8_t *data, uint16_t len) { (void)BspRefereeTx(data, len); }

/* ===== AUX / Host / ELRS ===== */
void BspAuxLinkSetRxEventCb(BspAuxLinkRxEventCb cb)
{
    k_spinlock_key_t key = k_spin_lock(&ArbAuxPort.lock); ArbAuxPort.aux_event_cb = cb; k_spin_unlock(&ArbAuxPort.lock, key);
}
void BspAuxLinkSetRxByteCb(BspAuxLinkRxByteCb cb)
{
    k_spinlock_key_t key = k_spin_lock(&ArbAuxPort.lock); ArbAuxPort.aux_byte_cb = cb; k_spin_unlock(&ArbAuxPort.lock, key);
}
void BspAuxLinkSetErrorCb(BspAuxLinkErrorCb cb)
{
    k_spinlock_key_t key = k_spin_lock(&ArbAuxPort.lock); ArbAuxPort.aux_error_cb = cb; k_spin_unlock(&ArbAuxPort.lock, key);
}
uint32_t BspAuxLinkGetBaudrate(void) { return ArbAuxPort.baudrate; }
int BspAuxLinkSetBaudrate(uint32_t baudrate) { return ArbUartSetBaudrate(&ArbAuxPort, baudrate); }
int BspAuxLinkTxDma(const uint8_t *data, uint16_t len) { return ArbUartTxStart(&ArbAuxPort, data, len, ARB_UART_TX_MAX_LEN); }
uint8_t BspAuxLinkTxReady(void) { return (ArbAuxPort.tx_busy == 0u) ? 1u : 0u; }
uint8_t BspAuxLinkRxHasDma(void) { return (ArbUartPrepare(&ArbAuxPort) == 0 && ArbAuxPort.async_ok != 0u) ? 1u : 0u; }
int BspAuxLinkRxItStart(void)
{
    ArbAuxPort.rx_mode = ArbUartRxAuxByte; ArbAuxPort.rx_buf = &ArbAuxPort.rx_byte; ArbAuxPort.rx_len = 1u;
    if (ArbAuxPort.dev != NULL) { (void)uart_rx_disable(ArbAuxPort.dev); }
    return ArbUartStartRx(&ArbAuxPort);
}
void BspAuxLinkRxItStop(void)
{
    ArbAuxPort.rx_mode = ArbUartRxOff;
    if (ArbAuxPort.dev != NULL) { (void)uart_rx_disable(ArbAuxPort.dev); uart_irq_rx_disable(ArbAuxPort.dev); }
}
int BspAuxLinkRxToIdleDmaStart(uint8_t *buf, uint16_t len)
{
    if (buf == NULL || len == 0u) { return -EINVAL; }
    if (ArbUartPrepare(&ArbAuxPort) != 0 || ArbAuxPort.async_ok == 0u) { return -ENOTSUP; }
    ArbAuxPort.rx_mode = ArbUartRxAuxStream; ArbAuxPort.rx_buf = buf; ArbAuxPort.rx_len = len;
    (void)uart_rx_disable(ArbAuxPort.dev);
    return ArbUartStartRx(&ArbAuxPort);
}
void usart1_tx_dma_init(void) { (void)ArbUartPrepare(&ArbAuxPort); }
void usart1_tx_dma_enable(uint8_t *data, uint16_t len) { (void)BspAuxLinkTxDma(data, len); }

/* ===== RS485 ===== */
static ArbUartPort *ArbRs485Port(uint8_t index) { return (index < ARRAY_SIZE(ArbRs485Ports)) ? &ArbRs485Ports[index] : NULL; }
static void ArbRs485SetByteCb(uint8_t index, BspUsartRxByteCb cb) { ArbUartPort *p = ArbRs485Port(index); if (p != NULL) { k_spinlock_key_t key = k_spin_lock(&p->lock); p->byte_cb = cb; k_spin_unlock(&p->lock, key); } }
static void ArbRs485SetErrorCb(uint8_t index, BspUsartErrorCb cb) { ArbUartPort *p = ArbRs485Port(index); if (p != NULL) { k_spinlock_key_t key = k_spin_lock(&p->lock); p->error_cb = cb; k_spin_unlock(&p->lock, key); } }
static uint32_t ArbRs485GetBaud(uint8_t index) { ArbUartPort *p = ArbRs485Port(index); return (p != NULL) ? p->baudrate : 0u; }
static int ArbRs485SetBaud(uint8_t index, uint32_t baud) { ArbUartPort *p = ArbRs485Port(index); return (p == NULL) ? -ENODEV : ArbUartSetBaudrate(p, baud); }
static int ArbRs485Tx(uint8_t index, const uint8_t *data, uint16_t len)
{
    ArbUartPort *p = ArbRs485Port(index);
#if defined(STM32H723xx)
    if (p == NULL)
    {
        return -ENODEV;
    }
    if (k_is_in_isr() || k_is_pre_kernel())
    {
        return -EWOULDBLOCK;
    }
    if (k_mutex_lock(&ArbRs485SubmitLock, K_FOREVER) != 0)
    {
        return -EIO;
    }
    if (atomic_get(&ArbRs485FaultLockedState) != 0)
    {
        k_mutex_unlock(&ArbRs485SubmitLock);
        return -EPERM;
    }
    const int result = ArbUartTxStart(p, data, len, ARB_UART_RS485_TX_MAX_LEN);
    k_mutex_unlock(&ArbRs485SubmitLock);
    return result;
#else
    return (p == NULL) ? -ENODEV :
           ArbUartTxStart(p, data, len, ARB_UART_RS485_TX_MAX_LEN);
#endif
}
static int ArbRs485Wait(uint8_t index, uint32_t timeout_ms)
{
    ArbUartPort *p = ArbRs485Port(index); if (p == NULL) { return -ENODEV; }
    return (k_sem_take(&p->tx_done, K_MSEC(timeout_ms)) == 0) ? 0 : -ETIMEDOUT;
}
static int ArbRs485RxStart(uint8_t index)
{
    ArbUartPort *p = ArbRs485Port(index); if (p == NULL) { return -ENODEV; }
    p->rx_mode = ArbUartRxRs485Byte; p->rx_buf = &p->rx_byte; p->rx_len = 1u;
    if (p->dev != NULL) { (void)uart_rx_disable(p->dev); }
    return ArbUartStartRx(p);
}
static void ArbRs485RxStop(uint8_t index)
{
    ArbUartPort *p = ArbRs485Port(index); if (p != NULL && p->dev != NULL) { p->rx_mode = ArbUartRxOff; (void)uart_rx_disable(p->dev); uart_irq_rx_disable(p->dev); }
}
void BspUsart2SetRxByteCb(BspUsartRxByteCb cb) { ArbRs485SetByteCb(0u, cb); }
void BspUsart2SetErrorCb(BspUsartErrorCb cb) { ArbRs485SetErrorCb(0u, cb); }
uint32_t BspUsart2GetBaudrate(void) { return ArbRs485GetBaud(0u); }
int BspUsart2SetBaudrate(uint32_t baudrate) { return ArbRs485SetBaud(0u, baudrate); }
int BspUsart2TxItPrepare(void) { return ArbUartPrepare(ArbRs485Port(0u)); }
int BspUsart2TxItStart(const uint8_t *data, uint16_t len) { return ArbRs485Tx(0u, data, len); }
int BspUsart2TxItWait(uint32_t timeout_ms) { return ArbRs485Wait(0u, timeout_ms); }
int BspUsart2RxItStart(void) { return ArbRs485RxStart(0u); }
void BspUsart2RxItStop(void) { ArbRs485RxStop(0u); }
void BspUsart3SetRxByteCb(BspUsartRxByteCb cb) { ArbRs485SetByteCb(1u, cb); }
void BspUsart3SetErrorCb(BspUsartErrorCb cb) { ArbRs485SetErrorCb(1u, cb); }
uint32_t BspUsart3GetBaudrate(void) { return ArbRs485GetBaud(1u); }
int BspUsart3SetBaudrate(uint32_t baudrate) { return ArbRs485SetBaud(1u, baudrate); }
int BspUsart3TxItPrepare(void) { return ArbUartPrepare(ArbRs485Port(1u)); }
int BspUsart3TxItStart(const uint8_t *data, uint16_t len) { return ArbRs485Tx(1u, data, len); }
int BspUsart3TxItWait(uint32_t timeout_ms) { return ArbRs485Wait(1u, timeout_ms); }
int BspUsart3RxItStart(void) { return ArbRs485RxStart(1u); }
void BspUsart3RxItStop(void) { ArbRs485RxStop(1u); }

#if defined(STM32H723xx)
uint8_t BspRs485FaultBaudPrepare(uint8_t port, uint32_t baudrate, uint32_t *out_brr)
{
    ARG_UNUSED(port);
    ARG_UNUSED(baudrate);
    if (out_brr != NULL)
    {
        *out_brr = 0u;
    }
    /* Zephyr's public UART API cannot reserve a register-level fault path. */
    return 0u;
}

void BspRs485FaultLock(void)
{
    /*
     * 异常/中断上下文不能等待可能被当前异常打断的发送任务；原子置位后系统会
     * 直接走复位。正常任务上下文则和最终驱动提交共用互斥量，锁函数返回后
     * 不会再出现新的普通 RS485 提交。
     */
    if (k_is_in_isr() || k_is_pre_kernel())
    {
        atomic_set(&ArbRs485FaultLockedState, 1);
        return;
    }
    if (k_mutex_lock(&ArbRs485SubmitLock, K_FOREVER) == 0)
    {
        atomic_set(&ArbRs485FaultLockedState, 1);
        k_mutex_unlock(&ArbRs485SubmitLock);
    }
    else
    {
        atomic_set(&ArbRs485FaultLockedState, 1);
    }
}

int BspRs485FaultTx(uint8_t port,
                    uint32_t baudrate,
                    uint32_t brr,
                    const uint8_t *data,
                    uint16_t len)
{
    ARG_UNUSED(port);
    ARG_UNUSED(baudrate);
    ARG_UNUSED(brr);
    ARG_UNUSED(data);
    ARG_UNUSED(len);
    /* Never claim that emergency output was delivered when it was not. */
    return -ENOTSUP;
}

uint8_t BspRs485FaultLocked(void)
{
    return (atomic_get(&ArbRs485FaultLockedState) != 0) ? 1u : 0u;
}
#endif
