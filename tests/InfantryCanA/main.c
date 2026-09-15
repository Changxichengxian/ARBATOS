/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#define ID_CAPACITY 64U
#define CAN_BITRATE 1000000U

struct id_record {
    uint32_t id;
    uint32_t frames;
    uint8_t data[8];
    uint8_t dlc;
    bool used;
};

struct can_bus {
    const char *name;
    const struct device *dev;
    struct id_record standard[ID_CAPACITY];
    struct id_record extended[ID_CAPACITY];
    uint32_t rx_count;
    uint32_t standard_overflow;
    uint32_t extended_overflow;
    int init_error;
    int standard_filter;
    int extended_filter;
};

static struct can_bus can1 = {
    .name = "CAN1",
    .dev = DEVICE_DT_GET(DT_NODELABEL(can1)),
    .standard_filter = -1,
    .extended_filter = -1,
};
static struct can_bus can2 = {
    .name = "CAN2",
    .dev = DEVICE_DT_GET(DT_NODELABEL(can2)),
    .standard_filter = -1,
    .extended_filter = -1,
};
static const struct device *const uart6 = DEVICE_DT_GET(DT_NODELABEL(usart6));

/* 这是应用实际调用发送接口的次数，不代表总线 ACK 的物理测量值。 */
static uint32_t application_tx_count;

static void UartText(const char *text)
{
    while (*text != '\0') {
        uart_poll_out(uart6, (unsigned char)*text++);
    }
}

static void UartLine(const char *format, ...)
{
    char text[176];
    va_list args;

    va_start(args, format);
    vsnprintf(text, sizeof(text), format, args);
    va_end(args);
    UartText(text);
}

static const char *CanStateName(enum can_state state)
{
    switch (state) {
    case CAN_STATE_ERROR_ACTIVE: return "active";
    case CAN_STATE_ERROR_WARNING: return "warning";
    case CAN_STATE_ERROR_PASSIVE: return "passive";
    case CAN_STATE_BUS_OFF: return "bus-off";
    case CAN_STATE_STOPPED: return "stopped";
    default: return "unknown";
    }
}

static void CanRecordFrame(struct can_bus *bus, const struct can_frame *frame)
{
    bool is_extended = (frame->flags & CAN_FRAME_IDE) != 0U;
    struct id_record *records = is_extended ? bus->extended : bus->standard;
    uint32_t *overflow = is_extended ? &bus->extended_overflow : &bus->standard_overflow;
    struct id_record *record = NULL;
    size_t i;

    for (i = 0; i < ID_CAPACITY; ++i) {
        if (records[i].used && records[i].id == frame->id) {
            record = &records[i];
            break;
        }
        if (!records[i].used && record == NULL) {
            record = &records[i];
        }
    }
    ++bus->rx_count;
    if (record == NULL) {
        ++*overflow;
        return;
    }
    if (!record->used) {
        record->used = true;
        record->id = frame->id;
        record->frames = 0U;
    }
    ++record->frames;
    record->dlc = frame->dlc;
    for (i = 0; i < ARRAY_SIZE(record->data); ++i) {
        record->data[i] = i < frame->dlc ? frame->data[i] : 0U;
    }
}

static void CanReceiveCallback(const struct device *dev, struct can_frame *frame, void *user_data)
{
    struct can_bus *bus = user_data;
    unsigned int key;

    ARG_UNUSED(dev);
    key = irq_lock();
    CanRecordFrame(bus, frame);
    irq_unlock(key);
}

static int CanStartBus(struct can_bus *bus)
{
    const struct can_filter standard_filter = { .id = 0U, .mask = 0U, .flags = 0U };
    const struct can_filter extended_filter = {
        .id = 0U, .mask = 0U, .flags = CAN_FILTER_IDE,
    };
    int ret;

    if (!device_is_ready(bus->dev)) {
        return -ENODEV;
    }
    ret = can_set_mode(bus->dev, CAN_MODE_NORMAL);
    if (ret != 0 && ret != -EBUSY) {
        return ret;
    }
    ret = can_set_bitrate(bus->dev, CAN_BITRATE);
    if (ret != 0) {
        return ret;
    }
    bus->standard_filter = can_add_rx_filter(bus->dev, CanReceiveCallback, bus, &standard_filter);
    if (bus->standard_filter < 0) {
        return bus->standard_filter;
    }
    bus->extended_filter = can_add_rx_filter(bus->dev, CanReceiveCallback, bus, &extended_filter);
    if (bus->extended_filter < 0) {
        can_remove_rx_filter(bus->dev, bus->standard_filter);
        bus->standard_filter = -1;
        return bus->extended_filter;
    }
    ret = can_start(bus->dev);
    if (ret != 0 && ret != -EALREADY) {
        can_remove_rx_filter(bus->dev, bus->extended_filter);
        can_remove_rx_filter(bus->dev, bus->standard_filter);
        bus->extended_filter = -1;
        bus->standard_filter = -1;
        return ret;
    }
    return 0;
}

static void CanPrintRecords(struct can_bus *bus, bool extended)
{
    size_t i;

    for (i = 0; i < ID_CAPACITY; ++i) {
        struct id_record record;
        unsigned int key = irq_lock();

        record = extended ? bus->extended[i] : bus->standard[i];
        irq_unlock(key);
        if (!record.used) {
            continue;
        }
        UartLine("%s %s id=0x%0*lx frames=%lu dlc=%u data=%02x %02x %02x %02x %02x %02x %02x %02x\r\n",
                  bus->name, extended ? "ext" : "std", extended ? 8 : 3,
                  (unsigned long)record.id, (unsigned long)record.frames,
                  (unsigned int)record.dlc, (unsigned int)record.data[0],
                  (unsigned int)record.data[1], (unsigned int)record.data[2],
                  (unsigned int)record.data[3], (unsigned int)record.data[4],
                  (unsigned int)record.data[5], (unsigned int)record.data[6],
                  (unsigned int)record.data[7]);
    }
}

static void CanPrintBus(struct can_bus *bus)
{
    enum can_state state = CAN_STATE_STOPPED;
    struct can_bus_err_cnt errors = { 0 };
    uint32_t rx_count;
    uint32_t standard_overflow;
    uint32_t extended_overflow;
    int state_ret = bus->init_error == 0 ? can_get_state(bus->dev, &state, &errors) : bus->init_error;
    unsigned int key = irq_lock();

    rx_count = bus->rx_count;
    standard_overflow = bus->standard_overflow;
    extended_overflow = bus->extended_overflow;
    irq_unlock(key);
    UartLine("%s init=%d state=%s state_err=%d rx=%lu hw_txerr=%u hw_rxerr=%u overflow_std=%lu overflow_ext=%lu\r\n",
              bus->name, bus->init_error, CanStateName(state), state_ret, (unsigned long)rx_count,
              (unsigned int)errors.tx_err_cnt, (unsigned int)errors.rx_err_cnt,
              (unsigned long)standard_overflow,
              (unsigned long)extended_overflow);
    CanPrintRecords(bus, false);
    CanPrintRecords(bus, true);
}

int main(void)
{
    if (!device_is_ready(uart6)) {
        return 0;
    }
    UartLine("INFANTRY-A board=dji_a_f427 STM32F427: UART6 PG14/PG9 115200 8N1\r\n");
    UartLine("CAN1/CAN2 normal receive at 1000000 bit/s; application_tx_count always remains zero\r\n");
    can1.init_error = CanStartBus(&can1);
    can2.init_error = CanStartBus(&can2);

    while (true) {
        UartLine("INFANTRY-A uptime_ms=%lld application_tx_count=%lu\r\n",
                  (long long)k_uptime_get(), (unsigned long)application_tx_count);
        CanPrintBus(&can1);
        CanPrintBus(&can2);
        k_sleep(K_SECONDS(1));
    }
}
