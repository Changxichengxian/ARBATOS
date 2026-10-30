/* SPDX-License-Identifier: Apache-2.0 */
#include "BspBmi088Port.h"
#include "SensorsDt.h"
#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>

#if !DT_NODE_HAS_PROP(ARBATOS_SENSORS_NODE, bmi088_spi) || \
    !DT_NODE_HAS_PROP(ARBATOS_SENSORS_NODE, bmi088_accel_cs_gpios) || \
    !DT_NODE_HAS_PROP(ARBATOS_SENSORS_NODE, bmi088_gyro_cs_gpios)
#error "BMI088 需要 bmi088-spi、bmi088-accel-cs-gpios 和 bmi088-gyro-cs-gpios。"
#endif

static const struct device *const Bmi088Spi = DEVICE_DT_GET(DT_PHANDLE(ARBATOS_SENSORS_NODE, bmi088_spi));
static const struct gpio_dt_spec Bmi088AccelCs = GPIO_DT_SPEC_GET(ARBATOS_SENSORS_NODE, bmi088_accel_cs_gpios);
static const struct gpio_dt_spec Bmi088GyroCs = GPIO_DT_SPEC_GET(ARBATOS_SENSORS_NODE, bmi088_gyro_cs_gpios);
static const struct spi_config Bmi088SpiCfg = {
    .frequency = DT_PROP_OR(ARBATOS_SENSORS_NODE, bmi088_spi_hz, 8000000),
    .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPOL | SPI_MODE_CPHA,
};
static volatile uint32_t Bmi088PortErrors;

#if DT_NODE_HAS_PROP(ARBATOS_SENSORS_NODE, bmi088_gyro_drdy_gpios)
static const struct gpio_dt_spec Bmi088Ready = GPIO_DT_SPEC_GET(ARBATOS_SENSORS_NODE, bmi088_gyro_drdy_gpios);
static struct gpio_callback Bmi088ReadyCallback;
static uint8_t Bmi088ReadyConfigured;
K_SEM_DEFINE(Bmi088ReadySem, 0, 1);

static void Bmi088OnReady(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins)
{
    ARG_UNUSED(port);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);
    k_sem_give(&Bmi088ReadySem);
}
#endif

int Bmi088PortWaitSample(uint32_t timeoutMs)
{
#if DT_NODE_HAS_PROP(ARBATOS_SENSORS_NODE, bmi088_gyro_drdy_gpios)
    if (!Bmi088ReadyConfigured) return -ENODEV;
    return k_sem_take(&Bmi088ReadySem, K_MSEC(timeoutMs));
#else
    ARG_UNUSED(timeoutMs);
    return -ENOTSUP;
#endif
}

uint32_t Bmi088PortErrorCount(void) { return Bmi088PortErrors; }

void BMI088_GPIO_init(void)
{
    if (!device_is_ready(Bmi088Spi) || !gpio_is_ready_dt(&Bmi088AccelCs) || !gpio_is_ready_dt(&Bmi088GyroCs)) {
        return;
    }
    (void)gpio_pin_configure_dt(&Bmi088AccelCs, GPIO_OUTPUT_INACTIVE);
    (void)gpio_pin_configure_dt(&Bmi088GyroCs, GPIO_OUTPUT_INACTIVE);
#if DT_NODE_HAS_PROP(ARBATOS_SENSORS_NODE, bmi088_gyro_drdy_gpios)
    if (!Bmi088ReadyConfigured && gpio_is_ready_dt(&Bmi088Ready) &&
        gpio_pin_configure_dt(&Bmi088Ready, GPIO_INPUT) == 0) {
        gpio_init_callback(&Bmi088ReadyCallback, Bmi088OnReady, BIT(Bmi088Ready.pin));
        if (gpio_add_callback(Bmi088Ready.port, &Bmi088ReadyCallback) == 0) {
            if (gpio_pin_interrupt_configure_dt(&Bmi088Ready, GPIO_INT_EDGE_TO_ACTIVE) == 0) {
                Bmi088ReadyConfigured = 1u;
            } else {
                (void)gpio_remove_callback(Bmi088Ready.port, &Bmi088ReadyCallback);
            }
        }
    }
#endif
}

void BMI088_com_init(void) { BMI088_GPIO_init(); }
void BMI088_delay_ms(uint16_t ms) { k_msleep(ms); }
void BMI088_delay_us(uint16_t us) { k_busy_wait(us); }
void BMI088_ACCEL_NS_L(void) { (void)gpio_pin_set_dt(&Bmi088AccelCs, 1); }
void BMI088_ACCEL_NS_H(void) { (void)gpio_pin_set_dt(&Bmi088AccelCs, 0); }
void BMI088_GYRO_NS_L(void) { (void)gpio_pin_set_dt(&Bmi088GyroCs, 1); }
void BMI088_GYRO_NS_H(void) { (void)gpio_pin_set_dt(&Bmi088GyroCs, 0); }

uint8_t BMI088_read_write_byte(uint8_t txdata)
{
    uint8_t rxdata = 0;
    struct spi_buf tx = {.buf = &txdata, .len = 1};
    struct spi_buf rx = {.buf = &rxdata, .len = 1};
    struct spi_buf_set txSet = {.buffers = &tx, .count = 1};
    struct spi_buf_set rxSet = {.buffers = &rx, .count = 1};
    int ret = spi_transceive(Bmi088Spi, &Bmi088SpiCfg, &txSet, &rxSet);
    if (ret != 0) {
        Bmi088PortErrors++;
        return 0;
    }
    return rxdata;
}

void Bmi088PortReadBytes(uint8_t command, uint8_t *data, uint8_t len, uint8_t dummy)
{
    uint8_t txBytes[18] = {0}, rxBytes[18] = {0};
    if (data == NULL || len == 0u || len > 16u || dummy > 1u) {
        Bmi088PortErrors++;
        return;
    }
    /* 一次事务传完整寄存器块，避免逐字节进入Zephyr SPI驱动。片选仍由调用方保持。 */
    txBytes[0] = command;
    struct spi_buf tx = {.buf = txBytes, .len = len + 1u + dummy};
    struct spi_buf rx = {.buf = rxBytes, .len = len + 1u + dummy};
    const struct spi_buf_set txSet = {.buffers = &tx, .count = 1};
    const struct spi_buf_set rxSet = {.buffers = &rx, .count = 1};
    if (spi_transceive(Bmi088Spi, &Bmi088SpiCfg, &txSet, &rxSet) != 0) {
        Bmi088PortErrors++;
        memset(data, 0, len);
        return;
    }
    memcpy(data, rxBytes + 1u + dummy, len);
}
