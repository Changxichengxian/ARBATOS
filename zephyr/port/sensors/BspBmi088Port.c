/* SPDX-License-Identifier: Apache-2.0 */
#include "BspBmi088Port.h"
#include "SensorsDt.h"

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

void BMI088_GPIO_init(void)
{
    if (!device_is_ready(Bmi088Spi) || !gpio_is_ready_dt(&Bmi088AccelCs) || !gpio_is_ready_dt(&Bmi088GyroCs)) {
        return;
    }
    (void)gpio_pin_configure_dt(&Bmi088AccelCs, GPIO_OUTPUT_INACTIVE);
    (void)gpio_pin_configure_dt(&Bmi088GyroCs, GPIO_OUTPUT_INACTIVE);
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
    return spi_transceive(Bmi088Spi, &Bmi088SpiCfg, &txSet, &rxSet) == 0 ? rxdata : 0;
}
