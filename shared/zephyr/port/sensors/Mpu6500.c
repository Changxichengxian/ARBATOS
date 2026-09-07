/* SPDX-License-Identifier: Apache-2.0 */
#include "Mpu6500.h"
#include "SensorsDt.h"

#include <errno.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>

#if !DT_NODE_HAS_PROP(ARBATOS_SENSORS_NODE, mpu6500_spi) || !DT_NODE_HAS_PROP(ARBATOS_SENSORS_NODE, mpu6500_cs_gpios)
#error "MPU6500 需要 mpu6500-spi 和 mpu6500-cs-gpios。"
#endif

#define MPU6500_WHO_AM_I 0x75
#define MPU6500_ACCEL_XOUT_H 0x3B
static const struct device *const Mpu6500Spi = DEVICE_DT_GET(DT_PHANDLE(ARBATOS_SENSORS_NODE, mpu6500_spi));
static const struct gpio_dt_spec Mpu6500Cs = GPIO_DT_SPEC_GET(ARBATOS_SENSORS_NODE, mpu6500_cs_gpios);
static const struct spi_config Mpu6500SpiCfg = {.frequency = DT_PROP_OR(ARBATOS_SENSORS_NODE, mpu6500_spi_hz, 8000000), .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB};
static uint8_t Mpu6500DmaRx[15];
static uint8_t Mpu6500DmaReady;

static int Mpu6500Transfer(uint8_t reg, uint8_t *rx, size_t len)
{
    uint8_t tx[15] = {0};
    uint8_t scratch[15] = {0};
    struct spi_buf txBuf = {.buf = tx, .len = len + 1};
    struct spi_buf rxBuf = {.buf = scratch, .len = len + 1};
    struct spi_buf_set txSet = {.buffers = &txBuf, .count = 1};
    struct spi_buf_set rxSet = {.buffers = &rxBuf, .count = 1};
    int ret;
    if (len > 14 || rx == NULL || !device_is_ready(Mpu6500Spi) || !gpio_is_ready_dt(&Mpu6500Cs)) return -ENODEV;
    tx[0] = reg | 0x80U;
    (void)gpio_pin_set_dt(&Mpu6500Cs, 1);
    ret = spi_transceive(Mpu6500Spi, &Mpu6500SpiCfg, &txSet, &rxSet);
    (void)gpio_pin_set_dt(&Mpu6500Cs, 0);
    if (ret == 0) memcpy(rx, &scratch[1], len);
    return ret;
}
static int Mpu6500Write(uint8_t reg, uint8_t data)
{
    uint8_t tx[2] = {reg & 0x7fU, data};
    struct spi_buf txBuf = {.buf = tx, .len = sizeof(tx)};
    struct spi_buf_set txSet = {.buffers = &txBuf, .count = 1};
    int ret;
    (void)gpio_pin_set_dt(&Mpu6500Cs, 1);
    ret = spi_write(Mpu6500Spi, &Mpu6500SpiCfg, &txSet);
    (void)gpio_pin_set_dt(&Mpu6500Cs, 0);
    return ret;
}
int mpu6500_init(void)
{
    static const uint8_t seq[][2] = {{0x6B,0x80},{0x6B,0x03},{0x6C,0x00},{0x1A,0x02},{0x1B,0x10},{0x1C,0x10},{0x1D,0x02},{0x6A,0x20},{0x37,0x10},{0x38,0x01}};
    uint8_t who;
    if (!device_is_ready(Mpu6500Spi) || !gpio_is_ready_dt(&Mpu6500Cs)) return -ENODEV;
    if (gpio_pin_configure_dt(&Mpu6500Cs, GPIO_OUTPUT_INACTIVE) != 0) return -EIO;
    k_msleep(100);
    if (Mpu6500Transfer(MPU6500_WHO_AM_I, &who, 1) != 0) return -EIO;
    for (size_t i = 0; i < ARRAY_SIZE(seq); ++i) { if (Mpu6500Write(seq[i][0], seq[i][1]) != 0) return -EIO; k_msleep(1); }
    return (who == 0x70 || who == 0x71) ? 0 : -ENODEV;
}
int mpu6500_read_raw(mpu6500_raw_t *out)
{
    uint8_t raw[14];
    if (out == NULL || Mpu6500Transfer(MPU6500_ACCEL_XOUT_H, raw, sizeof(raw)) != 0) return -EIO;
    for (int i = 0; i < 3; ++i) out->accel[i] = (int16_t)((raw[i * 2] << 8) | raw[i * 2 + 1]);
    out->temp = (int16_t)((raw[6] << 8) | raw[7]);
    for (int i = 0; i < 3; ++i) out->gyro[i] = (int16_t)((raw[8 + i * 2] << 8) | raw[9 + i * 2]);
    return 0;
}
int mpu6500_read_raw_dma_start(void) { Mpu6500DmaReady = Mpu6500Transfer(MPU6500_ACCEL_XOUT_H, &Mpu6500DmaRx[1], 14) == 0; return Mpu6500DmaReady ? 0 : -EIO; }
int mpu6500_read_raw_dma_finish(mpu6500_raw_t *out)
{
    const uint8_t *raw = &Mpu6500DmaRx[1];
    if (!Mpu6500DmaReady || out == NULL) return -EAGAIN;
    for (int i = 0; i < 3; ++i) out->accel[i] = (int16_t)((raw[i * 2] << 8) | raw[i * 2 + 1]);
    out->temp = (int16_t)((raw[6] << 8) | raw[7]);
    for (int i = 0; i < 3; ++i) out->gyro[i] = (int16_t)((raw[8 + i * 2] << 8) | raw[9 + i * 2]);
    Mpu6500DmaReady = 0;
    return 0;
}
void mpu6500_read_raw_dma_release_cs(void) { Mpu6500DmaReady = 0; }
float mpu6500_temp_c(int16_t temp_raw) { return ((float)temp_raw / 333.87f) + 21.0f; }
