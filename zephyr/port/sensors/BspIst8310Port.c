/* SPDX-License-Identifier: Apache-2.0 */
#include "BspIst8310Port.h"
#include "SensorsDt.h"

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>

#if !DT_NODE_HAS_PROP(ARBATOS_SENSORS_NODE, ist8310_i2c) || !DT_NODE_HAS_PROP(ARBATOS_SENSORS_NODE, ist8310_reset_gpios)
#error "IST8310 需要 ist8310-i2c 和 ist8310-reset-gpios。"
#endif

static const struct device *const Ist8310I2c = DEVICE_DT_GET(DT_PHANDLE(ARBATOS_SENSORS_NODE, ist8310_i2c));
static const struct gpio_dt_spec Ist8310Reset = GPIO_DT_SPEC_GET(ARBATOS_SENSORS_NODE, ist8310_reset_gpios);

void ist8310_GPIO_init(void)
{
    if (gpio_is_ready_dt(&Ist8310Reset)) {
        (void)gpio_pin_configure_dt(&Ist8310Reset, GPIO_OUTPUT_INACTIVE);
    }
}
void ist8310_com_init(void) { ist8310_GPIO_init(); }
uint8_t ist8310_IIC_read_single_reg(uint8_t reg)
{
    uint8_t data = 0;
    (void)i2c_burst_read(Ist8310I2c, IST8310_IIC_ADDRESS >> 1, reg, &data, 1);
    return data;
}
void ist8310_IIC_write_single_reg(uint8_t reg, uint8_t data) { (void)i2c_burst_write(Ist8310I2c, IST8310_IIC_ADDRESS >> 1, reg, &data, 1); }
void ist8310_IIC_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len) { if (buf != NULL) (void)i2c_burst_read(Ist8310I2c, IST8310_IIC_ADDRESS >> 1, reg, buf, len); }
void ist8310_IIC_write_muli_reg(uint8_t reg, uint8_t *data, uint8_t len) { if (data != NULL) (void)i2c_burst_write(Ist8310I2c, IST8310_IIC_ADDRESS >> 1, reg, data, len); }
void ist8310_delay_ms(uint16_t ms) { k_msleep(ms); }
void ist8310_delay_us(uint16_t us) { k_busy_wait(us); }
void ist8310_RST_H(void) { (void)gpio_pin_set_dt(&Ist8310Reset, 0); }
void ist8310_RST_L(void) { (void)gpio_pin_set_dt(&Ist8310Reset, 1); }
