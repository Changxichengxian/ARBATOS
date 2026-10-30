/* SPDX-License-Identifier: Apache-2.0 */
#include <stdint.h>
#include <errno.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>

#define P DT_PATH(arbatos_platform)
#if DT_NODE_EXISTS(P) && DT_NODE_HAS_PROP(P, key_gpios)
static const struct gpio_dt_spec Key = GPIO_DT_SPEC_GET(P, key_gpios);
static struct gpio_callback KeyCallback;
static volatile uint32_t Count, Last;
static uint32_t LastIrq;
static uint8_t HasIrq;
void BspKeyExti0Callback(void);

static void BspKeyGpioCallback(const struct device *dev, struct gpio_callback *callback,
                               uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(callback);
    ARG_UNUSED(pins);
    BspKeyExti0Callback();
}
/* 按设备树设置输入和上下拉，避免只读取复位后的悬空引脚。 */
static int BspKeyPortInit(void)
{
    int result;
    if (!gpio_is_ready_dt(&Key)) return -ENODEV;
    result = gpio_pin_configure_dt(&Key, GPIO_INPUT);
    if (result != 0) return result;
    gpio_init_callback(&KeyCallback, BspKeyGpioCallback, BIT(Key.pin));
    result = gpio_add_callback(Key.port, &KeyCallback);
    if (result != 0) return result;
    result = gpio_pin_interrupt_configure_dt(&Key, GPIO_INT_EDGE_TO_ACTIVE);
    if (result != 0) (void)gpio_remove_callback(Key.port, &KeyCallback);
    return result;
}
SYS_INIT(BspKeyPortInit, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
uint8_t BspKeyReadRawDown(void) { return gpio_is_ready_dt(&Key) && gpio_pin_get_dt(&Key) > 0 ? 1u : 0u; }
uint32_t BspKeyGetPressCnt(void) { return Count; }
uint32_t BspKeyGetLastPressTickMs(void) { return Last; }
void BspKeyExti0Callback(void)
{
    uint32_t now = k_uptime_get_32();
    if (HasIrq && (uint32_t)(now - LastIrq) < 30u) return;
    LastIrq = now;
    HasIrq = 1u;
    if (BspKeyReadRawDown()) { Count++; Last = now; }
}
#else
uint8_t BspKeyReadRawDown(void) { return 0u; }
uint32_t BspKeyGetPressCnt(void) { return 0u; }
uint32_t BspKeyGetLastPressTickMs(void) { return 0u; }
void BspKeyExti0Callback(void) {}
#endif
