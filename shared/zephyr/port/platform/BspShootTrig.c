/* SPDX-License-Identifier: Apache-2.0 */
#include "Types.h"
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#define P DT_PATH(arbatos_platform)
bool_t BspShootTrigReadRaw(uint8_t *out_level)
{
    if (out_level == NULL) return 0;
    *out_level = 1u;
#if DT_NODE_EXISTS(P) && DT_NODE_HAS_PROP(P, shoot_trig_gpios)
    static const struct gpio_dt_spec pin = GPIO_DT_SPEC_GET(P, shoot_trig_gpios);
    static uint8_t configured;
    if (!gpio_is_ready_dt(&pin)) return 0;
    if (!configured) {
        if (gpio_pin_configure_dt(&pin, GPIO_INPUT) != 0) return 0;
        configured = 1u;
    }
    /* 旧 C 板为上拉输入，按下读0；保持原始电平，由射击配置解释极性。 */
    int level = gpio_pin_get_raw(pin.port, pin.pin);
    if (level < 0) return 0;
    *out_level = level != 0 ? 1u : 0u;
    return 1;
#else
    return 0;
#endif
}
