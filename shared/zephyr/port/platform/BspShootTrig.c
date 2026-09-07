/* SPDX-License-Identifier: Apache-2.0 */
#include "Types.h"
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#define P DT_PATH(arbatos_platform)
bool_t BspShootTrigReadRaw(uint8_t *out_level)
{
#if DT_NODE_EXISTS(P) && DT_NODE_HAS_PROP(P, shoot_trig_gpios)
    static const struct gpio_dt_spec pin = GPIO_DT_SPEC_GET(P, shoot_trig_gpios);
    if (out_level == 0 || !gpio_is_ready_dt(&pin)) return 0;
    *out_level = (uint8_t)gpio_pin_get_dt(&pin);
    return 1;
#else
    (void)out_level; return 0;
#endif
}
