/* SPDX-License-Identifier: Apache-2.0 */
#include <stdint.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

#define P DT_PATH(arbatos_platform)
#if DT_NODE_EXISTS(P) && DT_NODE_HAS_PROP(P, key_gpios)
static const struct gpio_dt_spec Key = GPIO_DT_SPEC_GET(P, key_gpios);
static uint32_t Count, Last;
uint8_t BspKeyReadRawDown(void) { return gpio_is_ready_dt(&Key) ? (uint8_t)gpio_pin_get_dt(&Key) : 0u; }
uint32_t BspKeyGetPressCnt(void) { return Count; }
uint32_t BspKeyGetLastPressTickMs(void) { return Last; }
void BspKeyExti0Callback(void) { if (BspKeyReadRawDown()) { Count++; Last = k_uptime_get_32(); } }
#else
uint8_t BspKeyReadRawDown(void) { return 0u; }
uint32_t BspKeyGetPressCnt(void) { return 0u; }
uint32_t BspKeyGetLastPressTickMs(void) { return 0u; }
void BspKeyExti0Callback(void) {}
#endif
