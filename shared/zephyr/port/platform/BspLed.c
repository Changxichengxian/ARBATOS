/* SPDX-License-Identifier: Apache-2.0 */
#include "BspLed.h"

#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>
#include "ArbatosDt.h"

LOG_MODULE_REGISTER(arbatos_led, LOG_LEVEL_INF);

#define ARBATOS_PLATFORM_NODE DT_PATH(arbatos_platform)

/*
 * 只有板级描述明确给出 led0-gpios 后才驱动状态灯。
 * A 板按原理图使用 PF14 绿色指示灯，低电平点亮；其他板分别声明自己的接线。
 */
#if DT_NODE_EXISTS(ARBATOS_PLATFORM_NODE) && DT_NODE_HAS_PROP(ARBATOS_PLATFORM_NODE, rgb_pwms)
static const struct pwm_dt_spec Rgb[] = {
    ARBATOS_PWM_DT_SPEC_GET_BY_IDX(ARBATOS_PLATFORM_NODE, rgb_pwms, 0),
    ARBATOS_PWM_DT_SPEC_GET_BY_IDX(ARBATOS_PLATFORM_NODE, rgb_pwms, 1),
    ARBATOS_PWM_DT_SPEC_GET_BY_IDX(ARBATOS_PLATFORM_NODE, rgb_pwms, 2),
};
static uint8_t StatusLedReady;
void BspLedSet(uint8_t on);
void BspLedInit(void)
{
    StatusLedReady = pwm_is_ready_dt(&Rgb[0]) && pwm_is_ready_dt(&Rgb[1]) && pwm_is_ready_dt(&Rgb[2]);
    if (StatusLedReady) BspLedSet(0u);
}
void BspLedSet(uint8_t on)
{
    if (!StatusLedReady) return;
    for (unsigned int i = 0; i < 3u; i++)
        (void)pwm_set_dt(&Rgb[i], Rgb[i].period, on ? Rgb[i].period : 0u);
}
#elif DT_NODE_EXISTS(ARBATOS_PLATFORM_NODE) && \
    DT_NODE_HAS_PROP(ARBATOS_PLATFORM_NODE, led0_gpios)
static const struct gpio_dt_spec StatusLed =
    GPIO_DT_SPEC_GET(ARBATOS_PLATFORM_NODE, led0_gpios);

static uint8_t StatusLedReady;

void BspLedInit(void)
{
    int result;

    if (!gpio_is_ready_dt(&StatusLed))
    {
        LOG_WRN("status LED configured but GPIO device is unavailable");
        return;
    }

    result = gpio_pin_configure_dt(&StatusLed, GPIO_OUTPUT_INACTIVE);
    if (result != 0)
    {
        LOG_WRN("status LED GPIO configuration failed: %d", result);
        return;
    }

    StatusLedReady = 1u;
}

void BspLedSet(uint8_t on)
{
    if (StatusLedReady != 0u)
    {
        (void)gpio_pin_set_dt(&StatusLed, on != 0u);
    }
}
#else
void BspLedInit(void)
{
    LOG_WRN("status LED disabled: board has no verified led0-gpios mapping");
}

void BspLedSet(uint8_t on)
{
    (void)on;
}
#endif

void aRGB_led_show(uint32_t aRGB)
{
#if DT_NODE_EXISTS(ARBATOS_PLATFORM_NODE) && DT_NODE_HAS_PROP(ARBATOS_PLATFORM_NODE, rgb_pwms)
    if (!StatusLedReady) return;
    const uint32_t alpha = aRGB >> 24;
    for (unsigned int i = 0; i < 3u; i++)
    {
        const uint32_t value = (aRGB >> (i * 8u)) & 0xffu;
        (void)pwm_set_dt(&Rgb[i], Rgb[i].period,
                         (uint32_t)(((uint64_t)Rgb[i].period * value * alpha) / 65025u));
    }
#else
    const uint8_t alpha = (uint8_t)(aRGB >> 24);
    const uint32_t rgb = aRGB & 0x00FFFFFFu;

    /*
     * 单色状态灯无法保留 RGB 色相。保持旧接口的 alpha 关灯语义，任何非黑的
     * 可见颜色都点亮状态灯；RGB PWM 接线确认后再由板级描述扩展为三通道输出。
     */
    BspLedSet((uint8_t)((alpha != 0u) && (rgb != 0u)));
#endif
}
