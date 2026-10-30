/* SPDX-License-Identifier: Apache-2.0 */
#include "Types.h"
#include <math.h>
#if defined(CONFIG_BOARD_DJI_C_F407)
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <stm32_ll_adc.h>

static const struct device *const BatteryAdc = DEVICE_DT_GET(DT_NODELABEL(adc3));
static const struct device *const InternalAdc = DEVICE_DT_GET(DT_NODELABEL(adc1));
static const struct gpio_dt_spec HardwareVersionPins[] = {
    GPIO_DT_SPEC_GET_BY_IDX(DT_PATH(arbatos_platform), hardware_version_gpios, 0),
    GPIO_DT_SPEC_GET_BY_IDX(DT_PATH(arbatos_platform), hardware_version_gpios, 1),
    GPIO_DT_SPEC_GET_BY_IDX(DT_PATH(arbatos_platform), hardware_version_gpios, 2),
};
static uint8_t HardwareVersionReady;
K_MUTEX_DEFINE(AdcLock);
static uint8_t Started;
static uint32_t Ok, Fail;
static uint16_t BatteryRaw;

static int ReadChannel(const struct device *dev, uint8_t channel, uint16_t *raw)
{
    struct adc_sequence sequence = {
        .channels = BIT(channel), .buffer = raw, .buffer_size = sizeof(*raw), .resolution = 12,
    };
    return adc_read(dev, &sequence);
}

uint8_t BspAdcStart(void)
{
    int result = 0;
    if (k_is_in_isr()) return 0u;
    k_mutex_lock(&AdcLock, K_FOREVER);
    if (Started) { k_mutex_unlock(&AdcLock); return 1u; }
    if (!device_is_ready(BatteryAdc) || !device_is_ready(InternalAdc)) result = -1;
    for (uint8_t i = 0; result == 0 && i < 3u; i++)
    {
        struct adc_channel_cfg cfg = {
            .gain = ADC_GAIN_1, .reference = ADC_REF_INTERNAL,
            .acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_TICKS, 480),
            .channel_id = i == 0u ? 8u : (i == 1u ? 16u : 17u),
        };
        result = adc_channel_setup(i == 0u ? BatteryAdc : InternalAdc, &cfg);
    }
    if (result == 0)
    {
        ADC_TypeDef *adc = (ADC_TypeDef *)DT_REG_ADDR(DT_NODELABEL(adc1));
        LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(adc),
                                       LL_ADC_PATH_INTERNAL_VREFINT | LL_ADC_PATH_INTERNAL_TEMPSENSOR);
        k_usleep(LL_ADC_DELAY_TEMPSENSOR_STAB_US);
        result = ReadChannel(BatteryAdc, 8u, &BatteryRaw);
    }
    Started = result == 0;
    if (Started) Ok++; else Fail++;
    k_mutex_unlock(&AdcLock);
    return Started;
}
void init_vrefint_reciprocal(void) { (void)BspAdcStart(); }
uint8_t BspAdcIsStarted(void) { return Started; }
uint16_t BspAdcGetRaw(uint8_t index)
{
    uint16_t raw = 0u;
    if (index != 0u || !BspAdcStart()) return 0u;
    k_mutex_lock(&AdcLock, K_FOREVER);
    if (ReadChannel(BatteryAdc, 8u, &raw) != 0) { Fail++; raw = 0u; }
    BatteryRaw = raw;
    k_mutex_unlock(&AdcLock);
    return raw;
}
static fp32 AdcVoltage(uint8_t channel)
{
    uint16_t raw = 0u, reference = 0u;
    int result;
    if (!BspAdcStart()) return NAN;
    k_mutex_lock(&AdcLock, K_FOREVER);
    result = ReadChannel(InternalAdc, 17u, &reference);
    if (result == 0) result = ReadChannel(channel == 8u ? BatteryAdc : InternalAdc, channel, &raw);
    if (result != 0 || reference == 0u) Fail++;
    k_mutex_unlock(&AdcLock);
    return result == 0 && reference != 0u ? (fp32)raw * 1.2f / (fp32)reference : NAN;
}
fp32 BspAdcGetChannelVoltage(uint8_t index) { return index == 0u ? AdcVoltage(8u) : NAN; }
uint32_t BspAdcGetStartOkCount(void) { return Ok; }
uint32_t BspAdcGetStartFailCount(void) { return Fail; }
fp32 get_temprate(void) { return (AdcVoltage(16u) - 0.76f) * 400.0f + 25.0f; }
fp32 get_battery_voltage(void) { return BspAdcGetChannelVoltage(0u) * 10.0909090909f; }
static int BspHardwareVersionInit(void)
{
    for (unsigned int i = 0; i < ARRAY_SIZE(HardwareVersionPins); i++)
    {
        if (!gpio_is_ready_dt(&HardwareVersionPins[i]) ||
            gpio_pin_configure_dt(&HardwareVersionPins[i], GPIO_INPUT) != 0) return -1;
    }
    HardwareVersionReady = 1u;
    return 0;
}
SYS_INIT(BspHardwareVersionInit, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);

uint8_t get_hardware_version(void)
{
    uint8_t version = 0u;
    if (!HardwareVersionReady) return 0xffu;
    for (unsigned int i = 0; i < ARRAY_SIZE(HardwareVersionPins); i++)
    {
        int value = gpio_pin_get_dt(&HardwareVersionPins[i]);
        if (value < 0) return 0xffu;
        version |= (uint8_t)((uint8_t)(value != 0) << i);
    }
    return version;
}
#else
/* ADC conversion is intentionally unavailable until board DTS supplies
 * channel, gain, reference and divider calibration.  Do not return a
 * plausible voltage/temperature. */
static uint32_t Fail;
void init_vrefint_reciprocal(void) {}
uint8_t BspAdcStart(void) { Fail++; return 0; }
uint8_t BspAdcIsStarted(void) { return 0; }
uint16_t BspAdcGetRaw(uint8_t index) { (void)index; return 0; }
fp32 BspAdcGetChannelVoltage(uint8_t index) { (void)index; return NAN; }
uint32_t BspAdcGetStartOkCount(void) { return 0; }
uint32_t BspAdcGetStartFailCount(void) { return Fail; }
fp32 get_temprate(void) { return NAN; }
fp32 get_battery_voltage(void) { return NAN; }
uint8_t get_hardware_version(void) { return 0xffu; }
#endif
