/* SPDX-License-Identifier: Apache-2.0 */
#include "BspAdc.h"
#include <errno.h>
#include <math.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>

static const struct device *const Adc = DEVICE_DT_GET(DT_NODELABEL(adc1));
static const uint8_t Channels[2] = {4, 19};
K_MUTEX_DEFINE(AdcLock);
static uint16_t Raw[2];
static uint8_t Started;
static uint32_t Ok, Fail;
static int AdcRead(void)
{
    struct adc_sequence sequence = {
        .channels = BIT(4) | BIT(19), .buffer = Raw, .buffer_size = sizeof(Raw),
        .resolution = 16, .calibrate = Started == 0u,
    };
    return adc_read(Adc, &sequence);
}

uint8_t BspAdcStart(void)
{
    k_mutex_lock(&AdcLock, K_FOREVER);
    int ret = device_is_ready(Adc) ? 0 : -ENODEV;
    for (unsigned i = 0; ret == 0 && i < ARRAY_SIZE(Channels); i++) {
        struct adc_channel_cfg cfg = {
            .gain = ADC_GAIN_1, .reference = ADC_REF_INTERNAL,
            .acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_TICKS, 65), .channel_id = Channels[i],
        };
        ret = adc_channel_setup(Adc, &cfg);
    }
    if (ret == 0) ret = AdcRead();
    Started = ret == 0;
    if (Started) Ok++; else Fail++;
    k_mutex_unlock(&AdcLock);
    return Started;
}

void init_vrefint_reciprocal(void) { (void)BspAdcStart(); }
uint8_t BspAdcIsStarted(void) { return Started; }
uint16_t BspAdcGetRaw(uint8_t index)
{
    k_mutex_lock(&AdcLock, K_FOREVER);
    uint16_t value = index < 2u ? Raw[index] : 0u;
    k_mutex_unlock(&AdcLock);
    return value;
}
fp32 BspAdcGetChannelVoltage(uint8_t index)
{
    if (index >= 2u || !Started || k_is_in_isr()) return NAN;
    k_mutex_lock(&AdcLock, K_FOREVER);
    int ret = AdcRead();
    /* 按原理图标称 3.3 V 换算；精密测量仍需万用表标定 VDDA。 */
    fp32 value = ret == 0 ? (fp32)Raw[index] * 3.3f / 65535.0f : NAN;
    if (ret != 0) Fail++;
    k_mutex_unlock(&AdcLock);
    return value;
}
uint32_t BspAdcGetStartOkCount(void) { return Ok; }
uint32_t BspAdcGetStartFailCount(void) { return Fail; }
fp32 get_battery_voltage(void) { return BspAdcGetChannelVoltage(0) * 11.0f; }
fp32 get_temprate(void) { return NAN; }
uint8_t get_hardware_version(void) { return 0xffu; }
