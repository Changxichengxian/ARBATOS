/* SPDX-License-Identifier: Apache-2.0 */
#include "BspAdc.h"
#include <errno.h>
#include <math.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>
#include <stm32_ll_adc.h>

static const struct device *const Adc = DEVICE_DT_GET(DT_NODELABEL(adc1));
static const uint8_t Channels[2] = {4, 19};
K_MUTEX_DEFINE(AdcLock);
static uint16_t Raw[2];
static uint8_t Started;
static uint32_t Ok, Fail;

/* H723 的片内温度和 VREFINT 只接到 ADC3。两路一起采样，按序为温度、VREFINT。 */
static const struct device *const TempAdc = DEVICE_DT_GET(DT_NODELABEL(adc3));
static const uint8_t TempChannels[2] = {17, 18};
K_MUTEX_DEFINE(TempAdcLock);
static uint16_t TempRaw[2];
static uint8_t TempStarted;
static int AdcRead(void)
{
    struct adc_sequence sequence = {
        .channels = BIT(4) | BIT(19), .buffer = Raw, .buffer_size = sizeof(Raw),
        .resolution = 16, .calibrate = Started == 0u,
    };
    return adc_read(Adc, &sequence);
}

static int TempAdcRead(void)
{
    struct adc_sequence sequence = {
        .channels = BIT(TempChannels[0]) | BIT(TempChannels[1]),
        .buffer = TempRaw,
        .buffer_size = sizeof(TempRaw),
        .resolution = 12,
        .calibrate = TempStarted == 0u,
    };
    return adc_read(TempAdc, &sequence);
}

static int TempAdcStartLocked(void)
{
    int ret = device_is_ready(TempAdc) ? 0 : -ENODEV;
    for (unsigned i = 0; ret == 0 && i < ARRAY_SIZE(TempChannels); i++) {
        struct adc_channel_cfg cfg = {
            .gain = ADC_GAIN_1,
            .reference = ADC_REF_INTERNAL,
            .acquisition_time = ADC_ACQ_TIME_MAX,
            .channel_id = TempChannels[i],
        };
        ret = adc_channel_setup(TempAdc, &cfg);
    }
    if (ret != 0) return ret;

    ADC_TypeDef *adc = (ADC_TypeDef *)DT_REG_ADDR(DT_NODELABEL(adc3));
    uint32_t path = LL_ADC_GetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(adc));
    LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(adc),
                                   path | LL_ADC_PATH_INTERNAL_TEMPSENSOR |
                                       LL_ADC_PATH_INTERNAL_VREFINT);
    k_usleep(LL_ADC_DELAY_TEMPSENSOR_STAB_US);
    ret = TempAdcRead();
    TempStarted = ret == 0;
    return ret;
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
fp32 get_temprate(void)
{
    if (k_is_in_isr()) return NAN;

    k_mutex_lock(&TempAdcLock, K_FOREVER);
    int ret = TempStarted != 0u ? TempAdcRead() : TempAdcStartLocked();
    const uint16_t temp_raw = TempRaw[0];
    const uint16_t vref_raw = TempRaw[1];
    const uint16_t ts_cal1 = *TEMPSENSOR_CAL1_ADDR;
    const uint16_t ts_cal2 = *TEMPSENSOR_CAL2_ADDR;
    const uint16_t vref_cal = *VREFINT_CAL_ADDR;
    fp32 temperature = NAN;
    if (ret == 0 && temp_raw != 0u && vref_raw != 0u &&
        ts_cal2 > ts_cal1 && vref_cal != 0u) {
        /* 工厂常量为 16 位、3.3 V 条件；ADC3 运行时为 12 位。
         * 先用同批 VREFINT 消去 VDDA，再与两点工厂校准值线性换算。 */
        const fp32 compensated = (fp32)temp_raw * (fp32)vref_cal / (fp32)vref_raw;
        temperature = (fp32)TEMPSENSOR_CAL1_TEMP +
                      (compensated - (fp32)ts_cal1) *
                          ((fp32)TEMPSENSOR_CAL2_TEMP - (fp32)TEMPSENSOR_CAL1_TEMP) /
                          ((fp32)ts_cal2 - (fp32)ts_cal1);
        if (!isfinite(temperature) || temperature < -40.0f || temperature > 130.0f) {
            temperature = NAN;
        }
    }
    k_mutex_unlock(&TempAdcLock);
    return temperature;
}
uint8_t get_hardware_version(void) { return 0xffu; }
