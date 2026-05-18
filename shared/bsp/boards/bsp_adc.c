/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "bsp_adc.h"
#include "adc.h"
#include "main.h"

volatile fp32 voltage_vrefint_proportion = 8.0586080586080586080586080586081e-4f;

#if defined(STM32H723xx) || defined(STM32H7) || defined(STM32H7xx)

#define BSP_ADC_MC02_CHANNEL_COUNT 2u
#define BSP_ADC_MC02_VREF_V        3.3f
#define BSP_ADC_MC02_FULL_SCALE    65535.0f
#define BSP_ADC_MC02_BAT_INDEX     0u
#define BSP_ADC_MC02_BAT_DIVIDER   11.0f

static volatile uint16_t adc1_dma_buf[BSP_ADC_MC02_CHANNEL_COUNT];
static volatile uint8_t adc1_started;
static volatile uint8_t adc1_calibrated;
static volatile uint32_t adc1_start_ok_count;
static volatile uint32_t adc1_start_fail_count;

uint8_t bsp_adc_start(void)
{
    if (adc1_started != 0u)
    {
        return 1u;
    }

    if (adc1_calibrated == 0u)
    {
        if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK)
        {
            adc1_start_fail_count++;
            return 0u;
        }
        adc1_calibrated = 1u;
    }

    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc1_dma_buf, BSP_ADC_MC02_CHANNEL_COUNT) != HAL_OK)
    {
        adc1_start_fail_count++;
        return 0u;
    }

    adc1_started = 1u;
    adc1_start_ok_count++;
    return 1u;
}

uint8_t bsp_adc_is_started(void)
{
    return adc1_started;
}

uint16_t bsp_adc_get_raw(uint8_t index)
{
    if (index >= BSP_ADC_MC02_CHANNEL_COUNT)
    {
        return 0u;
    }

    (void)bsp_adc_start();
    return adc1_dma_buf[index];
}

fp32 bsp_adc_get_channel_voltage(uint8_t index)
{
    return (fp32)bsp_adc_get_raw(index) * BSP_ADC_MC02_VREF_V / BSP_ADC_MC02_FULL_SCALE;
}

uint32_t bsp_adc_get_start_ok_count(void)
{
    return adc1_start_ok_count;
}

uint32_t bsp_adc_get_start_fail_count(void)
{
    return adc1_start_fail_count;
}

void init_vrefint_reciprocal(void)
{
    voltage_vrefint_proportion = BSP_ADC_MC02_VREF_V / BSP_ADC_MC02_FULL_SCALE;
    (void)bsp_adc_start();
}

fp32 get_temprate(void)
{
    return 0.0f;
}

fp32 get_battery_voltage(void)
{
    return bsp_adc_get_channel_voltage(BSP_ADC_MC02_BAT_INDEX) * BSP_ADC_MC02_BAT_DIVIDER;
}

#else

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc3;

static uint16_t adcx_get_chx_value(ADC_HandleTypeDef *ADCx, uint32_t ch)
{
    static ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = ch;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;//ADC_SAMPLETIME_3CYCLES;

    if (HAL_ADC_ConfigChannel(ADCx, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_ADC_Start(ADCx);

    HAL_ADC_PollForConversion(ADCx, 10);
    return (uint16_t)HAL_ADC_GetValue(ADCx);

}
void init_vrefint_reciprocal(void)
{
    uint8_t i = 0;
    uint32_t total_adc = 0;
    for(i = 0; i < 200; i++)
    {
        total_adc += adcx_get_chx_value(&hadc1, ADC_CHANNEL_VREFINT);
    }

    voltage_vrefint_proportion = 200 * 1.2f / total_adc;

}
fp32 get_temprate(void)
{
    uint16_t adcx = 0;
    fp32 temperate;

    adcx = adcx_get_chx_value(&hadc1, ADC_CHANNEL_TEMPSENSOR);
    temperate = (fp32)adcx * voltage_vrefint_proportion;
    temperate = (temperate - 0.76f) * 400.0f + 25.0f;

    return temperate;
}


fp32 get_battery_voltage(void)
{
    fp32 voltage;
    uint16_t adcx = 0;

    adcx = adcx_get_chx_value(&hadc3, ADC_CHANNEL_8);
    voltage =  (fp32)adcx * voltage_vrefint_proportion * 10.090909090909090909090909090909f;

    return voltage;
}

uint8_t bsp_adc_start(void)
{
    return 1u;
}

uint8_t bsp_adc_is_started(void)
{
    return 1u;
}

uint16_t bsp_adc_get_raw(uint8_t index)
{
    if (index == 0u)
    {
        return adcx_get_chx_value(&hadc3, ADC_CHANNEL_8);
    }
    return 0u;
}

fp32 bsp_adc_get_channel_voltage(uint8_t index)
{
    if (index == 0u)
    {
        return (fp32)bsp_adc_get_raw(index) * voltage_vrefint_proportion;
    }
    return 0.0f;
}

uint32_t bsp_adc_get_start_ok_count(void)
{
    return 0u;
}

uint32_t bsp_adc_get_start_fail_count(void)
{
    return 0u;
}

#endif

uint8_t get_hardware_version(void)
{
#if defined(HW0_GPIO_Port) && defined(HW0_Pin) && defined(HW1_GPIO_Port) && defined(HW1_Pin) && defined(HW2_GPIO_Port) && defined(HW2_Pin)
    uint8_t hardware_version;
    hardware_version = HAL_GPIO_ReadPin(HW0_GPIO_Port, HW0_Pin)
                                | (HAL_GPIO_ReadPin(HW1_GPIO_Port, HW1_Pin)<<1)
                                | (HAL_GPIO_ReadPin(HW2_GPIO_Port, HW2_Pin)<<2);



    return hardware_version;
#else
    return 0u;
#endif
}
