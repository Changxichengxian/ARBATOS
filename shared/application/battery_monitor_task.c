/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */


#include "battery_monitor_task.h"
#include "cmsis_os.h"

#include "bsp_adc.h"
#include "user_lib.h"
#include "config.h"
#include "sdlog.h"

static const voltage_config_t *const voltage_cfg = &g_config.voltage;


static fp32 calc_battery_percentage(float voltage);


fp32 battery_voltage;
fp32 electricity_percentage;

/**
  * @brief          power ADC and calculate electricity percentage
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          电源采样和计算电源百分比
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void battery_monitor_task(void const * argument)
{
    osDelay(1000);
    //use inner 1.2v to calbrate
    init_vrefint_reciprocal();
    while(1)
    {
        battery_voltage = get_battery_voltage() + voltage_cfg->voltage_drop;
        electricity_percentage = calc_battery_percentage(battery_voltage);

        sdlog_battery_t pkt = {0};
        pkt.voltage = battery_voltage;
        pkt.percent = electricity_percentage;
        sdlog_write(SDLOG_TAG_BATTERY, &pkt, (uint16_t)sizeof(pkt));

        osDelay(100);
    }
}

static fp32 calc_battery_percentage(float voltage)
{
    fp32 percentage;
    fp32 voltage_2 = voltage * voltage;
    fp32 voltage_3 = voltage_2 * voltage;
    
    if(voltage < 19.5f)
    {
        percentage = 0.0f;
    }
    else if(voltage < 21.9f)
    {
        percentage = 0.005664f * voltage_3 - 0.3386f * voltage_2 + 6.765f * voltage - 45.17f;
    }
    else if(voltage < 25.5f)
    {
        percentage = 0.02269f * voltage_3 - 1.654f * voltage_2 + 40.34f * voltage - 328.4f;
    }
    else
    {
        percentage = 1.0f;
    }
    if(percentage < 0.0f)
    {
        percentage = 0.0f;
    }
    else if(percentage > 1.0f)
    {
        percentage = 1.0f;
    }
    //another formulas
    //另一套公式
//    if(voltage < 19.5f)
//    {
//        percentage = 0.0f;
//    }
//    else if(voltage < 22.5f)
//    {
////        percentage = 0.05776f * (voltage - 22.5f) * (voltage_2 - 39.0f * voltage + 383.4f) + 0.5f;
//        percentage = 0.05021f * voltage_3 - 3.075f * voltage_2 + 62.77f * voltage - 427.02953125f;
//    }
//    else if(voltage < 25.5f)
//    {
////        percentage = 0.01822f * (voltage - 22.5f) * (voltage_2 - 52.05f * voltage + 637.0f) + 0.5f;
//        percentage = 0.0178f * voltage_3 - 1.292f * voltage_2 + 31.41f * voltage - 254.903125f;
//    }
//    else
//    {
//        percentage = 1.0f;
//    }

    return percentage;
}

uint16_t get_battery_percentage(void)
{
    return (uint16_t)(electricity_percentage * 100.0f);
}
