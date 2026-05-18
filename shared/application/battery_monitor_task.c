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
#include "bsp_buzzer.h"
#include "user_lib.h"
#include "config.h"
#include "sdlog.h"

static const voltage_config_t *const voltage_cfg = &g_config.voltage;

#define BATTERY_MONITOR_PERIOD_MS             100u
#define BATTERY_LOW_EXIT_HYSTERESIS_V         0.5f
#define BATTERY_ALARM_BEEP_FREQ_HZ            2600u
#define BATTERY_ALARM_BEEP_VOLUME             180u
#define BATTERY_ALARM_BEEP_ON_MS              120u
#define BATTERY_ALARM_BEEP_OFF_MS             160u

static fp32 calc_battery_percentage(float voltage);
static void battery_low_alarm_update(void);


fp32 battery_voltage;
fp32 electricity_percentage;
static uint8_t battery_low_alarm;
static uint8_t battery_alarm_beep_on;
static uint16_t battery_alarm_beep_elapsed_ms;

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

        battery_low_alarm_update();

        osDelay(BATTERY_MONITOR_PERIOD_MS);
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

fp32 get_battery_voltage_cached(void)
{
    return battery_voltage;
}

fp32 get_battery_percentage_fp32(void)
{
    return electricity_percentage;
}

uint8_t battery_monitor_is_low_alarm(void)
{
    return battery_low_alarm;
}

static void battery_low_alarm_update(void)
{
    const fp32 low_voltage = voltage_cfg->low_battery_voltage;
    const uint8_t was_alarm = battery_low_alarm;

    if ((battery_voltage > 1.0f) && (battery_voltage <= low_voltage))
    {
        battery_low_alarm = 1u;
    }
    else if (battery_voltage > (low_voltage + BATTERY_LOW_EXIT_HYSTERESIS_V))
    {
        battery_low_alarm = 0u;
    }

    if (battery_low_alarm == 0u)
    {
        if (battery_alarm_beep_on != 0u)
        {
            buzzer_tone_stop();
        }
        battery_alarm_beep_on = 0u;
        battery_alarm_beep_elapsed_ms = 0u;
        return;
    }

    if (was_alarm == 0u)
    {
        battery_alarm_beep_on = 1u;
        battery_alarm_beep_elapsed_ms = 0u;
        (void)buzzer_tone_start_hz(BATTERY_ALARM_BEEP_FREQ_HZ, BATTERY_ALARM_BEEP_VOLUME);
        return;
    }

    battery_alarm_beep_elapsed_ms = (uint16_t)(battery_alarm_beep_elapsed_ms + BATTERY_MONITOR_PERIOD_MS);
    if (battery_alarm_beep_on != 0u)
    {
        if (battery_alarm_beep_elapsed_ms >= BATTERY_ALARM_BEEP_ON_MS)
        {
            buzzer_tone_stop();
            battery_alarm_beep_on = 0u;
            battery_alarm_beep_elapsed_ms = 0u;
        }
    }
    else if (battery_alarm_beep_elapsed_ms >= BATTERY_ALARM_BEEP_OFF_MS)
    {
        (void)buzzer_tone_start_hz(BATTERY_ALARM_BEEP_FREQ_HZ, BATTERY_ALARM_BEEP_VOLUME);
        battery_alarm_beep_on = 1u;
        battery_alarm_beep_elapsed_ms = 0u;
    }
}
