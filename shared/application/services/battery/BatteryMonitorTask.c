/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */


#include "BatteryMonitorTask.h"
#include "cmsis_os.h"

#include "BspAdc.h"
#include "BspBuzzer.h"
#include "UserLib.h"
#include "RobotConfig.h"
#include "SdLog.h"

static const voltage_config_t *const voltage_cfg = &g_config.voltage;

#define BATTERY_MONITOR_PERIOD_MS             100u
#define BATTERY_LOW_EXIT_HYSTERESIS_V         0.5f
#define BATTERY_ALARM_BEEP_FREQ_HZ            2600u
#define BATTERY_ALARM_BEEP_VOLUME             180u
#define BATTERY_ALARM_BEEP_ON_MS              120u
#define BATTERY_ALARM_BEEP_OFF_MS             160u

static fp32 calc_battery_percentage(float voltage);
static void BatteryLowAlarmUpdate(void);


fp32 BatteryVoltage;
fp32 electricity_percentage;
static uint8_t BatteryLowAlarm;
static uint8_t BatteryAlarmBeepOn;
static uint16_t BatteryAlarmBeepElapsedMs;

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
void BatteryMonitorTask(void const * argument)
{
    osDelay(1000);
    //use inner 1.2v to calbrate
    init_vrefint_reciprocal();
    while(1)
    {
        BatteryVoltage = get_battery_voltage() + voltage_cfg->voltage_drop;
        electricity_percentage = calc_battery_percentage(BatteryVoltage);

        sdlog_battery_t pkt = {0};
        pkt.voltage = BatteryVoltage;
        pkt.percent = electricity_percentage;
        SdLogWrite(SDLOG_TAG_BATTERY, &pkt, (uint16_t)sizeof(pkt));

        BatteryLowAlarmUpdate();

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
    return BatteryVoltage;
}

fp32 get_battery_percentage_fp32(void)
{
    return electricity_percentage;
}

uint8_t BatteryMonitorIsLowAlarm(void)
{
    return BatteryLowAlarm;
}

static void BatteryLowAlarmUpdate(void)
{
    const fp32 low_voltage = voltage_cfg->low_battery_voltage;
    const uint8_t was_alarm = BatteryLowAlarm;

    if ((BatteryVoltage > 1.0f) && (BatteryVoltage <= low_voltage))
    {
        BatteryLowAlarm = 1u;
    }
    else if (BatteryVoltage > (low_voltage + BATTERY_LOW_EXIT_HYSTERESIS_V))
    {
        BatteryLowAlarm = 0u;
    }

    if (BatteryLowAlarm == 0u)
    {
        if (BatteryAlarmBeepOn != 0u)
        {
            BuzzerToneStop();
        }
        BatteryAlarmBeepOn = 0u;
        BatteryAlarmBeepElapsedMs = 0u;
        return;
    }

    if (was_alarm == 0u)
    {
        BatteryAlarmBeepOn = 1u;
        BatteryAlarmBeepElapsedMs = 0u;
        (void)BuzzerToneStartHz(BATTERY_ALARM_BEEP_FREQ_HZ, BATTERY_ALARM_BEEP_VOLUME);
        return;
    }

    BatteryAlarmBeepElapsedMs = (uint16_t)(BatteryAlarmBeepElapsedMs + BATTERY_MONITOR_PERIOD_MS);
    if (BatteryAlarmBeepOn != 0u)
    {
        if (BatteryAlarmBeepElapsedMs >= BATTERY_ALARM_BEEP_ON_MS)
        {
            BuzzerToneStop();
            BatteryAlarmBeepOn = 0u;
            BatteryAlarmBeepElapsedMs = 0u;
        }
    }
    else if (BatteryAlarmBeepElapsedMs >= BATTERY_ALARM_BEEP_OFF_MS)
    {
        (void)BuzzerToneStartHz(BATTERY_ALARM_BEEP_FREQ_HZ, BATTERY_ALARM_BEEP_VOLUME);
        BatteryAlarmBeepOn = 1u;
        BatteryAlarmBeepElapsedMs = 0u;
    }
}
