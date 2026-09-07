/*
 * SPDX-FileCopyrightText: 2026 Chen Xuan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-06-25
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "SubBoardBringup.h"

#include <string.h>

#include "cmsis_os.h"
#include "main.h"
#include "SdLog.h"

#if defined(SUB_BOARD_FACTORY_TEST)
#include <stdio.h>

#include "BspBuzzer.h"
#include "BspSdSpiPort.h"
#include "fatfs/ff.h"
#include "SdSpi.h"
#include "tim.h"
#endif

#define SUB_BOARD_I2C_TIMEOUT_MS 100u
#define SUB_BOARD_PCF8563_ADDR   (0x51u << 1)

typedef struct
{
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint8_t weekday;
    uint8_t voltage_low;
} SubBoardDateTime;

static I2C_HandleTypeDef SubBoardI2c1;
static uint8_t SubBoardI2c1Inited = 0u;
static uint8_t SubBoardRtcProbed = 0u;

static void SubBoardI2c1Release(void);
static void SubBoardI2c1Init(void);
static void SubBoardI2cBusClear(void);
static void SubBoardRtcEnsureReady(void);
static uint8_t SubBoardFromBcd(uint8_t value);
static int SubBoardRtcRead(SubBoardDateTime *readback, uint32_t *hal_error);

#if defined(SUB_BOARD_FACTORY_TEST)
#define SUB_BOARD_FACTORY_IO_BYTES    (1024u * 1024u)
#define SUB_BOARD_FACTORY_CHUNK_BYTES 4096u
#define SUB_BOARD_FACTORY_DEBOUNCE_MS 20u

volatile SubBoardFactoryTestStatus g_sub_board_factory_test;
static uint8_t SubBoardFactoryIoBuf[SUB_BOARD_FACTORY_CHUNK_BYTES];
static FATFS SubBoardFactoryFs;
static uint8_t SubBoardFactoryButtonRaw14;
static uint8_t SubBoardFactoryButtonRaw15;
static uint8_t SubBoardFactoryButtonStable14;
static uint8_t SubBoardFactoryButtonStable15;
static uint32_t SubBoardFactoryButtonChanged14Ms;
static uint32_t SubBoardFactoryButtonChanged15Ms;

static uint8_t SubBoardFactoryToBcd(uint8_t value);
static uint8_t SubBoardFactoryDateValid(const SubBoardDateTime *time);
static uint8_t SubBoardFactoryRtcRawValid(const uint8_t raw[7]);
static uint32_t SubBoardFactorySecondOfDay(const SubBoardDateTime *time);
static uint8_t SubBoardFactoryWeekday(uint16_t year, uint8_t month, uint8_t day);
static void SubBoardFactoryCompileTime(SubBoardDateTime *time);
static int SubBoardFactoryReadRtcRaw(uint8_t raw[7], SubBoardDateTime *time, uint32_t *hal_error);
static int SubBoardFactoryWriteRtc(const SubBoardDateTime *time, uint32_t *hal_error);
static int SubBoardFactoryRunRtc(void);
static int SubBoardFactoryProbeSd(void);
static void SubBoardFactoryFill(uint8_t *buf, uint32_t offset, uint32_t seed);
static int SubBoardFactoryRunSdOne(uint32_t target_hz, SubBoardFactorySdSpeedResult *result, uint32_t file_id);
static void SubBoardFactoryButtonsInit(void);
static void SubBoardFactoryButtonsPoll(void);
static void SubBoardFactoryPromptButtons(void);
#endif

void SubBoardBringupRunOnce(void)
{
    SubBoardRtcEnsureReady();
}

void SubBoardBringupPoll(void)
{
}

int SdLogRtcNow(SdLogDateTime *out)
{
    SubBoardDateTime readback;
    uint32_t hal_error = 0u;

    if (out == NULL)
    {
        return 0;
    }

    SubBoardRtcEnsureReady();
    if (SubBoardRtcRead(&readback, &hal_error) != 0)
    {
        return 0;
    }

    out->year = readback.year;
    out->month = readback.month;
    out->day = readback.day;
    out->hour = readback.hour;
    out->minute = readback.minute;
    out->second = readback.second;
    return 1;
}

static void SubBoardI2c1Release(void)
{
    if (SubBoardI2c1Inited != 0u)
    {
        (void)HAL_I2C_DeInit(&SubBoardI2c1);
        __HAL_RCC_I2C1_FORCE_RESET();
        __HAL_RCC_I2C1_RELEASE_RESET();
        SubBoardI2c1Inited = 0u;
    }
}

static void SubBoardI2c1Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

    if (SubBoardI2c1Inited != 0u)
    {
        return;
    }

    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
    PeriphClkInitStruct.I2c123ClockSelection = RCC_I2C1CLKSOURCE_D2PCLK1;
    (void)HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();
    __HAL_RCC_I2C1_FORCE_RESET();
    __HAL_RCC_I2C1_RELEASE_RESET();

    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    memset(&SubBoardI2c1, 0, sizeof(SubBoardI2c1));
    SubBoardI2c1.Instance = I2C1;
    SubBoardI2c1.Init.Timing = 0x10707DBCu;
    SubBoardI2c1.Init.OwnAddress1 = 0;
    SubBoardI2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    SubBoardI2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    SubBoardI2c1.Init.OwnAddress2 = 0;
    SubBoardI2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    SubBoardI2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    SubBoardI2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    (void)HAL_I2C_Init(&SubBoardI2c1);
    SubBoardI2c1Inited = 1u;
}

static void SubBoardI2cBusClear(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    SubBoardI2c1Release();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_SET);
    osDelay(1);

    for (uint8_t i = 0u; i < 9u; i++)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
        osDelay(1);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
        osDelay(1);
    }

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
    osDelay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    osDelay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
    osDelay(1);
}

static void SubBoardRtcEnsureReady(void)
{
    if (SubBoardRtcProbed != 0u)
    {
        return;
    }

    SubBoardRtcProbed = 1u;
    SubBoardI2cBusClear();
    SubBoardI2c1Init();
}

static uint8_t SubBoardFromBcd(uint8_t value)
{
    return (uint8_t)(((value >> 4u) * 10u) + (value & 0x0Fu));
}

static int SubBoardRtcRead(SubBoardDateTime *readback, uint32_t *hal_error)
{
    uint8_t rx[7] = {0};

    if (readback == NULL || hal_error == NULL)
    {
        return -1;
    }

    *hal_error = 0u;

    if (HAL_I2C_IsDeviceReady(&SubBoardI2c1, SUB_BOARD_PCF8563_ADDR, 3u, SUB_BOARD_I2C_TIMEOUT_MS) != HAL_OK)
    {
        *hal_error = HAL_I2C_GetError(&SubBoardI2c1);
        return -2;
    }

    if (HAL_I2C_Mem_Read(&SubBoardI2c1, SUB_BOARD_PCF8563_ADDR, 0x02u, I2C_MEMADD_SIZE_8BIT,
                         rx, sizeof(rx), SUB_BOARD_I2C_TIMEOUT_MS) != HAL_OK)
    {
        *hal_error = HAL_I2C_GetError(&SubBoardI2c1);
        return -3;
    }

    readback->voltage_low = (rx[0] & 0x80u) ? 1u : 0u;
    readback->second = SubBoardFromBcd((uint8_t)(rx[0] & 0x7Fu));
    readback->minute = SubBoardFromBcd((uint8_t)(rx[1] & 0x7Fu));
    readback->hour = SubBoardFromBcd((uint8_t)(rx[2] & 0x3Fu));
    readback->day = SubBoardFromBcd((uint8_t)(rx[3] & 0x3Fu));
    readback->weekday = SubBoardFromBcd((uint8_t)(rx[4] & 0x07u));
    readback->month = SubBoardFromBcd((uint8_t)(rx[5] & 0x1Fu));
    readback->year = (uint16_t)(2000u + SubBoardFromBcd(rx[6]));

    return (readback->voltage_low != 0u) ? 1 : 0;
}

#if defined(SUB_BOARD_FACTORY_TEST)
static uint8_t SubBoardFactoryToBcd(uint8_t value)
{
    return (uint8_t)(((value / 10u) << 4u) | (value % 10u));
}

static uint8_t SubBoardFactoryDateValid(const SubBoardDateTime *time)
{
    static const uint8_t days_in_month[] = {31u, 28u, 31u, 30u, 31u, 30u, 31u, 31u, 30u, 31u, 30u, 31u};
    uint8_t max_day;

    if (time == NULL || time->year < 2024u || time->year > 2099u ||
        time->month < 1u || time->month > 12u || time->day < 1u ||
        time->hour > 23u || time->minute > 59u || time->second > 59u || time->weekday > 6u)
    {
        return 0u;
    }
    max_day = days_in_month[time->month - 1u];
    if (time->month == 2u &&
        ((time->year % 4u) == 0u && ((time->year % 100u) != 0u || (time->year % 400u) == 0u)))
    {
        max_day = 29u;
    }
    if (time->day > max_day)
    {
        return 0u;
    }
    return 1u;
}

static uint8_t SubBoardFactoryRtcRawValid(const uint8_t raw[7])
{
    static const uint8_t masks[] = {0x7Fu, 0x7Fu, 0x3Fu, 0x3Fu, 0x07u, 0x1Fu, 0xFFu};
    static const uint8_t maximum[] = {59u, 59u, 23u, 31u, 6u, 12u, 99u};

    if (raw == NULL)
    {
        return 0u;
    }
    for (uint8_t i = 0u; i < 7u; i++)
    {
        const uint8_t value = (uint8_t)(raw[i] & masks[i]);
        if ((value & 0x0Fu) > 9u || ((value >> 4u) & 0x0Fu) > 9u ||
            SubBoardFromBcd(value) > maximum[i])
        {
            return 0u;
        }
    }
    return 1u;
}

static uint32_t SubBoardFactorySecondOfDay(const SubBoardDateTime *time)
{
    return ((uint32_t)time->hour * 3600u) + ((uint32_t)time->minute * 60u) + time->second;
}

static uint8_t SubBoardFactoryWeekday(uint16_t year, uint8_t month, uint8_t day)
{
    static const uint8_t offsets[] = {0u, 3u, 2u, 5u, 0u, 3u, 5u, 1u, 4u, 6u, 2u, 4u};
    uint16_t adjusted_year = year;

    if (month < 3u)
    {
        adjusted_year--;
    }
    return (uint8_t)((adjusted_year + adjusted_year / 4u - adjusted_year / 100u + adjusted_year / 400u +
                      offsets[month - 1u] + day) % 7u);
}

static void SubBoardFactoryCompileTime(SubBoardDateTime *time)
{
    static const char months[] = "JanFebMarAprMayJunJulAugSepOctNovDec";
    const char *month = __DATE__;
    uint8_t month_id = 0u;

    memset(time, 0, sizeof(*time));
    for (uint8_t i = 0u; i < 12u; i++)
    {
        if (memcmp(month, &months[i * 3u], 3u) == 0)
        {
            month_id = (uint8_t)(i + 1u);
            break;
        }
    }
    time->year = (uint16_t)((__DATE__[7] - '0') * 1000u + (__DATE__[8] - '0') * 100u +
                            (__DATE__[9] - '0') * 10u + (__DATE__[10] - '0'));
    time->month = month_id;
    time->day = (uint8_t)(((__DATE__[4] == ' ') ? 0u : (__DATE__[4] - '0') * 10u) + (__DATE__[5] - '0'));
    time->hour = (uint8_t)((__TIME__[0] - '0') * 10u + (__TIME__[1] - '0'));
    time->minute = (uint8_t)((__TIME__[3] - '0') * 10u + (__TIME__[4] - '0'));
    time->second = (uint8_t)((__TIME__[6] - '0') * 10u + (__TIME__[7] - '0'));
    time->weekday = SubBoardFactoryWeekday(time->year, time->month, time->day);
}

static int SubBoardFactoryReadRtcRaw(uint8_t raw[7], SubBoardDateTime *time, uint32_t *hal_error)
{
    if (raw == NULL || time == NULL || hal_error == NULL)
    {
        return -1;
    }
    *hal_error = 0u;
    if (HAL_I2C_IsDeviceReady(&SubBoardI2c1, SUB_BOARD_PCF8563_ADDR, 3u, SUB_BOARD_I2C_TIMEOUT_MS) != HAL_OK)
    {
        *hal_error = HAL_I2C_GetError(&SubBoardI2c1);
        return -2;
    }
    if (HAL_I2C_Mem_Read(&SubBoardI2c1, SUB_BOARD_PCF8563_ADDR, 0x02u, I2C_MEMADD_SIZE_8BIT,
                         raw, 7u, SUB_BOARD_I2C_TIMEOUT_MS) != HAL_OK)
    {
        *hal_error = HAL_I2C_GetError(&SubBoardI2c1);
        return -3;
    }
    time->voltage_low = (raw[0] & 0x80u) ? 1u : 0u;
    time->second = SubBoardFromBcd((uint8_t)(raw[0] & 0x7Fu));
    time->minute = SubBoardFromBcd((uint8_t)(raw[1] & 0x7Fu));
    time->hour = SubBoardFromBcd((uint8_t)(raw[2] & 0x3Fu));
    time->day = SubBoardFromBcd((uint8_t)(raw[3] & 0x3Fu));
    time->weekday = SubBoardFromBcd((uint8_t)(raw[4] & 0x07u));
    time->month = SubBoardFromBcd((uint8_t)(raw[5] & 0x1Fu));
    time->year = (uint16_t)(2000u + SubBoardFromBcd(raw[6]));
    return 0;
}

static int SubBoardFactoryWriteRtc(const SubBoardDateTime *time, uint32_t *hal_error)
{
    uint8_t control[3] = {0x00u, 0x00u, 0x00u};
    uint8_t regs[8];

    if (time == NULL || hal_error == NULL)
    {
        return -1;
    }
    *hal_error = 0u;
    if (HAL_I2C_Master_Transmit(&SubBoardI2c1, SUB_BOARD_PCF8563_ADDR, control, sizeof(control),
                                SUB_BOARD_I2C_TIMEOUT_MS) != HAL_OK)
    {
        *hal_error = HAL_I2C_GetError(&SubBoardI2c1);
        return -2;
    }
    regs[0] = 0x02u;
    regs[1] = SubBoardFactoryToBcd(time->second);
    regs[2] = SubBoardFactoryToBcd(time->minute);
    regs[3] = SubBoardFactoryToBcd(time->hour);
    regs[4] = SubBoardFactoryToBcd(time->day);
    regs[5] = SubBoardFactoryToBcd(time->weekday);
    regs[6] = SubBoardFactoryToBcd(time->month);
    regs[7] = SubBoardFactoryToBcd((uint8_t)(time->year % 100u));
    if (HAL_I2C_Master_Transmit(&SubBoardI2c1, SUB_BOARD_PCF8563_ADDR, regs, sizeof(regs),
                                SUB_BOARD_I2C_TIMEOUT_MS) != HAL_OK)
    {
        *hal_error = HAL_I2C_GetError(&SubBoardI2c1);
        return -3;
    }
    return 0;
}

static int SubBoardFactoryRunRtc(void)
{
    SubBoardDateTime first;
    SubBoardDateTime later;
    uint32_t hal_error = 0u;
    uint32_t first_seconds;
    uint32_t later_seconds;
    int ret;

    SubBoardRtcProbed = 0u;
    SubBoardRtcEnsureReady();
    ret = SubBoardFactoryReadRtcRaw((uint8_t *)g_sub_board_factory_test.rtc_raw_before, &first, &hal_error);
    g_sub_board_factory_test.rtc_hal_error = hal_error;
    if (ret != 0)
    {
        g_sub_board_factory_test.error = ret;
        return ret;
    }
    g_sub_board_factory_test.rtc_initial_voltage_low = first.voltage_low;
    if (first.voltage_low != 0u ||
        SubBoardFactoryRtcRawValid((const uint8_t *)g_sub_board_factory_test.rtc_raw_before) == 0u ||
        SubBoardFactoryDateValid(&first) == 0u)
    {
        SubBoardFactoryCompileTime(&first);
        ret = SubBoardFactoryWriteRtc(&first, &hal_error);
        g_sub_board_factory_test.rtc_hal_error = hal_error;
        if (ret != 0)
        {
            g_sub_board_factory_test.error = ret;
            return ret;
        }
        g_sub_board_factory_test.rtc_initialized = 1u;
    }
    ret = SubBoardFactoryReadRtcRaw((uint8_t *)g_sub_board_factory_test.rtc_raw_after, &first, &hal_error);
    g_sub_board_factory_test.rtc_hal_error = hal_error;
    if (ret != 0 || first.voltage_low != 0u ||
        SubBoardFactoryRtcRawValid((const uint8_t *)g_sub_board_factory_test.rtc_raw_after) == 0u ||
        SubBoardFactoryDateValid(&first) == 0u)
    {
        g_sub_board_factory_test.error = (ret != 0) ? ret : -4;
        return g_sub_board_factory_test.error;
    }
    first_seconds = SubBoardFactorySecondOfDay(&first);
    osDelay(2200u);
    ret = SubBoardFactoryReadRtcRaw((uint8_t *)g_sub_board_factory_test.rtc_raw_after, &later, &hal_error);
    g_sub_board_factory_test.rtc_hal_error = hal_error;
    if (ret != 0)
    {
        g_sub_board_factory_test.error = ret;
        return ret;
    }
    later_seconds = SubBoardFactorySecondOfDay(&later);
    if (later_seconds < first_seconds)
    {
        if (first_seconds < 86395u || later_seconds > 5u)
        {
            g_sub_board_factory_test.error = -5;
            return -5;
        }
        later_seconds += 86400u;
    }
    if (later.voltage_low != 0u ||
        SubBoardFactoryRtcRawValid((const uint8_t *)g_sub_board_factory_test.rtc_raw_after) == 0u ||
        SubBoardFactoryDateValid(&later) == 0u ||
        (later_seconds - first_seconds) < 2u || (later_seconds - first_seconds) > 4u)
    {
        g_sub_board_factory_test.error = -6;
        return -6;
    }
    g_sub_board_factory_test.rtc_walk_seconds = (uint16_t)(later_seconds - first_seconds);
    g_sub_board_factory_test.rtc_walk_ok = 1u;
    g_sub_board_factory_test.rtc_year = later.year;
    g_sub_board_factory_test.rtc_month = later.month;
    g_sub_board_factory_test.rtc_day = later.day;
    g_sub_board_factory_test.rtc_hour = later.hour;
    g_sub_board_factory_test.rtc_minute = later.minute;
    g_sub_board_factory_test.rtc_second = later.second;
    return 0;
}

static void SubBoardFactoryFill(uint8_t *buf, uint32_t offset, uint32_t seed)
{
    for (uint32_t i = 0u; i < SUB_BOARD_FACTORY_CHUNK_BYTES; i++)
    {
        buf[i] = (uint8_t)((offset + i + seed) ^ ((offset + i) >> 8u));
    }
}

static int SubBoardFactoryProbeSd(void)
{
    uint32_t sectors = 0u;
    FRESULT fr = f_mount(&SubBoardFactoryFs, "0:", 1u);
    int ret = 0;

    if (fr != FR_OK)
    {
        ret = -(int)fr - 80;
        (void)f_mount(NULL, "0:", 0u);
        return ret;
    }
    g_sub_board_factory_test.sd_fs_type = SubBoardFactoryFs.fs_type;
    g_sub_board_factory_test.sd_card_type = (uint8_t)SdSpiGetCardType();
    if (SdSpiGetSectorCount(&sectors) != 0 || sectors == 0u)
    {
        ret = -81;
    }
    else
    {
        g_sub_board_factory_test.sd_sector_count = sectors;
    }
    (void)f_mount(NULL, "0:", 0u);
    return ret;
}

static int SubBoardFactoryRunSdOne(uint32_t target_hz, SubBoardFactorySdSpeedResult *result, uint32_t file_id)
{
    FIL file;
    char path[40];
    UINT count;
    uint32_t start;
    uint32_t sync_start;
    FRESULT fr;
    int ret = 0;
    uint8_t file_open = 0u;
    uint8_t file_created = 0u;

    memset(result, 0, sizeof(*result));
    result->target_hz = target_hz;
    result->data_bytes = SUB_BOARD_FACTORY_IO_BYTES;
    fr = f_mount(&SubBoardFactoryFs, "0:", 1u);
    if (fr != FR_OK)
    {
        ret = -(int)fr - 20;
        goto cleanup;
    }
    if (SdSpiPortFactorySetHz(target_hz, &result->actual_hz) != 0)
    {
        ret = -10;
        goto cleanup;
    }
    fr = FR_EXIST;
    for (uint32_t attempt = 0u; attempt < 32u && fr == FR_EXIST; attempt++)
    {
        (void)snprintf(path, sizeof(path), "0:/SBF%08lX.BIN", (unsigned long)file_id);
        fr = f_open(&file, path, FA_CREATE_NEW | FA_WRITE);
        file_id++;
    }
    file_id--;
    if (fr != FR_OK)
    {
        ret = -(int)fr - 40;
        goto cleanup;
    }
    file_open = 1u;
    file_created = 1u;
    start = HAL_GetTick();
    for (uint32_t offset = 0u; offset < SUB_BOARD_FACTORY_IO_BYTES; offset += SUB_BOARD_FACTORY_CHUNK_BYTES)
    {
        SubBoardFactoryFill(SubBoardFactoryIoBuf, offset, file_id);
        fr = f_write(&file, SubBoardFactoryIoBuf, SUB_BOARD_FACTORY_CHUNK_BYTES, &count);
        if (fr != FR_OK || count != SUB_BOARD_FACTORY_CHUNK_BYTES)
        {
            ret = -60;
            goto cleanup;
        }
    }
    result->write_ms = HAL_GetTick() - start;
    sync_start = HAL_GetTick();
    fr = f_sync(&file);
    result->sync_ms = HAL_GetTick() - sync_start;
    result->write_ms += result->sync_ms;
    if (fr != FR_OK)
    {
        ret = -61;
        goto cleanup;
    }
    fr = f_close(&file);
    if (fr != FR_OK)
    {
        ret = -66;
        goto cleanup;
    }
    file_open = 0u;
    result->write_bytes_per_s = (result->write_ms != 0u) ?
        (SUB_BOARD_FACTORY_IO_BYTES * 1000u) / result->write_ms : 0u;
    fr = f_open(&file, path, FA_READ);
    if (fr != FR_OK)
    {
        ret = -62;
        goto cleanup;
    }
    file_open = 1u;
    start = HAL_GetTick();
    for (uint32_t offset = 0u; offset < SUB_BOARD_FACTORY_IO_BYTES; offset += SUB_BOARD_FACTORY_CHUNK_BYTES)
    {
        fr = f_read(&file, SubBoardFactoryIoBuf, SUB_BOARD_FACTORY_CHUNK_BYTES, &count);
        if (fr != FR_OK || count != SUB_BOARD_FACTORY_CHUNK_BYTES)
        {
            ret = -63;
            goto cleanup;
        }
        for (uint32_t i = 0u; i < SUB_BOARD_FACTORY_CHUNK_BYTES; i++)
        {
            const uint8_t expected = (uint8_t)((offset + i + file_id) ^ ((offset + i) >> 8u));
            if (SubBoardFactoryIoBuf[i] != expected)
            {
                ret = -64;
                goto cleanup;
            }
        }
    }
    result->read_ms = HAL_GetTick() - start;
    fr = f_close(&file);
    if (fr != FR_OK)
    {
        ret = -67;
        goto cleanup;
    }
    file_open = 0u;
    result->read_bytes_per_s = (result->read_ms != 0u) ?
        (SUB_BOARD_FACTORY_IO_BYTES * 1000u) / result->read_ms : 0u;

cleanup:
    if (file_open != 0u)
    {
        (void)f_close(&file);
    }
    if (file_created != 0u && f_unlink(path) != FR_OK && ret == 0)
    {
        ret = -65;
    }
    (void)f_mount(NULL, "0:", 0u);
    return ret;
}

static void SubBoardFactoryButtonsInit(void)
{
    GPIO_InitTypeDef init = {0};
    const uint32_t now = HAL_GetTick();

    __HAL_RCC_GPIOD_CLK_ENABLE();
    init.Pin = GPIO_PIN_14 | GPIO_PIN_15;
    init.Mode = GPIO_MODE_INPUT;
    init.Pull = GPIO_PULLUP;
    init.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &init);
    osDelay(SUB_BOARD_FACTORY_DEBOUNCE_MS);
    SubBoardFactoryButtonRaw14 = (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_14) == GPIO_PIN_SET) ? 1u : 0u;
    SubBoardFactoryButtonRaw15 = (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_15) == GPIO_PIN_SET) ? 1u : 0u;
    SubBoardFactoryButtonStable14 = SubBoardFactoryButtonRaw14;
    SubBoardFactoryButtonStable15 = SubBoardFactoryButtonRaw15;
    SubBoardFactoryButtonChanged14Ms = now;
    SubBoardFactoryButtonChanged15Ms = now;
    g_sub_board_factory_test.pd14_raw = SubBoardFactoryButtonRaw14;
    g_sub_board_factory_test.pd15_raw = SubBoardFactoryButtonRaw15;
    g_sub_board_factory_test.pd14_stable = SubBoardFactoryButtonStable14;
    g_sub_board_factory_test.pd15_stable = SubBoardFactoryButtonStable15;
}

static void SubBoardFactoryButtonsPoll(void)
{
    const uint32_t now_ms = HAL_GetTick();
    const uint8_t now14 = (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_14) == GPIO_PIN_SET) ? 1u : 0u;
    const uint8_t now15 = (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_15) == GPIO_PIN_SET) ? 1u : 0u;

    g_sub_board_factory_test.pd14_raw = now14;
    g_sub_board_factory_test.pd15_raw = now15;
    if (now14 != SubBoardFactoryButtonRaw14)
    {
        SubBoardFactoryButtonRaw14 = now14;
        SubBoardFactoryButtonChanged14Ms = now_ms;
    }
    if (now15 != SubBoardFactoryButtonRaw15)
    {
        SubBoardFactoryButtonRaw15 = now15;
        SubBoardFactoryButtonChanged15Ms = now_ms;
    }
    if (SubBoardFactoryButtonStable14 != SubBoardFactoryButtonRaw14 &&
        (uint32_t)(now_ms - SubBoardFactoryButtonChanged14Ms) >= SUB_BOARD_FACTORY_DEBOUNCE_MS)
    {
        SubBoardFactoryButtonStable14 = SubBoardFactoryButtonRaw14;
        g_sub_board_factory_test.pd14_stable = SubBoardFactoryButtonStable14;
        if (SubBoardFactoryButtonStable14 == 0u)
        {
            g_sub_board_factory_test.pd14_pressed_count++;
        }
        else
        {
            g_sub_board_factory_test.pd14_released_count++;
        }
    }
    if (SubBoardFactoryButtonStable15 != SubBoardFactoryButtonRaw15 &&
        (uint32_t)(now_ms - SubBoardFactoryButtonChanged15Ms) >= SUB_BOARD_FACTORY_DEBOUNCE_MS)
    {
        SubBoardFactoryButtonStable15 = SubBoardFactoryButtonRaw15;
        g_sub_board_factory_test.pd15_stable = SubBoardFactoryButtonStable15;
        if (SubBoardFactoryButtonStable15 == 0u)
        {
            g_sub_board_factory_test.pd15_pressed_count++;
        }
        else
        {
            g_sub_board_factory_test.pd15_released_count++;
        }
    }
}

static void SubBoardFactoryPromptButtons(void)
{
    int ret;

    MX_TIM12_Init();
    BuzzerSetEnable(1u);
    ret = BuzzerToneStartHz(1600u, 180u);
    if (ret == 0)
    {
        osDelay(120u);
        BuzzerToneStop();
        osDelay(80u);
        ret = BuzzerToneStartHz(2200u, 180u);
    }
    if (ret == 0)
    {
        osDelay(120u);
        g_sub_board_factory_test.buzzer_prompted = 1u;
    }
    BuzzerToneStop();
    g_sub_board_factory_test.buzzer_result = ret;
    if (ret != 0 && g_sub_board_factory_test.error == 0)
    {
        g_sub_board_factory_test.error = -90;
    }
}

void SubBoardFactoryTestTask(void *argument)
{
    static const uint32_t speeds_hz[3] = {6250000u, 12500000u, 25000000u};
    (void)argument;
    memset((void *)&g_sub_board_factory_test, 0, sizeof(g_sub_board_factory_test));
    g_sub_board_factory_test.magic = SUB_BOARD_FACTORY_TEST_MAGIC;
    g_sub_board_factory_test.version = SUB_BOARD_FACTORY_TEST_VERSION;
    g_sub_board_factory_test.size = (uint16_t)sizeof(g_sub_board_factory_test);
    g_sub_board_factory_test.started_ms = HAL_GetTick();
    g_sub_board_factory_test.phase = SUB_BOARD_FACTORY_PHASE_RTC;
    if (SubBoardFactoryRunRtc() != 0)
    {
        g_sub_board_factory_test.phase = SUB_BOARD_FACTORY_PHASE_FAILED;
    }
    g_sub_board_factory_test.phase = SUB_BOARD_FACTORY_PHASE_SD;
    g_sub_board_factory_test.sd_probe_result = SubBoardFactoryProbeSd();
    if (g_sub_board_factory_test.sd_probe_result != 0 && g_sub_board_factory_test.error == 0)
    {
        g_sub_board_factory_test.error = g_sub_board_factory_test.sd_probe_result;
    }
    for (uint32_t i = 0u; i < 3u; i++)
    {
        const int ret = SubBoardFactoryRunSdOne(speeds_hz[i],
                                                (SubBoardFactorySdSpeedResult *)&g_sub_board_factory_test.sd[i],
                                                g_sub_board_factory_test.started_ms + i);
        g_sub_board_factory_test.sd[i].result = ret;
        if (ret != 0 && g_sub_board_factory_test.error == 0)
        {
            g_sub_board_factory_test.error = ret;
        }
    }
    SubBoardFactoryButtonsInit();
    SubBoardFactoryPromptButtons();
    g_sub_board_factory_test.phase = SUB_BOARD_FACTORY_PHASE_BUTTONS;
    for (;;)
    {
        SubBoardFactoryButtonsPoll();
        if (g_sub_board_factory_test.pd14_pressed_count != 0u &&
            g_sub_board_factory_test.pd14_released_count != 0u &&
            g_sub_board_factory_test.pd15_pressed_count != 0u &&
            g_sub_board_factory_test.pd15_released_count != 0u &&
            g_sub_board_factory_test.finished_ms == 0u)
        {
            g_sub_board_factory_test.finished_ms = HAL_GetTick();
            g_sub_board_factory_test.phase = (g_sub_board_factory_test.error == 0) ?
                SUB_BOARD_FACTORY_PHASE_DONE : SUB_BOARD_FACTORY_PHASE_FAILED;
        }
        osDelay(1u);
    }
}
#endif
