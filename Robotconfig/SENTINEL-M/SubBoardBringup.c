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
