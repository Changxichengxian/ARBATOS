/*
 * SPDX-FileCopyrightText: 2026 Chen Xuan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-06-25
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "SubBoardBringup.h"

#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "cmsis_os.h"
#include "BspUartDispatch.h"
#include "BspSdSpiPort.h"
#include "fatfs/ff.h"
#include "main.h"
#include "SdSpi.h"

#define SUB_BOARD_UART_TIMEOUT_MS 100u
#define SUB_BOARD_I2C_TIMEOUT_MS  100u
#define SUB_BOARD_PCF8563_ADDR    (0x51u << 1)
#define SUB_BOARD_CMD_BUF_SIZE    64u
#define SUB_BOARD_RX_BUF_SIZE     64u
#define SUB_BOARD_RX_RING_SIZE    128u
#define SUB_BOARD_I2C_HOLD_LOW_ON_BOOT 0u

typedef struct
{
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t weekday;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
} SubBoardDateTime;

static UART_HandleTypeDef SubBoardUart9;
static I2C_HandleTypeDef SubBoardI2c1;
static uint8_t SubBoardRunDone = 0u;
static uint8_t SubBoardI2c1Inited = 0u;
static char SubBoardCmdBuf[SUB_BOARD_CMD_BUF_SIZE];
static uint8_t SubBoardCmdLen = 0u;
static uint8_t SubBoardRxBuf[SUB_BOARD_RX_BUF_SIZE];
static uint8_t SubBoardRxRing[SUB_BOARD_RX_RING_SIZE];
static volatile uint16_t SubBoardRxHead = 0u;
static volatile uint16_t SubBoardRxTail = 0u;
static volatile uint32_t SubBoardRxDrop = 0u;
static volatile uint32_t SubBoardRxError = 0u;
static uint8_t SubBoardSdSector0[512];
static uint8_t SubBoardSdVerifyBuf[512];
static FATFS SubBoardSdFs;
static FIL SubBoardSdFile;
static char SubBoardSdWriteBuf[128];
static char SubBoardSdReadBuf[128];
static uint16_t SubBoardI2cSoftSclPin = GPIO_PIN_8;
static uint16_t SubBoardI2cSoftSdaPin = GPIO_PIN_9;
static uint32_t SubBoardI2cSoftDelayLoop = 1200u;

static void SubBoardUart9Init(void);
static void SubBoardUart9StartRx(void);
static void SubBoardUart9RxEvent(UART_HandleTypeDef *huart, uint16_t size);
static void SubBoardUart9Error(UART_HandleTypeDef *huart);
static uint8_t SubBoardRxPop(uint8_t *out);
static void SubBoardI2c1Init(void);
static void SubBoardI2c1Release(void);
static void SubBoardPrintf(const char *fmt, ...);
static bool SubBoardIsDigit(char c);
static uint8_t SubBoardDec2(const char *text);
static uint8_t SubBoardWeekday(uint16_t year, uint8_t month, uint8_t day);
static uint8_t SubBoardToBcd(uint8_t value);
static uint8_t SubBoardFromBcd(uint8_t value);
static const char *SubBoardSdCardTypeName(SdSpiCardType type);
static int SubBoardRtcWriteAndRead(const SubBoardDateTime *target, SubBoardDateTime *readback, uint32_t *hal_error);
static int SubBoardRtcRead(SubBoardDateTime *readback, uint32_t *hal_error);
static void SubBoardPrintRtcTime(const char *prefix, const SubBoardDateTime *time);
static bool SubBoardParseSetRtc(const char *cmd, SubBoardDateTime *out);
static void SubBoardHandleCommand(const char *cmd);
static void SubBoardPrintPinLevels(const char *prefix);
static void SubBoardI2cHoldLowApply(void);
static void SubBoardI2cPinsSet(GPIO_PinState state);
static void SubBoardI2cPinsSplit(GPIO_PinState scl_state, GPIO_PinState sda_state);
static void SubBoardI2cSoftScan(void);
static void SubBoardI2cSoftSlowScan(void);
static void SubBoardI2cSoftSwapScan(void);
static void SubBoardI2cSoftScanPins(uint16_t scl_pin, uint16_t sda_pin, const char *label);
static void SubBoardI2cBusClear(void);
static void SubBoardI2cProbe51(void);
static void SubBoardI2cRiseTest(void);
static uint32_t SubBoardI2cRiseOne(uint16_t pin);
static void SubBoardI2cSoftInit(uint16_t scl_pin, uint16_t sda_pin);
static void SubBoardI2cSoftDelay(void);
static void SubBoardI2cSoftSetScl(GPIO_PinState state);
static void SubBoardI2cSoftSetSda(GPIO_PinState state);
static GPIO_PinState SubBoardI2cSoftReadSda(void);
static void SubBoardI2cSoftStart(void);
static void SubBoardI2cSoftStop(void);
static uint8_t SubBoardI2cSoftWriteByte(uint8_t data);
static void SubBoardSdPinsSet(GPIO_PinState state);
static void SubBoardRunI2cPulse(void);
static void SubBoardRunSdPulse(void);
static void SubBoardRunSdTest(void);
static void SubBoardRunSdRawTest(void);
static void SubBoardRunSdRwTest(void);
static void SubBoardRunSdStressTest(void);
static void SubBoardFillPattern(uint8_t *buf, uint32_t sector, uint32_t seed);
static uint32_t SubBoardHashBytes(uint32_t hash, const uint8_t *buf, uint32_t len);
static uint8_t SubBoardSdRawCmd(uint8_t cmd, uint32_t arg, uint8_t crc, uint8_t *rx, uint8_t rx_len);
static void SubBoardRunI2cScan(void);
static void SubBoardRunRtcRead(void);

void SubBoardBringupRunOnce(void)
{
    if (SubBoardRunDone != 0u)
    {
        return;
    }
    SubBoardRunDone = 1u;

    SubBoardUart9Init();
    SubBoardI2cHoldLowApply();
    osDelay(100);

    SubBoardPrintf("\r\n[sub] bringup start\r\n");
    SubBoardPrintf("[sub] UART9 PD15/PD14 115200 8N1\r\n");
    SubBoardPrintf("[sub] SD SPI3 PC10/PC11/PC12 CS PE14\r\n");
    SubBoardPrintf("[sub] RTC PCF8563 I2C1 PB8/PB9 addr 0x51\r\n");

    SubBoardRunSdTest();
#if SUB_BOARD_I2C_HOLD_LOW_ON_BOOT
    SubBoardI2cHoldLowApply();
    SubBoardPrintf("[sub] I2C PB8/PB9 hold LOW on boot\r\n");
#else
    SubBoardRunRtcRead();
#endif

    SubBoardPrintf("[sub] cmd: RTC? I2C? I2CSOFT? I2CSLOW? I2CSWAP? I2C51? I2CRISE? SD? SDRW? SDSTRESS?\r\n");
    SubBoardPrintf("[sub] force: I2CCLEAR I2CLOW I2CHIGH SCLLOW SDALOW I2CPULSE SDRAW? SDLOW SDHIGH SDPULSE\r\n\r\n");
}

void SubBoardBringupPoll(void)
{
    uint8_t ch = 0u;
    uint8_t rx_budget = 16u;

    if (SubBoardRunDone == 0u)
    {
        return;
    }

    SubBoardI2cHoldLowApply();

    while (rx_budget-- > 0u)
    {
        if (SubBoardRxPop(&ch) == 0u)
        {
            return;
        }

        if (ch == '\n' || ch == '\r')
        {
            if (SubBoardCmdLen > 0u)
            {
                SubBoardCmdBuf[SubBoardCmdLen] = '\0';
                SubBoardHandleCommand(SubBoardCmdBuf);
                SubBoardCmdLen = 0u;
            }
            continue;
        }

        if (SubBoardCmdLen < (SUB_BOARD_CMD_BUF_SIZE - 1u))
        {
            SubBoardCmdBuf[SubBoardCmdLen++] = (char)ch;
        }
        else
        {
            SubBoardCmdLen = 0u;
            SubBoardPrintf("[sub] CMD FAIL line too long\r\n");
        }
    }
}

static void SubBoardUart9Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART9;
    PeriphClkInitStruct.Usart16ClockSelection = RCC_UART9CLKSOURCE_D2PCLK2;
    (void)HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_UART9_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF11_UART9;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    memset(&SubBoardUart9, 0, sizeof(SubBoardUart9));
    SubBoardRxHead = 0u;
    SubBoardRxTail = 0u;
    SubBoardRxDrop = 0u;
    SubBoardRxError = 0u;

    SubBoardUart9.Instance = UART9;
    SubBoardUart9.Init.BaudRate = 115200;
    SubBoardUart9.Init.WordLength = UART_WORDLENGTH_8B;
    SubBoardUart9.Init.StopBits = UART_STOPBITS_1;
    SubBoardUart9.Init.Parity = UART_PARITY_NONE;
    SubBoardUart9.Init.Mode = UART_MODE_TX_RX;
    SubBoardUart9.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    SubBoardUart9.Init.OverSampling = UART_OVERSAMPLING_16;
    SubBoardUart9.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    SubBoardUart9.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    SubBoardUart9.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    (void)HAL_UART_Init(&SubBoardUart9);
    (void)HAL_UARTEx_SetTxFifoThreshold(&SubBoardUart9, UART_TXFIFO_THRESHOLD_1_8);
    (void)HAL_UARTEx_SetRxFifoThreshold(&SubBoardUart9, UART_RXFIFO_THRESHOLD_1_8);
    (void)HAL_UARTEx_DisableFifoMode(&SubBoardUart9);

    (void)BspUartDispatchRegister(&SubBoardUart9, SubBoardUart9RxEvent, SubBoardUart9Error);
    HAL_NVIC_SetPriority(UART9_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(UART9_IRQn);
    SubBoardUart9StartRx();
}

void UART9_IRQHandler(void)
{
    HAL_UART_IRQHandler(&SubBoardUart9);
}

static void SubBoardUart9StartRx(void)
{
    (void)HAL_UART_AbortReceive(&SubBoardUart9);
    __HAL_UART_CLEAR_OREFLAG(&SubBoardUart9);
    __HAL_UART_CLEAR_NEFLAG(&SubBoardUart9);
    __HAL_UART_CLEAR_FEFLAG(&SubBoardUart9);
    (void)HAL_UARTEx_ReceiveToIdle_IT(&SubBoardUart9, SubBoardRxBuf, (uint16_t)sizeof(SubBoardRxBuf));
}

static void SubBoardUart9RxEvent(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart != &SubBoardUart9)
    {
        return;
    }

    if (size > (uint16_t)sizeof(SubBoardRxBuf))
    {
        size = (uint16_t)sizeof(SubBoardRxBuf);
    }

    for (uint16_t i = 0u; i < size; i++)
    {
        const uint16_t h = SubBoardRxHead;
        const uint16_t next = (uint16_t)((h + 1u) & (SUB_BOARD_RX_RING_SIZE - 1u));
        if (next == SubBoardRxTail)
        {
            SubBoardRxDrop++;
            break;
        }
        SubBoardRxRing[h] = SubBoardRxBuf[i];
        SubBoardRxHead = next;
    }

    SubBoardUart9StartRx();
}

static void SubBoardUart9Error(UART_HandleTypeDef *huart)
{
    if (huart != &SubBoardUart9)
    {
        return;
    }
    SubBoardRxError++;
    SubBoardUart9StartRx();
}

static uint8_t SubBoardRxPop(uint8_t *out)
{
    const uint16_t t = SubBoardRxTail;
    if (out == NULL || t == SubBoardRxHead)
    {
        return 0u;
    }

    *out = SubBoardRxRing[t];
    SubBoardRxTail = (uint16_t)((t + 1u) & (SUB_BOARD_RX_RING_SIZE - 1u));
    return 1u;
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
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
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

static void SubBoardPrintf(const char *fmt, ...)
{
    char buf[160];
    va_list args;

    va_start(args, fmt);
    const int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (len <= 0)
    {
        return;
    }

    const uint16_t tx_len = (len < (int)sizeof(buf)) ? (uint16_t)len : (uint16_t)(sizeof(buf) - 1u);
    (void)HAL_UART_Transmit(&SubBoardUart9, (uint8_t *)buf, tx_len, SUB_BOARD_UART_TIMEOUT_MS);
}

static bool SubBoardIsDigit(char c)
{
    return c >= '0' && c <= '9';
}

static uint8_t SubBoardDec2(const char *text)
{
    if (!SubBoardIsDigit(text[0]) || !SubBoardIsDigit(text[1]))
    {
        return 0u;
    }
    return (uint8_t)(((uint8_t)(text[0] - '0') * 10u) + (uint8_t)(text[1] - '0'));
}

static uint8_t SubBoardWeekday(uint16_t year, uint8_t month, uint8_t day)
{
    static const uint8_t offsets[] = {0u, 3u, 2u, 5u, 0u, 3u, 5u, 1u, 4u, 6u, 2u, 4u};
    uint16_t y = year;

    if (month < 3u)
    {
        y--;
    }
    return (uint8_t)((y + y / 4u - y / 100u + y / 400u + offsets[month - 1u] + day) % 7u);
}

static uint8_t SubBoardToBcd(uint8_t value)
{
    return (uint8_t)(((value / 10u) << 4u) | (value % 10u));
}

static uint8_t SubBoardFromBcd(uint8_t value)
{
    return (uint8_t)(((value >> 4u) * 10u) + (value & 0x0Fu));
}

static const char *SubBoardSdCardTypeName(SdSpiCardType type)
{
    switch (type)
    {
    case SD_SPI_CARD_MMC:
        return "MMC";
    case SD_SPI_CARD_SDSC:
        return "SDSC";
    case SD_SPI_CARD_SDHC:
        return "SDHC";
    case SD_SPI_CARD_NONE:
    default:
        return "NONE";
    }
}

static int SubBoardRtcWriteAndRead(const SubBoardDateTime *target, SubBoardDateTime *readback, uint32_t *hal_error)
{
    uint8_t control[3] = {0x00u, 0x00u, 0x00u};
    uint8_t time_regs[8];
    uint8_t rx[7] = {0};

    *hal_error = 0u;

    if (HAL_I2C_IsDeviceReady(&SubBoardI2c1, SUB_BOARD_PCF8563_ADDR, 3u, SUB_BOARD_I2C_TIMEOUT_MS) != HAL_OK)
    {
        *hal_error = HAL_I2C_GetError(&SubBoardI2c1);
        return -1;
    }

    if (HAL_I2C_Master_Transmit(&SubBoardI2c1, SUB_BOARD_PCF8563_ADDR, control, sizeof(control), SUB_BOARD_I2C_TIMEOUT_MS) != HAL_OK)
    {
        *hal_error = HAL_I2C_GetError(&SubBoardI2c1);
        return -2;
    }

    time_regs[0] = 0x02u;
    time_regs[1] = SubBoardToBcd(target->second) & 0x7Fu;
    time_regs[2] = SubBoardToBcd(target->minute) & 0x7Fu;
    time_regs[3] = SubBoardToBcd(target->hour) & 0x3Fu;
    time_regs[4] = SubBoardToBcd(target->day) & 0x3Fu;
    time_regs[5] = SubBoardToBcd(target->weekday) & 0x07u;
    time_regs[6] = SubBoardToBcd(target->month) & 0x1Fu;
    time_regs[7] = SubBoardToBcd((uint8_t)(target->year % 100u));

    if (HAL_I2C_Master_Transmit(&SubBoardI2c1, SUB_BOARD_PCF8563_ADDR, time_regs, sizeof(time_regs), SUB_BOARD_I2C_TIMEOUT_MS) != HAL_OK)
    {
        *hal_error = HAL_I2C_GetError(&SubBoardI2c1);
        return -3;
    }

    if (HAL_I2C_Mem_Read(&SubBoardI2c1, SUB_BOARD_PCF8563_ADDR, 0x02u, I2C_MEMADD_SIZE_8BIT, rx, sizeof(rx), SUB_BOARD_I2C_TIMEOUT_MS) != HAL_OK)
    {
        *hal_error = HAL_I2C_GetError(&SubBoardI2c1);
        return -4;
    }

    readback->second = SubBoardFromBcd(rx[0] & 0x7Fu);
    readback->minute = SubBoardFromBcd(rx[1] & 0x7Fu);
    readback->hour = SubBoardFromBcd(rx[2] & 0x3Fu);
    readback->day = SubBoardFromBcd(rx[3] & 0x3Fu);
    readback->weekday = SubBoardFromBcd(rx[4] & 0x07u);
    readback->month = SubBoardFromBcd(rx[5] & 0x1Fu);
    readback->year = (uint16_t)(2000u + SubBoardFromBcd(rx[6]));
    return 0;
}

static int SubBoardRtcRead(SubBoardDateTime *readback, uint32_t *hal_error)
{
    uint8_t rx[7] = {0};

    *hal_error = 0u;

    if (HAL_I2C_IsDeviceReady(&SubBoardI2c1, SUB_BOARD_PCF8563_ADDR, 3u, SUB_BOARD_I2C_TIMEOUT_MS) != HAL_OK)
    {
        *hal_error = HAL_I2C_GetError(&SubBoardI2c1);
        return -1;
    }

    if (HAL_I2C_Mem_Read(&SubBoardI2c1, SUB_BOARD_PCF8563_ADDR, 0x02u, I2C_MEMADD_SIZE_8BIT, rx, sizeof(rx), SUB_BOARD_I2C_TIMEOUT_MS) != HAL_OK)
    {
        *hal_error = HAL_I2C_GetError(&SubBoardI2c1);
        return -2;
    }

    readback->second = SubBoardFromBcd(rx[0] & 0x7Fu);
    readback->minute = SubBoardFromBcd(rx[1] & 0x7Fu);
    readback->hour = SubBoardFromBcd(rx[2] & 0x3Fu);
    readback->day = SubBoardFromBcd(rx[3] & 0x3Fu);
    readback->weekday = SubBoardFromBcd(rx[4] & 0x07u);
    readback->month = SubBoardFromBcd(rx[5] & 0x1Fu);
    readback->year = (uint16_t)(2000u + SubBoardFromBcd(rx[6]));
    return ((rx[0] & 0x80u) != 0u) ? 1 : 0;
}

static void SubBoardPrintRtcTime(const char *prefix, const SubBoardDateTime *time)
{
    SubBoardPrintf("%s%04u-%02u-%02u %02u:%02u:%02u wk%u\r\n",
                   prefix,
                   (unsigned int)time->year,
                   (unsigned int)time->month,
                   (unsigned int)time->day,
                   (unsigned int)time->hour,
                   (unsigned int)time->minute,
                   (unsigned int)time->second,
                   (unsigned int)time->weekday);
}

static bool SubBoardParseSetRtc(const char *cmd, SubBoardDateTime *out)
{
    if (strncmp(cmd, "SETRTC ", 7u) != 0)
    {
        return false;
    }

    const char *p = &cmd[7];
    if (strlen(p) != 19u ||
        p[4] != '-' || p[7] != '-' || p[10] != ' ' ||
        p[13] != ':' || p[16] != ':')
    {
        return false;
    }

    for (uint8_t i = 0u; i < 19u; i++)
    {
        if (i == 4u || i == 7u || i == 10u || i == 13u || i == 16u)
        {
            continue;
        }
        if (!SubBoardIsDigit(p[i]))
        {
            return false;
        }
    }

    out->year = (uint16_t)(((uint16_t)(p[0] - '0') * 1000u) +
                           ((uint16_t)(p[1] - '0') * 100u) +
                           ((uint16_t)(p[2] - '0') * 10u) +
                           (uint16_t)(p[3] - '0'));
    out->month = SubBoardDec2(&p[5]);
    out->day = SubBoardDec2(&p[8]);
    out->hour = SubBoardDec2(&p[11]);
    out->minute = SubBoardDec2(&p[14]);
    out->second = SubBoardDec2(&p[17]);

    if (out->year < 2000u || out->year > 2099u ||
        out->month < 1u || out->month > 12u ||
        out->day < 1u || out->day > 31u ||
        out->hour > 23u || out->minute > 59u || out->second > 59u)
    {
        return false;
    }

    out->weekday = SubBoardWeekday(out->year, out->month, out->day);
    return true;
}

static void SubBoardHandleCommand(const char *cmd)
{
    SubBoardDateTime target;
    SubBoardDateTime readback;
    uint32_t hal_error = 0u;

    SubBoardPrintf("[sub] rx %s\r\n", cmd);

    if (strcmp(cmd, "SD?") == 0)
    {
        SubBoardRunSdTest();
        return;
    }

    if (strcmp(cmd, "SDRAW?") == 0)
    {
        SubBoardRunSdRawTest();
        return;
    }

    if (strcmp(cmd, "SDRW?") == 0)
    {
        SubBoardRunSdRwTest();
        return;
    }

    if (strcmp(cmd, "SDSTRESS?") == 0)
    {
        SubBoardRunSdStressTest();
        return;
    }

    if (strcmp(cmd, "RTC?") == 0)
    {
        SubBoardRunRtcRead();
        return;
    }

    if (strcmp(cmd, "I2C?") == 0)
    {
        SubBoardRunI2cScan();
        return;
    }

    if (strcmp(cmd, "I2CSOFT?") == 0)
    {
        SubBoardI2cSoftScan();
        return;
    }

    if (strcmp(cmd, "I2CSLOW?") == 0)
    {
        SubBoardI2cSoftSlowScan();
        return;
    }

    if (strcmp(cmd, "I2CSWAP?") == 0)
    {
        SubBoardI2cSoftSwapScan();
        return;
    }

    if (strcmp(cmd, "I2CCLEAR") == 0)
    {
        SubBoardI2cBusClear();
        return;
    }

    if (strcmp(cmd, "I2C51?") == 0)
    {
        SubBoardI2cProbe51();
        return;
    }

    if (strcmp(cmd, "I2CRISE?") == 0)
    {
        SubBoardI2cRiseTest();
        return;
    }

    if (strcmp(cmd, "PINS?") == 0)
    {
        SubBoardPrintPinLevels("[sub] pins");
        return;
    }

    if (strcmp(cmd, "I2CLOW") == 0)
    {
        SubBoardI2cPinsSet(GPIO_PIN_RESET);
        return;
    }

    if (strcmp(cmd, "I2CHIGH") == 0)
    {
        SubBoardI2cPinsSet(GPIO_PIN_SET);
        return;
    }

    if (strcmp(cmd, "SCLLOW") == 0)
    {
        SubBoardI2cPinsSplit(GPIO_PIN_RESET, GPIO_PIN_SET);
        return;
    }

    if (strcmp(cmd, "SDALOW") == 0)
    {
        SubBoardI2cPinsSplit(GPIO_PIN_SET, GPIO_PIN_RESET);
        return;
    }

    if (strcmp(cmd, "I2CPULSE") == 0)
    {
        SubBoardRunI2cPulse();
        return;
    }

    if (strcmp(cmd, "SDLOW") == 0)
    {
        SubBoardSdPinsSet(GPIO_PIN_RESET);
        return;
    }

    if (strcmp(cmd, "SDHIGH") == 0)
    {
        SubBoardSdPinsSet(GPIO_PIN_SET);
        return;
    }

    if (strcmp(cmd, "SDPULSE") == 0)
    {
        SubBoardRunSdPulse();
        return;
    }

    if (SubBoardParseSetRtc(cmd, &target))
    {
        SubBoardI2c1Init();
        const int rtc_ret = SubBoardRtcWriteAndRead(&target, &readback, &hal_error);
        if (rtc_ret == 0)
        {
            SubBoardPrintRtcTime("[sub] RTC SET OK read ", &readback);
        }
        else
        {
            SubBoardPrintf("[sub] RTC SET FAIL step=%d hal=0x%08lX\r\n", rtc_ret, (unsigned long)hal_error);
        }
        return;
    }

    SubBoardPrintf("[sub] CMD FAIL use RTC? I2CSLOW? I2C51? I2CRISE? SDSTRESS?\r\n");
}

static void SubBoardPrintPinLevels(const char *prefix)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    SubBoardPrintf("%s PB8_SCL=%u PB9_SDA=%u PC10_CLK=%u PC11_MISO=%u PC12_MOSI=%u PE14_CS=%u\r\n",
                   prefix,
                   (unsigned int)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8),
                   (unsigned int)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9),
                   (unsigned int)HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10),
                   (unsigned int)HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11),
                   (unsigned int)HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12),
                   (unsigned int)HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_14));
}

static void SubBoardI2cHoldLowApply(void)
{
#if SUB_BOARD_I2C_HOLD_LOW_ON_BOOT
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    SubBoardI2c1Release();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_RESET);
#endif
}

static void SubBoardI2cPinsSet(GPIO_PinState state)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    SubBoardI2c1Release();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, state);

    SubBoardPrintf("[sub] I2C pins forced %s\r\n", (state == GPIO_PIN_SET) ? "HIGH" : "LOW");
    SubBoardPrintPinLevels("[sub] pins");
}

static void SubBoardI2cPinsSplit(GPIO_PinState scl_state, GPIO_PinState sda_state)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    SubBoardI2c1Release();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, scl_state);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, sda_state);

    SubBoardPrintf("[sub] I2C pins forced SCL=%u SDA=%u\r\n",
                   (unsigned int)scl_state,
                   (unsigned int)sda_state);
    SubBoardPrintPinLevels("[sub] pins");
}

static void SubBoardI2cSoftScan(void)
{
    SubBoardI2cSoftDelayLoop = 1200u;
    SubBoardI2cSoftScanPins(GPIO_PIN_8, GPIO_PIN_9, "soft I2C scan");
}

static void SubBoardI2cSoftSlowScan(void)
{
    SubBoardI2cSoftDelayLoop = 60000u;
    SubBoardI2cSoftScanPins(GPIO_PIN_8, GPIO_PIN_9, "slow I2C scan");
    SubBoardI2cSoftDelayLoop = 1200u;
}

static void SubBoardI2cSoftSwapScan(void)
{
    SubBoardI2cSoftDelayLoop = 1200u;
    SubBoardI2cSoftScanPins(GPIO_PIN_9, GPIO_PIN_8, "soft I2C swap scan");
}

static void SubBoardI2cSoftScanPins(uint16_t scl_pin, uint16_t sda_pin, const char *label)
{
    uint8_t found = 0u;

    SubBoardI2cSoftInit(scl_pin, sda_pin);
    SubBoardPrintf("[sub] %s:", label);
    for (uint8_t addr = 0x03u; addr <= 0x77u; addr++)
    {
        SubBoardI2cSoftStart();
        const uint8_t ack = SubBoardI2cSoftWriteByte((uint8_t)(addr << 1u));
        SubBoardI2cSoftStop();
        if (ack != 0u)
        {
            SubBoardPrintf(" 0x%02X", (unsigned int)addr);
            found++;
        }
        osDelay(1);
    }
    SubBoardPrintf(" found=%u\r\n", (unsigned int)found);
}

static void SubBoardI2cBusClear(void)
{
    SubBoardI2cSoftDelayLoop = 60000u;
    SubBoardI2cSoftInit(GPIO_PIN_8, GPIO_PIN_9);
    SubBoardI2cSoftSetSda(GPIO_PIN_SET);
    for (uint8_t i = 0u; i < 9u; i++)
    {
        SubBoardI2cSoftSetScl(GPIO_PIN_RESET);
        SubBoardI2cSoftSetScl(GPIO_PIN_SET);
    }
    SubBoardI2cSoftStop();
    SubBoardI2cSoftDelayLoop = 1200u;
    SubBoardPrintf("[sub] I2C clear done ");
    SubBoardPrintPinLevels("pins");
}

static void SubBoardI2cProbe51(void)
{
    uint8_t ack_w = 0u;
    uint8_t ack_r = 0u;

    SubBoardI2cSoftDelayLoop = 60000u;
    SubBoardI2cSoftInit(GPIO_PIN_8, GPIO_PIN_9);
    for (uint8_t i = 0u; i < 16u; i++)
    {
        SubBoardI2cSoftStart();
        ack_w = (uint8_t)(ack_w + SubBoardI2cSoftWriteByte((uint8_t)(0x51u << 1u)));
        SubBoardI2cSoftStop();

        SubBoardI2cSoftStart();
        ack_r = (uint8_t)(ack_r + SubBoardI2cSoftWriteByte((uint8_t)((0x51u << 1u) | 1u)));
        SubBoardI2cSoftStop();
        osDelay(1);
    }
    SubBoardI2cSoftDelayLoop = 1200u;

    SubBoardPrintf("[sub] I2C51 ackW=%u/16 ackR=%u/16 ", (unsigned int)ack_w, (unsigned int)ack_r);
    SubBoardPrintPinLevels("pins");
}

static void SubBoardI2cRiseTest(void)
{
    const uint32_t scl = SubBoardI2cRiseOne(GPIO_PIN_8);
    const uint32_t sda = SubBoardI2cRiseOne(GPIO_PIN_9);

    SubBoardPrintf("[sub] I2C rise loops SCL=%lu SDA=%lu ",
                   (unsigned long)scl,
                   (unsigned long)sda);
    SubBoardPrintPinLevels("pins");
}

static uint32_t SubBoardI2cRiseOne(uint16_t pin)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    uint32_t loops = 0u;

    SubBoardI2c1Release();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOB, pin, GPIO_PIN_RESET);
    osDelay(2);
    HAL_GPIO_WritePin(GPIOB, pin, GPIO_PIN_SET);

    while (HAL_GPIO_ReadPin(GPIOB, pin) == GPIO_PIN_RESET && loops < 200000u)
    {
        loops++;
    }
    return loops;
}

static void SubBoardI2cSoftInit(uint16_t scl_pin, uint16_t sda_pin)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    SubBoardI2c1Release();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    SubBoardI2cSoftSclPin = scl_pin;
    SubBoardI2cSoftSdaPin = sda_pin;

    GPIO_InitStruct.Pin = scl_pin | sda_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    SubBoardI2cSoftSetSda(GPIO_PIN_SET);
    SubBoardI2cSoftSetScl(GPIO_PIN_SET);
    SubBoardI2cSoftDelay();
}

static void SubBoardI2cSoftDelay(void)
{
    volatile uint32_t n = SubBoardI2cSoftDelayLoop;
    while (n-- > 0u)
    {
        __NOP();
    }
}

static void SubBoardI2cSoftSetScl(GPIO_PinState state)
{
    HAL_GPIO_WritePin(GPIOB, SubBoardI2cSoftSclPin, state);
    SubBoardI2cSoftDelay();
}

static void SubBoardI2cSoftSetSda(GPIO_PinState state)
{
    HAL_GPIO_WritePin(GPIOB, SubBoardI2cSoftSdaPin, state);
    SubBoardI2cSoftDelay();
}

static GPIO_PinState SubBoardI2cSoftReadSda(void)
{
    SubBoardI2cSoftDelay();
    return HAL_GPIO_ReadPin(GPIOB, SubBoardI2cSoftSdaPin);
}

static void SubBoardI2cSoftStart(void)
{
    SubBoardI2cSoftSetSda(GPIO_PIN_SET);
    SubBoardI2cSoftSetScl(GPIO_PIN_SET);
    SubBoardI2cSoftSetSda(GPIO_PIN_RESET);
    SubBoardI2cSoftSetScl(GPIO_PIN_RESET);
}

static void SubBoardI2cSoftStop(void)
{
    SubBoardI2cSoftSetSda(GPIO_PIN_RESET);
    SubBoardI2cSoftSetScl(GPIO_PIN_SET);
    SubBoardI2cSoftSetSda(GPIO_PIN_SET);
}

static uint8_t SubBoardI2cSoftWriteByte(uint8_t data)
{
    for (uint8_t bit = 0u; bit < 8u; bit++)
    {
        SubBoardI2cSoftSetSda((data & 0x80u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        SubBoardI2cSoftSetScl(GPIO_PIN_SET);
        SubBoardI2cSoftSetScl(GPIO_PIN_RESET);
        data <<= 1u;
    }

    SubBoardI2cSoftSetSda(GPIO_PIN_SET);
    SubBoardI2cSoftSetScl(GPIO_PIN_SET);
    const uint8_t ack = (SubBoardI2cSoftReadSda() == GPIO_PIN_RESET) ? 1u : 0u;
    SubBoardI2cSoftSetScl(GPIO_PIN_RESET);
    return ack;
}

static void SubBoardSdPinsSet(GPIO_PinState state)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10 | GPIO_PIN_12, state);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, state);

    SubBoardPrintf("[sub] SD pins forced %s, PC11_MISO is input pullup\r\n", (state == GPIO_PIN_SET) ? "HIGH" : "LOW");
    SubBoardPrintPinLevels("[sub] pins");
}

static void SubBoardRunI2cPulse(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    SubBoardI2c1Release();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    SubBoardPrintf("[sub] I2C pulse start, PB8/PB9 toggle every 500ms\r\n");
    for (uint8_t i = 0u; i < 12u; i++)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
        SubBoardPrintPinLevels("[sub] pins");
        osDelay(500);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
        SubBoardPrintPinLevels("[sub] pins");
        osDelay(500);
    }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_SET);
    SubBoardPrintf("[sub] I2C pulse done\r\n");
}

static void SubBoardRunSdPulse(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    SubBoardPrintf("[sub] SD pulse start, PC10/PC12/PE14 toggle every 500ms\r\n");
    for (uint8_t i = 0u; i < 12u; i++)
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10 | GPIO_PIN_12, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);
        SubBoardPrintPinLevels("[sub] pins");
        osDelay(500);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10 | GPIO_PIN_12, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET);
        SubBoardPrintPinLevels("[sub] pins");
        osDelay(500);
    }
    SubBoardPrintf("[sub] SD pulse done\r\n");
}

static void SubBoardRunSdTest(void)
{
    uint32_t sectors = 0u;
    const int init_ret = SdSpiInit();

    if (init_ret != 0)
    {
        SubBoardPrintf("[sub] SD FAIL init=%d\r\n", init_ret);
        return;
    }

    const SdSpiCardType card_type = SdSpiGetCardType();
    const int count_ret = SdSpiGetSectorCount(&sectors);
    const int read_ret = SdSpiRead(SubBoardSdSector0, 0u, 1u);

    if (count_ret == 0 && read_ret == 0)
    {
        const uint32_t mb = sectors / 2048u;
        SubBoardPrintf("[sub] SD OK type=%s sectors=%lu size=%luMB mbr=%02X%02X\r\n",
                       SubBoardSdCardTypeName(card_type),
                       (unsigned long)sectors,
                       (unsigned long)mb,
                       SubBoardSdSector0[510],
                       SubBoardSdSector0[511]);
        return;
    }

    SubBoardPrintf("[sub] SD PARTIAL type=%s count=%d read0=%d sectors=%lu\r\n",
                   SubBoardSdCardTypeName(card_type),
                   count_ret,
                   read_ret,
                   (unsigned long)sectors);
}

static uint8_t SubBoardSdRawCmd(uint8_t cmd, uint32_t arg, uint8_t crc, uint8_t *rx, uint8_t rx_len)
{
    uint8_t first = 0xFFu;
    const uint8_t packet[6] = {
        (uint8_t)(0x40u | cmd),
        (uint8_t)(arg >> 24u),
        (uint8_t)(arg >> 16u),
        (uint8_t)(arg >> 8u),
        (uint8_t)arg,
        crc,
    };

    for (uint8_t i = 0u; i < sizeof(packet); i++)
    {
        (void)SdSpiPortTxrx(packet[i]);
    }

    for (uint8_t i = 0u; i < rx_len; i++)
    {
        rx[i] = SdSpiPortTxrx(0xFFu);
        if (first == 0xFFu && (rx[i] & 0x80u) == 0u)
        {
            first = rx[i];
        }
    }
    return first;
}

static void SubBoardRunSdRawTest(void)
{
    uint8_t rx[16];

    SdSpiPortSetSpeed(SD_SPI_PORT_SPEED_INIT);
    SdSpiPortCsHigh();
    for (uint8_t i = 0u; i < 20u; i++)
    {
        (void)SdSpiPortTxrx(0xFFu);
    }

    SubBoardPrintf("[sub] SDRAW idle=%02X ", (unsigned int)SdSpiPortTxrx(0xFFu));
    SubBoardPrintPinLevels("pins");

    for (uint8_t try_id = 0u; try_id < 3u; try_id++)
    {
        memset(rx, 0xFF, sizeof(rx));
        SdSpiPortCsLow();
        (void)SdSpiPortTxrx(0xFFu);
        const uint8_t first = SubBoardSdRawCmd(0u, 0u, 0x95u, rx, sizeof(rx));
        SdSpiPortCsHigh();
        (void)SdSpiPortTxrx(0xFFu);

        SubBoardPrintf("[sub] SDRAW CMD0#%u first=%02X bytes", (unsigned int)(try_id + 1u), (unsigned int)first);
        for (uint8_t i = 0u; i < 8u; i++)
        {
            SubBoardPrintf(" %02X", (unsigned int)rx[i]);
        }
        SubBoardPrintf("\r\n");
        osDelay(2);
    }

    memset(rx, 0xFF, sizeof(rx));
    SdSpiPortCsLow();
    (void)SdSpiPortTxrx(0xFFu);
    const uint8_t first = SubBoardSdRawCmd(8u, 0x000001AAu, 0x87u, rx, sizeof(rx));
    SdSpiPortCsHigh();
    (void)SdSpiPortTxrx(0xFFu);

    SubBoardPrintf("[sub] SDRAW CMD8 first=%02X bytes", (unsigned int)first);
    for (uint8_t i = 0u; i < 8u; i++)
    {
        SubBoardPrintf(" %02X", (unsigned int)rx[i]);
    }
    SubBoardPrintf("\r\n");
}

static void SubBoardRunSdRwTest(void)
{
    static const char path[] = "0:/CODXSD.TXT";
    UINT bw = 0u;
    UINT br = 0u;
    const uint32_t tick = HAL_GetTick();

    memset(SubBoardSdWriteBuf, 0, sizeof(SubBoardSdWriteBuf));
    memset(SubBoardSdReadBuf, 0, sizeof(SubBoardSdReadBuf));

    const int len = snprintf(SubBoardSdWriteBuf,
                             sizeof(SubBoardSdWriteBuf),
                             "codex sd rw test\r\ntick=%lu\r\nmagic=0x5A%08lX\r\n",
                             (unsigned long)tick,
                             (unsigned long)(tick ^ 0x13579BDFu));
    const UINT to_write = (len > 0) ? (UINT)((len < (int)sizeof(SubBoardSdWriteBuf)) ? len : (int)sizeof(SubBoardSdWriteBuf) - 1) : 0u;

    FRESULT res = f_mount(&SubBoardSdFs, "0:", 1);
    if (res != FR_OK)
    {
        SubBoardPrintf("[sub] SDRW FAIL mount=%u\r\n", (unsigned int)res);
        return;
    }

    res = f_open(&SubBoardSdFile, path, FA_CREATE_ALWAYS | FA_WRITE);
    if (res != FR_OK)
    {
        SubBoardPrintf("[sub] SDRW FAIL openW=%u\r\n", (unsigned int)res);
        return;
    }

    res = f_write(&SubBoardSdFile, SubBoardSdWriteBuf, to_write, &bw);
    if (res == FR_OK)
    {
        res = f_sync(&SubBoardSdFile);
    }
    (void)f_close(&SubBoardSdFile);
    if (res != FR_OK || bw != to_write)
    {
        SubBoardPrintf("[sub] SDRW FAIL write=%u bw=%u/%u\r\n",
                       (unsigned int)res,
                       (unsigned int)bw,
                       (unsigned int)to_write);
        return;
    }

    res = f_open(&SubBoardSdFile, path, FA_READ);
    if (res != FR_OK)
    {
        SubBoardPrintf("[sub] SDRW FAIL openR=%u\r\n", (unsigned int)res);
        return;
    }

    res = f_read(&SubBoardSdFile, SubBoardSdReadBuf, to_write, &br);
    (void)f_close(&SubBoardSdFile);
    if (res != FR_OK || br != to_write)
    {
        SubBoardPrintf("[sub] SDRW FAIL read=%u br=%u/%u\r\n",
                       (unsigned int)res,
                       (unsigned int)br,
                       (unsigned int)to_write);
        return;
    }

    if (memcmp(SubBoardSdWriteBuf, SubBoardSdReadBuf, to_write) != 0)
    {
        SubBoardPrintf("[sub] SDRW FAIL compare len=%u\r\n", (unsigned int)to_write);
        return;
    }

    SubBoardPrintf("[sub] SDRW OK path=%s len=%u tick=%lu\r\n",
                   path,
                   (unsigned int)to_write,
                   (unsigned long)tick);
}

static void SubBoardRunSdStressTest(void)
{
    static const char path[] = "0:/CODX16K.BIN";
    static const uint32_t chunk_count = 32u;
    UINT done = 0u;
    uint32_t write_hash = 2166136261u;
    uint32_t read_hash = 2166136261u;
    const uint32_t seed = HAL_GetTick();
    const uint32_t start = HAL_GetTick();

    FRESULT res = f_mount(&SubBoardSdFs, "0:", 1);
    if (res != FR_OK)
    {
        SubBoardPrintf("[sub] SDSTRESS FAIL mount=%u\r\n", (unsigned int)res);
        return;
    }

    res = f_open(&SubBoardSdFile, path, FA_CREATE_ALWAYS | FA_WRITE);
    if (res != FR_OK)
    {
        SubBoardPrintf("[sub] SDSTRESS FAIL openW=%u\r\n", (unsigned int)res);
        return;
    }

    for (uint32_t i = 0u; i < chunk_count; i++)
    {
        SubBoardFillPattern(SubBoardSdSector0, i, seed);
        write_hash = SubBoardHashBytes(write_hash, SubBoardSdSector0, sizeof(SubBoardSdSector0));
        done = 0u;
        res = f_write(&SubBoardSdFile, SubBoardSdSector0, sizeof(SubBoardSdSector0), &done);
        if (res != FR_OK || done != sizeof(SubBoardSdSector0))
        {
            (void)f_close(&SubBoardSdFile);
            SubBoardPrintf("[sub] SDSTRESS FAIL write i=%lu res=%u bw=%u\r\n",
                           (unsigned long)i,
                           (unsigned int)res,
                           (unsigned int)done);
            return;
        }
    }
    res = f_sync(&SubBoardSdFile);
    (void)f_close(&SubBoardSdFile);
    if (res != FR_OK)
    {
        SubBoardPrintf("[sub] SDSTRESS FAIL sync=%u\r\n", (unsigned int)res);
        return;
    }

    res = f_open(&SubBoardSdFile, path, FA_READ);
    if (res != FR_OK)
    {
        SubBoardPrintf("[sub] SDSTRESS FAIL openR=%u\r\n", (unsigned int)res);
        return;
    }

    for (uint32_t i = 0u; i < chunk_count; i++)
    {
        done = 0u;
        res = f_read(&SubBoardSdFile, SubBoardSdVerifyBuf, sizeof(SubBoardSdVerifyBuf), &done);
        if (res != FR_OK || done != sizeof(SubBoardSdVerifyBuf))
        {
            (void)f_close(&SubBoardSdFile);
            SubBoardPrintf("[sub] SDSTRESS FAIL read i=%lu res=%u br=%u\r\n",
                           (unsigned long)i,
                           (unsigned int)res,
                           (unsigned int)done);
            return;
        }

        SubBoardFillPattern(SubBoardSdSector0, i, seed);
        if (memcmp(SubBoardSdSector0, SubBoardSdVerifyBuf, sizeof(SubBoardSdSector0)) != 0)
        {
            (void)f_close(&SubBoardSdFile);
            SubBoardPrintf("[sub] SDSTRESS FAIL cmp i=%lu\r\n", (unsigned long)i);
            return;
        }
        read_hash = SubBoardHashBytes(read_hash, SubBoardSdVerifyBuf, sizeof(SubBoardSdVerifyBuf));
    }
    (void)f_close(&SubBoardSdFile);

    if (read_hash != write_hash)
    {
        SubBoardPrintf("[sub] SDSTRESS FAIL hash wr=%08lX rd=%08lX\r\n",
                       (unsigned long)write_hash,
                       (unsigned long)read_hash);
        return;
    }

    SubBoardPrintf("[sub] SDSTRESS OK bytes=%lu hash=%08lX ms=%lu\r\n",
                   (unsigned long)(chunk_count * sizeof(SubBoardSdSector0)),
                   (unsigned long)read_hash,
                   (unsigned long)(HAL_GetTick() - start));
}

static void SubBoardFillPattern(uint8_t *buf, uint32_t sector, uint32_t seed)
{
    uint32_t x = seed ^ (sector * 0x9E3779B9u) ^ 0xA5A55A5Au;

    for (uint32_t i = 0u; i < 512u; i++)
    {
        x ^= x << 13u;
        x ^= x >> 17u;
        x ^= x << 5u;
        buf[i] = (uint8_t)(x >> 24u);
    }
}

static uint32_t SubBoardHashBytes(uint32_t hash, const uint8_t *buf, uint32_t len)
{
    for (uint32_t i = 0u; i < len; i++)
    {
        hash ^= buf[i];
        hash *= 16777619u;
    }
    return hash;
}

static void SubBoardRunI2cScan(void)
{
    uint8_t found = 0u;

    SubBoardI2c1Init();
    SubBoardPrintf("[sub] I2C scan:");
    for (uint8_t addr = 0x03u; addr <= 0x77u; addr++)
    {
        if (HAL_I2C_IsDeviceReady(&SubBoardI2c1, (uint16_t)addr << 1u, 1u, 10u) == HAL_OK)
        {
            SubBoardPrintf(" 0x%02X", (unsigned int)addr);
            found++;
        }
    }
    SubBoardPrintf(" found=%u\r\n", (unsigned int)found);
}

static void SubBoardRunRtcRead(void)
{
    SubBoardDateTime readback;
    uint32_t hal_error = 0u;

    SubBoardI2c1Init();
    memset(&readback, 0, sizeof(readback));

    const int rtc_ret = SubBoardRtcRead(&readback, &hal_error);
    if (rtc_ret >= 0)
    {
        SubBoardPrintRtcTime((rtc_ret == 0) ? "[sub] RTC OK read  " : "[sub] RTC VL read  ", &readback);
    }
    else
    {
        SubBoardPrintf("[sub] RTC READ FAIL step=%d hal=0x%08lX\r\n", rtc_ret, (unsigned long)hal_error);
    }
}
