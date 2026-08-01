/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>

#include <zephyr/drivers/hwinfo.h>
#include <zephyr/kernel.h>

#include "BspCrc32.h"
#include "BspDelay.h"
#include "BspDwt.h"
#include "BspFric.h"
#include "BspPwr.h"
#include "BspTime.h"

static uint8_t ArbDwtReady;
static uint32_t ArbResetCause;

void delay_init(void)
{
    BSP_DWT_Init();
}

void delay_us(uint16_t us)
{
    k_busy_wait((uint32_t)us);
}

void delay_ms(uint16_t ms)
{
    if (k_is_in_isr())
    {
        while (ms-- != 0u)
        {
            k_busy_wait(1000u);
        }
        return;
    }

    (void)k_msleep((int32_t)ms);
}

void BSP_DWT_Init(void)
{
    /*
     * Zephyr 的系统周期计数已经处理了 32 位硬件计数器回卷。这里保留旧接口，
     * 让实时统计代码不再直接读 DWT、SysTick 或 HAL 全局时钟。
     */
    (void)k_cycle_get_64();
    ArbDwtReady = 1u;
}

uint64_t BSP_DWT_GetCycles(void)
{
    if (ArbDwtReady == 0u)
    {
        BSP_DWT_Init();
    }
    return k_cycle_get_64();
}

uint64_t BSP_DWT_GetUs(void)
{
    return k_cyc_to_us_floor64(BSP_DWT_GetCycles());
}

uint8_t BSP_DWT_IsReady(void)
{
    if (ArbDwtReady == 0u)
    {
        BSP_DWT_Init();
    }
    return ArbDwtReady;
}

uint32_t BspTimeGetTickMs(void)
{
    return k_uptime_get_32();
}

uint32_t BspTimeGetTickUs(void)
{
    return (uint32_t)BSP_DWT_GetUs();
}

static uint32_t ArbStm32CrcWord(uint32_t crc, uint32_t word)
{
    crc ^= word;
    for (uint32_t bit = 0u; bit < 32u; bit++)
    {
        crc = ((crc & 0x80000000u) != 0u)
                  ? ((crc << 1u) ^ 0x04C11DB7u)
                  : (crc << 1u);
    }
    return crc;
}

uint32_t get_crc32_check_sum(uint32_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFu;

    if (data == NULL)
    {
        return crc;
    }
    for (uint32_t i = 0u; i < len; i++)
    {
        crc = ArbStm32CrcWord(crc, data[i]);
    }
    return crc;
}

bool_t verify_crc32_check_sum(uint32_t *data, uint32_t len)
{
    if (data == NULL || len == 0u)
    {
        return (bool_t)0;
    }
    return (bool_t)(get_crc32_check_sum(data, len - 1u) == data[len - 1u]);
}

void append_crc32_check_sum(uint32_t *data, uint32_t len)
{
    if (data != NULL && len != 0u)
    {
        data[len - 1u] = get_crc32_check_sum(data, len - 1u);
    }
}

void BspPwrPvdInit(void)
{
    /*
     * Zephyr 4.4 还没有 STM32 PVD 的通用设备接口。复位原因改由 hwinfo
     * 获取；低压位保持保守的“当前不可用”，由 ADC 电池监测承担运行期判断。
     */
    (void)hwinfo_get_reset_cause(&ArbResetCause);
}

uint8_t BspPwrPvdVddLow(void)
{
    return 0u;
}

uint32_t BspPwrRccCsr(void)
{
    (void)hwinfo_get_reset_cause(&ArbResetCause);
    return ArbResetCause;
}

void fric_off(void)
{
}

void fric1_on(uint16_t cmd)
{
    ARG_UNUSED(cmd);
}

void fric2_on(uint16_t cmd)
{
    ARG_UNUSED(cmd);
}
