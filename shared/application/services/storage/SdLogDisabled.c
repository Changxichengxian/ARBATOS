/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

/* 未选择 SDLOG 时的明确空实现。调用者不会把未写入当成成功。 */

#include "SdLog.h"
#include "SdLogTask.h"
#include <stddef.h>

#define SDLOG_DISABLED_ERROR (-95)

void SdLogTask(void const *argument)
{
    (void)argument;
}

int SdLogIsActive(void)
{
    return 0;
}

uint32_t SdLogDropped(void)
{
    return 0u;
}

int SdLogStart(void)
{
    return SDLOG_DISABLED_ERROR;
}

void SdLogStop(void)
{
}

int SdLogQuiesce(void)
{
    return SDLOG_DISABLED_ERROR;
}

uint8_t SdLogTryWrite(uint16_t tag, const void *payload, uint16_t len)
{
    (void)tag;
    (void)payload;
    (void)len;
    return 0u;
}

void SdLogWrite(uint16_t tag, const void *payload, uint16_t len)
{
    (void)tag;
    (void)payload;
    (void)len;
}

void SdLogWriteIsr(uint16_t tag, const void *payload, uint16_t len)
{
    (void)tag;
    (void)payload;
    (void)len;
}

void SdLogGetStats(SdLogStats *out)
{
    if (out != NULL)
    {
        *out = (SdLogStats){.last_error = SDLOG_DISABLED_ERROR};
    }
}

void SdLogGetPath(char *out, uint32_t capacity)
{
    if (out != NULL && capacity != 0u)
    {
        out[0] = '\0';
    }
}

uint8_t SdLogHighRateDiv(void)
{
    return 1u;
}

void SdLogPoll(void)
{
}
