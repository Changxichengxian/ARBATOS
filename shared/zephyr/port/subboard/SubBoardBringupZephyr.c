/* Zephyr replacement for SENTINEL-M SubBoardBringup.c. */
#include "SubBoardBringup.h"
#include "SubBoardBringupZephyr.h"
#include "SubBoardBringupZephyrConfig.h"

#include <errno.h>

#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>

#include "SdLog.h"

typedef struct
{
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint8_t voltage_low;
} SubBoardDateTime;

#ifdef ARB_SUBBOARD_RTC_NODE
static const struct i2c_dt_spec SubBoardRtc = I2C_DT_SPEC_GET(ARB_SUBBOARD_RTC_NODE);
#endif

static SubBoardBringupDiag SubBoardDiag;
static uint32_t SubBoardNextRetryMs;

static uint8_t SubBoardFromBcd(uint8_t value)
{
    return (uint8_t)(((value >> 4u) * 10u) + (value & 0x0Fu));
}

static int SubBoardRtcRead(SubBoardDateTime *out)
{
    uint8_t reg = 0x02u;
    uint8_t rx[7];
    int ret;

    if (out == NULL)
    {
        return -EINVAL;
    }
#ifdef ARB_SUBBOARD_RTC_NODE
    if (!i2c_is_ready_dt(&SubBoardRtc))
    {
        return -ENODEV;
    }
    ret = i2c_write_read_dt(&SubBoardRtc, &reg, sizeof(reg), rx, sizeof(rx));
    if (ret != 0)
    {
        return ret;
    }
    out->voltage_low = ((rx[0] & 0x80u) != 0u) ? 1u : 0u;
    out->second = SubBoardFromBcd((uint8_t)(rx[0] & 0x7Fu));
    out->minute = SubBoardFromBcd((uint8_t)(rx[1] & 0x7Fu));
    out->hour = SubBoardFromBcd((uint8_t)(rx[2] & 0x3Fu));
    out->day = SubBoardFromBcd((uint8_t)(rx[3] & 0x3Fu));
    out->month = SubBoardFromBcd((uint8_t)(rx[5] & 0x1Fu));
    out->year = (uint16_t)(2000u + SubBoardFromBcd(rx[6]));
    return (out->voltage_low != 0u) ? -ENODATA : 0;
#else
    ARG_UNUSED(reg); ARG_UNUSED(rx); ARG_UNUSED(ret);
    return -ENODEV;
#endif
}

static void SubBoardProbe(void)
{
    SubBoardDateTime now;
    const int ret = SubBoardRtcRead(&now);

    SubBoardDiag.probe_count++;
    SubBoardDiag.last_error = ret;
    SubBoardDiag.rtc_ready = (ret == 0) ? 1u : 0u;
    SubBoardDiag.rtc_voltage_low = (ret == -ENODATA) ? 1u : 0u;
    if (ret != 0)
    {
        SubBoardDiag.read_error_count++;
    }
}

void SubBoardBringupRunOnce(void)
{
    SubBoardProbe();
    SubBoardNextRetryMs = k_uptime_get_32() + ARB_SUBBOARD_RETRY_MS;
}

void SubBoardBringupPoll(void)
{
    const uint32_t now = k_uptime_get_32();

    if (SubBoardDiag.rtc_ready != 0u || (int32_t)(now - SubBoardNextRetryMs) < 0)
    {
        return;
    }
    SubBoardProbe();
    SubBoardNextRetryMs = now + ARB_SUBBOARD_RETRY_MS;
}

int SdLogRtcNow(SdLogDateTime *out)
{
    SubBoardDateTime now;
    const int ret = SubBoardRtcRead(&now);

    if (out == NULL)
    {
        return 0;
    }
    if (ret != 0)
    {
        SubBoardDiag.last_error = ret;
        SubBoardDiag.rtc_ready = 0u;
        SubBoardDiag.rtc_voltage_low = (ret == -ENODATA) ? 1u : 0u;
        SubBoardDiag.read_error_count++;
        return 0;
    }
    out->year = now.year;
    out->month = now.month;
    out->day = now.day;
    out->hour = now.hour;
    out->minute = now.minute;
    out->second = now.second;
    SubBoardDiag.last_error = 0;
    SubBoardDiag.rtc_ready = 1u;
    SubBoardDiag.rtc_voltage_low = 0u;
    return 1;
}

void SubBoardBringupGetDiag(SubBoardBringupDiag *out)
{
    if (out != NULL)
    {
        *out = SubBoardDiag;
    }
}
