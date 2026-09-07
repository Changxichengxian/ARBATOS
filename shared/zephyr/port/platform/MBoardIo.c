/* MC02 原理图：PC13/14/15 高电平开启，开机保持三路外供电关闭。 */
#include "BspAdc.h"
#include "SdLog.h"
#include "MBoardIo.h"
#include "RobotTargetConfig.h"
#include <errno.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>

static const struct device *const PowerGpio = DEVICE_DT_GET(DT_NODELABEL(gpioc));
static const struct i2c_dt_spec Rtc = I2C_DT_SPEC_GET(DT_ALIAS(subboard_rtc));
K_MUTEX_DEFINE(MBoardRtcLock);
volatile struct {
    int32_t powerError, adcReady, rtcError;
    uint32_t batteryMv, rawBattery, rawExternal, rtcDate, rtcTime;
} MBoardIoDiag;

static int MBoardPowerInit(void)
{
    int ret = device_is_ready(PowerGpio) ? 0 : -ENODEV;
    for (uint8_t pin = 13; pin <= 15 && ret == 0; pin++) {
        ret = gpio_pin_configure(PowerGpio, pin, GPIO_OUTPUT_LOW);
    }
    MBoardIoDiag.powerError = ret;
    return ret;
}
SYS_INIT(MBoardPowerInit, APPLICATION, 80);

int MBoardPowerSet(MBoardPower output, bool enabled)
{
    const uint8_t pins[] = {14, 13, 15};
    if ((unsigned)output >= ARRAY_SIZE(pins)) return -EINVAL;
    if (IS_ENABLED(CONFIG_ARBATOS_PREFLIGHT_ONLY) && enabled) return -EPERM;
    if (MBoardIoDiag.powerError != 0) return MBoardIoDiag.powerError;
    return gpio_pin_set(PowerGpio, pins[output], enabled);
}

static int MBoardBcd(uint8_t value)
{
    return (value & 15u) <= 9u && (value >> 4) <= 9u ?
        (value >> 4) * 10 + (value & 15u) : -1;
}

static int MBoardRtcValid(const SdLogDateTime *time)
{
    static const uint8_t days[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    if (time == NULL || time->year < 2000 || time->year > 2099 ||
        time->month < 1 || time->month > 12 || time->hour > 23 ||
        time->minute > 59 || time->second > 59) return 0;
    uint8_t limit = days[time->month - 1];
    if (time->month == 2 && time->year % 4 == 0) limit++;
    return time->day >= 1 && time->day <= limit;
}

int MBoardRtcRead(SdLogDateTime *out)
{
    if (out == NULL) return -EINVAL;
    k_mutex_lock(&MBoardRtcLock, K_FOREVER);
    uint8_t registers[9];
    const uint8_t *raw = &registers[2];
    int ret = i2c_is_ready_dt(&Rtc) ?
        i2c_burst_read_dt(&Rtc, 0, registers, sizeof(registers)) : -ENODEV;
    /* 低电压标志或时钟停止时，不能给日志提供貌似有效的时间。 */
    if (ret == 0 && ((raw[0] & 0x80u) || (registers[0] & 0x20u))) ret = -ENODATA;
    if (ret == 0) {
        int sec = MBoardBcd(raw[0] & 0x7f), min = MBoardBcd(raw[1] & 0x7f);
        int hour = MBoardBcd(raw[2] & 0x3f), day = MBoardBcd(raw[3] & 0x3f);
        int month = MBoardBcd(raw[5] & 0x1f), year = MBoardBcd(raw[6]);
        if (sec < 0 || sec > 59 || min < 0 || min > 59 || hour < 0 || hour > 23 ||
            day < 1 || day > 31 || month < 1 || month > 12 || year < 0) {
            ret = -EINVAL;
        } else {
            SdLogDateTime value = {.year = 2000 + year, .month = month, .day = day,
                                  .hour = hour, .minute = min, .second = sec};
            if (!MBoardRtcValid(&value) || (raw[5] & 0x80u) || (raw[4] & 7u) > 6u) {
                ret = -EINVAL;
            } else {
                *out = value;
                MBoardIoDiag.rtcDate = value.year * 10000u + month * 100u + day;
                MBoardIoDiag.rtcTime = hour * 10000u + min * 100u + sec;
            }
        }
    }
    if (ret != 0) MBoardIoDiag.rtcDate = MBoardIoDiag.rtcTime = 0u;
    MBoardIoDiag.rtcError = ret;
    k_mutex_unlock(&MBoardRtcLock);
    return ret;
}

int MBoardRtcSet(const SdLogDateTime *time)
{
    if (!MBoardRtcValid(time)) return -EINVAL;
    k_mutex_lock(&MBoardRtcLock, K_FOREVER);
    /* 暂停计数后一次写入七个时间寄存器，清除VL；写失败保持STOP，
     * 避免日期只更新一半却被日志误用。再次显式校时可恢复。 */
    uint8_t fields[] = {time->second, time->minute, time->hour, time->day, 0,
                        time->month, time->year % 100};
    static const uint8_t monthOffset[] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
    unsigned year = time->year - (time->month < 3);
    fields[4] = (year + year / 4 - year / 100 + year / 400 +
                 monthOffset[time->month - 1] + time->day) % 7;
    for (unsigned i = 0; i < sizeof(fields); i++) {
        fields[i] = (fields[i] / 10u) * 16u + fields[i] % 10u;
    }
    int ret = i2c_is_ready_dt(&Rtc) ? i2c_reg_write_byte_dt(&Rtc, 0, 0x20) : -ENODEV;
    if (ret == 0) ret = i2c_burst_write_dt(&Rtc, 2, fields, sizeof(fields));
    if (ret == 0) ret = i2c_reg_write_byte_dt(&Rtc, 0, 0);
    MBoardIoDiag.rtcError = ret;
    k_mutex_unlock(&MBoardRtcLock);
    if (ret == 0) {
        SdLogDateTime check;
        ret = MBoardRtcRead(&check);
    }
    return ret;
}

#if !ROBOT_SUBBOARD_RTC_SERVICE
int SdLogRtcNow(SdLogDateTime *out)
{
    return out != NULL && MBoardRtcRead(out) == 0;
}
#endif

void MBoardIoPoll(void)
{
    if (!BspAdcIsStarted()) MBoardIoDiag.adcReady = BspAdcStart();
    float volts = get_battery_voltage();
    MBoardIoDiag.batteryMv = volts > 0.0f && volts < 40.0f ? (uint32_t)(volts * 1000) : 0u;
    MBoardIoDiag.rawBattery = BspAdcGetRaw(0);
    MBoardIoDiag.rawExternal = BspAdcGetRaw(1);
    SdLogDateTime now;
    (void)MBoardRtcRead(&now);
}
