/* 上车前静态检查。邮箱只提供日志读取及显式启停限功率加热。 */
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "InsTask.h"
#include "SdCard.h"
#include "SdLog.h"
#include "MBoardIo.h"
#include "fatfs/ff.h"

LOG_MODULE_REGISTER(preflight, LOG_LEVEL_INF);

#if defined(CONFIG_ARBATOS_PREFLIGHT_AWAKE)
bool z_arm_on_enter_cpu_idle(void)
{
    /* 无线 DAP 在 H723 休眠后无法稳定重连；仅静态检查版保持唤醒。 */
    return false;
}
#endif

typedef struct
{
    uint32_t magic;
    uint32_t tickMs;
    int32_t mountResult;
    int32_t logResult;
    uint32_t imuValid;
    uint32_t imuSequence;
    uint32_t imuAgeMs;
    uint32_t calibrated;
    uint32_t calibrating;
    uint32_t heater;
    float temperature;
    float gyro[3];
    float accel[3];
    float angle[3];
    SdLogStats log;
} MPreflightStatus;

volatile MPreflightStatus MPreflightDiag = {.magic = 0x4d505246u};
volatile uint32_t MPreflightCommand;
volatile int32_t MPreflightResult;
volatile uint32_t MPreflightOffset;
volatile uint32_t MPreflightBytes;
volatile uint32_t MPreflightHeatEnable;
volatile uint32_t MPreflightRtcDate;
volatile uint32_t MPreflightRtcTime;
char MPreflightPath[384];
char MPreflightLogPath[96];
uint8_t MPreflightData[4096];
static struct k_thread MPreflightThread;
static struct k_thread MPreflightImuThread;
K_THREAD_STACK_DEFINE(MPreflightStack, 6144);
K_THREAD_STACK_DEFINE(MPreflightImuStack, 8192);
static uint8_t MPreflightLogEnabled;

static int MPreflightStopLog(void)
{
    MPreflightLogEnabled = 0u;
    /* 本模式由当前线程独占写卡；先拒绝其他生产者，再排空并关闭。 */
    return SdLogQuiesce();
}

#if defined(CONFIG_ARBATOS_BENCH_TELEMETRY)
static void MPreflightSerialPoll(void)
{
    static char line[144];
    static uint32_t used;
    static uint8_t overflow;
    const struct device *uart = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    unsigned char value;

    /* 只允许停止日志和小块读卡；不提供电机、加热或任意内存写入命令。 */
    for (unsigned n = 0; n < 64u && uart_poll_in(uart, &value) == 0; n++) {
        if (value != '\n' && value != '\r') {
            if (used + 1u < sizeof(line)) {
                line[used++] = (char)value;
            } else {
                overflow = 1u;
            }
            continue;
        }
        line[used] = '\0';
        if (overflow == 0u && strcmp(line, "SDS") == 0) {
            int result = MPreflightStopLog();
            FILINFO info = {0};
            FRESULT statResult = f_stat(MPreflightLogPath, &info);
            LOG_INF("@SDS,%d,%u,%lu,%s", result, (unsigned)statResult,
                    (unsigned long)info.fsize, MPreflightLogPath);
        } else if (overflow == 0u && strncmp(line, "SDR ", 4u) == 0) {
            unsigned int offset, length;
            char path[96];
            char extra;
            int fields = sscanf(line, "SDR %u %u %95s %c", &offset, &length, path, &extra);
            if ((fields == 2 || fields == 3) &&
                length > 0u && length <= 256u && MPreflightLogEnabled == 0u) {
                FIL file;
                UINT count = 0u;
                uint8_t data[256];
                FRESULT fr = f_open(&file, fields == 3 ? path : MPreflightLogPath, FA_READ);
                if (fr == FR_OK) {
                    fr = f_lseek(&file, offset);
                    if (fr == FR_OK) {
                        fr = f_read(&file, data, length, &count);
                    }
                    (void)f_close(&file);
                }
                LOG_INF("@SDR,%u,%u,%u,%u", offset, length, (unsigned)fr, count);
                if (fr == FR_OK) {
                    const char digits[] = "0123456789abcdef";
                    for (UINT i = 0u; i < count; i += 32u) {
                        char hex[65];
                        UINT bytes = MIN(32u, count - i);
                        for (UINT j = 0u; j < bytes; j++) {
                            hex[2u * j] = digits[data[i + j] >> 4];
                            hex[2u * j + 1u] = digits[data[i + j] & 15u];
                        }
                        hex[2u * bytes] = '\0';
                        LOG_INF("@SDB,%u,%s", offset + i, hex);
                    }
                }
            } else {
                LOG_INF("@SDR,invalid");
            }
        }
        used = 0u;
        overflow = 0u;
    }
}
#endif

static void MPreflightImu(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
    ImuFusionTask(NULL);
}

static void MPreflightRequest(void)
{
    uint32_t command = MPreflightCommand;
    if (command == 0u) {
        return;
    }
    MPreflightResult = 0;
    MPreflightBytes = 0u;
    MPreflightPath[sizeof(MPreflightPath) - 1u] = '\0';
    if (command == 1u) {
        MPreflightResult = MPreflightStopLog();
    } else if (command == 2u) {
        DIR dir;
        FILINFO info;
        FRESULT fr = f_opendir(&dir, MPreflightPath);
        if (fr == FR_OK) {
            uint32_t used = 0u;
            while ((fr = f_readdir(&dir, &info)) == FR_OK && info.fname[0] != '\0') {
                int n = snprintk((char *)&MPreflightData[used], sizeof(MPreflightData) - used,
                                 "%c %lu %s\n", (info.fattrib & AM_DIR) ? 'D' : 'F',
                                 (unsigned long)info.fsize, info.fname);
                if (n < 0 || (uint32_t)n >= sizeof(MPreflightData) - used) {
                    MPreflightResult = -ENOSPC;
                    break;
                }
                used += (uint32_t)n;
            }
            MPreflightBytes = used;
            (void)f_closedir(&dir);
        }
        if (fr != FR_OK) {
            MPreflightResult = -(int)fr;
        }
    } else if (command == 3u) {
        FIL file;
        UINT count = 0u;
        FRESULT fr = f_open(&file, MPreflightPath, FA_READ);
        if (fr == FR_OK) {
            fr = f_lseek(&file, MPreflightOffset);
            if (fr == FR_OK) {
                fr = f_read(&file, MPreflightData, sizeof(MPreflightData), &count);
            }
            (void)f_close(&file);
        }
        MPreflightBytes = count;
        MPreflightResult = -(int)fr;
    } else if (command == 4u || command == 5u) {
        MPreflightHeatEnable = command == 4u;
    } else if (command == 6u) {
        uint32_t date = MPreflightRtcDate, time = MPreflightRtcTime;
        SdLogDateTime value = {.year = date / 10000u, .month = date / 100u % 100u,
                              .day = date % 100u, .hour = time / 10000u,
                              .minute = time / 100u % 100u, .second = time % 100u};
        MPreflightResult = MBoardRtcSet(&value);
    } else {
        MPreflightResult = -EINVAL;
    }
    /* 数据先写完，再清请求；主机等待该标志后才读取邮箱。 */
    __sync_synchronize();
    MPreflightCommand = 0u;
}

static void MPreflightRun(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
    int ret = SdcardMount();
    MPreflightDiag.mountResult = ret;
    if (ret == 0) {
        ret = SdLogStart();
        MPreflightLogEnabled = ret == 0;
        SdLogGetPath(MPreflightLogPath, sizeof(MPreflightLogPath));
    }
    MPreflightDiag.logResult = ret;
    LOG_INF("SD mount=%d log=%d path=%s", MPreflightDiag.mountResult, ret, MPreflightLogPath);
    uint32_t nextReport = 0u;
    for (;;) {
        MPreflightRequest();
#if defined(CONFIG_ARBATOS_BENCH_TELEMETRY)
        MPreflightSerialPoll();
#endif
        InsSnapshot snapshot;
        MPreflightDiag.tickMs = k_uptime_get_32();
        MPreflightDiag.imuValid = InsSnapshotRead(&snapshot);
        if (MPreflightDiag.imuValid != 0u) {
            MPreflightDiag.imuSequence = snapshot.seq;
            MPreflightDiag.imuAgeMs = snapshot.age_ms;
            MPreflightDiag.temperature = snapshot.temperature_c;
            for (uint32_t i = 0u; i < 3u; i++) {
                MPreflightDiag.gyro[i] = snapshot.gyro[i];
                MPreflightDiag.accel[i] = snapshot.accel[i];
                MPreflightDiag.angle[i] = snapshot.angle[i];
            }
            if (MPreflightLogEnabled != 0u && snapshot.age_ms < 50u) {
                sdlog_imu_t sample;
                memcpy(sample.quat, snapshot.quat, sizeof(sample.quat));
                memcpy(sample.gyro, snapshot.gyro, sizeof(sample.gyro));
                memcpy(sample.accel, snapshot.accel, sizeof(sample.accel));
                sample.temp = snapshot.temperature_c;
                SdLogWrite(SDLOG_TAG_IMU, &sample, sizeof(sample));
            }
        }
        MPreflightDiag.calibrated = ins_is_gyro_boot_calibrated();
        MPreflightDiag.calibrating = ins_is_gyro_boot_calibrating();
        MPreflightDiag.heater = ins_get_imu_heater_pwm();
        SdLogPoll();
        SdLogStats stats;
        SdLogGetStats(&stats);
        MPreflightDiag.log = stats;
        if ((int32_t)(MPreflightDiag.tickMs - nextReport) >= 0) {
            extern void MBoardIoPoll(void);
            MBoardIoPoll();
            nextReport = MPreflightDiag.tickMs + 2000u;
            LOG_INF("IMU seq=%u age=%u temp_centi=%d cal=%u heater=%u; SD bytes=%u drop=%u err=%d",
                    MPreflightDiag.imuSequence, MPreflightDiag.imuAgeMs,
                    (int)(MPreflightDiag.temperature * 100.0f), MPreflightDiag.calibrated,
                    MPreflightDiag.heater, stats.bytes_flushed, stats.dropped, stats.last_error);
        }
        k_msleep(10);
    }
}

int MPreflightStart(void)
{
#if defined(CONFIG_ARBATOS_RECEIVE_ONLY)
    extern int MReceiveStart(void);
    if (MReceiveStart() != 0) {
        return -EIO;
    }
#endif
    k_thread_create(&MPreflightImuThread, MPreflightImuStack, K_THREAD_STACK_SIZEOF(MPreflightImuStack),
                    MPreflightImu, NULL, NULL, NULL, K_PRIO_PREEMPT(5), 0, K_NO_WAIT);
    k_thread_create(&MPreflightThread, MPreflightStack, K_THREAD_STACK_SIZEOF(MPreflightStack),
                    MPreflightRun, NULL, NULL, NULL, K_PRIO_PREEMPT(12), 0, K_NO_WAIT);
    return 0;
}
