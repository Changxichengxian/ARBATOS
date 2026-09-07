/* 上车前静态检查。邮箱只提供日志读取及显式启停限功率加热。 */
#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "InsTask.h"
#include "SdCard.h"
#include "SdLog.h"
#include "MBoardIo.h"
#include "fatfs/ff.h"

LOG_MODULE_REGISTER(preflight, LOG_LEVEL_INF);

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
        MPreflightLogEnabled = 0u;
        SdLogStats stats;
        for (uint32_t i = 0u; i < 1500u; i++) {
            SdLogPoll();
            SdLogGetStats(&stats);
            if (stats.ring_used == 0u) {
                break;
            }
            k_msleep(1);
        }
        MPreflightResult = stats.ring_used == 0u ? 0 : -ETIMEDOUT;
        SdLogStop();
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
