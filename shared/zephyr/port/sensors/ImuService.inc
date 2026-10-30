/* 复用 IMU 任务，只有客户端订阅时输出。手动校准锁住输出直到人工重启。 */
#include "ImuServiceProtocol.h"
#include "LowCmd.h"
#include "RobotSafety.h"
#include "SdLogTask.h"
#include <errno.h>
#include <stdio.h>
#include <zephyr/sys/atomic.h>

extern uint8_t CDC_Transmit_FS(uint8_t *buf, uint16_t len);
static atomic_t ImuWatchUntil;
static struct k_spinlock ImuCommandLock;
static ImuCommand ImuPendingCommand;
static uint32_t ImuPendingId;
static uint32_t ImuRequestId, ImuStartedMs, ImuWritingMs;
static uint8_t ImuState, ImuOutputHeld;
static uint8_t ImuModeSelected, ImuModePending, ImuModeOwned;
static int ImuError;
static GyroZeroCaliSampleState ImuSamples;
static GyroZeroCaliTempState ImuStable;

static uint8_t ImuServiceStartAllowed(void)
{
    ManualInputSnapshot input;
    if (!ManualInputSnapshotRead(&input) || RobotLifecycleFaultLatched() || !RobotSafetyOutputLocked()) return 0u;
    /* 明确离线时输出已关闭，可独立做台面校准；快照读取失败不等于离线。 */
    return input.online == 0u ||
        (input.dataValid && ControlInputSwitchIsPos(input.control.sw[INPUT_SW_GIMBAL_MODE], input.semantics.GimbalSafePos) &&
         ControlInputSwitchIsPos(input.control.sw[INPUT_SW_CHASSIS_MODE], input.semantics.ChassisSafePos));
}

void ImuServiceReceive(const uint8_t *bytes, uint32_t size)
{
    static char line[48];
    static unsigned used;
    static uint8_t overflow;
    for (uint32_t i = 0; i < size; ++i) {
        unsigned char c = bytes[i];
        if (c == '\n') {
            if (!overflow && used != 0u) {
                if (line[used - 1u] == '\r') --used;
                line[used] = '\0';
                uint32_t id;
                ImuCommand command = ImuCommandParse(line, &id);
                if (command == ImuCmdWatch) atomic_set(&ImuWatchUntil, k_uptime_get_32() + 3000u);
                else if (command == ImuCmdStop) atomic_clear(&ImuWatchUntil);
                else if (command != ImuCmdNone) {
                    k_spinlock_key_t key = k_spin_lock(&ImuCommandLock);
                    if (ImuPendingCommand == ImuCmdNone || command == ImuCmdCancel) {
                        ImuPendingCommand = command;
                        ImuPendingId = id;
                    }
                    k_spin_unlock(&ImuCommandLock, key);
                }
            }
            used = 0u;
            overflow = 0u;
        } else {
            /* 二进制主机链路不会被当作校准命令。完整行必须以 ! 开始。 */
            if (used == 0u && c != '!') overflow = 1u;
            if ((c < 32u && c != '\r') || c > 126u) overflow = 1u;
            if (!overflow && used + 1u < sizeof(line)) line[used++] = (char)c;
            else overflow = 1u;
        }
    }
}

static int ImuStorageReady(void)
{
#if ROBOT_TASK_BUILD_SDLOG
    return g_sdLogFlashPauseReady == SDLOG_FLASH_PAUSE_READY;
#else
    return 1;
#endif
}

int ImuServiceCanSave(void)
{
    return ImuState == 3u && ImuOutputHeld && LowCmdEmergencyActive() &&
           RobotSafetyOutputLocked() && !RobotLifecycleFaultLatched() &&
           k_uptime_get_32() - ImuStartedMs >= 1000u && ImuStorageReady();
}

static void ImuServiceFail(int error)
{
    ImuError = error;
    ImuState = 5u;
}

static void ImuServiceModeUpdate(uint8_t selected)
{
    selected = selected != 0u;
    if (selected && !ImuModeSelected) ImuModePending = 1u;
    if (!selected && ImuModeSelected) {
        ImuModePending = 0u;
        if (ImuModeOwned && ImuState >= 1u && ImuState <= 3u) {
            ImuState = 6u;
            ImuError = 0;
        }
        ImuModeOwned = 0u;
    }
    ImuModeSelected = selected;
}

static void ImuServiceUpdate(const fp32 raw[3], const fp32 accel[3], uint32_t now, uint8_t healthy)
{
    uint32_t id;
    k_spinlock_key_t key = k_spin_lock(&ImuCommandLock);
    ImuCommand command = ImuPendingCommand;
    id = ImuPendingId;
    ImuPendingCommand = ImuCmdNone;
    k_spin_unlock(&ImuCommandLock, key);
    uint8_t modeStart = ImuModePending;
    ImuModePending = 0u;
    if (command == ImuCmdNone && modeStart && !(ImuState >= 1u && ImuState <= 3u)) {
        command = ImuCmdCalibrate;
    } else {
        modeStart = 0u;
    }
    if (command != ImuCmdNone && (modeStart || id != ImuRequestId)) {
        if (command == ImuCmdCancel) {
            ImuRequestId = id;
            ImuModeOwned = 0u;
            ImuState = 6u;
            ImuError = 0;
        } else if (command == ImuCmdCalibrate && !(ImuState >= 1u && ImuState <= 3u)) {
            if (!modeStart) ImuRequestId = id;
            ImuModeOwned = modeStart;
            ImuError = 0;
            if (!healthy || !ImuServiceStartAllowed()) {
                ImuServiceFail(-EPERM);
            } else if (!RobotLifecycleEnterMaintenance()) {
                ImuServiceFail(-EBUSY);
            } else {
                ImuOutputHeld = 1u;
                if (!LowCmdEnterEmergencyStop(LOWCMD_WRITER_MAINTENANCE)) { ImuServiceFail(-EBUSY); return; }
                ImuStartedMs = now;
                ImuState = 1u;
                GyroZeroCaliSampleReset(&ImuSamples);
                GyroZeroCaliTempReset(&ImuStable);
            }
        }
    }
    if (ImuState < 1u || ImuState > 3u) return;
    if (!healthy) { ImuServiceFail(-EIO); return; }
    if (now - ImuStartedMs > 600000u) { ImuServiceFail(-ETIMEDOUT); return; }
    if (ImuState == 3u) {
#if ROBOT_TASK_BUILD_SDLOG
        if (g_sdLogFlashPauseReady == SDLOG_FLASH_PAUSE_FAILED) { ImuServiceFail(-EIO); return; }
#endif
        if (now - ImuWritingMs > 5000u) { ImuServiceFail(-ETIMEDOUT); return; }
        /* 给停机帧和写入前的状态回报留出时间，Flash 擦除期间硬件 PWM 已关闭。 */
        if (now - ImuWritingMs < 500u || !ImuServiceCanSave()) return;
        float offset[3];
        GyroZeroCaliCalcOffset(&ImuSamples, offset);
        int result = ImuCalStoreSave(offset, InsTemp);
        if (result != 0) ImuServiceFail(result);
        else {
            ImuApplyGyroOffset(offset);
            ImuCalibrationSampleDone(&ImuSamples, NULL);
            InsGyroCaliState.boot_adjust_finished = 1u;
            InsGyroCaliState.calibrated = 1u;
            InsGyroCaliState.calibrating = 0u;
            InsGyroCaliState.result = GYRO_ZERO_CALI_RESULT_SUCCESS;
            ImuState = 4u;
        }
        return;
    }
    if (!GyroZeroCaliTempStableUpdate(&ImuStable, InsTemp, get_control_temperature(), now, InsHeaterStable)) {
        ImuState = 1u;
        GyroZeroCaliSampleReset(&ImuSamples);
        return;
    }
    ImuState = 2u;
    float rotated[3], acceleration[3];
    ImuFrameRotate(rotated, raw);
    ImuFrameRotate(acceleration, accel);
    if (GyroZeroCaliCollectSample(&ImuSamples, rotated, acceleration, GYRO_ZERO_CALI_TEST_SAMPLES)) {
        ImuState = 3u;
        ImuWritingMs = now;
#if ROBOT_TASK_BUILD_SDLOG
        g_sdLogFlashPauseRequest = SDLOG_FLASH_PAUSE_REQUEST;
#endif
    }
}

static void ImuServicePublish(uint32_t now, uint8_t healthy)
{
    static uint32_t last;
    uint32_t until = (uint32_t)atomic_get(&ImuWatchUntil);
    if (until == 0u || (int32_t)(until - now) <= 0 || now - last < 100u) return;
    last = now;
    int map[3] = {0};
    for (int j = 0; j < 3; ++j) {
        float raw[3] = {0}, body[3];
        raw[j] = 1.0f;
        ImuFrameRotate(body, raw);
        for (int i = 0; i < 3; ++i) {
            if (body[i] > 0.5f) map[i] = j + 1;
            if (body[i] < -0.5f) map[i] = -j - 1;
        }
    }
    unsigned flags = (healthy ? 1u : 0u) |
        ((healthy && ImuServiceStartAllowed()) ? 2u : 0u) |
        (ImuOutputHeld ? 4u : 0u) | (ImuCalStoreLoaded() ? 8u : 0u) |
        (InsGyroCaliState.boot_adjust_finished ? 16u : 0u);
    const float degrees = 57.295779513f;
    char line[384];
    int length = snprintf(line, sizeof(line),
        "\n@IMU2,%s,%u,%u,%u,%d,%u,%u,%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
        ARBATOS_TARGET_NAME, now, ImuState, ImuRequestId, ImuError, ImuSamples.sample_count,
        ImuCalStoreSequence(), flags, (int)(InsTemp * 1000), get_control_temperature() * 1000,
        (int)(INS_angle[1] * degrees * 1000), (int)(INS_angle[2] * degrees * 1000),
        (int)(INS_angle[0] * degrees * 1000),
        (int)(InsGyro[0] * degrees * 1000), (int)(InsGyro[1] * degrees * 1000), (int)(InsGyro[2] * degrees * 1000),
        (int)(InsAccel[0] * 1000), (int)(InsAccel[1] * 1000), (int)(InsAccel[2] * 1000),
        (int)(InsGyroOffset[0] * degrees * 1000000), (int)(InsGyroOffset[1] * degrees * 1000000),
        (int)(InsGyroOffset[2] * degrees * 1000000), map[0], map[1], map[2]);
    if (length > 0 && (size_t)length < sizeof(line)) (void)CDC_Transmit_FS((uint8_t *)line, (uint16_t)length);
}
