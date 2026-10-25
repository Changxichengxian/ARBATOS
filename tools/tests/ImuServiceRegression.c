/*
 * IMU 服务接收与状态机主机回归。通过轻量替身直接包含生产 ImuService.inc，
 * 覆盖串口分包、溢出恢复、重复请求、取消和 watch 租期。
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

typedef float fp32;
typedef unsigned int k_spinlock_key_t;
struct k_spinlock { unsigned unused; };

typedef struct {
    uint32_t sample_count;
    fp32 gyro_sum[3];
} GyroZeroCaliSampleState;

typedef struct {
    uint32_t stable_since_ms;
} GyroZeroCaliTempState;

typedef struct {
    uint8_t boot_adjust_finished;
    uint8_t calibrated;
    uint8_t calibrating;
    uint8_t result;
} TestGyroRuntimeState;

#define INPUT_SW_GIMBAL_MODE 0u
#define INPUT_SW_CHASSIS_MODE 1u
typedef struct {
    uint8_t GimbalSafePos;
    uint8_t ChassisSafePos;
} TestInputSemantics;
typedef struct {
    uint16_t sw[2];
} TestInputControl;
typedef struct {
    uint8_t online;
    uint8_t dataValid;
    TestInputControl control;
    TestInputSemantics semantics;
} ManualInputSnapshot;

#define ROBOT_TASK_BUILD_SDLOG 1
#define GYRO_ZERO_CALI_TEST_SAMPLES 30000u
#define GYRO_ZERO_CALI_RESULT_SUCCESS 1u
#define ARBATOS_TARGET_NAME "TEST"

static uint32_t TestNow;
static uint8_t TestEmergency;
static uint16_t TestEmergencyWriter;
static uint8_t TestOutputLocked = 1u;
static uint8_t TestInputReadOk = 1u;
static uint8_t TestLifecycleFault;
static ManualInputSnapshot TestInput = {
    .online = 0u,
    .dataValid = 1u,
    .semantics = {.GimbalSafePos = 1u, .ChassisSafePos = 1u},
};
static unsigned TestTxCount;
static char TestLastTx[512];
static unsigned TestSaveCalls;
static int TestSaveResult;
static unsigned TestApplyCalls;

volatile uint32_t g_sdLogFlashPauseRequest;
volatile uint32_t g_sdLogFlashPauseReady;
static fp32 InsTemp = 40.0f;
static uint8_t InsHeaterStable = 1u;
static fp32 INS_angle[3];
static fp32 InsGyro[3];
static fp32 InsAccel[3];
static fp32 InsGyroOffset[3];
static TestGyroRuntimeState InsGyroCaliState;

static uint32_t k_uptime_get_32(void) { return TestNow; }
static k_spinlock_key_t k_spin_lock(struct k_spinlock *lock) { (void)lock; return 0u; }
static void k_spin_unlock(struct k_spinlock *lock, k_spinlock_key_t key)
{
    (void)lock;
    (void)key;
}

uint8_t LowCmdEnterEmergencyStop(uint16_t writer)
{
    TestEmergency = 1u;
    TestEmergencyWriter = writer;
    return 1u;
}

uint8_t LowCmdEmergencyActive(void) { return TestEmergency; }
uint8_t RobotSafetyOutputLocked(void) { return TestOutputLocked; }
static uint8_t ManualInputSnapshotRead(ManualInputSnapshot *out)
{
    if (!TestInputReadOk || out == NULL) return 0u;
    *out = TestInput;
    return 1u;
}
static uint8_t ControlInputSwitchIsPos(uint16_t raw, uint8_t pos) { return raw == pos; }
static uint8_t RobotLifecycleFaultLatched(void) { return TestLifecycleFault; }
static uint8_t RobotLifecycleEnterMaintenance(void)
{
    if (TestLifecycleFault || !TestOutputLocked) return 0u;
    TestOutputLocked = 1u;
    return 1u;
}

static void GyroZeroCaliSampleReset(GyroZeroCaliSampleState *state)
{
    (void)memset(state, 0, sizeof(*state));
}

static void GyroZeroCaliTempReset(GyroZeroCaliTempState *state)
{
    (void)memset(state, 0, sizeof(*state));
}

static uint8_t GyroZeroCaliTempStableUpdate(GyroZeroCaliTempState *state,
                                            fp32 temp,
                                            fp32 target,
                                            uint32_t now,
                                            uint8_t heaterStable)
{
    (void)state;
    (void)temp;
    (void)target;
    (void)now;
    (void)heaterStable;
    return 0u;
}

static uint8_t GyroZeroCaliCollectSample(GyroZeroCaliSampleState *state,
                                         const fp32 gyro[3],
                                         const fp32 accel[3],
                                         uint32_t target)
{
    (void)state;
    (void)gyro;
    (void)accel;
    (void)target;
    return 0u;
}

static void GyroZeroCaliCalcOffset(const GyroZeroCaliSampleState *state, fp32 offset[3])
{
    (void)state;
    offset[0] = 0.0f;
    offset[1] = 0.0f;
    offset[2] = 0.0f;
}

static int get_control_temperature(void) { return 40; }
static void ImuFrameRotate(fp32 out[3], const fp32 raw[3])
{
    out[0] = raw[0];
    out[1] = raw[1];
    out[2] = raw[2];
}
static int ImuCalStoreSave(const fp32 offset[3], fp32 temperature)
{
    (void)offset;
    (void)temperature;
    TestSaveCalls++;
    return TestSaveResult;
}
static uint32_t ImuCalStoreSequence(void) { return 0u; }
static int ImuCalStoreLoaded(void) { return 0; }
static void ImuApplyGyroOffset(const fp32 offset[3])
{
    (void)offset;
    TestApplyCalls++;
}

uint8_t CDC_Transmit_FS(uint8_t *buf, uint16_t len)
{
    size_t copy = len;
    if (copy >= sizeof(TestLastTx)) copy = sizeof(TestLastTx) - 1u;
    (void)memcpy(TestLastTx, buf, copy);
    TestLastTx[copy] = '\0';
    TestTxCount++;
    return 0u;
}

#include "ImuService.inc"

static int TestCheck(int condition, const char *message)
{
    if (condition != 0) return 1;
    (void)fprintf(stderr, "FAIL: %s\n", message);
    return 0;
}

static void TestFeed(const void *bytes, size_t size)
{
    ImuServiceReceive((const uint8_t *)bytes, (uint32_t)size);
}

static void TestFeedText(const char *text)
{
    TestFeed(text, strlen(text));
}

static void TestResetReceiver(void)
{
    TestFeedText("\n");
    ImuPendingCommand = ImuCmdNone;
    ImuPendingId = 0u;
}

static int TestSplitAndQueuePolicy(void)
{
    TestResetReceiver();
    TestFeedText("!IMU2,cal");
    if (!TestCheck(ImuPendingCommand == ImuCmdNone, "半包命令不应提前生效")) return 0;
    TestFeedText("ibrate,11\n");
    if (!TestCheck(ImuPendingCommand == ImuCmdCalibrate && ImuPendingId == 11u,
                   "跨包校准命令没有正确组行")) return 0;

    TestFeedText("!IMU2,calibrate,12\n");
    if (!TestCheck(ImuPendingCommand == ImuCmdCalibrate && ImuPendingId == 11u,
                   "重复校准不应覆盖尚未消费的首个请求")) return 0;
    TestFeedText("!IMU2,cancel,13\n");
    return TestCheck(ImuPendingCommand == ImuCmdCancel && ImuPendingId == 13u,
                     "取消必须能覆盖尚未消费的校准请求");
}

static int TestOverflowAndBinaryRecovery(void)
{
    char longLine[96];
    TestResetReceiver();
    (void)memset(longLine, 'A', sizeof(longLine));
    longLine[0] = '!';
    longLine[sizeof(longLine) - 1u] = '\n';
    TestFeed(longLine, sizeof(longLine));
    if (!TestCheck(ImuPendingCommand == ImuCmdNone, "溢出行不应产生命令")) return 0;
    TestFeedText("!IMU2,calibrate,21\n");
    if (!TestCheck(ImuPendingCommand == ImuCmdCalibrate && ImuPendingId == 21u,
                   "溢出行结束后没有恢复组行")) return 0;

    TestResetReceiver();
    TestFeedText("binary-prefix!IMU2,calibrate,22\n");
    return TestCheck(ImuPendingCommand == ImuCmdNone,
                     "二进制或普通文本行中的命令片段不应被执行");
}

static int TestControlCharactersRejected(void)
{
    static const uint8_t nulLine[] = {
        '!', 'I', 'M', 'U', '2', ',', 'c', 'a', 'l', 'i', 'b', 'r', 'a', 't', 'e', ',', '3', '1',
        0u, 'x', '\n'
    };
    static const uint8_t crLine[] = "!IMU2,calib\rrate,32\n";

    TestResetReceiver();
    TestFeed(nulLine, sizeof(nulLine));
    if (!TestCheck(ImuPendingCommand == ImuCmdNone,
                   "带行内 NUL 的命令必须整行拒绝")) return 0;
    TestFeed(crLine, sizeof(crLine) - 1u);
    return TestCheck(ImuPendingCommand == ImuCmdNone,
                     "行内 CR 不得把两个片段拼成有效命令");
}

static int TestDuplicateAndCancel(void)
{
    const fp32 zero[3] = {0.0f, 0.0f, 0.0f};
    TestResetReceiver();
    ImuRequestId = 0u;
    ImuState = 0u;
    ImuOutputHeld = 0u;
    TestEmergency = 0u;
    TestEmergencyWriter = 0u;
    TestInputReadOk = 1u;
    TestInput.online = 0u;
    TestInput.dataValid = 1u;
    TestLifecycleFault = 0u;
    TestOutputLocked = 1u;

    TestFeedText("!IMU2,calibrate,41\n");
    ImuServiceUpdate(zero, zero, 100u, 1u);
    if (!TestCheck(ImuState == 1u && ImuRequestId == 41u && ImuOutputHeld != 0u &&
                       TestEmergency != 0u && TestEmergencyWriter == LOWCMD_WRITER_MAINTENANCE,
                   "合法校准请求没有进入维护锁定状态")) return 0;

    TestFeedText("!IMU2,calibrate,41\n");
    ImuServiceUpdate(zero, zero, 102u, 1u);
    if (!TestCheck(ImuState == 1u && ImuRequestId == 41u,
                   "重复请求编号不应重新开始校准")) return 0;

    TestFeedText("!IMU2,cancel,42\n");
    ImuServiceUpdate(zero, zero, 104u, 1u);
    return TestCheck(ImuState == 6u && ImuRequestId == 42u && ImuOutputHeld != 0u &&
                         TestEmergency != 0u,
                     "取消后应终止采样并继续保持输出锁到重启");
}

static int TestStartRequiresReadableSafeInput(void)
{
    const fp32 zero[3] = {0.0f, 0.0f, 0.0f};

    TestResetReceiver();
    ImuRequestId = 0u;
    ImuState = 0u;
    ImuOutputHeld = 0u;
    TestEmergency = 0u;
    TestLifecycleFault = 0u;
    TestOutputLocked = 1u;
    TestInputReadOk = 0u;
    TestFeedText("!IMU2,calibrate,61\n");
    ImuServiceUpdate(zero, zero, 100u, 1u);
    if (!TestCheck(ImuState == 5u && ImuRequestId == 61u && ImuOutputHeld == 0u && TestEmergency == 0u,
                   "输入快照读取失败时不得进入维护校准")) return 0;

    ImuState = 0u;
    TestInputReadOk = 1u;
    TestInput.online = 1u;
    TestInput.dataValid = 0u;
    TestInput.control.sw[INPUT_SW_GIMBAL_MODE] = TestInput.semantics.GimbalSafePos;
    TestInput.control.sw[INPUT_SW_CHASSIS_MODE] = TestInput.semantics.ChassisSafePos;
    TestFeedText("!IMU2,calibrate,62\n");
    ImuServiceUpdate(zero, zero, 102u, 1u);
    if (!TestCheck(ImuState == 5u && ImuOutputHeld == 0u && TestEmergency == 0u,
                   "在线但数据无效时不得进入维护校准")) return 0;

    ImuState = 0u;
    TestInput.dataValid = 1u;
    TestInput.control.sw[INPUT_SW_CHASSIS_MODE] = 0u;
    TestFeedText("!IMU2,calibrate,63\n");
    ImuServiceUpdate(zero, zero, 104u, 1u);
    if (!TestCheck(ImuState == 5u && ImuOutputHeld == 0u && TestEmergency == 0u,
                   "在线且任一拨杆不在安全档时不得进入维护校准")) return 0;

    ImuState = 0u;
    TestInput.control.sw[INPUT_SW_CHASSIS_MODE] = TestInput.semantics.ChassisSafePos;
    TestFeedText("!IMU2,calibrate,64\n");
    ImuServiceUpdate(zero, zero, 106u, 1u);
    return TestCheck(ImuState == 1u && ImuRequestId == 64u && ImuOutputHeld != 0u && TestEmergency != 0u,
                     "在线有效且两拨杆均在安全档时应进入维护校准");
}

static int TestWatchLease(void)
{
    TestResetReceiver();
    TestTxCount = 0u;
    TestNow = 1000u;
    TestFeedText("!IMU2,watch,51\n");
    ImuServicePublish(TestNow, 1u);
    if (!TestCheck(TestTxCount == 1u && strstr(TestLastTx, "@IMU2,TEST,") != NULL,
                   "watch 后没有发布 IMU2 遥测")) return 0;
    ImuServicePublish(1050u, 1u);
    if (!TestCheck(TestTxCount == 1u, "watch 输出超过 10 Hz")) return 0;
    ImuServicePublish(1100u, 1u);
    if (!TestCheck(TestTxCount == 2u, "watch 输出未达到 10 Hz 节拍")) return 0;
    ImuServicePublish(4000u, 1u);
    if (!TestCheck(TestTxCount == 2u, "3 秒租期边界之后仍在输出")) return 0;

    TestNow = 5000u;
    TestFeedText("!IMU2,watch,52\n");
    TestFeedText("!IMU2,stop,53\n");
    ImuServicePublish(5100u, 1u);
    return TestCheck(TestTxCount == 2u, "stop 后仍在输出遥测");
}

static void TestPrepareWriting(uint32_t now, uint32_t writingMs)
{
    TestResetReceiver();
    TestNow = now;
    ImuState = 3u;
    ImuRequestId = 70u;
    ImuStartedMs = 0u;
    ImuWritingMs = writingMs;
    ImuOutputHeld = 1u;
    ImuError = 0;
    TestEmergency = 1u;
    TestOutputLocked = 1u;
    TestLifecycleFault = 0u;
    TestSaveCalls = 0u;
    TestSaveResult = 0;
    TestApplyCalls = 0u;
    g_sdLogFlashPauseReady = 0u;
    (void)memset(&InsGyroCaliState, 0, sizeof(InsGyroCaliState));
}

static int TestWritingGatesAndResult(void)
{
    const fp32 zero[3] = {0.0f, 0.0f, 0.0f};

    TestPrepareWriting(1600u, 1000u);
    ImuServiceUpdate(zero, zero, TestNow, 1u);
    if (!TestCheck(TestSaveCalls == 0u && ImuState == 3u,
                   "SD 未确认暂停时不得写入校准 Flash")) return 0;

    TestPrepareWriting(1499u, 1000u);
    g_sdLogFlashPauseReady = SDLOG_FLASH_PAUSE_READY;
    ImuServiceUpdate(zero, zero, TestNow, 1u);
    if (!TestCheck(TestSaveCalls == 0u && ImuState == 3u,
                   "进入写入状态不足 500 ms 时不得写入校准 Flash")) return 0;

    TestPrepareWriting(1600u, 1000u);
    g_sdLogFlashPauseReady = SDLOG_FLASH_PAUSE_READY;
    TestEmergency = 0u;
    ImuServiceUpdate(zero, zero, TestNow, 1u);
    if (!TestCheck(TestSaveCalls == 0u && ImuState == 3u,
                   "维护急停证据丢失时不得写入校准 Flash")) return 0;

    TestPrepareWriting(1600u, 1000u);
    g_sdLogFlashPauseReady = SDLOG_FLASH_PAUSE_READY;
    ImuServiceUpdate(zero, zero, TestNow, 1u);
    if (!TestCheck(TestSaveCalls == 1u && TestApplyCalls == 1u && ImuState == 4u &&
                       InsGyroCaliState.result == GYRO_ZERO_CALI_RESULT_SUCCESS &&
                       InsGyroCaliState.calibrated != 0u && InsGyroCaliState.boot_adjust_finished != 0u,
                   "写入门槛全部满足后应保存、应用并发布成功状态")) return 0;

    TestPrepareWriting(1600u, 1000u);
    g_sdLogFlashPauseReady = SDLOG_FLASH_PAUSE_READY;
    TestSaveResult = -EIO;
    ImuServiceUpdate(zero, zero, TestNow, 1u);
    if (!TestCheck(TestSaveCalls == 1u && TestApplyCalls == 0u && ImuState == 5u && ImuError == -EIO,
                   "Flash 保存失败时不得应用偏置，且必须发布失败状态")) return 0;

    TestPrepareWriting(1600u, 1000u);
    g_sdLogFlashPauseReady = SDLOG_FLASH_PAUSE_READY;
    TestFeedText("!IMU2,cancel,71\n");
    ImuServiceUpdate(zero, zero, TestNow, 1u);
    return TestCheck(TestSaveCalls == 0u && TestApplyCalls == 0u && ImuState == 6u && ImuRequestId == 71u,
                     "写入前收到取消命令时必须先取消且不得调用 Flash 保存");
}

int main(void)
{
    if (!TestSplitAndQueuePolicy()) return 1;
    if (!TestOverflowAndBinaryRecovery()) return 1;
    if (!TestControlCharactersRejected()) return 1;
    if (!TestDuplicateAndCancel()) return 1;
    if (!TestStartRequiresReadableSafeInput()) return 1;
    if (!TestWatchLease()) return 1;
    if (!TestWritingGatesAndResult()) return 1;
    (void)puts("PASS: IMU service receive and state regression");
    return 0;
}
