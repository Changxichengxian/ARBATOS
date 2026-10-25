/* 真实测试固件的主机回归：模拟 CAN 响应延迟/丢响应，不连接任何设备。 */
#include <assert.h>
#include <stdio.h>
#define main BenchFirmwareMain
#include "src/main.c"
#undef main

uint32_t TestNow;
struct device TestDevice;
static uint32_t ReplyAt[6];
static uint8_t MotorState[6];
static uint32_t DropEnable, DropAll, EnableCalls, RejectSend, FaultEnable;
static uint32_t MovingCalls, EnabledId, DropMoving, FastMoving, ShiftMoving, StopAt;
static uint16_t MotorSpeed[6], MotorPosition[6];

void TestAdvance(uint32_t us)
{
    TestNow += us;
    if (StopAt != 0u && TestNow >= StopAt) CanBenchStop = 1u;
    for (uint32_t i = 0u; i < 6u; i++) {
        if (ReplyAt[i] != 0u && (int32_t)(TestNow - ReplyAt[i]) >= 0) {
            struct can_frame frame = {.id = i + 1u, .dlc = 8u};
            frame.data[0] = (uint8_t)((MotorState[i] << 4) | (i + 1u));
            frame.data[1] = (uint8_t)(MotorPosition[i] >> 8);
            frame.data[2] = (uint8_t)MotorPosition[i];
            frame.data[3] = (uint8_t)(MotorSpeed[i] >> 4);
            frame.data[4] = (uint8_t)(MotorSpeed[i] << 4);
            BenchReceive(&TestDevice, &frame, NULL);
            ReplyAt[i] = 0u;
        }
    }
}

int can_send(const struct device *dev, const struct can_frame *frame, int timeout, can_tx_callback_t cb, void *arg)
{
    (void)timeout;
    assert(frame->id >= 1u && frame->id <= 6u && frame->dlc == 8u);
    if (RejectSend) return -EAGAIN;
    uint8_t command = frame->data[7];
    if (command == 0xfcu) {
        EnableCalls++;
        EnabledId = frame->id;
        MotorState[frame->id - 1u] = FaultEnable ? 8u : 1u;
    } else if (command == 0xfdu) {
        MotorState[frame->id - 1u] = 0u;
    } else {
        static const uint8_t neutral[8] = {0x80, 0x00, 0x80, 0x00, 0x00, 0x00, 0x08, 0x00};
        if (memcmp(frame->data, neutral, 8u) != 0) {
            assert(frame->id == 5u || frame->id == 6u);
            uint32_t velocity = ((uint32_t)frame->data[2] << 4) | (frame->data[3] >> 4);
            uint32_t kp = ((uint32_t)(frame->data[3] & 15u) << 8) | frame->data[4];
            uint32_t kd = ((uint32_t)frame->data[5] << 4) | (frame->data[6] >> 4);
            uint32_t torque = ((uint32_t)(frame->data[6] & 15u) << 8) | frame->data[7];
            assert(kp == 0u && kd == 25u && torque == 2048u);
            assert(velocity == (frame->id == 6u ? 2053u : 2042u));
            MovingCalls++;
            MotorSpeed[frame->id - 1u] = FastMoving ? 2100u : (uint16_t)velocity;
            if (ShiftMoving) MotorPosition[frame->id - 1u] += 3000u;
            if (DropMoving) {
                cb(dev, 0, arg);
                return 0;
            }
        }
    }
    cb(dev, 0, arg);
    if (DropAll || (command == 0xfcu && EnableCalls <= DropEnable)) return 0;
    ReplyAt[frame->id - 1u] = TestNow + 600u;
    return 0;
}

static void TestReset(void)
{
    memset((void *)&CanBenchDiag, 0, sizeof(CanBenchDiag));
    CanBenchDiag.magic = BENCH_MAGIC;
    CanBenchStop = 0u;
    CanBenchCommand = BenchReportRequested = 0u;
    TestNow = 1000u;
    memset(ReplyAt, 0, sizeof(ReplyAt));
    memset(MotorState, 0, sizeof(MotorState));
    DropEnable = DropAll = EnableCalls = RejectSend = FaultEnable = 0u;
    MovingCalls = EnabledId = DropMoving = FastMoving = ShiftMoving = StopAt = 0u;
    for (uint32_t i = 0u; i < 6u; i++) {
        MotorSpeed[i] = 2048u;
        MotorPosition[i] = 32768u;
    }
}

int main(void)
{
    TestReset();
    BenchSerialCommand("CB1 transitions");
    assert(CanBenchCommand == 0u);
    CanBenchDiag.ready = 1u;
    BenchSerialCommand("transitions");
    assert(CanBenchCommand == 0u);
    BenchSerialCommand("CB1 transitions");
    assert(CanBenchCommand == 2u);
    CanBenchDiag.busy = 2u;
    CanBenchCommand = 0u;
    BenchSerialCommand("CB1 load");
    assert(CanBenchCommand == 0u);
    BenchSerialCommand("CB1 stop");
    assert(CanBenchStop == 1u);
    BenchSerialCommand("CB1 status");
    assert(BenchReportRequested == 1u);

    TestReset();
    BenchRun(1u);
    assert(CanBenchDiag.txAttempts == 0u && CanBenchDiag.result == 1u);

    TestReset();
    BenchTrial trial = {0};
    assert(BenchTransition(1u, 0xfcu, &trial, false) == 1u);
    assert(trial.attempts == 1u && trial.completed == 1u && trial.elapsedUs == 600u);

    TestReset();
    DropEnable = 2u;
    trial = (BenchTrial){0};
    assert(BenchTransition(2u, 0xfcu, &trial, false) == 1u);
    assert(trial.attempts == 3u && trial.elapsedUs == 10600u);

    TestReset();
    DropAll = 1u;
    CanBenchDiag.motors[0] = (BenchMotor){.count = 50u, .cycle = 1u, .state = 1u};
    trial = (BenchTrial){0};
    assert(BenchTransition(1u, 0xfcu, &trial, false) == 0u);
    assert(trial.attempts == 20u && trial.outcome == 2u && trial.elapsedUs == 100000u);

    TestReset();
    RejectSend = 1u;
    trial = (BenchTrial){0};
    assert(BenchTransition(1u, 0xfcu, &trial, false) == 0u);
    assert(trial.accepted == 0u && trial.txErrors == 20u);

    TestReset();
    BenchRun(2u);
    assert(CanBenchDiag.result == 1u && CanBenchDiag.trialCount == 120u && EnableCalls == 60u);
    for (uint32_t i = 0u; i < 6u; i++) assert(MotorState[i] == 0u);

    TestReset();
    FaultEnable = 1u;
    BenchRun(2u);
    assert(CanBenchDiag.result == 2u && EnableCalls == 1u && CanBenchDiag.trialCount == 2u);
    for (uint32_t i = 0u; i < 6u; i++) assert(MotorState[i] == 0u);

    TestReset();
    DropAll = 1u;
    BenchRun(2u);
    assert(CanBenchDiag.result == 3u && EnableCalls == 0u);

    TestReset();
    CanBenchStop = 1u;
    BenchRun(2u);
    assert(CanBenchDiag.result == 2u && EnableCalls == 0u);

    TestReset();
    BenchRun(3u);
    assert(CanBenchDiag.result == 1u && EnableCalls == 0u && CanBenchDiag.windowCount == 5u);
    TestReset();
    BenchRun(4u);
    assert(CanBenchDiag.result == 1u && EnableCalls == 0u);
    for (uint32_t i = 0u; i < 6u; i++) assert(CanBenchDiag.motors[i].count == 1u);
    for (uint32_t command = 5u; command <= 6u; command++) {
        TestReset();
        BenchRun(command);
        assert(CanBenchDiag.result == 1u && EnableCalls == 1u && EnabledId == (command == 5u ? 6u : 5u));
        assert(MovingCalls == 200u && TestNow < 1100000u);
        for (uint32_t i = 0u; i < 6u; i++) assert(MotorState[i] == 0u);
    }
    TestReset();
    assert(BenchWheelVelocity(1u, true) == -EINVAL);
    BenchJogWheel(2u);
    assert(EnableCalls == 0u && MovingCalls == 0u);
    for (uint32_t failure = 0u; failure < 6u; failure++) {
        TestReset();
        DropMoving = failure == 0u;
        FastMoving = failure == 1u;
        ShiftMoving = failure == 2u;
        StopAt = failure == 3u ? 50000u : 0u;
        DropAll = failure == 4u;
        CanBenchStop = failure == 5u;
        BenchRun(5u);
        assert(CanBenchDiag.result != 1u && MovingCalls < 20u);
        for (uint32_t i = 0u; i < 6u; i++) assert(MotorState[i] == 0u);
    }
    puts("PASS: passive, latency, retries, stale feedback, rejected sends, 120 transitions, fault cleanup, failed precheck, load sweep");
    return 0;
}
