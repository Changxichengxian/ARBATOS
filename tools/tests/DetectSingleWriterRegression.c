/* Detect 单写者主机回归：接收只记事实，任务推进状态，查询只读已发布快照。 */

#include <stdio.h>
#include <string.h>

#include "DetectCommon.h"

static uint8_t s_dataBad;
static uint32_t s_dataCheckCount;
static uint32_t s_dataSolveCount;
static uint32_t s_lostSolveCount;

static bool_t TestDataIsError(void)
{
    s_dataCheckCount++;
    return s_dataBad;
}

static void TestSolveDataError(void)
{
    s_dataSolveCount++;
}

static void TestSolveLost(void)
{
    s_lostSolveCount++;
}

static int TestCheck(int condition, const char *message)
{
    if (!condition)
    {
        fprintf(stderr, "FAIL: %s\n", message);
        return 0;
    }
    return 1;
}

static void TestPublish(DetectRuntime *runtime, uint32_t nowMs)
{
    uint8_t nextIndex = (uint8_t)(runtime->activeIndex ^ 1u);
    uint32_t nextSeq = runtime->publishSeq + 1u;

    if (nextSeq == 0u)
    {
        nextSeq = 1u;
    }
    DetectCommonPublish(&runtime->snapshot[nextIndex],
                        runtime->working,
                        (uint8_t)DETECT_ERROR_COUNT,
                        nowMs,
                        nextSeq);
    runtime->publishSeq = nextSeq;
    runtime->activeIndex = nextIndex;
}

static void TestAdvance(DetectRuntime *runtime, uint32_t nowMs)
{
    for (uint8_t toe = 0u; toe < (uint8_t)DETECT_ERROR_COUNT; toe++)
    {
        DetectReceiptFact fact;
        DetectCommonTakeFact(runtime->receipt,
                             &fact,
                             (uint8_t)DETECT_ERROR_COUNT,
                             toe);
        DetectCommonRefreshOne(&runtime->working[toe], &fact, nowMs);
    }
    TestPublish(runtime, nowMs);
}

static DetectConfig TestConfig(void)
{
    DetectConfig config;

    memset(&config, 0, sizeof(config));
    config.enable_mask = 1u << DBUS_TOE;
    config.items[DBUS_TOE].offline_time_ms = 100u;
    config.items[DBUS_TOE].online_time_ms = 20u;
    config.items[DBUS_TOE].priority = 3u;
    return config;
}

static int TestQueriesArePure(void)
{
    DetectRuntime runtime;
    DetectSnapshot before;
    DetectSnapshot readback;
    DetectSummary summary;
    DetectConfig config = TestConfig();

    memset(&runtime, 0, sizeof(runtime));
    memset(&readback, 0xA5, sizeof(readback));
    memset(&summary, 0xA5, sizeof(summary));
    if (!TestCheck(DetectCommonSnapshotIsError(&runtime.snapshot[runtime.activeIndex],
                                               (uint8_t)DETECT_ERROR_COUNT,
                                               DBUS_TOE) != 0u,
                   "未发布时查询必须保守报错") ||
        !TestCheck(DetectCommonSnapshotRead(&runtime.snapshot[runtime.activeIndex], &readback) == 0u &&
                       readback.valid == 0u,
                   "未发布时读取必须返回无效快照") ||
        !TestCheck(DetectCommonSummaryRead(&runtime.snapshot[runtime.activeIndex], &summary) == 0u &&
                       summary.valid == 0u,
                   "未发布时读取必须返回无效汇总"))
    {
        return 0;
    }

    DetectCommonInitFromConfig(runtime.working,
                               (uint8_t)DETECT_ERROR_COUNT,
                               &config,
                               0u);
    DetectCommonSeedBaseline(runtime.receipt, (uint8_t)DETECT_ERROR_COUNT, 0u);
    runtime.working[DBUS_TOE].data_is_error_fun = TestDataIsError;
    runtime.working[DBUS_TOE].solve_data_error_fun = TestSolveDataError;
    runtime.working[DBUS_TOE].solve_lost_fun = TestSolveLost;
    runtime.writerInitialized = 1u;
    TestPublish(&runtime, 0u);

    if (!TestCheck(DetectCommonSummaryRead(&runtime.snapshot[runtime.activeIndex], &summary) != 0u &&
                       summary.errorMask == (uint16_t)(1u << DBUS_TOE) &&
                       summary.lostMask == (uint16_t)(1u << DBUS_TOE) &&
                       summary.dataErrorMask == (uint16_t)(1u << DBUS_TOE) &&
                       sizeof(summary) < sizeof(readback),
                   "初始汇总必须以小对象发布三类错误位图"))
    {
        return 0;
    }

    before = runtime.snapshot[runtime.activeIndex];
    DetectCommonHook(runtime.receipt, (uint8_t)DETECT_ERROR_COUNT, DBUS_TOE, 10u);
    if (!TestCheck(s_dataCheckCount == 0u && runtime.working[DBUS_TOE].is_lost != 0u,
                   "DetectHook 只能记录接收事实") ||
        !TestCheck(DetectCommonSnapshotIsError(&runtime.snapshot[runtime.activeIndex],
                                               (uint8_t)DETECT_ERROR_COUNT,
                                               DBUS_TOE) != 0u,
                   "任务推进前查询必须保持旧代状态") ||
        !TestCheck(memcmp(&before, &runtime.snapshot[runtime.activeIndex], sizeof(before)) == 0,
                   "查询和 Hook 都不能改已发布快照"))
    {
        return 0;
    }

    TestAdvance(&runtime, 10u);
    if (!TestCheck(runtime.snapshot[runtime.activeIndex].seq == 2u &&
                       runtime.snapshot[runtime.activeIndex].state[DBUS_TOE].isLost == 0u &&
                       runtime.snapshot[runtime.activeIndex].state[DBUS_TOE].errorExist != 0u &&
                       runtime.snapshot[runtime.activeIndex].errorMask == (uint16_t)(1u << DBUS_TOE) &&
                       runtime.snapshot[runtime.activeIndex].lostMask == 0u &&
                       runtime.snapshot[runtime.activeIndex].dataErrorMask == 0u,
                   "首次任务推进应恢复链路并保留上线稳定期"))
    {
        return 0;
    }

    TestAdvance(&runtime, 31u);
    if (!TestCheck(runtime.snapshot[runtime.activeIndex].seq == 3u &&
                       runtime.snapshot[runtime.activeIndex].errorMask == 0u &&
                       DetectCommonSnapshotIsError(&runtime.snapshot[runtime.activeIndex],
                                                   (uint8_t)DETECT_ERROR_COUNT,
                                                   DBUS_TOE) == 0u,
                   "稳定期结束后应发布健康状态"))
    {
        return 0;
    }

    s_dataBad = 1u;
    DetectCommonHook(runtime.receipt, (uint8_t)DETECT_ERROR_COUNT, DBUS_TOE, 40u);
    if (!TestCheck(s_dataCheckCount == 1u && s_dataSolveCount == 0u,
                   "数据检查和处理只能由任务推进调用"))
    {
        return 0;
    }
    TestAdvance(&runtime, 40u);
    if (!TestCheck(s_dataCheckCount == 2u && s_dataSolveCount == 1u &&
                       runtime.snapshot[runtime.activeIndex].state[DBUS_TOE].dataIsError != 0u &&
                       runtime.snapshot[runtime.activeIndex].state[DBUS_TOE].frequencyHz > 33.0f &&
                       runtime.snapshot[runtime.activeIndex].state[DBUS_TOE].frequencyHz < 34.0f,
                   "任务应在同一批事实上计算数据错误和频率"))
    {
        return 0;
    }

    before = runtime.snapshot[runtime.activeIndex];
    (void)DetectCommonSnapshotIsError(&runtime.snapshot[runtime.activeIndex],
                                      (uint8_t)DETECT_ERROR_COUNT,
                                      DBUS_TOE);
    (void)DetectCommonSnapshotRead(&runtime.snapshot[runtime.activeIndex], &readback);
    (void)DetectCommonSummaryRead(&runtime.snapshot[runtime.activeIndex], &summary);
    if (!TestCheck(s_dataCheckCount == 2u && s_dataSolveCount == 1u &&
                       memcmp(&before, &runtime.snapshot[runtime.activeIndex], sizeof(before)) == 0,
                   "所有查询都必须是无副作用读取"))
    {
        return 0;
    }

    s_dataBad = 0u;
    DetectCommonHook(runtime.receipt, (uint8_t)DETECT_ERROR_COUNT, DBUS_TOE, 60u);
    TestAdvance(&runtime, 60u);
    TestAdvance(&runtime, 161u);
    before = runtime.snapshot[runtime.activeIndex];
    (void)DetectCommonSnapshotIsError(&runtime.snapshot[runtime.activeIndex],
                                      (uint8_t)DETECT_ERROR_COUNT,
                                      DBUS_TOE);
    if (!TestCheck(before.state[DBUS_TOE].isLost != 0u &&
                       before.state[DBUS_TOE].lostTimeMs == 161u &&
                       before.errorMask == (uint16_t)(1u << DBUS_TOE) &&
                       before.lostMask == (uint16_t)(1u << DBUS_TOE) &&
                       before.dataErrorMask == 0u &&
                       s_lostSolveCount == 1u,
                   "超时和丢失处理只能在任务推进时发生") ||
        !TestCheck(memcmp(&before, &runtime.snapshot[runtime.activeIndex], sizeof(before)) == 0 &&
                       s_lostSolveCount == 1u,
                   "丢失查询不得改时间戳或重复处理"))
    {
        return 0;
    }

    return TestCheck(DetectCommonSnapshotIsError(&runtime.snapshot[runtime.activeIndex],
                                                 (uint8_t)DETECT_ERROR_COUNT,
                                                 (uint8_t)DETECT_ERROR_COUNT) != 0u,
                     "越界查询必须保守报错");
}

static int TestEarlyHookSurvivesWriterInit(void)
{
    DetectRuntime runtime;
    DetectConfig config = TestConfig();

    memset(&runtime, 0, sizeof(runtime));
    DetectCommonHook(runtime.receipt, (uint8_t)DETECT_ERROR_COUNT, DBUS_TOE, 5u);
    DetectCommonInitFromConfig(runtime.working,
                               (uint8_t)DETECT_ERROR_COUNT,
                               &config,
                               10u);
    DetectCommonSeedBaseline(runtime.receipt, (uint8_t)DETECT_ERROR_COUNT, 10u);

    if (!TestCheck(runtime.receipt[DBUS_TOE].received != 0u &&
                       runtime.receipt[DBUS_TOE].latestTickMs == 5u &&
                       runtime.receipt[DBUS_TOE].baselineValid == 0u,
                   "任务初始化不得覆盖早到的接收事实"))
    {
        return 0;
    }

    TestAdvance(&runtime, 10u);
    return TestCheck(runtime.working[DBUS_TOE].new_time == 5u &&
                         runtime.working[DBUS_TOE].is_lost == 0u,
                     "早到事实必须由首次任务推进消费");
}

static int TestTakeFactKeepsOtherToePending(void)
{
    DetectRuntime runtime;
    DetectReceiptFact dbusFact;
    DetectReceiptFact chassisFact;

    memset(&runtime, 0, sizeof(runtime));
    memset(&dbusFact, 0, sizeof(dbusFact));
    memset(&chassisFact, 0, sizeof(chassisFact));
    DetectCommonHook(runtime.receipt, (uint8_t)DETECT_ERROR_COUNT, DBUS_TOE, 7u);
    DetectCommonHook(runtime.receipt, (uint8_t)DETECT_ERROR_COUNT, CHASSIS_MOTOR1_TOE, 8u);
    DetectCommonTakeFact(runtime.receipt,
                         &dbusFact,
                         (uint8_t)DETECT_ERROR_COUNT,
                         DBUS_TOE);

    if (!TestCheck(runtime.receipt[DBUS_TOE].pending == 0u &&
                       dbusFact.pending != 0u &&
                       runtime.receipt[CHASSIS_MOTOR1_TOE].pending != 0u &&
                       chassisFact.pending == 0u,
                   "逐项取事实不得扩大临界区或清掉其他 TOE"))
    {
        return 0;
    }

    DetectCommonTakeFact(runtime.receipt,
                         &chassisFact,
                         (uint8_t)DETECT_ERROR_COUNT,
                         CHASSIS_MOTOR1_TOE);
    return TestCheck(runtime.receipt[CHASSIS_MOTOR1_TOE].pending == 0u &&
                         chassisFact.latestTickMs == 8u,
                     "第二个 TOE 应由自己的短临界区取走");
}

static int TestWriterCadenceBoundsTimeoutDelay(void)
{
    DetectRuntime runtime;
    DetectConfig config = TestConfig();

    memset(&runtime, 0, sizeof(runtime));
    DetectCommonInitFromConfig(runtime.working,
                               (uint8_t)DETECT_ERROR_COUNT,
                               &config,
                               0u);
    DetectCommonHook(runtime.receipt, (uint8_t)DETECT_ERROR_COUNT, DBUS_TOE, 10u);
    TestAdvance(&runtime, 10u);
    TestAdvance(&runtime, 110u);
    if (!TestCheck(runtime.working[DBUS_TOE].is_lost == 0u,
                   "离线阈值边界不能提前报错"))
    {
        return 0;
    }

    TestAdvance(&runtime, 110u + DETECT_COMMON_RUNTIME_POLL_MS);
    return TestCheck(DETECT_COMMON_RUNTIME_POLL_MS <= DETECT_CONTROL_TIME &&
                         runtime.working[DBUS_TOE].is_lost != 0u,
                     "单写者必须在一个 10 ms 控制周期内发布超时");
}

int main(void)
{
    if (!TestQueriesArePure() ||
        !TestEarlyHookSurvivesWriterInit() ||
        !TestTakeFactKeepsOtherToePending() ||
        !TestWriterCadenceBoundsTimeoutDelay())
    {
        return 1;
    }

    puts("Detect single-writer regression passed.");
    return 0;
}
