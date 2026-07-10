/* 真实 ManualInput/ControlInput 主机回归：覆盖统一快照发布、超时和并发边界。 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "ControlInput.h"
#include "ManualInputSnapshot.h"
#include "RobotConfig.h"
#include "ManualInput.h"
#include "timers.h"

Config g_config;

static TickType_t s_tick;
static TimerCallbackFunction_t s_timerCallback;
static int s_criticalDepth;
static uint32_t s_criticalErrorCount;
static uint32_t s_callbackOrderErrorCount;
static uint32_t s_watchCount;
static uint32_t s_detectHookCount;
static uint32_t s_logWriteCount;
static uint32_t s_bspInitCount;
static uint8_t s_boardKeyDown;
static uint8_t s_sdLogActive;
static uint8_t s_toeError = 1u;
static uint8_t s_injectPending;
static uint8_t s_injectSource;
static ManualInputState s_injectState;
static uint8_t s_watchInjectPending;
static uint8_t s_watchInjectSource;
static ManualInputState s_watchInjectState;
static uint32_t s_event[16];
static uint8_t s_eventCount;

#include "ControlInput.c"
#include "ManualInput.c"

static int TestCheck(int condition, const char *message)
{
    if (!condition)
    {
        fprintf(stderr, "FAIL: %s\n", message);
        return 0;
    }
    return 1;
}

static ManualInputState TestInput(int16_t ch0,
                                  int16_t ch1,
                                  uint8_t sw0,
                                  uint8_t sw1,
                                  uint16_t key)
{
    ManualInputState input;
    memset(&input, 0, sizeof(input));
    input.rc.ch[0] = ch0;
    input.rc.ch[1] = ch1;
    input.rc.s[0] = (char)sw0;
    input.rc.s[1] = (char)sw1;
    input.key.v = key;
    return input;
}

static void TestReset(uint16_t timeout_ms, uint16_t board_key_mask)
{
    memset(&g_config, 0, sizeof(g_config));
    g_config.manual_input.active_source = MANUAL_INPUT_SRC_AUTO;
    g_config.manual_input.mix_mode = MANUAL_INPUT_MIX_SELECT_LATEST;
    g_config.manual_input.source_timeout_ms = timeout_ms;
    g_config.manual_input.BoardKeyKeyMask = board_key_mask;
    for (uint32_t i = 0u; i < (uint32_t)INPUT_AXIS_COUNT; i++)
    {
        g_config.input.axis[i].rc_ch = (uint8_t)(i % 5u);
        g_config.input.axis[i].invert = 0u;
    }
    for (uint32_t i = 0u; i < (uint32_t)INPUT_SW_COUNT; i++)
    {
        g_config.input.sw[i].rc_sw = (uint8_t)(i % 2u);
        g_config.input.sw[i].invert = 0u;
    }

    s_tick = 0u;
    s_timerCallback = NULL;
    s_criticalDepth = 0;
    s_criticalErrorCount = 0u;
    s_callbackOrderErrorCount = 0u;
    s_watchCount = 0u;
    s_detectHookCount = 0u;
    s_logWriteCount = 0u;
    s_bspInitCount = 0u;
    s_boardKeyDown = 0u;
    s_sdLogActive = 0u;
    s_toeError = 1u;
    s_injectPending = 0u;
    s_injectSource = 0u;
    memset(&s_injectState, 0, sizeof(s_injectState));
    s_watchInjectPending = 0u;
    s_watchInjectSource = 0u;
    memset(&s_watchInjectState, 0, sizeof(s_watchInjectState));
    memset(s_event, 0, sizeof(s_event));
    s_eventCount = 0u;
    ManualInputInit();
}

static void TestFireTimer(void)
{
    if (s_timerCallback != NULL)
    {
        s_timerCallback((TimerHandle_t)(uintptr_t)1u);
    }
}

static int TestNoSourceSafe(void)
{
    ManualInputSnapshot snapshot;
    ManualInputState manual;
    ControlInputState control;

    TestReset(0u, 0u);
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u,
                   "初始快照必须可读")) return 0;
    if (!TestCheck(snapshot.activeSource == MANUAL_INPUT_SRC_AUTO &&
                   snapshot.activeMask == 0u && snapshot.online == 0u,
                   "无来源必须 AUTO、空掩码且离线")) return 0;
    if (!TestCheck(snapshot.sourceTickMs == 0u &&
                   snapshot.sourceAgeMs == UINT32_MAX &&
                   snapshot.sourceTimeoutMs == MANUAL_INPUT_DEFAULT_TIMEOUT_MS,
                   "无来源时间字段和零超时默认值必须安全")) return 0;
    if (!TestCheck(snapshot.publishSeq != 0u && snapshot.sourceSeq == 0u &&
                   snapshot.switchSeq == 0u,
                   "初始发布序号有效且尚未发生真实切源")) return 0;
    if (!TestCheck(snapshot.manual.rc.s[0] == RC_SW_UP &&
                   snapshot.manual.rc.s[1] == RC_SW_UP,
                   "无来源原始开关必须是安全上档")) return 0;
    for (uint32_t i = 0u; i < (uint32_t)INPUT_AXIS_COUNT; i++)
    {
        if (!TestCheck(snapshot.control.axis[i] == 0, "无来源控制轴必须清零")) return 0;
    }
    for (uint32_t i = 0u; i < (uint32_t)INPUT_SW_COUNT; i++)
    {
        if (!TestCheck(snapshot.control.sw[i] == RC_SW_UP, "无来源控制开关必须安全")) return 0;
    }
    if (!TestCheck(ManualInputGetCurrentCopy(&manual) != 0u &&
                   ControlInputGetCopy(&control) != 0u &&
                   ManualInputGetActiveSource() == MANUAL_INPUT_SRC_AUTO,
                   "旧读取接口必须委托统一快照")) return 0;
    return TestCheck(s_timerCallback != NULL && s_bspInitCount == 1u,
                     "初始化必须建立定时刷新并初始化板级遥控");
}

static int TestRawMappedSameGenerationAndReadOnly(void)
{
    ManualInputSnapshot snapshot;
    ManualInputState manual;
    ControlInputState control;
    ManualInputState input;
    uint32_t publish_seq;
    uint32_t watch_count;

    TestReset(100u, 0u);
    g_config.input.axis[0].invert = 1u;
    g_config.input.sw[0].rc_sw = 1u;
    g_config.input.sw[0].invert = 1u;
    input = TestInput(123, -456, RC_SW_DOWN, RC_SW_UP, 0x12u);
    ManualInputUpdateSource(MANUAL_INPUT_SRC_DBUS, &input);

    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                   snapshot.manual.rc.ch[0] == 123 &&
                   snapshot.control.axis[0] == -123 &&
                   snapshot.control.sw[0] == RC_SW_DOWN,
                   "原始输入和映射输入必须同代发布")) return 0;
    if (!TestCheck(RC_data_is_error() == 0u,
                   "在线且内容合法的统一输入不得报数据错误")) return 0;
    if (!TestCheck(ManualInputGetCurrentCopy(&manual) != 0u &&
                   ControlInputGetCopy(&control) != 0u &&
                   memcmp(&manual, &snapshot.manual, sizeof(manual)) == 0 &&
                   memcmp(&control, &snapshot.control, sizeof(control)) == 0,
                   "旧 copy 接口必须读到同一代")) return 0;

    publish_seq = snapshot.publishSeq;
    watch_count = s_watchCount;
    for (uint32_t i = 0u; i < 100u; i++)
    {
        s_tick = i;
        if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u,
                       "重复读取必须成功")) return 0;
        if (!TestCheck(snapshot.publishSeq == publish_seq,
                       "读取不得触发重新发布")) return 0;
    }
    return TestCheck(s_watchCount == watch_count,
                     "读取不得触发 Watch 副作用");
}

static int TestSameTickLatestSource(void)
{
    ManualInputSnapshot snapshot;
    ManualInputState dbus;
    ManualInputState elrs;

    TestReset(100u, 0u);
    s_tick = 10u;
    dbus = TestInput(100, 0, RC_SW_UP, RC_SW_UP, 1u);
    elrs = TestInput(200, 0, RC_SW_DOWN, RC_SW_MID, 2u);
    ManualInputUpdateSource(MANUAL_INPUT_SRC_DBUS, &dbus);
    ManualInputUpdateSource(MANUAL_INPUT_SRC_ELRS, &elrs);
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                   snapshot.activeSource == MANUAL_INPUT_SRC_ELRS &&
                   snapshot.manual.rc.ch[0] == 200,
                   "同 tick 后到来源必须胜出")) return 0;
    return TestCheck(snapshot.sourceSeq == manual_src[MANUAL_INPUT_SRC_ELRS].update_seq &&
                     s_detectHookCount == 2u,
                     "来源序号和过渡 DetectHook 兼容必须更新");
}

static int TestDirtyGenerationRejectsOldCandidate(void)
{
    ManualInputSnapshot snapshot;
    ManualInputState dbus;
    uint32_t publish_before;

    TestReset(100u, KEY_PRESSED_OFFSET_Q);
    s_sdLogActive = 1u;
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u,
                   "注入测试初始快照必须可读")) return 0;
    publish_before = snapshot.publishSeq;
    dbus = TestInput(111, 0, RC_SW_UP, RC_SW_UP, 1u);
    s_injectState = TestInput(777, 0, RC_SW_DOWN, RC_SW_MID, 2u);
    s_injectSource = MANUAL_INPUT_SRC_ELRS;
    s_injectPending = 1u;
    s_tick = 20u;
    ManualInputUpdateSource(MANUAL_INPUT_SRC_DBUS, &dbus);

    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                   snapshot.activeSource == MANUAL_INPUT_SRC_ELRS &&
                   snapshot.manual.rc.ch[0] == 777,
                   "计算期间的新帧必须使旧候选失效")) return 0;
    if (!TestCheck(snapshot.publishSeq == ManualInputSeqNext(publish_before) &&
                   s_watchCount == 1u,
                   "旧候选不得增加发布序号或触发 Watch")) return 0;
    if (!TestCheck(ManualInputRefreshBusy == 0u &&
                   ManualInputRefreshSeq == ManualInputDirtySeq,
                   "有界重试后 writer 和 dirty generation 必须收敛")) return 0;
    return TestCheck(s_callbackOrderErrorCount == 0u && s_logWriteCount == 1u,
                     "Watch/日志必须位于 writer busy 保护范围内");
}

static int TestWatchSideEffectOrdering(void)
{
    ManualInputSnapshot snapshot;
    ManualInputState dbus;

    TestReset(100u, 0u);
    s_sdLogActive = 1u;
    dbus = TestInput(101, 0, RC_SW_UP, RC_SW_UP, 0u);
    s_watchInjectState = TestInput(202, 0, RC_SW_DOWN, RC_SW_DOWN, 0u);
    s_watchInjectSource = MANUAL_INPUT_SRC_ELRS;
    s_watchInjectPending = 1u;
    ManualInputUpdateSource(MANUAL_INPUT_SRC_DBUS, &dbus);

    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                   snapshot.activeSource == MANUAL_INPUT_SRC_DBUS &&
                   snapshot.manual.rc.ch[0] == 101 && ManualInputRefreshDirty != 0u,
                   "Watch 内到达的新来源必须等待当前副作用完成")) return 0;
    if (!TestCheck(s_eventCount == 2u &&
                   s_event[0] == (0x10000u | 101u) &&
                   s_event[1] == (0x20000u | MANUAL_INPUT_SRC_DBUS),
                   "旧发布必须先完成 Watch 和切源日志")) return 0;

    ManualInputRefresh();
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                   snapshot.activeSource == MANUAL_INPUT_SRC_ELRS &&
                   snapshot.manual.rc.ch[0] == 202,
                   "busy 清除后的下一次刷新必须发布新代")) return 0;
    return TestCheck(s_eventCount == 4u &&
                     s_event[2] == (0x10000u | 202u) &&
                     s_event[3] == (0x20000u | MANUAL_INPUT_SRC_ELRS) &&
                     s_callbackOrderErrorCount == 0u,
                     "新代副作用不得越过旧代副作用");
}

static int TestTimeoutBoundaryAndLegacySafety(void)
{
    ManualInputSnapshot snapshot;
    ManualInputState manual;
    ControlInputState control;
    ManualInputState input;
    uint32_t publish_seq;
    uint32_t switch_seq;

    TestReset(100u, 0u);
    s_tick = 7u;
    input = TestInput(333, 0, RC_SW_DOWN, RC_SW_MID, 5u);
    ManualInputUpdateSource(MANUAL_INPUT_SRC_DBUS, &input);
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u,
                   "超时测试输入必须发布")) return 0;
    publish_seq = snapshot.publishSeq;
    switch_seq = snapshot.switchSeq;

    s_tick = 107u;
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                   snapshot.online == 1u && snapshot.sourceAgeMs == 100u &&
                   snapshot.activeSource == MANUAL_INPUT_SRC_DBUS,
                   "100ms 边界仍应在线")) return 0;

    s_tick = 108u;
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                   snapshot.online == 0u && snapshot.activeSource == MANUAL_INPUT_SRC_AUTO &&
                   snapshot.activeMask == 0u && snapshot.sourceSeq == 0u,
                   "101ms 读端必须压成无来源安全帧")) return 0;
    if (!TestCheck(snapshot.sourceTickMs == 0u && snapshot.sourceAgeMs == UINT32_MAX &&
                   snapshot.publishSeq == publish_seq && snapshot.switchSeq == switch_seq,
                   "安全压缩必须清来源时间但保留发布和切源序号")) return 0;
    if (!TestCheck(snapshot.manual.rc.ch[0] == 0 &&
                   snapshot.manual.rc.s[0] == RC_SW_UP &&
                   snapshot.manual.rc.s[1] == RC_SW_UP,
                   "超时原始输入必须安全化")) return 0;
    for (uint32_t i = 0u; i < (uint32_t)INPUT_SW_COUNT; i++)
    {
        if (!TestCheck(snapshot.control.sw[i] == RC_SW_UP,
                       "超时控制开关必须全部安全上档")) return 0;
    }
    if (!TestCheck(ManualInputGetCurrentCopy(&manual) != 0u &&
                   manual.rc.ch[0] == 0 && manual.rc.s[0] == RC_SW_UP &&
                   ControlInputGetCopy(&control) != 0u && control.axis[0] == 0 &&
                   ManualInputGetActiveSource() == MANUAL_INPUT_SRC_AUTO,
                   "旧 getter 在定时器停滞时也必须返回安全值")) return 0;
    if (!TestCheck(RC_data_is_error() != 0u,
                   "过期安全帧不能被诊断成在线合法输入")) return 0;

    TestFireTimer();
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                   snapshot.publishSeq != publish_seq &&
                   snapshot.activeSource == MANUAL_INPUT_SRC_AUTO,
                   "定时刷新应正式发布无来源帧")) return 0;
    return 1;
}

static int TestOtherSourcesStayOnline(void)
{
    ManualInputSnapshot snapshot;
    ManualInputState input;

    TestReset(100u, 0u);
    input = TestInput(10, 0, RC_SW_UP, RC_SW_UP, 0u);
    ManualInputUpdateSource(MANUAL_INPUT_SRC_DBUS, &input);
    s_tick = 50u;
    input.rc.ch[0] = 20;
    ManualInputUpdateSource(MANUAL_INPUT_SRC_ELRS, &input);
    s_tick = 100u;
    input.rc.ch[0] = 30;
    ManualInputUpdateSource(MANUAL_INPUT_SRC_IMAGE, &input);
    s_tick = 150u;
    input.rc.ch[0] = 40;
    ManualInputUpdateSource(MANUAL_INPUT_SRC_ELRS, &input);

    if (!TestCheck(toe_is_error(DBUS_TOE) != 0u,
                   "测试桩应保持物理 DBUS 离线")) return 0;
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u && snapshot.online == 1u &&
                   snapshot.activeSource == MANUAL_INPUT_SRC_ELRS,
                   "DBUS 停止时持续更新的 ELRS/Image 必须在线")) return 0;
    return TestCheck(snapshot.activeMask ==
                         (ManualInputSourceMask(MANUAL_INPUT_SRC_ELRS) |
                          ManualInputSourceMask(MANUAL_INPUT_SRC_IMAGE)),
                     "活动来源掩码必须排除已超时 DBUS");
}

static int TestSpecifiedSourceAndMerge(void)
{
    ManualInputSnapshot snapshot;
    ManualInputState dbus;
    ManualInputState elrs;

    TestReset(100u, 0u);
    g_config.manual_input.active_source = MANUAL_INPUT_SRC_DBUS;
    s_tick = 1u;
    dbus = TestInput(100, 10, RC_SW_UP, RC_SW_UP, 1u);
    ManualInputUpdateSource(MANUAL_INPUT_SRC_DBUS, &dbus);
    s_tick = 2u;
    elrs = TestInput(500, 20, RC_SW_DOWN, RC_SW_MID, 2u);
    ManualInputUpdateSource(MANUAL_INPUT_SRC_ELRS, &elrs);
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                   snapshot.activeSource == MANUAL_INPUT_SRC_DBUS &&
                   snapshot.manual.rc.ch[0] == 100,
                   "指定且活跃的来源必须优先于较新来源")) return 0;

    g_config.manual_input.active_source = MANUAL_INPUT_SRC_AUTO;
    g_config.manual_input.mix_mode = MANUAL_INPUT_MIX_MERGE;
    ManualInputRefresh();
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                   snapshot.mixMode == MANUAL_INPUT_MIX_MERGE &&
                   snapshot.activeSource == MANUAL_INPUT_SRC_ELRS,
                   "合并模式活动来源应记录最新来源")) return 0;
    return TestCheck(snapshot.manual.rc.ch[0] == 500 &&
                     snapshot.manual.rc.ch[1] == 20 &&
                     snapshot.manual.key.v == 3u &&
                     snapshot.manual.rc.s[0] == RC_SW_DOWN,
                     "合并模式必须保留最大幅值、按键并集和最新开关");
}

static int TestSequenceWrapSkipsZero(void)
{
    ManualInputSnapshot snapshot;
    ManualInputState dbus;
    ManualInputState elrs;

    TestReset(100u, 0u);
    ManualInputSourceSeq = UINT32_MAX - 1u;
    ManualInputDirtySeq = UINT32_MAX - 1u;
    ManualInputPublishSeq = UINT32_MAX;
    ManualInputSwitchSeq = UINT32_MAX;
    s_tick = 9u;
    dbus = TestInput(100, 0, RC_SW_UP, RC_SW_UP, 0u);
    elrs = TestInput(200, 0, RC_SW_DOWN, RC_SW_DOWN, 0u);
    ManualInputUpdateSource(MANUAL_INPUT_SRC_DBUS, &dbus);
    ManualInputUpdateSource(MANUAL_INPUT_SRC_ELRS, &elrs);

    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                   snapshot.activeSource == MANUAL_INPUT_SRC_ELRS &&
                   snapshot.sourceSeq == 1u,
                   "来源序号跨回绕时同 tick 后到者仍须胜出")) return 0;
    return TestCheck(snapshot.publishSeq != 0u && snapshot.switchSeq != 0u &&
                     ManualInputDirtySeq != 0u && ManualInputRefreshSeq != 0u &&
                     ManualInputSeqNewer(1u, UINT32_MAX) != 0u,
                     "所有已启用序号回绕必须跳过 0");
}

static int TestExpirySurvivesTickWrap(void)
{
    ManualInputSnapshot snapshot;
    ManualInputState input;

    /* 定时器停滞时由 reader latch 记住已经超时的发布代。 */
    TestReset(100u, 0u);
    s_tick = 20u;
    input = TestInput(321, 0, RC_SW_DOWN, RC_SW_DOWN, 0u);
    ManualInputUpdateSource(MANUAL_INPUT_SRC_DBUS, &input);
    s_tick = 121u;
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                   snapshot.activeSource == MANUAL_INPUT_SRC_AUTO &&
                   ManualInputStore.expired[ManualInputStore.active_bank] != 0u,
                   "reader 首次确认超时后必须设置代级 latch")) return 0;
    s_tick = 20u;
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                   snapshot.activeSource == MANUAL_INPUT_SRC_AUTO &&
                   snapshot.manual.rc.ch[0] == 0,
                   "tick 完整绕回旧值后 latch 必须阻止旧命令复活")) return 0;
    input.rc.ch[0] = 654;
    ManualInputUpdateSource(MANUAL_INPUT_SRC_DBUS, &input);
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u && snapshot.online == 1u &&
                   snapshot.activeSource == MANUAL_INPUT_SRC_DBUS &&
                   snapshot.manual.rc.ch[0] == 654,
                   "同来源新帧必须清除旧代 latch 并恢复")) return 0;

    /* 定时刷新确认超时后还要持久清 global valid。 */
    TestReset(100u, 0u);
    s_tick = 10u;
    input = TestInput(777, 0, RC_SW_DOWN, RC_SW_DOWN, 0u);
    ManualInputUpdateSource(MANUAL_INPUT_SRC_DBUS, &input);
    s_tick = 111u;
    TestFireTimer();
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                   snapshot.activeSource == MANUAL_INPUT_SRC_AUTO &&
                   manual_src[MANUAL_INPUT_SRC_DBUS].valid == 0u,
                   "writer 确认超时后必须持久清来源 valid")) return 0;
    s_tick = 10u;
    TestFireTimer();
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                   snapshot.activeSource == MANUAL_INPUT_SRC_AUTO &&
                   snapshot.manual.rc.ch[0] == 0,
                   "持久失效来源不得在 tick 绕回后复活")) return 0;
    input.rc.ch[0] = 888;
    ManualInputUpdateSource(MANUAL_INPUT_SRC_DBUS, &input);
    return TestCheck(ManualInputSnapshotRead(&snapshot) != 0u && snapshot.online == 1u &&
                     snapshot.manual.rc.ch[0] == 888,
                     "持久失效后只有新 update 才能恢复来源");
}

static int TestPinnedBankAndConfigRefresh(void)
{
    ManualInputSnapshot snapshot;
    ManualInputState input;
    uint8_t inactive_bank;
    uint32_t publish_seq;

    TestReset(100u, 0u);
    input = TestInput(100, 0, RC_SW_UP, RC_SW_UP, 0u);
    ManualInputUpdateSource(MANUAL_INPUT_SRC_DBUS, &input);
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u,
                   "bank 测试输入必须发布")) return 0;
    publish_seq = snapshot.publishSeq;
    inactive_bank = (uint8_t)(ManualInputStore.active_bank ^ 1u);
    ManualInputStore.readers[inactive_bank] = 1u;
    ManualInputMarkDirty();
    ManualInputRefreshIfNeeded(1u);
    if (!TestCheck(ManualInputPublishSeq == publish_seq && ManualInputRefreshDirty != 0u,
                   "仍被读者固定的 bank 不得被 writer 覆盖")) return 0;
    ManualInputStore.readers[inactive_bank] = 0u;

    g_config.input.axis[0].invert = 1u;
    ManualInputRefresh();
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                   snapshot.manual.rc.ch[0] == 100 && snapshot.control.axis[0] == -100,
                   "配置刷新必须只重建映射并保持原始输入")) return 0;
    return TestCheck(snapshot.publishSeq == ManualInputSeqNext(publish_seq),
                     "bank 可用后只应接受一次新发布");
}

static int TestBoardKeyWithoutRemoteSource(void)
{
    ManualInputSnapshot snapshot;

    TestReset(100u, KEY_PRESSED_OFFSET_Q);
    s_boardKeyDown = 1u;
    s_tick = 5u;
    TestFireTimer();
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                   snapshot.activeSource == MANUAL_INPUT_SRC_AUTO &&
                   snapshot.sourceSeq == 0u && snapshot.online == 0u,
                   "板载键不能伪装成远程在线来源")) return 0;
    return TestCheck((snapshot.manual.key.v & KEY_PRESSED_OFFSET_Q) != 0u,
                     "新鲜无来源快照必须保留原有板载按键语义");
}

static int TestInvalidArguments(void)
{
    ManualInputSnapshot snapshot;
    ManualInputState input;
    ControlInputState control;
    uint32_t set_count;
    uint32_t publish_seq;

    TestReset(100u, 0u);
    input = TestInput(1, 2, RC_SW_UP, RC_SW_UP, 0u);
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u,
                   "空参测试初始快照必须可读")) return 0;
    set_count = ManualInputGetSetSourceCount();
    publish_seq = snapshot.publishSeq;
    ManualInputUpdateSource(MANUAL_INPUT_SRC_AUTO, &input);
    ManualInputUpdateSource((uint8_t)(MANUAL_INPUT_SRC_MAX + 1u), &input);
    ManualInputUpdateSource(MANUAL_INPUT_SRC_DBUS, NULL);
    if (!TestCheck(ManualInputSnapshotRead(NULL) == 0u &&
                   ManualInputGetCurrentCopy(NULL) == 0u &&
                   ControlInputGetCopy(NULL) == 0u,
                   "所有 copy 接口必须拒绝空输出")) return 0;
    ControlInputBuild(NULL, NULL, &control);
    if (!TestCheck(control.axis[0] == 0 && control.sw[0] == RC_SW_UP,
                   "纯映射空输入必须生成安全值")) return 0;
    if (!TestCheck(ManualInputGetSetSourceCount() == set_count,
                   "非法来源和空输入不得计数")) return 0;
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                   snapshot.publishSeq == publish_seq,
                   "非法输入不得发布")) return 0;
    return TestCheck(s_criticalErrorCount == 0u,
                     "临界区进入退出必须平衡");
}

TickType_t xTaskGetTickCount(void)
{
    return s_tick;
}

void ManualInputTestEnterCritical(void)
{
    s_criticalDepth++;
}

void ManualInputTestExitCritical(void)
{
    if (s_criticalDepth <= 0)
    {
        s_criticalErrorCount++;
        return;
    }
    s_criticalDepth--;
}

uint32_t __get_IPSR(void)
{
    return 0u;
}

void ManualInputTestDmb(void)
{
}

TimerHandle_t xTimerCreateStatic(const char *name,
                                 TickType_t period,
                                 uint32_t auto_reload,
                                 void *timer_id,
                                 TimerCallbackFunction_t callback,
                                 StaticTimer_t *buffer)
{
    (void)name;
    (void)period;
    (void)auto_reload;
    (void)timer_id;
    s_timerCallback = callback;
    return (TimerHandle_t)buffer;
}

uint32_t xTimerStart(TimerHandle_t timer, TickType_t wait_ticks)
{
    (void)timer;
    (void)wait_ticks;
    return pdTRUE;
}

uint8_t BspKeyReadRawDown(void)
{
    if (s_criticalDepth != 0)
    {
        s_callbackOrderErrorCount++;
    }
    if (s_injectPending != 0u)
    {
        s_injectPending = 0u;
        ManualInputUpdateSource(s_injectSource, &s_injectState);
    }
    return s_boardKeyDown;
}

void WatchUpdateRcSnapshot(const struct ManualInputState *snapshot)
{
    if (s_criticalDepth != 0 || ManualInputRefreshBusy == 0u)
    {
        s_callbackOrderErrorCount++;
    }
    if (snapshot != NULL && s_eventCount < (uint8_t)(sizeof(s_event) / sizeof(s_event[0])))
    {
        s_event[s_eventCount++] = 0x10000u | (uint16_t)snapshot->rc.ch[0];
    }
    s_watchCount++;
    if (s_watchInjectPending != 0u)
    {
        s_watchInjectPending = 0u;
        ManualInputUpdateSource(s_watchInjectSource, &s_watchInjectState);
    }
}

void DetectHook(uint8_t toe)
{
    (void)toe;
    if (s_criticalDepth != 0)
    {
        s_callbackOrderErrorCount++;
    }
    s_detectHookCount++;
}

uint8_t toe_is_error(uint8_t toe)
{
    (void)toe;
    return s_toeError;
}

uint8_t SdLogIsActive(void)
{
    return s_sdLogActive;
}

void SdLogWrite(uint16_t tag, const void *payload, uint16_t payload_size)
{
    if (s_criticalDepth != 0 || ManualInputRefreshBusy == 0u)
    {
        s_callbackOrderErrorCount++;
    }
    if (tag == SDLOG_TAG_EVENT && payload != NULL && payload_size == sizeof(sdlog_event_t) &&
        s_eventCount < (uint8_t)(sizeof(s_event) / sizeof(s_event[0])))
    {
        const sdlog_event_t *event = (const sdlog_event_t *)payload;
        s_event[s_eventCount++] = 0x20000u | event->arg0_u16;
    }
    s_logWriteCount++;
}

void BspRcSbusInit(void)
{
    s_bspInitCount++;
}

void RC_restart(uint16_t dma_buf_num)
{
    (void)dma_buf_num;
}

int main(void)
{
    if (!TestNoSourceSafe()) return 1;
    if (!TestRawMappedSameGenerationAndReadOnly()) return 1;
    if (!TestSameTickLatestSource()) return 1;
    if (!TestDirtyGenerationRejectsOldCandidate()) return 1;
    if (!TestWatchSideEffectOrdering()) return 1;
    if (!TestTimeoutBoundaryAndLegacySafety()) return 1;
    if (!TestOtherSourcesStayOnline()) return 1;
    if (!TestSpecifiedSourceAndMerge()) return 1;
    if (!TestSequenceWrapSkipsZero()) return 1;
    if (!TestExpirySurvivesTickWrap()) return 1;
    if (!TestPinnedBankAndConfigRefresh()) return 1;
    if (!TestBoardKeyWithoutRemoteSource()) return 1;
    if (!TestInvalidArguments()) return 1;
    puts("ManualInputSnapshot regression: PASS");
    return 0;
}
