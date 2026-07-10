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
static uint32_t s_detectHookCount;
static uint32_t s_logWriteCount;
static uint32_t s_bspInitCount;
static uint8_t s_boardKeyDown;
static uint8_t s_sdLogActive;
static uint8_t s_toeError = 1u;
static uint8_t s_injectPending;
static uint8_t s_injectSource;
static ManualInputState s_injectState;
static uint8_t s_logInjectPending;
static uint8_t s_logInjectSource;
static ManualInputState s_logInjectState;
static uint8_t s_updateMutatePending;
static ManualInputState *s_updateMutateTarget;
static ManualInputState s_updateMutateValue;
static uint32_t s_event[16];
static uint8_t s_eventCount;

#include "ControlInput.c"
#include "ManualInput.c"

static void TestUpdateSource(uint8_t source, const ManualInputState *input)
{
    uint8_t protocol = MANUAL_INPUT_PROTOCOL_NONE;

    if (source == MANUAL_INPUT_SRC_DBUS)
    {
        protocol = MANUAL_INPUT_PROTOCOL_DBUS;
    }
    else if (source == MANUAL_INPUT_SRC_ELRS)
    {
        protocol = MANUAL_INPUT_PROTOCOL_CRSF;
    }
    ManualInputUpdateSourceDetail(source, input, protocol, 0u, 0u, NULL);
}

static void TestUpdateSourceMeta(uint8_t source,
                                 const ManualInputState *input,
                                 uint8_t protocol,
                                 uint8_t rawFlags,
                                 uint8_t rawSwitch1)
{
    if (source == MANUAL_INPUT_SRC_IMAGE)
    {
        ManualInputUpdateImageSource(input, protocol, rawFlags, rawSwitch1);
    }
}

#define ManualInputUpdateSource TestUpdateSource
#define ManualInputUpdateSourceMeta TestUpdateSourceMeta

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
    for (uint8_t i = 0u; i < 5u; i++)
    {
        g_config.input.ElrsChMap[i] = (i < 4u) ? i : 6u;
    }
    g_config.input.ElrsSwMap[0] = 4u;
    g_config.input.ElrsSwMap[1] = 5u;
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
    s_detectHookCount = 0u;
    s_logWriteCount = 0u;
    s_bspInitCount = 0u;
    s_boardKeyDown = 0u;
    s_sdLogActive = 0u;
    s_toeError = 1u;
    s_injectPending = 0u;
    s_injectSource = 0u;
    memset(&s_injectState, 0, sizeof(s_injectState));
    s_logInjectPending = 0u;
    s_logInjectSource = 0u;
    memset(&s_logInjectState, 0, sizeof(s_logInjectState));
    s_updateMutatePending = 0u;
    s_updateMutateTarget = NULL;
    memset(&s_updateMutateValue, 0, sizeof(s_updateMutateValue));
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

    TestReset(0u, 0u);
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u,
                   "初始快照必须可读")) return 0;
    if (!TestCheck(snapshot.activeSource == MANUAL_INPUT_SRC_AUTO &&
                    snapshot.activeMask == 0u && snapshot.online == 0u &&
                    snapshot.dataValid == 0u,
                    "无来源必须 AUTO、空掩码且离线")) return 0;
    if (!TestCheck(snapshot.sourceTickMs == 0u &&
                   snapshot.sourceAgeMs == UINT32_MAX &&
                   snapshot.sourceTimeoutMs == MANUAL_INPUT_DEFAULT_TIMEOUT_MS,
                   "无来源时间字段和零超时默认值必须安全")) return 0;
    if (!TestCheck(snapshot.publishSeq != 0u && snapshot.sourceSeq == 0u &&
                   snapshot.switchSeq == 0u && snapshot.semanticsSeq == 1u,
                   "初始发布序号和语义代有效，且尚未发生真实切源")) return 0;
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
    return TestCheck(s_timerCallback != NULL && s_bspInitCount == 1u,
                     "初始化必须建立定时刷新并初始化板级遥控");
}

static int TestRawMappedSameGenerationAndReadOnly(void)
{
    ManualInputSnapshot snapshot;
    ManualInputState input;
    uint32_t publish_seq;
    uint32_t log_count;

    TestReset(100u, 0u);
    g_config.input.axis[0].invert = 1u;
    g_config.input.sw[0].rc_sw = 1u;
    g_config.input.sw[0].invert = 1u;
    input = TestInput(123, -456, RC_SW_DOWN, RC_SW_UP, 0x12u);
    ManualInputUpdateSource(MANUAL_INPUT_SRC_DBUS, &input);

    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                    snapshot.manual.rc.ch[0] == 123 &&
                    snapshot.control.axis[0] == -123 &&
                    snapshot.control.sw[0] == RC_SW_DOWN &&
                    snapshot.dataValid != 0u,
                    "原始输入和映射输入必须同代发布")) return 0;
    publish_seq = snapshot.publishSeq;
    log_count = s_logWriteCount;
    for (uint32_t i = 0u; i < 100u; i++)
    {
        s_tick = i;
        if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u,
                       "重复读取必须成功")) return 0;
        if (!TestCheck(snapshot.publishSeq == publish_seq,
                       "读取不得触发重新发布")) return 0;
    }
    return TestCheck(s_logWriteCount == log_count,
                     "读取不得触发日志副作用");
}

static int TestPublishedDataValidity(void)
{
    ManualInputSnapshot snapshot;
    ManualInputState dbus;
    ManualInputState elrs;
    ManualInputState invalidInput;
    uint32_t switchSeq;

    TestReset(100u, 0u);
    s_sdLogActive = 1u;
    s_tick = 1u;
    dbus = TestInput(42, 0, RC_SW_DOWN, RC_SW_UP, 1u);
    ManualInputUpdateSource(MANUAL_INPUT_SRC_DBUS, &dbus);
    s_tick = 2u;
    elrs = TestInput(84, 0, RC_SW_MID, RC_SW_UP, 2u);
    ManualInputUpdateSource(MANUAL_INPUT_SRC_ELRS, &elrs);
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                   snapshot.activeSource == MANUAL_INPUT_SRC_ELRS,
                   "非法化前 ELRS 必须先真实成为活动来源")) return 0;
    switchSeq = snapshot.switchSeq;

    s_tick = 3u;
    invalidInput = elrs;
    invalidInput.rc.ch[4] = 32767;
    ManualInputUpdateSource(MANUAL_INPUT_SRC_ELRS, &invalidInput);

    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                   snapshot.online != 0u && snapshot.dataValid != 0u &&
                   snapshot.activeSource == MANUAL_INPUT_SRC_DBUS &&
                   snapshot.activeMask == ManualInputSourceMask(MANUAL_INPUT_SRC_DBUS) &&
                   snapshot.sourceSeq == manual_src[MANUAL_INPUT_SRC_DBUS].update_seq &&
                   snapshot.sourceProtocol == MANUAL_INPUT_PROTOCOL_DBUS &&
                   snapshot.switchSeq == ManualInputSeqNext(switchSeq) &&
                   snapshot.manual.rc.ch[0] == 42 &&
                   s_eventCount == 3u &&
                   s_event[2] == (0x20000u | MANUAL_INPUT_SRC_DBUS),
                   "活动来源非法后必须正式切回健康来源并记录一致元数据")) return 0;

    s_tick = 4u;
    ManualInputUpdateSource(MANUAL_INPUT_SRC_ELRS, &elrs);
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                   snapshot.activeSource == MANUAL_INPUT_SRC_ELRS &&
                   snapshot.manual.rc.ch[0] == 84 && snapshot.online != 0u,
                   "非法来源只有新合法帧正式发布后才能恢复")) return 0;

    TestReset(100u, 0u);
    ManualInputUpdateSource(MANUAL_INPUT_SRC_ELRS, &invalidInput);
    return TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                     snapshot.online == 0u && snapshot.dataValid == 0u &&
                     snapshot.activeSource == MANUAL_INPUT_SRC_AUTO,
                     "只有非法来源时必须发布安全离线帧");
}

static int TestDeferredInvalidationBarrier(void)
{
    ManualInputSnapshot snapshot;
    ManualInputState dbus;
    ManualInputState elrs;
    ManualInputState invalidElrs;
    uint32_t publishSeq;

    TestReset(100u, 0u);
    s_tick = 1u;
    dbus = TestInput(111, 0, RC_SW_UP, RC_SW_UP, 1u);
    ManualInputUpdateSource(MANUAL_INPUT_SRC_DBUS, &dbus);
    s_tick = 2u;
    elrs = TestInput(222, 0, RC_SW_DOWN, RC_SW_MID, 2u);
    ManualInputUpdateSource(MANUAL_INPUT_SRC_ELRS, &elrs);
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                   snapshot.activeSource == MANUAL_INPUT_SRC_ELRS,
                   "延迟失效测试必须先发布 ELRS")) return 0;
    publishSeq = snapshot.publishSeq;

    ManualInputRefreshBusy = 1u;
    s_tick = 3u;
    invalidElrs = elrs;
    invalidElrs.rc.ch[4] = 32767;
    ManualInputUpdateSource(MANUAL_INPUT_SRC_ELRS, &invalidElrs);
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                   snapshot.publishSeq == publishSeq &&
                   snapshot.activeSource == MANUAL_INPUT_SRC_DBUS &&
                   snapshot.manual.rc.ch[0] == 111,
                   "writer 忙时失效屏障仍必须立即挡住旧活动命令")) return 0;
    ManualInputRefreshBusy = 0u;

    ManualInputRefreshBusy = 1u;
    s_tick = 4u;
    ManualInputUpdateSource(MANUAL_INPUT_SRC_ELRS, &elrs);
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                   snapshot.publishSeq == publishSeq &&
                   snapshot.activeSource == MANUAL_INPUT_SRC_DBUS,
                   "失效后的合法帧尚未发布时不能复活失效前旧 bank")) return 0;
    ManualInputRefreshBusy = 0u;
    ManualInputRefreshIfNeeded(1u);
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                   snapshot.publishSeq == ManualInputSeqNext(publishSeq) &&
                   snapshot.activeSource == MANUAL_INPUT_SRC_ELRS &&
                   snapshot.manual.rc.ch[0] == 222,
                   "合法新代正式翻 bank 后才允许来源恢复")) return 0;

    TestReset(100u, 0u);
    s_tick = 1u;
    ManualInputUpdateSource(MANUAL_INPUT_SRC_DBUS, &dbus);
    s_tick = 2u;
    ManualInputUpdateSource(MANUAL_INPUT_SRC_ELRS, &elrs);
    (void)ManualInputSnapshotRead(&snapshot);
    publishSeq = snapshot.publishSeq;
    ManualInputRefreshBusy = 1u;
    s_tick = 3u;
    ManualInputUpdateSource(MANUAL_INPUT_SRC_ELRS, &invalidElrs);
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                   snapshot.publishSeq == publishSeq &&
                   snapshot.activeSource == MANUAL_INPUT_SRC_DBUS,
                   "writer busy 时非法活动来源也必须由读端同代回退")) return 0;
    ManualInputRefreshBusy = 0u;
    return 1;
}

static int TestPinnedSourceTimeoutFallback(void)
{
    ManualInputSnapshot snapshot;
    ManualInputState dbus;
    ManualInputState elrs;
    uint32_t publishSeq;
    uint32_t switchSeq;

    TestReset(100u, 0u);
    g_config.manual_input.active_source = MANUAL_INPUT_SRC_DBUS;
    s_tick = 0u;
    dbus = TestInput(100, 0, RC_SW_UP, RC_SW_UP, 1u);
    ManualInputUpdateSource(MANUAL_INPUT_SRC_DBUS, &dbus);
    s_tick = 50u;
    elrs = TestInput(200, 0, RC_SW_DOWN, RC_SW_MID, 2u);
    ManualInputUpdateSource(MANUAL_INPUT_SRC_ELRS, &elrs);
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                   snapshot.activeSource == MANUAL_INPUT_SRC_DBUS,
                   "固定且健康的 DBUS 必须优先")) return 0;
    publishSeq = snapshot.publishSeq;
    switchSeq = snapshot.switchSeq;

    s_tick = 100u;
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                   snapshot.activeSource == MANUAL_INPUT_SRC_DBUS &&
                   snapshot.sourceAgeMs == 100u,
                   "固定来源 100ms 边界仍应在线")) return 0;
    s_tick = 101u;
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                   snapshot.online != 0u &&
                   snapshot.activeSource == MANUAL_INPUT_SRC_ELRS &&
                   snapshot.activeMask == ManualInputSourceMask(MANUAL_INPUT_SRC_ELRS) &&
                   snapshot.sourceProtocol == MANUAL_INPUT_PROTOCOL_CRSF &&
                   snapshot.manual.rc.ch[0] == 200 &&
                   snapshot.publishSeq == publishSeq && snapshot.switchSeq == switchSeq,
                   "固定主来源过期时必须从同一 bank 无空档回退健康来源")) return 0;

    s_tick = 151u;
    return TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                     snapshot.online == 0u && snapshot.activeSource == MANUAL_INPUT_SRC_AUTO,
                     "备用来源也过期后才允许进入安全离线帧");
}

static int TestMergeDegradesPerSource(void)
{
    ManualInputSnapshot snapshot;
    ManualInputState image;
    ManualInputState dbus;
    const uint8_t rawFlags = MANUAL_INPUT_SOURCE_RAW_PAUSE |
                             MANUAL_INPUT_SOURCE_RAW_BTN_L;
    const uint8_t expectedFlags = MANUAL_INPUT_SOURCE_FLAG_AUTO_AIM |
                                  MANUAL_INPUT_SOURCE_FLAG_AUX_FIRE;
    uint32_t publishSeq;

    TestReset(100u, 0u);
    g_config.manual_input.mix_mode = MANUAL_INPUT_MIX_MERGE;
    g_config.manual_input.vt13.auto_aim_pause_enable = 1u;
    g_config.manual_input.vt13.AuxFireBtnLEnable = 1u;
    ManualInputRefresh();

    s_tick = 1u;
    image = TestInput(600, 0, RC_SW_UP, RC_SW_UP, 2u);
    image.mouse.x = INT16_MIN;
    ManualInputUpdateSourceMeta(MANUAL_INPUT_SRC_IMAGE,
                                &image,
                                MANUAL_INPUT_PROTOCOL_IMAGE_VT13,
                                rawFlags,
                                0u);
    s_tick = 50u;
    dbus = TestInput(100, 0, RC_SW_DOWN, RC_SW_MID, 1u);
    ManualInputUpdateSource(MANUAL_INPUT_SRC_DBUS, &dbus);

    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                   snapshot.mixMode == MANUAL_INPUT_MIX_MERGE &&
                   snapshot.activeSource == MANUAL_INPUT_SRC_IMAGE &&
                   snapshot.sourceProtocol == MANUAL_INPUT_PROTOCOL_IMAGE_VT13 &&
                   snapshot.activeMask == (ManualInputSourceMask(MANUAL_INPUT_SRC_DBUS) |
                                           ManualInputSourceMask(MANUAL_INPUT_SRC_IMAGE)) &&
                   snapshot.sourceFlags == expectedFlags &&
                   snapshot.manual.rc.ch[0] == 600 &&
                   snapshot.manual.mouse.x == INT16_MIN &&
                   snapshot.manual.key.v == 2u,
                   "MERGE 必须由代表来源独占操纵帧，只合并类型明确的业务请求")) return 0;
    publishSeq = snapshot.publishSeq;

    s_tick = 102u;
    return TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                     snapshot.online != 0u &&
                     snapshot.activeSource == MANUAL_INPUT_SRC_DBUS &&
                     snapshot.activeMask == ManualInputSourceMask(MANUAL_INPUT_SRC_DBUS) &&
                     snapshot.sourceFlags == 0u &&
                     snapshot.manual.rc.ch[0] == 100 &&
                     snapshot.manual.mouse.x == 0 &&
                     snapshot.manual.key.v == 1u &&
                     snapshot.publishSeq == publishSeq,
                     "MERGE 成员过期时必须只移除该来源贡献并保留健康来源");
}

static int TestStickyAuthorityAndExplicitReclaim(void)
{
    ManualInputSnapshot snapshot;
    ManualInputState dbus;
    ManualInputState elrs;
    uint32_t dbus_action_seq;
    uint32_t dbus_authority_seq;
    uint32_t fallback_action_seq;
    uint32_t fallback_authority_seq;
    uint32_t switch_seq;

    TestReset(100u, 0u);
    g_config.manual_input.mix_mode = MANUAL_INPUT_MIX_SELECT_STICKY;
    ManualInputRefresh();
    s_tick = 1u;
    dbus = TestInput(100, 0, RC_SW_UP, RC_SW_UP, 1u);
    ManualInputUpdateSource(MANUAL_INPUT_SRC_DBUS, &dbus);
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                   snapshot.activeSource == MANUAL_INPUT_SRC_DBUS,
                   "STICKY 首个健康来源必须取得控制权")) return 0;
    dbus_action_seq = snapshot.actionSeq;
    dbus_authority_seq = snapshot.authoritySeq;
    switch_seq = snapshot.switchSeq;

    s_tick = 2u;
    elrs = TestInput(200, 0, RC_SW_DOWN, RC_SW_MID, 2u);
    ManualInputUpdateSource(MANUAL_INPUT_SRC_ELRS, &elrs);
    s_tick = 3u;
    elrs.rc.ch[0] = 300;
    ManualInputUpdateSource(MANUAL_INPUT_SRC_ELRS, &elrs);
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                   snapshot.activeSource == MANUAL_INPUT_SRC_DBUS &&
                   snapshot.manual.rc.ch[0] == 100 &&
                   snapshot.actionSeq == dbus_action_seq &&
                   snapshot.authoritySeq == dbus_authority_seq &&
                   snapshot.switchSeq == switch_seq,
                   "STICKY 下备用来源交错更新不得抢权或重置离散动作")) return 0;

    s_tick = 4u;
    ManualInputInvalidateSource(MANUAL_INPUT_SRC_DBUS);
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                   snapshot.activeSource == MANUAL_INPUT_SRC_ELRS &&
                   snapshot.manual.rc.ch[0] == 300 &&
                   snapshot.actionSeq != dbus_action_seq &&
                   snapshot.authoritySeq != dbus_authority_seq,
                   "当前来源失效后必须立即回退并推进动作代和控制权威代")) return 0;
    fallback_action_seq = snapshot.actionSeq;
    fallback_authority_seq = snapshot.authoritySeq;

    s_tick = 5u;
    ManualInputUpdateSource(MANUAL_INPUT_SRC_DBUS, &dbus);
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                   snapshot.activeSource == MANUAL_INPUT_SRC_ELRS &&
                   snapshot.actionSeq == fallback_action_seq &&
                   snapshot.authoritySeq == fallback_authority_seq,
                   "AUTO+STICKY 下原来源恢复不得反抢健康回退来源")) return 0;

    g_config.manual_input.active_source = MANUAL_INPUT_SRC_DBUS;
    ManualInputRefresh();
    return TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                     snapshot.activeSource == MANUAL_INPUT_SRC_DBUS &&
                     snapshot.actionSeq != fallback_action_seq &&
                     snapshot.authoritySeq != fallback_authority_seq,
                     "显式主来源恢复后必须按配置接管并推进动作代和控制权威代");
}

static int TestDerivedMultiStageAuthorityFallback(void)
{
    ManualInputSnapshot snapshot;
    ManualInputState input;
    uint32_t first_fallback_seq;
    uint32_t first_authority_seq;

    TestReset(100u, 0u);
    g_config.manual_input.active_source = MANUAL_INPUT_SRC_DBUS;
    g_config.manual_input.mix_mode = MANUAL_INPUT_MIX_SELECT_STICKY;
    ManualInputRefresh();
    s_tick = 1u;
    input = TestInput(100, 0, RC_SW_UP, RC_SW_UP, 0u);
    ManualInputUpdateSource(MANUAL_INPUT_SRC_DBUS, &input);
    s_tick = 2u;
    input.rc.ch[0] = 200;
    ManualInputUpdateSource(MANUAL_INPUT_SRC_IMAGE, &input);
    s_tick = 3u;
    input.rc.ch[0] = 300;
    ManualInputUpdateSource(MANUAL_INPUT_SRC_ELRS, &input);

    ManualInputRefreshBusy = 1u;
    s_tick = 4u;
    ManualInputInvalidateSource(MANUAL_INPUT_SRC_DBUS);
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                   snapshot.activeSource == MANUAL_INPUT_SRC_ELRS,
                   "writer 忙时第一层回退必须选到最新健康备用来源")) return 0;
    first_fallback_seq = snapshot.actionSeq;
    first_authority_seq = snapshot.authoritySeq;

    s_tick = 5u;
    ManualInputInvalidateSource(MANUAL_INPUT_SRC_ELRS);
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                   snapshot.activeSource == MANUAL_INPUT_SRC_IMAGE &&
                   snapshot.actionSeq == ManualInputSeqNext(first_fallback_seq) &&
                   snapshot.authoritySeq == ManualInputSeqNext(first_authority_seq),
                   "同一正式 bank 上连续两级回退必须逐次推进动作代和控制权威代")) return 0;
    ManualInputRefreshBusy = 0u;
    ManualInputRefreshIfNeeded(1u);
    return 1;
}

static int TestMergeAuxiliaryActionDoesNotChangeControlAuthority(void)
{
    ManualInputSnapshot snapshot;
    ManualInputState dbus;
    ManualInputState image;
    uint32_t base_action_seq;
    uint32_t base_authority_seq;

    TestReset(100u, 0u);
    g_config.manual_input.mix_mode = MANUAL_INPUT_MIX_MERGE;
    g_config.manual_input.vt13.AuxFireBtnLEnable = 1u;
    ManualInputRefresh();
    s_tick = 1u;
    dbus = TestInput(100, 0, RC_SW_UP, RC_SW_UP, 0u);
    ManualInputUpdateSource(MANUAL_INPUT_SRC_DBUS, &dbus);
    (void)ManualInputSnapshotRead(&snapshot);
    base_action_seq = snapshot.actionSeq;
    base_authority_seq = snapshot.authoritySeq;

    s_tick = 2u;
    image = TestInput(0, 0, RC_SW_DOWN, RC_SW_DOWN, KEY_PRESSED_OFFSET_G);
    ManualInputUpdateSourceMeta(MANUAL_INPUT_SRC_IMAGE,
                                &image,
                                MANUAL_INPUT_PROTOCOL_IMAGE_CUSTOM,
                                MANUAL_INPUT_SOURCE_RAW_BTN_L,
                                0u);
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                   snapshot.activeSource == MANUAL_INPUT_SRC_DBUS &&
                   snapshot.actionSeq != base_action_seq &&
                   snapshot.authoritySeq == base_authority_seq &&
                   snapshot.sourceFlags == MANUAL_INPUT_SOURCE_FLAG_AUX_FIRE &&
                   snapshot.manual.rc.ch[0] == 100 &&
                   snapshot.manual.rc.s[0] == RC_SW_UP &&
                   snapshot.manual.key.v == 0u,
                   "MERGE 辅助动作来源加入只应推进动作代，不得改变主控制权")) return 0;
    base_action_seq = snapshot.actionSeq;

    ManualInputInvalidateSource(MANUAL_INPUT_SRC_IMAGE);
    return TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                     snapshot.activeSource == MANUAL_INPUT_SRC_DBUS &&
                     snapshot.actionSeq != base_action_seq &&
                     snapshot.authoritySeq == base_authority_seq,
                     "MERGE 非代表来源掉线必须局部降级，不得触发整机权威换代");
}

static int TestCrsfMappingRefreshUsesFrozenRaw(void)
{
    ManualInputSnapshot snapshot;
    ManualInputState decoded = {0};
    uint16_t raw[MANUAL_INPUT_CRSF_CHANNEL_COUNT];
    uint32_t source_seq;

    TestReset(100u, 0u);
    for (uint8_t i = 0u; i < MANUAL_INPUT_CRSF_CHANNEL_COUNT; i++)
    {
        raw[i] = MANUAL_INPUT_CRSF_VALUE_MID;
    }
    raw[0] = MANUAL_INPUT_CRSF_VALUE_MAX;
    raw[1] = MANUAL_INPUT_CRSF_VALUE_MIN;
    ManualInputCrsfDecode(raw, &g_config.input, &decoded);
    ManualInputUpdateElrsChannels(&decoded, raw);
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                   snapshot.manual.rc.ch[0] == (int16_t)RC_CH_VALUE_ABS_MAX,
                   "CRSF 原始帧必须按当前冻结映射生成控制值")) return 0;
    source_seq = snapshot.sourceSeq;

    g_config.input.ElrsChMap[0] = 1u;
    ManualInputRefresh();
    return TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                     snapshot.manual.rc.ch[0] == -(int16_t)RC_CH_VALUE_ABS_MAX &&
                     snapshot.sourceSeq == source_seq,
                     "热改 CRSF 映射后无需新帧也必须从同代原始通道重建");
}

static int TestSourceCopyMatchesValidation(void)
{
    ManualInputSnapshot snapshot;
    ManualInputState input;

    TestReset(100u, 0u);
    input = TestInput(123, 0, RC_SW_DOWN, RC_SW_UP, 0u);
    s_updateMutateTarget = &input;
    s_updateMutateValue = TestInput(777, 0, RC_SW_DOWN, RC_SW_UP, 0u);
    s_updateMutateValue.rc.ch[4] = 32767;
    s_updateMutatePending = 1u;

    ManualInputUpdateSource(MANUAL_INPUT_SRC_DBUS, &input);
    return TestCheck(s_updateMutatePending == 0u &&
                     input.rc.ch[0] == 777 &&
                     ManualInputSnapshotRead(&snapshot) != 0u &&
                     snapshot.online != 0u && snapshot.dataValid != 0u &&
                     snapshot.manual.rc.ch[0] == 123 &&
                     snapshot.manual.rc.ch[4] == 0,
                     "来源校验和入库必须使用进入临界区前的同一份副本");
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
                     s_detectHookCount == 1u,
                     "来源序号必须更新，只有物理 DBUS 帧可以刷新 DBUS_TOE");
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
    if (!TestCheck(snapshot.publishSeq == ManualInputSeqNext(publish_before),
                   "旧候选不得增加发布序号")) return 0;
    if (!TestCheck(ManualInputRefreshBusy == 0u &&
                   ManualInputRefreshSeq == ManualInputDirtySeq,
                   "有界重试后 writer 和 dirty generation 必须收敛")) return 0;
    return TestCheck(s_callbackOrderErrorCount == 0u && s_logWriteCount == 1u,
                     "日志必须位于 writer busy 保护范围内");
}

static int TestLogSideEffectOrdering(void)
{
    ManualInputSnapshot snapshot;
    ManualInputState dbus;

    TestReset(100u, 0u);
    s_sdLogActive = 1u;
    dbus = TestInput(101, 0, RC_SW_UP, RC_SW_UP, 0u);
    s_logInjectState = TestInput(202, 0, RC_SW_DOWN, RC_SW_DOWN, 0u);
    s_logInjectSource = MANUAL_INPUT_SRC_ELRS;
    s_logInjectPending = 1u;
    ManualInputUpdateSource(MANUAL_INPUT_SRC_DBUS, &dbus);

    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                   snapshot.activeSource == MANUAL_INPUT_SRC_ELRS &&
                   snapshot.manual.rc.ch[0] == 202 &&
                   ManualInputRefreshDirty == 0u,
                   "日志回调内到达的新来源必须在副作用完成后有界补刷")) return 0;
    return TestCheck(s_eventCount == 2u &&
                     s_event[0] == (0x20000u | MANUAL_INPUT_SRC_DBUS) &&
                     s_event[1] == (0x20000u | MANUAL_INPUT_SRC_ELRS) &&
                     s_callbackOrderErrorCount == 0u,
                     "补刷的新代日志不得越过旧代日志");
}

static int TestTimeoutBoundarySafety(void)
{
    ManualInputSnapshot snapshot;
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
                   snapshot.activeSource == MANUAL_INPUT_SRC_DBUS,
                   "合并模式代表来源应在健康期间保持稳定")) return 0;
    return TestCheck(snapshot.manual.rc.ch[0] == 100 &&
                     snapshot.manual.rc.ch[1] == 10 &&
                     snapshot.manual.key.v == 1u &&
                     snapshot.manual.rc.s[0] == RC_SW_UP,
                     "合并模式不得把其他来源的轴和通用按键混入代表操纵帧");
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
    if (!TestCheck(snapshot.publishSeq != 0u && snapshot.switchSeq != 0u &&
                   ManualInputDirtySeq != 0u && ManualInputRefreshSeq != 0u &&
                   ManualInputSeqNewer(1u, UINT32_MAX) != 0u,
                   "来源、发布、切源和内部序号回绕必须跳过 0")) return 0;

    ManualInputStore.bank[ManualInputStore.active_bank].semanticsSeq = UINT32_MAX;
    g_config.input.sw[INPUT_SW_GIMBAL_MODE].invert = 1u;
    ManualInputRefresh();
    return TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                         snapshot.semanticsSeq == 1u,
                     "输入解释代跨回绕时也必须跳过 0");
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
                   (ManualInputStore.excluded_mask[ManualInputStore.active_bank] &
                    ManualInputSourceMask(MANUAL_INPUT_SRC_DBUS)) != 0u,
                   "reader 首次确认超时后必须设置逐来源 latch")) return 0;
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

static int TestBankAndConfigRefresh(void)
{
    ManualInputSnapshot snapshot;
    ManualInputState input;
    uint32_t publish_seq;

    TestReset(100u, 0u);
    input = TestInput(100, 0, RC_SW_UP, RC_SW_UP, 0u);
    ManualInputUpdateSource(MANUAL_INPUT_SRC_DBUS, &input);
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u,
                   "bank 测试输入必须发布")) return 0;
    publish_seq = snapshot.publishSeq;
    g_config.input.axis[0].invert = 1u;
    ManualInputRefresh();
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                   snapshot.manual.rc.ch[0] == 100 && snapshot.control.axis[0] == -100,
                   "配置刷新必须只重建映射并保持原始输入")) return 0;
    return TestCheck(snapshot.publishSeq == ManualInputSeqNext(publish_seq),
                     "配置变化只应接受一次新发布");
}

static int TestSemanticsSameGeneration(void)
{
    ManualInputSnapshot snapshot;
    ManualInputState input;
    ManualInputSemanticsConfig frozen_semantics;
    const ManualInputSemanticsConfig updated_semantics = {
        .GimbalSafePos = MANUAL_INPUT_SWITCH_POS_MID,
        .ChassisSafePos = MANUAL_INPUT_SWITCH_POS_DOWN,
        .ChassisFollowPos = MANUAL_INPUT_SWITCH_POS_UP,
        .ChassisSpinPos = MANUAL_INPUT_SWITCH_POS_DOWN,
        .ShootStopPos = MANUAL_INPUT_SWITCH_POS_MID,
        .ShootReadyPos = MANUAL_INPUT_SWITCH_POS_DOWN,
        .ShootFirePos = MANUAL_INPUT_SWITCH_POS_UP,
        .image_vt13_shoot_switch_input = 1u,
    };
    uint32_t publish_seq;
    uint32_t semantics_seq;

    TestReset(100u, 0u);
    input = TestInput(246, 0, RC_SW_DOWN, RC_SW_MID, 0u);
    ManualInputUpdateSource(MANUAL_INPUT_SRC_DBUS, &input);
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                   snapshot.online != 0u && snapshot.semanticsSeq == 1u,
                   "首代输入必须携带初始语义代")) return 0;
    frozen_semantics = snapshot.semantics;
    publish_seq = snapshot.publishSeq;
    semantics_seq = snapshot.semanticsSeq;

    /* 模拟 AuxParam 已改实时配置，但刷新调用尚未获得运行机会的抢占窗口。 */
    g_config.manual_input.semantics = updated_semantics;
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                   memcmp(&snapshot.semantics,
                          &frozen_semantics,
                          sizeof(frozen_semantics)) == 0 &&
                   snapshot.semanticsSeq == semantics_seq &&
                   snapshot.publishSeq == publish_seq,
                   "刷新前的读者必须继续使用旧帧冻结语义")) return 0;

    ManualInputRefresh();
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                   memcmp(&snapshot.semantics,
                          &updated_semantics,
                          sizeof(updated_semantics)) == 0 &&
                   snapshot.semanticsSeq == ManualInputSeqNext(semantics_seq) &&
                   snapshot.publishSeq == ManualInputSeqNext(publish_seq) &&
                   snapshot.manual.rc.ch[0] == 246 &&
                   snapshot.control.axis[0] == 246,
                   "刷新后新语义必须和同一帧原始/映射输入一起发布")) return 0;
    semantics_seq = snapshot.semanticsSeq;
    publish_seq = snapshot.publishSeq;

    ManualInputRefresh();
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                   snapshot.semanticsSeq == semantics_seq &&
                   snapshot.publishSeq == ManualInputSeqNext(publish_seq),
                   "未改变语义的普通刷新不得推进语义代")) return 0;

    g_config.input.sw[INPUT_SW_GIMBAL_MODE].invert = 1u;
    ManualInputRefresh();
    return TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                         snapshot.control.sw[INPUT_SW_GIMBAL_MODE] == RC_SW_UP &&
                         snapshot.semanticsSeq == ManualInputSeqNext(semantics_seq) &&
                         memcmp(&snapshot.semantics,
                                &updated_semantics,
                                sizeof(updated_semantics)) == 0,
                     "安全开关映射变化必须推进解释代并保持冻结业务语义");
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

static int TestIdleTimerDoesNotRepublish(void)
{
    ManualInputSnapshot snapshot;
    uint32_t publishSeq;

    TestReset(100u, KEY_PRESSED_OFFSET_Q);
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u,
                   "空闲定时器测试初始快照必须可读")) return 0;
    publishSeq = snapshot.publishSeq;
    s_tick = 50u;
    TestFireTimer();
    return TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                         snapshot.publishSeq == publishSeq,
                     "无输入、配置和板载键变化时不得固定频率重建快照");
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
    if (!TestCheck(ManualInputSnapshotRead(NULL) == 0u,
                   "统一快照必须拒绝空输出")) return 0;
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

static int TestTypedWriterNullRevokesSource(void)
{
    ManualInputSnapshot snapshot;
    ManualInputState input = TestInput(10, 20, RC_SW_UP, RC_SW_UP, 0u);
    uint16_t raw[MANUAL_INPUT_CRSF_CHANNEL_COUNT] = {0};

    TestReset(100u, 0u);
    ManualInputUpdateImageSource(&input,
                                 MANUAL_INPUT_PROTOCOL_IMAGE_CUSTOM,
                                 0u,
                                 0u);
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u && snapshot.online != 0u,
                   "图传空参撤销测试必须先建立有效来源")) return 0;
    ManualInputUpdateImageSource(NULL,
                                 MANUAL_INPUT_PROTOCOL_IMAGE_CUSTOM,
                                 0u,
                                 0u);
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u && snapshot.online == 0u,
                   "图传专用写入口收到空载荷必须立即撤销旧命令")) return 0;

    for (uint8_t i = 0u; i < MANUAL_INPUT_CRSF_CHANNEL_COUNT; i++)
    {
        raw[i] = 992u;
    }
    ManualInputUpdateElrsChannels(&input, raw);
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u && snapshot.online != 0u,
                   "ELRS 空参撤销测试必须先建立有效来源")) return 0;
    ManualInputUpdateElrsChannels(NULL, raw);
    return TestCheck(ManualInputSnapshotRead(&snapshot) != 0u && snapshot.online == 0u,
                     "ELRS 专用写入口收到空载荷必须立即撤销旧命令");
}

static int TestSourceMetadataSameGeneration(void)
{
    ManualInputSnapshot snapshot;
    ManualInputState image = TestInput(321, -123, RC_SW_DOWN, RC_SW_MID, 0u);
    ManualInputState dbus = TestInput(11, 22, RC_SW_UP, RC_SW_UP, 0u);
    const uint8_t rawFlags = MANUAL_INPUT_SOURCE_RAW_PAUSE |
                             MANUAL_INPUT_SOURCE_RAW_MOUSE_R |
                             MANUAL_INPUT_SOURCE_RAW_BTN_L |
                             MANUAL_INPUT_SOURCE_RAW_MOUSE_L;
    const uint8_t flags = MANUAL_INPUT_SOURCE_FLAG_AUTO_AIM |
                          MANUAL_INPUT_SOURCE_FLAG_AUX_FIRE;

    TestReset(20u, 0u);
    g_config.manual_input.vt13.auto_aim_pause_enable = 1u;
    g_config.manual_input.vt13.auto_aim_mouse_r_enable = 1u;
    g_config.manual_input.vt13.AuxFireBtnLEnable = 1u;
    g_config.manual_input.vt13.AuxFireMouseLEnable = 1u;
    ManualInputUpdateSourceMeta(MANUAL_INPUT_SRC_IMAGE,
                                &image,
                                MANUAL_INPUT_PROTOCOL_IMAGE_VT13,
                                rawFlags,
                                0u);
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                   snapshot.online != 0u &&
                   snapshot.activeSource == MANUAL_INPUT_SRC_IMAGE &&
                   snapshot.sourceProtocol == MANUAL_INPUT_PROTOCOL_IMAGE_VT13 &&
                   snapshot.sourceFlags == flags &&
                   snapshot.manual.rc.ch[0] == image.rc.ch[0],
                   "图传协议与业务标志必须和原始/映射输入同代发布")) return 0;

    g_config.manual_input.vt13.auto_aim_pause_enable = 0u;
    g_config.manual_input.vt13.auto_aim_mouse_r_enable = 0u;
    g_config.manual_input.vt13.AuxFireBtnLEnable = 0u;
    g_config.manual_input.vt13.AuxFireMouseLEnable = 0u;
    ManualInputRefresh();
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                   snapshot.sourceProtocol == MANUAL_INPUT_PROTOCOL_IMAGE_VT13 &&
                   snapshot.sourceFlags == 0u,
                   "热更新语义开关后必须从同代原始业务位重新构建标志")) return 0;

    s_tick = 21u;
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                   snapshot.online == 0u &&
                   snapshot.activeSource == MANUAL_INPUT_SRC_AUTO &&
                   snapshot.sourceProtocol == MANUAL_INPUT_PROTOCOL_NONE &&
                   snapshot.sourceFlags == 0u,
                   "来源过期时协议与业务标志必须和输入一起清零")) return 0;

    TestReset(20u, 0u);
    g_config.manual_input.vt13.auto_aim_pause_enable = 1u;
    g_config.manual_input.vt13.auto_aim_mouse_r_enable = 1u;
    g_config.manual_input.vt13.AuxFireBtnLEnable = 1u;
    g_config.manual_input.vt13.AuxFireMouseLEnable = 1u;
    ManualInputUpdateSourceMeta(MANUAL_INPUT_SRC_IMAGE,
                                &image,
                                MANUAL_INPUT_PROTOCOL_IMAGE_VT13,
                                rawFlags,
                                0u);
    ManualInputUpdateSource(MANUAL_INPUT_SRC_DBUS, &dbus);
    return TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                         snapshot.activeSource == MANUAL_INPUT_SRC_DBUS &&
                         snapshot.sourceProtocol == MANUAL_INPUT_PROTOCOL_DBUS &&
                         snapshot.sourceFlags == 0u,
                     "切到 DBUS 时协议应随来源更新，且不得残留图传业务标志");
}

TickType_t xTaskGetTickCount(void)
{
    return s_tick;
}

void ManualInputTestEnterCritical(void)
{
    if (s_updateMutatePending != 0u && s_updateMutateTarget != NULL)
    {
        s_updateMutatePending = 0u;
        *s_updateMutateTarget = s_updateMutateValue;
    }
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
    if (s_logInjectPending != 0u)
    {
        s_logInjectPending = 0u;
        ManualInputUpdateSource(s_logInjectSource, &s_logInjectState);
    }
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
    if (!TestPublishedDataValidity()) return 1;
    if (!TestDeferredInvalidationBarrier()) return 1;
    if (!TestPinnedSourceTimeoutFallback()) return 1;
    if (!TestMergeDegradesPerSource()) return 1;
    if (!TestStickyAuthorityAndExplicitReclaim()) return 1;
    if (!TestDerivedMultiStageAuthorityFallback()) return 1;
    if (!TestMergeAuxiliaryActionDoesNotChangeControlAuthority()) return 1;
    if (!TestCrsfMappingRefreshUsesFrozenRaw()) return 1;
    if (!TestSourceCopyMatchesValidation()) return 1;
    if (!TestSameTickLatestSource()) return 1;
    if (!TestDirtyGenerationRejectsOldCandidate()) return 1;
    if (!TestLogSideEffectOrdering()) return 1;
    if (!TestTimeoutBoundarySafety()) return 1;
    if (!TestOtherSourcesStayOnline()) return 1;
    if (!TestSpecifiedSourceAndMerge()) return 1;
    if (!TestSequenceWrapSkipsZero()) return 1;
    if (!TestExpirySurvivesTickWrap()) return 1;
    if (!TestBankAndConfigRefresh()) return 1;
    if (!TestSemanticsSameGeneration()) return 1;
    if (!TestBoardKeyWithoutRemoteSource()) return 1;
    if (!TestIdleTimerDoesNotRepublish()) return 1;
    if (!TestInvalidArguments()) return 1;
    if (!TestTypedWriterNullRevokesSource()) return 1;
    if (!TestSourceMetadataSameGeneration()) return 1;
    puts("ManualInputSnapshot regression: PASS");
    return 0;
}
