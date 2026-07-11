/* 真实 ElrsTask/ManualInput 主机回归：覆盖严格链路证据、批裁决和来源回退。 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "ControlInput.h"
#include "ElrsTask.h"
#include "ManualInputSnapshot.h"
#include "RobotConfig.h"
#include "timers.h"

Config g_config;

static TickType_t s_tick;
static int s_criticalDepth;
static uint32_t s_auxBaudrate;
static uint8_t s_auxDmaAvailable;
static uint8_t *s_auxDmaBuffer;
static uint16_t s_auxDmaLength;
static uint32_t s_rcLogCount;
static uint32_t s_sdLogCount;
static uint8_t s_injectItOverflowOnTickRead;
static uint8_t s_waitNotifyScheduled;
static TickType_t s_waitNotifyTick;
static uint32_t s_waitNotifyBits;
static TickType_t s_lastWaitTicks;
static uint32_t s_taskNotifyBits;
static uint8_t s_injectRestartDuringStatsTickRead;
static uint8_t s_injectDmaTcDuringBatchTickRead;
static uint8_t s_injectDmaTcOnSdLogWrite;
static uint8_t s_injectDmaTcOnBaudRead;
static uint8_t s_injectUartErrorDuringBatchTickRead;
static uint8_t s_injectUartErrorOnBaudRead;
static uint8_t s_injectUartErrorOnSdLogWrite;
static const uint8_t *s_injectItAppendData;
static uint16_t s_injectItAppendLength;

#include "ControlInput.c"
#include "ManualInput.c"
#include "ElrsTask.c"

#define TEST_FRAME_BUFFER_SIZE 192u

static int TestCheck(int condition, const char *message)
{
    if (!condition)
    {
        (void)fprintf(stderr, "FAIL: %s\n", message);
        return 0;
    }
    return 1;
}

static uint8_t TestGuardReject(void *context)
{
    (void)context;
    return 0u;
}

static uint8_t TestGuardAccept(void *context)
{
    (void)context;
    return 1u;
}

static void TestQueueDmaBatch(const uint8_t *bytes, uint16_t length)
{
    if (bytes == NULL || length > ELRS_LINK_DMA_RX_BUF_SIZE)
    {
        return;
    }
    memcpy(ElrsLinkRx.dma, bytes, length);
    ElrsLinkOnRxEvent(length, BSP_AUX_LINK_RXEVENT_IDLE);
}

static void TestConfigInit(void)
{
    memset(&g_config, 0, sizeof(g_config));
    g_config.manual_input.active_source = MANUAL_INPUT_SRC_AUTO;
    g_config.manual_input.mix_mode = MANUAL_INPUT_MIX_SELECT_LATEST;
    g_config.manual_input.source_timeout_ms = 100u;
    g_config.manual_input.semantics.GimbalSafePos = MANUAL_INPUT_SWITCH_POS_UP;
    g_config.manual_input.semantics.ChassisSafePos = MANUAL_INPUT_SWITCH_POS_UP;
    g_config.manual_input.semantics.ChassisFollowPos = MANUAL_INPUT_SWITCH_POS_MID;
    g_config.manual_input.semantics.ChassisSpinPos = MANUAL_INPUT_SWITCH_POS_DOWN;
    g_config.manual_input.semantics.ShootStopPos = MANUAL_INPUT_SWITCH_POS_UP;
    g_config.manual_input.semantics.ShootReadyPos = MANUAL_INPUT_SWITCH_POS_MID;
    g_config.manual_input.semantics.ShootFirePos = MANUAL_INPUT_SWITCH_POS_DOWN;
    g_config.input.ElrsChMap[0] = 0u;
    g_config.input.ElrsChMap[1] = 1u;
    g_config.input.ElrsChMap[2] = 2u;
    g_config.input.ElrsChMap[3] = 3u;
    g_config.input.ElrsChMap[4] = 6u;
    g_config.input.ElrsSwMap[0] = 4u;
    g_config.input.ElrsSwMap[1] = 5u;
    for (uint8_t i = 0u; i < (uint8_t)INPUT_AXIS_COUNT; i++)
    {
        g_config.input.axis[i].rc_ch = (uint8_t)(i % 5u);
    }
    for (uint8_t i = 0u; i < (uint8_t)INPUT_SW_COUNT; i++)
    {
        g_config.input.sw[i].rc_sw = (uint8_t)(i % 2u);
    }
}

static void TestReset(uint8_t dmaAvailable)
{
    s_auxBaudrate = ELRS_LINK_BAUD;
    ElrsLinkStop();
    TestConfigInit();
    s_tick = 0u;
    s_criticalDepth = 0;
    s_auxDmaAvailable = dmaAvailable;
    s_auxDmaBuffer = NULL;
    s_auxDmaLength = 0u;
    s_rcLogCount = 0u;
    s_sdLogCount = 0u;
    s_injectItOverflowOnTickRead = 0u;
    s_waitNotifyScheduled = 0u;
    s_waitNotifyTick = 0u;
    s_waitNotifyBits = 0u;
    s_lastWaitTicks = 0u;
    s_taskNotifyBits = 0u;
    s_injectRestartDuringStatsTickRead = 0u;
    s_injectDmaTcDuringBatchTickRead = 0u;
    s_injectDmaTcOnSdLogWrite = 0u;
    s_injectDmaTcOnBaudRead = 0u;
    s_injectUartErrorDuringBatchTickRead = 0u;
    s_injectUartErrorOnBaudRead = 0u;
    s_injectUartErrorOnSdLogWrite = 0u;
    s_injectItAppendData = NULL;
    s_injectItAppendLength = 0u;

    memset(&ElrsLinkDiag, 0, sizeof(ElrsLinkDiag));
    ElrsLinkDiag.state = ELRS_LINK_STATE_WAIT_STATS;
    memset(&ElrsLinkPendingRc, 0, sizeof(ElrsLinkPendingRc));
    ElrsLinkLastStatsTick = 0u;
    ElrsLinkLastStatsSessionGen = 0u;
    ElrsLinkSessionGen = 1u;
    ElrsLinkTransportEpoch = 1u;
    ElrsLinkBatchSessionGen = 1u;
    ElrsLinkBatchTransportEpoch = 1u;
    ElrsLinkBatchActive = 0u;
    ElrsLinkBatchForceInvalidate = 0u;
    ElrsLinkTaskHandle = (TaskHandle_t)(uintptr_t)1u;
    ElrsLinkNotifyPending = 0u;
    ElrsLinkReset();

    ManualInputInit();
    ElrsLinkRxStart();
}

static void TestChannels(uint16_t channel[MANUAL_INPUT_CRSF_CHANNEL_COUNT])
{
    for (uint8_t i = 0u; i < MANUAL_INPUT_CRSF_CHANNEL_COUNT; i++)
    {
        channel[i] = MANUAL_INPUT_CRSF_VALUE_MID;
    }
    channel[4] = MANUAL_INPUT_CRSF_VALUE_MIN;
    channel[5] = MANUAL_INPUT_CRSF_VALUE_MID;
}

static uint8_t TestBuildFrame(uint8_t *frame,
                              uint8_t address,
                              uint8_t type,
                              const uint8_t *payload,
                              uint8_t payloadLength)
{
    const uint8_t length = (uint8_t)(payloadLength + 2u);
    const uint8_t total = (uint8_t)(length + 2u);

    frame[0] = address;
    frame[1] = length;
    frame[2] = type;
    if (payloadLength != 0u)
    {
        memcpy(&frame[3], payload, payloadLength);
    }
    frame[total - 1u] = ElrsLinkCrc8DvbS2(&frame[2], (uint8_t)(length - 1u));
    return total;
}

static uint8_t TestBuildStats(uint8_t *frame, uint8_t address, uint8_t lq, uint8_t extra)
{
    uint8_t payload[11] = {70u, 72u, 0u, (uint8_t)-5, 0u, 2u, 3u, 80u, 90u, 4u, 99u};

    payload[2] = lq;
    return TestBuildFrame(frame,
                          address,
                          CRSF_FRAMETYPE_LINK_STATISTICS,
                          payload,
                          (uint8_t)(10u + (extra != 0u ? 1u : 0u)));
}

static uint8_t TestBuildRc(uint8_t *frame,
                           uint8_t address,
                           const uint16_t channel[MANUAL_INPUT_CRSF_CHANNEL_COUNT],
                           uint8_t extra)
{
    uint8_t payload[23] = {0};
    uint32_t bitbuf = 0u;
    uint8_t bitcnt = 0u;
    uint8_t out = 0u;

    for (uint8_t i = 0u; i < MANUAL_INPUT_CRSF_CHANNEL_COUNT; i++)
    {
        bitbuf |= ((uint32_t)channel[i]) << bitcnt;
        bitcnt = (uint8_t)(bitcnt + 11u);
        while (bitcnt >= 8u)
        {
            payload[out++] = (uint8_t)(bitbuf & 0xFFu);
            bitbuf >>= 8u;
            bitcnt = (uint8_t)(bitcnt - 8u);
        }
    }
    if (bitcnt != 0u)
    {
        payload[out++] = (uint8_t)(bitbuf & 0xFFu);
    }
    if (extra != 0u)
    {
        payload[out++] = 0xA5u;
    }
    return TestBuildFrame(frame,
                          address,
                          CRSF_FRAMETYPE_RC_CHANNELS_PACKED,
                          payload,
                          out);
}

static void TestFeedBatch(const uint8_t *bytes, uint16_t length)
{
    ElrsLinkExpireStats();
    ElrsLinkBatchBegin(ElrsLinkTransportEpoch);
    for (uint16_t i = 0u; i < length; i++)
    {
        ElrsLinkOnByte(bytes[i]);
    }
    ElrsLinkBatchCommit();
}

static uint16_t TestAppend(uint8_t *buffer,
                           uint16_t position,
                           const uint8_t *frame,
                           uint8_t length)
{
    memcpy(&buffer[position], frame, length);
    return (uint16_t)(position + length);
}

static int TestStrictStatsAndOrder(void)
{
    static const uint8_t knownStats[] = {
        0xC8u, 0x0Cu, 0x14u, 0x46u, 0x48u, 0x50u, 0xFBu,
        0x00u, 0x02u, 0x03u, 0x50u, 0x5Au, 0x04u, 0x74u};
    ManualInputSnapshot snapshot;
    uint16_t channel[MANUAL_INPUT_CRSF_CHANNEL_COUNT];
    uint8_t rc[CRSF_FRAME_SIZE_MAX];
    uint8_t stats[CRSF_FRAME_SIZE_MAX];
    uint8_t batch[TEST_FRAME_BUFFER_SIZE];
    uint8_t rcLength;
    uint8_t statsLength;
    uint16_t length;

    TestReset(1u);
    TestFeedBatch(knownStats, (uint16_t)sizeof(knownStats));
    if (!TestCheck(ElrsLinkDiag.state == ELRS_LINK_STATE_WAIT_RC &&
                       ElrsLinkDiag.uplink_lq == 80u,
                   "固定 CRSF CRC-DVB-S2 向量必须被真实解析器接受")) return 0;

    TestReset(1u);
    TestChannels(channel);
    channel[0] = MANUAL_INPUT_CRSF_VALUE_MAX;
    rcLength = TestBuildRc(rc, CRSF_ADDRESS_FLIGHT_CONTROLLER, channel, 0u);
    statsLength = TestBuildStats(stats, CRSF_ADDRESS_FLIGHT_CONTROLLER, 80u, 0u);

    TestFeedBatch(rc, rcLength);
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u && snapshot.online == 0u &&
                       ElrsLinkDiag.state == ELRS_LINK_STATE_WAIT_STATS,
                   "仅有 0x16 的泛 CRSF 设备不得激活严格 ELRS 来源")) return 0;

    length = 0u;
    length = TestAppend(batch, length, rc, rcLength);
    length = TestAppend(batch, length, stats, statsLength);
    TestFeedBatch(batch, length);
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u && snapshot.online == 0u &&
                       ElrsLinkDiag.state == ELRS_LINK_STATE_WAIT_RC,
                   "同批 RC 在正 LQ 之前到达时不得被事后追认")) return 0;

    TestFeedBatch(rc, rcLength);
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u && snapshot.online != 0u &&
                       snapshot.activeSource == MANUAL_INPUT_SRC_ELRS &&
                       snapshot.manual.rc.ch[0] == (int16_t)RC_CH_VALUE_ABS_MAX &&
                       ElrsLinkDiag.state == ELRS_LINK_STATE_UP,
                   "正 LQ 后的新合法 RC 必须激活 ELRS")) return 0;

    TestReset(1u);
    length = 0u;
    length = TestAppend(batch, length, stats, statsLength);
    length = TestAppend(batch, length, rc, rcLength);
    TestFeedBatch(batch, length);
    return TestCheck(ManualInputSnapshotRead(&snapshot) != 0u && snapshot.online != 0u &&
                         ElrsLinkDiag.rc_publish_count == 1u && s_rcLogCount == 1u,
                     "同批正 LQ 在前、新 RC 在后时应只发布最后一帧");
}

static int TestMappedChannelValidation(void)
{
    ManualInputSnapshot snapshot;
    uint16_t channel[MANUAL_INPUT_CRSF_CHANNEL_COUNT];
    uint8_t rc[CRSF_FRAME_SIZE_MAX];
    uint8_t stats[CRSF_FRAME_SIZE_MAX];
    uint8_t batch[TEST_FRAME_BUFFER_SIZE];
    uint8_t rcLength;
    uint8_t statsLength;
    uint16_t length = 0u;

    TestReset(1u);
    TestChannels(channel);
    statsLength = TestBuildStats(stats, CRSF_ADDRESS_FLIGHT_CONTROLLER, 80u, 0u);
    rcLength = TestBuildRc(rc, CRSF_ADDRESS_FLIGHT_CONTROLLER, channel, 0u);
    length = TestAppend(batch, length, stats, statsLength);
    length = TestAppend(batch, length, rc, rcLength);
    TestFeedBatch(batch, length);

    channel[0] = 0u;
    rcLength = TestBuildRc(rc, CRSF_ADDRESS_FLIGHT_CONTROLLER, channel, 0u);
    TestFeedBatch(rc, rcLength);
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u && snapshot.online == 0u &&
                       ElrsLinkDiag.channel_reject_count == 1u,
                   "实际映射通道超出 172..1811 时必须立即撤销 ELRS")) return 0;

    TestChannels(channel);
    channel[15] = 0u;
    rcLength = TestBuildRc(rc, CRSF_ADDRESS_FLIGHT_CONTROLLER, channel, 0u);
    TestFeedBatch(rc, rcLength);
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u && snapshot.online != 0u,
                   "未映射 AUX 的占位异常值不得误伤有效控制帧")) return 0;

    g_config.input.ElrsChMap[0] = 15u;
    ManualInputRefresh();
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u && snapshot.online == 0u,
                   "配置热改映射到旧帧异常通道时必须按同代配置排除 ELRS")) return 0;
    g_config.input.ElrsChMap[0] = 0u;
    ManualInputRefresh();
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u && snapshot.online != 0u,
                   "恢复合法冻结映射后旧帧仍可在来源时效内重建")) return 0;
    g_config.input.ElrsChMap[0] = MANUAL_INPUT_CRSF_CHANNEL_COUNT;
    ManualInputRefresh();
    return TestCheck(ManualInputSnapshotRead(&snapshot) != 0u && snapshot.online == 0u,
                     "CRSF 映射索引越界必须失效，不得静默回退默认通道");
}

static int TestParserValidationAndResync(void)
{
    static const uint8_t ignoredAddresses[] = {
        CRSF_ADDRESS_BROADCAST,
        CRSF_ADDRESS_RADIO_TRANSMITTER,
        CRSF_ADDRESS_CRSF_RECEIVER};
    ManualInputSnapshot snapshot;
    ElrsLinkStats diag;
    uint16_t channel[MANUAL_INPUT_CRSF_CHANNEL_COUNT];
    uint8_t rc[CRSF_FRAME_SIZE_MAX];
    uint8_t stats[CRSF_FRAME_SIZE_MAX];
    uint8_t bad[CRSF_FRAME_SIZE_MAX];
    uint8_t batch[TEST_FRAME_BUFFER_SIZE];
    uint8_t rcLength;
    uint8_t statsLength;
    uint8_t badLength;
    uint16_t length;

    TestReset(1u);
    TestChannels(channel);
    length = 0u;
    for (uint8_t i = 0u; i < (uint8_t)sizeof(ignoredAddresses); i++)
    {
        statsLength = TestBuildStats(stats, ignoredAddresses[i], 80u, 0u);
        rcLength = TestBuildRc(rc, ignoredAddresses[i], channel, 0u);
        length = TestAppend(batch, length, stats, statsLength);
        length = TestAppend(batch, length, rc, rcLength);
    }
    TestFeedBatch(batch, length);
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u && snapshot.online == 0u &&
                       ElrsLinkDiag.valid_frame_count == 6u,
                   "0x00/0xEA/0xEC 的合法 0x14/0x16 均不得更新飞控输入")) return 0;

    statsLength = TestBuildStats(stats, CRSF_ADDRESS_FLIGHT_CONTROLLER, 80u, 1u);
    rcLength = TestBuildRc(rc, CRSF_ADDRESS_FLIGHT_CONTROLLER, channel, 1u);
    badLength = TestBuildStats(bad, CRSF_ADDRESS_FLIGHT_CONTROLLER, 70u, 0u);
    bad[badLength - 1u] ^= 0x5Au;
    length = 0u;
    batch[length++] = 0x55u;
    batch[length++] = CRSF_ADDRESS_FLIGHT_CONTROLLER;
    batch[length++] = 1u;
    batch[length++] = CRSF_ADDRESS_FLIGHT_CONTROLLER;
    batch[length++] = 63u;
    length = TestAppend(batch, length, bad, badLength);
    length = TestAppend(batch, length, stats, statsLength);
    length = TestAppend(batch, length, rc, rcLength);
    TestFeedBatch(batch, length);
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u && snapshot.online != 0u &&
                       ElrsLinkDiag.length_error_count >= 1u &&
                       ElrsLinkDiag.crc_error_count >= 1u,
                   "噪声、非法长度和坏 CRC 后必须重同步到带扩展尾字段的合法帧")) return 0;

    TestReset(1u);
    TestChannels(channel);
    badLength = TestBuildRc(bad, CRSF_ADDRESS_FLIGHT_CONTROLLER, channel, 0u);
    bad[1] = (uint8_t)(bad[1] + 1u);
    statsLength = TestBuildStats(stats, CRSF_ADDRESS_FLIGHT_CONTROLLER, 85u, 0u);
    rcLength = TestBuildRc(rc, CRSF_ADDRESS_FLIGHT_CONTROLLER, channel, 0u);
    length = 0u;
    length = TestAppend(batch, length, bad, badLength);
    length = TestAppend(batch, length, stats, statsLength);
    length = TestAppend(batch, length, rc, rcLength);
    TestFeedBatch(batch, length);
    ElrsLinkGetStats(&diag);
    return TestCheck(ManualInputSnapshotRead(&snapshot) != 0u && snapshot.online != 0u &&
                         diag.crc_error_count == 1u && diag.rc_publish_count == 1u &&
                         diag.port_active != 0u,
                     "长度位损坏吞入下一帧同步字节后必须滑动保留后缀并恢复");
}

static int TestInvalidLinkQuality(void)
{
    ManualInputSnapshot snapshot;
    uint16_t channel[MANUAL_INPUT_CRSF_CHANNEL_COUNT];
    uint8_t rc[CRSF_FRAME_SIZE_MAX];
    uint8_t stats[CRSF_FRAME_SIZE_MAX];
    uint8_t batch[TEST_FRAME_BUFFER_SIZE];
    uint8_t rcLength;
    uint8_t statsLength;
    uint16_t length = 0u;

    TestReset(1u);
    TestChannels(channel);
    rcLength = TestBuildRc(rc, CRSF_ADDRESS_FLIGHT_CONTROLLER, channel, 0u);
    statsLength = TestBuildStats(stats, CRSF_ADDRESS_FLIGHT_CONTROLLER, 80u, 0u);
    length = TestAppend(batch, length, stats, statsLength);
    length = TestAppend(batch, length, rc, rcLength);
    TestFeedBatch(batch, length);

    statsLength = TestBuildStats(stats, CRSF_ADDRESS_FLIGHT_CONTROLLER, 101u, 0u);
    length = 0u;
    length = TestAppend(batch, length, stats, statsLength);
    length = TestAppend(batch, length, rc, rcLength);
    TestFeedBatch(batch, length);
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u && snapshot.online == 0u &&
                       ElrsLinkDiag.state == ELRS_LINK_STATE_DOWN,
                   "LQ>100 的 CRC 正确统计也必须清除旧健康证据")) return 0;

    statsLength = TestBuildStats(stats, CRSF_ADDRESS_FLIGHT_CONTROLLER, 80u, 0u);
    length = 0u;
    length = TestAppend(batch, length, stats, statsLength);
    length = TestAppend(batch, length, rc, rcLength);
    TestFeedBatch(batch, length);
    return TestCheck(ManualInputSnapshotRead(&snapshot) != 0u && snapshot.online != 0u,
                     "非法 LQ 后必须由后续批的新正统计和新 RC 恢复");
}

static int TestSessionGenerationBarrier(void)
{
    ManualInputSnapshot snapshot;
    uint16_t channel[MANUAL_INPUT_CRSF_CHANNEL_COUNT];
    uint8_t rc[CRSF_FRAME_SIZE_MAX];
    uint8_t stats[CRSF_FRAME_SIZE_MAX];
    uint8_t batch[TEST_FRAME_BUFFER_SIZE];
    uint8_t rcLength;
    uint8_t statsLength;
    uint16_t length = 0u;
    uint32_t oldSession;

    TestReset(1u);
    TestChannels(channel);
    statsLength = TestBuildStats(stats, CRSF_ADDRESS_FLIGHT_CONTROLLER, 80u, 0u);
    rcLength = TestBuildRc(rc, CRSF_ADDRESS_FLIGHT_CONTROLLER, channel, 0u);
    length = TestAppend(batch, length, stats, statsLength);
    length = TestAppend(batch, length, rc, rcLength);

    ElrsLinkExpireStats();
    ElrsLinkBatchBegin(ElrsLinkTransportEpoch);
    for (uint16_t i = 0u; i < length; i++)
    {
        ElrsLinkOnByte(batch[i]);
    }
    oldSession = ElrsLinkBatchSessionGen;
    if (!TestCheck(ElrsLinkPendingRc.valid != 0u,
                   "会话隔离测试必须先形成未提交 RC 候选")) return 0;
    ElrsLinkStop();
    ElrsLinkRxStart();
    ElrsLinkBatchCommit();
    if (!TestCheck(ElrsLinkSessionGen != oldSession &&
                       ManualInputSnapshotRead(&snapshot) != 0u && snapshot.online == 0u &&
                       ElrsLinkDiag.rc_publish_count == 0u,
                   "旧批候选跨 stop/restart 后不得在新会话复活")) return 0;

    ElrsLinkBatchBegin(ElrsLinkTransportEpoch);
    for (uint8_t i = 0u; i < 5u; i++)
    {
        ElrsLinkOnByte(stats[i]);
    }
    oldSession = ElrsLinkBatchSessionGen;
    ElrsLinkStop();
    ElrsLinkRxStart();
    for (uint8_t i = 5u; i < statsLength; i++)
    {
        ElrsLinkOnByte(stats[i]);
    }
    ElrsLinkBatchCommit();
    if (!TestCheck(ElrsLinkSessionGen != oldSession && ElrsCrsfPos == 0u &&
                       ManualInputSnapshotRead(&snapshot) != 0u && snapshot.online == 0u,
                   "旧会话半帧恢复后不得污染新会话解析游标")) return 0;

    TestFeedBatch(batch, length);
    return TestCheck(ManualInputSnapshotRead(&snapshot) != 0u && snapshot.online != 0u &&
                         ElrsLinkDiag.rc_publish_count == 1u,
                     "新会话必须重新经历正统计和新 RC 才能恢复");
}

static int TestTransportBatchBoundaries(void)
{
    ManualInputSnapshot snapshot;
    uint16_t channel[MANUAL_INPUT_CRSF_CHANNEL_COUNT];
    uint8_t rc[CRSF_FRAME_SIZE_MAX];
    uint8_t statsUp[CRSF_FRAME_SIZE_MAX];
    uint8_t statsDown[CRSF_FRAME_SIZE_MAX];
    uint8_t batch[TEST_FRAME_BUFFER_SIZE];
    uint8_t rcLength;
    uint8_t upLength;
    uint8_t downLength;
    uint16_t length = 0u;

    TestReset(0u);
    TestChannels(channel);
    upLength = TestBuildStats(statsUp, CRSF_ADDRESS_FLIGHT_CONTROLLER, 80u, 0u);
    rcLength = TestBuildRc(rc, CRSF_ADDRESS_FLIGHT_CONTROLLER, channel, 0u);
    length = TestAppend(batch, length, statsUp, upLength);
    length = TestAppend(batch, length, rc, rcLength);
    for (uint16_t i = 0u; i < length; i++)
    {
        ElrsLinkOnItByte(batch[i]);
    }
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u && snapshot.online == 0u &&
                       ElrsLinkDiag.rc_publish_count == 0u,
                   "逐字节中断只能入环，任务 drain 前不得解析或发布")) return 0;
    s_injectItOverflowOnTickRead = 1u;
    ElrsLinkItDrain();
    if (!TestCheck(ElrsLinkDiag.overflow_count == 1u &&
                       ElrsLinkDiag.state == ELRS_LINK_STATE_WAIT_STATS &&
                       ManualInputSnapshotRead(&snapshot) != 0u && snapshot.online == 0u,
                   "IT drain 末尾发现并发溢出时不得提交此前候选")) return 0;

    for (uint16_t i = 0u; i < length; i++)
    {
        ElrsLinkOnItByte(batch[i]);
    }
    ElrsLinkExpireStats();
    ElrsLinkItDrain();
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u && snapshot.online != 0u,
                   "IT 溢出后必须重新见到完整统计加 RC 才能恢复")) return 0;

    TestReset(1u);
    TestChannels(channel);
    upLength = TestBuildStats(statsUp, CRSF_ADDRESS_FLIGHT_CONTROLLER, 80u, 0u);
    rcLength = TestBuildRc(rc, CRSF_ADDRESS_FLIGHT_CONTROLLER, channel, 0u);
    length = 0u;
    length = TestAppend(batch, length, statsUp, upLength);
    length = TestAppend(batch, length, rc, rcLength);
    TestFeedBatch(batch, length);
    downLength = TestBuildStats(statsDown, CRSF_ADDRESS_FLIGHT_CONTROLLER, 0u, 0u);

    ElrsLinkDmaLastPos = (uint16_t)(ELRS_LINK_DMA_RX_BUF_SIZE - rcLength);
    memcpy(&ElrsLinkRx.dma[ElrsLinkDmaLastPos], rc, rcLength);
    memcpy(ElrsLinkRx.dma, statsDown, downLength);
    ElrsLinkExpireStats();
    ElrsLinkBatchBegin(ElrsLinkTransportEpoch);
    if (!TestCheck(ElrsLinkDmaProcessTo((uint16_t)ELRS_LINK_DMA_RX_BUF_SIZE) != 0u &&
                       ElrsLinkDmaProcessTo(downLength) != 0u,
                   "DMA wrap 两段必须在同一会话内完成解析")) return 0;
    ElrsLinkBatchCommit();
    return TestCheck(ManualInputSnapshotRead(&snapshot) != 0u && snapshot.online == 0u,
                     "DMA wrap 尾段 RC 后头段 LQ=0 必须作为同一批整体失效");
}

static int TestStatsTickWrap(void)
{
    ManualInputSnapshot snapshot;
    uint16_t channel[MANUAL_INPUT_CRSF_CHANNEL_COUNT];
    uint8_t rc[CRSF_FRAME_SIZE_MAX];
    uint8_t stats[CRSF_FRAME_SIZE_MAX];
    uint8_t batch[TEST_FRAME_BUFFER_SIZE];
    uint8_t rcLength;
    uint8_t statsLength;
    uint16_t length = 0u;

    TestReset(1u);
    TestChannels(channel);
    statsLength = TestBuildStats(stats, CRSF_ADDRESS_FLIGHT_CONTROLLER, 80u, 0u);
    rcLength = TestBuildRc(rc, CRSF_ADDRESS_FLIGHT_CONTROLLER, channel, 0u);
    length = TestAppend(batch, length, stats, statsLength);
    length = TestAppend(batch, length, rc, rcLength);
    s_tick = UINT32_MAX - 100u;
    TestFeedBatch(batch, length);
    s_tick = 149u;
    TestFeedBatch(rc, rcLength);
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u && snapshot.online != 0u,
                   "Link Statistics 时效跨 tick 回绕的 250ms 边界必须有效")) return 0;
    s_tick = 150u;
    TestFeedBatch(rc, rcLength);
    return TestCheck(ManualInputSnapshotRead(&snapshot) != 0u && snapshot.online == 0u,
                     "Link Statistics 时效跨 tick 回绕到 251ms 必须失效");
}

static int TestLinkDownDominatesBatch(void)
{
    ManualInputSnapshot snapshot;
    ManualInputState dbus;
    uint16_t channel[MANUAL_INPUT_CRSF_CHANNEL_COUNT];
    uint8_t rc[CRSF_FRAME_SIZE_MAX];
    uint8_t statsUp[CRSF_FRAME_SIZE_MAX];
    uint8_t statsDown[CRSF_FRAME_SIZE_MAX];
    uint8_t batch[TEST_FRAME_BUFFER_SIZE];
    uint8_t rcLength;
    uint8_t upLength;
    uint8_t downLength;
    uint16_t length = 0u;

    TestReset(1u);
    memset(&dbus, 0, sizeof(dbus));
    dbus.rc.ch[0] = 123;
    dbus.rc.s[0] = RC_SW_UP;
    dbus.rc.s[1] = RC_SW_UP;
    ManualInputUpdateSourceDetail(MANUAL_INPUT_SRC_DBUS,
                                  &dbus,
                                  MANUAL_INPUT_PROTOCOL_DBUS,
                                  0u,
                                  0u,
                                  NULL);
    TestChannels(channel);
    rcLength = TestBuildRc(rc, CRSF_ADDRESS_FLIGHT_CONTROLLER, channel, 0u);
    upLength = TestBuildStats(statsUp, CRSF_ADDRESS_FLIGHT_CONTROLLER, 90u, 0u);
    downLength = TestBuildStats(statsDown, CRSF_ADDRESS_FLIGHT_CONTROLLER, 0u, 0u);
    length = TestAppend(batch, length, statsUp, upLength);
    length = TestAppend(batch, length, rc, rcLength);
    TestFeedBatch(batch, length);

    length = 0u;
    length = TestAppend(batch, length, rc, rcLength);
    length = TestAppend(batch, length, statsDown, downLength);
    length = TestAppend(batch, length, statsUp, upLength);
    length = TestAppend(batch, length, rc, rcLength);
    TestFeedBatch(batch, length);
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u && snapshot.online != 0u &&
                       snapshot.activeSource == MANUAL_INPUT_SRC_DBUS &&
                       snapshot.manual.rc.ch[0] == 123 &&
                       ElrsLinkDiag.link_down_count == 1u,
                   "同批任意 LQ=0 必须压过全部 RC，并只回退到健康 DBUS")) return 0;

    TestFeedBatch(rc, rcLength);
    return TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                         snapshot.activeSource == MANUAL_INPUT_SRC_ELRS,
                     "LQ=0 后必须等后续批的新 RC 才能恢复 ELRS");
}

static int TestStatsTimeoutBoundary(void)
{
    ManualInputSnapshot snapshot;
    uint16_t channel[MANUAL_INPUT_CRSF_CHANNEL_COUNT];
    uint8_t rc[CRSF_FRAME_SIZE_MAX];
    uint8_t stats[CRSF_FRAME_SIZE_MAX];
    uint8_t batch[TEST_FRAME_BUFFER_SIZE];
    uint8_t rcLength;
    uint8_t statsLength;
    uint16_t length = 0u;

    TestReset(1u);
    TestChannels(channel);
    rcLength = TestBuildRc(rc, CRSF_ADDRESS_FLIGHT_CONTROLLER, channel, 0u);
    statsLength = TestBuildStats(stats, CRSF_ADDRESS_FLIGHT_CONTROLLER, 75u, 0u);
    length = TestAppend(batch, length, stats, statsLength);
    length = TestAppend(batch, length, rc, rcLength);
    TestFeedBatch(batch, length);

    s_tick = ELRS_LINK_STATS_TIMEOUT_MS;
    TestFeedBatch(rc, rcLength);
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u && snapshot.online != 0u,
                   "Link Statistics 的 250ms 边界仍应有效")) return 0;

    s_tick = ELRS_LINK_STATS_TIMEOUT_MS + 1u;
    TestFeedBatch(rc, rcLength);
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u && snapshot.online == 0u &&
                       ElrsLinkDiag.state == ELRS_LINK_STATE_DOWN &&
                       ElrsLinkDiag.stats_timeout_count == 1u,
                   "Link Statistics 到 251ms 必须撤销 ELRS")) return 0;

    s_tick++;
    length = 0u;
    length = TestAppend(batch, length, stats, statsLength);
    length = TestAppend(batch, length, rc, rcLength);
    TestFeedBatch(batch, length);
    return TestCheck(ManualInputSnapshotRead(&snapshot) != 0u && snapshot.online != 0u,
                     "超时撤销后的后续批可由新统计加新 RC 恢复");
}

static int TestTaskStatsDeadlineWait(void)
{
    ManualInputSnapshot snapshot;
    uint16_t channel[MANUAL_INPUT_CRSF_CHANNEL_COUNT];
    uint8_t rc[CRSF_FRAME_SIZE_MAX];
    uint8_t stats[CRSF_FRAME_SIZE_MAX];
    uint8_t batch[TEST_FRAME_BUFFER_SIZE];
    uint8_t rcLength;
    uint8_t statsLength;
    uint16_t length = 0u;

    TestReset(1u);
    TestChannels(channel);
    rcLength = TestBuildRc(rc, CRSF_ADDRESS_FLIGHT_CONTROLLER, channel, 0u);
    statsLength = TestBuildStats(stats, CRSF_ADDRESS_FLIGHT_CONTROLLER, 75u, 0u);
    length = TestAppend(batch, length, stats, statsLength);
    length = TestAppend(batch, length, rc, rcLength);
    TestFeedBatch(batch, length);

    ElrsLinkTaskRunOnce();
    if (!TestCheck(s_lastWaitTicks == ELRS_LINK_STATS_TIMEOUT_MS + 1u &&
                       s_tick == ELRS_LINK_STATS_TIMEOUT_MS + 1u &&
                       ElrsLinkDiag.state == ELRS_LINK_STATE_DOWN &&
                       ElrsLinkDiag.stats_timeout_count == 1u &&
                       ManualInputSnapshotRead(&snapshot) != 0u && snapshot.online == 0u,
                   "任务完全静默时必须直接等到 251ms 并撤销 ELRS")) return 0;

    TestReset(1u);
    TestChannels(channel);
    length = 0u;
    length = TestAppend(batch, length, stats, statsLength);
    length = TestAppend(batch, length, rc, rcLength);
    TestFeedBatch(batch, length);
    s_waitNotifyScheduled = 1u;
    s_waitNotifyTick = ELRS_LINK_STATS_TIMEOUT_MS - 1u;
    s_waitNotifyBits = ELRS_LINK_NOTIFY_RX;

    ElrsLinkTaskRunOnce();
    if (!TestCheck(s_tick == ELRS_LINK_STATS_TIMEOUT_MS - 1u &&
                       s_lastWaitTicks == ELRS_LINK_STATS_TIMEOUT_MS + 1u &&
                       ElrsLinkDiag.state == ELRS_LINK_STATE_UP,
                   "249ms 的通知应提前唤醒任务且不得提前撤销")) return 0;

    ElrsLinkTaskRunOnce();
    if (!TestCheck(s_lastWaitTicks == 2u &&
                       s_tick == ELRS_LINK_STATS_TIMEOUT_MS + 1u &&
                       ElrsLinkDiag.state == ELRS_LINK_STATE_DOWN &&
                       ElrsLinkDiag.stats_timeout_count == 1u &&
                       ManualInputSnapshotRead(&snapshot) != 0u && snapshot.online == 0u,
                   "249ms 通知后必须只等剩余 2 tick，并在 251ms 撤销 ELRS")) return 0;

    TestReset(1u);
    TestChannels(channel);
    length = 0u;
    length = TestAppend(batch, length, stats, statsLength);
    length = TestAppend(batch, length, rc, rcLength);
    TestFeedBatch(batch, length);
    s_auxBaudrate = 0u;

    ElrsLinkTaskRunOnce();
    return TestCheck(s_tick == ELRS_LINK_STATS_TIMEOUT_MS + 1u &&
                         ElrsLinkDiag.state == ELRS_LINK_STATE_DOWN &&
                         ElrsLinkDiag.stats_timeout_count == 1u,
                     "端口波特率异常时也必须在返回前按 251ms 截止期撤销 ELRS");
}

static int TestManualInputGuardedCommit(void)
{
    ManualInputSnapshot beforeSnapshot;
    ManualInputSnapshot afterSnapshot;
    ManualInputState decoded;
    ManualInputSrcState sourceBefore;
    ManualInputCrsfState crsfBefore;
    uint16_t channel[MANUAL_INPUT_CRSF_CHANNEL_COUNT];
    uint32_t sourceSeq;
    uint32_t updateSeq;
    uint32_t dirtySeq;
    uint32_t setSourceCount;
    uint32_t logCount;
    uint8_t refreshDirty;

    TestReset(1u);
    TestChannels(channel);
    memset(&decoded, 0, sizeof(decoded));
    if (!TestCheck(ManualInputCrsfDecode(channel, &g_config.input, &decoded) != 0u &&
                       ManualInputSnapshotRead(&beforeSnapshot) != 0u,
                   "guard 回归必须先构造合法 ELRS 输入")) return 0;
    crsfBefore = manual_crsf;
    sourceBefore = manual_src[MANUAL_INPUT_SRC_ELRS];
    sourceSeq = ManualInputSourceSeq;
    updateSeq = manual_src[MANUAL_INPUT_SRC_ELRS].update_seq;
    dirtySeq = ManualInputDirtySeq;
    setSourceCount = g_remote_control_set_source_cnt;
    logCount = s_sdLogCount;
    refreshDirty = ManualInputRefreshDirty;

    if (!TestCheck(ManualInputUpdateElrsChannelsGuarded(&decoded,
                                                        channel,
                                                        TestGuardReject,
                                                        NULL) == 0u &&
                       ManualInputSnapshotRead(&afterSnapshot) != 0u &&
                       ManualInputSourceSeq == sourceSeq &&
                       manual_src[MANUAL_INPUT_SRC_ELRS].update_seq == updateSeq &&
                       ManualInputDirtySeq == dirtySeq &&
                       g_remote_control_set_source_cnt == setSourceCount &&
                       memcmp(&manual_src[MANUAL_INPUT_SRC_ELRS],
                              &sourceBefore,
                              sizeof(sourceBefore)) == 0 &&
                       memcmp(&manual_crsf, &crsfBefore, sizeof(manual_crsf)) == 0 &&
                       ManualInputRefreshDirty == refreshDirty &&
                       afterSnapshot.publishSeq == beforeSnapshot.publishSeq &&
                       s_sdLogCount == logCount,
                   "guard 失配时来源、原始通道、序号、快照和日志必须完全不变")) return 0;

    if (!TestCheck(ManualInputUpdateElrsChannelsGuarded(&decoded,
                                                        channel,
                                                        TestGuardAccept,
                                                        NULL) != 0u,
                   "guard 成功时必须允许一次 ELRS 条件提交")) return 0;
    return TestCheck(ManualInputSourceSeq == ManualInputSeqNext(sourceSeq) &&
                         manual_src[MANUAL_INPUT_SRC_ELRS].update_seq == ManualInputSourceSeq &&
                         g_remote_control_set_source_cnt == setSourceCount + 1u &&
                         manual_crsf.valid != 0u &&
                         memcmp(manual_crsf.channel, channel, sizeof(manual_crsf.channel)) == 0,
                     "guard 成功只能写入一次来源和同代 CRSF 原始通道");
}

static int TestStatsSessionWriteBarrier(void)
{
    ManualInputSnapshot snapshot;
    uint16_t channel[MANUAL_INPUT_CRSF_CHANNEL_COUNT];
    uint8_t rc[CRSF_FRAME_SIZE_MAX];
    uint8_t stats[CRSF_FRAME_SIZE_MAX];
    uint8_t batch[TEST_FRAME_BUFFER_SIZE];
    uint8_t rcLength;
    uint8_t statsLength;
    uint16_t length = 0u;
    uint32_t oldSession;

    TestReset(1u);
    TestChannels(channel);
    rcLength = TestBuildRc(rc, CRSF_ADDRESS_FLIGHT_CONTROLLER, channel, 0u);
    statsLength = TestBuildStats(stats, CRSF_ADDRESS_FLIGHT_CONTROLLER, 80u, 0u);
    length = TestAppend(batch, length, stats, statsLength);
    length = TestAppend(batch, length, rc, rcLength);
    oldSession = ElrsLinkSessionGen;
    TestQueueDmaBatch(batch, length);
    s_injectRestartDuringStatsTickRead = 1u;
    ElrsLinkTaskRunOnce();

    if (!TestCheck(ElrsLinkSessionGen != oldSession &&
                       ElrsLinkLastStatsSessionGen == 0u &&
                       ElrsLinkDiag.link_stats_count == 0u &&
                       ElrsLinkDiag.uplink_lq == 0u &&
                       ElrsLinkDiag.state == ELRS_LINK_STATE_WAIT_STATS &&
                       ElrsLinkDiag.rc_publish_count == 0u &&
                       ManualInputSnapshotRead(&snapshot) != 0u && snapshot.online == 0u,
                   "HandleStats 写回前 stop/start 时旧统计不得落入新会话")) return 0;

    TestQueueDmaBatch(batch, length);
    ElrsLinkTaskRunOnce();
    return TestCheck(ElrsLinkLastStatsSessionGen == ElrsLinkSessionGen &&
                         ElrsLinkDiag.state == ELRS_LINK_STATE_UP &&
                         ElrsLinkDiag.rc_publish_count == 1u &&
                         ManualInputSnapshotRead(&snapshot) != 0u && snapshot.online != 0u,
                     "会话竞态后必须由新会话统计加 RC 重新恢复");
}

static int TestTransportEpochBarriers(void)
{
    ManualInputSnapshot snapshot;
    uint16_t channel[MANUAL_INPUT_CRSF_CHANNEL_COUNT];
    uint8_t rc[CRSF_FRAME_SIZE_MAX];
    uint8_t statsUp[CRSF_FRAME_SIZE_MAX];
    uint8_t statsDown[CRSF_FRAME_SIZE_MAX];
    uint8_t batch[TEST_FRAME_BUFFER_SIZE];
    uint8_t rcLength;
    uint8_t upLength;
    uint8_t downLength;
    uint16_t length;
    uint32_t invalidateGen;
    uint32_t publishCount;
    uint32_t statsCount;

    TestReset(1u);
    TestChannels(channel);
    rcLength = TestBuildRc(rc, CRSF_ADDRESS_FLIGHT_CONTROLLER, channel, 0u);
    upLength = TestBuildStats(statsUp, CRSF_ADDRESS_FLIGHT_CONTROLLER, 80u, 0u);
    downLength = TestBuildStats(statsDown, CRSF_ADDRESS_FLIGHT_CONTROLLER, 0u, 0u);
    length = 0u;
    length = TestAppend(batch, length, statsUp, upLength);
    length = TestAppend(batch, length, rc, rcLength);
    TestFeedBatch(batch, length);
    invalidateGen = manual_src[MANUAL_INPUT_SRC_ELRS].invalidate_gen;
    ElrsLinkOnRxEvent(0u, BSP_AUX_LINK_RXEVENT_TC);
    ElrsLinkOnRxEvent(0u, BSP_AUX_LINK_RXEVENT_TC);
    ElrsLinkTaskRunOnce();
    if (!TestCheck(ElrsLinkDiag.overflow_count == 1u &&
                       ElrsLinkDiag.state == ELRS_LINK_STATE_WAIT_STATS &&
                       manual_src[MANUAL_INPUT_SRC_ELRS].invalidate_gen ==
                           ManualInputSeqNext(invalidateGen) &&
                       ManualInputSnapshotRead(&snapshot) != 0u && snapshot.online == 0u,
                   "真实任务 multi-wrap 分支必须重置会话并只撤销一次旧来源")) return 0;

    TestReset(1u);
    TestFeedBatch(batch, length);
    invalidateGen = manual_src[MANUAL_INPUT_SRC_ELRS].invalidate_gen;
    publishCount = ElrsLinkDiag.rc_publish_count;
    statsCount = ElrsLinkDiag.link_stats_count;
    TestQueueDmaBatch(batch, length);
    s_injectDmaTcDuringBatchTickRead = 1u;
    ElrsLinkTaskRunOnce();
    if (!TestCheck(ElrsLinkDiag.overflow_count == 1u &&
                       ElrsLinkDiag.link_stats_count == statsCount &&
                       ElrsLinkDiag.rc_publish_count == publishCount &&
                       ElrsLinkDiag.state == ELRS_LINK_STATE_WAIT_STATS &&
                       manual_src[MANUAL_INPUT_SRC_ELRS].invalidate_gen ==
                           ManualInputSeqNext(invalidateGen) &&
                       ManualInputSnapshotRead(&snapshot) != 0u && snapshot.online == 0u,
                   "DMA 解析中 TC 改变运输代时不得写统计或发布，并只撤销一次")) return 0;

    TestReset(1u);
    invalidateGen = manual_src[MANUAL_INPUT_SRC_ELRS].invalidate_gen;
    TestQueueDmaBatch(batch, length);
    s_injectDmaTcOnSdLogWrite = 1u;
    ElrsLinkTaskRunOnce();
    if (!TestCheck(ElrsLinkDiag.overflow_count == 1u &&
                       ElrsLinkDiag.rc_publish_count == 0u &&
                       ElrsLinkDiag.state == ELRS_LINK_STATE_WAIT_STATS &&
                       manual_src[MANUAL_INPUT_SRC_ELRS].invalidate_gen ==
                           ManualInputSeqNext(invalidateGen) &&
                       ManualInputSnapshotRead(&snapshot) != 0u && snapshot.online == 0u,
                   "ManualInput 发布中 TC 必须在 post-check 撤销且只推进一次失效代")) return 0;

    TestReset(1u);
    TestFeedBatch(batch, length);
    invalidateGen = manual_src[MANUAL_INPUT_SRC_ELRS].invalidate_gen;
    TestQueueDmaBatch(statsDown, downLength);
    s_injectDmaTcOnBaudRead = 1u;
    ElrsLinkTaskRunOnce();
    return TestCheck(ElrsLinkDiag.overflow_count == 1u &&
                         ElrsLinkDiag.state == ELRS_LINK_STATE_WAIT_STATS &&
                         manual_src[MANUAL_INPUT_SRC_ELRS].invalidate_gen ==
                             ManualInputSeqNext(invalidateGen) &&
                         ManualInputSnapshotRead(&snapshot) != 0u && snapshot.online == 0u,
                     "LQ=0 force 批预检后 TC 也必须统一中止并只撤销一次");
}

static int TestItDrainNormalAppend(void)
{
    ManualInputSnapshot snapshot;
    uint16_t channel[MANUAL_INPUT_CRSF_CHANNEL_COUNT];
    uint8_t rc[CRSF_FRAME_SIZE_MAX];
    uint8_t stats[CRSF_FRAME_SIZE_MAX];
    uint8_t batch[TEST_FRAME_BUFFER_SIZE];
    uint8_t rcLength;
    uint8_t statsLength;
    uint16_t length = 0u;

    TestReset(0u);
    TestChannels(channel);
    rcLength = TestBuildRc(rc, CRSF_ADDRESS_FLIGHT_CONTROLLER, channel, 0u);
    statsLength = TestBuildStats(stats, CRSF_ADDRESS_FLIGHT_CONTROLLER, 80u, 0u);
    length = TestAppend(batch, length, stats, statsLength);
    length = TestAppend(batch, length, rc, rcLength);
    for (uint16_t i = 0u; i < length; i++) ElrsLinkOnItByte(batch[i]);
    s_injectItAppendData = rc;
    s_injectItAppendLength = rcLength;

    ElrsLinkTaskRunOnce();
    if (!TestCheck(ElrsLinkDiag.overflow_count == 0u &&
                       ElrsLinkDiag.rc_publish_count == 1u &&
                       ElrsLinkItRxTail != ElrsLinkItRxHead &&
                       (s_taskNotifyBits & ELRS_LINK_NOTIFY_RX) != 0u,
                   "IT drain 中正常追加必须留给下一批并可靠自通知")) return 0;

    ElrsLinkTaskRunOnce();
    return TestCheck(ElrsLinkDiag.overflow_count == 0u &&
                         ElrsLinkDiag.rc_publish_count == 2u &&
                         ElrsLinkItRxTail == ElrsLinkItRxHead &&
                         ManualInputSnapshotRead(&snapshot) != 0u && snapshot.online != 0u,
                     "IT drain 期间追加的无溢出字节不得饿死或误拒");
}

static int TestUartErrorEpochBarrier(void)
{
    ManualInputSnapshot snapshot;
    uint16_t channel[MANUAL_INPUT_CRSF_CHANNEL_COUNT];
    uint8_t rc[CRSF_FRAME_SIZE_MAX];
    uint8_t stats[CRSF_FRAME_SIZE_MAX];
    uint8_t batch[TEST_FRAME_BUFFER_SIZE];
    uint8_t rcLength;
    uint8_t statsLength;
    uint16_t length = 0u;
    uint32_t invalidateGen;
    uint32_t publishCount;
    uint32_t statsCount;

    TestReset(1u);
    TestChannels(channel);
    rcLength = TestBuildRc(rc, CRSF_ADDRESS_FLIGHT_CONTROLLER, channel, 0u);
    statsLength = TestBuildStats(stats, CRSF_ADDRESS_FLIGHT_CONTROLLER, 80u, 0u);
    length = TestAppend(batch, length, stats, statsLength);
    length = TestAppend(batch, length, rc, rcLength);
    TestFeedBatch(batch, length);
    invalidateGen = manual_src[MANUAL_INPUT_SRC_ELRS].invalidate_gen;
    publishCount = ElrsLinkDiag.rc_publish_count;
    statsCount = ElrsLinkDiag.link_stats_count;
    TestQueueDmaBatch(batch, length);
    s_injectUartErrorDuringBatchTickRead = 1u;
    ElrsLinkTaskRunOnce();
    if (!TestCheck(ElrsLinkDmaRestartReq != 0u &&
                       ElrsLinkDiag.overflow_count == 0u &&
                       ElrsLinkDiag.link_stats_count == statsCount &&
                       ElrsLinkDiag.rc_publish_count == publishCount &&
                       ElrsLinkDiag.state == ELRS_LINK_STATE_WAIT_STATS &&
                       manual_src[MANUAL_INPUT_SRC_ELRS].invalidate_gen ==
                           ManualInputSeqNext(invalidateGen),
                   "解析中 UART error 必须推进运输代并拒绝旧批统计和 RC")) return 0;

    TestReset(1u);
    invalidateGen = manual_src[MANUAL_INPUT_SRC_ELRS].invalidate_gen;
    TestQueueDmaBatch(batch, length);
    s_injectUartErrorOnSdLogWrite = 1u;
    ElrsLinkTaskRunOnce();
    if (!TestCheck(ElrsLinkDmaRestartReq != 0u &&
                       ElrsLinkDiag.overflow_count == 0u &&
                       ElrsLinkDiag.rc_publish_count == 0u &&
                       ElrsLinkDiag.state == ELRS_LINK_STATE_WAIT_STATS &&
                       manual_src[MANUAL_INPUT_SRC_ELRS].invalidate_gen ==
                           ManualInputSeqNext(invalidateGen) &&
                       ManualInputSnapshotRead(&snapshot) != 0u && snapshot.online == 0u,
                   "发布中 UART error 必须回滚来源、不得提交 UP 且只撤销一次")) return 0;

    ElrsLinkTaskRunOnce();
    return TestCheck(ElrsLinkDmaRestartReq == 0u &&
                         ElrsLinkDiag.restart_count == 1u &&
                         ElrsLinkDiag.port_active != 0u &&
                         ElrsLinkDiag.state == ELRS_LINK_STATE_WAIT_STATS,
                     "运输代中止后下一次任务处理必须完成 UART 重启并等待新统计");
}

TickType_t xTaskGetTickCount(void)
{
    if (s_injectRestartDuringStatsTickRead != 0u && ElrsLinkBatchActive != 0u)
    {
        s_injectRestartDuringStatsTickRead = 0u;
        ElrsLinkStop();
        ElrsLinkRxStart();
    }
    if (s_injectDmaTcDuringBatchTickRead != 0u && ElrsLinkBatchActive != 0u)
    {
        s_injectDmaTcDuringBatchTickRead = 0u;
        ElrsLinkOnRxEvent(0u, BSP_AUX_LINK_RXEVENT_TC);
    }
    if (s_injectUartErrorDuringBatchTickRead != 0u && ElrsLinkBatchActive != 0u)
    {
        s_injectUartErrorDuringBatchTickRead = 0u;
        (void)ElrsLinkOnUartError();
    }
    if (s_injectItAppendData != NULL && ElrsLinkBatchActive != 0u)
    {
        const uint8_t *data = s_injectItAppendData;
        const uint16_t length = s_injectItAppendLength;
        s_injectItAppendData = NULL;
        s_injectItAppendLength = 0u;
        for (uint16_t i = 0u; i < length; i++)
        {
            ElrsLinkOnItByte(data[i]);
        }
    }
    if (s_injectItOverflowOnTickRead != 0u)
    {
        s_injectItOverflowOnTickRead = 0u;
        if (ElrsLinkItRxOverflow == 0u)
        {
            ElrsLinkItRxOverflow = 1u;
            ElrsLinkTransportEpoch = ElrsLinkSeqNext(ElrsLinkTransportEpoch);
        }
    }
    return s_tick;
}

TaskHandle_t xTaskGetCurrentTaskHandle(void)
{
    return (TaskHandle_t)(uintptr_t)1u;
}

BaseType_t xTaskNotify(TaskHandle_t task, uint32_t value, eNotifyAction action)
{
    (void)task;
    (void)action;
    s_taskNotifyBits |= value;
    return pdTRUE;
}

BaseType_t xTaskNotifyWait(uint32_t clearOnEntry,
                           uint32_t clearOnExit,
                           uint32_t *value,
                           TickType_t waitTicks)
{
    (void)clearOnEntry;
    (void)clearOnExit;
    s_lastWaitTicks = waitTicks;
    if (s_taskNotifyBits != 0u)
    {
        if (value != NULL) *value = s_taskNotifyBits;
        s_taskNotifyBits = 0u;
        return pdTRUE;
    }
    if (s_waitNotifyScheduled != 0u)
    {
        const TickType_t until_notify = (TickType_t)(s_waitNotifyTick - s_tick);
        if (until_notify <= waitTicks)
        {
            s_tick = s_waitNotifyTick;
            s_waitNotifyScheduled = 0u;
            if (value != NULL) *value = s_waitNotifyBits;
            return pdTRUE;
        }
    }
    s_tick = (TickType_t)(s_tick + waitTicks);
    if (value != NULL) *value = 0u;
    return pdFALSE;
}

BaseType_t xTaskNotifyFromISR(TaskHandle_t task,
                              uint32_t value,
                              eNotifyAction action,
                              BaseType_t *higherPriorityTaskWoken)
{
    (void)task;
    (void)action;
    s_taskNotifyBits |= value;
    if (higherPriorityTaskWoken != NULL) *higherPriorityTaskWoken = pdFALSE;
    return pdTRUE;
}

void ManualInputTestEnterCritical(void)
{
    s_criticalDepth++;
}

void ManualInputTestExitCritical(void)
{
    if (s_criticalDepth > 0) s_criticalDepth--;
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
                                 uint32_t autoReload,
                                 void *timerId,
                                 TimerCallbackFunction_t callback,
                                 StaticTimer_t *buffer)
{
    (void)name;
    (void)period;
    (void)autoReload;
    (void)timerId;
    (void)callback;
    return (TimerHandle_t)buffer;
}

uint32_t xTimerStart(TimerHandle_t timer, TickType_t waitTicks)
{
    (void)timer;
    (void)waitTicks;
    return pdTRUE;
}

uint8_t BspKeyReadRawDown(void)
{
    return 0u;
}

void DetectHook(uint8_t toe)
{
    (void)toe;
}

uint8_t DetectIsError(uint8_t toe)
{
    (void)toe;
    return 1u;
}

uint8_t SdLogIsActive(void)
{
    return 0u;
}

void SdLogWrite(uint16_t tag, const void *payload, uint16_t payloadSize)
{
    (void)payload;
    (void)payloadSize;
    if (s_injectDmaTcOnSdLogWrite != 0u && ElrsLinkBatchActive != 0u)
    {
        s_injectDmaTcOnSdLogWrite = 0u;
        ElrsLinkOnRxEvent(0u, BSP_AUX_LINK_RXEVENT_TC);
    }
    if (s_injectUartErrorOnSdLogWrite != 0u && ElrsLinkBatchActive != 0u)
    {
        s_injectUartErrorOnSdLogWrite = 0u;
        (void)ElrsLinkOnUartError();
    }
    s_sdLogCount++;
    if (tag == SDLOG_TAG_RC_CRSF) s_rcLogCount++;
}

void BspRcSbusInit(void)
{
}

void RC_restart(uint16_t dmaBufferLength)
{
    (void)dmaBufferLength;
}

void BspAuxLinkSetRxEventCb(BspAuxLinkRxEventCb callback)
{
    (void)callback;
}

void BspAuxLinkSetRxByteCb(BspAuxLinkRxByteCb callback)
{
    (void)callback;
}

void BspAuxLinkSetErrorCb(BspAuxLinkErrorCb callback)
{
    (void)callback;
}

uint32_t BspAuxLinkGetBaudrate(void)
{
    if (s_injectDmaTcOnBaudRead != 0u && ElrsLinkBatchActive != 0u)
    {
        s_injectDmaTcOnBaudRead = 0u;
        ElrsLinkOnRxEvent(0u, BSP_AUX_LINK_RXEVENT_TC);
    }
    if (s_injectUartErrorOnBaudRead != 0u && ElrsLinkBatchActive != 0u)
    {
        s_injectUartErrorOnBaudRead = 0u;
        (void)ElrsLinkOnUartError();
    }
    return s_auxBaudrate;
}

uint8_t BspAuxLinkRxHasDma(void)
{
    return s_auxDmaAvailable;
}

int BspAuxLinkRxToIdleDmaStart(uint8_t *buffer, uint16_t length)
{
    s_auxDmaBuffer = buffer;
    s_auxDmaLength = length;
    return 0;
}

int BspAuxLinkRxItStart(void)
{
    return 0;
}

void BspAuxLinkRxItStop(void)
{
    s_auxDmaBuffer = NULL;
    s_auxDmaLength = 0u;
}

void WatchTaskWait(uint8_t taskId)
{
    (void)taskId;
}

void WatchTaskBeat(uint8_t taskId)
{
    (void)taskId;
}

void WatchTaskError(uint8_t taskId)
{
    (void)taskId;
}

void WatchTaskTimeout(uint8_t taskId)
{
    (void)taskId;
}

int main(void)
{
    if (!TestStrictStatsAndOrder()) return 1;
    if (!TestLinkDownDominatesBatch()) return 1;
    if (!TestStatsTimeoutBoundary()) return 1;
    if (!TestTaskStatsDeadlineWait()) return 1;
    if (!TestManualInputGuardedCommit()) return 1;
    if (!TestMappedChannelValidation()) return 1;
    if (!TestParserValidationAndResync()) return 1;
    if (!TestInvalidLinkQuality()) return 1;
    if (!TestSessionGenerationBarrier()) return 1;
    if (!TestStatsSessionWriteBarrier()) return 1;
    if (!TestTransportBatchBoundaries()) return 1;
    if (!TestTransportEpochBarriers()) return 1;
    if (!TestItDrainNormalAppend()) return 1;
    if (!TestUartErrorEpochBarrier()) return 1;
    if (!TestStatsTickWrap()) return 1;
    if (!TestCheck(s_criticalDepth == 0,
                   "ELRS 回归结束时所有临界区必须成对退出")) return 1;
    (void)puts("PASS: ELRS reliable-input regression");
    return 0;
}
