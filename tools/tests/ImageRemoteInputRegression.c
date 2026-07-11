/* 图传输入真实解析回归：覆盖自定义协议、VT13、来源合并和过期安全化。 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "ControlInput.h"
#include "ImageRemoteLink.h"
#include "ManualInputSnapshot.h"
#include "RobotConfig.h"
#include "timers.h"

Config g_config;

#define TEST_IMAGE_REMOTE_FRAME_MAX_SIZE 64u

static TickType_t s_tick;
static int s_criticalDepth;
static uint8_t *s_auxDmaBuffer;
static uint16_t s_auxDmaLength;
static uint16_t s_auxWritePos;
static uint8_t s_auxDmaAvailable;
static int s_auxDmaStartResult;
static uint32_t s_auxBaudrate;
static BspAuxLinkRxEventCb s_auxEventCallback;
static BspAuxLinkRxByteCb s_auxByteCallback;
static BspAuxLinkErrorCb s_auxErrorCallback;
static uint8_t s_publishInjectPending;
static uint8_t s_publishInjectFailed;
static uint8_t s_publishInjectFrame[TEST_IMAGE_REMOTE_FRAME_MAX_SIZE];
static uint16_t s_publishInjectLength;
static uint8_t s_sdLogActive;
static uint32_t s_rawLogCount;
static sdlog_manual_input_raw_t s_lastRawLog;

#include "ControlInput.c"
#include "ManualInput.c"
#include "Crc8Crc16.c"
#include "ImageRemoteLink.c"

static void TestUpdateSource(uint8_t source, const ManualInputState *input)
{
    ManualInputUpdateSourceDetail(source,
                                  input,
                                  MANUAL_INPUT_PROTOCOL_NONE,
                                  0u,
                                  0u,
                                  NULL);
}

#define ManualInputUpdateSource TestUpdateSource

static int TestCheck(int condition, const char *message)
{
    if (!condition)
    {
        (void)fprintf(stderr, "FAIL: %s\n", message);
        return 0;
    }
    return 1;
}

static void TestReset(uint16_t timeoutMs)
{
    ImageRemoteLinkStop();
    memset(&g_config, 0, sizeof(g_config));
    g_config.manual_input.active_source = MANUAL_INPUT_SRC_AUTO;
    g_config.manual_input.mix_mode = MANUAL_INPUT_MIX_SELECT_LATEST;
    g_config.manual_input.source_timeout_ms = timeoutMs;
    g_config.manual_input.semantics.GimbalSafePos = MANUAL_INPUT_SWITCH_POS_UP;
    g_config.manual_input.semantics.ChassisSafePos = MANUAL_INPUT_SWITCH_POS_UP;
    g_config.manual_input.semantics.ChassisFollowPos = MANUAL_INPUT_SWITCH_POS_MID;
    g_config.manual_input.semantics.ChassisSpinPos = MANUAL_INPUT_SWITCH_POS_DOWN;
    g_config.manual_input.semantics.ShootStopPos = MANUAL_INPUT_SWITCH_POS_UP;
    g_config.manual_input.semantics.ShootReadyPos = MANUAL_INPUT_SWITCH_POS_MID;
    g_config.manual_input.semantics.ShootFirePos = MANUAL_INPUT_SWITCH_POS_DOWN;
    g_config.manual_input.vt13.switch1_safe_value = 0u;
    g_config.manual_input.vt13.switch1_normal_value = 1u;
    g_config.manual_input.vt13.switch1_spin_value = 2u;
    g_config.manual_input.vt13.switch2_pause_pos = MANUAL_INPUT_SWITCH_POS_UP;
    g_config.manual_input.vt13.switch2_btn_l_pos = MANUAL_INPUT_SWITCH_POS_MID;
    g_config.manual_input.vt13.switch2_btn_r_pos = MANUAL_INPUT_SWITCH_POS_DOWN;
    for (uint32_t i = 0u; i < (uint32_t)INPUT_AXIS_COUNT; i++)
    {
        g_config.input.axis[i].rc_ch = (uint8_t)(i % 5u);
    }
    for (uint32_t i = 0u; i < (uint32_t)INPUT_SW_COUNT; i++)
    {
        g_config.input.sw[i].rc_sw = (uint8_t)(i % 2u);
    }

    s_tick = 0u;
    s_criticalDepth = 0;
    s_auxDmaBuffer = NULL;
    s_auxDmaLength = 0u;
    s_auxWritePos = 0u;
    s_auxDmaAvailable = 1u;
    s_auxDmaStartResult = 0;
    s_auxBaudrate = IMAGE_REMOTE_LINK_BAUD;
    s_auxEventCallback = NULL;
    s_auxByteCallback = NULL;
    s_auxErrorCallback = NULL;
    s_publishInjectPending = 0u;
    s_publishInjectFailed = 0u;
    memset(s_publishInjectFrame, 0, sizeof(s_publishInjectFrame));
    s_publishInjectLength = 0u;
    s_sdLogActive = 0u;
    s_rawLogCount = 0u;
    memset(&s_lastRawLog, 0, sizeof(s_lastRawLog));

    memset(&s_image_remote_state, 0, sizeof(s_image_remote_state));
    ImageRemoteLastRxTickMs = 0u;
    ImageRemoteFrameCnt = 0u;
    ImageRemoteControllerFrameCnt = 0u;
    ImageRemoteClientFrameCnt = 0u;
    ImageRemoteVt13FrameCnt = 0u;
    ImageRemoteCrcErrorCnt = 0u;
    ImageRemoteParseErrorCnt = 0u;
    ImageRemoteRestartCnt = 0u;
    ImageRemoteLastCmdId = 0u;
    ImageRemoteLastRangeMode = 0u;

    ManualInputInit();
    ImageRemoteLinkStart();
}

static int TestFeedFrame(const uint8_t *frame, uint16_t length)
{
    if (!TestCheck(frame != NULL && length != 0u,
                   "测试帧必须有效")) return 0;
    if (!TestCheck(s_auxDmaBuffer != NULL &&
                       s_auxDmaLength == (uint16_t)IMAGE_REMOTE_DMA_RX_BUF_SIZE &&
                       s_auxEventCallback != NULL,
                   "图传链路必须通过 DMA 事件入口启动")) return 0;
    if (!TestCheck((uint32_t)s_auxWritePos + length < s_auxDmaLength,
                   "测试帧不得越过 DMA 缓冲区")) return 0;

    memcpy(&s_auxDmaBuffer[s_auxWritePos], frame, length);
    s_auxWritePos = (uint16_t)(s_auxWritePos + length);
    s_auxEventCallback(s_auxWritePos, BSP_AUX_LINK_RXEVENT_IDLE);
    ImageRemoteLinkPoll();
    return 1;
}

static uint16_t TestBuildCustomPayloadFrame(uint8_t *frame,
                                            uint16_t commandId,
                                            const void *payload,
                                            uint16_t payloadLength)
{
    const uint16_t frameLength = (uint16_t)(payloadLength + 9u);

    memset(frame, 0, frameLength);
    frame[0] = IMAGE_REMOTE_FRAME_SOF;
    frame[1] = (uint8_t)(payloadLength & 0xFFu);
    frame[2] = (uint8_t)(payloadLength >> 8);
    frame[3] = 0x5Au;
    append_CRC8_check_sum(frame, 5u);
    frame[5] = (uint8_t)(commandId & 0xFFu);
    frame[6] = (uint8_t)(commandId >> 8);
    memcpy(&frame[7], payload, payloadLength);
    append_CRC16_check_sum(frame, frameLength);
    return frameLength;
}

static uint16_t TestBuildCustomFrame(uint8_t *frame,
                                     uint16_t commandId,
                                     const ImageRemoteRcPacket *packet)
{
    return TestBuildCustomPayloadFrame(frame,
                                       commandId,
                                       packet,
                                       (uint16_t)sizeof(*packet));
}

static void TestPackVt13Channels(uint8_t frame[IMAGE_REMOTE_VT13_FRAME_SIZE],
                                 const uint16_t channel[5])
{
    frame[2] = (uint8_t)(channel[0] & 0xFFu);
    frame[3] = (uint8_t)(((channel[0] >> 8) & 0x07u) |
                         ((channel[1] & 0x1Fu) << 3));
    frame[4] = (uint8_t)(((channel[1] >> 5) & 0x3Fu) |
                         ((channel[2] & 0x03u) << 6));
    frame[5] = (uint8_t)((channel[2] >> 2) & 0xFFu);
    frame[6] = (uint8_t)(((channel[2] >> 10) & 0x01u) |
                         ((channel[3] & 0x7Fu) << 1));
    frame[7] = (uint8_t)((channel[3] >> 7) & 0x0Fu);
    frame[8] = (uint8_t)((channel[4] & 0x7Fu) << 1);
    frame[9] = (uint8_t)((channel[4] >> 7) & 0x0Fu);
}

static void TestBuildVt13Frame(uint8_t frame[IMAGE_REMOTE_VT13_FRAME_SIZE],
                               const uint16_t channel[5],
                               uint8_t switch1,
                               uint8_t pause,
                               uint8_t buttonLeft,
                               uint8_t buttonRight,
                               uint8_t mouseLeft,
                               uint8_t mouseRight)
{
    memset(frame, 0, IMAGE_REMOTE_VT13_FRAME_SIZE);
    frame[0] = 0xA9u;
    frame[1] = 0x53u;
    TestPackVt13Channels(frame, channel);
    frame[7] |= (uint8_t)((switch1 & 0x03u) << 4);
    frame[7] |= (uint8_t)((pause & 0x01u) << 6);
    frame[7] |= (uint8_t)((buttonLeft & 0x01u) << 7);
    frame[8] |= (uint8_t)(buttonRight & 0x01u);
    frame[16] = (uint8_t)((mouseLeft & 0x01u) |
                          ((mouseRight & 0x01u) << 1));
    frame[17] = (uint8_t)(KEY_PRESSED_OFFSET_Q & 0xFFu);
    frame[18] = (uint8_t)(KEY_PRESSED_OFFSET_Q >> 8);
    append_CRC16_check_sum(frame, IMAGE_REMOTE_VT13_FRAME_SIZE);
}

static ImageRemoteRcPacket TestCustomPacket(int16_t channel0,
                                             int16_t channel1,
                                             uint8_t mouseButtons)
{
    ImageRemoteRcPacket packet;

    memset(&packet, 0, sizeof(packet));
    packet.magic0 = IMAGE_REMOTE_RC_MAGIC0;
    packet.magic1 = IMAGE_REMOTE_RC_MAGIC1;
    packet.version = IMAGE_REMOTE_RC_VERSION;
    packet.range_mode = IMAGE_REMOTE_RC_RANGE_DBUS;
    packet.ch[0] = channel0;
    packet.ch[1] = channel1;
    packet.sw[0] = RC_SW_MID;
    packet.sw[1] = RC_SW_DOWN;
    packet.mouse_x = 125;
    packet.mouse_y = -75;
    packet.mouse_btns = mouseButtons;
    packet.key_value = KEY_PRESSED_OFFSET_E;
    return packet;
}

static int TestCustomProtocolFailureAndExpiry(void)
{
    ManualInputSnapshot snapshot;
    ImageRemoteState imageState;
    sdlog_image_link_stats_t stats;
    uint8_t frame[IMAGE_REMOTE_RM_FRAME_MAX_SIZE];
    ImageRemoteRcPacket packet;
    uint16_t frameLength;
    uint32_t sourceSeq;

    TestReset(20u);
    g_config.manual_input.vt13.auto_aim_mouse_r_enable = 1u;
    g_config.manual_input.vt13.AuxFireMouseLEnable = 1u;
    s_tick = 2u;
    packet = TestCustomPacket(330, -660,
                              IMAGE_REMOTE_RC_BTN_LEFT | IMAGE_REMOTE_RC_BTN_RIGHT);
    frameLength = TestBuildCustomFrame(frame,
                                       IMAGE_REMOTE_CMD_CUSTOM_CONTROLLER_RX,
                                       &packet);
    if (!TestFeedFrame(frame, frameLength)) return 0;

    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                       snapshot.online != 0u &&
                       snapshot.activeSource == MANUAL_INPUT_SRC_IMAGE &&
                       snapshot.sourceProtocol == MANUAL_INPUT_PROTOCOL_IMAGE_CUSTOM &&
                       snapshot.sourceFlags == (MANUAL_INPUT_SOURCE_FLAG_AUTO_AIM |
                                                MANUAL_INPUT_SOURCE_FLAG_AUX_FIRE) &&
                       snapshot.manual.rc.ch[0] == 512 &&
                       snapshot.manual.rc.ch[1] == -1024 &&
                       snapshot.manual.mouse.press_l == 1u &&
                       snapshot.manual.mouse.press_r == 1u,
                   "自定义图传帧必须同代发布输入、协议和业务标志")) return 0;
    if (!TestCheck(ImageRemoteGetState(&imageState) &&
                       imageState.proto == SDLOG_MANUAL_INPUT_PROTO_IMAGE_CUSTOM &&
                       imageState.range_mode == SDLOG_MANUAL_INPUT_RANGE_CENTERED_660,
                   "图传诊断状态必须保留自定义协议结果")) return 0;
    ImageRemoteLinkGetStats(&stats);
    if (!TestCheck(stats.frame_count == 1u &&
                       stats.controller_frame_count == 1u &&
                       stats.client_frame_count == 0u &&
                       stats.vt13_frame_count == 0u &&
                       stats.last_cmd_id == IMAGE_REMOTE_CMD_CUSTOM_CONTROLLER_RX,
                   "自定义协议统计必须记录真实解析入口")) return 0;

    ManualInputStore.ready = 0u;
    memset(&snapshot, 0xA5, sizeof(snapshot));
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) == 0u &&
                       snapshot.online == 0u &&
                       snapshot.sourceProtocol == MANUAL_INPUT_PROTOCOL_NONE &&
                       snapshot.sourceFlags == 0u,
                   "统一快照读取失败时必须返回安全元数据")) return 0;
    ManualInputStore.ready = 1u;

    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u,
                   "恢复快照存储后必须仍可读取有效代")) return 0;
    sourceSeq = snapshot.sourceSeq;
    s_tick = 10u;
    frame[frameLength - 1u] ^= 0x5Au;
    if (!TestFeedFrame(frame, frameLength)) return 0;
    ImageRemoteLinkGetStats(&stats);
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                       snapshot.sourceSeq == sourceSeq &&
                       snapshot.sourceTickMs == 2u &&
                       stats.crc_error_count == 1u &&
                       stats.frame_count == 1u,
                   "损坏帧不得覆盖或刷新上一代有效图传输入")) return 0;

    s_tick = 23u;
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                       snapshot.online == 0u &&
                       snapshot.activeSource == MANUAL_INPUT_SRC_AUTO &&
                       snapshot.sourceProtocol == MANUAL_INPUT_PROTOCOL_NONE &&
                       snapshot.sourceFlags == 0u &&
                       snapshot.manual.rc.ch[0] == 0,
                   "图传读取持续失败并过期后必须清空协议、标志和控制值")) return 0;
    return 1;
}

static int TestInvalidCustomSwitchInvalidatesVt13(void)
{
    ManualInputSnapshot before;
    ManualInputSnapshot after;
    ManualInputState elrs;
    ImageRemoteState imageState;
    sdlog_image_link_stats_t stats;
    uint8_t customFrame[IMAGE_REMOTE_RM_FRAME_MAX_SIZE];
    uint8_t vt13Frame[IMAGE_REMOTE_VT13_FRAME_SIZE];
    const uint16_t channel[5] = {1024u, 1024u, 1024u, 1024u, 1024u};
    ImageRemoteRcPacket packet;
    uint16_t customFrameLength;

    TestReset(20u);
    s_sdLogActive = 1u;
    memset(&elrs, 0, sizeof(elrs));
    elrs.rc.ch[0] = 321;
    elrs.rc.s[0] = RC_SW_UP;
    elrs.rc.s[1] = RC_SW_MID;
    s_tick = 1u;
    ManualInputUpdateSource(MANUAL_INPUT_SRC_ELRS, &elrs);
    s_tick = 2u;
    TestBuildVt13Frame(vt13Frame, channel, 0u, 0u, 0u, 0u, 0u, 0u);
    if (!TestFeedFrame(vt13Frame, IMAGE_REMOTE_VT13_FRAME_SIZE)) return 0;
    if (!TestCheck(ManualInputSnapshotRead(&before) != 0u &&
                       before.online != 0u &&
                       before.sourceProtocol == MANUAL_INPUT_PROTOCOL_IMAGE_VT13 &&
                       before.manual.rc.s[0] == RC_SW_UP &&
                       before.manual.rc.s[1] == RC_SW_UP &&
                       s_rawLogCount == 1u,
                   "无按键 VT13 必须先建立双安全档基线")) return 0;

    packet = TestCustomPacket(0, 0, 0u);
    packet.sw[0] = 0u;
    packet.sw[1] = RC_SW_DOWN;
    customFrameLength = TestBuildCustomFrame(customFrame,
                                              IMAGE_REMOTE_CMD_CUSTOM_CONTROLLER_RX,
                                              &packet);
    s_tick = 10u;
    if (!TestFeedFrame(customFrame, customFrameLength)) return 0;
    ImageRemoteLinkGetStats(&stats);
    if (!TestCheck(ManualInputSnapshotRead(&after) != 0u &&
                       after.online != 0u &&
                       after.activeSource == MANUAL_INPUT_SRC_ELRS &&
                       after.sourceProtocol == MANUAL_INPUT_PROTOCOL_NONE &&
                       after.sourceSeq != before.sourceSeq &&
                       after.sourceTickMs == 1u &&
                       after.manual.rc.ch[0] == 321 &&
                       after.manual.rc.s[0] == RC_SW_UP &&
                       stats.parse_error_count == 1u &&
                       stats.crc_error_count == 0u &&
                       s_rawLogCount == 1u,
                   "CRC 正确但拨杆非法的 custom 帧必须撤销 VT13 且不得变成 DOWN")) return 0;
    if (!TestCheck(!ImageRemoteGetState(&imageState) &&
                       imageState.proto == SDLOG_MANUAL_INPUT_PROTO_IMAGE_VT13 &&
                       (uint8_t)imageState.s[0] == 0u,
                   "非法 custom 帧必须清有效位，同时保留最近协议供故障诊断")) return 0;

    s_tick = 22u;
    return TestCheck(ManualInputSnapshotRead(&after) != 0u &&
                         after.online == 0u &&
                         after.activeSource == MANUAL_INPUT_SRC_AUTO &&
                         after.sourceProtocol == MANUAL_INPUT_PROTOCOL_NONE &&
                         after.manual.rc.s[0] == RC_SW_UP,
                     "非法 custom 帧不得给 IMAGE 或备用来源续期");
}

static int TestFeedInvalidCustomPacket(const ImageRemoteRcPacket *invalidPacket,
                                       const char *message)
{
    ManualInputSnapshot snapshot;
    sdlog_image_link_stats_t stats;
    uint8_t frame[IMAGE_REMOTE_RM_FRAME_MAX_SIZE];
    ImageRemoteRcPacket validPacket;
    uint16_t frameLength;

    TestReset(100u);
    validPacket = TestCustomPacket(100, -100, 0u);
    frameLength = TestBuildCustomFrame(frame,
                                       IMAGE_REMOTE_CMD_CUSTOM_CONTROLLER_RX,
                                       &validPacket);
    if (!TestFeedFrame(frame, frameLength)) return 0;
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u && snapshot.online != 0u,
                   "非法字段测试必须先建立有效图传来源")) return 0;

    frameLength = TestBuildCustomFrame(frame,
                                       IMAGE_REMOTE_CMD_CUSTOM_CONTROLLER_RX,
                                       invalidPacket);
    if (!TestFeedFrame(frame, frameLength)) return 0;
    ImageRemoteLinkGetStats(&stats);
    return TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                         snapshot.online == 0u &&
                         snapshot.activeSource == MANUAL_INPUT_SRC_AUTO &&
                         stats.parse_error_count == 1u,
                     message);
}

static int TestInvalidCustomBusinessFields(void)
{
    ManualInputSnapshot snapshot;
    sdlog_image_link_stats_t stats;
    uint8_t frame[IMAGE_REMOTE_RM_FRAME_MAX_SIZE];
    ImageRemoteRcPacket packet;
    uint16_t frameLength;

    packet = TestCustomPacket(0, 0, 0u);
    packet.version++;
    if (!TestFeedInvalidCustomPacket(&packet,
                                     "RC magic 匹配但版本非法时必须立即撤销图传来源")) return 0;

    packet = TestCustomPacket(0, 0, 0u);
    packet.range_mode = 0xFFu;
    if (!TestFeedInvalidCustomPacket(&packet,
                                     "RC range_mode 非法时必须立即撤销图传来源")) return 0;

    packet = TestCustomPacket((int16_t)(IMAGE_REMOTE_RC_ABS_MAX_DBUS + 1), 0, 0u);
    if (!TestFeedInvalidCustomPacket(&packet,
                                     "RC 原始轴越界时不得截成满量程动作")) return 0;

    packet = TestCustomPacket(0, 0, 0x80u);
    if (!TestFeedInvalidCustomPacket(&packet,
                                     "RC 未定义鼠标位出现时必须立即撤销图传来源")) return 0;

    packet = TestCustomPacket(0, 0, 0u);
    packet.reserved[0] = 1u;
    if (!TestFeedInvalidCustomPacket(&packet,
                                     "RC 当前版本保留字段非零时必须立即撤销图传来源")) return 0;

    TestReset(100u);
    packet = TestCustomPacket(100, 0, 0u);
    frameLength = TestBuildCustomFrame(frame,
                                       IMAGE_REMOTE_CMD_CUSTOM_CONTROLLER_RX,
                                       &packet);
    if (!TestFeedFrame(frame, frameLength)) return 0;
    frameLength = TestBuildCustomPayloadFrame(frame,
                                              IMAGE_REMOTE_CMD_CUSTOM_CONTROLLER_RX,
                                              &packet,
                                              (uint16_t)(sizeof(packet) - 1u));
    if (!TestFeedFrame(frame, frameLength)) return 0;
    ImageRemoteLinkGetStats(&stats);
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                       snapshot.online == 0u &&
                       snapshot.activeSource == MANUAL_INPUT_SRC_AUTO &&
                       stats.parse_error_count == 1u,
                   "已知图传遥控命令的错误载荷长度必须立即撤销旧来源")) return 0;

    TestReset(100u);
    packet = TestCustomPacket(100, 0, 0u);
    frameLength = TestBuildCustomFrame(frame,
                                       IMAGE_REMOTE_CMD_CUSTOM_CONTROLLER_RX,
                                       &packet);
    if (!TestFeedFrame(frame, frameLength)) return 0;
    packet.magic0 = 'X';
    packet.magic1 = 'Y';
    frameLength = TestBuildCustomFrame(frame,
                                       IMAGE_REMOTE_CMD_CUSTOM_CONTROLLER_RX,
                                       &packet);
    if (!TestFeedFrame(frame, frameLength)) return 0;
    ImageRemoteLinkGetStats(&stats);
    return TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                         snapshot.online != 0u &&
                         snapshot.activeSource == MANUAL_INPUT_SRC_IMAGE &&
                         stats.parse_error_count == 0u,
                     "通用自定义通道上的非 RC magic 载荷不得误伤已有图传遥控");
}

static int TestInterruptFallbackDefersParsingToTask(void)
{
    ManualInputSnapshot snapshot;
    uint8_t frame[IMAGE_REMOTE_RM_FRAME_MAX_SIZE];
    ImageRemoteRcPacket packet;
    uint16_t frameLength;

    TestReset(100u);
    ImageRemoteLinkStop();
    s_auxDmaAvailable = 0u;
    ImageRemoteLinkStart();
    if (!TestCheck(s_auxByteCallback != NULL && s_auxDmaBuffer == NULL,
                   "无 DMA 板必须启动逐字节中断缓冲入口")) return 0;

    packet = TestCustomPacket(330, 0, 0u);
    frameLength = TestBuildCustomFrame(frame,
                                       IMAGE_REMOTE_CMD_CUSTOM_CONTROLLER_RX,
                                       &packet);
    for (uint16_t i = 0u; i < frameLength; i++)
    {
        s_auxByteCallback(frame[i]);
    }
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u && snapshot.online == 0u,
                   "逐字节 ISR 只能入环形缓冲，不得直接解析或发布")) return 0;
    ImageRemoteLinkPoll();
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                       snapshot.online != 0u &&
                       snapshot.activeSource == MANUAL_INPUT_SRC_IMAGE,
                   "任务轮询必须完整消费逐字节缓冲并发布图传输入")) return 0;

    ImageRemoteLinkStop();
    for (uint16_t i = 0u; i < frameLength; i++)
    {
        ImageRemoteLinkOnItByte(frame[i]);
    }
    ImageRemoteLinkPoll();
    return TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                         snapshot.online == 0u &&
                         snapshot.activeSource == MANUAL_INPUT_SRC_AUTO,
                     "链路停止后迟到的逐字节数据不得重新发布旧图传来源");
}

static int TestVt13FrozenConfigRemap(void)
{
    ManualInputSnapshot snapshot;
    ImageRemoteState imageState;
    uint8_t frame[IMAGE_REMOTE_VT13_FRAME_SIZE];
    const uint16_t channel[5] = {1536u, 512u, 1024u, 1280u, 768u};

    TestReset(50u);
    g_config.manual_input.vt13.auto_aim_pause_enable = 1u;
    g_config.manual_input.vt13.auto_aim_mouse_r_enable = 1u;
    g_config.manual_input.vt13.AuxFireBtnLEnable = 1u;
    g_config.manual_input.vt13.AuxFireMouseLEnable = 1u;
    s_sdLogActive = 1u;
    s_tick = 3u;
    TestBuildVt13Frame(frame, channel, 2u, 0u, 1u, 1u, 1u, 1u);
    if (!TestFeedFrame(frame, IMAGE_REMOTE_VT13_FRAME_SIZE)) return 0;
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                       snapshot.sourceProtocol == MANUAL_INPUT_PROTOCOL_IMAGE_VT13 &&
                       snapshot.sourceFlags == (MANUAL_INPUT_SOURCE_FLAG_AUTO_AIM |
                                                MANUAL_INPUT_SOURCE_FLAG_AUX_FIRE) &&
                       snapshot.manual.rc.ch[0] == 512 &&
                       snapshot.manual.rc.ch[1] == -512 &&
                       snapshot.manual.rc.s[0] == RC_SW_DOWN &&
                       snapshot.manual.rc.s[1] == RC_SW_MID,
                   "VT13 原始开关和业务位必须按同代配置映射")) return 0;
    if (!TestCheck(ImageRemoteGetState(&imageState) &&
                       (uint8_t)imageState.s[0] == 2u &&
                       (uint8_t)imageState.s[1] == 0x06u &&
                       imageState.pause == 0u &&
                       imageState.btn_l == 1u &&
                       imageState.btn_r == 1u,
                   "VT13 图传诊断必须保留原始拨杆和全部按键位")) return 0;
    if (!TestCheck(s_rawLogCount == 1u &&
                       s_lastRawLog.proto == SDLOG_MANUAL_INPUT_PROTO_IMAGE_VT13 &&
                       s_lastRawLog.range_mode == SDLOG_MANUAL_INPUT_RANGE_RAW_11BIT &&
                       s_lastRawLog.sw[0] == 2u &&
                       s_lastRawLog.sw[1] == 0x06u &&
                       s_lastRawLog.mouse_btns == 0x03u,
                   "VT13 原始日志必须记录真实拨杆、按键组合和鼠标键")) return 0;

    g_config.manual_input.vt13.switch1_safe_value = 2u;
    g_config.manual_input.vt13.switch1_spin_value = 0u;
    g_config.manual_input.vt13.switch2_btn_l_pos = MANUAL_INPUT_SWITCH_POS_DOWN;
    g_config.manual_input.vt13.auto_aim_pause_enable = 0u;
    g_config.manual_input.vt13.auto_aim_mouse_r_enable = 0u;
    g_config.manual_input.vt13.AuxFireBtnLEnable = 0u;
    g_config.manual_input.vt13.AuxFireMouseLEnable = 0u;
    ManualInputRefresh();
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                       snapshot.sourceProtocol == MANUAL_INPUT_PROTOCOL_IMAGE_VT13 &&
                       snapshot.sourceFlags == 0u &&
                       snapshot.manual.rc.s[0] == RC_SW_UP &&
                       snapshot.manual.rc.s[1] == RC_SW_DOWN,
                   "VT13 配置热更新必须重映射冻结的原始开关和业务位")) return 0;
    if (!TestCheck(ImageRemoteGetState(&imageState) &&
                       (uint8_t)imageState.s[0] == 2u &&
                       (uint8_t)imageState.s[1] == 0x06u &&
                       s_rawLogCount == 1u,
                   "VT13 配置热更新不得反向改写协议原始诊断值")) return 0;

    s_tick = 4u;
    TestBuildVt13Frame(frame, channel, 1u, 0u, 0u, 1u, 0u, 0u);
    if (!TestFeedFrame(frame, IMAGE_REMOTE_VT13_FRAME_SIZE)) return 0;
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                       snapshot.manual.rc.s[1] == RC_SW_DOWN &&
                       snapshot.sourceFlags == 0u,
                   "VT13 右键原始位必须独立保留并映射到配置档位")) return 0;
    if (!TestCheck(ImageRemoteGetState(&imageState) &&
                       (uint8_t)imageState.s[0] == 1u &&
                       (uint8_t)imageState.s[1] == 0x04u,
                   "VT13 第二帧诊断不得残留上一帧左键")) return 0;
    return TestCheck(s_rawLogCount == 2u &&
                         s_lastRawLog.sw[0] == 1u &&
                         s_lastRawLog.sw[1] == 0x04u &&
                         s_lastRawLog.mouse_btns == 0u,
                     "VT13 第二帧原始日志必须完整替换拨杆与按键状态");
}

static int TestMergeAuthorityMetadata(void)
{
    ManualInputSnapshot snapshot;
    ManualInputState crsf;
    uint8_t frame[IMAGE_REMOTE_RM_FRAME_MAX_SIZE];
    ImageRemoteRcPacket packet;
    uint16_t frameLength;

    TestReset(100u);
    g_config.manual_input.mix_mode = MANUAL_INPUT_MIX_MERGE;
    g_config.manual_input.vt13.auto_aim_mouse_r_enable = 1u;
    g_config.manual_input.vt13.AuxFireMouseLEnable = 1u;
    s_tick = 5u;
    packet = TestCustomPacket(660, 100,
                              IMAGE_REMOTE_RC_BTN_LEFT | IMAGE_REMOTE_RC_BTN_RIGHT);
    frameLength = TestBuildCustomFrame(frame,
                                       IMAGE_REMOTE_CMD_CUSTOM_CLIENT_RX,
                                       &packet);
    if (!TestFeedFrame(frame, frameLength)) return 0;

    memset(&crsf, 0, sizeof(crsf));
    crsf.rc.ch[0] = 100;
    crsf.rc.ch[1] = -900;
    crsf.rc.s[0] = RC_SW_UP;
    crsf.rc.s[1] = RC_SW_MID;
    crsf.key.v = KEY_PRESSED_OFFSET_W;
    s_tick = 6u;
    ManualInputUpdateSource(MANUAL_INPUT_SRC_ELRS, &crsf);
    if (!TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                       snapshot.mixMode == MANUAL_INPUT_MIX_MERGE &&
                       snapshot.activeSource == MANUAL_INPUT_SRC_IMAGE &&
                       snapshot.sourceProtocol == MANUAL_INPUT_PROTOCOL_IMAGE_CUSTOM &&
                       snapshot.sourceFlags == (MANUAL_INPUT_SOURCE_FLAG_AUTO_AIM |
                                                MANUAL_INPUT_SOURCE_FLAG_AUX_FIRE) &&
                       snapshot.manual.rc.ch[0] == 1024 &&
                       snapshot.manual.rc.ch[1] == 155 &&
                       snapshot.manual.key.v == KEY_PRESSED_OFFSET_E &&
                       snapshot.manual.rc.s[0] == RC_SW_MID,
                   "合并模式必须保持图传完整操纵帧，同时合并明确的图传业务请求")) return 0;

    s_tick = 7u;
    if (!TestFeedFrame(frame, frameLength)) return 0;
    return TestCheck(ManualInputSnapshotRead(&snapshot) != 0u &&
                         snapshot.activeSource == MANUAL_INPUT_SRC_IMAGE &&
                         snapshot.sourceProtocol == MANUAL_INPUT_PROTOCOL_IMAGE_CUSTOM &&
                         snapshot.sourceFlags == (MANUAL_INPUT_SOURCE_FLAG_AUTO_AIM |
                                                  MANUAL_INPUT_SOURCE_FLAG_AUX_FIRE) &&
                         snapshot.manual.rc.ch[1] == 155 &&
                         snapshot.manual.key.v == KEY_PRESSED_OFFSET_E,
                     "稳定图传代表来源更新时不得混入其他来源的轴和通用按键");
}

static int TestParserArrivalDuringPublish(void)
{
    ManualInputSnapshot before;
    ManualInputSnapshot after;
    ManualInputState crsf;
    ImageRemoteRcPacket packet;

    TestReset(100u);
    g_config.manual_input.vt13.auto_aim_mouse_r_enable = 1u;
    g_config.manual_input.vt13.AuxFireMouseLEnable = 1u;
    memset(&crsf, 0, sizeof(crsf));
    crsf.rc.ch[0] = 100;
    crsf.rc.s[0] = RC_SW_UP;
    crsf.rc.s[1] = RC_SW_UP;
    s_tick = 1u;
    ManualInputUpdateSource(MANUAL_INPUT_SRC_ELRS, &crsf);
    if (!TestCheck(ManualInputSnapshotRead(&before) != 0u &&
                       before.sourceProtocol == MANUAL_INPUT_PROTOCOL_NONE,
                   "发布竞态测试必须先建立 ELRS 来源基线")) return 0;

    packet = TestCustomPacket(660, -330,
                              IMAGE_REMOTE_RC_BTN_LEFT | IMAGE_REMOTE_RC_BTN_RIGHT);
    s_publishInjectLength = TestBuildCustomFrame(
        s_publishInjectFrame,
        IMAGE_REMOTE_CMD_CUSTOM_CONTROLLER_RX,
        &packet);
    s_publishInjectPending = 1u;
    s_tick = 2u;
    g_config.manual_input.BoardKeyKeyMask = KEY_PRESSED_OFFSET_Q;
    g_config.input.axis[0].invert = 1u;
    ManualInputRefresh();

    if (!TestCheck(s_publishInjectPending == 0u &&
                       s_publishInjectFailed == 0u,
                   "发布计算中到达的真实图传帧必须作废旧候选")) return 0;
    return TestCheck(ManualInputSnapshotRead(&after) != 0u &&
                         after.publishSeq == before.publishSeq + 1u &&
                         after.activeSource == MANUAL_INPUT_SRC_IMAGE &&
                         after.sourceProtocol == MANUAL_INPUT_PROTOCOL_IMAGE_CUSTOM &&
                         after.sourceFlags == (MANUAL_INPUT_SOURCE_FLAG_AUTO_AIM |
                                               MANUAL_INPUT_SOURCE_FLAG_AUX_FIRE) &&
                         after.manual.rc.ch[0] == 1024 &&
                         after.control.axis[0] == -1024 &&
                         after.sourceSeq == manual_src[MANUAL_INPUT_SRC_IMAGE].update_seq,
                     "竞态收敛后输入、映射、协议和业务标志必须属于同一发布代");
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
    if (s_criticalDepth > 0)
    {
        s_criticalDepth--;
    }
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
    if (s_publishInjectPending != 0u)
    {
        s_publishInjectPending = 0u;
        if (!TestFeedFrame(s_publishInjectFrame, s_publishInjectLength))
        {
            s_publishInjectFailed = 1u;
        }
    }
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
    return s_sdLogActive;
}

void SdLogWrite(uint16_t tag, const void *payload, uint16_t payloadSize)
{
    if (tag == SDLOG_TAG_MANUAL_INPUT_RAW && payload != NULL &&
        payloadSize == (uint16_t)sizeof(s_lastRawLog))
    {
        s_lastRawLog = *(const sdlog_manual_input_raw_t *)payload;
        s_rawLogCount++;
    }
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
    s_auxEventCallback = callback;
}

void BspAuxLinkSetRxByteCb(BspAuxLinkRxByteCb callback)
{
    s_auxByteCallback = callback;
}

void BspAuxLinkSetErrorCb(BspAuxLinkErrorCb callback)
{
    s_auxErrorCallback = callback;
}

uint32_t BspAuxLinkGetBaudrate(void)
{
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
    s_auxWritePos = 0u;
    return s_auxDmaStartResult;
}

int BspAuxLinkRxItStart(void)
{
    return s_auxDmaStartResult;
}

void BspAuxLinkRxItStop(void)
{
    s_auxDmaBuffer = NULL;
    s_auxDmaLength = 0u;
    s_auxWritePos = 0u;
}

int main(void)
{
    if (!TestCustomProtocolFailureAndExpiry()) return 1;
    if (!TestInvalidCustomSwitchInvalidatesVt13()) return 1;
    if (!TestInvalidCustomBusinessFields()) return 1;
    if (!TestInterruptFallbackDefersParsingToTask()) return 1;
    if (!TestMergeAuthorityMetadata()) return 1;
    if (!TestParserArrivalDuringPublish()) return 1;
    if (!TestVt13FrozenConfigRemap()) return 1;
    if (!TestCheck(s_criticalDepth == 0,
                   "所有输入解析和发布临界区必须成对退出")) return 1;
    (void)puts("PASS: ImageRemote unified-input regression");
    return 0;
}
