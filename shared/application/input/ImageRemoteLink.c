/*
 * SPDX-FileCopyrightText: 2026 陈卓 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "ImageRemoteLink.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_compiler.h"

#include "ManualInputSnapshot.h"
#include "Crc8Crc16.h"

#define IMAGE_REMOTE_FRAME_SOF          0xA5u
#define IMAGE_REMOTE_DMA_RX_BUF_SIZE    1024u
#define IMAGE_REMOTE_IT_RX_RING_SIZE    1024u
#define IMAGE_REMOTE_RM_FRAME_MAX_SIZE  64u
#define IMAGE_REMOTE_VT13_FRAME_SIZE    21u
#define IMAGE_REMOTE_RC_MAGIC0          'R'
#define IMAGE_REMOTE_RC_MAGIC1          'C'
#define IMAGE_REMOTE_RC_VERSION         1u
#define IMAGE_REMOTE_RC_RANGE_DBUS      0u
#define IMAGE_REMOTE_RC_RANGE_VT13      1u
#define IMAGE_REMOTE_RC_BTN_LEFT        (1u << 0)
#define IMAGE_REMOTE_RC_BTN_RIGHT       (1u << 1)
#define IMAGE_REMOTE_RC_ABS_MAX_DBUS    ((int16_t)RC_CH_VALUE_ABS_LEGACY)
#define IMAGE_REMOTE_RC_ABS_MAX_VT13    ((int16_t)RC_CH_VALUE_ABS_MAX)
#define IMAGE_REMOTE_CMD_CUSTOM_CONTROLLER_RX 0x0302u
#define IMAGE_REMOTE_CMD_CUSTOM_CLIENT_RX     0x0311u
#define IMAGE_REMOTE_KEY_FLAG(value, mask) ((((value) & (mask)) != 0u) ? 1u : 0u)

typedef struct __attribute__((packed))
{
    uint8_t magic0;
    uint8_t magic1;
    uint8_t version;
    uint8_t range_mode;
    int16_t ch[5];
    uint8_t sw[2];
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint8_t mouse_btns;
    uint16_t key_value;
    uint8_t reserved[5];
} ImageRemoteRcPacket;

typedef char _check_image_remote_rc_packet_size[(sizeof(ImageRemoteRcPacket) == 30u) ? 1 : -1];
typedef char _check_image_remote_it_ring_power_of_two[
    ((IMAGE_REMOTE_IT_RX_RING_SIZE & (IMAGE_REMOTE_IT_RX_RING_SIZE - 1u)) == 0u) ? 1 : -1];

typedef enum
{
    IMAGE_REMOTE_PARSE_IDLE = 0,
    IMAGE_REMOTE_PARSE_RM,
    IMAGE_REMOTE_PARSE_VT13,
} ImageRemoteParseMode;

typedef union
{
    uint8_t dma[IMAGE_REMOTE_DMA_RX_BUF_SIZE];
    uint8_t itRing[IMAGE_REMOTE_IT_RX_RING_SIZE];
} ImageRemoteRxStorage;

/* DMA 与逐字节中断接收由同一串口模式决定，不会同时运行。 */
static ImageRemoteRxStorage ImageRemoteRx;
static volatile uint16_t ImageRemoteItRxHead = 0u;
static volatile uint16_t ImageRemoteItRxTail = 0u;
static volatile uint8_t ImageRemoteItRxOverflow = 0u;
static volatile uint16_t ImageRemoteDmaPos = 0u;
static uint16_t ImageRemoteDmaLastPos = 0u;
static volatile uint32_t ImageRemoteDmaWrapCnt = 0u;
static uint32_t ImageRemoteDmaLastWrapCnt = 0u;
static volatile uint8_t ImageRemoteDmaActive = 0u;
static volatile uint8_t ImageRemoteDmaRestartReq = 0u;
static ImageRemoteParseMode s_parse_mode = IMAGE_REMOTE_PARSE_IDLE;
static uint8_t ImageRemoteRmBuf[IMAGE_REMOTE_RM_FRAME_MAX_SIZE];
static uint16_t ImageRemoteRmPos = 0u;
static uint16_t ImageRemoteRmExpected = 0u;
static uint8_t ImageRemoteVt13Buf[IMAGE_REMOTE_VT13_FRAME_SIZE];
static uint8_t ImageRemoteVt13Pos = 0u;
static volatile uint32_t ImageRemoteLastRxTickMs = 0u;
static volatile uint32_t ImageRemoteFrameCnt = 0u;
static volatile uint32_t ImageRemoteControllerFrameCnt = 0u;
static volatile uint32_t ImageRemoteClientFrameCnt = 0u;
static volatile uint32_t ImageRemoteVt13FrameCnt = 0u;
static volatile uint32_t ImageRemoteCrcErrorCnt = 0u;
static volatile uint32_t ImageRemoteParseErrorCnt = 0u;
static volatile uint32_t ImageRemoteRestartCnt = 0u;
static volatile uint16_t ImageRemoteLastCmdId = 0u;
static volatile uint8_t ImageRemoteLastRangeMode = 0u;
static ImageRemoteState s_image_remote_state = {0};

static void ImageRemoteStore(const ImageRemoteState *state);
static void ImageRemoteInvalidate(void);
static uint8_t ImageRemoteRawFlags(const ImageRemoteState *state);
static void ImageRemoteLinkProcessTo(uint16_t pos);
static void ImageRemoteLinkResetParser(void);
static void ImageRemoteLinkFeedByte(uint8_t b);
static void ImageRemoteLinkHandleRmFrame(const uint8_t *frame, uint16_t frame_len);
static void ImageRemoteLinkHandleVt13Frame(const uint8_t *frame, uint16_t frame_len);
static bool_t ImageRemoteLinkTryDecodeCustomRc(const uint8_t *data);
static int16_t ImageRemoteLinkScaleAxis(int16_t raw, int16_t raw_abs_max);
static bool_t ImageRemoteLinkSwitchValid(uint8_t value);

bool ImageRemoteGetState(ImageRemoteState *out)
{
    if (out == NULL)
    {
        return false;
    }

    taskENTER_CRITICAL();
    *out = s_image_remote_state;
    taskEXIT_CRITICAL();
    return (out->valid != 0u);
}

void ImageRemoteLinkGetStats(sdlog_image_link_stats_t *out)
{
    if (out == NULL)
    {
        return;
    }

    memset(out, 0, sizeof(*out));
    taskENTER_CRITICAL();
    out->last_rx_tick_ms = ImageRemoteLastRxTickMs;
    out->frame_count = ImageRemoteFrameCnt;
    out->controller_frame_count = ImageRemoteControllerFrameCnt;
    out->client_frame_count = ImageRemoteClientFrameCnt;
    out->vt13_frame_count = ImageRemoteVt13FrameCnt;
    out->crc_error_count = ImageRemoteCrcErrorCnt;
    out->parse_error_count = ImageRemoteParseErrorCnt;
    out->restart_count = ImageRemoteRestartCnt;
    out->last_cmd_id = ImageRemoteLastCmdId;
    out->port_active = ImageRemoteDmaActive;
    out->last_range_mode = ImageRemoteLastRangeMode;
    taskEXIT_CRITICAL();
}

void ImageRemoteLinkStart(void)
{
    ImageRemoteLinkStop();
    ImageRemoteDmaPos = 0u;
    ImageRemoteDmaLastPos = 0u;
    ImageRemoteDmaWrapCnt = 0u;
    ImageRemoteDmaLastWrapCnt = 0u;
    ImageRemoteDmaRestartReq = 0u;
    ImageRemoteLinkResetParser();

    BspAuxLinkSetRxEventCb(ImageRemoteLinkOnRxEvent);
    BspAuxLinkSetRxByteCb(ImageRemoteLinkOnItByte);
    BspAuxLinkSetErrorCb(ImageRemoteLinkOnUartError);

    if (BspAuxLinkRxHasDma() == 0u)
    {
        ImageRemoteItRxHead = 0u;
        ImageRemoteItRxTail = 0u;
        ImageRemoteItRxOverflow = 0u;
        ImageRemoteDmaActive = 1u;
        if (BspAuxLinkRxItStart() != 0)
        {
            ImageRemoteDmaActive = 0u;
            ImageRemoteParseErrorCnt++;
        }
        return;
    }
    if (BspAuxLinkRxToIdleDmaStart(ImageRemoteRx.dma, (uint16_t)IMAGE_REMOTE_DMA_RX_BUF_SIZE) != 0)
    {
        return;
    }

    ImageRemoteDmaActive = 1u;
}

void ImageRemoteLinkStop(void)
{
    ImageRemoteDmaActive = 0u;
    __DMB();
    ImageRemoteDmaRestartReq = 0u;
    ImageRemoteDmaPos = 0u;
    ImageRemoteDmaLastPos = 0u;
    ImageRemoteDmaWrapCnt = 0u;
    ImageRemoteDmaLastWrapCnt = 0u;
    ImageRemoteItRxHead = 0u;
    ImageRemoteItRxTail = 0u;
    ImageRemoteItRxOverflow = 0u;
    ImageRemoteLinkResetParser();

    BspAuxLinkSetRxEventCb(NULL);
    BspAuxLinkSetRxByteCb(NULL);
    BspAuxLinkSetErrorCb(NULL);
    BspAuxLinkRxItStop();
    ImageRemoteInvalidate();
}

void ImageRemoteLinkPoll(void)
{
    if (BspAuxLinkGetBaudrate() != IMAGE_REMOTE_LINK_BAUD)
    {
        return;
    }
    if (ImageRemoteDmaRestartReq != 0u)
    {
        ImageRemoteDmaRestartReq = 0u;
        ImageRemoteLinkStart();
        return;
    }
    if (ImageRemoteDmaActive == 0u)
    {
        return;
    }

    if (BspAuxLinkRxHasDma() == 0u)
    {
        if (ImageRemoteItRxOverflow != 0u)
        {
            taskENTER_CRITICAL();
            ImageRemoteItRxOverflow = 0u;
            ImageRemoteItRxTail = ImageRemoteItRxHead;
            taskEXIT_CRITICAL();
            ImageRemoteParseErrorCnt++;
            ImageRemoteLinkResetParser();
            ImageRemoteInvalidate();
            return;
        }
        while (ImageRemoteItRxTail != ImageRemoteItRxHead)
        {
            const uint16_t tail = ImageRemoteItRxTail;
            const uint8_t value = ImageRemoteRx.itRing[tail];
            ImageRemoteItRxTail =
                (uint16_t)((tail + 1u) &
                           (uint16_t)(IMAGE_REMOTE_IT_RX_RING_SIZE - 1u));
            ImageRemoteLinkFeedByte(value);
        }
        if (ImageRemoteItRxOverflow != 0u)
        {
            taskENTER_CRITICAL();
            ImageRemoteItRxOverflow = 0u;
            ImageRemoteItRxTail = ImageRemoteItRxHead;
            taskEXIT_CRITICAL();
            ImageRemoteParseErrorCnt++;
            ImageRemoteLinkResetParser();
            ImageRemoteInvalidate();
        }
        return;
    }

    uint32_t wrap_cnt;
    uint16_t pos;
    taskENTER_CRITICAL();
    wrap_cnt = ImageRemoteDmaWrapCnt;
    pos = ImageRemoteDmaPos;
    taskEXIT_CRITICAL();
    const uint32_t wraps = wrap_cnt - ImageRemoteDmaLastWrapCnt;

    if (wraps > 1u ||
        (wraps == 1u && pos > ImageRemoteDmaLastPos))
    {
        ImageRemoteParseErrorCnt++;
        ImageRemoteLinkResetParser();
        ImageRemoteInvalidate();
        ImageRemoteDmaLastWrapCnt = wrap_cnt;
        ImageRemoteDmaLastPos = pos;
        return;
    }
    if (wraps == 1u)
    {
        ImageRemoteLinkProcessTo((uint16_t)IMAGE_REMOTE_DMA_RX_BUF_SIZE);
        ImageRemoteDmaLastWrapCnt = wrap_cnt;
    }
    ImageRemoteLinkProcessTo(pos);
    ImageRemoteDmaLastWrapCnt = wrap_cnt;
}

void ImageRemoteLinkOnRxEvent(uint16_t size, BspAuxLinkRxEvent evt)
{
    if (BspAuxLinkGetBaudrate() != IMAGE_REMOTE_LINK_BAUD || ImageRemoteDmaActive == 0u)
    {
        return;
    }

    if (evt == BSP_AUX_LINK_RXEVENT_IDLE && size >= (uint16_t)IMAGE_REMOTE_DMA_RX_BUF_SIZE)
    {
        return;
    }

    if (evt == BSP_AUX_LINK_RXEVENT_TC)
    {
        ImageRemoteDmaWrapCnt++;
    }

    ImageRemoteDmaPos = (size >= (uint16_t)IMAGE_REMOTE_DMA_RX_BUF_SIZE) ? 0u : size;
}

void ImageRemoteLinkOnItByte(uint8_t value)
{
    const uint16_t head = ImageRemoteItRxHead;
    const uint16_t next =
        (uint16_t)((head + 1u) & (uint16_t)(IMAGE_REMOTE_IT_RX_RING_SIZE - 1u));

    if (BspAuxLinkGetBaudrate() != IMAGE_REMOTE_LINK_BAUD || ImageRemoteDmaActive == 0u)
    {
        return;
    }
    if (next == ImageRemoteItRxTail)
    {
        ImageRemoteItRxOverflow = 1u;
        return;
    }
    ImageRemoteRx.itRing[head] = value;
    __DMB();
    ImageRemoteItRxHead = next;
}

uint8_t ImageRemoteLinkOnUartError(void)
{
    if (ImageRemoteDmaActive == 0u)
    {
        return 0u;
    }

    ImageRemoteRestartCnt++;
    ImageRemoteDmaRestartReq = 1u;
    return 1u;
}

static void ImageRemoteStore(const ImageRemoteState *state)
{
    if (state == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    s_image_remote_state = *state;
    taskEXIT_CRITICAL();
}

static void ImageRemoteInvalidate(void)
{
    taskENTER_CRITICAL();
    s_image_remote_state.valid = 0u;
    taskEXIT_CRITICAL();
    ManualInputInvalidateSource(MANUAL_INPUT_SRC_IMAGE);
}

static uint8_t ImageRemoteRawFlags(const ImageRemoteState *state)
{
    uint8_t flags = 0u;

    if (state == NULL)
    {
        return 0u;
    }
    if (state->pause != 0u)
    {
        flags |= MANUAL_INPUT_SOURCE_RAW_PAUSE;
    }
    if (state->mouse_r != 0u)
    {
        flags |= MANUAL_INPUT_SOURCE_RAW_MOUSE_R;
    }
    if (state->btn_l != 0u)
    {
        flags |= MANUAL_INPUT_SOURCE_RAW_BTN_L;
    }
    if (state->mouse_l != 0u)
    {
        flags |= MANUAL_INPUT_SOURCE_RAW_MOUSE_L;
    }
    if (state->btn_r != 0u)
    {
        flags |= MANUAL_INPUT_SOURCE_RAW_BTN_R;
    }
    return flags;
}

static void ImageRemoteLinkProcessTo(uint16_t pos)
{
    const uint16_t size = (uint16_t)IMAGE_REMOTE_DMA_RX_BUF_SIZE;
    uint16_t last = ImageRemoteDmaLastPos;

    if (pos > size)
    {
        pos = (uint16_t)(pos % size);
    }
    if (last >= size)
    {
        last = 0u;
    }

    if (last <= pos)
    {
        for (uint16_t i = last; i < pos; i++)
        {
            ImageRemoteLinkFeedByte(ImageRemoteRx.dma[i]);
        }
    }
    else
    {
        for (uint16_t i = last; i < size; i++)
        {
            ImageRemoteLinkFeedByte(ImageRemoteRx.dma[i]);
        }
        for (uint16_t i = 0u; i < pos; i++)
        {
            ImageRemoteLinkFeedByte(ImageRemoteRx.dma[i]);
        }
    }

    ImageRemoteDmaLastPos = (pos == size) ? 0u : pos;
}

static void ImageRemoteLinkResetParser(void)
{
    s_parse_mode = IMAGE_REMOTE_PARSE_IDLE;
    ImageRemoteRmPos = 0u;
    ImageRemoteRmExpected = 0u;
    ImageRemoteVt13Pos = 0u;
}

static void ImageRemoteLinkFeedByte(uint8_t b)
{
retry_parse:
    switch (s_parse_mode)
    {
    case IMAGE_REMOTE_PARSE_IDLE:
        if (b == IMAGE_REMOTE_FRAME_SOF)
        {
            s_parse_mode = IMAGE_REMOTE_PARSE_RM;
            ImageRemoteRmPos = 0u;
            ImageRemoteRmExpected = 0u;
            ImageRemoteRmBuf[ImageRemoteRmPos++] = b;
        }
        else if (b == 0xA9u)
        {
            s_parse_mode = IMAGE_REMOTE_PARSE_VT13;
            ImageRemoteVt13Pos = 0u;
            ImageRemoteVt13Buf[ImageRemoteVt13Pos++] = b;
        }
        return;

    case IMAGE_REMOTE_PARSE_RM:
        if (ImageRemoteRmPos >= (uint16_t)IMAGE_REMOTE_RM_FRAME_MAX_SIZE)
        {
            ImageRemoteParseErrorCnt++;
            ImageRemoteLinkResetParser();
            goto retry_parse;
        }

        ImageRemoteRmBuf[ImageRemoteRmPos++] = b;

        if (ImageRemoteRmPos == 5u)
        {
            if (!verify_CRC8_check_sum(ImageRemoteRmBuf, 5u))
            {
                ImageRemoteCrcErrorCnt++;
                ImageRemoteLinkResetParser();
                goto retry_parse;
            }

            const uint16_t payload_len = (uint16_t)(ImageRemoteRmBuf[1] | ((uint16_t)ImageRemoteRmBuf[2] << 8));
            ImageRemoteRmExpected = (uint16_t)(payload_len + 9u);
            if (ImageRemoteRmExpected < 9u || ImageRemoteRmExpected > (uint16_t)IMAGE_REMOTE_RM_FRAME_MAX_SIZE)
            {
                ImageRemoteParseErrorCnt++;
                ImageRemoteLinkResetParser();
                goto retry_parse;
            }
        }

        if (ImageRemoteRmExpected != 0u && ImageRemoteRmPos >= ImageRemoteRmExpected)
        {
            if (verify_CRC16_check_sum(ImageRemoteRmBuf, ImageRemoteRmExpected))
            {
                ImageRemoteLinkHandleRmFrame(ImageRemoteRmBuf, ImageRemoteRmExpected);
            }
            else
            {
                ImageRemoteCrcErrorCnt++;
            }
            ImageRemoteLinkResetParser();
        }
        return;

    case IMAGE_REMOTE_PARSE_VT13:
        if (ImageRemoteVt13Pos == 1u && b != 0x53u)
        {
            ImageRemoteParseErrorCnt++;
            ImageRemoteLinkResetParser();
            goto retry_parse;
        }
        if (ImageRemoteVt13Pos >= IMAGE_REMOTE_VT13_FRAME_SIZE)
        {
            ImageRemoteParseErrorCnt++;
            ImageRemoteLinkResetParser();
            goto retry_parse;
        }

        ImageRemoteVt13Buf[ImageRemoteVt13Pos++] = b;
        if (ImageRemoteVt13Pos >= IMAGE_REMOTE_VT13_FRAME_SIZE)
        {
            ImageRemoteLinkHandleVt13Frame(ImageRemoteVt13Buf, IMAGE_REMOTE_VT13_FRAME_SIZE);
            ImageRemoteLinkResetParser();
        }
        return;

    default:
        ImageRemoteParseErrorCnt++;
        ImageRemoteLinkResetParser();
        goto retry_parse;
    }
}

static void ImageRemoteLinkHandleRmFrame(const uint8_t *frame, uint16_t frame_len)
{
    if (frame == NULL || frame_len < 9u)
    {
        return;
    }

    const uint16_t payload_len = (uint16_t)(frame[1] | ((uint16_t)frame[2] << 8));
    const uint16_t cmd_id = (uint16_t)(frame[5] | ((uint16_t)frame[6] << 8));
    const uint8_t known_rc_command =
        (uint8_t)(cmd_id == IMAGE_REMOTE_CMD_CUSTOM_CONTROLLER_RX ||
                  cmd_id == IMAGE_REMOTE_CMD_CUSTOM_CLIENT_RX);
    if ((uint16_t)(payload_len + 9u) != frame_len || payload_len != (uint16_t)sizeof(ImageRemoteRcPacket))
    {
        ImageRemoteParseErrorCnt++;
        if (known_rc_command != 0u)
        {
            /* CRC 正确且命令明确属于遥控时，错误长度也必须立即撤销旧控制。 */
            ImageRemoteInvalidate();
        }
        return;
    }

    const uint8_t *payload = &frame[7];

    if (known_rc_command != 0u)
    {
        ImageRemoteLastRxTickMs = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        ImageRemoteFrameCnt++;
        ImageRemoteLastCmdId = cmd_id;
        if (cmd_id == IMAGE_REMOTE_CMD_CUSTOM_CONTROLLER_RX)
        {
            ImageRemoteControllerFrameCnt++;
        }
        else
        {
            ImageRemoteClientFrameCnt++;
        }
        (void)ImageRemoteLinkTryDecodeCustomRc(payload);
    }
}

static void ImageRemoteLinkHandleVt13Frame(const uint8_t *frame, uint16_t frame_len)
{
    if (frame == NULL || frame_len != IMAGE_REMOTE_VT13_FRAME_SIZE)
    {
        return;
    }
    if (frame[0] != 0xA9u || frame[1] != 0x53u)
    {
        ImageRemoteParseErrorCnt++;
        return;
    }
    if (!verify_CRC16_check_sum((uint8_t *)frame, frame_len))
    {
        ImageRemoteCrcErrorCnt++;
        return;
    }

    int16_t ch_raw[5] = {0};
    ch_raw[0] = (int16_t)(((uint16_t)frame[2] | ((uint16_t)frame[3] << 8)) & 0x07FFu);
    ch_raw[1] = (int16_t)((((uint16_t)frame[3] >> 3) | ((uint16_t)frame[4] << 5)) & 0x07FFu);
    ch_raw[2] = (int16_t)((((uint16_t)frame[4] >> 6) | ((uint16_t)frame[5] << 2) | ((uint16_t)frame[6] << 10)) & 0x07FFu);
    ch_raw[3] = (int16_t)((((uint16_t)frame[6] >> 1) | ((uint16_t)frame[7] << 7)) & 0x07FFu);
    ch_raw[4] = (int16_t)((((uint16_t)frame[8] >> 1) | ((uint16_t)frame[9] << 7)) & 0x07FFu);
    const uint8_t vt13_pause = (uint8_t)((frame[7] >> 6) & 0x01u);
    const uint8_t vt13_btn_l = (uint8_t)((frame[7] >> 7) & 0x01u);
    const uint8_t vt13_btn_r = (uint8_t)(frame[8] & 0x01u);
    const uint16_t vt13_dial = (uint16_t)(((uint16_t)frame[8] >> 1) | ((uint16_t)frame[9] << 7));
    const uint8_t vt13_trigger = (uint8_t)((frame[9] >> 4) & 0x01u);
    const uint8_t vt13_mouse_l = (uint8_t)(frame[16] & 0x01u);
    const uint8_t vt13_mouse_r = (uint8_t)((frame[16] >> 1) & 0x01u);
    const uint8_t vt13_mouse_mid = (uint8_t)((frame[16] >> 2) & 0x01u);
    const uint8_t vt13_switch1 = (uint8_t)((frame[7] >> 4) & 0x03u);
    /* 诊断保留协议原值：sw[0] 是两位拨杆，sw[1] 的 bit0..2 依次是 pause、btn_l、btn_r。 */
    const uint8_t vt13_raw_sw[2] = {
        vt13_switch1,
        (uint8_t)(vt13_pause | (uint8_t)(vt13_btn_l << 1) | (uint8_t)(vt13_btn_r << 2)),
    };

    ManualInputState rc = {0};
    rc.rc.ch[0] = ImageRemoteLinkScaleAxis((int16_t)(ch_raw[0] - RC_CH_VALUE_OFFSET), IMAGE_REMOTE_RC_ABS_MAX_VT13);
    rc.rc.ch[1] = ImageRemoteLinkScaleAxis((int16_t)(ch_raw[1] - RC_CH_VALUE_OFFSET), IMAGE_REMOTE_RC_ABS_MAX_VT13);
    rc.rc.ch[2] = ImageRemoteLinkScaleAxis((int16_t)(ch_raw[2] - RC_CH_VALUE_OFFSET), IMAGE_REMOTE_RC_ABS_MAX_VT13);
    rc.rc.ch[3] = ImageRemoteLinkScaleAxis((int16_t)(ch_raw[3] - RC_CH_VALUE_OFFSET), IMAGE_REMOTE_RC_ABS_MAX_VT13);
    rc.rc.ch[4] = ImageRemoteLinkScaleAxis((int16_t)(ch_raw[4] - RC_CH_VALUE_OFFSET), IMAGE_REMOTE_RC_ABS_MAX_VT13);

    /* VT13 原始拨杆稍后在 ManualInput 快照里用冻结配置统一解释。 */
    rc.rc.s[0] = (char)RC_SW_UP;
    rc.rc.s[1] = (char)RC_SW_UP;

    rc.mouse.x = (int16_t)((uint16_t)frame[10] | ((uint16_t)frame[11] << 8));
    rc.mouse.y = (int16_t)((uint16_t)frame[12] | ((uint16_t)frame[13] << 8));
    rc.mouse.z = (int16_t)((uint16_t)frame[14] | ((uint16_t)frame[15] << 8));
    rc.mouse.press_l = vt13_mouse_l;
    rc.mouse.press_r = vt13_mouse_r;
    rc.key.v = (uint16_t)(frame[17] | ((uint16_t)frame[18] << 8));

    ImageRemoteLastRxTickMs = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    ImageRemoteFrameCnt++;
    ImageRemoteVt13FrameCnt++;
    ImageRemoteLastCmdId = 0u;
    ImageRemoteLastRangeMode = SDLOG_MANUAL_INPUT_RANGE_RAW_11BIT;
    ManualInputLogRawSource(MANUAL_INPUT_SRC_IMAGE,
                                  SDLOG_MANUAL_INPUT_PROTO_IMAGE_VT13,
                                  SDLOG_MANUAL_INPUT_RANGE_RAW_11BIT,
                                  5u,
                                  ch_raw,
                                  vt13_raw_sw,
                                  &rc);
    ImageRemoteState state = {
        .valid = 1u,
        .proto = SDLOG_MANUAL_INPUT_PROTO_IMAGE_VT13,
        .range_mode = SDLOG_MANUAL_INPUT_RANGE_RAW_11BIT,
        .raw_ch = { ch_raw[0], ch_raw[1], ch_raw[2], ch_raw[3], ch_raw[4] },
        .ch = { rc.rc.ch[0], rc.rc.ch[1], rc.rc.ch[2], rc.rc.ch[3], rc.rc.ch[4] },
        .s = { (char)vt13_raw_sw[0], (char)vt13_raw_sw[1] },
        .mouse_x = rc.mouse.x,
        .mouse_y = rc.mouse.y,
        .mouse_z = rc.mouse.z,
        .mouse_l = vt13_mouse_l,
        .mouse_r = vt13_mouse_r,
        .mouse_mid = vt13_mouse_mid,
        .pause = vt13_pause,
        .btn_l = vt13_btn_l,
        .btn_r = vt13_btn_r,
        .trigger = vt13_trigger,
        .dial = vt13_dial,
        .key_value = rc.key.v,
        .key_w = IMAGE_REMOTE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_W),
        .key_s = IMAGE_REMOTE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_S),
        .key_a = IMAGE_REMOTE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_A),
        .key_d = IMAGE_REMOTE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_D),
        .key_shift = IMAGE_REMOTE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_SHIFT),
        .key_ctrl = IMAGE_REMOTE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_CTRL),
        .key_q = IMAGE_REMOTE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_Q),
        .key_e = IMAGE_REMOTE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_E),
        .key_r = IMAGE_REMOTE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_R),
        .key_f = IMAGE_REMOTE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_F),
        .key_g = IMAGE_REMOTE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_G),
        .key_z = IMAGE_REMOTE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_Z),
        .key_x = IMAGE_REMOTE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_X),
        .key_c = IMAGE_REMOTE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_C),
        .key_v = IMAGE_REMOTE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_V),
        .key_b = IMAGE_REMOTE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_B),
        .last_rx_tick_ms = ImageRemoteLastRxTickMs,
    };
    ImageRemoteStore(&state);
    ManualInputUpdateImageSource(&rc,
                                 state.proto,
                                 ImageRemoteRawFlags(&state),
                                 vt13_switch1);
}

static bool_t ImageRemoteLinkTryDecodeCustomRc(const uint8_t *data)
{
    if (data == NULL)
    {
        return 0;
    }

    ImageRemoteRcPacket pkt;
    memcpy(&pkt, data, sizeof(pkt));
    /* 0x0302/0x0311 是通用通道；magic 不匹配表示其他子协议，不得误伤遥控。 */
    if (pkt.magic0 != (uint8_t)IMAGE_REMOTE_RC_MAGIC0 ||
        pkt.magic1 != (uint8_t)IMAGE_REMOTE_RC_MAGIC1)
    {
        return 0;
    }
    if (pkt.version != IMAGE_REMOTE_RC_VERSION)
    {
        ImageRemoteParseErrorCnt++;
        ImageRemoteInvalidate();
        return 0;
    }
    if (!ImageRemoteLinkSwitchValid(pkt.sw[0]) ||
        !ImageRemoteLinkSwitchValid(pkt.sw[1]))
    {
        /* 非法拨杆不能被修成动作档；立即撤销旧 IMAGE 命令，等待下一帧合法输入。 */
        ImageRemoteParseErrorCnt++;
        ImageRemoteInvalidate();
        return 0;
    }

    int16_t raw_abs_max;
    uint8_t log_range_mode;
    if (pkt.range_mode == IMAGE_REMOTE_RC_RANGE_DBUS)
    {
        raw_abs_max = IMAGE_REMOTE_RC_ABS_MAX_DBUS;
        log_range_mode = SDLOG_MANUAL_INPUT_RANGE_CENTERED_660;
    }
    else if (pkt.range_mode == IMAGE_REMOTE_RC_RANGE_VT13)
    {
        raw_abs_max = IMAGE_REMOTE_RC_ABS_MAX_VT13;
        log_range_mode = SDLOG_MANUAL_INPUT_RANGE_CENTERED_1024;
    }
    else
    {
        ImageRemoteParseErrorCnt++;
        ImageRemoteInvalidate();
        return 0;
    }
    if ((pkt.mouse_btns & (uint8_t)~(IMAGE_REMOTE_RC_BTN_LEFT |
                                      IMAGE_REMOTE_RC_BTN_RIGHT)) != 0u)
    {
        ImageRemoteParseErrorCnt++;
        ImageRemoteInvalidate();
        return 0;
    }
    for (uint8_t i = 0u; i < (uint8_t)sizeof(pkt.reserved); i++)
    {
        if (pkt.reserved[i] != 0u)
        {
            ImageRemoteParseErrorCnt++;
            ImageRemoteInvalidate();
            return 0;
        }
    }

    ManualInputState rc = {0};
    int16_t raw_ch[5] = {0};
    uint8_t raw_sw[2] = {0};
    for (uint8_t i = 0u; i < 5u; i++)
    {
        raw_ch[i] = pkt.ch[i];
        if ((int32_t)pkt.ch[i] < -(int32_t)raw_abs_max ||
            (int32_t)pkt.ch[i] > (int32_t)raw_abs_max)
        {
            ImageRemoteParseErrorCnt++;
            ImageRemoteInvalidate();
            return 0;
        }
        rc.rc.ch[i] = ImageRemoteLinkScaleAxis(pkt.ch[i], raw_abs_max);
    }

    raw_sw[0] = pkt.sw[0];
    raw_sw[1] = pkt.sw[1];
    rc.rc.s[0] = (char)pkt.sw[0];
    rc.rc.s[1] = (char)pkt.sw[1];
    rc.mouse.x = pkt.mouse_x;
    rc.mouse.y = pkt.mouse_y;
    rc.mouse.z = pkt.mouse_z;
    rc.mouse.press_l = ((pkt.mouse_btns & IMAGE_REMOTE_RC_BTN_LEFT) != 0u) ? 1u : 0u;
    rc.mouse.press_r = ((pkt.mouse_btns & IMAGE_REMOTE_RC_BTN_RIGHT) != 0u) ? 1u : 0u;
    rc.key.v = pkt.key_value;

    ManualInputLogRawSource(MANUAL_INPUT_SRC_IMAGE,
                                  SDLOG_MANUAL_INPUT_PROTO_IMAGE_CUSTOM,
                                  log_range_mode,
                                  5u,
                                  raw_ch,
                                  raw_sw,
                                  &rc);
    ImageRemoteLastRangeMode = log_range_mode;
    ImageRemoteState state = {
        .valid = 1u,
        .proto = SDLOG_MANUAL_INPUT_PROTO_IMAGE_CUSTOM,
        .range_mode = log_range_mode,
        .raw_ch = { raw_ch[0], raw_ch[1], raw_ch[2], raw_ch[3], raw_ch[4] },
        .ch = { rc.rc.ch[0], rc.rc.ch[1], rc.rc.ch[2], rc.rc.ch[3], rc.rc.ch[4] },
        .s = { rc.rc.s[0], rc.rc.s[1] },
        .mouse_x = rc.mouse.x,
        .mouse_y = rc.mouse.y,
        .mouse_z = rc.mouse.z,
        .mouse_l = rc.mouse.press_l,
        .mouse_r = rc.mouse.press_r,
        .mouse_mid = 0u,
        .pause = 0u,
        .btn_l = 0u,
        .btn_r = 0u,
        .trigger = 0u,
        .dial = rc.rc.ch[4],
        .key_value = rc.key.v,
        .key_w = IMAGE_REMOTE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_W),
        .key_s = IMAGE_REMOTE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_S),
        .key_a = IMAGE_REMOTE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_A),
        .key_d = IMAGE_REMOTE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_D),
        .key_shift = IMAGE_REMOTE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_SHIFT),
        .key_ctrl = IMAGE_REMOTE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_CTRL),
        .key_q = IMAGE_REMOTE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_Q),
        .key_e = IMAGE_REMOTE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_E),
        .key_r = IMAGE_REMOTE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_R),
        .key_f = IMAGE_REMOTE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_F),
        .key_g = IMAGE_REMOTE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_G),
        .key_z = IMAGE_REMOTE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_Z),
        .key_x = IMAGE_REMOTE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_X),
        .key_c = IMAGE_REMOTE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_C),
        .key_v = IMAGE_REMOTE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_V),
        .key_b = IMAGE_REMOTE_KEY_FLAG(rc.key.v, KEY_PRESSED_OFFSET_B),
        .last_rx_tick_ms = ImageRemoteLastRxTickMs,
    };
    ImageRemoteStore(&state);
    ManualInputUpdateImageSource(&rc,
                                 state.proto,
                                 ImageRemoteRawFlags(&state),
                                 0u);
    return 1;
}

static int16_t ImageRemoteLinkScaleAxis(int16_t raw, int16_t raw_abs_max)
{
    if (raw_abs_max <= 0)
    {
        return 0;
    }
    return rc_scale_axis_by_abs(raw, raw_abs_max, (int16_t)RC_CH_VALUE_ABS_MAX);
}

static bool_t ImageRemoteLinkSwitchValid(uint8_t value)
{
    return (bool_t)(value == RC_SW_UP || value == RC_SW_MID || value == RC_SW_DOWN);
}
