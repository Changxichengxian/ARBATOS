/*
 * Lightweight host regression for the production DBUS codec and referee parser.
 * Build through tools/TestInputReferee.ps1; this file is not part of firmware targets.
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ManualInputDbus.h"

/* Include both production translation units so the test can drive the private FIFO parser. */
#include "Referee.c"
#include "RefereeRxTask.c"

static uint32_t DetectHookCount;

void *heap_malloc(uint32_t size)
{
    return malloc((size_t)size);
}

void heap_free(void *ptr)
{
    free(ptr);
}

uint32_t HAL_GetTick(void)
{
    return 0u;
}

TaskHandle_t xTaskGetCurrentTaskHandle(void)
{
    return NULL;
}

uint32_t ulTaskNotifyTake(uint32_t clearOnExit, uint32_t waitTicks)
{
    (void)clearOnExit;
    (void)waitTicks;
    return 0u;
}

void BspRefereeUartInit(void) {}
void BspRefereeRxAttachTask(TaskHandle_t task) { (void)task; }
int BspRefereeRxPop(uint8_t *out, uint16_t *outLen) { (void)out; (void)outLen; return 0; }
uint32_t BspRefereeRxGetDropCount(void) { return 0u; }
int BspRefereeTx(const uint8_t *data, uint16_t len) { (void)data; return (int)len; }
uint8_t BspRefereeTxReady(void) { return 1u; }

void DetectHook(uint8_t toe)
{
    if (toe == (uint8_t)REFEREE_TOE)
    {
        DetectHookCount++;
    }
}

void SdLogWrite(uint16_t tag, const void *payload, uint16_t len)
{
    (void)tag;
    (void)payload;
    (void)len;
}

static int TestCheck(int condition, const char *message)
{
    if (condition)
    {
        return 1;
    }

    (void)fprintf(stderr, "FAIL: %s\n", message);
    return 0;
}

static void DbusPack(uint8_t frame[MANUAL_INPUT_DBUS_FRAME_LENGTH],
                     const uint16_t channel[MANUAL_INPUT_DBUS_CHANNEL_COUNT],
                     uint8_t sw0,
                     uint8_t sw1)
{
    (void)memset(frame, 0, MANUAL_INPUT_DBUS_FRAME_LENGTH);
    frame[0] = (uint8_t)(channel[0] & 0x00FFu);
    frame[1] = (uint8_t)(((channel[0] >> 8) & 0x07u) | ((channel[1] & 0x001Fu) << 3));
    frame[2] = (uint8_t)(((channel[1] >> 5) & 0x003Fu) | ((channel[2] & 0x0003u) << 6));
    frame[3] = (uint8_t)((channel[2] >> 2) & 0x00FFu);
    frame[4] = (uint8_t)(((channel[2] >> 10) & 0x01u) | ((channel[3] & 0x007Fu) << 1));
    frame[5] = (uint8_t)(((channel[3] >> 7) & 0x0Fu) | ((sw0 & 0x03u) << 4) | ((sw1 & 0x03u) << 6));
    frame[16] = (uint8_t)(channel[4] & 0x00FFu);
    frame[17] = (uint8_t)((channel[4] >> 8) & 0x00FFu);
}

static int TestDbusCodec(void)
{
    uint8_t frame[MANUAL_INPUT_DBUS_FRAME_LENGTH];
    uint16_t channel[MANUAL_INPUT_DBUS_CHANNEL_COUNT] = {1024u, 1024u, 1024u, 1024u, 1024u};
    ManualInputDbusData data;

    DbusPack(frame, channel, MANUAL_INPUT_DBUS_SWITCH_UP, MANUAL_INPUT_DBUS_SWITCH_DOWN);
    if (!TestCheck(ManualInputDbusDecode(frame, &data) != 0u, "valid DBUS frame decodes") ||
        !TestCheck(ManualInputDbusValid(&data) != 0u, "centered DBUS frame passes content checks"))
    {
        return 0;
    }

    DbusPack(frame, channel, 0u, MANUAL_INPUT_DBUS_SWITCH_DOWN);
    (void)ManualInputDbusDecode(frame, &data);
    if (!TestCheck(ManualInputDbusValid(&data) == 0u, "zero/invalid DBUS switch is rejected"))
    {
        return 0;
    }

    channel[0] = (uint16_t)(MANUAL_INPUT_DBUS_CHANNEL_OFFSET + MANUAL_INPUT_DBUS_CHANNEL_ERROR_ABS + 1u);
    DbusPack(frame, channel, MANUAL_INPUT_DBUS_SWITCH_UP, MANUAL_INPUT_DBUS_SWITCH_MID);
    (void)ManualInputDbusDecode(frame, &data);
    if (!TestCheck(ManualInputDbusValid(&data) == 0u, "out-of-range DBUS channel is rejected"))
    {
        return 0;
    }

    channel[0] = MANUAL_INPUT_DBUS_CHANNEL_OFFSET;
    DbusPack(frame, channel, MANUAL_INPUT_DBUS_SWITCH_UP, MANUAL_INPUT_DBUS_SWITCH_MID);
    frame[12] = 2u;
    (void)ManualInputDbusDecode(frame, &data);
    return TestCheck(ManualInputDbusValid(&data) == 0u, "invalid DBUS mouse button is rejected");
}

static uint16_t RefereeMakeFrame(uint8_t frame[REF_PROTOCOL_FRAME_MAX_SIZE],
                                 uint16_t cmd,
                                 const uint8_t *payload,
                                 uint16_t payloadLen)
{
    uint16_t index;
    uint16_t frameLen = (uint16_t)(REF_HEADER_CRC_CMDID_LEN + payloadLen);

    (void)memset(frame, 0, REF_PROTOCOL_FRAME_MAX_SIZE);
    frame[0] = HEADER_SOF;
    frame[1] = (uint8_t)(payloadLen & 0x00FFu);
    frame[2] = (uint8_t)((payloadLen >> 8) & 0x00FFu);
    frame[3] = 1u;
    append_CRC8_check_sum(frame, (uint32_t)REF_PROTOCOL_HEADER_SIZE);

    index = (uint16_t)REF_PROTOCOL_HEADER_SIZE;
    frame[index++] = (uint8_t)(cmd & 0x00FFu);
    frame[index++] = (uint8_t)((cmd >> 8) & 0x00FFu);
    if (payload != NULL && payloadLen != 0u)
    {
        (void)memcpy(frame + index, payload, payloadLen);
    }
    append_CRC16_check_sum(frame, frameLen);
    return frameLen;
}

static void RefereeParserReset(void)
{
    (void)memset(&RefereeUnpackObj, 0, sizeof(RefereeUnpackObj));
    (void)fifo_s_init(&RefereeFifo, RefereeFifoBuf, REFEREE_FIFO_BUF_LENGTH);
    init_referee_struct_data();
    DetectHookCount = 0u;
}

static void RefereeFeed(const uint8_t *data, uint16_t len)
{
    (void)fifo_s_puts(&RefereeFifo, (char *)data, len);
    RefereeUnpackFifoData();
}

static int TestRefereeParser(void)
{
    uint8_t frame[REF_PROTOCOL_FRAME_MAX_SIZE];
    uint8_t payload = 0x5Au;
    uint16_t len;

    RefereeParserReset();
    len = RefereeMakeFrame(frame, GAME_RESULT_CMD_ID, &payload, 1u);
    RefereeFeed(frame, len);
    if (!TestCheck(DetectHookCount == 1u, "valid CRC8/CRC16 frame refreshes referee link once") ||
        !TestCheck(game_result.winner == payload, "one-byte payload is copied without subtracting cmd_id"))
    {
        return 0;
    }

    RefereeParserReset();
    len = RefereeMakeFrame(frame, GAME_RESULT_CMD_ID, &payload, 1u);
    frame[4] ^= 0x01u;
    RefereeFeed(frame, len);
    if (!TestCheck(DetectHookCount == 0u, "bad header CRC does not refresh referee link"))
    {
        return 0;
    }

    RefereeParserReset();
    len = RefereeMakeFrame(frame, GAME_RESULT_CMD_ID, &payload, 1u);
    frame[len - 1u] ^= 0x01u;
    RefereeFeed(frame, len);
    if (!TestCheck(DetectHookCount == 0u, "bad frame CRC does not refresh referee link"))
    {
        return 0;
    }

    RefereeParserReset();
    len = RefereeMakeFrame(frame, 0x7F01u, &payload, 1u);
    RefereeFeed(frame, len);
    if (!TestCheck(DetectHookCount == 1u, "unknown valid command still proves the physical referee link"))
    {
        return 0;
    }

    RefereeParserReset();
    game_state.stage_remain_time = 1234u;
    len = RefereeMakeFrame(frame, GAME_STATE_CMD_ID, &payload, 1u);
    RefereeFeed(frame, len);
    if (!TestCheck(DetectHookCount == 1u, "CRC-valid short known command still proves the physical link") ||
        !TestCheck(game_state.stage_remain_time == 1234u, "short known payload cannot overwrite prior state"))
    {
        return 0;
    }

    RefereeParserReset();
    len = RefereeMakeFrame(frame, GAME_RESULT_CMD_ID, &payload, 1u);
    RefereeFeed(frame, 3u);
    if (!TestCheck(DetectHookCount == 0u, "partial referee frame does not refresh link"))
    {
        return 0;
    }
    RefereeFeed(frame + 3u, (uint16_t)(len - 3u));
    return TestCheck(DetectHookCount == 1u, "fragmented complete frame refreshes after final CRC byte");
}

int main(void)
{
    if (!TestDbusCodec() || !TestRefereeParser())
    {
        return 1;
    }

    (void)puts("PASS: input/referee host regression");
    return 0;
}
