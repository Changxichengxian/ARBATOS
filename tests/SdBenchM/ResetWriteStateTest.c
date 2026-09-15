/*
 * MCU复位时卡仍在单块写入数据阶段的有界模型。
 *
 * 直接运行真实 SdSpiInit()，模拟卡已经接收了一个512字节块的前缀，
 * 只剩少量数据和两个CRC字节。卡端按SPI写协议在完整数据块后比较CRC16，
 * CRC关闭时忽略收到的CRC，CRC开启时仅在CRC匹配时提交目标扇区。
 */

#include <stdio.h>
#include <string.h>

#define SD_BENCH_TEST
#include "../../shared/components/devices/SdSpi.c"

#define MODEL_SECTOR_SIZE 512u
#define MODEL_RESPONSE_CAPACITY 16u

typedef enum
{
    MODEL_WRITE_DATA = 0,
    MODEL_WRITE_CRC_HIGH,
    MODEL_WRITE_CRC_LOW,
    MODEL_WRITE_RESPONSE,
    MODEL_WRITE_BUSY,
    MODEL_COMMAND,
} ModelPhase;

typedef struct
{
    ModelPhase phase;
    uint8_t csLow;
    uint8_t crcEnabled;
    uint8_t writeAccepted;
    uint8_t writeRejected;
    uint8_t pendingResponse;
    uint8_t command[6];
    uint8_t commandLength;
    uint8_t response[MODEL_RESPONSE_CAPACITY];
    uint8_t responseHead;
    uint8_t responseTail;
    uint8_t appCommandPending;
    uint8_t target[MODEL_SECTOR_SIZE];
    uint8_t sentinel[MODEL_SECTOR_SIZE];
    uint8_t staging[MODEL_SECTOR_SIZE];
    uint16_t receivedCrc;
    uint16_t dataLength;
    uint32_t tick;
    uint32_t commandCount[64];
} ModelCard;

static ModelCard Card;

static uint16_t ModelCrc16(const uint8_t *data, uint32_t length)
{
    uint16_t crc = 0u;

    while (length--)
    {
        crc ^= (uint16_t)(*data++) << 8u;
        for (uint8_t bit = 0u; bit < 8u; bit++)
        {
            crc = (crc & 0x8000u) != 0u ? (uint16_t)((crc << 1u) ^ 0x1021u) : (uint16_t)(crc << 1u);
        }
    }
    return crc;
}

static uint8_t ModelCrc7(const uint8_t *data, uint32_t length)
{
    uint8_t crc = 0u;

    while (length--)
    {
        uint8_t value = *data++;
        for (uint8_t bit = 0u; bit < 8u; bit++)
        {
            crc <<= 1u;
            if (((value ^ crc) & 0x80u) != 0u)
            {
                crc ^= 0x09u;
            }
            value <<= 1u;
        }
    }
    return crc & 0x7Fu;
}

static void ModelQueue(uint8_t value)
{
    if (Card.responseTail < MODEL_RESPONSE_CAPACITY)
    {
        Card.response[Card.responseTail++] = value;
    }
}

static uint8_t ModelPopResponse(void)
{
    if (Card.responseHead < Card.responseTail)
    {
        return Card.response[Card.responseHead++];
    }
    Card.responseHead = 0u;
    Card.responseTail = 0u;
    return 0xFFu;
}

static uint32_t ModelCommandArg(void)
{
    return ((uint32_t)Card.command[1] << 24u) |
           ((uint32_t)Card.command[2] << 16u) |
           ((uint32_t)Card.command[3] << 8u) |
           (uint32_t)Card.command[4];
}

static uint8_t ModelCommandCrcValid(void)
{
    return Card.command[5] == (uint8_t)((ModelCrc7(Card.command, 5u) << 1u) | 1u);
}

static void ModelHandleCommand(void)
{
    const uint8_t command = Card.command[0] & 0x3Fu;
    const uint32_t argument = ModelCommandArg();
    const uint8_t crcRequired = (uint8_t)(Card.crcEnabled || command == 0u || command == 8u);

    Card.commandCount[command]++;
    Card.commandLength = 0u;

    if (crcRequired && !ModelCommandCrcValid())
    {
        ModelQueue(0x09u);
        Card.appCommandPending = 0u;
        return;
    }

    if (command == 0u)
    {
        Card.crcEnabled = 0u;
        Card.appCommandPending = 0u;
        ModelQueue(0x01u);
    }
    else if (command == 59u && argument == 1u)
    {
        Card.crcEnabled = 1u;
        Card.appCommandPending = 0u;
        ModelQueue(0x01u);
    }
    else if (command == 8u && argument == 0x000001AAu)
    {
        Card.appCommandPending = 0u;
        ModelQueue(0x01u);
        ModelQueue(0x00u);
        ModelQueue(0x00u);
        ModelQueue(0x01u);
        ModelQueue(0xAAu);
    }
    else if (command == 55u)
    {
        Card.appCommandPending = 1u;
        ModelQueue(0x01u);
    }
    else if (command == 41u && Card.appCommandPending)
    {
        Card.appCommandPending = 0u;
        ModelQueue(0x00u);
    }
    else if (command == 58u)
    {
        Card.appCommandPending = 0u;
        ModelQueue(0x00u);
        ModelQueue(0x40u);
        ModelQueue(0x00u);
        ModelQueue(0x00u);
        ModelQueue(0x00u);
    }
    else
    {
        Card.appCommandPending = 0u;
        ModelQueue(0x04u);
    }
}

static void ModelBeginInterruptedWrite(uint16_t remaining, uint8_t crcEnabled)
{
    memset(&Card, 0, sizeof(Card));
    Card.phase = MODEL_WRITE_DATA;
    Card.crcEnabled = crcEnabled;
    Card.dataLength = (uint16_t)(MODEL_SECTOR_SIZE - remaining);

    for (uint16_t i = 0u; i < MODEL_SECTOR_SIZE; i++)
    {
        Card.sentinel[i] = (uint8_t)(0xD3u ^ (uint8_t)i);
        Card.target[i] = Card.sentinel[i];
        Card.staging[i] = (uint8_t)(11u + (uint8_t)(i * 37u));
    }
}

static uint8_t ModelContainsCmd0(const uint8_t *data, uint32_t length)
{
    static const uint8_t command[] = {0x40u, 0x00u, 0x00u, 0x00u, 0x00u, 0x95u};

    for (uint32_t i = 0u; i + sizeof(command) <= length; i++)
    {
        if (memcmp(&data[i], command, sizeof(command)) == 0)
        {
            return 1u;
        }
    }
    return 0u;
}

void SdSpiPortCsHigh(void)
{
    Card.csLow = 0u;
}

void SdSpiPortCsLow(void)
{
    Card.csLow = 1u;
}

uint8_t SdSpiPortTxrx(uint8_t data)
{
    Card.tick++;
    if (!Card.csLow)
    {
        return 0xFFu;
    }

    if (Card.phase == MODEL_WRITE_DATA)
    {
        Card.staging[Card.dataLength++] = data;
        if (Card.dataLength == MODEL_SECTOR_SIZE)
        {
            Card.phase = MODEL_WRITE_CRC_HIGH;
        }
        return 0xFFu;
    }
    if (Card.phase == MODEL_WRITE_CRC_HIGH)
    {
        Card.receivedCrc = (uint16_t)data << 8u;
        Card.phase = MODEL_WRITE_CRC_LOW;
        return 0xFFu;
    }
    if (Card.phase == MODEL_WRITE_CRC_LOW)
    {
        Card.receivedCrc |= data;
        if (!Card.crcEnabled || Card.receivedCrc == ModelCrc16(Card.staging, MODEL_SECTOR_SIZE))
        {
            memcpy(Card.target, Card.staging, MODEL_SECTOR_SIZE);
            Card.writeAccepted = 1u;
            Card.pendingResponse = 0x05u;
        }
        else
        {
            Card.writeRejected = 1u;
            Card.pendingResponse = 0x0Bu;
        }
        Card.phase = MODEL_WRITE_RESPONSE;
        return 0xFFu;
    }
    if (Card.phase == MODEL_WRITE_RESPONSE)
    {
        const uint8_t response = Card.pendingResponse;
        Card.phase = Card.writeAccepted ? MODEL_WRITE_BUSY : MODEL_COMMAND;
        return response;
    }
    if (Card.phase == MODEL_WRITE_BUSY)
    {
        Card.phase = MODEL_COMMAND;
        return 0x00u;
    }

    const uint8_t response = ModelPopResponse();
    if (Card.commandLength != 0u || (data & 0xC0u) == 0x40u)
    {
        Card.command[Card.commandLength++] = data;
        if (Card.commandLength == sizeof(Card.command))
        {
            ModelHandleCommand();
        }
    }
    return response;
}

uint32_t SdSpiPortTickMs(void)
{
    return Card.tick;
}

int SdSpiPortTxrxDma(const uint8_t *tx, uint8_t *rx, uint16_t len, uint32_t timeoutMs)
{
    (void)tx;
    (void)rx;
    (void)len;
    (void)timeoutMs;
    return -1;
}

int SdSpiPortReceive(uint8_t *buf, uint16_t len, uint32_t timeoutMs)
{
    (void)timeoutMs;
    for (uint16_t i = 0u; i < len; i++)
    {
        buf[i] = SdSpiPortTxrx(0xFFu);
    }
    return 0;
}

int SdSpiPortTransmit(const uint8_t *buf, uint16_t len, uint32_t timeoutMs)
{
    (void)buf;
    (void)len;
    (void)timeoutMs;
    return -1;
}

void SdSpiPortSetSpeed(SdSpiPortSpeed speed)
{
    (void)speed;
}

osMutexId_t osMutexNew(const osMutexAttr_t *attr)
{
    (void)attr;
    return (osMutexId_t)1;
}

int32_t osMutexAcquire(osMutexId_t mutexId, uint32_t timeout)
{
    (void)mutexId;
    (void)timeout;
    return 0;
}

int32_t osMutexRelease(osMutexId_t mutexId)
{
    (void)mutexId;
    return 0;
}

void osDelay(uint32_t ticks)
{
    Card.tick += ticks;
}

static int TestAssert(int condition, const char *name, uint16_t remaining)
{
    if (!condition)
    {
        printf("FAIL remaining=%u: %s\n", remaining, name);
        return 1;
    }
    return 0;
}

static int TestModelAcceptsValidCrc(void)
{
    static const uint8_t suffix[8] = {0x51u, 0x29u, 0x73u, 0x18u, 0xA6u, 0xC4u, 0x02u, 0xEFu};
    ModelBeginInterruptedWrite((uint16_t)sizeof(suffix), 1u);
    SdSpiPortCsLow();
    for (uint32_t i = 0u; i < sizeof(suffix); i++)
    {
        (void)SdSpiPortTxrx(suffix[i]);
    }
    const uint16_t crc = ModelCrc16(Card.staging, MODEL_SECTOR_SIZE);
    (void)SdSpiPortTxrx((uint8_t)(crc >> 8u));
    (void)SdSpiPortTxrx((uint8_t)crc);
    const uint8_t response = SdSpiPortTxrx(0xFFu);

    return TestAssert(response == 0x05u, "正确CRC返回data accepted", 8u) |
           TestAssert(Card.writeAccepted && !Card.writeRejected, "正确CRC允许提交", 8u) |
           TestAssert(memcmp(Card.target, Card.staging, MODEL_SECTOR_SIZE) == 0,
                      "正确CRC提交完整数据", 8u);
}

static int TestInterruptedWrite(uint16_t remaining, uint8_t priorCrcEnabled)
{
    ModelBeginInterruptedWrite(remaining, priorCrcEnabled);
    const int initResult = SdSpiInit();
    const uint8_t containsCmd0 = ModelContainsCmd0(Card.staging, MODEL_SECTOR_SIZE);
    const uint8_t targetChanged = memcmp(Card.target, Card.sentinel, MODEL_SECTOR_SIZE) != 0;
    int failed = 0;

    failed |= TestAssert(initResult == 0, "真实初始化流可在下一次CMD0恢复", remaining);
    failed |= TestAssert(containsCmd0, "旧数据块尾部包含完整CMD0", remaining);
    failed |= TestAssert(Card.commandCount[0] >= 1u && Card.commandCount[59] == 1u,
                         "恢复后执行CMD0和CMD59", remaining);
    failed |= TestAssert(Card.crcEnabled && SdSpiCrcEnabled,
                         "恢复完成后主机和卡均开启CRC", remaining);

    if (priorCrcEnabled)
    {
        failed |= TestAssert(Card.writeRejected && !Card.writeAccepted,
                             "旧会话CRC开启时拒绝初始化流尾部", remaining);
        failed |= TestAssert(!targetChanged, "CRC拒绝后哨兵扇区保持不变", remaining);
    }
    else
    {
        failed |= TestAssert(Card.writeAccepted && !Card.writeRejected,
                             "旧会话CRC关闭时接受初始化流尾部", remaining);
        failed |= TestAssert(targetChanged, "CRC关闭时目标扇区被提交", remaining);
    }
    return failed;
}

int main(void)
{
    static const uint16_t remainingCases[] = {8u, 12u, 15u};
    int failed = TestModelAcceptsValidCrc();

    for (uint32_t i = 0u; i < sizeof(remainingCases) / sizeof(remainingCases[0]); i++)
    {
        failed |= TestInterruptedWrite(remainingCases[i], 0u);
        failed |= TestInterruptedWrite(remainingCases[i], 1u);
    }

    if (!failed)
    {
        printf("SdSpi reset during write model: PASS (remaining 8, 12, 15)\n");
    }
    return failed ? 1 : 0;
}
