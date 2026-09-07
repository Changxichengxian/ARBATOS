/*
 * 主机协议回归：直接编译真实 SdSpi.c，用模拟 SPI 端口检查 SD 命令序列。
 * 运行方式见文件末尾，测试不接触实际卡、文件系统或板级外设。
 */

#include <stdio.h>
#include <string.h>

#define SD_BENCH_TEST
#include "../../shared/components/devices/SdSpi.c"

#define TEST_RX_CAPACITY 4096u

typedef enum
{
    CMD6_UNSUPPORTED = 0,
    CMD6_SUPPORTED,
    CMD6_BUSY,
    CMD6_SELECTION_REJECT,
    CMD6_SWITCH_SELECTION_REJECT,
    CMD6_ILLEGAL,
} Cmd6Case;

static uint8_t TestRx[TEST_RX_CAPACITY];
static uint32_t TestRxHead;
static uint32_t TestRxTail;
static uint8_t TestCmd[6];
static uint8_t TestCmdLen;
static uint32_t TestTick;
static uint8_t TestCsLow;
static uint8_t TestCmd18Blocks;
static uint8_t TestCmd18FailAfter;
static Cmd6Case TestCmd6Case;
static uint32_t TestCmdCount[64];
static uint8_t TestCorruptCrc;

static void TestQueueByte(uint8_t value)
{
    if (TestRxTail < TEST_RX_CAPACITY)
    {
        TestRx[TestRxTail++] = value;
    }
}

static void TestQueueBlock(uint8_t value)
{
    /* Python binascii.crc_hqx 的独立固定向量。 */
    const uint16_t crc = value == 1u ? 58286u : value == 10u ? 47850u : value == 11u ? 22852u : 0u;
    TestQueueByte(0xFEu);
    for (uint32_t i = 0u; i < 512u; i++)
    {
        TestQueueByte(value);
    }
    TestQueueByte((uint8_t)(crc >> 8u));
    TestQueueByte((uint8_t)crc ^ TestCorruptCrc);
}

static void TestQueueCmd6Status(uint8_t switch_mode)
{
    uint8_t status[64] = {0};

    if (TestCmd6Case != CMD6_UNSUPPORTED)
    {
        status[13] = 0x02u;
    }
    if (TestCmd6Case == CMD6_SUPPORTED ||
        (TestCmd6Case == CMD6_SWITCH_SELECTION_REJECT && !switch_mode))
    {
        status[16] = 0x01u;
    }
    if (TestCmd6Case == CMD6_BUSY)
    {
        status[17] = 0x01u;
        status[29] = 0x02u;
    }

    TestQueueByte(0xFEu);
    for (uint32_t i = 0u; i < sizeof(status); i++)
    {
        TestQueueByte(status[i]);
    }
    /* 支持高速状态的独立 CRC 向量为 0x6207；其余旧用例未开CRC。 */
    TestQueueByte(TestCmd6Case == CMD6_SUPPORTED ? 0x62u : 0u);
    TestQueueByte((TestCmd6Case == CMD6_SUPPORTED ? 0x07u : 0u) ^ TestCorruptCrc);
}

static void TestHandleCommand(void)
{
    const uint8_t cmd = TestCmd[0] & 0x3Fu;
    const uint32_t arg = ((uint32_t)TestCmd[1] << 24u) |
                         ((uint32_t)TestCmd[2] << 16u) |
                         ((uint32_t)TestCmd[3] << 8u) |
                         (uint32_t)TestCmd[4];
    TestCmdCount[cmd]++;

    if (cmd == 17u)
    {
        TestQueueByte(0u);
        TestQueueBlock((uint8_t)arg);
    }
    else if (cmd == 18u)
    {
        TestQueueByte(0u);
        for (uint8_t i = 0u; i < TestCmd18Blocks; i++)
        {
            if (i == TestCmd18FailAfter)
            {
                break;
            }
            TestQueueBlock(i);
        }
    }
    else if (cmd == 12u)
    {
        TestQueueByte(0xFFu);
        TestQueueByte(0u);
    }
    else if (cmd == 6u)
    {
        if (TestCmd6Case == CMD6_ILLEGAL)
        {
            TestQueueByte(0x04u);
            return;
        }
        TestQueueByte(0u);
        TestQueueCmd6Status((uint8_t)(arg >> 31u));
    }
}

static void TestReset(void)
{
    memset(TestRx, 0, sizeof(TestRx));
    memset(TestCmd, 0, sizeof(TestCmd));
    memset(TestCmdCount, 0, sizeof(TestCmdCount));
    TestRxHead = 0u;
    TestRxTail = 0u;
    TestCmdLen = 0u;
    TestTick = 0u;
    TestCsLow = 0u;
    TestCmd18Blocks = 0u;
    TestCmd18FailAfter = 0xFFu;
    TestCmd6Case = CMD6_UNSUPPORTED;
    TestCorruptCrc = 0u;

    SdSpiInited = 1u;
    SdSpiType = SD_SPI_TYPE_SDHC;
    SdSpiBenchMultiRead = 0u;
    SdSpiBenchCrc16 = 0u;
}

void SdSpiPortCsHigh(void)
{
    TestCsLow = 0u;
}

void SdSpiPortCsLow(void)
{
    TestCsLow = 1u;
}

uint8_t SdSpiPortTxrx(uint8_t data)
{
    uint8_t response = 0xFFu;

    TestTick++;
    if (TestRxHead < TestRxTail)
    {
        response = TestRx[TestRxHead++];
    }

    if (TestCmdLen != 0u || (data & 0xC0u) == 0x40u)
    {
        TestCmd[TestCmdLen++] = data;
        if (TestCmdLen == sizeof(TestCmd))
        {
            TestHandleCommand();
            TestCmdLen = 0u;
        }
    }

    return response;
}

uint32_t SdSpiPortTickMs(void)
{
    return TestTick;
}

int SdSpiPortTxrxDma(const uint8_t *tx, uint8_t *rx, uint16_t len, uint32_t timeout_ms)
{
    (void)tx;
    (void)rx;
    (void)len;
    (void)timeout_ms;
    return -1;
}

int SdSpiPortReceive(uint8_t *buf, uint16_t len, uint32_t timeout_ms)
{
    (void)timeout_ms;
    for (uint16_t i = 0u; i < len; i++)
    {
        buf[i] = SdSpiPortTxrx(0xFFu);
    }
    return 0;
}

int SdSpiPortTransmit(const uint8_t *buf, uint16_t len, uint32_t timeout_ms)
{
    (void)buf;
    (void)len;
    (void)timeout_ms;
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

int32_t osMutexAcquire(osMutexId_t mutex_id, uint32_t timeout)
{
    (void)mutex_id;
    (void)timeout;
    return 0;
}

int32_t osMutexRelease(osMutexId_t mutex_id)
{
    (void)mutex_id;
    return 0;
}

void osDelay(uint32_t ticks)
{
    TestTick += ticks;
}

static int TestAssert(int condition, const char *name)
{
    if (!condition)
    {
        printf("FAIL: %s\n", name);
        return 1;
    }
    return 0;
}

static int TestDefaultCmd17(void)
{
    uint8_t buf[1024];
    TestReset();

    const int ret = SdSpiRead(buf, 10u, 2u);
    return TestAssert(ret == 0, "CMD17 read returns success") |
           TestAssert(TestCmdCount[17] == 2u, "default uses CMD17 per block") |
           TestAssert(TestCmdCount[18] == 0u, "default does not use CMD18") |
           TestAssert(TestCmdCount[12] == 0u, "default does not use CMD12");
}

static int TestCmd18Stop(void)
{
    uint8_t buf[1024];
    TestReset();
    TestCmd18Blocks = 2u;
    SdSpiBenchSetMultiRead(1u);

    const int ret = SdSpiRead(buf, 10u, 2u);
    return TestAssert(ret == 0, "CMD18 read returns success") |
           TestAssert(TestCmdCount[18] == 1u, "enabled multi read uses CMD18") |
           TestAssert(TestCmdCount[12] == 1u, "CMD18 ends with CMD12") |
           TestAssert(TestCsLow == 0u, "CMD18 releases CS");
}

static int TestCmd18FailureStop(void)
{
    uint8_t buf[1024];
    TestReset();
    TestCmd18Blocks = 2u;
    TestCmd18FailAfter = 1u;
    SdSpiBenchSetMultiRead(1u);

    const int ret = SdSpiRead(buf, 10u, 2u);
    return TestAssert(ret == -4, "CMD18 read data failure is reported") |
           TestAssert(TestCmdCount[12] == 1u, "CMD18 failure still sends CMD12") |
           TestAssert(TestCsLow == 0u, "CMD18 failure releases CS");
}

static int TestCmd6Unsupported(void)
{
    TestReset();

    const int ret = SdSpiBenchHighSpeed();
    return TestAssert(ret == SD_SPI_BENCH_HS_UNSUPPORTED, "CMD6 unsupported is reported") |
           TestAssert(TestCmdCount[6] == 1u, "unsupported card does not receive switch CMD6");
}

static int TestCmd6Success(void)
{
    TestReset();
    TestCmd6Case = CMD6_SUPPORTED;

    const int ret = SdSpiBenchHighSpeed();
    return TestAssert(ret == SD_SPI_BENCH_HS_OK, "CMD6 supported card switches successfully") |
           TestAssert(TestCmdCount[6] == 2u, "supported card receives query and switch CMD6");
}

static int TestCmd6Busy(void)
{
    TestReset();
    TestCmd6Case = CMD6_BUSY;

    const int ret = SdSpiBenchHighSpeed();
    return TestAssert(ret == SD_SPI_BENCH_HS_CHECK_BUSY, "CMD6 busy is reported") |
           TestAssert(TestCmdCount[6] == 1u, "busy card does not receive switch CMD6");
}

static int TestCmd6SelectionReject(void)
{
    TestReset();
    TestCmd6Case = CMD6_SELECTION_REJECT;

    const int ret = SdSpiBenchHighSpeed();
    return TestAssert(ret == SD_SPI_BENCH_HS_CHECK_SELECTION, "CMD6 query selection rejection is reported") |
           TestAssert(TestCmdCount[6] == 1u, "selection rejection does not switch CMD6");
}

static int TestCmd6SwitchSelectionReject(void)
{
    TestReset();
    TestCmd6Case = CMD6_SWITCH_SELECTION_REJECT;

    const int ret = SdSpiBenchHighSpeed();
    return TestAssert(ret == SD_SPI_BENCH_HS_SWITCH_SELECTION, "CMD6 switch selection rejection is reported") |
           TestAssert(TestCmdCount[6] == 2u, "switch selection rejection follows one query");
}

static int TestCrc(void)
{
    uint8_t buffer[1024];
    int failed = TestAssert(SdSpiCrc16((const uint8_t *)"123456789", 9u) == 0x31C3u, "CRC16 known vector");
    TestReset();
    TestCmd18Blocks = 2u;
    SdSpiBenchSetMultiRead(1u);
    SdSpiBenchSetCrc16(1u);
    failed |= TestAssert(SdSpiRead(buffer, 10u, 2u) == 0, "CMD18 valid data CRC accepted");
    TestReset();
    TestCmd18Blocks = 2u;
    TestCorruptCrc = 1u;
    SdSpiBenchSetMultiRead(1u);
    SdSpiBenchSetCrc16(1u);
    failed |= TestAssert(SdSpiRead(buffer, 10u, 2u) != 0, "CMD18 corrupted CRC rejected");
    failed |= TestAssert(TestCmdCount[12] == 1u && TestCsLow == 0u, "CRC failure stops and releases card");
    TestReset();
    TestCmd6Case = CMD6_SUPPORTED;
    SdSpiBenchSetCrc16(1u);
    failed |= TestAssert(SdSpiBenchHighSpeed() == 0, "CMD6 valid CRC accepted");
    TestReset();
    TestCmd6Case = CMD6_SUPPORTED;
    TestCorruptCrc = 1u;
    SdSpiBenchSetCrc16(1u);
    failed |= TestAssert(SdSpiBenchHighSpeed() == SD_SPI_BENCH_HS_CHECK_DATA, "CMD6 corrupted status rejected");
    failed |= TestAssert(TestCmdCount[6] == 1u, "corrupted query does not switch card");
    TestReset();
    TestCmd6Case = CMD6_ILLEGAL;
    failed |= TestAssert(SdSpiBenchHighSpeed() == SD_SPI_BENCH_HS_UNSUPPORTED, "legacy illegal CMD6 means unsupported");
    return failed;
}

int main(void)
{
    int failed = 0;

    failed += TestDefaultCmd17();
    failed += TestCmd18Stop();
    failed += TestCmd18FailureStop();
    failed += TestCmd6Unsupported();
    failed += TestCmd6Success();
    failed += TestCmd6Busy();
    failed += TestCmd6SelectionReject();
    failed += TestCmd6SwitchSelectionReject();
    failed += TestCrc();

    if (failed == 0)
    {
        printf("SdSpi protocol regression: PASS\n");
    }
    return failed == 0 ? 0 : 1;
}

/*
 * Windows PowerShell:
 * clang -std=c11 -Wall -Wextra -I tests/SdBenchM/ProtocolStub -I shared/hal tests/SdBenchM/ProtocolTest.c -o tests/SdBenchM/ProtocolTest.exe
 * .\\tests\\SdBenchM\\ProtocolTest.exe
 */
