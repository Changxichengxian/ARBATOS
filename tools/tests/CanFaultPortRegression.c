/* 直接包含 Zephyr CAN 故障端口源码，以假寄存器验证寄存器级分支。 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "soc.h"
#include <zephyr/drivers/can/can_mcan.h>

TestRcc TestRccInstance;
uint32_t TestPrimask;
uint32_t TestDsbCount;

enum
{
    TestBusCount = 3,
    TestRegWords = 64,
    TestRamWords = 128,
};

typedef enum
{
    TestTxComplete,
    TestTxPending,
} TestTxMode;

static uint32_t TestRegs[TestBusCount][TestRegWords];
static uint32_t TestRam[TestBusCount][TestRamWords];
static uint32_t TestTxbarWrites[TestBusCount];
static uint32_t TestTxbcrWrites[TestBusCount];
static TestTxMode TestMode[TestBusCount];

static int TestBusForRegs(uintptr_t addr)
{
    for (int bus = 0; bus < TestBusCount; bus++)
    {
        const uintptr_t base = 0x1000u * (uintptr_t)(bus + 1);
        if (addr >= base && addr < base + 0x100u)
        {
            return bus;
        }
    }
    return -1;
}

static int TestBusForRam(uintptr_t addr)
{
    for (int bus = 0; bus < TestBusCount; bus++)
    {
        const uintptr_t base = 0x10000u * (uintptr_t)(bus + 1);
        if (addr >= base && addr < base + sizeof(TestRam[bus]))
        {
            return bus;
        }
    }
    return -1;
}

uint32_t sys_read32(uintptr_t addr)
{
    int bus = TestBusForRegs(addr);
    if (bus >= 0)
    {
        return TestRegs[bus][(addr - 0x1000u * (uintptr_t)(bus + 1)) / 4u];
    }
    bus = TestBusForRam(addr);
    if (bus >= 0)
    {
        return TestRam[bus][(addr - 0x10000u * (uintptr_t)(bus + 1)) / 4u];
    }
    return 0u;
}

void sys_write32(uint32_t value, uintptr_t addr)
{
    int bus = TestBusForRegs(addr);
    if (bus >= 0)
    {
        const uint32_t offset = (uint32_t)(addr - 0x1000u * (uintptr_t)(bus + 1));
        TestRegs[bus][offset / 4u] = value;
        if (offset == CAN_MCAN_TXBAR)
        {
            TestTxbarWrites[bus] |= value;
            TestRegs[bus][CAN_MCAN_TXBTO / 4u] &= ~value;
            TestRegs[bus][CAN_MCAN_TXBRP / 4u] |= value;
            if (TestMode[bus] == TestTxComplete)
            {
                TestRegs[bus][CAN_MCAN_TXBRP / 4u] &= ~value;
                TestRegs[bus][CAN_MCAN_TXBTO / 4u] |= value;
            }
        }
        if (offset == CAN_MCAN_TXBCR)
        {
            TestTxbcrWrites[bus] |= value;
            TestRegs[bus][CAN_MCAN_TXBRP / 4u] &= ~value;
        }
        return;
    }
    bus = TestBusForRam(addr);
    if (bus >= 0)
    {
        TestRam[bus][(addr - 0x10000u * (uintptr_t)(bus + 1)) / 4u] = value;
    }
}

#include "../../shared/zephyr/port/can/BspCanFaultPort.c"

static int Check(int condition, const char *message)
{
    if (condition != 0)
    {
        return 1;
    }
    (void)fprintf(stderr, "FAIL: %s\n", message);
    return 0;
}

static void Reset(void)
{
    (void)memset(TestRegs, 0, sizeof(TestRegs));
    (void)memset(TestRam, 0, sizeof(TestRam));
    (void)memset(TestTxbarWrites, 0, sizeof(TestTxbarWrites));
    (void)memset(TestTxbcrWrites, 0, sizeof(TestTxbcrWrites));
    (void)memset(TestMode, 0, sizeof(TestMode));
    TestRccInstance.APB1HENR = RCC_APB1HENR_FDCANEN;
    TestPrimask = 1u;
    TestDsbCount = 0u;
    for (uint32_t bus = 0u; bus < TestBusCount; bus++)
    {
        TestRegs[bus][CAN_MCAN_TXBC / 4u] = 0x100u | (3u << 24u);
        TestRegs[bus][CAN_MCAN_TXESC / 4u] = 7u;
        TestRegs[bus][CAN_MCAN_TXFQS / 4u] = 1u;
    }
}

static int TestSendCompleteAndIsolation(void)
{
    const uint8_t data[3] = {0x11u, 0x22u, 0x33u};
    Reset();
    if (!Check(BspCanFaultPortSupported() == 1u, "H7 端口必须报告支持")) return 0;
    if (!Check(BspCanFaultPortSend(2u, 0x321u, data, 3u) == 0, "空闲总线必须完成故障帧")) return 0;
    if (!Check(TestTxbarWrites[1] == (1u << 1u), "必须只提交 bus2 的硬件槽")) return 0;
    if (!Check(TestTxbarWrites[0] == 0u && TestTxbarWrites[2] == 0u, "三条总线不能串扰")) return 0;
    if (!Check(TestRam[1][(0x100u + 16u) / 4u] == (0x321u << 18u), "标准帧 ID 必须写入选中槽")) return 0;
    return Check(TestRam[1][(0x100u + 16u + 8u) / 4u] == 0x00332211u,
                 "负载必须按小端字节打包");
}

static int TestRejectAndReadyChecks(void)
{
    const uint8_t data[8] = {0};
    Reset();
    TestPrimask = 0u;
    if (!Check(BspCanFaultPortSend(1u, 1u, data, 1u) == 1, "未关中断必须拒绝发送")) return 0;
    TestPrimask = 1u;
    if (!Check(BspCanFaultPortSend(0u, 1u, data, 1u) == 1 &&
               BspCanFaultPortSend(4u, 1u, data, 1u) == 1 &&
               BspCanFaultPortSend(1u, 0x800u, data, 1u) == 1 &&
               BspCanFaultPortSend(1u, 1u, NULL, 1u) == 1 &&
               BspCanFaultPortSend(1u, 1u, data, 9u) == 1,
               "坏 bus、ID、数据和 DLC 必须拒绝")) return 0;
    TestRccInstance.APB1HENR = 0u;
    if (!Check(BspCanFaultPortSend(1u, 1u, data, 1u) == 1, "总线时钟关闭必须拒绝")) return 0;
    Reset();
    TestRegs[0][CAN_MCAN_PSR / 4u] = CAN_MCAN_PSR_BO;
    return Check(BspCanFaultPortSend(1u, 1u, data, 1u) == 1, "bus-off 必须拒绝");
}

static int TestConfigAndPendingBoundaries(void)
{
    const uint8_t data[8] = {0};
    Reset();
    TestRegs[0][CAN_MCAN_TXBC / 4u] ^= 1u;
    if (!Check(BspCanFaultPortSend(1u, 1u, data, 8u) == 1, "MRAM 基址不匹配必须拒绝")) return 0;
    Reset();
    TestRegs[0][CAN_MCAN_TXBC / 4u] = 0x100u | (1u << 16u) | (3u << 24u);
    if (!Check(BspCanFaultPortSend(1u, 1u, data, 8u) == 1, "专用 TX 槽不为零必须拒绝")) return 0;
    Reset();
    TestRegs[0][CAN_MCAN_TXFQS / 4u] = 3u;
    if (!Check(BspCanFaultPortSend(1u, 1u, data, 8u) == 1, "槽号越界必须拒绝")) return 0;
    Reset();
    TestRegs[0][CAN_MCAN_TXBRP / 4u] = 1u;
    return Check(BspCanFaultPortSend(1u, 1u, data, 8u) == 1 && TestTxbarWrites[0] == 0u,
                 "普通 pending 必须直接拒绝，不得退出或覆盖");
}

static int TestOldCompleteTimeoutAndAbort(void)
{
    const uint8_t data[1] = {0};
    Reset();
    TestMode[0] = TestTxPending;
    TestRegs[0][CAN_MCAN_TXBTO / 4u] = 1u;
    if (!Check(BspCanFaultPortSend(1u, 1u, data, 1u) == 3, "旧完成位不能让新帧误报完成")) return 0;
    if (!Check((TestTxbcrWrites[0] & (1u << 1u)) != 0u, "等待超时必须取消新请求")) return 0;
    Reset();
    TestRegs[0][CAN_MCAN_TXBRP / 4u] = 1u;
    TestRegs[2][CAN_MCAN_TXBRP / 4u] = 4u;
    BspCanFaultPortAbort();
    if (!Check(TestTxbcrWrites[0] == 1u && TestTxbcrWrites[2] == 4u, "中止必须取消各总线自身 pending")) return 0;
    return Check(TestTxbcrWrites[1] == 0u && TestDsbCount != 0u,
                 "无 pending 总线不可写取消，退出前必须执行屏障");
}

int main(void)
{
    if (!TestSendCompleteAndIsolation() || !TestRejectAndReadyChecks() ||
        !TestConfigAndPendingBoundaries() || !TestOldCompleteTimeoutAndAbort())
    {
        return 1;
    }
    (void)puts("CAN fault port host regression passed");
    return 0;
}
