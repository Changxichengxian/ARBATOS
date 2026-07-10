/*
 * StateStore 主机回归。直接包含生产实现，以便验证双缓冲忙碌分支。
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

static uint32_t s_test_tick_ms;

uint32_t BspTimeGetTickMs(void)
{
    return s_test_tick_ms;
}

uint32_t BspTimeGetTickUs(void)
{
    return s_test_tick_ms * 1000u;
}

#include "StateStore.c"

static int TestCheck(int condition, const char *message)
{
    if (condition != 0)
    {
        return 1;
    }

    fprintf(stderr, "FAIL: %s\n", message);
    return 0;
}

int main(void)
{
    const uint32_t first[4] = {1u, 2u, 3u, 4u};
    const uint32_t second[4] = {5u, 6u, 7u, 8u};
    uint32_t out[4] = {0u};
    state_info_t info = StateStoreInfo(STATE_CHASSIS);

    if (!TestCheck(info.valid == 0u && info.age_ms == 0u, "初始状态应无有效快照")) return 1;

    s_test_tick_ms = 100u;
    if (!TestCheck(StateStoreWrite(STATE_CHASSIS, first, (uint16_t)sizeof(first)) != 0u,
                   "首次写入失败")) return 1;

    s_test_tick_ms = 107u;
    if (!TestCheck(StateStoreReadSnapshot(STATE_CHASSIS, out, (uint16_t)sizeof(out), &info) != 0u,
                   "快照读取失败")) return 1;
    if (!TestCheck(memcmp(first, out, sizeof(first)) == 0, "快照内容不一致")) return 1;
    if (!TestCheck(info.seq == 1u && info.write_tick_ms == 100u && info.age_ms == 7u,
                   "序号或时间信息错误")) return 1;

    out[0] = 99u;
    if (!TestCheck(StateStoreRead(STATE_CHASSIS, out, (uint16_t)(sizeof(out) - 1u)) == 0u,
                   "长度不匹配应拒绝读取")) return 1;
    if (!TestCheck(out[0] == 99u, "失败读取不应改写输出")) return 1;

    {
        state_slot_t *slot = &s_state_slots[STATE_CHASSIS];
        const uint8_t next_index = (uint8_t)(slot->active_index ^ 1u);
        slot->readers[next_index] = 1u;
        if (!TestCheck(StateStoreWrite(STATE_CHASSIS, second, (uint16_t)sizeof(second)) == 0u,
                       "旧缓冲仍被读取时应拒绝覆盖")) return 1;
        slot->readers[next_index] = 0u;
    }

    info = StateStoreInfo(STATE_CHASSIS);
    if (!TestCheck(info.seq == 1u && info.write_drop_count == 1u,
                   "拒绝写入应保留旧快照并累计诊断")) return 1;

    s_test_tick_ms = 110u;
    if (!TestCheck(StateStoreWrite(STATE_CHASSIS, second, (uint16_t)sizeof(second)) != 0u,
                   "读者释放后写入失败")) return 1;
    if (!TestCheck(StateStoreReadSnapshot(STATE_CHASSIS, out, (uint16_t)sizeof(out), &info) != 0u,
                   "第二次快照读取失败")) return 1;
    if (!TestCheck(memcmp(second, out, sizeof(second)) == 0 && info.seq == 2u,
                   "第二次快照内容或序号错误")) return 1;

    puts("PASS: StateStore host regression");
    return 0;
}
