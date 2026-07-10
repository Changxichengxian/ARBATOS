/*
 * LowCmd 主机回归。直接包含生产实现，验证批量清除、持续禁写和优先级边界。
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

typedef uint32_t UBaseType_t;

static uint32_t s_test_tick_ms;

static inline uint32_t __get_IPSR(void)
{
    return 0u;
}

#define taskENTER_CRITICAL_FROM_ISR() ((UBaseType_t)0u)
#define taskEXIT_CRITICAL_FROM_ISR(savedMask) ((void)(savedMask))

uint32_t HAL_GetTick(void)
{
    return s_test_tick_ms;
}

#include "LowCmd.c"
#include "MotorInstBestEffort.h"
#include "CanTxCommandPolicy.h"

static int TestCheck(int condition, const char *message)
{
    if (condition != 0)
    {
        return 1;
    }

    (void)fprintf(stderr, "FAIL: %s\n", message);
    return 0;
}

static MotorCmd TestCurrentCmd(int16_t current)
{
    MotorCmd cmd;

    (void)memset(&cmd, 0, sizeof(cmd));
    cmd.active = 1u;
    cmd.mode = (uint8_t)MotorModeCurrent;
    cmd.current = current;
    return cmd;
}

static int TestRead(MotorId id, MotorCmd *out)
{
    return TestCheck(LowCmdGetMotor(id, out) != 0u, "读取电机命令失败");
}

static int TestClearManySuccess(void)
{
    const MotorId ids[3] = {Motor0, Motor2, Motor4};
    MotorCmd cmds[3] = {
        TestCurrentCmd(100),
        TestCurrentCmd(200),
        TestCurrentCmd(300),
    };
    MotorCmd untouched_before;
    MotorCmd untouched_after;
    uint32_t seq_before;
    uint32_t seq_after;

    LowCmdClearAll();
    s_test_tick_ms = 10u;
    if (!TestCheck(LowCmdSetMotorManyFrom(ids, cmds, 3u, LOWCMD_WRITER_CONTROL) != 0u,
                   "准备批量命令失败")) return 0;
    {
        const MotorCmd untouched = TestCurrentCmd(444);
        if (!TestCheck(LowCmdSetMotorFrom(Motor5, &untouched, LOWCMD_WRITER_CONTROL) != 0u,
                       "准备无关命令失败")) return 0;
    }
    if (!TestRead(Motor5, &untouched_before)) return 0;

    seq_before = LowCmdSeq();
    s_test_tick_ms = 11u;
    if (!TestCheck(LowCmdClearManyFrom(ids, 3u, LOWCMD_WRITER_SAFETY) != 0u,
                   "一次清除多个命令失败")) return 0;
    seq_after = LowCmdSeq();
    if (!TestCheck(seq_after == seq_before + 1u, "批量清除只能增加一次全局序号")) return 0;

    for (uint8_t i = 0u; i < 3u; i++)
    {
        MotorCmd cleared;
        if (!TestRead(ids[i], &cleared)) return 0;
        if (!TestCheck(cleared.active == 0u &&
                       cleared.mode == (uint8_t)MotorModeNone &&
                       cleared.current == 0 &&
                       cleared.seq == seq_after &&
                       cleared.tick == 11u,
                       "批量清除结果或统一时间戳错误")) return 0;
    }

    if (!TestRead(Motor5, &untouched_after)) return 0;
    return TestCheck(memcmp(&untouched_before, &untouched_after, sizeof(untouched_before)) == 0,
                     "批量清除不应改动未列出的命令");
}

static int TestInvalidBatchIsNoOp(void)
{
    const MotorId valid_ids[2] = {Motor1, Motor3};
    const MotorId duplicate_ids[2] = {Motor1, Motor1};
    const MotorId invalid_ids[2] = {Motor1, MotorCount};
    MotorCmd cmds[2] = {TestCurrentCmd(111), TestCurrentCmd(333)};
    LowCmd before;
    LowCmd after;
    LowCmdDiag diag_before;
    LowCmdDiag diag_after;

    LowCmdClearAll();
    s_test_tick_ms = 20u;
    if (!TestCheck(LowCmdSetMotorManyFrom(valid_ids, cmds, 2u, LOWCMD_WRITER_CONTROL) != 0u,
                   "准备非法批次测试失败")) return 0;
    if (!TestCheck(LowCmdGet(&before) != 0u && LowCmdGetDiag(&diag_before) != 0u,
                   "读取非法批次测试前状态失败")) return 0;

    if (!TestCheck(LowCmdClearManyFrom(duplicate_ids, 2u, LOWCMD_WRITER_SAFETY) == 0u,
                   "重复电机 ID 应拒绝整批清除")) return 0;
    if (!TestCheck(LowCmdClearManyFrom(invalid_ids, 2u, LOWCMD_WRITER_SAFETY) == 0u,
                   "越界电机 ID 应拒绝整批清除")) return 0;
    if (!TestCheck(LowCmdClearManyFrom(NULL, 1u, LOWCMD_WRITER_SAFETY) == 0u,
                   "非空批次缺少 ID 数组应拒绝")) return 0;
    if (!TestCheck(LowCmdClearManyFrom(valid_ids,
                                       (uint8_t)((uint8_t)MotorCount + 1u),
                                       LOWCMD_WRITER_SAFETY) == 0u,
                   "超过电机总数的批次应拒绝")) return 0;
    if (!TestCheck(LowCmdInhibitManyFrom(duplicate_ids, 2u, LOWCMD_WRITER_SAFETY) == 0u &&
                   LowCmdInhibitManyFrom(invalid_ids, 2u, LOWCMD_WRITER_SAFETY) == 0u &&
                   LowCmdReleaseInhibitManyFrom(duplicate_ids, 2u, LOWCMD_WRITER_SAFETY) == 0u &&
                   LowCmdReleaseInhibitManyFrom(invalid_ids, 2u, LOWCMD_WRITER_SAFETY) == 0u,
                   "禁写与释放也应拒绝重复或越界 ID")) return 0;

    if (!TestCheck(LowCmdGet(&after) != 0u && LowCmdGetDiag(&diag_after) != 0u,
                   "读取非法批次测试后状态失败")) return 0;
    if (!TestCheck(before.seq == after.seq &&
                   memcmp(before.motorCmd, after.motorCmd, sizeof(before.motorCmd)) == 0,
                   "非法批次不应改动任何命令或序号")) return 0;
    return TestCheck(memcmp(&diag_before, &diag_after, sizeof(diag_before)) == 0,
                     "参数校验失败不应改动诊断状态");
}

static int TestPriorityRejectIsAtomic(void)
{
    const MotorId ids[2] = {Motor0, Motor1};
    const MotorCmd low_cmd = TestCurrentCmd(10);
    const MotorCmd high_cmd = TestCurrentCmd(20);
    MotorCmd before[2];
    MotorCmd after[2];
    LowCmdDiag diag_before;
    LowCmdDiag diag_after;
    uint32_t seq_before;

    LowCmdClearAll();
    s_test_tick_ms = 30u;
    if (!TestCheck(LowCmdSetMotorFrom(Motor0, &low_cmd, LOWCMD_WRITER_CONTROL) != 0u &&
                   LowCmdSetMotorFrom(Motor1, &high_cmd, LOWCMD_WRITER_SAFETY) != 0u,
                   "准备优先级测试失败")) return 0;
    if (!TestCheck(LowCmdGetMotorMany(ids, before, 2u) != 0u &&
                   LowCmdGetDiag(&diag_before) != 0u,
                   "读取优先级测试前状态失败")) return 0;
    seq_before = LowCmdSeq();

    s_test_tick_ms = 31u;
    if (!TestCheck(LowCmdClearManyFrom(ids, 2u, LOWCMD_WRITER_CONTROL) == 0u,
                   "低优先级不应清除仍在保护期内的高优先级命令")) return 0;
    if (!TestCheck(LowCmdGetMotorMany(ids, after, 2u) != 0u &&
                   LowCmdGetDiag(&diag_after) != 0u,
                   "读取优先级测试后状态失败")) return 0;
    if (!TestCheck(LowCmdSeq() == seq_before && memcmp(before, after, sizeof(before)) == 0,
                   "批内任一命令无权清除时应原子拒绝整批")) return 0;
    return TestCheck(diag_after.rejected_count == diag_before.rejected_count + 1u &&
                     diag_after.last_reject_writer == (uint16_t)LOWCMD_WRITER_CONTROL &&
                     diag_after.last_reject_owner == (uint16_t)LOWCMD_WRITER_SAFETY,
                     "优先级拒绝诊断信息错误");
}

static int TestSafetyMayClearControl(void)
{
    const MotorCmd cmd = TestCurrentCmd(123);
    MotorCmd cleared;
    uint32_t seq_before;

    LowCmdClearAll();
    s_test_tick_ms = 40u;
    if (!TestCheck(LowCmdSetMotorFrom(Motor6, &cmd, LOWCMD_WRITER_CONTROL) != 0u,
                   "准备安全清除测试失败")) return 0;
    seq_before = LowCmdSeq();

    s_test_tick_ms = 41u;
    if (!TestCheck(LowCmdClearManyFrom((const MotorId[]){Motor6}, 1u, LOWCMD_WRITER_SAFETY) != 0u,
                   "SAFETY 应能清除 CONTROL 命令")) return 0;
    if (!TestRead(Motor6, &cleared)) return 0;
    return TestCheck(cleared.active == 0u && LowCmdSeq() == seq_before + 1u,
                     "SAFETY 清除 CONTROL 后状态错误");
}

static int TestEmergencyLockRejectsSafetyClear(void)
{
    const MotorId ids[2] = {Motor7, Motor8};
    MotorCmd before[2];
    MotorCmd after[2];
    LowCmdDiag diag_before;
    LowCmdDiag diag_after;
    uint32_t seq_before;

    LowCmdClearAll();
    s_test_tick_ms = 50u;
    if (!TestCheck(LowCmdEnterEmergencyStop(LOWCMD_WRITER_FAULT) != 0u,
                   "进入 FAULT 急停失败")) return 0;
    if (!TestCheck(LowCmdGetMotorMany(ids, before, 2u) != 0u &&
                   LowCmdGetDiag(&diag_before) != 0u,
                   "读取急停清除前状态失败")) return 0;
    seq_before = LowCmdSeq();

    s_test_tick_ms = 51u;
    if (!TestCheck(LowCmdClearManyFrom(ids, 2u, LOWCMD_WRITER_SAFETY) == 0u,
                   "FAULT 急停锁下应拒绝 SAFETY 清除")) return 0;
    if (!TestCheck(LowCmdGetMotorMany(ids, after, 2u) != 0u &&
                   LowCmdGetDiag(&diag_after) != 0u,
                   "读取急停清除后状态失败")) return 0;
    if (!TestCheck(LowCmdSeq() == seq_before && memcmp(before, after, sizeof(before)) == 0,
                   "急停锁拒绝清除时不应改动命令")) return 0;
    return TestCheck(diag_after.rejected_count == diag_before.rejected_count + 1u &&
                     diag_after.last_reject_writer == (uint16_t)LOWCMD_WRITER_SAFETY &&
                     diag_after.last_reject_owner == (uint16_t)LOWCMD_WRITER_FAULT &&
                     diag_after.emergency_active != 0u,
                     "急停锁拒绝诊断信息错误");
}

static int TestZeroCountIsNoOp(void)
{
    const MotorCmd cmd = TestCurrentCmd(777);
    const MotorId unused_id = Motor9;
    LowCmd before;
    LowCmd after;
    LowCmdDiag diag_before;
    LowCmdDiag diag_after;

    LowCmdClearAll();
    s_test_tick_ms = 60u;
    if (!TestCheck(LowCmdSetMotorFrom(Motor9, &cmd, LOWCMD_WRITER_CONTROL) != 0u,
                   "准备空批次测试失败")) return 0;
    if (!TestCheck(LowCmdEnterEmergencyStop(LOWCMD_WRITER_FAULT) != 0u,
                   "准备空批次急停状态失败")) return 0;
    if (!TestCheck(LowCmdGet(&before) != 0u && LowCmdGetDiag(&diag_before) != 0u,
                   "读取空批次测试前状态失败")) return 0;

    s_test_tick_ms = 61u;
    if (!TestCheck(LowCmdClearManyFrom(NULL, 0u, LOWCMD_WRITER_SAFETY) != 0u &&
                   LowCmdClearManyFrom(&unused_id, 0u, LOWCMD_WRITER_CONTROL) != 0u &&
                   LowCmdInhibitManyFrom(NULL, 0u, LOWCMD_WRITER_SAFETY) != 0u &&
                   LowCmdReleaseInhibitManyFrom(NULL, 0u, LOWCMD_WRITER_CONTROL) != 0u,
                   "count 为 0 时应允许空指针并成功返回")) return 0;
    if (!TestCheck(LowCmdGet(&after) != 0u && LowCmdGetDiag(&diag_after) != 0u,
                   "读取空批次测试后状态失败")) return 0;
    if (!TestCheck(before.seq == after.seq &&
                   memcmp(before.motorCmd, after.motorCmd, sizeof(before.motorCmd)) == 0,
                   "count 为 0 时不应改动命令或序号")) return 0;
    return TestCheck(memcmp(&diag_before, &diag_after, sizeof(diag_before)) == 0,
                     "count 为 0 时不应改动诊断或急停状态");
}

static int TestInhibitClearsAndBlocksLowerWriters(void)
{
    const MotorId ids[2] = {Motor0, Motor1};
    MotorCmd cmds[2] = {TestCurrentCmd(100), TestCurrentCmd(200)};
    const MotorCmd untouched_cmd = TestCurrentCmd(300);
    MotorCmd cleared[2];
    MotorCmd untouched_before;
    MotorCmd untouched_after;
    MotorCmd after_reject[2];
    MotorCmd before_idempotent[2];
    MotorCmd after_idempotent[2];
    LowCmdDiag diag_before_idempotent;
    LowCmdDiag diag_after_idempotent;
    uint16_t inhibit_writer = 0u;
    uint32_t seq_before;
    uint32_t seq_after;

    LowCmdClearAll();
    s_test_tick_ms = 70u;
    if (!TestCheck(LowCmdSetMotorManyFrom(ids, cmds, 2u, LOWCMD_WRITER_CONTROL) != 0u &&
                   LowCmdSetMotorFrom(Motor2, &untouched_cmd, LOWCMD_WRITER_CONTROL) != 0u,
                   "准备持续禁写测试失败")) return 0;
    if (!TestRead(Motor2, &untouched_before)) return 0;

    seq_before = LowCmdSeq();
    s_test_tick_ms = 71u;
    if (!TestCheck(LowCmdInhibitManyFrom(ids, 2u, LOWCMD_WRITER_SAFETY) != 0u,
                   "SAFETY 获取批量禁写失败")) return 0;
    seq_after = LowCmdSeq();
    if (!TestCheck(seq_after == seq_before + 1u,
                   "批量禁写清空命令时只能增加一次全局序号")) return 0;
    if (!TestCheck(LowCmdGetMotorMany(ids, cleared, 2u) != 0u,
                   "读取禁写后的命令失败")) return 0;
    for (uint8_t i = 0u; i < 2u; i++)
    {
        if (!TestCheck(cleared[i].active == 0u &&
                       cleared[i].seq == seq_after &&
                       cleared[i].tick == 71u &&
                       cleared[i].writer == (uint16_t)LOWCMD_WRITER_SAFETY,
                       "获取禁写必须原子清空所选命令")) return 0;
        if (!TestCheck(LowCmdGetInhibitWriter(ids[i], &inhibit_writer) != 0u &&
                       inhibit_writer == (uint16_t)LOWCMD_WRITER_SAFETY,
                       "每个电机应保存独立禁写 writer")) return 0;
    }
    if (!TestCheck(LowCmdGetDiag(&diag_after_idempotent) != 0u &&
                   diag_after_idempotent.inhibit_mask == ((1ul << Motor0) | (1ul << Motor1)) &&
                   diag_after_idempotent.inhibit_acquire_count == 1u &&
                   diag_after_idempotent.inhibit_release_count == 0u,
                   "禁写位图或获取计数错误")) return 0;
    if (!TestRead(Motor2, &untouched_after)) return 0;
    if (!TestCheck(memcmp(&untouched_before, &untouched_after, sizeof(untouched_before)) == 0,
                   "批量禁写不应改动未列出的电机")) return 0;

    s_test_tick_ms = 72u;
    if (!TestCheck(LowCmdSetMotorManyFrom(ids, cmds, 2u, LOWCMD_WRITER_CONTROL) == 0u &&
                   LowCmdSetMotorManyFrom(ids, cmds, 2u, LOWCMD_WRITER_HOST) == 0u,
                   "CONTROL 与 HOST 在命令 inactive 时仍应被持续禁写")) return 0;
    if (!TestCheck(LowCmdGetMotorMany(ids, after_reject, 2u) != 0u &&
                   memcmp(cleared, after_reject, sizeof(cleared)) == 0,
                   "低优先级写入被拒绝后命令必须保持不变")) return 0;

    if (!TestCheck(LowCmdSetMotorFrom(Motor0, &cmds[0], LOWCMD_WRITER_SAFETY) != 0u &&
                   LowCmdSetMotorFrom(Motor1, &cmds[1], LOWCMD_WRITER_FAULT) != 0u,
                   "禁写 owner 同级或更高 writer 应保留安全接管能力")) return 0;
    if (!TestCheck(LowCmdGetInhibitWriter(Motor1, &inhibit_writer) != 0u &&
                   inhibit_writer == (uint16_t)LOWCMD_WRITER_SAFETY,
                   "更高 writer 写命令不应偷偷释放局部禁写")) return 0;

    if (!TestCheck(LowCmdGetMotorMany(ids, before_idempotent, 2u) != 0u &&
                   LowCmdGetDiag(&diag_before_idempotent) != 0u,
                   "读取幂等获取前状态失败")) return 0;
    seq_before = LowCmdSeq();
    if (!TestCheck(LowCmdInhibitManyFrom(ids, 2u, LOWCMD_WRITER_SAFETY) != 0u,
                   "同 writer 重复获取禁写应成功")) return 0;
    if (!TestCheck(LowCmdGetMotorMany(ids, after_idempotent, 2u) != 0u &&
                   LowCmdGetDiag(&diag_after_idempotent) != 0u &&
                   LowCmdSeq() == seq_before &&
                   memcmp(before_idempotent, after_idempotent, sizeof(before_idempotent)) == 0 &&
                   diag_after_idempotent.inhibit_acquire_count ==
                       diag_before_idempotent.inhibit_acquire_count,
                   "同 writer 重复获取不得清命令、增长序号或重复计数")) return 0;
    return 1;
}

static int TestInhibitAcquireRejectIsAtomic(void)
{
    const MotorId ids[2] = {Motor3, Motor4};
    const MotorCmd control_cmd = TestCurrentCmd(30);
    const MotorCmd fault_cmd = TestCurrentCmd(40);
    MotorCmd before[2];
    MotorCmd after[2];
    LowCmdDiag diag;
    uint16_t inhibit_writer = 1u;
    uint32_t seq_before;

    LowCmdClearAll();
    s_test_tick_ms = 80u;
    if (!TestCheck(LowCmdSetMotorFrom(Motor3, &control_cmd, LOWCMD_WRITER_CONTROL) != 0u &&
                   LowCmdSetMotorFrom(Motor4, &fault_cmd, LOWCMD_WRITER_FAULT) != 0u,
                   "准备禁写原子拒绝测试失败")) return 0;
    if (!TestCheck(LowCmdGetMotorMany(ids, before, 2u) != 0u,
                   "读取禁写原子拒绝前状态失败")) return 0;
    seq_before = LowCmdSeq();

    s_test_tick_ms = 81u;
    if (!TestCheck(LowCmdInhibitManyFrom(ids, 2u, LOWCMD_WRITER_SAFETY) == 0u,
                   "SAFETY 不应覆盖仍在保护期内的 FAULT 命令")) return 0;
    if (!TestCheck(LowCmdGetMotorMany(ids, after, 2u) != 0u &&
                   LowCmdSeq() == seq_before &&
                   memcmp(before, after, sizeof(before)) == 0,
                   "批内任一电机不可禁写时整批命令必须不变")) return 0;
    if (!TestCheck(LowCmdGetInhibitWriter(Motor3, &inhibit_writer) != 0u &&
                   inhibit_writer == (uint16_t)LOWCMD_WRITER_NONE,
                   "原子拒绝不能给前面的电机留下半批禁写")) return 0;
    if (!TestCheck(LowCmdGetInhibitWriter(Motor4, &inhibit_writer) != 0u &&
                   inhibit_writer == (uint16_t)LOWCMD_WRITER_NONE,
                   "原子拒绝不能给后面的电机留下禁写")) return 0;
    return TestCheck(LowCmdGetDiag(&diag) != 0u &&
                     diag.inhibit_mask == 0u &&
                     diag.inhibit_acquire_count == 0u,
                     "获取被拒绝时禁写诊断也不应产生半批状态");
}

static int TestReleaseRejectIsAtomic(void)
{
    const MotorId ids[2] = {Motor5, Motor6};
    MotorCmd cmds[2] = {TestCurrentCmd(50), TestCurrentCmd(60)};
    LowCmdDiag diag;
    uint16_t writer5 = 0u;
    uint16_t writer6 = 0u;
    uint32_t seq_before;

    LowCmdClearAll();
    s_test_tick_ms = 90u;
    if (!TestCheck(LowCmdInhibitManyFrom(ids, 2u, LOWCMD_WRITER_SAFETY) != 0u &&
                   LowCmdInhibitManyFrom(&ids[1], 1u, LOWCMD_WRITER_FAULT) != 0u,
                   "准备分级禁写失败")) return 0;
    seq_before = LowCmdSeq();
    if (!TestCheck(LowCmdGetDiag(&diag) != 0u &&
                   diag.inhibit_mask == ((1ul << Motor5) | (1ul << Motor6)) &&
                   diag.inhibit_acquire_count == 2u,
                   "升级禁写后的位图或获取计数错误")) return 0;

    s_test_tick_ms = 91u;
    if (!TestCheck(LowCmdReleaseInhibitManyFrom(ids, 2u, LOWCMD_WRITER_SAFETY) == 0u,
                   "低优先级不应释放批内 FAULT 禁写")) return 0;
    if (!TestCheck(LowCmdGetInhibitWriter(Motor5, &writer5) != 0u &&
                   LowCmdGetInhibitWriter(Motor6, &writer6) != 0u &&
                   writer5 == (uint16_t)LOWCMD_WRITER_SAFETY &&
                   writer6 == (uint16_t)LOWCMD_WRITER_FAULT &&
                   LowCmdSeq() == seq_before,
                   "释放被拒绝时整批禁写和命令序号都必须不变")) return 0;
    if (!TestCheck(LowCmdGetDiag(&diag) != 0u &&
                   diag.inhibit_release_count == 0u &&
                   diag.inhibit_mask == ((1ul << Motor5) | (1ul << Motor6)),
                   "释放被拒绝时不应累计释放或改变位图")) return 0;
    if (!TestCheck(LowCmdSetMotorFrom(Motor5, &cmds[0], LOWCMD_WRITER_CONTROL) == 0u,
                   "释放原子拒绝后前项仍应保持禁写")) return 0;

    if (!TestCheck(LowCmdReleaseInhibitManyFrom(ids, 2u, LOWCMD_WRITER_FAULT) != 0u,
                   "FAULT 应能整批释放较低或同级禁写")) return 0;
    if (!TestCheck(LowCmdGetInhibitWriter(Motor5, &writer5) != 0u &&
                   LowCmdGetInhibitWriter(Motor6, &writer6) != 0u &&
                   writer5 == (uint16_t)LOWCMD_WRITER_NONE &&
                   writer6 == (uint16_t)LOWCMD_WRITER_NONE &&
                   LowCmdSeq() == seq_before,
                   "释放只撤销禁写，不应改命令序号")) return 0;
    if (!TestCheck(LowCmdGetDiag(&diag) != 0u &&
                   diag.inhibit_release_count == 1u &&
                   diag.inhibit_mask == 0u,
                   "成功释放后的计数或位图错误")) return 0;
    if (!TestCheck(LowCmdReleaseInhibitManyFrom(ids, 2u, LOWCMD_WRITER_CONTROL) != 0u &&
                   LowCmdGetDiag(&diag) != 0u &&
                   diag.inhibit_release_count == 1u,
                   "重复释放已解除的禁写应幂等且不重复计数")) return 0;
    return TestCheck(LowCmdSetMotorManyFrom(ids, cmds, 2u, LOWCMD_WRITER_CONTROL) != 0u,
                     "释放后 CONTROL 应恢复整批写入");
}

static int TestEmergencyAndLocalInhibitStack(void)
{
    const MotorId id = Motor7;
    const MotorCmd cmd = TestCurrentCmd(70);
    LowCmdDiag diag;
    uint16_t inhibit_writer = 0u;

    LowCmdClearAll();
    s_test_tick_ms = 100u;
    if (!TestCheck(LowCmdInhibitManyFrom(&id, 1u, LOWCMD_WRITER_SAFETY) != 0u,
                   "准备局部禁写失败")) return 0;
    if (!TestCheck(LowCmdEnterEmergencyStop(LOWCMD_WRITER_FAULT) != 0u,
                   "进入全局急停失败")) return 0;
    if (!TestCheck(LowCmdGetInhibitWriter(id, &inhibit_writer) != 0u &&
                   inhibit_writer == (uint16_t)LOWCMD_WRITER_SAFETY,
                   "全局急停不应覆盖或清除局部禁写 owner")) return 0;
    if (!TestCheck(LowCmdInhibitManyFrom((const MotorId[]){Motor8}, 1u, LOWCMD_WRITER_SAFETY) == 0u,
                   "FAULT 急停期间 SAFETY 不应新增清空型禁写")) return 0;

    if (!TestCheck(LowCmdClearEmergencyStop(LOWCMD_WRITER_FAULT) != 0u,
                   "解除全局急停失败")) return 0;
    s_test_tick_ms = 201u;
    if (!TestCheck(LowCmdSetMotorFrom(id, &cmd, LOWCMD_WRITER_CONTROL) == 0u,
                   "全局急停解除后局部禁写仍应继续生效")) return 0;
    if (!TestCheck(LowCmdReleaseInhibitManyFrom(&id, 1u, LOWCMD_WRITER_SAFETY) != 0u &&
                   LowCmdSetMotorFrom(id, &cmd, LOWCMD_WRITER_CONTROL) != 0u,
                   "显式释放局部禁写后 CONTROL 应恢复")) return 0;

    if (!TestCheck(LowCmdInhibitManyFrom(&id, 1u, LOWCMD_WRITER_FAULT) != 0u,
                   "准备 ClearAll 禁写清理测试失败")) return 0;
    LowCmdClearAll();
    return TestCheck(LowCmdGetInhibitWriter(id, &inhibit_writer) != 0u &&
                     LowCmdGetDiag(&diag) != 0u &&
                     inhibit_writer == (uint16_t)LOWCMD_WRITER_NONE &&
                     diag.inhibit_mask == 0u &&
                     diag.inhibit_acquire_count == 0u &&
                     diag.inhibit_release_count == 0u &&
                     LowCmdEmergencyActive() == 0u,
                     "LowCmdClearAll 应同时清除全局急停、禁写和诊断计数");
}

static int TestBestEffortSkipsInhibitedAxis(void)
{
    const MotorId ids[3] = {Motor0, Motor1, Motor2};
    const int16_t currents[3] = {100, 200, 300};
    MotorCmd trigger;
    MotorCmd yaw;
    MotorCmd pitch;

    LowCmdClearAll();
    s_test_tick_ms = 220u;
    if (!TestCheck(LowCmdInhibitManyFrom(&ids[0], 1u, LOWCMD_WRITER_SAFETY) != 0u,
                   "准备 BestEffort 单轴禁写失败")) return 0;
    if (!TestCheck(MotorInstLowCmdSetCurrentBestEffort(ids, currents, 3u) == 2u,
                   "BestEffort 应在批量拒绝后继续写两个健康轴")) return 0;
    if (!TestRead(Motor0, &trigger) || !TestRead(Motor1, &yaw) || !TestRead(Motor2, &pitch)) return 0;

    return TestCheck(trigger.active == 0u &&
                     yaw.active != 0u && yaw.mode == (uint8_t)MotorModeCurrent && yaw.current == 200 &&
                     pitch.active != 0u && pitch.mode == (uint8_t)MotorModeCurrent && pitch.current == 300,
                     "trigger 禁写不得拖住 yaw/pitch，且 trigger 必须保持 inactive");
}

static int TestCanTxRejectsStaleCachedCommand(void)
{
    MotorCmd cached = TestCurrentCmd(100);
    MotorCmd latest;

    cached.writer = (uint16_t)LOWCMD_WRITER_CONTROL;
    cached.seq = 10u;
    latest = cached;
    if (!TestCheck(CanTxCachedCmdAuthorized(&cached,
                                            &latest,
                                            (uint16_t)LOWCMD_WRITER_NONE) != 0u,
                   "未变化且无禁写的缓存命令应允许发送")) return 0;

    latest.active = 0u;
    latest.seq = 11u;
    latest.writer = (uint16_t)LOWCMD_WRITER_SAFETY;
    if (!TestCheck(CanTxCachedCmdAuthorized(&cached,
                                            &latest,
                                            (uint16_t)LOWCMD_WRITER_SAFETY) == 0u,
                   "缓存后发生禁写清空时旧 active 命令必须被拒绝")) return 0;

    latest = cached;
    if (!TestCheck(CanTxCachedCmdAuthorized(&cached,
                                            &latest,
                                            (uint16_t)LOWCMD_WRITER_SAFETY) == 0u,
                   "低于 inhibit owner 的缓存 writer 必须被拒绝")) return 0;

    cached.writer = (uint16_t)LOWCMD_WRITER_SAFETY;
    cached.seq = 12u;
    latest = cached;
    if (!TestCheck(CanTxCachedCmdAuthorized(&cached,
                                            &latest,
                                            (uint16_t)LOWCMD_WRITER_SAFETY) != 0u,
                   "同级 SAFETY 安全命令应允许通过禁写门")) return 0;

    latest.seq++;
    return TestCheck(CanTxCachedCmdAuthorized(&cached,
                                              &latest,
                                              (uint16_t)LOWCMD_WRITER_SAFETY) == 0u,
                     "发送前 latest seq 改变时必须拒绝旧缓存");
}

static int TestCanTxCommandExpiryBoundaries(void)
{
    MotorCmd cmd = TestCurrentCmd(100);
    CanTxCmdExpiryLatch latch = {0};

    cmd.timeoutMs = 100u;
    cmd.tick = 0u;
    if (!TestCheck(CanTxCmdExpired(&cmd, 100u) == 0u,
                   "tick=0 发布的命令在超时边界上不应提前过期") ||
        !TestCheck(CanTxCmdExpired(&cmd, 101u) != 0u,
                   "tick=0 是合法发布时刻，超时后必须过期"))
    {
        return 0;
    }

    cmd.tick = UINT32_MAX - 5u;
    cmd.timeoutMs = 10u;
    if (!TestCheck(CanTxCmdExpired(&cmd, 3u) == 0u,
                   "计数回绕后的新鲜命令不应被误判过期") ||
        !TestCheck(CanTxCmdExpired(&cmd, 6u) != 0u,
                   "计数回绕后仍应按实际年龄超时"))
    {
        return 0;
    }

    cmd.tick = 101u;
    cmd.timeoutMs = 10u;
    if (!TestCheck(CanTxCmdExpired(&cmd, 100u) == 0u,
                   "读取与发布的一毫秒竞态应被容忍"))
    {
        return 0;
    }
    cmd.tick = 102u;
    if (!TestCheck(CanTxCmdExpired(&cmd, 100u) != 0u,
                   "不得把超出竞态窗口的异常时间永久当成新命令"))
    {
        return 0;
    }

    cmd.seq = 7u;
    cmd.tick = 0u;
    cmd.timeoutMs = 10u;
    if (!TestCheck(CanTxCmdExpiryLatchCheck(&latch, &cmd, 11u) != 0u && latch.valid != 0u,
                   "命令首次过期时应锁定发布代") ||
        !TestCheck(CanTxCmdExpired(&cmd, 5u) == 0u &&
                       CanTxCmdExpiryLatchCheck(&latch, &cmd, 5u) != 0u,
                   "完整 tick 回绕不得复活已过期的同代命令"))
    {
        return 0;
    }
    {
        MotorCmd local_disable;

        (void)memset(&local_disable, 0, sizeof(local_disable));
        local_disable.active = 1u;
        local_disable.mode = (uint8_t)MotorModeDisable;
        local_disable.tick = 12u;
        if (!TestCheck(CanTxCmdExpiryLatchCheck(&latch, &local_disable, 12u) == 0u &&
                           latch.valid != 0u && latch.seq == 7u,
                       "CanTx 本地 Disable 不得清掉原命令的过期锁定") ||
            !TestCheck(CanTxCmdExpiryLatchCheck(&latch, &cmd, 5u) != 0u,
                       "经过本地 Disable 后完整 tick 回绕仍不得复活旧命令"))
        {
            return 0;
        }
    }
    cmd.seq = 8u;
    cmd.tick = 5u;
    if (!TestCheck(CanTxCmdExpiryLatchCheck(&latch, &cmd, 5u) == 0u && latch.valid == 0u,
                   "新发布代应清除旧命令的过期锁定"))
    {
        return 0;
    }

    cmd.timeoutMs = 0u;
    if (!TestCheck(CanTxCmdExpired(&cmd, 1000u) == 0u,
                   "timeout=0 的显式持续命令不应自动过期"))
    {
        return 0;
    }
    cmd.active = 0u;
    cmd.timeoutMs = 1u;
    return TestCheck(CanTxCmdExpired(&cmd, 1000u) == 0u,
                     "inactive 命令不需要标记超时");
}

static int TestCanTxUnlockGenerationBarrier(void)
{
    CanTxCmdUnlockBarrier barrier = {0};
    MotorCmd beforeUnlock = TestCurrentCmd(100);
    MotorCmd afterUnlock;

    beforeUnlock.active = 1u;
    beforeUnlock.seq = 41u;
    beforeUnlock.writer = (uint16_t)LOWCMD_WRITER_CONTROL;
    CanTxCmdUnlockBarrierCapture(&barrier, &beforeUnlock);
    if (!TestCheck(CanTxCmdPublishedAfterUnlock(&barrier, &beforeUnlock) == 0u,
                   "解锁前已存在的同代命令不得立即复活")) return 0;

    afterUnlock = beforeUnlock;
    afterUnlock.seq++;
    if (!TestCheck(CanTxCmdPublishedAfterUnlock(&barrier, &afterUnlock) != 0u,
                   "控制任务重新发布一代后才允许越过解锁屏障")) return 0;

    afterUnlock = beforeUnlock;
    afterUnlock.writer = (uint16_t)LOWCMD_WRITER_SAFETY;
    if (!TestCheck(CanTxCmdPublishedAfterUnlock(&barrier, &afterUnlock) != 0u,
                   "写者切换产生的新命令也应视为解锁后的发布")) return 0;

    beforeUnlock.active = 0u;
    CanTxCmdUnlockBarrierCapture(&barrier, &beforeUnlock);
    afterUnlock = beforeUnlock;
    afterUnlock.active = 1u;
    afterUnlock.seq = 1u;
    return TestCheck(CanTxCmdPublishedAfterUnlock(&barrier, &afterUnlock) != 0u,
                     "解锁时没有活动命令的轴应接受之后首次发布");
}

static int TestWriterAuthorityComesFromApi(void)
{
    MotorCmd cmd = TestCurrentCmd(1234);
    MotorCmd stored;
    const uint16_t invalid_writers[] = {1u, 9u, 11u, 239u, 241u, 254u, 256u, 65535u};
    LowCmd before;
    LowCmd after;

    LowCmdClearAll();
    s_test_tick_ms = 230u;
    cmd.writer = (uint16_t)LOWCMD_WRITER_FAULT;
    if (!TestCheck(LowCmdSetMotorFrom(Motor0, &cmd, LOWCMD_WRITER_CONTROL) != 0u,
                   "MotorCmd 载荷中的 writer 不应让合法 CONTROL 写入失败") ||
        !TestRead(Motor0, &stored) ||
        !TestCheck(stored.writer == (uint16_t)LOWCMD_WRITER_CONTROL,
                   "MotorCmd 载荷不得把 CONTROL 写入提权为 FAULT"))
    {
        return 0;
    }

    cmd.writer = (uint16_t)LOWCMD_WRITER_NONE;
    if (!TestCheck(LowCmdSetMotorFrom(Motor1, &cmd, LOWCMD_WRITER_NONE) != 0u,
                   "writer NONE 应按兼容约定映射为 CONTROL") ||
        !TestRead(Motor1, &stored) ||
        !TestCheck(stored.writer == (uint16_t)LOWCMD_WRITER_CONTROL,
                   "writer NONE 映射后的归属应为 CONTROL"))
    {
        return 0;
    }

    if (!TestCheck(LowCmdGet(&before) != 0u, "读取非法 writer 测试前状态失败"))
    {
        return 0;
    }
    for (uint8_t i = 0u; i < (uint8_t)(sizeof(invalid_writers) / sizeof(invalid_writers[0])); i++)
    {
        if (!TestCheck(LowCmdSetMotorFrom(Motor2, &cmd, invalid_writers[i]) == 0u,
                       "LowCmdSetMotorFrom 接受了白名单空洞 writer"))
        {
            return 0;
        }
    }
    if (!TestCheck(LowCmdSetCurrentManyFrom((const MotorId[]){Motor2},
                                            (const int16_t[]){12},
                                            1u,
                                            65535u) == 0u &&
                       LowCmdClearManyFrom((const MotorId[]){Motor0}, 1u, 65535u) == 0u &&
                       LowCmdInhibitManyFrom((const MotorId[]){Motor0}, 1u, 65535u) == 0u &&
                       LowCmdReleaseInhibitManyFrom((const MotorId[]){Motor0}, 1u, 65535u) == 0u &&
                       LowCmdEnterEmergencyStop(65535u) == 0u &&
                       LowCmdClearEmergencyStop(65535u) == 0u,
                   "未声明的 writer 值必须被所有权限入口拒绝"))
    {
        return 0;
    }
    LowCmdSetDisableFrom(Motor2, 65535u);
    if (!TestCheck(LowCmdGet(&after) != 0u, "读取非法 writer 测试后状态失败"))
    {
        return 0;
    }
    return TestCheck(before.seq == after.seq &&
                         memcmp(before.motorCmd, after.motorCmd, sizeof(before.motorCmd)) == 0,
                     "拒绝非法 writer 时不得改动命令或序号");
}

static int TestLowStateCopiesLastEcd(void)
{
    const MotorId ids[2] = {Motor1, Motor6};
    MotorState feedback[2] = {{0}, {0}};
    MotorState single;
    MotorState many[2];
    LowState state;

    feedback[0].lastRxTick = 101u;
    feedback[0].lastEcd = 0x1234u;
    feedback[0].ecd = 0x2345u;
    feedback[1].lastRxTick = 102u;
    feedback[1].lastEcd = 0x5678u;
    feedback[1].ecd = 0x6789u;

    LowStateClearAll();
    LowStateUpdateMotor(ids[0], &feedback[0]);
    LowStateUpdateMotor(ids[1], &feedback[1]);

    if (!TestCheck(LowStateGetMotor(ids[0], &single) != 0u &&
                       single.lastEcd == feedback[0].lastEcd &&
                       single.ecd == feedback[0].ecd,
                   "LowState 单轴快照丢失上一编码器值"))
    {
        return 0;
    }
    if (!TestCheck(LowStateGetMotorMany(ids, many, 2u) != 0u &&
                       many[0].lastEcd == feedback[0].lastEcd &&
                       many[1].lastEcd == feedback[1].lastEcd,
                   "LowState 批量快照丢失上一编码器值"))
    {
        return 0;
    }
    return TestCheck(LowStateGet(&state) != 0u &&
                         state.motorState[ids[0]].lastEcd == feedback[0].lastEcd &&
                         state.motorState[ids[1]].lastEcd == feedback[1].lastEcd,
                     "LowState 整体快照丢失上一编码器值");
}

int main(void)
{
    if (!TestClearManySuccess()) return 1;
    if (!TestInvalidBatchIsNoOp()) return 1;
    if (!TestPriorityRejectIsAtomic()) return 1;
    if (!TestSafetyMayClearControl()) return 1;
    if (!TestEmergencyLockRejectsSafetyClear()) return 1;
    if (!TestZeroCountIsNoOp()) return 1;
    if (!TestInhibitClearsAndBlocksLowerWriters()) return 1;
    if (!TestInhibitAcquireRejectIsAtomic()) return 1;
    if (!TestReleaseRejectIsAtomic()) return 1;
    if (!TestEmergencyAndLocalInhibitStack()) return 1;
    if (!TestBestEffortSkipsInhibitedAxis()) return 1;
    if (!TestCanTxRejectsStaleCachedCommand()) return 1;
    if (!TestCanTxCommandExpiryBoundaries()) return 1;
    if (!TestCanTxUnlockGenerationBarrier()) return 1;
    if (!TestWriterAuthorityComesFromApi()) return 1;
    if (!TestLowStateCopiesLastEcd()) return 1;

    (void)puts("PASS: LowCmd clear and inhibit host regression");
    return 0;
}
