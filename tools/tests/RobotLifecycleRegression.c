/* RobotLifecycle 主机回归：真实状态机，计数桩提供统一手动输入快照。 */

#include <stdint.h>
#include <stdio.h>

#include "ControlInput.h"
#include "ManualInputSnapshot.h"
#include "RobotConfig.h"
#include "RobotLifecycle.h"

#define TEST_SWITCH_SAFE 1u
#define TEST_SWITCH_RUN  2u
#define TEST_SEMANTICS_OLD_SEQ 1u
#define TEST_SEMANTICS_NEW_SEQ 2u

RobotLifecycleTestConfig g_config = {
    .manual_input = {
        .semantics = {
            .GimbalSafePos = TEST_SWITCH_SAFE,
        },
    },
};

static ManualInputSnapshot s_manualInput;
static uint32_t s_manualReadCount;
static uint32_t s_tickMs;
static uint8_t s_manualReadOk;

uint32_t __get_IPSR(void)
{
    return 0u;
}

uint32_t HAL_GetTick(void)
{
    return s_tickMs;
}

uint8_t ControlInputSwitchIsPos(uint16_t raw, uint8_t pos)
{
    return (uint8_t)(raw == (uint16_t)pos);
}

uint8_t ManualInputSnapshotRead(ManualInputSnapshot *out)
{
    s_manualReadCount++;
    if (out == NULL || s_manualReadOk == 0u)
    {
        return 0u;
    }

    *out = s_manualInput;
    return 1u;
}

static int TestCheck(int condition, const char *message)
{
    if (condition != 0)
    {
        return 1;
    }

    (void)fprintf(stderr, "FAIL: %s\n", message);
    return 0;
}

static void TestSetInput(uint8_t readOk,
                         uint8_t online,
                         uint16_t modeSwitch,
                         uint8_t snapshotSafePos,
                         uint32_t semanticsSeq)
{
    s_manualReadOk = readOk;
    s_manualInput.online = online;
    s_manualInput.control.sw[INPUT_SW_GIMBAL_MODE] = modeSwitch;
    s_manualInput.semantics.GimbalSafePos = snapshotSafePos;
    s_manualInput.semanticsSeq = semanticsSeq;
}

static int TestUpdateReadsOnce(const char *message)
{
    const uint32_t readsBefore = s_manualReadCount;

    s_tickMs++;
    RobotLifecycleUpdate();
    return TestCheck(s_manualReadCount == readsBefore + 1u, message);
}

static int TestCachedState(RobotLifecycleState expectedState,
                           RobotLifecycleReason expectedReason,
                           uint8_t expectedOutputAllowed,
                           uint8_t expectedFaultLatched,
                           uint8_t expectedStartupSafeSeen,
                           const char *message)
{
    RobotLifecycleSnapshot snapshot;
    const uint32_t readsBefore = s_manualReadCount;
    const RobotLifecycleState state = RobotLifecycleCurrent();
    const uint8_t outputAllowed = RobotLifecycleOutputAllowed();
    const uint8_t snapshotOk = RobotLifecycleGetSnapshot(&snapshot);
    const uint8_t faultLatched = RobotLifecycleFaultLatched();

    return TestCheck(snapshotOk != 0u &&
                         state == expectedState &&
                         outputAllowed == expectedOutputAllowed &&
                         faultLatched == expectedFaultLatched &&
                         snapshot.state == expectedState &&
                         snapshot.reason == expectedReason &&
                         snapshot.output_allowed == expectedOutputAllowed &&
                         snapshot.fault_latched == expectedFaultLatched &&
                         snapshot.startup_safe_seen == expectedStartupSafeSeen &&
                         s_manualReadCount == readsBefore,
                     message);
}

static int TestCachedSemanticsSeq(uint32_t expectedSeq, const char *message)
{
    RobotLifecycleSnapshot snapshot;
    const uint32_t readsBefore = s_manualReadCount;

    return TestCheck(RobotLifecycleGetSnapshot(&snapshot) != 0u &&
                         snapshot.manual_semantics_seq == expectedSeq &&
                         s_manualReadCount == readsBefore,
                     message);
}

int main(void)
{
    uint32_t readsBefore;

    RobotLifecycleInit();
    if (!TestCheck(s_manualReadCount == 0u,
                   "Init 不得读取手动输入快照")) return 1;
    if (!TestCachedState(ROBOT_LIFECYCLE_BOOT,
                         ROBOT_LIFECYCLE_REASON_BOOT,
                         0u,
                         0u,
                         0u,
                         "Init 后必须保持 BOOT 锁定，所有查询必须为纯缓存读取")) return 1;

    TestSetInput(1u, 1u, TEST_SWITCH_RUN,
                 TEST_SWITCH_SAFE, TEST_SEMANTICS_OLD_SEQ);
    if (!TestUpdateReadsOnce("启动运行档 Update 必须且只能读取一次快照")) return 1;
    if (!TestCachedState(ROBOT_LIFECYCLE_SAFE,
                         ROBOT_LIFECYCLE_REASON_STARTUP_SAFE_REQUIRED,
                         0u,
                         0u,
                         0u,
                         "启动后未见安全档时不得放行")) return 1;
    if (!TestCachedSemanticsSeq(TEST_SEMANTICS_OLD_SEQ,
                                "首帧必须缓存非零输入语义代")) return 1;

    TestSetInput(1u, 1u, TEST_SWITCH_SAFE,
                 TEST_SWITCH_SAFE, TEST_SEMANTICS_OLD_SEQ);
    if (!TestUpdateReadsOnce("安全档 Update 必须且只能读取一次快照")) return 1;
    if (!TestCachedState(ROBOT_LIFECYCLE_SAFE,
                         ROBOT_LIFECYCLE_REASON_MANUAL_SAFE_SWITCH,
                         0u,
                         0u,
                         1u,
                         "安全档只能解除启动联锁，不能直接放行")) return 1;

    TestSetInput(1u, 1u, TEST_SWITCH_RUN,
                 TEST_SWITCH_SAFE, TEST_SEMANTICS_OLD_SEQ);
    if (!TestUpdateReadsOnce("安全档后的运行 Update 必须且只能读取一次快照")) return 1;
    if (!TestCachedState(ROBOT_LIFECYCLE_ACTIVE,
                         ROBOT_LIFECYCLE_REASON_NONE,
                         1u,
                         0u,
                         1u,
                         "安全档后切到运行档才允许输出")) return 1;

    TestSetInput(1u, 0u, TEST_SWITCH_RUN,
                 TEST_SWITCH_SAFE, TEST_SEMANTICS_OLD_SEQ);
    if (!TestUpdateReadsOnce("离线 Update 必须且只能读取一次快照")) return 1;
    if (!TestCachedState(ROBOT_LIFECYCLE_SAFE,
                         ROBOT_LIFECYCLE_REASON_MANUAL_OFFLINE,
                         0u,
                         0u,
                         1u,
                         "手动输入离线必须重新锁定输出")) return 1;

    TestSetInput(0u, 1u, TEST_SWITCH_RUN,
                 TEST_SWITCH_SAFE, TEST_SEMANTICS_OLD_SEQ);
    if (!TestUpdateReadsOnce("快照读取失败的 Update 也必须恰好尝试一次")) return 1;
    if (!TestCachedState(ROBOT_LIFECYCLE_SAFE,
                         ROBOT_LIFECYCLE_REASON_MANUAL_OFFLINE,
                         0u,
                         0u,
                         1u,
                         "快照读取失败必须按离线锁定")) return 1;

    TestSetInput(1u, 1u, TEST_SWITCH_RUN,
                 TEST_SWITCH_SAFE, TEST_SEMANTICS_OLD_SEQ);
    if (!TestUpdateReadsOnce("恢复运行 Update 必须且只能读取一次快照")) return 1;
    if (!TestCachedState(ROBOT_LIFECYCLE_ACTIVE,
                         ROBOT_LIFECYCLE_REASON_NONE,
                         1u,
                         0u,
                         1u,
                         "已见过安全档后在线运行可恢复输出")) return 1;

    readsBefore = s_manualReadCount;
    s_tickMs++;
    RobotLifecycleEnterFault(ROBOT_LIFECYCLE_REASON_NONE);
    if (!TestCheck(s_manualReadCount == readsBefore,
                   "EnterFault 不得读取手动输入快照")) return 1;
    if (!TestCachedState(ROBOT_LIFECYCLE_FAULT,
                         ROBOT_LIFECYCLE_REASON_FATAL_FAULT,
                         0u,
                         1u,
                         1u,
                         "EnterFault 必须立即锁存故障并清除输出许可")) return 1;

    TestSetInput(1u, 1u, TEST_SWITCH_RUN,
                 TEST_SWITCH_SAFE, TEST_SEMANTICS_OLD_SEQ);
    if (!TestUpdateReadsOnce("故障锁存后的运行 Update 必须且只能读取一次快照")) return 1;
    if (!TestCachedState(ROBOT_LIFECYCLE_FAULT,
                         ROBOT_LIFECYCLE_REASON_FATAL_FAULT,
                         0u,
                         1u,
                         1u,
                         "运行输入不得越过锁存故障")) return 1;

    TestSetInput(1u, 1u, TEST_SWITCH_SAFE,
                 TEST_SWITCH_SAFE, TEST_SEMANTICS_OLD_SEQ);
    if (!TestUpdateReadsOnce("故障锁存后的安全 Update 必须且只能读取一次快照")) return 1;
    if (!TestCachedState(ROBOT_LIFECYCLE_FAULT,
                         ROBOT_LIFECYCLE_REASON_FATAL_FAULT,
                         0u,
                         1u,
                         1u,
                         "安全输入也不得自行清除锁存故障")) return 1;

    readsBefore = s_manualReadCount;
    RobotLifecycleClearFault();
    if (!TestCheck(s_manualReadCount == readsBefore,
                   "ClearFault 不得读取手动输入快照")) return 1;
    if (!TestCachedState(ROBOT_LIFECYCLE_FAULT,
                         ROBOT_LIFECYCLE_REASON_FATAL_FAULT,
                         0u,
                         0u,
                         1u,
                         "ClearFault 只能解除锁存，不能立即放行或重算状态")) return 1;

    TestSetInput(1u, 1u, TEST_SWITCH_SAFE,
                 TEST_SWITCH_SAFE, TEST_SEMANTICS_OLD_SEQ);
    if (!TestUpdateReadsOnce("清故障后的安全 Update 必须且只能读取一次快照")) return 1;
    if (!TestCachedState(ROBOT_LIFECYCLE_SAFE,
                         ROBOT_LIFECYCLE_REASON_MANUAL_SAFE_SWITCH,
                         0u,
                         0u,
                         1u,
                         "清故障后必须先保持安全档锁定")) return 1;

    TestSetInput(1u, 1u, TEST_SWITCH_RUN,
                 TEST_SWITCH_SAFE, TEST_SEMANTICS_OLD_SEQ);
    if (!TestUpdateReadsOnce("清故障后的运行 Update 必须且只能读取一次快照")) return 1;
    if (!TestCachedState(ROBOT_LIFECYCLE_ACTIVE,
                         ROBOT_LIFECYCLE_REASON_NONE,
                         1u,
                         0u,
                         1u,
                         "清故障后的安全档到运行档序列应恢复输出")) return 1;

    readsBefore = s_manualReadCount;
    s_tickMs += 21u;
    if (!TestCachedState(ROBOT_LIFECYCLE_SAFE,
                         ROBOT_LIFECYCLE_REASON_UPDATE_STALE,
                         0u,
                         0u,
                         1u,
                         "唯一推进者停滞后，所有缓存读取者必须自动得到锁定结论") ||
        !TestCheck(s_manualReadCount == readsBefore,
                   "生命周期过期裁决不得偷偷读取手动输入")) return 1;
    if (!TestUpdateReadsOnce("推进者恢复后仍必须且只能读取一次快照")) return 1;
    if (!TestCachedState(ROBOT_LIFECYCLE_ACTIVE,
                         ROBOT_LIFECYCLE_REASON_NONE,
                         1u,
                         0u,
                         1u,
                         "推进者恢复并确认在线运行后才可重新放行")) return 1;

    /*
     * 模拟 AuxParam 已提交实时配置、ManualInputRefresh 尚未运行的抢占窗口。
     * 生命周期必须继续按旧快照解释当前档位，不能拿新配置重释旧输入。
     */
    g_config.input.gimbalModeInvert = 1u;
    TestSetInput(1u, 1u, TEST_SWITCH_RUN,
                 TEST_SWITCH_SAFE, TEST_SEMANTICS_OLD_SEQ);
    if (!TestUpdateReadsOnce("实时配置先变化时仍必须只读取一次旧代快照")) return 1;
    if (!TestCachedState(ROBOT_LIFECYCLE_ACTIVE,
                         ROBOT_LIFECYCLE_REASON_NONE,
                         1u,
                         0u,
                         1u,
                         "刷新前必须按旧快照语义保持原运行结论")) return 1;
    if (!TestCachedSemanticsSeq(TEST_SEMANTICS_OLD_SEQ,
                                "实时配置变化不得偷偷推进生命周期语义代")) return 1;

    /* 新映射发布时，即使业务安全位置不变，旧解锁资格也必须清零。 */
    TestSetInput(1u, 1u, TEST_SWITCH_RUN,
                 TEST_SWITCH_SAFE, TEST_SEMANTICS_NEW_SEQ);
    if (!TestUpdateReadsOnce("新映射非安全档 Update 必须且只能读取一次快照")) return 1;
    if (!TestCachedState(ROBOT_LIFECYCLE_SAFE,
                         ROBOT_LIFECYCLE_REASON_STARTUP_SAFE_REQUIRED,
                         0u,
                         0u,
                         0u,
                         "新输入解释代到达时必须撤销旧代解锁资格")) return 1;
    if (!TestCachedSemanticsSeq(TEST_SEMANTICS_NEW_SEQ,
                                "生命周期必须记住已裁决的新语义代")) return 1;

    TestSetInput(1u, 1u, TEST_SWITCH_SAFE,
                 TEST_SWITCH_SAFE, TEST_SEMANTICS_NEW_SEQ);
    if (!TestUpdateReadsOnce("新映射安全档 Update 必须且只能读取一次快照")) return 1;
    if (!TestCachedState(ROBOT_LIFECYCLE_SAFE,
                         ROBOT_LIFECYCLE_REASON_MANUAL_SAFE_SWITCH,
                         0u,
                         0u,
                         1u,
                         "新映射下必须重新真实见到安全档")) return 1;

    TestSetInput(1u, 1u, TEST_SWITCH_RUN,
                 TEST_SWITCH_SAFE, TEST_SEMANTICS_NEW_SEQ);
    if (!TestUpdateReadsOnce("新映射运行档 Update 必须且只能读取一次快照")) return 1;
    if (!TestCachedState(ROBOT_LIFECYCLE_ACTIVE,
                         ROBOT_LIFECYCLE_REASON_NONE,
                         1u,
                         0u,
                         1u,
                         "新映射下重新经过安全档后才能恢复输出")) return 1;

    (void)puts("PASS: RobotLifecycle 统一快照与锁定状态回归");
    return 0;
}
