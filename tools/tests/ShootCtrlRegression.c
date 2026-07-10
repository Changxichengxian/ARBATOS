/* Shoot 控制域生命周期回归：真实 ControlMgr/ShootCtrl，强桩替代执行体。 */

#include <stdint.h>
#include <stdio.h>

#include "ControlMgr.h"
#include "ShootCtrl.h"

struct ManualInputSnapshot
{
    uint32_t marker;
};

static uint32_t s_runtimeInitCount;
static uint32_t s_runtimeStepCount;
static uint32_t s_runtimeSafeStepCount;
static uint32_t s_runtimeStopCount;
static const struct ManualInputSnapshot *s_runtimeStepInputs[16];
static const struct ManualInputSnapshot *s_runtimeSafeStepInputs[8];
static char s_runtimeEvents[16];
static uint8_t s_runtimeEventCount;

static const ControlController s_systemBlocker = {
    .id = ControlIdCustomBase,
    .domain = ControlDomainSystem,
    .claim_mask = ControlResArm,
    .name = "controller.test_blocker",
};

static const ControlController s_blockedShoot = {
    .id = ControlIdCustomBase + 1u,
    .domain = ControlDomainShoot,
    .claim_mask = ControlResShootTrigger | ControlResShootFriction | ControlResArm,
    .name = "controller.test_blocked_shoot",
};

static void RuntimeRecord(char event)
{
    if (s_runtimeEventCount < (uint8_t)sizeof(s_runtimeEvents))
    {
        s_runtimeEvents[s_runtimeEventCount++] = event;
    }
}

void ShootRuntimeInit(void)
{
    s_runtimeInitCount++;
    RuntimeRecord('I');
}

int16_t ShootRuntimeStep(const struct ManualInputSnapshot *manualInput)
{
    if (s_runtimeStepCount < (uint32_t)(sizeof(s_runtimeStepInputs) / sizeof(s_runtimeStepInputs[0])))
    {
        s_runtimeStepInputs[s_runtimeStepCount] = manualInput;
    }
    s_runtimeStepCount++;
    RuntimeRecord('U');
    return (int16_t)(300 + s_runtimeStepCount);
}

void ShootRuntimeSafeStep(const struct ManualInputSnapshot *manualInput)
{
    if (s_runtimeSafeStepCount <
        (uint32_t)(sizeof(s_runtimeSafeStepInputs) / sizeof(s_runtimeSafeStepInputs[0])))
    {
        s_runtimeSafeStepInputs[s_runtimeSafeStepCount] = manualInput;
    }
    s_runtimeSafeStepCount++;
    RuntimeRecord('F');
}

void ShootRuntimeStop(void)
{
    s_runtimeStopCount++;
    RuntimeRecord('S');
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

static int TestStatus(ControlStatus *status)
{
    return TestCheck(ControlMgrGetStatus(ControlDomainShoot, status) == ControlResultOk,
                     "读取 Shoot 控制域状态失败");
}

int main(void)
{
    const ControlController *descriptor;
    const struct ManualInputSnapshot firstInput = {1u};
    const struct ManualInputSnapshot nextInput = {2u};
    const struct ManualInputSnapshot recoveryInput = {3u};
    const struct ManualInputSnapshot safeInput = {4u};
    const struct ManualInputSnapshot resumeInput = {5u};
    const struct ManualInputSnapshot restartInput = {6u};
    ShootCtrlInput input = {
        .tickMs = 100u,
        .periodMs = 1u,
        .forceSafe = 0u,
        .manualInput = &firstInput,
    };
    ShootCtrlOutput output = {1234};
    ControlCtx managerContext = {0};
    ControlStatus status;
    uint32_t updatesBeforeStop;
    uint32_t stepsBeforeRestart;
    uint32_t stepsBeforeFault;
    uint32_t stopsBeforeFault;
    uint8_t eventsBeforeRestart;

    ControlMgrReset();
    descriptor = ShootCtrlDesc();
    if (!TestCheck(descriptor != NULL, "Shoot 控制器描述不能为空")) return 1;
    if (!TestCheck(ControlMgrRegister(descriptor) == ControlResultOk,
                   "测试预注册 Shoot 控制器失败")) return 1;
    if (!TestCheck(ControlMgrSwitch(ControlIdShoot, ControlReasonProfile) == ControlResultOk,
                   "请求启用 Shoot 控制器失败")) return 1;
    if (!TestStatus(&status)) return 1;
    if (!TestCheck(status.pending_request == ControlRequestSwitch &&
                   status.active == 0u &&
                   s_runtimeInitCount == 0u &&
                   s_runtimeStepCount == 0u &&
                   s_runtimeSafeStepCount == 0u &&
                   s_runtimeStopCount == 0u,
                   "Switch 只能留下 pending，不能提前启动执行体")) return 1;

    if (!TestCheck(ShootCtrlStep(NULL, &output) == ControlResultBadArgument &&
                   output.triggerCurrent == 0,
                   "空输入应清零输出并返回 BadArgument")) return 1;
    if (!TestCheck(ShootCtrlStep(&input, NULL) == ControlResultBadArgument,
                   "空输出应返回 BadArgument")) return 1;
    output.triggerCurrent = 1234;
    if (!TestCheck(ShootCtrlStep(&input, &output) == ControlResultNotActive &&
                   output.triggerCurrent == 0,
                   "Prepare 前应清零输出并返回 NotActive")) return 1;
    if (!TestStatus(&status)) return 1;
    if (!TestCheck(status.pending_request == ControlRequestSwitch &&
                   status.active == 0u &&
                   status.update_count == 0u &&
                   s_runtimeInitCount == 0u &&
                   s_runtimeStepCount == 0u &&
                   s_runtimeSafeStepCount == 0u &&
                   s_runtimeStopCount == 0u,
                   "坏参数和未 Prepare 调用不得消费 pending")) return 1;

    ShootCtrlPrepare();
    if (!TestCheck(s_runtimeInitCount == 0u &&
                   s_runtimeStepCount == 0u &&
                   s_runtimeSafeStepCount == 0u &&
                   s_runtimeStopCount == 0u,
                   "Prepare 只能准备门面，不能提前运行执行体")) return 1;
    if (!TestCheck(ShootCtrlStep(&input, &output) == ControlResultOk &&
                   s_runtimeInitCount == 1u &&
                   s_runtimeStepCount == 1u &&
                   s_runtimeSafeStepCount == 0u &&
                   s_runtimeStopCount == 0u &&
                   s_runtimeStepInputs[0] == &firstInput &&
                   s_runtimeEventCount == 2u &&
                   s_runtimeEvents[0] == 'I' &&
                   s_runtimeEvents[1] == 'U' &&
                   output.triggerCurrent == 301,
                   "首次有效 Step 应依次 Init、Step，并返回执行体输出")) return 1;
    if (!TestStatus(&status)) return 1;
    if (!TestCheck(status.pending_request == ControlRequestNone &&
                   status.active != 0u &&
                   status.active_id == ControlIdShoot &&
                   status.update_count == 1u,
                   "首次有效 Step 应消费 pending 且只更新一次")) return 1;

    input.tickMs++;
    input.manualInput = &nextInput;
    if (!TestCheck(ShootCtrlStep(&input, &output) == ControlResultOk &&
                   s_runtimeInitCount == 1u &&
                   s_runtimeStepCount == 2u &&
                   s_runtimeStepInputs[1] == &nextInput &&
                   output.triggerCurrent == 302,
                   "连续 Step 不得重复 Init 或单周期重复更新")) return 1;
    if (!TestStatus(&status)) return 1;
    if (!TestCheck(status.update_count == 2u,
                    "第二周期 ControlMgr 更新计数应只增加一次")) return 1;

    input.tickMs++;
    input.manualInput = NULL;
    if (!TestCheck(ShootCtrlStep(&input, &output) == ControlResultOk &&
                   s_runtimeStepCount == 3u &&
                   s_runtimeStepInputs[2] == NULL &&
                   s_runtimeStopCount == 0u &&
                   output.triggerCurrent == 303,
                   "普通帧必须把空快照原样传给 Runtime，且不能退出控制域")) return 1;
    if (!TestStatus(&status)) return 1;
    if (!TestCheck(status.active != 0u &&
                   status.active_id == ControlIdShoot &&
                   status.update_count == 3u,
                   "空快照普通帧仍应保持 Shoot 控制域活动")) return 1;

    if (!TestCheck(ControlMgrRegister(&s_systemBlocker) == ControlResultOk &&
                   ControlMgrRegister(&s_blockedShoot) == ControlResultOk &&
                   ControlMgrSwitch(ControlIdCustomBase, ControlReasonTest) == ControlResultOk &&
                   ControlMgrUpdateDomain(ControlDomainSystem, &managerContext) == ControlResultOk,
                   "准备资源冲突场景失败")) return 1;
    if (!TestCheck(ControlMgrSwitch(ControlIdCustomBase + 1u, ControlReasonModeSwitch) == ControlResultOk,
                   "请求资源冲突的 Shoot 控制器失败")) return 1;
    input.tickMs++;
    output.triggerCurrent = 1234;
    if (!TestCheck(ShootCtrlStep(&input, &output) == ControlResultResourceBusy &&
                   output.triggerCurrent == 0 &&
                   s_runtimeStepCount == 3u &&
                   s_runtimeStopCount == 1u,
                   "管理层拒绝更新时必须立即清除整个 Runtime 输出")) return 1;
    if (!TestStatus(&status)) return 1;
    if (!TestCheck(status.active_id == ControlIdShoot &&
                   status.update_count == 3u &&
                   status.last_result == ControlResultResourceBusy,
                   "资源冲突不得替换当前 Shoot 控制器或伪造更新计数")) return 1;

    input.tickMs++;
    input.manualInput = &recoveryInput;
    if (!TestCheck(ShootCtrlStep(&input, &output) == ControlResultOk &&
                   s_runtimeStepCount == 4u &&
                   s_runtimeStepInputs[3] == &recoveryInput &&
                   s_runtimeStopCount == 1u &&
                   output.triggerCurrent == 304,
                   "管理层异常后的下一正常帧应恢复且只更新一次")) return 1;

    input.tickMs++;
    input.forceSafe = 1u;
    input.manualInput = &safeInput;
    output.triggerCurrent = 1234;
    if (!TestCheck(ShootCtrlStep(&input, &output) == ControlResultOk &&
                   output.triggerCurrent == 0 &&
                   s_runtimeStepCount == 4u &&
                   s_runtimeSafeStepCount == 1u &&
                   s_runtimeSafeStepInputs[0] == &safeInput &&
                   s_runtimeStopCount == 1u,
                   "forceSafe 应把同一快照传给安全帧并保持零输出")) return 1;
    if (!TestStatus(&status)) return 1;
    if (!TestCheck(status.active != 0u &&
                   status.active_id == ControlIdShoot &&
                   status.update_count == 5u,
                   "forceSafe 不能停用 Shoot 控制域")) return 1;

    input.tickMs++;
    input.forceSafe = 0u;
    input.manualInput = &resumeInput;
    if (!TestCheck(ShootCtrlStep(&input, &output) == ControlResultOk &&
                   s_runtimeInitCount == 1u &&
                   s_runtimeStepCount == 5u &&
                   s_runtimeStepInputs[4] == &resumeInput &&
                   s_runtimeSafeStepCount == 1u &&
                   s_runtimeStopCount == 1u &&
                   output.triggerCurrent == 305,
                   "forceSafe 恢复后应直接 Step，不能重复 Init")) return 1;
    if (!TestStatus(&status)) return 1;
    if (!TestCheck(status.active != 0u &&
                   status.active_id == ControlIdShoot &&
                   status.update_count == 6u,
                   "安全帧恢复后 Shoot 控制域必须保持活动")) return 1;

    if (!TestCheck(ControlMgrStop(ControlDomainShoot, ControlReasonDisable) == ControlResultOk,
                   "请求停用 Shoot 控制域失败")) return 1;
    if (!TestStatus(&status)) return 1;
    if (!TestCheck(status.pending_request == ControlRequestStop &&
                   status.active != 0u &&
                   s_runtimeStopCount == 1u,
                   "Stop 请求在下一次 Step 前只能保持 pending")) return 1;
    updatesBeforeStop = status.update_count;
    input.tickMs++;
    output.triggerCurrent = 1234;
    if (!TestCheck(ShootCtrlStep(&input, &output) == ControlResultNotActive &&
                   output.triggerCurrent == 0 &&
                   s_runtimeStopCount == 2u,
                   "显式 Stop 应执行停止回调、清零输出并返回 NotActive")) return 1;
    if (!TestStatus(&status)) return 1;
    if (!TestCheck(status.active == 0u &&
                   status.state == ControlStateStopped &&
                   status.pending_request == ControlRequestNone &&
                   status.update_count == updatesBeforeStop,
                   "显式 Stop 后必须停用控制域且不能再执行 update")) return 1;

    stepsBeforeRestart = s_runtimeStepCount;
    eventsBeforeRestart = s_runtimeEventCount;
    if (!TestCheck(ControlMgrSwitch(ControlIdShoot, ControlReasonModeSwitch) == ControlResultOk,
                   "重新请求 Shoot 控制器失败")) return 1;
    input.tickMs++;
    input.manualInput = &restartInput;
    if (!TestCheck(ShootCtrlStep(&input, &output) == ControlResultOk &&
                   s_runtimeInitCount == 1u &&
                   s_runtimeStepCount == stepsBeforeRestart + 1u &&
                   s_runtimeStepInputs[5] == &restartInput &&
                   s_runtimeStopCount == 3u &&
                   s_runtimeEventCount == (uint8_t)(eventsBeforeRestart + 2u) &&
                   s_runtimeEvents[eventsBeforeRestart] == 'S' &&
                   s_runtimeEvents[eventsBeforeRestart + 1u] == 'U' &&
                   output.triggerCurrent == (int16_t)(300u + s_runtimeStepCount),
                   "重新启用应先清零，再保持单次 Init 且只 Step 一次")) return 1;
    if (!TestStatus(&status)) return 1;
    if (!TestCheck(status.active != 0u &&
                   status.active_id == ControlIdShoot &&
                   status.update_count == updatesBeforeStop + 1u,
                   "重新启用后应恢复活动状态且只增加一次更新计数")) return 1;

    stepsBeforeFault = s_runtimeStepCount;
    stopsBeforeFault = s_runtimeStopCount;
    if (!TestCheck(ControlMgrUpdateDomain(ControlDomainShoot, NULL) == ControlResultCallbackFailed &&
                   s_runtimeStepCount == stepsBeforeFault &&
                   s_runtimeStopCount == stopsBeforeFault + 1u,
                   "绕过门面的空上下文必须触发停止回调并进入故障")) return 1;
    if (!TestStatus(&status)) return 1;
    if (!TestCheck(status.active == 0u &&
                   status.state == ControlStateFault &&
                   status.last_result == ControlResultBadArgument,
                   "空上下文回调失败后 Shoot 控制域必须处于故障停用状态")) return 1;

    (void)puts("PASS: Shoot controller lifecycle regression");
    return 0;
}
