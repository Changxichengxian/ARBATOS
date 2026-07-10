/* 底盘控制域生命周期回归：真实 ControlMgr/ChassisCtrl，强桩替代执行体。 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

struct ManualInputSnapshot
{
    uint32_t token;
};

#include "ChassisCtrl.h"
#include "ControlMgr.h"

static uint32_t s_runtimeInitCount;
static uint32_t s_runtimeStepCount;
static uint32_t s_runtimeSafeStepCount;
static uint32_t s_runtimeStopCount;
static uint32_t s_lastTickMs;
static uint16_t s_lastPeriodMs;
static const struct ManualInputSnapshot *s_lastManualInput;
static char s_runtimeEvents[24];
static uint8_t s_runtimeEventCount;

static const ControlController s_systemBlocker = {
    .id = ControlIdCustomBase,
    .domain = ControlDomainSystem,
    .claim_mask = ControlResArm,
    .name = "controller.test_blocker",
};

static const ControlController s_blockedChassis = {
    .id = ControlIdCustomBase + 1u,
    .domain = ControlDomainChassis,
    .claim_mask = ControlResChassisWheels | ControlResArm,
    .name = "controller.test_blocked_chassis",
};

static void RuntimeRecord(char event)
{
    if (s_runtimeEventCount < (uint8_t)sizeof(s_runtimeEvents))
    {
        s_runtimeEvents[s_runtimeEventCount++] = event;
    }
}

void ChassisRuntimeInit(void)
{
    s_runtimeInitCount++;
    RuntimeRecord('I');
}

void ChassisRuntimeStep(const struct ManualInputSnapshot *manualInput,
                        uint32_t tickMs,
                        uint16_t periodMs,
                        int16_t motorCurrent[4])
{
    s_runtimeStepCount++;
    s_lastTickMs = tickMs;
    s_lastPeriodMs = periodMs;
    s_lastManualInput = manualInput;
    RuntimeRecord('U');
    for (uint8_t i = 0u; i < 4u; i++)
    {
        motorCurrent[i] = (int16_t)(100u * s_runtimeStepCount + i);
    }
}

void ChassisRuntimeSafeStep(const struct ManualInputSnapshot *manualInput,
                            uint32_t tickMs,
                            uint16_t periodMs)
{
    s_runtimeSafeStepCount++;
    s_lastTickMs = tickMs;
    s_lastPeriodMs = periodMs;
    s_lastManualInput = manualInput;
    RuntimeRecord('F');
}

void ChassisRuntimeStop(void)
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

static int TestOutputZero(const ChassisCtrlOutput *output)
{
    for (uint8_t i = 0u; i < 4u; i++)
    {
        if (output->motorCurrent[i] != 0)
        {
            return 0;
        }
    }
    return 1;
}

static int TestStatus(ControlStatus *status)
{
    return TestCheck(ControlMgrGetStatus(ControlDomainChassis, status) == ControlResultOk,
                     "读取底盘控制域状态失败");
}

int main(void)
{
    const ControlController *descriptor;
    struct ManualInputSnapshot manualInputNormal = {0x12345678u};
    struct ManualInputSnapshot manualInputSafe = {0x87654321u};
    struct ManualInputSnapshot manualInputResume = {0x55AA55AAu};
    ChassisCtrlInput input = {
        .manualInput = &manualInputNormal,
        .tickMs = 100u,
        .periodMs = 2u,
        .forceSafe = 0u,
    };
    ChassisCtrlOutput output = {{11, 22, 33, 44}};
    ControlCtx managerContext = {0};
    ControlStatus status;
    uint32_t updatesBeforeStop;
    uint32_t stepsBeforeRestart;
    uint32_t stopsBeforeRestart;
    uint32_t stepsBeforeFault;
    uint32_t stopsBeforeFault;

    ControlMgrReset();
    descriptor = ChassisCtrlDesc();
    if (!TestCheck(descriptor != NULL &&
                   descriptor->id == ControlIdClassicChassis &&
                   descriptor->domain == ControlDomainChassis &&
                   descriptor->claim_mask == ControlResChassisWheels &&
                   descriptor->meta.input_count == 3u &&
                   descriptor->meta.output_count == 4u &&
                   descriptor->enter != NULL &&
                   descriptor->update != NULL &&
                   descriptor->stop != NULL,
                   "底盘控制器描述不完整")) return 1;
    if (!TestCheck(strcmp(ControlOutputName(descriptor, 3u), "motor.chassis3") == 0,
                   "底盘描述的输出名称错误")) return 1;
    if (!TestCheck(ControlMgrRegister(descriptor) == ControlResultOk,
                   "测试预注册底盘控制器失败")) return 1;
    if (!TestCheck(ControlMgrSwitch(ControlIdClassicChassis, ControlReasonProfile) == ControlResultOk,
                   "请求启用底盘控制器失败")) return 1;
    if (!TestStatus(&status)) return 1;
    if (!TestCheck(status.pending_request == ControlRequestSwitch &&
                   status.active == 0u &&
                   s_runtimeInitCount == 0u &&
                   s_runtimeStepCount == 0u &&
                   s_runtimeSafeStepCount == 0u &&
                   s_runtimeStopCount == 0u,
                   "Switch 只能留下 pending，不能提前启动执行体")) return 1;

    if (!TestCheck(ChassisCtrlStep(NULL, &output) == ControlResultBadArgument &&
                   TestOutputZero(&output),
                   "空输入应清零输出并返回 BadArgument")) return 1;
    if (!TestCheck(ChassisCtrlStep(&input, NULL) == ControlResultBadArgument,
                   "空输出应返回 BadArgument")) return 1;
    output.motorCurrent[0] = 1234;
    if (!TestCheck(ChassisCtrlStep(&input, &output) == ControlResultNotActive &&
                   TestOutputZero(&output),
                   "Prepare 前应清零输出并返回 NotActive")) return 1;
    if (!TestStatus(&status)) return 1;
    if (!TestCheck(status.pending_request == ControlRequestSwitch &&
                   status.active == 0u &&
                   status.update_count == 0u &&
                   s_runtimeInitCount == 0u,
                   "坏参数和未 Prepare 调用不得消费 pending")) return 1;

    ChassisCtrlPrepare();
    if (!TestCheck(s_runtimeInitCount == 0u &&
                   s_runtimeStepCount == 0u &&
                   s_runtimeSafeStepCount == 0u &&
                   s_runtimeStopCount == 0u,
                   "Prepare 不能提前运行执行体")) return 1;
    s_lastManualInput = NULL;
    if (!TestCheck(ChassisCtrlStep(&input, &output) == ControlResultOk &&
                   s_runtimeInitCount == 1u &&
                   s_runtimeStepCount == 1u &&
                   s_runtimeStopCount == 0u &&
                   s_runtimeEventCount == 2u &&
                   s_runtimeEvents[0] == 'I' &&
                   s_runtimeEvents[1] == 'U' &&
                   output.motorCurrent[0] == 100 &&
                   output.motorCurrent[3] == 103 &&
                   s_lastManualInput == input.manualInput &&
                   s_lastTickMs == input.tickMs &&
                   s_lastPeriodMs == input.periodMs,
                   "首次有效 Step 应依次 Init、Update，并返回四路输出")) return 1;
    if (!TestStatus(&status)) return 1;
    if (!TestCheck(status.pending_request == ControlRequestNone &&
                   status.active != 0u &&
                   status.active_id == ControlIdClassicChassis &&
                   status.update_count == 1u,
                   "首次有效 Step 应消费 pending 且只更新一次")) return 1;

    input.tickMs += input.periodMs;
    input.forceSafe = 1u;
    input.manualInput = &manualInputSafe;
    s_lastManualInput = &manualInputNormal;
    if (!TestCheck(ChassisCtrlStep(&input, &output) == ControlResultOk &&
                   s_runtimeInitCount == 1u &&
                   s_runtimeStepCount == 1u &&
                   s_runtimeSafeStepCount == 1u &&
                   s_runtimeStopCount == 0u &&
                   s_lastManualInput == input.manualInput &&
                   s_lastTickMs == input.tickMs &&
                   s_lastPeriodMs == input.periodMs &&
                   TestOutputZero(&output),
                   "forceSafe 应执行 SafeStep、清零输出且不退出控制域")) return 1;
    input.tickMs += input.periodMs;
    input.manualInput = NULL;
    s_lastManualInput = &manualInputSafe;
    if (!TestCheck(ChassisCtrlStep(&input, &output) == ControlResultOk &&
                   s_runtimeSafeStepCount == 2u &&
                   s_runtimeStepCount == 1u &&
                   s_lastManualInput == NULL &&
                   s_lastTickMs == input.tickMs &&
                   s_lastPeriodMs == input.periodMs &&
                   TestOutputZero(&output),
                   "连续 forceSafe 帧必须逐帧刷新安全执行体，并原样传递读取失败的 NULL")) return 1;
    if (!TestStatus(&status)) return 1;
    if (!TestCheck(status.active != 0u &&
                   status.active_id == ControlIdClassicChassis &&
                   status.update_count == 3u,
                   "forceSafe 不能停用底盘控制域")) return 1;

    input.tickMs += input.periodMs;
    input.forceSafe = 0u;
    input.manualInput = &manualInputResume;
    s_lastManualInput = &manualInputSafe;
    if (!TestCheck(ChassisCtrlStep(&input, &output) == ControlResultOk &&
                   s_runtimeInitCount == 1u &&
                   s_runtimeStepCount == 2u &&
                   s_runtimeSafeStepCount == 2u &&
                   s_lastManualInput == input.manualInput &&
                   output.motorCurrent[0] == 200,
                   "forceSafe 恢复后应直接 Update，不能重复 Init")) return 1;

    if (!TestCheck(ControlMgrRegister(&s_systemBlocker) == ControlResultOk &&
                   ControlMgrRegister(&s_blockedChassis) == ControlResultOk &&
                   ControlMgrSwitch(ControlIdCustomBase, ControlReasonTest) == ControlResultOk &&
                   ControlMgrUpdateDomain(ControlDomainSystem, &managerContext) == ControlResultOk,
                   "准备资源冲突场景失败")) return 1;
    if (!TestCheck(ControlMgrSwitch(ControlIdCustomBase + 1u, ControlReasonModeSwitch) == ControlResultOk,
                   "请求资源冲突的底盘控制器失败")) return 1;
    input.tickMs += input.periodMs;
    output.motorCurrent[0] = 1234;
    if (!TestCheck(ChassisCtrlStep(&input, &output) == ControlResultResourceBusy &&
                   TestOutputZero(&output) &&
                   s_runtimeStepCount == 2u &&
                   s_runtimeStopCount == 1u,
                   "资源冲突帧必须保留旧控制器并立即清除 Runtime 输出")) return 1;
    if (!TestStatus(&status)) return 1;
    if (!TestCheck(status.active_id == ControlIdClassicChassis &&
                   status.update_count == 4u &&
                   status.last_result == ControlResultResourceBusy,
                   "资源冲突不得替换当前底盘控制器或伪造更新计数")) return 1;

    input.tickMs += input.periodMs;
    if (!TestCheck(ChassisCtrlStep(&input, &output) == ControlResultOk &&
                   s_runtimeInitCount == 1u &&
                   s_runtimeStepCount == 3u &&
                   s_runtimeStopCount == 1u &&
                   output.motorCurrent[0] == 300,
                   "资源冲突后的下一帧应直接恢复 Update")) return 1;

    if (!TestCheck(ControlMgrStop(ControlDomainChassis, ControlReasonDisable) == ControlResultOk,
                   "请求停用底盘控制域失败")) return 1;
    if (!TestStatus(&status)) return 1;
    if (!TestCheck(status.pending_request == ControlRequestStop &&
                   status.active != 0u &&
                   s_runtimeStopCount == 1u,
                   "Stop 请求在下一次 Step 前只能保持 pending")) return 1;
    updatesBeforeStop = status.update_count;
    input.tickMs += input.periodMs;
    output.motorCurrent[0] = 1234;
    if (!TestCheck(ChassisCtrlStep(&input, &output) == ControlResultNotActive &&
                   TestOutputZero(&output) &&
                   s_runtimeStopCount == 2u,
                   "显式 Stop 应执行停止回调、清零输出并返回 NotActive")) return 1;
    if (!TestStatus(&status)) return 1;
    if (!TestCheck(status.active == 0u &&
                   status.state == ControlStateStopped &&
                   status.pending_request == ControlRequestNone &&
                   status.update_count == updatesBeforeStop,
                   "显式 Stop 后不能继续执行 update")) return 1;

    stepsBeforeRestart = s_runtimeStepCount;
    stopsBeforeRestart = s_runtimeStopCount;
    if (!TestCheck(ControlMgrSwitch(ControlIdClassicChassis, ControlReasonModeSwitch) == ControlResultOk,
                   "重新请求底盘控制器失败")) return 1;
    input.tickMs += input.periodMs;
    if (!TestCheck(ChassisCtrlStep(&input, &output) == ControlResultOk &&
                   s_runtimeInitCount == 1u &&
                   s_runtimeStepCount == stepsBeforeRestart + 1u &&
                   s_runtimeStopCount == stopsBeforeRestart + 1u &&
                   s_runtimeEvents[s_runtimeEventCount - 2u] == 'S' &&
                   s_runtimeEvents[s_runtimeEventCount - 1u] == 'U' &&
                   output.motorCurrent[0] == (int16_t)(100u * s_runtimeStepCount),
                   "重切应先 Stop 后 Update，且整个生命周期只 Init 一次")) return 1;
    if (!TestStatus(&status)) return 1;
    if (!TestCheck(status.active != 0u &&
                   status.active_id == ControlIdClassicChassis &&
                   status.update_count == updatesBeforeStop + 1u,
                   "重新启用后应恢复活动状态且只增加一次更新计数")) return 1;

    stepsBeforeFault = s_runtimeStepCount;
    stopsBeforeFault = s_runtimeStopCount;
    if (!TestCheck(ControlMgrUpdateDomain(ControlDomainChassis, NULL) == ControlResultCallbackFailed &&
                   s_runtimeStepCount == stepsBeforeFault &&
                   s_runtimeStopCount == stopsBeforeFault + 1u,
                   "绕过门面的空上下文必须触发停止回调并进入故障")) return 1;
    if (!TestStatus(&status)) return 1;
    if (!TestCheck(status.active == 0u &&
                   status.state == ControlStateFault &&
                   status.last_result == ControlResultBadArgument,
                   "空上下文回调失败后底盘控制域必须处于故障停用状态")) return 1;

    (void)puts("PASS: Chassis controller lifecycle regression");
    return 0;
}
