/* 真实 ControlMgr 主机回归：覆盖重入、资源预留和保护停机边界。 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "ControlMgr.h"

typedef struct
{
    uint32_t enterCount;
    uint32_t updateCount;
    uint32_t exitCount;
    uint32_t stopCount;
    uint8_t recurseUpdate;
    uint8_t stopQueueSame;
    uint16_t enterSwitchId;
    uint16_t exitSwitchId;
    uint16_t stopSwitchId;
    ControlDomain exitUpdateDomain;
    ControlReason enterStopReason;
    ControlReason updateStopReason;
    ControlReason exitStopReason;
    ControlReason stopUpgradeReason;
    ControlResult enterResult;
    ControlResult updateResult;
    ControlResult exitResult;
    ControlResult stopResult;
    ControlResult nestedUpdateResult;
    ControlResult enterStopResult;
    ControlResult enterSwitchResult;
    ControlResult updateStopResult;
    ControlResult exitStopResult;
    ControlResult exitSwitchResult;
    ControlResult exitUpdateResult;
    ControlResult stopSameResult;
    ControlResult stopUpgradeResult;
    ControlResult stopSwitchResult;
    ControlReason lastStopReason;
} TestBehavior;

static int s_criticalDepth;
static uint32_t s_criticalErrorCount;
static uint32_t s_callbackCriticalErrorCount;

void ControlMgrTestEnterCritical(void)
{
    s_criticalDepth++;
}

void ControlMgrTestExitCritical(void)
{
    if (s_criticalDepth <= 0)
    {
        s_criticalErrorCount++;
        return;
    }
    s_criticalDepth--;
}

static void TestCallbackBoundary(void)
{
    if (s_criticalDepth != 0)
    {
        s_callbackCriticalErrorCount++;
    }
}

static ControlResult TestEnter(const ControlController *controller, ControlCtx *context)
{
    TestBehavior *behavior = (TestBehavior *)controller->user;
    (void)context;
    TestCallbackBoundary();
    behavior->enterCount++;
    if (behavior->enterStopReason != ControlReasonNone)
    {
        behavior->enterStopResult = ControlMgrStop(controller->domain,
                                                   behavior->enterStopReason);
    }
    if (behavior->enterSwitchId != ControlIdNone)
    {
        behavior->enterSwitchResult = ControlMgrSwitch(behavior->enterSwitchId,
                                                       ControlReasonModeSwitch);
    }
    return behavior->enterResult;
}

static ControlResult TestUpdate(const ControlController *controller, ControlCtx *context)
{
    TestBehavior *behavior = (TestBehavior *)controller->user;
    TestCallbackBoundary();
    behavior->updateCount++;
    if (behavior->recurseUpdate != 0u)
    {
        behavior->recurseUpdate = 0u;
        behavior->nestedUpdateResult = ControlMgrUpdateDomainDue(controller->domain,
                                                                 context->tick_ms,
                                                                 context);
    }
    if (behavior->updateStopReason != ControlReasonNone)
    {
        behavior->updateStopResult = ControlMgrStop(controller->domain,
                                                    behavior->updateStopReason);
    }
    return behavior->updateResult;
}

static ControlResult TestExit(const ControlController *controller, ControlCtx *context)
{
    TestBehavior *behavior = (TestBehavior *)controller->user;
    (void)context;
    TestCallbackBoundary();
    behavior->exitCount++;
    if (behavior->exitStopReason != ControlReasonNone)
    {
        behavior->exitStopResult = ControlMgrStop(controller->domain,
                                                  behavior->exitStopReason);
    }
    if (behavior->exitSwitchId != ControlIdNone)
    {
        behavior->exitSwitchResult = ControlMgrSwitch(behavior->exitSwitchId,
                                                      ControlReasonTest);
        behavior->exitUpdateResult = ControlMgrUpdateDomain(behavior->exitUpdateDomain,
                                                            context);
    }
    return behavior->exitResult;
}

static ControlResult TestStop(const ControlController *controller, ControlCtx *context)
{
    TestBehavior *behavior = (TestBehavior *)controller->user;
    TestCallbackBoundary();
    behavior->stopCount++;
    behavior->lastStopReason = context->reason;
    if (behavior->stopUpgradeReason != ControlReasonNone)
    {
        behavior->stopUpgradeResult = ControlMgrStop(controller->domain,
                                                     behavior->stopUpgradeReason);
    }
    if (behavior->stopQueueSame != 0u)
    {
        behavior->stopSameResult = ControlMgrStop(controller->domain, context->reason);
    }
    if (behavior->stopSwitchId != ControlIdNone)
    {
        behavior->stopSwitchResult = ControlMgrSwitch(behavior->stopSwitchId,
                                                      ControlReasonModeSwitch);
    }
    return behavior->stopResult;
}

static ControlController TestController(uint16_t id,
                                        ControlDomain domain,
                                        uint32_t claims,
                                        const char *name,
                                        TestBehavior *behavior)
{
    ControlController controller;
    memset(&controller, 0, sizeof(controller));
    controller.id = id;
    controller.domain = domain;
    controller.claim_mask = claims;
    controller.name = name;
    controller.meta.period_ms = 1u;
    controller.enter = TestEnter;
    controller.update = TestUpdate;
    controller.exit = TestExit;
    controller.stop = TestStop;
    controller.user = behavior;
    return controller;
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

#define TEST_CHECK(condition, message) \
    do { if (!TestCheck((condition), (message))) return 0; } while (0)

static int TestUpdateReentry(void)
{
    TestBehavior behavior = {0};
    ControlController controller;
    ControlStatus status;
    ControlMgrDiag diag;

    ControlMgrReset();
    behavior.recurseUpdate = 1u;
    controller = TestController(ControlIdCustomBase,
                                ControlDomainChassis,
                                ControlResChassisWheels,
                                "controller.test.reentry",
                                &behavior);
    TEST_CHECK(ControlMgrRegister(&controller) == ControlResultOk, "重入场景注册失败");
    TEST_CHECK(ControlMgrSwitch(controller.id, ControlReasonTest) == ControlResultOk,
               "重入场景切换请求失败");
    TEST_CHECK(ControlMgrUpdateDomain(ControlDomainChassis, NULL) == ControlResultOk,
               "外层更新不应被内层重入破坏");
    TEST_CHECK(behavior.updateCount == 1u &&
               behavior.nestedUpdateResult == ControlResultResourceBusy,
               "同域递归更新必须被共享更新门拒绝");
    TEST_CHECK(ControlMgrGetStatus(ControlDomainChassis, &status) == ControlResultOk &&
               status.update_count == 1u && status.state == ControlStateRunning,
               "递归拒绝不能伪造更新计数或停掉控制器");
    TEST_CHECK(ControlMgrGetDiag(&diag) == ControlResultOk && diag.updateReentryCount == 1u,
               "递归更新诊断计数错误");
    return 1;
}

static int TestCrossDomainReservation(void)
{
    TestBehavior oldBehavior = {0};
    TestBehavior nextBehavior = {0};
    TestBehavior blockerBehavior = {0};
    ControlController oldController;
    ControlController nextController;
    ControlController blocker;
    ControlMgrDiag diag;

    ControlMgrReset();
    oldController = TestController(ControlIdCustomBase,
                                   ControlDomainChassis,
                                   ControlResChassisWheels,
                                   "controller.test.old",
                                   &oldBehavior);
    nextController = TestController(ControlIdCustomBase + 1u,
                                    ControlDomainChassis,
                                    ControlResArm,
                                    "controller.test.next",
                                    &nextBehavior);
    blocker = TestController(ControlIdCustomBase + 2u,
                             ControlDomainSystem,
                             ControlResArm,
                             "controller.test.blocker",
                             &blockerBehavior);
    TEST_CHECK(ControlMgrRegister(&oldController) == ControlResultOk &&
               ControlMgrRegister(&nextController) == ControlResultOk &&
               ControlMgrRegister(&blocker) == ControlResultOk,
               "资源预留场景注册失败");
    TEST_CHECK(ControlMgrSwitch(oldController.id, ControlReasonStartup) == ControlResultOk &&
               ControlMgrUpdateDomain(ControlDomainChassis, NULL) == ControlResultOk,
               "旧控制器启动失败");

    oldBehavior.exitSwitchId = blocker.id;
    oldBehavior.exitUpdateDomain = ControlDomainSystem;
    TEST_CHECK(ControlMgrSwitch(nextController.id, ControlReasonModeSwitch) == ControlResultOk &&
               ControlMgrUpdateDomain(ControlDomainChassis, NULL) == ControlResultOk,
               "外层控制器切换失败");
    TEST_CHECK(oldBehavior.exitSwitchResult == ControlResultOk &&
               oldBehavior.exitUpdateResult == ControlResultResourceBusy,
               "旧 exit 内的跨域启动必须看见外层资源预留");
    TEST_CHECK(blockerBehavior.enterCount == 0u && blockerBehavior.updateCount == 0u &&
               nextBehavior.enterCount == 1u && nextBehavior.updateCount == 1u,
               "冲突控制器不能与预留控制器同时进入运行态");
    TEST_CHECK(ControlMgrActiveId(ControlDomainChassis) == nextController.id &&
               ControlMgrActiveId(ControlDomainSystem) == ControlIdNone &&
               ControlMgrActiveClaimMask() == ControlResArm,
               "资源预留切换后的活动资源错误");
    TEST_CHECK(ControlMgrGetDiag(&diag) == ControlResultOk &&
               diag.claimConflictCount == 1u && diag.reservedClaimMask == 0u,
               "资源冲突或预留释放诊断错误");
    return 1;
}

static int TestReservationFailureCleanup(void)
{
    TestBehavior oldBehavior = {0};
    TestBehavior nextBehavior = {0};
    TestBehavior otherBehavior = {0};
    ControlController oldController;
    ControlController nextController;
    ControlController otherController;
    ControlMgrDiag diag;

    ControlMgrReset();
    oldBehavior.exitResult = ControlResultBadArgument;
    oldController = TestController(ControlIdCustomBase,
                                   ControlDomainChassis,
                                   ControlResChassisWheels,
                                   "controller.test.exit_fail_old",
                                   &oldBehavior);
    nextController = TestController(ControlIdCustomBase + 1u,
                                    ControlDomainChassis,
                                    ControlResArm,
                                    "controller.test.exit_fail_next",
                                    &nextBehavior);
    otherController = TestController(ControlIdCustomBase + 2u,
                                     ControlDomainSystem,
                                     ControlResArm,
                                     "controller.test.exit_fail_other",
                                     &otherBehavior);
    TEST_CHECK(ControlMgrRegister(&oldController) == ControlResultOk &&
               ControlMgrRegister(&nextController) == ControlResultOk &&
               ControlMgrRegister(&otherController) == ControlResultOk &&
               ControlMgrSwitch(oldController.id, ControlReasonStartup) == ControlResultOk &&
               ControlMgrUpdateDomain(ControlDomainChassis, NULL) == ControlResultOk,
               "exit 失败场景准备失败");
    TEST_CHECK(ControlMgrSwitch(nextController.id, ControlReasonModeSwitch) == ControlResultOk &&
               ControlMgrUpdateDomain(ControlDomainChassis, NULL) == ControlResultBadArgument,
               "旧 exit 失败必须返回原始错误");
    TEST_CHECK(ControlMgrGetDiag(&diag) == ControlResultOk && diag.reservedClaimMask == 0u,
               "旧 exit 失败后必须释放预留资源");
    TEST_CHECK(ControlMgrSwitch(otherController.id, ControlReasonTest) == ControlResultOk &&
               ControlMgrUpdateDomain(ControlDomainSystem, NULL) == ControlResultOk &&
               ControlMgrActiveId(ControlDomainSystem) == otherController.id,
               "旧 exit 失败释放的资源必须可被其他域获取");

    memset(&nextBehavior, 0, sizeof(nextBehavior));
    memset(&otherBehavior, 0, sizeof(otherBehavior));
    ControlMgrReset();
    nextBehavior.enterResult = ControlResultBadArgument;
    nextController = TestController(ControlIdCustomBase + 3u,
                                    ControlDomainChassis,
                                    ControlResArm,
                                    "controller.test.enter_fail_next",
                                    &nextBehavior);
    otherController = TestController(ControlIdCustomBase + 4u,
                                     ControlDomainSystem,
                                     ControlResArm,
                                     "controller.test.enter_fail_other",
                                     &otherBehavior);
    TEST_CHECK(ControlMgrRegister(&nextController) == ControlResultOk &&
               ControlMgrRegister(&otherController) == ControlResultOk,
               "enter 失败场景注册失败");
    TEST_CHECK(ControlMgrSwitch(nextController.id, ControlReasonTest) == ControlResultOk &&
               ControlMgrUpdateDomain(ControlDomainChassis, NULL) == ControlResultBadArgument &&
               ControlMgrActiveId(ControlDomainChassis) == ControlIdNone,
               "新 enter 失败必须回滚活动控制器");
    TEST_CHECK(ControlMgrGetDiag(&diag) == ControlResultOk && diag.reservedClaimMask == 0u,
               "新 enter 失败后必须释放预留资源");
    TEST_CHECK(ControlMgrSwitch(otherController.id, ControlReasonTest) == ControlResultOk &&
               ControlMgrUpdateDomain(ControlDomainSystem, NULL) == ControlResultOk &&
               ControlMgrActiveId(ControlDomainSystem) == otherController.id,
               "新 enter 失败释放的资源必须可被其他域获取");
    return 1;
}

static int TestProtectedPendingPriority(void)
{
    TestBehavior firstBehavior = {0};
    TestBehavior secondBehavior = {0};
    ControlController first;
    ControlController second;
    ControlStatus status;
    ControlMgrDiag diag;

    ControlMgrReset();
    first = TestController(ControlIdCustomBase,
                           ControlDomainChassis,
                           ControlResChassisWheels,
                           "controller.test.protected_first",
                           &firstBehavior);
    second = TestController(ControlIdCustomBase + 1u,
                            ControlDomainChassis,
                            ControlResChassisWheels,
                            "controller.test.protected_second",
                            &secondBehavior);
    TEST_CHECK(ControlMgrRegister(&first) == ControlResultOk &&
               ControlMgrRegister(&second) == ControlResultOk &&
               ControlMgrSwitch(first.id, ControlReasonStartup) == ControlResultOk &&
               ControlMgrUpdateDomain(ControlDomainChassis, NULL) == ControlResultOk,
               "保护停机场景准备失败");
    TEST_CHECK(ControlMgrStop(ControlDomainChassis, ControlReasonFault) == ControlResultOk,
               "Fault 停机请求入队失败");
    TEST_CHECK(ControlMgrSwitch(second.id, ControlReasonModeSwitch) == ControlResultResourceBusy &&
               ControlMgrStop(ControlDomainChassis, ControlReasonDisable) == ControlResultResourceBusy,
               "普通 Switch/Stop 不得覆盖 Fault 停机");
    ControlMgrClearPending(ControlDomainChassis);
    TEST_CHECK(ControlMgrGetStatus(ControlDomainChassis, &status) == ControlResultOk &&
               status.pending_request == ControlRequestStop,
               "ClearPending 不得清除 Fault 停机");
    TEST_CHECK(ControlMgrUpdateDomain(ControlDomainChassis, NULL) == ControlResultNotActive &&
               firstBehavior.stopCount == 1u &&
               firstBehavior.lastStopReason == ControlReasonFault &&
               secondBehavior.enterCount == 0u,
               "Fault 停机必须保持到执行且不能误启普通切换");
    TEST_CHECK(ControlMgrGetDiag(&diag) == ControlResultOk &&
               diag.protectedRequestRejectCount == 3u,
               "受保护请求拒绝计数错误");

    memset(&firstBehavior, 0, sizeof(firstBehavior));
    ControlMgrReset();
    first = TestController(ControlIdCustomBase + 2u,
                           ControlDomainChassis,
                           ControlResChassisWheels,
                           "controller.test.priority",
                           &firstBehavior);
    TEST_CHECK(ControlMgrRegister(&first) == ControlResultOk &&
               ControlMgrSwitch(first.id, ControlReasonStartup) == ControlResultOk &&
               ControlMgrUpdateDomain(ControlDomainChassis, NULL) == ControlResultOk,
               "停机优先级场景准备失败");
    TEST_CHECK(ControlMgrStop(ControlDomainChassis, ControlReasonFault) == ControlResultOk &&
               ControlMgrStop(ControlDomainChassis, ControlReasonFault) == ControlResultOk &&
               ControlMgrStop(ControlDomainChassis, ControlReasonEmergencyStop) == ControlResultOk &&
               ControlMgrStop(ControlDomainChassis, ControlReasonFault) == ControlResultResourceBusy,
               "pending 阶段应允许同级幂等和 Emergency 升级，并拒绝降级");
    TEST_CHECK(ControlMgrUpdateDomain(ControlDomainChassis, NULL) == ControlResultNotActive &&
               firstBehavior.lastStopReason == ControlReasonEmergencyStop,
               "Emergency 必须以最高优先级执行");

    memset(&firstBehavior, 0, sizeof(firstBehavior));
    memset(&secondBehavior, 0, sizeof(secondBehavior));
    ControlMgrReset();
    first = TestController(ControlIdCustomBase + 3u,
                           ControlDomainChassis,
                           ControlResChassisWheels,
                           "controller.test.disable_first",
                           &firstBehavior);
    second = TestController(ControlIdCustomBase + 4u,
                            ControlDomainChassis,
                            ControlResChassisWheels,
                            "controller.test.disable_second",
                            &secondBehavior);
    TEST_CHECK(ControlMgrRegister(&first) == ControlResultOk &&
               ControlMgrRegister(&second) == ControlResultOk &&
               ControlMgrSwitch(first.id, ControlReasonStartup) == ControlResultOk &&
               ControlMgrUpdateDomain(ControlDomainChassis, NULL) == ControlResultOk &&
               ControlMgrStop(ControlDomainChassis, ControlReasonDisable) == ControlResultOk &&
               ControlMgrSwitch(second.id, ControlReasonModeSwitch) == ControlResultOk &&
               ControlMgrUpdateDomain(ControlDomainChassis, NULL) == ControlResultOk,
               "普通 Disable Stop 应能被后续 Switch 覆盖");
    TEST_CHECK(firstBehavior.exitCount == 1u && firstBehavior.stopCount == 0u &&
               secondBehavior.enterCount == 1u &&
               ControlMgrActiveId(ControlDomainChassis) == second.id,
               "普通停机覆盖后的生命周期错误");
    return 1;
}

static int TestExitFailureConsumesEmergencyStop(void)
{
    TestBehavior oldBehavior = {0};
    TestBehavior nextBehavior = {0};
    ControlController oldController;
    ControlController nextController;
    ControlStatus status;
    ControlMgrDiag diag;

    ControlMgrReset();
    oldBehavior.exitResult = ControlResultBadArgument;
    oldController = TestController(ControlIdCustomBase,
                                   ControlDomainChassis,
                                   ControlResChassisWheels,
                                   "controller.test.exit_estop_failure_old",
                                   &oldBehavior);
    nextController = TestController(ControlIdCustomBase + 1u,
                                    ControlDomainChassis,
                                    ControlResArm,
                                    "controller.test.exit_estop_failure_next",
                                    &nextBehavior);
    TEST_CHECK(ControlMgrRegister(&oldController) == ControlResultOk &&
               ControlMgrRegister(&nextController) == ControlResultOk &&
               ControlMgrSwitch(oldController.id, ControlReasonStartup) == ControlResultOk &&
               ControlMgrUpdateDomain(ControlDomainChassis, NULL) == ControlResultOk,
               "exit 失败合并急停场景准备失败");
    oldBehavior.exitStopReason = ControlReasonEmergencyStop;
    TEST_CHECK(ControlMgrSwitch(nextController.id, ControlReasonModeSwitch) == ControlResultOk &&
               ControlMgrUpdateDomain(ControlDomainChassis, NULL) == ControlResultBadArgument,
               "exit 失败必须保留原始返回值");
    TEST_CHECK(oldBehavior.exitStopResult == ControlResultOk &&
               oldBehavior.exitCount == 1u && nextBehavior.enterCount == 0u &&
               ControlMgrGetStatus(ControlDomainChassis, &status) == ControlResultOk &&
               status.active == 0u && status.state == ControlStateFault &&
               status.pending_request == ControlRequestNone &&
               status.last_reason == ControlReasonEmergencyStop &&
               status.last_result == ControlResultBadArgument &&
               ControlMgrActiveClaimMask() == 0u,
               "exit 失败必须消费急停、释放旧资源并保留 Fault");
    TEST_CHECK(ControlMgrGetDiag(&diag) == ControlResultOk && diag.reservedClaimMask == 0u,
               "exit 失败合并急停后不得泄漏资源预留");
    TEST_CHECK(ControlMgrUpdateDomain(ControlDomainChassis, NULL) == ControlResultNotActive &&
               ControlMgrGetStatus(ControlDomainChassis, &status) == ControlResultOk &&
               status.state == ControlStateFault &&
               status.last_result == ControlResultBadArgument,
               "下一帧不得用空域停机覆盖 exit 失败状态");
    return 1;
}

static int TestProtectedStopCallback(void)
{
    TestBehavior firstBehavior = {0};
    TestBehavior secondBehavior = {0};
    ControlController first;
    ControlController second;
    ControlStatus status;
    ControlMgrDiag diag;

    ControlMgrReset();
    first = TestController(ControlIdCustomBase,
                           ControlDomainChassis,
                           ControlResChassisWheels,
                           "controller.test.stop_callback_first",
                           &firstBehavior);
    second = TestController(ControlIdCustomBase + 1u,
                            ControlDomainChassis,
                            ControlResChassisWheels,
                            "controller.test.stop_callback_second",
                            &secondBehavior);
    TEST_CHECK(ControlMgrRegister(&first) == ControlResultOk &&
               ControlMgrRegister(&second) == ControlResultOk &&
               ControlMgrSwitch(first.id, ControlReasonStartup) == ControlResultOk &&
               ControlMgrUpdateDomain(ControlDomainChassis, NULL) == ControlResultOk,
               "保护停机回调场景准备失败");
    firstBehavior.stopQueueSame = 1u;
    firstBehavior.stopSwitchId = second.id;
    TEST_CHECK(ControlMgrStop(ControlDomainChassis, ControlReasonFault) == ControlResultOk &&
               ControlMgrUpdateDomain(ControlDomainChassis, NULL) == ControlResultNotActive,
               "保护停机执行失败");
    TEST_CHECK(firstBehavior.stopSameResult == ControlResultResourceBusy &&
               firstBehavior.stopSwitchResult == ControlResultResourceBusy,
               "保护停机执行中必须拒绝同级 Stop 和普通 Switch");
    TEST_CHECK(ControlMgrGetStatus(ControlDomainChassis, &status) == ControlResultOk &&
               status.pending_request == ControlRequestNone && status.active == 0u,
               "保护停机回调不得留下空域 pending Stop");
    TEST_CHECK(ControlMgrUpdateDomain(ControlDomainChassis, NULL) == ControlResultNotActive &&
               secondBehavior.enterCount == 0u,
               "回调内被拒绝的请求不得在下一帧复活");
    TEST_CHECK(ControlMgrGetDiag(&diag) == ControlResultOk &&
               diag.protectedRequestRejectCount == 2u,
               "执行中保护停机拒绝计数错误");
    TEST_CHECK(ControlMgrSwitch(second.id, ControlReasonModeSwitch) == ControlResultOk &&
               ControlMgrUpdateDomain(ControlDomainChassis, NULL) == ControlResultOk &&
               secondBehavior.enterCount == 1u,
               "保护停机完成后应允许新的显式 Switch");

    memset(&firstBehavior, 0, sizeof(firstBehavior));
    ControlMgrReset();
    firstBehavior.stopUpgradeReason = ControlReasonEmergencyStop;
    first = TestController(ControlIdCustomBase + 2u,
                           ControlDomainChassis,
                           ControlResChassisWheels,
                           "controller.test.stop_upgrade",
                           &firstBehavior);
    TEST_CHECK(ControlMgrRegister(&first) == ControlResultOk &&
               ControlMgrSwitch(first.id, ControlReasonStartup) == ControlResultOk &&
               ControlMgrUpdateDomain(ControlDomainChassis, NULL) == ControlResultOk &&
               ControlMgrStop(ControlDomainChassis, ControlReasonFault) == ControlResultOk &&
               ControlMgrUpdateDomain(ControlDomainChassis, NULL) == ControlResultNotActive,
               "执行中停机升级场景失败");
    TEST_CHECK(firstBehavior.stopUpgradeResult == ControlResultOk &&
               ControlMgrGetStatus(ControlDomainChassis, &status) == ControlResultOk &&
               status.last_reason == ControlReasonEmergencyStop &&
               status.pending_request == ControlRequestNone,
               "执行中的 Fault 只能原地升级为 Emergency，不能留下 pending");
    return 1;
}

static int TestCallbackCreatedEmergencyStop(void)
{
    TestBehavior oldBehavior = {0};
    TestBehavior nextBehavior = {0};
    ControlController oldController;
    ControlController nextController;
    ControlStatus status;
    ControlMgrDiag diag;

    ControlMgrReset();
    oldController = TestController(ControlIdCustomBase,
                                   ControlDomainChassis,
                                   ControlResChassisWheels,
                                   "controller.test.exit_estop_old",
                                   &oldBehavior);
    nextController = TestController(ControlIdCustomBase + 1u,
                                    ControlDomainChassis,
                                    ControlResArm,
                                    "controller.test.exit_estop_next",
                                    &nextBehavior);
    TEST_CHECK(ControlMgrRegister(&oldController) == ControlResultOk &&
               ControlMgrRegister(&nextController) == ControlResultOk &&
               ControlMgrSwitch(oldController.id, ControlReasonStartup) == ControlResultOk &&
               ControlMgrUpdateDomain(ControlDomainChassis, NULL) == ControlResultOk,
               "exit 急停场景准备失败");
    oldBehavior.exitStopReason = ControlReasonEmergencyStop;
    TEST_CHECK(ControlMgrSwitch(nextController.id, ControlReasonModeSwitch) == ControlResultOk &&
               ControlMgrUpdateDomain(ControlDomainChassis, NULL) == ControlResultResourceBusy,
               "旧 exit 新增急停后必须中止新控制器");
    TEST_CHECK(oldBehavior.exitStopResult == ControlResultOk &&
               nextBehavior.enterCount == 0u && nextBehavior.updateCount == 0u &&
               ControlMgrActiveId(ControlDomainChassis) == ControlIdNone,
               "急停插入切换时不得 enter/update 新控制器");
    TEST_CHECK(ControlMgrGetStatus(ControlDomainChassis, &status) == ControlResultOk &&
               status.pending_request == ControlRequestStop &&
               ControlMgrStop(ControlDomainChassis, ControlReasonFault) == ControlResultResourceBusy,
               "旧 exit 产生的 EmergencyStop 必须原样保留且不可降级");
    TEST_CHECK(ControlMgrGetDiag(&diag) == ControlResultOk && diag.reservedClaimMask == 0u,
               "急停中止切换后必须释放资源预留");
    TEST_CHECK(ControlMgrUpdateDomain(ControlDomainChassis, NULL) == ControlResultNotActive &&
               ControlMgrGetStatus(ControlDomainChassis, &status) == ControlResultOk &&
               status.state == ControlStateStopped &&
               status.pending_request == ControlRequestNone &&
               status.last_reason == ControlReasonEmergencyStop,
               "中止切换后的空域急停必须被消费并记录原因");

    memset(&nextBehavior, 0, sizeof(nextBehavior));
    ControlMgrReset();
    nextBehavior.enterStopReason = ControlReasonEmergencyStop;
    nextController = TestController(ControlIdCustomBase + 2u,
                                    ControlDomainChassis,
                                    ControlResChassisWheels,
                                    "controller.test.enter_estop",
                                    &nextBehavior);
    TEST_CHECK(ControlMgrRegister(&nextController) == ControlResultOk &&
               ControlMgrSwitch(nextController.id, ControlReasonStartup) == ControlResultOk &&
               ControlMgrUpdateDomain(ControlDomainChassis, NULL) == ControlResultNotActive,
               "enter 内急停必须在同帧应用");
    TEST_CHECK(nextBehavior.enterStopResult == ControlResultOk &&
               nextBehavior.enterCount == 1u && nextBehavior.updateCount == 0u &&
               nextBehavior.stopCount == 1u &&
               nextBehavior.lastStopReason == ControlReasonEmergencyStop,
               "enter 内急停后不得多运行一帧 update");
    TEST_CHECK(ControlMgrGetStatus(ControlDomainChassis, &status) == ControlResultOk &&
               status.active == 0u && status.pending_request == ControlRequestNone,
               "enter 内急停执行后不得残留活动控制器或 pending");
    return 1;
}

static int TestUpdateFailureMergesEmergencyStop(void)
{
    TestBehavior behavior = {0};
    ControlController controller;
    ControlStatus status;

    ControlMgrReset();
    behavior.updateStopReason = ControlReasonEmergencyStop;
    behavior.updateResult = ControlResultBadArgument;
    controller = TestController(ControlIdCustomBase,
                                ControlDomainChassis,
                                ControlResChassisWheels,
                                "controller.test.update_estop_failure",
                                &behavior);
    TEST_CHECK(ControlMgrRegister(&controller) == ControlResultOk &&
               ControlMgrSwitch(controller.id, ControlReasonStartup) == ControlResultOk,
               "update 失败合并急停场景准备失败");
    TEST_CHECK(ControlMgrUpdateDomain(ControlDomainChassis, NULL) == ControlResultCallbackFailed,
               "update 回调失败必须返回 CallbackFailed");
    TEST_CHECK(behavior.updateStopResult == ControlResultOk &&
               behavior.stopCount == 1u &&
               behavior.lastStopReason == ControlReasonEmergencyStop,
               "update 内 EmergencyStop 必须覆盖自动 Fault 停机原因");
    TEST_CHECK(ControlMgrGetStatus(ControlDomainChassis, &status) == ControlResultOk &&
               status.active == 0u && status.state == ControlStateFault &&
               status.pending_request == ControlRequestNone &&
               status.last_reason == ControlReasonEmergencyStop &&
               status.last_result == ControlResultBadArgument,
               "自动 Fault 停机必须消费已排队急停并保留原始回调错误");
    TEST_CHECK(ControlMgrUpdateDomain(ControlDomainChassis, NULL) == ControlResultNotActive &&
               ControlMgrGetStatus(ControlDomainChassis, &status) == ControlResultOk &&
               status.state == ControlStateFault,
               "下一帧不得用空域 Stop 覆盖 Fault 状态");
    return 1;
}

static int TestEnterFailureConsumesEmergencyStop(void)
{
    TestBehavior behavior = {0};
    ControlController controller;
    ControlStatus status;

    ControlMgrReset();
    behavior.enterStopReason = ControlReasonEmergencyStop;
    behavior.enterResult = ControlResultBadArgument;
    controller = TestController(ControlIdCustomBase,
                                ControlDomainChassis,
                                ControlResChassisWheels,
                                "controller.test.enter_estop_failure",
                                &behavior);
    TEST_CHECK(ControlMgrRegister(&controller) == ControlResultOk &&
               ControlMgrSwitch(controller.id, ControlReasonStartup) == ControlResultOk,
               "enter 失败合并急停场景准备失败");
    TEST_CHECK(ControlMgrUpdateDomain(ControlDomainChassis, NULL) == ControlResultBadArgument,
               "enter 失败必须保留原始返回值");
    TEST_CHECK(behavior.enterStopResult == ControlResultOk &&
               behavior.enterCount == 1u && behavior.updateCount == 0u &&
               behavior.stopCount == 1u &&
               behavior.lastStopReason == ControlReasonEmergencyStop,
               "enter 失败必须用最高优先级停机清理已进入的执行体");
    TEST_CHECK(ControlMgrGetStatus(ControlDomainChassis, &status) == ControlResultOk &&
               status.active == 0u && status.state == ControlStateFault &&
               status.pending_request == ControlRequestNone &&
               status.last_reason == ControlReasonEmergencyStop &&
               status.last_result == ControlResultBadArgument &&
               ControlMgrActiveClaimMask() == 0u,
               "enter 失败后必须释放资源、消费急停并保留 Fault");
    TEST_CHECK(ControlMgrUpdateDomain(ControlDomainChassis, NULL) == ControlResultNotActive &&
               ControlMgrGetStatus(ControlDomainChassis, &status) == ControlResultOk &&
               status.state == ControlStateFault &&
               status.last_result == ControlResultBadArgument,
               "下一帧不得用空域停机覆盖 enter 失败状态");
    return 1;
}

static int TestUpdateSuccessAppliesEmergencyStop(void)
{
    TestBehavior behavior = {0};
    ControlController controller;
    ControlStatus status;

    ControlMgrReset();
    behavior.updateStopReason = ControlReasonEmergencyStop;
    controller = TestController(ControlIdCustomBase,
                                ControlDomainChassis,
                                ControlResChassisWheels,
                                "controller.test.update_estop_success",
                                &behavior);
    TEST_CHECK(ControlMgrRegister(&controller) == ControlResultOk &&
               ControlMgrSwitch(controller.id, ControlReasonStartup) == ControlResultOk,
               "update 成功后急停场景准备失败");
    TEST_CHECK(ControlMgrUpdateDomain(ControlDomainChassis, NULL) == ControlResultNotActive,
               "update 回调排入急停后必须同帧停止控制域");
    TEST_CHECK(behavior.updateStopResult == ControlResultOk &&
               behavior.updateCount == 1u && behavior.stopCount == 1u &&
               behavior.lastStopReason == ControlReasonEmergencyStop,
               "update 内急停必须在返回任务前执行 stop");
    TEST_CHECK(ControlMgrGetStatus(ControlDomainChassis, &status) == ControlResultOk &&
               status.active == 0u && status.state == ControlStateStopped &&
               status.pending_request == ControlRequestNone &&
               status.update_count == 1u &&
               status.last_reason == ControlReasonEmergencyStop &&
               ControlMgrActiveClaimMask() == 0u,
               "update 内急停必须清活动资源且不能留下 pending");
    return 1;
}

static int TestFaultStateSurvivesEmptyProtectedStop(void)
{
    TestBehavior behavior = {0};
    ControlController controller;
    ControlStatus status;

    ControlMgrReset();
    behavior.updateResult = ControlResultBadArgument;
    controller = TestController(ControlIdCustomBase,
                                ControlDomainChassis,
                                ControlResChassisWheels,
                                "controller.test.fault_empty_estop",
                                &behavior);
    TEST_CHECK(ControlMgrRegister(&controller) == ControlResultOk &&
               ControlMgrSwitch(controller.id, ControlReasonStartup) == ControlResultOk &&
               ControlMgrUpdateDomain(ControlDomainChassis, NULL) == ControlResultCallbackFailed,
               "空域保护停机状态场景准备失败");
    TEST_CHECK(ControlMgrStop(ControlDomainChassis, ControlReasonEmergencyStop) == ControlResultOk &&
               ControlMgrUpdateDomain(ControlDomainChassis, NULL) == ControlResultNotActive,
               "Fault 空域的 EmergencyStop 应被正常消费");
    TEST_CHECK(ControlMgrGetStatus(ControlDomainChassis, &status) == ControlResultOk &&
               status.active == 0u && status.state == ControlStateFault &&
               status.pending_request == ControlRequestNone &&
               status.last_reason == ControlReasonEmergencyStop &&
               status.last_result == ControlResultBadArgument,
               "空域保护停机不能覆盖原 Fault 和原始错误");
    return 1;
}

static int TestOrdinaryCallbackRequestWaitsNextFrame(void)
{
    TestBehavior firstBehavior = {0};
    TestBehavior secondBehavior = {0};
    ControlController first;
    ControlController second;
    ControlStatus status;

    ControlMgrReset();
    first = TestController(ControlIdCustomBase,
                           ControlDomainChassis,
                           ControlResChassisWheels,
                           "controller.test.callback_pending_first",
                           &firstBehavior);
    second = TestController(ControlIdCustomBase + 1u,
                            ControlDomainChassis,
                            ControlResChassisWheels,
                            "controller.test.callback_pending_second",
                            &secondBehavior);
    firstBehavior.enterSwitchId = second.id;
    TEST_CHECK(ControlMgrRegister(&first) == ControlResultOk &&
               ControlMgrRegister(&second) == ControlResultOk &&
               ControlMgrSwitch(first.id, ControlReasonStartup) == ControlResultOk &&
               ControlMgrUpdateDomain(ControlDomainChassis, NULL) == ControlResultOk,
               "enter 内普通 Switch 场景首帧失败");
    TEST_CHECK(firstBehavior.enterSwitchResult == ControlResultOk &&
               firstBehavior.updateCount == 1u && secondBehavior.enterCount == 0u &&
               ControlMgrGetStatus(ControlDomainChassis, &status) == ControlResultOk &&
               status.active_id == first.id &&
               status.pending_request == ControlRequestSwitch &&
               status.pending_id == second.id,
               "回调产生的普通请求必须保留到下一帧");
    TEST_CHECK(ControlMgrUpdateDomain(ControlDomainChassis, NULL) == ControlResultOk &&
               firstBehavior.exitCount == 1u && secondBehavior.enterCount == 1u &&
               secondBehavior.updateCount == 1u &&
               ControlMgrActiveId(ControlDomainChassis) == second.id,
               "下一帧必须正常消费回调产生的普通请求");
    return 1;
}

static int TestDiagnostics(void)
{
    TestBehavior behavior = {0};
    ControlController controller;
    ControlMgrDiag diag;
    ControlMgrDiag zero = {0};

    ControlMgrReset();
    TEST_CHECK(ControlMgrGetDiag(&diag) == ControlResultOk &&
               memcmp(&diag, &zero, sizeof(diag)) == 0,
               "Reset 后管理器诊断必须清零");
    TEST_CHECK(ControlMgrRegister(NULL) == ControlResultBadArgument,
               "空控制器注册应失败并进入诊断");
    controller = TestController(ControlIdCustomBase,
                                ControlDomainChassis,
                                ControlResChassisWheels,
                                "controller.test.diag",
                                &behavior);
    TEST_CHECK(ControlMgrRegister(&controller) == ControlResultOk,
               "诊断场景有效注册失败");
    TEST_CHECK(ControlMgrGetDiag(&diag) == ControlResultOk &&
               diag.registerAttemptCount == 2u && diag.registerFailCount == 1u &&
               diag.lastRegisterErrorId == ControlIdNone &&
               diag.lastRegisterError == ControlResultBadArgument,
               "成功注册不得擦除最近一次注册错误");
    TEST_CHECK(ControlMgrRegister(&controller) == ControlResultDuplicate,
               "重复注册应返回 Duplicate");
    TEST_CHECK(ControlMgrSwitch(999u, ControlReasonTest) == ControlResultNotFound &&
               ControlMgrSwitch(controller.id, ControlReasonTest) == ControlResultOk,
               "Switch 诊断场景请求结果错误");
    TEST_CHECK(ControlMgrGetDiag(&diag) == ControlResultOk &&
               diag.switchAttemptCount == 2u && diag.switchFailCount == 1u &&
               diag.lastSwitchErrorId == 999u &&
               diag.lastSwitchError == ControlResultNotFound,
               "成功 Switch 不得擦除最近一次 Switch 错误");

    for (uint16_t i = 1u; i < (uint16_t)CONTROL_MGR_MAX_CONTROLLERS; i++)
    {
        controller = TestController((uint16_t)(ControlIdCustomBase + i),
                                    ControlDomainSystem,
                                    0u,
                                    "controller.test.diag_fill",
                                    &behavior);
        TEST_CHECK(ControlMgrRegister(&controller) == ControlResultOk,
                   "填满注册表失败");
    }
    controller = TestController(200u,
                                ControlDomainSystem,
                                0u,
                                "controller.test.diag_full",
                                &behavior);
    TEST_CHECK(ControlMgrRegister(&controller) == ControlResultFull,
               "满注册表应返回 Full");
    TEST_CHECK(ControlMgrGetDiag(&diag) == ControlResultOk &&
               diag.registerAttemptCount == 19u && diag.registerFailCount == 3u &&
               diag.lastRegisterErrorId == 200u &&
               diag.lastRegisterError == ControlResultFull,
               "注册尝试、失败和最近错误诊断错误");

    ControlMgrReset();
    TEST_CHECK(ControlMgrGetDiag(&diag) == ControlResultOk &&
               memcmp(&diag, &zero, sizeof(diag)) == 0,
               "再次 Reset 必须清除全部诊断");
    return 1;
}

int main(void)
{
    if (!TestUpdateReentry()) return 1;
    if (!TestCrossDomainReservation()) return 1;
    if (!TestReservationFailureCleanup()) return 1;
    if (!TestProtectedPendingPriority()) return 1;
    if (!TestExitFailureConsumesEmergencyStop()) return 1;
    if (!TestProtectedStopCallback()) return 1;
    if (!TestCallbackCreatedEmergencyStop()) return 1;
    if (!TestUpdateFailureMergesEmergencyStop()) return 1;
    if (!TestEnterFailureConsumesEmergencyStop()) return 1;
    if (!TestUpdateSuccessAppliesEmergencyStop()) return 1;
    if (!TestFaultStateSurvivesEmptyProtectedStop()) return 1;
    if (!TestOrdinaryCallbackRequestWaitsNextFrame()) return 1;
    if (!TestDiagnostics()) return 1;
    if (!TestCheck(s_criticalDepth == 0 &&
                   s_criticalErrorCount == 0u &&
                   s_callbackCriticalErrorCount == 0u,
                   "临界区未配对，或回调在临界区内执行")) return 1;
    (void)puts("PASS: ControlMgr regression");
    return 0;
}
