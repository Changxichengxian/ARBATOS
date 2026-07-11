/*
 * FaultMgr 主机回归：验证隔离边界、强耦合域和恢复门槛。
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "FaultMgr.h"

#define REASON_OFFLINE (1u << 0)
#define REASON_DATA (1u << 1)

static int TestCheck(int condition, const char *message)
{
    if (condition != 0)
    {
        return 1;
    }

    fprintf(stderr, "FAIL: %s\n", message);
    return 0;
}

static FaultMgrConfig TestConfig(void)
{
    FaultMgrConfig config;

    (void)memset(&config, 0, sizeof(config));
    config.deviceCount = 8u;
    config.domainCount = 1u;
    config.device[0].stableMs = 10u;
    config.device[4].stableMs = 100u;
    config.device[4].requireSafeInput = 1u;
    config.domain[0].memberMask = 0x3Fu;
    config.domain[0].criticalMask = 0x1Fu;
    config.domain[0].recovery.stableMs = 50u;
    config.domain[0].recovery.requireSafeInput = 1u;
    return config;
}

static int TestInvalidAndConservative(void)
{
    FaultMgr mgr;
    FaultMgrConfig config = TestConfig();
    FaultDeviceStatus device;
    FaultDomainStatus domain;

    (void)memset(&mgr, 0, sizeof(mgr));
    if (!TestCheck(FaultMgrDeviceAction(&mgr, 0u) == FaultActionStopDomain &&
                   FaultMgrDeviceAction(NULL, 0u) == FaultActionStopDomain,
                   "未初始化设备查询必须保守停本域")) return 0;
    if (!TestCheck(FaultMgrDomainAction(&mgr, 0u) == FaultActionStopDomain &&
                   FaultMgrDomainAction(NULL, 0u) == FaultActionStopDomain,
                   "未初始化域查询必须保守停本域")) return 0;
    if (!TestCheck(FaultMgrGetDeviceStatus(&mgr, 0u, &device) == FaultMgrResultNotReady &&
                   device.action == FaultActionStopDomain,
                   "未初始化设备状态必须保守停本域")) return 0;
    if (!TestCheck(FaultMgrGetDomainStatus(&mgr, 0u, &domain) == FaultMgrResultNotReady &&
                   domain.action == FaultActionStopDomain,
                   "未初始化域状态必须保守停本域")) return 0;
    if (!TestCheck(FaultMgrInit(NULL, &config) == FaultMgrResultInvalidArg,
                   "空管理器应拒绝初始化")) return 0;

    config.domain[0].criticalMask = 0x40u;
    if (!TestCheck(FaultMgrInit(&mgr, &config) == FaultMgrResultInvalidConfig,
                   "关键成员必须属于域")) return 0;
    if (!TestCheck(FaultMgrDeviceAction(&mgr, 0u) == FaultActionStopDomain &&
                   FaultMgrDomainAction(&mgr, 0u) == FaultActionStopDomain,
                   "非法配置后仍应保守停本域")) return 0;

    config = TestConfig();
    if (!TestCheck(FaultMgrInit(&mgr, &config) == FaultMgrResultOk,
                   "有效配置初始化失败")) return 0;
    if (!TestCheck(FaultMgrDeviceAction(&mgr, config.deviceCount) == FaultActionStopDomain &&
                   FaultMgrDomainAction(&mgr, config.domainCount) == FaultActionStopDomain,
                   "越界动作查询必须保守停本域")) return 0;
    if (!TestCheck(FaultMgrGetDeviceStatus(&mgr, config.deviceCount, &device) ==
                       FaultMgrResultInvalidArg &&
                   device.action == FaultActionStopDomain,
                   "越界设备状态必须保守停本域")) return 0;
    if (!TestCheck(FaultMgrGetDomainStatus(&mgr, config.domainCount, &domain) ==
                       FaultMgrResultInvalidArg &&
                   domain.action == FaultActionStopDomain,
                   "越界域状态必须保守停本域")) return 0;

    config = TestConfig();
    config.domainCount = 2u;
    config.domain[1].memberMask = 0x30u;
    config.domain[1].criticalMask = 0x10u;
    if (!TestCheck(FaultMgrInit(&mgr, &config) == FaultMgrResultInvalidConfig,
                   "设备不可同时属于两个域")) return 0;
    return 1;
}

static int TestIsolationAndStrongDomain(void)
{
    FaultMgr mgr;
    FaultMgrConfig config = TestConfig();
    FaultDomainStatus domain;
    FaultDeviceStatus device;

    if (!TestCheck(FaultMgrInit(&mgr, &config) == FaultMgrResultOk, "初始化失败")) return 0;

    if (!TestCheck(FaultMgrSetDeviceFault(&mgr, 6u, REASON_OFFLINE, 1u, 100u) == FaultMgrResultOk,
                   "未分组设备故障上报失败")) return 0;
    if (!TestCheck(FaultMgrDeviceAction(&mgr, 6u) == FaultActionIsolateDevice,
                   "未分组设备应只隔离自身")) return 0;
    if (!TestCheck(FaultMgrDeviceAction(&mgr, 7u) == FaultActionRun,
                   "未分组设备故障不应影响其它设备")) return 0;
    if (!TestCheck(FaultMgrDomainAction(&mgr, 0u) == FaultActionRun,
                   "未分组设备故障不应停域")) return 0;

    if (!TestCheck(FaultMgrSetDeviceFault(&mgr, 5u, REASON_DATA, 1u, 110u) == FaultMgrResultOk,
                   "非关键域成员故障上报失败")) return 0;
    if (!TestCheck(FaultMgrDeviceAction(&mgr, 5u) == FaultActionIsolateDevice,
                   "非关键域成员应只隔离自身")) return 0;
    if (!TestCheck(FaultMgrDeviceAction(&mgr, 0u) == FaultActionRun &&
                   FaultMgrDomainAction(&mgr, 0u) == FaultActionRun,
                   "非关键域成员不应拖停全域")) return 0;

    if (!TestCheck(FaultMgrSetDeviceFault(&mgr, 2u, REASON_OFFLINE, 1u, 120u) == FaultMgrResultOk,
                   "关键域成员故障上报失败")) return 0;
    (void)FaultMgrSetDeviceFault(&mgr, 2u, REASON_OFFLINE, 1u, 150u);
    if (!TestCheck(FaultMgrDomainAction(&mgr, 0u) == FaultActionStopDomain,
                   "关键成员故障必须停域")) return 0;
    if (!TestCheck(FaultMgrDeviceAction(&mgr, 0u) == FaultActionStopDomain &&
                   FaultMgrDeviceAction(&mgr, 5u) == FaultActionStopDomain,
                   "域内健康与非关键成员都应服从停域动作")) return 0;
    if (!TestCheck(FaultMgrDeviceAction(&mgr, 6u) == FaultActionIsolateDevice,
                   "停域不应扩大到域外设备")) return 0;

    if (!TestCheck(FaultMgrGetDomainStatus(&mgr, 0u, &domain) == FaultMgrResultOk,
                   "域状态查询失败")) return 0;
    if (!TestCheck(domain.activeMemberMask == (1u << 2) &&
                   domain.blockingMemberMask == (1u << 2) &&
                   domain.activeReasonMask == REASON_OFFLINE,
                   "域成员或原因位图错误")) return 0;
    if (!TestCheck(domain.firstFaultMs == 120u && domain.lastFaultMs == 120u,
                   "重复采样不应刷新域故障边沿时间")) return 0;

    if (!TestCheck(FaultMgrGetDeviceStatus(&mgr, 2u, &device) == FaultMgrResultOk,
                   "设备状态查询失败")) return 0;
    if (!TestCheck(device.domainId == 0u && device.firstFaultMs == 120u &&
                   device.lastFaultMs == 120u && device.blockingReasonMask == REASON_OFFLINE,
                   "设备归属、边沿时间或阻断原因错误")) return 0;
    return 1;
}

static int TestMultiReasonAndRecovery(void)
{
    FaultMgr mgr;
    FaultMgrConfig config = TestConfig();
    FaultDeviceStatus device;
    FaultDomainStatus domain;

    if (!TestCheck(FaultMgrInit(&mgr, &config) == FaultMgrResultOk, "初始化失败")) return 0;
    (void)FaultMgrSetDeviceFault(&mgr, 4u, REASON_OFFLINE, 1u, 100u);
    (void)FaultMgrSetDeviceFault(&mgr, 4u, REASON_DATA, 1u, 120u);
    (void)FaultMgrSetDeviceFault(&mgr, 4u, REASON_OFFLINE, 0u, 130u);

    if (!TestCheck(FaultMgrGetDeviceStatus(&mgr, 4u, &device) == FaultMgrResultOk,
                   "设备状态查询失败")) return 0;
    if (!TestCheck(device.activeReasonMask == REASON_DATA &&
                   device.blockingReasonMask == (REASON_OFFLINE | REASON_DATA),
                   "逐位清除不应误清其它原因")) return 0;
    if (!TestCheck(device.firstFaultMs == 100u && device.lastFaultMs == 120u,
                   "首次和最近故障时间应跨多原因保留")) return 0;

    (void)FaultMgrSetDeviceFault(&mgr, 4u, REASON_DATA, 0u, 140u);
    (void)FaultMgrUpdate(&mgr, 239u, 1u << 4, 1u);
    if (!TestCheck(FaultMgrDeviceAction(&mgr, 4u) == FaultActionStopDomain,
                   "稳定时间不足时域应保持停止")) return 0;
    (void)FaultMgrUpdate(&mgr, 240u, 0u, 1u);
    if (!TestCheck(FaultMgrDeviceAction(&mgr, 4u) == FaultActionStopDomain,
                   "需要安全输入时不得恢复设备")) return 0;
    (void)FaultMgrUpdate(&mgr, 240u, 1u << 4, 1u);
    if (!TestCheck(FaultMgrDomainAction(&mgr, 0u) == FaultActionStopDomain,
                   "设备恢复后还应等待域稳定时间")) return 0;

    (void)FaultMgrUpdate(&mgr, 289u, 1u << 4, 1u);
    if (!TestCheck(FaultMgrDomainAction(&mgr, 0u) == FaultActionStopDomain,
                   "域稳定时间不足时不得恢复")) return 0;
    (void)FaultMgrUpdate(&mgr, 290u, 1u << 4, 0u);
    if (!TestCheck(FaultMgrDomainAction(&mgr, 0u) == FaultActionStopDomain,
                   "域恢复需要安全输入")) return 0;
    (void)FaultMgrUpdate(&mgr, 290u, 1u << 4, 1u);
    if (!TestCheck(FaultMgrDomainAction(&mgr, 0u) == FaultActionRun,
                   "满足稳定时间和安全输入后应恢复域")) return 0;

    if (!TestCheck(FaultMgrGetDeviceStatus(&mgr, 4u, &device) == FaultMgrResultOk,
                   "恢复后设备状态查询失败")) return 0;
    if (!TestCheck(device.blockingReasonMask == 0u &&
                   device.historyReasonMask == (REASON_OFFLINE | REASON_DATA) &&
                   device.everFaulted != 0u,
                   "恢复后应清阻断原因并保留历史")) return 0;
    if (!TestCheck(FaultMgrGetDomainStatus(&mgr, 0u, &domain) == FaultMgrResultOk &&
                   domain.blockingReasonMask == 0u &&
                   domain.historyReasonMask == (REASON_OFFLINE | REASON_DATA),
                   "域恢复后应保留历史原因")) return 0;
    return 1;
}

static int TestTickWrap(void)
{
    FaultMgr mgr;
    FaultMgrConfig config = TestConfig();

    config.device[0].stableMs = 32u;
    if (!TestCheck(FaultMgrInit(&mgr, &config) == FaultMgrResultOk, "初始化失败")) return 0;
    (void)FaultMgrSetDeviceFault(&mgr, 0u, REASON_OFFLINE, 1u, 0xFFFFFFE0u);
    (void)FaultMgrSetDeviceFault(&mgr, 0u, REASON_OFFLINE, 0u, 0xFFFFFFF0u);
    (void)FaultMgrUpdate(&mgr, 0x0000000Fu, 0u, 1u);
    if (!TestCheck(FaultMgrDeviceAction(&mgr, 0u) == FaultActionStopDomain,
                   "回绕后31ms不应提前恢复")) return 0;
    (void)FaultMgrUpdate(&mgr, 0x00000010u, 0u, 1u);
    if (!TestCheck(FaultMgrDeviceAction(&mgr, 0u) == FaultActionStopDomain,
                   "设备恢复后域仍应执行自己的稳定门槛")) return 0;
    return 1;
}

static int TestShootAndArmLocalPolicies(void)
{
    FaultMgr shoot;
    FaultMgr arm;
    FaultMgrConfig config;

    (void)memset(&config, 0, sizeof(config));
    config.deviceCount = 5u;
    for (uint8_t i = 0u; i < config.deviceCount; i++)
    {
        config.device[i].stableMs = 200u;
        config.device[i].requireSafeInput = 1u;
    }
    if (!TestCheck(FaultMgrInit(&shoot, &config) == FaultMgrResultOk,
                   "射击局部策略初始化失败")) return 0;

    (void)FaultMgrSetDeviceFault(&shoot, 0u, REASON_OFFLINE, 1u, 10u);
    if (!TestCheck(FaultMgrDeviceAction(&shoot, 0u) == FaultActionIsolateDevice &&
                   FaultMgrDeviceAction(&shoot, 1u) == FaultActionRun,
                   "拨弹故障应只隔离拨弹，摩擦轮继续运行")) return 0;
    (void)FaultMgrSetDeviceFault(&shoot, 0u, REASON_OFFLINE, 0u, 20u);
    (void)FaultMgrUpdate(&shoot, 219u, 1u, 0u);
    if (!TestCheck(FaultMgrDeviceAction(&shoot, 0u) == FaultActionIsolateDevice,
                   "拨弹健康不足200ms不得恢复")) return 0;
    (void)FaultMgrUpdate(&shoot, 220u, 0u, 0u);
    if (!TestCheck(FaultMgrDeviceAction(&shoot, 0u) == FaultActionIsolateDevice,
                   "拨弹恢复必须回停火安全位")) return 0;
    (void)FaultMgrUpdate(&shoot, 220u, 1u, 0u);
    if (!TestCheck(FaultMgrDeviceAction(&shoot, 0u) == FaultActionRun,
                   "拨弹满足稳定时间和停火位后应恢复")) return 0;

    (void)FaultMgrSetDeviceFault(&shoot, 1u, REASON_OFFLINE, 1u, 300u);
    if (!TestCheck(FaultMgrDeviceAction(&shoot, 1u) == FaultActionIsolateDevice &&
                   FaultMgrDeviceAction(&shoot, 0u) == FaultActionRun &&
                   FaultMgrDeviceAction(&shoot, 2u) == FaultActionRun,
                   "单个摩擦轮故障应只隔离自身")) return 0;
    (void)FaultMgrSetDeviceFault(&shoot, 1u, REASON_OFFLINE, 0u, 310u);
    (void)FaultMgrUpdate(&shoot, 509u, 1u << 1, 0u);
    if (!TestCheck(FaultMgrDeviceAction(&shoot, 1u) == FaultActionIsolateDevice,
                   "摩擦轮健康不足200ms不得恢复")) return 0;
    (void)FaultMgrUpdate(&shoot, 510u, 0u, 0u);
    if (!TestCheck(FaultMgrDeviceAction(&shoot, 1u) == FaultActionIsolateDevice,
                   "摩擦轮恢复必须回停火安全位")) return 0;
    (void)FaultMgrUpdate(&shoot, 510u, 1u << 1, 0u);
    if (!TestCheck(FaultMgrDeviceAction(&shoot, 1u) == FaultActionRun &&
                   FaultMgrDeviceAction(&shoot, 2u) == FaultActionRun,
                   "摩擦轮应逐轴恢复")) return 0;

    (void)memset(&config, 0, sizeof(config));
    config.deviceCount = 6u;
    for (uint8_t i = 0u; i < config.deviceCount; i++)
    {
        config.device[i].stableMs = 200u;
        config.device[i].requireSafeInput = 1u;
    }
    if (!TestCheck(FaultMgrInit(&arm, &config) == FaultMgrResultOk,
                   "机械臂逐轴策略初始化失败")) return 0;
    (void)FaultMgrSetDeviceFault(&arm, 2u, REASON_DATA, 1u, 1000u);
    if (!TestCheck(FaultMgrDeviceAction(&arm, 2u) == FaultActionIsolateDevice &&
                   FaultMgrDeviceAction(&arm, 1u) == FaultActionRun &&
                   FaultMgrDeviceAction(&arm, 3u) == FaultActionRun,
                   "机械臂单关节故障不得拖停相邻关节")) return 0;
    (void)FaultMgrSetDeviceFault(&arm, 2u, REASON_DATA, 0u, 1010u);
    (void)FaultMgrUpdate(&arm, 1210u, 0u, 0u);
    if (!TestCheck(FaultMgrDeviceAction(&arm, 2u) == FaultActionIsolateDevice,
                   "机械臂关节按键未释放时不得恢复")) return 0;
    (void)FaultMgrUpdate(&arm, 1210u, 1u << 2, 0u);
    return TestCheck(FaultMgrDeviceAction(&arm, 2u) == FaultActionRun &&
                         FaultMgrDeviceAction(&arm, 1u) == FaultActionRun,
                     "机械臂应只恢复满足门槛的关节");
}

int main(void)
{
    if (!TestInvalidAndConservative()) return 1;
    if (!TestIsolationAndStrongDomain()) return 1;
    if (!TestMultiReasonAndRecovery()) return 1;
    if (!TestTickWrap()) return 1;
    if (!TestShootAndArmLocalPolicies()) return 1;

    printf("PASS: FaultMgr host regression (sizeof(FaultMgr)=%u)\n", (unsigned)sizeof(FaultMgr));
    return 0;
}
