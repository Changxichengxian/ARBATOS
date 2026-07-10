/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef FAULT_MGR_H
#define FAULT_MGR_H

#include <stdint.h>

/*
 * 默认容量覆盖当前最大的轮腿故障域（六电机、IMU、运行保护）。
 * 每个控制任务持有独立实例，保持无锁；目标确有更大域时可全工程统一覆盖。
 */
#ifndef FAULT_MGR_DEVICE_MAX
#define FAULT_MGR_DEVICE_MAX 8u
#endif
#ifndef FAULT_MGR_DOMAIN_MAX
#define FAULT_MGR_DOMAIN_MAX 2u
#endif
#if FAULT_MGR_DEVICE_MAX == 0u || FAULT_MGR_DEVICE_MAX > 32u
#error "FAULT_MGR_DEVICE_MAX must be in [1, 32]"
#endif
#if FAULT_MGR_DOMAIN_MAX == 0u || FAULT_MGR_DOMAIN_MAX > 32u
#error "FAULT_MGR_DOMAIN_MAX must be in [1, 32]"
#endif
#define FAULT_MGR_DOMAIN_NONE 0xFFu
#define FAULT_MGR_REASON_ALL 0xFFFFFFFFu

typedef enum
{
    FaultActionRun = 0u,
    FaultActionIsolateDevice,
    FaultActionStopDomain,
    FaultActionStopGlobal,
} FaultAction;

typedef enum
{
    FaultMgrResultOk = 0u,
    FaultMgrResultInvalidArg,
    FaultMgrResultInvalidConfig,
    FaultMgrResultNotReady,
} FaultMgrResult;

typedef struct
{
    uint32_t stableMs;
    uint8_t requireSafeInput;
} FaultRecoveryRule;

typedef struct
{
    uint32_t memberMask;
    uint32_t criticalMask;
    FaultRecoveryRule recovery;
} FaultDomainConfig;

/* firstFaultMs/lastFaultMs 记录故障边沿；周期性重复上报同一活动原因不会刷新 lastFaultMs。 */

typedef struct
{
    uint8_t deviceCount;
    uint8_t domainCount;
    FaultRecoveryRule device[FAULT_MGR_DEVICE_MAX];
    FaultDomainConfig domain[FAULT_MGR_DOMAIN_MAX];
    FaultRecoveryRule system;
} FaultMgrConfig;

typedef struct
{
    FaultAction action;
    uint32_t activeReasonMask;
    uint32_t blockingReasonMask;
    uint32_t historyReasonMask;
    uint32_t firstFaultMs;
    uint32_t lastFaultMs;
    uint32_t healthySinceMs;
    uint8_t domainId;
    uint8_t faultActive;
    uint8_t recoveryPending;
    uint8_t everFaulted;
} FaultDeviceStatus;

typedef struct
{
    FaultAction action;
    uint32_t memberMask;
    uint32_t criticalMask;
    uint32_t activeMemberMask;
    uint32_t blockingMemberMask;
    uint32_t activeReasonMask;
    uint32_t blockingReasonMask;
    uint32_t historyReasonMask;
    uint32_t firstFaultMs;
    uint32_t lastFaultMs;
    uint32_t healthySinceMs;
    uint8_t faultActive;
    uint8_t recoveryPending;
    uint8_t everFaulted;
} FaultDomainStatus;

typedef struct
{
    FaultAction action;
    uint32_t activeReasonMask;
    uint32_t blockingReasonMask;
    uint32_t historyReasonMask;
    uint32_t firstFaultMs;
    uint32_t lastFaultMs;
    uint32_t healthySinceMs;
    uint8_t faultActive;
    uint8_t recoveryPending;
    uint8_t everFaulted;
} FaultSystemStatus;

/* 内部记录放在结构体里，调用方只需要静态分配 FaultMgr，不应直接读写这些字段。 */
typedef struct
{
    uint32_t activeReasonMask;
    uint32_t incidentReasonMask;
    uint32_t historyReasonMask;
    uint32_t firstFaultMs;
    uint32_t lastFaultMs;
    uint32_t healthySinceMs;
} FaultMgrRecord;

typedef struct
{
    FaultMgrRecord device[FAULT_MGR_DEVICE_MAX];
    FaultMgrRecord domain[FAULT_MGR_DOMAIN_MAX];
    FaultMgrRecord system;
    uint32_t deviceStableMs[FAULT_MGR_DEVICE_MAX];
    uint32_t domainMemberMask[FAULT_MGR_DOMAIN_MAX];
    uint32_t domainCriticalMask[FAULT_MGR_DOMAIN_MAX];
    uint32_t domainStableMs[FAULT_MGR_DOMAIN_MAX];
    uint32_t deviceRequireSafeMask;
    uint32_t domainRequireSafeMask;
    uint32_t domainRecoveryStartedMask;
    uint32_t systemStableMs;
    uint8_t deviceDomain[FAULT_MGR_DEVICE_MAX];
    uint8_t deviceCount;
    uint8_t domainCount;
    uint8_t systemRequireSafe;
    uint8_t initialized;
} FaultMgr;

/*
 * FaultMgr 由一个任务串行调用，不在内部加锁。设备未加入显式故障域时，故障只隔离自身。
 * Update 的安全位与设备/域编号对应；无需安全输入的规则会忽略对应位。
 */
FaultMgrResult FaultMgrInit(FaultMgr *mgr, const FaultMgrConfig *config);
FaultMgrResult FaultMgrSetDeviceFault(FaultMgr *mgr,
                                      uint8_t deviceId,
                                      uint32_t reasonMask,
                                      uint8_t active,
                                      uint32_t nowMs);
FaultMgrResult FaultMgrSetSystemFatal(FaultMgr *mgr,
                                      uint32_t reasonMask,
                                      uint8_t active,
                                      uint32_t nowMs);
FaultMgrResult FaultMgrUpdate(FaultMgr *mgr,
                              uint32_t nowMs,
                              uint32_t deviceSafeMask,
                              uint32_t domainSafeMask,
                              uint8_t systemSafe);

FaultAction FaultMgrDeviceAction(const FaultMgr *mgr, uint8_t deviceId);
FaultAction FaultMgrDomainAction(const FaultMgr *mgr, uint8_t domainId);
FaultAction FaultMgrSystemAction(const FaultMgr *mgr);

FaultMgrResult FaultMgrGetDeviceStatus(const FaultMgr *mgr,
                                       uint8_t deviceId,
                                       FaultDeviceStatus *status);
FaultMgrResult FaultMgrGetDomainStatus(const FaultMgr *mgr,
                                       uint8_t domainId,
                                       FaultDomainStatus *status);
FaultMgrResult FaultMgrGetSystemStatus(const FaultMgr *mgr, FaultSystemStatus *status);

#endif
