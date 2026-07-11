/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include "FaultMgr.h"

#include <stddef.h>
#include <string.h>

static uint32_t FaultMgrDeviceLimitMask(uint8_t count)
{
    if (count >= 32u)
    {
        return FAULT_MGR_REASON_ALL;
    }
    return (1u << count) - 1u;
}

static uint8_t FaultMgrRecordBlocked(const FaultMgrRecord *record)
{
    return (uint8_t)(record->incidentReasonMask != 0u);
}

static uint8_t FaultMgrRecordRecovering(const FaultMgrRecord *record)
{
    return (uint8_t)(record->incidentReasonMask != 0u && record->activeReasonMask == 0u);
}

static void FaultMgrRecordRaise(FaultMgrRecord *record, uint32_t reasonMask, uint32_t nowMs)
{
    const uint32_t newlyActiveReasons = reasonMask & ~record->activeReasonMask;

    if (record->historyReasonMask == 0u)
    {
        record->firstFaultMs = nowMs;
    }

    record->activeReasonMask |= reasonMask;
    record->incidentReasonMask |= reasonMask;
    record->historyReasonMask |= reasonMask;
    /* 周期性健康检查会重复上报同一故障；时间只记录故障边沿或新增原因。 */
    if (newlyActiveReasons != 0u)
    {
        record->lastFaultMs = nowMs;
    }
    record->healthySinceMs = 0u;
}

static void FaultMgrRecordClear(FaultMgrRecord *record, uint32_t reasonMask, uint32_t nowMs)
{
    const uint32_t activeBefore = record->activeReasonMask;

    record->activeReasonMask &= ~reasonMask;
    if (activeBefore != 0u && record->activeReasonMask == 0u && record->incidentReasonMask != 0u)
    {
        record->healthySinceMs = nowMs;
    }
}

static uint8_t FaultMgrRecoveryReady(const FaultMgrRecord *record,
                                     uint32_t stableMs,
                                     uint8_t requireSafe,
                                     uint8_t safe,
                                     uint32_t nowMs)
{
    if (FaultMgrRecordRecovering(record) == 0u)
    {
        return 0u;
    }
    if ((uint32_t)(nowMs - record->healthySinceMs) < stableMs)
    {
        return 0u;
    }
    if (requireSafe != 0u && safe == 0u)
    {
        return 0u;
    }
    return 1u;
}

static void FaultMgrRecordRecover(FaultMgrRecord *record)
{
    record->incidentReasonMask = 0u;
}

static uint32_t FaultMgrDomainActiveMemberMask(const FaultMgr *mgr, uint8_t domainId)
{
    uint32_t mask = 0u;
    const uint32_t criticalMask = mgr->domainCriticalMask[domainId];

    for (uint8_t i = 0u; i < mgr->deviceCount; i++)
    {
        const uint32_t bit = 1u << i;
        if ((criticalMask & bit) != 0u && mgr->device[i].activeReasonMask != 0u)
        {
            mask |= bit;
        }
    }
    return mask;
}

static uint32_t FaultMgrDomainBlockingMemberMask(const FaultMgr *mgr, uint8_t domainId)
{
    uint32_t mask = 0u;
    const uint32_t criticalMask = mgr->domainCriticalMask[domainId];

    for (uint8_t i = 0u; i < mgr->deviceCount; i++)
    {
        const uint32_t bit = 1u << i;
        if ((criticalMask & bit) != 0u && FaultMgrRecordBlocked(&mgr->device[i]) != 0u)
        {
            mask |= bit;
        }
    }
    return mask;
}

static uint32_t FaultMgrDomainActiveReasons(const FaultMgr *mgr, uint8_t domainId)
{
    uint32_t reasons = 0u;
    const uint32_t criticalMask = mgr->domainCriticalMask[domainId];

    for (uint8_t i = 0u; i < mgr->deviceCount; i++)
    {
        if ((criticalMask & (1u << i)) != 0u)
        {
            reasons |= mgr->device[i].activeReasonMask;
        }
    }
    return reasons;
}

static void FaultMgrNoteDomainFault(FaultMgr *mgr, uint8_t domainId, uint32_t reasonMask, uint32_t nowMs)
{
    FaultMgrRecord *record = &mgr->domain[domainId];

    if (record->historyReasonMask == 0u)
    {
        record->firstFaultMs = nowMs;
    }
    record->incidentReasonMask |= reasonMask;
    record->historyReasonMask |= reasonMask;
    record->lastFaultMs = nowMs;
    record->healthySinceMs = 0u;
    mgr->domainRecoveryStartedMask &= ~(1u << domainId);
}

static FaultMgrResult FaultMgrCheckReady(const FaultMgr *mgr)
{
    if (mgr == NULL)
    {
        return FaultMgrResultInvalidArg;
    }
    if (mgr->initialized == 0u)
    {
        return FaultMgrResultNotReady;
    }
    return FaultMgrResultOk;
}

FaultMgrResult FaultMgrInit(FaultMgr *mgr, const FaultMgrConfig *config)
{
    uint32_t usedMemberMask = 0u;
    uint32_t deviceLimitMask;

    if (mgr == NULL || config == NULL)
    {
        return FaultMgrResultInvalidArg;
    }

    (void)memset(mgr, 0, sizeof(*mgr));
    if (config->deviceCount == 0u || config->deviceCount > FAULT_MGR_DEVICE_MAX ||
        config->domainCount > FAULT_MGR_DOMAIN_MAX)
    {
        return FaultMgrResultInvalidConfig;
    }

    deviceLimitMask = FaultMgrDeviceLimitMask(config->deviceCount);
    for (uint8_t i = 0u; i < config->deviceCount; i++)
    {
        if (config->device[i].requireSafeInput > 1u)
        {
            return FaultMgrResultInvalidConfig;
        }
    }

    for (uint8_t i = 0u; i < config->domainCount; i++)
    {
        const FaultDomainConfig *domain = &config->domain[i];
        if (domain->memberMask == 0u || domain->criticalMask == 0u ||
            (domain->memberMask & ~deviceLimitMask) != 0u ||
            (domain->criticalMask & ~domain->memberMask) != 0u ||
            (domain->memberMask & usedMemberMask) != 0u || domain->recovery.requireSafeInput > 1u)
        {
            return FaultMgrResultInvalidConfig;
        }
        usedMemberMask |= domain->memberMask;
    }

    mgr->deviceCount = config->deviceCount;
    mgr->domainCount = config->domainCount;
    for (uint8_t i = 0u; i < FAULT_MGR_DEVICE_MAX; i++)
    {
        mgr->deviceDomain[i] = FAULT_MGR_DOMAIN_NONE;
    }
    for (uint8_t i = 0u; i < config->deviceCount; i++)
    {
        mgr->deviceStableMs[i] = config->device[i].stableMs;
        if (config->device[i].requireSafeInput != 0u)
        {
            mgr->deviceRequireSafeMask |= 1u << i;
        }
    }
    for (uint8_t domainId = 0u; domainId < config->domainCount; domainId++)
    {
        const FaultDomainConfig *domain = &config->domain[domainId];
        mgr->domainMemberMask[domainId] = domain->memberMask;
        mgr->domainCriticalMask[domainId] = domain->criticalMask;
        mgr->domainStableMs[domainId] = domain->recovery.stableMs;
        if (domain->recovery.requireSafeInput != 0u)
        {
            mgr->domainRequireSafeMask |= 1u << domainId;
        }
        for (uint8_t deviceId = 0u; deviceId < config->deviceCount; deviceId++)
        {
            if ((domain->memberMask & (1u << deviceId)) != 0u)
            {
                mgr->deviceDomain[deviceId] = domainId;
            }
        }
    }

    mgr->initialized = 1u;
    return FaultMgrResultOk;
}

FaultMgrResult FaultMgrSetDeviceFault(FaultMgr *mgr,
                                      uint8_t deviceId,
                                      uint32_t reasonMask,
                                      uint8_t active,
                                      uint32_t nowMs)
{
    FaultMgrResult result = FaultMgrCheckReady(mgr);
    uint8_t domainId;
    uint32_t newlyActiveReasons;

    if (result != FaultMgrResultOk)
    {
        return result;
    }
    if (deviceId >= mgr->deviceCount || reasonMask == 0u || active > 1u)
    {
        return FaultMgrResultInvalidArg;
    }

    domainId = mgr->deviceDomain[deviceId];
    if (active != 0u)
    {
        newlyActiveReasons = reasonMask & ~mgr->device[deviceId].activeReasonMask;
        FaultMgrRecordRaise(&mgr->device[deviceId], reasonMask, nowMs);
        if (newlyActiveReasons != 0u && domainId != FAULT_MGR_DOMAIN_NONE &&
            (mgr->domainCriticalMask[domainId] & (1u << deviceId)) != 0u)
        {
            FaultMgrNoteDomainFault(mgr, domainId, newlyActiveReasons, nowMs);
        }
    }
    else
    {
        FaultMgrRecordClear(&mgr->device[deviceId], reasonMask, nowMs);
    }
    return FaultMgrResultOk;
}

FaultMgrResult FaultMgrUpdate(FaultMgr *mgr,
                              uint32_t nowMs,
                              uint32_t deviceSafeMask,
                              uint32_t domainSafeMask)
{
    FaultMgrResult result = FaultMgrCheckReady(mgr);

    if (result != FaultMgrResultOk)
    {
        return result;
    }
    for (uint8_t deviceId = 0u; deviceId < mgr->deviceCount; deviceId++)
    {
        const uint8_t requireSafe = (uint8_t)((mgr->deviceRequireSafeMask >> deviceId) & 1u);
        const uint8_t safe = (uint8_t)((deviceSafeMask >> deviceId) & 1u);
        if (FaultMgrRecoveryReady(&mgr->device[deviceId],
                                  mgr->deviceStableMs[deviceId],
                                  requireSafe,
                                  safe,
                                  nowMs) != 0u)
        {
            FaultMgrRecordRecover(&mgr->device[deviceId]);
        }
    }

    for (uint8_t domainId = 0u; domainId < mgr->domainCount; domainId++)
    {
        FaultMgrRecord *record = &mgr->domain[domainId];
        const uint32_t blockingMembers = FaultMgrDomainBlockingMemberMask(mgr, domainId);

        record->activeReasonMask = FaultMgrDomainActiveReasons(mgr, domainId);
        if (blockingMembers != 0u)
        {
            record->healthySinceMs = 0u;
            mgr->domainRecoveryStartedMask &= ~(1u << domainId);
            continue;
        }
        if (record->incidentReasonMask != 0u && record->activeReasonMask == 0u &&
            (mgr->domainRecoveryStartedMask & (1u << domainId)) == 0u)
        {
            record->healthySinceMs = nowMs;
            mgr->domainRecoveryStartedMask |= 1u << domainId;
        }

        const uint8_t requireSafe = (uint8_t)((mgr->domainRequireSafeMask >> domainId) & 1u);
        const uint8_t safe = (uint8_t)((domainSafeMask >> domainId) & 1u);
        if (FaultMgrRecoveryReady(record, mgr->domainStableMs[domainId], requireSafe, safe, nowMs) != 0u)
        {
            FaultMgrRecordRecover(record);
            mgr->domainRecoveryStartedMask &= ~(1u << domainId);
        }
    }

    return FaultMgrResultOk;
}

FaultAction FaultMgrDomainAction(const FaultMgr *mgr, uint8_t domainId)
{
    if (FaultMgrCheckReady(mgr) != FaultMgrResultOk || domainId >= mgr->domainCount)
    {
        return FaultActionStopDomain;
    }
    return (FaultMgrRecordBlocked(&mgr->domain[domainId]) != 0u) ? FaultActionStopDomain : FaultActionRun;
}

FaultAction FaultMgrDeviceAction(const FaultMgr *mgr, uint8_t deviceId)
{
    uint8_t domainId;

    if (FaultMgrCheckReady(mgr) != FaultMgrResultOk || deviceId >= mgr->deviceCount)
    {
        return FaultActionStopDomain;
    }

    domainId = mgr->deviceDomain[deviceId];
    if (domainId != FAULT_MGR_DOMAIN_NONE && FaultMgrDomainAction(mgr, domainId) == FaultActionStopDomain)
    {
        return FaultActionStopDomain;
    }
    return (FaultMgrRecordBlocked(&mgr->device[deviceId]) != 0u) ? FaultActionIsolateDevice : FaultActionRun;
}

FaultMgrResult FaultMgrGetDeviceStatus(const FaultMgr *mgr,
                                       uint8_t deviceId,
                                       FaultDeviceStatus *status)
{
    FaultMgrResult result = FaultMgrCheckReady(mgr);
    const FaultMgrRecord *record;

    if (status == NULL)
    {
        return FaultMgrResultInvalidArg;
    }
    (void)memset(status, 0, sizeof(*status));
    if (result != FaultMgrResultOk)
    {
        status->action = FaultActionStopDomain;
        return result;
    }
    if (deviceId >= mgr->deviceCount)
    {
        status->action = FaultActionStopDomain;
        return FaultMgrResultInvalidArg;
    }

    record = &mgr->device[deviceId];
    status->action = FaultMgrDeviceAction(mgr, deviceId);
    status->activeReasonMask = record->activeReasonMask;
    status->blockingReasonMask = record->incidentReasonMask;
    status->historyReasonMask = record->historyReasonMask;
    status->firstFaultMs = record->firstFaultMs;
    status->lastFaultMs = record->lastFaultMs;
    status->healthySinceMs = record->healthySinceMs;
    status->domainId = mgr->deviceDomain[deviceId];
    status->faultActive = (uint8_t)(record->activeReasonMask != 0u);
    status->recoveryPending = FaultMgrRecordRecovering(record);
    status->everFaulted = (uint8_t)(record->historyReasonMask != 0u);
    return FaultMgrResultOk;
}

FaultMgrResult FaultMgrGetDomainStatus(const FaultMgr *mgr,
                                       uint8_t domainId,
                                       FaultDomainStatus *status)
{
    FaultMgrResult result = FaultMgrCheckReady(mgr);
    const FaultMgrRecord *record;

    if (status == NULL)
    {
        return FaultMgrResultInvalidArg;
    }
    (void)memset(status, 0, sizeof(*status));
    if (result != FaultMgrResultOk)
    {
        status->action = FaultActionStopDomain;
        return result;
    }
    if (domainId >= mgr->domainCount)
    {
        status->action = FaultActionStopDomain;
        return FaultMgrResultInvalidArg;
    }

    record = &mgr->domain[domainId];
    status->action = FaultMgrDomainAction(mgr, domainId);
    status->memberMask = mgr->domainMemberMask[domainId];
    status->criticalMask = mgr->domainCriticalMask[domainId];
    status->activeMemberMask = FaultMgrDomainActiveMemberMask(mgr, domainId);
    status->blockingMemberMask = FaultMgrDomainBlockingMemberMask(mgr, domainId);
    status->activeReasonMask = FaultMgrDomainActiveReasons(mgr, domainId);
    status->blockingReasonMask = record->incidentReasonMask;
    status->historyReasonMask = record->historyReasonMask;
    status->firstFaultMs = record->firstFaultMs;
    status->lastFaultMs = record->lastFaultMs;
    status->healthySinceMs = record->healthySinceMs;
    status->faultActive = (uint8_t)(status->activeMemberMask != 0u);
    status->recoveryPending = FaultMgrRecordRecovering(record);
    status->everFaulted = (uint8_t)(record->historyReasonMask != 0u);
    return FaultMgrResultOk;
}
