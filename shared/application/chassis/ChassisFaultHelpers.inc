/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Private implementation chunk included by ChassisControlTask.c.
 */

#define CHASSIS_FAULT_REASON_MOTOR (1u << 0)

typedef struct
{
    FaultMgr mgr;
    uint32_t configuredMask;
    uint32_t activeMask;
    uint32_t blockingMask;
    uint32_t recoveryMask;
    uint32_t inhibitMask;
    uint32_t holdZeroMask;
    uint16_t reasonMask[CHASSIS_MOTOR_COUNT];
    uint32_t feedbackAgeMs[CHASSIS_MOTOR_COUNT];
    uint32_t inhibitFailCount;
    uint32_t releaseFailCount;
    uint8_t initialized;
    uint8_t recoveryInputSafe;
} ChassisFaultState;

static ChassisFaultState s_chassisFault;

static uint32_t ChassisFaultConfiguredMask(const ChassisRuntimeSnapshot *snapshot)
{
    uint32_t mask = 0u;

    for (uint8_t i = 0u; i < CHASSIS_MOTOR_COUNT; i++)
    {
        const motor_node_param_t *node = (snapshot != NULL) ?
                                             snapshot->motor_cfg[i] :
                                             &g_config.motor.chassis[i];

        if (MotorCfgNodeId(node) != 0u)
        {
            mask |= 1u << i;
        }
    }
    return mask;
}

static uint8_t ChassisFaultRecoveryInputSafe(const ChassisRuntimeSnapshot *snapshot)
{
    return (snapshot != NULL && snapshot->recovery_input_safe != 0u) ? 1u : 0u;
}

static void ChassisFaultInit(void)
{
    FaultMgrConfig config;

    (void)memset(&s_chassisFault, 0, sizeof(s_chassisFault));
    (void)memset(&config, 0, sizeof(config));
    s_chassisFault.configuredMask = ChassisFaultConfiguredMask(NULL);
    config.deviceCount = CHASSIS_MOTOR_COUNT;
    for (uint8_t i = 0u; i < CHASSIS_MOTOR_COUNT; i++)
    {
        config.device[i].stableMs = CHASSIS_FAULT_RECOVERY_MS;
        config.device[i].requireSafeInput = 1u;
        s_chassisFault.feedbackAgeMs[i] = MOTOR_HEALTH_AGE_UNKNOWN;
    }
    s_chassisFault.initialized =
        (FaultMgrInit(&s_chassisFault.mgr, &config) == FaultMgrResultOk) ? 1u : 0u;
    if (s_chassisFault.initialized == 0u)
    {
        s_chassisFault.blockingMask = s_chassisFault.configuredMask;
    }
}

static void ChassisFaultUpdate(const ChassisRuntimeSnapshot *snapshot)
{
    uint32_t nowMs;
    uint32_t safeMask = 0u;
    uint8_t updateOk = 1u;

    s_chassisFault.configuredMask = ChassisFaultConfiguredMask(snapshot);
    s_chassisFault.activeMask = 0u;
    s_chassisFault.blockingMask = 0u;
    s_chassisFault.recoveryMask = 0u;
    s_chassisFault.recoveryInputSafe = ChassisFaultRecoveryInputSafe(snapshot);

    if (s_chassisFault.initialized == 0u || snapshot == NULL)
    {
        s_chassisFault.blockingMask = s_chassisFault.configuredMask;
        return;
    }

    nowMs = snapshot->tick_ms;

    for (uint8_t i = 0u; i < CHASSIS_MOTOR_COUNT; i++)
    {
        const uint32_t bit = 1u << i;
        const uint8_t configured = ((s_chassisFault.configuredMask & bit) != 0u) ? 1u : 0u;
        const uint8_t motorFault = (uint8_t)(configured != 0u &&
                                             snapshot->motor_health[i].healthy == 0u);

        s_chassisFault.reasonMask[i] = snapshot->motor_health[i].reasonMask;
        s_chassisFault.feedbackAgeMs[i] = snapshot->motor_health[i].ageMs;
        if (motorFault != 0u)
        {
            s_chassisFault.activeMask |= bit;
        }
        if (FaultMgrSetDeviceFault(&s_chassisFault.mgr,
                                   i,
                                   CHASSIS_FAULT_REASON_MOTOR,
                                   motorFault,
                                   nowMs) != FaultMgrResultOk)
        {
            updateOk = 0u;
        }
    }

    if (s_chassisFault.recoveryInputSafe != 0u)
    {
        safeMask = s_chassisFault.configuredMask;
    }
    if (updateOk == 0u ||
        FaultMgrUpdate(&s_chassisFault.mgr, nowMs, safeMask, 0u, 1u) != FaultMgrResultOk)
    {
        s_chassisFault.blockingMask = s_chassisFault.configuredMask;
        return;
    }

    for (uint8_t i = 0u; i < CHASSIS_MOTOR_COUNT; i++)
    {
        const uint32_t bit = 1u << i;
        FaultDeviceStatus status;

        if ((s_chassisFault.configuredMask & bit) == 0u)
        {
            continue;
        }
        if (FaultMgrGetDeviceStatus(&s_chassisFault.mgr, i, &status) != FaultMgrResultOk)
        {
            s_chassisFault.blockingMask |= bit;
            continue;
        }
        if (status.action != FaultActionRun)
        {
            s_chassisFault.blockingMask |= bit;
        }
        if (status.recoveryPending != 0u)
        {
            s_chassisFault.recoveryMask |= bit;
        }
    }
}

static uint8_t ChassisFaultCollectIds(uint32_t mask,
                                      MotorId out[CHASSIS_MOTOR_COUNT],
                                      uint32_t *collectedMask)
{
    uint32_t actualMask = 0u;
    uint8_t count = 0u;

    for (uint8_t i = 0u; i < CHASSIS_MOTOR_COUNT; i++)
    {
        const uint32_t bit = 1u << i;
        const MotorId id = ChassisMotorCurrentBindings[i].actuator_id;

        if ((mask & bit) == 0u ||
            (uint32_t)id >= (uint32_t)MotorCount)
        {
            continue;
        }
        out[count++] = id;
        actualMask |= bit;
    }
    if (collectedMask != NULL)
    {
        *collectedMask = actualMask;
    }
    return count;
}

static void ChassisFaultResetControl(ChassisMove *control, uint32_t mask)
{
    if (control == NULL)
    {
        return;
    }

    for (uint8_t i = 0u; i < CHASSIS_MOTOR_COUNT; i++)
    {
        if ((mask & (1u << i)) == 0u)
        {
            continue;
        }
        PID_clear(&control->motor_speed_pid[i]);
        control->motor_chassis[i].speed_set = 0.0f;
        control->motor_chassis[i].give_current = 0;
    }
}

static void ChassisFaultSyncInhibit(ChassisMove *control)
{
    const MotorAxisFaultInhibitPlan plan =
        MotorAxisFaultInhibitPlanMake(s_chassisFault.configuredMask,
                                      s_chassisFault.blockingMask,
                                      s_chassisFault.inhibitMask);
    MotorId ids[CHASSIS_MOTOR_COUNT];
    uint32_t actualMask = 0u;
    uint8_t count;

    s_chassisFault.holdZeroMask = plan.holdZeroMask;

    count = ChassisFaultCollectIds(plan.acquireMask, ids, &actualMask);
    if (actualMask != plan.acquireMask)
    {
        s_chassisFault.inhibitFailCount++;
    }
    if (count != 0u)
    {
        if (LowCmdInhibitManyFrom(ids, count, (uint16_t)LOWCMD_WRITER_SAFETY) != 0u)
        {
            s_chassisFault.inhibitMask |= actualMask;
        }
        else
        {
            uint32_t acquiredMask = 0u;

            for (uint8_t i = 0u; i < CHASSIS_MOTOR_COUNT; i++)
            {
                const uint32_t bit = 1u << i;
                const MotorId id = ChassisMotorCurrentBindings[i].actuator_id;

                if ((actualMask & bit) != 0u &&
                    LowCmdInhibitManyFrom(&id, 1u, (uint16_t)LOWCMD_WRITER_SAFETY) != 0u)
                {
                    acquiredMask |= bit;
                }
            }
            s_chassisFault.inhibitMask |= acquiredMask;
            if (acquiredMask != actualMask)
            {
                s_chassisFault.inhibitFailCount++;
            }
        }
    }

    count = ChassisFaultCollectIds(plan.releaseMask, ids, &actualMask);
    if (actualMask != plan.releaseMask)
    {
        s_chassisFault.releaseFailCount++;
    }
    if (count == 0u)
    {
        return;
    }

    ChassisFaultResetControl(control, actualMask);
    if (LowCmdClearManyFrom(ids, count, (uint16_t)LOWCMD_WRITER_SAFETY) != 0u &&
        LowCmdReleaseInhibitManyFrom(ids, count, (uint16_t)LOWCMD_WRITER_SAFETY) != 0u)
    {
        s_chassisFault.inhibitMask &= ~actualMask;
        return;
    }

    {
        uint32_t releasedMask = 0u;

        for (uint8_t i = 0u; i < CHASSIS_MOTOR_COUNT; i++)
        {
            const uint32_t bit = 1u << i;
            const MotorId id = ChassisMotorCurrentBindings[i].actuator_id;

            if ((actualMask & bit) != 0u &&
                LowCmdClearManyFrom(&id, 1u, (uint16_t)LOWCMD_WRITER_SAFETY) != 0u &&
                LowCmdReleaseInhibitManyFrom(&id, 1u, (uint16_t)LOWCMD_WRITER_SAFETY) != 0u)
            {
                releasedMask |= bit;
            }
        }
        s_chassisFault.inhibitMask &= ~releasedMask;
        if (releasedMask != actualMask)
        {
            s_chassisFault.releaseFailCount++;
        }
    }
}

static void ChassisFaultApplyControl(ChassisMove *control)
{
    ChassisFaultResetControl(control, s_chassisFault.holdZeroMask);
}
