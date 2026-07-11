/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Private implementation chunk included by GimbalControlTask.c.
 */

#define GIMBAL_FAULT_REASON_MOTOR (1u << 0)
#define GIMBAL_FAULT_REASON_IMU   (1u << 1)

typedef struct
{
    FaultMgr mgr;
    uint32_t configuredMask;
    uint32_t activeMask;
    uint32_t blockingMask;
    uint32_t recoveryMask;
    uint32_t inhibitMask;
    uint32_t holdZeroMask;
    uint32_t imuRequiredMask;
    uint32_t inhibitFailCount;
    uint32_t releaseFailCount;
    uint8_t initialized;
    uint8_t recoveryInputSafe;
} GimbalFaultState;

static GimbalFaultState s_gimbalFault;

static MotorId GimbalFaultAxisId(uint8_t axis)
{
    switch (axis)
    {
    case GIMBAL_FAULT_AXIS_YAW:
        return s_gimbal_yaw_id;
    case GIMBAL_FAULT_AXIS_YAW_UPPER:
        return s_gimbal_yaw_upper_id;
    case GIMBAL_FAULT_AXIS_PITCH:
        return s_gimbal_pitch_id;
    default:
        return MotorCount;
    }
}

static const MotorHealthResult *GimbalFaultAxisHealth(const GimbalRuntimeSnapshot *snapshot,
                                                       uint8_t axis)
{
    if (snapshot == NULL)
    {
        return NULL;
    }

    switch (axis)
    {
    case GIMBAL_FAULT_AXIS_YAW:
        return &snapshot->yaw_health;
    case GIMBAL_FAULT_AXIS_YAW_UPPER:
        return &snapshot->yaw_upper_health;
    case GIMBAL_FAULT_AXIS_PITCH:
        return &snapshot->pitch_health;
    default:
        return NULL;
    }
}

static uint32_t GimbalFaultConfiguredMask(const GimbalRuntimeSnapshot *snapshot)
{
    uint32_t mask = 0u;

    if (snapshot != NULL)
    {
        if (snapshot->yaw_configured != 0u)
        {
            mask |= GIMBAL_FAULT_MASK_YAW;
        }
        if (snapshot->yaw_upper_configured != 0u)
        {
            mask |= GIMBAL_FAULT_MASK_YAW_UPPER;
        }
        if (snapshot->pitch_configured != 0u)
        {
            mask |= GIMBAL_FAULT_MASK_PITCH;
        }
        return mask;
    }

    if (MotorCfgNodeId(&g_config.motor.yaw) != 0u)
    {
        mask |= GIMBAL_FAULT_MASK_YAW;
    }
    if (MotorCfgNodeId(&g_config.motor.yaw_upper) != 0u)
    {
        mask |= GIMBAL_FAULT_MASK_YAW_UPPER;
    }
    if (MotorCfgNodeId(&g_config.motor.pitch) != 0u)
    {
        mask |= GIMBAL_FAULT_MASK_PITCH;
    }
    return mask;
}

static uint8_t GimbalFaultRecoveryInputSafe(const GimbalRuntimeSnapshot *snapshot)
{
    return (snapshot != NULL && snapshot->recovery_input_safe != 0u) ? 1u : 0u;
}

static void GimbalFaultInit(void)
{
    FaultMgrConfig config;

    (void)memset(&s_gimbalFault, 0, sizeof(s_gimbalFault));
    (void)memset(&config, 0, sizeof(config));
    GimbalMotorIdsEnsure();
    s_gimbalFault.configuredMask = GimbalFaultConfiguredMask(NULL);
    config.deviceCount = GIMBAL_FAULT_AXIS_COUNT;
    for (uint8_t axis = 0u; axis < GIMBAL_FAULT_AXIS_COUNT; axis++)
    {
        config.device[axis].stableMs = GIMBAL_FAULT_RECOVERY_MS;
        config.device[axis].requireSafeInput = 1u;
    }
    s_gimbalFault.initialized =
        (FaultMgrInit(&s_gimbalFault.mgr, &config) == FaultMgrResultOk) ? 1u : 0u;
    if (s_gimbalFault.initialized == 0u)
    {
        s_gimbalFault.blockingMask = s_gimbalFault.configuredMask;
    }
}

static void GimbalFaultUpdate(const GimbalRuntimeSnapshot *snapshot)
{
    const uint32_t nowMs = BspTimeGetTickMs();
    uint32_t safeMask = 0u;
    uint8_t updateOk = 1u;

    s_gimbalFault.configuredMask = GimbalFaultConfiguredMask(snapshot);
    s_gimbalFault.activeMask = 0u;
    s_gimbalFault.blockingMask = 0u;
    s_gimbalFault.recoveryMask = 0u;
    s_gimbalFault.imuRequiredMask = (snapshot != NULL) ? snapshot->imu_required_mask : 0u;
    s_gimbalFault.recoveryInputSafe = GimbalFaultRecoveryInputSafe(snapshot);

    if (s_gimbalFault.initialized == 0u || snapshot == NULL)
    {
        s_gimbalFault.blockingMask = s_gimbalFault.configuredMask;
        return;
    }

    for (uint8_t axis = 0u; axis < GIMBAL_FAULT_AXIS_COUNT; axis++)
    {
        const uint32_t bit = 1u << axis;
        const MotorHealthResult *health = GimbalFaultAxisHealth(snapshot, axis);
        const uint8_t configured = ((s_gimbalFault.configuredMask & bit) != 0u) ? 1u : 0u;
        const uint8_t motorFault = (uint8_t)(configured != 0u &&
                                             (health == NULL || health->healthy == 0u));
        const uint8_t imuFault = (uint8_t)(configured != 0u &&
                                           (s_gimbalFault.imuRequiredMask & bit) != 0u &&
                                           snapshot->imu_online == 0u);

        if (motorFault != 0u || imuFault != 0u)
        {
            s_gimbalFault.activeMask |= bit;
        }
        if (FaultMgrSetDeviceFault(&s_gimbalFault.mgr,
                                   axis,
                                   GIMBAL_FAULT_REASON_MOTOR,
                                   motorFault,
                                   nowMs) != FaultMgrResultOk ||
            FaultMgrSetDeviceFault(&s_gimbalFault.mgr,
                                   axis,
                                   GIMBAL_FAULT_REASON_IMU,
                                   imuFault,
                                   nowMs) != FaultMgrResultOk)
        {
            updateOk = 0u;
        }
    }

    if (s_gimbalFault.recoveryInputSafe != 0u)
    {
        safeMask = s_gimbalFault.configuredMask;
    }
    if (updateOk == 0u ||
        FaultMgrUpdate(&s_gimbalFault.mgr, nowMs, safeMask, 0u, 1u) != FaultMgrResultOk)
    {
        s_gimbalFault.blockingMask = s_gimbalFault.configuredMask;
        return;
    }

    for (uint8_t axis = 0u; axis < GIMBAL_FAULT_AXIS_COUNT; axis++)
    {
        const uint32_t bit = 1u << axis;
        FaultDeviceStatus status;

        if ((s_gimbalFault.configuredMask & bit) == 0u)
        {
            continue;
        }
        if (FaultMgrGetDeviceStatus(&s_gimbalFault.mgr, axis, &status) != FaultMgrResultOk)
        {
            s_gimbalFault.blockingMask |= bit;
            continue;
        }
        if (status.action != FaultActionRun)
        {
            s_gimbalFault.blockingMask |= bit;
        }
        if (status.recoveryPending != 0u)
        {
            s_gimbalFault.recoveryMask |= bit;
        }
    }
}

static uint8_t GimbalFaultCollectIds(uint32_t mask,
                                     MotorId out[GIMBAL_FAULT_AXIS_COUNT],
                                     uint32_t *collectedMask)
{
    uint32_t actualMask = 0u;
    uint8_t count = 0u;

    for (uint8_t axis = 0u; axis < GIMBAL_FAULT_AXIS_COUNT; axis++)
    {
        const uint32_t bit = 1u << axis;
        const MotorId id = GimbalFaultAxisId(axis);

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

static void GimbalFaultResetControl(GimbalControl *control,
                                    const GimbalRuntimeSnapshot *snapshot,
                                    uint32_t mask)
{
    uint32_t yawControlMask;

    if (control == NULL || snapshot == NULL)
    {
        return;
    }
    yawControlMask = (snapshot->yaw_control_is_upper != 0u) ?
                         GIMBAL_FAULT_MASK_YAW_UPPER :
                         GIMBAL_FAULT_MASK_YAW;
    if ((mask & yawControlMask) != 0u)
    {
        GimbalAxisClear(&control->GimbalYawMotor);
    }
    if ((mask & GIMBAL_FAULT_MASK_PITCH) != 0u)
    {
        GimbalAxisClear(&control->GimbalPitchMotor);
    }
}

static void GimbalFaultSyncInhibit(GimbalControl *control,
                                   const GimbalRuntimeSnapshot *snapshot,
                                   const ControlOutputPermit *permit)
{
    const MotorAxisFaultInhibitPlan plan =
        MotorAxisFaultInhibitPlanMake(s_gimbalFault.configuredMask,
                                      s_gimbalFault.blockingMask,
                                      s_gimbalFault.inhibitMask);
    MotorId ids[GIMBAL_FAULT_AXIS_COUNT];
    uint32_t actualMask = 0u;
    uint8_t count;

    s_gimbalFault.holdZeroMask = plan.holdZeroMask;

    count = GimbalFaultCollectIds(plan.acquireMask, ids, &actualMask);
    if (actualMask != plan.acquireMask)
    {
        s_gimbalFault.inhibitFailCount++;
    }
    if (count != 0u)
    {
        if (LowCmdInhibitManyFrom(ids, count, (uint16_t)LOWCMD_WRITER_SAFETY) != 0u)
        {
            s_gimbalFault.inhibitMask |= actualMask;
        }
        else
        {
            uint32_t acquiredMask = 0u;

            for (uint8_t axis = 0u; axis < GIMBAL_FAULT_AXIS_COUNT; axis++)
            {
                const uint32_t bit = 1u << axis;
                const MotorId id = GimbalFaultAxisId(axis);

                if ((actualMask & bit) != 0u &&
                    LowCmdInhibitManyFrom(&id, 1u, (uint16_t)LOWCMD_WRITER_SAFETY) != 0u)
                {
                    acquiredMask |= bit;
                }
            }
            s_gimbalFault.inhibitMask |= acquiredMask;
            if (acquiredMask != actualMask)
            {
                s_gimbalFault.inhibitFailCount++;
            }
        }
    }

    count = GimbalFaultCollectIds(plan.releaseMask, ids, &actualMask);
    if (actualMask != plan.releaseMask)
    {
        s_gimbalFault.releaseFailCount++;
    }
    if (count == 0u)
    {
        return;
    }

    GimbalFaultResetControl(control, snapshot, actualMask);
    if (LowCmdRecoverSafetyInhibitManyWithPermit(ids, count, permit) != 0u)
    {
        s_gimbalFault.inhibitMask &= ~actualMask;
        return;
    }

    {
        uint32_t releasedMask = 0u;

        for (uint8_t axis = 0u; axis < GIMBAL_FAULT_AXIS_COUNT; axis++)
        {
            const uint32_t bit = 1u << axis;
            const MotorId id = GimbalFaultAxisId(axis);

            if ((actualMask & bit) != 0u &&
                LowCmdRecoverSafetyInhibitWithPermit(id, permit) != 0u)
            {
                releasedMask |= bit;
            }
        }
        s_gimbalFault.inhibitMask &= ~releasedMask;
        if (releasedMask != actualMask)
        {
            s_gimbalFault.releaseFailCount++;
        }
    }
}

static void GimbalFaultApplyControl(GimbalControl *control,
                                    const GimbalRuntimeSnapshot *snapshot)
{
    GimbalFaultResetControl(control, snapshot, s_gimbalFault.holdZeroMask);
}

static void GimbalFaultApplyOutput(int16_t *yawCurrent,
                                   int16_t *yawUpperCurrent,
                                   int16_t *pitchCurrent)
{
    if (yawCurrent != NULL &&
        (s_gimbalFault.holdZeroMask & GIMBAL_FAULT_MASK_YAW) != 0u)
    {
        *yawCurrent = 0;
    }
    if (yawUpperCurrent != NULL &&
        (s_gimbalFault.holdZeroMask & GIMBAL_FAULT_MASK_YAW_UPPER) != 0u)
    {
        *yawUpperCurrent = 0;
    }
    if (pitchCurrent != NULL &&
        (s_gimbalFault.holdZeroMask & GIMBAL_FAULT_MASK_PITCH) != 0u)
    {
        *pitchCurrent = 0;
    }
}
