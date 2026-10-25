/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * 云台故障判定、输出禁止和安全恢复。
 */

#include "GimbalTaskPrivate.h"

#if ROBOT_TASK_BUILD_ANY_GIMBAL

#include <string.h>

#include "BspTime.h"
#include "LowCmd.h"
#include "MotorAxisFaultPolicy.h"
#include "MotorConfig.h"

#define GIMBAL_FAULT_REASON_MOTOR (1u << 0)

MotorId GimbalFaultAxisId(GimbalTaskContext *ctx, uint8_t axis)
{
    switch (axis)
    {
    case GIMBAL_FAULT_AXIS_YAW:
        return GimbalRuntimeMotorId(ctx, axis);
    case GIMBAL_FAULT_AXIS_YAW_UPPER:
        return GimbalRuntimeMotorId(ctx, axis);
    case GIMBAL_FAULT_AXIS_PITCH:
        return GimbalRuntimeMotorId(ctx, axis);
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

void GimbalFaultInit(GimbalTaskContext *ctx)
{
    FaultMgrConfig config;

    (void)memset(&ctx->fault, 0, sizeof(ctx->fault));
    (void)memset(&config, 0, sizeof(config));
    GimbalMotorIdsEnsure(ctx);
    ctx->fault.configuredMask = GimbalFaultConfiguredMask(NULL);
    config.deviceCount = GIMBAL_FAULT_AXIS_COUNT;
    for (uint8_t axis = 0u; axis < GIMBAL_FAULT_AXIS_COUNT; axis++)
    {
        config.device[axis].stableMs = GIMBAL_FAULT_RECOVERY_MS;
        config.device[axis].requireSafeInput = 1u;
    }
    ctx->fault.initialized =
        (FaultMgrInit(&ctx->fault.mgr, &config) == FaultMgrResultOk) ? 1u : 0u;
    if (ctx->fault.initialized == 0u)
    {
        ctx->fault.motorBlockingMask = ctx->fault.configuredMask;
        ctx->fault.blockingMask = ctx->fault.configuredMask;
    }
}

void GimbalFaultUpdate(GimbalTaskContext *ctx, const GimbalRuntimeSnapshot *snapshot)
{
    const uint32_t nowMs = BspTimeGetTickMs();
    uint32_t safeMask = 0u;
    uint8_t updateOk = 1u;

    ctx->fault.configuredMask = GimbalFaultConfiguredMask(snapshot);
    ctx->fault.activeMask = 0u;
    ctx->fault.motorBlockingMask = 0u;
    ctx->fault.blockingMask = 0u;
    ctx->fault.recoveryMask = 0u;
    ctx->fault.imuRequiredMask = (snapshot != NULL) ? snapshot->imu_required_mask : 0u;
    ctx->fault.recoveryInputSafe = GimbalFaultRecoveryInputSafe(snapshot);

    if (ctx->fault.initialized == 0u || snapshot == NULL)
    {
        ctx->fault.motorBlockingMask = ctx->fault.configuredMask;
        ctx->fault.blockingMask = ctx->fault.motorBlockingMask |
                                     ((snapshot != NULL) ? snapshot->feedback_block_mask : 0u);
        return;
    }

    for (uint8_t axis = 0u; axis < GIMBAL_FAULT_AXIS_COUNT; axis++)
    {
        const uint32_t bit = 1u << axis;
        const MotorHealthResult *health = GimbalFaultAxisHealth(snapshot, axis);
        const uint8_t configured = ((ctx->fault.configuredMask & bit) != 0u) ? 1u : 0u;
        const uint8_t motorFault = (uint8_t)(configured != 0u &&
                                             (health == NULL || health->healthy == 0u));

        if (motorFault != 0u)
        {
            ctx->fault.activeMask |= bit;
        }
        if (FaultMgrSetDeviceFault(&ctx->fault.mgr,
                                   axis,
                                   GIMBAL_FAULT_REASON_MOTOR,
                                   motorFault,
                                   nowMs) != FaultMgrResultOk)
        {
            updateOk = 0u;
        }
    }

    if (ctx->fault.recoveryInputSafe != 0u)
    {
        safeMask = ctx->fault.configuredMask;
    }
    if (updateOk == 0u ||
        FaultMgrUpdate(&ctx->fault.mgr, nowMs, safeMask, 0u) != FaultMgrResultOk)
    {
        ctx->fault.motorBlockingMask = ctx->fault.configuredMask;
        ctx->fault.blockingMask = ctx->fault.motorBlockingMask |
                                     snapshot->feedback_block_mask;
        return;
    }

    for (uint8_t axis = 0u; axis < GIMBAL_FAULT_AXIS_COUNT; axis++)
    {
        const uint32_t bit = 1u << axis;
        FaultDeviceStatus status;

        if ((ctx->fault.configuredMask & bit) == 0u)
        {
            continue;
        }
        if (FaultMgrGetDeviceStatus(&ctx->fault.mgr, axis, &status) != FaultMgrResultOk)
        {
            ctx->fault.motorBlockingMask |= bit;
            continue;
        }
        if (status.action != FaultActionRun)
        {
            ctx->fault.motorBlockingMask |= bit;
        }
        if (status.recoveryPending != 0u)
        {
            ctx->fault.recoveryMask |= bit;
        }
    }
    ctx->fault.blockingMask = ctx->fault.motorBlockingMask |
                                 snapshot->feedback_block_mask;
}

static uint8_t GimbalFaultCollectIds(GimbalTaskContext *ctx,
                                     uint32_t mask,
                                     MotorId out[GIMBAL_FAULT_AXIS_COUNT],
                                     uint32_t *collectedMask)
{
    uint32_t actualMask = 0u;
    uint8_t count = 0u;

    for (uint8_t axis = 0u; axis < GIMBAL_FAULT_AXIS_COUNT; axis++)
    {
        const uint32_t bit = 1u << axis;
        const MotorId id = GimbalFaultAxisId(ctx, axis);

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

void GimbalFaultSyncInhibit(GimbalTaskContext *ctx,
                            GimbalControl *control,
                                   const GimbalRuntimeSnapshot *snapshot,
                                   const ControlOutputPermit *permit)
{
    const MotorAxisFaultInhibitPlan plan =
        MotorAxisFaultInhibitPlanMake(ctx->fault.configuredMask,
                                      ctx->fault.blockingMask,
                                      ctx->fault.inhibitMask);
    MotorId ids[GIMBAL_FAULT_AXIS_COUNT];
    uint32_t actualMask = 0u;
    uint8_t count;

    ctx->fault.holdZeroMask = plan.holdZeroMask |
                                 ((snapshot != NULL) ?
                                      snapshot->feedback_transition_zero_mask :
                                      0u) |
                                 GimbalFeedbackRouteDebtMask(
                                     &ctx->feedbackRouteDebt);

    count = GimbalFaultCollectIds(ctx, plan.acquireMask, ids, &actualMask);
    if (actualMask != plan.acquireMask)
    {
        ctx->fault.inhibitFailCount++;
    }
    if (count != 0u)
    {
        if (LowCmdInhibitManyFrom(ids, count, (uint16_t)LOWCMD_WRITER_SAFETY) != 0u)
        {
            ctx->fault.inhibitMask |= actualMask;
        }
        else
        {
            uint32_t acquiredMask = 0u;

            for (uint8_t axis = 0u; axis < GIMBAL_FAULT_AXIS_COUNT; axis++)
            {
                const uint32_t bit = 1u << axis;
                const MotorId id = GimbalFaultAxisId(ctx, axis);

                if ((actualMask & bit) != 0u &&
                    LowCmdInhibitManyFrom(&id, 1u, (uint16_t)LOWCMD_WRITER_SAFETY) != 0u)
                {
                    acquiredMask |= bit;
                }
            }
            ctx->fault.inhibitMask |= acquiredMask;
            if (acquiredMask != actualMask)
            {
                ctx->fault.inhibitFailCount++;
            }
        }
    }

    count = GimbalFaultCollectIds(ctx, plan.releaseMask, ids, &actualMask);
    if (actualMask != plan.releaseMask)
    {
        ctx->fault.releaseFailCount++;
    }
    if (count == 0u)
    {
        return;
    }

    GimbalFaultResetControl(control, snapshot, actualMask);
    if (LowCmdRecoverSafetyInhibitManyWithPermit(ids, count, permit) != 0u)
    {
        ctx->fault.inhibitMask &= ~actualMask;
        return;
    }

    {
        uint32_t releasedMask = 0u;

        for (uint8_t axis = 0u; axis < GIMBAL_FAULT_AXIS_COUNT; axis++)
        {
            const uint32_t bit = 1u << axis;
            const MotorId id = GimbalFaultAxisId(ctx, axis);

            if ((actualMask & bit) != 0u &&
                LowCmdRecoverSafetyInhibitWithPermit(id, permit) != 0u)
            {
                releasedMask |= bit;
            }
        }
        ctx->fault.inhibitMask &= ~releasedMask;
        if (releasedMask != actualMask)
        {
            ctx->fault.releaseFailCount++;
        }
    }
}

void GimbalFaultApplyControl(GimbalTaskContext *ctx,
                             GimbalControl *control,
                                    const GimbalRuntimeSnapshot *snapshot)
{
    GimbalFaultResetControl(control, snapshot, ctx->fault.holdZeroMask);
}

void GimbalFaultApplyOutput(const GimbalTaskContext *ctx,
                            int16_t *yawCurrent,
                                   int16_t *yawUpperCurrent,
                                   int16_t *pitchCurrent)
{
    if (yawCurrent != NULL &&
        (ctx->fault.holdZeroMask & GIMBAL_FAULT_MASK_YAW) != 0u)
    {
        *yawCurrent = 0;
    }
    if (yawUpperCurrent != NULL &&
        (ctx->fault.holdZeroMask & GIMBAL_FAULT_MASK_YAW_UPPER) != 0u)
    {
        *yawUpperCurrent = 0;
    }
    if (pitchCurrent != NULL &&
        (ctx->fault.holdZeroMask & GIMBAL_FAULT_MASK_PITCH) != 0u)
    {
        *pitchCurrent = 0;
    }
}

uint8_t GimbalFaultInhibitsMotor(GimbalTaskContext *ctx, MotorId id)
{
    for (uint8_t axis = 0u; axis < GIMBAL_FAULT_AXIS_COUNT; axis++)
    {
        if ((ctx->fault.inhibitMask & (1ul << axis)) != 0u &&
            GimbalFaultAxisId(ctx, axis) == id)
        {
            return 1u;
        }
    }
    return 0u;
}

#endif
