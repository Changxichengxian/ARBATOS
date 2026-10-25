/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * 云台输出发布和反馈切换清零确认。
 */

#include "GimbalTaskPrivate.h"

#if ROBOT_TASK_BUILD_ANY_GIMBAL

#include <string.h>

#include "LowCmd.h"
#include "MotorAxisFaultPolicy.h"

static uint32_t GimbalMotorBit(MotorId id)
{
    return ((uint32_t)id < (uint32_t)MotorCount) ? (1ul << (uint32_t)id) : 0u;
}

void GimbalControlInhibitOutputs(GimbalTaskContext *ctx,
                                 const MotorCurrentBind *bindings,
                                        uint8_t count)
{
    if (bindings == NULL)
    {
        return;
    }
    for (uint8_t i = 0u; i < count; i++)
    {
        const MotorId id = bindings[i].actuator_id;
        const uint32_t bit = GimbalMotorBit(id);
        uint16_t writer = (uint16_t)LOWCMD_WRITER_NONE;

        if (bindings[i].enabled == 0u || bit == 0u ||
            (ctx->controlInhibitMask & bit) != 0u ||
            LowCmdGetInhibitWriter(id, &writer) == 0u)
        {
            continue;
        }
        if (writer >= (uint16_t)LOWCMD_WRITER_SAFETY)
        {
            continue;
        }
        if (LowCmdInhibitManyFrom(&id, 1u, (uint16_t)LOWCMD_WRITER_SAFETY) != 0u)
        {
            ctx->controlInhibitMask |= bit;
        }
    }
}

uint8_t GimbalControlReleaseOutputs(GimbalTaskContext *ctx,
                                    const MotorCurrentBind *bindings,
                                           uint8_t bindingCount,
                                           const ControlOutputPermit *permit)
{
    MotorId ids[DUAL_YAW_GIMBAL_OUTPUT_MOTOR_COUNT];
    uint32_t requiredMask = 0u;
    uint8_t count = 0u;

    if (bindings == NULL || bindingCount > DUAL_YAW_GIMBAL_OUTPUT_MOTOR_COUNT)
    {
        return 0u;
    }
    for (uint8_t i = 0u; i < bindingCount; i++)
    {
        const MotorId id = bindings[i].actuator_id;
        const uint32_t bit = GimbalMotorBit(id);
        uint16_t writer = (uint16_t)LOWCMD_WRITER_NONE;

        if (bindings[i].enabled == 0u || bit == 0u ||
            (ctx->controlInhibitMask & bit) == 0u ||
            GimbalFaultInhibitsMotor(ctx, id) != 0u ||
            LowCmdGetInhibitWriter(id, &writer) == 0u)
        {
            continue;
        }
        if (writer == (uint16_t)LOWCMD_WRITER_NONE)
        {
            ctx->controlInhibitMask &= ~bit;
            continue;
        }
        if (writer != (uint16_t)LOWCMD_WRITER_SAFETY)
        {
            continue;
        }
        ids[count++] = id;
        requiredMask |= bit;
    }

    if (count == 0u)
    {
        return 1u;
    }
    if (LowCmdRecoverSafetyInhibitManyWithPermit(ids, count, permit) != 0u)
    {
        ctx->controlInhibitMask &= ~requiredMask;
        return 1u;
    }

    {
        uint32_t recoveredMask = 0u;

        for (uint8_t i = 0u; i < count; i++)
        {
            const uint32_t bit = GimbalMotorBit(ids[i]);

            if (LowCmdRecoverSafetyInhibitWithPermit(ids[i], permit) != 0u)
            {
                recoveredMask |= bit;
            }
        }
        ctx->controlInhibitMask &= ~recoveredMask;
        return (recoveredMask == requiredMask) ? 1u : 0u;
    }
}

uint8_t GimbalPublishCurrents(GimbalTaskContext *ctx,
                              const MotorCurrentBind *bindings,
                                     const uint32_t *axisMasks,
                                     const int16_t *currents,
                                     uint8_t bindingCount,
                                     const ControlOutputPermit *permit,
                                     uint32_t preserveAxisMask,
                                     GimbalPublishResult *result)
{
    MotorId ids[DUAL_YAW_GIMBAL_OUTPUT_MOTOR_COUNT];
    int16_t publishedCurrents[DUAL_YAW_GIMBAL_OUTPUT_MOTOR_COUNT];
    uint8_t count = 0u;

    if (result != NULL)
    {
        (void)memset(result, 0, sizeof(*result));
    }
    if (bindings == NULL || axisMasks == NULL || currents == NULL ||
        bindingCount > DUAL_YAW_GIMBAL_OUTPUT_MOTOR_COUNT)
    {
        return 0u;
    }
    for (uint8_t i = 0u; i < bindingCount; i++)
    {
        uint16_t writer = (uint16_t)LOWCMD_WRITER_NONE;

        if ((preserveAxisMask & axisMasks[i]) != 0u ||
            bindings[i].enabled == 0u ||
            (uint32_t)bindings[i].actuator_id >= (uint32_t)MotorCount ||
            LowCmdGetInhibitWriter(bindings[i].actuator_id, &writer) == 0u ||
            writer != (uint16_t)LOWCMD_WRITER_NONE)
        {
            continue;
        }
        ids[count] = bindings[i].actuator_id;
        if (result != NULL)
        {
            result->ids[count] = bindings[i].actuator_id;
            result->axisMasks[count] = axisMasks[i];
        }
        publishedCurrents[count] = GimbalFaultFrameCurrent(
            axisMasks[i],
            ctx->fault.holdZeroMask,
            currents[i]);
        count++;
    }

    if (count == 0u)
    {
        return ControlMgrOutputPermitValid(permit, 0u);
    }
    if (MotorInstSetCurrentIdsWithPermit(ids,
                                         publishedCurrents,
                                         count,
                                         permit) == 0u)
    {
        return 0u;
    }
    if (result != NULL)
    {
        result->count = count;
    }
    return 1u;
}

static uint8_t GimbalFeedbackRouteDebtHeld(const GimbalTaskContext *ctx,
                                           uint8_t axis, MotorCmd *held)
{
    uint32_t axisMask;
    MotorId id;

    if (axis >= GIMBAL_FAULT_AXIS_COUNT || held == NULL)
    {
        return 0u;
    }
    axisMask = 1u << axis;
    id = (MotorId)ctx->feedbackRouteDebt.motorId[axis];
    if ((GimbalFeedbackRouteDebtMask(&ctx->feedbackRouteDebt) & axisMask) == 0u ||
        (uint32_t)id >= (uint32_t)MotorCount ||
        LowCmdGetMotor(id, held) == 0u ||
        held->active == 0u ||
        held->mode != (uint8_t)MotorModeCurrent ||
        held->writer != (uint16_t)LOWCMD_WRITER_CONTROL ||
        held->current != 0 ||
        held->seq != ctx->feedbackRouteDebt.cmdSeq[axis] ||
        held->seqEpoch != ctx->feedbackRouteDebt.cmdSeqEpoch[axis] ||
        held->tick != ctx->feedbackRouteDebt.cmdTick[axis])
    {
        return 0u;
    }
    return 1u;
}

uint8_t GimbalFeedbackRouteDebtHasMotor(const GimbalTaskContext *ctx, MotorId id)
{
    const uint32_t debtMask =
        GimbalFeedbackRouteDebtMask(&ctx->feedbackRouteDebt);

    if ((uint32_t)id >= (uint32_t)MotorCount)
    {
        return 0u;
    }
    for (uint8_t axis = 0u; axis < GIMBAL_FAULT_AXIS_COUNT; axis++)
    {
        if ((debtMask & (1u << axis)) != 0u &&
            ctx->feedbackRouteDebt.motorId[axis] == (uint8_t)id)
        {
            return 1u;
        }
    }
    return 0u;
}

void GimbalSingleMitRouteObserve(GimbalTaskContext *ctx,
                                 const GimbalRuntimeSnapshot *snapshot)
{
    MotorId target;

    if (GimbalSingleMitYawTestActive(ctx, snapshot) == 0u)
    {
        ctx->singleMitDesiredPrev = 0u;
        ctx->singleMitTargetPrev = MotorCount;
        return;
    }
    target = snapshot->target_motor;
    if (ctx->singleMitDesiredPrev != 0u &&
        ctx->singleMitTargetPrev == target)
    {
        return;
    }
    for (uint8_t axis = 0u; axis < GIMBAL_FAULT_AXIS_COUNT; axis++)
    {
        if (GimbalFaultAxisId(ctx, axis) != target)
        {
            continue;
        }
        if (GimbalFeedbackRouteDebtHasMotor(ctx, target) != 0u ||
            GimbalFeedbackRouteDebtRequestPublish(
                &ctx->feedbackRouteDebt,
                1u << axis,
                (uint8_t)target) != 0u)
        {
            ctx->singleMitDesiredPrev = 1u;
            ctx->singleMitTargetPrev = target;
        }
        return;
    }
    ctx->singleMitDesiredPrev = 0u;
    ctx->singleMitTargetPrev = MotorCount;
}

void GimbalFeedbackRouteDebtObserve(GimbalTaskContext *ctx,
    const GimbalRuntimeSnapshot *snapshot,
    const MotorCurrentBind *bindings,
    const uint32_t *axisMasks,
    uint8_t bindingCount)
{
    const uint32_t debtMask =
        GimbalFeedbackRouteDebtMask(&ctx->feedbackRouteDebt);

    if (bindings == NULL || axisMasks == NULL ||
        bindingCount > DUAL_YAW_GIMBAL_OUTPUT_MOTOR_COUNT)
    {
        return;
    }
    for (uint8_t i = 0u; i < bindingCount; i++)
    {
        if (bindings[i].enabled == 0u ||
            (uint32_t)bindings[i].actuator_id >= (uint32_t)MotorCount ||
            (debtMask & axisMasks[i]) != 0u ||
            GimbalRuntimeAllowsMotor(snapshot,
                                     bindings[i].actuator_id) != 0u)
        {
            continue;
        }
        (void)GimbalFeedbackRouteDebtRequestPublish(
            &ctx->feedbackRouteDebt,
            axisMasks[i],
            (uint8_t)bindings[i].actuator_id);
    }
}

void GimbalFeedbackRouteDebtPoll(GimbalTaskContext *ctx,
    const GimbalRuntimeSnapshot *snapshot)
{
    for (uint8_t axis = 0u; axis < GIMBAL_FAULT_AXIS_COUNT; axis++)
    {
        const uint32_t axisMask = 1u << axis;
        const MotorId id = (MotorId)ctx->feedbackRouteDebt.motorId[axis];
        MotorCmd held;

        if ((ctx->feedbackRouteDebt.blockedMask & axisMask) == 0u)
        {
            continue;
        }
        if (GimbalFeedbackRouteDebtHeld(ctx, axis, &held) == 0u ||
            GimbalRuntimeAllowsMotor(snapshot, id) != 0u)
        {
            (void)GimbalFeedbackRouteDebtRequestPublish(
                &ctx->feedbackRouteDebt,
                axisMask,
                (uint8_t)id);
        }
    }

    for (uint8_t axis = 0u; axis < GIMBAL_FAULT_AXIS_COUNT; axis++)
    {
        const uint32_t axisMask = 1u << axis;
        const MotorId id = (MotorId)ctx->feedbackRouteDebt.motorId[axis];
        MotorCmd held;
        MotorTxReceipt receipt;

        if ((ctx->feedbackRouteDebt.waitMask & axisMask) == 0u)
        {
            continue;
        }
        if (GimbalFeedbackRouteDebtHeld(ctx, axis, &held) == 0u)
        {
            (void)GimbalFeedbackRouteDebtRequestPublish(
                &ctx->feedbackRouteDebt,
                axisMask,
                (uint8_t)id);
            continue;
        }
        if (GimbalRuntimeAllowsMotor(snapshot, id) == 0u)
        {
            (void)GimbalFeedbackRouteDebtHold(&ctx->feedbackRouteDebt,
                                              axisMask,
                                              (uint8_t)id,
                                              held.seq,
                                              held.seqEpoch,
                                              held.tick);
            continue;
        }
        if (LowStateGetTxReceipt(id, &receipt) != 0u &&
            MotorTxReceiptMatches(&receipt,
                                  held.seq,
                                  held.seqEpoch,
                                  held.tick,
                                  (uint16_t)LOWCMD_WRITER_CONTROL,
                                  (uint8_t)MotorModeCurrent,
                                  0) != 0u)
        {
            (void)GimbalFeedbackRouteDebtComplete(&ctx->feedbackRouteDebt,
                                                  axisMask);
        }
    }
}

void GimbalFeedbackRouteDebtCapture(GimbalTaskContext *ctx,
    const GimbalRuntimeSnapshot *snapshot,
    const GimbalPublishResult *result)
{
    if (result == NULL)
    {
        return;
    }

    for (uint8_t i = 0u; i < result->count; i++)
    {
        const uint32_t axisMask = result->axisMasks[i];
        MotorCmd cmd;

        if ((ctx->feedbackRouteDebt.publishMask & axisMask) == 0u ||
            LowCmdGetMotor(result->ids[i], &cmd) == 0u ||
            cmd.active == 0u ||
            cmd.mode != (uint8_t)MotorModeCurrent ||
            cmd.writer != (uint16_t)LOWCMD_WRITER_CONTROL ||
            cmd.current != 0)
        {
            continue;
        }
        if (GimbalRuntimeAllowsMotor(snapshot, result->ids[i]) == 0u)
        {
            (void)GimbalFeedbackRouteDebtHold(&ctx->feedbackRouteDebt,
                                              axisMask,
                                              (uint8_t)result->ids[i],
                                              cmd.seq,
                                              cmd.seqEpoch,
                                              cmd.tick);
        }
        else
        {
            (void)GimbalFeedbackRouteDebtWait(&ctx->feedbackRouteDebt,
                                              axisMask,
                                              (uint8_t)result->ids[i],
                                              cmd.seq,
                                              cmd.seqEpoch,
                                              cmd.tick);
        }
    }
}

static uint8_t GimbalFeedbackRouteDebtCoverBarrier(GimbalTaskContext *ctx,
    const GimbalRuntimeSnapshot *snapshot,
    uint32_t axisMask)
{
    const uint8_t axis = (axisMask == GIMBAL_FAULT_MASK_YAW) ?
                             GIMBAL_FAULT_AXIS_YAW :
                             ((axisMask == GIMBAL_FAULT_MASK_YAW_UPPER) ?
                                  GIMBAL_FAULT_AXIS_YAW_UPPER :
                                  GIMBAL_FAULT_AXIS_PITCH);
    const MotorId id = (MotorId)ctx->feedbackRouteDebt.motorId[axis];
    MotorCmd held;

    if (GimbalFeedbackRouteDebtHeld(ctx, axis, &held) == 0u)
    {
        return 0u;
    }
    if ((ctx->feedbackRouteDebt.blockedMask & axisMask) != 0u)
    {
        return (uint8_t)(GimbalRuntimeAllowsMotor(snapshot, id) == 0u);
    }
    if ((ctx->feedbackRouteDebt.waitMask & axisMask) != 0u &&
        GimbalRuntimeAllowsMotor(snapshot, id) != 0u)
    {
        return GimbalFeedbackZeroBarrierAdd(&ctx->feedbackZeroBarrier,
                                            axisMask,
                                            (uint8_t)id,
                                            held.seq,
                                            held.seqEpoch,
                                            held.tick);
    }
    return 0u;
}

uint8_t GimbalFeedbackZeroBarrierCapture(GimbalTaskContext *ctx,
    const GimbalRuntimeSnapshot *snapshot,
    const GimbalPublishResult *result)
{
    uint32_t coveredMask;

    if (snapshot == NULL || result == NULL ||
        snapshot->feedback_transition_pending == 0u)
    {
        return 0u;
    }

    GimbalFeedbackZeroBarrierBegin(&ctx->feedbackZeroBarrier,
                                   snapshot->feedback_transition_seq);
    coveredMask = 0u;
    for (uint8_t i = 0u; i < result->count; i++)
    {
        MotorCmd cmd;
        const uint32_t axisMask = result->axisMasks[i];

        if ((snapshot->feedback_transition_zero_mask & axisMask) == 0u)
        {
            continue;
        }
        if (LowCmdGetMotor(result->ids[i], &cmd) == 0u ||
            cmd.active == 0u ||
            cmd.mode != (uint8_t)MotorModeCurrent ||
            cmd.writer != (uint16_t)LOWCMD_WRITER_CONTROL ||
            cmd.current != 0)
        {
            GimbalFeedbackZeroBarrierInit(&ctx->feedbackZeroBarrier);
            return 0u;
        }
        if (GimbalRuntimeAllowsMotor(snapshot, result->ids[i]) == 0u)
        {
            if (GimbalFeedbackRouteDebtHold(&ctx->feedbackRouteDebt,
                                            axisMask,
                                            (uint8_t)result->ids[i],
                                            cmd.seq,
                                            cmd.seqEpoch,
                                            cmd.tick) == 0u)
            {
                GimbalFeedbackZeroBarrierInit(&ctx->feedbackZeroBarrier);
                return 0u;
            }
        }
        else if (GimbalFeedbackZeroBarrierAdd(&ctx->feedbackZeroBarrier,
                                              axisMask,
                                              (uint8_t)result->ids[i],
                                              cmd.seq,
                                              cmd.seqEpoch,
                                              cmd.tick) == 0u)
        {
            GimbalFeedbackZeroBarrierInit(&ctx->feedbackZeroBarrier);
            return 0u;
        }
        coveredMask |= axisMask;
    }
    for (uint8_t axis = 0u; axis < GIMBAL_FAULT_AXIS_COUNT; axis++)
    {
        const uint32_t axisMask = 1u << axis;
        const MotorId id = GimbalFaultAxisId(ctx, axis);
        uint16_t writer = (uint16_t)LOWCMD_WRITER_NONE;

        if ((snapshot->feedback_transition_zero_mask & axisMask) == 0u ||
            (coveredMask & axisMask) != 0u)
        {
            continue;
        }
        if (GimbalFeedbackRouteDebtCoverBarrier(ctx, snapshot, axisMask) != 0u)
        {
            coveredMask |= axisMask;
            continue;
        }
        if ((ctx->fault.inhibitMask & axisMask) == 0u)
        {
            continue;
        }
        if ((uint32_t)id < (uint32_t)MotorCount &&
            LowCmdGetInhibitWriter(id, &writer) != 0u &&
            writer == (uint16_t)LOWCMD_WRITER_SAFETY &&
            GimbalFeedbackZeroBarrierExcludeSafe(
                &ctx->feedbackZeroBarrier,
                axisMask) != 0u)
        {
            coveredMask |= axisMask;
        }
    }
    if ((coveredMask & snapshot->feedback_transition_zero_mask) !=
        snapshot->feedback_transition_zero_mask)
    {
        GimbalFeedbackZeroBarrierInit(&ctx->feedbackZeroBarrier);
        return 0u;
    }
    return 1u;
}

GimbalFeedbackZeroWait GimbalFeedbackZeroBarrierPoll(GimbalTaskContext *ctx,
    const GimbalRuntimeSnapshot *snapshot)
{
    uint8_t allReady = 1u;

    if (snapshot == NULL || snapshot->feedback_transition_pending == 0u)
    {
        GimbalFeedbackZeroBarrierInit(&ctx->feedbackZeroBarrier);
        return GimbalFeedbackZeroWaitNone;
    }
    if (GimbalFeedbackZeroBarrierMatches(&ctx->feedbackZeroBarrier,
                                         snapshot->feedback_transition_seq) == 0u)
    {
        GimbalFeedbackZeroBarrierInit(&ctx->feedbackZeroBarrier);
        return GimbalFeedbackZeroWaitNone;
    }

    for (uint8_t axis = 0u; axis < GIMBAL_FAULT_AXIS_COUNT; axis++)
    {
        const uint32_t axisMask = 1u << axis;
        const MotorId id = GimbalFaultAxisId(ctx, axis);
        uint16_t inhibitWriter = (uint16_t)LOWCMD_WRITER_NONE;

        if ((ctx->feedbackZeroBarrier.safeInhibitMask & axisMask) == 0u)
        {
            continue;
        }
        if ((ctx->fault.inhibitMask & axisMask) == 0u ||
            (uint32_t)id >= (uint32_t)MotorCount ||
            LowCmdGetInhibitWriter(id, &inhibitWriter) == 0u ||
            inhibitWriter != (uint16_t)LOWCMD_WRITER_SAFETY)
        {
            GimbalFeedbackZeroBarrierInit(&ctx->feedbackZeroBarrier);
            return GimbalFeedbackZeroWaitNone;
        }
    }

    for (uint8_t axis = 0u; axis < 3u; axis++)
    {
        const uint32_t axisMask = 1u << axis;
        const MotorId id = (MotorId)ctx->feedbackZeroBarrier.motorId[axis];
        MotorCmd held;
        MotorTxReceipt receipt;
        uint16_t inhibitWriter = (uint16_t)LOWCMD_WRITER_NONE;

        if ((ctx->feedbackZeroBarrier.waitMask & axisMask) == 0u)
        {
            continue;
        }
        if ((uint32_t)id >= (uint32_t)MotorCount ||
            LowCmdGetInhibitWriter(id, &inhibitWriter) == 0u)
        {
            GimbalFeedbackZeroBarrierInit(&ctx->feedbackZeroBarrier);
            return GimbalFeedbackZeroWaitNone;
        }
        if (inhibitWriter != (uint16_t)LOWCMD_WRITER_NONE)
        {
            if (inhibitWriter == (uint16_t)LOWCMD_WRITER_SAFETY &&
                (ctx->fault.inhibitMask & axisMask) != 0u)
            {
                if (GimbalFeedbackZeroBarrierExcludeSafe(
                        &ctx->feedbackZeroBarrier,
                        axisMask) == 0u)
                {
                    GimbalFeedbackZeroBarrierInit(&ctx->feedbackZeroBarrier);
                    return GimbalFeedbackZeroWaitNone;
                }
                ctx->feedbackZeroBarrier.waitMask &= ~axisMask;
                continue;
            }
            GimbalFeedbackZeroBarrierInit(&ctx->feedbackZeroBarrier);
            return GimbalFeedbackZeroWaitNone;
        }
        if (LowCmdGetMotor(id, &held) == 0u ||
            held.active == 0u ||
            held.mode != (uint8_t)MotorModeCurrent ||
            held.writer != (uint16_t)LOWCMD_WRITER_CONTROL ||
            held.current != 0 ||
            held.seq != ctx->feedbackZeroBarrier.cmdSeq[axis] ||
            held.seqEpoch != ctx->feedbackZeroBarrier.cmdSeqEpoch[axis] ||
            held.tick != ctx->feedbackZeroBarrier.cmdTick[axis])
        {
            GimbalFeedbackZeroBarrierInit(&ctx->feedbackZeroBarrier);
            return GimbalFeedbackZeroWaitNone;
        }
        if (LowStateGetTxReceipt(id, &receipt) == 0u ||
            MotorTxReceiptMatches(&receipt,
                                  held.seq,
                                  held.seqEpoch,
                                  held.tick,
                                  (uint16_t)LOWCMD_WRITER_CONTROL,
                                  (uint8_t)MotorModeCurrent,
                                  0) == 0u)
        {
            allReady = 0u;
        }
    }
    return (allReady != 0u) ?
               GimbalFeedbackZeroReady :
               GimbalFeedbackZeroWaiting;
}

#endif
