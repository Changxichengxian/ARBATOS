/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Private implementation chunk included by CanTxTask.c.
 */

#define CAN_TX_PHYSICAL_PENDING_TIMEOUT_MS 50u

static CanTxPhysicalPending s_can_tx_physical_pending[MotorCount];
static BspCanTxTicket s_can_tx_next_ticket;

static BspCanTxTicket CanTxPhysicalTicketNext(void)
{
    return CanTxTicketAdvance(&s_can_tx_next_ticket);
}

/* 新命令可以立即取代旧命令的软件等待项；旧硬件票据晚到时会因票据不同被忽略。 */
static uint8_t CanTxPhysicalPrepare(MotorId id,
                                    const MotorCmd *cmd,
                                    MotorTxIdentity *identity)
{
    CanTxPhysicalPending *pending;

    if (CanTxActuatorIdValid(id) == 0u || cmd == NULL || identity == NULL)
    {
        return 0u;
    }

    MotorTxIdentityFromCmd(identity, cmd);
    pending = &s_can_tx_physical_pending[id];
    return CanTxPendingPrepare(pending, identity);
}

static void CanTxPhysicalArm(MotorId id,
                             const MotorTxIdentity *identity,
                             const BspCanTxTicket *ticket)
{
    CanTxPhysicalPending *pending;

    if (CanTxActuatorIdValid(id) == 0u || identity == NULL || ticket == NULL)
    {
        return;
    }

    pending = &s_can_tx_physical_pending[id];
    CanTxPendingArm(pending, identity, ticket, CanTxNowMs());
}

static void CanTxPhysicalProcessCompletions(void)
{
    BspCanTxCompletion completion;
    const uint32_t now = CanTxNowMs();

    BspCanTxCompletionPoll();
    while (BspCanTxCompletionPop(&completion) != 0)
    {
        for (uint8_t id = 0u; id < (uint8_t)MotorCount; id++)
        {
            CanTxPhysicalPending *pending = &s_can_tx_physical_pending[id];
            MotorTxIdentity identity;

            if (CanTxPendingTake(pending,
                                 &completion.ticket,
                                 &identity) == 0u)
            {
                continue;
            }
            if (completion.result == (uint8_t)BspCanTxResultComplete)
            {
                LowStateUpdateTxComplete((MotorId)id, &identity);
            }
        }
    }

    for (uint8_t id = 0u; id < (uint8_t)MotorCount; id++)
    {
        CanTxPhysicalPending *pending = &s_can_tx_physical_pending[id];

        (void)CanTxPendingExpire(pending,
                                 now,
                                 CAN_TX_PHYSICAL_PENDING_TIMEOUT_MS);
    }
}
