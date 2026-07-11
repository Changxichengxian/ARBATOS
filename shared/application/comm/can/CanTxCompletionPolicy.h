/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CAN_TX_COMPLETION_POLICY_H
#define CAN_TX_COMPLETION_POLICY_H

#include "BspCan.h"
#include "LowCmd.h"

typedef struct
{
    BspCanTxTicket ticket;
    MotorTxIdentity identity;
    uint32_t submittedTick;
    uint8_t active;
} CanTxPhysicalPending;

static inline uint8_t CanTxTicketMatches(const BspCanTxTicket *left,
                                         const BspCanTxTicket *right)
{
    return (uint8_t)(left != NULL && right != NULL &&
                     left->epoch == right->epoch &&
                     left->seq == right->seq);
}

static inline BspCanTxTicket CanTxTicketAdvance(BspCanTxTicket *state)
{
    BspCanTxTicket ticket = {0u, 0u};

    if (state == NULL)
    {
        return ticket;
    }
    state->seq++;
    if (state->seq == 0u)
    {
        state->epoch++;
        if (state->epoch == 0u)
        {
            state->epoch = 1u;
        }
    }
    return *state;
}

/* 同一命令已有未决帧时不重复占账本；新命令立即使旧等待项失效。 */
static inline uint8_t CanTxPendingPrepare(CanTxPhysicalPending *pending,
                                          const MotorTxIdentity *identity)
{
    if (pending == NULL || identity == NULL)
    {
        return 0u;
    }
    if (pending->active != 0u &&
        MotorTxIdentityEqual(&pending->identity, identity) == 0u)
    {
        pending->active = 0u;
    }
    return (pending->active == 0u) ? 1u : 0u;
}

static inline void CanTxPendingArm(CanTxPhysicalPending *pending,
                                   const MotorTxIdentity *identity,
                                   const BspCanTxTicket *ticket,
                                   uint32_t submittedTick)
{
    if (pending == NULL || identity == NULL || ticket == NULL)
    {
        return;
    }
    pending->ticket = *ticket;
    pending->identity = *identity;
    pending->submittedTick = submittedTick;
    pending->active = 1u;
}

static inline uint8_t CanTxPendingTake(CanTxPhysicalPending *pending,
                                       const BspCanTxTicket *ticket,
                                       MotorTxIdentity *identity)
{
    if (pending == NULL || ticket == NULL || pending->active == 0u ||
        CanTxTicketMatches(&pending->ticket, ticket) == 0u)
    {
        return 0u;
    }
    if (identity != NULL)
    {
        *identity = pending->identity;
    }
    pending->active = 0u;
    return 1u;
}

static inline uint8_t CanTxPendingExpire(CanTxPhysicalPending *pending,
                                         uint32_t now,
                                         uint32_t timeoutMs)
{
    if (pending == NULL || pending->active == 0u ||
        (uint32_t)(now - pending->submittedTick) <= timeoutMs)
    {
        return 0u;
    }
    pending->active = 0u;
    return 1u;
}

#endif
