/* CAN 物理完成账本主机回归：票据回绕、共享帧和迟到结果。 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "CanTxCompletionPolicy.h"

static int TestCheck(int condition, const char *message)
{
    if (condition != 0)
    {
        return 1;
    }
    (void)fprintf(stderr, "FAIL: %s\n", message);
    return 0;
}

static MotorTxIdentity TestIdentity(uint32_t epoch, uint32_t seq, int16_t current)
{
    MotorTxIdentity identity;

    (void)memset(&identity, 0, sizeof(identity));
    identity.cmdSeqEpoch = epoch;
    identity.cmdSeq = seq;
    identity.cmdTick = 100u;
    identity.current = current;
    identity.writer = (uint16_t)LOWCMD_WRITER_CONTROL;
    identity.active = 1u;
    identity.mode = (uint8_t)MotorModeCurrent;
    return identity;
}

static int TestTicketWrap(void)
{
    BspCanTxTicket state = {0u, 0u};
    BspCanTxTicket ticket = CanTxTicketAdvance(&state);

    if (!TestCheck(ticket.epoch == 0u && ticket.seq == 1u,
                   "首张票据必须跳过全零保留值")) return 0;
    state.epoch = 7u;
    state.seq = UINT32_MAX;
    ticket = CanTxTicketAdvance(&state);
    if (!TestCheck(ticket.epoch == 8u && ticket.seq == 0u,
                   "票据低位回绕必须推进高位代次")) return 0;
    state.epoch = UINT32_MAX;
    state.seq = UINT32_MAX;
    ticket = CanTxTicketAdvance(&state);
    return TestCheck(ticket.epoch == 1u && ticket.seq == 0u,
                     "完整回绕也不能生成全零票据");
}

static int TestPendingReplacementAndLateTicket(void)
{
    CanTxPhysicalPending pending = {0};
    MotorTxIdentity first = TestIdentity(2u, 10u, 0);
    MotorTxIdentity next = TestIdentity(2u, 11u, 20);
    MotorTxIdentity taken;
    const BspCanTxTicket old_ticket = {3u, 20u};
    const BspCanTxTicket new_ticket = {3u, 21u};

    if (!TestCheck(CanTxPendingPrepare(&pending, &first) != 0u,
                   "空账本必须允许建立跟踪")) return 0;
    CanTxPendingArm(&pending, &first, &old_ticket, 100u);
    if (!TestCheck(CanTxPendingPrepare(&pending, &first) == 0u,
                   "同一命令已有未决帧时不能重复占账本")) return 0;
    if (!TestCheck(CanTxPendingPrepare(&pending, &next) != 0u,
                   "新命令必须立即取代旧的软件等待项")) return 0;
    CanTxPendingArm(&pending, &next, &new_ticket, 101u);
    if (!TestCheck(CanTxPendingTake(&pending, &old_ticket, &taken) == 0u &&
                       pending.active != 0u,
                   "迟到旧票据不能终结新命令")) return 0;
    return TestCheck(CanTxPendingTake(&pending, &new_ticket, &taken) != 0u &&
                         MotorTxIdentityEqual(&taken, &next) != 0u &&
                         pending.active == 0u,
                     "当前票据必须只交付一次精确命令身份");
}

static int TestRmSharedFrameAndTimeoutWrap(void)
{
    CanTxPhysicalPending pending[4] = {{0}, {0}, {0}, {0}};
    MotorTxIdentity identities[4];
    const BspCanTxTicket ticket = {5u, 30u};
    MotorTxIdentity taken;

    for (uint8_t axis = 0u; axis < 4u; axis++)
    {
        identities[axis] = TestIdentity(4u, (uint32_t)(40u + axis), axis);
        CanTxPendingArm(&pending[axis],
                        &identities[axis],
                        &ticket,
                        UINT32_MAX - 10u);
    }
    for (uint8_t axis = 0u; axis < 4u; axis++)
    {
        if (!TestCheck(CanTxPendingTake(&pending[axis], &ticket, &taken) != 0u &&
                           MotorTxIdentityEqual(&taken, &identities[axis]) != 0u,
                       "RM 同一物理帧必须能投影到每个精确轴身份")) return 0;
    }

    CanTxPendingArm(&pending[0],
                    &identities[0],
                    &ticket,
                    UINT32_MAX - 10u);
    return TestCheck(CanTxPendingExpire(&pending[0], 5u, 15u) != 0u &&
                         pending[0].active == 0u,
                     "未决超时必须按无符号时间差跨毫秒回绕工作");
}

int main(void)
{
    if (!TestTicketWrap()) return 1;
    if (!TestPendingReplacementAndLateTicket()) return 1;
    if (!TestRmSharedFrameAndTimeoutWrap()) return 1;
    (void)puts("PASS: CAN physical completion policy regression");
    return 0;
}
