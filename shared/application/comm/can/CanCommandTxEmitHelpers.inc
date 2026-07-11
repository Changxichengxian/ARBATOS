/*
 * SPDX-FileCopyrightText: 2026 ?? <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Private implementation chunk included by CanTxTask.c.
 */

static uint8_t CanTxRmFrameHasOutput(const int16_t frame[4])
{
    return (uint8_t)(frame[0] != 0 ||
                     frame[1] != 0 ||
                     frame[2] != 0 ||
                     frame[3] != 0);
}

static uint8_t CanTxRmGroupHasConfig(uint8_t bus, uint16_t group_id)
{
    uint8_t bus_index = 0u;
    uint8_t group_index = 0u;

    if (CanTxBusIndex(bus, &bus_index) == 0u ||
        CanTxRmGroupIndex(group_id, &group_index) == 0u)
    {
        return 0u;
    }

    return s_can_tx_rm_group_configured[bus_index][group_index];
}

static void CanTxRecheckRmFrame(uint8_t bus,
                                uint16_t group_id,
                                int16_t frame[4],
                                uint8_t receiptEligible[4],
                                sdlog_actuator_current_t *log)
{
    uint8_t bus_index = 0u;
    uint8_t group_index = 0u;

    if (frame == NULL || receiptEligible == NULL ||
        CanTxBusIndex(bus, &bus_index) == 0u ||
        CanTxRmGroupIndex(group_id, &group_index) == 0u)
    {
        return;
    }

    for (uint8_t slot = 0u; slot < 4u; slot++)
    {
        const MotorId actuator_id = s_can_tx_rm_motor[bus_index][group_index][slot];

        receiptEligible[slot] = 0u;
        if (CanTxActuatorIdValid(actuator_id) != 0u &&
            s_can_tx_cmd_cache_valid[actuator_id] != 0u &&
            LowCmdOutputSnapshotAuthorized(actuator_id,
                                           &s_can_tx_cmd_cache[actuator_id],
                                           &s_can_tx_owner_cache[actuator_id]) != 0u)
        {
            receiptEligible[slot] = 1u;
            continue;
        }

        if (frame[slot] != 0)
        {
            frame[slot] = 0;
            if (CanTxActuatorIdValid(actuator_id) != 0u)
            {
                const MotorRoute *route = MotorRouteFindByMotor(actuator_id);

                s_can_tx_authority_reject_count[actuator_id]++;
                CanTxUpdateApplied(route,
                                   NULL,
                                   0,
                                   (uint8_t)MotorAppliedFlagForceDisabled);
                CanTxLogMotorCmd(log, route, 0);
            }
        }
    }
}

static void CanTxEmitRmGroupIfNeeded(uint8_t bus,
                                     uint16_t group_id,
                                     int16_t frame[4],
                                     sdlog_actuator_current_t *log)
{
    uint8_t bus_index = 0u;
    uint8_t group_index = 0u;
    uint8_t receiptEligible[4] = {0u};
    uint8_t trackEligible[4] = {0u};
    uint8_t haveTrackCandidate = 0u;
    uint8_t tracked = 0u;
    MotorTxIdentity identities[4] = {0};
    BspCanTxTicket ticket = {0};
    int tx_ret;

    /*
     * 与 LowCmd 的禁写获取使用同一任务临界区，把“最终复核→非阻塞入队”
     * 线性化：故障要么先取得禁写并让本帧归零，要么发生在本帧已经入队之后。
     */
    taskENTER_CRITICAL();
    if (RobotSafetyOutputLocked() != 0u)
    {
        (void)memset(frame, 0, sizeof(int16_t) * 4u);
    }
    else
    {
        CanTxRecheckRmFrame(bus, group_id, frame, receiptEligible, log);
    }
    if (CanTxRmFrameHasOutput(frame) == 0u &&
        CanTxRmGroupHasConfig(bus, group_id) == 0u)
    {
        taskEXIT_CRITICAL();
        return;
    }

    if (CanTxBusIndex(bus, &bus_index) != 0u &&
        CanTxRmGroupIndex(group_id, &group_index) != 0u)
    {
        for (uint8_t slot = 0u; slot < 4u; slot++)
        {
            const MotorId actuator_id = s_can_tx_rm_motor[bus_index][group_index][slot];

            if (receiptEligible[slot] != 0u &&
                CanTxActuatorIdValid(actuator_id) != 0u &&
                s_can_tx_cmd_cache_valid[actuator_id] != 0u)
            {
                trackEligible[slot] = CanTxPhysicalPrepare(
                    actuator_id,
                    &s_can_tx_cmd_cache[actuator_id],
                    &identities[slot]);
                haveTrackCandidate |= trackEligible[slot];
            }
        }
    }
    if (haveTrackCandidate != 0u)
    {
        ticket = CanTxPhysicalTicketNext();
        tx_ret = CAN_cmd_rm_group_tracked(bus,
                                          group_id,
                                          frame[0],
                                          frame[1],
                                          frame[2],
                                          frame[3],
                                          &ticket,
                                          &tracked);
    }
    else
    {
        tx_ret = CAN_cmd_rm_group(bus,
                                  group_id,
                                  frame[0],
                                  frame[1],
                                  frame[2],
                                  frame[3]);
    }
    if (tx_ret == 0 &&
        CanTxBusIndex(bus, &bus_index) != 0u &&
        CanTxRmGroupIndex(group_id, &group_index) != 0u)
    {
        for (uint8_t slot = 0u; slot < 4u; slot++)
        {
            const MotorId actuator_id = s_can_tx_rm_motor[bus_index][group_index][slot];

            if (receiptEligible[slot] != 0u &&
                CanTxActuatorIdValid(actuator_id) != 0u &&
                s_can_tx_cmd_cache_valid[actuator_id] != 0u)
            {
                LowStateUpdateTxQueued(actuator_id, &identities[slot]);
                if (tracked != 0u && trackEligible[slot] != 0u)
                {
                    CanTxPhysicalArm(actuator_id,
                                     &identities[slot],
                                     &ticket);
                }
            }
        }
    }
    taskEXIT_CRITICAL();
}

static void CanTxEmitRmFrames(sdlog_actuator_current_t *log)
{
    CanTxEmitRmGroupIfNeeded(1u, (uint16_t)CAN_RM_GROUP_0X200_ID, s_can_tx_can1_200, log);
    CanTxEmitRmGroupIfNeeded(1u, (uint16_t)CAN_RM_GROUP_0X1FF_ID, s_can_tx_can1_1ff, log);
    CanTxEmitRmGroupIfNeeded(2u, (uint16_t)CAN_RM_GROUP_0X200_ID, s_can_tx_can2_200, log);
    CanTxEmitRmGroupIfNeeded(2u, (uint16_t)CAN_RM_GROUP_0X1FF_ID, s_can_tx_can2_1ff, log);
}
