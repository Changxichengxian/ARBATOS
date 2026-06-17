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

static void CanTxEmitRmGroupIfNeeded(uint8_t bus, uint16_t group_id, const int16_t frame[4])
{
    if (CanTxRmFrameHasOutput(frame) == 0u &&
        CanTxRmGroupHasConfig(bus, group_id) == 0u)
    {
        return;
    }

    CAN_cmd_rm_group(bus, group_id, frame[0], frame[1], frame[2], frame[3]);
}

static void CanTxEmitRmFrames(void)
{
    CanTxEmitRmGroupIfNeeded(1u, (uint16_t)CAN_RM_GROUP_0X200_ID, s_can_tx_can1_200);
    CanTxEmitRmGroupIfNeeded(1u, (uint16_t)CAN_RM_GROUP_0X1FF_ID, s_can_tx_can1_1ff);
    CanTxEmitRmGroupIfNeeded(2u, (uint16_t)CAN_RM_GROUP_0X200_ID, s_can_tx_can2_200);
    CanTxEmitRmGroupIfNeeded(2u, (uint16_t)CAN_RM_GROUP_0X1FF_ID, s_can_tx_can2_1ff);
}
