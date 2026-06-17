/*
 * SPDX-FileCopyrightText: 2026 ?? <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Private implementation chunk included by CanTxTask.c.
 */

static inline bool_t CanTxAllowChassis(void)
{
    return (robot_mode_allow_chassis() != 0u) ? 1 : 0;
}

// 按任务周期换算日志间隔，避免每个 CAN 周期都写 SD 卡。
static uint8_t CanTxLogDue(void)
{
    static uint16_t skip = 0u;
    const uint16_t period_ms = RobotProfileCanCommandTxPeriodMs();
    const uint16_t log_period_ms = RobotProfileCanCommandTxLogPeriodMs();
    const uint16_t divisor = (log_period_ms + period_ms - 1u) / period_ms;

    if (skip != 0u)
    {
        skip--;
        return 0u;
    }

    skip = (divisor > 1u) ? (uint16_t)(divisor - 1u) : 0u;
    return 1u;
}

static inline fp32 CanTxClampFp32(fp32 x, fp32 x_min, fp32 x_max)
{
    if (x < x_min)
    {
        return x_min;
    }
    if (x > x_max)
    {
        return x_max;
    }
    return x;
}

static inline uint8_t CanTxActuatorIdValid(MotorId id)
{
    return ((uint32_t)id < (uint32_t)MotorCount) ? 1u : 0u;
}

static uint32_t CanTxNowMs(void)
{
    return BspTimeGetTickMs();
}

static uint8_t CanTxCmdExpired(const MotorCmd *cmd, uint32_t now_ms)
{
    uint32_t age_ms;

    if (cmd == NULL || cmd->active == 0u || cmd->timeoutMs == 0u || cmd->tick == 0u)
    {
        return 0u;
    }

    if ((int32_t)(now_ms - cmd->tick) < 0)
    {
        return 0u;
    }

    age_ms = (uint32_t)(now_ms - cmd->tick);
    return (age_ms > (uint32_t)cmd->timeoutMs) ? 1u : 0u;
}

static MotorDriveState CanTxDriveStateFromFeedback(MotorId id)
{
    MotorState fb;

    if (LowStateGetMotor(id, &fb) == 0u || fb.online == 0u)
    {
        return MotorDriveStateOffline;
    }
    if (fb.driveState != (uint8_t)MotorDriveStateUnknown)
    {
        return (MotorDriveState)fb.driveState;
    }
    return MotorDriveStateEnabled;
}

static uint8_t CanTxGetCmdCopy(MotorId id, MotorCmd *out)
{
    if (out == NULL || CanTxActuatorIdValid(id) == 0u)
    {
        return 0u;
    }

    if (s_can_tx_cmd_cache_valid[id] != 0u)
    {
        *out = s_can_tx_cmd_cache[id];
        if (CanTxCmdExpired(out, CanTxNowMs()) != 0u)
        {
            (void)memset(out, 0, sizeof(*out));
            out->mode = (uint8_t)MotorModeDisable;
            s_can_tx_cmd_expired[id] = 1u;
        }
        return 1u;
    }

    if (LowCmdGetMotor(id, out) == 0u)
    {
        return 0u;
    }
    if (CanTxCmdExpired(out, CanTxNowMs()) != 0u)
    {
        (void)memset(out, 0, sizeof(*out));
        out->mode = (uint8_t)MotorModeDisable;
        s_can_tx_cmd_expired[id] = 1u;
    }
    return 1u;
}

static void CanTxClearCmdCache(void)
{
    (void)memset(s_can_tx_cmd_cache_valid, 0, sizeof(s_can_tx_cmd_cache_valid));
    (void)memset(s_can_tx_cmd_expired, 0, sizeof(s_can_tx_cmd_expired));
}

static void CanTxPrepareCmdCacheIds(void)
{
    static uint8_t ready = 0u;

    if (ready != 0u)
    {
        return;
    }

    for (uint8_t i = 0u; i < (uint8_t)MotorCount; i++)
    {
        s_can_tx_cmd_cache_ids[i] = (MotorId)i;
    }
    ready = 1u;
}

static void CanTxCacheLowcmd(void)
{
    CanTxClearCmdCache();
    CanTxPrepareCmdCacheIds();

    if (LowCmdGetMotorMany(s_can_tx_cmd_cache_ids, s_can_tx_cmd_cache, (uint8_t)MotorCount) == 0u)
    {
        return;
    }

    (void)memset(s_can_tx_cmd_cache_valid, 1, sizeof(s_can_tx_cmd_cache_valid));
}

static void CanTxForceDisabledCmd(MotorId id)
{
    if (CanTxActuatorIdValid(id) == 0u)
    {
        return;
    }

    (void)memset(&s_can_tx_cmd_cache[id], 0, sizeof(s_can_tx_cmd_cache[id]));
    s_can_tx_cmd_cache[id].active = 1u;
    s_can_tx_cmd_cache[id].mode = (uint8_t)MotorModeDisable;
    s_can_tx_cmd_cache[id].tick = CanTxNowMs();
    s_can_tx_cmd_cache_valid[id] = 1u;
}

static uint8_t CanTxBusIndex(uint8_t bus, uint8_t *out)
{
    if (out == NULL || (bus != 1u && bus != 2u))
    {
        return 0u;
    }

    *out = (uint8_t)(bus - 1u);
    return 1u;
}

static uint8_t CanTxRmGroupIndex(uint16_t group_id, uint8_t *out)
{
    if (out == NULL)
    {
        return 0u;
    }
    if (group_id == (uint16_t)CAN_RM_GROUP_0X200_ID)
    {
        *out = 0u;
        return 1u;
    }
    if (group_id == (uint16_t)CAN_RM_GROUP_0X1FF_ID)
    {
        *out = 1u;
        return 1u;
    }
    return 0u;
}

static void CanTxMarkRmGroupConfigured(uint8_t bus, uint16_t can_id)
{
    uint8_t bus_index = 0u;
    uint8_t group_index = 0u;

    if (can_id >= 0x201u && can_id <= 0x204u)
    {
        group_index = 0u;
    }
    else if (can_id >= 0x205u && can_id <= 0x208u)
    {
        group_index = 1u;
    }
    else
    {
        return;
    }

    if (CanTxBusIndex(bus, &bus_index) == 0u)
    {
        return;
    }

    s_can_tx_rm_group_configured[bus_index][group_index] = 1u;
}

static void CanTxCacheRmGroups(void)
{
    const uint8_t count = MotorRouteCount();

    (void)memset(s_can_tx_rm_group_configured, 0, sizeof(s_can_tx_rm_group_configured));

    for (uint8_t i = 0u; i < count; i++)
    {
        const MotorRoute *route = MotorRouteGet(i);
        const uint16_t can_id = (route != NULL) ? route->canId : 0u;

        if (route == NULL ||
            route->enabled == 0u ||
            route->bus == 0u ||
            route->transport != (uint8_t)MotorTransportCAN ||
            route->isRmGroup == 0u ||
            can_id == 0u)
        {
            continue;
        }

        CanTxMarkRmGroupConfigured(route->bus, can_id);
    }
}
