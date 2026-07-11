/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CONTROL_ACTUATOR_POLICY_H
#define CONTROL_ACTUATOR_POLICY_H

#include <stdint.h>

#include "ControlMgr.h"
#include "LowCmd.h"
#include "RobotConfigTypes.h"

static inline void ControlActuatorAuditAddUnresolved(ControlActuatorAudit *audit,
                                                     uint16_t count)
{
    if (audit != NULL)
    {
        audit->unresolvedOutputCount = (uint16_t)(audit->unresolvedOutputCount + count);
    }
}

static inline uint8_t ControlActuatorAuditAdd(ControlActuatorAudit *audit,
                                              uint32_t *mask,
                                              uint16_t raw_id)
{
    uint32_t bit;

    if (audit == NULL || mask == NULL)
    {
        return 0u;
    }
    if (raw_id >= (uint16_t)MotorCount || raw_id >= 32u)
    {
        audit->invalidIdCount++;
        return 0u;
    }

    bit = ((uint32_t)1u << raw_id);
    if ((*mask & bit) != 0u)
    {
        audit->duplicateMask |= bit;
    }
    *mask |= bit;
    return 1u;
}

/* 合法声明不会因为当前没有实例而消失；缺实例单独进入未解析诊断。 */
static inline void ControlActuatorAuditDeclare(ControlActuatorAudit *audit,
                                               uint32_t *mask,
                                               uint16_t raw_id,
                                               uint8_t instance_available)
{
    if (ControlActuatorAuditAdd(audit, mask, raw_id) != 0u &&
        instance_available == 0u)
    {
        ControlActuatorAuditAddUnresolved(audit, 1u);
    }
}

/* 只有 0xFFFF 是“继承角色默认值”；其他越界值都是配置错误。 */
static inline uint8_t ControlActuatorResolveSourceId(uint16_t source_id,
                                                     MotorId fallback,
                                                     MotorId *out)
{
    if (out == NULL)
    {
        return 0u;
    }

    *out = MotorCount;
    if (source_id < (uint16_t)MotorCount)
    {
        *out = (MotorId)source_id;
        return 1u;
    }
    if (source_id == UINT16_MAX && (uint32_t)fallback < (uint32_t)MotorCount)
    {
        *out = fallback;
        return 1u;
    }
    return 0u;
}

typedef struct
{
    uint16_t canId;
    uint16_t deviceId;
    uint8_t enabled;
    uint8_t transport;
    uint8_t protocol;
    uint8_t isRmGroup;
    uint8_t bus;
    uint8_t hasLimits;
    uint8_t canBusCount;
    uint8_t rs485PortCount;
} ControlActuatorRouteView;

static inline uint8_t ControlActuatorRouteRoutable(const ControlActuatorRouteView *route)
{
    if (route == NULL || route->enabled == 0u)
    {
        return 0u;
    }
    if (route->transport == (uint8_t)MotorTransportCAN)
    {
        if (route->bus < 1u || route->bus > route->canBusCount ||
            route->canId < 1u || route->canId > 0x7FFu)
        {
            return 0u;
        }
        if (route->protocol == (uint8_t)MOTOR_PROTOCOL_RM_GROUP)
        {
            return (uint8_t)(route->isRmGroup != 0u &&
                             route->canId >= 0x201u && route->canId <= 0x208u);
        }
        if (route->protocol == (uint8_t)MOTOR_PROTOCOL_DM_3MODE ||
            route->protocol == (uint8_t)MOTOR_PROTOCOL_DM_EXT_V1 ||
            route->protocol == (uint8_t)MOTOR_PROTOCOL_DM_EXT_V2)
        {
            return (uint8_t)(route->isRmGroup == 0u && route->hasLimits != 0u);
        }
        return 0u;
    }
    if (route->transport == (uint8_t)MotorTransportRS485)
    {
        if (route->isRmGroup != 0u || route->hasLimits == 0u ||
            route->bus >= route->rs485PortCount)
        {
            return 0u;
        }
        if (route->protocol == (uint8_t)MOTOR_PROTOCOL_UNITREE_RS485)
        {
            return (uint8_t)(route->deviceId >= 1u && route->deviceId <= UINT8_MAX);
        }
        if (route->protocol == (uint8_t)MOTOR_PROTOCOL_N6014B_RS485)
        {
            return (uint8_t)(route->deviceId <= 15u);
        }
        return 0u;
    }
    return 0u;
}

#endif
