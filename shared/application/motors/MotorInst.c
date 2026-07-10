/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "MotorInst.h"
#include "MotorInstBestEffort.h"

#include "ControlMgr.h"
#include "DetectTask.h"
#include "MotorConfig.h"
#include "RobotDeviceConfig.h"
#include "RobotTaskProfile.h"

#include <string.h>

#define MOTOR_INST_FEEDBACK_LOOKUP_CAPACITY (MotorCount * 2u)

typedef struct
{
    uint8_t used;
    uint8_t bus;
    uint16_t std_id;
    const MotorInst *inst;
} MotorFeedbackSlot;

static motor_measure_t sMotorMeasure[MotorCount];
static MotorInst sMotorInst[MotorCount];
static MotorInst *sMotorById[MotorCount];
static MotorRoute sMotorRoute[MotorCount];
static MotorRoute *sRouteByMotor[MotorCount];
static MotorFeedbackSlot sMotorFbLookup[MOTOR_INST_FEEDBACK_LOOKUP_CAPACITY];
static MotorInstDiag sMotorInstDiag;
static uint8_t sMotorInstCount = 0u;
static uint8_t sMotorRouteCount = 0u;
static uint8_t sMotorInstReady = 0u;

static uint8_t MotorInstNodeCaps(const motor_node_param_t *node);

static void MotorInstAdd(MotorId actuator_id,
                               MotorRole role,
                               uint8_t role_index,
                               uint8_t fallback_bus,
                               uint8_t DetectToe,
                               uint8_t use_detect,
                               const char *name,
                               const motor_node_param_t *node,
                               motor_measure_t *measure)
{
    MotorInst *inst = NULL;

    if (sMotorInstCount >= (uint8_t)MotorCount)
    {
        return;
    }
    if ((uint32_t)actuator_id >= (uint32_t)MotorCount)
    {
        return;
    }

    inst = &sMotorInst[sMotorInstCount++];
    inst->actuator_id = actuator_id;
    inst->role = role;
    inst->role_index = role_index;
    inst->fallback_bus = fallback_bus;
    inst->DetectToe = DetectToe;
    inst->use_detect = use_detect;
    inst->name = name;
    inst->node = node;
    inst->measure = measure;
    sMotorById[actuator_id] = inst;
}

static MotorRole MotorInstRoleFromConfig(uint8_t group)
{
    switch ((RobotConfigMotorGroup)group)
    {
    case ROBOT_CONFIG_MOTOR_GROUP_CHASSIS:
        return MotorRoleChassis;
    case ROBOT_CONFIG_MOTOR_GROUP_YAW:
        return MotorRoleYaw;
    case ROBOT_CONFIG_MOTOR_GROUP_YAW_UPPER:
        return MotorRoleYawUpper;
    case ROBOT_CONFIG_MOTOR_GROUP_PITCH:
        return MotorRolePitch;
    case ROBOT_CONFIG_MOTOR_GROUP_TRIGGER:
        return MotorRoleTrigger;
    case ROBOT_CONFIG_MOTOR_GROUP_FRICTION:
        return MotorRoleFriction;
    case ROBOT_CONFIG_MOTOR_GROUP_ARM:
        return MotorRoleArm;
    default:
        return MotorRoleChassis;
    }
}

static uint8_t MotorInstDetectToeFromConfig(const RobotConfigMotorDevice *device)
{
    if (device == NULL)
    {
        return MOTOR_INST_INVALID_DETECT_TOE;
    }

    switch ((RobotConfigMotorGroup)device->group)
    {
    case ROBOT_CONFIG_MOTOR_GROUP_CHASSIS:
        return (uint8_t)(CHASSIS_MOTOR1_TOE + device->group_index);
    case ROBOT_CONFIG_MOTOR_GROUP_YAW:
        return YAW_GIMBAL_MOTOR_TOE;
    case ROBOT_CONFIG_MOTOR_GROUP_PITCH:
        return PITCH_GIMBAL_MOTOR_TOE;
    case ROBOT_CONFIG_MOTOR_GROUP_TRIGGER:
        return TRIGGER_MOTOR_TOE;
    default:
        return MOTOR_INST_INVALID_DETECT_TOE;
    }
}

static uint8_t MotorInstUseDetectFromConfig(const RobotConfigMotorDevice *device)
{
    if (device == NULL)
    {
        return 0u;
    }

    switch ((RobotConfigMotorGroup)device->group)
    {
    case ROBOT_CONFIG_MOTOR_GROUP_CHASSIS:
    case ROBOT_CONFIG_MOTOR_GROUP_YAW:
    case ROBOT_CONFIG_MOTOR_GROUP_PITCH:
    case ROBOT_CONFIG_MOTOR_GROUP_TRIGGER:
        return 1u;
    default:
        return 0u;
    }
}

static motor_measure_t *MotorInstMeasureFromId(MotorId actuator_id)
{
    if ((uint32_t)actuator_id >= (uint32_t)MotorCount)
    {
        return NULL;
    }

    return &sMotorMeasure[actuator_id];
}

static uint8_t MotorInstArmRoleEnabled(void)
{
    return (uint8_t)(RobotProfileNeedArmTask() || RobotProfileIsWheelLegMit());
}

static uint8_t MotorInstRxEnabled(const MotorInst *inst)
{
    if (MotorInstEnabled(inst) == 0u)
    {
        return 0u;
    }
    if (inst->role == MotorRoleArm && MotorInstArmRoleEnabled() == 0u)
    {
        return 0u;
    }
    return 1u;
}

static uint8_t MotorInstFeedbackHash(uint8_t bus, uint16_t std_id)
{
    return (uint8_t)(((uint16_t)(std_id ^ ((uint16_t)bus << 5u))) %
                     (uint16_t)MOTOR_INST_FEEDBACK_LOOKUP_CAPACITY);
}

static void MotorInstFeedbackClear(void)
{
    (void)memset(sMotorFbLookup, 0, sizeof(sMotorFbLookup));
}

static void MotorInstDiagClear(void)
{
    (void)memset(&sMotorInstDiag, 0, sizeof(sMotorInstDiag));
    sMotorInstDiag.last_feedback_conflict_kept = (uint8_t)MotorCount;
    sMotorInstDiag.last_feedback_conflict_dropped = (uint8_t)MotorCount;
}

static void MotorInstFeedbackRecordConflict(const MotorFeedbackSlot *entry,
                                            const MotorInst *dropped,
                                            uint8_t bus,
                                            uint16_t std_id)
{
    sMotorInstDiag.feedback_conflict_count++;
    sMotorInstDiag.last_feedback_conflict_bus = bus;
    sMotorInstDiag.last_feedback_conflict_id = std_id;
    sMotorInstDiag.last_feedback_conflict_kept =
        (entry != NULL && entry->inst != NULL) ? (uint8_t)entry->inst->actuator_id : (uint8_t)MotorCount;
    sMotorInstDiag.last_feedback_conflict_dropped =
        (dropped != NULL) ? (uint8_t)dropped->actuator_id : (uint8_t)MotorCount;
    sMotorInstDiag.last_feedback_conflict_kept_name =
        (entry != NULL && entry->inst != NULL) ? entry->inst->name : NULL;
    sMotorInstDiag.last_feedback_conflict_dropped_name =
        (dropped != NULL) ? dropped->name : NULL;
}

static void MotorInstFeedbackInsert(const MotorInst *inst)
{
    uint8_t slot = 0u;
    const uint8_t bus = (inst != NULL) ? MotorInstBus(inst) : 0u;
    const uint16_t std_id = (inst != NULL) ? MotorCfgFeedbackId(inst->node) : 0u;

    if (inst == NULL ||
        MotorInstRxEnabled(inst) == 0u ||
        bus == 0u ||
        MotorCfgTransport(inst->node) != MOTOR_TRANSPORT_CAN ||
        MotorCfgCanId(inst->node) == 0u)
    {
        return;
    }

    slot = MotorInstFeedbackHash(bus, std_id);
    for (uint8_t i = 0u; i < (uint8_t)MOTOR_INST_FEEDBACK_LOOKUP_CAPACITY; i++)
    {
        MotorFeedbackSlot *entry =
            &sMotorFbLookup[(uint8_t)((slot + i) % (uint8_t)MOTOR_INST_FEEDBACK_LOOKUP_CAPACITY)];

        if (entry->used == 0u)
        {
            entry->used = 1u;
            entry->bus = bus;
            entry->std_id = std_id;
            entry->inst = inst;
            return;
        }
        if (entry->bus == bus && entry->std_id == std_id)
        {
            if (entry->inst != inst)
            {
                MotorInstFeedbackRecordConflict(entry, inst, bus, std_id);
            }
            return;
        }
    }

    sMotorInstDiag.feedback_table_full_count++;
}

static void MotorInstFeedbackRebuild(void)
{
    MotorInstFeedbackClear();

    for (uint8_t pass = 0u; pass < 2u; pass++)
    {
        for (uint8_t i = 0u; i < sMotorInstCount; i++)
        {
            const MotorInst *inst = &sMotorInst[i];

            if ((pass == 0u && inst->role != MotorRoleArm) ||
                (pass != 0u && inst->role == MotorRoleArm))
            {
                continue;
            }

            MotorInstFeedbackInsert(inst);
        }
    }
}

static uint8_t MotorRouteNodeBus(const MotorInst *inst)
{
    if (inst == NULL || inst->node == NULL)
    {
        return 0u;
    }
    if (MotorCfgTransport(inst->node) == MOTOR_TRANSPORT_RS485)
    {
        return inst->node->rs485_port;
    }
    return MotorCfgCanBus(inst->fallback_bus, inst->node);
}

static void MotorRouteAdd(const MotorInst *inst)
{
    MotorRoute *route;

    if (inst == NULL ||
        inst->node == NULL ||
        MotorInstEnabled(inst) == 0u ||
        sMotorRouteCount >= (uint8_t)MotorCount ||
        (uint32_t)inst->actuator_id >= (uint32_t)MotorCount)
    {
        return;
    }

    route = &sMotorRoute[sMotorRouteCount++];
    (void)memset(route, 0, sizeof(*route));
    route->motorId = inst->actuator_id;
    route->role = inst->role;
    route->roleIndex = inst->role_index;
    route->fallbackBus = inst->fallback_bus;
    route->bus = MotorRouteNodeBus(inst);
    route->enabled = 1u;
    route->transport = (uint8_t)MotorCfgTransport(inst->node);
    route->protocol = (uint8_t)MotorCfgProtocol(inst->node);
    route->controlMode = (uint8_t)MotorCfgControlMode(inst->node);
    route->isRmGroup = MotorCfgIsRmGroupProtocol(inst->node);
    route->cmdCaps = MotorInstNodeCaps(inst->node);
    route->model = (uint8_t)inst->node->model;
    route->canId = MotorCfgCanId(inst->node);
    route->feedbackId = MotorCfgFeedbackId(inst->node);
    route->name = inst->name;
    route->node = inst->node;
    route->measure = inst->measure;
    route->mitLimits = MotorCfgMitLimits(inst->node);
    sRouteByMotor[route->motorId] = route;
}

static void MotorRouteRebuild(void)
{
    sMotorRouteCount = 0u;
    (void)memset(sMotorRoute, 0, sizeof(sMotorRoute));
    (void)memset(sRouteByMotor, 0, sizeof(sRouteByMotor));

    for (uint8_t i = 0u; i < sMotorInstCount; i++)
    {
        MotorRouteAdd(&sMotorInst[i]);
    }
}

static const MotorInst *MotorInstFeedbackGet(uint8_t bus, uint16_t std_id)
{
    const uint8_t slot = MotorInstFeedbackHash(bus, std_id);

    for (uint8_t i = 0u; i < (uint8_t)MOTOR_INST_FEEDBACK_LOOKUP_CAPACITY; i++)
    {
        const MotorFeedbackSlot *entry =
            &sMotorFbLookup[(uint8_t)((slot + i) % (uint8_t)MOTOR_INST_FEEDBACK_LOOKUP_CAPACITY)];

        if (entry->used == 0u)
        {
            return NULL;
        }
        if (entry->bus == bus && entry->std_id == std_id)
        {
            return entry->inst;
        }
    }

    return NULL;
}

static void MotorInstEnsure(void)
{
    if (sMotorInstReady == 0u)
    {
        MotorInstRefresh();
    }
}

void MotorInstRefresh(void)
{
    const uint8_t device_count = RobotConfigMotorDeviceCount();

    sMotorInstCount = 0u;
    (void)memset(sMotorInst, 0, sizeof(sMotorInst));
    (void)memset(sMotorById, 0, sizeof(sMotorById));
    MotorInstFeedbackClear();
    MotorInstDiagClear();

    for (uint8_t i = 0u; i < device_count; i++)
    {
        RobotConfigMotorDevice device;
        motor_measure_t *measure;

        if (RobotConfigMotorDeviceGet(i, &device) == 0u)
        {
            continue;
        }

        measure = MotorInstMeasureFromId(device.actuator_id);
        if (measure == NULL)
        {
            continue;
        }

        MotorInstAdd(device.actuator_id,
                           MotorInstRoleFromConfig(device.group),
                           device.group_index,
                           device.fallback_bus,
                           MotorInstDetectToeFromConfig(&device),
                           MotorInstUseDetectFromConfig(&device),
                           device.name,
                           device.node,
                           measure);
    }

    MotorInstFeedbackRebuild();
    MotorRouteRebuild();
    sMotorInstReady = 1u;
}

uint8_t MotorInstCount(void)
{
    MotorInstEnsure();
    return sMotorInstCount;
}

const MotorInst *MotorInstGet(uint8_t index)
{
    MotorInstEnsure();
    if (index >= sMotorInstCount)
    {
        return NULL;
    }
    return &sMotorInst[index];
}

const MotorInst *MotorInstFindByMotor(MotorId id)
{
    MotorInstEnsure();
    if ((uint32_t)id >= (uint32_t)MotorCount)
    {
        return NULL;
    }
    return sMotorById[id];
}

const MotorInst *MotorInstFindByName(const char *name)
{
    uint8_t i = 0u;

    if (name == NULL)
    {
        return NULL;
    }

    MotorInstEnsure();
    for (i = 0u; i < sMotorInstCount; i++)
    {
        if (sMotorInst[i].name != NULL &&
            strcmp(sMotorInst[i].name, name) == 0)
        {
            return &sMotorInst[i];
        }
    }
    return NULL;
}

uint8_t MotorInstGetDiag(MotorInstDiag *out)
{
    if (out == NULL)
    {
        return 0u;
    }

    MotorInstEnsure();
    *out = sMotorInstDiag;
    return 1u;
}

uint8_t MotorRouteCount(void)
{
    MotorInstEnsure();
    return sMotorRouteCount;
}

const MotorRoute *MotorRouteGet(uint8_t index)
{
    MotorInstEnsure();
    if (index >= sMotorRouteCount)
    {
        return NULL;
    }
    return &sMotorRoute[index];
}

const MotorRoute *MotorRouteFindByMotor(MotorId id)
{
    MotorInstEnsure();
    if ((uint32_t)id >= (uint32_t)MotorCount)
    {
        return NULL;
    }
    return sRouteByMotor[id];
}

uint8_t MotorInstResolveIds(const char *const *names, uint8_t count, MotorId *out, uint8_t out_cap)
{
    uint8_t resolved = 0u;

    if (names == NULL || out == NULL || count > out_cap)
    {
        return 0u;
    }

    for (uint8_t i = 0u; i < count; i++)
    {
        RobotConfigMotorDevice device;

        if (RobotConfigMotorDeviceFindByName(names[i], &device) != 0u)
        {
            out[i] = device.actuator_id;
            resolved++;
        }
        else
        {
            out[i] = MotorCount;
        }
    }

    return resolved;
}

uint8_t MotorInstBindCurrent(const char *const *names,
                                            uint8_t count,
                                            MotorCurrentBind *out,
                                            uint8_t out_cap)
{
    uint8_t bound = 0u;

    if (names == NULL || out == NULL || count > out_cap)
    {
        return 0u;
    }

    for (uint8_t i = 0u; i < count; i++)
    {
        const MotorInst *inst = MotorInstFindByName(names[i]);

        out[i].actuator_id = MotorCount;
        out[i].enabled = 0u;

        if (inst == NULL || MotorInstEnabled(inst) == 0u)
        {
            continue;
        }

        out[i].actuator_id = inst->actuator_id;
        out[i].enabled = 1u;
        bound++;
    }

    return bound;
}

const char *MotorInstName(const MotorInst *inst)
{
    return (inst != NULL) ? inst->name : NULL;
}

MotorId MotorInstId(const MotorInst *inst)
{
    return (inst != NULL) ? inst->actuator_id : MotorCount;
}

MotorId MotorInstIdByName(const char *name)
{
    return MotorInstId(MotorInstFindByName(name));
}

uint8_t MotorInstBus(const MotorInst *inst)
{
    if (inst == NULL || inst->node == NULL)
    {
        return 0u;
    }
    return MotorCfgCanBus(inst->fallback_bus, inst->node);
}

uint8_t MotorInstEnabled(const MotorInst *inst)
{
    if (inst == NULL || inst->node == NULL)
    {
        return 0u;
    }
    return (MotorCfgNodeId(inst->node) != 0u) ? 1u : 0u;
}

static uint8_t MotorInstResolveCmdTarget(const char *name, MotorId *out)
{
    const MotorInst *inst = MotorInstFindByName(name);

    if (inst == NULL || out == NULL || MotorInstEnabled(inst) == 0u)
    {
        return 0u;
    }

    *out = inst->actuator_id;
    return 1u;
}

static uint8_t MotorInstCmdEnabled(MotorId id)
{
    const MotorInst *inst = MotorInstFindByMotor(id);

    return (uint8_t)(inst != NULL && MotorInstEnabled(inst) != 0u);
}

static uint8_t MotorInstNodeCaps(const motor_node_param_t *node)
{
    const MotorModelDbEntry *entry;
    uint8_t caps = (uint8_t)MotorCmdCapCurrent;
    const motor_transport_e transport = MotorCfgTransport(node);
    const motor_protocol_e protocol = MotorCfgProtocol(node);

    if (node == NULL || MotorCfgNodeId(node) == 0u)
    {
        return 0u;
    }

    caps |= (uint8_t)MotorCmdCapFeedback;
    if (transport == MOTOR_TRANSPORT_CAN && protocol == MOTOR_PROTOCOL_RM_GROUP)
    {
        return (uint8_t)(caps |
                         (uint8_t)MotorCmdCapStateTorque |
                         (uint8_t)MotorCmdCapPosVel |
                         (uint8_t)MotorCmdCapSpeed |
                         (uint8_t)MotorCmdCapForcePos);
    }

    entry = MotorCfgModelDb(node->model);
    if (entry == NULL)
    {
        return caps;
    }
    if ((entry->caps & (uint8_t)MOTOR_MODEL_CAP_MIT) != 0u)
    {
        caps |= (uint8_t)MotorCmdCapStateTorque;
    }
    if ((entry->caps & (uint8_t)MOTOR_MODEL_CAP_POS_VEL) != 0u)
    {
        caps |= (uint8_t)MotorCmdCapPosVel;
    }
    if ((entry->caps & (uint8_t)MOTOR_MODEL_CAP_SPEED) != 0u)
    {
        caps |= (uint8_t)MotorCmdCapSpeed;
    }
    if ((entry->caps & (uint8_t)MOTOR_MODEL_CAP_FORCE_POS) != 0u)
    {
        caps |= (uint8_t)MotorCmdCapForcePos;
    }

    return caps;
}

uint8_t MotorInstModelId(MotorId id)
{
    const motor_node_param_t *node = MotorInstNode(id);

    return (node != NULL) ? (uint8_t)node->model : 0xFFu;
}

uint8_t MotorInstTransportId(MotorId id)
{
    const motor_node_param_t *node = MotorInstNode(id);

    return (node != NULL) ? (uint8_t)MotorCfgTransport(node) : 0u;
}

uint8_t MotorInstProtocolId(MotorId id)
{
    const motor_node_param_t *node = MotorInstNode(id);

    return (node != NULL) ? (uint8_t)MotorCfgProtocol(node) : 0u;
}

uint8_t MotorInstControlModeId(MotorId id)
{
    const motor_node_param_t *node = MotorInstNode(id);

    return (node != NULL) ? (uint8_t)MotorCfgControlMode(node) : 0u;
}

uint8_t MotorInstCapsId(MotorId id)
{
    const MotorInst *inst = MotorInstFindByMotor(id);

    if (inst == NULL || MotorInstEnabled(inst) == 0u)
    {
        return 0u;
    }

    return MotorInstNodeCaps(inst->node);
}

uint8_t MotorInstModeSupportedId(MotorId id, MotorMode mode)
{
    const uint8_t caps = MotorInstCapsId(id);

    if (mode == MotorModeNone)
    {
        return (MotorInstFindByMotor(id) != NULL) ? 1u : 0u;
    }
    if (MotorModeKnown((uint8_t)mode) == 0u || caps == 0u)
    {
        return 0u;
    }

    switch (mode)
    {
    case MotorModeDisable:
        return 1u;
    case MotorModeDamping:
        return (uint8_t)((caps & (uint8_t)MotorCmdCapFeedback) != 0u);
    case MotorModeCurrent:
        return (uint8_t)((caps & (uint8_t)MotorCmdCapCurrent) != 0u);
    case MotorModeStateTorque:
        return (uint8_t)((caps & (uint8_t)MotorCmdCapStateTorque) != 0u);
    case MotorModePosVel:
        return (uint8_t)((caps & (uint8_t)MotorCmdCapPosVel) != 0u);
    case MotorModeSpeed:
        return (uint8_t)((caps & (uint8_t)MotorCmdCapSpeed) != 0u);
    case MotorModeForcePos:
        return (uint8_t)((caps & (uint8_t)MotorCmdCapForcePos) != 0u);
    default:
        return 0u;
    }
}

uint8_t MotorInstClearId(MotorId id)
{
    if ((uint32_t)id >= (uint32_t)MotorCount ||
        MotorInstFindByMotor(id) == NULL)
    {
        return 0u;
    }

    LowCmdClear(id);
    return 1u;
}

static uint8_t MotorInstControlIdsValid(const MotorId *ids, uint8_t count)
{
    if (count > (uint8_t)MotorCount || (count != 0u && ids == NULL))
    {
        return 0u;
    }

    for (uint8_t i = 0u; i < count; i++)
    {
        if ((uint32_t)ids[i] >= (uint32_t)MotorCount ||
            MotorInstFindByMotor(ids[i]) == NULL)
        {
            return 0u;
        }
        for (uint8_t j = 0u; j < i; j++)
        {
            if (ids[i] == ids[j])
            {
                return 0u;
            }
        }
    }

    return 1u;
}

uint8_t MotorInstClearIds(const MotorId *ids, uint8_t count)
{
    if (MotorInstControlIdsValid(ids, count) == 0u)
    {
        return 0u;
    }

    return LowCmdClearManyFrom(ids, count, (uint16_t)LOWCMD_WRITER_SAFETY);
}

uint8_t MotorInstInhibitIdsFrom(const MotorId *ids, uint8_t count, uint16_t writer)
{
    if (MotorInstControlIdsValid(ids, count) == 0u)
    {
        return 0u;
    }

    return LowCmdInhibitManyFrom(ids, count, writer);
}

uint8_t MotorInstInhibitIds(const MotorId *ids, uint8_t count)
{
    return MotorInstInhibitIdsFrom(ids, count, (uint16_t)LOWCMD_WRITER_SAFETY);
}

uint8_t MotorInstReleaseInhibitIdsFrom(const MotorId *ids, uint8_t count, uint16_t writer)
{
    if (MotorInstControlIdsValid(ids, count) == 0u)
    {
        return 0u;
    }

    return LowCmdReleaseInhibitManyFrom(ids, count, writer);
}

uint8_t MotorInstReleaseInhibitIds(const MotorId *ids, uint8_t count)
{
    return MotorInstReleaseInhibitIdsFrom(ids, count, (uint16_t)LOWCMD_WRITER_SAFETY);
}

uint8_t MotorInstSetIds(const MotorId *ids, const MotorCmd *cmds, uint8_t count)
{
    if (count > (uint8_t)MotorCount)
    {
        return 0u;
    }
    if (count != 0u && (ids == NULL || cmds == NULL))
    {
        return 0u;
    }

    for (uint8_t i = 0u; i < count; i++)
    {
        if (MotorInstCmdEnabled(ids[i]) == 0u ||
            MotorInstModeSupportedId(ids[i], (MotorMode)cmds[i].mode) == 0u)
        {
            return 0u;
        }
    }

    return LowCmdSetMotorMany(ids, cmds, count);
}

uint8_t MotorInstSetIdsBestEffort(const MotorId *ids, const MotorCmd *cmds, uint8_t count)
{
    MotorId writable_ids[MotorCount];
    MotorCmd writable_cmds[MotorCount];
    uint8_t written = 0u;

    if (ids == NULL || cmds == NULL || count > (uint8_t)MotorCount)
    {
        return 0u;
    }

    for (uint8_t i = 0u; i < count; i++)
    {
        if (MotorInstCmdEnabled(ids[i]) == 0u ||
            MotorInstModeSupportedId(ids[i], (MotorMode)cmds[i].mode) == 0u)
        {
            continue;
        }

        writable_ids[written] = ids[i];
        writable_cmds[written] = cmds[i];
        written++;
    }

    return MotorInstLowCmdSetBestEffort(writable_ids, writable_cmds, written);
}

uint8_t MotorInstSetCurrentId(MotorId id, int16_t current)
{
    if (MotorInstCmdEnabled(id) == 0u)
    {
        return 0u;
    }

    LowCmdSetCurrent(id, current);
    return 1u;
}

uint8_t MotorInstSetStateTorqueId(MotorId id, const MotorCmd *cmd)
{
    MotorCmd tmp;

    if (cmd == NULL)
    {
        return 0u;
    }

    tmp = *cmd;
    tmp.active = 1u;
    tmp.mode = (uint8_t)MotorModeStateTorque;
    return MotorInstSetIds(&id, &tmp, 1u);
}

uint8_t MotorInstSetStateTorqueIds(const MotorId *ids, const MotorCmd *cmds, uint8_t count)
{
    MotorCmd prepared[MotorCount];

    if (count > (uint8_t)MotorCount)
    {
        return 0u;
    }
    if (count != 0u && (ids == NULL || cmds == NULL))
    {
        return 0u;
    }

    for (uint8_t i = 0u; i < count; i++)
    {
        prepared[i] = cmds[i];
        prepared[i].active = 1u;
        prepared[i].mode = (uint8_t)MotorModeStateTorque;
    }

    return MotorInstSetIds(ids, prepared, count);
}

uint8_t MotorInstSetStateTorqueIdsBestEffort(const MotorId *ids,
                                                            const MotorCmd *cmds,
                                                            uint8_t count)
{
    MotorCmd prepared[MotorCount];

    if (count > (uint8_t)MotorCount)
    {
        return 0u;
    }
    if (count != 0u && (ids == NULL || cmds == NULL))
    {
        return 0u;
    }

    for (uint8_t i = 0u; i < count; i++)
    {
        prepared[i] = cmds[i];
        prepared[i].active = 1u;
        prepared[i].mode = (uint8_t)MotorModeStateTorque;
    }

    return MotorInstSetIdsBestEffort(ids, prepared, count);
}

uint8_t MotorInstSetDisableId(MotorId id)
{
    MotorCmd cmd;

    (void)memset(&cmd, 0, sizeof(cmd));
    cmd.active = 1u;
    cmd.mode = (uint8_t)MotorModeDisable;
    return MotorInstSetIds(&id, &cmd, 1u);
}

uint8_t MotorInstSetDampingId(MotorId id, fp32 kd, fp32 tau)
{
    MotorCmd cmd;

    (void)memset(&cmd, 0, sizeof(cmd));
    cmd.active = 1u;
    cmd.mode = (uint8_t)MotorModeDamping;
    cmd.dq = 0.0f;
    cmd.kd = kd;
    cmd.tau = tau;
    return MotorInstSetIds(&id, &cmd, 1u);
}

uint8_t MotorInstSetSpeedId(MotorId id, fp32 velocity, fp32 kd, fp32 torque)
{
    MotorCmd cmd;

    (void)memset(&cmd, 0, sizeof(cmd));
    cmd.active = 1u;
    cmd.mode = (uint8_t)MotorModeSpeed;
    cmd.dq = velocity;
    cmd.kd = kd;
    cmd.tau = torque;
    return MotorInstSetIds(&id, &cmd, 1u);
}

uint8_t MotorInstClear(const char *name)
{
    return MotorInstClearId(MotorInstIdByName(name));
}

uint8_t MotorInstSetDisable(const char *name)
{
    MotorId id;

    if (MotorInstResolveCmdTarget(name, &id) == 0u)
    {
        return 0u;
    }

    return MotorInstSetDisableId(id);
}

uint8_t MotorInstSetDamping(const char *name, fp32 kd, fp32 tau)
{
    MotorId id;

    if (MotorInstResolveCmdTarget(name, &id) == 0u)
    {
        return 0u;
    }

    return MotorInstSetDampingId(id, kd, tau);
}

uint8_t MotorInstSetCurrent(const char *name, int16_t current)
{
    MotorId id;

    if (MotorInstResolveCmdTarget(name, &id) == 0u)
    {
        return 0u;
    }

    return MotorInstSetCurrentId(id, current);
}

uint8_t MotorInstSetStateTorque(const char *name, const MotorCmd *cmd)
{
    MotorId id;

    if (MotorInstResolveCmdTarget(name, &id) == 0u)
    {
        return 0u;
    }

    return MotorInstSetStateTorqueId(id, cmd);
}

uint8_t MotorInstSetSpeed(const char *name, fp32 velocity, fp32 kd, fp32 torque)
{
    MotorId id;

    if (MotorInstResolveCmdTarget(name, &id) == 0u)
    {
        return 0u;
    }

    return MotorInstSetSpeedId(id, velocity, kd, torque);
}

uint8_t MotorInstGetCmd(const char *name, MotorCmd *out)
{
    const MotorId id = MotorInstIdByName(name);

    if (id == MotorCount || out == NULL)
    {
        return 0u;
    }

    return LowCmdGetMotor(id, out);
}

uint8_t MotorInstGetFeedback(const char *name, MotorState *out)
{
    const MotorId id = MotorInstIdByName(name);

    if (id == MotorCount || out == NULL)
    {
        return 0u;
    }

    return LowStateGetMotor(id, out);
}

uint8_t MotorInstSetCurrentIds(const MotorId *ids, const int16_t *currents, uint8_t count)
{
    if (ids == NULL || currents == NULL)
    {
        return 0u;
    }

    for (uint8_t i = 0u; i < count; i++)
    {
        if (MotorInstCmdEnabled(ids[i]) == 0u)
        {
            return 0u;
        }
    }

    return LowCmdSetCurrentMany(ids, currents, count);
}

uint8_t MotorInstSetCurrentMany(const char *const *names, const int16_t *currents, uint8_t count)
{
    MotorId ids[MotorCount];

    if (count > (uint8_t)MotorCount)
    {
        return 0u;
    }
    if (MotorInstResolveIds(names, count, ids, (uint8_t)MotorCount) != count)
    {
        return 0u;
    }

    return MotorInstSetCurrentIds(ids, currents, count);
}

uint8_t MotorInstSetCurrentIdsBestEffort(const MotorId *ids, const int16_t *currents, uint8_t count)
{
    MotorId writable_ids[MotorCount];
    int16_t writable_currents[MotorCount];
    uint8_t written = 0u;

    if (ids == NULL || currents == NULL || count > (uint8_t)MotorCount)
    {
        return 0u;
    }

    for (uint8_t i = 0u; i < count; i++)
    {
        if (MotorInstCmdEnabled(ids[i]) == 0u)
        {
            continue;
        }

        writable_ids[written] = ids[i];
        writable_currents[written] = currents[i];
        written++;
    }

    return MotorInstLowCmdSetCurrentBestEffort(writable_ids, writable_currents, written);
}

uint8_t MotorInstSetCurrentManyBestEffort(const char *const *names, const int16_t *currents, uint8_t count)
{
    MotorId writable_ids[MotorCount];
    int16_t writable_currents[MotorCount];
    uint8_t written = 0u;

    if (names == NULL || currents == NULL || count > (uint8_t)MotorCount)
    {
        return 0u;
    }

    for (uint8_t i = 0u; i < count; i++)
    {
        MotorId id;

        if (MotorInstResolveCmdTarget(names[i], &id) == 0u)
        {
            continue;
        }

        writable_ids[written] = id;
        writable_currents[written] = currents[i];
        written++;
    }

    return MotorInstLowCmdSetCurrentBestEffort(writable_ids, writable_currents, written);
}

uint8_t MotorInstSetCurrentBindsBestEffort(const MotorCurrentBind *bindings,
                                                            const int16_t *currents,
                                                            uint8_t count)
{
    MotorId writable_ids[MotorCount];
    int16_t writable_currents[MotorCount];
    uint8_t written = 0u;

    if (bindings == NULL || currents == NULL || count > (uint8_t)MotorCount)
    {
        return 0u;
    }

    for (uint8_t i = 0u; i < count; i++)
    {
        if (bindings[i].enabled == 0u ||
            (uint32_t)bindings[i].actuator_id >= (uint32_t)MotorCount)
        {
            continue;
        }

        writable_ids[written] = bindings[i].actuator_id;
        writable_currents[written] = currents[i];
        written++;
    }

    return MotorInstLowCmdSetCurrentBestEffort(writable_ids, writable_currents, written);
}

uint8_t MotorInstGetFeedbackIds(const MotorId *ids, MotorState *out, uint8_t count)
{
    if (ids == NULL || out == NULL)
    {
        return 0u;
    }

    for (uint8_t i = 0u; i < count; i++)
    {
        if ((uint32_t)ids[i] >= (uint32_t)MotorCount)
        {
            return 0u;
        }
    }

    return LowStateGetMotorMany(ids, out, count);
}

uint8_t MotorInstGetFeedbackMany(const char *const *names, MotorState *out, uint8_t count)
{
    MotorId ids[MotorCount];

    if (count > (uint8_t)MotorCount)
    {
        return 0u;
    }
    if (MotorInstResolveIds(names, count, ids, (uint8_t)MotorCount) != count)
    {
        return 0u;
    }

    return MotorInstGetFeedbackIds(ids, out, count);
}

uint8_t MotorInstResolveControllerOutputs(const struct control_controller *controller,
                                                  MotorId *out,
                                                  uint8_t out_cap)
{
    if (controller == NULL)
    {
        return 0u;
    }

    return MotorInstResolveIds(controller->meta.outputs,
                                              controller->meta.output_count,
                                              out,
                                              out_cap);
}

const MotorInst *MotorInstFindFeedback(uint8_t bus, uint16_t std_id)
{
    MotorInstEnsure();
    return MotorInstFeedbackGet(bus, std_id);
}

const motor_node_param_t *MotorInstNode(MotorId id)
{
    const MotorInst *inst = MotorInstFindByMotor(id);
    return (inst != NULL) ? inst->node : NULL;
}

motor_measure_t *MotorInstMeasure(MotorId id)
{
    const MotorInst *inst = MotorInstFindByMotor(id);
    return (inst != NULL) ? inst->measure : NULL;
}

const motor_measure_t *MotorInstMeasureConst(MotorId id)
{
    return MotorInstMeasure(id);
}
