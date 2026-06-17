/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef MOTOR_INST_H
#define MOTOR_INST_H

#include <stdint.h>

#include "LowCmd.h"
#include "CanReceive.h"
#include "config.h"
#include "MotorModelDb.h"

struct control_controller;

#define MOTOR_INST_INVALID_DETECT_TOE 0xFFu

typedef enum
{
    MotorRoleChassis = 0u,
    MotorRoleYaw,
    MotorRoleYawUpper,
    MotorRolePitch,
    MotorRoleTrigger,
    MotorRoleFriction,
    MotorRoleArm,
} MotorRole;

typedef struct
{
    MotorId actuator_id;
    MotorRole role;
    uint8_t role_index;
    uint8_t fallback_bus;
    uint8_t DetectToe;
    uint8_t use_detect;
    const char *name;
    const motor_node_param_t *node;
    motor_measure_t *measure;
} MotorInst;

typedef struct
{
    MotorId motorId;
    MotorRole role;
    uint8_t roleIndex;
    uint8_t fallbackBus;
    uint8_t bus;
    uint8_t enabled;
    uint8_t transport;
    uint8_t protocol;
    uint8_t controlMode;
    uint8_t isRmGroup;
    uint8_t cmdCaps;
    uint8_t model;
    uint16_t canId;
    uint16_t feedbackId;
    const char *name;
    const motor_node_param_t *node;
    motor_measure_t *measure;
    const MotorModelMitLimits *mitLimits;
} MotorRoute;

typedef struct
{
    MotorId actuator_id;
    uint8_t enabled;
} MotorCurrentBind;

/*
 * Usage budget:
 * - Init/config/diagnostics code may use name lookup and resolve helpers.
 * - High-rate control loops should use actuator_id or pre-bound current bindings.
 */
void MotorInstRefresh(void);
uint8_t MotorInstCount(void);
const MotorInst *MotorInstGet(uint8_t index);
const MotorInst *MotorInstFindByMotor(MotorId id);
const MotorInst *MotorInstFindByName(const char *name);
const MotorInst *MotorInstFindFeedback(uint8_t bus, uint16_t std_id);
uint8_t MotorRouteCount(void);
const MotorRoute *MotorRouteGet(uint8_t index);
const MotorRoute *MotorRouteFindByMotor(MotorId id);
uint8_t MotorInstResolveIds(const char *const *names, uint8_t count, MotorId *out, uint8_t out_cap);
uint8_t MotorInstBindCurrent(const char *const *names,
                                            uint8_t count,
                                            MotorCurrentBind *out,
                                            uint8_t out_cap);

const char *MotorInstName(const MotorInst *inst);
MotorId MotorInstId(const MotorInst *inst);
MotorId MotorInstIdByName(const char *name);
uint8_t MotorInstBus(const MotorInst *inst);
uint8_t MotorInstEnabled(const MotorInst *inst);
const motor_node_param_t *MotorInstNode(MotorId id);
motor_measure_t *MotorInstMeasure(MotorId id);
const motor_measure_t *MotorInstMeasureConst(MotorId id);
uint8_t MotorInstModelId(MotorId id);
uint8_t MotorInstTransportId(MotorId id);
uint8_t MotorInstProtocolId(MotorId id);
uint8_t MotorInstControlModeId(MotorId id);
uint8_t MotorInstCapsId(MotorId id);
uint8_t MotorInstModeSupportedId(MotorId id, MotorMode mode);

uint8_t MotorInstClearId(MotorId id);
uint8_t MotorInstSetIds(const MotorId *ids, const MotorCmd *cmds, uint8_t count);
uint8_t MotorInstSetIdsBestEffort(const MotorId *ids, const MotorCmd *cmds, uint8_t count);
uint8_t MotorInstSetCurrentId(MotorId id, int16_t current);
uint8_t MotorInstSetStateTorqueId(MotorId id, const MotorCmd *cmd);
uint8_t MotorInstSetStateTorqueIds(const MotorId *ids, const MotorCmd *cmds, uint8_t count);
uint8_t MotorInstSetStateTorqueIdsBestEffort(const MotorId *ids,
                                                            const MotorCmd *cmds,
                                                            uint8_t count);
uint8_t MotorInstSetDisableId(MotorId id);
uint8_t MotorInstSetDampingId(MotorId id, fp32 kd, fp32 tau);
uint8_t MotorInstSetSpeedId(MotorId id, fp32 velocity, fp32 kd, fp32 torque);
uint8_t MotorInstClear(const char *name);
uint8_t MotorInstSetDisable(const char *name);
uint8_t MotorInstSetDamping(const char *name, fp32 kd, fp32 tau);
uint8_t MotorInstSetCurrent(const char *name, int16_t current);
uint8_t MotorInstSetStateTorque(const char *name, const MotorCmd *cmd);
uint8_t MotorInstSetSpeed(const char *name, fp32 velocity, fp32 kd, fp32 torque);
uint8_t MotorInstGetCmd(const char *name, MotorCmd *out);
uint8_t MotorInstGetFeedback(const char *name, MotorState *out);
uint8_t MotorInstSetCurrentIds(const MotorId *ids, const int16_t *currents, uint8_t count);
uint8_t MotorInstSetCurrentMany(const char *const *names, const int16_t *currents, uint8_t count);
uint8_t MotorInstSetCurrentIdsBestEffort(const MotorId *ids, const int16_t *currents, uint8_t count);
uint8_t MotorInstSetCurrentManyBestEffort(const char *const *names, const int16_t *currents, uint8_t count);
uint8_t MotorInstSetCurrentBindsBestEffort(const MotorCurrentBind *bindings,
                                                            const int16_t *currents,
                                                            uint8_t count);
uint8_t MotorInstGetFeedbackIds(const MotorId *ids, MotorState *out, uint8_t count);
uint8_t MotorInstGetFeedbackMany(const char *const *names, MotorState *out, uint8_t count);
uint8_t MotorInstResolveControllerOutputs(const struct control_controller *controller,
                                                  MotorId *out,
                                                  uint8_t out_cap);

#endif
