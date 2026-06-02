/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef LOWCMD_H
#define LOWCMD_H

#include <stdint.h>

#include "types.h"

typedef enum
{
    Motor0 = 0u,
    Motor1,
    Motor2,
    Motor3,
    Motor4,
    Motor5,
    Motor6,
    Motor7,
    Motor8,
    Motor9,
    Motor10,
    Motor11,
    Motor12,
    Motor13,
    Motor14,
    Motor15,
    Motor16,
    Motor17,
    MotorCount
} MotorId;

typedef enum
{
    MotorModeNone = 0u,
    MotorModeDisable,
    MotorModeDamping,
    MotorModeCurrent,
    MotorModeStateTorque, // q/dq/kp/kd/tau, transport-independent
    MotorModePosVel,
    MotorModeSpeed,
    MotorModeForcePos,
    MotorModeCount,
} MotorMode;

typedef enum
{
    MotorCmdCapCurrent = 1u << 0,
    MotorCmdCapStateTorque = 1u << 1,
    MotorCmdCapPosVel = 1u << 2,
    MotorCmdCapSpeed = 1u << 3,
    MotorCmdCapForcePos = 1u << 4,
    MotorCmdCapFeedback = 1u << 5,
} MotorCmdCap;

typedef enum
{
    MotorTransportNone = 0u,
    MotorTransportCAN,
    MotorTransportRS485,
} MotorTransport;

typedef struct
{
    uint8_t active;
    uint8_t mode; // MotorMode
    uint16_t timeoutMs;
    uint16_t writer;
    uint32_t seq;
    uint32_t tick;
    int16_t current;
    fp32 q;
    fp32 dq;
    fp32 kp;
    fp32 kd;
    fp32 tau;
} MotorCmd;

typedef struct
{
    uint8_t online;
    uint8_t bus;
    uint8_t rxDlc;
    uint8_t rxData0;   // raw first byte from the last feedback frame when available
    uint8_t transport; // MotorTransport
    uint8_t motorId;   // MIT feedback motor id when available
    uint8_t state;     // MIT feedback state when available
    uint16_t rxId;
    uint32_t rxCount;
    uint32_t lastRxTick;
    fp32 q;
    fp32 dq;
    fp32 tauEst;
    uint16_t ecd;
    int16_t speedRpm;
    int16_t current;
    uint8_t temperature;
} MotorState;

typedef struct
{
    uint32_t seq;
    uint32_t tick;
    MotorCmd motorCmd[MotorCount];
} LowCmd;

typedef struct
{
    uint32_t tick;
    MotorState motorState[MotorCount];
} LowState;

static inline MotorId MotorIdRange(MotorId first, uint8_t index, uint8_t count)
{
    if (index >= count || (uint32_t)first >= (uint32_t)MotorCount)
    {
        return MotorCount;
    }
    if (((uint32_t)first + (uint32_t)index) >= (uint32_t)MotorCount)
    {
        return MotorCount;
    }
    return (MotorId)((uint32_t)first + (uint32_t)index);
}

static inline uint8_t MotorModeKnown(uint8_t mode)
{
    return (mode < (uint8_t)MotorModeCount) ? 1u : 0u;
}

void LowCmdClearAll(void);
void LowCmdClear(MotorId id);
uint8_t LowCmdSetMotor(MotorId id, const MotorCmd *cmd);
uint8_t LowCmdSetMotorMany(const MotorId *ids, const MotorCmd *cmds, uint8_t count);
const char *MotorModeName(MotorMode mode);
uint32_t LowCmdSeq(void);
uint8_t LowCmdGet(LowCmd *out);
void LowCmdSetCurrent(MotorId id, int16_t current);
uint8_t LowCmdSetCurrentMany(const MotorId *ids, const int16_t *currents, uint8_t count);
int16_t LowCmdGetCurrent(MotorId id);
uint8_t LowCmdGetCurrentMany(const MotorId *ids, int16_t *out, uint8_t count);
void LowCmdSetStateTorque(MotorId id, const MotorCmd *cmd);
void LowCmdSetSpeed(MotorId id, fp32 velocity, fp32 kd, fp32 torque);
uint8_t LowCmdGetMotor(MotorId id, MotorCmd *out);
uint8_t LowCmdGetMotorMany(const MotorId *ids, MotorCmd *out, uint8_t count);
const MotorCmd *LowCmdGetMotorPtr(MotorId id);

void LowStateClearAll(void);
void LowStateUpdateMotor(MotorId id, const MotorState *feedback);
uint8_t LowStateGet(LowState *out);
uint8_t LowStateGetMotor(MotorId id, MotorState *out);
uint8_t LowStateGetMotorMany(const MotorId *ids, MotorState *out, uint8_t count);
const MotorState *LowStateGetMotorPtr(MotorId id);

#endif
