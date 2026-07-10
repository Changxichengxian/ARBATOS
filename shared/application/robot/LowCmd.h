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

#include "Types.h"

#define LOWCMD_DEFAULT_TIMEOUT_MS 100u
#define LOWCMD_PRIORITY_HOLD_MS 100u

typedef enum
{
    LOWCMD_WRITER_NONE = 0u,
    LOWCMD_WRITER_CONTROL = 10u,
    LOWCMD_WRITER_MANUAL = 20u,
    LOWCMD_WRITER_HOST = 30u,
    LOWCMD_WRITER_SAFETY = 240u,
    LOWCMD_WRITER_FAULT = 255u,
} LowCmdWriter;

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
    MotorDriveStateUnknown = 0u,
    MotorDriveStateOffline,
    MotorDriveStateDisabled,
    MotorDriveStateReady,
    MotorDriveStateEnabled,
    MotorDriveStateFault,
} MotorDriveState;

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

typedef enum
{
    MotorAppliedFlagLimited = 1u << 0,
    MotorAppliedFlagForceDisabled = 1u << 1,
    MotorAppliedFlagCmdExpired = 1u << 2,
    MotorAppliedFlagSkipped = 1u << 3,
} MotorAppliedFlag;

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
    uint8_t driveState; // MotorDriveState
    uint16_t rxId;
    uint16_t lastEcd;
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
    uint8_t active;
    uint8_t mode;       // MotorMode
    uint8_t driveState; // MotorDriveState
    uint8_t flags;      // MotorAppliedFlag
    uint8_t bus;
    uint8_t transport; // MotorTransport
    uint8_t protocol;
    uint8_t reserved0;
    uint16_t txId;
    uint32_t tick;
    int16_t current;
    fp32 q;
    fp32 dq;
    fp32 kp;
    fp32 kd;
    fp32 tau;
} MotorApplied;

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
    MotorApplied motorApplied[MotorCount];
} LowState;

typedef struct
{
    uint32_t seq;
    uint32_t rejected_count;
    uint32_t emergency_stop_count;
    uint32_t inhibit_acquire_count;
    uint32_t inhibit_release_count;
    uint32_t inhibit_mask;
    uint32_t snapshot_retry_count;
    uint32_t snapshot_fallback_count;
    uint32_t last_reject_tick;
    uint16_t last_reject_writer;
    uint16_t last_reject_owner;
    uint16_t emergency_writer;
    uint8_t emergency_active;
    uint8_t reserved0;
} LowCmdDiag;

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
uint8_t LowCmdClearManyFrom(const MotorId *ids, uint8_t count, uint16_t writer);
/*
 * 原子清空新获取或升级禁写的命令，并持续阻止更低优先级 writer。
 * 同 writer 重复获取是无副作用成功；更高 writer 可升级 owner；更低 writer 整批失败。
 * 禁写 owner 及更高 writer 仍可写命令，写命令不会自动释放禁写。
 */
uint8_t LowCmdInhibitManyFrom(const MotorId *ids, uint8_t count, uint16_t writer);
/*
 * 同级或更高 writer 可释放；未禁写的电机视为已经释放。
 * 任一电机由更高 writer 禁写时整批不变，释放不改当前命令及命令序号。
 */
uint8_t LowCmdReleaseInhibitManyFrom(const MotorId *ids, uint8_t count, uint16_t writer);
uint8_t LowCmdGetInhibitWriter(MotorId id, uint16_t *out);
uint8_t LowCmdSetMotor(MotorId id, const MotorCmd *cmd);
uint8_t LowCmdSetMotorMany(const MotorId *ids, const MotorCmd *cmds, uint8_t count);
/* writer 以 API 参数为准；MotorCmd.writer 是发布后的只读归属信息，不能由载荷提权。 */
uint8_t LowCmdSetMotorFrom(MotorId id, const MotorCmd *cmd, uint16_t writer);
uint8_t LowCmdSetMotorManyFrom(const MotorId *ids, const MotorCmd *cmds, uint8_t count, uint16_t writer);
const char *MotorModeName(MotorMode mode);
uint32_t LowCmdSeq(void);
uint8_t LowCmdGet(LowCmd *out);
void LowCmdSetDisable(MotorId id);
void LowCmdSetDisableFrom(MotorId id, uint16_t writer);
void LowCmdSetDamping(MotorId id, fp32 kd, fp32 tau);
void LowCmdSetCurrent(MotorId id, int16_t current);
uint8_t LowCmdSetCurrentMany(const MotorId *ids, const int16_t *currents, uint8_t count);
uint8_t LowCmdSetCurrentManyFrom(const MotorId *ids, const int16_t *currents, uint8_t count, uint16_t writer);
int16_t LowCmdGetCurrent(MotorId id);
uint8_t LowCmdGetCurrentMany(const MotorId *ids, int16_t *out, uint8_t count);
void LowCmdSetStateTorque(MotorId id, const MotorCmd *cmd);
void LowCmdSetSpeed(MotorId id, fp32 velocity, fp32 kd, fp32 torque);
uint8_t LowCmdGetMotor(MotorId id, MotorCmd *out);
uint8_t LowCmdGetMotorMany(const MotorId *ids, MotorCmd *out, uint8_t count);
/* 兼容调试接口：返回快照缓存，新代码优先用 LowCmdGetMotor。 */
const MotorCmd *LowCmdGetMotorPtr(MotorId id);
uint8_t LowCmdEnterEmergencyStop(uint16_t writer);
uint8_t LowCmdClearEmergencyStop(uint16_t writer);
uint8_t LowCmdEmergencyActive(void);
uint8_t LowCmdGetDiag(LowCmdDiag *out);

void LowStateClearAll(void);
void LowStateUpdateMotor(MotorId id, const MotorState *feedback);
void LowStateUpdateApplied(MotorId id, const MotorApplied *applied);
uint8_t LowStateGet(LowState *out);
uint8_t LowStateGetMotor(MotorId id, MotorState *out);
uint8_t LowStateGetMotorMany(const MotorId *ids, MotorState *out, uint8_t count);
/* 兼容调试接口：返回快照缓存，新代码优先用 LowStateGetMotor。 */
const MotorState *LowStateGetMotorPtr(MotorId id);
uint8_t LowStateGetApplied(MotorId id, MotorApplied *out);
/* 兼容调试接口：返回快照缓存，新代码优先用 LowStateGetApplied。 */
const MotorApplied *LowStateGetAppliedPtr(MotorId id);

#endif
