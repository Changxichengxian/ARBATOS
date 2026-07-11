/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef CONTROL_MGR_H
#define CONTROL_MGR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef CONTROL_MGR_MAX_CONTROLLERS
#define CONTROL_MGR_MAX_CONTROLLERS 16u
#endif

/*
 * Current role:
 * - register controllers and the resources they claim;
 * - expose runtime status for diagnostics;
 * - arbitrate explicit low-rate switch/stop requests;
 * - provide the shared scheduling gate that task modules call before output.
 *
 * During the migration, existing real-time loops still live in their task
 * modules. They must enter through ControlMgrUpdate*() so controller
 * activation, stop requests, and resource ownership stay in one place.
 *
 * All public APIs in this file are task-context only. Interrupt handlers must
 * notify a task instead of entering ControlMgr directly. Controller callbacks
 * always run outside the manager critical section and must not recursively
 * update their own domain. ControlMgrReset() is only valid before scheduling
 * starts or after every control task has stopped.
 */

typedef enum
{
    ControlDomainChassis = 0,
    ControlDomainGimbal,
    ControlDomainShoot,
    ControlDomainArm,
    ControlDomainWheelleg,
    ControlDomainSystem,
    ControlDomainCount
} ControlDomain;

typedef enum
{
    ControlIdNone = 0,
    ControlIdClassicChassis,
    ControlIdSingleGimbal,
    ControlIdDualYawGimbal,
    ControlIdShoot,
    ControlIdArmMotion,
    ControlIdWheellegServoCalibration,
    ControlIdWheellegServoBalance,
    ControlIdWheellegMitCalibration,
    ControlIdWheellegMitStandup,
    ControlIdWheellegMitBalance,
    ControlIdWheellegMitJump,
    ControlIdWheellegMitRecovery,
    ControlIdCustomBase = 128
} ControlId;

typedef enum
{
    ControlReasonNone = 0,
    ControlReasonStartup,
    ControlReasonProfile,
    ControlReasonModeSwitch,
    ControlReasonCalibration,
    ControlReasonTest,
    ControlReasonDisable,
    ControlReasonOffline,
    ControlReasonFault,
    ControlReasonEmergencyStop,
} ControlReason;

typedef enum
{
    ControlStateEmpty = 0,
    ControlStateStopped,
    ControlStateRunning,
    ControlStateFault,
} ControlState;

typedef enum
{
    ControlResultOk = 0,
    ControlResultBadArgument,
    ControlResultFull,
    ControlResultDuplicate,
    ControlResultNotFound,
    ControlResultDomainMismatch,
    ControlResultResourceBusy,
    ControlResultNotActive,
    ControlResultNotDue,
    ControlResultCallbackFailed,
} ControlResult;

typedef enum
{
    ControlRequestNone = 0,
    ControlRequestSwitch,
    ControlRequestStop,
} ControlRequest;

enum
{
    ControlResChassisWheels = (1ul << 0),
    ControlResGimbalYaw = (1ul << 1),
    ControlResGimbalPitch = (1ul << 2),
    ControlResShootTrigger = (1ul << 3),
    ControlResShootFriction = (1ul << 4),
    ControlResArm = (1ul << 5),
    ControlResWheellegLeftLeg = (1ul << 6),
    ControlResWheellegRightLeg = (1ul << 7),
    ControlResWheellegLeftWheel = (1ul << 8),
    ControlResWheellegRightWheel = (1ul << 9),
};

typedef struct
{
    float dt_s;
    uint32_t tick_ms;
    uint32_t flags;
    ControlReason reason;
    void *input;
    void *output;
} ControlCtx;

struct control_controller;
typedef ControlResult (*ControlCallback)(const struct control_controller *controller,
                                                          ControlCtx *context);

typedef struct
{
    uint16_t period_ms;
    uint16_t phase_ms;
    uint8_t priority;
    uint8_t input_count;
    uint8_t output_count;
    const char *const *inputs;
    const char *const *outputs;
} ControlMeta;

typedef struct control_controller
{
    uint16_t id;
    ControlDomain domain;
    uint32_t claim_mask;
    uint32_t actuator_mask;
    const char *name;
    ControlMeta meta;
    ControlCallback enter;
    ControlCallback update;
    ControlCallback exit;
    ControlCallback stop;
    void *user;
} ControlController;

typedef struct
{
    uint8_t active;
    uint16_t active_id;
    const char *active_name;
    ControlDomain domain;
    ControlState state;
    ControlRequest pending_request;
    uint16_t pending_id;
    uint32_t active_claim_mask;
    ControlReason last_reason;
    ControlResult last_result;
    uint32_t update_count;
    uint32_t transition_count;
    uint32_t reject_count;
} ControlStatus;

typedef struct
{
    uint32_t registerAttemptCount;
    uint32_t registerFailCount;
    uint32_t switchAttemptCount;
    uint32_t switchFailCount;
    uint32_t claimConflictCount;
    uint32_t updateReentryCount;
    uint32_t protectedRequestRejectCount;
    uint32_t reservedClaimMask;
    uint16_t lastRegisterErrorId;
    uint16_t lastSwitchErrorId;
    uint8_t lastRegisterError;
    uint8_t lastSwitchError;
    uint8_t reserved[2];
} ControlMgrDiag;

/*
 * 组合入口负责把名称和运行配置解析成物理执行器；ControlMgr 只保存结果。
 * 这一层诊断暂不参与控制器切换和输出仲裁。
 */
typedef struct
{
    uint32_t routableMask;
    uint32_t duplicateMask;
    uint16_t unresolvedOutputCount;
    uint16_t invalidIdCount;
} ControlActuatorAudit;

typedef struct
{
    uint32_t routableMask;
    uint32_t registeredMask;
    uint32_t activeMask;
    uint32_t duplicateMask;
    uint32_t crossDomainOverlapMask;
    uint32_t unownedMask;
    uint32_t unroutableMask;
    uint16_t unresolvedOutputCount;
    uint16_t invalidIdCount;
} ControlActuatorDiag;

void ControlMgrInit(void);
void ControlMgrReset(void);

const char *ControlDomainName(ControlDomain domain);
ControlResult ControlMgrRegister(const ControlController *controller);
uint8_t ControlMgrCount(void);
const ControlController *ControlMgrGet(uint8_t index);

/* Registry/name helpers may scan strings; keep them in init, commands, or diagnostics. */
const ControlController *ControlMgrFindByName(const char *name);
uint16_t ControlMgrFindIdByName(const char *name);
const char *ControlInputName(const ControlController *controller, uint8_t index);
const char *ControlOutputName(const ControlController *controller, uint8_t index);
uint16_t ControlPeriodMs(const ControlController *controller);
uint8_t ControlDue(const ControlController *controller, uint32_t tick_ms);
uint8_t ControlInputCount(const ControlController *controller);
uint8_t ControlOutputCount(const ControlController *controller);

ControlResult ControlMgrSwitch(uint16_t controller_id, ControlReason reason);
/* By-name switch/status helpers are for low-rate command paths. */
ControlResult ControlMgrSwitchByName(const char *name, ControlReason reason);
ControlResult ControlMgrStop(ControlDomain domain, ControlReason reason);
void ControlMgrStopAll(ControlReason reason);
/* A queued fault or emergency stop cannot be cleared through this helper. */
void ControlMgrClearPending(ControlDomain domain);

ControlResult ControlMgrUpdateDomain(ControlDomain domain, ControlCtx *context);
ControlResult ControlMgrUpdateAll(ControlCtx *context);
ControlResult ControlMgrUpdateDomainDue(ControlDomain domain, uint32_t tick_ms, ControlCtx *context);
ControlResult ControlMgrUpdateDueAll(uint32_t tick_ms, ControlCtx *context);

uint8_t ControlMgrIsActive(uint16_t controller_id);
uint8_t ControlMgrIsActiveByName(const char *name);
uint16_t ControlMgrActiveId(ControlDomain domain);
const char *ControlMgrActiveName(ControlDomain domain);
ControlResult ControlMgrGetStatus(ControlDomain domain, ControlStatus *out);
uint32_t ControlMgrActiveClaimMask(void);
ControlResult ControlMgrGetDiag(ControlMgrDiag *out);
ControlResult ControlMgrSetActuatorAudit(const ControlActuatorAudit *audit);
uint32_t ControlMgrActiveActuatorMask(void);
ControlResult ControlMgrGetActuatorDiag(ControlActuatorDiag *out);

#ifdef __cplusplus
}
#endif

#endif
