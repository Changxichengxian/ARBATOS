/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ControlRuntime.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "BspTime.h"
#include "ControlCore.h"
#include "InsTask.h"
#include "LowCmd.h"
#include "ManualInput.h"
#include "ManualInputSnapshot.h"
#include "MotorInst.h"
#include "RobotLifecycle.h"
#include "RobotMode.h"

#define CONTROL_RUNTIME_STATE_BYTES 192u
#define CONTROL_RUNTIME_SAFE_WRITER ((uint16_t)LOWCMD_WRITER_SAFETY)

typedef struct
{
    ControlModuleSpec spec;
    ControlController controller;
    MotorId motorIds[CONTROL_ALGORITHM_MAX_OUTPUTS];
    union
    {
        long double alignLongDouble;
        void *alignPointer;
        uint8_t bytes[CONTROL_RUNTIME_STATE_BYTES];
    } state;
    uint32_t lastTickMs;
    uint8_t configured;
    uint8_t inhibited;
} ControlRuntimeSlot;

static ControlRuntimeSlot s_controlRuntime[CONTROL_RUNTIME_MAX_MODULES];
static uint8_t s_controlRuntimeCount;

static ControlDomain ControlRuntimeDomain(const ControlAlgorithm *algorithm)
{
    if (algorithm != NULL && algorithm->domain == ControlAlgorithmDomainGimbal) {
        return ControlDomainGimbal;
    }
    return ControlDomainChassis;
}

static uint32_t ControlRuntimeClaim(ControlDomain domain)
{
    if (domain == ControlDomainGimbal) {
        return ControlResGimbalYaw | ControlResGimbalPitch;
    }
    if (domain == ControlDomainChassis) {
        return ControlResChassisWheels;
    }
    return 0u;
}

static uint16_t ControlRuntimeId(ControlDomain domain)
{
    return (uint16_t)((uint16_t)ControlIdCustomBase + (uint16_t)domain);
}

static uint8_t ControlRuntimeParamValid(const ControlAlgorithm *algorithm, const ControlParam *params,
                                        uint8_t paramCount)
{
    if (algorithm == NULL || (paramCount != 0u && params == NULL)) {
        return 0u;
    }

    for (uint8_t i = 0u; i < paramCount; i++) {
        const ControlParamDef *definition = NULL;

        if (params[i].name == NULL) {
            return 0u;
        }
        for (uint8_t j = 0u; j < algorithm->paramDefCount; j++) {
            if (algorithm->paramDefs != NULL && algorithm->paramDefs[j].name != NULL &&
                strcmp(params[i].name, algorithm->paramDefs[j].name) == 0) {
                definition = &algorithm->paramDefs[j];
                break;
            }
        }
        if (definition == NULL || !isfinite(params[i].value) || params[i].value < definition->minValue ||
            params[i].value > definition->maxValue) {
            return 0u;
        }
        for (uint8_t j = 0u; j < i; j++) {
            if (strcmp(params[i].name, params[j].name) == 0) {
                return 0u;
            }
        }
    }
    return 1u;
}

static uint8_t ControlRuntimeSpecValid(const ControlModuleSpec *spec)
{
    if (spec == NULL || spec->algorithm == NULL || spec->algorithm->name == NULL || spec->algorithm->step == NULL ||
        spec->periodMs == 0u || spec->outputCount == 0u || spec->outputCount > CONTROL_ALGORITHM_MAX_OUTPUTS ||
        spec->outputCount != spec->algorithm->outputCount || spec->outputNames == NULL || spec->requireImu > 1u ||
        spec->algorithm->stateBytes > CONTROL_RUNTIME_STATE_BYTES ||
        spec->algorithm->domain > ControlAlgorithmDomainGimbal ||
        (spec->algorithm->paramDefCount != 0u && spec->algorithm->paramDefs == NULL) ||
        ControlRuntimeParamValid(spec->algorithm, spec->params, spec->paramCount) == 0u) {
        return 0u;
    }
    return 1u;
}

static void ControlRuntimeReset(ControlRuntimeSlot *slot)
{
    if (slot != NULL && slot->spec.algorithm != NULL && slot->spec.algorithm->reset != NULL) {
        slot->spec.algorithm->reset(slot->state.bytes);
    }
    if (slot != NULL) {
        slot->lastTickMs = 0u;
    }
}

static void ControlRuntimeInhibit(ControlRuntimeSlot *slot)
{
    if (slot == NULL || slot->spec.outputCount == 0u) {
        return;
    }
    (void)LowCmdInhibitManyFrom(slot->motorIds, slot->spec.outputCount, CONTROL_RUNTIME_SAFE_WRITER);
    slot->inhibited = 1u;
    ControlRuntimeReset(slot);
}

static uint8_t ControlRuntimeInputSafe(ControlDomain domain, const ManualInputSnapshot *manualInput)
{
    uint8_t switchIndex;
    uint8_t safePosition;

    if (manualInput == NULL || manualInput->online == 0u || manualInput->dataValid == 0u ||
        manualInput->sourceTimeoutMs == 0u || manualInput->sourceAgeMs > manualInput->sourceTimeoutMs ||
        RobotLifecycleOutputAllowed() == 0u) {
        return 0u;
    }

    if (domain == ControlDomainChassis) {
        switchIndex = (uint8_t)INPUT_SW_CHASSIS_MODE;
        safePosition = manualInput->semantics.ChassisSafePos;
    } else if (domain == ControlDomainGimbal) {
        switchIndex = (uint8_t)INPUT_SW_GIMBAL_MODE;
        safePosition = manualInput->semantics.GimbalSafePos;
    } else {
        return 0u;
    }

    return (uint8_t)(ControlInputSwitchIsPos(manualInput->control.sw[switchIndex], safePosition) == 0u);
}

static uint8_t ControlRuntimeModeAllowed(ControlDomain domain)
{
    const robot_run_mode_e mode = robot_mode_current();
    const robot_run_variant_e variant = robot_mode_variant();

    if (mode == ROBOT_RUN_MODE_SINGLE_MOTOR || mode == ROBOT_RUN_MODE_CALIBRATION) {
        return 0u;
    }
    if (mode == ROBOT_RUN_MODE_SINGLE_TASK) {
        if (domain == ControlDomainChassis) {
            return robot_mode_is_single_task(ROBOT_TASK_MODULE_CONTROL_CHASSIS);
        }
        if (domain == ControlDomainGimbal) {
            return robot_mode_is_single_task(ROBOT_TASK_MODULE_CONTROL_GIMBAL);
        }
        return 0u;
    }
    if (mode != ROBOT_RUN_MODE_FULL && mode != ROBOT_RUN_MODE_ENTERTAIN) {
        return 0u;
    }

    if (domain == ControlDomainChassis) {
        return (uint8_t)(variant == ROBOT_RUN_VARIANT_NORMAL || variant == ROBOT_RUN_VARIANT_CHASSIS_ONLY);
    }
    if (domain == ControlDomainGimbal) {
        /* 输出角色尚未进入描述表，单轴和双 yaw 模式不能猜测轴含义。 */
        return (uint8_t)(variant == ROBOT_RUN_VARIANT_NORMAL);
    }
    return 0u;
}

static float ControlRuntimeAxisNormalize(int16_t value)
{
    float normalized = (float)value / (float)RC_CH_VALUE_ABS_MAX;

    if (normalized > 1.0f) {
        normalized = 1.0f;
    } else if (normalized < -1.0f) {
        normalized = -1.0f;
    }
    return normalized;
}

static uint8_t ControlRuntimeInputBuild(ControlRuntimeSlot *slot, const ManualInputSnapshot *manualInput,
                                        ControlAlgorithmInput *input)
{
    if (slot == NULL || manualInput == NULL || input == NULL) {
        return 0u;
    }

    (void)memset(input, 0, sizeof(*input));
    input->tickMs = BspTimeGetTickMs();
    input->dt = (slot->lastTickMs == 0u) ? (float)slot->spec.periodMs * 0.001f
                                         : (float)(input->tickMs - slot->lastTickMs) * 0.001f;
    slot->lastTickMs = input->tickMs;
    input->authoritySeq = manualInput->authoritySeq;
    input->outputCount = slot->spec.outputCount;
    input->axis[ControlAxisChassisX] = ControlRuntimeAxisNormalize(manualInput->control.axis[INPUT_AXIS_CHASSIS_X]);
    input->axis[ControlAxisChassisY] = ControlRuntimeAxisNormalize(manualInput->control.axis[INPUT_AXIS_CHASSIS_Y]);
    input->axis[ControlAxisChassisWz] = ControlRuntimeAxisNormalize(manualInput->control.axis[INPUT_AXIS_CHASSIS_WZ]);
    input->axis[ControlAxisGimbalYaw] = ControlRuntimeAxisNormalize(manualInput->control.axis[INPUT_AXIS_GIMBAL_YAW]);
    input->axis[ControlAxisGimbalPitch] =
        ControlRuntimeAxisNormalize(manualInput->control.axis[INPUT_AXIS_GIMBAL_PITCH]);

    if (slot->spec.requireImu != 0u) {
        InsSnapshot snapshot;

        if (InsSnapshotRead(&snapshot) == 0u || snapshot.age_ms > CONTROL_RUNTIME_IMU_TIMEOUT_MS) {
            return 0u;
        }
        for (uint8_t i = 0u; i < 4u; i++) {
            if (!isfinite(snapshot.quat[i])) {
                return 0u;
            }
        }
        for (uint8_t i = 0u; i < 3u; i++) {
            if (!isfinite(snapshot.gyro[i]) || !isfinite(snapshot.angle[i])) {
                return 0u;
            }
        }
        input->imu.quaternionW = snapshot.quat[0];
        input->imu.quaternionX = snapshot.quat[1];
        input->imu.quaternionY = snapshot.quat[2];
        input->imu.quaternionZ = snapshot.quat[3];
        input->imu.angularVelocityXRadPerSec = snapshot.gyro[0];
        input->imu.angularVelocityYRadPerSec = snapshot.gyro[1];
        input->imu.angularVelocityZRadPerSec = snapshot.gyro[2];
        input->imu.yawRad = snapshot.angle[INS_YAW_ADDRESS_OFFSET];
        input->imu.rollRad = snapshot.angle[INS_ROLL_ADDRESS_OFFSET];
        input->imu.pitchRad = snapshot.angle[INS_PITCH_ADDRESS_OFFSET];
        input->imu.ageMs = snapshot.age_ms;
        input->imu.valid = 1u;
    }

    for (uint8_t i = 0u; i < slot->spec.outputCount; i++) {
        MotorState state;

        if (LowStateGetMotor(slot->motorIds[i], &state) == 0u) {
            return 0u;
        }
        input->motor[i].q = state.q;
        input->motor[i].dq = state.dq;
        input->motor[i].tau = state.tauEst;
        input->motor[i].speedRpm = state.speedRpm;
        input->motor[i].current = state.current;
        input->motor[i].ageMs = input->tickMs - state.lastRxTick;
        input->motor[i].online =
            (uint8_t)(state.online != 0u && input->motor[i].ageMs <= CONTROL_RUNTIME_FEEDBACK_TIMEOUT_MS);
        if (input->motor[i].online == 0u || !isfinite(input->motor[i].q) || !isfinite(input->motor[i].dq) ||
            !isfinite(input->motor[i].tau)) {
            return 0u;
        }
    }
    return 1u;
}

static uint8_t ControlRuntimeOutputBuild(const ControlAlgorithmOutput *source, uint8_t expectedCount,
                                         MotorCmd target[CONTROL_ALGORITHM_MAX_OUTPUTS])
{
    if (source == NULL || target == NULL || source->outputCount != expectedCount) {
        return 0u;
    }

    for (uint8_t i = 0u; i < expectedCount; i++) {
        switch ((ControlMotorMode)source->motor[i].mode) {
        case ControlMotorModeDisable:
            control_core_cmd_set_disable(&target[i]);
            break;
        case ControlMotorModeCurrent:
            control_core_cmd_set_current(&target[i], source->motor[i].current);
            break;
        case ControlMotorModeDamping:
            if (!isfinite(source->motor[i].kd) || !isfinite(source->motor[i].tau)) {
                return 0u;
            }
            control_core_cmd_set_damping(&target[i], source->motor[i].kd, source->motor[i].tau);
            break;
        case ControlMotorModeStateTorque:
            if (!isfinite(source->motor[i].q) || !isfinite(source->motor[i].dq) || !isfinite(source->motor[i].kp) ||
                !isfinite(source->motor[i].kd) || !isfinite(source->motor[i].tau)) {
                return 0u;
            }
            control_core_cmd_set_state_torque(&target[i], source->motor[i].q, source->motor[i].dq, source->motor[i].kp,
                                              source->motor[i].kd, source->motor[i].tau);
            break;
        case ControlMotorModeSpeed:
            if (!isfinite(source->motor[i].dq) || !isfinite(source->motor[i].kd) || !isfinite(source->motor[i].tau)) {
                return 0u;
            }
            control_core_cmd_set_speed(&target[i], source->motor[i].dq, source->motor[i].kd, source->motor[i].tau);
            break;
        default:
            return 0u;
        }
    }
    return 1u;
}

static ControlResult ControlRuntimeEnter(const ControlController *controller, ControlCtx *context)
{
    ControlRuntimeSlot *slot;

    (void)context;
    if (controller == NULL || controller->user == NULL) {
        return ControlResultBadArgument;
    }
    slot = (ControlRuntimeSlot *)controller->user;
    ControlRuntimeInhibit(slot);
    return ControlResultOk;
}

static ControlResult ControlRuntimeUpdate(const ControlController *controller, ControlCtx *context)
{
    ControlRuntimeSlot *slot;
    ManualInputSnapshot manualInput;
    ControlAlgorithmInput input;
    ControlAlgorithmOutput output;
    MotorCmd motorCmd[CONTROL_ALGORITHM_MAX_OUTPUTS];

    if (controller == NULL || controller->user == NULL || context == NULL) {
        return ControlResultBadArgument;
    }
    slot = (ControlRuntimeSlot *)controller->user;
    if (ManualInputSnapshotRead(&manualInput) == 0u ||
        ControlRuntimeInputSafe(controller->domain, &manualInput) == 0u ||
        ControlRuntimeModeAllowed(controller->domain) == 0u) {
        ControlRuntimeInhibit(slot);
        return ControlResultOk;
    }
    if (ControlRuntimeInputBuild(slot, &manualInput, &input) == 0u) {
        ControlRuntimeInhibit(slot);
        return ControlResultOk;
    }

    if (slot->inhibited != 0u) {
        if (LowCmdRecoverSafetyInhibitManyWithPermit(slot->motorIds, slot->spec.outputCount, &context->outputPermit) ==
            0u) {
            ControlRuntimeInhibit(slot);
            return ControlResultCallbackFailed;
        }
        slot->inhibited = 0u;
        ControlRuntimeReset(slot);
        return ControlResultOk;
    }

    (void)memset(&output, 0, sizeof(output));
    if (slot->spec.algorithm->step(slot->state.bytes, &input, &output) != ControlAlgorithmResultOk ||
        ControlRuntimeOutputBuild(&output, slot->spec.outputCount, motorCmd) == 0u ||
        MotorInstSetIdsWithPermit(slot->motorIds, motorCmd, slot->spec.outputCount, &context->outputPermit) == 0u) {
        ControlRuntimeInhibit(slot);
        return ControlResultCallbackFailed;
    }
    return ControlResultOk;
}

static ControlResult ControlRuntimeLeave(const ControlController *controller, ControlCtx *context)
{
    (void)context;
    if (controller == NULL || controller->user == NULL) {
        return ControlResultBadArgument;
    }
    ControlRuntimeInhibit((ControlRuntimeSlot *)controller->user);
    return ControlResultOk;
}

ControlResult ControlRuntimeConfigure(const ControlModuleSpec *specs, uint8_t count)
{
    uint32_t domainMask = 0u;

    (void)memset(s_controlRuntime, 0, sizeof(s_controlRuntime));
    s_controlRuntimeCount = 0u;
    if (count > CONTROL_RUNTIME_MAX_MODULES || (count != 0u && specs == NULL)) {
        return ControlResultBadArgument;
    }

    for (uint8_t i = 0u; i < count; i++) {
        const ControlModuleSpec *spec = &specs[i];
        const ControlDomain domain = ControlRuntimeDomain(spec->algorithm);
        const uint32_t domainBit = 1ul << (uint32_t)domain;
        ControlRuntimeSlot *slot = &s_controlRuntime[i];

        if (ControlRuntimeSpecValid(spec) == 0u || (domainMask & domainBit) != 0u) {
            (void)memset(s_controlRuntime, 0, sizeof(s_controlRuntime));
            return ControlResultBadArgument;
        }
        domainMask |= domainBit;
        slot->spec = *spec;
        for (uint8_t j = 0u; j < spec->outputCount; j++) {
            const MotorInst *inst;
            const MotorRoute *route;

            if (spec->outputNames[j] == NULL) {
                (void)memset(s_controlRuntime, 0, sizeof(s_controlRuntime));
                return ControlResultBadArgument;
            }
            inst = MotorInstFindByName(spec->outputNames[j]);
            if (inst == NULL || MotorInstEnabled(inst) == 0u) {
                (void)memset(s_controlRuntime, 0, sizeof(s_controlRuntime));
                return ControlResultNotFound;
            }
            slot->motorIds[j] = MotorInstId(inst);
            route = MotorRouteFindByMotor(slot->motorIds[j]);
            if (slot->motorIds[j] >= MotorCount || route == NULL || route->enabled == 0u) {
                (void)memset(s_controlRuntime, 0, sizeof(s_controlRuntime));
                return ControlResultNotFound;
            }
            for (uint8_t k = 0u; k < j; k++) {
                if (slot->motorIds[k] == slot->motorIds[j]) {
                    (void)memset(s_controlRuntime, 0, sizeof(s_controlRuntime));
                    return ControlResultDuplicate;
                }
            }
            for (uint8_t previous = 0u; previous < i; previous++) {
                for (uint8_t k = 0u; k < s_controlRuntime[previous].spec.outputCount; k++) {
                    if (s_controlRuntime[previous].motorIds[k] == slot->motorIds[j]) {
                        (void)memset(s_controlRuntime, 0, sizeof(s_controlRuntime));
                        return ControlResultResourceBusy;
                    }
                }
            }
        }
        if (spec->algorithm->configure != NULL &&
            spec->algorithm->configure(slot->state.bytes, spec->params, spec->paramCount) != ControlAlgorithmResultOk) {
            (void)memset(s_controlRuntime, 0, sizeof(s_controlRuntime));
            return ControlResultBadArgument;
        }

        slot->controller.id = ControlRuntimeId(domain);
        slot->controller.domain = domain;
        slot->controller.claim_mask = ControlRuntimeClaim(domain);
        slot->controller.name = spec->algorithm->name;
        slot->controller.meta.period_ms = spec->periodMs;
        slot->controller.meta.output_count = spec->outputCount;
        slot->controller.meta.outputs = spec->outputNames;
        slot->controller.enter = ControlRuntimeEnter;
        slot->controller.update = ControlRuntimeUpdate;
        slot->controller.exit = ControlRuntimeLeave;
        slot->controller.stop = ControlRuntimeLeave;
        slot->controller.user = slot;
        slot->configured = 1u;
    }
    s_controlRuntimeCount = count;
    return ControlResultOk;
}

uint8_t ControlRuntimeCount(void) { return s_controlRuntimeCount; }

const ControlController *ControlRuntimeController(uint8_t index)
{
    if (index >= s_controlRuntimeCount || s_controlRuntime[index].configured == 0u) {
        return NULL;
    }
    return &s_controlRuntime[index].controller;
}

ControlResult ControlRuntimeStartDefaults(void)
{
    for (uint8_t i = 0u; i < s_controlRuntimeCount; i++) {
        const ControlResult result = ControlMgrSwitch(s_controlRuntime[i].controller.id, ControlReasonProfile);
        if (result != ControlResultOk) {
            return result;
        }
    }
    return ControlResultOk;
}

ControlResult ControlRuntimeRunDomain(ControlDomain domain)
{
    ControlCtx context = {0};

    context.tick_ms = BspTimeGetTickMs();
    context.dt_s = (float)ControlRuntimePeriodMs(domain) * 0.001f;
    return ControlMgrUpdateDomain(domain, &context);
}

uint16_t ControlRuntimePeriodMs(ControlDomain domain)
{
    for (uint8_t i = 0u; i < s_controlRuntimeCount; i++) {
        if (s_controlRuntime[i].controller.domain == domain) {
            return s_controlRuntime[i].spec.periodMs;
        }
    }
    return 1u;
}
