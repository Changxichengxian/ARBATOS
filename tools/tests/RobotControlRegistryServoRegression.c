/* SPDX-License-Identifier: Apache-2.0 */
#include <assert.h>
#include <stdio.h>

#include "RobotControlRegistry.h"

Config g_config = {
    .profile = {
        .task_module_count = 1u,
        .task_modules = {ROBOT_TASK_MODULE_SERVO},
    },
    .servo = {
        .configured = 1u,
        .channels = {
            {.enabled = 1u, .port = 0u, .minUs = 1000u, .centerUs = 1500u, .maxUs = 2000u, .stepUs = 10u},
        },
    },
};

static uint16_t EmergencyWriter;

void ControlMgrTestEnterCritical(void)
{
}

void ControlMgrTestExitCritical(void)
{
}

void MotorInstRefresh(void)
{
}

uint8_t MotorInstCount(void)
{
    return 0u;
}

const MotorInst *MotorInstGet(uint8_t index)
{
    (void)index;
    return NULL;
}

const MotorInst *MotorInstFindByMotor(MotorId id)
{
    (void)id;
    return NULL;
}

const MotorInst *MotorInstFindByName(const char *name)
{
    (void)name;
    return NULL;
}

uint8_t MotorRouteCount(void)
{
    return 0u;
}

const MotorRoute *MotorRouteGet(uint8_t index)
{
    (void)index;
    return NULL;
}

const char *MotorInstName(const MotorInst *inst)
{
    return (inst != NULL) ? inst->name : NULL;
}

MotorId MotorInstId(const MotorInst *inst)
{
    return (inst != NULL) ? inst->actuator_id : MotorCount;
}

void CAN_rx_prepare_motor_measure_points(void)
{
}

uint8_t LowCmdEnterEmergencyStop(uint16_t writer)
{
    EmergencyWriter = writer;
    return 1u;
}

int main(void)
{
    ControlActuatorDiag diag;
    ControlCtx context = {0};
    ControlStatus status;
    const uint32_t servoMask = (uint32_t)0x0Fu << (uint32_t)LOWCMD_SERVO_FIRST;

    assert(MotorCount == 22);
    assert(RobotControlBootstrapProfileDefaults() != 0u);
    assert(EmergencyWriter == 0u);
    assert(ControlMgrGetActuatorDiag(&diag) == ControlResultOk);
    assert(diag.routableMask == servoMask);
    assert(diag.registeredMask == servoMask);
    assert(diag.activeMask == 0u);
    assert(diag.unroutableMask == 0u);
    assert(diag.unownedMask == 0u);
    assert(diag.crossDomainOverlapMask == 0u);
    assert(ControlMgrGetStatus(ControlDomainSystem, &status) == ControlResultOk);
    assert(status.active == 0u && status.pending_request == ControlRequestSwitch &&
           status.pending_id == ControlIdServo);
    assert(ControlMgrUpdateDomain(ControlDomainSystem, &context) == ControlResultOk);
    assert(ControlMgrOutputPermitValid(&context.outputPermit, servoMask) != 0u);
    assert(ControlMgrGetActuatorDiag(&diag) == ControlResultOk && diag.activeMask == servoMask);
    assert(ControlMgrGetStatus(ControlDomainSystem, &status) == ControlResultOk);
    assert(status.active != 0u && status.active_id == ControlIdServo);

    puts("PASS: servo profile registry bootstrap and actuator audit");
    return 0;
}
