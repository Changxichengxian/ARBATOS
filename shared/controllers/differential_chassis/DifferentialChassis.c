/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include "DifferentialChassis.h"
#include "DifferentialChassisParams.h"

#include <string.h>

#define DIFFERENTIAL_PI 3.14159265358979323846f

typedef struct
{
    float trackWidth;
    float wheelRadius;
    float gearRatio;
    float maxSpeed;
    float maxYawRate;
    float speedKp;
    float maxCurrent;
} DifferentialState;

static ControlAlgorithmResult DifferentialConfigure(void *state,
                                                     const ControlParam *params,
                                                     uint8_t paramCount)
{
    DifferentialState *runtime = (DifferentialState *)state;

    if (runtime == NULL || (paramCount != 0u && params == NULL))
    {
        return ControlAlgorithmResultBadConfig;
    }
    runtime->trackWidth = ControlParamGet(
        params, paramCount, &DifferentialChassisParams[DifferentialChassisParam_track_width_m]);
    runtime->wheelRadius = ControlParamGet(
        params, paramCount, &DifferentialChassisParams[DifferentialChassisParam_wheel_radius_m]);
    runtime->gearRatio = ControlParamGet(
        params, paramCount, &DifferentialChassisParams[DifferentialChassisParam_gear_ratio]);
    runtime->maxSpeed = ControlParamGet(
        params, paramCount, &DifferentialChassisParams[DifferentialChassisParam_max_speed_mps]);
    runtime->maxYawRate = ControlParamGet(
        params, paramCount, &DifferentialChassisParams[DifferentialChassisParam_max_yaw_rate_radps]);
    runtime->speedKp = ControlParamGet(
        params, paramCount, &DifferentialChassisParams[DifferentialChassisParam_speed_kp]);
    runtime->maxCurrent = ControlParamGet(
        params, paramCount, &DifferentialChassisParams[DifferentialChassisParam_max_current]);
    return ControlAlgorithmResultOk;
}

static void DifferentialReset(void *state)
{
    (void)state;
}

static int16_t DifferentialCurrent(float value, float limit)
{
    if (value > limit)
    {
        value = limit;
    }
    else if (value < -limit)
    {
        value = -limit;
    }
    return (int16_t)value;
}

static ControlAlgorithmResult DifferentialStep(void *state,
                                                const ControlAlgorithmInput *input,
                                                ControlAlgorithmOutput *output)
{
    DifferentialState *runtime = (DifferentialState *)state;
    float targetLinear;
    float targetYaw;
    float targetRpm[2];
    float current[2];

    if (runtime == NULL || input == NULL || output == NULL || input->outputCount != 2u ||
        input->motor[0].online == 0u || input->motor[1].online == 0u ||
        runtime->wheelRadius <= 0.0f || runtime->gearRatio <= 0.0f)
    {
        return ControlAlgorithmResultBadInput;
    }

    targetLinear = input->axis[ControlAxisChassisX] * runtime->maxSpeed;
    targetYaw = input->axis[ControlAxisChassisWz] * runtime->maxYawRate;
    targetRpm[0] = (targetLinear - targetYaw * runtime->trackWidth * 0.5f) *
                   60.0f * runtime->gearRatio / (2.0f * DIFFERENTIAL_PI * runtime->wheelRadius);
    targetRpm[1] = (targetLinear + targetYaw * runtime->trackWidth * 0.5f) *
                   60.0f * runtime->gearRatio / (2.0f * DIFFERENTIAL_PI * runtime->wheelRadius);
    current[0] = (targetRpm[0] - (float)input->motor[0].speedRpm) * runtime->speedKp;
    current[1] = (targetRpm[1] - (float)input->motor[1].speedRpm) * runtime->speedKp;

    (void)memset(output, 0, sizeof(*output));
    output->outputCount = 2u;
    output->motor[0].mode = (uint8_t)ControlMotorModeCurrent;
    output->motor[1].mode = (uint8_t)ControlMotorModeCurrent;
    output->motor[0].current = DifferentialCurrent(current[0], runtime->maxCurrent);
    output->motor[1].current = DifferentialCurrent(current[1], runtime->maxCurrent);
    return ControlAlgorithmResultOk;
}

const ControlAlgorithm DifferentialChassis = {
    .name = "controller.differential",
    .domain = ControlAlgorithmDomainChassis,
    .stateBytes = (uint16_t)sizeof(DifferentialState),
    .outputCount = 2u,
    .paramDefs = DifferentialChassisParams,
    .paramDefCount = (uint8_t)(sizeof(DifferentialChassisParams) / sizeof(DifferentialChassisParams[0])),
    .configure = DifferentialConfigure,
    .reset = DifferentialReset,
    .step = DifferentialStep,
};
