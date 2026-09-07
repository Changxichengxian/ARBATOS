/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CONTROL_ALGORITHM_H
#define CONTROL_ALGORITHM_H

#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CONTROL_ALGORITHM_MAX_OUTPUTS 8u
#define CONTROL_ALGORITHM_AXIS_COUNT 5u

typedef enum
{
    ControlAlgorithmDomainChassis = 0,
    ControlAlgorithmDomainGimbal,
} ControlAlgorithmDomain;

typedef enum
{
    ControlAlgorithmResultOk = 0,
    ControlAlgorithmResultBadConfig,
    ControlAlgorithmResultBadInput,
    ControlAlgorithmResultFault,
} ControlAlgorithmResult;

typedef enum
{
    ControlMotorModeDisable = 0,
    ControlMotorModeCurrent,
    ControlMotorModeDamping,
    ControlMotorModeStateTorque,
    ControlMotorModeSpeed,
} ControlMotorMode;

typedef enum
{
    ControlAxisChassisX = 0,
    ControlAxisChassisY,
    ControlAxisChassisWz,
    ControlAxisGimbalYaw,
    ControlAxisGimbalPitch,
} ControlAxis;

typedef struct
{
    const char *name;
    float value;
} ControlParam;

typedef struct
{
    const char *name;
    float defaultValue;
    float minValue;
    float maxValue;
} ControlParamDef;

static inline float ControlParamGet(const ControlParam *params, uint8_t count, const ControlParamDef *definition)
{
    if (definition == NULL) {
        return 0.0f;
    }
    for (uint8_t i = 0u; i < count; i++) {
        if (params != NULL && params[i].name != NULL && strcmp(params[i].name, definition->name) == 0) {
            return params[i].value;
        }
    }
    return definition->defaultValue;
}

typedef struct
{
    float q;
    float dq;
    float tau;
    int16_t speedRpm;
    int16_t current;
    uint32_t ageMs;
    uint8_t online;
} ControlMotorFeedback;

typedef struct
{
    float quaternionW;
    float quaternionX;
    float quaternionY;
    float quaternionZ;
    float angularVelocityXRadPerSec;
    float angularVelocityYRadPerSec;
    float angularVelocityZRadPerSec;
    float yawRad;
    float rollRad;
    float pitchRad;
    uint32_t ageMs;
    uint8_t valid;
} ControlImuInput;

typedef struct
{
    float axis[CONTROL_ALGORITHM_AXIS_COUNT];
    ControlMotorFeedback motor[CONTROL_ALGORITHM_MAX_OUTPUTS];
    ControlImuInput imu;
    float dt;
    uint32_t tickMs;
    uint32_t authoritySeq;
    uint8_t outputCount;
} ControlAlgorithmInput;

typedef struct
{
    float q;
    float dq;
    float kp;
    float kd;
    float tau;
    int16_t current;
    uint8_t mode;
} ControlMotorOutput;

typedef struct
{
    ControlMotorOutput motor[CONTROL_ALGORITHM_MAX_OUTPUTS];
    uint8_t outputCount;
} ControlAlgorithmOutput;

typedef ControlAlgorithmResult (*ControlAlgorithmConfigure)(void *state, const ControlParam *params,
                                                            uint8_t paramCount);
typedef void (*ControlAlgorithmReset)(void *state);
typedef ControlAlgorithmResult (*ControlAlgorithmStep)(void *state, const ControlAlgorithmInput *input,
                                                       ControlAlgorithmOutput *output);

typedef struct
{
    const char *name;
    ControlAlgorithmDomain domain;
    uint16_t stateBytes;
    uint8_t outputCount;
    const ControlParamDef *paramDefs;
    uint8_t paramDefCount;
    ControlAlgorithmConfigure configure;
    ControlAlgorithmReset reset;
    ControlAlgorithmStep step;
} ControlAlgorithm;

#ifdef __cplusplus
}
#endif

#endif
