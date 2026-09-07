/* SPDX-License-Identifier: Apache-2.0 */
#include "SpeedGimbal.h"
#include "SpeedGimbalParams.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

typedef struct
{
    float yawSpeed;
    float pitchSpeed;
    float speedKp;
    float maxCurrent;
} SpeedGimbalState;

static ControlAlgorithmResult SpeedGimbalConfigure(void *state, const ControlParam *params, uint8_t count)
{
    SpeedGimbalState *config = state;
    if (config == NULL || (count != 0 && params == NULL)) {
        return ControlAlgorithmResultBadConfig;
    }
    config->yawSpeed = ControlParamGet(params, count, &SpeedGimbalParams[SpeedGimbalParam_yaw_speed_rpm]);
    config->pitchSpeed = ControlParamGet(params, count, &SpeedGimbalParams[SpeedGimbalParam_pitch_speed_rpm]);
    config->speedKp = ControlParamGet(params, count, &SpeedGimbalParams[SpeedGimbalParam_speed_kp]);
    config->maxCurrent = ControlParamGet(params, count, &SpeedGimbalParams[SpeedGimbalParam_max_current]);
    return ControlAlgorithmResultOk;
}

static void SpeedGimbalReset(void *state)
{
    (void)state;
}

static ControlAlgorithmResult SpeedGimbalStep(void *state, const ControlAlgorithmInput *input,
                                             ControlAlgorithmOutput *output)
{
    const SpeedGimbalState *config = state;
    if (config == NULL || input == NULL || output == NULL || input->outputCount != 2) {
        return ControlAlgorithmResultBadInput;
    }
    memset(output, 0, sizeof(*output));
    output->outputCount = 2;
    for (uint8_t i = 0; i < 2; i++) {
        const float axis = input->axis[i == 0 ? ControlAxisGimbalYaw : ControlAxisGimbalPitch];
        const float speed = i == 0 ? config->yawSpeed : config->pitchSpeed;
        float current = (axis * speed - input->motor[i].speedRpm) * config->speedKp;
        if (!isfinite(current)) {
            return ControlAlgorithmResultBadInput;
        }
        if (current > config->maxCurrent) {
            current = config->maxCurrent;
        } else if (current < -config->maxCurrent) {
            current = -config->maxCurrent;
        }
        output->motor[i].mode = ControlMotorModeCurrent;
        output->motor[i].current = (int16_t)current;
    }
    return ControlAlgorithmResultOk;
}

/* 同一接口用于底盘和云台；绑定、失联和发送由 ControlRuntime 统一处理。 */
const ControlAlgorithm SpeedGimbal = {
    .name = "controller.speed_gimbal",
    .domain = ControlAlgorithmDomainGimbal,
    .stateBytes = sizeof(SpeedGimbalState),
    .outputCount = 2,
    .paramDefs = SpeedGimbalParams,
    .paramDefCount = SpeedGimbalParamCount,
    .configure = SpeedGimbalConfigure,
    .reset = SpeedGimbalReset,
    .step = SpeedGimbalStep,
};
