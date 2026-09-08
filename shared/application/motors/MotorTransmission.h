#pragma once

#include "Types.h"

/*
 * 外置减速比定义为“电机侧转角 / 最终输出侧转角”。
 * 0 和 1 都表示直连；异常值按直连处理，避免配置损坏放大实时控制量。
 */
#define MOTOR_TRANS_EXTERNAL_RATIO_MAX 10000.0f

static inline fp32 MotorTransRatioSafe(fp32 configured)
{
    return (configured > 1.0f && configured <= MOTOR_TRANS_EXTERNAL_RATIO_MAX) ?
               configured : 1.0f;
}

static inline fp32 MotorTransPositionToMotor(fp32 output, fp32 ratio)
{
    ratio = MotorTransRatioSafe(ratio);
    return (ratio == 1.0f) ? output : output * ratio;
}

static inline fp32 MotorTransPositionToOutput(fp32 motor, fp32 ratio)
{
    ratio = MotorTransRatioSafe(ratio);
    return (ratio == 1.0f) ? motor : motor / ratio;
}

static inline fp32 MotorTransVelocityToMotor(fp32 output, fp32 ratio)
{
    return MotorTransPositionToMotor(output, ratio);
}

static inline fp32 MotorTransVelocityToOutput(fp32 motor, fp32 ratio)
{
    return MotorTransPositionToOutput(motor, ratio);
}

static inline fp32 MotorTransGainToMotor(fp32 output, fp32 ratio)
{
    ratio = MotorTransRatioSafe(ratio);
    return (ratio == 1.0f) ? output : output / (ratio * ratio);
}

static inline fp32 MotorTransGainToOutput(fp32 motor, fp32 ratio)
{
    ratio = MotorTransRatioSafe(ratio);
    return (ratio == 1.0f) ? motor : motor * ratio * ratio;
}

static inline fp32 MotorTransTorqueToMotor(fp32 output, fp32 ratio)
{
    ratio = MotorTransRatioSafe(ratio);
    return (ratio == 1.0f) ? output : output / ratio;
}

static inline fp32 MotorTransTorqueToOutput(fp32 motor, fp32 ratio)
{
    ratio = MotorTransRatioSafe(ratio);
    return (ratio == 1.0f) ? motor : motor * ratio;
}
