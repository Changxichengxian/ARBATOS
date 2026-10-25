#ifndef WHEELLEG_BENCH_H
#define WHEELLEG_BENCH_H

#include <stdint.h>

typedef struct
{
    /* 两侧均按前向坐标；顺序为右前、右后、左前、左后。 */
    float joint[4];
    float jointRate[4];
    float wheel[2];
    float wheelRate[2];
    float pitch, roll, yaw;
    float gyro[3];
    float targetV, targetYaw, dt;
} WheelLegBenchInput;

typedef struct
{
    float hold[4], feedforward[4];
    float reference, speed, yawRate, yawReference;
    uint32_t ready, driving;
} WheelLegBenchState;

typedef struct
{
    /* 右前、右后、右轮、左前、左后、左轮，均为模型正方向。 */
    float torque[6];
    float state[6];
    float length[2], alpha[2];
    /* 交给电机本地速度环，不能经 CAN 反馈再计算这一项。 */
    float jointDamping;
} WheelLegBenchOutput;

typedef struct
{
    WheelLegBenchState balance;
    float start[4], target[4];
    float elapsed, duration, settled;
    uint32_t phase;
    uint32_t reject;
    float rejectValue;
} WheelLegBenchAuto;

void WheelLegBenchReset(WheelLegBenchState *state);
int WheelLegBenchStep(WheelLegBenchState *state, const WheelLegBenchInput *input,
                     WheelLegBenchOutput *output);
void WheelLegBenchAutoReset(WheelLegBenchAuto *state);
int WheelLegBenchAutoStep(WheelLegBenchAuto *state, const WheelLegBenchInput *input,
                         WheelLegBenchOutput *output);

#endif
