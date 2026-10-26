#ifndef WHEELLEG_BENCH_H
#define WHEELLEG_BENCH_H

#include <stdint.h>

#define WHEELLEG_BENCH_TRIM_GAIN (-.08f)
#define WHEELLEG_BENCH_TRIM_LIMIT (.06f)
#define WHEELLEG_BENCH_TRIM_RATE (.008f)
#define WHEELLEG_BENCH_TRIM_DEADBAND (.005f)
#define WHEELLEG_BENCH_TRIM_LPF (.03f)

typedef struct
{
    float previous, position, rate;
    uint32_t tick, ready;
} WheelLegBenchWheel;

/* 使用同一帧的编码器角度和接收时间；reportedRate 只用于独立异常保护。 */
int WheelLegBenchWheelObserve(WheelLegBenchWheel *state, float raw, float reportedRate,
                             uint32_t tick);

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
    float pitchTrim;
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
    /* 四关节直接交给电机内部位置环，轮端仍由主控计算力矩。 */
    float jointTarget[4], jointStiffness;
    /* 轮端本地阻尼，仿真与实际 MIT 命令使用同一个值。 */
    float wheelDamping;
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
float WheelLegBenchJointDir(unsigned index);
float WheelLegBenchJointFromRaw(unsigned index, float raw);
float WheelLegBenchJointToRaw(unsigned index, float angle);
int WheelLegBenchStep(WheelLegBenchState *state, const WheelLegBenchInput *input,
                     WheelLegBenchOutput *output);
void WheelLegBenchAutoReset(WheelLegBenchAuto *state);
int WheelLegBenchAutoStep(WheelLegBenchAuto *state, const WheelLegBenchInput *input,
                         WheelLegBenchOutput *output);
int WheelLegBenchPoseStep(WheelLegBenchAuto *state, const WheelLegBenchInput *input,
                         WheelLegBenchOutput *output, uint32_t allowWheels);

#endif
