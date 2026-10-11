/* SPDX-License-Identifier: Apache-2.0
 * 仅用于主机仿真的小幅学习修正；不接入固件或电机总线。
 * 直接包含冻结的桥源码，使零修正复用完全相同的控制计算。
 */
#include "../reference/OwnWbrBridge.c"

/* 每侧依次为支撑力、腿转矩、轮转矩；输入是 [-1, 1]。
 * 当前实验只允许正常平衡阶段。起身、跳跃和上台仍由原控制处理。
 */
int WbrLearningStep(void *handle, const float *sensor, const float *command,
                    const float *action, float *out)
{
    if (out != NULL) {
        memset(out, 0, 40 * sizeof(*out));
    }
    if (action == NULL || out == NULL) {
        return 6;
    }
    for (int i = 0; i < 6; i++) {
        if (!isfinite(action[i])) {
            return 6;
        }
    }
    const int result = OwnWbrCommandStep(handle, sensor, command, out);
    if (result != 0) {
        return result;
    }
    OwnWbrState *state = (OwnWbrState *)handle;
    if (state->stages.stage != 3u || state->rise_phase != 0 ||
        state->jump_phase != 0u || state->step_climb_phase != 0u ||
        state->recovery_requested) {
        return 0;
    }
    for (int side = 0; side < 2; side++) {
        const float sign = side == 0 ? -1.0f : 1.0f;
        const float df = 10.0f * WheelLegCoreClamp(action[side*3], -1.0f, 1.0f);
        const float dtp = 0.75f * WheelLegCoreClamp(action[side*3+1], -1.0f, 1.0f);
        const float dw = 0.35f * WheelLegCoreClamp(action[side*3+2], -1.0f, 1.0f);
        if (df == 0.0f && dtp == 0.0f && dw == 0.0f) {
            continue;
        }
        WheelLegCoreLegCalc leg = state->leg[side];
        const float old0 = leg.joint_torque[0];
        const float old1 = leg.joint_torque[1];
        leg.f0 = WheelLegCoreClamp(leg.f0 + df, -150.0f, 180.0f);
        leg.tp += dtp;
        if (!WheelLegCoreCalcVmc(&leg)) {
            memset(out, 0, 40 * sizeof(*out));
            return 7;
        }
        /* 补上 VMC 映射后的增量，原气簧补偿保持一次，最终重新限幅。
         * 基线已经饱和时此增量不是未限幅总力矩的严格残差，报告保留该边界。
         */
        out[side*3] = WheelLegCoreClamp(out[side*3] - sign*(leg.joint_torque[1]-old1),
                                        -state->joint_continuous, state->joint_continuous);
        out[side*3+1] = WheelLegCoreClamp(out[side*3+1] - sign*(leg.joint_torque[0]-old0),
                                          -state->joint_continuous, state->joint_continuous);
        out[side*3+2] = sign * WheelLegCoreClamp(sign*out[side*3+2] + dw, -5.2f, 5.2f);
        out[16+side*6] = leg.tp;
        out[17+side*6] = leg.f0;
    }
    return 0;
}
