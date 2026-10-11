/* SPDX-License-Identifier: Apache-2.0
 * WBR 的位置控制阶段；电机映射由调用方写入模型。
 */
#ifndef OWN_WBR_STAGES_H
#define OWN_WBR_STAGES_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "WheelLegCore.h"

#define OWN_WBR_STAGE_HOLD 0u
#define OWN_WBR_STAGE_LEG_POSITION 1u

typedef struct
{
    uint8_t stage;
    uint8_t initialized;
    fp32 target_length_m;
    fp32 command_length_m;
    fp32 hold_angle[2][2];
    fp32 foot_dir_x[2];
    fp32 foot_dir_y[2];
} OwnWbrStages;

static inline fp32 OwnWbrStageJointTorque(fp32 target, fp32 position, fp32 velocity)
{
    return WheelLegCoreClamp(280.0f * (target - position) - 4.0f * velocity, -20.0f, 20.0f);
}

static inline void OwnWbrStagesInit(OwnWbrStages *state, uint8_t stage, fp32 target_length_m)
{
    if (state == NULL) {
        return;
    }
    (void)memset(state, 0, sizeof(*state));
    state->stage = stage;
    state->target_length_m = WheelLegCoreClamp(target_length_m, 0.18f, 0.24f);
}

/* sensor每侧9项：q1,dq1,tau1,q4,dq4,tau4,...；腿数组须是本帧Core正解结果。 */
static inline uint8_t OwnWbrStageStep(const WheelLegCoreGeometry *geo,
                                     const fp32 sensor[28],
                                     const WheelLegCoreLegCalc leg[2],
                                     fp32 dt,
                                     OwnWbrStages *state,
                                     fp32 hip_torque[4])
{
    static const fp32 sign[2] = {-1.0f, 1.0f};
    static const fp32 front_zero = 1.804837140117f;
    static const fp32 back_zero = -1.486361593218f;
    fp32 raw_q[2][2];
    fp32 raw_dq[2][2];
    uint8_t side;

    if (geo == NULL || sensor == NULL || leg == NULL || state == NULL || hip_torque == NULL ||
        dt <= 0.0f || dt > 0.05f ||
        (state->stage != OWN_WBR_STAGE_HOLD && state->stage != OWN_WBR_STAGE_LEG_POSITION)) {
        return 0u;
    }

    for (side = 0u; side < 2u; side++) {
        const uint8_t base = (uint8_t)(10u + side * 9u);
        raw_q[side][0] = sensor[base];
        raw_dq[side][0] = sensor[base + 1u];
        raw_q[side][1] = sensor[base + 3u];
        raw_dq[side][1] = sensor[base + 4u];
        if (leg[side].length <= 0.01f) {
            return 0u;
        }
    }

    if (state->initialized == 0u) {
        for (side = 0u; side < 2u; side++) {
            state->hold_angle[side][0] = raw_q[side][0];
            state->hold_angle[side][1] = raw_q[side][1];
            state->foot_dir_x[side] = sinf(leg[side].alpha);
            state->foot_dir_y[side] = cosf(leg[side].alpha);
        }
        state->command_length_m = 0.5f * (leg[0].length + leg[1].length);
        state->target_length_m = WheelLegCoreClamp(state->target_length_m, 0.18f, 0.32f);
        state->initialized = 1u;
    }

    if (state->stage == OWN_WBR_STAGE_LEG_POSITION) {
        const fp32 target = WheelLegCoreClamp(state->target_length_m, 0.18f, 0.32f);
        const fp32 max_step = 0.04f * dt;
        state->command_length_m += WheelLegCoreClamp(target - state->command_length_m,
                                                     -max_step,
                                                     max_step);
    }
    else {
        state->command_length_m = 0.5f * (leg[0].length + leg[1].length);
    }

    for (side = 0u; side < 2u; side++) {
        fp32 target_q1 = state->hold_angle[side][0];
        fp32 target_q4 = state->hold_angle[side][1];

        if (state->stage == OWN_WBR_STAGE_LEG_POSITION) {
            WheelLegCoreFootPoint target_foot;
            fp32 front_target;
            fp32 back_target;

            target_foot.x_m = state->foot_dir_x[side] * state->command_length_m;
            target_foot.y_m = state->foot_dir_y[side] * state->command_length_m;
            target_foot.length_m = state->command_length_m;
            if (WheelLegCoreInversePoint(geo,
                                         &target_foot,
                                         front_zero - sign[side] * raw_q[side][1],
                                         back_zero - sign[side] * raw_q[side][0],
                                         &front_target,
                                         &back_target) == 0u) {
                return 0u;
            }
            target_q1 = (-1.486361593218f - back_target) / sign[side];
            target_q4 = (1.804837140117f - front_target) / sign[side];
        }

        hip_torque[side * 2u] =
            OwnWbrStageJointTorque(target_q1, raw_q[side][0], raw_dq[side][0]);
        hip_torque[side * 2u + 1u] =
            OwnWbrStageJointTorque(target_q4, raw_q[side][1], raw_dq[side][1]);
    }
    return 1u;
}

#endif
