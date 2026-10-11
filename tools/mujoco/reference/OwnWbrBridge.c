/* SPDX-License-Identifier: Apache-2.0
 * WBR 仅提供机械模型；控制计算直接编译本项目 WheelLegCore。
 */
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "WheelLegCore.h"
#include "OwnWbrStages.h"

#ifndef OWN_WBR_RISE_HOLD_S
#define OWN_WBR_RISE_HOLD_S 0.09f
#endif
#ifndef OWN_WBR_RISE_MOVE_S
#define OWN_WBR_RISE_MOVE_S 0.70f
#endif
#ifndef OWN_WBR_RISE_PEAK_FADE_S
#define OWN_WBR_RISE_PEAK_FADE_S 0.25f
#endif
#ifndef OWN_WBR_RISE_HANDOFF_MIN_S
#define OWN_WBR_RISE_HANDOFF_MIN_S 0.15f
#endif
#ifndef OWN_WBR_RISE_HANDOFF_STABLE_S
#define OWN_WBR_RISE_HANDOFF_STABLE_S 0.15f
#endif
#ifndef OWN_WBR_RISE_FINISH_STABLE_S
#define OWN_WBR_RISE_FINISH_STABLE_S 0.20f
#endif
#ifndef OWN_WBR_RISE_WHEEL_PITCH_KP
#define OWN_WBR_RISE_WHEEL_PITCH_KP 6.0f
#endif
#ifndef OWN_WBR_RECOVERY_WHEEL_PITCH_KP
#define OWN_WBR_RECOVERY_WHEEL_PITCH_KP 2.0f
#endif
#ifndef OWN_WBR_RECOVERY_WHEEL_DAMPING
#define OWN_WBR_RECOVERY_WHEEL_DAMPING 0.15f
#endif
#ifndef OWN_WBR_RISE_WHEEL_GYRO_KD
#define OWN_WBR_RISE_WHEEL_GYRO_KD 0.3f
#endif
#ifndef OWN_WBR_RISE_TARGET_THETA_BIAS
#define OWN_WBR_RISE_TARGET_THETA_BIAS -0.40f
#endif
#ifndef OWN_WBR_RECOVERY_SWEEP_RAD
#define OWN_WBR_RECOVERY_SWEEP_RAD 2.20f
#endif
#ifndef OWN_WBR_RECOVERY_TARGET_RATE_RADPS
#define OWN_WBR_RECOVERY_TARGET_RATE_RADPS 0.80f
#endif
#ifndef OWN_WBR_RECOVERY_JOINT_SPEED_LIMIT_RADPS
#define OWN_WBR_RECOVERY_JOINT_SPEED_LIMIT_RADPS 2.50f
#endif
#ifndef OWN_WBR_RECOVERY_JOINT_TRAVEL_LIMIT_RAD
#define OWN_WBR_RECOVERY_JOINT_TRAVEL_LIMIT_RAD 2.45f
#endif
#ifndef OWN_WBR_RECOVERY_JOINT_TORQUE_LIMIT_NM
#define OWN_WBR_RECOVERY_JOINT_TORQUE_LIMIT_NM 12.0f
#endif
#ifndef OWN_WBR_RECOVERY_WHEEL_SPEED_LIMIT_RADPS
#define OWN_WBR_RECOVERY_WHEEL_SPEED_LIMIT_RADPS 6.0f
#endif
#ifndef OWN_WBR_RECOVERY_WHEEL_TORQUE_LIMIT_NM
#define OWN_WBR_RECOVERY_WHEEL_TORQUE_LIMIT_NM 0.60f
#endif
#ifndef OWN_WBR_RECOVERY_BODY_RATE_LIMIT_RADPS
#define OWN_WBR_RECOVERY_BODY_RATE_LIMIT_RADPS 1.50f
#endif
#ifndef OWN_WBR_RECOVERY_NO_PROGRESS_S
#define OWN_WBR_RECOVERY_NO_PROGRESS_S 2.40f
#endif
#ifndef OWN_WBR_RECOVERY_TOTAL_SWEEP_S
#define OWN_WBR_RECOVERY_TOTAL_SWEEP_S 3.20f
#endif
#ifndef OWN_WBR_RECOVERY_PEAK_TIME_S
#define OWN_WBR_RECOVERY_PEAK_TIME_S 0.40f
#endif
#ifndef OWN_WBR_RECOVERY_PEAK_WORK_J
#define OWN_WBR_RECOVERY_PEAK_WORK_J 15.0f
#endif
#ifndef OWN_WBR_RECOVERY_RELEASE_S
#define OWN_WBR_RECOVERY_RELEASE_S 0.20f
#endif
#ifndef OWN_WBR_RECOVERY_SETTLE_TIMEOUT_S
#define OWN_WBR_RECOVERY_SETTLE_TIMEOUT_S 1.50f
#endif
#ifndef OWN_WBR_RECOVERY_SETTLE_STABLE_S
#define OWN_WBR_RECOVERY_SETTLE_STABLE_S 0.15f
#endif
#ifndef OWN_WBR_RECOVERY_SETTLE_MIN_S
#define OWN_WBR_RECOVERY_SETTLE_MIN_S 1.00f
#endif
#ifndef OWN_WBR_RECOVERY_SETTLE_JOINT_RADPS
#define OWN_WBR_RECOVERY_SETTLE_JOINT_RADPS 0.80f
#endif
#ifndef OWN_WBR_RECOVERY_SETTLE_WHEEL_RADPS
#define OWN_WBR_RECOVERY_SETTLE_WHEEL_RADPS 4.00f
#endif
#ifndef OWN_WBR_RECOVERY_SETTLE_BODY_RADPS
#define OWN_WBR_RECOVERY_SETTLE_BODY_RADPS 0.80f
#endif

typedef struct
{
    WheelLegCoreLegCalc leg[2];
    WheelLegCorePid length_pid[2];
    WheelLegCorePid split_pid;
    WheelLegCoreObserver observer;
    OwnWbrStages stages;
    float k[12];
    float theta0;
    float tp0;
    float wheel0;
    float support;
    float length;
    float length_target;
    float length_peak_time;
    float length_peak_rest;
    float jump_prepare_elapsed;
    uint8_t jump_prepare_cancelled;
    float jump_base_length;
    float jump_drive_speed;
    float radius;
    float joint_continuous;
    float joint_peak;
    float gas_spring_force;
    float last_phi3[2];
    float x_ref;
    float v_ref;
    float yaw_ref;
    float yaw_rate;
    float last_drive_command;
    float roll_length_bias;
    float roll_adapt_time;
    uint8_t roll_adapt_active;
    float rise_start_q[2][2];
    float rise_target_q[2][2];
    float recovery_origin_q[2][2];
    float recovery_command_q[2][2];
    float recovery_start_foot[2][2];
    float rise_phase_time;
    float rise_elapsed;
    float rise_stable_time;
    float recovery_peak_time[2][2];
    float recovery_work_j[2][2];
    float recovery_sweep_floor;
    float recovery_best_tilt;
    float recovery_progress_time;
    float recovery_release_time;
    float jump_time;
    float jump_stable_time;
    float jump_landing_time;
    float jump_push_offset;
    uint32_t jump_count;
    int8_t rise_phase;
    uint8_t rise_enabled;
    uint8_t rise_initialized;
    uint8_t recovery_requested;
    uint8_t recovery_path;
    uint8_t recovery_wait_needed;
    uint8_t jump_phase;
    uint8_t jump_pressed;
    uint8_t jump_profile;
    uint8_t jump_profile_requested;
    uint8_t step_jump;
    uint8_t step_jump_hold;
    uint8_t drive_mode;
    uint8_t drive_engaged;
    uint8_t reverse_brake;
    uint8_t jump_reverse_prepare;
    uint8_t jump_flight_seen;
    float step_climb_elapsed;
    float step_climb_total_time;
    float step_climb_ready_time;
    float step_climb_drive_x;
    float step_climb_armed_time;
    float step_climb_q[2][2];
    float step_climb_foot[2][2];
    float step_climb_peak_time[2][2];
    float step_climb_balance_time;
    float step_climb_p4_brake_time;
    float climb_assist_level;
    uint8_t step_climb_phase;
    uint8_t step_climb_drive_started;
    uint8_t step_climb_enabled;
    uint8_t step_climb_hold;
    uint8_t step_climb_park_shifted;
    uint8_t step_climb_high;
    uint8_t step_climb_short_fold;
    uint8_t step_climb_fault;
    uint8_t step_climb_balance_handoff;
    uint8_t step_climb_p4_brake_latched;
    uint8_t climb_assist_pressed;
    uint8_t climb_assist_latched;
    uint8_t first;
} OwnWbrState;

static const WheelLegCoreGeometry s_geometry = {
    .l1_m = 0.214999936171f, .l2_m = 0.254000234356f,
    .l3_m = 0.254000234356f, .l4_m = 0.214999936171f, .l5_m = 0.0f
};

/* 配置：负 K 的 12 项、theta0、tp0、wheel0、支撑力、腿长、半径、长度 P/D。 */
void *OwnWbrCreate(const float *config)
{
    if (config == NULL) {
        return NULL;
    }
    OwnWbrState *state = (OwnWbrState *)calloc(1, sizeof(*state));
    if (state == NULL) {
        return NULL;
    }
    memcpy(state->k, config, sizeof(state->k));
    state->theta0 = config[12];
    state->tp0 = config[13];
    state->wheel0 = config[14];
    state->support = config[15];
    state->length = config[16];
    state->length_target = state->length;
    state->jump_base_length = state->length;
    state->radius = config[17];
    state->joint_continuous = 20.0f;
    state->joint_peak = 20.0f;
    OwnWbrStagesInit(&state->stages, 3u, state->length);
    for (int side = 0; side < 2; side++) {
        /* Core PID 的微分是逐次误差差值，按固定 3 ms 换算。 */
        WheelLegCorePidConfigure(&state->length_pid[side], config[18], 0.0f,
                                config[19] / 0.003f, 100.0f, 0.0f);
    }
    WheelLegCorePidConfigure(&state->split_pid, 15.0f, 0.0f, 1.0f / 0.003f, 6.0f, 0.0f);
    return state;
}

void OwnWbrDestroy(void *handle)
{
    free(handle);
}

/* 阶段切换由运行器重建实例，避免沿用上一阶段的积分和位置目标。 */
int OwnWbrSetStage(void *handle, int stage)
{
    OwnWbrState *state = (OwnWbrState *)handle;
    if (state == NULL || state->first || stage < 0 || stage > 3) {
        return 0;
    }
    OwnWbrStagesInit(&state->stages, (uint8_t)stage, state->length);
    return 1;
}

int OwnWbrSetLegLength(void *handle, float length_m)
{
    OwnWbrState *state = (OwnWbrState *)handle;
    if (state == NULL || state->stages.stage < 1u || state->stages.stage > 3u ||
        !isfinite(length_m) ||
        length_m < 0.18f || length_m > 0.32f) {
        return 0;
    }
    state->stages.target_length_m = length_m;
    state->length_target = length_m;
    return 1;
}

/* 0 保持位置停车；1 松开行进键时允许滑行，反向请求仍先主动制动。 */
int OwnWbrSetDriveMode(void *handle, int mode)
{
    OwnWbrState *state = (OwnWbrState *)handle;
    if (state == NULL || (mode != 0 && mode != 1)) {
        return 0;
    }
    state->drive_mode = (uint8_t)mode;
    if (mode == 0) {
        state->drive_engaged = 0u;
        state->reverse_brake = 0u;
        state->last_drive_command = 0.0f;
    }
    return 1;
}

static uint8_t OwnWbrStepJumpConfigValid(const OwnWbrState *state)
{
    const uint8_t spring_supported = state != NULL &&
        ((state->gas_spring_force >= 299.0f && state->gas_spring_force <= 301.0f) ||
         (state->gas_spring_force >= 449.0f && state->gas_spring_force <= 451.0f));
    return state != NULL && spring_supported &&
        state->jump_profile_requested == 3u && state->jump_profile == 3u &&
        state->joint_continuous >= 19.5f && state->joint_continuous <= 20.0f &&
        state->joint_peak >= 39.5f && state->joint_peak <= 40.0f;
}

/* 越台大跳由场景显式选择；动作开始后禁止改轨迹。 */
int OwnWbrSetStepJump(void *handle, int enabled)
{
    OwnWbrState *state = (OwnWbrState *)handle;
    if (state == NULL || (enabled != 0 && enabled != 1) || state->stages.stage != 3u) {
        return 0;
    }
    if ((enabled && (state->step_climb_enabled ||
                     !OwnWbrStepJumpConfigValid(state))) ||
        (state->jump_phase != 0u && state->step_jump != (uint8_t)enabled)) {
        return 0;
    }
    state->step_jump = (uint8_t)enabled;
    if (!enabled) {
        state->step_jump_hold = 0u;
    }
    return 1;
}

int OwnWbrSetJointPeak(void *handle, float peak_nm)
{
    OwnWbrState *state = (OwnWbrState *)handle;
    if (state == NULL || !isfinite(peak_nm) || peak_nm < state->joint_continuous || peak_nm > 45.0f) {
        return 0;
    }
    if (state->step_jump) {
        return WheelLegCoreAbs(peak_nm - state->joint_peak) <= 0.001f;
    }
    state->joint_peak = peak_nm;
    return 1;
}

int OwnWbrSetJointLimits(void *handle, float continuous_nm, float peak_nm)
{
    OwnWbrState *state = (OwnWbrState *)handle;
    if (state == NULL || !isfinite(continuous_nm) || !isfinite(peak_nm) ||
        continuous_nm <= 0.0f || continuous_nm > peak_nm || peak_nm > 45.0f) {
        return 0;
    }
    if (state->step_jump) {
        return WheelLegCoreAbs(continuous_nm - state->joint_continuous) <= 0.001f &&
            WheelLegCoreAbs(peak_nm - state->joint_peak) <= 0.001f;
    }
    state->joint_continuous = continuous_nm;
    state->joint_peak = peak_nm;
    return 1;
}

/* 与借用模型配套的物理层使用同一个力值；默认 0 保持旧模型行为。 */
int OwnWbrSetGasSpring(void *handle, float force_n)
{
    OwnWbrState *state = (OwnWbrState *)handle;
    if (state == NULL || state->first || !isfinite(force_n) ||
        force_n < 0.0f || force_n > 1000.0f) {
        return 0;
    }
    if (state->step_jump) {
        return WheelLegCoreAbs(force_n - state->gas_spring_force) <= 0.001f;
    }
    state->gas_spring_force = force_n;
    return 1;
}

/* 0/1 保留原碰撞模型的小跳/增强跳，2/3 对应排除四处凸包假自碰后的模型。 */
int OwnWbrSetJumpProfile(void *handle, int profile)
{
    OwnWbrState *state = (OwnWbrState *)handle;
    const uint8_t spring_300 = state != NULL &&
        state->gas_spring_force >= 299.0f && state->gas_spring_force <= 301.0f;
    const uint8_t spring_450 = state != NULL &&
        state->gas_spring_force >= 449.0f && state->gas_spring_force <= 451.0f;
    const uint8_t legacy_limits = state != NULL && spring_450 &&
        state->joint_continuous >= 9.5f && state->joint_continuous <= 10.5f &&
        state->joint_peak >= 44.5f && state->joint_peak <= 45.0f;
    const uint8_t motor_8009_limits = state != NULL && (spring_300 || spring_450) &&
        state->joint_continuous >= 19.5f && state->joint_continuous <= 20.0f &&
        state->joint_peak >= 39.5f && state->joint_peak <= 40.0f;
    if (state == NULL || profile < 0 || profile > 3 ||
        (state->step_jump && profile != 3) ||
        (profile != 0 && !legacy_limits && !motor_8009_limits)) {
        return 0;
    }
    state->jump_profile_requested = (uint8_t)profile;
    if (state->jump_phase == 0u) {
        state->jump_profile = (uint8_t)profile;
    }
    return 1;
}

int OwnWbrGetJumpProfile(void *handle)
{
    const OwnWbrState *state = (const OwnWbrState *)handle;
    return state == NULL ? -1 : (int)state->jump_profile;
}

/* 1 正在准备，0 无等待，-1 本次准备超时取消，-2 无效控制器。 */
int OwnWbrGetJumpPreparationStatus(void *handle)
{
    const OwnWbrState *state = (const OwnWbrState *)handle;
    if (state == NULL) {
        return -2;
    }
    if (state->jump_prepare_cancelled) {
        return -1;
    }
    return state->jump_profile >= 2u && state->jump_phase == 1u &&
        state->jump_time <= 0.0f ? 1 : 0;
}

int OwnWbrSetStepClimb(void *handle, int enable)
{
    OwnWbrState *state = (OwnWbrState *)handle;
    if (state == NULL || enable < 0 || enable > 2 ||
        state->stages.stage != 3u ||
        (enable && (state->step_jump || state->step_jump_hold ||
                    state->jump_phase != 0u)) ||
        (state->step_climb_phase >= 1u && state->step_climb_phase <= 7u)) {
        return 0;
    }
    state->step_climb_enabled = (uint8_t)enable;
    state->step_climb_phase = 0u;
    state->step_climb_elapsed = 0.0f;
    state->step_climb_total_time = 0.0f;
    state->step_climb_ready_time = 0.0f;
    state->step_climb_armed_time = 0.0f;
    state->step_climb_drive_started = 0u;
    state->step_climb_hold = 0u;
    state->step_climb_park_shifted = 0u;
    state->step_climb_high = 0u;
    state->step_climb_balance_handoff = 0u;
    state->step_climb_balance_time = 0.0f;
    state->step_climb_p4_brake_latched = 0u;
    state->step_climb_p4_brake_time = 0.0f;
    state->climb_assist_pressed = 0u;
    state->climb_assist_latched = 0u;
    state->climb_assist_level = 0.0f;
    memset(state->step_climb_peak_time, 0, sizeof(state->step_climb_peak_time));
    return 1;
}

/* 短腿折叠轨迹只能在首个控制周期前选择，动作或故障期间不允许切换。 */
int OwnWbrSetClimbFold(void *handle, int enabled)
{
    OwnWbrState *state = (OwnWbrState *)handle;
    if (state == NULL || (enabled != 0 && enabled != 1) || state->first ||
        state->step_climb_fault != 0u || state->rise_phase != 0 ||
        state->jump_phase != 0u || state->step_climb_phase != 0u ||
        state->recovery_requested) {
        return 0;
    }
    state->step_climb_short_fold = (uint8_t)enabled;
    return 1;
}

int OwnWbrSetClimbAssist(void *handle, int pressed)
{
    OwnWbrState *state = (OwnWbrState *)handle;
    if (state == NULL || (pressed != 0 && pressed != 1)) {
        return 0;
    }
    if (!pressed) {
        if (state->step_climb_phase == 0u) {
            state->climb_assist_pressed = 0u;
        }
        return 1;
    }
    if (state->stages.stage != 3u || state->step_climb_enabled != 2u) {
        return 0;
    }
    /* 动作一旦触发就锁存完成；松键只影响触发前的准备姿态。 */
    if (state->step_climb_phase == 0u) {
        state->climb_assist_pressed = (uint8_t)pressed;
    }
    return 1;
}

int OwnWbrGetStepClimbPhase(void *handle)
{
    const OwnWbrState *state = (const OwnWbrState *)handle;
    return state == NULL ? -1 : (int)state->step_climb_phase;
}

float OwnWbrGetJumpSpeedLimit(void *handle)
{
    const OwnWbrState *state = (const OwnWbrState *)handle;
    if (state == NULL) {
        return -1.0f;
    }
    if (state->step_jump && state->jump_profile_requested == 3u) {
        return 1.0f;
    }
    return (state->jump_profile_requested & 1u) != 0u ? 0.25f : 3.0f;
}

float OwnWbrGetJumpReverseSpeedLimit(void *handle)
{
    const OwnWbrState *state = (const OwnWbrState *)handle;
    if (state == NULL) {
        return -1.0f;
    }
    return (state->jump_profile_requested & 1u) != 0u ? 0.25f : 1.0f;
}

int OwnWbrEnableRise(void *handle, int enable)
{
    OwnWbrState *state = (OwnWbrState *)handle;
    /* 只允许第一帧前选择；运行中取消会把位置起身直接切到平衡环。 */
    if (state == NULL || state->first) {
        return 0;
    }
    state->rise_enabled = enable != 0;
    state->rise_phase = state->rise_enabled ? 1 : 0;
    state->rise_initialized = 0u;
    state->rise_phase_time = 0.0f;
    state->rise_elapsed = 0.0f;
    state->rise_stable_time = 0.0f;
    memset(state->recovery_peak_time, 0, sizeof(state->recovery_peak_time));
    memset(state->recovery_work_j, 0, sizeof(state->recovery_work_j));
    state->recovery_sweep_floor = 0.0f;
    state->recovery_best_tilt = 0.0f;
    state->recovery_progress_time = 0.0f;
    state->recovery_release_time = 0.0f;
    state->recovery_wait_needed = 0u;
    state->jump_phase = 0u;
    state->jump_time = 0.0f;
    state->jump_pressed = 0u;
    return 1;
}

/* 保留仿真中的刚体状态，只请求控制器用电机尝试一次有界恢复。
 * out[34]：11 辨认姿态，12 建立支撑，13 有限翻身，14 回站立关节位，15 平衡接管。
 * -11 表示传感器姿态无效，-12 表示站立目标不可解，-13 表示未翻过临界点，
 * -14 表示站立或接管超时，-15/-16 表示关节超速/超行程，-17 表示轮速过高，
 * -18/-19 表示落稳或动作无进展/单次翻身超时，-20 表示机身角速度过高；负阶段保持所有执行器为零。 */
int OwnWbrRequestRecovery(void *handle)
{
    OwnWbrState *state = (OwnWbrState *)handle;
    if (state == NULL) {
        return 0;
    }
    state->recovery_requested = 1u;
    state->rise_enabled = 1u;
    state->rise_phase = 11;
    state->rise_initialized = 0u;
    state->rise_phase_time = 0.0f;
    state->rise_elapsed = 0.0f;
    state->rise_stable_time = 0.0f;
    memset(state->recovery_peak_time, 0, sizeof(state->recovery_peak_time));
    memset(state->recovery_work_j, 0, sizeof(state->recovery_work_j));
    state->recovery_sweep_floor = 0.0f;
    state->recovery_best_tilt = 0.0f;
    state->recovery_progress_time = 0.0f;
    state->recovery_release_time = 0.0f;
    state->recovery_wait_needed = 0u;
    state->jump_phase = 0u;
    state->jump_time = 0.0f;
    state->jump_pressed = 0u;
    return 1;
}

static float OwnWbrSmoothStep(float value)
{
    const float x = WheelLegCoreClamp(value, 0.0f, 1.0f);
    return x * x * (3.0f - 2.0f * x);
}

static void OwnWbrDriveStep(OwnWbrState *state, float target_v,
                            uint8_t rise_busy, float dt)
{
    const float command_deadband = 0.01f;

    /* 观察器在空中暂停。保留起跳速度，并把位置参考钉在最后可信里程上。 */
    if (state->jump_phase == 3u) {
        state->x_ref = state->observer.x_m;
        state->jump_flight_seen = 1u;
        return;
    }
    if (state->jump_flight_seen) {
        /* 落地第一帧只用编码器观察器重锚定，不借用仿真真值。 */
        state->x_ref = state->observer.x_m;
        state->jump_flight_seen = 0u;
    }

    if (rise_busy || state->drive_mode == 0u) {
        const float decel = state->step_jump && state->jump_profile == 3u &&
            state->jump_phase == 4u &&
            WheelLegCoreAbs(state->jump_drive_speed) > 0.50f ?
            8.0f : (state->step_jump && state->jump_profile == 3u &&
            state->jump_phase >= 4u ?
            3.0f : (state->jump_phase != 0u ? 0.8f : 0.4f));
        const float max_step = decel * dt;
        state->v_ref += WheelLegCoreClamp(target_v - state->v_ref,
                                          -max_step, max_step);
        if (state->step_jump && state->jump_profile == 3u && state->jump_phase == 4u &&
            WheelLegCoreAbs(target_v) <= command_deadband &&
            WheelLegCoreAbs(state->v_ref) < 0.03f) {
            /* 落地姿态尚在恢复时跟随编码器里程，恢复结束再锁住当前位置。 */
            state->x_ref = state->observer.x_m;
        } else {
            state->x_ref += state->v_ref * dt;
        }
        return;
    }

    if (WheelLegCoreAbs(target_v) <= command_deadband) {
        /* 只有实际操作过纵向指令才进入滑行，静止打开网页不会跟随估计偏置。 */
        state->reverse_brake = 0u;
        if (!state->drive_engaged) {
            state->v_ref = 0.0f;
            return;
        }
        state->x_ref = state->observer.x_m;
        {
            const float coast_step = 0.12f * dt;
            state->v_ref += WheelLegCoreClamp(-state->v_ref,
                                              -coast_step, coast_step);
        }
        if (WheelLegCoreAbs(state->observer.v_mps) < 0.04f &&
            WheelLegCoreAbs(state->v_ref) < 0.04f) {
            state->drive_engaged = 0u;
            state->v_ref = 0.0f;
        }
        return;
    }

    state->drive_engaged = 1u;

    if (!state->reverse_brake &&
        target_v * state->last_drive_command < -0.0001f) {
        state->reverse_brake = 1u;
    }
    if (state->reverse_brake) {
        const float brake_step = 2.0f * dt;
        state->v_ref += WheelLegCoreClamp(-state->v_ref,
                                          -brake_step, brake_step);
        state->x_ref += state->v_ref * dt;
        if (WheelLegCoreAbs(state->observer.v_mps) < 0.06f &&
            WheelLegCoreAbs(state->v_ref) < 0.08f) {
            state->reverse_brake = 0u;
            state->x_ref = state->observer.x_m;
            state->v_ref = 0.0f;
            state->last_drive_command = target_v;
        }
        return;
    }

    {
        const float accel = state->step_jump && state->jump_profile == 3u &&
            state->jump_phase == 4u &&
            WheelLegCoreAbs(state->jump_drive_speed) > 0.50f ? 8.0f :
            (state->step_jump && state->jump_profile == 3u &&
             ((state->jump_phase == 1u &&
               WheelLegCoreAbs(state->jump_drive_speed) > 0.05f) ||
              state->jump_phase >= 4u) ? 2.5f : 1.0f);
        const float max_step = accel * dt;
        state->last_drive_command = target_v;
        state->v_ref += WheelLegCoreClamp(target_v - state->v_ref,
                                          -max_step, max_step);
        state->x_ref += state->v_ref * dt;
    }
}

/* 开源 WBR 的安装参数：P 在上杆、Q 在下杆，恒力气弹簧将两点推开。
 * 直接从两个电机编码器重建安装点；不读取仿真关节或外力真值。 */
static void OwnWbrGasSpringTorque(const OwnWbrState *state, int side,
                                  float q1, float q4,
                                  float *q1_torque, float *q4_torque)
{
    const float sign = side == 0 ? -1.0f : 1.0f;
    const float l1 = sqrtf(0.214234f * 0.214234f + 0.0181319f * 0.0181319f);
    const float l2 = sqrtf(0.233282f * 0.233282f + 0.100477f * 0.100477f);
    const float d1 = 0.03f;
    const float d2 = 0.05f;
    const float mount_angle = 0.2164f;
    const float phi1 = atan2f(0.0181319f, -0.214234f) + sign * q1;
    const float phi4 = atan2f(-0.023191f, 0.0972737f) + sign * q4;
    const float xdb = l1 * (cosf(phi4) - cosf(phi1));
    const float ydb = l1 * (sinf(phi4) - sinf(phi1));
    const float a0 = 2.0f * l2 * xdb;
    const float b0 = 2.0f * l2 * ydb;
    const float c0 = xdb * xdb + ydb * ydb;
    const float discriminant = a0 * a0 + b0 * b0 - c0 * c0;

    *q1_torque = 0.0f;
    *q4_torque = 0.0f;
    if (state->gas_spring_force <= 0.0f || discriminant <= 0.0f) {
        return;
    }
    const float phi2 = 2.0f * atan2f(b0 + sqrtf(discriminant), a0 + c0);
    const float cx = l1 * cosf(phi1) + l2 * cosf(phi2);
    const float cy = l1 * sinf(phi1) + l2 * sinf(phi2);
    const float phi3 = WHEELLEG_CORE_PI +
        atan2f(l1 * sinf(phi4) - cy, l1 * cosf(phi4) - cx);
    const float spring_angle = phi1 + mount_angle - WHEELLEG_CORE_PI * 0.5f;
    const float px = d1 * cosf(spring_angle);
    const float py = d1 * sinf(spring_angle);
    const float qx = l1 * cosf(phi1) + d2 * cosf(phi2);
    const float qy = l1 * sinf(phi1) + d2 * sinf(phi2);
    const float dx = qx - px;
    const float dy = qy - py;
    const float spring_length = sqrtf(dx * dx + dy * dy);
    const float sin23 = sinf(phi3 - phi2);

    if (spring_length < 1e-5f ||
        WheelLegCoreAbs(sin23) < 0.05f) {
        return;
    }

    const float force_x = state->gas_spring_force * dx / spring_length;
    const float force_y = state->gas_spring_force * dy / spring_length;
    /* 闭链隐式导数经安装点距离有限差分核对；上游实现这两项符号相反。 */
    const float dphi2_dphi1 = -l1 * sinf(phi3 - phi1) / (l2 * sin23);
    const float dphi2_dphi4 = l1 * sinf(phi3 - phi4) / (l2 * sin23);
    const float dpx_dphi1 = -d1 * sinf(spring_angle);
    const float dpy_dphi1 = d1 * cosf(spring_angle);
    const float dqx_dphi1 = -l1 * sinf(phi1) - d2 * sinf(phi2) * dphi2_dphi1;
    const float dqy_dphi1 = l1 * cosf(phi1) + d2 * cosf(phi2) * dphi2_dphi1;
    const float dqx_dphi4 = -d2 * sinf(phi2) * dphi2_dphi4;
    const float dqy_dphi4 = d2 * cosf(phi2) * dphi2_dphi4;
    const float tau_phi1 = force_x * (dqx_dphi1 - dpx_dphi1) +
        force_y * (dqy_dphi1 - dpy_dphi1);
    const float tau_phi4 = force_x * dqx_dphi4 + force_y * dqy_dphi4;
    *q1_torque = sign * tau_phi1;
    *q4_torque = sign * tau_phi4;
}

static void OwnWbrStepClimbCapture(OwnWbrState *state, const float *sensor,
                                   float pitch)
{
    for (int side = 0; side < 2; side++) {
        const int base = 10 + side * 9;
        const float core_x = state->leg[side].length * sinf(state->leg[side].alpha);
        const float core_y = state->leg[side].length * cosf(state->leg[side].alpha);
        state->step_climb_q[side][0] = sensor[base];
        state->step_climb_q[side][1] = sensor[base + 3];
        state->step_climb_foot[side][0] =
            cosf(pitch)*core_x + sinf(pitch)*core_y;
        state->step_climb_foot[side][1] =
            -sinf(pitch)*core_x + cosf(pitch)*core_y;
    }
}

static void OwnWbrResetBalance(OwnWbrState *state, float yaw);

static int OwnWbrStepClimbLowOverride(OwnWbrState *state, const float *sensor,
                                      float pitch, float yaw, float *out)
{
    const uint8_t phase = state->step_climb_phase;
    if (phase < 1u || phase > 6u) {
        return 0;
    }
    state->step_climb_elapsed += 0.003f;
    state->step_climb_total_time += 0.003f;
    if (state->step_climb_total_time > 12.0f) {
        return 11;
    }
    float target_x = 0.25f;
    float target_y = 0.18f;
    float move_s = 0.40f;
    float blend = 1.0f;
    if (phase == 1u) {
        blend = OwnWbrSmoothStep(state->step_climb_elapsed / move_s);
    } else if (phase == 3u) {
        target_y = 0.06f;
        move_s = 0.50f;
        blend = OwnWbrSmoothStep(state->step_climb_elapsed / move_s);
    } else if (phase == 4u) {
        target_x = 0.18f;
        target_y = 0.06f;
        move_s = 0.30f;
        blend = OwnWbrSmoothStep(state->step_climb_elapsed / move_s);
    } else if (phase == 5u) {
        target_x = 0.10f;
        target_y = 0.18f;
        move_s = 0.25f;
        blend = OwnWbrSmoothStep(state->step_climb_elapsed / move_s);
    } else if (phase == 6u) {
        /* 用姿态反馈把轮支点收到质心下方，收稳后再捕获驻车参考。 */
        target_x = state->step_climb_enabled == 2u ?
            WheelLegCoreClamp(0.015f - 0.09f*pitch - 0.015f*sensor[5],
                              -0.040f, 0.070f) : 0.0f;
        target_y = 0.20f;
        move_s = state->step_climb_enabled == 2u ? 0.45f : 0.25f;
        blend = OwnWbrSmoothStep(state->step_climb_elapsed / move_s);
    }
    for (int side = 0; side < 2; side++) {
        const int base = 10 + side * 9;
        const float sign = side == 0 ? -1.0f : 1.0f;
        WheelLegCoreFootPoint point;
        float front;
        float back;
        point.x_m = target_x;
        point.y_m = target_y;
        point.length_m = sqrtf(target_x*target_x + target_y*target_y);
        if (phase >= 5u) {
            float world_x = state->step_climb_foot[side][0] +
                blend * (target_x - state->step_climb_foot[side][0]);
            float world_y = state->step_climb_foot[side][1] +
                blend * (target_y - state->step_climb_foot[side][1]);
            point.x_m = cosf(pitch)*world_x - sinf(pitch)*world_y;
            point.y_m = sinf(pitch)*world_x + cosf(pitch)*world_y;
            point.length_m = sqrtf(world_x*world_x + world_y*world_y);
            if (phase == 5u && point.length_m < 0.190f) {
                if (point.length_m < 1e-5f) {
                    return 9;
                }
                /* 直线收腿会内凹，留出受撞压缩的余量，避免碰短腿挡块。 */
                const float scale = 0.190f / point.length_m;
                point.x_m *= scale;
                point.y_m *= scale;
                point.length_m = 0.190f;
            }
        }
        if (!WheelLegCoreInversePoint(&s_geometry, &point, 1.25f, -1.38f,
                                      &front, &back)) {
            return 9;
        }
        float q1 = WheelLegCoreNearAngle((-1.486361593218f-back)/sign,
                                         state->step_climb_q[side][0]);
        float q4 = WheelLegCoreNearAngle((1.804837140117f-front)/sign,
                                         state->step_climb_q[side][1]);
        if (phase < 5u) {
            q1 = state->step_climb_q[side][0] + blend * (q1-state->step_climb_q[side][0]);
            q4 = state->step_climb_q[side][1] + blend * (q4-state->step_climb_q[side][1]);
        }
        float spring_q1;
        float spring_q4;
        OwnWbrGasSpringTorque(state, side, sensor[base], sensor[base + 3],
                              &spring_q1, &spring_q4);
        out[side*3] = WheelLegCoreClamp(300.0f*(q1-sensor[base]) -
            10.0f*sensor[base+1] - spring_q1, -state->joint_peak,
            state->joint_peak);
        out[side*3+1] = WheelLegCoreClamp(300.0f*(q4-sensor[base+3]) -
            10.0f*sensor[base+4] - spring_q4, -state->joint_peak,
            state->joint_peak);
        for (int joint = 0; joint < 2; joint++) {
            const int output_index = side*3 + joint;
            if (WheelLegCoreAbs(out[output_index]) > state->joint_continuous) {
                state->step_climb_peak_time[side][joint] += 0.003f;
            }
            if (state->step_climb_peak_time[side][joint] >= 3.50f) {
                /* 3.5 s 是保守控制预算，不代表厂家的电机热允许时间。 */
                return 10;
            }
        }
        /* 导轮搭台后用轮速抑制继续前冲，避免在低台上滑过支撑点。 */
        float wheel = phase <= 2u ?
            WheelLegCoreClamp(-1.2f * state->observer.v_mps, -3.0f, 0.0f) : 0.6f;
        if (phase >= 5u) {
            wheel = WheelLegCoreClamp(0.3f + 4.0f*pitch + 0.5f*sensor[5], -5.2f, 5.2f);
            if (phase == 6u) {
                /* 腿调整支点，轮端只消掉交接前的平动。 */
                if (state->step_climb_enabled == 2u) {
                    wheel = WheelLegCoreClamp(-1.2f*state->observer.v_mps,
                                              -1.2f, 1.2f);
                } else {
                    wheel = state->step_climb_elapsed >= 0.24f ? 5.2f :
                        WheelLegCoreClamp(-4.0f*pitch - 0.5f*sensor[5], -5.2f, 5.2f);
                }
            }
        }
        out[side*3+2] = sign * wheel;
    }
    out[31] = state->joint_peak;
    const float end_s = phase == 2u ? 0.50f : move_s + 0.05f;
    uint8_t advance = state->step_climb_elapsed >= end_s;
    if (phase == 6u && state->step_climb_enabled == 2u) {
        const float mean_theta = 0.5f *
            (state->leg[0].theta + state->leg[1].theta);
        const float mean_length = 0.5f *
            (state->leg[0].length + state->leg[1].length);
        const float mean_d_length = 0.5f *
            (WheelLegCoreAbs(state->leg[0].d_length) +
             WheelLegCoreAbs(state->leg[1].d_length));
        const uint8_t stable = WheelLegCoreAbs(pitch) < 0.0873f &&
            WheelLegCoreAbs(out[6]) < 0.0873f &&
            WheelLegCoreAbs(sensor[4]) < 0.35f &&
            WheelLegCoreAbs(sensor[6]) < 0.35f &&
            WheelLegCoreAbs(sensor[5]) < 0.35f &&
            WheelLegCoreAbs(state->observer.v_mps) < 0.12f &&
            WheelLegCoreAbs(mean_theta - state->theta0) < 0.08f &&
            mean_length > 0.19f && mean_length < 0.22f &&
            mean_d_length < 0.18f;
        state->step_climb_ready_time = stable ?
            state->step_climb_ready_time + 0.003f : 0.0f;
        advance = state->step_climb_elapsed >= move_s &&
            state->step_climb_ready_time >= 0.18f;
    }
    if (advance) {
        state->step_climb_phase++;
        state->step_climb_elapsed = 0.0f;
        state->step_climb_ready_time = 0.0f;
        OwnWbrStepClimbCapture(state, sensor, pitch);
        if (state->step_climb_phase == 7u) {
            state->length = 0.20f;
            state->length_target = 0.20f;
            state->stages.target_length_m = 0.20f;
            state->x_ref = state->observer.x_m;
            state->v_ref = 0.0f;
            state->yaw_ref = yaw;
            if (state->step_climb_enabled == 2u) {
                state->step_climb_hold = 1u;
                state->step_climb_park_shifted = 0u;
            }
        }
    }
    return 0;
}

/* 高站姿 越台：每阶段只使用一种目标表达，避免足端 IK 与关节圆弧互相覆盖。 */
static int OwnWbrStepClimbHighOverride(OwnWbrState *state, const float *sensor,
                                       float pitch, float yaw,
                                       const float *normal_out, float *out)
{
    const uint8_t phase = state->step_climb_phase;
    const uint8_t short_fold = state->step_climb_short_fold;
    if (phase < 1u || phase > 5u) {
        return 0;
    }
    state->step_climb_elapsed += 0.003f;
    state->step_climb_total_time += 0.003f;
    if (state->step_climb_total_time > 12.0f) {
        return 11;
    }

    float target_x = 0.03f;
    float target_y = 0.20f;
    float move_s = 0.55f;
    if (phase == 1u) {
        target_x = 0.30f;
        target_y = 0.10f;
        move_s = state->climb_assist_latched ? 0.28f : 0.40f;
    } else if (phase == 2u) {
        target_x = 0.30f;
        target_y = 0.10f;
        move_s = state->climb_assist_latched ? 0.08f : 0.15f;
    } else if (phase == 3u) {
        target_x = short_fold ? 0.115f : 0.18f;
        target_y = 0.06f;
        move_s = state->climb_assist_latched ? 0.44f : 0.68f;
    } else if (phase == 4u) {
        /* 默认按 IMU 俯仰前送支点；短腿模式在下方改为世界系极坐标收腿。 */
        target_x = WheelLegCoreClamp(0.03f - 0.09f*pitch - 0.015f*sensor[5],
                                     -0.040f, 0.030f);
        target_y = short_fold ? 0.16f : 0.21f;
        move_s = short_fold ? 0.40f : 0.70f;
    } else if (phase == 5u) {
        /* 先把轮支点送到前倾质心下方；姿态回正后自动退回正常站立角。 */
        target_x = WheelLegCoreClamp(0.015f - 0.09f*pitch - 0.015f*sensor[5],
                                     -0.040f, 0.070f);
        target_y = 0.20f;
        move_s = 0.45f;
    }
    const float blend = OwnWbrSmoothStep(state->step_climb_elapsed / move_s);
    if (phase == 4u && !state->step_climb_p4_brake_latched &&
        state->step_climb_elapsed >= 0.18f && pitch > 0.00872665f) {
        state->step_climb_p4_brake_latched = 1u;
        state->step_climb_p4_brake_time = 0.0f;
    }
    if (phase == 5u && !state->step_climb_balance_handoff) {
        const float mean_theta = 0.5f *
            (state->leg[0].theta + state->leg[1].theta);
        const float mean_length = 0.5f *
            (state->leg[0].length + state->leg[1].length);
        if (WheelLegCoreAbs(pitch) < 0.55f &&
            WheelLegCoreAbs(sensor[5]) < 2.5f &&
            WheelLegCoreAbs(mean_theta - state->theta0) <
                (short_fold ? 0.30f : 0.70f) &&
            mean_length > (short_fold ? 0.125f : 0.185f) &&
            mean_length < 0.23f) {
            state->step_climb_balance_handoff = 1u;
            state->step_climb_balance_time = 0.0f;
        }
    }
    for (int side = 0; side < 2; side++) {
        const int base = 10 + side * 9;
        const float sign = side == 0 ? -1.0f : 1.0f;
        float world_x = state->step_climb_foot[side][0] +
            blend*(target_x - state->step_climb_foot[side][0]);
        float world_y = state->step_climb_foot[side][1] +
            blend*(target_y - state->step_climb_foot[side][1]);
        if (phase == 3u) {
            const float lift_s = state->climb_assist_latched ? 0.12f : 0.18f;
            if (state->step_climb_elapsed < lift_s) {
                const float lift_blend = OwnWbrSmoothStep(
                    state->step_climb_elapsed / lift_s);
                world_x = state->step_climb_foot[side][0] +
                    lift_blend*(0.30f - state->step_climb_foot[side][0]);
                world_y = state->step_climb_foot[side][1] +
                    lift_blend*(0.06f - state->step_climb_foot[side][1]);
            } else {
                const float forward_blend = OwnWbrSmoothStep(
                    (state->step_climb_elapsed - lift_s) / (move_s - lift_s));
                world_x = 0.30f + forward_blend*(target_x - 0.30f);
                world_y = 0.06f;
            }
        }
        WheelLegCoreFootPoint point;
        point.x_m = cosf(pitch)*world_x - sinf(pitch)*world_y;
        point.y_m = sinf(pitch)*world_x + cosf(pitch)*world_y;
        point.length_m = sqrtf(world_x*world_x + world_y*world_y);
        const float min_length = short_fold ? 0.130f : 0.190f;
        if (point.length_m < min_length) {
            if (point.length_m < 1e-5f) {
                return 9;
            }
            const float scale = min_length / point.length_m;
            point.x_m *= scale;
            point.y_m *= scale;
            point.length_m = min_length;
        }
        if (short_fold && phase == 4u) {
            const float start_x = state->step_climb_foot[side][0];
            const float start_y = state->step_climb_foot[side][1];
            const float start_length = sqrtf(start_x*start_x + start_y*start_y);
            const float start_alpha = atan2f(start_x, start_y);
            const float length = start_length + blend*(0.130f - start_length);
            const float fold_theta = state->theta0 - 0.10f;
            const float alpha = start_alpha + blend*(fold_theta - start_alpha);
            point.x_m = length*sinf(alpha - pitch);
            point.y_m = length*cosf(alpha - pitch);
            point.length_m = length;
        }
        float front;
        float back;
        const float front_seed = 1.804837140117f - sign*sensor[base+3];
        const float back_seed = -1.486361593218f - sign*sensor[base];
        if (!WheelLegCoreInversePoint(&s_geometry, &point, front_seed, back_seed,
                                      &front, &back)) {
            return 9;
        }
        const float q1 = WheelLegCoreNearAngle((-1.486361593218f-back)/sign,
                                               state->step_climb_q[side][0]);
        const float q4 = WheelLegCoreNearAngle((1.804837140117f-front)/sign,
                                               state->step_climb_q[side][1]);

        float spring_q1;
        float spring_q4;
        OwnWbrGasSpringTorque(state, side, sensor[base], sensor[base + 3],
                              &spring_q1, &spring_q4);
        const float limit = phase == 6u && state->step_climb_elapsed >= 0.30f ?
            state->joint_continuous : state->joint_peak;
        if (phase == 1u && state->climb_assist_latched) {
            const float e1 = q1 - sensor[base];
            const float e4 = q4 - sensor[base+3];
            const float e_common = 0.5f*(e1 + e4);
            const float e_diff = 0.5f*(e1 - e4);
            const float v_common = 0.5f*(sensor[base+1] + sensor[base+4]);
            const float v_diff = 0.5f*(sensor[base+1] - sensor[base+4]);
            const float common = 40.0f*e_common - 0.5f*v_common;
            const float diff = 300.0f*e_diff - 10.0f*v_diff;
            const float body_tp = -80.0f*pitch - 8.0f*sensor[5];
            const float body = -sign*0.5f*body_tp;
            out[side*3] = WheelLegCoreClamp(common + diff + body - spring_q1,
                                            -limit, limit);
            out[side*3+1] = WheelLegCoreClamp(common - diff + body - spring_q4,
                                              -limit, limit);
        } else {
            out[side*3] = WheelLegCoreClamp(300.0f*(q1-sensor[base]) -
                10.0f*sensor[base+1] - spring_q1, -limit, limit);
            out[side*3+1] = WheelLegCoreClamp(300.0f*(q4-sensor[base+3]) -
                10.0f*sensor[base+4] - spring_q4, -limit, limit);
        }
        float wheel = 0.6f;
        if (phase == 1u) {
            wheel = WheelLegCoreClamp(-0.4f*state->observer.v_mps, -1.0f, 0.0f);
        } else if (phase == 2u || phase == 3u) {
            /* 提轮时只抑制自身空转，不把它当车速反馈放大。 */
            wheel = WheelLegCoreClamp(-0.12f*sign*sensor[base+7], -3.0f, 3.0f);
        } else if (phase == 4u) {
            wheel = 0.0f;
            if (state->step_climb_p4_brake_latched) {
                const float brake_blend = OwnWbrSmoothStep(
                    state->step_climb_p4_brake_time / 0.12f);
                const float brake = WheelLegCoreClamp(
                    -1.2f*state->observer.v_mps, -1.2f, 1.2f);
                wheel += brake_blend*(brake - wheel);
            }
        } else if (phase == 5u) {
            /* 腿负责相对前送支点，轮端只消掉阶段 4 留下的平动。 */
            wheel = WheelLegCoreClamp(-1.2f*state->observer.v_mps, -1.2f, 1.2f);
        }
        if (short_fold &&
            (phase == 4u || (phase == 5u && !state->step_climb_balance_handoff))) {
            wheel = WheelLegCoreClamp(25.0f*pitch + 3.0f*sensor[5] -
                4.0f*state->observer.v_mps, -5.2f, 5.2f);
        }
        out[side*3+2] = sign*wheel;
    }
    if (phase == 4u && state->step_climb_p4_brake_latched) {
        state->step_climb_p4_brake_time += 0.003f;
    }
    if (phase == 5u && state->step_climb_balance_handoff) {
        const float handoff_blend = OwnWbrSmoothStep(
            state->step_climb_balance_time / 0.12f);
        for (int index = 0; index < 6; index++) {
            out[index] += handoff_blend * (normal_out[index] - out[index]);
        }
        state->step_climb_balance_time += 0.003f;
    }
    for (int side = 0; side < 2; side++) {
        for (int joint = 0; joint < 2; joint++) {
            const int output_index = side*3 + joint;
            if (WheelLegCoreAbs(out[output_index]) > state->joint_continuous) {
                state->step_climb_peak_time[side][joint] += 0.003f;
            }
            if (state->step_climb_peak_time[side][joint] >= 3.50f) {
                /* 3.5 s 是保守控制预算，不代表厂家的电机热允许时间。 */
                return 10;
            }
        }
    }
    out[31] = state->joint_peak;

    uint8_t advance = state->step_climb_elapsed >= move_s + 0.05f;
    if (short_fold && phase == 4u) {
        const float mean_theta = 0.5f *
            (state->leg[0].theta + state->leg[1].theta);
        const float mean_length = 0.5f *
            (state->leg[0].length + state->leg[1].length);
        advance = state->step_climb_elapsed >= move_s &&
            WheelLegCoreAbs(mean_theta - state->theta0) < 0.16f &&
            mean_length < 0.155f;
    } else if (phase == 4u && state->step_climb_elapsed >= 0.33f) {
        const float mean_theta = 0.5f *
            (state->leg[0].theta + state->leg[1].theta);
        const float mean_length = 0.5f *
            (state->leg[0].length + state->leg[1].length);
        advance = advance || (WheelLegCoreAbs(pitch) < 0.55f &&
            WheelLegCoreAbs(sensor[5]) < 2.5f &&
            WheelLegCoreAbs(mean_theta - state->theta0) < 0.70f &&
            mean_length > 0.185f && mean_length < 0.23f);
    } else if (phase == 5u) {
        const float mean_theta = 0.5f *
            (state->leg[0].theta + state->leg[1].theta);
        const float mean_length = 0.5f *
            (state->leg[0].length + state->leg[1].length);
        const float mean_d_length = 0.5f *
            (WheelLegCoreAbs(state->leg[0].d_length) +
             WheelLegCoreAbs(state->leg[1].d_length));
        const uint8_t stable = WheelLegCoreAbs(pitch) < 0.0873f &&
            WheelLegCoreAbs(out[6]) < 0.0873f &&
            WheelLegCoreAbs(sensor[4]) < 0.35f &&
            WheelLegCoreAbs(sensor[6]) < 0.35f &&
            WheelLegCoreAbs(sensor[5]) < 0.35f &&
            WheelLegCoreAbs(state->observer.v_mps) < 0.12f &&
            WheelLegCoreAbs(mean_theta - state->theta0) < 0.08f &&
            mean_length > 0.19f && mean_length < 0.22f &&
            mean_d_length < 0.18f;
        state->step_climb_ready_time = stable ?
            state->step_climb_ready_time + 0.003f : 0.0f;
        advance = state->step_climb_elapsed >= move_s &&
            state->step_climb_ready_time >= 0.18f;
    }
    if (advance) {
        state->step_climb_phase = phase >= 5u ? 7u : (uint8_t)(phase + 1u);
        state->step_climb_elapsed = 0.0f;
        state->step_climb_ready_time = 0.0f;
        OwnWbrStepClimbCapture(state, sensor, pitch);
        if (state->step_climb_phase == 5u) {
            state->length = 0.20f;
            state->length_target = 0.20f;
            state->stages.target_length_m = 0.20f;
            state->step_climb_balance_handoff = 0u;
            state->step_climb_balance_time = 0.0f;
            state->step_climb_p4_brake_latched = 0u;
            state->step_climb_p4_brake_time = 0.0f;
        }
        if (state->step_climb_phase == 7u) {
            state->length = 0.20f;
            state->length_target = 0.20f;
            state->stages.target_length_m = 0.20f;
            state->x_ref = state->observer.x_m;
            state->v_ref = 0.0f;
            state->yaw_ref = yaw;
            state->step_climb_hold = 1u;
            state->step_climb_park_shifted = 0u;
            /* 越台期间观察器一直有效；只对齐位置参考，保留真实速度动量。 */
        }
    }
    return 0;
}

static int OwnWbrStepClimbOverride(OwnWbrState *state, const float *sensor,
                                   float pitch, float yaw,
                                   const float *normal_out, float *out)
{
    return state->step_climb_high ?
        OwnWbrStepClimbHighOverride(state, sensor, pitch, yaw, normal_out, out) :
        OwnWbrStepClimbLowOverride(state, sensor, pitch, yaw, out);
}

static void OwnWbrResetBalance(OwnWbrState *state, float yaw)
{
    memset(&state->observer, 0, sizeof(state->observer));
    state->x_ref = 0.0f;
    state->v_ref = 0.0f;
    state->yaw_ref = yaw;
    state->yaw_rate = 0.0f;
    state->first = 0u;
    for (int side = 0; side < 2; side++) {
        state->leg[side].first = 0u;
        WheelLegCorePidClear(&state->length_pid[side]);
    }
    WheelLegCorePidClear(&state->split_pid);
}

static uint8_t OwnWbrPrepareRise(OwnWbrState *state, const float *sensor)
{
    WheelLegCoreFootPoint target;
    float front_target;
    float back_target;

    /* 起身先把轮子收向机身下方，再交给平衡环，减小交接时追赶支撑点的位移。 */
    const float target_theta = state->theta0 +
        (state->stages.stage == 3u && !state->recovery_requested ?
         OWN_WBR_RISE_TARGET_THETA_BIAS : 0.0f);
    target.x_m = sinf(target_theta) * state->length;
    target.y_m = cosf(target_theta) * state->length;
    target.length_m = state->length;
    if (!WheelLegCoreInversePoint(&s_geometry, &target, 1.25f, -1.38f,
                                  &front_target, &back_target)) {
        return 0u;
    }
    for (int side = 0; side < 2; side++) {
        const int base = 10 + side * 9;
        const float sign = side == 0 ? -1.0f : 1.0f;
        state->rise_start_q[side][0] = sensor[base];
        state->rise_start_q[side][1] = sensor[base + 3];
        state->recovery_start_foot[side][0] =
            state->leg[side].length * sinf(state->leg[side].alpha);
        state->recovery_start_foot[side][1] =
            state->leg[side].length * cosf(state->leg[side].alpha);
        state->rise_target_q[side][0] = (-1.486361593218f - back_target) / sign;
        state->rise_target_q[side][1] = (1.804837140117f - front_target) / sign;
        if (state->recovery_requested) {
            state->rise_target_q[side][0] = WheelLegCoreNearAngle(
                state->rise_target_q[side][0], state->rise_start_q[side][0]);
            state->rise_target_q[side][1] = WheelLegCoreNearAngle(
                state->rise_target_q[side][1], state->rise_start_q[side][1]);
        }
    }
    state->rise_initialized = 1u;
    return 1u;
}

static void OwnWbrFillRiseOutput(OwnWbrState *state, float roll, float pitch, float yaw,
                                 float joint_limit, float *out)
{
    for (int side = 0; side < 2; side++) {
        const WheelLegCoreLegCalc *leg = &state->leg[side];
        out[12+side*6] = leg->length;
        out[13+side*6] = leg->d_length;
        out[14+side*6] = leg->theta;
        out[15+side*6] = leg->d_theta;
    }
    out[6] = roll; out[7] = pitch; out[8] = yaw;
    out[24] = 1.0f;
    out[27] = state->yaw_ref;
    out[29] = (float)state->jump_phase;
    out[30] = (float)state->jump_count;
    out[31] = joint_limit;
    out[32] = (float)state->stages.stage;
    out[33] = state->stages.target_length_m;
    out[34] = (float)state->rise_phase;
    out[35] = state->rise_elapsed;
}

static uint8_t OwnWbrRisePositionStep(OwnWbrState *state, const float *sensor,
                                      float roll, float pitch, float yaw, float gyro,
                                      float *out)
{
    const float dt = 0.003f;
    const uint8_t fast_rise = state->stages.stage == 3u;
    const float move_time = fast_rise ? OWN_WBR_RISE_MOVE_S : 3.0f;
    float joint_limit = state->rise_phase == 2 ? state->joint_peak : state->joint_continuous;
    float blend = state->rise_phase == 2 ?
        OwnWbrSmoothStep(state->rise_phase_time / move_time) : 0.0f;
    float max_error = 0.0f;

    if (!state->rise_initialized && !OwnWbrPrepareRise(state, sensor)) {
        state->rise_phase = -2;
        OwnWbrFillRiseOutput(state, roll, pitch, yaw, 0.0f, out);
        return 1u;
    }
    for (int side = 0; side < 2; side++) {
        const int base = 10 + side * 9;
        const float sign = side == 0 ? -1.0f : 1.0f;
        for (int joint = 0; joint < 2; joint++) {
            const int sensor_offset = joint == 0 ? 0 : 3;
            const float target = state->rise_start_q[side][joint] +
                (state->rise_target_q[side][joint] - state->rise_start_q[side][joint]) * blend;
            const float position = sensor[base + sensor_offset];
            const float velocity = sensor[base + sensor_offset + 1];
            const float error = target - position;
            const int output_index = side * 3 + joint;
            out[output_index] = 300.0f * error - 10.0f * velocity;
            if (WheelLegCoreAbs(error) > max_error) {
                max_error = WheelLegCoreAbs(error);
            }
        }
        float spring_q1;
        float spring_q4;
        OwnWbrGasSpringTorque(state, side, sensor[base], sensor[base + 3],
                              &spring_q1, &spring_q4);
        out[side*3] = WheelLegCoreClamp(out[side*3] - spring_q1,
                                        -joint_limit, joint_limit);
        out[side*3+1] = WheelLegCoreClamp(out[side*3+1] - spring_q4,
                                          -joint_limit, joint_limit);
        out[36+side*2] = spring_q1;
        out[37+side*2] = spring_q4;
        if (state->rise_phase == 2 && fast_rise) {
            const float wheel = WheelLegCoreClamp(
                OWN_WBR_RISE_WHEEL_PITCH_KP * pitch +
                OWN_WBR_RISE_WHEEL_GYRO_KD * gyro, -5.2f, 5.2f);
            out[side*3+2] = sign * wheel;
        }
    }
    OwnWbrFillRiseOutput(state, roll, pitch, yaw, joint_limit, out);
    state->rise_phase_time += dt;
    state->rise_elapsed += dt;

    if (state->rise_phase == 1 && state->rise_phase_time >=
        (fast_rise ? OWN_WBR_RISE_HOLD_S : 0.30f)) {
        state->rise_phase = 2;
        state->rise_phase_time = 0.0f;
    } else if (state->rise_phase == 2 && state->rise_phase_time >= move_time &&
               max_error < 0.16f && WheelLegCoreAbs(pitch) < 0.70f &&
               WheelLegCoreAbs(gyro) < 3.0f) {
        state->rise_phase = 3;
        state->rise_phase_time = 0.0f;
        state->rise_stable_time = 0.0f;
        OwnWbrResetBalance(state, yaw);
    } else if (state->rise_phase == 2 && state->rise_phase_time >= 4.5f) {
        state->rise_phase = -3;
    }
    return 1u;
}

static uint8_t OwnWbrRecoveryPositionStep(OwnWbrState *state, const float *sensor,
                                           float roll, float pitch, float yaw,
                                           float gyro, float *out)
{
    const float dt = 0.003f;
    const float move_time = state->rise_phase == 14 ? 2.50f : 0.24f;
    const float blend = OwnWbrSmoothStep(state->rise_phase_time / move_time);
    /* 各髋电机分别累计超连续力矩时间；这不是电机热模型或厂家峰值时长。 */
    float reported_joint_limit = state->joint_continuous;
    float max_error = 0.0f;

    /* 恢复期间只信绝对速度大小，不用倒扣时可能符号相反的车速估计参与保护。 */
    for (int side = 0; side < 2; side++) {
        const int base = 10 + side * 9;
        if (WheelLegCoreAbs(sensor[base + 7]) >
            OWN_WBR_RECOVERY_WHEEL_SPEED_LIMIT_RADPS) {
            state->rise_phase = -17;
        }
        for (int joint = 0; joint < 2; joint++) {
            const int sensor_offset = joint == 0 ? 0 : 3;
            if (WheelLegCoreAbs(sensor[base + sensor_offset + 1]) >
                OWN_WBR_RECOVERY_JOINT_SPEED_LIMIT_RADPS) {
                state->rise_phase = -15;
            }
            if (WheelLegCoreAbs(sensor[base + sensor_offset] -
                                state->recovery_origin_q[side][joint]) >
                OWN_WBR_RECOVERY_JOINT_TRAVEL_LIMIT_RAD) {
                state->rise_phase = -16;
            }
        }
    }
    const float body_rate = sqrtf(sensor[4] * sensor[4] + sensor[5] * sensor[5] +
                                  sensor[6] * sensor[6]);
    if (body_rate > OWN_WBR_RECOVERY_BODY_RATE_LIMIT_RADPS) {
        state->rise_phase = -20;
    }
    if (state->rise_phase < 0) {
        memset(out, 0, 40 * sizeof(*out));
        OwnWbrFillRiseOutput(state, roll, pitch, yaw, 0.0f, out);
        return 1u;
    }

    if (state->rise_phase == 14 && !state->rise_initialized &&
        !OwnWbrPrepareRise(state, sensor)) {
        state->rise_phase = -12;
        memset(out, 0, 40 * sizeof(*out));
        OwnWbrFillRiseOutput(state, roll, pitch, yaw, 0.0f, out);
        return 1u;
    }

    for (int side = 0; side < 2; side++) {
        const int base = 10 + side * 9;
        float recovery_target_q[2] = {0.0f, 0.0f};
        if (state->rise_phase == 14) {
            const float sign = side == 0 ? -1.0f : 1.0f;
            const float start_x = state->recovery_start_foot[side][0];
            const float start_y = state->recovery_start_foot[side][1];
            const float start_length = sqrtf(start_x * start_x + start_y * start_y);
            const float start_alpha = atan2f(start_x, start_y);
            const float rotate_blend = OwnWbrSmoothStep(blend / 0.55f);
            const float shorten_blend = OwnWbrSmoothStep((blend - 0.55f) / 0.45f);
            const float foot_alpha = start_alpha +
                WheelLegCoreWrapPi(state->theta0 - start_alpha) * rotate_blend;
            const float foot_length = start_length +
                (state->length - start_length) * shorten_blend;
            const float foot_x = sinf(foot_alpha) * foot_length;
            const float foot_y = cosf(foot_alpha) * foot_length;
            const float front_ref = 1.804837140117f - sign * sensor[base + 3];
            const float back_ref = -1.486361593218f - sign * sensor[base];
            WheelLegCoreFootPoint foot = {
                .x_m = foot_x,
                .y_m = foot_y,
                .length_m = sqrtf(foot_x * foot_x + foot_y * foot_y)
            };
            float front_target;
            float back_target;
            if (!WheelLegCoreInversePoint(&s_geometry, &foot, front_ref, back_ref,
                                          &front_target, &back_target)) {
                state->rise_phase = -12;
                memset(out, 0, 40 * sizeof(*out));
                OwnWbrFillRiseOutput(state, roll, pitch, yaw, 0.0f, out);
                return 1u;
            }
            recovery_target_q[0] = WheelLegCoreNearAngle(
                (-1.486361593218f - back_target) / sign, sensor[base]);
            recovery_target_q[1] = WheelLegCoreNearAngle(
                (1.804837140117f - front_target) / sign, sensor[base + 3]);
        }
        for (int joint = 0; joint < 2; joint++) {
            const int sensor_offset = joint == 0 ? 0 : 3;
            const float velocity = sensor[base + sensor_offset + 1];
            const int output_index = side * 3 + joint;
            float target_velocity = 0.0f;
            if (state->rise_phase <= 13) {
                static const int8_t sweep_direction[4][4] = {
                    { 1,  1, -1, -1},
                    { 1,  1,  1,  1},
                    {-1, -1,  1,  1},
                    { 1,  1, -1, -1}
                };
                const int path = (int)state->recovery_path - 1;
                const float direction =
                    (float)sweep_direction[path][side * 2 + joint];
                const float travel = WheelLegCoreClamp(
                    state->rise_elapsed * OWN_WBR_RECOVERY_TARGET_RATE_RADPS,
                    0.0f, OWN_WBR_RECOVERY_SWEEP_RAD);
                const float raw_target = state->rise_initialized ?
                    state->rise_target_q[side][joint] :
                    state->recovery_origin_q[side][joint] + direction * travel;
                const float max_step = OWN_WBR_RECOVERY_TARGET_RATE_RADPS * dt;
                const float command_error = raw_target -
                    state->recovery_command_q[side][joint];
                state->recovery_command_q[side][joint] += WheelLegCoreClamp(
                    command_error, -max_step, max_step);
                target_velocity = state->rise_initialized ||
                    travel >= OWN_WBR_RECOVERY_SWEEP_RAD ? 0.0f :
                    direction * OWN_WBR_RECOVERY_TARGET_RATE_RADPS;
                const float error = state->recovery_command_q[side][joint] -
                    sensor[base + sensor_offset];
                out[output_index] = 32.0f * error +
                    4.0f * (target_velocity - velocity);
            } else {
                const float max_step = OWN_WBR_RECOVERY_TARGET_RATE_RADPS * dt;
                const float command_error = recovery_target_q[joint] -
                    state->recovery_command_q[side][joint];
                state->recovery_command_q[side][joint] += WheelLegCoreClamp(
                    command_error, -max_step, max_step);
                const float error = state->recovery_command_q[side][joint] -
                    sensor[base + sensor_offset];
                out[output_index] = 45.0f * error - 7.0f * velocity;
                if (WheelLegCoreAbs(error) > max_error) {
                    max_error = WheelLegCoreAbs(error);
                }
            }
        }
        if (state->rise_phase == 14) {
            float spring_q1;
            float spring_q4;
            OwnWbrGasSpringTorque(state, side, sensor[base], sensor[base + 3],
                                  &spring_q1, &spring_q4);
            out[side*3] -= spring_q1;
            out[side*3+1] -= spring_q4;
            out[36+side*2] = spring_q1;
            out[37+side*2] = spring_q4;
        }
        for (int joint = 0; joint < 2; joint++) {
            const int sensor_offset = joint == 0 ? 0 : 3;
            const int output_index = side * 3 + joint;
            const uint8_t peak_allowed =
                state->recovery_peak_time[side][joint] < OWN_WBR_RECOVERY_PEAK_TIME_S &&
                state->recovery_work_j[side][joint] < OWN_WBR_RECOVERY_PEAK_WORK_J;
            const float budget_limit = peak_allowed ?
                state->joint_peak : state->joint_continuous;
            const float joint_limit = fminf(budget_limit,
                                             OWN_WBR_RECOVERY_JOINT_TORQUE_LIMIT_NM);
            out[output_index] = WheelLegCoreClamp(out[output_index],
                                                  -joint_limit, joint_limit);
            if (WheelLegCoreAbs(out[output_index]) > state->joint_continuous) {
                state->recovery_peak_time[side][joint] += dt;
            }
            state->recovery_work_j[side][joint] += dt * WheelLegCoreAbs(
                out[output_index] * sensor[base+sensor_offset+1]);
            if (joint_limit > reported_joint_limit) {
                reported_joint_limit = joint_limit;
            }
        }
        if (state->rise_phase == 14 && blend > 0.80f &&
            cosf(state->leg[0].alpha) > 0.35f &&
            cosf(state->leg[1].alpha) > 0.35f) {
            const float sign = side == 0 ? -1.0f : 1.0f;
            const float wheel = WheelLegCoreClamp(
                OWN_WBR_RECOVERY_WHEEL_PITCH_KP * pitch +
                OWN_WBR_RISE_WHEEL_GYRO_KD * gyro -
                OWN_WBR_RECOVERY_WHEEL_DAMPING * sign * sensor[base + 7],
                -OWN_WBR_RECOVERY_WHEEL_TORQUE_LIMIT_NM,
                OWN_WBR_RECOVERY_WHEEL_TORQUE_LIMIT_NM);
            out[side*3+2] = sign * wheel;
        } else {
            out[side*3+2] = WheelLegCoreClamp(
                -0.15f * sensor[base + 7],
                -OWN_WBR_RECOVERY_WHEEL_TORQUE_LIMIT_NM,
                OWN_WBR_RECOVERY_WHEEL_TORQUE_LIMIT_NM);
        }
    }
    OwnWbrFillRiseOutput(state, roll, pitch, yaw, reported_joint_limit, out);
    state->rise_phase_time += dt;
    state->rise_elapsed += dt;

    if (state->rise_phase == 12 && state->rise_phase_time >= move_time) {
        state->rise_phase = 13;
        state->rise_phase_time = 0.0f;
        state->rise_stable_time = 0.0f;
    } else if (state->rise_phase == 13) {
        if (!state->rise_initialized && WheelLegCoreAbs(roll) < 0.436332f &&
            WheelLegCoreAbs(pitch) < 0.436332f) {
            /* 接近直立后锁住零转速，用电机制动消掉翻身惯性，再收腿。 */
            state->rise_initialized = 1u;
            for (int side = 0; side < 2; side++) {
                const int base = 10 + side * 9;
                state->rise_target_q[side][0] = sensor[base];
                state->rise_target_q[side][1] = sensor[base + 3];
                state->recovery_command_q[side][0] = sensor[base];
                state->recovery_command_q[side][1] = sensor[base + 3];
            }
        }
        const uint8_t hips_stopped = WheelLegCoreAbs(sensor[11]) < 0.8f &&
            WheelLegCoreAbs(sensor[14]) < 0.8f &&
            WheelLegCoreAbs(sensor[20]) < 0.8f &&
            WheelLegCoreAbs(sensor[23]) < 0.8f;
        const uint8_t stopped_upright = state->rise_initialized && hips_stopped &&
            WheelLegCoreAbs(roll) < 0.436332f &&
            WheelLegCoreAbs(pitch) < 0.436332f &&
            WheelLegCoreAbs(sensor[4]) < 0.8f && WheelLegCoreAbs(gyro) < 0.8f;
        state->rise_stable_time = stopped_upright ?
            state->rise_stable_time + dt : 0.0f;
        if (state->rise_stable_time >= 0.30f) {
            state->rise_initialized = 0u;
            state->rise_phase = 14;
            state->rise_phase_time = 0.0f;
            state->rise_stable_time = 0.0f;
        } else {
            const float tilt = acosf(WheelLegCoreClamp(
                cosf(roll) * cosf(pitch), -1.0f, 1.0f));
            if (tilt < state->recovery_best_tilt - 0.087266f) {
                state->recovery_best_tilt = tilt;
                state->recovery_progress_time = state->rise_elapsed;
            }
            if (state->rise_elapsed - state->recovery_progress_time >=
                OWN_WBR_RECOVERY_NO_PROGRESS_S) {
                state->rise_phase = -18;
            } else if (state->rise_elapsed >= OWN_WBR_RECOVERY_TOTAL_SWEEP_S) {
                state->rise_phase = -19;
            }
        }
    } else if (state->rise_phase == 14) {
        const uint8_t wheels_below = cosf(state->leg[0].alpha) > 0.35f &&
            cosf(state->leg[1].alpha) > 0.35f;
        const uint8_t posture_ready = max_error < 0.20f &&
            WheelLegCoreAbs(roll) < 0.52f && WheelLegCoreAbs(pitch) < 0.52f &&
            WheelLegCoreAbs(sensor[4]) < 0.8f && WheelLegCoreAbs(gyro) < 0.8f;
        state->rise_stable_time = posture_ready && wheels_below ?
            state->rise_stable_time + dt : 0.0f;
        if (state->rise_phase_time >= move_time && state->rise_stable_time >= 0.15f) {
            OwnWbrResetBalance(state, yaw);
            state->rise_phase = 15;
            state->rise_phase_time = 0.0f;
            state->rise_stable_time = 0.0f;
        } else if (state->rise_phase_time >= 6.0f) {
            state->rise_phase = -14;
        }
    }
    if (state->rise_phase < 0) {
        memset(out, 0, 40 * sizeof(*out));
        OwnWbrFillRiseOutput(state, roll, pitch, yaw, 0.0f, out);
    }
    return 1u;
}

static void OwnWbrJumpCommand(OwnWbrState *state, float pressed,
                              float target_v,
                              float roll, float pitch,
                              float roll_gyro, float pitch_gyro, float yaw_gyro,
                              float *length, float *support)
{
    const float dt = 0.003f;
    const uint8_t button = pressed > 0.5f;
    if (state->jump_phase == 0u && button && !state->jump_pressed) {
        if (state->step_jump && !OwnWbrStepJumpConfigValid(state)) {
            state->jump_prepare_cancelled = 1u;
            state->jump_pressed = button;
            return;
        }
        state->jump_prepare_elapsed = 0.0f;
        state->jump_prepare_cancelled = 0u;
        state->jump_profile = state->jump_profile_requested;
        state->jump_reverse_prepare = state->jump_profile == 2u &&
            fminf(target_v, fminf(state->v_ref, state->observer.v_mps)) < -1.0f;
        if (state->jump_profile == 3u) {
            const float motion_speed = WheelLegCoreAbs(target_v) > 0.01f ?
                target_v : (WheelLegCoreAbs(state->v_ref) >= WheelLegCoreAbs(state->observer.v_mps) ?
                    state->v_ref : state->observer.v_mps);
            state->jump_drive_speed = state->step_jump ?
                WheelLegCoreClamp(motion_speed, -1.0f, 1.0f) :
                WheelLegCoreClamp(motion_speed, -0.25f, 0.25f);
        }
    }
    if (state->jump_profile >= 2u) {
        const uint8_t enhanced = state->jump_profile == 3u;
        const uint8_t moving_enhanced = enhanced && state->step_jump &&
            WheelLegCoreAbs(state->jump_drive_speed) > 0.05f;
        const float crouch_time = moving_enhanced ? 0.54f : (enhanced ? 0.55f : 0.20f);
        const float push_time = moving_enhanced ? 0.111f : (enhanced ? 0.12f : 0.080f);
        const float flight_time = enhanced ? 0.46f : 0.36f;
        const float land_time = enhanced ? 0.65f : 0.58f;
        const float mean_length = 0.5f * (state->leg[0].length + state->leg[1].length);
        const float mean_d_length = 0.5f * (state->leg[0].d_length + state->leg[1].d_length);
        const float jump_pitch_limit = state->step_climb_enabled == 2u &&
            state->step_climb_phase == 8u ? 0.0873f : 0.0524f;
        const uint8_t posture_ready =
            WheelLegCoreAbs(roll) < 0.0873f &&
            WheelLegCoreAbs(pitch) < jump_pitch_limit &&
            WheelLegCoreAbs(roll_gyro) < 0.50f && WheelLegCoreAbs(pitch_gyro) < 0.35f &&
            WheelLegCoreAbs(yaw_gyro) < 0.35f &&
            WheelLegCoreAbs(state->leg[0].theta - state->leg[1].theta) < 0.12f &&
            WheelLegCoreAbs(state->leg[0].d_length) < 0.20f &&
            WheelLegCoreAbs(state->leg[1].d_length) < 0.20f;

        if (state->jump_phase == 0u && button && !state->jump_pressed) {
            state->jump_phase = 1u;
            state->jump_time = 0.0f;
            state->jump_stable_time = 0.0f;
            state->jump_push_offset = 0.0f;
            /* 各档腿高先回到同一已验证基准，动作结束后再恢复用户目标。 */
            state->jump_base_length = 0.20f;
            if (state->step_jump && enhanced) {
                state->step_jump_hold = 1u;
            }
            state->jump_count++;
        }
        state->jump_pressed = button;
        if (state->jump_phase == 1u && state->jump_time <= 0.0f) {
            state->jump_prepare_elapsed += dt;
            if (state->jump_prepare_elapsed >= 10.0f) {
                /* 收腿预算耗尽或姿态未就绪时退出准备，继续平衡；长按不能重新触发。 */
                state->jump_phase = 0u;
                state->jump_time = 0.0f;
                state->jump_stable_time = 0.0f;
                state->jump_prepare_elapsed = 0.0f;
                state->jump_prepare_cancelled = 1u;
                state->step_jump_hold = 0u;
                state->jump_reverse_prepare = 0u;
                *length = state->length;
                *support = state->support;
                return;
            }
        }
        if (state->jump_phase == 1u && state->jump_time <= 0.0f && moving_enhanced) {
            const float prepare_step = 0.04f * dt;
            state->length += WheelLegCoreClamp(state->jump_base_length - state->length,
                                               -prepare_step, prepare_step);
            const uint8_t step_length_ready =
                WheelLegCoreAbs(state->length - state->jump_base_length) < 0.003f &&
                WheelLegCoreAbs(state->leg[0].length - state->jump_base_length) < 0.030f &&
                WheelLegCoreAbs(state->leg[1].length - state->jump_base_length) < 0.030f;
            const float step_direction = state->jump_drive_speed > 0.0f ? 1.0f : -1.0f;
            const float step_speed = WheelLegCoreAbs(state->jump_drive_speed);
            const uint8_t step_speed_ready =
                step_direction * state->observer.v_mps >= 0.80f * step_speed &&
                WheelLegCoreAbs(state->observer.v_mps - state->jump_drive_speed) < 0.15f &&
                step_direction * state->v_ref >= 0.80f * step_speed &&
                step_direction * state->v_ref <= step_speed + 0.25f;
            const uint8_t step_posture_ready =
                WheelLegCoreAbs(roll) < 0.14f && WheelLegCoreAbs(pitch) < 0.14f &&
                WheelLegCoreAbs(roll_gyro) < 1.0f && WheelLegCoreAbs(pitch_gyro) < 1.0f &&
                WheelLegCoreAbs(yaw_gyro) < 1.0f &&
                WheelLegCoreAbs(state->leg[0].theta - state->leg[1].theta) < 0.18f &&
                WheelLegCoreAbs(state->leg[0].d_length) < 0.35f &&
                WheelLegCoreAbs(state->leg[1].d_length) < 0.35f;
            *length = state->length;
            *support = state->support;
            if (!step_length_ready || !step_speed_ready || !step_posture_ready) {
                return;
            }
        }
        if (state->jump_phase == 1u && state->jump_time <= 0.0f && !moving_enhanced) {
            const float prepare_step = 0.04f * dt;
            state->length += WheelLegCoreClamp(state->jump_base_length - state->length,
                                               -prepare_step, prepare_step);
            const uint8_t length_ready =
                WheelLegCoreAbs(state->length - state->jump_base_length) < 0.001f &&
                WheelLegCoreAbs(state->leg[0].length - state->jump_base_length) < 0.008f &&
                WheelLegCoreAbs(state->leg[1].length - state->jump_base_length) < 0.008f;
            const uint8_t speed_ready = enhanced ?
                (WheelLegCoreAbs(state->observer.v_mps - state->jump_drive_speed) < 0.05f) :
                (!state->jump_reverse_prepare ||
                 (WheelLegCoreAbs(state->v_ref + 1.0f) < 0.03f &&
                  WheelLegCoreAbs(state->observer.v_mps + 1.0f) < 0.05f));
            state->jump_stable_time = length_ready && posture_ready && speed_ready ?
                state->jump_stable_time + dt : 0.0f;
            *length = state->length;
            *support = state->support;
            if (state->jump_stable_time < 0.12f) {
                return;
            }
            state->jump_stable_time = 0.0f;
        }
        if (state->jump_phase == 1u && state->jump_time <= 0.0f) {
            state->jump_prepare_elapsed = 0.0f;
        }
        if ((state->jump_phase == 1u &&
             (state->jump_time >= crouch_time ||
              (enhanced && !moving_enhanced && mean_length <= 0.185f))) ||
            (state->jump_phase == 2u &&
             (state->jump_time >= push_time ||
              (!moving_enhanced &&
               mean_length + WheelLegCoreClamp(mean_d_length, 0.0f, 2.0f) * 0.018f >= 0.315f))) ||
            (state->jump_phase == 3u &&
             (state->jump_time >= flight_time ||
              (state->jump_time >= 0.15f &&
               mean_d_length < -0.55f))) ||
            (state->jump_phase == 4u && state->jump_time >= land_time)) {
            if (state->jump_phase == 1u && !enhanced) {
                /* 起身后的实际腿长会略低于直接站立；按编码器短缺补足小跳蹬伸行程。 */
                state->jump_push_offset = WheelLegCoreClamp(2.0f *
                    (state->jump_base_length + 0.005f - mean_length), 0.0f, 0.018f);
            }
            if (state->jump_phase == 3u) {
                /* 电机反馈到腿长开始快速回缩，说明轮端已承重；保留伸腿进度再接落地段。 */
                state->jump_landing_time = WheelLegCoreClamp(state->jump_time, 0.0f,
                                                              flight_time);
            }
            state->jump_phase++;
            state->jump_time = 0.0f;
        } else if (state->jump_phase == 5u && state->jump_time >= 0.80f) {
            state->jump_phase = 0u;
            state->jump_time = 0.0f;
            state->jump_stable_time = 0.0f;
        }

        const float base_length = state->jump_phase == 0u ?
            state->length : state->jump_base_length;
        const float crouch_length = WheelLegCoreClamp(base_length -
                                                      (moving_enhanced ? 0.0f : 0.010f),
                                                      0.18f, 0.315f);
        const float push_length = WheelLegCoreClamp(enhanced ? 0.320f :
                                                    base_length + 0.090f + state->jump_push_offset,
                                                    0.18f, 0.320f);
        const float tuck_length = WheelLegCoreClamp(base_length + (enhanced ? -0.008f : 0.0f),
                                                    0.18f, 0.320f);
        const float landing_length = WheelLegCoreClamp(moving_enhanced ?
                                                       base_length + 0.09f : base_length + 0.10f,
                                                       0.18f, 0.320f);
        const float tuck_support = enhanced ? -30.0f : -20.0f;
        const float extend_start = enhanced ? 0.10f : 0.08f;
        const float small_push_support = 80.0f + WheelLegCoreClamp(
            3000.0f * state->jump_push_offset, 0.0f, 50.0f);
        *length = base_length;
        *support = state->support;
        if (state->jump_phase == 1u && enhanced) {
            const float blend = OwnWbrSmoothStep(state->jump_time / 0.385f);
            const float unload = sinf(WHEELLEG_CORE_PI *
                WheelLegCoreClamp(state->jump_time / 0.55f, 0.0f, 1.0f));
            *length = base_length + (crouch_length - base_length) * blend;
            *support = state->support - (moving_enhanced ? 15.0f : 40.0f) * unload;
        } else if (state->jump_phase == 2u) {
            const float blend = OwnWbrSmoothStep(state->jump_time / push_time);
            const float force_blend = OwnWbrSmoothStep(state->jump_time /
                (enhanced ? 0.020f : 0.015f));
            *length = crouch_length + (push_length - crouch_length) * blend;
            *support = state->support + (enhanced ? 124.0f : small_push_support) * force_blend;
        } else if (state->jump_phase == 3u) {
            const float tuck_time = 0.025f;
            const float tuck_delay = moving_enhanced ? 0.0f : 0.003f;
            if (state->jump_time < tuck_delay) {
                *length = push_length;
                *support = state->support + (enhanced ? 124.0f : small_push_support);
            } else if (state->jump_time < extend_start) {
                const float blend = OwnWbrSmoothStep(
                    (state->jump_time - tuck_delay) / tuck_time);
                *length = push_length + (tuck_length - push_length) * blend;
                *support = state->support + (tuck_support - state->support) * blend;
            } else {
                const float blend = OwnWbrSmoothStep((state->jump_time - extend_start) /
                    (flight_time - extend_start));
                *length = tuck_length + (landing_length - tuck_length) * blend;
                *support = tuck_support + (state->support - tuck_support) * blend;
            }
        } else if (state->jump_phase == 4u) {
            const float extend_left = enhanced ? 0.0f :
                fmaxf(0.0f, flight_time - state->jump_landing_time);
            if (!enhanced && state->jump_time < extend_left) {
                const float blend = OwnWbrSmoothStep((state->jump_landing_time +
                    state->jump_time - extend_start) / (flight_time - extend_start));
                *length = tuck_length + (landing_length - tuck_length) * blend;
                *support = tuck_support + (state->support - tuck_support) * blend;
            } else {
                const float blend = OwnWbrSmoothStep((state->jump_time - extend_left) /
                    fmaxf(0.001f, land_time - extend_left));
                *length = landing_length + (base_length - landing_length) * blend;
            }
        }
        if (state->jump_phase != 0u) {
            state->jump_time += dt;
        }
        return;
    }
    if (state->jump_profile == 1u) {
        if (state->jump_phase == 0u && button && !state->jump_pressed) {
            state->jump_phase = 1u;
            state->jump_time = 0.0f;
            state->jump_count++;
        }
        state->jump_pressed = button;
        if ((state->jump_phase == 1u && state->jump_time >= 0.55f &&
             WheelLegCoreAbs(state->v_ref) < 0.03f &&
             WheelLegCoreAbs(state->yaw_rate) < 0.05f) ||
            (state->jump_phase == 2u && state->jump_time >= 0.15f) ||
            (state->jump_phase == 3u && state->jump_time >= 0.25f) ||
            (state->jump_phase == 4u && state->jump_time >= 0.55f)) {
            state->jump_phase++;
            state->jump_time = 0.0f;
        } else if (state->jump_phase == 5u && state->jump_time >= 0.80f) {
            state->jump_phase = 0u;
            state->jump_time = 0.0f;
        }

        *length = state->length;
        *support = state->support;
        switch (state->jump_phase) {
        case 1u: {
            const float blend = OwnWbrSmoothStep(state->jump_time / 0.385f);
            const float unload = sinf(WHEELLEG_CORE_PI *
                WheelLegCoreClamp(state->jump_time / 0.55f, 0.0f, 1.0f));
            *length = state->length - 0.010f * blend;
            *support = state->support - 40.0f * unload;
            break;
        }
        case 2u: {
            const float blend = OwnWbrSmoothStep(state->jump_time / 0.15f);
            const float force_blend = OwnWbrSmoothStep(state->jump_time / 0.020f);
            *length = state->length - 0.010f + 0.130f * blend;
            *support = state->support + 400.0f * force_blend;
            break;
        }
        case 3u:
            if (state->jump_time < 0.003f) {
                *length = state->length + 0.120f;
                *support = state->support + 400.0f;
            } else if (state->jump_time < 0.15f) {
                const float blend = OwnWbrSmoothStep((state->jump_time - 0.003f) / 0.030f);
                *length = state->length + 0.120f - 0.120f * blend;
                *support = state->support + (-100.0f - state->support) * blend;
            } else {
                const float blend = OwnWbrSmoothStep((state->jump_time - 0.15f) / 0.10f);
                *length = state->length + 0.040f * blend;
                *support = -100.0f * (1.0f - blend);
            }
            break;
        case 4u: {
            const float blend = OwnWbrSmoothStep(state->jump_time / 0.55f);
            *length = state->length + 0.040f * (1.0f - blend);
            *support = state->support * blend;
            break;
        }
        case 5u:
            break;
        default:
            state->jump_phase = 0u;
            state->jump_time = 0.0f;
            break;
        }
        if (state->jump_phase != 0u) {
            state->jump_time += dt;
        }
        return;
    }
    const uint8_t motor_6216p = state->joint_continuous <= 10.5f && state->joint_peak >= 44.5f;
    const float push_time = motor_6216p ? 0.215f : 0.22f;

    if (state->jump_phase == 0u && button && !state->jump_pressed) {
        state->jump_phase = 1u;
        state->jump_time = 0.0f;
        state->jump_count++;
    }
    state->jump_pressed = button;

    if ((state->jump_phase == 1u && state->jump_time >= 0.55f &&
         WheelLegCoreAbs(state->v_ref) < 0.03f &&
         WheelLegCoreAbs(state->yaw_rate) < 0.05f) ||
        (state->jump_phase == 2u && state->jump_time >= push_time) ||
        (state->jump_phase == 3u && state->jump_time >= 0.138f) ||
        (state->jump_phase == 4u && state->jump_time >= 0.40f)) {
        state->jump_phase++;
        state->jump_time = 0.0f;
    } else if (state->jump_phase == 5u && state->jump_time >= 0.80f) {
        state->jump_phase = 0u;
        state->jump_time = 0.0f;
    }

    *length = state->length;
    *support = state->support;
    switch (state->jump_phase) {
    case 1u: {
        /* 短时卸载后恢复静态支撑，使位置环下蹲且不依赖关节止挡。 */
        const float blend = OwnWbrSmoothStep(state->jump_time / 0.35f);
        const float unload = state->jump_time < 0.20f ?
            OwnWbrSmoothStep(state->jump_time / 0.20f) :
            1.0f - OwnWbrSmoothStep((state->jump_time - 0.20f) / 0.20f);
        *length = state->length - 0.010f * blend;
        *support = state->support - 40.0f * unload;
        break;
    }
    case 2u: {
        /* 只通过腿长 PID 与 VMC 蹬伸，不向模型施加额外外力。 */
        const float length_blend = OwnWbrSmoothStep(state->jump_time / 0.18f);
        const float force_blend = OwnWbrSmoothStep(state->jump_time / 0.04f);
        *length = state->length - 0.010f + 0.110f * length_blend;
        *support = state->support + 180.0f * force_blend;
        break;
    }
    case 3u: {
        /* 先用短脉冲带起腿轮，再按编码器位置卸载，避免持续拉低机身。 */
        if (state->jump_time < 0.015f) {
            *length = state->length - 0.020f;
            *support = state->support - 140.0f;
        } else if (state->jump_time < 0.105f) {
            const float blend = OwnWbrSmoothStep((state->jump_time - 0.015f) / 0.050f);
            *length = state->length + 0.105f - 0.093f * blend;
            *support = -120.0f * blend;
            if (state->jump_time < 0.018f) {
                for (int side = 0; side < 2; side++) {
                    state->length_pid[side].last_err = *length - state->leg[side].length;
                }
            }
        } else {
            const float blend = OwnWbrSmoothStep((state->jump_time - 0.105f) / 0.025f);
            *length = state->length + 0.012f + 0.058f * blend;
            *support = -120.0f + 120.0f * blend;
        }
        break;
    }
    case 4u: {
        /* 逐步恢复正常腿长和支撑，减少落地冲击。 */
        const float blend = OwnWbrSmoothStep(state->jump_time / 0.34f);
        *length = state->length + 0.070f - 0.070f * blend;
        *support = state->support * OwnWbrSmoothStep(state->jump_time / 0.18f);
        break;
    }
    case 5u:
        break;
    default:
        state->jump_phase = 0u;
        state->jump_time = 0.0f;
        break;
    }
    if (state->jump_phase != 0u) {
        state->jump_time += dt;
    }
}

/* 输入仅有 IMU 和六个电机的编码器/反馈；不接受机身真值位置、速度。 */
int OwnWbrCommandStep(void *handle, const float *sensor, const float *command, float *out)
{
    OwnWbrState *state = (OwnWbrState *)handle;
    if (out != NULL) {
        memset(out, 0, 40 * sizeof(*out));
    }
    if (state == NULL || sensor == NULL || out == NULL) {
        return 5;
    }
    /* 上台故障保持零输出，只有重新创建控制器才解除。 */
    if (state->step_climb_fault != 0u) {
        return state->step_climb_fault;
    }
    /* 错误返回也必须给出零电机命令，不能让调用方复用上帧输出或 NaN。 */
    if (state->rise_phase != 11) {
        for (int i = 0; i < 28; i++) {
            if (!isfinite(sensor[i])) {
                return 3;
            }
        }
    }
    if (command != NULL) {
        for (int i = 0; i < 3; i++) {
            if (!isfinite(command[i])) {
                return 3;
            }
        }
    }
    const float dt = 0.003f;
    if (state->recovery_release_time > 0.0f && state->rise_phase == 0) {
        state->recovery_release_time = fmaxf(0.0f,
                                             state->recovery_release_time - dt);
    }
    const float requested_v = command != NULL ? command[0] : 0.0f;
    const float requested_yaw_rate = command != NULL ?
        WheelLegCoreClamp(command[1], -3.0f, 3.0f) : 0.0f;
    const float requested_jump = command != NULL && state->stages.stage == 3u ? command[2] : 0.0f;
    const uint8_t rise_busy = state->rise_phase != 0;
    float target_v = rise_busy ? 0.0f : requested_v;
    const float target_yaw_rate = rise_busy ? 0.0f : requested_yaw_rate;
    const uint8_t climb_assist_driving = state->climb_assist_pressed &&
        requested_v > 0.30f && WheelLegCoreAbs(requested_yaw_rate) < 0.05f;
    const uint8_t step_climb_locks_jump = state->step_climb_enabled == 1u ||
        (state->step_climb_enabled == 2u && state->step_climb_phase == 0u &&
         climb_assist_driving) ||
        (state->step_climb_enabled == 2u && state->step_climb_phase != 0u &&
         state->step_climb_phase != 8u);
    const float jump_button = rise_busy || step_climb_locks_jump ?
        0.0f : requested_jump;
    const float w = sensor[0], x = sensor[1], y = sensor[2], z = sensor[3];
    const float roll = atan2f(2.0f * (w*x + y*z), 1.0f - 2.0f * (x*x + y*y));
    const float pitch = asinf(WheelLegCoreClamp(2.0f * (w*y - z*x), -1.0f, 1.0f));
    const float yaw = atan2f(2.0f * (w*z + x*y), 1.0f - 2.0f * (y*y + z*z));
    if (!state->first) {
        state->yaw_ref = yaw;
    }
    const float gyro = -(sensor[5] * cosf(roll) - sensor[6] * sinf(roll));
    float wheel_arg[2];
    float wheel_abs[2];
    if (state->rise_phase == 11) {
        for (int i = 0; i < 28; i++) {
            if (!isfinite(sensor[i])) {
                state->rise_phase = -11;
                state->yaw_ref = 0.0f;
                OwnWbrFillRiseOutput(state, 0.0f, 0.0f, 0.0f, 0.0f, out);
                state->first = 1u;
                return 0;
            }
        }
        const float quaternion_norm = sensor[0]*sensor[0] + sensor[1]*sensor[1] +
            sensor[2]*sensor[2] + sensor[3]*sensor[3];
        if (quaternion_norm < 0.25f || quaternion_norm > 2.25f) {
            state->rise_phase = -11;
            state->yaw_ref = 0.0f;
            OwnWbrFillRiseOutput(state, 0.0f, 0.0f, 0.0f, 0.0f, out);
            state->first = 1u;
            return 0;
        }
    }
    for (int side = 0; side < 2; side++) {
        const int base = 10 + side * 9;
        const float sign = side == 0 ? -1.0f : 1.0f;
        WheelLegCoreLegCalc *leg = &state->leg[side];
        if (!WheelLegCoreCalcKinematics(&s_geometry, leg,
                1.804837140117f - sign * sensor[base + 3],
                -1.486361593218f - sign * sensor[base],
                -pitch, gyro, 0u, dt)) {
            memset(out, 0, 40 * sizeof(*out));
            return 1;
        }
        /* 轮轴安装在 joint1 下连杆；反射并交换支链后对应 phi3。
         * 先恢复轮子的绝对角速度，再转换成 Core 观察器的输入约定。 */
        const float rod_rate = state->first ?
            WheelLegCoreWrapPi(leg->phi3 - state->last_phi3[side]) / dt : 0.0f;
        const float omega_abs = sign * sensor[base + 7] - rod_rate + sensor[5];
        wheel_abs[side] = omega_abs;
        wheel_arg[side] = omega_abs - leg->d_alpha + gyro;
        state->last_phi3[side] = leg->phi3;
    }
    if (state->recovery_requested && state->rise_phase == 15) {
        int8_t fault_phase = 0;
        for (int side = 0; side < 2; side++) {
            const int base = 10 + side * 9;
            if (WheelLegCoreAbs(sensor[base + 7]) >
                OWN_WBR_RECOVERY_WHEEL_SPEED_LIMIT_RADPS) {
                fault_phase = -17;
            }
            if (WheelLegCoreAbs(sensor[base + 1]) >
                    OWN_WBR_RECOVERY_JOINT_SPEED_LIMIT_RADPS ||
                WheelLegCoreAbs(sensor[base + 4]) >
                    OWN_WBR_RECOVERY_JOINT_SPEED_LIMIT_RADPS) {
                fault_phase = -15;
            }
            if (WheelLegCoreAbs(sensor[base] -
                    state->recovery_origin_q[side][0]) >
                    OWN_WBR_RECOVERY_JOINT_TRAVEL_LIMIT_RAD ||
                WheelLegCoreAbs(sensor[base + 3] -
                    state->recovery_origin_q[side][1]) >
                    OWN_WBR_RECOVERY_JOINT_TRAVEL_LIMIT_RAD) {
                fault_phase = -16;
            }
        }
        const float body_rate = sqrtf(sensor[4] * sensor[4] +
                                      sensor[5] * sensor[5] +
                                      sensor[6] * sensor[6]);
        if (body_rate > OWN_WBR_RECOVERY_BODY_RATE_LIMIT_RADPS) {
            fault_phase = -20;
        }
        if (fault_phase < 0) {
            state->rise_phase = fault_phase;
            OwnWbrFillRiseOutput(state, roll, pitch, yaw, 0.0f, out);
            state->first = 1u;
            return 0;
        }
    }
    if (state->rise_phase == 11) {
        /* 最多等待 1.5 秒落稳；等待期间六路输出保持为零，只启动一次恢复。 */
        OwnWbrFillRiseOutput(state, roll, pitch, yaw, 0.0f, out);
        state->rise_phase_time += dt;
        state->rise_elapsed += dt;
        if (!isfinite(roll) || !isfinite(pitch) || !isfinite(gyro)) {
            state->rise_phase = -11;
        } else {
            if (WheelLegCoreAbs(roll) >= 0.52f ||
                WheelLegCoreAbs(pitch) >= 0.52f) {
                state->recovery_wait_needed = 1u;
            }
            const float body_rate = sqrtf(sensor[4] * sensor[4] +
                                          sensor[5] * sensor[5] +
                                          sensor[6] * sensor[6]);
            uint8_t settled = body_rate <= OWN_WBR_RECOVERY_SETTLE_BODY_RADPS;
            for (int side = 0; side < 2; side++) {
                const int base = 10 + side * 9;
                if (WheelLegCoreAbs(sensor[base + 1]) >
                        OWN_WBR_RECOVERY_SETTLE_JOINT_RADPS ||
                    WheelLegCoreAbs(sensor[base + 4]) >
                        OWN_WBR_RECOVERY_SETTLE_JOINT_RADPS ||
                    WheelLegCoreAbs(sensor[base + 7]) >
                        OWN_WBR_RECOVERY_SETTLE_WHEEL_RADPS) {
                    settled = 0u;
                }
            }
            state->rise_stable_time = settled ?
                state->rise_stable_time + dt : 0.0f;
        }
        if (state->rise_phase < 0) {
            OwnWbrFillRiseOutput(state, roll, pitch, yaw, 0.0f, out);
        } else if (state->rise_phase_time >= OWN_WBR_RECOVERY_SETTLE_TIMEOUT_S &&
                   state->rise_stable_time < OWN_WBR_RECOVERY_SETTLE_STABLE_S) {
            state->rise_phase = -18;
            OwnWbrFillRiseOutput(state, roll, pitch, yaw, 0.0f, out);
        } else if ((!state->recovery_wait_needed ||
                    state->rise_phase_time >= OWN_WBR_RECOVERY_SETTLE_MIN_S) &&
                   state->rise_stable_time >= OWN_WBR_RECOVERY_SETTLE_STABLE_S) {
            const float tilt = acosf(WheelLegCoreClamp(
                cosf(roll) * cosf(pitch), -1.0f, 1.0f));
            state->recovery_best_tilt = tilt;
            state->recovery_progress_time = state->rise_elapsed;
            for (int side = 0; side < 2; side++) {
                const int base = 10 + side * 9;
                state->recovery_origin_q[side][0] = sensor[base];
                state->recovery_origin_q[side][1] = sensor[base + 3];
                state->recovery_command_q[side][0] = sensor[base];
                state->recovery_command_q[side][1] = sensor[base + 3];
            }
            if (WheelLegCoreAbs(roll) < 0.52f && WheelLegCoreAbs(pitch) < 0.52f) {
                const uint8_t wheels_below = cosf(state->leg[0].alpha) > 0.35f &&
                    cosf(state->leg[1].alpha) > 0.35f;
                const float mean_length = 0.5f *
                    (state->leg[0].length + state->leg[1].length);
                if (wheels_below && mean_length > 0.16f && mean_length < 0.25f) {
                    OwnWbrResetBalance(state, yaw);
                    state->rise_phase = 15;
                } else if (!OwnWbrPrepareRise(state, sensor)) {
                    state->rise_phase = -12;
                } else {
                    state->rise_phase = 14;
                }
                state->rise_phase_time = 0.0f;
                state->rise_stable_time = 0.0f;
            } else {
                /* 每个髋关节只沿选定方向执行一次有限角度翻身。 */
                state->recovery_path = WheelLegCoreAbs(pitch) > 0.75f ?
                    (pitch >= 0.0f ? 3u : 4u) : (roll >= 0.0f ? 1u : 2u);
                state->recovery_sweep_floor =
                    fmaxf(WheelLegCoreAbs(roll), WheelLegCoreAbs(pitch)) < 1.31f ?
                    3.0f : 1.0f;
                state->rise_phase = 12;
                state->rise_phase_time = 0.0f;
                state->rise_stable_time = 0.0f;
            }
        }
        state->first = 1u;
        return 0;
    }
    if (state->rise_phase >= 12 && state->rise_phase <= 14) {
        state->jump_pressed = requested_jump > 0.5f;
        OwnWbrRecoveryPositionStep(state, sensor, roll, pitch, yaw, gyro, out);
        state->first = 1u;
        for (int i = 0; i < 40; i++) {
            if (!isfinite(out[i])) {
                memset(out, 0, 40 * sizeof(*out));
                return 3;
            }
        }
        return 0;
    }
    if (state->rise_phase == 15) {
        const float mean_length = 0.5f * (state->leg[0].length + state->leg[1].length);
        const float mean_d_length = 0.5f * (WheelLegCoreAbs(state->leg[0].d_length) +
                                             WheelLegCoreAbs(state->leg[1].d_length));
        const uint8_t stable = WheelLegCoreAbs(roll) < 0.0873f &&
            WheelLegCoreAbs(pitch) < 0.0873f && WheelLegCoreAbs(sensor[4]) < 0.35f &&
            WheelLegCoreAbs(gyro) < 0.35f && mean_length > 0.16f &&
            mean_length < 0.25f && mean_d_length < 0.18f;
        state->rise_phase_time += dt;
        state->rise_elapsed += dt;
        state->rise_stable_time = stable ? state->rise_stable_time + dt : 0.0f;
        if (state->rise_stable_time >= 0.50f) {
            state->rise_phase = 0;
            state->rise_phase_time = 0.0f;
            state->rise_stable_time = 0.0f;
            state->recovery_requested = 0u;
            state->recovery_release_time = OWN_WBR_RECOVERY_RELEASE_S;
        } else if (state->rise_phase_time >= 5.0f) {
            state->rise_phase = -14;
            OwnWbrFillRiseOutput(state, roll, pitch, yaw, 0.0f, out);
            state->first = 1u;
            return 0;
        }
    }
    if (state->rise_phase < 0) {
        state->jump_pressed = requested_jump > 0.5f;
        OwnWbrFillRiseOutput(state, roll, pitch, yaw, 0.0f, out);
        state->first = 1u;
        return 0;
    }
    if (state->rise_phase == 1 || state->rise_phase == 2) {
        state->jump_pressed = requested_jump > 0.5f;
        OwnWbrRisePositionStep(state, sensor, roll, pitch, yaw, gyro, out);
        state->first = 1u;
        for (int i = 0; i < 40; i++) {
            if (!isfinite(out[i])) {
                memset(out, 0, 40 * sizeof(*out));
                return 3;
            }
        }
        return 0;
    }
    if (state->rise_phase == 3 || state->rise_phase == 4) {
        const float mean_length = 0.5f * (state->leg[0].length + state->leg[1].length);
        const float mean_d_length = 0.5f * (WheelLegCoreAbs(state->leg[0].d_length) +
                                             WheelLegCoreAbs(state->leg[1].d_length));
        const float pitch_limit = 0.10f;
        const float gyro_limit = state->rise_phase == 3 ? 0.45f : 0.35f;
        const uint8_t stable = WheelLegCoreAbs(pitch) < pitch_limit &&
            WheelLegCoreAbs(gyro) < gyro_limit && mean_length > 0.17f &&
            mean_length < 0.23f && mean_d_length < 0.18f &&
            (state->rise_phase != 3 || state->rise_phase_time >=
             (state->stages.stage == 3u ? OWN_WBR_RISE_HANDOFF_MIN_S : 1.5f));
        state->jump_pressed = requested_jump > 0.5f;
        state->rise_phase_time += dt;
        state->rise_elapsed += dt;
        state->rise_stable_time = stable ? state->rise_stable_time + dt : 0.0f;
        if (state->rise_phase == 3 && state->rise_stable_time >=
            (state->stages.stage == 3u ? OWN_WBR_RISE_HANDOFF_STABLE_S : 0.75f)) {
            state->rise_phase = 4;
            state->rise_phase_time = 0.0f;
            state->rise_stable_time = 0.0f;
        } else if (state->rise_phase == 4 && state->rise_stable_time >=
                   (state->stages.stage == 2u ? 3.0f :
                    (state->stages.stage == 3u ? OWN_WBR_RISE_FINISH_STABLE_S : 1.0f))) {
            state->rise_phase = 0;
            state->rise_phase_time = 0.0f;
            state->rise_stable_time = 0.0f;
        } else if ((state->rise_phase == 3 && state->rise_phase_time >= 4.5f) ||
                   (state->rise_phase == 4 && state->rise_phase_time >=
                    (state->stages.stage == 2u ? 4.5f : 4.0f)) ||
                   state->rise_elapsed >= 12.0f) {
            state->rise_phase = -4;
            OwnWbrFillRiseOutput(state, roll, pitch, yaw, 0.0f, out);
            state->first = 1u;
            return 0;
        }
    }
    if (state->jump_phase != 3u) {
        WheelLegCoreObserverUpdate(&state->observer, &state->leg[0], &state->leg[1],
            wheel_arg[0], wheel_arg[1], state->radius, gyro, 0.18f, dt);
    }
    if (!rise_busy && state->jump_phase == 0u && state->stages.stage >= 2u) {
        const float length_step = 0.04f * dt;
        state->length += WheelLegCoreClamp(state->length_target - state->length,
                                           -length_step, length_step);
    }
    if (state->step_climb_phase == 0u) {
        const uint8_t assist_allowed = state->step_climb_enabled == 2u &&
            state->rise_phase == 0 && state->stages.stage == 3u &&
            state->climb_assist_pressed &&
            state->jump_phase == 0u && requested_jump <= 0.5f &&
            requested_v > 0.30f && WheelLegCoreAbs(requested_yaw_rate) < 0.05f;
        const float assist_target = assist_allowed ? 1.0f : 0.0f;
        const float assist_step = dt / 1.20f;
        state->climb_assist_level += WheelLegCoreClamp(
            assist_target - state->climb_assist_level, -assist_step, assist_step);
    }
    if (state->step_climb_enabled == 1u && state->step_climb_phase == 0u &&
        state->rise_phase == 0 && state->stages.stage == 3u) {
        state->length_target = 0.31f;
        const uint8_t ready = state->length >= 0.3095f &&
            WheelLegCoreAbs(state->leg[0].length - 0.31f) < 0.0105f &&
            WheelLegCoreAbs(state->leg[1].length - 0.31f) < 0.0105f &&
            WheelLegCoreAbs(state->leg[0].d_length) < 0.05f &&
            WheelLegCoreAbs(state->leg[1].d_length) < 0.05f &&
            WheelLegCoreAbs(pitch) < 0.05236f;
        if (!state->step_climb_drive_started) {
            state->step_climb_ready_time = ready ?
                state->step_climb_ready_time + dt : 0.0f;
        }
        if (requested_v > 0.10f) {
            target_v = state->step_climb_drive_started ||
                state->step_climb_ready_time >= 0.15f ? 0.50f : 0.0f;
            if (target_v > 0.10f && !state->step_climb_drive_started) {
                state->step_climb_drive_x = state->observer.x_m;
                state->step_climb_drive_started = 1u;
            }
            if (state->step_climb_drive_started &&
                state->observer.x_m - state->step_climb_drive_x > 0.55f &&
                pitch > 0.0174533f) {
                state->step_climb_phase = 1u;
                state->step_climb_elapsed = 0.0f;
                OwnWbrStepClimbCapture(state, sensor, pitch);
                target_v = 0.0f;
                state->roll_adapt_active = 0u;
                state->roll_adapt_time = 0.0f;
                state->roll_length_bias = 0.0f;
            }
        } else {
            target_v = requested_v;
        }
    } else if (state->step_climb_enabled == 2u &&
               state->step_climb_phase == 0u && state->rise_phase == 0 &&
               state->stages.stage == 3u) {
        /* 普通驾驶不预先抬高车身。IMU 纵向冲击结合轮速观察器识别正向撞台。 */
        const uint8_t posture_ready = state->jump_phase == 0u &&
            WheelLegCoreAbs(pitch) < 0.14f && WheelLegCoreAbs(gyro) < 1.0f;
        /* 松开行进键后仍可能带速撞台；只沿用本次正向行驶状态，不把静止漂移当成触台。 */
        const uint8_t forward_motion = requested_v > 0.30f ||
            (WheelLegCoreAbs(requested_v) <= 0.02f &&
             state->drive_mode == 1u && state->drive_engaged &&
             state->last_drive_command > 0.30f && state->observer.v_mps > 0.45f);
        state->step_climb_armed_time = posture_ready && forward_motion ?
            state->step_climb_armed_time + dt : 0.0f;
        /* 小台阶会向上托轮；用去重力后的冲击方向排除它，阈值按本参考轮径验证。 */
        const float impact_up = sensor[9] - 9.80665f*cosf(roll)*cosf(pitch);
        const uint8_t impact = state->step_climb_armed_time > 0.30f &&
            forward_motion &&
            state->observer.v_mps > 0.45f && sensor[7] < -12.0f &&
            impact_up < -0.32f*sensor[7];
        state->step_climb_ready_time = impact ?
            state->step_climb_ready_time + dt : 0.0f;
        if (state->step_climb_ready_time >= 0.003f) {
            /* 只按编码器实测腿长锁存高姿态分支，不读取台阶高度或位置。 */
            state->step_climb_high = state->leg[0].length > 0.285f &&
                state->leg[1].length > 0.285f;
            state->climb_assist_latched = state->climb_assist_pressed &&
                state->climb_assist_level > 0.05f &&
                WheelLegCoreAbs(requested_yaw_rate) < 0.05f && requested_jump <= 0.5f;
            state->step_climb_total_time = 0.0f;
            memset(state->step_climb_peak_time, 0, sizeof(state->step_climb_peak_time));
            state->step_climb_balance_handoff = 0u;
            state->step_climb_balance_time = 0.0f;
            state->step_climb_p4_brake_latched = 0u;
            state->step_climb_p4_brake_time = 0.0f;
            state->step_climb_phase = 1u;
            state->step_climb_elapsed = 0.0f;
            state->x_ref = state->observer.x_m;
            state->v_ref = 0.0f;
            state->drive_engaged = 0u;
            state->reverse_brake = 0u;
            state->last_drive_command = 0.0f;
            OwnWbrStepClimbCapture(state, sensor, pitch);
            target_v = 0.0f;
            state->roll_adapt_active = 0u;
            state->roll_adapt_time = 0.0f;
            state->roll_length_bias = 0.0f;
        }
    } else if (state->step_climb_enabled == 2u &&
               state->step_climb_phase == 8u) {
        const uint8_t stable = state->jump_phase == 0u &&
            WheelLegCoreAbs(requested_v) <= 0.02f &&
            WheelLegCoreAbs(state->observer.v_mps) < 0.05f &&
            WheelLegCoreAbs(pitch) < 0.14f && WheelLegCoreAbs(gyro) < 0.35f;
        state->step_climb_armed_time = stable ?
            state->step_climb_armed_time + dt : 0.0f;
        if (state->step_climb_armed_time >= 0.50f) {
            state->step_climb_phase = 0u;
            state->step_climb_armed_time = 0.0f;
            state->step_climb_ready_time = 0.0f;
        }
    } else if (state->step_climb_enabled &&
               state->step_climb_phase >= 1u &&
               state->step_climb_phase <= 7u) {
        target_v = 0.0f;
        if (state->step_climb_enabled == 2u && state->step_climb_phase == 7u) {
            if (WheelLegCoreAbs(requested_v) <= 0.02f) {
                /* 松键后继续保持上台时的位置参考；真实行驶或起跳时再交回普通控制。 */
                state->step_climb_phase = 8u;
                state->step_climb_armed_time = 0.0f;
            }
        }
        if (state->step_climb_enabled == 1u && state->step_climb_phase == 7u &&
            WheelLegCoreAbs(requested_v) <= 0.02f) {
            /* 上台后先驻车；松开纵向键，下一次按键才恢复普通驾驶。 */
            state->step_climb_phase = 8u;
        }
    }
    if (state->step_climb_enabled == 2u && state->step_climb_hold &&
        state->step_climb_phase != 7u &&
        (WheelLegCoreAbs(requested_v) > 0.02f || requested_jump > 0.5f)) {
        state->step_climb_hold = 0u;
        if (state->step_climb_phase == 8u &&
            WheelLegCoreAbs(requested_v) > 0.02f) {
            state->step_climb_phase = 0u;
            state->step_climb_armed_time = 0.0f;
            state->step_climb_ready_time = 0.0f;
        }
        state->x_ref = state->observer.x_m;
        state->v_ref = 0.0f;
        state->drive_engaged = 0u;
        state->reverse_brake = 0u;
        state->last_drive_command = 0.0f;
    }
    if (state->step_climb_enabled == 2u && state->step_climb_hold &&
        !state->step_climb_park_shifted) {
        state->step_climb_elapsed += dt;
        if (state->step_climb_elapsed >= 0.40f) {
            /* 保持回站时的驻车参考，短台面不追加固定前送距离。 */
            state->step_climb_park_shifted = 1u;
        }
    }
    const uint8_t step_climb_action = state->step_climb_phase >= 1u &&
        state->step_climb_phase <= 7u;
    const uint8_t jump_active = rise_busy || state->jump_phase != 0u ||
        step_climb_action ||
        (jump_button > 0.5f && !state->jump_pressed);
    float jump_target_v = state->jump_profile == 3u && state->jump_phase != 0u ?
        state->jump_drive_speed : target_v;
    if (state->step_jump && state->jump_profile == 3u && state->jump_phase >= 4u) {
        /* 台顶落地后平滑停车；位置参考由编码器观察器重锚定。 */
        jump_target_v = 0.0f;
    }
    if (state->jump_profile == 2u && state->jump_phase != 0u) {
        /* 小跳前进沿用用户请求；后退限到已验证的 1 m/s，仍保持带速起跳。 */
        jump_target_v = fmaxf(target_v, -1.0f);
    }
    if (!state->step_jump && state->jump_profile == 3u &&
        state->jump_phase == 4u && WheelLegCoreAbs(state->jump_drive_speed) > 0.05f) {
        /* 普通增强跳保留原落地参考。 */
        jump_target_v = state->jump_drive_speed - 0.12f;
    }
    if (state->step_jump_hold && state->jump_phase == 0u) {
        if (WheelLegCoreAbs(target_v) <= 0.02f) {
            /* 松开纵向键后解除越台驻车；下一次按键恢复普通手动行驶。 */
            state->step_jump_hold = 0u;
        } else {
            jump_target_v = 0.0f;
        }
    }
    if (state->jump_profile == 3u && state->jump_phase == 1u) {
        if (state->jump_time <= 0.0f) {
            const float speed_prepare_step = 0.50f * dt;
            jump_target_v = state->v_ref + WheelLegCoreClamp(
                state->jump_drive_speed - state->v_ref, -speed_prepare_step, speed_prepare_step);
        }
        if (WheelLegCoreAbs(state->jump_drive_speed) > 0.01f) {
            const float direction = state->jump_drive_speed > 0.0f ? 1.0f : -1.0f;
            const float approach_error = direction *
                (state->observer.v_mps - state->jump_drive_speed);
            if (approach_error < 0.08f) {
                const float recovery = WheelLegCoreClamp(-2.0f * approach_error,
                                                         0.0f, 0.20f);
                jump_target_v = state->jump_drive_speed + direction * recovery;
            }
        }
    }
    if (state->jump_profile == 2u && state->jump_phase == 1u &&
        state->jump_time <= 0.0f && state->jump_reverse_prepare) {
        const float speed_prepare_step = 0.50f * dt;
        jump_target_v = state->v_ref + WheelLegCoreClamp(
            -1.0f - state->v_ref, -speed_prepare_step, speed_prepare_step);
    }
    const uint8_t step_jump_parking = state->step_jump_hold &&
        (state->jump_phase >= 4u || state->jump_phase == 0u);
    const uint8_t step_climb_parking = state->step_climb_enabled == 2u &&
        state->step_climb_hold;
    OwnWbrDriveStep(state, jump_target_v,
                    rise_busy || step_jump_parking || step_climb_parking, dt);
    if (state->step_climb_enabled == 2u && state->step_climb_phase == 5u) {
        /* 渐交接期间不提前锁住驻车位置，只要求常规平衡消掉当前速度。 */
        state->x_ref = state->observer.x_m;
        state->v_ref = 0.0f;
    }
    if (jump_active) {
        state->yaw_rate = 0.0f;
    } else {
        state->yaw_rate += WheelLegCoreClamp(target_yaw_rate - state->yaw_rate,
                                             -1.5f*dt, 1.5f*dt);
    }
    state->yaw_ref = WheelLegCoreWrapPi(state->yaw_ref + state->yaw_rate * dt);
    const float x_err = state->observer.x_m - state->x_ref;
    const float v_err = state->observer.v_mps - state->v_ref;
    const float turn = WheelLegCoreTurnTorque(state->yaw_ref, yaw,
        sensor[6] - state->yaw_rate, 3.0f, 0.6f, 1.0f);
    /* 转弯前馈只用轮速观察器和 IMU。先向弯内倾，再由横滚环抑制偏差，
     * 避免高速转弯等到一侧轮开始卸载才补偿。 */
    const float turn_yaw_rate = 0.75f*state->yaw_rate + 0.25f*sensor[6];
    const float turn_roll_target = !rise_busy && !step_climb_action &&
        state->jump_phase == 0u && state->stages.stage == 3u ?
        WheelLegCoreClamp(-atanf(state->observer.v_mps*turn_yaw_rate/9.80665f),
                          -0.32f, 0.32f) : 0.0f;
    const float roll_f = WheelLegCoreRollForce(turn_roll_target, roll,
        sensor[4], 180.0f, 15.0f, 35.0f);
    float length_command;
    float support_command;
    if (rise_busy) {
        length_command = state->length;
        support_command = state->support;
    } else {
        OwnWbrJumpCommand(state, jump_button, target_v, roll, pitch,
                          sensor[4], gyro, sensor[6],
                          &length_command, &support_command);
    }
    length_command = WheelLegCoreClamp(length_command, 0.18f, 0.320f);
    /* 用 IMU 和腿编码器估计两轮支撑高差，进出横坡都及时找平。 */
    if (!rise_busy && !step_climb_action && state->jump_phase == 0u &&
        state->stages.stage == 3u) {
        const float left_drop = state->leg[0].length * cosf(state->leg[0].alpha);
        const float right_drop = state->leg[1].length * cosf(state->leg[1].alpha);
        const float mean_vertical = fmaxf(0.80f, 0.5f *
            (cosf(state->leg[0].alpha) + cosf(state->leg[1].alpha)));
        /* 0.4491 m 是本参考模型左右轮中心的横向间距。 */
        const float support_delta = right_drop - left_drop + 0.4491f * sinf(roll);
        const float bias_target = WheelLegCoreClamp(
            0.5f * 0.4491f * sinf(turn_roll_target) / mean_vertical -
            0.5f * support_delta / mean_vertical -
            0.050f * (roll - turn_roll_target) - 0.018f * sensor[4],
            -0.070f, 0.070f);
        if (fabsf(turn_roll_target) > 0.0131f || fabsf(roll) > 0.0131f ||
            fabsf(sensor[4]) > 0.08f ||
            fabsf(state->roll_length_bias) > 0.0005f) {
            state->roll_adapt_active = 1u;
        }
        if (state->roll_adapt_active) {
            const float bias_step = 0.16f * dt;
            state->roll_length_bias += WheelLegCoreClamp(
                bias_target - state->roll_length_bias, -bias_step, bias_step);
            if (fabsf(state->roll_length_bias) < 0.0005f &&
                fabsf(bias_target) < 0.0005f && fabsf(roll) < 0.0131f &&
                fabsf(sensor[4]) < 0.08f) {
                state->roll_length_bias = 0.0f;
                state->roll_adapt_active = 0u;
            }
        }
    }
    /* 这组增益按两腿共同运动推导；单腿差异交给同步 PID，避免抵消转向。 */
    const float theta = .5f * (state->leg[0].theta + state->leg[1].theta) - state->theta0;
    const float d_theta = .5f * (state->leg[0].d_theta + state->leg[1].d_theta);
    const float split_error = state->leg[0].theta - state->leg[1].theta;
    if (!state->first) {
        state->split_pid.last_err = split_error;
    }
    const float split = state->stages.stage == 3u ?
        WheelLegCorePidCalc(&state->split_pid, -split_error, 0.0f) : 0.0f;
    const uint8_t jump_split_window = state->jump_profile >= 2u &&
        state->jump_phase != 2u && state->jump_phase != 3u;
    const float corrected_split = jump_split_window ?
        WheelLegCoreClamp(30.0f * split_error +
            2.0f * (state->leg[0].d_theta - state->leg[1].d_theta), -12.0f, 12.0f) : 0.0f;
    const float landing_blend = state->jump_phase == 4u ?
        OwnWbrSmoothStep(state->jump_time / 0.18f) : 1.0f;
    const uint8_t short_fold_parking = state->step_climb_high &&
        state->step_climb_short_fold && state->step_climb_hold &&
        state->step_climb_phase >= 7u;
    const float lqr_x_scale = short_fold_parking ? 4.0f : 1.0f;
    const float lqr_x_err = state->jump_phase == 3u ? 0.0f :
        lqr_x_scale * x_err * landing_blend;
    const float lqr_v_err = state->jump_phase == 3u ? 0.0f : v_err * landing_blend;
    const uint8_t corrected_landing_peak = state->jump_profile >= 2u &&
        state->jump_phase == 4u && state->jump_time <
        ((state->step_jump && state->jump_profile == 3u &&
          WheelLegCoreAbs(state->jump_drive_speed) > 0.50f) ?
         0.35f : 0.18f);
    float joint_limit = (state->rise_phase == 3 || state->jump_phase == 2u ||
                         state->jump_phase == 3u || corrected_landing_peak) ?
        state->joint_peak : state->joint_continuous;
    /* 克服气弹簧与静摩擦的短时收腿余量；累计预算不能因重新按键而清零。 */
    const uint8_t retract_peak = state->stages.stage == 3u && state->jump_profile >= 2u &&
        !rise_busy && !step_climb_action &&
        state->jump_phase != 2u && state->jump_phase != 3u &&
        state->length_peak_time < 0.75f && joint_limit < fminf(state->joint_peak, 14.0f) &&
        0.5f * (state->leg[0].length + state->leg[1].length) > length_command + 0.012f;
    if (retract_peak) {
        joint_limit = fmaxf(joint_limit, fminf(state->joint_peak, 14.0f));
    }
    if (state->rise_phase == 3) {
        const float fade_time = state->stages.stage == 3u ?
            OWN_WBR_RISE_PEAK_FADE_S : 1.5f;
        const float blend = OwnWbrSmoothStep(state->rise_phase_time / fade_time);
        joint_limit += (state->joint_continuous - joint_limit) * blend;
    }
    float position_torque[4] = {0};
    if (state->stages.stage < 2u && !OwnWbrStageStep(&s_geometry, sensor, state->leg,
            dt, &state->stages, position_torque)) {
        memset(out, 0, 40 * sizeof(*out));
        return 4;
    }
    for (int side = 0; side < 2; side++) {
        WheelLegCoreLegCalc *leg = &state->leg[side];
        const int base = 10 + side * 9;
        const float sign = side == 0 ? -1.0f : 1.0f;
        float wheel = state->wheel0 + WheelLegCoreLqrWheelOutput(state->k,
            theta, d_theta, lqr_x_err, lqr_v_err, -pitch, gyro);
        wheel += side == 0 ? -turn : turn;
        if (state->recovery_requested && state->rise_phase == 15) {
            /* 接管阶段保留常规平衡，同时消掉恢复动作留下的轮端转速。 */
            wheel -= OWN_WBR_RECOVERY_WHEEL_DAMPING * sign * sensor[base + 7];
        }
        if (state->jump_profile == 2u && state->jump_phase == 3u) {
            /* 离地后保持轮缘速度接近车速，减小重新触地时的纵向冲击。 */
            const float flight_wheel_target = state->v_ref / state->radius;
            wheel = 0.30f * (flight_wheel_target - wheel_abs[side]);
        }
        leg->tp = state->stages.stage == 3u ? state->tp0 + WheelLegCoreLqrHipOutput(state->k,
            theta, d_theta, lqr_x_err, lqr_v_err, -pitch, gyro,
            side == 0 ? split + corrected_split : -split - corrected_split) : 0.0f;
        if (state->jump_profile == 3u && state->jump_phase == 3u) {
            leg->tp += 5.0f;
        }
        if (!state->first) {
            /* 释放前同步微分历史，避免第一帧长度误差产生冲击。 */
            state->length_pid[side].last_err = length_command - leg->length;
        }
        const float saved_length_kd = state->length_pid[side].kd;
        const float saved_length_kp = state->length_pid[side].kp;
        if (state->jump_profile >= 2u && state->jump_phase == 1u) {
            state->length_pid[side].kp *= 3.0f;
        }
        const float slope_bias = !rise_busy && !step_climb_action &&
            state->jump_phase == 0u ?
            state->roll_length_bias : 0.0f;
        /* 短腿到下限后抬高平均腿长，给低侧留出伸腿行程。 */
        const float slope_mean_length = fmaxf(length_command, 0.182f + fabsf(slope_bias));
        const float side_length = state->jump_phase == 0u && !rise_busy &&
            state->roll_adapt_active ?
            WheelLegCoreClamp(slope_mean_length +
                (side == 0 ? slope_bias : -slope_bias), 0.182f, 0.320f) : length_command;
        if (state->jump_profile >= 2u && state->jump_phase == 3u &&
            state->jump_time >= (state->jump_profile == 3u ? 0.10f : 0.060f)) {
            state->length_pid[side].kd *= state->jump_profile == 3u ? 0.30f : 0.10f;
        }
        leg->f0 = support_command + WheelLegCorePidCalc(&state->length_pid[side],
            leg->length, side_length) + (side == 0 ? roll_f : -roll_f);
        state->length_pid[side].kd = saved_length_kd;
        state->length_pid[side].kp = saved_length_kp;
        const float force_max = state->jump_profile == 3u && corrected_landing_peak ? 240.0f :
            state->jump_profile == 1u &&
            (state->jump_phase == 2u || state->jump_phase == 3u) ? 550.0f :
            (state->jump_profile == 3u &&
             (state->jump_phase == 2u || state->jump_phase == 3u) ? 340.0f :
             (state->jump_profile == 2u &&
              (state->jump_phase == 2u || state->jump_phase == 3u) ? 240.0f :
            ((state->jump_phase == 2u && state->joint_peak > state->joint_continuous) ?
             300.0f : 180.0f)));
        const float force_min = state->jump_profile == 1u && state->jump_phase == 3u ?
            -550.0f : (state->jump_profile == 3u && state->jump_phase == 3u ?
            -340.0f : (state->jump_profile == 2u && state->jump_phase == 3u ?
            -240.0f : -150.0f));
        if (state->jump_profile >= 2u && (state->jump_phase == 3u ||
            (state->jump_profile == 3u && state->jump_phase == 4u))) {
            /* 用编码器回缩速度提前缓冲，避免在触地识别前压到最短机械限位。 */
            const float predict_length = leg->length + fminf(leg->d_length, 0.0f) * 0.040f;
            if (predict_length < 0.195f) {
                const float margin_force = state->jump_profile == 3u && state->jump_phase == 4u ?
                    200.0f + 4000.0f * (0.195f - predict_length) :
                    80.0f + 2000.0f * (0.195f - predict_length);
                leg->f0 = fmaxf(leg->f0, margin_force);
            }
        }
        leg->f0 = WheelLegCoreClamp(leg->f0, force_min, force_max);
        if (!WheelLegCoreCalcVmc(leg)) {
            memset(out, 0, 40 * sizeof(*out));
            return 2;
        }
        float q1_torque = -sign * leg->joint_torque[1];
        float q4_torque = -sign * leg->joint_torque[0];
        if (state->stages.stage < 2u) {
            /* 电机位置伺服替代腿部 VMC；轮端仍使用同一轮平衡计算。 */
            q1_torque = position_torque[side*2];
            q4_torque = position_torque[side*2+1];
            leg->f0 = 0.0f;
            leg->tp = 0.0f;
        }
        float spring_q1;
        float spring_q4;
        OwnWbrGasSpringTorque(state, side, sensor[base], sensor[base + 3],
                              &spring_q1, &spring_q4);
        out[side*3] = WheelLegCoreClamp(q1_torque - spring_q1,
                                        -joint_limit, joint_limit);
        out[side*3+1] = WheelLegCoreClamp(q4_torque - spring_q4,
                                          -joint_limit, joint_limit);
        out[36+side*2] = spring_q1;
        out[37+side*2] = spring_q4;
        float wheel_limit = 5.2f;
        if (state->recovery_requested && state->rise_phase == 15) {
            wheel_limit = OWN_WBR_RECOVERY_WHEEL_TORQUE_LIMIT_NM;
        } else if (state->recovery_release_time > 0.0f) {
            const float release_blend = 1.0f -
                state->recovery_release_time / OWN_WBR_RECOVERY_RELEASE_S;
            wheel_limit = OWN_WBR_RECOVERY_WHEEL_TORQUE_LIMIT_NM +
                (5.2f - OWN_WBR_RECOVERY_WHEEL_TORQUE_LIMIT_NM) *
                WheelLegCoreClamp(release_blend, 0.0f, 1.0f);
        }
        out[side*3+2] = sign * WheelLegCoreClamp(wheel, -wheel_limit, wheel_limit);
        out[12+side*6] = leg->length;
        out[13+side*6] = leg->d_length;
        out[14+side*6] = leg->theta;
        out[15+side*6] = leg->d_theta;
        out[16+side*6] = leg->tp;
        out[17+side*6] = leg->f0;
    }
    out[6] = roll; out[7] = pitch; out[8] = yaw;
    out[9] = state->observer.x_m; out[10] = state->observer.v_mps;
    out[24] = 1.0f; out[25] = state->x_ref; out[26] = state->v_ref;
    out[27] = state->yaw_ref; out[28] = state->yaw_rate;
    /* 这是仿真动作预算；低负载连续恢复两秒才补回，不代表电机厂家热模型。 */
    if (retract_peak && (fabsf(out[0]) > state->joint_continuous + 0.01f ||
        fabsf(out[1]) > state->joint_continuous + 0.01f ||
        fabsf(out[3]) > state->joint_continuous + 0.01f ||
        fabsf(out[4]) > state->joint_continuous + 0.01f)) {
        state->length_peak_time += dt;
        state->length_peak_rest = 0.0f;
    } else if (!rise_busy && !step_climb_action && state->jump_phase == 0u &&
        fabsf(state->leg[0].d_length) < 0.04f && fabsf(state->leg[1].d_length) < 0.04f &&
        fmaxf(fmaxf(fabsf(out[0]), fabsf(out[1])), fmaxf(fabsf(out[3]), fabsf(out[4]))) <
            0.9f * state->joint_continuous) {
        state->length_peak_rest += dt;
        if (state->length_peak_rest >= 2.0f) {
            state->length_peak_time = 0.0f;
            state->length_peak_rest = 2.0f;
        }
    } else {
        state->length_peak_rest = 0.0f;
    }
    out[29] = (float)state->jump_phase; out[30] = (float)state->jump_count;
    out[31] = joint_limit;
    out[32] = (float)state->stages.stage;
    out[33] = state->stages.stage == 1u ?
        state->stages.command_length_m : length_command;
    out[34] = (float)state->rise_phase;
    out[35] = state->rise_elapsed;
    float normal_step_climb_out[6];
    memcpy(normal_step_climb_out, out, sizeof(normal_step_climb_out));
    /* 高台动作严重偏离后退出，避免一直满轮矩拖行到总超时。
     * 这是仿真动作边界，不能当作实车人员接近的安全认证。 */
    if (state->step_climb_high && state->step_climb_phase >= 1u &&
        state->step_climb_phase <= 5u &&
        (WheelLegCoreAbs(pitch) > 1.05f || WheelLegCoreAbs(roll) > 0.70f)) {
        state->step_climb_fault = 12u;
        memset(out, 0, 40 * sizeof(*out));
        return 12;
    }
    /* 送轮/站起时轮速异常增大也停止；空转时此估计不能称作真实车速。 */
    if (state->step_climb_high && state->step_climb_phase >= 4u &&
        state->step_climb_phase <= 5u &&
        WheelLegCoreAbs(state->observer.v_mps) > 2.5f) {
        state->step_climb_fault = 13u;
        memset(out, 0, 40 * sizeof(*out));
        return 13;
    }
    const int step_climb_result = OwnWbrStepClimbOverride(
        state, sensor, pitch, yaw, normal_step_climb_out, out);
    if (step_climb_result != 0) {
        state->step_climb_fault = (uint8_t)step_climb_result;
        memset(out, 0, 40 * sizeof(*out));
        return step_climb_result;
    }
    state->first = 1u;
    for (int i = 0; i < 40; i++) {
        if (!isfinite(out[i])) {
            memset(out, 0, 40 * sizeof(*out));
            return 3;
        }
    }
    return 0;
}

int OwnWbrStep(void *handle, const float *sensor, float target_v, float *out)
{
    const float command[3] = {target_v, 0.0f, 0.0f};
    return OwnWbrCommandStep(handle, sensor, command, out);
}
