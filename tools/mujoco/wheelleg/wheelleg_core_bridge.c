/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Tiny native bridge for the MuJoCo wheel-leg runner. It keeps the physics
 * adapter out of firmware tasks while calling the same reusable WheelLegCore.h
 * functions that firmware uses.
 */

#include <string.h>

#include "WheelLegCore.h"

#define ARBATOS_WHEELLEG_BRIDGE_VERSION 1u
#define ARBATOS_WHEELLEG_HOLD_LIMIT_M 0.05f
#define ARBATOS_WHEELLEG_MOTION_EPS 1.0e-4f

typedef struct
{
    fp32 kp;
    fp32 ki;
    fp32 kd;
    fp32 max_out;
    fp32 max_iout;
} arbatos_wheelleg_bridge_pid_config_t;

typedef struct
{
    WheelLegCoreGeometry geometry;
    fp32 wheel_radius_m;
    fp32 lqr_poly[WHEELLEG_CORE_LQR_ROW_COUNT][WHEELLEG_CORE_LQR_COEFF_COUNT];
    fp32 support_bias_n;
    fp32 min_leg_length_m;
    fp32 max_leg_length_m;
    fp32 default_leg_length_m;
    fp32 max_wheel_torque_nm;
    fp32 max_joint_torque_nm;
    fp32 max_support_force_n;
    fp32 observer_lpf;
    fp32 lqr_wheel_torque_scale;
    fp32 lqr_hip_torque_scale;
    fp32 pitch_balance_offset_right_rad;
    fp32 pitch_balance_offset_left_rad;
    arbatos_wheelleg_bridge_pid_config_t leg_length_pid;
    arbatos_wheelleg_bridge_pid_config_t leg_split_pid;
    arbatos_wheelleg_bridge_pid_config_t turn_pid;
    arbatos_wheelleg_bridge_pid_config_t roll_pid;
} arbatos_wheelleg_bridge_config_t;

typedef struct
{
    WheelLegCoreLegCalc leg[WHEELLEG_SIDE_COUNT];
    WheelLegCoreObserver observer;
    WheelLegCorePid leg_pid[WHEELLEG_SIDE_COUNT];
    WheelLegCorePid split_pid;
    fp32 yaw_set;
    uint8_t yaw_inited;
} arbatos_wheelleg_bridge_state_t;

typedef struct
{
    fp32 dt_s;
    fp32 pitch_rad;
    fp32 roll_rad;
    fp32 yaw_rad;
    fp32 gyro_x_radps;
    fp32 gyro_y_radps;
    fp32 gyro_z_radps;
    fp32 right_front_pos_rad;
    fp32 right_back_pos_rad;
    fp32 right_wheel_vel_radps;
    fp32 left_front_pos_rad;
    fp32 left_back_pos_rad;
    fp32 left_wheel_vel_radps;
    fp32 target_v_mps;
    fp32 target_leg_m;
    fp32 target_foot_x_m;
    fp32 target_yaw_rate_radps;
    uint8_t use_vmc;
    uint8_t support_only;
    fp32 jump_force_n;
} arbatos_wheelleg_bridge_input_t;

typedef struct
{
    fp32 wheel_torque_nm[WHEELLEG_SIDE_COUNT];
    fp32 joint_torque_nm[WHEELLEG_SIDE_COUNT][WHEELLEG_CORE_JOINT_COUNT];
    fp32 leg_length_m[WHEELLEG_SIDE_COUNT];
    fp32 leg_theta_rad[WHEELLEG_SIDE_COUNT];
    fp32 observer_x_m;
    fp32 observer_v_mps;
    fp32 actuator_torque_nm[WHEELLEG_CORE_ACTUATOR_COUNT];
    uint8_t ok;
} arbatos_wheelleg_bridge_output_t;

static const fp32 s_default_lqr_poly[WHEELLEG_CORE_LQR_ROW_COUNT][WHEELLEG_CORE_LQR_COEFF_COUNT] = {
    {-243.932f, 105.148f, -19.1838f, -0.199759f},
    {-6.33721f, 2.6174f, -1.08798f, -0.0047227f},
    {-43.8763f, 16.3233f, -2.10154f, -0.127721f},
    {-52.0411f, 19.3719f, -2.60375f, -0.168927f},
    {-805.793f, 328.293f, -48.8092f, 2.92903f},
    {-40.1396f, 16.7832f, -2.61208f, 0.17602f},
    {-962.682f, 417.508f, -67.9889f, 4.84346f},
    {-89.4595f, 37.3246f, -5.80403f, 0.414703f},
    {-618.557f, 251.264f, -37.1404f, 2.18751f},
    {-800.904f, 324.39f, -47.7682f, 2.80481f},
    {3575.2f, -1332.91f, 172.196f, 9.48315f},
    {202.812f, -76.6035f, 10.1013f, 0.345984f},
};

static void BridgePidConfigure(WheelLegCorePid *pid,
                                 const arbatos_wheelleg_bridge_pid_config_t *config)
{
    if (pid == 0 || config == 0)
    {
        return;
    }

    WheelLegCorePidConfigure(pid,
                                config->kp,
                                config->ki,
                                config->kd,
                                config->max_out,
                                config->max_iout);
}

static void BridgeEvalLqr(const arbatos_wheelleg_bridge_config_t *config,
                            fp32 leg_length,
                            fp32 out[WHEELLEG_CORE_LQR_ROW_COUNT])
{
    fp32 length = leg_length;
    uint8_t i;

    if (config == 0 || out == 0)
    {
        return;
    }

    if (config->min_leg_length_m < config->max_leg_length_m)
    {
        length = WheelLegCoreClamp(length, config->min_leg_length_m, config->max_leg_length_m);
    }

    for (i = 0u; i < WHEELLEG_CORE_LQR_ROW_COUNT; i++)
    {
        const fp32 *row = (WheelLegCoreLqrRowIsZero(&config->lqr_poly[i][0]) == 0u)
                              ? &config->lqr_poly[i][0]
                              : &s_default_lqr_poly[i][0];
        out[i] = WheelLegCorePoly4(row, length);
    }
}

uint32_t arbatos_wheelleg_bridge_version(void)
{
    return ARBATOS_WHEELLEG_BRIDGE_VERSION;
}

void arbatos_wheelleg_bridge_config_defaults(arbatos_wheelleg_bridge_config_t *config)
{
    uint8_t i;
    uint8_t j;

    if (config == 0)
    {
        return;
    }

    (void)memset(config, 0, sizeof(*config));
    config->geometry.l1_m = 0.05000f;
    config->geometry.l2_m = 0.11404f;
    config->geometry.l3_m = 0.11404f;
    config->geometry.l4_m = 0.05000f;
    config->geometry.l5_m = 0.06000f;
    config->wheel_radius_m = 0.03275f;
    for (i = 0u; i < WHEELLEG_CORE_LQR_ROW_COUNT; i++)
    {
        for (j = 0u; j < WHEELLEG_CORE_LQR_COEFF_COUNT; j++)
        {
            config->lqr_poly[i][j] = s_default_lqr_poly[i][j];
        }
    }
    config->support_bias_n = 11.0f;
    config->min_leg_length_m = 0.085f;
    config->max_leg_length_m = 0.120f;
    config->default_leg_length_m = 0.100f;
    config->max_wheel_torque_nm = 0.45f;
    config->max_joint_torque_nm = 3.0f;
    config->max_support_force_n = 60.0f;
    config->observer_lpf = 0.18f;
    config->lqr_wheel_torque_scale = 0.18f;
    config->lqr_hip_torque_scale = 0.18f;
    config->leg_length_pid.kp = 280.0f;
    config->leg_length_pid.kd = 1200.0f;
    config->leg_length_pid.max_out = 35.0f;
    config->leg_split_pid.kp = 30.0f;
    config->leg_split_pid.kd = 1.0f;
    config->leg_split_pid.max_out = 2.0f;
    config->turn_pid.kp = 0.6f;
    config->turn_pid.kd = 0.08f;
    config->turn_pid.max_out = 0.20f;
    config->roll_pid.kp = 25.0f;
    config->roll_pid.kd = 3.0f;
    config->roll_pid.max_out = 15.0f;
}

void arbatos_wheelleg_bridge_state_init(arbatos_wheelleg_bridge_state_t *state,
                                        const arbatos_wheelleg_bridge_config_t *config)
{
    if (state == 0 || config == 0)
    {
        return;
    }

    (void)memset(state, 0, sizeof(*state));
    BridgePidConfigure(&state->leg_pid[WHEELLEG_SIDE_LEFT], &config->leg_length_pid);
    BridgePidConfigure(&state->leg_pid[WHEELLEG_SIDE_RIGHT], &config->leg_length_pid);
    BridgePidConfigure(&state->split_pid, &config->leg_split_pid);
}

uint8_t arbatos_wheelleg_bridge_home_pose(const arbatos_wheelleg_bridge_config_t *config,
                                          fp32 *front_out,
                                          fp32 *back_out)
{
    WheelLegCoreFootPoint target;
    fp32 length;

    if (config == 0 || front_out == 0 || back_out == 0)
    {
        return 0u;
    }

    length = (config->min_leg_length_m > 0.02f) ? config->min_leg_length_m : config->default_leg_length_m;
    if (config->max_leg_length_m > 0.02f)
    {
        length = WheelLegCoreClamp(length, 0.02f, config->max_leg_length_m);
    }

    target.x_m = 0.0f;
    target.y_m = length;
    target.length_m = length;
    return WheelLegCoreInversePoint(&config->geometry,
                                       &target,
                                       -WHEELLEG_CORE_PI,
                                       -WHEELLEG_CORE_PI,
                                       front_out,
                                       back_out);
}

uint8_t arbatos_wheelleg_bridge_step(const arbatos_wheelleg_bridge_config_t *config,
                                     arbatos_wheelleg_bridge_state_t *state,
                                     const arbatos_wheelleg_bridge_input_t *input,
                                     arbatos_wheelleg_bridge_output_t *output)
{
    fp32 k_right[WHEELLEG_CORE_LQR_ROW_COUNT];
    fp32 k_left[WHEELLEG_CORE_LQR_ROW_COUNT];
    fp32 right_theta_err;
    fp32 left_theta_err;
    fp32 x_err_r;
    fp32 v_err_r;
    fp32 x_err_l;
    fp32 v_err_l;
    fp32 target_theta;
    fp32 turn_t;
    fp32 roll_f0;
    fp32 split_tp;
    fp32 hip_scale;
    WheelLegCoreOutput CoreOutput;
    uint8_t i;

    if (config == 0 || state == 0 || input == 0 || output == 0 || input->dt_s <= 0.0f)
    {
        return 0u;
    }

    (void)memset(output, 0, sizeof(*output));
    if (WheelLegCoreCalcKinematics(&config->geometry,
                                      &state->leg[WHEELLEG_SIDE_RIGHT],
                                      input->right_front_pos_rad,
                                      input->right_back_pos_rad,
                                      input->pitch_rad,
                                      input->gyro_y_radps,
                                      0u,
                                      input->dt_s) == 0u ||
        WheelLegCoreCalcKinematics(&config->geometry,
                                      &state->leg[WHEELLEG_SIDE_LEFT],
                                      input->left_front_pos_rad,
                                      input->left_back_pos_rad,
                                      input->pitch_rad,
                                      input->gyro_y_radps,
                                      1u,
                                      input->dt_s) == 0u)
    {
        return 0u;
    }

    WheelLegCoreObserverUpdate(&state->observer,
                                  &state->leg[WHEELLEG_SIDE_LEFT],
                                  &state->leg[WHEELLEG_SIDE_RIGHT],
                                  input->left_wheel_vel_radps,
                                  input->right_wheel_vel_radps,
                                  config->wheel_radius_m,
                                  input->gyro_y_radps,
                                  config->observer_lpf,
                                  input->dt_s);

    BridgeEvalLqr(config, state->leg[WHEELLEG_SIDE_RIGHT].length, k_right);
    BridgeEvalLqr(config, state->leg[WHEELLEG_SIDE_LEFT].length, k_left);

    if (state->yaw_inited == 0u)
    {
        state->yaw_set = input->yaw_rad;
        state->yaw_inited = 1u;
    }
    state->yaw_set += input->target_yaw_rate_radps * input->dt_s;
    turn_t = WheelLegCoreTurnTorque(state->yaw_set,
                                       input->yaw_rad,
                                       input->gyro_z_radps,
                                       config->turn_pid.kp,
                                       config->turn_pid.kd,
                                       config->turn_pid.max_out);
    roll_f0 = WheelLegCoreRollForce(0.0f,
                                       input->roll_rad,
                                       input->gyro_x_radps,
                                       config->roll_pid.kp,
                                       config->roll_pid.kd,
                                       config->roll_pid.max_out);

    target_theta = WheelLegCoreTargetThetaFromFootX(input->target_foot_x_m, input->target_leg_m);
    right_theta_err = state->leg[WHEELLEG_SIDE_RIGHT].theta - target_theta;
    left_theta_err = state->leg[WHEELLEG_SIDE_LEFT].theta - target_theta;
    x_err_r = WheelLegCoreLqrXError(state->observer.x_m,
                                        input->target_v_mps,
                                        input->target_yaw_rate_radps,
                                        ARBATOS_WHEELLEG_HOLD_LIMIT_M,
                                        ARBATOS_WHEELLEG_MOTION_EPS);
    v_err_r = state->observer.v_mps - input->target_v_mps;
    x_err_l = -x_err_r;
    v_err_l = input->target_v_mps - state->observer.v_mps;

    output->wheel_torque_nm[WHEELLEG_SIDE_RIGHT] =
        WheelLegCoreLqrWheelOutput(k_right,
                                       right_theta_err,
                                       state->leg[WHEELLEG_SIDE_RIGHT].d_theta,
                                       x_err_r,
                                       v_err_r,
                                       input->pitch_rad - config->pitch_balance_offset_right_rad,
                                       input->gyro_y_radps);
    output->wheel_torque_nm[WHEELLEG_SIDE_LEFT] =
        WheelLegCoreLqrWheelOutput(k_left,
                                       left_theta_err,
                                       state->leg[WHEELLEG_SIDE_LEFT].d_theta,
                                       x_err_l,
                                       v_err_l,
                                       -input->pitch_rad - config->pitch_balance_offset_left_rad,
                                       -input->gyro_y_radps);

    hip_scale = (input->use_vmc != 0u && input->support_only == 0u) ? config->lqr_hip_torque_scale : 0.0f;
    if (input->use_vmc != 0u && input->support_only == 0u)
    {
        split_tp = WheelLegCorePidCalc(&state->split_pid,
                                          state->leg[WHEELLEG_SIDE_RIGHT].theta +
                                              state->leg[WHEELLEG_SIDE_LEFT].theta,
                                          2.0f * target_theta);
        state->leg[WHEELLEG_SIDE_RIGHT].tp =
            hip_scale * WheelLegCoreLqrHipOutput(k_right,
                                                     right_theta_err,
                                                     state->leg[WHEELLEG_SIDE_RIGHT].d_theta,
                                                     x_err_r,
                                                     v_err_r,
                                                     input->pitch_rad - config->pitch_balance_offset_right_rad,
                                                     input->gyro_y_radps,
                                                     split_tp);
        state->leg[WHEELLEG_SIDE_LEFT].tp =
            hip_scale * WheelLegCoreLqrHipOutput(k_left,
                                                     left_theta_err,
                                                     state->leg[WHEELLEG_SIDE_LEFT].d_theta,
                                                     x_err_l,
                                                     v_err_l,
                                                     -input->pitch_rad - config->pitch_balance_offset_left_rad,
                                                     -input->gyro_y_radps,
                                                     split_tp);
    }
    else
    {
        WheelLegCorePidClear(&state->split_pid);
        state->leg[WHEELLEG_SIDE_RIGHT].tp = 0.0f;
        state->leg[WHEELLEG_SIDE_LEFT].tp = 0.0f;
    }

    for (i = 0u; i < WHEELLEG_SIDE_COUNT; i++)
    {
        fp32 cos_theta = cosf(state->leg[i].theta);
        if (WheelLegCoreAbs(cos_theta) < 0.1f)
        {
            cos_theta = (cos_theta >= 0.0f) ? 0.1f : -0.1f;
        }

        if (input->use_vmc != 0u)
        {
            state->leg[i].f0 = config->support_bias_n / cos_theta +
                               WheelLegCorePidCalc(&state->leg_pid[i],
                                                      state->leg[i].length,
                                                      input->target_leg_m);
            state->leg[i].f0 += input->jump_force_n;
            state->leg[i].f0 += (i == WHEELLEG_SIDE_RIGHT) ? roll_f0 : -roll_f0;
            state->leg[i].f0 = WheelLegCoreClamp(state->leg[i].f0,
                                                   -config->max_support_force_n,
                                                   config->max_support_force_n);
            state->leg[i].tp = WheelLegCoreClamp(state->leg[i].tp,
                                                   -config->max_joint_torque_nm,
                                                   config->max_joint_torque_nm);
            if (WheelLegCoreCalcVmc(&state->leg[i]) == 0u)
            {
                return 0u;
            }
            state->leg[i].joint_torque[(uint8_t)WHEELLEG_CORE_JOINT_FRONT] =
                WheelLegCoreClamp(state->leg[i].joint_torque[(uint8_t)WHEELLEG_CORE_JOINT_FRONT],
                                    -config->max_joint_torque_nm,
                                    config->max_joint_torque_nm);
            state->leg[i].joint_torque[(uint8_t)WHEELLEG_CORE_JOINT_BACK] =
                WheelLegCoreClamp(state->leg[i].joint_torque[(uint8_t)WHEELLEG_CORE_JOINT_BACK],
                                    -config->max_joint_torque_nm,
                                    config->max_joint_torque_nm);
        }
        else
        {
            state->leg[i].f0 = 0.0f;
            state->leg[i].joint_torque[(uint8_t)WHEELLEG_CORE_JOINT_FRONT] = 0.0f;
            state->leg[i].joint_torque[(uint8_t)WHEELLEG_CORE_JOINT_BACK] = 0.0f;
        }

        output->wheel_torque_nm[i] =
            WheelLegCoreClamp(output->wheel_torque_nm[i] * config->lqr_wheel_torque_scale - turn_t,
                                -config->max_wheel_torque_nm,
                                config->max_wheel_torque_nm);
        output->joint_torque_nm[i][(uint8_t)WHEELLEG_CORE_JOINT_FRONT] =
            state->leg[i].joint_torque[(uint8_t)WHEELLEG_CORE_JOINT_FRONT];
        output->joint_torque_nm[i][(uint8_t)WHEELLEG_CORE_JOINT_BACK] =
            state->leg[i].joint_torque[(uint8_t)WHEELLEG_CORE_JOINT_BACK];
        output->leg_length_m[i] = state->leg[i].length;
        output->leg_theta_rad[i] = state->leg[i].theta;
    }

    WheelLegCoreOutputClear(&CoreOutput);
    WheelLegCoreSetWheelTorques(&CoreOutput, output->wheel_torque_nm);
    if (input->use_vmc != 0u)
    {
        WheelLegCoreSetVmcJointTorques(&CoreOutput, state->leg, 1, 1, 1, 1);
    }

    for (i = 0u; i < WHEELLEG_CORE_ACTUATOR_COUNT; i++)
    {
        output->actuator_torque_nm[i] = CoreOutput.actuator[i].tau;
    }
    output->observer_x_m = state->observer.x_m;
    output->observer_v_mps = state->observer.v_mps;
    output->ok = 1u;
    return 1u;
}
