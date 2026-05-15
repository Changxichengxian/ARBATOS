/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

/*
 * 阅读地图：
 * - 前段：底盘运行快照、日志缓存、调参接口。
 * - 中段：模式/反馈更新、运动学换算、功率限制前后的电流计算。
 * - 后段：chassis_control_task() 主循环，处理测试模式、离线保护和日志。
 * - 输出：电流命令写入 actuator_cmd，由 CAN 发送任务统一发出。
 */

#include "chassis_control_task.h"
#include "chassis_behaviour.h"

#include "cmsis_os.h"

#include <math.h>

#include "arm_math.h"
#include "pid.h"
#include "manual_input.h"
#include "control_input.h"
#include "CAN_receive.h"
#include "actuator_cmd.h"
#include "chassis_interface.h"
#include "motor_config.h"
#include "watch.h"
#include "detect_task.h"
#include "INS_task.h"
#include "bsp_time.h"
#include "kalman_filter.h"
#include "chassis_power_control.h"
#include "sdlog.h"
#include "rt_profiler.h"
#include "robot_task_profile.h"

#include <string.h>

#define CHASSIS_MOTOR_COUNT 4U

// Chassis follow-yaw stop window (reduces dithering when nearly aligned).
// NOTE: yaw error uses gimbal-relative angle (rad), wz uses chassis rotation speed (rad/s).
#define CHASSIS_FOLLOW_YAW_STOP_ERR_RAD (0.01f)
#define CHASSIS_FOLLOW_YAW_STOP_WZ_RADPS (0.10f)

#ifndef HALF_ECD_RANGE
#define HALF_ECD_RANGE 4096
#endif

#ifndef ECD_RANGE
#define ECD_RANGE 8191
#endif

#ifndef MOTOR_ECD_TO_RAD
#define MOTOR_ECD_TO_RAD (g_config.gimbal.motor_ecd_to_rad)
#endif

#ifndef YAW_TURN
#define YAW_TURN (g_config.gimbal.yaw_turn)
#endif

// Chassis yaw-rate fusion (wheel odom + IMU yaw-rate minus gimbal-yaw rate).
// IMU is installed on the gimbal, so gyro Z includes chassis yaw-rate + gimbal yaw-rate.
// We subtract gimbal yaw motor rate to obtain chassis yaw-rate measurement.
#define CHASSIS_GIMBAL_MOTOR_RPM_TO_RADPS (0.104719755f) // 2*pi/60
#define CHASSIS_WZ_KF_Q_WZ (0.0010f)
#define CHASSIS_WZ_KF_Q_GYRO_BIAS (0.00001f)
#define CHASSIS_WZ_KF_R_WHEEL (0.050f)
#define CHASSIS_WZ_KF_R_IMU (0.200f)
#define CHASSIS_SDLOG_BASE_STREAM_MAX_SAMPLES 16u

typedef struct
{
    const manual_input_state_t *manual_input;
    uint8_t gimbal_state_valid;
    app_gimbal_motor_state_t yaw_motor;
    app_gimbal_motor_state_t pitch_motor;
    const fp32 *ins_angle;
    const fp32 *gyro;
    const motor_measure_t *motor_measure[CHASSIS_MOTOR_COUNT];
    const motor_node_param_t *motor_cfg[CHASSIS_MOTOR_COUNT];
    test_mode_e test_mode;
    uint16_t period_ms;
    uint32_t period_us;
    fp32 period_s;
    fp32 control_hz;
    fp32 motor_rpm_to_vector;
    fp32 motor_speed_to_vx;
    fp32 motor_speed_to_vy;
    fp32 motor_speed_to_wz;
    fp32 motor_distance_to_center;
    fp32 max_wheel_speed;
    uint8_t wheel_type;
    int8_t motor_dir[CHASSIS_MOTOR_COUNT];
} chassis_runtime_snapshot_t;

static const fp32 *chassis_INT_gyro_point = NULL;
static kalman_2x1_t chassis_wz_kf;
static bool_t chassis_wz_kf_inited = 0;

#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }

static fp32 chassis_get_gimbal_yaw_relative_angle(const app_gimbal_motor_state_t *yaw_motor)
{
    if (yaw_motor == NULL || yaw_motor->valid == 0u || yaw_motor->measure.valid == 0u)
    {
        return 0.0f;
    }

    int32_t relative_ecd = (int32_t)yaw_motor->measure.ecd - (int32_t)yaw_motor->offset_ecd;
    if (relative_ecd > HALF_ECD_RANGE)
    {
        relative_ecd -= ECD_RANGE;
    }
    else if (relative_ecd < -HALF_ECD_RANGE)
    {
        relative_ecd += ECD_RANGE;
    }

    fp32 angle = (fp32)relative_ecd * MOTOR_ECD_TO_RAD;
    return YAW_TURN ? -angle : angle;
}

static fp32 chassis_get_gimbal_yaw_relative_rate(const app_gimbal_motor_state_t *yaw_motor)
{
    if (yaw_motor == NULL || yaw_motor->valid == 0u || yaw_motor->measure.valid == 0u)
    {
        return 0.0f;
    }

    fp32 w = (fp32)yaw_motor->measure.speed_rpm * CHASSIS_GIMBAL_MOTOR_RPM_TO_RADPS;
    return YAW_TURN ? -w : w;
}

static bool_t chassis_get_turnaround_frame_yaw(fp32 *out_yaw_relative)
{
    app_gimbal_state_t gimbal_state;
    if (out_yaw_relative == NULL ||
        app_copy_gimbal_state(&gimbal_state) == 0u ||
        gimbal_state.valid == 0u ||
        gimbal_state.turnaround_frame_valid == 0u)
    {
        return 0;
    }

    *out_yaw_relative = gimbal_state.turnaround_frame_yaw_relative;
    return 1;
}

static void chassis_fill_motor_measure_state(app_motor_measure_state_t *out, const motor_measure_t *measure)
{
    if (out == NULL)
    {
        return;
    }

    memset(out, 0, sizeof(*out));
    if (measure == NULL)
    {
        return;
    }

    out->valid = 1u;
    out->ecd = measure->ecd;
    out->speed_rpm = measure->speed_rpm;
    out->given_current = measure->given_current;
    out->temperature = measure->temperate;
    out->last_ecd = measure->last_ecd;
}

static void chassis_wz_kf_reset(fp32 wz0)
{
    kalman_2x1_init(&chassis_wz_kf,
                    wz0,
                    0.0f,
                    1.0f,
                    0.0f,
                    1.0f,
                    CHASSIS_WZ_KF_Q_WZ,
                    0.0f,
                    CHASSIS_WZ_KF_Q_GYRO_BIAS,
                    CHASSIS_WZ_KF_R_WHEEL,
                    1.0f,
                    0.0f);
    chassis_wz_kf_inited = 1;
}

static fp32 chassis_wz_kf_step(fp32 wz_wheel, bool_t imu_valid, fp32 wz_imu)
{
    if (!chassis_wz_kf_inited)
    {
        chassis_wz_kf_reset(wz_wheel);
    }

    // random-walk model: x(k+1) = x(k) + w
    kalman_2x1_predict(&chassis_wz_kf, 1.0f, 0.0f, 0.0f, 1.0f);

    // wheel measurement: z = wz + v
    kalman_2x1_set_H(&chassis_wz_kf, 1.0f, 0.0f);
    kalman_2x1_set_R(&chassis_wz_kf, CHASSIS_WZ_KF_R_WHEEL);
    kalman_2x1_update(&chassis_wz_kf, wz_wheel);

    // IMU measurement (after gimbal-yaw rate removal): z = wz + bias + v
    if (imu_valid)
    {
        kalman_2x1_set_H(&chassis_wz_kf, 1.0f, 1.0f);
        kalman_2x1_set_R(&chassis_wz_kf, CHASSIS_WZ_KF_R_IMU);
        kalman_2x1_update(&chassis_wz_kf, wz_imu);
    }

    return chassis_wz_kf.x0;
}

static void chassis_snapshot_capture(chassis_runtime_snapshot_t *snapshot, chassis_move_t *control);
static bool_t test_mode_allow_chassis(const chassis_runtime_snapshot_t *snapshot)
{
    test_mode_e mode = (snapshot != NULL) ? snapshot->test_mode : (test_mode_e)g_config.test.mode;
    return (mode == TEST_MODE_NONE) || (mode == TEST_MODE_ENTERTAIN) || (mode == TEST_MODE_CHASSIS_ONLY);
}

static void chassis_snapshot_capture(chassis_runtime_snapshot_t *snapshot, chassis_move_t *control)
{
    if (snapshot == NULL)
    {
        return;
    }

    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->manual_input = (control != NULL) ? control->chassis_RC : get_remote_control_point();
    {
        app_gimbal_state_t gimbal_state;
        if (app_copy_gimbal_state(&gimbal_state) != 0u && gimbal_state.valid != 0u)
        {
            snapshot->gimbal_state_valid = 1u;
            snapshot->yaw_motor = gimbal_state.yaw;
            snapshot->pitch_motor = gimbal_state.pitch;
        }
        else if (control != NULL && control->gimbal_state_valid != 0u)
        {
            snapshot->gimbal_state_valid = 1u;
            snapshot->yaw_motor = control->chassis_yaw_motor;
            snapshot->pitch_motor = control->chassis_pitch_motor;
        }
    }
    snapshot->ins_angle = (control != NULL) ? control->chassis_INS_angle : get_INS_angle_point();
    snapshot->gyro = chassis_INT_gyro_point;
    snapshot->test_mode = (test_mode_e)g_config.test.mode;
    snapshot->period_ms = robot_profile_chassis_control_period_ms();
    snapshot->period_us = (uint32_t)snapshot->period_ms * 1000u;
    snapshot->period_s = (fp32)robot_profile_chassis_control_period_s();
    snapshot->control_hz = (snapshot->period_s > 0.0f) ? (1.0f / snapshot->period_s) : CHASSIS_CONTROL_FREQUENCE;
    snapshot->motor_rpm_to_vector = M3508_MOTOR_RPM_TO_VECTOR;
    snapshot->motor_speed_to_vx = MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
    snapshot->motor_speed_to_vy = MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
    snapshot->motor_speed_to_wz = MOTOR_SPEED_TO_CHASSIS_SPEED_WZ;
    snapshot->motor_distance_to_center = MOTOR_DISTANCE_TO_CENTER;
    snapshot->max_wheel_speed = MAX_WHEEL_SPEED;
    snapshot->wheel_type = g_config.chassis.wheel_type;

    for (uint8_t i = 0u; i < CHASSIS_MOTOR_COUNT; i++)
    {
        snapshot->motor_measure[i] = (control != NULL) ? control->motor_chassis[i].chassis_motor_measure : get_chassis_motor_measure_point(i);
        snapshot->motor_cfg[i] = &g_config.motor.chassis[i];
        snapshot->motor_dir[i] = g_config.chassis.motor_dir[i];
    }

    if (control != NULL)
    {
        chassis_control_snapshot_t fast = {0};
        const manual_input_state_t *manual_input = snapshot->manual_input;

        if (snapshot->gimbal_state_valid != 0u)
        {
            control->gimbal_state_valid = 1u;
            control->chassis_yaw_motor = snapshot->yaw_motor;
            control->chassis_pitch_motor = snapshot->pitch_motor;
        }

        fast.manual_online = toe_is_error(DBUS_TOE) ? 0u : 1u;
        fast.test_mode = snapshot->test_mode;
        fast.mode_sw = input_switch(INPUT_SW_CHASSIS_MODE);
        fast.safe_pos = g_config.manual_input.semantics.chassis_safe_pos;
        fast.follow_pos = g_config.manual_input.semantics.chassis_follow_pos;
        fast.spin_pos = g_config.manual_input.semantics.chassis_spin_pos;
        fast.axis_x = input_axis(INPUT_AXIS_CHASSIS_X);
        fast.axis_y = input_axis(INPUT_AXIS_CHASSIS_Y);
        fast.axis_wz = input_axis(INPUT_AXIS_CHASSIS_WZ);
        fast.rc_deadband = CHASSIS_RC_DEADLINE;
        fast.key_mask = (manual_input != NULL) ? manual_input->key.v : 0u;
        fast.front_key_mask = CHASSIS_FRONT_KEY;
        fast.back_key_mask = CHASSIS_BACK_KEY;
        fast.left_key_mask = CHASSIS_LEFT_KEY;
        fast.right_key_mask = CHASSIS_RIGHT_KEY;
        fast.gyro_spin_key_mask = SWING_KEY;
        fast.gyro_spin_var_key_mask = CHASSIS_GYRO_SPIN_VAR_KEY;
        fast.swing_key_mask = CHASSIS_SWING_KEY;
        fast.vx_rc_sen = CHASSIS_VX_RC_SEN;
        fast.vy_rc_sen = CHASSIS_VY_RC_SEN;
        fast.angle_z_rc_sen = CHASSIS_ANGLE_Z_RC_SEN;
        fast.wz_rc_sen = CHASSIS_WZ_RC_SEN;
        fast.open_rc_scale = (fp32)CHASSIS_OPEN_RC_SCALE;
        fast.swing_no_move_angle = SWING_NO_MOVE_ANGLE;
        fast.swing_move_angle = SWING_MOVE_ANGLE;
        fast.swing_amp_rad = CHASSIS_SWING_AMP_RAD;
        fast.swing_half_period_ms = (uint32_t)CHASSIS_SWING_HALF_PERIOD_MS;
        fast.swing_center_hold_min_ms = (uint32_t)CHASSIS_SWING_CENTER_HOLD_MIN_MS;
        fast.swing_center_hold_max_ms = (uint32_t)CHASSIS_SWING_CENTER_HOLD_MAX_MS;

        control->fast = fast;
    }
}

/**
  * @brief          "chassis_move" valiable initialization, include pid initialization, remote control data point initialization, 3508 chassis motors
  *                 data point initialization, gimbal motor data point initialization, and gyro sensor angle point initialization.
  * @param[out]     chassis_move_init: "chassis_move" valiable point
  * @retval         none
  */
static void chassis_init(chassis_move_t *chassis_move_init);

/**
  * @brief          set chassis control mode, mainly call 'chassis_behaviour_mode_set' function
  * @param[out]     chassis_move_mode: "chassis_move" valiable point
  * @retval         none
  */
static void chassis_set_mode(chassis_move_t *chassis_move_mode);

/**
  * @brief          when chassis mode change, some param should be changed, suan as chassis yaw_set should be now chassis yaw
  * @param[out]     chassis_move_transit: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          底盘模式改变，有些参数需要改变，例如底盘控制yaw角度设定值应该变成当前底盘yaw角度
  * @param[out]     chassis_move_transit:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);
/**
  * @param[out]     chassis_move_update: "chassis_move" valiable point
  * @retval         none
  */
static void chassis_feedback_update(chassis_move_t *chassis_move_update, const chassis_runtime_snapshot_t *snapshot);
/**
  * @brief          set chassis control set-point, three movement control value is set by "chassis_behaviour_control_set".
  *
  * @param[out]     chassis_move_update: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief
  * @param[out]     chassis_move_update:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_set_contorl(chassis_move_t *chassis_move_control);
/**
  * @brief          control loop, according to control set-point, calculate motor current,
  *                 motor current will be sentto motor
  * @param[out]     chassis_move_control_loop: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
  * @param[out]     chassis_move_control_loop:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop,
                                 const chassis_runtime_snapshot_t *snapshot,
                                 int16_t pre_power_current[CHASSIS_MOTOR_COUNT]);

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t chassis_high_water;
#endif

//底盘运动数据
chassis_move_t chassis_move;
volatile uint32_t chassis_loop_counter = 0;

typedef struct
{
    uint32_t start_tick_ms;
    uint32_t last_tick_ms;
    uint32_t period_us;
    uint16_t sample_count;
    uint16_t sample_div_counter;
    sdlog_chassis_base_sample_t samples[CHASSIS_SDLOG_BASE_STREAM_MAX_SAMPLES];
} chassis_sdlog_base_stream_state_t;

static chassis_sdlog_base_stream_state_t s_chassis_sdlog_base_stream = {0};

static int16_t chassis_sdlog_clamp_current(fp32 current)
{
    if (current > 32767.0f)
    {
        return 32767;
    }
    if (current < -32768.0f)
    {
        return -32768;
    }
    return (int16_t)current;
}

static void chassis_sdlog_begin_base_stream(uint32_t now_ms, uint32_t period_us)
{
    s_chassis_sdlog_base_stream.start_tick_ms = now_ms;
    s_chassis_sdlog_base_stream.last_tick_ms = now_ms;
    s_chassis_sdlog_base_stream.period_us = period_us;
    s_chassis_sdlog_base_stream.sample_count = 0u;
}

static void chassis_sdlog_flush_base_stream(void)
{
    if (s_chassis_sdlog_base_stream.sample_count == 0u)
    {
        return;
    }

    sdlog_chassis_base_stream_header_t header = {0};
    uint8_t payload[sizeof(header) + sizeof(s_chassis_sdlog_base_stream.samples)];
    const uint16_t payload_len =
        (uint16_t)(sizeof(header) + (uint32_t)s_chassis_sdlog_base_stream.sample_count * sizeof(sdlog_chassis_base_sample_t));

    header.start_tick_ms = s_chassis_sdlog_base_stream.start_tick_ms;
    header.period_us = (s_chassis_sdlog_base_stream.period_us > 0xFFFFu) ? 0xFFFFu : (uint16_t)s_chassis_sdlog_base_stream.period_us;
    header.sample_count = (uint8_t)s_chassis_sdlog_base_stream.sample_count;
    header.version = SDLOG_CHASSIS_BASE_STREAM_VERSION;

    memcpy(payload, &header, sizeof(header));
    memcpy(&payload[sizeof(header)],
           s_chassis_sdlog_base_stream.samples,
           (uint32_t)s_chassis_sdlog_base_stream.sample_count * sizeof(sdlog_chassis_base_sample_t));
    sdlog_write(SDLOG_TAG_CHASSIS_BASE_STREAM, payload, payload_len);
    s_chassis_sdlog_base_stream.sample_count = 0u;
}

static void chassis_sdlog_append_base_sample(const sdlog_chassis_base_sample_t *sample,
                                             uint32_t now_ms,
                                             uint32_t period_us)
{
    if (sample == NULL)
    {
        return;
    }

    if (period_us == 0u)
    {
        period_us = 1000u;
    }

    const uint8_t div = sdlog_high_rate_divider();
    if (div > 1u)
    {
        const uint16_t slot = s_chassis_sdlog_base_stream.sample_div_counter++;
        if ((slot % (uint16_t)div) != 0u)
        {
            return;
        }
        period_us *= (uint32_t)div;
    }
    else
    {
        s_chassis_sdlog_base_stream.sample_div_counter = 0u;
    }

    if (s_chassis_sdlog_base_stream.sample_count == 0u)
    {
        chassis_sdlog_begin_base_stream(now_ms, period_us);
    }
    else
    {
        const uint32_t expected_dt_ms = (s_chassis_sdlog_base_stream.period_us + 999u) / 1000u;
        const uint32_t actual_dt_ms = now_ms - s_chassis_sdlog_base_stream.last_tick_ms;
        if (s_chassis_sdlog_base_stream.period_us != period_us || actual_dt_ms != expected_dt_ms)
        {
            chassis_sdlog_flush_base_stream();
            chassis_sdlog_begin_base_stream(now_ms, period_us);
        }
    }

    s_chassis_sdlog_base_stream.samples[s_chassis_sdlog_base_stream.sample_count++] = *sample;
    s_chassis_sdlog_base_stream.last_tick_ms = now_ms;

    if (s_chassis_sdlog_base_stream.sample_count >= CHASSIS_SDLOG_BASE_STREAM_MAX_SAMPLES)
    {
        chassis_sdlog_flush_base_stream();
    }
}

const chassis_move_t *get_chassis_move_point(void)
{
    return &chassis_move;
}

void chassis_tune_get_follow_pid(pid_param_t *out)
{
    if (out == NULL)
    {
        return;
    }
    out->kp = chassis_move.chassis_angle_pid.Kp;
    out->ki = chassis_move.chassis_angle_pid.Ki;
    out->kd = chassis_move.chassis_angle_pid.Kd;
    out->max_out = chassis_move.chassis_angle_pid.max_out;
    out->max_iout = chassis_move.chassis_angle_pid.max_iout;
}

void chassis_tune_set_follow_pid(const pid_param_t *pid, bool_t clear_state)
{
    if (pid == NULL)
    {
        return;
    }

    chassis_move.chassis_angle_pid.Kp = pid->kp;
    chassis_move.chassis_angle_pid.Ki = pid->ki;
    chassis_move.chassis_angle_pid.Kd = pid->kd;
    chassis_move.chassis_angle_pid.max_out = pid->max_out;
    chassis_move.chassis_angle_pid.max_iout = pid->max_iout;

    if (clear_state)
    {
        PID_clear(&chassis_move.chassis_angle_pid);
    }
}

void chassis_tune_clear_follow_pid(void)
{
    PID_clear(&chassis_move.chassis_angle_pid);
}

void chassis_tune_get_motor_speed_pid(pid_param_t *out)
{
    if (out == NULL)
    {
        return;
    }

    const pid_type_def *pid = &chassis_move.motor_speed_pid[0];
    out->kp = pid->Kp;
    out->ki = pid->Ki;
    out->kd = pid->Kd;
    out->max_out = pid->max_out;
    out->max_iout = pid->max_iout;
}

void chassis_tune_set_motor_speed_pid(const pid_param_t *pid, bool_t clear_state)
{
    if (pid == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    for (uint8_t i = 0; i < CHASSIS_MOTOR_COUNT; i++)
    {
        pid_type_def *dst = &chassis_move.motor_speed_pid[i];
        dst->Kp = pid->kp;
        dst->Ki = pid->ki;
        dst->Kd = pid->kd;
        dst->max_out = pid->max_out;
        dst->max_iout = pid->max_iout;
        if (clear_state)
        {
            PID_clear(dst);
        }
    }
    taskEXIT_CRITICAL();
}

static void chassis_publish_state(const chassis_move_t *control)
{
    if (control == NULL)
    {
        return;
    }

    app_chassis_state_t state = {0};
    state.valid = 1u;
    state.mode = (uint8_t)control->chassis_mode;
    state.last_mode = (uint8_t)control->last_chassis_mode;
    state.vx = control->vx;
    state.vy = control->vy;
    state.wz = control->wz;
    state.vx_set = control->vx_set;
    state.vy_set = control->vy_set;
    state.wz_set = control->wz_set;
    state.chassis_yaw_offset = control->chassis_yaw_offset;
    state.chassis_yaw_offset_set = control->chassis_yaw_offset_set;
    state.chassis_yaw_set = control->chassis_yaw_set;
    state.chassis_yaw = control->chassis_yaw;
    state.chassis_pitch = control->chassis_pitch;
    state.chassis_roll = control->chassis_roll;
    state.angle_pid = control->chassis_angle_pid;

    for (uint8_t i = 0u; i < CHASSIS_MOTOR_COUNT && i < APP_CHASSIS_MOTOR_COUNT; i++)
    {
        chassis_fill_motor_measure_state(&state.motor[i].measure, control->motor_chassis[i].chassis_motor_measure);
        state.motor[i].accel = control->motor_chassis[i].accel;
        state.motor[i].speed = control->motor_chassis[i].speed;
        state.motor[i].speed_set = control->motor_chassis[i].speed_set;
        state.motor[i].give_current = control->motor_chassis[i].give_current;
    }

    (void)app_publish_chassis_state(&state);
}

/**
  * @brief          chassis task, osDelay CHASSIS_CONTROL_TIME_MS (2ms)
  * @param[in]      pvParameters: null
  * @retval         none
  */
void chassis_control_task(void const *pvParameters)
{
    TickType_t last_wake = 0;

    // 函数地图：初始化并等遥控在线；循环里取快照、选模式、算电流、做保护、写日志和延时。
    //wait a time
    vTaskDelay(CHASSIS_TASK_INIT_TIME);
    //chassis init
    chassis_init(&chassis_move);
    while (toe_is_error(DBUS_TOE))
    {
        watch_task_wait(WATCH_TASK_CHASSIS_CONTROL);
        vTaskDelay(pdMS_TO_TICKS(robot_profile_chassis_control_period_ms()));
    }

    last_wake = xTaskGetTickCount();
    while (1)
    {
        const uint64_t loop_start_us = rt_profiler_begin();
        chassis_runtime_snapshot_t snapshot;
        chassis_snapshot_capture(&snapshot, &chassis_move);
        watch_task_beat(WATCH_TASK_CHASSIS_CONTROL);
        //set chassis control mode
        //设置底盘控制模式
        chassis_set_mode(&chassis_move);
        //when mode changes, some data save
        //模式切换数据保存
        chassis_mode_change_control_transit(&chassis_move);
        //chassis data update
        //底盘数据更新
        chassis_feedback_update(&chassis_move, &snapshot);

        // test mode: only none/chassis_only allow normal chassis control
        if (!test_mode_allow_chassis(&snapshot))
        {
            chassis_sdlog_flush_base_stream();
            for (uint8_t i = 0; i < CHASSIS_MOTOR_COUNT; i++)
            {
                PID_clear(&chassis_move.motor_speed_pid[i]);
                chassis_move.motor_chassis[i].speed_set = 0.0f;
                chassis_move.motor_chassis[i].give_current = 0;
            }
            chassis_move.vx_set = 0.0f;
            chassis_move.vy_set = 0.0f;
            chassis_move.wz_set = 0.0f;

            for (uint8_t i = 0; i < CHASSIS_MOTOR_COUNT; i++)
            {
                actuator_cmd_set_chassis_current(i, 0);
            }

            chassis_publish_state(&chassis_move);
            rt_profiler_end(RT_PROFILER_CHASSIS_CONTROL_LOOP, loop_start_us);
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(snapshot.period_ms));

#if INCLUDE_uxTaskGetStackHighWaterMark
            chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
            continue;
        }

        //set chassis control set-point
        chassis_set_contorl(&chassis_move);
        //chassis control pid calculate
        int16_t chassis_pre_power_cmd[CHASSIS_MOTOR_COUNT] = {0};
        chassis_control_loop(&chassis_move, &snapshot, chassis_pre_power_cmd);

        // motor offline guard: if feedback is offline, clear PID and outputs to avoid runaway
        for (uint8_t i = 0; i < CHASSIS_MOTOR_COUNT; i++)
        {
            if (toe_is_error(CHASSIS_MOTOR1_TOE + i))
            {
                PID_clear(&chassis_move.motor_speed_pid[i]);
                chassis_move.motor_chassis[i].speed_set = 0.0f;
                chassis_move.motor_chassis[i].give_current = 0;
            }
        }

        // Update CAN1 chassis motor currents for CAN TX task
        int16_t chassis_current_cmd[CHASSIS_MOTOR_COUNT] = {0};
        if (chassis_move.fast.manual_online != 0u)
        {
            for (uint8_t i = 0; i < CHASSIS_MOTOR_COUNT; i++)
            {
                const int16_t current = chassis_move.motor_chassis[i].give_current;
                chassis_current_cmd[i] = motor_cfg_limit_current_node(snapshot.motor_cfg[i], current);
            }
        }

        for (uint8_t i = 0; i < CHASSIS_MOTOR_COUNT; i++)
        {
            actuator_cmd_set_chassis_current(i, chassis_current_cmd[i]);
        }

        chassis_loop_counter++;
        {
            sdlog_chassis_base_sample_t sample = {0};
            const uint32_t now_ms = bsp_time_get_tick_ms();

            sample.chassis_mode = (uint8_t)chassis_move.chassis_mode;
            sample.last_chassis_mode = (uint8_t)chassis_move.last_chassis_mode;
            for (uint8_t i = 0; i < CHASSIS_MOTOR_COUNT; i++)
            {
                sample.wheel_rpm[i] = (chassis_move.motor_chassis[i].chassis_motor_measure != NULL) ?
                                          chassis_move.motor_chassis[i].chassis_motor_measure->speed_rpm :
                                          0;
                sample.current_request[i] = chassis_pre_power_cmd[i];
                sample.current_output[i] = chassis_current_cmd[i];
            }

            chassis_sdlog_append_base_sample(&sample, now_ms, snapshot.period_us);
        }
        chassis_publish_state(&chassis_move);
        //os delay
        //系统延时
        rt_profiler_end(RT_PROFILER_CHASSIS_CONTROL_LOOP, loop_start_us);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(snapshot.period_ms));

#if INCLUDE_uxTaskGetStackHighWaterMark
        chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

/**
  * @brief          "chassis_move" valiable initialization, include pid initialization, remote control data point initialization, 3508 chassis motors
  *                 data point initialization, gimbal motor data point initialization, and gyro sensor angle point initialization.
  * @param[out]     chassis_move_init: "chassis_move" valiable point
  * @retval         none
  */
static void chassis_init(chassis_move_t *chassis_move_init)
{
    if (chassis_move_init == NULL)
    {
        return;
    }

    //chassis motor speed PID
    const fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};

    //chassis angle PID
    const fp32 chassis_yaw_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};

    const fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    const fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
    uint8_t i;

    chassis_move_init->chassis_mode = CHASSIS_VECTOR_RAW;
    //get remote control point
    chassis_move_init->chassis_RC = get_remote_control_point();
    //get gyro sensor euler angle point
    chassis_move_init->chassis_INS_angle = get_INS_angle_point();
    chassis_INT_gyro_point = get_gyro_data_point();
    // cache gimbal state from app topics when it becomes available
    chassis_move_init->gimbal_state_valid = 0u;
    memset(&chassis_move_init->chassis_yaw_motor, 0, sizeof(chassis_move_init->chassis_yaw_motor));
    memset(&chassis_move_init->chassis_pitch_motor, 0, sizeof(chassis_move_init->chassis_pitch_motor));

    //get chassis motor data point,  initialize motor speed PID
    //获取底盘电机数据指针，初始化PID
    for (i = 0; i < 4; i++)
    {
        chassis_move_init->motor_chassis[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
        PID_init(&chassis_move_init->motor_speed_pid[i], PID_POSITION, motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
    }
    //initialize angle PID
    //初始化角度PID
    PID_init(&chassis_move_init->chassis_angle_pid, PID_POSITION, chassis_yaw_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);

    //first order low-pass filter  replace ramp function
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, robot_profile_chassis_control_period_s(), chassis_x_order_filter);
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, robot_profile_chassis_control_period_s(), chassis_y_order_filter);

    //max and min speed
    chassis_move_init->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
    chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;

    chassis_move_init->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
    chassis_move_init->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;

    //update data
    {
        chassis_runtime_snapshot_t snapshot;
        chassis_snapshot_capture(&snapshot, chassis_move_init);
        chassis_feedback_update(chassis_move_init, &snapshot);
        chassis_publish_state(chassis_move_init);
    }
}

/**
  * @brief          set chassis control mode, mainly call 'chassis_behaviour_mode_set' function
  * @param[out]     chassis_move_mode: "chassis_move" valiable point
  * @retval         none
  */
static void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }
    //in file "chassis_behaviour.c"
    chassis_behaviour_mode_set(chassis_move_mode);
}

/**
  * @brief          when chassis mode change, some param should be changed, suan as chassis yaw_set should be now chassis yaw
  * @param[out]     chassis_move_transit: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          底盘模式改变，有些参数需要改变，例如底盘控制yaw角度设定值应该变成当前底盘yaw角度
  * @param[out]     chassis_move_transit:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit)
{
    if (chassis_move_transit == NULL)
    {
        return;
    }

    if (chassis_move_transit->last_chassis_mode == chassis_move_transit->chassis_mode)
    {
        return;
    }

    // 当从安全/原始模式（例如零力）切回使能模式时，清理 PID 积分和指令，防止 I 项遗留导致复位后猛冲
    if (chassis_move_transit->last_chassis_mode == CHASSIS_VECTOR_RAW &&
        chassis_move_transit->chassis_mode != CHASSIS_VECTOR_RAW)
    {
        for (uint8_t i = 0; i < CHASSIS_MOTOR_COUNT; i++)
        {
            PID_clear(&chassis_move_transit->motor_speed_pid[i]);
            chassis_move_transit->motor_chassis[i].speed_set = 0.0f;
            chassis_move_transit->motor_chassis[i].give_current = 0;
        }
        chassis_move_transit->vx_set = 0.0f;
        chassis_move_transit->vy_set = 0.0f;
        chassis_move_transit->wz_set = 0.0f;
        chassis_move_transit->chassis_cmd_slow_set_vx.out = 0.0f;
        chassis_move_transit->chassis_cmd_slow_set_vy.out = 0.0f;
    }

    //change to follow gimbal angle mode
    //切入跟随云台模式
    if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
        chassis_move_transit->chassis_yaw_offset_set = 0.0f;
    }
    //change to follow chassis yaw angle
    //切入跟随底盘角度模式
    else if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW)
    {
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
    }
    //change to no follow angle
    else if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_NO_FOLLOW_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
    {
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
    }

    chassis_move_transit->last_chassis_mode = chassis_move_transit->chassis_mode;
}

/**
  * @param[out]     chassis_move_update: "chassis_move" valiable point
  * @retval         none
  */
static void chassis_feedback_update(chassis_move_t *chassis_move_update, const chassis_runtime_snapshot_t *snapshot)
{
    if (chassis_move_update == NULL)
    {
        return;
    }

    uint8_t i = 0;
    for (i = 0; i < 4; i++)
    {
        //update motor speed, accel is differential of speed PID
        //更新电机速度，加速度是速度的PID微分
        const motor_measure_t *measure = (snapshot != NULL) ? snapshot->motor_measure[i] : chassis_move_update->motor_chassis[i].chassis_motor_measure;
        const int8_t motor_dir = (snapshot != NULL) ? snapshot->motor_dir[i] : g_config.chassis.motor_dir[i];
        const fp32 rpm_to_vector = (snapshot != NULL) ? snapshot->motor_rpm_to_vector : CHASSIS_MOTOR_RPM_TO_VECTOR_SEN;
        const fp32 control_hz = (snapshot != NULL) ? snapshot->control_hz : CHASSIS_CONTROL_FREQUENCE;
        const int16_t speed_rpm = (measure != NULL) ? measure->speed_rpm : 0;
        chassis_move_update->motor_chassis[i].speed = rpm_to_vector * speed_rpm * (fp32)motor_dir;
        chassis_move_update->motor_chassis[i].accel = chassis_move_update->motor_speed_pid[i].Dbuf[0] * control_hz;
    }

    //calculate chassis speeds (robot frame, right-hand):
    // - vx: forward is positive
    // - vy: left is positive
    // - wz: CCW is positive
    // NOTE: keep consistent with chassis_vector_to_mecanum_wheel_speed() and g_config.chassis.wheel_type.
    const fp32 w0 = chassis_move_update->motor_chassis[0].speed;
    const fp32 w1 = chassis_move_update->motor_chassis[1].speed;
    const fp32 w2 = chassis_move_update->motor_chassis[2].speed;
    const fp32 w3 = chassis_move_update->motor_chassis[3].speed;

    fp32 wz_wheel = 0.0f;
    const uint8_t wheel_type = (snapshot != NULL) ? snapshot->wheel_type : g_config.chassis.wheel_type;
    const fp32 motor_speed_to_vx = (snapshot != NULL) ? snapshot->motor_speed_to_vx : MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
    const fp32 motor_speed_to_vy = (snapshot != NULL) ? snapshot->motor_speed_to_vy : MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
    const fp32 motor_speed_to_wz = (snapshot != NULL) ? snapshot->motor_speed_to_wz : MOTOR_SPEED_TO_CHASSIS_SPEED_WZ;
    const fp32 motor_distance_to_center = (snapshot != NULL) ? snapshot->motor_distance_to_center : MOTOR_DISTANCE_TO_CENTER;
    if (wheel_type == (uint8_t)CHASSIS_WHEEL_TYPE_XDRIVE)
    {
        // X-drive (45° omni) inverse kinematics:
        // w0 = +vx +vy +yaw, w1 = -vx +vy +yaw, w2 = -vx -vy +yaw, w3 = +vx -vy +yaw
        chassis_move_update->vx = (w0 - w1 - w2 + w3) * motor_speed_to_vx;
        chassis_move_update->vy = (w0 + w1 - w2 - w3) * motor_speed_to_vy;
        wz_wheel = (w0 + w1 + w2 + w3) * motor_speed_to_wz / motor_distance_to_center;
    }
    else
    {
        // Mecanum inverse kinematics (DJI legacy mapping):
        // w0 = +vx -vy -yaw, w1 = +vx +vy +yaw, w2 = +vx +vy -yaw, w3 = +vx -vy +yaw
        chassis_move_update->vx = (w0 + w1 + w2 + w3) * motor_speed_to_vx;
        chassis_move_update->vy = (-w0 + w1 + w2 - w3) * motor_speed_to_vy;
        wz_wheel = (-w0 + w1 - w2 + w3) * motor_speed_to_wz / motor_distance_to_center;
    }
    chassis_move_update->wz = wz_wheel;

    // Fuse yaw-rate using IMU (on gimbal) if available.
    // - In chassis-only test mode, keep wheel odom only to avoid relying on IMU/gimbal signals.
    bool_t imu_valid = 0;
    fp32 wz_imu = 0.0f;
    const test_mode_e test_mode = (snapshot != NULL) ? snapshot->test_mode : (test_mode_e)g_config.test.mode;
    const fp32 *gyro = (snapshot != NULL) ? snapshot->gyro : chassis_INT_gyro_point;
    const app_gimbal_motor_state_t *yaw_motor = NULL;
    const app_gimbal_motor_state_t *pitch_motor = NULL;
    if (snapshot != NULL && snapshot->gimbal_state_valid != 0u)
    {
        yaw_motor = &snapshot->yaw_motor;
        pitch_motor = &snapshot->pitch_motor;
    }
    else if (chassis_move_update->gimbal_state_valid != 0u)
    {
        yaw_motor = &chassis_move_update->chassis_yaw_motor;
        pitch_motor = &chassis_move_update->chassis_pitch_motor;
    }
    const fp32 *ins_angle = (snapshot != NULL) ? snapshot->ins_angle : chassis_move_update->chassis_INS_angle;
    if (test_mode != TEST_MODE_CHASSIS_ONLY &&
        !toe_is_error(RM_IMU_TOE) &&
        !toe_is_error(YAW_GIMBAL_MOTOR_TOE) &&
        gyro != NULL &&
        yaw_motor != NULL)
    {
        const fp32 gyro_z = gyro[INS_GYRO_Z_ADDRESS_OFFSET];
        const fp32 gimbal_wz = chassis_get_gimbal_yaw_relative_rate(yaw_motor);
        wz_imu = gyro_z - gimbal_wz;
        imu_valid = 1;
    }
    chassis_move_update->wz = chassis_wz_kf_step(wz_wheel, imu_valid, wz_imu);

    //calculate chassis euler angle, if chassis add a new gyro sensor,please change this code
    if (test_mode == TEST_MODE_CHASSIS_ONLY)
    {
        chassis_move_update->chassis_yaw = 0.0f;
        chassis_move_update->chassis_pitch = 0.0f;
        chassis_move_update->chassis_roll = 0.0f;
    }
    else
    {
        const fp32 yaw_relative = chassis_get_gimbal_yaw_relative_angle(yaw_motor);
        const fp32 yaw = (ins_angle != NULL) ? ins_angle[INS_YAW_ADDRESS_OFFSET] : 0.0f;
        const fp32 pitch = (ins_angle != NULL) ? ins_angle[INS_PITCH_ADDRESS_OFFSET] : 0.0f;
        const fp32 roll = (ins_angle != NULL) ? ins_angle[INS_ROLL_ADDRESS_OFFSET] : 0.0f;
        const fp32 gimbal_pitch = (pitch_motor != NULL && pitch_motor->valid != 0u) ? pitch_motor->angle : 0.0f;
        chassis_move_update->chassis_yaw = rad_format(yaw - yaw_relative);
        chassis_move_update->chassis_pitch = rad_format(pitch - gimbal_pitch);
        chassis_move_update->chassis_roll = roll;
    }
}

static fp32 chassis_follow_yaw_pd_calc(pid_type_def *pid, fp32 yaw_error, fp32 chassis_wz)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    // Reuse pid_type_def fields for telemetry/debug:
    // - P/I acts on yaw_error (rad)
    // - D term is damping on chassis_wz (rad/s), i.e. derivative on measurement
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->error[0] = yaw_error;
    pid->set = yaw_error;
    pid->fdb = 0.0f;

    pid->Pout = pid->Kp * yaw_error;
    pid->Iout += pid->Ki * yaw_error;
    pid->Iout = fp32_constrain(pid->Iout, -pid->max_iout, pid->max_iout);
    pid->Dout = -pid->Kd * chassis_wz;

    pid->out = pid->Pout + pid->Iout + pid->Dout;
    pid->out = fp32_constrain(pid->out, -pid->max_out, pid->max_out);
    return pid->out;
}
/**
  * @brief          accroding to the channel value of remote control, calculate chassis vertical and horizontal speed set-point
  *
  * @param[out]     vx_set: vertical speed set-point
  * @param[out]     vy_set: horizontal speed set-point
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          根据遥控器通道值，计算纵向和横移速度
  *
  * @param[out]     vx_set: 纵向速度指针
  * @param[out]     vy_set: 横向速度指针
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" 变量指针
  * @retval         none
  */
void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL)
    {
        return;
    }

    int16_t vx_channel, vy_channel;
    fp32 vx_set_channel, vy_set_channel;
    const chassis_control_snapshot_t *fast = &chassis_move_rc_to_vector->fast;
    const uint16_t key_mask = fast->key_mask;
    //deadline, because some remote control need be calibrated,  the value of rocker is not zero in middle place,
    rc_deadband_limit(fast->axis_x, vx_channel, fast->rc_deadband);
    rc_deadband_limit(fast->axis_y, vy_channel, fast->rc_deadband);

    vx_set_channel = vx_channel * fast->vx_rc_sen;
    // vy: positive means left, negative means right.
    vy_set_channel = vy_channel * -fast->vy_rc_sen;

    //keyboard set speed set-point
    //键盘控制
    if (key_mask & fast->front_key_mask)
    {
        vx_set_channel = chassis_move_rc_to_vector->vx_max_speed;
    }
    else if (key_mask & fast->back_key_mask)
    {
        vx_set_channel = chassis_move_rc_to_vector->vx_min_speed;
    }

    if (key_mask & fast->left_key_mask)
    {
        vy_set_channel = chassis_move_rc_to_vector->vy_max_speed;
    }
    else if (key_mask & fast->right_key_mask)
    {
        vy_set_channel = chassis_move_rc_to_vector->vy_min_speed;
    }

    //first order low-pass replace ramp function, calculate chassis speed set-point to improve control performance
    //一阶低通滤波代替斜波作为底盘速度输入
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, vy_set_channel);
    //stop command, need not slow change, set zero derectly
    const fp32 vx_stop_threshold = fabsf((fp32)fast->rc_deadband * fast->vx_rc_sen);
    const fp32 vy_stop_threshold = fabsf((fp32)fast->rc_deadband * fast->vy_rc_sen);
    if (vx_set_channel < vx_stop_threshold && vx_set_channel > -vx_stop_threshold)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
    }

    if (vy_set_channel < vy_stop_threshold && vy_set_channel > -vy_stop_threshold)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out = 0.0f;
    }

    *vx_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
    *vy_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out;
}
/**
  * @brief          set chassis control set-point, three movement control value is set by "chassis_behaviour_control_set".
  * @param[out]     chassis_move_update: "chassis_move" valiable point
  * @retval         none
  */
static void chassis_set_contorl(chassis_move_t *chassis_move_control)
{

    if (chassis_move_control == NULL)
    {
        return;
    }

    fp32 vx_set = 0.0f, vy_set = 0.0f, angle_set = 0.0f;
    chassis_behaviour_control_set(&vx_set, &vy_set, &angle_set, chassis_move_control);

    //follow gimbal mode
    if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
        const fp32 yaw_relative_meas = chassis_get_gimbal_yaw_relative_angle(&chassis_move_control->chassis_yaw_motor);
        fp32 yaw_frame = yaw_relative_meas;
        fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
        (void)chassis_get_turnaround_frame_yaw(&yaw_frame);
        //rotate chassis direction, make sure vertial direction follow gimbal
        //旋转控制底盘速度方向，保证前进方向是云台方向，有利于运动平稳
        sin_yaw = arm_sin_f32(-yaw_frame);
        cos_yaw = arm_cos_f32(-yaw_frame);
        chassis_move_control->vx_set = cos_yaw * vx_set + sin_yaw * vy_set;
        chassis_move_control->vy_set = -sin_yaw * vx_set + cos_yaw * vy_set;
        //set control gimbal-chassis angle set-point
        //设置控制云台-底盘角度
        chassis_move_control->chassis_yaw_offset_set = rad_format(angle_set);
        // Follow gimbal yaw:
        // - yaw_relative is the gimbal-chassis relative angle (rad)
        // - yaw_error keeps the same sign as wz_set, so the chassis rotates to reduce yaw_error
        const fp32 yaw_error = rad_format(yaw_relative_meas - chassis_move_control->chassis_yaw_offset_set);
        if (fabsf(yaw_error) < CHASSIS_FOLLOW_YAW_STOP_ERR_RAD && fabsf(chassis_move_control->wz) < CHASSIS_FOLLOW_YAW_STOP_WZ_RADPS)
        {
            PID_clear(&chassis_move_control->chassis_angle_pid);
            chassis_move_control->wz_set = 0.0f;
        }
        else
        {
            chassis_move_control->wz_set = chassis_follow_yaw_pd_calc(&chassis_move_control->chassis_angle_pid, yaw_error, chassis_move_control->wz);
        }
        //speed limit
        //速度限幅
        chassis_move_control->vx_set = fp32_constrain(chassis_move_control->vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(chassis_move_control->vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW)
    {
        //set chassis yaw angle set-point
        chassis_move_control->chassis_yaw_set = rad_format(angle_set);
        const fp32 yaw_error = rad_format(chassis_move_control->chassis_yaw_set - chassis_move_control->chassis_yaw);
        if (fabsf(yaw_error) < CHASSIS_FOLLOW_YAW_STOP_ERR_RAD && fabsf(chassis_move_control->wz) < CHASSIS_FOLLOW_YAW_STOP_WZ_RADPS)
        {
            PID_clear(&chassis_move_control->chassis_angle_pid);
            chassis_move_control->wz_set = 0.0f;
        }
        else
        {
            chassis_move_control->wz_set = chassis_follow_yaw_pd_calc(&chassis_move_control->chassis_angle_pid, yaw_error, chassis_move_control->wz);
        }
        //speed limit
        //速度限幅
        chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
    {
        //"angle_set" is rotation speed set-point
        chassis_move_control->wz_set = angle_set;
        chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_RAW)
    {
        //in raw mode, set-point is sent to CAN bus
        //在原始模式，设置值是发送到CAN总线
        chassis_move_control->vx_set = vx_set;
        chassis_move_control->vy_set = vy_set;
        chassis_move_control->wz_set = angle_set;
        chassis_move_control->chassis_cmd_slow_set_vx.out = 0.0f;
        chassis_move_control->chassis_cmd_slow_set_vy.out = 0.0f;
    }
}

/**
  * @brief          four wheels speed is calculated by three param (mecanum / X-drive).
  * @param[in]      vx_set: vertial speed
  * @param[in]      vy_set: horizontal speed
  * @param[in]      wz_set: rotation speed
  * @param[out]     wheel_speed: four mecanum wheels speed
  * @retval         none
  */
static void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set,
                                                  const fp32 vy_set,
                                                  const fp32 wz_set,
                                                  const chassis_runtime_snapshot_t *snapshot,
                                                  fp32 wheel_speed[4])
{
    // Wheel order: 0=LF(0x201), 1=RF(0x202), 2=LR(0x203), 3=RR(0x204)
    // Robot frame: vx forward +, vy left +, wz CCW +
    const fp32 motor_distance_to_center = (snapshot != NULL) ? snapshot->motor_distance_to_center : MOTOR_DISTANCE_TO_CENTER;
    const uint8_t wheel_type = (snapshot != NULL) ? snapshot->wheel_type : g_config.chassis.wheel_type;
    const fp32 yaw_term = motor_distance_to_center * wz_set;

    if (wheel_type == (uint8_t)CHASSIS_WHEEL_TYPE_XDRIVE)
    {
        // X-drive (45° omni): rotate uses all wheels same direction.
        wheel_speed[0] = vx_set + vy_set + yaw_term;  // LF
        wheel_speed[1] = -vx_set + vy_set + yaw_term; // RF
        wheel_speed[2] = -vx_set - vy_set + yaw_term; // LR
        wheel_speed[3] = vx_set - vy_set + yaw_term;  // RR
    }
    else
    {
        // Mecanum: rotate uses left/right opposite direction.
        wheel_speed[0] = vx_set - vy_set - yaw_term; // LF
        wheel_speed[1] = vx_set + vy_set + yaw_term; // RF
        wheel_speed[2] = vx_set + vy_set - yaw_term; // LR
        wheel_speed[3] = vx_set - vy_set + yaw_term; // RR
    }

    // motor_dir is applied in feedback and current output paths
}
/**
  * @brief          control loop, according to control set-point, calculate motor current,
  *                 motor current will be sentto motor
  * @param[out]     chassis_move_control_loop: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
  * @param[out]     chassis_move_control_loop:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop,
                                 const chassis_runtime_snapshot_t *snapshot,
                                 int16_t pre_power_current[CHASSIS_MOTOR_COUNT])
{
    fp32 max_vector = 0.0f, vector_rate = 0.0f;
    fp32 temp = 0.0f;
    fp32 wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t i = 0;

    chassis_power_control_apply_speed_limit(chassis_move_control_loop);

    //mecanum wheel speed calculation
    //麦轮运动分解
    chassis_vector_to_mecanum_wheel_speed(chassis_move_control_loop->vx_set,
                                          chassis_move_control_loop->vy_set,
                                          chassis_move_control_loop->wz_set,
                                          snapshot,
                                          wheel_speed);

    if (chassis_move_control_loop->chassis_mode == CHASSIS_VECTOR_RAW)
    {

        for (i = 0; i < 4; i++)
        {
            const int8_t motor_dir = (snapshot != NULL) ? snapshot->motor_dir[i] : g_config.chassis.motor_dir[i];
            chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(wheel_speed[i] * (fp32)motor_dir);
            if (pre_power_current != NULL)
            {
                pre_power_current[i] = chassis_move_control_loop->motor_chassis[i].give_current;
            }
        }
        //in raw mode, derectly return
        //raw控制直接返回
        return;
    }

    //calculate the max speed in four wheels, limit the max speed
    //计算轮子控制最大速度，并限制其最大速度
    for (i = 0; i < 4; i++)
    {
        chassis_move_control_loop->motor_chassis[i].speed_set = wheel_speed[i];
        temp = fabs(chassis_move_control_loop->motor_chassis[i].speed_set);
        if (max_vector < temp)
        {
            max_vector = temp;
        }
    }

    const fp32 max_wheel_speed = (snapshot != NULL) ? snapshot->max_wheel_speed : MAX_WHEEL_SPEED;
    if (max_vector > max_wheel_speed)
    {
        vector_rate = max_wheel_speed / max_vector;
        for (i = 0; i < 4; i++)
        {
            chassis_move_control_loop->motor_chassis[i].speed_set *= vector_rate;
        }
    }

    //calculate pid
    //计算pid
    for (i = 0; i < 4; i++)
    {
        PID_calc(&chassis_move_control_loop->motor_speed_pid[i], chassis_move_control_loop->motor_chassis[i].speed, chassis_move_control_loop->motor_chassis[i].speed_set);
        if (pre_power_current != NULL)
        {
            pre_power_current[i] = chassis_sdlog_clamp_current(chassis_move_control_loop->motor_speed_pid[i].out *
                                                               (fp32)((snapshot != NULL) ? snapshot->motor_dir[i] : g_config.chassis.motor_dir[i]));
        }
    }

    //功率控制
    chassis_power_control(chassis_move_control_loop);

    for (i = 0; i < 4; i++)
    {
        const int8_t motor_dir = (snapshot != NULL) ? snapshot->motor_dir[i] : g_config.chassis.motor_dir[i];
        chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(chassis_move_control_loop->motor_speed_pid[i].out * (fp32)motor_dir);
    }
}
