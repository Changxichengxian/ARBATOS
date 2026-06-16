/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

/*
 * 阅读地图：
 * - 前段：云台快照、PID 调参入口、yaw/pitch 电机反馈整理。
 * - 中段：初始化、校准、模式设置，以及角度/速度目标生成。
 * - 后段：双环 PID 算电流、离线保护、日志采样。
 * - 入口：gimbal_control_task() 按任务周期运行，最后把电流写入 LowCmd。
 */

#include "config.h"
#include "robot_task_build_config.h"

#if ROBOT_TASK_BUILD_ANY_GIMBAL

#include "gimbal_control_task.h"
#include "gimbal_core.h"

#include "cmsis_os.h"

#include "arm_math.h"
#include "CAN_receive.h"
#include "LowCmd.h"
#include "MotorInst.h"
#include "motor_config.h"
#include "user_lib.h"
#include "axis_current_conditioner.h"
#include "detect_task.h"
#include "manual_input.h"
#include "control_input.h"
#include "chassis_state.h"
#include "gimbal_state.h"
#include "gimbal_behaviour.h"
#include "INS_task.h"
#include "shoot.h"
#include "pid.h"
#include "host_link_task.h"
#include "watch.h"
#include "SdLog.h"
#include "robot_mode.h"
#include "pitch_cali.h"
#include "bsp_time.h"
#include "RtProf.h"
#include "robot_task_profile.h"
#include "ControlMgr.h"

#include <string.h>

#define GIMBAL_OUTPUT_MOTOR_COUNT 3u
#define DUAL_YAW_GIMBAL_OUTPUT_MOTOR_COUNT 4u

static const char *const gimbal_output_motor_names[GIMBAL_OUTPUT_MOTOR_COUNT] = {
    "motor.trigger",
    "motor.yaw",
    "motor.pitch",
};
static MotorCurrentBind gimbal_output_current_bindings[GIMBAL_OUTPUT_MOTOR_COUNT];

static const char *const dual_yaw_gimbal_output_motor_names[DUAL_YAW_GIMBAL_OUTPUT_MOTOR_COUNT] = {
    "motor.trigger",
    "motor.yaw",
    "motor.yaw_upper",
    "motor.pitch",
};
static MotorCurrentBind dual_yaw_gimbal_output_current_bindings[DUAL_YAW_GIMBAL_OUTPUT_MOTOR_COUNT];

__weak void shoot_init(void)
{
}

__weak int16_t shoot_control_loop(void)
{
    return 0;
}

__weak void shoot_stop_outputs(void)
{
}

typedef struct
{
    manual_input_state_t manual_input_copy;
    const manual_input_state_t *manual_input;
    const fp32 *gyro;
    const fp32 *ins_angle;
    const motor_measure_t *yaw_measure;
    const motor_measure_t *pitch_measure;
    const motor_node_param_t *yaw_motor_cfg;
    const motor_node_param_t *pitch_motor_cfg;
    const motor_node_param_t *trigger_motor_cfg;
    robot_run_variant_e run_variant;
    uint8_t pitch_cali_mode;
    uint16_t period_ms;
    uint32_t period_us;
    uint8_t yaw_turn;
    uint8_t pitch_turn;
    uint8_t yaw_control_is_upper;
} gimbal_runtime_snapshot_t;

//motor encoder value format, range[0, ECD_RANGE - 1]
#define ecd_format(ecd)         \
    {                           \
        if ((ecd) >= ECD_RANGE) \
            (ecd) -= ECD_RANGE; \
        else if ((ecd) < 0)     \
            (ecd) += ECD_RANGE; \
    }

#define gimbal_total_pid_clear(gimbal_clear)                             \
    {                                                                    \
        gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_angle_pid);   \
        PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_gyro_pid);            \
                                                                           \
        gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_angle_pid); \
        PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_gyro_pid);          \
    }

#define PITCH_KICK_ERR_OFF_RAD 0.008f
#define PITCH_KICK_ERR_ON_RAD  0.025f
#define GIMBAL_SDLOG_BASE_STREAM_MAX_SAMPLES 16u

#ifndef GIMBAL_YAW_CHASSIS_SPIN_FF_CURRENT_PER_RADPS
#define GIMBAL_YAW_CHASSIS_SPIN_FF_CURRENT_PER_RADPS 1500.0f
#endif

#ifndef GIMBAL_YAW_CHASSIS_SPIN_FF_MAX_CURRENT
#define GIMBAL_YAW_CHASSIS_SPIN_FF_MAX_CURRENT 6000.0f
#endif

#ifndef GIMBAL_YAW_CHASSIS_SPIN_FF_MIN_WZ_RADPS
#define GIMBAL_YAW_CHASSIS_SPIN_FF_MIN_WZ_RADPS 0.20f
#endif

#ifndef GIMBAL_YAW_CHASSIS_SPIN_EXIT_FF_TIME_MS
#define GIMBAL_YAW_CHASSIS_SPIN_EXIT_FF_TIME_MS 700u
#endif

#ifndef GIMBAL_YAW_CHASSIS_SPIN_EXIT_RATE_CURRENT_PER_RADPS
#define GIMBAL_YAW_CHASSIS_SPIN_EXIT_RATE_CURRENT_PER_RADPS 1200.0f
#endif

#ifndef GIMBAL_YAW_CHASSIS_SPIN_EXIT_ERR_CURRENT_PER_RAD
#define GIMBAL_YAW_CHASSIS_SPIN_EXIT_ERR_CURRENT_PER_RAD 18000.0f
#endif

#ifndef GIMBAL_YAW_CHASSIS_SPIN_EXIT_FF_MAX_CURRENT
#define GIMBAL_YAW_CHASSIS_SPIN_EXIT_FF_MAX_CURRENT 5000.0f
#endif

#ifndef DUAL_YAW_UPPER_TURN
#define DUAL_YAW_UPPER_TURN 0u
#endif

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t gimbal_high_water;
#endif

/**
  * @brief          "gimbal_control" variable initialization, include pid initialization, remote control data point initialization, gimbal motors
  *                 data point initialization, and gyro sensor angle point initialization.
  * @param[out]     init: "gimbal_control" variable point
  * @retval         none
  */
static void gimbal_init(gimbal_control_t *init);

/**
  * @brief          set gimbal control mode, mainly call 'gimbal_behaviour_mode_set' function
  * @param[out]     gimbal_set_mode: "gimbal_control" variable point
  * @retval         none
  */
static void gimbal_set_mode(gimbal_control_t *set_mode);
/**
  * @brief          gimbal some measure data update, such as motor encoder, euler angle, gyro
  * @param[out]     gimbal_feedback_update: "gimbal_control" variable point
  * @retval         none
  */
/**
  * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
  * @param[out]     gimbal_feedback_update:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_snapshot_capture(gimbal_runtime_snapshot_t *snapshot, gimbal_control_t *control);
static void gimbal_feedback_update(gimbal_control_t *feedback_update, const gimbal_runtime_snapshot_t *snapshot);

/**
  * @brief          when gimbal mode change, some param should be changed, such as  yaw_set should be new yaw
  * @param[out]     mode_change: "gimbal_control" variable point
  * @retval         none
  */
/**
  * @brief          云台模式改变，有些参数需要改变，例如控制yaw角度设定值应该变成当前yaw角度
  * @param[out]     mode_change:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_mode_change_control_transit(gimbal_control_t *mode_change);

/**
  * @brief          calculate the encoder angle between ecd and offset_ecd
  * @param[in]      ecd: motor now encode
  * @param[in]      offset_ecd: gimbal offset encode
  * @retval         angle, unit rad
  */
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);
#if GIMBAL_USE_ENCODER_FEEDBACK
static void gimbal_feedback_from_encoder(gimbal_motor_t *motor,
                                         const motor_measure_t *measure,
                                         uint8_t turn);
#endif
/**
  * @brief          set gimbal control set-point, control set-point is set by "gimbal_behaviour_control_set".
  * @param[out]     gimbal_set_control: "gimbal_control" variable point
  * @retval         none
  */
static void gimbal_set_control(gimbal_control_t *set_control);

static void gimbal_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add);
static fp32 gimbal_yaw_chassis_spin_ff_current(const gimbal_motor_t *yaw_motor);
static fp32 gimbal_yaw_kick_current(const gimbal_motor_t *yaw_motor);
static void gimbal_write_state(void);

/**
  * @brief          gimbal control mode :GIMBAL_MOTOR_ENCODER, use the encoder angle to control.
  * @param[out]     gimbal_motor: yaw motor or pitch motor
  * @retval         none
  */
/**
  * @brief          云台控制模式:GIMBAL_MOTOR_ENCODER，使用编码角进行控制
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
  */
static void gimbal_motor_angle_control(gimbal_motor_t *gimbal_motor);

static void gimbal_motor_raw_angle_control(gimbal_motor_t *gimbal_motor);

static void gimbal_control_loop(gimbal_control_t *control_loop);
static int16_t gimbal_apply_output_turn(int16_t current, uint8_t turn);
/**
  * @brief          limit angle set in encoder angle mode, avoid exceeding the max angle
  * @param[out]     gimbal_motor: yaw motor or pitch motor
  * @retval         none
  */
/**
  * @brief          limit angle set in GIMBAL_MOTOR_ENCODER mode, avoid exceeding the max angle
  * @param[out]     gimbal_motor: yaw motor or pitch motor
  * @retval         none
  */

/**
  * @brief          gimbal angle pid init, because angle is in range(-pi,pi),can't use PID in pid.c
  * @param[out]     pid: pid data pointer stucture
  * @param[in]      maxout: pid max out
  * @param[in]      intergral_limit: pid max iout
  * @param[in]      kp: pid kp
  * @param[in]      ki: pid ki
  * @param[in]      kd: pid kd
  * @retval         none
  */
/**
  * @param[in]      ki: pid ki
  * @param[in]      kd: pid kd
  * @retval         none
  */

/**
  * @brief          gimbal PID clear, clear pid.out, iout.
  * @param[out]     pid_clear: "gimbal_control" variable point
  * @retval         none
  */
/**
  * @brief          gimbal angle pid calc, because angle is in range(-pi,pi),can't use PID in pid.c
  * @param[out]     pid: pid data pointer stucture
  * @param[in]      get: angle feedback
  * @param[in]      set: angle set-point
  * @param[in]      error_delta: rotation speed
  * @retval         pid out
  */

/**
  * @brief          gimbal calibration calculate
  * @param[in]      gimbal_cali: cali data
  * @param[out]     yaw_offset:yaw motor middle place encode
  * @param[out]     pitch_offset:pitch motor middle place encode
  * @param[out]     max_yaw:yaw motor max machine angle
  * @param[out]     min_yaw: yaw motor min machine angle
  * @param[out]     max_pitch: pitch motor max machine angle
  * @param[out]     min_pitch: pitch motor min machine angle
  * @retval         none
  */
static void calc_gimbal_cali(const gimbal_step_cali_t *gimbal_cali, uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch);

#if GIMBAL_TEST_MODE
//j-scope 帮助pid调参
static void J_scope_gimbal_test(void);
static fp32 gimbal_pitch_kick_scale(fp32 angle_err);
#endif

//gimbal control data
gimbal_control_t gimbal_control;
volatile uint32_t gimbal_loop_counter = 0;

//motor current
//发送的电机电流
static int16_t yaw_can_set_current = 0, pitch_can_set_current = 0, shoot_can_set_current = 0;
volatile int16_t gimbal_watch_yaw_current = 0;
volatile int16_t gimbal_watch_yaw_upper_current = 0;
volatile int16_t gimbal_watch_pitch_current = 0;
volatile int16_t gimbal_yaw_easytest_current = 3000;

typedef struct
{
    uint32_t start_tick_ms;
    uint32_t last_tick_ms;
    uint32_t period_us;
    uint16_t sample_count;
    uint16_t sample_div_counter;
    sdlog_gimbal_base_sample_t samples[GIMBAL_SDLOG_BASE_STREAM_MAX_SAMPLES];
} gimbal_sdlog_base_stream_state_t;

static gimbal_sdlog_base_stream_state_t s_gimbal_sdlog_base_stream = {0};

#include "gimbal_sdlog_helpers.inc"

#include "gimbal_runtime_helpers.inc"


/**
  * @brief          gimbal task, osDelay GIMBAL_CONTROL_TIME (1ms)
  * @param[in]      pvParameters: null
  * @retval         none
  */

void gimbal_control_task(void const *pvParameters)
{
    TickType_t last_wake = 0;
    //等待陀螺仪任务更新陀螺仪数据
    //wait a time
    vTaskDelay(GIMBAL_TASK_INIT_TIME);
    //gimbal init
    gimbal_init(&gimbal_control);
    gimbal_write_state();
    //shoot init
    shoot_init();
    pitch_cali_boot_load();
    last_wake = xTaskGetTickCount();
    while (1)
    {
        const uint64_t loop_start_us = RtProfBegin();
        gimbal_runtime_snapshot_t snapshot;
        watch_task_beat(WATCH_TASK_GIMBAL_CONTROL);
        gimbal_snapshot_capture(&snapshot, &gimbal_control);
        gimbal_loop_counter++;
        if (!GimbalControlMgrAllows(ControlIdSingleGimbal, &snapshot))
        {
            gimbal_stop_outputs(gimbal_output_current_bindings, GIMBAL_OUTPUT_MOTOR_COUNT);
            RtProfEnd(RtProfGimbalLoop, loop_start_us);
            gimbal_control_delay(&last_wake, snapshot.period_ms);

#if INCLUDE_uxTaskGetStackHighWaterMark
            gimbal_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
            continue;
        }

        gimbal_set_mode(&gimbal_control);                    //设置云台控制模式
        pitch_cali_tick_pre(&gimbal_control, gimbal_behaviour_watch, snapshot.pitch_cali_mode);
        gimbal_mode_change_control_transit(&gimbal_control); //控制模式切换 控制数据过渡
        gimbal_feedback_update(&gimbal_control, &snapshot);  //云台数据反馈
        gimbal_set_control(&gimbal_control);
        gimbal_control_loop(&gimbal_control);
        pitch_cali_tick_post(&gimbal_control, gimbal_behaviour_watch, snapshot.pitch_cali_mode);
        gimbal_write_state();
        shoot_can_set_current = gimbal_run_shoot_control(&snapshot); // 拨盘电流
        yaw_can_set_current = gimbal_apply_output_turn(gimbal_control.gimbal_yaw_motor.given_current, snapshot.yaw_turn);
        pitch_can_set_current = gimbal_apply_output_turn(gimbal_control.gimbal_pitch_motor.given_current, snapshot.pitch_turn);

        const int16_t yaw_current_request = yaw_can_set_current;
        const int16_t pitch_current_request = pitch_can_set_current;

        gimbal_apply_operation_mode(&snapshot, &yaw_can_set_current, &pitch_can_set_current);

        yaw_can_set_current = motor_cfg_limit_current_node(snapshot.yaw_motor_cfg, yaw_can_set_current);
        pitch_can_set_current = motor_cfg_limit_current_node(snapshot.pitch_motor_cfg, pitch_can_set_current);
        shoot_can_set_current = motor_cfg_limit_current_node(snapshot.trigger_motor_cfg, shoot_can_set_current);

        // watch 输出：观察最终下发电流（含运行模式、安全模式及方向翻转后的值）
        gimbal_watch_yaw_current = yaw_can_set_current;
        gimbal_watch_pitch_current = pitch_can_set_current;
        {
            const int16_t gimbal_current_cmd[] = {
                shoot_can_set_current,
                yaw_can_set_current,
                pitch_can_set_current,
            };

            (void)MotorInstSetCurrentBindsBestEffort(gimbal_output_current_bindings,
                                                                      gimbal_current_cmd,
                                                                      GIMBAL_OUTPUT_MOTOR_COUNT);
        }

        {
            sdlog_gimbal_base_sample_t sample = {0};

            sample.gimbal_behaviour = (uint8_t)gimbal_behaviour_watch;
            sample.test_mode = (uint8_t)snapshot.run_variant;
            sample.yaw_motor_mode = (uint8_t)gimbal_control.gimbal_yaw_motor.gimbal_motor_mode;
            sample.pitch_motor_mode = (uint8_t)gimbal_control.gimbal_pitch_motor.gimbal_motor_mode;
            sample.yaw_angle = gimbal_control.gimbal_yaw_motor.angle;
            sample.pitch_angle = gimbal_control.gimbal_pitch_motor.angle;
            sample.yaw_gyro = gimbal_control.gimbal_yaw_motor.motor_gyro;
            sample.pitch_gyro = gimbal_control.gimbal_pitch_motor.motor_gyro;
            sample.yaw_current_request = yaw_current_request;
            sample.pitch_current_request = pitch_current_request;
            sample.yaw_current_output = gimbal_sdlog_clamp_current(yaw_can_set_current);
            sample.pitch_current_output = gimbal_sdlog_clamp_current(pitch_can_set_current);

            gimbal_sdlog_append_base_sample(&sample, bsp_time_get_tick_ms(), snapshot.period_us);
        }

#if GIMBAL_TEST_MODE
        J_scope_gimbal_test();
#endif

        RtProfEnd(RtProfGimbalLoop, loop_start_us);
        gimbal_control_delay(&last_wake, snapshot.period_ms);

#if INCLUDE_uxTaskGetStackHighWaterMark
        gimbal_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

void dual_yaw_gimbal_control_task(void const *pvParameters)
{
    (void)pvParameters;
    TickType_t last_wake = 0;

    vTaskDelay(GIMBAL_TASK_INIT_TIME);
    gimbal_init(&gimbal_control);
    gimbal_write_state();
    shoot_init();

    last_wake = xTaskGetTickCount();
    while (1)
    {
        const uint64_t loop_start_us = RtProfBegin();
        gimbal_runtime_snapshot_t snapshot;

        watch_task_beat(WATCH_TASK_GIMBAL_CONTROL);
        gimbal_snapshot_capture(&snapshot, &gimbal_control);
        gimbal_loop_counter++;
        if (!GimbalControlMgrAllows(ControlIdDualYawGimbal, &snapshot))
        {
            gimbal_stop_outputs(dual_yaw_gimbal_output_current_bindings, DUAL_YAW_GIMBAL_OUTPUT_MOTOR_COUNT);
            RtProfEnd(RtProfGimbalLoop, loop_start_us);
            gimbal_control_delay(&last_wake, snapshot.period_ms);

#if INCLUDE_uxTaskGetStackHighWaterMark
            gimbal_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
            continue;
        }

        gimbal_set_mode(&gimbal_control);
        gimbal_mode_change_control_transit(&gimbal_control);
        gimbal_feedback_update(&gimbal_control, &snapshot);
        gimbal_set_control(&gimbal_control);
        gimbal_control_loop(&gimbal_control);
        gimbal_write_state();

        shoot_can_set_current = gimbal_run_shoot_control(&snapshot);
        yaw_can_set_current = gimbal_apply_output_turn(gimbal_control.gimbal_yaw_motor.given_current, snapshot.yaw_turn);
        pitch_can_set_current = gimbal_apply_output_turn(gimbal_control.gimbal_pitch_motor.given_current, snapshot.pitch_turn);

        const int16_t yaw_current_request = yaw_can_set_current;
        const int16_t pitch_current_request = pitch_can_set_current;

        gimbal_apply_operation_mode(&snapshot, &yaw_can_set_current, &pitch_can_set_current);

        int16_t yaw_upper_can_set_current = gimbal_apply_output_turn(yaw_can_set_current, (uint8_t)DUAL_YAW_UPPER_TURN);
        int16_t yaw_log_current_output;

        if (snapshot.yaw_control_is_upper != 0u)
        {
            yaw_upper_can_set_current = yaw_can_set_current;
            yaw_can_set_current = 0;
        }

        yaw_can_set_current = motor_cfg_limit_current_node(&g_config.motor.yaw, yaw_can_set_current);
        yaw_upper_can_set_current = motor_cfg_limit_current_node(&g_config.motor.yaw_upper, yaw_upper_can_set_current);
        pitch_can_set_current = motor_cfg_limit_current_node(snapshot.pitch_motor_cfg, pitch_can_set_current);
        shoot_can_set_current = motor_cfg_limit_current_node(snapshot.trigger_motor_cfg, shoot_can_set_current);
        yaw_log_current_output = (snapshot.yaw_control_is_upper != 0u) ? yaw_upper_can_set_current : yaw_can_set_current;

        gimbal_watch_yaw_current = yaw_can_set_current;
        gimbal_watch_yaw_upper_current = yaw_upper_can_set_current;
        gimbal_watch_pitch_current = pitch_can_set_current;
        {
            const int16_t gimbal_current_cmd[] = {
                shoot_can_set_current,
                yaw_can_set_current,
                yaw_upper_can_set_current,
                pitch_can_set_current,
            };

            (void)MotorInstSetCurrentBindsBestEffort(dual_yaw_gimbal_output_current_bindings,
                                                                      gimbal_current_cmd,
                                                                      DUAL_YAW_GIMBAL_OUTPUT_MOTOR_COUNT);
        }

        {
            sdlog_gimbal_base_sample_t sample = {0};

            sample.gimbal_behaviour = (uint8_t)gimbal_behaviour_watch;
            sample.test_mode = (uint8_t)snapshot.run_variant;
            sample.yaw_motor_mode = (uint8_t)gimbal_control.gimbal_yaw_motor.gimbal_motor_mode;
            sample.pitch_motor_mode = (uint8_t)gimbal_control.gimbal_pitch_motor.gimbal_motor_mode;
            sample.yaw_angle = gimbal_control.gimbal_yaw_motor.angle;
            sample.pitch_angle = gimbal_control.gimbal_pitch_motor.angle;
            sample.yaw_gyro = gimbal_control.gimbal_yaw_motor.motor_gyro;
            sample.pitch_gyro = gimbal_control.gimbal_pitch_motor.motor_gyro;
            sample.yaw_current_request = yaw_current_request;
            sample.pitch_current_request = pitch_current_request;
            sample.yaw_current_output = gimbal_sdlog_clamp_current(yaw_log_current_output);
            sample.pitch_current_output = gimbal_sdlog_clamp_current(pitch_can_set_current);

            gimbal_sdlog_append_base_sample(&sample, bsp_time_get_tick_ms(), snapshot.period_us);
        }

        RtProfEnd(RtProfGimbalLoop, loop_start_us);
        gimbal_control_delay(&last_wake, snapshot.period_ms);

#if INCLUDE_uxTaskGetStackHighWaterMark
        gimbal_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

#include "gimbal_cali_helpers.inc"

#include "gimbal_core_control.inc"

#include "gimbal_tune_api.inc"

#endif
