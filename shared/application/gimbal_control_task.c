/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "gimbal_control_task.h"

#include "cmsis_os.h"

#include "arm_math.h"
#include "CAN_receive.h"
#include "actuator_cmd.h"
#include "motor_config.h"
#include "user_lib.h"
#include "axis_current_conditioner.h"
#include "detect_task.h"
#include "manual_input.h"
#include "control_input.h"
#include "gimbal_behaviour.h"
#include "INS_task.h"
#include "shoot.h"
#include "pid.h"
#include "host_link_task.h"
#include "watch.h"
#include "sdlog.h"
#include "pitch_cali.h"
#include "bsp_time.h"
#include "rt_profiler.h"
#include "robot_task_profile.h"

#include <string.h>

typedef struct
{
    const manual_input_state_t *manual_input;
    const fp32 *gyro;
    const fp32 *ins_angle;
    const motor_measure_t *yaw_measure;
    const motor_measure_t *pitch_measure;
    const motor_node_param_t *yaw_motor_cfg;
    const motor_node_param_t *pitch_motor_cfg;
    const motor_node_param_t *trigger_motor_cfg;
    test_mode_e test_mode;
    uint16_t period_ms;
    uint32_t period_us;
    uint8_t yaw_turn;
    uint8_t pitch_turn;
} gimbal_runtime_snapshot_t;

//motor enconde value format, range[0-8191]
#define ecd_format(ecd)         \
    {                           \
        if ((ecd) > ECD_RANGE)  \
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

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t gimbal_high_water;
#endif

/**
  * @brief          "gimbal_control" valiable initialization, include pid initialization, remote control data point initialization, gimbal motors
  *                 data point initialization, and gyro sensor angle point initialization.
  * @param[out]     init: "gimbal_control" valiable point
  * @retval         none
  */
static void gimbal_init(gimbal_control_t *init);

/**
  * @brief          set gimbal control mode, mainly call 'gimbal_behaviour_mode_set' function
  * @param[out]     gimbal_set_mode: "gimbal_control" valiable point
  * @retval         none
  */
static void gimbal_set_mode(gimbal_control_t *set_mode);
/**
  * @brief          gimbal some measure data updata, such as motor enconde, euler angle, gyro
  * @param[out]     gimbal_feedback_update: "gimbal_control" valiable point
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
  * @brief          when gimbal mode change, some param should be changed, suan as  yaw_set should be new yaw
  * @param[out]     mode_change: "gimbal_control" valiable point
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
/**
  * @brief          set gimbal control set-point, control set-point is set by "gimbal_behaviour_control_set".
  * @param[out]     gimbal_set_control: "gimbal_control" valiable point
  * @retval         none
  */
static void gimbal_set_control(gimbal_control_t *set_control);

static void gimbal_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add);

/**
  * @brief          gimbal control mode :GIMBAL_MOTOR_ENCONDE, use the encode angle to control.
  * @param[out]     gimbal_motor: yaw motor or pitch motor
  * @retval         none
  */
/**
  * @brief          云台控制模式:GIMBAL_MOTOR_ENCONDE，使用编码角进行控制
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
  */
static void gimbal_motor_angle_control(gimbal_motor_t *gimbal_motor);

static void gimbal_motor_raw_angle_control(gimbal_motor_t *gimbal_motor);

static void gimbal_control_loop(gimbal_control_t *control_loop);
/**
  * @brief          limit angle set in encoder angle mode, avoid exceeding the max angle
  * @param[out]     gimbal_motor: yaw motor or pitch motor
  * @retval         none
  */
/**
  * @brief          limit angle set in GIMBAL_MOTOR_ENCONDE mode, avoid exceeding the max angle
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
  * @param[out]     pid_clear: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          gimbal angle pid calc, because angle is in range(-pi,pi),can't use PID in pid.c
  * @param[out]     pid: pid data pointer stucture
  * @param[in]      get: angle feeback
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
volatile int16_t gimbal_watch_pitch_current = 0;
volatile int16_t gimbal_yaw_easytest_current = 3000;

extern shoot_control_t shoot_control;

typedef struct
{
    uint32_t start_tick_ms;
    uint32_t last_tick_ms;
    uint32_t period_us;
    uint16_t sample_count;
    sdlog_gimbal_base_sample_t samples[GIMBAL_SDLOG_BASE_STREAM_MAX_SAMPLES];
} gimbal_sdlog_base_stream_state_t;

static gimbal_sdlog_base_stream_state_t s_gimbal_sdlog_base_stream = {0};

static int16_t gimbal_sdlog_clamp_current(fp32 current)
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

static void gimbal_sdlog_begin_base_stream(uint32_t now_ms, uint32_t period_us)
{
    s_gimbal_sdlog_base_stream.start_tick_ms = now_ms;
    s_gimbal_sdlog_base_stream.last_tick_ms = now_ms;
    s_gimbal_sdlog_base_stream.period_us = period_us;
    s_gimbal_sdlog_base_stream.sample_count = 0u;
}

static void gimbal_sdlog_flush_base_stream(void)
{
    if (s_gimbal_sdlog_base_stream.sample_count == 0u)
    {
        return;
    }

    sdlog_gimbal_base_stream_header_t header = {0};
    uint8_t payload[sizeof(header) + sizeof(s_gimbal_sdlog_base_stream.samples)];
    const uint16_t payload_len =
        (uint16_t)(sizeof(header) + (uint32_t)s_gimbal_sdlog_base_stream.sample_count * sizeof(sdlog_gimbal_base_sample_t));

    header.start_tick_ms = s_gimbal_sdlog_base_stream.start_tick_ms;
    header.period_us = (s_gimbal_sdlog_base_stream.period_us > 0xFFFFu) ? 0xFFFFu : (uint16_t)s_gimbal_sdlog_base_stream.period_us;
    header.sample_count = (uint8_t)s_gimbal_sdlog_base_stream.sample_count;
    header.version = SDLOG_GIMBAL_BASE_STREAM_VERSION;

    memcpy(payload, &header, sizeof(header));
    memcpy(&payload[sizeof(header)],
           s_gimbal_sdlog_base_stream.samples,
           (uint32_t)s_gimbal_sdlog_base_stream.sample_count * sizeof(sdlog_gimbal_base_sample_t));
    sdlog_write(SDLOG_TAG_GIMBAL_BASE_STREAM, payload, payload_len);
    s_gimbal_sdlog_base_stream.sample_count = 0u;
}

static void gimbal_sdlog_append_base_sample(const sdlog_gimbal_base_sample_t *sample,
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

    if (s_gimbal_sdlog_base_stream.sample_count == 0u)
    {
        gimbal_sdlog_begin_base_stream(now_ms, period_us);
    }
    else
    {
        const uint32_t expected_dt_ms = (s_gimbal_sdlog_base_stream.period_us + 999u) / 1000u;
        const uint32_t actual_dt_ms = now_ms - s_gimbal_sdlog_base_stream.last_tick_ms;
        if (s_gimbal_sdlog_base_stream.period_us != period_us || actual_dt_ms != expected_dt_ms)
        {
            gimbal_sdlog_flush_base_stream();
            gimbal_sdlog_begin_base_stream(now_ms, period_us);
        }
    }

    s_gimbal_sdlog_base_stream.samples[s_gimbal_sdlog_base_stream.sample_count++] = *sample;
    s_gimbal_sdlog_base_stream.last_tick_ms = now_ms;

    if (s_gimbal_sdlog_base_stream.sample_count >= GIMBAL_SDLOG_BASE_STREAM_MAX_SAMPLES)
    {
        gimbal_sdlog_flush_base_stream();
    }
}

static fp32 gimbal_pitch_kick_scale(fp32 angle_err)
{
    const fp32 err_abs = fabsf(angle_err);

    if (err_abs <= PITCH_KICK_ERR_OFF_RAD)
    {
        return 0.0f;
    }
    if (err_abs >= PITCH_KICK_ERR_ON_RAD)
    {
        return 1.0f;
    }

    return (err_abs - PITCH_KICK_ERR_OFF_RAD) / (PITCH_KICK_ERR_ON_RAD - PITCH_KICK_ERR_OFF_RAD);
}

static test_mode_e current_test_mode(void)
{
    return (test_mode_e)g_config.test.mode;
}

static void gimbal_snapshot_capture(gimbal_runtime_snapshot_t *snapshot, gimbal_control_t *control)
{
    if (snapshot == NULL)
    {
        return;
    }

    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->manual_input = (control != NULL) ? control->gimbal_rc_ctrl : get_remote_control_point();
    snapshot->gyro = (control != NULL) ? control->gimbal_INT_gyro_point : get_gyro_data_point();
    snapshot->ins_angle = (control != NULL) ? control->gimbal_INS_angle_point : get_INS_angle_point();
    snapshot->yaw_measure = (control != NULL) ? control->gimbal_yaw_motor.gimbal_motor_measure : get_yaw_gimbal_motor_measure_point();
    snapshot->pitch_measure = (control != NULL) ? control->gimbal_pitch_motor.gimbal_motor_measure : get_pitch_gimbal_motor_measure_point();
    snapshot->yaw_motor_cfg = &g_config.motor.yaw;
    snapshot->pitch_motor_cfg = &g_config.motor.pitch;
    snapshot->trigger_motor_cfg = &g_config.motor.trigger;
    snapshot->test_mode = current_test_mode();
    snapshot->period_ms = robot_profile_gimbal_control_period_ms();
    snapshot->period_us = (uint32_t)snapshot->period_ms * 1000u;
    snapshot->yaw_turn = YAW_TURN ? 1u : 0u;
    snapshot->pitch_turn = PITCH_TURN ? 1u : 0u;

    if (control != NULL)
    {
        gimbal_control_snapshot_t fast = {0};
        const manual_input_state_t *manual_input = snapshot->manual_input;

        fast.dbus_offline = toe_is_error(DBUS_TOE);
        fast.test_mode = snapshot->test_mode;
        fast.mode_sw = input_switch(INPUT_SW_GIMBAL_MODE);
        fast.safe_pos = g_config.manual_input.semantics.gimbal_safe_pos;
        fast.active_source = remote_control_get_active_source();
        fast.image_auto_aim_requested = image_remote_auto_aim_requested() ? 1u : 0u;
        fast.yaw_axis = input_axis(INPUT_AXIS_GIMBAL_YAW);
        fast.pitch_axis = input_axis(INPUT_AXIS_GIMBAL_PITCH);
        fast.mouse_x = (manual_input != NULL) ? manual_input->mouse.x : 0;
        fast.mouse_y = (manual_input != NULL) ? manual_input->mouse.y : 0;
        fast.rc_deadband = RC_DEADBAND;
        fast.key_mask = (manual_input != NULL) ? manual_input->key.v : 0u;
        fast.turn_key_mask = TURN_KEYBOARD;
        fast.turn_speed = TURN_SPEED;
        fast.yaw_rc_sen = YAW_RC_SEN;
        fast.pitch_rc_sen = PITCH_RC_SEN;
        fast.yaw_mouse_sen = YAW_MOUSE_SEN;
        fast.pitch_mouse_sen = PITCH_MOUSE_SEN;

        control->fast = fast;
    }
}

static void gimbal_apply_test_mode(const gimbal_runtime_snapshot_t *snapshot, int16_t *yaw_current, int16_t *pitch_current)
{
    const test_mode_e mode = (snapshot != NULL) ? snapshot->test_mode : current_test_mode();

    if (gimbal_behaviour_watch == GIMBAL_ZERO_FORCE)
    {
        gimbal_total_pid_clear(&gimbal_control);
        *yaw_current = 0;
        *pitch_current = 0;
        return;
    }

    switch (mode)
    {
    case TEST_MODE_NONE:
    case TEST_MODE_GIMBAL_DUAL:
    case TEST_MODE_ENTERTAIN:
    case TEST_MODE_PITCH_CALI:
        return;
    case TEST_MODE_YAW_ONLY:
        gimbal_PID_clear(&gimbal_control.gimbal_pitch_motor.gimbal_motor_angle_pid);
        PID_clear(&gimbal_control.gimbal_pitch_motor.gimbal_motor_gyro_pid);
        *pitch_current = 0;
        return;
    case TEST_MODE_PITCH_ONLY:
        gimbal_PID_clear(&gimbal_control.gimbal_yaw_motor.gimbal_motor_angle_pid);
        PID_clear(&gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid);
        *yaw_current = 0;
        return;
    case TEST_MODE_YAW_EASY_TEST:
        gimbal_total_pid_clear(&gimbal_control);
        *yaw_current = gimbal_yaw_easytest_current;
        *pitch_current = 0;
        return;
    default:
        gimbal_total_pid_clear(&gimbal_control);
        *yaw_current = 0;
        *pitch_current = 0;
        return;
    }
}

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
    //shoot init
    shoot_init();
    pitch_cali_boot_load();
    last_wake = xTaskGetTickCount();
    while (1)
    {
        const uint64_t loop_start_us = rt_profiler_begin();
        gimbal_runtime_snapshot_t snapshot;
        watch_task_beat(WATCH_TASK_GIMBAL_CONTROL);
        gimbal_snapshot_capture(&snapshot, &gimbal_control);
        gimbal_loop_counter++;
        gimbal_set_mode(&gimbal_control);                    //设置云台控制模式
        pitch_cali_tick_pre(&gimbal_control, gimbal_behaviour_watch, snapshot.test_mode);
        gimbal_mode_change_control_transit(&gimbal_control); //控制模式切换 控制数据过渡
        gimbal_feedback_update(&gimbal_control, &snapshot);  //云台数据反馈
        gimbal_set_control(&gimbal_control);
        gimbal_control_loop(&gimbal_control);
        pitch_cali_tick_post(&gimbal_control, gimbal_behaviour_watch, snapshot.test_mode);
        shoot_can_set_current = shoot_control_loop();        // 拨盘电流
        if (snapshot.yaw_turn != 0u)
        {
            yaw_can_set_current = -gimbal_control.gimbal_yaw_motor.given_current;
        }
        else
        {
            yaw_can_set_current = gimbal_control.gimbal_yaw_motor.given_current;
        }

        if (snapshot.pitch_turn != 0u)
        {
            pitch_can_set_current = -gimbal_control.gimbal_pitch_motor.given_current;
        }
        else
        {
            pitch_can_set_current = gimbal_control.gimbal_pitch_motor.given_current;
        }

        const int16_t yaw_current_request = yaw_can_set_current;
        const int16_t pitch_current_request = pitch_can_set_current;

        gimbal_apply_test_mode(&snapshot, &yaw_can_set_current, &pitch_can_set_current);

        yaw_can_set_current = motor_cfg_limit_current_node(snapshot.yaw_motor_cfg, yaw_can_set_current);
        pitch_can_set_current = motor_cfg_limit_current_node(snapshot.pitch_motor_cfg, pitch_can_set_current);
        shoot_can_set_current = motor_cfg_limit_current_node(snapshot.trigger_motor_cfg, shoot_can_set_current);

        // watch 输出：观察最终下发电流（含测试模式、安全模式及方向翻转后的值）
        gimbal_watch_yaw_current = yaw_can_set_current;
        gimbal_watch_pitch_current = pitch_can_set_current;
        actuator_cmd_set_trigger_current(shoot_can_set_current);
        actuator_cmd_set_yaw_current(yaw_can_set_current);
        actuator_cmd_set_pitch_current(pitch_can_set_current);

        {
            sdlog_gimbal_base_sample_t sample = {0};

            sample.gimbal_behaviour = (uint8_t)gimbal_behaviour_watch;
            sample.test_mode = (uint8_t)snapshot.test_mode;
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

        rt_profiler_end(RT_PROFILER_GIMBAL_CONTROL_LOOP, loop_start_us);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(snapshot.period_ms));

#if INCLUDE_uxTaskGetStackHighWaterMark
        gimbal_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

/**
  * @brief          gimbal cali data, set motor offset encode, max and min angle
  * @param[in]      yaw_offse:yaw middle place encode
  * @param[in]      pitch_offset:pitch place encode
  * @param[in]      max_yaw:yaw max angle
  * @param[in]      min_yaw:yaw min angle
  * @param[in]      max_yaw:pitch max angle
  * @param[in]      min_yaw:pitch min angle
  * @retval         none
  */
void set_cali_gimbal_hook(const uint16_t yaw_offset, const uint16_t pitch_offset, const fp32 max_yaw, const fp32 min_yaw, const fp32 max_pitch, const fp32 min_pitch)
{
    gimbal_control.gimbal_yaw_motor.offset_ecd = yaw_offset;
    gimbal_control.gimbal_yaw_motor.max_angle = max_yaw;
    gimbal_control.gimbal_yaw_motor.min_angle = min_yaw;

    gimbal_control.gimbal_pitch_motor.offset_ecd = pitch_offset;
    gimbal_control.gimbal_pitch_motor.max_angle = max_pitch;
    gimbal_control.gimbal_pitch_motor.min_angle = min_pitch;
}

/**
  * @brief          gimbal cali calculate, return motor offset encode, max and min angle
  * @param[out]     yaw_offse:yaw middle place encode
  * @param[out]     pitch_offset:pitch place encode
  * @param[out]     max_yaw:yaw max angle
  * @param[out]     min_yaw:yaw min angle
  * @param[out]     max_yaw:pitch max angle
  * @param[out]     min_yaw:pitch min angle
  * @retval         none
  */
bool_t cmd_cali_gimbal_hook(uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch)
{
    if (gimbal_control.gimbal_cali.step == 0)
    {
        gimbal_control.gimbal_cali.step             = GIMBAL_CALI_START_STEP;
        gimbal_control.gimbal_cali.max_pitch        = gimbal_control.gimbal_pitch_motor.angle;
        gimbal_control.gimbal_cali.max_pitch_ecd    = gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->ecd;
        gimbal_control.gimbal_cali.max_yaw          = gimbal_control.gimbal_yaw_motor.angle;
        gimbal_control.gimbal_cali.max_yaw_ecd      = gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd;
        gimbal_control.gimbal_cali.min_pitch        = gimbal_control.gimbal_pitch_motor.angle;
        gimbal_control.gimbal_cali.min_pitch_ecd    = gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->ecd;
        gimbal_control.gimbal_cali.min_yaw          = gimbal_control.gimbal_yaw_motor.angle;
        gimbal_control.gimbal_cali.min_yaw_ecd      = gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd;
        return 0;
    }
    else if (gimbal_control.gimbal_cali.step == GIMBAL_CALI_END_STEP)
    {
        calc_gimbal_cali(&gimbal_control.gimbal_cali, yaw_offset, pitch_offset, max_yaw, min_yaw, max_pitch, min_pitch);
        (*max_yaw) -= GIMBAL_CALI_REDUNDANT_ANGLE;
        (*min_yaw) += GIMBAL_CALI_REDUNDANT_ANGLE;
        (*max_pitch) -= GIMBAL_CALI_REDUNDANT_ANGLE;
        (*min_pitch) += GIMBAL_CALI_REDUNDANT_ANGLE;
        gimbal_control.gimbal_yaw_motor.offset_ecd              = *yaw_offset;
        gimbal_control.gimbal_yaw_motor.max_angle      = *max_yaw;
        gimbal_control.gimbal_yaw_motor.min_angle      = *min_yaw;
        gimbal_control.gimbal_pitch_motor.offset_ecd            = *pitch_offset;
        gimbal_control.gimbal_pitch_motor.max_angle    = *max_pitch;
        gimbal_control.gimbal_pitch_motor.min_angle    = *min_pitch;
        gimbal_control.gimbal_cali.step = 0;
        return 1;
    }
    else
    {
        return 0;
    }
}

/**
  * @brief          calc motor offset encode, max and min angle
  * @param[out]     yaw_offse:yaw middle place encode
  * @param[out]     pitch_offset:pitch place encode
  * @param[out]     max_yaw:yaw max angle
  * @param[out]     min_yaw:yaw min angle
  * @param[out]     max_yaw:pitch max angle
  * @param[out]     min_yaw:pitch min angle
  * @retval         none
  */
static void calc_gimbal_cali(const gimbal_step_cali_t *gimbal_cali, uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch)
{
    if (gimbal_cali == NULL || yaw_offset == NULL || pitch_offset == NULL || max_yaw == NULL || min_yaw == NULL || max_pitch == NULL || min_pitch == NULL)
    {
        return;
    }

    int16_t temp_max_ecd = 0, temp_min_ecd = 0, temp_ecd = 0;

    if (YAW_TURN)
    {
        temp_ecd = gimbal_cali->min_yaw_ecd - gimbal_cali->max_yaw_ecd;

        if (temp_ecd < 0)
        {
        temp_ecd += ECD_RANGE;
        }
        temp_ecd = gimbal_cali->max_yaw_ecd + (temp_ecd / 2);

        ecd_format(temp_ecd);
        *yaw_offset = temp_ecd;
        *max_yaw = -motor_ecd_to_angle_change(gimbal_cali->max_yaw_ecd, *yaw_offset);
        *min_yaw = -motor_ecd_to_angle_change(gimbal_cali->min_yaw_ecd, *yaw_offset);
    }
    else
    {

        temp_ecd = gimbal_cali->max_yaw_ecd - gimbal_cali->min_yaw_ecd;

        if (temp_ecd < 0)
        {
            temp_ecd += ECD_RANGE;
        }
        temp_ecd = gimbal_cali->max_yaw_ecd - (temp_ecd / 2);

        ecd_format(temp_ecd);
        *yaw_offset = temp_ecd;
        *max_yaw = motor_ecd_to_angle_change(gimbal_cali->max_yaw_ecd, *yaw_offset);
        *min_yaw = motor_ecd_to_angle_change(gimbal_cali->min_yaw_ecd, *yaw_offset);
    }

    if (PITCH_TURN)

    {
        temp_ecd = (int16_t)(gimbal_cali->max_pitch / MOTOR_ECD_TO_RAD);
        temp_max_ecd = gimbal_cali->max_pitch_ecd + temp_ecd;
        temp_ecd = (int16_t)(gimbal_cali->min_pitch / MOTOR_ECD_TO_RAD);
        temp_min_ecd = gimbal_cali->min_pitch_ecd + temp_ecd;

        ecd_format(temp_max_ecd);
        ecd_format(temp_min_ecd);

        temp_ecd = temp_max_ecd - temp_min_ecd;

        if (temp_ecd > HALF_ECD_RANGE)
        {
            temp_ecd -= ECD_RANGE;
        }
        else if (temp_ecd < -HALF_ECD_RANGE)
        {
            temp_ecd += ECD_RANGE;
        }

        if (temp_max_ecd > temp_min_ecd)
        {
            temp_min_ecd += ECD_RANGE;
        }

        temp_ecd = temp_max_ecd - temp_ecd / 2;

        ecd_format(temp_ecd);

        *pitch_offset = temp_ecd;

        *max_pitch = -motor_ecd_to_angle_change(gimbal_cali->max_pitch_ecd, *pitch_offset);
        *min_pitch = -motor_ecd_to_angle_change(gimbal_cali->min_pitch_ecd, *pitch_offset);
    }
    else
    {
        temp_ecd = (int16_t)(gimbal_cali->max_pitch / MOTOR_ECD_TO_RAD);
        temp_max_ecd = gimbal_cali->max_pitch_ecd - temp_ecd;
        temp_ecd = (int16_t)(gimbal_cali->min_pitch / MOTOR_ECD_TO_RAD);
        temp_min_ecd = gimbal_cali->min_pitch_ecd - temp_ecd;

        ecd_format(temp_max_ecd);
        ecd_format(temp_min_ecd);

        temp_ecd = temp_max_ecd - temp_min_ecd;

        if (temp_ecd > HALF_ECD_RANGE)
        {
            temp_ecd -= ECD_RANGE;
        }
        else if (temp_ecd < -HALF_ECD_RANGE)
        {
            temp_ecd += ECD_RANGE;
        }

        temp_ecd = temp_max_ecd - temp_ecd / 2;

        ecd_format(temp_ecd);

        *pitch_offset = temp_ecd;

        *max_pitch = motor_ecd_to_angle_change(gimbal_cali->max_pitch_ecd, *pitch_offset);
        *min_pitch = motor_ecd_to_angle_change(gimbal_cali->min_pitch_ecd, *pitch_offset);
    }
}

/**
  * @brief          return yaw motor data point
  * @param[in]      none
  * @retval         yaw motor data point
  */
/**
  * @brief          返回yaw 电机数据指针
  * @param[in]      none
  * @retval         yaw电机指针
  */
const gimbal_motor_t *get_yaw_motor_point(void)
{
    return &gimbal_control.gimbal_yaw_motor;
}

/**
  * @brief          return pitch motor data point
  * @param[in]      none
  * @retval         pitch motor data point
  */
/**
  * @brief          返回pitch 电机数据指针
  * @param[in]      none
  * @retval         pitch
  */
const gimbal_motor_t *get_pitch_motor_point(void)
{
    return &gimbal_control.gimbal_pitch_motor;
}

/**
  * @brief          "gimbal_control" valiable initialization, include pid initialization, remote control data point initialization, gimbal motors
  *                 data point initialization, and gyro sensor angle point initialization.
  * @param[out]     init: "gimbal_control" valiable point
  * @retval         none
  */
static void gimbal_init(gimbal_control_t *init)
{

    const fp32 Pitch_speed_pid[3] = {PITCH_SPEED_PID_KP, PITCH_SPEED_PID_KI, PITCH_SPEED_PID_KD};
    const fp32 Yaw_speed_pid[3] = {YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD};
    //电机数据指针获取
    init->gimbal_yaw_motor.gimbal_motor_measure = get_yaw_gimbal_motor_measure_point();
    init->gimbal_pitch_motor.gimbal_motor_measure = get_pitch_gimbal_motor_measure_point();
    init->gimbal_INT_gyro_point = get_gyro_data_point();
    init->gimbal_INS_angle_point = get_INS_angle_point();
    init->gimbal_rc_ctrl = get_remote_control_point();
    init->gimbal_pitch_motor.gimbal_motor_mode = init->gimbal_pitch_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    init->gimbal_yaw_motor.gimbal_motor_mode = init->gimbal_yaw_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    //初始化yaw电机pid
    gimbal_PID_init(&init->gimbal_yaw_motor.gimbal_motor_angle_pid, YAW_ENCODE_ANGLE_PID_MAX_OUT, YAW_ENCODE_ANGLE_PID_MAX_IOUT, YAW_ENCODE_ANGLE_PID_KP, YAW_ENCODE_ANGLE_PID_KI, YAW_ENCODE_ANGLE_PID_KD);
    PID_init(&init->gimbal_yaw_motor.gimbal_motor_gyro_pid, PID_POSITION, Yaw_speed_pid, YAW_SPEED_PID_MAX_OUT, YAW_SPEED_PID_MAX_IOUT);
    //初始化pitch电机pid
    gimbal_PID_init(&init->gimbal_pitch_motor.gimbal_motor_angle_pid, PITCH_ENCODE_ANGLE_PID_MAX_OUT, PITCH_ENCODE_ANGLE_PID_MAX_IOUT, PITCH_ENCODE_ANGLE_PID_KP, PITCH_ENCODE_ANGLE_PID_KI, PITCH_ENCODE_ANGLE_PID_KD);
    PID_init(&init->gimbal_pitch_motor.gimbal_motor_gyro_pid, PID_POSITION, Pitch_speed_pid, PITCH_SPEED_PID_MAX_OUT, PITCH_SPEED_PID_MAX_IOUT);
    {
        const fp32 limit = fabsf(PITCH_CURRENT_LIMIT);
        if (limit > 0.0f)
        {
            if (init->gimbal_pitch_motor.gimbal_motor_gyro_pid.max_out > limit)
            {
                init->gimbal_pitch_motor.gimbal_motor_gyro_pid.max_out = limit;
            }
            if (init->gimbal_pitch_motor.gimbal_motor_gyro_pid.max_iout > limit)
            {
                init->gimbal_pitch_motor.gimbal_motor_gyro_pid.max_iout = limit;
            }
        }
    }

    //清除所有PID
    gimbal_total_pid_clear(init);
    init->gimbal_yaw_motor.offset_ecd = g_config.gimbal.yaw_middle_ecd;

    {
        gimbal_runtime_snapshot_t snapshot;
        gimbal_snapshot_capture(&snapshot, init);
        gimbal_feedback_update(init, &snapshot);
    }

    init->gimbal_yaw_motor.angle_set = init->gimbal_yaw_motor.angle;
    init->gimbal_yaw_motor.motor_gyro_set = init->gimbal_yaw_motor.motor_gyro;

    init->gimbal_pitch_motor.angle_set = init->gimbal_pitch_motor.angle;
    init->gimbal_pitch_motor.motor_gyro_set = init->gimbal_pitch_motor.motor_gyro;
    if ((init->gimbal_yaw_motor.max_angle <= init->gimbal_yaw_motor.min_angle) ||

        ((init->gimbal_yaw_motor.max_angle == 0.0f) && (init->gimbal_yaw_motor.min_angle == 0.0f)) ||
        (fabsf(init->gimbal_yaw_motor.max_angle) > 100.0f) ||
        (fabsf(init->gimbal_yaw_motor.min_angle) > 100.0f))
    {
        init->gimbal_yaw_motor.max_angle = PI;
        init->gimbal_yaw_motor.min_angle = -PI;
    }

    {
        const fp32 cfg_up = PITCH_SOFT_LIMIT_UP;
        const fp32 cfg_down = PITCH_SOFT_LIMIT_DOWN;
        fp32 cfg_min = cfg_up;
        fp32 cfg_max = cfg_down;
        if (cfg_max < cfg_min)
        {
            const fp32 t = cfg_max;
            cfg_max = cfg_min;
            cfg_min = t;
        }
        if ((cfg_max > cfg_min) && (fabsf(cfg_max) < 100.0f) && (fabsf(cfg_min) < 100.0f))
        {
            init->gimbal_pitch_motor.max_angle = cfg_max;
            init->gimbal_pitch_motor.min_angle = cfg_min;
        }
        else
        {
            if ((init->gimbal_pitch_motor.max_angle <= init->gimbal_pitch_motor.min_angle) ||
                ((init->gimbal_pitch_motor.max_angle == 0.0f) && (init->gimbal_pitch_motor.min_angle == 0.0f)) ||
                (fabsf(init->gimbal_pitch_motor.max_angle) > 100.0f) ||
                (fabsf(init->gimbal_pitch_motor.min_angle) > 100.0f))
            {
                init->gimbal_pitch_motor.max_angle = 4.0f * PI;
                init->gimbal_pitch_motor.min_angle = -4.0f * PI;
            }
        }
    }

}

/**
  * @brief          set gimbal control mode, mainly call 'gimbal_behaviour_mode_set' function
  * @param[out]     gimbal_set_mode: "gimbal_control" valiable point
  * @retval         none
  */
static void gimbal_set_mode(gimbal_control_t *set_mode)
{
    if (set_mode == NULL)
    {
        return;
    }
    gimbal_behaviour_mode_set(set_mode);
}
/**
  * @brief          gimbal some measure data updata, such as motor enconde, euler angle, gyro
  * @param[out]     gimbal_feedback_update: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
  * @param[out]     gimbal_feedback_update:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_feedback_update(gimbal_control_t *feedback_update, const gimbal_runtime_snapshot_t *snapshot)
{
    if (feedback_update == NULL)
    {
        return;
    }

    {
        fp32 pitch_angle_raw = 0.0f;
        fp32 pitch_speed_raw = 0.0f;

        const fp32 *ins_angle = (snapshot != NULL) ? snapshot->ins_angle : feedback_update->gimbal_INS_angle_point;
        const fp32 *gyro = (snapshot != NULL) ? snapshot->gyro : feedback_update->gimbal_INT_gyro_point;
        const uint8_t pitch_turn = (snapshot != NULL) ? snapshot->pitch_turn : (PITCH_TURN ? 1u : 0u);

        if (ins_angle != NULL)
        {
            pitch_angle_raw = ins_angle[INS_ROLL_ADDRESS_OFFSET];
        }
        if (gyro != NULL)
        {
            pitch_speed_raw = gyro[INS_GYRO_X_ADDRESS_OFFSET];
        }

        feedback_update->gimbal_pitch_motor.angle = (pitch_turn != 0u) ? -pitch_angle_raw : pitch_angle_raw;
        feedback_update->gimbal_pitch_motor.motor_gyro = (pitch_turn != 0u) ? -pitch_speed_raw : pitch_speed_raw;
    }

    {
        fp32 yaw_angle_raw = 0.0f;
        fp32 yaw_speed_raw = 0.0f;
        const fp32 *ins_angle = (snapshot != NULL) ? snapshot->ins_angle : feedback_update->gimbal_INS_angle_point;
        const fp32 *gyro = (snapshot != NULL) ? snapshot->gyro : feedback_update->gimbal_INT_gyro_point;
        const uint8_t yaw_turn = (snapshot != NULL) ? snapshot->yaw_turn : (YAW_TURN ? 1u : 0u);

        if (ins_angle != NULL)
        {
            yaw_angle_raw = ins_angle[INS_YAW_ADDRESS_OFFSET];
        }

        if (gyro != NULL)
        {
            yaw_speed_raw = gyro[INS_GYRO_Z_ADDRESS_OFFSET];
        }

        feedback_update->gimbal_yaw_motor.angle = (yaw_turn != 0u) ? -yaw_angle_raw : yaw_angle_raw;
        feedback_update->gimbal_yaw_motor.motor_gyro = (yaw_turn != 0u) ? -yaw_speed_raw : yaw_speed_raw;
    }
}

/**
  * @brief          calculate the encoder angle between ecd and offset_ecd
  * @param[in]      ecd: motor now encode
  * @param[in]      offset_ecd: gimbal offset encode
  * @retval         angle, unit rad
  */
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
    int32_t relative_ecd = ecd - offset_ecd;
    if (relative_ecd > HALF_ECD_RANGE)
    {
        relative_ecd -= ECD_RANGE;
    }
    else if (relative_ecd < -HALF_ECD_RANGE)
    {
        relative_ecd += ECD_RANGE;
    }

    return relative_ecd * MOTOR_ECD_TO_RAD;
}

/**
  * @brief          when gimbal mode change, some param should be changed, suan as  yaw_set should be new yaw
  * @param[out]     gimbal_mode_change: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          云台模式改变，有些参数需要改变，例如控制yaw角度设定值应该变成当前yaw角度
  * @param[out]     gimbal_mode_change:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_mode_change_control_transit(gimbal_control_t *gimbal_mode_change)
{
    if (gimbal_mode_change == NULL)
    {
        return;
    }
    //yaw电机状态机切换保存数据
    if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_mode_change->gimbal_yaw_motor.raw_cmd_current = gimbal_mode_change->gimbal_yaw_motor.current_set = gimbal_mode_change->gimbal_yaw_motor.given_current;
    }
    else if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_mode_change->gimbal_yaw_motor.angle_set = gimbal_mode_change->gimbal_yaw_motor.angle;
    }
    gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode;

    //pitch电机状态机切换保存数据
    if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_mode_change->gimbal_pitch_motor.raw_cmd_current = gimbal_mode_change->gimbal_pitch_motor.current_set = gimbal_mode_change->gimbal_pitch_motor.given_current;
    }
    else if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_mode_change->gimbal_pitch_motor.angle_set = gimbal_mode_change->gimbal_pitch_motor.angle;
    }

    gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode;
}
/**
  * @brief          set gimbal control set-point, control set-point is set by "gimbal_behaviour_control_set".
  * @param[out]     gimbal_set_control: "gimbal_control" valiable point
  * @retval         none
  */
static void gimbal_set_control(gimbal_control_t *set_control)
{
    fp32 add_yaw_angle = 0.0f;
    fp32 add_pitch_angle = 0.0f;
    VisionToGimbal vision_cmd;
    uint8_t image_manual_active;
    uint8_t image_auto_aim_on;

    if (set_control == NULL)
    {
        return;
    }

    gimbal_behaviour_control_set(&add_yaw_angle, &add_pitch_angle, set_control);

    image_manual_active = (set_control->fast.active_source == MANUAL_INPUT_SRC_IMAGE) ? 1u : 0u;
    image_auto_aim_on = set_control->fast.image_auto_aim_requested;
    if (vision_take_latest(&vision_cmd) && (vision_cmd.mode == 1U || vision_cmd.mode == 2U) &&
        (image_manual_active == 0u || image_auto_aim_on != 0u) &&
        set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE &&
        set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE &&
        gimbal_behaviour_watch == GIMBAL_ANGLE &&
        !gimbal_turnaround_is_active())
    {
        set_control->gimbal_yaw_motor.angle_set = rad_format(vision_cmd.yaw);
        set_control->gimbal_pitch_motor.angle_set = fp32_constrain(-vision_cmd.pitch,
                                                                   set_control->gimbal_pitch_motor.min_angle,
                                                                   set_control->gimbal_pitch_motor.max_angle);
        add_yaw_angle = 0.0f;
        add_pitch_angle = 0.0f;
    }

    if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        set_control->gimbal_yaw_motor.raw_cmd_current = add_yaw_angle;
    }
    else if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_angle_limit(&set_control->gimbal_yaw_motor, add_yaw_angle);
    }

    if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        set_control->gimbal_pitch_motor.raw_cmd_current = add_pitch_angle;
    }
    else if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_angle_limit(&set_control->gimbal_pitch_motor, add_pitch_angle);
    }
}

static void gimbal_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    gimbal_motor->angle_set += add;

    // Yaw uses IMU Euler angle in (-PI, PI). To support continuous rotation, always wrap the setpoint
    // instead of clamping at +/- PI.
    if (gimbal_motor == &gimbal_control.gimbal_yaw_motor)
    {
        gimbal_motor->angle_set = rad_format(gimbal_motor->angle_set);
        return;
    }
    if (gimbal_motor->angle_set > gimbal_motor->max_angle)
    {
        gimbal_motor->angle_set = gimbal_motor->max_angle;
    }
    else if (gimbal_motor->angle_set < gimbal_motor->min_angle)
    {
        gimbal_motor->angle_set = gimbal_motor->min_angle;
    }
}
static void gimbal_control_loop(gimbal_control_t *control_loop)
{
    if (control_loop == NULL)
    {
        return;
    }

    if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_motor_raw_angle_control(&control_loop->gimbal_yaw_motor);
    }
    else if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_motor_angle_control(&control_loop->gimbal_yaw_motor);
    }

    if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_motor_raw_angle_control(&control_loop->gimbal_pitch_motor);
    }
    else if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_motor_angle_control(&control_loop->gimbal_pitch_motor);
    }
}

/**
  * @brief          gimbal control mode :GIMBAL_MOTOR_ENCONDE, use the encode angle to control.
  * @param[out]     gimbal_motor: yaw motor or pitch motor
  * @retval         none
  */
/**
  * @brief          云台控制模式:GIMBAL_MOTOR_ENCONDE，使用编码角进行控制
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
  */
static void gimbal_motor_angle_control(gimbal_motor_t *gimbal_motor)
{
    fp32 pitch_err;
    fp32 kick_up;
    fp32 kick_down;
    fp32 ff_hold;
    fp32 ff_kick_up;
    fp32 ff_kick_down;
    fp32 kick_scale;
    fp32 current_limit;
    axis_current_conditioner_info_t limit_info;

    if (gimbal_motor == NULL)
    {
        return;
    }

    gimbal_motor->motor_gyro_set = gimbal_PID_calc(&gimbal_motor->gimbal_motor_angle_pid,
                                                   gimbal_motor->angle,
                                                   gimbal_motor->angle_set,
                                                   gimbal_motor->motor_gyro);
    gimbal_motor->current_set = PID_calc(&gimbal_motor->gimbal_motor_gyro_pid,
                                         gimbal_motor->motor_gyro,
                                         gimbal_motor->motor_gyro_set);
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);

    if (gimbal_motor == &gimbal_control.gimbal_pitch_motor)
    {
        pitch_err = gimbal_motor->angle_set - gimbal_motor->angle;
        kick_up = fabsf(PITCH_KICK_UP_CURRENT);
        kick_down = fabsf(PITCH_KICK_DOWN_CURRENT);
        ff_hold = 0.0f;
        ff_kick_up = 0.0f;
        ff_kick_down = 0.0f;
        kick_scale = 0.0f;
        current_limit = fabsf(PITCH_CURRENT_LIMIT);
        memset(&limit_info, 0, sizeof(limit_info));

        if (pitch_cali_get_comp(gimbal_motor->angle, &ff_hold, &ff_kick_up, &ff_kick_down))
        {
            gimbal_motor->current_set += ff_hold;
            kick_up = fabsf(ff_kick_up);
            kick_down = fabsf(ff_kick_down);
        }

        kick_scale = gimbal_pitch_kick_scale(pitch_err);
        kick_up *= kick_scale;
        kick_down *= kick_scale;

        gimbal_motor->current_set = axis_current_conditioner_apply(
            gimbal_motor->current_set,
            gimbal_motor->motor_gyro_set,
            kick_up,
            kick_down,
            gimbal_motor->angle,
            gimbal_motor->min_angle,
            gimbal_motor->max_angle,
            current_limit,
            &gimbal_motor->gimbal_motor_angle_pid.Iout,
            &gimbal_motor->gimbal_motor_gyro_pid.Iout,
            &limit_info);

        gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
    }
}

static void gimbal_motor_raw_angle_control(gimbal_motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    gimbal_motor->current_set = gimbal_motor->raw_cmd_current;
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}

#if GIMBAL_TEST_MODE
int32_t yaw_angle_int_1000, pitch_angle_int_1000;
int32_t yaw_angle_set_1000, pitch_angle_set_1000;
int32_t yaw_speed_int_1000, pitch_speed_int_1000;
int32_t yaw_speed_set_int_1000, pitch_speed_set_int_1000;
static void J_scope_gimbal_test(void)
{
    yaw_angle_int_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.angle * 1000);
    yaw_angle_set_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.angle_set * 1000);
    yaw_speed_int_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.motor_gyro * 1000);
    yaw_speed_set_int_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.motor_gyro_set * 1000);

    pitch_angle_int_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.angle * 1000);
    pitch_angle_set_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.angle_set * 1000);
    pitch_speed_int_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.motor_gyro * 1000);
    pitch_speed_set_int_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.motor_gyro_set * 1000);
}

#endif

/**
  * @brief          "gimbal_control" valiable initialization, include pid initialization, remote control data point initialization, gimbal motors
  *                 data point initialization, and gyro sensor angle point initialization.
  * @param[out]     gimbal_init: "gimbal_control" valiable point
  * @retval         none
  */
void gimbal_tune_get_yaw_speed_pid(pid_param_t *out)
{
    if (out == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    const pid_type_def *pid = &gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid;
    out->kp = pid->Kp;
    out->ki = pid->Ki;
    out->kd = pid->Kd;
    out->max_out = pid->max_out;
    out->max_iout = pid->max_iout;
    taskEXIT_CRITICAL();
}

void gimbal_tune_get_yaw_angle_pid(pid_param_t *out)
{
    if (out == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    const gimbal_PID_t *pid = &gimbal_control.gimbal_yaw_motor.gimbal_motor_angle_pid;
    out->kp = pid->kp;
    out->ki = pid->ki;
    out->kd = pid->kd;
    out->max_out = pid->max_out;
    out->max_iout = pid->max_iout;
    taskEXIT_CRITICAL();
}

void gimbal_tune_set_yaw_speed_pid(const pid_param_t *pid, bool_t clear_state)
{
    if (pid == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    pid_type_def *dst = &gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid;
    dst->Kp = pid->kp;
    dst->Ki = pid->ki;
    dst->Kd = pid->kd;
    dst->max_out = pid->max_out;
    dst->max_iout = pid->max_iout;
    if (clear_state)
    {
        PID_clear(dst);
    }
    taskEXIT_CRITICAL();
}

void gimbal_tune_set_yaw_angle_pid(const pid_param_t *pid, bool_t clear_state)
{
    if (pid == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    gimbal_PID_t *dst = &gimbal_control.gimbal_yaw_motor.gimbal_motor_angle_pid;
    dst->kp = pid->kp;
    dst->ki = pid->ki;
    dst->kd = pid->kd;
    dst->max_out = pid->max_out;
    dst->max_iout = pid->max_iout;
    if (clear_state)
    {
        gimbal_PID_clear(dst);
    }
    taskEXIT_CRITICAL();
}

void gimbal_tune_clear_yaw_pid(void)
{
    taskENTER_CRITICAL();
    gimbal_PID_clear(&gimbal_control.gimbal_yaw_motor.gimbal_motor_angle_pid);
    PID_clear(&gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid);
    taskEXIT_CRITICAL();
}

void gimbal_tune_get_pitch_speed_pid(pid_param_t *out)
{
    const pid_type_def *pid;

    if (out == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    pid = &gimbal_control.gimbal_pitch_motor.gimbal_motor_gyro_pid;
    out->kp = pid->Kp;
    out->ki = pid->Ki;
    out->kd = pid->Kd;
    out->max_out = pid->max_out;
    out->max_iout = pid->max_iout;
    taskEXIT_CRITICAL();
}

void gimbal_tune_get_pitch_angle_pid(pid_param_t *out)
{
    const gimbal_PID_t *pid;

    if (out == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    pid = &gimbal_control.gimbal_pitch_motor.gimbal_motor_angle_pid;
    out->kp = pid->kp;
    out->ki = pid->ki;
    out->kd = pid->kd;
    out->max_out = pid->max_out;
    out->max_iout = pid->max_iout;
    taskEXIT_CRITICAL();
}

void gimbal_tune_set_pitch_speed_pid(const pid_param_t *pid, bool_t clear_state)
{
    pid_type_def *dst;
    fp32 max_out;
    fp32 max_iout;
    fp32 limit;

    if (pid == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    dst = &gimbal_control.gimbal_pitch_motor.gimbal_motor_gyro_pid;
    dst->Kp = pid->kp;
    dst->Ki = pid->ki;
    dst->Kd = pid->kd;
    max_out = fabsf(pid->max_out);
    max_iout = fabsf(pid->max_iout);
    limit = fabsf(PITCH_CURRENT_LIMIT);
    if (limit > 0.0f)
    {
        if (max_out > limit)
        {
            max_out = limit;
        }
        if (max_iout > limit)
        {
            max_iout = limit;
        }
    }
    dst->max_out = max_out;
    dst->max_iout = max_iout;
    if (clear_state)
    {
        PID_clear(dst);
    }
    taskEXIT_CRITICAL();
}

void gimbal_tune_set_pitch_angle_pid(const pid_param_t *pid, bool_t clear_state)
{
    gimbal_PID_t *dst;

    if (pid == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    dst = &gimbal_control.gimbal_pitch_motor.gimbal_motor_angle_pid;
    dst->kp = pid->kp;
    dst->ki = pid->ki;
    dst->kd = pid->kd;
    dst->max_out = pid->max_out;
    dst->max_iout = pid->max_iout;
    if (clear_state)
    {
        gimbal_PID_clear(dst);
    }
    taskEXIT_CRITICAL();
}

void gimbal_tune_clear_pitch_pid(void)
{
    taskENTER_CRITICAL();
    gimbal_PID_clear(&gimbal_control.gimbal_pitch_motor.gimbal_motor_angle_pid);
    PID_clear(&gimbal_control.gimbal_pitch_motor.gimbal_motor_gyro_pid);
    taskEXIT_CRITICAL();
}
