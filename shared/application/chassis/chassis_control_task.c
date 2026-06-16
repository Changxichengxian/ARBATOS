/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

/*
 * 阅读地图：
 * - 前段：底盘运行快照、日志缓存、调参接口。
 * - 中段：模式/反馈更新、运动学换算、功率限制前后的电流计算。
 * - 后段：chassis_control_task() 主循环，处理运行模式、离线保护和日志。
 * - 输出：电流命令写入 LowCmd，由 CAN 发送任务统一发出。
 */

#include "config.h"
#include "robot_task_build_config.h"

#if ROBOT_TASK_BUILD_CLASSIC_CHASSIS

#include "chassis_control_task.h"
#include "chassis_behaviour.h"
#include "chassis_core.h"

#include "cmsis_os.h"

#include <math.h>

#include "arm_math.h"
#include "pid.h"
#include "manual_input.h"
#include "control_input.h"
#include "CAN_receive.h"
#include "LowCmd.h"
#include "chassis_state.h"
#include "motor_instance.h"
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
#include "robot_mode.h"
#include "control_manager.h"

#include <string.h>

#define CHASSIS_MOTOR_COUNT 4U

static const char *const chassis_motor_instance_names[CHASSIS_MOTOR_COUNT] = {
    "motor.chassis0",
    "motor.chassis1",
    "motor.chassis2",
    "motor.chassis3",
};
static motor_instance_current_binding_t chassis_motor_current_bindings[CHASSIS_MOTOR_COUNT];
static const int16_t chassis_zero_current_cmd[CHASSIS_MOTOR_COUNT] = {0};

// Chassis follow-yaw stop window (reduces dithering when nearly aligned).
// NOTE: yaw error uses gimbal-relative angle (rad), wz uses chassis rotation speed (rad/s).
#define CHASSIS_FOLLOW_YAW_STOP_ERR_RAD (0.01f)
#define CHASSIS_FOLLOW_YAW_STOP_WZ_RADPS (0.10f)

#ifndef HALF_ECD_RANGE
#define HALF_ECD_RANGE 4096
#endif

#ifndef ECD_RANGE
#define ECD_RANGE 8192
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

#ifndef CHASSIS_USE_IMU_YAW_FEEDBACK
#define CHASSIS_USE_IMU_YAW_FEEDBACK 1u
#endif

typedef struct
{
    manual_input_state_t manual_input_copy;
    const manual_input_state_t *manual_input;
    uint8_t gimbal_state_valid;
    uint8_t gimbal_online;
    gimbal_motor_state_t yaw_motor;
    gimbal_motor_state_t pitch_motor;
    const fp32 *ins_angle;
    const fp32 *gyro;
    const motor_measure_t *motor_measure[CHASSIS_MOTOR_COUNT];
    const motor_node_param_t *motor_cfg[CHASSIS_MOTOR_COUNT];
    uint8_t chassis_only_mode;
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

#include "chassis_snapshot_helpers.inc"


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

#include "chassis_sdlog_helpers.inc"

#include "chassis_tune_state.inc"


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
        if (!operation_mode_allow_chassis(&snapshot))
        {
            chassis_control_stop_outputs(&chassis_move);
            chassis_write_state(&chassis_move);
            rt_profiler_end(RT_PROFILER_CHASSIS_CONTROL_LOOP, loop_start_us);
            chassis_control_delay(&last_wake, snapshot.period_ms);

#if INCLUDE_uxTaskGetStackHighWaterMark
            chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
            continue;
        }

        if (!chassis_control_manager_allows(&snapshot))
        {
            chassis_control_stop_outputs(&chassis_move);
            chassis_write_state(&chassis_move);
            rt_profiler_end(RT_PROFILER_CHASSIS_CONTROL_LOOP, loop_start_us);
            chassis_control_delay(&last_wake, snapshot.period_ms);

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

        (void)motor_instance_cmd_set_current_bindings_best_effort(chassis_motor_current_bindings,
                                                                  chassis_current_cmd,
                                                                  CHASSIS_MOTOR_COUNT);

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
        chassis_write_state(&chassis_move);
        //os delay
        //系统延时
        rt_profiler_end(RT_PROFILER_CHASSIS_CONTROL_LOOP, loop_start_us);
        chassis_control_delay(&last_wake, snapshot.period_ms);

#if INCLUDE_uxTaskGetStackHighWaterMark
        chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

#include "chassis_core_control.inc"

#endif
