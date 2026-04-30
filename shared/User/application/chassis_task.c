/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */


#include "chassis_task.h"
#include "chassis_behaviour.h"
#include "gimbal_behaviour.h"

#include "cmsis_os.h"

#include <math.h>

#include "arm_math.h"
#include "pid.h"
#include "remote_control.h"
#include "app_input.h"
#include "CAN_receive.h"
#include "actuator_cmd.h"
#include "motor_config.h"
#include "app_watch.h"
#include "detect_task.h"
#include "INS_task.h"
#include "kalman_filter.h"
#include "chassis_power_control.h"
#include "sdlog.h"

#include <string.h>

#define CHASSIS_MOTOR_COUNT 4U

// Chassis follow-yaw stop window (reduces dithering when nearly aligned).
// NOTE: yaw error uses gimbal-relative angle (rad), wz uses chassis rotation speed (rad/s).
#define CHASSIS_FOLLOW_YAW_STOP_ERR_RAD (0.01f)
#define CHASSIS_FOLLOW_YAW_STOP_WZ_RADPS (0.10f)

// Chassis yaw-rate fusion (wheel odom + IMU yaw-rate minus gimbal-yaw rate).
// IMU is installed on the gimbal, so gyro Z includes chassis yaw-rate + gimbal yaw-rate.
// We subtract gimbal yaw motor rate to obtain chassis yaw-rate measurement.
#define CHASSIS_GIMBAL_MOTOR_RPM_TO_RADPS (0.104719755f) // 2*pi/60
#define CHASSIS_WZ_KF_Q_WZ (0.0010f)
#define CHASSIS_WZ_KF_Q_GYRO_BIAS (0.00001f)
#define CHASSIS_WZ_KF_R_WHEEL (0.050f)
#define CHASSIS_WZ_KF_R_IMU (0.200f)

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

static fp32 chassis_get_gimbal_yaw_relative_angle(const gimbal_motor_t *yaw_motor)
{
    if (yaw_motor == NULL || yaw_motor->gimbal_motor_measure == NULL)
    {
        return 0.0f;
    }

    int32_t relative_ecd = (int32_t)yaw_motor->gimbal_motor_measure->ecd - (int32_t)yaw_motor->offset_ecd;
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

static fp32 chassis_get_gimbal_yaw_relative_rate(const gimbal_motor_t *yaw_motor)
{
    if (yaw_motor == NULL || yaw_motor->gimbal_motor_measure == NULL)
    {
        return 0.0f;
    }

    fp32 w = (fp32)yaw_motor->gimbal_motor_measure->speed_rpm * CHASSIS_GIMBAL_MOTOR_RPM_TO_RADPS;
    return YAW_TURN ? -w : w;
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

static bool_t test_mode_allow_chassis(void)
{
    test_mode_e mode = (test_mode_e)g_config.test.mode;
    return (mode == TEST_MODE_NONE) || (mode == TEST_MODE_ENTERTAIN) || (mode == TEST_MODE_CHASSIS_ONLY);
}


/**
  * @brief          "chassis_move" valiable initialization, include pid initialization, remote control data point initialization, 3508 chassis motors
  *                 data point initialization, gimbal motor data point initialization, and gyro sensor angle point initialization.
  * @param[out]     chassis_move_init: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          初始化"chassis_move"变量，包括pid初始化， 遥控器指针初始化，3508底盘电机指针初始化，云台电机初始化，陀螺仪角度指针初始化
  * @param[out]     chassis_move_init:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_init(chassis_move_t *chassis_move_init);

/**
  * @brief          set chassis control mode, mainly call 'chassis_behaviour_mode_set' function
  * @param[out]     chassis_move_mode: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          设置底盘控制模式，主要在'chassis_behaviour_mode_set'函数中改变
  * @param[out]     chassis_move_mode:"chassis_move"变量指针.
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
void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);
/**
  * @brief          chassis some measure data updata, such as motor speed, euler angle， robot speed
  * @param[out]     chassis_move_update: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
  * @param[out]     chassis_move_update:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_feedback_update(chassis_move_t *chassis_move_update);
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
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop);

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t chassis_high_water;
#endif



//底盘运动数据
chassis_move_t chassis_move;
volatile uint32_t chassis_loop_counter = 0;

static void sdlog_pack_pid(sdlog_pid_runtime_t *out, uint16_t pid_id, const pid_type_def *pid)
{
    if (out == NULL || pid == NULL)
    {
        return;
    }

    memset(out, 0, sizeof(*out));
    out->pid_id = pid_id;
    out->mode = pid->mode;
    out->set = pid->set;
    out->fdb = pid->fdb;
    out->out = pid->out;
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

/**
  * @brief          chassis task, osDelay CHASSIS_CONTROL_TIME_MS (2ms)
  * @param[in]      pvParameters: null
  * @retval         none
  */
/**
  * @brief          底盘任务，间隔 CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: 空
  * @retval         none
  */
void chassis_task(void const *pvParameters)
{
    //wait a time
    //空闲一段时间
    vTaskDelay(CHASSIS_TASK_INIT_TIME);
    //chassis init
    //底盘初始化
    chassis_init(&chassis_move);
    // 只等待遥控上线，其他掉线不阻塞启动
    while (toe_is_error(DBUS_TOE))
    {
        app_watch_task_wait(APP_WATCH_TASK_CHASSIS);
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);
    }

    while (1)
    {
        app_watch_task_beat(APP_WATCH_TASK_CHASSIS);
        //set chassis control mode
        //设置底盘控制模式
        chassis_set_mode(&chassis_move);
        //when mode changes, some data save
        //模式切换数据保存
        chassis_mode_change_control_transit(&chassis_move);
        //chassis data update
        //底盘数据更新
        chassis_feedback_update(&chassis_move);

        // test mode: only none/chassis_only allow normal chassis control
        if (!test_mode_allow_chassis())
        {
            for (uint8_t i = 0; i < CHASSIS_MOTOR_COUNT; i++)
            {
                PID_clear(&chassis_move.motor_speed_pid[i]);
                chassis_move.motor_chassis[i].speed_set = 0.0f;
                chassis_move.motor_chassis[i].give_current = 0;
            }
            chassis_move.vx_set = 0.0f;
            chassis_move.vy_set = 0.0f;
            chassis_move.wz_set = 0.0f;

            taskENTER_CRITICAL();
            for (uint8_t i = 0; i < CHASSIS_MOTOR_COUNT; i++)
            {
                actuator_cmd_set_chassis_current_can1(i, 0);
            }
            taskEXIT_CRITICAL();

            vTaskDelay(CHASSIS_CONTROL_TIME_MS);

#if INCLUDE_uxTaskGetStackHighWaterMark
            chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
            continue;
        }

        //set chassis control set-point
        //底盘控制量设置
        chassis_set_contorl(&chassis_move);
        //chassis control pid calculate
        //底盘控制PID计算
        chassis_control_loop(&chassis_move);

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
        if (!toe_is_error(DBUS_TOE))
        {
            for (uint8_t i = 0; i < CHASSIS_MOTOR_COUNT; i++)
            {
                const int16_t current = chassis_move.motor_chassis[i].give_current;
                chassis_current_cmd[i] = motor_cfg_limit_current_node(&g_config.motor.chassis[i], current);
            }
        }

        taskENTER_CRITICAL();
        for (uint8_t i = 0; i < CHASSIS_MOTOR_COUNT; i++)
        {
            actuator_cmd_set_chassis_current_can1(i, chassis_current_cmd[i]);
        }
        taskEXIT_CRITICAL();

        chassis_loop_counter++;
        sdlog_chassis_loop_t log = {0};
        log.loop_cnt = chassis_loop_counter;
        log.chassis_mode = (uint8_t)chassis_move.chassis_mode;
        log.last_chassis_mode = (uint8_t)chassis_move.last_chassis_mode;
        log.vx = chassis_move.vx;
        log.vy = chassis_move.vy;
        log.wz = chassis_move.wz;
        log.vx_set = chassis_move.vx_set;
        log.vy_set = chassis_move.vy_set;
        log.wz_set = chassis_move.wz_set;
        sdlog_pack_pid(&log.motor_speed_pid[0], SDLOG_PID_CHASSIS_M1_SPEED, &chassis_move.motor_speed_pid[0]);
        sdlog_pack_pid(&log.motor_speed_pid[1], SDLOG_PID_CHASSIS_M2_SPEED, &chassis_move.motor_speed_pid[1]);
        sdlog_pack_pid(&log.motor_speed_pid[2], SDLOG_PID_CHASSIS_M3_SPEED, &chassis_move.motor_speed_pid[2]);
        sdlog_pack_pid(&log.motor_speed_pid[3], SDLOG_PID_CHASSIS_M4_SPEED, &chassis_move.motor_speed_pid[3]);
        sdlog_pack_pid(&log.chassis_angle_pid, SDLOG_PID_CHASSIS_FOLLOW, &chassis_move.chassis_angle_pid);
        sdlog_write(SDLOG_TAG_CHASSIS_LOOP, &log, (uint16_t)sizeof(log));
        //os delay
        //系统延时
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);

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
/**
  * @brief          初始化"chassis_move"变量，包括pid初始化， 遥控器指针初始化，3508底盘电机指针初始化，云台电机初始化，陀螺仪角度指针初始化
  * @param[out]     chassis_move_init:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_init(chassis_move_t *chassis_move_init)
{
    if (chassis_move_init == NULL)
    {
        return;
    }

    //chassis motor speed PID
    //底盘速度环pid值
    const fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};

    //chassis angle PID
    //底盘角度pid值
    const fp32 chassis_yaw_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};

    const fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    const fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
    uint8_t i;

    //in beginning， chassis mode is raw
    //底盘开机状态为原始
    chassis_move_init->chassis_mode = CHASSIS_VECTOR_RAW;
    //get remote control point
    //获取遥控器指针
    chassis_move_init->chassis_RC = get_remote_control_point();
    //get gyro sensor euler angle point
    //获取陀螺仪姿态角指针
    chassis_move_init->chassis_INS_angle = get_INS_angle_point();
    chassis_INT_gyro_point = get_gyro_data_point();
    //get gimbal motor data point
    //获取云台电机数据指针
    chassis_move_init->chassis_yaw_motor = get_yaw_motor_point();
    chassis_move_init->chassis_pitch_motor = get_pitch_motor_point();

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
    //用一阶滤波代替斜波函数生成
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);

    //max and min speed
    //最大 最小速度
    chassis_move_init->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
    chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;

    chassis_move_init->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
    chassis_move_init->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;

    //update data
    //更新一下数据
    chassis_feedback_update(chassis_move_init);
}

/**
  * @brief          set chassis control mode, mainly call 'chassis_behaviour_mode_set' function
  * @param[out]     chassis_move_mode: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          设置底盘控制模式，主要在'chassis_behaviour_mode_set'函数中改变
  * @param[out]     chassis_move_mode:"chassis_move"变量指针.
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
    //切入不跟随云台模式
    else if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_NO_FOLLOW_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
    {
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
    }

    chassis_move_transit->last_chassis_mode = chassis_move_transit->chassis_mode;
}

/**
  * @brief          chassis some measure data updata, such as motor speed, euler angle， robot speed
  * @param[out]     chassis_move_update: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
  * @param[out]     chassis_move_update:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_feedback_update(chassis_move_t *chassis_move_update)
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
        chassis_move_update->motor_chassis[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_move_update->motor_chassis[i].chassis_motor_measure->speed_rpm * g_config.chassis.motor_dir[i];
        chassis_move_update->motor_chassis[i].accel = chassis_move_update->motor_speed_pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;
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
    if (g_config.chassis.wheel_type == (uint8_t)CHASSIS_WHEEL_TYPE_XDRIVE)
    {
        // X-drive (45° omni) inverse kinematics:
        // w0 = +vx +vy +yaw, w1 = -vx +vy +yaw, w2 = -vx -vy +yaw, w3 = +vx -vy +yaw
        chassis_move_update->vx = (w0 - w1 - w2 + w3) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
        chassis_move_update->vy = (w0 + w1 - w2 - w3) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
        wz_wheel = (w0 + w1 + w2 + w3) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;
    }
    else
    {
        // Mecanum inverse kinematics (DJI legacy mapping):
        // w0 = +vx -vy -yaw, w1 = +vx +vy +yaw, w2 = +vx +vy -yaw, w3 = +vx -vy +yaw
        chassis_move_update->vx = (w0 + w1 + w2 + w3) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
        chassis_move_update->vy = (-w0 + w1 + w2 - w3) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
        wz_wheel = (-w0 + w1 - w2 + w3) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;
    }
    chassis_move_update->wz = wz_wheel;

    // Fuse yaw-rate using IMU (on gimbal) if available.
    // - In chassis-only test mode, keep wheel odom only to avoid relying on IMU/gimbal signals.
    bool_t imu_valid = 0;
    fp32 wz_imu = 0.0f;
    if (((test_mode_e)g_config.test.mode) != TEST_MODE_CHASSIS_ONLY &&
        !toe_is_error(RM_IMU_TOE) &&
        !toe_is_error(YAW_GIMBAL_MOTOR_TOE) &&
        chassis_INT_gyro_point != NULL &&
        chassis_move_update->chassis_yaw_motor != NULL)
    {
        const fp32 gyro_z = chassis_INT_gyro_point[INS_GYRO_Z_ADDRESS_OFFSET];
        const fp32 gimbal_wz = chassis_get_gimbal_yaw_relative_rate(chassis_move_update->chassis_yaw_motor);
        wz_imu = gyro_z - gimbal_wz;
        imu_valid = 1;
    }
    chassis_move_update->wz = chassis_wz_kf_step(wz_wheel, imu_valid, wz_imu);

    //calculate chassis euler angle, if chassis add a new gyro sensor,please change this code
    //计算底盘姿态角度；chassis_only 测试模式下不使用 IMU/编码器姿态，保持为 0 以简化仅速度控制
    if (((test_mode_e)g_config.test.mode) == TEST_MODE_CHASSIS_ONLY)
    {
        chassis_move_update->chassis_yaw = 0.0f;
        chassis_move_update->chassis_pitch = 0.0f;
        chassis_move_update->chassis_roll = 0.0f;
    }
    else
    {
        const fp32 yaw_relative = chassis_get_gimbal_yaw_relative_angle(chassis_move_update->chassis_yaw_motor);
        chassis_move_update->chassis_yaw = rad_format(*(chassis_move_update->chassis_INS_angle + INS_YAW_ADDRESS_OFFSET) - yaw_relative);
        chassis_move_update->chassis_pitch = rad_format(*(chassis_move_update->chassis_INS_angle + INS_PITCH_ADDRESS_OFFSET) - chassis_move_update->chassis_pitch_motor->angle);
        chassis_move_update->chassis_roll = *(chassis_move_update->chassis_INS_angle + INS_ROLL_ADDRESS_OFFSET);
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
    //deadline, because some remote control need be calibrated,  the value of rocker is not zero in middle place,
    //死区限制，因为遥控器可能存在差异 摇杆在中间，其值不为0
    rc_deadband_limit(app_input_axis(INPUT_AXIS_CHASSIS_X), vx_channel, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(app_input_axis(INPUT_AXIS_CHASSIS_Y), vy_channel, CHASSIS_RC_DEADLINE);

    vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;
    // vy: positive means left, negative means right.
    vy_set_channel = vy_channel * -CHASSIS_VY_RC_SEN;

    //keyboard set speed set-point
    //键盘控制
    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY)
    {
        vx_set_channel = chassis_move_rc_to_vector->vx_max_speed;
    }
    else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY)
    {
        vx_set_channel = chassis_move_rc_to_vector->vx_min_speed;
    }

    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY)
    {
        vy_set_channel = chassis_move_rc_to_vector->vy_max_speed;
    }
    else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY)
    {
        vy_set_channel = chassis_move_rc_to_vector->vy_min_speed;
    }

    //first order low-pass replace ramp function, calculate chassis speed set-point to improve control performance
    //一阶低通滤波代替斜波作为底盘速度输入
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, vy_set_channel);
    //stop command, need not slow change, set zero derectly
    //停止信号，不需要缓慢加速，直接减速到零
    if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
    }

    if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
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
/**
  * @brief          设置底盘控制设置值, 三运动控制值是通过chassis_behaviour_control_set函数设置的
  * @param[out]     chassis_move_update:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_set_contorl(chassis_move_t *chassis_move_control)
{

    if (chassis_move_control == NULL)
    {
        return;
    }


    fp32 vx_set = 0.0f, vy_set = 0.0f, angle_set = 0.0f;
    //get three control set-point, 获取三个控制设置值
    chassis_behaviour_control_set(&vx_set, &vy_set, &angle_set, chassis_move_control);

    //follow gimbal mode
    //跟随云台模式
    if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
        const fp32 yaw_relative_meas = chassis_get_gimbal_yaw_relative_angle(chassis_move_control->chassis_yaw_motor);
        fp32 yaw_frame = yaw_relative_meas;
        fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
        (void)gimbal_turnaround_get_frame_yaw_relative(&yaw_frame);
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
        //设置底盘控制的角度
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
        //“angle_set” 是旋转速度控制
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
/**
  * @brief          四轮速度解算（麦轮/全向轮 X-drive）
  * @param[in]      vx_set: 纵向速度
  * @param[in]      vy_set: 横向速度
  * @param[in]      wz_set: 旋转速度
  * @param[out]     wheel_speed: 四个麦轮速度
  * @retval         none
  */
static void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4])
{
    // Wheel order: 0=LF(0x201), 1=RF(0x202), 2=LR(0x203), 3=RR(0x204)
    // Robot frame: vx forward +, vy left +, wz CCW +
    const fp32 yaw_term = MOTOR_DISTANCE_TO_CENTER * wz_set;

    if (g_config.chassis.wheel_type == (uint8_t)CHASSIS_WHEEL_TYPE_XDRIVE)
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
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
    fp32 max_vector = 0.0f, vector_rate = 0.0f;
    fp32 temp = 0.0f;
    fp32 wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t i = 0;

    chassis_power_control_apply_speed_limit(chassis_move_control_loop);

    //mecanum wheel speed calculation
    //麦轮运动分解
    chassis_vector_to_mecanum_wheel_speed(chassis_move_control_loop->vx_set,
                                          chassis_move_control_loop->vy_set, chassis_move_control_loop->wz_set, wheel_speed);

    if (chassis_move_control_loop->chassis_mode == CHASSIS_VECTOR_RAW)
    {

        for (i = 0; i < 4; i++)
        {
            chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(wheel_speed[i] * g_config.chassis.motor_dir[i]);
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

    if (max_vector > MAX_WHEEL_SPEED)
    {
        vector_rate = MAX_WHEEL_SPEED / max_vector;
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
    }


    //功率控制
    chassis_power_control(chassis_move_control_loop);


    //赋值电流值
    for (i = 0; i < 4; i++)
    {
        chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(chassis_move_control_loop->motor_speed_pid[i].out * g_config.chassis.motor_dir[i]);
    }
}
