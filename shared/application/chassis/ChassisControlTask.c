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
 * - 后段：ChassisControlTask() 主循环，处理运行模式、离线保护和日志。
 * - 输出：电流命令写入 LowCmd，由 CAN 发送任务统一发出。
 */

#include "config.h"
#include "RobotTaskBuildConfig.h"

#if ROBOT_TASK_BUILD_CLASSIC_CHASSIS

#include "ChassisControlTask.h"
#include "ChassisBehaviour.h"
#include "ChassisCore.h"

#include "cmsis_os.h"

#include <math.h>

#include "arm_math.h"
#include "Pid.h"
#include "ManualInput.h"
#include "ControlInput.h"
#include "CanReceive.h"
#include "LowCmd.h"
#include "ChassisState.h"
#include "MotorInst.h"
#include "MotorConfig.h"
#include "Watch.h"
#include "DetectTask.h"
#include "InsTask.h"
#include "BspTime.h"
#include "kalman_filter.h"
#include "ChassisPowerControl.h"
#include "SdLog.h"
#include "RtProf.h"
#include "RobotTaskProfile.h"
#include "RobotMode.h"
#include "ControlMgr.h"

#include <string.h>

#define CHASSIS_MOTOR_COUNT 4U

static const char *const ChassisMotorInstNames[CHASSIS_MOTOR_COUNT] = {
    "motor.chassis0",
    "motor.chassis1",
    "motor.chassis2",
    "motor.chassis3",
};
static MotorCurrentBind ChassisMotorCurrentBindings[CHASSIS_MOTOR_COUNT];
static const int16_t ChassisZeroCurrentCmd[CHASSIS_MOTOR_COUNT] = {0};

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
    ManualInputState ManualInputCopy;
    const ManualInputState *manual_input;
    uint8_t GimbalStateValid;
    uint8_t GimbalOnline;
    GimbalMotorState yaw_motor;
    GimbalMotorState pitch_motor;
    const fp32 *ins_angle;
    const fp32 *gyro;
    const motor_measure_t *motor_measure[CHASSIS_MOTOR_COUNT];
    const motor_node_param_t *motor_cfg[CHASSIS_MOTOR_COUNT];
    uint8_t ChassisOnlyMode;
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
} ChassisRuntimeSnapshot;

static const fp32 *ChassisINTGyroPoint = NULL;
static kalman_2x1_t ChassisWzKf;
static bool_t ChassisWzKfInited = 0;

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

#include "ChassisSnapshotHelpers.inc"


/**
  * @brief          "ChassisMove" variable initialization, include pid initialization, remote control data point initialization, 3508 chassis motors
  *                 data point initialization, gimbal motor data point initialization, and gyro sensor angle point initialization.
  * @param[out]     ChassisMoveInit: "ChassisMove" variable point
  * @retval         none
  */
static void ChassisInit(ChassisMove *ChassisMoveInit);

/**
  * @brief          set chassis control mode, mainly call 'ChassisBehaviourModeSet' function
  * @param[out]     ChassisMoveMode: "ChassisMove" variable point
  * @retval         none
  */
static void ChassisSetMode(ChassisMove *ChassisMoveMode);

/**
  * @brief          when chassis mode change, some param should be changed, such as chassis yaw_set should be now chassis yaw
  * @param[out]     ChassisMoveTransit: "ChassisMove" variable point
  * @retval         none
  */
/**
  * @brief          底盘模式改变，有些参数需要改变，例如底盘控制yaw角度设定值应该变成当前底盘yaw角度
  * @param[out]     ChassisMoveTransit:"ChassisMove"变量指针.
  * @retval         none
  */
static void ChassisModeChangeControlTransit(ChassisMove *ChassisMoveTransit);
/**
  * @param[out]     ChassisMoveUpdate: "ChassisMove" variable point
  * @retval         none
  */
static void ChassisFeedbackUpdate(ChassisMove *ChassisMoveUpdate, const ChassisRuntimeSnapshot *snapshot);
/**
  * @brief          set chassis control set-point, three movement control value is set by "ChassisBehaviourControlSet".
  *
  * @param[out]     ChassisMoveUpdate: "ChassisMove" variable point
  * @retval         none
  */
/**
  * @brief
  * @param[out]     ChassisMoveUpdate:"ChassisMove"变量指针.
  * @retval         none
  */
static void ChassisSetControl(ChassisMove *ChassisMoveControl);
/**
  * @brief          control loop, according to control set-point, calculate motor current,
  *                 motor current will be sent to motor
  * @param[out]     ChassisMoveControlLoop: "ChassisMove" variable point
  * @retval         none
  */
/**
  * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
  * @param[out]     ChassisMoveControlLoop:"ChassisMove"变量指针.
  * @retval         none
  */
static void ChassisControlLoop(ChassisMove *ChassisMoveControlLoop,
                                 const ChassisRuntimeSnapshot *snapshot,
                                 int16_t pre_power_current[CHASSIS_MOTOR_COUNT]);

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t ChassisHighWater;
#endif

//底盘运动数据
static ChassisMove g_chassis;
volatile uint32_t ChassisLoopCounter = 0;

typedef struct
{
    uint32_t start_tick_ms;
    uint32_t last_tick_ms;
    uint32_t period_us;
    uint16_t sample_count;
    uint16_t sample_div_counter;
    sdlog_chassis_base_sample_t samples[CHASSIS_SDLOG_BASE_STREAM_MAX_SAMPLES];
} ChassisSdLogBaseStreamState;

static ChassisSdLogBaseStreamState s_chassis_sdlog_base_stream = {0};

#include "ChassisSdlogHelpers.inc"

#include "ChassisTuneState.inc"


/**
  * @brief          chassis task, osDelay CHASSIS_CONTROL_TIME_MS (2ms)
  * @param[in]      pvParameters: null
  * @retval         none
  */
void ChassisControlTask(void const *pvParameters)
{
    TickType_t last_wake = 0;

    // 函数地图：初始化并等遥控在线；循环里取快照、选模式、算电流、做保护、写日志和延时。
    //wait a time
    vTaskDelay(CHASSIS_TASK_INIT_TIME);
    //chassis init
    ChassisInit(&g_chassis);
    while (toe_is_error(DBUS_TOE))
    {
        WatchTaskWait(WATCH_TASK_CHASSIS_CONTROL);
        vTaskDelay(pdMS_TO_TICKS(RobotProfileChassisControlPeriodMs()));
    }

    last_wake = xTaskGetTickCount();
    while (1)
    {
        const uint64_t loop_start_us = RtProfBegin();
        ChassisRuntimeSnapshot snapshot;
        ChassisSnapshotCapture(&snapshot, &g_chassis);
        WatchTaskBeat(WATCH_TASK_CHASSIS_CONTROL);
        //set chassis control mode
        //设置底盘控制模式
        ChassisSetMode(&g_chassis);
        //when mode changes, some data save
        //模式切换数据保存
        ChassisModeChangeControlTransit(&g_chassis);
        //chassis data update
        //底盘数据更新
        ChassisFeedbackUpdate(&g_chassis, &snapshot);

        // test mode: only none/ChassisOnly allow normal chassis control
        if (!operation_mode_allow_chassis(&snapshot))
        {
            ChassisControlStopOutputs(&g_chassis);
            ChassisWriteState(&g_chassis);
            RtProfEnd(RtProfChassisLoop, loop_start_us);
            ChassisControlDelay(&last_wake, snapshot.period_ms);

#if INCLUDE_uxTaskGetStackHighWaterMark
            ChassisHighWater = uxTaskGetStackHighWaterMark(NULL);
#endif
            continue;
        }

        if (!ChassisControlMgrAllows(&snapshot))
        {
            ChassisControlStopOutputs(&g_chassis);
            ChassisWriteState(&g_chassis);
            RtProfEnd(RtProfChassisLoop, loop_start_us);
            ChassisControlDelay(&last_wake, snapshot.period_ms);

#if INCLUDE_uxTaskGetStackHighWaterMark
            ChassisHighWater = uxTaskGetStackHighWaterMark(NULL);
#endif
            continue;
        }

        //set chassis control set-point
        ChassisSetControl(&g_chassis);
        //chassis control pid calculate
        int16_t ChassisPrePowerCmd[CHASSIS_MOTOR_COUNT] = {0};
        ChassisControlLoop(&g_chassis, &snapshot, ChassisPrePowerCmd);

        // motor offline guard: if feedback is offline, clear PID and outputs to avoid runaway
        for (uint8_t i = 0; i < CHASSIS_MOTOR_COUNT; i++)
        {
            if (toe_is_error(CHASSIS_MOTOR1_TOE + i))
            {
                PID_clear(&g_chassis.motor_speed_pid[i]);
                g_chassis.motor_chassis[i].speed_set = 0.0f;
                g_chassis.motor_chassis[i].give_current = 0;
            }
        }

        // Update CAN1 chassis motor currents for CAN TX task
        int16_t ChassisCurrentCmd[CHASSIS_MOTOR_COUNT] = {0};
        if (g_chassis.fast.manual_online != 0u)
        {
            for (uint8_t i = 0; i < CHASSIS_MOTOR_COUNT; i++)
            {
                const int16_t current = g_chassis.motor_chassis[i].give_current;
                ChassisCurrentCmd[i] = MotorCfgLimitCurrentNode(snapshot.motor_cfg[i], current);
            }
        }

        (void)MotorInstSetCurrentBindsBestEffort(ChassisMotorCurrentBindings,
                                                                  ChassisCurrentCmd,
                                                                  CHASSIS_MOTOR_COUNT);

        ChassisLoopCounter++;
        {
            sdlog_chassis_base_sample_t sample = {0};
            const uint32_t now_ms = BspTimeGetTickMs();

            sample.ChassisMode = (uint8_t)g_chassis.mode;
            sample.last_chassis_mode = (uint8_t)g_chassis.last_mode;
            for (uint8_t i = 0; i < CHASSIS_MOTOR_COUNT; i++)
            {
                sample.wheel_rpm[i] = (g_chassis.motor_chassis[i].ChassisMotorMeasure != NULL) ?
                                          g_chassis.motor_chassis[i].ChassisMotorMeasure->speed_rpm :
                                          0;
                sample.current_request[i] = ChassisPrePowerCmd[i];
                sample.current_output[i] = ChassisCurrentCmd[i];
            }

            ChassisSdLogAppendBaseSample(&sample, now_ms, snapshot.period_us);
        }
        ChassisWriteState(&g_chassis);
        //os delay
        //系统延时
        RtProfEnd(RtProfChassisLoop, loop_start_us);
        ChassisControlDelay(&last_wake, snapshot.period_ms);

#if INCLUDE_uxTaskGetStackHighWaterMark
        ChassisHighWater = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

#include "ChassisCoreControl.inc"

#endif
