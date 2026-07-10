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

#include "RobotConfig.h"
#include "RobotTaskBuildConfig.h"

#if ROBOT_TASK_BUILD_CLASSIC_CHASSIS

#include "ChassisControlTask.h"
#include "ChassisBehaviour.h"
#include "ChassisCore.h"

#include "cmsis_os.h"

#include <math.h>

#include "arm_math.h"
#include "Pid.h"
#include "ManualInputSnapshot.h"
#include "ControlInput.h"
#include "CanReceive.h"
#include "LowCmd.h"
#include "FaultMgr.h"
#include "ChassisState.h"
#include "MotorInst.h"
#include "MotorAxisFaultPolicy.h"
#include "MotorHealth.h"
#include "MotorConfig.h"
#include "Watch.h"
#include "DetectTask.h"
#include "InsTask.h"
#include "BspTime.h"
#include "KalmanFilter.h"
#include "ChassisPowerControl.h"
#include "SdLog.h"
#include "RtProf.h"
#include "RobotTaskProfile.h"
#include "RobotMode.h"
#include "ChassisCtrl.h"
#include "ChassisRuntime.h"

#include <string.h>

#define CHASSIS_MOTOR_COUNT 4U
#define CHASSIS_STACK_SAMPLE_PERIOD_MS 1000u
#define CHASSIS_MOTOR_FEEDBACK_TIMEOUT_MS 50u
#define CHASSIS_FAULT_RECOVERY_MS 200u

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

#ifndef CHASSIS_GIMBAL_YAW_RELATIVE_TURN
#define CHASSIS_GIMBAL_YAW_RELATIVE_TURN YAW_TURN
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
    InsSnapshot ImuSnapshot;
    const ManualInputSnapshot *manual_input;
    ChassisGimbalSnapshot gimbal;
    const fp32 *ins_angle;
    const fp32 *gyro;
    motor_measure_t motor_measure[CHASSIS_MOTOR_COUNT];
    const motor_node_param_t *motor_cfg[CHASSIS_MOTOR_COUNT];
    uint8_t ChassisOnlyMode;
    uint16_t period_ms;
    uint32_t tick_ms;
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
    uint8_t manual_online;
    uint8_t recovery_input_safe;
    int8_t motor_dir[CHASSIS_MOTOR_COUNT];
    MotorHealthResult motor_health[CHASSIS_MOTOR_COUNT];
} ChassisRuntimeSnapshot;

static MotorState s_chassisMotorReadFeedback[CHASSIS_MOTOR_COUNT];

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
static void ChassisSetControl(ChassisMove *ChassisMoveControl,
                              const ChassisRuntimeSnapshot *snapshot);
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

static void ChassisStackSampleMaybe(void)
{
    static TickType_t last_sample_tick = 0u;
    static uint8_t sampled = 0u;
    const TickType_t now = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(CHASSIS_STACK_SAMPLE_PERIOD_MS);

    if (sampled != 0u && (TickType_t)(now - last_sample_tick) < period)
    {
        return;
    }

    ChassisHighWater = uxTaskGetStackHighWaterMark(NULL);
    last_sample_tick = now;
    sampled = 1u;
}
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

#include "ChassisFaultHelpers.inc"

#include "ChassisSdlogHelpers.inc"

#include "ChassisTuneState.inc"


static void ChassisRuntimeReadFrame(ChassisRuntimeSnapshot *snapshot,
                                    const ManualInputSnapshot *manualInput,
                                    uint32_t tickMs,
                                    uint16_t periodMs)
{
    ChassisSnapshotCapture(snapshot, &g_chassis, manualInput, tickMs, periodMs);
    ChassisFaultUpdate(snapshot);
    ChassisFaultSyncInhibit(&g_chassis);
    ChassisSetMode(&g_chassis);
    ChassisModeChangeControlTransit(&g_chassis);
    ChassisFeedbackUpdate(&g_chassis, snapshot);
}

static void ChassisRuntimePublishSafeFrame(void)
{
    ChassisControlClearOutputs(&g_chassis);
    ChassisWriteState(&g_chassis);
}

void ChassisRuntimeInit(void)
{
    ChassisInit(&g_chassis);
    ChassisFaultInit();
}

void ChassisRuntimeSafeStep(const ManualInputSnapshot *manualInput,
                            uint32_t tickMs,
                            uint16_t periodMs)
{
    ChassisRuntimeSnapshot snapshot;

    /* 安全帧仍刷新反馈、故障分组和逐轴禁写，只跳过控制量及功率计算。 */
    ChassisBehaviourInputGateBlock();
    ChassisRuntimeReadFrame(&snapshot, manualInput, tickMs, periodMs);
    ChassisRuntimePublishSafeFrame();
}

void ChassisRuntimeStep(const ManualInputSnapshot *manualInput,
                        uint32_t tickMs,
                        uint16_t periodMs,
                        int16_t motorCurrent[CHASSIS_MOTOR_COUNT])
{
    ChassisRuntimeSnapshot snapshot;
    int16_t ChassisPrePowerCmd[CHASSIS_MOTOR_COUNT] = {0};
    int16_t ChassisCurrentCmd[CHASSIS_MOTOR_COUNT] = {0};

    if (motorCurrent == NULL)
    {
        ChassisRuntimeStop();
        return;
    }

    ChassisRuntimeReadFrame(&snapshot, manualInput, tickMs, periodMs);
    if (snapshot.manual_online == 0u || robot_mode_allow_chassis() == 0u)
    {
        ChassisBehaviourInputGateBlock();
        ChassisRuntimePublishSafeFrame();
        for (uint8_t i = 0u; i < CHASSIS_MOTOR_COUNT; i++)
        {
            motorCurrent[i] = 0;
        }
        return;
    }

    ChassisSetControl(&g_chassis, &snapshot);
    ChassisControlLoop(&g_chassis, &snapshot, ChassisPrePowerCmd);
    ChassisFaultApplyControl(&g_chassis);

    if (g_chassis.fast.manual_online != 0u)
    {
        for (uint8_t i = 0u; i < CHASSIS_MOTOR_COUNT; i++)
        {
            const uint32_t bit = 1u << i;
            const int16_t current = g_chassis.motor_chassis[i].give_current;
            ChassisCurrentCmd[i] = ((s_chassisFault.configuredMask & bit) == 0u ||
                                    (s_chassisFault.holdZeroMask & bit) != 0u) ?
                                       0 :
                                       MotorCfgLimitCurrentNode(snapshot.motor_cfg[i], current);
        }
    }

    for (uint8_t i = 0u; i < CHASSIS_MOTOR_COUNT; i++)
    {
        motorCurrent[i] = ChassisCurrentCmd[i];
    }

    ChassisLoopCounter++;
    {
        sdlog_chassis_base_sample_t sample = {0};
        sample.ChassisMode = (uint8_t)g_chassis.mode;
        sample.last_chassis_mode = (uint8_t)g_chassis.last_mode;
        for (uint8_t i = 0u; i < CHASSIS_MOTOR_COUNT; i++)
        {
            sample.wheel_rpm[i] = (g_chassis.motor_chassis[i].measureValid != 0u) ?
                                      g_chassis.motor_chassis[i].measure.speed_rpm :
                                      0;
            sample.current_request[i] = ChassisPrePowerCmd[i];
            sample.current_output[i] = ChassisCurrentCmd[i];
        }

        ChassisSdLogAppendBaseSample(&sample, snapshot.tick_ms, snapshot.period_us);
    }
    ChassisWriteState(&g_chassis);
}

void ChassisRuntimeStop(void)
{
    ChassisBehaviourInputGateBlock();
    ChassisControlStopOutputs(&g_chassis);
    ChassisWriteState(&g_chassis);
}

static ControlResult ChassisTaskRunFrame(const ManualInputSnapshot *manualInput,
                                         uint8_t forceSafe)
{
    ChassisCtrlInput input = {
        .manualInput = manualInput,
        .tickMs = BspTimeGetTickMs(),
        .periodMs = RobotProfileChassisControlPeriodMs(),
        .forceSafe = forceSafe,
    };
    ChassisCtrlOutput output = {0};
    const ControlResult result = ChassisCtrlStep(&input, &output);

    /* ChassisCtrlOutput 是唯一正常输出边界；Runtime 只负责计算。 */
    (void)MotorInstSetCurrentBindsBestEffort(ChassisMotorCurrentBindings,
                                             output.motorCurrent,
                                             CHASSIS_MOTOR_COUNT);
    return result;
}

/**
  * @brief          chassis task, osDelay CHASSIS_CONTROL_TIME_MS (2ms)
  * @param[in]      pvParameters: null
  * @retval         none
  */
void ChassisControlTask(void const *pvParameters)
{
    TickType_t last_wake;

    (void)pvParameters;

    // 函数地图：每帧只经过 ChassisCtrl；掉线和运行模式限制走安全帧，不退出控制域。
    vTaskDelay(CHASSIS_TASK_INIT_TIME);
    ChassisCtrlPrepare();
    last_wake = xTaskGetTickCount();
    while (1)
    {
        const uint64_t loop_start_us = RtProfBegin();
        ManualInputSnapshot manualInput;
        const ManualInputSnapshot *frameInput =
            (ManualInputSnapshotRead(&manualInput) != 0u) ? &manualInput : NULL;
        const uint8_t manualOffline =
            (uint8_t)(frameInput == NULL || frameInput->online == 0u);
        const uint8_t forceSafe = (uint8_t)(manualOffline != 0u ||
                                            robot_mode_allow_chassis() == 0u);

        if (manualOffline != 0u)
        {
            WatchTaskWait(WATCH_TASK_CHASSIS_CONTROL);
        }
        else
        {
            WatchTaskBeat(WATCH_TASK_CHASSIS_CONTROL);
        }

        (void)ChassisTaskRunFrame(frameInput, forceSafe);
        RtProfEnd(RtProfChassisLoop, loop_start_us);
        ChassisControlDelay(&last_wake, RobotProfileChassisControlPeriodMs());

#if INCLUDE_uxTaskGetStackHighWaterMark
        ChassisStackSampleMaybe();
#endif
    }
}

#include "ChassisCoreControl.inc"

#endif
