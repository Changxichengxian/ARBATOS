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
 * - 入口：GimbalControlTask() 按任务周期运行，最后把电流写入 LowCmd。
 */

#include "RobotConfig.h"
#include "RobotTaskBuildConfig.h"

#if ROBOT_TASK_BUILD_ANY_GIMBAL

#include "GimbalControlTask.h"
#include "GimbalCore.h"

#include "cmsis_os.h"

#include "arm_math.h"
#include "CanReceive.h"
#include "LowCmd.h"
#include "FaultMgr.h"
#include "MotorInst.h"
#include "MotorAxisFaultPolicy.h"
#include "MotorHealth.h"
#include "MotorConfig.h"
#include "UserLib.h"
#include "AxisCurrentConditioner.h"
#include "DetectTask.h"
#include "ManualInput.h"
#include "ControlInput.h"
#include "ChassisState.h"
#include "GimbalState.h"
#include "GimbalFaultPolicy.h"
#include "GimbalBehaviour.h"
#include "InsTask.h"
#include "Shoot.h"
#include "Pid.h"
#include "HostLinkTask.h"
#include "Watch.h"
#include "SdLog.h"
#include "RobotMode.h"
#include "PitchCali.h"
#include "BspTime.h"
#include "RtProf.h"
#include "RobotTaskProfile.h"
#include "ControlMgr.h"
#include "RobotLifecycle.h"

#include <string.h>

#define GIMBAL_OUTPUT_MOTOR_COUNT 3u
#define DUAL_YAW_GIMBAL_OUTPUT_MOTOR_COUNT 4u
#define GIMBAL_STACK_SAMPLE_PERIOD_MS 1000u
/* 与现有 Detect 默认值一致；高频控制循环不再反复读取诊断配置表。 */
#define GIMBAL_MOTOR_FEEDBACK_TIMEOUT_MS 50u
#define GIMBAL_IMU_SNAPSHOT_TIMEOUT_MS 10u
#define GIMBAL_FAULT_RECOVERY_MS 200u

static const char *const GimbalOutputMotorNames[GIMBAL_OUTPUT_MOTOR_COUNT] = {
    "motor.trigger",
    "motor.yaw",
    "motor.pitch",
};
static MotorCurrentBind GimbalOutputCurrentBindings[GIMBAL_OUTPUT_MOTOR_COUNT];

static const char *const DualYawGimbalOutputMotorNames[DUAL_YAW_GIMBAL_OUTPUT_MOTOR_COUNT] = {
    "motor.trigger",
    "motor.yaw",
    "motor.yaw_upper",
    "motor.pitch",
};
static MotorCurrentBind DualYawGimbalOutputCurrentBindings[DUAL_YAW_GIMBAL_OUTPUT_MOTOR_COUNT];

__weak void ShootInit(void)
{
}

__weak int16_t ShootControlLoop(void)
{
    return 0;
}

__weak void ShootStopOutputs(void)
{
}

typedef struct
{
    ManualInputState ManualInputCopy;
    ControlInputState ControlInputCopy;
    InsSnapshot ImuSnapshot;
    const ManualInputState *manual_input;
    const fp32 *gyro;
    const fp32 *ins_angle;
    const motor_measure_t *yaw_measure;
    const motor_measure_t *pitch_measure;
    const motor_node_param_t *yaw_motor_cfg;
    const motor_node_param_t *pitch_motor_cfg;
    const motor_node_param_t *trigger_motor_cfg;
    robot_run_variant_e run_variant;
    uint8_t PitchCaliMode;
    uint16_t period_ms;
    uint32_t period_us;
    uint8_t yaw_turn;
    uint8_t pitch_turn;
    uint8_t yaw_control_is_upper;
    uint8_t imu_online;
    uint8_t yaw_configured;
    uint8_t yaw_upper_configured;
    uint8_t pitch_configured;
    uint8_t control_input_valid;
    uint8_t manual_online;
    uint8_t recovery_input_safe;
    uint32_t imu_required_mask;
    MotorHealthResult yaw_health;
    MotorHealthResult yaw_upper_health;
    MotorHealthResult pitch_health;
} GimbalRuntimeSnapshot;

//motor encoder value format, range[0, ECD_RANGE - 1]
#define ecd_format(ecd)         \
    {                           \
        if ((ecd) >= ECD_RANGE) \
            (ecd) -= ECD_RANGE; \
        else if ((ecd) < 0)     \
            (ecd) += ECD_RANGE; \
    }

#define GimbalTotalPidClear(GimbalClear)                             \
    {                                                                    \
        GimbalPidClear(&(GimbalClear)->GimbalYawMotor.GimbalMotorAnglePid);   \
        PID_clear(&(GimbalClear)->GimbalYawMotor.GimbalMotorGyroPid);            \
                                                                           \
        GimbalPidClear(&(GimbalClear)->GimbalPitchMotor.GimbalMotorAnglePid); \
        PID_clear(&(GimbalClear)->GimbalPitchMotor.GimbalMotorGyroPid);          \
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

#ifndef GIMBAL_DUAL_YAW_IMU_TURN
#define GIMBAL_DUAL_YAW_IMU_TURN YAW_TURN
#endif

#ifndef GIMBAL_SINGLE_MIT_TEST_MAX_TORQUE_NM
#define GIMBAL_SINGLE_MIT_TEST_MAX_TORQUE_NM 4.0f
#endif

#ifndef GIMBAL_SINGLE_MIT_TEST_DAMPING_KD
#define GIMBAL_SINGLE_MIT_TEST_DAMPING_KD 0.2f
#endif

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t GimbalHighWater;

static void GimbalStackSampleMaybe(void)
{
    static TickType_t last_sample_tick = 0u;
    static uint8_t sampled = 0u;
    const TickType_t now = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(GIMBAL_STACK_SAMPLE_PERIOD_MS);

    if (sampled != 0u && (TickType_t)(now - last_sample_tick) < period)
    {
        return;
    }

    GimbalHighWater = uxTaskGetStackHighWaterMark(NULL);
    last_sample_tick = now;
    sampled = 1u;
}
#endif

/**
  * @brief          "GimbalControl" variable initialization, include pid initialization, remote control data point initialization, gimbal motors
  *                 data point initialization, and gyro sensor angle point initialization.
  * @param[out]     init: "GimbalControl" variable point
  * @retval         none
  */
static void GimbalInit(GimbalControl *init);

/**
  * @brief          set gimbal control mode, mainly call 'GimbalBehaviourModeSet' function
  * @param[out]     GimbalSetMode: "GimbalControl" variable point
  * @retval         none
  */
static void GimbalSetMode(GimbalControl *set_mode);
/**
  * @brief          gimbal some measure data update, such as motor encoder, euler angle, gyro
  * @param[out]     GimbalFeedbackUpdate: "GimbalControl" variable point
  * @retval         none
  */
/**
  * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
  * @param[out]     GimbalFeedbackUpdate:"GimbalControl"变量指针.
  * @retval         none
  */
static void GimbalSnapshotCapture(GimbalRuntimeSnapshot *snapshot, GimbalControl *control);
static void GimbalFeedbackUpdate(GimbalControl *feedback_update, const GimbalRuntimeSnapshot *snapshot);
static void GimbalDualYawImuCenterUpdateOnOutput(GimbalControl *control,
                                                    const GimbalRuntimeSnapshot *snapshot,
                                                    uint8_t output_allowed);

/**
  * @brief          when gimbal mode change, some param should be changed, such as  yaw_set should be new yaw
  * @param[out]     mode_change: "GimbalControl" variable point
  * @retval         none
  */
/**
  * @brief          云台模式改变，有些参数需要改变，例如控制yaw角度设定值应该变成当前yaw角度
  * @param[out]     mode_change:"GimbalControl"变量指针.
  * @retval         none
  */
static void GimbalModeChangeControlTransit(GimbalControl *mode_change);

/**
  * @brief          calculate the encoder angle between ecd and offset_ecd
  * @param[in]      ecd: motor now encode
  * @param[in]      offset_ecd: gimbal offset encode
  * @retval         angle, unit rad
  */
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);
#if GIMBAL_USE_ENCODER_FEEDBACK
static void GimbalFeedbackFromEncoder(GimbalMotor *motor,
                                         const motor_measure_t *measure,
                                         uint8_t turn);
#endif
/**
  * @brief          set gimbal control set-point, control set-point is set by "GimbalBehaviourControlSet".
  * @param[out]     GimbalSetControl: "GimbalControl" variable point
  * @retval         none
  */
static void GimbalSetControl(GimbalControl *set_control);

static void GimbalAngleLimit(GimbalMotor *GimbalMotor, fp32 add);
static fp32 GimbalYawChassisSpinFfCurrent(const GimbalMotor *yaw_motor);
static fp32 GimbalYawKickCurrent(const GimbalMotor *yaw_motor);
static void GimbalWriteState(const GimbalRuntimeSnapshot *snapshot);

/**
  * @brief          gimbal control mode :GIMBAL_MOTOR_ENCODER, use the encoder angle to control.
  * @param[out]     GimbalMotor: yaw motor or pitch motor
  * @retval         none
  */
/**
  * @brief          云台控制模式:GIMBAL_MOTOR_ENCODER，使用编码角进行控制
  * @param[out]     GimbalMotor:yaw电机或者pitch电机
  * @retval         none
  */
static void GimbalMotorAngleControl(GimbalMotor *GimbalMotor);

static void GimbalMotorRawAngleControl(GimbalMotor *GimbalMotor);

static void GimbalControlLoop(GimbalControl *control_loop);
static int16_t GimbalApplyOutputTurn(int16_t current, uint8_t turn);
/**
  * @brief          limit angle set in encoder angle mode, avoid exceeding the max angle
  * @param[out]     GimbalMotor: yaw motor or pitch motor
  * @retval         none
  */
/**
  * @brief          limit angle set in GIMBAL_MOTOR_ENCODER mode, avoid exceeding the max angle
  * @param[out]     GimbalMotor: yaw motor or pitch motor
  * @retval         none
  */

/**
  * @brief          gimbal angle pid init, because angle is in range(-pi,pi),can't use PID in Pid.c
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
  * @param[out]     PidClear: "GimbalControl" variable point
  * @retval         none
  */
/**
  * @brief          gimbal angle pid calc, because angle is in range(-pi,pi),can't use PID in Pid.c
  * @param[out]     pid: pid data pointer stucture
  * @param[in]      get: angle feedback
  * @param[in]      set: angle set-point
  * @param[in]      error_delta: rotation speed
  * @retval         pid out
  */

/**
  * @brief          gimbal calibration calculate
  * @param[in]      GimbalCali: cali data
  * @param[out]     yaw_offset:yaw motor middle place encode
  * @param[out]     pitch_offset:pitch motor middle place encode
  * @param[out]     max_yaw:yaw motor max machine angle
  * @param[out]     min_yaw: yaw motor min machine angle
  * @param[out]     max_pitch: pitch motor max machine angle
  * @param[out]     min_pitch: pitch motor min machine angle
  * @retval         none
  */
static void GimbalCalcCali(const GimbalStepCali *GimbalCali, uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch);

#if GIMBAL_TEST_MODE
//j-scope 帮助pid调参
static void J_scope_gimbal_test(void);
static fp32 GimbalPitchKickScale(fp32 angle_err);
#endif

//gimbal control data
static GimbalControl g_gimbal;
static uint8_t s_dual_yaw_imu_center_ready = 0u;
static fp32 s_dual_yaw_imu_center = 0.0f;
static uint8_t s_dual_yaw_output_allowed_prev = 0u;
volatile uint32_t GimbalLoopCounter = 0;

//motor current
//发送的电机电流
static int16_t yaw_can_set_current = 0, pitch_can_set_current = 0, ShootCanSetCurrent = 0;
volatile int16_t GimbalWatchYawCurrent = 0;
volatile int16_t GimbalWatchYawUpperCurrent = 0;
volatile int16_t GimbalWatchPitchCurrent = 0;
volatile int16_t GimbalYawEasytestCurrent = 3000;

typedef struct
{
    uint32_t start_tick_ms;
    uint32_t last_tick_ms;
    uint32_t period_us;
    uint16_t sample_count;
    uint16_t sample_div_counter;
    sdlog_gimbal_base_sample_t samples[GIMBAL_SDLOG_BASE_STREAM_MAX_SAMPLES];
} GimbalSdLogBaseStreamState;

static GimbalSdLogBaseStreamState s_gimbal_sdlog_base_stream = {0};

#include "GimbalRuntimeHelpers.inc"

#include "GimbalFaultHelpers.inc"

#include "GimbalSdlogHelpers.inc"

#include "GimbalDualYawHelpers.inc"


/**
  * @brief          gimbal task, osDelay GIMBAL_CONTROL_TIME (1ms)
  * @param[in]      pvParameters: null
  * @retval         none
  */

void GimbalControlTask(void const *pvParameters)
{
    TickType_t last_wake = 0;
    //等待陀螺仪任务更新陀螺仪数据
    //wait a time
    vTaskDelay(GIMBAL_TASK_INIT_TIME);
    //gimbal init
    GimbalInit(&g_gimbal);
    GimbalFaultInit();
    GimbalWriteState(NULL);
    //shoot init
    ShootInit();
    PitchCaliBootLoad();
    last_wake = xTaskGetTickCount();
    while (1)
    {
        const uint64_t loop_start_us = RtProfBegin();
        GimbalRuntimeSnapshot snapshot;
        WatchTaskBeat(WATCH_TASK_GIMBAL_CONTROL);
        GimbalSnapshotCapture(&snapshot, &g_gimbal);
        GimbalFaultUpdate(&snapshot);
        GimbalFaultSyncInhibit(&g_gimbal, &snapshot);
        GimbalLoopCounter++;
        if (!GimbalControlMgrAllows(ControlIdSingleGimbal, &snapshot))
        {
            GimbalStopOutputs(GimbalOutputCurrentBindings, GIMBAL_OUTPUT_MOTOR_COUNT, &snapshot);
            RtProfEnd(RtProfGimbalLoop, loop_start_us);
            GimbalControlDelay(&last_wake, snapshot.period_ms);

#if INCLUDE_uxTaskGetStackHighWaterMark
            GimbalStackSampleMaybe();
#endif
            continue;
        }

        GimbalSetMode(&g_gimbal);                    //设置云台控制模式
        PitchCaliTickPre(&g_gimbal, GimbalBehaviourWatch, snapshot.PitchCaliMode);
        GimbalModeChangeControlTransit(&g_gimbal); //控制模式切换 控制数据过渡
        GimbalFeedbackUpdate(&g_gimbal, &snapshot);  //云台数据反馈
        GimbalSetControl(&g_gimbal);
        GimbalControlLoop(&g_gimbal);
        PitchCaliTickPost(&g_gimbal, GimbalBehaviourWatch, snapshot.PitchCaliMode);
        GimbalApplyHealthToControl(&g_gimbal, &snapshot);
        GimbalFaultApplyControl(&g_gimbal, &snapshot);
        GimbalWriteState(&snapshot);
        ShootCanSetCurrent = GimbalRunShootControl(&snapshot); // 拨盘电流
        yaw_can_set_current = GimbalApplyOutputTurn(g_gimbal.GimbalYawMotor.given_current, snapshot.yaw_turn);
        pitch_can_set_current = GimbalApplyOutputTurn(g_gimbal.GimbalPitchMotor.given_current, snapshot.pitch_turn);

        const int16_t yaw_current_request = yaw_can_set_current;
        const int16_t pitch_current_request = pitch_can_set_current;

        GimbalApplyOperationMode(&snapshot, &yaw_can_set_current, &pitch_can_set_current);
        GimbalApplyOutputHealth(&snapshot, &yaw_can_set_current, NULL, &pitch_can_set_current);
        GimbalFaultApplyOutput(&yaw_can_set_current, NULL, &pitch_can_set_current);

        yaw_can_set_current = MotorCfgLimitCurrentNode(snapshot.yaw_motor_cfg, yaw_can_set_current);
        pitch_can_set_current = MotorCfgLimitCurrentNode(snapshot.pitch_motor_cfg, pitch_can_set_current);
        ShootCanSetCurrent = MotorCfgLimitCurrentNode(snapshot.trigger_motor_cfg, ShootCanSetCurrent);

        // watch 输出：观察最终下发电流（含运行模式、安全模式及方向翻转后的值）
        GimbalWatchYawCurrent = yaw_can_set_current;
        GimbalWatchPitchCurrent = pitch_can_set_current;
        if (GimbalSingleMitYawTestActive(&snapshot) != 0u)
        {
            ShootCanSetCurrent = 0;
            GimbalWatchYawCurrent = 0;
            GimbalWatchPitchCurrent = 0;
            GimbalApplySingleMitYawTest(&snapshot);
        }
        else
        {
            const int16_t GimbalCurrentCmd[] = {
                ShootCanSetCurrent,
                yaw_can_set_current,
                pitch_can_set_current,
            };

            (void)MotorInstSetCurrentBindsBestEffort(GimbalOutputCurrentBindings,
                                                                      GimbalCurrentCmd,
                                                                      GIMBAL_OUTPUT_MOTOR_COUNT);
        }

        {
            sdlog_gimbal_base_sample_t sample = {0};

            sample.GimbalBehaviour = (uint8_t)GimbalBehaviourWatch;
            sample.test_mode = (uint8_t)snapshot.run_variant;
            sample.yaw_motor_mode = (uint8_t)g_gimbal.GimbalYawMotor.mode;
            sample.pitch_motor_mode = (uint8_t)g_gimbal.GimbalPitchMotor.mode;
            sample.yaw_angle = g_gimbal.GimbalYawMotor.angle;
            sample.pitch_angle = g_gimbal.GimbalPitchMotor.angle;
            sample.yaw_gyro = g_gimbal.GimbalYawMotor.motor_gyro;
            sample.pitch_gyro = g_gimbal.GimbalPitchMotor.motor_gyro;
            sample.yaw_current_request = yaw_current_request;
            sample.pitch_current_request = pitch_current_request;
            sample.yaw_current_output = GimbalSdLogClampCurrent(yaw_can_set_current);
            sample.pitch_current_output = GimbalSdLogClampCurrent(pitch_can_set_current);

            GimbalSdLogAppendBaseSample(&sample, BspTimeGetTickMs(), snapshot.period_us);
        }

#if GIMBAL_TEST_MODE
        J_scope_gimbal_test();
#endif

        RtProfEnd(RtProfGimbalLoop, loop_start_us);
        GimbalControlDelay(&last_wake, snapshot.period_ms);

#if INCLUDE_uxTaskGetStackHighWaterMark
        GimbalStackSampleMaybe();
#endif
    }
}

void DualYawGimbalControlTask(void const *pvParameters)
{
    (void)pvParameters;
    TickType_t last_wake = 0;

    vTaskDelay(GIMBAL_TASK_INIT_TIME);
    GimbalInit(&g_gimbal);
    GimbalFaultInit();
    GimbalWriteState(NULL);
    ShootInit();

    last_wake = xTaskGetTickCount();
    while (1)
    {
        const uint64_t loop_start_us = RtProfBegin();
        GimbalRuntimeSnapshot snapshot;

        WatchTaskBeat(WATCH_TASK_GIMBAL_CONTROL);
        GimbalSnapshotCapture(&snapshot, &g_gimbal);
        GimbalFaultUpdate(&snapshot);
        GimbalFaultSyncInhibit(&g_gimbal, &snapshot);
        const uint8_t output_allowed = RobotLifecycleOutputAllowed();
        GimbalLoopCounter++;
        if (!GimbalControlMgrAllows(ControlIdDualYawGimbal, &snapshot))
        {
            if (output_allowed == 0u)
            {
                GimbalDualYawImuCenterUpdateOnOutput(&g_gimbal, &snapshot, 0u);
            }
            GimbalStopOutputs(DualYawGimbalOutputCurrentBindings,
                              DUAL_YAW_GIMBAL_OUTPUT_MOTOR_COUNT,
                              &snapshot);
            RtProfEnd(RtProfGimbalLoop, loop_start_us);
            GimbalControlDelay(&last_wake, snapshot.period_ms);

#if INCLUDE_uxTaskGetStackHighWaterMark
            GimbalStackSampleMaybe();
#endif
            continue;
        }

        GimbalSetMode(&g_gimbal);
        GimbalModeChangeControlTransit(&g_gimbal);
        GimbalFeedbackUpdate(&g_gimbal, &snapshot);
        GimbalDualYawImuCenterUpdateOnOutput(&g_gimbal, &snapshot, output_allowed);
        GimbalSetControl(&g_gimbal);
        GimbalControlLoop(&g_gimbal);
        GimbalApplyHealthToControl(&g_gimbal, &snapshot);
        GimbalFaultApplyControl(&g_gimbal, &snapshot);
        GimbalWriteState(&snapshot);

        ShootCanSetCurrent = GimbalRunShootControl(&snapshot);
        yaw_can_set_current = GimbalApplyOutputTurn(g_gimbal.GimbalYawMotor.given_current, snapshot.yaw_turn);
        pitch_can_set_current = GimbalApplyOutputTurn(g_gimbal.GimbalPitchMotor.given_current, snapshot.pitch_turn);

        const int16_t yaw_current_request = yaw_can_set_current;
        const int16_t pitch_current_request = pitch_can_set_current;

        GimbalApplyOperationMode(&snapshot, &yaw_can_set_current, &pitch_can_set_current);

        int16_t yaw_upper_can_set_current = GimbalDualYawUpperControlCurrent(&snapshot, yaw_can_set_current);
        int16_t yaw_log_current_output;

        if (snapshot.yaw_control_is_upper != 0u)
        {
            yaw_upper_can_set_current = yaw_can_set_current;
            yaw_can_set_current = 0;
        }

        GimbalApplyOutputHealth(&snapshot,
                                &yaw_can_set_current,
                                &yaw_upper_can_set_current,
                                &pitch_can_set_current);
        GimbalFaultApplyOutput(&yaw_can_set_current,
                               &yaw_upper_can_set_current,
                               &pitch_can_set_current);

        yaw_can_set_current = MotorCfgLimitCurrentNode(&g_config.motor.yaw, yaw_can_set_current);
        yaw_upper_can_set_current = MotorCfgLimitCurrentNode(&g_config.motor.yaw_upper, yaw_upper_can_set_current);
        pitch_can_set_current = MotorCfgLimitCurrentNode(snapshot.pitch_motor_cfg, pitch_can_set_current);
        ShootCanSetCurrent = MotorCfgLimitCurrentNode(snapshot.trigger_motor_cfg, ShootCanSetCurrent);
        yaw_log_current_output = (snapshot.yaw_control_is_upper != 0u) ? yaw_upper_can_set_current : yaw_can_set_current;

        GimbalWatchYawCurrent = yaw_can_set_current;
        GimbalWatchYawUpperCurrent = yaw_upper_can_set_current;
        GimbalWatchPitchCurrent = pitch_can_set_current;
        if (GimbalSingleMitYawTestActive(&snapshot) != 0u)
        {
            ShootCanSetCurrent = 0;
            GimbalWatchYawCurrent = 0;
            GimbalWatchYawUpperCurrent = 0;
            GimbalWatchPitchCurrent = 0;
            GimbalApplySingleMitYawTest(&snapshot);
        }
        else
        {
            const int16_t GimbalCurrentCmd[] = {
                ShootCanSetCurrent,
                yaw_can_set_current,
                yaw_upper_can_set_current,
                pitch_can_set_current,
            };

            (void)MotorInstSetCurrentBindsBestEffort(DualYawGimbalOutputCurrentBindings,
                                                                      GimbalCurrentCmd,
                                                                      DUAL_YAW_GIMBAL_OUTPUT_MOTOR_COUNT);
        }

        {
            sdlog_gimbal_base_sample_t sample = {0};

            sample.GimbalBehaviour = (uint8_t)GimbalBehaviourWatch;
            sample.test_mode = (uint8_t)snapshot.run_variant;
            sample.yaw_motor_mode = (uint8_t)g_gimbal.GimbalYawMotor.mode;
            sample.pitch_motor_mode = (uint8_t)g_gimbal.GimbalPitchMotor.mode;
            sample.yaw_angle = g_gimbal.GimbalYawMotor.angle;
            sample.pitch_angle = g_gimbal.GimbalPitchMotor.angle;
            sample.yaw_gyro = g_gimbal.GimbalYawMotor.motor_gyro;
            sample.pitch_gyro = g_gimbal.GimbalPitchMotor.motor_gyro;
            sample.yaw_current_request = yaw_current_request;
            sample.pitch_current_request = pitch_current_request;
            sample.yaw_current_output = GimbalSdLogClampCurrent(yaw_log_current_output);
            sample.pitch_current_output = GimbalSdLogClampCurrent(pitch_can_set_current);

            GimbalSdLogAppendBaseSample(&sample, BspTimeGetTickMs(), snapshot.period_us);
        }

        RtProfEnd(RtProfGimbalLoop, loop_start_us);
        GimbalControlDelay(&last_wake, snapshot.period_ms);

#if INCLUDE_uxTaskGetStackHighWaterMark
        GimbalStackSampleMaybe();
#endif
    }
}

#include "GimbalCaliHelpers.inc"

#include "GimbalCoreControl.inc"

#include "GimbalTuneApi.inc"

#endif
