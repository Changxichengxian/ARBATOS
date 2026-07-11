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
#include "ManualInputSnapshot.h"
#include "ControlInput.h"
#include "ChassisState.h"
#include "GimbalState.h"
#include "GimbalFaultPolicy.h"
#include "GimbalFeedbackPolicy.h"
#include "GimbalBehaviour.h"
#include "InsTask.h"
#include "ShootCtrl.h"
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

#define GIMBAL_OUTPUT_MOTOR_COUNT 2u
#define DUAL_YAW_GIMBAL_OUTPUT_MOTOR_COUNT 3u
#define GIMBAL_STACK_SAMPLE_PERIOD_MS 1000u
/* 与现有 Detect 默认值一致；高频控制循环不再反复读取诊断配置表。 */
#define GIMBAL_MOTOR_FEEDBACK_TIMEOUT_MS 50u
#define GIMBAL_IMU_SNAPSHOT_TIMEOUT_MS 10u
#define GIMBAL_FAULT_RECOVERY_MS 200u

static const char *const GimbalOutputMotorNames[GIMBAL_OUTPUT_MOTOR_COUNT] = {
    "motor.yaw",
    "motor.pitch",
};
static MotorCurrentBind GimbalOutputCurrentBindings[GIMBAL_OUTPUT_MOTOR_COUNT];
static const uint32_t GimbalOutputAxisMasks[GIMBAL_OUTPUT_MOTOR_COUNT] = {
    GIMBAL_FAULT_MASK_YAW,
    GIMBAL_FAULT_MASK_PITCH,
};

static const char *const DualYawGimbalOutputMotorNames[DUAL_YAW_GIMBAL_OUTPUT_MOTOR_COUNT] = {
    "motor.yaw",
    "motor.yaw_upper",
    "motor.pitch",
};
static MotorCurrentBind DualYawGimbalOutputCurrentBindings[DUAL_YAW_GIMBAL_OUTPUT_MOTOR_COUNT];
static const uint32_t DualYawGimbalOutputAxisMasks[DUAL_YAW_GIMBAL_OUTPUT_MOTOR_COUNT] = {
    GIMBAL_FAULT_MASK_YAW,
    GIMBAL_FAULT_MASK_YAW_UPPER,
    GIMBAL_FAULT_MASK_PITCH,
};

typedef struct
{
    InsSnapshot ImuSnapshot;
    const ManualInputSnapshot *manual_input;
    const fp32 *gyro;
    const fp32 *ins_angle;
    const motor_measure_t *yaw_measure;
    const motor_measure_t *pitch_measure;
    const motor_node_param_t *yaw_motor_cfg;
    const motor_node_param_t *pitch_motor_cfg;
    robot_run_mode_e run_mode;
    robot_run_variant_e run_variant;
    MotorId target_motor;
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
    uint8_t manual_online;
    uint8_t recovery_input_safe;
    uint8_t control_allowed;
    uint32_t imu_required_mask;
    uint32_t encoder_fallback_mask;
    uint32_t encoder_degraded_mask;
    uint32_t feedback_block_mask;
    uint32_t feedback_transition_zero_mask;
    uint32_t feedback_transition_seq;
    uint8_t feedback_mode;
    uint8_t feedback_recovery_pending;
    uint8_t feedback_transition_pending;
    MotorHealthResult yaw_health;
    MotorHealthResult yaw_upper_health;
    MotorHealthResult pitch_health;
} GimbalRuntimeSnapshot;

typedef struct
{
    MotorId ids[DUAL_YAW_GIMBAL_OUTPUT_MOTOR_COUNT];
    uint32_t axisMasks[DUAL_YAW_GIMBAL_OUTPUT_MOTOR_COUNT];
    uint8_t count;
} GimbalPublishResult;

typedef enum
{
    GimbalFeedbackZeroWaitNone = 0u,
    GimbalFeedbackZeroWaiting,
    GimbalFeedbackZeroReady,
} GimbalFeedbackZeroWait;

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
static void GimbalSnapshotCapture(GimbalRuntimeSnapshot *snapshot,
                                  GimbalControl *control,
                                  const ManualInputSnapshot *manual_input);
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
static fp32 motor_ecd_to_angle_change(uint16_t ecd,
                                      uint16_t offset_ecd,
                                      const motor_node_param_t *node);
static void GimbalFeedbackFromEncoder(GimbalMotor *motor,
                                      const motor_measure_t *measure,
                                      const motor_node_param_t *node,
                                      uint8_t turn);
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
static GimbalFeedbackPolicyState s_gimbalFeedbackPolicy;
static GimbalFeedbackZeroBarrier s_gimbalFeedbackZeroBarrier;
static GimbalFeedbackRouteDebt s_gimbalFeedbackRouteDebt;
static uint8_t s_gimbalSingleMitDesiredPrev;
static MotorId s_gimbalSingleMitTargetPrev;
static uint8_t s_dual_yaw_imu_center_ready = 0u;
static fp32 s_dual_yaw_imu_center = 0.0f;
static uint8_t s_dual_yaw_output_allowed_prev = 0u;
volatile uint32_t GimbalLoopCounter = 0;

//motor current
//发送的电机电流
static int16_t yaw_can_set_current = 0, pitch_can_set_current = 0;
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

static uint32_t s_gimbalControlInhibitMask;

static uint32_t GimbalMotorBit(MotorId id)
{
    return ((uint32_t)id < (uint32_t)MotorCount) ? (1ul << (uint32_t)id) : 0u;
}

static uint8_t GimbalFaultInhibitsMotor(MotorId id)
{
    for (uint8_t axis = 0u; axis < GIMBAL_FAULT_AXIS_COUNT; axis++)
    {
        if ((s_gimbalFault.inhibitMask & (1ul << axis)) != 0u &&
            GimbalFaultAxisId(axis) == id)
        {
            return 1u;
        }
    }
    return 0u;
}

static void GimbalControlInhibitOutputs(const MotorCurrentBind *bindings,
                                        uint8_t count)
{
    if (bindings == NULL)
    {
        return;
    }
    for (uint8_t i = 0u; i < count; i++)
    {
        const MotorId id = bindings[i].actuator_id;
        const uint32_t bit = GimbalMotorBit(id);
        uint16_t writer = (uint16_t)LOWCMD_WRITER_NONE;

        if (bindings[i].enabled == 0u || bit == 0u ||
            (s_gimbalControlInhibitMask & bit) != 0u ||
            LowCmdGetInhibitWriter(id, &writer) == 0u)
        {
            continue;
        }
        if (writer >= (uint16_t)LOWCMD_WRITER_SAFETY)
        {
            continue;
        }
        if (LowCmdInhibitManyFrom(&id, 1u, (uint16_t)LOWCMD_WRITER_SAFETY) != 0u)
        {
            s_gimbalControlInhibitMask |= bit;
        }
    }
}

static uint8_t GimbalControlReleaseOutputs(const MotorCurrentBind *bindings,
                                           uint8_t bindingCount,
                                           const ControlOutputPermit *permit)
{
    MotorId ids[DUAL_YAW_GIMBAL_OUTPUT_MOTOR_COUNT];
    uint32_t requiredMask = 0u;
    uint8_t count = 0u;

    if (bindings == NULL || bindingCount > DUAL_YAW_GIMBAL_OUTPUT_MOTOR_COUNT)
    {
        return 0u;
    }
    for (uint8_t i = 0u; i < bindingCount; i++)
    {
        const MotorId id = bindings[i].actuator_id;
        const uint32_t bit = GimbalMotorBit(id);
        uint16_t writer = (uint16_t)LOWCMD_WRITER_NONE;

        if (bindings[i].enabled == 0u || bit == 0u ||
            (s_gimbalControlInhibitMask & bit) == 0u ||
            GimbalFaultInhibitsMotor(id) != 0u ||
            LowCmdGetInhibitWriter(id, &writer) == 0u)
        {
            continue;
        }
        if (writer == (uint16_t)LOWCMD_WRITER_NONE)
        {
            s_gimbalControlInhibitMask &= ~bit;
            continue;
        }
        if (writer != (uint16_t)LOWCMD_WRITER_SAFETY)
        {
            continue;
        }
        ids[count++] = id;
        requiredMask |= bit;
    }

    if (count == 0u)
    {
        return 1u;
    }
    if (LowCmdRecoverSafetyInhibitManyWithPermit(ids, count, permit) != 0u)
    {
        s_gimbalControlInhibitMask &= ~requiredMask;
        return 1u;
    }

    {
        uint32_t recoveredMask = 0u;

        for (uint8_t i = 0u; i < count; i++)
        {
            const uint32_t bit = GimbalMotorBit(ids[i]);

            if (LowCmdRecoverSafetyInhibitWithPermit(ids[i], permit) != 0u)
            {
                recoveredMask |= bit;
            }
        }
        s_gimbalControlInhibitMask &= ~recoveredMask;
        return (recoveredMask == requiredMask) ? 1u : 0u;
    }
}

static uint8_t GimbalPublishCurrents(const MotorCurrentBind *bindings,
                                     const uint32_t *axisMasks,
                                     const int16_t *currents,
                                     uint8_t bindingCount,
                                     const ControlOutputPermit *permit,
                                     uint32_t preserveAxisMask,
                                     GimbalPublishResult *result)
{
    MotorId ids[DUAL_YAW_GIMBAL_OUTPUT_MOTOR_COUNT];
    int16_t publishedCurrents[DUAL_YAW_GIMBAL_OUTPUT_MOTOR_COUNT];
    uint8_t count = 0u;

    if (result != NULL)
    {
        (void)memset(result, 0, sizeof(*result));
    }
    if (bindings == NULL || axisMasks == NULL || currents == NULL ||
        bindingCount > DUAL_YAW_GIMBAL_OUTPUT_MOTOR_COUNT)
    {
        return 0u;
    }
    for (uint8_t i = 0u; i < bindingCount; i++)
    {
        uint16_t writer = (uint16_t)LOWCMD_WRITER_NONE;

        if ((preserveAxisMask & axisMasks[i]) != 0u ||
            bindings[i].enabled == 0u ||
            (uint32_t)bindings[i].actuator_id >= (uint32_t)MotorCount ||
            LowCmdGetInhibitWriter(bindings[i].actuator_id, &writer) == 0u ||
            writer != (uint16_t)LOWCMD_WRITER_NONE)
        {
            continue;
        }
        ids[count] = bindings[i].actuator_id;
        if (result != NULL)
        {
            result->ids[count] = bindings[i].actuator_id;
            result->axisMasks[count] = axisMasks[i];
        }
        publishedCurrents[count] = GimbalFaultFrameCurrent(
            axisMasks[i],
            s_gimbalFault.holdZeroMask,
            currents[i]);
        count++;
    }

    if (count == 0u)
    {
        return ControlMgrOutputPermitValid(permit, 0u);
    }
    if (MotorInstSetCurrentIdsWithPermit(ids,
                                         publishedCurrents,
                                         count,
                                         permit) == 0u)
    {
        return 0u;
    }
    if (result != NULL)
    {
        result->count = count;
    }
    return 1u;
}

static uint8_t GimbalFeedbackRouteDebtHeld(uint8_t axis, MotorCmd *held)
{
    uint32_t axisMask;
    MotorId id;

    if (axis >= GIMBAL_FAULT_AXIS_COUNT || held == NULL)
    {
        return 0u;
    }
    axisMask = 1u << axis;
    id = (MotorId)s_gimbalFeedbackRouteDebt.motorId[axis];
    if ((GimbalFeedbackRouteDebtMask(&s_gimbalFeedbackRouteDebt) & axisMask) == 0u ||
        (uint32_t)id >= (uint32_t)MotorCount ||
        LowCmdGetMotor(id, held) == 0u ||
        held->active == 0u ||
        held->mode != (uint8_t)MotorModeCurrent ||
        held->writer != (uint16_t)LOWCMD_WRITER_CONTROL ||
        held->current != 0 ||
        held->seq != s_gimbalFeedbackRouteDebt.cmdSeq[axis] ||
        held->seqEpoch != s_gimbalFeedbackRouteDebt.cmdSeqEpoch[axis] ||
        held->tick != s_gimbalFeedbackRouteDebt.cmdTick[axis])
    {
        return 0u;
    }
    return 1u;
}

static uint8_t GimbalFeedbackRouteDebtHasMotor(MotorId id)
{
    const uint32_t debtMask =
        GimbalFeedbackRouteDebtMask(&s_gimbalFeedbackRouteDebt);

    if ((uint32_t)id >= (uint32_t)MotorCount)
    {
        return 0u;
    }
    for (uint8_t axis = 0u; axis < GIMBAL_FAULT_AXIS_COUNT; axis++)
    {
        if ((debtMask & (1u << axis)) != 0u &&
            s_gimbalFeedbackRouteDebt.motorId[axis] == (uint8_t)id)
        {
            return 1u;
        }
    }
    return 0u;
}

static void GimbalSingleMitRouteObserve(const GimbalRuntimeSnapshot *snapshot)
{
    MotorId target;

    if (GimbalSingleMitYawTestActive(snapshot) == 0u)
    {
        s_gimbalSingleMitDesiredPrev = 0u;
        s_gimbalSingleMitTargetPrev = MotorCount;
        return;
    }
    target = snapshot->target_motor;
    if (s_gimbalSingleMitDesiredPrev != 0u &&
        s_gimbalSingleMitTargetPrev == target)
    {
        return;
    }
    for (uint8_t axis = 0u; axis < GIMBAL_FAULT_AXIS_COUNT; axis++)
    {
        if (GimbalFaultAxisId(axis) != target)
        {
            continue;
        }
        if (GimbalFeedbackRouteDebtHasMotor(target) != 0u ||
            GimbalFeedbackRouteDebtRequestPublish(
                &s_gimbalFeedbackRouteDebt,
                1u << axis,
                (uint8_t)target) != 0u)
        {
            s_gimbalSingleMitDesiredPrev = 1u;
            s_gimbalSingleMitTargetPrev = target;
        }
        return;
    }
    s_gimbalSingleMitDesiredPrev = 0u;
    s_gimbalSingleMitTargetPrev = MotorCount;
}

static void GimbalFeedbackRouteDebtObserve(
    const GimbalRuntimeSnapshot *snapshot,
    const MotorCurrentBind *bindings,
    const uint32_t *axisMasks,
    uint8_t bindingCount)
{
    const uint32_t debtMask =
        GimbalFeedbackRouteDebtMask(&s_gimbalFeedbackRouteDebt);

    if (bindings == NULL || axisMasks == NULL ||
        bindingCount > DUAL_YAW_GIMBAL_OUTPUT_MOTOR_COUNT)
    {
        return;
    }
    for (uint8_t i = 0u; i < bindingCount; i++)
    {
        if (bindings[i].enabled == 0u ||
            (uint32_t)bindings[i].actuator_id >= (uint32_t)MotorCount ||
            (debtMask & axisMasks[i]) != 0u ||
            GimbalRuntimeAllowsMotor(snapshot,
                                     bindings[i].actuator_id) != 0u)
        {
            continue;
        }
        (void)GimbalFeedbackRouteDebtRequestPublish(
            &s_gimbalFeedbackRouteDebt,
            axisMasks[i],
            (uint8_t)bindings[i].actuator_id);
    }
}

static void GimbalFeedbackRouteDebtPoll(
    const GimbalRuntimeSnapshot *snapshot)
{
    for (uint8_t axis = 0u; axis < GIMBAL_FAULT_AXIS_COUNT; axis++)
    {
        const uint32_t axisMask = 1u << axis;
        const MotorId id = (MotorId)s_gimbalFeedbackRouteDebt.motorId[axis];
        MotorCmd held;

        if ((s_gimbalFeedbackRouteDebt.blockedMask & axisMask) == 0u)
        {
            continue;
        }
        if (GimbalFeedbackRouteDebtHeld(axis, &held) == 0u ||
            GimbalRuntimeAllowsMotor(snapshot, id) != 0u)
        {
            (void)GimbalFeedbackRouteDebtRequestPublish(
                &s_gimbalFeedbackRouteDebt,
                axisMask,
                (uint8_t)id);
        }
    }

    for (uint8_t axis = 0u; axis < GIMBAL_FAULT_AXIS_COUNT; axis++)
    {
        const uint32_t axisMask = 1u << axis;
        const MotorId id = (MotorId)s_gimbalFeedbackRouteDebt.motorId[axis];
        MotorCmd held;
        MotorTxReceipt receipt;

        if ((s_gimbalFeedbackRouteDebt.waitMask & axisMask) == 0u)
        {
            continue;
        }
        if (GimbalFeedbackRouteDebtHeld(axis, &held) == 0u)
        {
            (void)GimbalFeedbackRouteDebtRequestPublish(
                &s_gimbalFeedbackRouteDebt,
                axisMask,
                (uint8_t)id);
            continue;
        }
        if (GimbalRuntimeAllowsMotor(snapshot, id) == 0u)
        {
            (void)GimbalFeedbackRouteDebtHold(&s_gimbalFeedbackRouteDebt,
                                              axisMask,
                                              (uint8_t)id,
                                              held.seq,
                                              held.seqEpoch,
                                              held.tick);
            continue;
        }
        if (LowStateGetTxReceipt(id, &receipt) != 0u &&
            MotorTxReceiptMatches(&receipt,
                                  held.seq,
                                  held.seqEpoch,
                                  held.tick,
                                  (uint16_t)LOWCMD_WRITER_CONTROL,
                                  (uint8_t)MotorModeCurrent,
                                  0) != 0u)
        {
            (void)GimbalFeedbackRouteDebtComplete(&s_gimbalFeedbackRouteDebt,
                                                  axisMask);
        }
    }
}

static void GimbalFeedbackRouteDebtCapture(
    const GimbalRuntimeSnapshot *snapshot,
    const GimbalPublishResult *result)
{
    if (result == NULL)
    {
        return;
    }

    for (uint8_t i = 0u; i < result->count; i++)
    {
        const uint32_t axisMask = result->axisMasks[i];
        MotorCmd cmd;

        if ((s_gimbalFeedbackRouteDebt.publishMask & axisMask) == 0u ||
            LowCmdGetMotor(result->ids[i], &cmd) == 0u ||
            cmd.active == 0u ||
            cmd.mode != (uint8_t)MotorModeCurrent ||
            cmd.writer != (uint16_t)LOWCMD_WRITER_CONTROL ||
            cmd.current != 0)
        {
            continue;
        }
        if (GimbalRuntimeAllowsMotor(snapshot, result->ids[i]) == 0u)
        {
            (void)GimbalFeedbackRouteDebtHold(&s_gimbalFeedbackRouteDebt,
                                              axisMask,
                                              (uint8_t)result->ids[i],
                                              cmd.seq,
                                              cmd.seqEpoch,
                                              cmd.tick);
        }
        else
        {
            (void)GimbalFeedbackRouteDebtWait(&s_gimbalFeedbackRouteDebt,
                                              axisMask,
                                              (uint8_t)result->ids[i],
                                              cmd.seq,
                                              cmd.seqEpoch,
                                              cmd.tick);
        }
    }
}

static uint8_t GimbalFeedbackRouteDebtCoverBarrier(
    const GimbalRuntimeSnapshot *snapshot,
    uint32_t axisMask)
{
    const uint8_t axis = (axisMask == GIMBAL_FAULT_MASK_YAW) ?
                             GIMBAL_FAULT_AXIS_YAW :
                             ((axisMask == GIMBAL_FAULT_MASK_YAW_UPPER) ?
                                  GIMBAL_FAULT_AXIS_YAW_UPPER :
                                  GIMBAL_FAULT_AXIS_PITCH);
    const MotorId id = (MotorId)s_gimbalFeedbackRouteDebt.motorId[axis];
    MotorCmd held;

    if (GimbalFeedbackRouteDebtHeld(axis, &held) == 0u)
    {
        return 0u;
    }
    if ((s_gimbalFeedbackRouteDebt.blockedMask & axisMask) != 0u)
    {
        return (uint8_t)(GimbalRuntimeAllowsMotor(snapshot, id) == 0u);
    }
    if ((s_gimbalFeedbackRouteDebt.waitMask & axisMask) != 0u &&
        GimbalRuntimeAllowsMotor(snapshot, id) != 0u)
    {
        return GimbalFeedbackZeroBarrierAdd(&s_gimbalFeedbackZeroBarrier,
                                            axisMask,
                                            (uint8_t)id,
                                            held.seq,
                                            held.seqEpoch,
                                            held.tick);
    }
    return 0u;
}

static uint8_t GimbalFeedbackZeroBarrierCapture(
    const GimbalRuntimeSnapshot *snapshot,
    const GimbalPublishResult *result)
{
    uint32_t coveredMask;

    if (snapshot == NULL || result == NULL ||
        snapshot->feedback_transition_pending == 0u)
    {
        return 0u;
    }

    GimbalFeedbackZeroBarrierBegin(&s_gimbalFeedbackZeroBarrier,
                                   snapshot->feedback_transition_seq);
    coveredMask = 0u;
    for (uint8_t i = 0u; i < result->count; i++)
    {
        MotorCmd cmd;
        const uint32_t axisMask = result->axisMasks[i];

        if ((snapshot->feedback_transition_zero_mask & axisMask) == 0u)
        {
            continue;
        }
        if (LowCmdGetMotor(result->ids[i], &cmd) == 0u ||
            cmd.active == 0u ||
            cmd.mode != (uint8_t)MotorModeCurrent ||
            cmd.writer != (uint16_t)LOWCMD_WRITER_CONTROL ||
            cmd.current != 0)
        {
            GimbalFeedbackZeroBarrierInit(&s_gimbalFeedbackZeroBarrier);
            return 0u;
        }
        if (GimbalRuntimeAllowsMotor(snapshot, result->ids[i]) == 0u)
        {
            if (GimbalFeedbackRouteDebtHold(&s_gimbalFeedbackRouteDebt,
                                            axisMask,
                                            (uint8_t)result->ids[i],
                                            cmd.seq,
                                            cmd.seqEpoch,
                                            cmd.tick) == 0u)
            {
                GimbalFeedbackZeroBarrierInit(&s_gimbalFeedbackZeroBarrier);
                return 0u;
            }
        }
        else if (GimbalFeedbackZeroBarrierAdd(&s_gimbalFeedbackZeroBarrier,
                                              axisMask,
                                              (uint8_t)result->ids[i],
                                              cmd.seq,
                                              cmd.seqEpoch,
                                              cmd.tick) == 0u)
        {
            GimbalFeedbackZeroBarrierInit(&s_gimbalFeedbackZeroBarrier);
            return 0u;
        }
        coveredMask |= axisMask;
    }
    for (uint8_t axis = 0u; axis < GIMBAL_FAULT_AXIS_COUNT; axis++)
    {
        const uint32_t axisMask = 1u << axis;
        const MotorId id = GimbalFaultAxisId(axis);
        uint16_t writer = (uint16_t)LOWCMD_WRITER_NONE;

        if ((snapshot->feedback_transition_zero_mask & axisMask) == 0u ||
            (coveredMask & axisMask) != 0u)
        {
            continue;
        }
        if (GimbalFeedbackRouteDebtCoverBarrier(snapshot, axisMask) != 0u)
        {
            coveredMask |= axisMask;
            continue;
        }
        if ((s_gimbalFault.inhibitMask & axisMask) == 0u)
        {
            continue;
        }
        if ((uint32_t)id < (uint32_t)MotorCount &&
            LowCmdGetInhibitWriter(id, &writer) != 0u &&
            writer == (uint16_t)LOWCMD_WRITER_SAFETY &&
            GimbalFeedbackZeroBarrierExcludeSafe(
                &s_gimbalFeedbackZeroBarrier,
                axisMask) != 0u)
        {
            coveredMask |= axisMask;
        }
    }
    if ((coveredMask & snapshot->feedback_transition_zero_mask) !=
        snapshot->feedback_transition_zero_mask)
    {
        GimbalFeedbackZeroBarrierInit(&s_gimbalFeedbackZeroBarrier);
        return 0u;
    }
    return 1u;
}

static GimbalFeedbackZeroWait GimbalFeedbackZeroBarrierPoll(
    const GimbalRuntimeSnapshot *snapshot)
{
    uint8_t allReady = 1u;

    if (snapshot == NULL || snapshot->feedback_transition_pending == 0u)
    {
        GimbalFeedbackZeroBarrierInit(&s_gimbalFeedbackZeroBarrier);
        return GimbalFeedbackZeroWaitNone;
    }
    if (GimbalFeedbackZeroBarrierMatches(&s_gimbalFeedbackZeroBarrier,
                                         snapshot->feedback_transition_seq) == 0u)
    {
        GimbalFeedbackZeroBarrierInit(&s_gimbalFeedbackZeroBarrier);
        return GimbalFeedbackZeroWaitNone;
    }

    for (uint8_t axis = 0u; axis < GIMBAL_FAULT_AXIS_COUNT; axis++)
    {
        const uint32_t axisMask = 1u << axis;
        const MotorId id = GimbalFaultAxisId(axis);
        uint16_t inhibitWriter = (uint16_t)LOWCMD_WRITER_NONE;

        if ((s_gimbalFeedbackZeroBarrier.safeInhibitMask & axisMask) == 0u)
        {
            continue;
        }
        if ((s_gimbalFault.inhibitMask & axisMask) == 0u ||
            (uint32_t)id >= (uint32_t)MotorCount ||
            LowCmdGetInhibitWriter(id, &inhibitWriter) == 0u ||
            inhibitWriter != (uint16_t)LOWCMD_WRITER_SAFETY)
        {
            GimbalFeedbackZeroBarrierInit(&s_gimbalFeedbackZeroBarrier);
            return GimbalFeedbackZeroWaitNone;
        }
    }

    for (uint8_t axis = 0u; axis < 3u; axis++)
    {
        const uint32_t axisMask = 1u << axis;
        const MotorId id = (MotorId)s_gimbalFeedbackZeroBarrier.motorId[axis];
        MotorCmd held;
        MotorTxReceipt receipt;
        uint16_t inhibitWriter = (uint16_t)LOWCMD_WRITER_NONE;

        if ((s_gimbalFeedbackZeroBarrier.waitMask & axisMask) == 0u)
        {
            continue;
        }
        if ((uint32_t)id >= (uint32_t)MotorCount ||
            LowCmdGetInhibitWriter(id, &inhibitWriter) == 0u)
        {
            GimbalFeedbackZeroBarrierInit(&s_gimbalFeedbackZeroBarrier);
            return GimbalFeedbackZeroWaitNone;
        }
        if (inhibitWriter != (uint16_t)LOWCMD_WRITER_NONE)
        {
            if (inhibitWriter == (uint16_t)LOWCMD_WRITER_SAFETY &&
                (s_gimbalFault.inhibitMask & axisMask) != 0u)
            {
                if (GimbalFeedbackZeroBarrierExcludeSafe(
                        &s_gimbalFeedbackZeroBarrier,
                        axisMask) == 0u)
                {
                    GimbalFeedbackZeroBarrierInit(&s_gimbalFeedbackZeroBarrier);
                    return GimbalFeedbackZeroWaitNone;
                }
                s_gimbalFeedbackZeroBarrier.waitMask &= ~axisMask;
                continue;
            }
            GimbalFeedbackZeroBarrierInit(&s_gimbalFeedbackZeroBarrier);
            return GimbalFeedbackZeroWaitNone;
        }
        if (LowCmdGetMotor(id, &held) == 0u ||
            held.active == 0u ||
            held.mode != (uint8_t)MotorModeCurrent ||
            held.writer != (uint16_t)LOWCMD_WRITER_CONTROL ||
            held.current != 0 ||
            held.seq != s_gimbalFeedbackZeroBarrier.cmdSeq[axis] ||
            held.seqEpoch != s_gimbalFeedbackZeroBarrier.cmdSeqEpoch[axis] ||
            held.tick != s_gimbalFeedbackZeroBarrier.cmdTick[axis])
        {
            GimbalFeedbackZeroBarrierInit(&s_gimbalFeedbackZeroBarrier);
            return GimbalFeedbackZeroWaitNone;
        }
        if (LowStateGetTxReceipt(id, &receipt) == 0u ||
            MotorTxReceiptMatches(&receipt,
                                  held.seq,
                                  held.seqEpoch,
                                  held.tick,
                                  (uint16_t)LOWCMD_WRITER_CONTROL,
                                  (uint8_t)MotorModeCurrent,
                                  0) == 0u)
        {
            allReady = 0u;
        }
    }
    return (allReady != 0u) ?
               GimbalFeedbackZeroReady :
               GimbalFeedbackZeroWaiting;
}


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
    ShootCtrlPrepare();
    PitchCaliBootLoad();
    last_wake = xTaskGetTickCount();
    while (1)
    {
        const uint64_t loop_start_us = RtProfBegin();
        ManualInputSnapshot manual_input;
        const ManualInputSnapshot *frame_input =
            (ManualInputSnapshotRead(&manual_input) != 0u) ? &manual_input : NULL;
        GimbalRuntimeSnapshot snapshot;
        GimbalPublishResult publish_result;
        ControlOutputPermit output_permit = {0};
        uint8_t gimbal_allowed;
        uint8_t publish_ok = 0u;
        uint8_t feedback_transition_complete = 0u;
        uint32_t feedback_zero_wait_mask = 0u;
        (void)memset(&publish_result, 0, sizeof(publish_result));
        WatchTaskBeat(WATCH_TASK_GIMBAL_CONTROL);
        GimbalSnapshotCapture(&snapshot, &g_gimbal, frame_input);
        GimbalFeedbackDiscardVision(&snapshot);
        GimbalFaultUpdate(&snapshot);
        gimbal_allowed = (uint8_t)GimbalControlMgrAllows(ControlIdSingleGimbal,
                                                         &snapshot,
                                                         &output_permit);
        snapshot.control_allowed = gimbal_allowed;
        GimbalSingleMitRouteObserve(&snapshot);
        GimbalFeedbackRouteDebtObserve(&snapshot,
                                       GimbalOutputCurrentBindings,
                                       GimbalOutputAxisMasks,
                                       GIMBAL_OUTPUT_MOTOR_COUNT);
        GimbalFaultSyncInhibit(&g_gimbal,
                               &snapshot,
                               (gimbal_allowed != 0u) ? &output_permit : NULL);
        GimbalFeedbackRouteDebtPoll(&snapshot);
        feedback_zero_wait_mask = s_gimbalFeedbackRouteDebt.blockedMask |
                                  s_gimbalFeedbackRouteDebt.waitMask;
        GimbalLoopCounter++;
        if (gimbal_allowed == 0u)
        {
            GimbalBehaviourInputGateBlock();
            GimbalControlInhibitOutputs(GimbalOutputCurrentBindings,
                                        GIMBAL_OUTPUT_MOTOR_COUNT);
            GimbalStopOutputs(&snapshot);
            GimbalRunShootControl(&snapshot);
            RtProfEnd(RtProfGimbalLoop, loop_start_us);
            GimbalControlDelay(&last_wake, snapshot.period_ms);

#if INCLUDE_uxTaskGetStackHighWaterMark
            GimbalStackSampleMaybe();
#endif
            continue;
        }
        (void)GimbalControlReleaseOutputs(GimbalOutputCurrentBindings,
                                          GIMBAL_OUTPUT_MOTOR_COUNT,
                                          &output_permit);
        {
            const GimbalFeedbackZeroWait zeroWait =
                GimbalFeedbackZeroBarrierPoll(&snapshot);

            if (zeroWait == GimbalFeedbackZeroReady)
            {
                feedback_transition_complete = 1u;
                feedback_zero_wait_mask |= s_gimbalFeedbackZeroBarrier.waitMask;
            }
            else if (zeroWait == GimbalFeedbackZeroWaiting)
            {
                feedback_zero_wait_mask |= s_gimbalFeedbackZeroBarrier.waitMask;
            }
        }

        GimbalSetMode(&g_gimbal);                    //设置云台控制模式
        PitchCaliTickPre(&g_gimbal, GimbalBehaviourWatch, snapshot.PitchCaliMode);
        GimbalModeChangeControlTransit(&g_gimbal); //控制模式切换 控制数据过渡
        GimbalFeedbackUpdate(&g_gimbal, &snapshot);  //云台数据反馈
        GimbalFeedbackTransitionReset(&g_gimbal, &snapshot);
        GimbalSetControl(&g_gimbal);
        GimbalControlLoop(&g_gimbal);
        PitchCaliTickPost(&g_gimbal, GimbalBehaviourWatch, snapshot.PitchCaliMode);
        GimbalApplyHealthToControl(&g_gimbal, &snapshot);
        GimbalFaultApplyControl(&g_gimbal, &snapshot);
        GimbalWriteState(&snapshot);
        GimbalRunShootControl(&snapshot);
        yaw_can_set_current = GimbalApplyOutputTurn(g_gimbal.GimbalYawMotor.given_current, snapshot.yaw_turn);
        pitch_can_set_current = GimbalApplyOutputTurn(g_gimbal.GimbalPitchMotor.given_current, snapshot.pitch_turn);

        const int16_t yaw_current_request = yaw_can_set_current;
        const int16_t pitch_current_request = pitch_can_set_current;

        GimbalApplyOperationMode(&snapshot, &yaw_can_set_current, &pitch_can_set_current);
        GimbalApplyOutputHealth(&snapshot, &yaw_can_set_current, NULL, &pitch_can_set_current);
        GimbalFaultApplyOutput(&yaw_can_set_current, NULL, &pitch_can_set_current);

        yaw_can_set_current = MotorCfgLimitCurrentNode(snapshot.yaw_motor_cfg, yaw_can_set_current);
        pitch_can_set_current = MotorCfgLimitCurrentNode(snapshot.pitch_motor_cfg, pitch_can_set_current);

        // watch 输出：观察最终下发电流（含运行模式、安全模式及方向翻转后的值）
        GimbalWatchYawCurrent = yaw_can_set_current;
        GimbalWatchPitchCurrent = pitch_can_set_current;
        if (GimbalSingleMitYawTestActive(&snapshot) != 0u &&
            snapshot.feedback_transition_pending == 0u &&
            GimbalFeedbackRouteDebtHasMotor(snapshot.target_motor) == 0u)
        {
            GimbalWatchYawCurrent = 0;
            GimbalWatchPitchCurrent = 0;
            publish_ok = GimbalApplySingleMitYawTest(
                &snapshot,
                GimbalOutputCurrentBindings,
                GimbalOutputAxisMasks,
                GIMBAL_OUTPUT_MOTOR_COUNT,
                &output_permit,
                &publish_result);
        }
        else
        {
            const int16_t GimbalCurrentCmd[] = {
                yaw_can_set_current,
                pitch_can_set_current,
            };

            publish_ok = GimbalPublishCurrents(GimbalOutputCurrentBindings,
                                               GimbalOutputAxisMasks,
                                               GimbalCurrentCmd,
                                               GIMBAL_OUTPUT_MOTOR_COUNT,
                                               &output_permit,
                                               feedback_zero_wait_mask,
                                               &publish_result);
        }
        if (publish_ok != 0u && s_gimbalFeedbackRouteDebt.publishMask != 0u)
        {
            GimbalFeedbackRouteDebtCapture(&snapshot, &publish_result);
        }
        if (snapshot.feedback_transition_pending != 0u &&
            publish_ok != 0u &&
            GimbalFeedbackZeroBarrierMatches(&s_gimbalFeedbackZeroBarrier,
                                             snapshot.feedback_transition_seq) == 0u)
        {
            publish_ok = GimbalFeedbackZeroBarrierCapture(&snapshot,
                                                          &publish_result);
        }
        if (feedback_transition_complete != 0u)
        {
            const MotorAxisFaultInhibitPlan faultPlan =
                MotorAxisFaultInhibitPlanMake(s_gimbalFault.configuredMask,
                                              s_gimbalFault.blockingMask,
                                              s_gimbalFault.inhibitMask);

            GimbalFeedbackPolicyConsumeTransition(&s_gimbalFeedbackPolicy);
            GimbalFeedbackZeroBarrierInit(&s_gimbalFeedbackZeroBarrier);
            s_gimbalFault.holdZeroMask = faultPlan.holdZeroMask |
                                         GimbalFeedbackRouteDebtMask(
                                             &s_gimbalFeedbackRouteDebt);
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
    ShootCtrlPrepare();

    last_wake = xTaskGetTickCount();
    while (1)
    {
        const uint64_t loop_start_us = RtProfBegin();
        ManualInputSnapshot manual_input;
        const ManualInputSnapshot *frame_input =
            (ManualInputSnapshotRead(&manual_input) != 0u) ? &manual_input : NULL;
        GimbalRuntimeSnapshot snapshot;
        GimbalPublishResult publish_result;
        ControlOutputPermit output_permit = {0};
        uint8_t gimbal_allowed;
        uint8_t publish_ok = 0u;
        uint8_t feedback_transition_complete = 0u;
        uint32_t feedback_zero_wait_mask = 0u;

        (void)memset(&publish_result, 0, sizeof(publish_result));
        WatchTaskBeat(WATCH_TASK_GIMBAL_CONTROL);
        GimbalSnapshotCapture(&snapshot, &g_gimbal, frame_input);
        GimbalFeedbackDiscardVision(&snapshot);
        GimbalFaultUpdate(&snapshot);
        gimbal_allowed = (uint8_t)GimbalControlMgrAllows(ControlIdDualYawGimbal,
                                                         &snapshot,
                                                         &output_permit);
        snapshot.control_allowed = gimbal_allowed;
        GimbalSingleMitRouteObserve(&snapshot);
        GimbalFeedbackRouteDebtObserve(&snapshot,
                                       DualYawGimbalOutputCurrentBindings,
                                       DualYawGimbalOutputAxisMasks,
                                       DUAL_YAW_GIMBAL_OUTPUT_MOTOR_COUNT);
        GimbalFaultSyncInhibit(&g_gimbal,
                               &snapshot,
                               (gimbal_allowed != 0u) ? &output_permit : NULL);
        GimbalFeedbackRouteDebtPoll(&snapshot);
        feedback_zero_wait_mask = s_gimbalFeedbackRouteDebt.blockedMask |
                                  s_gimbalFeedbackRouteDebt.waitMask;
        const uint8_t output_allowed = RobotLifecycleOutputAllowed();
        GimbalLoopCounter++;
        if (gimbal_allowed == 0u)
        {
            GimbalBehaviourInputGateBlock();
            if (output_allowed == 0u)
            {
                GimbalDualYawImuCenterUpdateOnOutput(&g_gimbal, &snapshot, 0u);
            }
            GimbalControlInhibitOutputs(DualYawGimbalOutputCurrentBindings,
                                        DUAL_YAW_GIMBAL_OUTPUT_MOTOR_COUNT);
            GimbalStopOutputs(&snapshot);
            GimbalRunShootControl(&snapshot);
            RtProfEnd(RtProfGimbalLoop, loop_start_us);
            GimbalControlDelay(&last_wake, snapshot.period_ms);

#if INCLUDE_uxTaskGetStackHighWaterMark
            GimbalStackSampleMaybe();
#endif
            continue;
        }
        (void)GimbalControlReleaseOutputs(DualYawGimbalOutputCurrentBindings,
                                          DUAL_YAW_GIMBAL_OUTPUT_MOTOR_COUNT,
                                          &output_permit);
        {
            const GimbalFeedbackZeroWait zeroWait =
                GimbalFeedbackZeroBarrierPoll(&snapshot);

            if (zeroWait == GimbalFeedbackZeroReady)
            {
                feedback_transition_complete = 1u;
                feedback_zero_wait_mask |= s_gimbalFeedbackZeroBarrier.waitMask;
            }
            else if (zeroWait == GimbalFeedbackZeroWaiting)
            {
                feedback_zero_wait_mask |= s_gimbalFeedbackZeroBarrier.waitMask;
            }
        }

        GimbalSetMode(&g_gimbal);
        GimbalModeChangeControlTransit(&g_gimbal);
        GimbalFeedbackUpdate(&g_gimbal, &snapshot);
        GimbalFeedbackTransitionReset(&g_gimbal, &snapshot);
        GimbalDualYawImuCenterUpdateOnOutput(&g_gimbal, &snapshot, output_allowed);
        GimbalSetControl(&g_gimbal);
        GimbalControlLoop(&g_gimbal);
        GimbalApplyHealthToControl(&g_gimbal, &snapshot);
        GimbalFaultApplyControl(&g_gimbal, &snapshot);
        GimbalWriteState(&snapshot);
        GimbalRunShootControl(&snapshot);

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
        yaw_log_current_output = (snapshot.yaw_control_is_upper != 0u) ? yaw_upper_can_set_current : yaw_can_set_current;

        GimbalWatchYawCurrent = yaw_can_set_current;
        GimbalWatchYawUpperCurrent = yaw_upper_can_set_current;
        GimbalWatchPitchCurrent = pitch_can_set_current;
        if (GimbalSingleMitYawTestActive(&snapshot) != 0u &&
            snapshot.feedback_transition_pending == 0u &&
            GimbalFeedbackRouteDebtHasMotor(snapshot.target_motor) == 0u)
        {
            GimbalWatchYawCurrent = 0;
            GimbalWatchYawUpperCurrent = 0;
            GimbalWatchPitchCurrent = 0;
            publish_ok = GimbalApplySingleMitYawTest(
                &snapshot,
                DualYawGimbalOutputCurrentBindings,
                DualYawGimbalOutputAxisMasks,
                DUAL_YAW_GIMBAL_OUTPUT_MOTOR_COUNT,
                &output_permit,
                &publish_result);
        }
        else
        {
            const int16_t GimbalCurrentCmd[] = {
                yaw_can_set_current,
                yaw_upper_can_set_current,
                pitch_can_set_current,
            };

            publish_ok = GimbalPublishCurrents(DualYawGimbalOutputCurrentBindings,
                                               DualYawGimbalOutputAxisMasks,
                                               GimbalCurrentCmd,
                                               DUAL_YAW_GIMBAL_OUTPUT_MOTOR_COUNT,
                                               &output_permit,
                                               feedback_zero_wait_mask,
                                               &publish_result);
        }
        if (publish_ok != 0u && s_gimbalFeedbackRouteDebt.publishMask != 0u)
        {
            GimbalFeedbackRouteDebtCapture(&snapshot, &publish_result);
        }
        if (snapshot.feedback_transition_pending != 0u &&
            publish_ok != 0u &&
            GimbalFeedbackZeroBarrierMatches(&s_gimbalFeedbackZeroBarrier,
                                             snapshot.feedback_transition_seq) == 0u)
        {
            publish_ok = GimbalFeedbackZeroBarrierCapture(&snapshot,
                                                          &publish_result);
        }
        if (feedback_transition_complete != 0u)
        {
            const MotorAxisFaultInhibitPlan faultPlan =
                MotorAxisFaultInhibitPlanMake(s_gimbalFault.configuredMask,
                                              s_gimbalFault.blockingMask,
                                              s_gimbalFault.inhibitMask);

            GimbalFeedbackPolicyConsumeTransition(&s_gimbalFeedbackPolicy);
            GimbalFeedbackZeroBarrierInit(&s_gimbalFeedbackZeroBarrier);
            s_gimbalFault.holdZeroMask = faultPlan.holdZeroMask |
                                         GimbalFeedbackRouteDebtMask(
                                             &s_gimbalFeedbackRouteDebt);
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
