/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

/*
 * 阅读地图：
 * - 本文件只持有任务上下文、兼容接口和单/双 yaw 周期编排。
 * - 快照与反馈、故障、输出确认、日志、控制运算及校准调参分别在同目录独立模块。
 * - 两个任务入口按同一安全顺序更新状态、计算电流并通过 LowCmd 发布。
 */

#include "RobotConfig.h"
#include "RobotTaskBuildConfig.h"

#if ROBOT_TASK_BUILD_ANY_GIMBAL

#include "GimbalControlTask.h"
#include "GimbalCore.h"
#include "GimbalTaskPrivate.h"

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
#if ROBOT_TASK_BUILD_CALIBRATION
#include "CalibrateTask.h"
#endif

#include <string.h>

#define GIMBAL_STACK_SAMPLE_PERIOD_MS 1000u
static const uint32_t GimbalOutputAxisMasks[GIMBAL_OUTPUT_MOTOR_COUNT] = {
    GIMBAL_FAULT_MASK_YAW,
    GIMBAL_FAULT_MASK_PITCH,
};

static const uint32_t DualYawGimbalOutputAxisMasks[DUAL_YAW_GIMBAL_OUTPUT_MOTOR_COUNT] = {
    GIMBAL_FAULT_MASK_YAW,
    GIMBAL_FAULT_MASK_YAW_UPPER,
    GIMBAL_FAULT_MASK_PITCH,
};


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

static GimbalTaskContext s_gimbalTaskContext;
static volatile uint8_t GimbalCaliRequested;
static volatile uint8_t GimbalCaliControlReady;
static uint32_t GimbalCaliAuthoritySeq;
static uint32_t GimbalCaliSemanticsSeq;

static uint8_t GimbalCalibrationInputReady(ManualInputSnapshot *input)
{
    return (uint8_t)(robot_mode_gimbal_cali() != 0u &&
        RobotLifecycleOutputAllowed() != 0u &&
        s_gimbalTaskContext.control.fast.GimbalCaliMode != 0u &&
        ManualInputSnapshotRead(input) != 0u && input->online != 0u && input->dataValid != 0u &&
        input->sourceTimeoutMs != 0u && input->sourceAgeMs <= input->sourceTimeoutMs &&
        ControlInputSwitchIsPos(input->control.sw[INPUT_SW_GIMBAL_MODE], input->semantics.GimbalSafePos) == 0u);
}

void GimbalCalibrationCancel(void)
{
    taskENTER_CRITICAL();
    const uint8_t wasRequested = GimbalCaliRequested;
    GimbalCaliRequested = 0u;
    s_gimbalTaskContext.control.GimbalCali.step = 0u;
    s_gimbalTaskContext.control.GimbalCali.stableTicks = 0u;
    if (wasRequested != 0u) {
        /* 同时撤销已发布命令，拦住取消前算完、取消后才提交的电流。 */
        GimbalControlInhibitOutputs(&s_gimbalTaskContext, s_gimbalTaskContext.outputBindings,
                                    GIMBAL_OUTPUT_MOTOR_COUNT);
    }
    taskEXIT_CRITICAL();
}

uint8_t GimbalCalibrationRequest(void)
{
    ManualInputSnapshot input;
    if (GimbalCalibrationInputReady(&input) == 0u || GimbalCaliRequested != 0u) {
        return 0u;
    }
    taskENTER_CRITICAL();
    s_gimbalTaskContext.control.GimbalCali.step = 0u;
    s_gimbalTaskContext.control.GimbalCali.stableTicks = 0u;
    GimbalCaliAuthoritySeq = input.authoritySeq;
    GimbalCaliSemanticsSeq = input.semanticsSeq;
    GimbalCaliRequested = 1u;
    taskEXIT_CRITICAL();
    return 1u;
}

uint8_t GimbalCalibrationPending(void)
{
    return GimbalCaliRequested;
}

uint8_t GimbalCalibrationControlReady(void)
{
    return GimbalCaliControlReady;
}

uint8_t GimbalCalibrationRestoreAllowed(void)
{
    ManualInputSnapshot input;
    if (GimbalCaliControlReady == 0u || GimbalCaliRequested != 0u ||
        GimbalWatchYawCurrent != 0 || GimbalWatchPitchCurrent != 0)
    {
        return 0u;
    }
    if (robot_mode_gimbal_cali() != 0u) {
        return 1u;
    }
    return (uint8_t)(ManualInputSnapshotRead(&input) != 0u &&
        (input.online == 0u || input.dataValid == 0u ||
         ControlInputSwitchIsPos(input.control.sw[INPUT_SW_GIMBAL_MODE], input.semantics.GimbalSafePos) != 0u));
}

uint8_t GimbalCalibrationRestore(uint16_t yawOffset, uint16_t pitchOffset,
                                  fp32 maxYaw, fp32 minYaw, fp32 maxPitch, fp32 minPitch)
{
    uint8_t restored = 0u;
    taskENTER_CRITICAL();
    if (GimbalCalibrationRestoreAllowed() != 0u) {
        GimbalCalibrationSet(&s_gimbalTaskContext.control, yawOffset, pitchOffset,
                             maxYaw, minYaw, maxPitch, minPitch);
        restored = 1u;
    }
    taskEXIT_CRITICAL();
    return restored;
}

uint8_t GimbalCalibrationOutputAllowed(void)
{
    ManualInputSnapshot input;
    const uint8_t step = s_gimbalTaskContext.control.GimbalCali.step;
    return (uint8_t)(GimbalCaliRequested != 0u && step != 0u && step != GIMBAL_CALI_END_STEP &&
        GimbalCalibrationInputReady(&input) != 0u && input.authoritySeq == GimbalCaliAuthoritySeq &&
        input.semanticsSeq == GimbalCaliSemanticsSeq);
}

volatile uint32_t GimbalLoopCounter = 0;

volatile int16_t GimbalWatchYawCurrent = 0;
volatile int16_t GimbalWatchYawUpperCurrent = 0;
volatile int16_t GimbalWatchPitchCurrent = 0;
volatile int16_t GimbalYawEasytestCurrent = 3000;

void set_cali_gimbal_hook(const uint16_t yaw_offset,
                          const uint16_t pitch_offset,
                          const fp32 max_yaw,
                          const fp32 min_yaw,
                          const fp32 max_pitch,
                          const fp32 min_pitch)
{
    GimbalCalibrationSet(&s_gimbalTaskContext.control,
                         yaw_offset,
                         pitch_offset,
                         max_yaw,
                         min_yaw,
                         max_pitch,
                         min_pitch);
}

bool_t cmd_cali_gimbal_hook(uint16_t *yaw_offset,
                            uint16_t *pitch_offset,
                            fp32 *max_yaw,
                            fp32 *min_yaw,
                            fp32 *max_pitch,
                            fp32 *min_pitch)
{
    ManualInputSnapshot input;
    bool_t done;
    if (GimbalCaliRequested == 0u) {
        return 0;
    }
    if (GimbalCalibrationInputReady(&input) == 0u || input.authoritySeq != GimbalCaliAuthoritySeq ||
        input.semanticsSeq != GimbalCaliSemanticsSeq) {
        GimbalCalibrationCancel();
        return 0;
    }
    taskENTER_CRITICAL();
    done = GimbalCalibrationStep(&s_gimbalTaskContext.control,
                                 yaw_offset,
                                 pitch_offset,
                                 max_yaw,
                                 min_yaw,
                                 max_pitch,
                                 min_pitch);
    if (done != 0) {
        GimbalCaliRequested = 0u;
    }
    taskEXIT_CRITICAL();
    return done;
}

const GimbalMotor *get_yaw_motor_point(void)
{
    return GimbalCalibrationYawMotor(&s_gimbalTaskContext.control);
}

const GimbalMotor *get_pitch_motor_point(void)
{
    return GimbalCalibrationPitchMotor(&s_gimbalTaskContext.control);
}

uint8_t GimbalDualYawDebugRead(GimbalDualYawDebugState *out)
{
    return GimbalDualYawDebugReadFrom(&s_gimbalTaskContext.dualYawDebug, out);
}

void GimbalTuneGetYawSpeedPid(PidParam *out)
{
    GimbalTuneGetYawSpeedPidFor(&s_gimbalTaskContext.control, out);
}

void GimbalTuneGetYawAnglePid(PidParam *out)
{
    GimbalTuneGetYawAnglePidFor(&s_gimbalTaskContext.control, out);
}

void GimbalTuneSetYawSpeedPid(const PidParam *pid, bool_t clear_state)
{
    GimbalTuneSetYawSpeedPidFor(&s_gimbalTaskContext.control, pid, clear_state);
}

void GimbalTuneSetYawAnglePid(const PidParam *pid, bool_t clear_state)
{
    GimbalTuneSetYawAnglePidFor(&s_gimbalTaskContext.control, pid, clear_state);
}

void GimbalTuneClearYawPid(void)
{
    GimbalTuneClearYawPidFor(&s_gimbalTaskContext.control);
}

void GimbalTuneGetPitchSpeedPid(PidParam *out)
{
    GimbalTuneGetPitchSpeedPidFor(&s_gimbalTaskContext.control, out);
}

void GimbalTuneGetPitchAnglePid(PidParam *out)
{
    GimbalTuneGetPitchAnglePidFor(&s_gimbalTaskContext.control, out);
}

void GimbalTuneSetPitchSpeedPid(const PidParam *pid, bool_t clear_state)
{
    GimbalTuneSetPitchSpeedPidFor(&s_gimbalTaskContext.control, pid, clear_state);
}

void GimbalTuneSetPitchAnglePid(const PidParam *pid, bool_t clear_state)
{
    GimbalTuneSetPitchAnglePidFor(&s_gimbalTaskContext.control, pid, clear_state);
}

void GimbalTuneClearPitchPid(void)
{
    GimbalTuneClearPitchPidFor(&s_gimbalTaskContext.control);
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
    GimbalInit(&s_gimbalTaskContext, &s_gimbalTaskContext.control);
    GimbalFaultInit(&s_gimbalTaskContext);
    GimbalWriteState(&s_gimbalTaskContext.control, &s_gimbalTaskContext.fault, NULL);
    PitchCaliBootLoad();
    GimbalCaliControlReady = 1u;
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
        GimbalSnapshotCapture(&s_gimbalTaskContext, &snapshot, &s_gimbalTaskContext.control, frame_input);
        GimbalFeedbackDiscardVision(&snapshot);
        GimbalFaultUpdate(&s_gimbalTaskContext, &snapshot);
        gimbal_allowed = (uint8_t)GimbalControlMgrAllows(ControlIdSingleGimbal,
                                                         &snapshot,
                                                         &output_permit);
        snapshot.control_allowed = gimbal_allowed;
#if ROBOT_TASK_BUILD_CALIBRATION
        if (robot_mode_gimbal_cali() != 0u && CalibrateReady() == 0u) {
            gimbal_allowed = 0u;
            snapshot.control_allowed = 0u;
        }
#endif
        if (GimbalCaliRequested != 0u) {
            ManualInputSnapshot caliInput;
            if (gimbal_allowed == 0u || s_gimbalTaskContext.fault.blockingMask != 0u ||
                GimbalCalibrationInputReady(&caliInput) == 0u ||
                caliInput.authoritySeq != GimbalCaliAuthoritySeq ||
                caliInput.semanticsSeq != GimbalCaliSemanticsSeq) {
                GimbalCalibrationCancel();
            }
        }
        GimbalSingleMitRouteObserve(&s_gimbalTaskContext, &snapshot);
        GimbalFeedbackRouteDebtObserve(&s_gimbalTaskContext, &snapshot,
                                       s_gimbalTaskContext.outputBindings,
                                       GimbalOutputAxisMasks,
                                       GIMBAL_OUTPUT_MOTOR_COUNT);
        GimbalFaultSyncInhibit(&s_gimbalTaskContext, &s_gimbalTaskContext.control,
                               &snapshot,
                               (gimbal_allowed != 0u) ? &output_permit : NULL);
        GimbalFeedbackRouteDebtPoll(&s_gimbalTaskContext, &snapshot);
        feedback_zero_wait_mask = s_gimbalTaskContext.feedbackRouteDebt.blockedMask |
                                  s_gimbalTaskContext.feedbackRouteDebt.waitMask;
        GimbalLoopCounter++;
        if (gimbal_allowed == 0u)
        {
            GimbalBehaviourInputGateBlock();
            GimbalControlInhibitOutputs(&s_gimbalTaskContext, s_gimbalTaskContext.outputBindings,
                                        GIMBAL_OUTPUT_MOTOR_COUNT);
            GimbalStopOutputs(&s_gimbalTaskContext, &snapshot);
            RtProfEnd(RtProfGimbalLoop, loop_start_us);
            GimbalControlDelay(&last_wake, snapshot.period_ms);

#if INCLUDE_uxTaskGetStackHighWaterMark
            GimbalStackSampleMaybe();
#endif
            continue;
        }
        (void)GimbalControlReleaseOutputs(&s_gimbalTaskContext, s_gimbalTaskContext.outputBindings,
                                          GIMBAL_OUTPUT_MOTOR_COUNT,
                                          &output_permit);
        {
            const GimbalFeedbackZeroWait zeroWait =
                GimbalFeedbackZeroBarrierPoll(&s_gimbalTaskContext, &snapshot);

            if (zeroWait == GimbalFeedbackZeroReady)
            {
                feedback_transition_complete = 1u;
                feedback_zero_wait_mask |= s_gimbalTaskContext.feedbackZeroBarrier.waitMask;
            }
            else if (zeroWait == GimbalFeedbackZeroWaiting)
            {
                feedback_zero_wait_mask |= s_gimbalTaskContext.feedbackZeroBarrier.waitMask;
            }
        }

        GimbalSetMode(&s_gimbalTaskContext, &s_gimbalTaskContext.control);                    //设置云台控制模式
        PitchCaliTickPre(&s_gimbalTaskContext.control, GimbalBehaviourWatch, snapshot.PitchCaliMode);
        GimbalModeChangeControlTransit(&s_gimbalTaskContext, &s_gimbalTaskContext.control); //控制模式切换 控制数据过渡
        GimbalFeedbackUpdate(&s_gimbalTaskContext, &s_gimbalTaskContext.control, &snapshot);  //云台数据反馈
        GimbalFeedbackTransitionReset(&s_gimbalTaskContext, &s_gimbalTaskContext.control, &snapshot);
        GimbalSetControl(&s_gimbalTaskContext, &s_gimbalTaskContext.control);
        GimbalControlLoop(&s_gimbalTaskContext, &s_gimbalTaskContext.control);
        PitchCaliTickPost(&s_gimbalTaskContext.control, GimbalBehaviourWatch, snapshot.PitchCaliMode);
        GimbalApplyHealthToControl(&s_gimbalTaskContext.control, &snapshot);
        GimbalFaultApplyControl(&s_gimbalTaskContext, &s_gimbalTaskContext.control, &snapshot);
        GimbalWriteState(&s_gimbalTaskContext.control, &s_gimbalTaskContext.fault, &snapshot);
        s_gimbalTaskContext.yawCurrent = GimbalApplyOutputTurn(s_gimbalTaskContext.control.GimbalYawMotor.given_current, snapshot.yaw_turn);
        s_gimbalTaskContext.pitchCurrent = GimbalApplyOutputTurn(s_gimbalTaskContext.control.GimbalPitchMotor.given_current, snapshot.pitch_turn);

        const int16_t yaw_current_request = s_gimbalTaskContext.yawCurrent;
        const int16_t pitch_current_request = s_gimbalTaskContext.pitchCurrent;

        GimbalApplyOperationMode(&s_gimbalTaskContext, &snapshot, &s_gimbalTaskContext.yawCurrent, &s_gimbalTaskContext.pitchCurrent);
        GimbalApplyOutputHealth(&snapshot, &s_gimbalTaskContext.yawCurrent, NULL, &s_gimbalTaskContext.pitchCurrent);
        GimbalFaultApplyOutput(&s_gimbalTaskContext, &s_gimbalTaskContext.yawCurrent, NULL, &s_gimbalTaskContext.pitchCurrent);

        s_gimbalTaskContext.yawCurrent = MotorCfgLimitCurrentNode(snapshot.yaw_motor_cfg, s_gimbalTaskContext.yawCurrent);
        s_gimbalTaskContext.pitchCurrent = MotorCfgLimitCurrentNode(snapshot.pitch_motor_cfg, s_gimbalTaskContext.pitchCurrent);

        // watch 输出：观察最终下发电流（含运行模式、安全模式及方向翻转后的值）
        GimbalWatchYawCurrent = s_gimbalTaskContext.yawCurrent;
        GimbalWatchPitchCurrent = s_gimbalTaskContext.pitchCurrent;
        if (GimbalSingleMitYawTestActive(&s_gimbalTaskContext, &snapshot) != 0u &&
            snapshot.feedback_transition_pending == 0u &&
            GimbalFeedbackRouteDebtHasMotor(&s_gimbalTaskContext, snapshot.target_motor) == 0u)
        {
            GimbalWatchYawCurrent = 0;
            GimbalWatchPitchCurrent = 0;
            publish_ok = GimbalApplySingleMitYawTest(
                &s_gimbalTaskContext,
                &snapshot,
                s_gimbalTaskContext.outputBindings,
                GimbalOutputAxisMasks,
                GIMBAL_OUTPUT_MOTOR_COUNT,
                &output_permit,
                &publish_result);
        }
        else
        {
            const int16_t GimbalCurrentCmd[] = {
                s_gimbalTaskContext.yawCurrent,
                s_gimbalTaskContext.pitchCurrent,
            };

            publish_ok = GimbalPublishCurrents(&s_gimbalTaskContext, s_gimbalTaskContext.outputBindings,
                                               GimbalOutputAxisMasks,
                                               GimbalCurrentCmd,
                                               GIMBAL_OUTPUT_MOTOR_COUNT,
                                               &output_permit,
                                               feedback_zero_wait_mask,
                                               &publish_result);
        }
        if (publish_ok != 0u && s_gimbalTaskContext.feedbackRouteDebt.publishMask != 0u)
        {
            GimbalFeedbackRouteDebtCapture(&s_gimbalTaskContext, &snapshot, &publish_result);
        }
        if (snapshot.feedback_transition_pending != 0u &&
            publish_ok != 0u &&
            GimbalFeedbackZeroBarrierMatches(&s_gimbalTaskContext.feedbackZeroBarrier,
                                             snapshot.feedback_transition_seq) == 0u)
        {
            publish_ok = GimbalFeedbackZeroBarrierCapture(&s_gimbalTaskContext, &snapshot,
                                                          &publish_result);
        }
        if (feedback_transition_complete != 0u)
        {
            const MotorAxisFaultInhibitPlan faultPlan =
                MotorAxisFaultInhibitPlanMake(s_gimbalTaskContext.fault.configuredMask,
                                              s_gimbalTaskContext.fault.blockingMask,
                                              s_gimbalTaskContext.fault.inhibitMask);

            GimbalFeedbackPolicyConsumeTransition(&s_gimbalTaskContext.feedbackPolicy);
            GimbalFeedbackZeroBarrierInit(&s_gimbalTaskContext.feedbackZeroBarrier);
            s_gimbalTaskContext.fault.holdZeroMask = faultPlan.holdZeroMask |
                                         GimbalFeedbackRouteDebtMask(
                                             &s_gimbalTaskContext.feedbackRouteDebt);
        }

        {
            sdlog_gimbal_base_sample_t sample = {0};

            sample.GimbalBehaviour = (uint8_t)GimbalBehaviourWatch;
            sample.test_mode = (uint8_t)snapshot.run_variant;
            sample.yaw_motor_mode = (uint8_t)s_gimbalTaskContext.control.GimbalYawMotor.mode;
            sample.pitch_motor_mode = (uint8_t)s_gimbalTaskContext.control.GimbalPitchMotor.mode;
            sample.yaw_angle = s_gimbalTaskContext.control.GimbalYawMotor.angle;
            sample.pitch_angle = s_gimbalTaskContext.control.GimbalPitchMotor.angle;
            sample.yaw_gyro = s_gimbalTaskContext.control.GimbalYawMotor.motor_gyro;
            sample.pitch_gyro = s_gimbalTaskContext.control.GimbalPitchMotor.motor_gyro;
            sample.yaw_current_request = yaw_current_request;
            sample.pitch_current_request = pitch_current_request;
            sample.yaw_current_output = GimbalSdLogClampCurrent(s_gimbalTaskContext.yawCurrent);
            sample.pitch_current_output = GimbalSdLogClampCurrent(s_gimbalTaskContext.pitchCurrent);

            GimbalSdLogAppendBaseSample(&s_gimbalTaskContext.log, &sample, BspTimeGetTickMs(), snapshot.period_us);
        }

#if GIMBAL_TEST_MODE
        J_scope_gimbal_test(&s_gimbalTaskContext);
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
    GimbalInit(&s_gimbalTaskContext, &s_gimbalTaskContext.control);
    GimbalFaultInit(&s_gimbalTaskContext);
    GimbalWriteState(&s_gimbalTaskContext.control, &s_gimbalTaskContext.fault, NULL);

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
        GimbalSnapshotCapture(&s_gimbalTaskContext, &snapshot, &s_gimbalTaskContext.control, frame_input);
        GimbalFeedbackDiscardVision(&snapshot);
        GimbalFaultUpdate(&s_gimbalTaskContext, &snapshot);
        gimbal_allowed = (uint8_t)GimbalControlMgrAllows(ControlIdDualYawGimbal,
                                                         &snapshot,
                                                         &output_permit);
        snapshot.control_allowed = gimbal_allowed;
        GimbalSingleMitRouteObserve(&s_gimbalTaskContext, &snapshot);
        GimbalFeedbackRouteDebtObserve(&s_gimbalTaskContext, &snapshot,
                                       s_gimbalTaskContext.dualYawOutputBindings,
                                       DualYawGimbalOutputAxisMasks,
                                       DUAL_YAW_GIMBAL_OUTPUT_MOTOR_COUNT);
        GimbalFaultSyncInhibit(&s_gimbalTaskContext, &s_gimbalTaskContext.control,
                               &snapshot,
                               (gimbal_allowed != 0u) ? &output_permit : NULL);
        GimbalFeedbackRouteDebtPoll(&s_gimbalTaskContext, &snapshot);
        feedback_zero_wait_mask = s_gimbalTaskContext.feedbackRouteDebt.blockedMask |
                                  s_gimbalTaskContext.feedbackRouteDebt.waitMask;
        const uint8_t output_allowed = RobotLifecycleOutputAllowed();
        GimbalLoopCounter++;
        if (gimbal_allowed == 0u)
        {
            GimbalBehaviourInputGateBlock();
            if (output_allowed == 0u)
            {
                GimbalDualYawImuCenterUpdateOnOutput(&s_gimbalTaskContext, &s_gimbalTaskContext.control, &snapshot, 0u);
            }
            GimbalControlInhibitOutputs(&s_gimbalTaskContext,
                                        s_gimbalTaskContext.dualYawOutputBindings,
                                        DUAL_YAW_GIMBAL_OUTPUT_MOTOR_COUNT);
            GimbalStopOutputs(&s_gimbalTaskContext, &snapshot);
            RtProfEnd(RtProfGimbalLoop, loop_start_us);
            GimbalControlDelay(&last_wake, snapshot.period_ms);

#if INCLUDE_uxTaskGetStackHighWaterMark
            GimbalStackSampleMaybe();
#endif
            continue;
        }
        (void)GimbalControlReleaseOutputs(&s_gimbalTaskContext,
                                          s_gimbalTaskContext.dualYawOutputBindings,
                                          DUAL_YAW_GIMBAL_OUTPUT_MOTOR_COUNT,
                                          &output_permit);
        {
            const GimbalFeedbackZeroWait zeroWait =
                GimbalFeedbackZeroBarrierPoll(&s_gimbalTaskContext, &snapshot);

            if (zeroWait == GimbalFeedbackZeroReady)
            {
                feedback_transition_complete = 1u;
                feedback_zero_wait_mask |= s_gimbalTaskContext.feedbackZeroBarrier.waitMask;
            }
            else if (zeroWait == GimbalFeedbackZeroWaiting)
            {
                feedback_zero_wait_mask |= s_gimbalTaskContext.feedbackZeroBarrier.waitMask;
            }
        }

        GimbalSetMode(&s_gimbalTaskContext, &s_gimbalTaskContext.control);
        GimbalModeChangeControlTransit(&s_gimbalTaskContext, &s_gimbalTaskContext.control);
        GimbalFeedbackUpdate(&s_gimbalTaskContext, &s_gimbalTaskContext.control, &snapshot);
        GimbalFeedbackTransitionReset(&s_gimbalTaskContext, &s_gimbalTaskContext.control, &snapshot);
        GimbalDualYawImuCenterUpdateOnOutput(&s_gimbalTaskContext, &s_gimbalTaskContext.control, &snapshot, output_allowed);
        GimbalSetControl(&s_gimbalTaskContext, &s_gimbalTaskContext.control);
        GimbalControlLoop(&s_gimbalTaskContext, &s_gimbalTaskContext.control);
        GimbalApplyHealthToControl(&s_gimbalTaskContext.control, &snapshot);
        GimbalFaultApplyControl(&s_gimbalTaskContext, &s_gimbalTaskContext.control, &snapshot);
        GimbalWriteState(&s_gimbalTaskContext.control, &s_gimbalTaskContext.fault, &snapshot);

        s_gimbalTaskContext.yawCurrent = GimbalApplyOutputTurn(s_gimbalTaskContext.control.GimbalYawMotor.given_current, snapshot.yaw_turn);
        s_gimbalTaskContext.pitchCurrent = GimbalApplyOutputTurn(s_gimbalTaskContext.control.GimbalPitchMotor.given_current, snapshot.pitch_turn);

        const int16_t yaw_current_request = s_gimbalTaskContext.yawCurrent;
        const int16_t pitch_current_request = s_gimbalTaskContext.pitchCurrent;

        GimbalApplyOperationMode(&s_gimbalTaskContext, &snapshot, &s_gimbalTaskContext.yawCurrent, &s_gimbalTaskContext.pitchCurrent);

        int16_t yaw_upper_can_set_current = GimbalDualYawUpperControlCurrent(&s_gimbalTaskContext.control,
                                                                       &s_gimbalTaskContext.dualYawDebug,
                                                                       &snapshot, s_gimbalTaskContext.yawCurrent);
        int16_t yaw_log_current_output;

        if (snapshot.yaw_control_is_upper != 0u)
        {
            yaw_upper_can_set_current = s_gimbalTaskContext.yawCurrent;
            s_gimbalTaskContext.yawCurrent = 0;
        }

        GimbalApplyOutputHealth(&snapshot,
                                &s_gimbalTaskContext.yawCurrent,
                                &yaw_upper_can_set_current,
                                &s_gimbalTaskContext.pitchCurrent);
        GimbalFaultApplyOutput(&s_gimbalTaskContext, &s_gimbalTaskContext.yawCurrent,
                               &yaw_upper_can_set_current,
                               &s_gimbalTaskContext.pitchCurrent);

        s_gimbalTaskContext.yawCurrent = MotorCfgLimitCurrentNode(&g_config.motor.yaw, s_gimbalTaskContext.yawCurrent);
        yaw_upper_can_set_current = MotorCfgLimitCurrentNode(&g_config.motor.yaw_upper, yaw_upper_can_set_current);
        s_gimbalTaskContext.pitchCurrent = MotorCfgLimitCurrentNode(snapshot.pitch_motor_cfg, s_gimbalTaskContext.pitchCurrent);
        yaw_log_current_output = (snapshot.yaw_control_is_upper != 0u) ? yaw_upper_can_set_current : s_gimbalTaskContext.yawCurrent;

        GimbalWatchYawCurrent = s_gimbalTaskContext.yawCurrent;
        GimbalWatchYawUpperCurrent = yaw_upper_can_set_current;
        GimbalWatchPitchCurrent = s_gimbalTaskContext.pitchCurrent;
        if (GimbalSingleMitYawTestActive(&s_gimbalTaskContext, &snapshot) != 0u &&
            snapshot.feedback_transition_pending == 0u &&
            GimbalFeedbackRouteDebtHasMotor(&s_gimbalTaskContext, snapshot.target_motor) == 0u)
        {
            GimbalWatchYawCurrent = 0;
            GimbalWatchYawUpperCurrent = 0;
            GimbalWatchPitchCurrent = 0;
            publish_ok = GimbalApplySingleMitYawTest(
                &s_gimbalTaskContext,
                &snapshot,
                s_gimbalTaskContext.dualYawOutputBindings,
                DualYawGimbalOutputAxisMasks,
                DUAL_YAW_GIMBAL_OUTPUT_MOTOR_COUNT,
                &output_permit,
                &publish_result);
        }
        else
        {
            const int16_t GimbalCurrentCmd[] = {
                s_gimbalTaskContext.yawCurrent,
                yaw_upper_can_set_current,
                s_gimbalTaskContext.pitchCurrent,
            };

            publish_ok = GimbalPublishCurrents(&s_gimbalTaskContext,
                                               s_gimbalTaskContext.dualYawOutputBindings,
                                               DualYawGimbalOutputAxisMasks,
                                               GimbalCurrentCmd,
                                               DUAL_YAW_GIMBAL_OUTPUT_MOTOR_COUNT,
                                               &output_permit,
                                               feedback_zero_wait_mask,
                                               &publish_result);
        }
        if (publish_ok != 0u && s_gimbalTaskContext.feedbackRouteDebt.publishMask != 0u)
        {
            GimbalFeedbackRouteDebtCapture(&s_gimbalTaskContext, &snapshot, &publish_result);
        }
        if (snapshot.feedback_transition_pending != 0u &&
            publish_ok != 0u &&
            GimbalFeedbackZeroBarrierMatches(&s_gimbalTaskContext.feedbackZeroBarrier,
                                             snapshot.feedback_transition_seq) == 0u)
        {
            publish_ok = GimbalFeedbackZeroBarrierCapture(&s_gimbalTaskContext, &snapshot,
                                                          &publish_result);
        }
        if (feedback_transition_complete != 0u)
        {
            const MotorAxisFaultInhibitPlan faultPlan =
                MotorAxisFaultInhibitPlanMake(s_gimbalTaskContext.fault.configuredMask,
                                              s_gimbalTaskContext.fault.blockingMask,
                                              s_gimbalTaskContext.fault.inhibitMask);

            GimbalFeedbackPolicyConsumeTransition(&s_gimbalTaskContext.feedbackPolicy);
            GimbalFeedbackZeroBarrierInit(&s_gimbalTaskContext.feedbackZeroBarrier);
            s_gimbalTaskContext.fault.holdZeroMask = faultPlan.holdZeroMask |
                                         GimbalFeedbackRouteDebtMask(
                                             &s_gimbalTaskContext.feedbackRouteDebt);
        }

        {
            sdlog_gimbal_base_sample_t sample = {0};

            sample.GimbalBehaviour = (uint8_t)GimbalBehaviourWatch;
            sample.test_mode = (uint8_t)snapshot.run_variant;
            sample.yaw_motor_mode = (uint8_t)s_gimbalTaskContext.control.GimbalYawMotor.mode;
            sample.pitch_motor_mode = (uint8_t)s_gimbalTaskContext.control.GimbalPitchMotor.mode;
            sample.yaw_angle = s_gimbalTaskContext.control.GimbalYawMotor.angle;
            sample.pitch_angle = s_gimbalTaskContext.control.GimbalPitchMotor.angle;
            sample.yaw_gyro = s_gimbalTaskContext.control.GimbalYawMotor.motor_gyro;
            sample.pitch_gyro = s_gimbalTaskContext.control.GimbalPitchMotor.motor_gyro;
            sample.yaw_current_request = yaw_current_request;
            sample.pitch_current_request = pitch_current_request;
            sample.yaw_current_output = GimbalSdLogClampCurrent(yaw_log_current_output);
            sample.pitch_current_output = GimbalSdLogClampCurrent(s_gimbalTaskContext.pitchCurrent);

            GimbalSdLogAppendBaseSample(&s_gimbalTaskContext.log, &sample, BspTimeGetTickMs(), snapshot.period_us);
        }

        RtProfEnd(RtProfGimbalLoop, loop_start_us);
        GimbalControlDelay(&last_wake, snapshot.period_ms);

#if INCLUDE_uxTaskGetStackHighWaterMark
        GimbalStackSampleMaybe();
#endif
    }
}

#endif
