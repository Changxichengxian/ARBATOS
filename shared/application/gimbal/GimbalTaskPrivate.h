/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * 云台任务内部接口。业务外部只包含 GimbalControlTask.h 和 GimbalState.h。
 */

#ifndef GIMBAL_TASK_PRIVATE_H
#define GIMBAL_TASK_PRIVATE_H

#include "RobotConfig.h"
#include "RobotTaskBuildConfig.h"

#if ROBOT_TASK_BUILD_ANY_GIMBAL

#include <stdint.h>

#include "ControlMgr.h"
#include "FaultMgr.h"
#include "FreeRTOS.h"
#include "GimbalControlTask.h"
#include "GimbalFaultPolicy.h"
#include "GimbalFeedbackPolicy.h"
#include "InsTask.h"
#include "ManualInputSnapshot.h"
#include "MotorHealth.h"
#include "MotorInst.h"
#include "SdLog.h"
#include "task.h"

#define GIMBAL_OUTPUT_MOTOR_COUNT 2u
#define DUAL_YAW_GIMBAL_OUTPUT_MOTOR_COUNT 3u
#define GIMBAL_MOTOR_FEEDBACK_TIMEOUT_MS 50u
#define GIMBAL_IMU_SNAPSHOT_TIMEOUT_MS 10u
#define GIMBAL_FAULT_RECOVERY_MS 200u
#define GIMBAL_SDLOG_BASE_STREAM_MAX_SAMPLES 16u

#define PITCH_KICK_ERR_OFF_RAD 0.008f
#define PITCH_KICK_ERR_ON_RAD  0.025f

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

#define GimbalTotalPidClear(control)                                  \
    do {                                                              \
        GimbalPidClear(&(control)->GimbalYawMotor.GimbalMotorAnglePid); \
        PID_clear(&(control)->GimbalYawMotor.GimbalMotorGyroPid);      \
        GimbalPidClear(&(control)->GimbalPitchMotor.GimbalMotorAnglePid); \
        PID_clear(&(control)->GimbalPitchMotor.GimbalMotorGyroPid);    \
    } while (0)

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

typedef struct
{
    FaultMgr mgr;
    uint32_t configuredMask;
    uint32_t activeMask;
    uint32_t motorBlockingMask;
    uint32_t blockingMask;
    uint32_t recoveryMask;
    uint32_t inhibitMask;
    uint32_t holdZeroMask;
    uint32_t imuRequiredMask;
    uint32_t inhibitFailCount;
    uint32_t releaseFailCount;
    uint8_t initialized;
    uint8_t recoveryInputSafe;
} GimbalFaultState;

typedef struct
{
    uint32_t start_tick_ms;
    uint32_t last_tick_ms;
    uint32_t period_us;
    uint16_t sample_count;
    uint16_t sample_div_counter;
    sdlog_gimbal_base_sample_t samples[GIMBAL_SDLOG_BASE_STREAM_MAX_SAMPLES];
} GimbalSdLogBaseStreamState;

typedef struct
{
    GimbalControl control;
    GimbalFeedbackPolicyState feedbackPolicy;
    GimbalFeedbackZeroBarrier feedbackZeroBarrier;
    GimbalFeedbackRouteDebt feedbackRouteDebt;
    GimbalFaultState fault;
    GimbalSdLogBaseStreamState log;
    GimbalDualYawDebugState dualYawDebug;
    MotorCurrentBind outputBindings[GIMBAL_OUTPUT_MOTOR_COUNT];
    MotorCurrentBind dualYawOutputBindings[DUAL_YAW_GIMBAL_OUTPUT_MOTOR_COUNT];
    MotorId yawId;
    MotorId yawUpperId;
    MotorId pitchId;
    MotorId singleMitTargetPrev;
    uint32_t controlInhibitMask;
    fp32 dualYawImuCenter;
    int16_t yawCurrent;
    int16_t pitchCurrent;
    uint8_t motorIdsReady;
    uint8_t singleMitDesiredPrev;
    uint8_t dualYawImuCenterReady;
    uint8_t dualYawOutputAllowedPrev;
} GimbalTaskContext;

fp32 GimbalMotorAngleFromEncoder(uint16_t ecd, uint16_t offset_ecd,
                                const motor_node_param_t *node);

void GimbalCalibrationSet(GimbalControl *control,
                          uint16_t yawOffset,
                          uint16_t pitchOffset,
                          fp32 maxYaw,
                          fp32 minYaw,
                          fp32 maxPitch,
                          fp32 minPitch);
bool_t GimbalCalibrationStep(GimbalControl *control,
                             uint16_t *yawOffset,
                             uint16_t *pitchOffset,
                             fp32 *maxYaw,
                             fp32 *minYaw,
                             fp32 *maxPitch,
                             fp32 *minPitch);
const GimbalMotor *GimbalCalibrationYawMotor(const GimbalControl *control);
const GimbalMotor *GimbalCalibrationPitchMotor(const GimbalControl *control);

void GimbalTuneGetYawSpeedPidFor(const GimbalControl *control, PidParam *out);
void GimbalTuneGetYawAnglePidFor(const GimbalControl *control, PidParam *out);
void GimbalTuneSetYawSpeedPidFor(GimbalControl *control, const PidParam *pid, bool_t clearState);
void GimbalTuneSetYawAnglePidFor(GimbalControl *control, const PidParam *pid, bool_t clearState);
void GimbalTuneClearYawPidFor(GimbalControl *control);
void GimbalTuneGetPitchSpeedPidFor(const GimbalControl *control, PidParam *out);
void GimbalTuneGetPitchAnglePidFor(const GimbalControl *control, PidParam *out);
void GimbalTuneSetPitchSpeedPidFor(GimbalControl *control, const PidParam *pid, bool_t clearState);
void GimbalTuneSetPitchAnglePidFor(GimbalControl *control, const PidParam *pid, bool_t clearState);
void GimbalTuneClearPitchPidFor(GimbalControl *control);

void GimbalInit(GimbalTaskContext *ctx, GimbalControl *control);
void GimbalSetMode(GimbalTaskContext *ctx, GimbalControl *control);
void GimbalFeedbackUpdate(GimbalTaskContext *ctx,
                          GimbalControl *control,
                          const GimbalRuntimeSnapshot *snapshot);
void GimbalDualYawImuCenterUpdateOnOutput(GimbalTaskContext *ctx,
                                          GimbalControl *control,
                                          const GimbalRuntimeSnapshot *snapshot,
                                          uint8_t outputAllowed);
void GimbalModeChangeControlTransit(GimbalTaskContext *ctx, GimbalControl *control);
void GimbalSetControl(GimbalTaskContext *ctx, GimbalControl *control);
void GimbalControlLoop(GimbalTaskContext *ctx, GimbalControl *control);

void GimbalMotorIdsEnsure(GimbalTaskContext *ctx);
MotorId GimbalRuntimeMotorId(GimbalTaskContext *ctx, uint8_t axis);
fp32 GimbalPitchKickScale(fp32 angleErr);
uint8_t GimbalImuFeedbackReady(const GimbalRuntimeSnapshot *snapshot);
uint8_t GimbalYawControlHealthy(const GimbalRuntimeSnapshot *snapshot);
void GimbalAxisClear(GimbalMotor *motor);
uint8_t GimbalFeedbackAxisReady(const GimbalRuntimeSnapshot *snapshot, uint32_t axisMask);
void GimbalFeedbackDiscardVision(const GimbalRuntimeSnapshot *snapshot);
void GimbalFeedbackTransitionReset(GimbalTaskContext *ctx,
                                   GimbalControl *control,
                                   const GimbalRuntimeSnapshot *snapshot);
void GimbalApplyHealthToControl(GimbalControl *control, const GimbalRuntimeSnapshot *snapshot);
void GimbalApplyOutputHealth(const GimbalRuntimeSnapshot *snapshot,
                             int16_t *yawCurrent,
                             int16_t *yawUpperCurrent,
                             int16_t *pitchCurrent);
int16_t GimbalApplyOutputTurn(int16_t current, uint8_t turn);
uint8_t GimbalRuntimeAllowsMotor(const GimbalRuntimeSnapshot *snapshot, MotorId id);
uint8_t GimbalSingleMitYawTestActive(GimbalTaskContext *ctx,
                                     const GimbalRuntimeSnapshot *snapshot);
uint8_t GimbalApplySingleMitYawTest(GimbalTaskContext *ctx,
                                    const GimbalRuntimeSnapshot *snapshot,
                                    const MotorCurrentBind *bindings,
                                    const uint32_t *axisMasks,
                                    uint8_t bindingCount,
                                    const ControlOutputPermit *permit,
                                    GimbalPublishResult *result);
uint8_t GimbalDualYawUseUpperControl(void);
const motor_measure_t *GimbalYawControlMeasurePoint(void);
const motor_node_param_t *GimbalYawControlMotorCfg(void);
uint8_t GimbalYawSoftLimitEnabled(void);
void GimbalSnapshotCapture(GimbalTaskContext *ctx,
                           GimbalRuntimeSnapshot *snapshot,
                           GimbalControl *control,
                           const ManualInputSnapshot *manualInput);
void GimbalApplyOperationMode(GimbalTaskContext *ctx,
                              const GimbalRuntimeSnapshot *snapshot,
                              int16_t *yawCurrent,
                              int16_t *pitchCurrent);
bool_t GimbalControlMgrAllows(uint16_t expectedControllerId,
                              const GimbalRuntimeSnapshot *snapshot,
                              ControlOutputPermit *permit);
void GimbalStopOutputs(GimbalTaskContext *ctx, const GimbalRuntimeSnapshot *snapshot);
void GimbalControlDelay(TickType_t *lastWake, uint16_t periodMs);

MotorId GimbalFaultAxisId(GimbalTaskContext *ctx, uint8_t axis);
void GimbalFaultInit(GimbalTaskContext *ctx);
void GimbalFaultUpdate(GimbalTaskContext *ctx, const GimbalRuntimeSnapshot *snapshot);
void GimbalFaultSyncInhibit(GimbalTaskContext *ctx,
                            GimbalControl *control,
                            const GimbalRuntimeSnapshot *snapshot,
                            const ControlOutputPermit *permit);
void GimbalFaultApplyControl(GimbalTaskContext *ctx,
                             GimbalControl *control,
                             const GimbalRuntimeSnapshot *snapshot);
void GimbalFaultApplyOutput(const GimbalTaskContext *ctx,
                            int16_t *yawCurrent,
                            int16_t *yawUpperCurrent,
                            int16_t *pitchCurrent);
uint8_t GimbalFaultInhibitsMotor(GimbalTaskContext *ctx, MotorId id);

void GimbalControlInhibitOutputs(GimbalTaskContext *ctx,
                                 const MotorCurrentBind *bindings,
                                 uint8_t count);
uint8_t GimbalControlReleaseOutputs(GimbalTaskContext *ctx,
                                    const MotorCurrentBind *bindings,
                                    uint8_t bindingCount,
                                    const ControlOutputPermit *permit);
uint8_t GimbalPublishCurrents(GimbalTaskContext *ctx,
                              const MotorCurrentBind *bindings,
                              const uint32_t *axisMasks,
                              const int16_t *currents,
                              uint8_t bindingCount,
                              const ControlOutputPermit *permit,
                              uint32_t preserveAxisMask,
                              GimbalPublishResult *result);
uint8_t GimbalFeedbackRouteDebtHasMotor(const GimbalTaskContext *ctx, MotorId id);
void GimbalSingleMitRouteObserve(GimbalTaskContext *ctx,
                                 const GimbalRuntimeSnapshot *snapshot);
void GimbalFeedbackRouteDebtObserve(GimbalTaskContext *ctx,
                                    const GimbalRuntimeSnapshot *snapshot,
                                    const MotorCurrentBind *bindings,
                                    const uint32_t *axisMasks,
                                    uint8_t bindingCount);
void GimbalFeedbackRouteDebtPoll(GimbalTaskContext *ctx,
                                 const GimbalRuntimeSnapshot *snapshot);
void GimbalFeedbackRouteDebtCapture(GimbalTaskContext *ctx,
                                    const GimbalRuntimeSnapshot *snapshot,
                                    const GimbalPublishResult *result);
uint8_t GimbalFeedbackZeroBarrierCapture(GimbalTaskContext *ctx,
                                         const GimbalRuntimeSnapshot *snapshot,
                                         const GimbalPublishResult *result);
GimbalFeedbackZeroWait GimbalFeedbackZeroBarrierPoll(GimbalTaskContext *ctx,
                                                     const GimbalRuntimeSnapshot *snapshot);

void GimbalWriteState(const GimbalControl *control,
                      const GimbalFaultState *fault,
                      const GimbalRuntimeSnapshot *snapshot);
int16_t GimbalSdLogClampCurrent(fp32 current);
void GimbalSdLogAppendBaseSample(GimbalSdLogBaseStreamState *log,
                                 const sdlog_gimbal_base_sample_t *sample,
                                 uint32_t nowMs,
                                 uint32_t periodUs);

uint8_t GimbalDualYawDebugReadFrom(const GimbalDualYawDebugState *state,
                                   GimbalDualYawDebugState *out);
int16_t GimbalDualYawUpperControlCurrent(const GimbalControl *control,
                                         GimbalDualYawDebugState *debugState,
                                         const GimbalRuntimeSnapshot *snapshot,
                                         int16_t yawCurrent);

#endif

#endif
