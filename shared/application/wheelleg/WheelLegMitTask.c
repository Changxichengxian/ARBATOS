/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include "RobotConfig.h"
#include "RobotTaskBuildConfig.h"

#if ROBOT_TASK_BUILD_WHEELLEG_MIT

#include "WheelLegMitTask.h"
#include "WheelLegCore.h"

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"

#include "InsTask.h"
#include "CanReceive.h"
#include "LowCmd.h"
#include "BspCan.h"
#include "RobotConfig.h"
#include "ControlInput.h"
#include "ManualInputSnapshot.h"
#include "DetectTask.h"
#include "FaultMgr.h"
#include "MotorHealth.h"
#include "MotorInst.h"
#include "RobotMsg.h"
#include "RobotTaskProfile.h"
#include "RobotMode.h"
#include "RtProf.h"
#include "ControlMgr.h"
#include "SdLog.h"
#include "Watch.h"
#include "WheelLegMsg.h"
#include "WheelLegOutputPlan.h"

#include <math.h>
#include <string.h>

#define WHEELLEG_PI 3.14159265358979323846f
#define WHEELLEG_TWO_PI 6.28318530717958647692f
#define WHEELLEG_FEEDBACK_TIMEOUT_MS 80u
#define WHEELLEG_BENCH_CENTER_LEG_M 0.085f
#define WHEELLEG_MANUAL_FOOT_X_RANGE_M 0.060f
#define WHEELLEG_TARGET_LEG_SLEW_MPS 0.030f
#define WHEELLEG_TARGET_FOOT_X_SLEW_MPS 0.060f
#define WHEELLEG_AUTO_LEG_REACH_EPS_M 0.003f
#define WHEELLEG_AUTO_LEG_DWELL_MS 500u
#define WHEELLEG_DETACHED_JOINT_TEST_KP 2.0f
#define WHEELLEG_DETACHED_JOINT_TEST_KD 0.16f
#define WHEELLEG_SDLOG_BASE_PERIOD_MS 10u
#define WHEELLEG_LQR_WHEEL_TORQUE_SCALE_DEFAULT 0.18f
#define WHEELLEG_LQR_HIP_TORQUE_SCALE_DEFAULT 0.18f
#define WHEELLEG_LQR_X_HOLD_LIMIT_M 0.05f
#define WHEELLEG_LQR_MOTION_EPS 1.0e-4f
#define WHEELLEG_PITCH_TRIM_GAIN_DEFAULT 0.08f
#define WHEELLEG_PITCH_TRIM_MAX_RAD_DEFAULT 0.18f
#define WHEELLEG_PITCH_TRIM_RATE_RADPS_DEFAULT 0.015f
#define WHEELLEG_PITCH_TRIM_V_DEADBAND_MPS_DEFAULT 0.015f
#define WHEELLEG_PITCH_TRIM_LPF_DEFAULT 0.03f

#define WHEELLEG_CONTROL_STAGE_BENCH_LQR 0u
#define WHEELLEG_CONTROL_STAGE_POSITION_LQR 1u
#define WHEELLEG_CONTROL_STAGE_VMC_HEIGHT_LQR 2u
#define WHEELLEG_CONTROL_STAGE_VMC_FULL_LQR 3u
#define WHEELLEG_CONTROL_STAGE_MAX WHEELLEG_CONTROL_STAGE_VMC_FULL_LQR
#define WHEELLEG_DOMAIN_MEMBER_COUNT 6u
#define WHEELLEG_DOMAIN_RECOVERY_STABLE_MS 500u
#define WHEELLEG_DOMAIN_REASON_MOTOR_OFFLINE (1u << 0)
#define WHEELLEG_DOMAIN_REASON_IMU_OFFLINE (1u << 1)
#define WHEELLEG_DOMAIN_REASON_CONFIG (1u << 2)
#define WHEELLEG_DOMAIN_REASON_TEST_TRANSITION (1u << 3)

#ifndef WHEELLEG_BENCH_TARGET_FOOT_X_M
#define WHEELLEG_BENCH_TARGET_FOOT_X_M 0.0f
#endif

#ifndef WHEELLEG_ENABLE_VMC_BALANCE
#define WHEELLEG_ENABLE_VMC_BALANCE 0
#endif

#ifndef WHEELLEG_IMU_BODY_ROLL_OFFSET
#define WHEELLEG_IMU_BODY_ROLL_OFFSET INS_ROLL_ADDRESS_OFFSET
#endif

#ifndef WHEELLEG_IMU_BODY_PITCH_OFFSET
#define WHEELLEG_IMU_BODY_PITCH_OFFSET INS_PITCH_ADDRESS_OFFSET
#endif

#ifndef WHEELLEG_IMU_BODY_YAW_OFFSET
#define WHEELLEG_IMU_BODY_YAW_OFFSET INS_YAW_ADDRESS_OFFSET
#endif

#ifndef WHEELLEG_IMU_ROLL_SIGN
#define WHEELLEG_IMU_ROLL_SIGN 1.0f
#endif

#ifndef WHEELLEG_IMU_PITCH_SIGN
#define WHEELLEG_IMU_PITCH_SIGN 1.0f
#endif

#ifndef WHEELLEG_IMU_YAW_SIGN
#define WHEELLEG_IMU_YAW_SIGN 1.0f
#endif

#ifndef WHEELLEG_IMU_BODY_GYRO_X_OFFSET
#define WHEELLEG_IMU_BODY_GYRO_X_OFFSET INS_GYRO_X_ADDRESS_OFFSET
#endif

#ifndef WHEELLEG_IMU_BODY_GYRO_Y_OFFSET
#define WHEELLEG_IMU_BODY_GYRO_Y_OFFSET INS_GYRO_Y_ADDRESS_OFFSET
#endif

#ifndef WHEELLEG_IMU_BODY_GYRO_Z_OFFSET
#define WHEELLEG_IMU_BODY_GYRO_Z_OFFSET INS_GYRO_Z_ADDRESS_OFFSET
#endif

#ifndef WHEELLEG_IMU_GYRO_X_SIGN
#define WHEELLEG_IMU_GYRO_X_SIGN WHEELLEG_IMU_ROLL_SIGN
#endif

#ifndef WHEELLEG_IMU_GYRO_Y_SIGN
#define WHEELLEG_IMU_GYRO_Y_SIGN WHEELLEG_IMU_PITCH_SIGN
#endif

#ifndef WHEELLEG_IMU_GYRO_Z_SIGN
#define WHEELLEG_IMU_GYRO_Z_SIGN WHEELLEG_IMU_YAW_SIGN
#endif

typedef struct
{
    MotorId front;
    MotorId back;
    MotorId wheel;
} WheelLegActuatorMap;

typedef enum
{
    WheelLegMemberLeftFront = 0u,
    WheelLegMemberLeftBack,
    WheelLegMemberLeftWheel,
    WheelLegMemberRightFront,
    WheelLegMemberRightBack,
    WheelLegMemberRightWheel,
    WheelLegMemberCount = WHEELLEG_DOMAIN_MEMBER_COUNT,
} WheelLegDomainMember;

typedef enum
{
    WheelLegFaultDeviceLeftFront = WheelLegMemberLeftFront,
    WheelLegFaultDeviceLeftBack = WheelLegMemberLeftBack,
    WheelLegFaultDeviceLeftWheel = WheelLegMemberLeftWheel,
    WheelLegFaultDeviceRightFront = WheelLegMemberRightFront,
    WheelLegFaultDeviceRightBack = WheelLegMemberRightBack,
    WheelLegFaultDeviceRightWheel = WheelLegMemberRightWheel,
    WheelLegFaultDeviceImu,
    WheelLegFaultDeviceGuard,
    WheelLegFaultDeviceCount,
} WheelLegFaultDevice;

typedef enum
{
    WheelLegFaultDomainDrive = 0u,
    WheelLegFaultDomainCount,
} WheelLegFaultDomain;

#define WHEELLEG_FAULT_DOMAIN_MEMBER_MASK ((1u << WheelLegFaultDeviceCount) - 1u)

static const FaultMgrConfig s_wheelleg_fault_config = {
    .deviceCount = (uint8_t)WheelLegFaultDeviceCount,
    .domainCount = (uint8_t)WheelLegFaultDomainCount,
    .domain = {
        {
            .memberMask = WHEELLEG_FAULT_DOMAIN_MEMBER_MASK,
            .criticalMask = WHEELLEG_FAULT_DOMAIN_MEMBER_MASK,
            .recovery = {
                .stableMs = WHEELLEG_DOMAIN_RECOVERY_STABLE_MS,
                .requireSafeInput = 1u,
            },
        },
    },
};

typedef struct
{
    MotorId actuator;
    uint16_t offline_fault;
} WheelLegDomainBinding;

typedef WheelLegCorePid WheelLegPid;
typedef WheelLegCoreLegCalc WheelLegLegCalc;

typedef struct
{
    fp32 x_m;
    fp32 y_m;
    fp32 length_m;
} WheelLegFootPoint;

static WheelLegCoreGeometry WheelLegCoreGeometryFromConfig(void)
{
    WheelLegCoreGeometry geo;
    const WheelLegMitConfig *cfg = &g_config.WheelLegMit;

    geo.l1_m = cfg->l1_m;
    geo.l2_m = cfg->l2_m;
    geo.l3_m = cfg->l3_m;
    geo.l4_m = cfg->l4_m;
    geo.l5_m = cfg->l5_m;
    return geo;
}

static void WheelLegCorePointFromLocal(const WheelLegFootPoint *src, WheelLegCoreFootPoint *dst)
{
    if (src == NULL || dst == NULL)
    {
        return;
    }

    dst->x_m = src->x_m;
    dst->y_m = src->y_m;
    dst->length_m = src->length_m;
}

typedef struct
{
    WheelLegActuatorMap actuator[WHEELLEG_SIDE_COUNT];
    WheelLegDomainBinding domain_member[WHEELLEG_DOMAIN_MEMBER_COUNT];
    FaultMgr fault_mgr;
    FaultDomainStatus fault_domain_status;
    MotorState front_fb[WHEELLEG_SIDE_COUNT];
    MotorState back_fb[WHEELLEG_SIDE_COUNT];
    MotorState wheel_fb[WHEELLEG_SIDE_COUNT];
    WheelLegLegCalc leg[WHEELLEG_SIDE_COUNT];
    WheelLegPid leg_pid[WHEELLEG_SIDE_COUNT];
    WheelLegPid split_pid;
    fp32 x_m;
    fp32 v_mps;
    fp32 target_leg_smooth;
    fp32 target_foot_x_smooth;
    fp32 target_foot_y_smooth;
    fp32 yaw_set;
    fp32 last_yaw;
    uint8_t target_smooth_valid;
    uint8_t yaw_inited;
    uint8_t balance_idle_latched;
    uint8_t ever_commanded;
    uint8_t control_stage;
    WheelLegMode mode;
    WheelLegMode last_mode;
    uint32_t overrun_count;
    uint8_t left_test_active;
    uint32_t left_test_start_ms;
    uint8_t foot_test_active;
    uint32_t foot_test_start_ms;
    uint8_t foot_test_phase;
    uint8_t foot_test_ik_ok;
    uint8_t detached_test_active;
    uint32_t detached_test_start_ms;
    uint8_t auto_leg_stage;
    uint32_t auto_leg_stage_tick_ms;
    fp32 bench_hold_front_rad[WHEELLEG_SIDE_COUNT];
    fp32 bench_hold_back_rad[WHEELLEG_SIDE_COUNT];
    uint8_t bench_hold_front_valid[WHEELLEG_SIDE_COUNT];
    uint8_t bench_hold_back_valid[WHEELLEG_SIDE_COUNT];
    uint8_t bench_hold_pose_valid;
    fp32 bench_hold_target_leg_m;
    fp32 bench_hold_target_foot_x_m;
    fp32 pitch_auto_offset_rad;
    fp32 pitch_trim_v_lpf_mps;
    WheelLegFootPoint foot_test_target[WHEELLEG_SIDE_COUNT];
    fp32 foot_test_wheel_zero_rad[WHEELLEG_SIDE_COUNT];
    uint8_t foot_test_wheel_zero_valid[WHEELLEG_SIDE_COUNT];
    fp32 foot_test_wheel_dx_m[WHEELLEG_SIDE_COUNT];
    fp32 foot_test_wheel_comp_rad[WHEELLEG_SIDE_COUNT];
    fp32 foot_test_wheel_target_rad[WHEELLEG_SIDE_COUNT];
    uint16_t feedback_faults;
    uint16_t domain_faults;
    uint8_t domain_online_mask;
    uint8_t domain_binding_valid;
    uint8_t manual_on;
    uint8_t recovery_input_safe;
    uint8_t domain_outputs_active;
    uint8_t domain_inhibit_complete;
    uint8_t domain_stop_clear_complete;
    uint8_t fault_mgr_ready;
    uint8_t single_test_last;
    MotorId single_test_target_last;
    FaultAction domain_last_action;
    uint32_t domain_stop_count;
    uint32_t domain_stop_fail_count;
    uint32_t domain_inhibit_fail_count;
    uint32_t domain_inhibit_release_fail_count;
    uint32_t domain_last_stop_tick_ms;
    WheelLegOutputPlan output_plan;
} WheelLegMitCtrl;

typedef struct
{
    uint32_t loop_start_ms;
    uint32_t now_ms;
    uint16_t period_ms;
    fp32 dt;
    fp32 gyro_aligned[3];
    const fp32 *gyro_wheelleg;
    fp32 pitch;
    fp32 roll;
    fp32 yaw;
    int16_t vx_axis;
    int16_t yaw_axis;
    int16_t single_test_axis;
    int16_t leg_axis;
    MotorId single_test_target;
    ManualInputSnapshot manual_input_snapshot;
    uint8_t control_stage;
    uint8_t profile_on;
    uint8_t manual_input_valid;
    uint8_t chassis_switch;
    uint8_t manual_on;
    uint8_t imu_ok;
    uint8_t single_test;
    uint8_t single_test_member;
    uint8_t single_test_target_valid;
    uint8_t single_test_target_online;
    uint8_t left_leg_test;
    uint8_t foot_test;
    uint8_t operation_test_active;
    uint8_t enabled;
    uint8_t controller_active;
    uint8_t kinematics_ok;
    uint8_t recovery_input_safe;
    uint8_t domain_recovery_pending;
    uint8_t domain_recovered_now;
    FaultAction domain_action;
    uint16_t faults;
    uint16_t feedback_faults;
    uint16_t domain_faults;
    fp32 target_v;
    fp32 target_yaw_rate;
    fp32 target_leg;
    fp32 target_foot_x;
    fp32 target_foot_y;
    fp32 wheel_torque[WHEELLEG_SIDE_COUNT];
    ControlOutputPermit output_permit;
    uint8_t control_allowed;
} WheelLegTaskFrame;

typedef struct
{
    fp32 theta_err;
    fp32 d_theta;
    fp32 x_err;
    fp32 v_err;
    fp32 pitch_err;
    fp32 gyro_y;
} WheelLegLqrSideState;

static WheelLegMitCtrl s_wheelleg;
static uint8_t s_wheelleg_sdlog_config_logged = 0u;
static uint32_t s_wheelleg_sdlog_last_status_ms = 0u;

static uint8_t WheelLegFeedbackFresh(MotorId id, const MotorState *fb, uint32_t now_ms);
static void WheelLegSdLogWriteMotorDiag(uint32_t now_ms);

static fp32 WheelLegAxisToFp32(int16_t axis, fp32 max_abs, uint16_t deadband);
static fp32 WheelLegLqrXError(fp32 target_v, fp32 target_yaw_rate);
static fp32 WheelLegLqrPitchOffsetForSide(uint8_t side);
static void WheelLegPitchTrimUpdate(fp32 dt,
                                       int16_t vx_axis,
                                       int16_t yaw_axis,
                                       fp32 target_v,
                                       fp32 target_yaw_rate);
static MotorId WheelLegSingleTestActuator(void);
static void WheelLegResetDomainControlState(void);

uint8_t WheelLegMitGetFootTestPhase(void)
{
    return s_wheelleg.foot_test_phase;
}

uint8_t WheelLegMitGetFootTestIkOk(void)
{
    return s_wheelleg.foot_test_ik_ok;
}

void WheelLegMitGetFootTestTarget(uint8_t side, fp32 *x_m, fp32 *y_m, fp32 *length_m)
{
    const WheelLegFootPoint zero = {0.0f, 0.0f, 0.0f};
    const WheelLegFootPoint *target =
        (side < WHEELLEG_SIDE_COUNT) ? &s_wheelleg.foot_test_target[side] : &zero;

    if (x_m != NULL)
    {
        *x_m = target->x_m;
    }
    if (y_m != NULL)
    {
        *y_m = target->y_m;
    }
    if (length_m != NULL)
    {
        *length_m = target->length_m;
    }
}

void WheelLegMitGetFootTestWheel(uint8_t side,
                                      uint8_t *zero_valid,
                                      fp32 *zero_rad,
                                      fp32 *dx_m,
                                      fp32 *comp_rad,
                                      fp32 *target_rad)
{
    if (side >= WHEELLEG_SIDE_COUNT)
    {
        if (zero_valid != NULL)
        {
            *zero_valid = 0u;
        }
        if (zero_rad != NULL)
        {
            *zero_rad = 0.0f;
        }
        if (dx_m != NULL)
        {
            *dx_m = 0.0f;
        }
        if (comp_rad != NULL)
        {
            *comp_rad = 0.0f;
        }
        if (target_rad != NULL)
        {
            *target_rad = 0.0f;
        }
        return;
    }

    if (zero_valid != NULL)
    {
        *zero_valid = s_wheelleg.foot_test_wheel_zero_valid[side];
    }
    if (zero_rad != NULL)
    {
        *zero_rad = s_wheelleg.foot_test_wheel_zero_rad[side];
    }
    if (dx_m != NULL)
    {
        *dx_m = s_wheelleg.foot_test_wheel_dx_m[side];
    }
    if (comp_rad != NULL)
    {
        *comp_rad = s_wheelleg.foot_test_wheel_comp_rad[side];
    }
    if (target_rad != NULL)
    {
        *target_rad = s_wheelleg.foot_test_wheel_target_rad[side];
    }
}

#include "WheelLegMitMathHelpers.inc"

#include "WheelLegMitSdlog.inc"
#include "WheelLegMitIoHelpers.inc"

#include "WheelLegMitControlHelpers.inc"
#include "WheelLegMitRuntimeHelpers.inc"
#include "WheelLegMitPositionHelpers.inc"

#include "WheelLegMitControlLoop.inc"

#include "WheelLegMitFrameRunner.inc"

void WheelLegMitTask(void const *pvParameters)
{
    TickType_t last_wake;

    (void)pvParameters;
    memset(&s_wheelleg, 0, sizeof(s_wheelleg));
    s_wheelleg.single_test_target_last = MotorCount;
    WheelLegTargetSmoothReset();
    osDelay(g_config.WheelLegMit.task_init_time_ms);
    WheelLegConfigureActuators();
    s_wheelleg.fault_mgr_ready =
        (uint8_t)(FaultMgrInit(&s_wheelleg.fault_mgr, &s_wheelleg_fault_config) == FaultMgrResultOk);
    s_wheelleg.domain_last_action = FaultActionRun;
    last_wake = xTaskGetTickCount();

    while (1)
    {
        const uint64_t profiler_start_us = RtProfBegin();
        WheelLegTaskFrame frame;

        WheelLegTaskFrameInit(&frame);
        if (frame.control_allowed == 0u)
        {
            WheelLegHandleDisabledFrame(&frame);
            WheelLegTaskFinishFrame(&frame, frame.faults, profiler_start_us, &last_wake, 0u);
            continue;
        }

        if (frame.enabled == 0u)
        {
            WheelLegHandleDisabledFrame(&frame);
            WheelLegTaskFinishFrame(&frame, frame.faults, profiler_start_us, &last_wake, 0u);
            continue;
        }

        if (WheelLegOutputBegin() == 0u)
        {
            frame.faults |= WHEELLEG_FAULT_CONTROLLER;
            WheelLegHandleDisabledFrame(&frame);
            WheelLegTaskFinishFrame(&frame, frame.faults, profiler_start_us, &last_wake, 0u);
            continue;
        }

        if (WheelLegRunOperationTest(&frame) != 0u)
        {
            (void)WheelLegCommitFrameOutput(&frame);
            WheelLegTaskFinishFrame(&frame, frame.faults, profiler_start_us, &last_wake, 1u);
            continue;
        }

        WheelLegRunBalanceFrame(&frame);
        (void)WheelLegCommitFrameOutput(&frame);
        WheelLegTaskFinishFrame(&frame,
                                   (uint16_t)(frame.faults | frame.feedback_faults),
                                   profiler_start_us,
                                   &last_wake,
                                   1u);
    }
}

#endif
