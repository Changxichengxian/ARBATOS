/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "Watch.h"

#include <string.h>

#include "RobotConfig.h"
#include "RobotTaskBuildConfig.h"
#include "main.h"

#include "FreeRTOS.h"
#include "task.h"

#include "CanReceive.h"
#include "CanTxTask.h"
#include "LowCmd.h"
#include "ArmTask.h"
#include "BatteryMonitorTask.h"
#include "BspAdc.h"
#include "ChassisBehaviour.h"
#include "ChassisState.h"
#include "ControlMgr.h"
#include "ControlInput.h"
#include "GimbalControlTask.h"
#include "GimbalState.h"
#include "InsTask.h"
#include "Bmi088Driver.h"
#include "DetectTask.h"
#include "MemMang.h"
#include "ManualInput.h"
#include "BspCan.h"
#include "BspRc.h"
#include "MotorConfig.h"
#include "MotorInst.h"
#include "RobotLifecycle.h"
#include "SdCard.h"
#include "SdLog.h"
#include "ShootState.h"
#include "HostLinkTask.h"
#include "ExternalMotionIntent.h"
#include "RobotDeviceConfig.h"
#include "RobotModule.h"
#include "RobotTaskProfile.h"
#include "RobotMode.h"
#include "RtProf.h"
#include "VisionLink.h"
#include "WheelLegMitTask.h"
#include "WheelLegMsg.h"

Watch g_watch;

typedef struct
{
    uint8_t used;
    uint8_t module;
    uint8_t create_state;
    uint8_t reserved0;
    uint16_t create_fail_count;
    uint16_t reserved1;
    uint32_t thread_handle;
    uint32_t create_attempt_count;
    const char *name;
} WatchTaskModuleCreateSlot;

static WatchTaskModuleCreateSlot s_task_module_create[WATCH_RUNTIME_MAX_TASK_MODULES];

#if WATCH_ENABLE_SHOOT_RM
static const char *const s_watch_friction_motor_names[4u] = {
    "motor.friction0",
    "motor.friction1",
    "motor.friction2",
    "motor.friction3",
};
static MotorId s_watch_friction_motor_ids[4u] = {MotorCount, MotorCount, MotorCount, MotorCount};
static uint8_t s_watch_friction_motor_ids_ready = 0u;

static void WatchPrepareMotorIds(void)
{
    if (s_watch_friction_motor_ids_ready != 0u)
    {
        return;
    }

    (void)MotorInstResolveIds(s_watch_friction_motor_names,
                                              4u,
                                              s_watch_friction_motor_ids,
                                              4u);
    for (uint8_t i = 0u; i < 4u; i++)
    {
        if (s_watch_friction_motor_ids[i] == MotorCount)
        {
            s_watch_friction_motor_ids[i] = MotorIdRange(Motor8, i, 4u);
        }
    }
    s_watch_friction_motor_ids_ready = 1u;
}

static MotorId WatchFrictionMotorId(uint8_t index)
{
    index = (uint8_t)(index & 0x03u);
    if (s_watch_friction_motor_ids_ready == 0u)
    {
        return MotorIdRange(Motor8, index, 4u);
    }

    return s_watch_friction_motor_ids[index];
}
#endif

// Some targets do not compile ArmTask.c at all. Keep watch linkable there and
// let real ArmTask.c override this fallback when the Unitree executor is used.
__weak const ArmJ0UnitreeState *ArmJ0UnitreeGetState(void)
{
    return NULL;
}

__weak uint8_t WheelLegMitGetFootTestPhase(void)
{
    return 0u;
}

__weak uint8_t WheelLegMitGetFootTestIkOk(void)
{
    return 0u;
}

__weak void WheelLegMitGetFootTestTarget(uint8_t side, fp32 *x_m, fp32 *y_m, fp32 *length_m)
{
    (void)side;
    if (x_m != NULL)
    {
        *x_m = 0.0f;
    }
    if (y_m != NULL)
    {
        *y_m = 0.0f;
    }
    if (length_m != NULL)
    {
        *length_m = 0.0f;
    }
}

__weak void WheelLegMitGetFootTestWheel(uint8_t side,
                                             uint8_t *zero_valid,
                                             fp32 *zero_rad,
                                             fp32 *dx_m,
                                             fp32 *comp_rad,
                                             fp32 *target_rad)
{
    (void)side;
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
}

__weak fp32 ins_get_imu_temperature_c(void)
{
    return 0.0f;
}

__weak uint16_t ins_get_imu_heater_pwm(void)
{
    return 0u;
}

__weak uint8_t ins_get_imu_heater_mode(void)
{
    return 0u;
}

__weak fp32 ins_get_imu_heater_pid_out(void)
{
    return 0.0f;
}

__weak void BMI088_get_diag(bmi088_diag_t *out)
{
    if (out != NULL)
    {
        memset(out, 0, sizeof(*out));
        out->init_last_error = 0xEEu;
        out->init_fail_reg = 0xEEu;
        out->init_fail_expect = 0xEEu;
        out->init_fail_actual = 0xEEu;
        out->accel_chip_id = 0xEEu;
        out->gyro_chip_id = 0xEEu;
        out->gyro_read_chip_id = 0xEEu;
    }
}

__weak fp32 get_battery_voltage_cached(void)
{
    return 0.0f;
}

__weak fp32 get_battery_percentage_fp32(void)
{
    return 0.0f;
}

__weak uint8_t BatteryMonitorIsLowAlarm(void)
{
    return 0u;
}

__weak uint8_t BspAdcIsStarted(void)
{
    return 0u;
}

__weak uint16_t BspAdcGetRaw(uint8_t index)
{
    (void)index;
    return 0u;
}

__weak fp32 BspAdcGetChannelVoltage(uint8_t index)
{
    (void)index;
    return 0.0f;
}

__weak uint32_t BspAdcGetStartOkCount(void)
{
    return 0u;
}

__weak uint32_t BspAdcGetStartFailCount(void)
{
    return 0u;
}

static ManualInputState rc_snapshot;
static const ManualInputState *rc_src;
static const fp32 *ins_quat_src;
static const fp32 *ins_angle_src;
static const fp32 *ins_gyro_src;
static const fp32 *ins_accel_src;

static void WatchCopyRc(void);
static void WatchCopyNewrc(void);
#if WATCH_ENABLE_COMM_COPY
static void WatchCopyComm(void);
#endif
#if WATCH_ENABLE_RUNTIME_COPY
static void WatchCopyRuntime(void);
#endif
static void WatchCopyImu(void);
#if WATCH_ENABLE_LOCOMOTION_CLASSIC
static void WatchCopyChassis(void);
#endif
#if WATCH_ENABLE_GIMBAL_SINGLE
static void WatchCopyGimbal(void);
#endif
#if WATCH_ENABLE_GIMBAL_DUAL
static void WatchCopyDualGimbal(void);
#endif
#if WATCH_ENABLE_SHOOT_RM
static void WatchCopyShoot(void);
#endif
#if WATCH_ENABLE_ARM_J0_UNITREE
static void WatchCopyArmJ0Unitree(void);
#endif
#if WATCH_ENABLE_LOCOMOTION_WHEELLEG_MIT
static void WatchCopyWheelLegMit(void);
#endif
static void WatchCopyDiag(void);
static void WatchCopyRtos(void);
static void WatchDiagPushStage(WatchBootStage stage);
#if WATCH_ENABLE_RUNTIME_COPY
static void WatchRuntimeAddEntry(const char *name,
                                    RuntimeInstanceKind kind,
                                    RuntimeInstanceState state,
                                    uint16_t source_id,
                                    uint16_t source_index,
                                    uint16_t parent_index);
static RuntimeInstanceState WatchRuntimeDeviceState(const RobotConfigDevice *device);
static const WatchTaskModuleCreateSlot *WatchTaskModuleCreateFind(uint8_t module);
#endif
static WatchTaskDiagEntry *WatchTaskDiagGet(WatchTaskId task_id);
static WatchIrqDiagEntry *WatchIrqDiagGet(WatchIrqId irq_id);
static uint8_t WatchBlockActiveAlways(void);
#if WATCH_ENABLE_LOCOMOTION_CLASSIC
static uint8_t WatchBlockActiveLocomotionClassic(void);
#endif
#if WATCH_ENABLE_LOCOMOTION_WHEELLEG_SERVO
static uint8_t WatchBlockActiveWheelLegServo(void);
#endif
#if WATCH_ENABLE_LOCOMOTION_WHEELLEG_MIT
static uint8_t WatchBlockActiveWheelLegMit(void);
#endif
#if WATCH_ENABLE_GIMBAL_SINGLE
static uint8_t WatchBlockActiveGimbalSingle(void);
#endif
#if WATCH_ENABLE_GIMBAL_DUAL
static uint8_t WatchBlockActiveGimbalDual(void);
#endif
#if WATCH_ENABLE_SHOOT_RM
static uint8_t WatchBlockActiveShootRm(void);
#endif
#if WATCH_ENABLE_ARM_J0_UNITREE
static uint8_t WatchBlockActiveArm(void);
#endif

static const WatchBlockDesc g_watch_blocks[] = {
    {WATCH_BLOCK_RC, "input.rc", &g_watch.rc, sizeof(g_watch.rc), WatchBlockActiveAlways},
    {WATCH_BLOCK_NEWRC, "input.newrc", &g_watch.newrc, sizeof(g_watch.newrc), WatchBlockActiveAlways},
#if WATCH_ENABLE_RUNTIME_COPY
    {WATCH_BLOCK_RUNTIME, "runtime.instances", &g_watch.runtime, sizeof(g_watch.runtime), WatchBlockActiveAlways},
#endif
#if WATCH_ENABLE_LOCOMOTION_CLASSIC
    {WATCH_BLOCK_LOCOMOTION_CLASSIC, "locomotion.classic", &g_watch.chassis, sizeof(g_watch.chassis), WatchBlockActiveLocomotionClassic},
#endif
#if WATCH_ENABLE_LOCOMOTION_WHEELLEG_SERVO
    {WATCH_BLOCK_LOCOMOTION_WHEELLEG_SERVO, "locomotion.WheelLegServo", &g_watch.WheelLegServo, sizeof(g_watch.WheelLegServo), WatchBlockActiveWheelLegServo},
#endif
#if WATCH_ENABLE_LOCOMOTION_WHEELLEG_MIT
    {WATCH_BLOCK_LOCOMOTION_WHEELLEG_MIT, "locomotion.WheelLegMit", &g_watch.WheelLegMit, sizeof(g_watch.WheelLegMit), WatchBlockActiveWheelLegMit},
#endif
#if WATCH_ENABLE_GIMBAL_SINGLE
    {WATCH_BLOCK_GIMBAL_SINGLE, "gimbal.single", &g_watch.gimbal, sizeof(g_watch.gimbal), WatchBlockActiveGimbalSingle},
#endif
#if WATCH_ENABLE_GIMBAL_DUAL
    {WATCH_BLOCK_GIMBAL_DUAL, "gimbal.dual", &g_watch.dual_gimbal, sizeof(g_watch.dual_gimbal), WatchBlockActiveGimbalDual},
#endif
#if WATCH_ENABLE_SHOOT_RM
    {WATCH_BLOCK_SHOOT_RM, "shoot.rm", &g_watch.shoot, sizeof(g_watch.shoot), WatchBlockActiveShootRm},
#endif
#if WATCH_ENABLE_ARM_J0_UNITREE
    {WATCH_BLOCK_ARM_J0_UNITREE, "arm.j0_unitree", &g_watch.ArmJ0Unitree, sizeof(g_watch.ArmJ0Unitree), WatchBlockActiveArm},
#endif
    {WATCH_BLOCK_IMU, "common.imu", &g_watch.imu, sizeof(g_watch.imu), WatchBlockActiveAlways},
    {WATCH_BLOCK_DIAG, "common.diag", &g_watch.diag, sizeof(g_watch.diag), WatchBlockActiveAlways},
    {WATCH_BLOCK_RTOS, "common.rtos", &g_watch.rtos, sizeof(g_watch.rtos), WatchBlockActiveAlways},
    {WATCH_BLOCK_FAULT, "common.fault", &g_watch.fault, sizeof(g_watch.fault), WatchBlockActiveAlways},
#if WATCH_ENABLE_COMM_COPY
    {WATCH_BLOCK_COMM, "common.comm", &g_watch.comm, sizeof(g_watch.comm), WatchBlockActiveAlways},
#endif
};

static WatchBlockDesc g_watch_active_blocks[WATCH_BLOCK_COUNT];

#include "WatchCoreHelpers.inc"

#include "WatchRuntimeCopy.inc"

#include "WatchStateCopy.inc"

#include "WatchWheelLegCopy.inc"

#include "WatchDiagCopy.inc"
