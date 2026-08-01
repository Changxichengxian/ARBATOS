/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>

#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/linker/section_tags.h>

#include "FreeRTOS.h"
#include "cmsis_os2.h"

#include "ArbatosRuntime.h"
#include "AppTaskBootstrap.h"
#include "BspBuzzer.h"
#include "BspUsb.h"
#include "ManualInput.h"
#include "RobotConfig.h"
#include "RobotControlRegistry.h"
#include "RobotTaskBuildConfig.h"
#include "Watch.h"

#if ROBOT_TASK_BUILD_STARTUP_SERVICE
#include "StartupServiceTask.h"
#endif
#if ROBOT_TASK_BUILD_CALIBRATION
#include "CalibrateTask.h"
#endif
#if ROBOT_TASK_BUILD_CAN_COMMAND_TX
#include "CanTxTask.h"
#endif
#if ROBOT_TASK_BUILD_CAN_FEEDBACK_RX
#include "CanRxTask.h"
#endif
#if ROBOT_TASK_BUILD_CLASSIC_CHASSIS
#include "ChassisControlTask.h"
#endif
#if ROBOT_TASK_BUILD_HEALTH_MONITOR
#include "DetectTask.h"
#endif
#if ROBOT_TASK_BUILD_ANY_GIMBAL
#include "GimbalControlTask.h"
#endif
#if ROBOT_TASK_BUILD_IMU
#include "InsTask.h"
#endif
#if ROBOT_TASK_BUILD_STATUS_LED
#include "StatusLedTask.h"
#endif
#if ROBOT_TASK_BUILD_RC_SBUS
#include "RcSbusTask.h"
#endif
#if ROBOT_TASK_BUILD_REFEREE_RX
#include "RefereeRxTask.h"
#endif
#if ROBOT_TASK_BUILD_HOST_LINK
#include "HostLinkTask.h"
#endif
#if ROBOT_TASK_BUILD_ELRS_LINK
#include "ElrsTask.h"
#endif
#if ROBOT_TASK_BUILD_BATTERY_MONITOR
#include "BatteryMonitorTask.h"
#endif
#if ROBOT_TASK_BUILD_SERVO
#include "ServoControlTask.h"
#endif
#if ROBOT_TASK_BUILD_SDLOG
#include "SdLogTask.h"
#endif
#if ROBOT_TASK_BUILD_ARM
#include "ArmTask.h"
#endif
#if ROBOT_TASK_BUILD_WHEELLEG_MIT
#include "WheelLegMitTask.h"
#endif

#ifndef ROBOT_WATCH_UPDATE_PERIOD_MS
#define ROBOT_WATCH_UPDATE_PERIOD_MS 1000u
#endif

/* H723 的旧工程给总线和控制任务分配了更大的栈，保持各板原有字节数。 */
#if defined(CONFIG_ARBATOS_TARGET_HERO_M) || \
    defined(CONFIG_ARBATOS_TARGET_SENTINEL_M) || \
    defined(CONFIG_ARBATOS_TARGET_MINIWHEELEG_M)
#define ARB_RUNTIME_DEFAULT_STACK_WORDS 1024u
#define ARB_RUNTIME_RC_STACK_WORDS 512u
#define ARB_RUNTIME_REFEREE_STACK_WORDS 256u
#define ARB_RUNTIME_BATTERY_STACK_WORDS 256u
#define ARB_RUNTIME_CAN_TX_STACK_WORDS 512u
#define ARB_RUNTIME_CAN_RX_STACK_WORDS 512u
#define ARB_RUNTIME_CHASSIS_STACK_WORDS 1024u
#define ARB_RUNTIME_WHEELLEG_STACK_WORDS 768u
#define ARB_RUNTIME_ARM_STACK_WORDS 768u
#define ARB_RUNTIME_IMU_STACK_WORDS 768u
#else
#define ARB_RUNTIME_DEFAULT_STACK_WORDS 256u
#define ARB_RUNTIME_RC_STACK_WORDS 256u
#define ARB_RUNTIME_REFEREE_STACK_WORDS 128u
#define ARB_RUNTIME_BATTERY_STACK_WORDS 128u
#define ARB_RUNTIME_CAN_TX_STACK_WORDS 256u
#define ARB_RUNTIME_CAN_RX_STACK_WORDS 256u
#define ARB_RUNTIME_CHASSIS_STACK_WORDS 768u
#define ARB_RUNTIME_WHEELLEG_STACK_WORDS 768u
#define ARB_RUNTIME_ARM_STACK_WORDS 768u
#define ARB_RUNTIME_IMU_STACK_WORDS 1024u
#endif

#if DT_HAS_CHOSEN(zephyr_dtcm)
#define ARB_RUNTIME_STACK_SECTION __dtcm_bss_section
#else
#define ARB_RUNTIME_STACK_SECTION
#endif

#define ARB_STATIC_THREAD(thread_id, prio, stack_words)                                  \
    static struct z_thread_stack_element thread_id##_stack[                              \
        K_KERNEL_STACK_LEN((stack_words) * sizeof(StackType_t))]                         \
        __aligned(Z_KERNEL_STACK_OBJ_ALIGN) ARB_RUNTIME_STACK_SECTION;                    \
    static StaticTask_t thread_id##_control_block;                                       \
    static const osThreadAttr_t thread_id##_attr = {                                     \
        .name = #thread_id,                                                              \
        .priority = (prio),                                                              \
        .stack_mem = thread_id##_stack,                                                  \
        .stack_size = sizeof(thread_id##_stack),                                         \
        .cb_mem = &thread_id##_control_block,                                            \
        .cb_size = sizeof(thread_id##_control_block),                                    \
    }

#define ARB_THREAD_CREATE(thread_id, entry) \
    osThreadNew((osThreadFunc_t)(entry), NULL, &thread_id##_attr)

static osThreadId_t ArbDefaultTaskHandle;
#if ROBOT_TASK_BUILD_STARTUP_SERVICE
static osThreadId_t ArbStartupTaskHandle;
#endif
#if ROBOT_TASK_BUILD_CALIBRATION
static osThreadId_t ArbCalibrationTaskHandle;
#endif
#if ROBOT_TASK_BUILD_CAN_COMMAND_TX
static osThreadId_t ArbCanTxTaskHandle;
#endif
#if ROBOT_TASK_BUILD_CAN_FEEDBACK_RX
static osThreadId_t ArbCanRxTaskHandle;
#endif
#if ROBOT_TASK_BUILD_CLASSIC_CHASSIS
static osThreadId_t ArbChassisTaskHandle;
#endif
#if ROBOT_TASK_BUILD_HEALTH_MONITOR
static osThreadId_t ArbHealthTaskHandle;
#endif
#if ROBOT_TASK_BUILD_ANY_GIMBAL
static osThreadId_t ArbGimbalTaskHandle;
#endif
#if ROBOT_TASK_BUILD_IMU
static osThreadId_t ArbImuTaskHandle;
#endif
#if ROBOT_TASK_BUILD_STATUS_LED
static osThreadId_t ArbStatusLedTaskHandle;
#endif
#if ROBOT_TASK_BUILD_RC_SBUS
static osThreadId_t ArbRcTaskHandle;
#endif
#if ROBOT_TASK_BUILD_REFEREE_RX
static osThreadId_t ArbRefereeTaskHandle;
#endif
#if ROBOT_TASK_BUILD_HOST_LINK
static osThreadId_t ArbHostTaskHandle;
#endif
#if ROBOT_TASK_BUILD_ELRS_LINK
static osThreadId_t ArbElrsTaskHandle;
#endif
#if ROBOT_TASK_BUILD_BATTERY_MONITOR
static osThreadId_t ArbBatteryTaskHandle;
#endif
#if ROBOT_TASK_BUILD_SERVO
static osThreadId_t ArbServoTaskHandle;
#endif
#if ROBOT_TASK_BUILD_SDLOG
static osThreadId_t ArbSdLogTaskHandle;
#endif
#if ROBOT_TASK_BUILD_ARM
static osThreadId_t ArbArmTaskHandle;
#endif
#if ROBOT_TASK_BUILD_WHEELLEG_MIT
static osThreadId_t ArbWheelLegTaskHandle;
#endif

ARB_STATIC_THREAD(defaultTask, osPriorityNormal, ARB_RUNTIME_DEFAULT_STACK_WORDS);
#if ROBOT_TASK_BUILD_STARTUP_SERVICE
ARB_STATIC_THREAD(startupServiceTask, osPriorityNormal, 768u);
#endif
#if ROBOT_TASK_BUILD_CALIBRATION
ARB_STATIC_THREAD(calibrationTask, osPriorityNormal, 512u);
#endif
#if ROBOT_TASK_BUILD_CAN_COMMAND_TX
ARB_STATIC_THREAD(canCommandTxTask, osPriorityAboveNormal, ARB_RUNTIME_CAN_TX_STACK_WORDS);
#endif
#if ROBOT_TASK_BUILD_CAN_FEEDBACK_RX
ARB_STATIC_THREAD(canFeedbackRxTask, osPriorityHigh, ARB_RUNTIME_CAN_RX_STACK_WORDS);
#endif
#if ROBOT_TASK_BUILD_CLASSIC_CHASSIS
ARB_STATIC_THREAD(chassisControlTask, osPriorityAboveNormal, ARB_RUNTIME_CHASSIS_STACK_WORDS);
#endif
#if ROBOT_TASK_BUILD_HEALTH_MONITOR
ARB_STATIC_THREAD(healthMonitorTask, osPriorityNormal, 384u);
#endif
#if ROBOT_TASK_BUILD_ANY_GIMBAL
ARB_STATIC_THREAD(gimbalControlTask, osPriorityHigh, 1024u);
#endif
#if ROBOT_TASK_BUILD_IMU
ARB_STATIC_THREAD(imuFusionTask, osPriorityRealtime, ARB_RUNTIME_IMU_STACK_WORDS);
#endif
#if ROBOT_TASK_BUILD_STATUS_LED
ARB_STATIC_THREAD(statusLedTask, osPriorityNormal, 256u);
#endif
#if ROBOT_TASK_BUILD_RC_SBUS
ARB_STATIC_THREAD(rcSbusTask, osPriorityAboveNormal, ARB_RUNTIME_RC_STACK_WORDS);
#endif
#if ROBOT_TASK_BUILD_REFEREE_RX
ARB_STATIC_THREAD(refereeRxTask, osPriorityNormal, ARB_RUNTIME_REFEREE_STACK_WORDS);
#endif
#if ROBOT_TASK_BUILD_HOST_LINK
ARB_STATIC_THREAD(hostLinkTask, osPriorityNormal, 512u);
#endif
#if ROBOT_TASK_BUILD_ELRS_LINK
ARB_STATIC_THREAD(elrsLinkTask, osPriorityAboveNormal, 256u);
#endif
#if ROBOT_TASK_BUILD_BATTERY_MONITOR
ARB_STATIC_THREAD(batteryMonitorTask, osPriorityNormal, ARB_RUNTIME_BATTERY_STACK_WORDS);
#endif
#if ROBOT_TASK_BUILD_SERVO
ARB_STATIC_THREAD(servoControlTask, osPriorityNormal, 192u);
#endif
#if ROBOT_TASK_BUILD_SDLOG
ARB_STATIC_THREAD(sdlogTask, osPriorityLow, 512u);
#endif
#if ROBOT_TASK_BUILD_ARM
ARB_STATIC_THREAD(armTask, osPriorityNormal, ARB_RUNTIME_ARM_STACK_WORDS);
#endif
#if ROBOT_TASK_BUILD_WHEELLEG_MIT
ARB_STATIC_THREAD(wheellegMitTask, osPriorityAboveNormal, ARB_RUNTIME_WHEELLEG_STACK_WORDS);
#endif

static void ArbDefaultTask(void *argument)
{
    (void)argument;

    WatchInit();
    WatchDiagSetBootStage(WATCH_BOOT_STAGE_DEFAULT_TASK_START);
    WatchDiagSetBootStage(WATCH_BOOT_STAGE_RUN);

    for (;;)
    {
        WatchTaskBeat(WATCH_TASK_DEFAULT);
        WatchUpdate();
        (void)osDelay(ROBOT_WATCH_UPDATE_PERIOD_MS);
    }
}

#if ROBOT_TASK_BUILD_STARTUP_SERVICE
static osThreadId_t ArbCreateStartupTask(void)
{
    return ARB_THREAD_CREATE(startupServiceTask, StartupServiceTask);
}
#endif
#if ROBOT_TASK_BUILD_CALIBRATION
static osThreadId_t ArbCreateCalibrationTask(void)
{
    return ARB_THREAD_CREATE(calibrationTask, CalibrateTask);
}
#endif
#if ROBOT_TASK_BUILD_CAN_COMMAND_TX
static osThreadId_t ArbCreateCanTxTask(void)
{
    return ARB_THREAD_CREATE(canCommandTxTask, CanTxTask);
}
#endif
#if ROBOT_TASK_BUILD_CAN_FEEDBACK_RX
static osThreadId_t ArbCreateCanRxTask(void)
{
    return ARB_THREAD_CREATE(canFeedbackRxTask, CanRxTask);
}
#endif
#if ROBOT_TASK_BUILD_CLASSIC_CHASSIS
static osThreadId_t ArbCreateChassisTask(void)
{
    return ARB_THREAD_CREATE(chassisControlTask, ChassisControlTask);
}
#endif
#if ROBOT_TASK_BUILD_HEALTH_MONITOR
static osThreadId_t ArbCreateHealthTask(void)
{
    return ARB_THREAD_CREATE(healthMonitorTask, HealthMonitorTask);
}
#endif
#if ROBOT_TASK_BUILD_SINGLE_GIMBAL
static osThreadId_t ArbCreateSingleGimbalTask(void)
{
    return ARB_THREAD_CREATE(gimbalControlTask, GimbalControlTask);
}
#endif
#if ROBOT_TASK_BUILD_DUAL_YAW_GIMBAL
static osThreadId_t ArbCreateDualYawGimbalTask(void)
{
    return ARB_THREAD_CREATE(gimbalControlTask, DualYawGimbalControlTask);
}
#endif
#if ROBOT_TASK_BUILD_IMU
static osThreadId_t ArbCreateImuTask(void)
{
    return ARB_THREAD_CREATE(imuFusionTask, ImuFusionTask);
}
#endif
#if ROBOT_TASK_BUILD_STATUS_LED
static osThreadId_t ArbCreateStatusLedTask(void)
{
    return ARB_THREAD_CREATE(statusLedTask, StatusLedTask);
}
#endif
#if ROBOT_TASK_BUILD_RC_SBUS
static osThreadId_t ArbCreateRcTask(void)
{
    return ARB_THREAD_CREATE(rcSbusTask, RcSbusTask);
}
#endif
#if ROBOT_TASK_BUILD_REFEREE_RX
static osThreadId_t ArbCreateRefereeTask(void)
{
    return ARB_THREAD_CREATE(refereeRxTask, RefereeRxTask);
}
#endif
#if ROBOT_TASK_BUILD_HOST_LINK
static osThreadId_t ArbCreateHostTask(void)
{
    return ARB_THREAD_CREATE(hostLinkTask, HostLinkTask);
}
#endif
#if ROBOT_TASK_BUILD_ELRS_LINK
static osThreadId_t ArbCreateElrsTask(void)
{
    return ARB_THREAD_CREATE(elrsLinkTask, ElrsLinkTask);
}
#endif
#if ROBOT_TASK_BUILD_BATTERY_MONITOR
static osThreadId_t ArbCreateBatteryTask(void)
{
    return ARB_THREAD_CREATE(batteryMonitorTask, BatteryMonitorTask);
}
#endif
#if ROBOT_TASK_BUILD_SERVO
static osThreadId_t ArbCreateServoTask(void)
{
    return ARB_THREAD_CREATE(servoControlTask, ServoControlTask);
}
#endif
#if ROBOT_TASK_BUILD_SDLOG
static osThreadId_t ArbCreateSdLogTask(void)
{
    return ARB_THREAD_CREATE(sdlogTask, SdLogTask);
}
#endif
#if ROBOT_TASK_BUILD_ARM
static osThreadId_t ArbCreateArmTask(void)
{
    return ARB_THREAD_CREATE(armTask, ArmTask);
}
#endif
#if ROBOT_TASK_BUILD_WHEELLEG_MIT
static osThreadId_t ArbCreateWheelLegTask(void)
{
    return ARB_THREAD_CREATE(wheellegMitTask, WheelLegMitTask);
}
#endif

static uint8_t ArbCreateModuleTasks(void)
{
    const AppTaskModuleDesc moduleTasks[] = {
#if ROBOT_TASK_BUILD_STARTUP_SERVICE
        APP_TASK_MODULE_DESC(ROBOT_TASK_MODULE_STARTUP_SERVICE, &ArbStartupTaskHandle, ArbCreateStartupTask),
#endif
#if ROBOT_TASK_BUILD_CALIBRATION
        APP_TASK_MODULE_DESC(ROBOT_TASK_MODULE_CALIBRATION, &ArbCalibrationTaskHandle, ArbCreateCalibrationTask),
#endif
#if ROBOT_TASK_BUILD_CAN_COMMAND_TX
        APP_TASK_MODULE_DESC(ROBOT_TASK_MODULE_CAN_COMMAND_TX, &ArbCanTxTaskHandle, ArbCreateCanTxTask),
#endif
#if ROBOT_TASK_BUILD_CAN_FEEDBACK_RX
        APP_TASK_MODULE_DESC(ROBOT_TASK_MODULE_CAN_FEEDBACK_RX, &ArbCanRxTaskHandle, ArbCreateCanRxTask),
#endif
#if ROBOT_TASK_BUILD_CLASSIC_CHASSIS
        APP_TASK_MODULE_DESC(ROBOT_TASK_MODULE_CLASSIC_CHASSIS, &ArbChassisTaskHandle, ArbCreateChassisTask),
#endif
#if ROBOT_TASK_BUILD_HEALTH_MONITOR
        APP_TASK_MODULE_DESC(ROBOT_TASK_MODULE_HEALTH_MONITOR, &ArbHealthTaskHandle, ArbCreateHealthTask),
#endif
#if ROBOT_TASK_BUILD_SINGLE_GIMBAL
        APP_TASK_MODULE_DESC(ROBOT_TASK_MODULE_SINGLE_GIMBAL, &ArbGimbalTaskHandle, ArbCreateSingleGimbalTask),
#endif
#if ROBOT_TASK_BUILD_DUAL_YAW_GIMBAL
        APP_TASK_MODULE_DESC(ROBOT_TASK_MODULE_DUAL_YAW_GIMBAL, &ArbGimbalTaskHandle, ArbCreateDualYawGimbalTask),
#endif
#if ROBOT_TASK_BUILD_IMU
        APP_TASK_MODULE_DESC(ROBOT_TASK_MODULE_IMU, &ArbImuTaskHandle, ArbCreateImuTask),
#endif
#if ROBOT_TASK_BUILD_STATUS_LED
        APP_TASK_MODULE_DESC(ROBOT_TASK_MODULE_STATUS_LED, &ArbStatusLedTaskHandle, ArbCreateStatusLedTask),
#endif
#if ROBOT_TASK_BUILD_RC_SBUS
        APP_TASK_MODULE_DESC(ROBOT_TASK_MODULE_RC_SBUS, &ArbRcTaskHandle, ArbCreateRcTask),
#endif
#if ROBOT_TASK_BUILD_REFEREE_RX
        APP_TASK_MODULE_DESC(ROBOT_TASK_MODULE_REFEREE_RX, &ArbRefereeTaskHandle, ArbCreateRefereeTask),
#endif
#if ROBOT_TASK_BUILD_HOST_LINK
        APP_TASK_MODULE_DESC(ROBOT_TASK_MODULE_HOST_LINK, &ArbHostTaskHandle, ArbCreateHostTask),
#endif
#if ROBOT_TASK_BUILD_ELRS_LINK
        APP_TASK_MODULE_DESC(ROBOT_TASK_MODULE_ELRS_LINK, &ArbElrsTaskHandle, ArbCreateElrsTask),
#endif
#if ROBOT_TASK_BUILD_BATTERY_MONITOR
        APP_TASK_MODULE_DESC(ROBOT_TASK_MODULE_BATTERY_MONITOR, &ArbBatteryTaskHandle, ArbCreateBatteryTask),
#endif
#if ROBOT_TASK_BUILD_SERVO
        APP_TASK_MODULE_DESC(ROBOT_TASK_MODULE_SERVO, &ArbServoTaskHandle, ArbCreateServoTask),
#endif
#if ROBOT_TASK_BUILD_SDLOG
        APP_TASK_MODULE_DESC(ROBOT_TASK_MODULE_SDLOG, &ArbSdLogTaskHandle, ArbCreateSdLogTask),
#endif
#if ROBOT_TASK_BUILD_ARM
        APP_TASK_MODULE_DESC(ROBOT_TASK_MODULE_ARM, &ArbArmTaskHandle, ArbCreateArmTask),
#endif
#if ROBOT_TASK_BUILD_WHEELLEG_MIT
        APP_TASK_MODULE_DESC(ROBOT_TASK_MODULE_WHEELLEG_MIT, &ArbWheelLegTaskHandle, ArbCreateWheelLegTask),
#endif
        {ROBOT_TASK_MODULE_NONE, NULL, NULL, NULL},
    };

    return AppCreateEnabledModuleTasks(moduleTasks,
                                       (uint32_t)(sizeof(moduleTasks) / sizeof(moduleTasks[0])));
}

ArbatosRuntimeStatus ArbatosRuntimeStart(void)
{
    static uint8_t attempted;
    static ArbatosRuntimeStatus result = ARBATOS_RUNTIME_OK;

    if (attempted != 0u)
    {
        return (result == ARBATOS_RUNTIME_OK) ? ARBATOS_RUNTIME_ALREADY_STARTED : result;
    }
    attempted = 1u;

    if (ArbatosPlatformInit() != 0)
    {
        result = ARBATOS_RUNTIME_PLATFORM_INIT_FAILED;
        return result;
    }

    RobotControlBootstrapProfileDefaults();
    ManualInputInit();
    BuzzerSetEnable(1u);

#if ROBOT_TASK_BUILD_CALIBRATION
    cali_param_init();
#endif

#if ROBOT_TASK_BUILD_HOST_LINK
    /* HostLink/VisionLink uses USB CDC even when StartupServiceTask is absent. */
    BspUsbDeviceInit();
#endif

    ArbDefaultTaskHandle = ARB_THREAD_CREATE(defaultTask, ArbDefaultTask);
    if (ArbDefaultTaskHandle == NULL)
    {
        result = ARBATOS_RUNTIME_DEFAULT_TASK_FAILED;
        return result;
    }
    if (ArbCreateModuleTasks() != 0u)
    {
        result = ARBATOS_RUNTIME_MODULE_TASK_FAILED;
        return result;
    }

    return result;
}

const char *ArbatosRuntimeStatusName(ArbatosRuntimeStatus status)
{
    switch (status)
    {
        case ARBATOS_RUNTIME_OK:
            return "ok";
        case ARBATOS_RUNTIME_ALREADY_STARTED:
            return "already-started";
        case ARBATOS_RUNTIME_PLATFORM_INIT_FAILED:
            return "platform-init-failed";
        case ARBATOS_RUNTIME_DEFAULT_TASK_FAILED:
            return "default-task-failed";
        case ARBATOS_RUNTIME_MODULE_TASK_FAILED:
            return "module-task-failed";
        default:
            return "unknown";
    }
}
