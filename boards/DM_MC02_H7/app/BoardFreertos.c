
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os2.h"

#include "RobotConfig.h"
#include "RobotTaskBuildConfig.h"

#if ROBOT_TASK_BUILD_IMU
#include "InsTask.h"
#endif
#if ROBOT_TASK_BUILD_CAN_FEEDBACK_RX
#include "CanRxTask.h"
#endif
#if ROBOT_TASK_BUILD_CAN_COMMAND_TX
#include "CanTxTask.h"
#endif
#if ROBOT_TASK_BUILD_ARM
#include "ArmTask.h"
#endif
#if ROBOT_TASK_BUILD_BATTERY_MONITOR
#include "BatteryMonitorTask.h"
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
#if ROBOT_TASK_BUILD_RC_SBUS
#include "RcSbusTask.h"
#endif
#if ROBOT_TASK_BUILD_REFEREE_RX
#include "RefereeRxTask.h"
#endif
#if ROBOT_TASK_BUILD_SDLOG
#include "SdLogTask.h"
#endif
#include "AppTaskBootstrap.h"
#include "ControlMgr.h"
#include "RobotControlRegistry.h"
#include "Watch.h"
#if ROBOT_TASK_BUILD_WHEELLEG_MIT
#include "WheelLegMitTask.h"
#endif

osThreadId_t defaultTaskHandle;
osThreadId_t rcSbusTaskHandle;
osThreadId_t detectTaskHandle;
osThreadId_t sdlogTaskHandle;
osThreadId_t batteryMonitorTaskHandle;
osThreadId_t canFeedbackRxTaskHandle;
osThreadId_t canCommandTxTaskHandle;
osThreadId_t armTaskHandle;
osThreadId_t chassisControlTaskHandle;
osThreadId_t wheellegMitTaskHandle;
osThreadId_t gimbalControlTaskHandle;
osThreadId_t imuTaskHandle;
osThreadId_t refereeRxTaskHandle;

void StartDefaultTask(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void);

#define APP_THREAD_ATTR(thread_id, prio, stack_words) \
    static const osThreadAttr_t thread_id##_attr = { \
        .name = #thread_id, \
        .priority = (prio), \
        .stack_size = (stack_words) * sizeof(StackType_t), \
    }

#define APP_THREAD_CREATE(thread_id, entry) \
    osThreadNew((osThreadFunc_t)(entry), NULL, &thread_id##_attr)

APP_THREAD_ATTR(defaultTask, osPriorityNormal, 1024);
#if ROBOT_TASK_BUILD_RC_SBUS
APP_THREAD_ATTR(rcSbusTask, osPriorityAboveNormal, 512);
#endif
#if ROBOT_TASK_BUILD_REFEREE_RX
APP_THREAD_ATTR(refereeRxTask, osPriorityNormal, 256);
#endif
#if ROBOT_TASK_BUILD_HEALTH_MONITOR
APP_THREAD_ATTR(healthMonitorTask, osPriorityNormal, 256);
#endif
#if ROBOT_TASK_BUILD_SDLOG
APP_THREAD_ATTR(sdlogTask, osPriorityLow, 512);
#endif
#if ROBOT_TASK_BUILD_BATTERY_MONITOR
APP_THREAD_ATTR(batteryMonitorTask, osPriorityLow, 256);
#endif
#if ROBOT_TASK_BUILD_CAN_COMMAND_TX
APP_THREAD_ATTR(canCommandTxTask, osPriorityAboveNormal, 512);
#endif
#if ROBOT_TASK_BUILD_CAN_FEEDBACK_RX
APP_THREAD_ATTR(canFeedbackRxTask, osPriorityHigh, 512);
#endif
#if ROBOT_TASK_BUILD_ARM
APP_THREAD_ATTR(armTask, osPriorityNormal, 768);
#endif
#if ROBOT_TASK_BUILD_CLASSIC_CHASSIS
APP_THREAD_ATTR(chassisControlTask, osPriorityAboveNormal, 1024);
#endif
#if ROBOT_TASK_BUILD_WHEELLEG_MIT
APP_THREAD_ATTR(wheellegMitTask, osPriorityAboveNormal, 768);
#endif
#if ROBOT_TASK_BUILD_ANY_GIMBAL
APP_THREAD_ATTR(gimbalControlTask, osPriorityHigh, 1024);
#endif
#if ROBOT_TASK_BUILD_IMU
APP_THREAD_ATTR(imuFusionTask, osPriorityRealtime, 768);
#endif

#if ROBOT_TASK_BUILD_IMU
static osThreadId_t AppCreateImuTask(void)
{
    return APP_THREAD_CREATE(imuFusionTask, ImuFusionTask);
}
#endif

#if ROBOT_TASK_BUILD_RC_SBUS
static osThreadId_t AppCreateRcSbusTask(void)
{
    return APP_THREAD_CREATE(rcSbusTask, RcSbusTask);
}
#endif

#if ROBOT_TASK_BUILD_REFEREE_RX
static osThreadId_t AppCreateRefereeRxTask(void)
{
    return APP_THREAD_CREATE(refereeRxTask, RefereeRxTask);
}
#endif

#if ROBOT_TASK_BUILD_HEALTH_MONITOR
static osThreadId_t AppCreateHealthMonitorTask(void)
{
    return APP_THREAD_CREATE(healthMonitorTask, HealthMonitorTask);
}
#endif

#if ROBOT_TASK_BUILD_SDLOG
static osThreadId_t AppCreateSdLogTask(void)
{
    return APP_THREAD_CREATE(sdlogTask, SdLogTask);
}
#endif

#if ROBOT_TASK_BUILD_BATTERY_MONITOR
static osThreadId_t AppCreateBatteryMonitorTask(void)
{
    return APP_THREAD_CREATE(batteryMonitorTask, BatteryMonitorTask);
}
#endif

#if ROBOT_TASK_BUILD_CAN_COMMAND_TX
static osThreadId_t AppCreateCanTxTask(void)
{
    return APP_THREAD_CREATE(canCommandTxTask, CanTxTask);
}
#endif

#if ROBOT_TASK_BUILD_CAN_FEEDBACK_RX
static osThreadId_t AppCreateCanRxTask(void)
{
    return APP_THREAD_CREATE(canFeedbackRxTask, CanRxTask);
}
#endif

#if ROBOT_TASK_BUILD_ARM
static osThreadId_t AppCreateArmTask(void)
{
    return APP_THREAD_CREATE(armTask, ArmTask);
}
#endif

#if ROBOT_TASK_BUILD_CLASSIC_CHASSIS
static osThreadId_t AppCreateChassisControlTask(void)
{
    return APP_THREAD_CREATE(chassisControlTask, ChassisControlTask);
}
#endif

#if ROBOT_TASK_BUILD_WHEELLEG_MIT
static osThreadId_t AppCreateWheelLegMitTask(void)
{
    return APP_THREAD_CREATE(wheellegMitTask, WheelLegMitTask);
}
#endif

#if ROBOT_TASK_BUILD_SINGLE_GIMBAL
static osThreadId_t AppCreateSingleGimbalTask(void)
{
    return APP_THREAD_CREATE(gimbalControlTask, GimbalControlTask);
}
#endif

#if ROBOT_TASK_BUILD_DUAL_YAW_GIMBAL
static osThreadId_t AppCreateDualYawGimbalTask(void)
{
    return APP_THREAD_CREATE(gimbalControlTask, DualYawGimbalControlTask);
}
#endif

static void AppCreateModuleTasks(void)
{
    static const AppTaskModuleDesc module_tasks[] =
    {
#if ROBOT_TASK_BUILD_IMU
        {ROBOT_TASK_MODULE_IMU, &imuTaskHandle, AppCreateImuTask},
#endif
#if ROBOT_TASK_BUILD_RC_SBUS
        {ROBOT_TASK_MODULE_RC_SBUS, &rcSbusTaskHandle, AppCreateRcSbusTask},
#endif
#if ROBOT_TASK_BUILD_REFEREE_RX
        {ROBOT_TASK_MODULE_REFEREE_RX, &refereeRxTaskHandle, AppCreateRefereeRxTask},
#endif
#if ROBOT_TASK_BUILD_HEALTH_MONITOR
        {ROBOT_TASK_MODULE_HEALTH_MONITOR, &detectTaskHandle, AppCreateHealthMonitorTask},
#endif
#if ROBOT_TASK_BUILD_SDLOG
        {ROBOT_TASK_MODULE_SDLOG, &sdlogTaskHandle, AppCreateSdLogTask},
#endif
#if ROBOT_TASK_BUILD_BATTERY_MONITOR
        {ROBOT_TASK_MODULE_BATTERY_MONITOR, &batteryMonitorTaskHandle, AppCreateBatteryMonitorTask},
#endif
#if ROBOT_TASK_BUILD_CAN_COMMAND_TX
        {ROBOT_TASK_MODULE_CAN_COMMAND_TX, &canCommandTxTaskHandle, AppCreateCanTxTask},
#endif
#if ROBOT_TASK_BUILD_CAN_FEEDBACK_RX
        {ROBOT_TASK_MODULE_CAN_FEEDBACK_RX, &canFeedbackRxTaskHandle, AppCreateCanRxTask},
#endif
#if ROBOT_TASK_BUILD_ARM
        {ROBOT_TASK_MODULE_ARM, &armTaskHandle, AppCreateArmTask},
#endif
#ifndef CARRIER_DIRECT_ARM_BRINGUP
#if ROBOT_TASK_BUILD_CLASSIC_CHASSIS
        {ROBOT_TASK_MODULE_CLASSIC_CHASSIS, &chassisControlTaskHandle, AppCreateChassisControlTask},
#endif
#if ROBOT_TASK_BUILD_WHEELLEG_MIT
        {ROBOT_TASK_MODULE_WHEELLEG_MIT, &wheellegMitTaskHandle, AppCreateWheelLegMitTask},
#endif
#if ROBOT_TASK_BUILD_SINGLE_GIMBAL
        {ROBOT_TASK_MODULE_SINGLE_GIMBAL, &gimbalControlTaskHandle, AppCreateSingleGimbalTask},
#endif
#if ROBOT_TASK_BUILD_DUAL_YAW_GIMBAL
        {ROBOT_TASK_MODULE_DUAL_YAW_GIMBAL, &gimbalControlTaskHandle, AppCreateDualYawGimbalTask},
#endif
#endif
    };

    AppCreateEnabledModuleTasks(module_tasks, (uint32_t)(sizeof(module_tasks) / sizeof(module_tasks[0])));
}

static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   uint32_t *pulIdleTaskStackSize)
{
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
    *ppxIdleTaskStackBuffer = &xIdleStack[0];
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
                                    StackType_t **ppxTimerTaskStackBuffer,
                                    uint32_t *pulTimerTaskStackSize)
{
    *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
    *ppxTimerTaskStackBuffer = &xTimerStack[0];
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

void MX_FREERTOS_Init(void)
{
    RobotControlBootstrapProfileDefaults();

    defaultTaskHandle = APP_THREAD_CREATE(defaultTask, StartDefaultTask);

    AppCreateModuleTasks();

}

void StartDefaultTask(void *argument)
{
    (void)argument;
    WatchInit();
    WatchDiagSetBootStage(WATCH_BOOT_STAGE_DEFAULT_TASK_START);

    MX_USB_DEVICE_Init();
    WatchDiagSetBootStage(WATCH_BOOT_STAGE_USB_DEVICE_START);
    WatchDiagSetBootStage(WATCH_BOOT_STAGE_RUN);

    for (;;)
    {
        WatchTaskBeat(WATCH_TASK_DEFAULT);
        WatchUpdate();
        osDelay(10);
    }
}

static const char *AppTaskNameFromHandle(TaskHandle_t task, const char *fallback_name)
{
    if (task == (TaskHandle_t)defaultTaskHandle)
    {
        return "defaultTask";
    }
#if ROBOT_TASK_BUILD_RC_SBUS
    if (task == (TaskHandle_t)rcSbusTaskHandle)
    {
        return "rcSbusTask";
    }
#endif
#if ROBOT_TASK_BUILD_REFEREE_RX
    if (task == (TaskHandle_t)refereeRxTaskHandle)
    {
        return "refereeRxTask";
    }
#endif
#if ROBOT_TASK_BUILD_HEALTH_MONITOR
    if (task == (TaskHandle_t)detectTaskHandle)
    {
        return "healthMonitorTask";
    }
#endif
#if ROBOT_TASK_BUILD_SDLOG
    if (task == (TaskHandle_t)sdlogTaskHandle)
    {
        return "sdlogTask";
    }
#endif
#if ROBOT_TASK_BUILD_BATTERY_MONITOR
    if (task == (TaskHandle_t)batteryMonitorTaskHandle)
    {
        return "batteryMonitorTask";
    }
#endif
#if ROBOT_TASK_BUILD_CAN_COMMAND_TX
    if (task == (TaskHandle_t)canCommandTxTaskHandle)
    {
        return "canCommandTxTask";
    }
#endif
#if ROBOT_TASK_BUILD_CAN_FEEDBACK_RX
    if (task == (TaskHandle_t)canFeedbackRxTaskHandle)
    {
        return "canFeedbackRxTask";
    }
#endif
#if ROBOT_TASK_BUILD_ARM
    if (task == (TaskHandle_t)armTaskHandle)
    {
        return "armTask";
    }
#endif
#if ROBOT_TASK_BUILD_CLASSIC_CHASSIS
    if (task == (TaskHandle_t)chassisControlTaskHandle)
    {
        return "chassisControlTask";
    }
#endif
#if ROBOT_TASK_BUILD_WHEELLEG_MIT
    if (task == (TaskHandle_t)wheellegMitTaskHandle)
    {
        return "wheellegMitTask";
    }
#endif
#if ROBOT_TASK_BUILD_ANY_GIMBAL
    if (task == (TaskHandle_t)gimbalControlTaskHandle)
    {
        return "gimbalControlTask";
    }
#endif
#if ROBOT_TASK_BUILD_IMU
    if (task == (TaskHandle_t)imuTaskHandle)
    {
        return "imuFusionTask";
    }
#endif

    if (fallback_name != NULL && fallback_name[0] != '\0')
    {
        return fallback_name;
    }
    return "?";
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    const char *task_name = AppTaskNameFromHandle(xTask, pcTaskName);
    WatchDiagMarkFatal(1u, (uint32_t)(uintptr_t)xTask, task_name);

    if ((CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) != 0U)
    {
        __BKPT(0);
    }

    taskDISABLE_INTERRUPTS();
    for (;;)
    {
        __NOP();
    }
}

void vApplicationMallocFailedHook(void)
{
    TaskHandle_t task = xTaskGetCurrentTaskHandle();
    const char *task_name = AppTaskNameFromHandle(task, pcTaskGetName(task));
    WatchDiagMarkFatal(2u, (uint32_t)(uintptr_t)task, task_name);

    if ((CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) != 0U)
    {
        __BKPT(0);
    }

    taskDISABLE_INTERRUPTS();
    for (;;)
    {
        __NOP();
    }
}
