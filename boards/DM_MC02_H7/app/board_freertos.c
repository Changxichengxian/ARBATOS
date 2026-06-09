
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os2.h"

#include "config.h"
#include "robot_task_build_config.h"

#if ROBOT_TASK_BUILD_IMU
#include "INS_task.h"
#endif
#if ROBOT_TASK_BUILD_CAN_FEEDBACK_RX
#include "can_feedback_rx_task.h"
#endif
#if ROBOT_TASK_BUILD_CAN_COMMAND_TX
#include "can_command_tx_task.h"
#endif
#if ROBOT_TASK_BUILD_ARM
#include "arm_task.h"
#endif
#if ROBOT_TASK_BUILD_BATTERY_MONITOR
#include "battery_monitor_task.h"
#endif
#if ROBOT_TASK_BUILD_CLASSIC_CHASSIS
#include "chassis_control_task.h"
#endif
#if ROBOT_TASK_BUILD_HEALTH_MONITOR
#include "detect_task.h"
#endif
#if ROBOT_TASK_BUILD_ANY_GIMBAL
#include "gimbal_control_task.h"
#endif
#if ROBOT_TASK_BUILD_RC_SBUS
#include "rc_sbus_task.h"
#endif
#if ROBOT_TASK_BUILD_REFEREE_RX
#include "referee_rx_task.h"
#endif
#if ROBOT_TASK_BUILD_SDLOG
#include "sdlog_task.h"
#endif
#include "app_task_bootstrap.h"
#include "control_manager.h"
#include "robot_control_registry.h"
#include "watch.h"
#if ROBOT_TASK_BUILD_WHEELLEG_MIT
#include "wheelleg_mit_task.h"
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
static osThreadId_t app_create_imu_task(void)
{
    return APP_THREAD_CREATE(imuFusionTask, imu_fusion_task);
}
#endif

#if ROBOT_TASK_BUILD_RC_SBUS
static osThreadId_t app_create_rc_sbus_task(void)
{
    return APP_THREAD_CREATE(rcSbusTask, rc_sbus_task);
}
#endif

#if ROBOT_TASK_BUILD_REFEREE_RX
static osThreadId_t app_create_referee_rx_task(void)
{
    return APP_THREAD_CREATE(refereeRxTask, referee_rx_task);
}
#endif

#if ROBOT_TASK_BUILD_HEALTH_MONITOR
static osThreadId_t app_create_health_monitor_task(void)
{
    return APP_THREAD_CREATE(healthMonitorTask, health_monitor_task);
}
#endif

#if ROBOT_TASK_BUILD_SDLOG
static osThreadId_t app_create_sdlog_task(void)
{
    return APP_THREAD_CREATE(sdlogTask, sdlog_task);
}
#endif

#if ROBOT_TASK_BUILD_BATTERY_MONITOR
static osThreadId_t app_create_battery_monitor_task(void)
{
    return APP_THREAD_CREATE(batteryMonitorTask, battery_monitor_task);
}
#endif

#if ROBOT_TASK_BUILD_CAN_COMMAND_TX
static osThreadId_t app_create_can_command_tx_task(void)
{
    return APP_THREAD_CREATE(canCommandTxTask, can_command_tx_task);
}
#endif

#if ROBOT_TASK_BUILD_CAN_FEEDBACK_RX
static osThreadId_t app_create_can_feedback_rx_task(void)
{
    return APP_THREAD_CREATE(canFeedbackRxTask, can_feedback_rx_task);
}
#endif

#if ROBOT_TASK_BUILD_ARM
static osThreadId_t app_create_arm_task(void)
{
    return APP_THREAD_CREATE(armTask, arm_task);
}
#endif

#if ROBOT_TASK_BUILD_CLASSIC_CHASSIS
static osThreadId_t app_create_chassis_control_task(void)
{
    return APP_THREAD_CREATE(chassisControlTask, chassis_control_task);
}
#endif

#if ROBOT_TASK_BUILD_WHEELLEG_MIT
static osThreadId_t app_create_wheelleg_mit_task(void)
{
    return APP_THREAD_CREATE(wheellegMitTask, wheelleg_mit_task);
}
#endif

#if ROBOT_TASK_BUILD_SINGLE_GIMBAL
static osThreadId_t app_create_single_gimbal_task(void)
{
    return APP_THREAD_CREATE(gimbalControlTask, gimbal_control_task);
}
#endif

#if ROBOT_TASK_BUILD_DUAL_YAW_GIMBAL
static osThreadId_t app_create_dual_yaw_gimbal_task(void)
{
    return APP_THREAD_CREATE(gimbalControlTask, dual_yaw_gimbal_control_task);
}
#endif

static void app_clear_module_task_handles(void)
{
    imuTaskHandle = NULL;
    rcSbusTaskHandle = NULL;
    refereeRxTaskHandle = NULL;
    detectTaskHandle = NULL;
    sdlogTaskHandle = NULL;
    batteryMonitorTaskHandle = NULL;
    canCommandTxTaskHandle = NULL;
    canFeedbackRxTaskHandle = NULL;
    armTaskHandle = NULL;
    chassisControlTaskHandle = NULL;
    wheellegMitTaskHandle = NULL;
    gimbalControlTaskHandle = NULL;
}

static void app_create_module_tasks(void)
{
    static const app_task_module_desc_t module_tasks[] =
    {
#if ROBOT_TASK_BUILD_IMU
        {ROBOT_TASK_MODULE_IMU, &imuTaskHandle, app_create_imu_task},
#endif
#if ROBOT_TASK_BUILD_RC_SBUS
        {ROBOT_TASK_MODULE_RC_SBUS, &rcSbusTaskHandle, app_create_rc_sbus_task},
#endif
#if ROBOT_TASK_BUILD_REFEREE_RX
        {ROBOT_TASK_MODULE_REFEREE_RX, &refereeRxTaskHandle, app_create_referee_rx_task},
#endif
#if ROBOT_TASK_BUILD_HEALTH_MONITOR
        {ROBOT_TASK_MODULE_HEALTH_MONITOR, &detectTaskHandle, app_create_health_monitor_task},
#endif
#if ROBOT_TASK_BUILD_SDLOG
        {ROBOT_TASK_MODULE_SDLOG, &sdlogTaskHandle, app_create_sdlog_task},
#endif
#if ROBOT_TASK_BUILD_BATTERY_MONITOR
        {ROBOT_TASK_MODULE_BATTERY_MONITOR, &batteryMonitorTaskHandle, app_create_battery_monitor_task},
#endif
#if ROBOT_TASK_BUILD_CAN_COMMAND_TX
        {ROBOT_TASK_MODULE_CAN_COMMAND_TX, &canCommandTxTaskHandle, app_create_can_command_tx_task},
#endif
#if ROBOT_TASK_BUILD_CAN_FEEDBACK_RX
        {ROBOT_TASK_MODULE_CAN_FEEDBACK_RX, &canFeedbackRxTaskHandle, app_create_can_feedback_rx_task},
#endif
#if ROBOT_TASK_BUILD_ARM
        {ROBOT_TASK_MODULE_ARM, &armTaskHandle, app_create_arm_task},
#endif
#ifndef CARRIER_DIRECT_ARM_BRINGUP
#if ROBOT_TASK_BUILD_CLASSIC_CHASSIS
        {ROBOT_TASK_MODULE_CLASSIC_CHASSIS, &chassisControlTaskHandle, app_create_chassis_control_task},
#endif
#if ROBOT_TASK_BUILD_WHEELLEG_MIT
        {ROBOT_TASK_MODULE_WHEELLEG_MIT, &wheellegMitTaskHandle, app_create_wheelleg_mit_task},
#endif
#if ROBOT_TASK_BUILD_SINGLE_GIMBAL
        {ROBOT_TASK_MODULE_SINGLE_GIMBAL, &gimbalControlTaskHandle, app_create_single_gimbal_task},
#endif
#if ROBOT_TASK_BUILD_DUAL_YAW_GIMBAL
        {ROBOT_TASK_MODULE_DUAL_YAW_GIMBAL, &gimbalControlTaskHandle, app_create_dual_yaw_gimbal_task},
#endif
#endif
    };

    app_clear_module_task_handles();

    app_create_enabled_module_tasks(module_tasks, (uint32_t)(sizeof(module_tasks) / sizeof(module_tasks[0])));
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
    control_manager_init();
    robot_control_register_profile_defaults();

    defaultTaskHandle = APP_THREAD_CREATE(defaultTask, StartDefaultTask);

    app_create_module_tasks();

}

void StartDefaultTask(void *argument)
{
    (void)argument;
    watch_init();
    watch_diag_set_boot_stage(WATCH_BOOT_STAGE_DEFAULT_TASK_START);

    MX_USB_DEVICE_Init();
    watch_diag_set_boot_stage(WATCH_BOOT_STAGE_USB_DEVICE_START);
    watch_diag_set_boot_stage(WATCH_BOOT_STAGE_RUN);

    for (;;)
    {
        watch_task_beat(WATCH_TASK_DEFAULT);
        watch_update();
        osDelay(10);
    }
}

static const char *app_task_name_from_handle(TaskHandle_t task, const char *fallback_name)
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
    const char *task_name = app_task_name_from_handle(xTask, pcTaskName);
    watch_diag_mark_fatal(1u, (uint32_t)(uintptr_t)xTask, task_name);

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
    const char *task_name = app_task_name_from_handle(task, pcTaskGetName(task));
    watch_diag_mark_fatal(2u, (uint32_t)(uintptr_t)task, task_name);

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
