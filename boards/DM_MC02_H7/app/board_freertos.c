
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os2.h"

#include "INS_task.h"
#include "can_feedback_rx_task.h"
#include "can_command_tx_task.h"
#include "arm_task.h"
#include "battery_monitor_task.h"
#include "chassis_control_task.h"
#include "detect_task.h"
#include "gimbal_control_task.h"
#include "rc_sbus_task.h"
#include "referee_rx_task.h"
#include "sdlog_task.h"
#include "robot_task_profile.h"
#include "control_manager.h"
#include "watch.h"
#include "wheelleg_mit_task.h"

typedef osThreadId_t (*app_task_create_fn_t)(void);

typedef struct
{
    robot_task_module_e module;
    osThreadId_t *handle;
    app_task_create_fn_t create;
} app_task_module_desc_t;

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

APP_THREAD_ATTR(defaultTask, osPriorityNormal, 512);
APP_THREAD_ATTR(rcSbusTask, osPriorityAboveNormal, 256);
APP_THREAD_ATTR(refereeRxTask, osPriorityNormal, 256);
APP_THREAD_ATTR(healthMonitorTask, osPriorityNormal, 256);
APP_THREAD_ATTR(sdlogTask, osPriorityLow, 512);
APP_THREAD_ATTR(batteryMonitorTask, osPriorityLow, 256);
APP_THREAD_ATTR(canCommandTxTask, osPriorityAboveNormal, 256);
APP_THREAD_ATTR(canFeedbackRxTask, osPriorityHigh, 256);
APP_THREAD_ATTR(armTask, osPriorityNormal, 768);
APP_THREAD_ATTR(chassisControlTask, osPriorityAboveNormal, 512);
APP_THREAD_ATTR(wheellegMitTask, osPriorityAboveNormal, 768);
APP_THREAD_ATTR(gimbalControlTask, osPriorityHigh, 1024);
APP_THREAD_ATTR(imuFusionTask, osPriorityRealtime, 512);

static osThreadId_t app_create_imu_task(void)
{
    return APP_THREAD_CREATE(imuFusionTask, imu_fusion_task);
}

static osThreadId_t app_create_rc_sbus_task(void)
{
    return APP_THREAD_CREATE(rcSbusTask, rc_sbus_task);
}

static osThreadId_t app_create_referee_rx_task(void)
{
    return APP_THREAD_CREATE(refereeRxTask, referee_rx_task);
}

static osThreadId_t app_create_health_monitor_task(void)
{
    return APP_THREAD_CREATE(healthMonitorTask, health_monitor_task);
}

static osThreadId_t app_create_sdlog_task(void)
{
    return APP_THREAD_CREATE(sdlogTask, sdlog_task);
}

static osThreadId_t app_create_battery_monitor_task(void)
{
    return APP_THREAD_CREATE(batteryMonitorTask, battery_monitor_task);
}

static osThreadId_t app_create_can_command_tx_task(void)
{
    return APP_THREAD_CREATE(canCommandTxTask, can_command_tx_task);
}

static osThreadId_t app_create_can_feedback_rx_task(void)
{
    return APP_THREAD_CREATE(canFeedbackRxTask, can_feedback_rx_task);
}

static osThreadId_t app_create_arm_task(void)
{
    return APP_THREAD_CREATE(armTask, arm_task);
}

static osThreadId_t app_create_chassis_control_task(void)
{
    return APP_THREAD_CREATE(chassisControlTask, chassis_control_task);
}

static osThreadId_t app_create_wheelleg_mit_task(void)
{
    return APP_THREAD_CREATE(wheellegMitTask, wheelleg_mit_task);
}

static osThreadId_t app_create_single_gimbal_task(void)
{
    return APP_THREAD_CREATE(gimbalControlTask, gimbal_control_task);
}

static osThreadId_t app_create_dual_yaw_gimbal_task(void)
{
    return APP_THREAD_CREATE(gimbalControlTask, dual_yaw_gimbal_control_task);
}

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
        {ROBOT_TASK_MODULE_IMU, &imuTaskHandle, app_create_imu_task},
        {ROBOT_TASK_MODULE_RC_SBUS, &rcSbusTaskHandle, app_create_rc_sbus_task},
        {ROBOT_TASK_MODULE_REFEREE_RX, &refereeRxTaskHandle, app_create_referee_rx_task},
        {ROBOT_TASK_MODULE_HEALTH_MONITOR, &detectTaskHandle, app_create_health_monitor_task},
        {ROBOT_TASK_MODULE_SDLOG, &sdlogTaskHandle, app_create_sdlog_task},
        {ROBOT_TASK_MODULE_BATTERY_MONITOR, &batteryMonitorTaskHandle, app_create_battery_monitor_task},
        {ROBOT_TASK_MODULE_CAN_COMMAND_TX, &canCommandTxTaskHandle, app_create_can_command_tx_task},
        {ROBOT_TASK_MODULE_CAN_FEEDBACK_RX, &canFeedbackRxTaskHandle, app_create_can_feedback_rx_task},
        {ROBOT_TASK_MODULE_ARM, &armTaskHandle, app_create_arm_task},
#ifndef CARRIER_DIRECT_ARM_BRINGUP
        {ROBOT_TASK_MODULE_CLASSIC_CHASSIS, &chassisControlTaskHandle, app_create_chassis_control_task},
        {ROBOT_TASK_MODULE_WHEELLEG_MIT, &wheellegMitTaskHandle, app_create_wheelleg_mit_task},
        {ROBOT_TASK_MODULE_SINGLE_GIMBAL, &gimbalControlTaskHandle, app_create_single_gimbal_task},
        {ROBOT_TASK_MODULE_DUAL_YAW_GIMBAL, &gimbalControlTaskHandle, app_create_dual_yaw_gimbal_task},
#endif
    };

    app_clear_module_task_handles();

    for (uint32_t i = 0u; i < (uint32_t)(sizeof(module_tasks) / sizeof(module_tasks[0])); i++)
    {
        const app_task_module_desc_t *task = &module_tasks[i];
        if (robot_profile_module_enabled(task->module) == 0u || task->handle == NULL || task->create == NULL)
        {
            continue;
        }
        if (*task->handle != NULL)
        {
            continue;
        }
        *task->handle = task->create();
    }
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

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;

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
