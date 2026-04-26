
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os2.h"

#include "INS_task.h"
#include "can_feedback_rx_task.h"
#include "can_command_tx_task.h"
#include "arm_task.h"
#include "chassis_control_task.h"
#include "detect_task.h"
#include "gimbal_control_task.h"
#include "rc_sbus_task.h"
#include "referee_rx_task.h"
#include "sdlog_task.h"
#include "robot_task_profile.h"

osThreadId_t defaultTaskHandle;
osThreadId_t rcSbusTaskHandle;
osThreadId_t detectTaskHandle;
osThreadId_t sdlogTaskHandle;
osThreadId_t canFeedbackRxTaskHandle;
osThreadId_t canCommandTxTaskHandle;
osThreadId_t armTaskHandle;
osThreadId_t chassisControlTaskHandle;
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
APP_THREAD_ATTR(canCommandTxTask, osPriorityAboveNormal, 256);
APP_THREAD_ATTR(canFeedbackRxTask, osPriorityHigh, 256);
APP_THREAD_ATTR(armTask, osPriorityNormal, 768);
APP_THREAD_ATTR(chassisControlTask, osPriorityAboveNormal, 512);
APP_THREAD_ATTR(gimbalControlTask, osPriorityHigh, 1024);
APP_THREAD_ATTR(imuFusionTask, osPriorityRealtime, 1024);

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
    defaultTaskHandle = APP_THREAD_CREATE(defaultTask, StartDefaultTask);

    rcSbusTaskHandle = APP_THREAD_CREATE(rcSbusTask, rc_sbus_task);

    refereeRxTaskHandle = APP_THREAD_CREATE(refereeRxTask, referee_rx_task);

    detectTaskHandle = APP_THREAD_CREATE(healthMonitorTask, health_monitor_task);

    sdlogTaskHandle = APP_THREAD_CREATE(sdlogTask, sdlog_task);

    canCommandTxTaskHandle = APP_THREAD_CREATE(canCommandTxTask, can_command_tx_task);

    canFeedbackRxTaskHandle = APP_THREAD_CREATE(canFeedbackRxTask, can_feedback_rx_task);

    if (robot_profile_need_arm_task())
    {
        armTaskHandle = APP_THREAD_CREATE(armTask, arm_task);
    }
    else
    {
        armTaskHandle = NULL;
    }

#ifndef CARRIER_DIRECT_ARM_BRINGUP
    if (robot_profile_need_classic_chassis_control_task())
    {
        chassisControlTaskHandle = APP_THREAD_CREATE(chassisControlTask, chassis_control_task);
    }
    else
    {
        chassisControlTaskHandle = NULL;
    }

    if (robot_profile_need_single_gimbal_control_task())
    {
        gimbalControlTaskHandle = APP_THREAD_CREATE(gimbalControlTask, gimbal_control_task);
    }
    else
    {
        gimbalControlTaskHandle = NULL;
    }
#endif

    imuTaskHandle = APP_THREAD_CREATE(imuFusionTask, imu_fusion_task);
}

void StartDefaultTask(void *argument)
{
    (void)argument;
    MX_USB_DEVICE_Init();

    for (;;)
    {
        osDelay(1);
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
