/*
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 */

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os2.h"

#include "dm_mit_tool_task.h"
#include "rc_sbus_task.h"

osThreadId_t defaultTaskHandle;
osThreadId_t rcSbusTaskHandle;
osThreadId_t dmMitToolTaskHandle;

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
APP_THREAD_ATTR(dmMitToolTask, osPriorityAboveNormal, 768);

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
    dmMitToolTaskHandle = APP_THREAD_CREATE(dmMitToolTask, dm_mit_tool_task);
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
