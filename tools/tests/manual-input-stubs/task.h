#ifndef MANUAL_INPUT_TEST_TASK_H
#define MANUAL_INPUT_TEST_TASK_H

#include "FreeRTOS.h"

typedef int32_t BaseType_t;
typedef void *TaskHandle_t;

typedef enum
{
    eNoAction = 0,
    eSetBits = 1
} eNotifyAction;

TickType_t xTaskGetTickCount(void);
TaskHandle_t xTaskGetCurrentTaskHandle(void);
BaseType_t xTaskNotify(TaskHandle_t task, uint32_t value, eNotifyAction action);
BaseType_t xTaskNotifyWait(uint32_t clearOnEntry,
                           uint32_t clearOnExit,
                           uint32_t *value,
                           TickType_t waitTicks);
BaseType_t xTaskNotifyFromISR(TaskHandle_t task,
                              uint32_t value,
                              eNotifyAction action,
                              BaseType_t *higherPriorityTaskWoken);
void ManualInputTestEnterCritical(void);
void ManualInputTestExitCritical(void);

#define pdFALSE                 ((BaseType_t)0)
#define portMAX_DELAY           ((TickType_t)0xFFFFFFFFu)
#define portYIELD_FROM_ISR(x)   do { (void)(x); } while (0)

#define taskENTER_CRITICAL()           ManualInputTestEnterCritical()
#define taskEXIT_CRITICAL()            ManualInputTestExitCritical()
#define taskENTER_CRITICAL_FROM_ISR()  (ManualInputTestEnterCritical(), (UBaseType_t)0u)
#define taskEXIT_CRITICAL_FROM_ISR(x)  do { (void)(x); ManualInputTestExitCritical(); } while (0)

#endif
