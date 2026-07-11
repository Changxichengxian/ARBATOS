#ifndef HOST_TEST_TASK_H
#define HOST_TEST_TASK_H

#include "FreeRTOS.h"

TaskHandle_t xTaskGetCurrentTaskHandle(void);
uint32_t ulTaskNotifyTake(uint32_t clearOnExit, uint32_t waitTicks);

#ifndef taskENTER_CRITICAL
#define taskENTER_CRITICAL() ((void)0)
#endif
#ifndef taskEXIT_CRITICAL
#define taskEXIT_CRITICAL()  ((void)0)
#endif

#endif
