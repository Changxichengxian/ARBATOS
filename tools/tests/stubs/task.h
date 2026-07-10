#ifndef HOST_TEST_TASK_H
#define HOST_TEST_TASK_H

#include "FreeRTOS.h"

TaskHandle_t xTaskGetCurrentTaskHandle(void);
uint32_t ulTaskNotifyTake(uint32_t clearOnExit, uint32_t waitTicks);

#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL()  ((void)0)

#endif
