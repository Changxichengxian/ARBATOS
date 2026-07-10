#ifndef MANUAL_INPUT_TEST_TASK_H
#define MANUAL_INPUT_TEST_TASK_H

#include "FreeRTOS.h"

TickType_t xTaskGetTickCount(void);
void ManualInputTestEnterCritical(void);
void ManualInputTestExitCritical(void);

#define taskENTER_CRITICAL()           ManualInputTestEnterCritical()
#define taskEXIT_CRITICAL()            ManualInputTestExitCritical()
#define taskENTER_CRITICAL_FROM_ISR()  (ManualInputTestEnterCritical(), (UBaseType_t)0u)
#define taskEXIT_CRITICAL_FROM_ISR(x)  do { (void)(x); ManualInputTestExitCritical(); } while (0)

#endif
