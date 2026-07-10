#ifndef ROBOT_LIFECYCLE_TEST_TASK_H
#define ROBOT_LIFECYCLE_TEST_TASK_H

#include "FreeRTOS.h"

#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#define taskENTER_CRITICAL_FROM_ISR() ((UBaseType_t)0u)
#define taskEXIT_CRITICAL_FROM_ISR(mask) \
    do                                   \
    {                                    \
        (void)(mask);                    \
    } while (0)

#endif
