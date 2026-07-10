#ifndef MANUAL_INPUT_TEST_TIMERS_H
#define MANUAL_INPUT_TEST_TIMERS_H

#include "FreeRTOS.h"

typedef void (*TimerCallbackFunction_t)(TimerHandle_t timer);

TimerHandle_t xTimerCreateStatic(const char *name,
                                 TickType_t period,
                                 uint32_t auto_reload,
                                 void *timer_id,
                                 TimerCallbackFunction_t callback,
                                 StaticTimer_t *buffer);
uint32_t xTimerStart(TimerHandle_t timer, TickType_t wait_ticks);

#endif
