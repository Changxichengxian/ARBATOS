/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ARBATOS_ZEPHYR_FREERTOS_TIMERS_H
#define ARBATOS_ZEPHYR_FREERTOS_TIMERS_H

#include "FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

TimerHandle_t xTimerCreateStatic(const char *name,
                                 TickType_t period,
                                 BaseType_t auto_reload,
                                 void *timer_id,
                                 TimerCallbackFunction_t callback,
                                 StaticTimer_t *timer_buffer);
BaseType_t xTimerStart(TimerHandle_t timer, TickType_t command_timeout);
BaseType_t xTimerStop(TimerHandle_t timer, TickType_t command_timeout);
void *pvTimerGetTimerID(TimerHandle_t timer);

#ifdef __cplusplus
}
#endif

#endif
