/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ARBATOS_ZEPHYR_FREERTOS_TASK_H
#define ARBATOS_ZEPHYR_FREERTOS_TASK_H

#include "FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

TickType_t xTaskGetTickCount(void);
TickType_t xTaskGetTickCountFromISR(void);
TaskHandle_t xTaskGetCurrentTaskHandle(void);
TaskHandle_t xTaskGetHandle(const char *name);
TaskHandle_t xTaskGetIdleTaskHandle(void);
const char *pcTaskGetName(TaskHandle_t task);
const char *pcTaskGetTaskName(TaskHandle_t task);
UBaseType_t uxTaskGetNumberOfTasks(void);
BaseType_t xTaskGetSchedulerState(void);

void vTaskDelay(TickType_t ticks);
void vTaskDelayUntil(TickType_t *last_wake, TickType_t increment);
void vTaskSuspendAll(void);
BaseType_t xTaskResumeAll(void);

UBaseType_t uxTaskGetStackHighWaterMark(TaskHandle_t task);

TaskHandle_t xTaskCreateStatic(TaskFunction_t entry,
                               const char *name,
                               uint32_t stack_depth,
                               void *argument,
                               UBaseType_t priority,
                               StackType_t *stack,
                               StaticTask_t *task_buffer);

BaseType_t xTaskNotify(TaskHandle_t task, uint32_t value, eNotifyAction action);
BaseType_t xTaskNotifyFromISR(TaskHandle_t task,
                             uint32_t value,
                             eNotifyAction action,
                             BaseType_t *higher_priority_task_woken);
BaseType_t xTaskNotifyWait(uint32_t clear_on_entry,
                           uint32_t clear_on_exit,
                           uint32_t *value,
                           TickType_t timeout);
void vTaskNotifyGiveFromISR(TaskHandle_t task, BaseType_t *higher_priority_task_woken);
uint32_t ulTaskNotifyTake(BaseType_t clear_on_exit, TickType_t timeout);

#ifdef __cplusplus
}
#endif

#endif
