/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ARBATOS_ZEPHYR_FREERTOS_H
#define ARBATOS_ZEPHYR_FREERTOS_H

#include <stddef.h>
#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef int32_t BaseType_t;
typedef uint32_t UBaseType_t;
typedef uint32_t TickType_t;
typedef uint32_t StackType_t;

typedef void *TaskHandle_t;
typedef void (*TaskFunction_t)(void *);

typedef struct ArbStaticTask
{
    struct k_thread thread;
    TaskFunction_t entry;
    void *argument;
} StaticTask_t;

typedef struct ArbStaticSemaphore
{
    uint8_t kind;
    union
    {
        struct k_sem semaphore;
        struct k_mutex mutex;
    } object;
} StaticSemaphore_t;

typedef StaticSemaphore_t *SemaphoreHandle_t;

struct ArbStaticTimer;
typedef struct ArbStaticTimer *TimerHandle_t;
typedef void (*TimerCallbackFunction_t)(TimerHandle_t timer);

typedef struct ArbStaticTimer
{
    struct k_timer timer;
    struct k_work work;
    const char *name;
    void *timer_id;
    TimerCallbackFunction_t callback;
    TickType_t period;
    BaseType_t auto_reload;
} StaticTimer_t;

typedef enum
{
    eNoAction = 0,
    eSetBits,
    eIncrement,
    eSetValueWithOverwrite,
    eSetValueWithoutOverwrite
} eNotifyAction;

#define pdFALSE ((BaseType_t)0)
#define pdTRUE ((BaseType_t)1)
#define pdFAIL pdFALSE
#define pdPASS pdTRUE

#define portMAX_DELAY UINT32_MAX
#define portTICK_PERIOD_MS 1u
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))

#define tskIDLE_PRIORITY ((UBaseType_t)0u)
#define taskSCHEDULER_NOT_STARTED ((BaseType_t)0)
#define taskSCHEDULER_RUNNING ((BaseType_t)2)

/*
 * 旧板级代码只用这个值配置可调用 RTOS 接口的中断优先级。
 * Zephyr 接管中断后不再读取它，保留定义用于迁移期编译。
 */
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5u
#define configMINIMAL_STACK_SIZE 128u
#define configTIMER_TASK_STACK_DEPTH 512u

void ArbTaskEnterCritical(void);
void ArbTaskExitCritical(void);
UBaseType_t ArbTaskEnterCriticalFromIsr(void);
void ArbTaskExitCriticalFromIsr(UBaseType_t key);

#define taskENTER_CRITICAL() ArbTaskEnterCritical()
#define taskEXIT_CRITICAL() ArbTaskExitCritical()
#define taskENTER_CRITICAL_FROM_ISR() ArbTaskEnterCriticalFromIsr()
#define taskEXIT_CRITICAL_FROM_ISR(key) ArbTaskExitCriticalFromIsr((key))

/*
 * Zephyr 会在中断退出时自行决定是否调度，不需要显式 PendSV 请求。
 */
#define portYIELD_FROM_ISR(requested) \
    do                                \
    {                                 \
        ARG_UNUSED(requested);        \
    } while (0)

static inline BaseType_t xPortIsInsideInterrupt(void)
{
    return k_is_in_isr() ? pdTRUE : pdFALSE;
}

void *pvPortMalloc(size_t size);
void vPortFree(void *ptr);
size_t xPortGetFreeHeapSize(void);
size_t xPortGetMinimumEverFreeHeapSize(void);

#ifdef __cplusplus
}
#endif

#endif
