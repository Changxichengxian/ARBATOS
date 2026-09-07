/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ARBATOS_ZEPHYR_CMSIS_OS2_H
#define ARBATOS_ZEPHYR_CMSIS_OS2_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define osWaitForever 0xFFFFFFFFu

typedef enum
{
    osPriorityNone = 0,
    osPriorityIdle = 1,
    osPriorityLow = 8,
    osPriorityLow1 = 9,
    osPriorityLow2 = 10,
    osPriorityLow3 = 11,
    osPriorityLow4 = 12,
    osPriorityLow5 = 13,
    osPriorityLow6 = 14,
    osPriorityLow7 = 15,
    osPriorityBelowNormal = 16,
    osPriorityBelowNormal1 = 17,
    osPriorityBelowNormal2 = 18,
    osPriorityBelowNormal3 = 19,
    osPriorityBelowNormal4 = 20,
    osPriorityBelowNormal5 = 21,
    osPriorityBelowNormal6 = 22,
    osPriorityBelowNormal7 = 23,
    osPriorityNormal = 24,
    osPriorityNormal1 = 25,
    osPriorityNormal2 = 26,
    osPriorityNormal3 = 27,
    osPriorityNormal4 = 28,
    osPriorityNormal5 = 29,
    osPriorityNormal6 = 30,
    osPriorityNormal7 = 31,
    osPriorityAboveNormal = 32,
    osPriorityAboveNormal1 = 33,
    osPriorityAboveNormal2 = 34,
    osPriorityAboveNormal3 = 35,
    osPriorityAboveNormal4 = 36,
    osPriorityAboveNormal5 = 37,
    osPriorityAboveNormal6 = 38,
    osPriorityAboveNormal7 = 39,
    osPriorityHigh = 40,
    osPriorityHigh1 = 41,
    osPriorityHigh2 = 42,
    osPriorityHigh3 = 43,
    osPriorityHigh4 = 44,
    osPriorityHigh5 = 45,
    osPriorityHigh6 = 46,
    osPriorityHigh7 = 47,
    osPriorityRealtime = 48,
    osPriorityRealtime1 = 49,
    osPriorityRealtime2 = 50,
    osPriorityRealtime3 = 51,
    osPriorityRealtime4 = 52,
    osPriorityRealtime5 = 53,
    osPriorityRealtime6 = 54,
    osPriorityRealtime7 = 55,
    osPriorityISR = 56,
    osPriorityError = -1
} osPriority_t;

typedef enum
{
    osOK = 0,
    osError = -1,
    osErrorTimeout = -2,
    osErrorResource = -3,
    osErrorParameter = -4,
    osErrorNoMemory = -5,
    osErrorISR = -6
} osStatus_t;

typedef enum
{
    osKernelInactive = 0,
    osKernelReady = 1,
    osKernelRunning = 2,
    osKernelLocked = 3,
    osKernelSuspended = 4,
    osKernelError = -1
} osKernelState_t;

typedef void (*osThreadFunc_t)(void *argument);
typedef void *osThreadId_t;
typedef void *osMutexId_t;

typedef struct
{
    const char *name;
    uint32_t attr_bits;
    void *cb_mem;
    uint32_t cb_size;
    void *stack_mem;
    uint32_t stack_size;
    osPriority_t priority;
    uint32_t tz_module;
    uint32_t reserved;
} osThreadAttr_t;

typedef struct
{
    const char *name;
    uint32_t attr_bits;
    void *cb_mem;
    uint32_t cb_size;
} osMutexAttr_t;

osStatus_t osKernelInitialize(void);
osStatus_t osKernelStart(void);
osKernelState_t osKernelGetState(void);

osThreadId_t osThreadNew(osThreadFunc_t function,
                         void *argument,
                         const osThreadAttr_t *attributes);
osThreadId_t osThreadGetId(void);
const char *osThreadGetName(osThreadId_t thread_id);
osStatus_t osThreadYield(void);
osStatus_t osDelay(uint32_t ticks);

osMutexId_t osMutexNew(const osMutexAttr_t *attributes);
osStatus_t osMutexAcquire(osMutexId_t mutex_id, uint32_t timeout);
osStatus_t osMutexRelease(osMutexId_t mutex_id);

#ifdef __cplusplus
}
#endif

#endif
