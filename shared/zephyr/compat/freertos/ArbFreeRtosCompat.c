/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "timers.h"
#include "cmsis_os2.h"

#include <errno.h>
#include <limits.h>
#include <stddef.h>
#include <string.h>

#include <zephyr/arch/cpu.h>
#include <zephyr/devicetree.h>
#include <zephyr/linker/section_tags.h>
#include <zephyr/sys/atomic.h>

#ifndef CONFIG_ARBATOS_FREERTOS_MAX_TASKS
#define CONFIG_ARBATOS_FREERTOS_MAX_TASKS 32
#endif

#ifndef CONFIG_ARBATOS_THREAD_STACK_POOL_SIZE
#define CONFIG_ARBATOS_THREAD_STACK_POOL_SIZE 65536
#endif

#ifndef CONFIG_ARBATOS_CMSIS_MUTEX_COUNT
#define CONFIG_ARBATOS_CMSIS_MUTEX_COUNT 16
#endif

#ifndef CONFIG_ARBATOS_LEGACY_HEAP_SIZE
#define CONFIG_ARBATOS_LEGACY_HEAP_SIZE 8192
#endif

enum
{
    ARB_SEMAPHORE_BINARY = 1,
    ARB_SEMAPHORE_MUTEX = 2
};

typedef struct
{
    TaskHandle_t handle;
    k_tid_t tid;
    struct k_sem count_notify;
    struct k_sem bits_notify;
    atomic_t notify_bits;
} ArbTaskCompat;

typedef struct
{
    bool used;
    struct k_thread thread;
    osThreadFunc_t entry;
    void *argument;
} ArbCmsisThread;

typedef struct
{
    bool used;
    struct k_mutex mutex;
} ArbCmsisMutex;

#if DT_HAS_CHOSEN(zephyr_dtcm)
#define ARB_CPU_LOCAL_BSS __dtcm_bss_section
#else
#define ARB_CPU_LOCAL_BSS
#endif

static ArbTaskCompat ArbTasks[CONFIG_ARBATOS_FREERTOS_MAX_TASKS]
    ARB_CPU_LOCAL_BSS;
static struct k_spinlock ArbTaskRegistryLock;

static ArbCmsisThread ArbCmsisThreads[CONFIG_ARBATOS_FREERTOS_MAX_TASKS]
    ARB_CPU_LOCAL_BSS;
#if DT_HAS_CHOSEN(zephyr_dtcm)
static uint8_t ArbThreadStackPool[CONFIG_ARBATOS_THREAD_STACK_POOL_SIZE]
    __aligned(Z_KERNEL_STACK_OBJ_ALIGN) __dtcm_bss_section;
#else
static uint8_t ArbThreadStackPool[CONFIG_ARBATOS_THREAD_STACK_POOL_SIZE]
    __aligned(Z_KERNEL_STACK_OBJ_ALIGN);
#endif
static size_t ArbThreadStackOffset;
static struct k_spinlock ArbCmsisCreateLock;

static ArbCmsisMutex ArbCmsisMutexes[CONFIG_ARBATOS_CMSIS_MUTEX_COUNT];
static struct k_spinlock ArbCmsisMutexLock;

static unsigned int ArbCriticalKey;
static uint32_t ArbCriticalDepth;

typedef union
{
    max_align_t alignment;
    size_t requested;
} ArbHeapHeader;

K_HEAP_DEFINE(ArbLegacyHeap, CONFIG_ARBATOS_LEGACY_HEAP_SIZE);
static atomic_t ArbLegacyHeapFree =
    ATOMIC_INIT(CONFIG_ARBATOS_LEGACY_HEAP_SIZE);
static atomic_t ArbLegacyHeapMinimum =
    ATOMIC_INIT(CONFIG_ARBATOS_LEGACY_HEAP_SIZE);

static k_timeout_t ArbTimeoutFromTicks(TickType_t ticks)
{
    if (ticks == portMAX_DELAY)
    {
        return K_FOREVER;
    }
    if (ticks == 0u)
    {
        return K_NO_WAIT;
    }
    return K_TICKS(ticks);
}

static ArbTaskCompat *ArbTaskFindByHandleLocked(TaskHandle_t handle)
{
    for (size_t i = 0u; i < ARRAY_SIZE(ArbTasks); i++)
    {
        if (ArbTasks[i].handle == handle)
        {
            return &ArbTasks[i];
        }
    }
    return NULL;
}

static ArbTaskCompat *ArbTaskFindByTidLocked(k_tid_t tid)
{
    for (size_t i = 0u; i < ARRAY_SIZE(ArbTasks); i++)
    {
        if (ArbTasks[i].handle != NULL && ArbTasks[i].tid == tid)
        {
            return &ArbTasks[i];
        }
    }
    return NULL;
}

static ArbTaskCompat *ArbTaskRegister(TaskHandle_t handle, k_tid_t tid)
{
    ArbTaskCompat *slot = NULL;
    k_spinlock_key_t key = k_spin_lock(&ArbTaskRegistryLock);

    slot = ArbTaskFindByHandleLocked(handle);
    if (slot == NULL)
    {
        for (size_t i = 0u; i < ARRAY_SIZE(ArbTasks); i++)
        {
            if (ArbTasks[i].handle == NULL)
            {
                slot = &ArbTasks[i];
                slot->handle = handle;
                slot->tid = tid;
                k_sem_init(&slot->count_notify, 0u, UINT_MAX);
                k_sem_init(&slot->bits_notify, 0u, 1u);
                atomic_clear(&slot->notify_bits);
                break;
            }
        }
    }

    k_spin_unlock(&ArbTaskRegistryLock, key);
    return slot;
}

static ArbTaskCompat *ArbTaskForHandle(TaskHandle_t handle)
{
    if (handle == NULL)
    {
        handle = xTaskGetCurrentTaskHandle();
    }

    k_spinlock_key_t key = k_spin_lock(&ArbTaskRegistryLock);
    ArbTaskCompat *slot = ArbTaskFindByHandleLocked(handle);
    k_spin_unlock(&ArbTaskRegistryLock, key);

    if (slot != NULL)
    {
        return slot;
    }

    return NULL;
}

void ArbTaskEnterCritical(void)
{
    unsigned int key = irq_lock();

    if (ArbCriticalDepth == 0u)
    {
        ArbCriticalKey = key;
    }
    ArbCriticalDepth++;
}

void ArbTaskExitCritical(void)
{
    __ASSERT_NO_MSG(ArbCriticalDepth != 0u);
    ArbCriticalDepth--;
    if (ArbCriticalDepth == 0u)
    {
        irq_unlock(ArbCriticalKey);
    }
}

UBaseType_t ArbTaskEnterCriticalFromIsr(void)
{
    return (UBaseType_t)irq_lock();
}

void ArbTaskExitCriticalFromIsr(UBaseType_t key)
{
    irq_unlock((unsigned int)key);
}

void *pvPortMalloc(size_t size)
{
    if (size == 0u || k_is_in_isr())
    {
        return NULL;
    }

    ArbHeapHeader *header =
        k_heap_alloc(&ArbLegacyHeap, sizeof(*header) + size, K_NO_WAIT);
    if (header == NULL)
    {
        return NULL;
    }

    header->requested = size;
    atomic_val_t free_now = atomic_sub(&ArbLegacyHeapFree, (atomic_val_t)size) -
                            (atomic_val_t)size;
    atomic_val_t minimum = atomic_get(&ArbLegacyHeapMinimum);
    while (free_now < minimum &&
           !atomic_cas(&ArbLegacyHeapMinimum, minimum, free_now))
    {
        minimum = atomic_get(&ArbLegacyHeapMinimum);
    }
    return (void *)(header + 1);
}

void vPortFree(void *ptr)
{
    if (ptr == NULL || k_is_in_isr())
    {
        return;
    }

    ArbHeapHeader *header = ((ArbHeapHeader *)ptr) - 1;
    size_t requested = header->requested;
    header->requested = 0u;
    atomic_add(&ArbLegacyHeapFree, (atomic_val_t)requested);
    k_heap_free(&ArbLegacyHeap, header);
}

size_t xPortGetFreeHeapSize(void)
{
    return (size_t)MAX(atomic_get(&ArbLegacyHeapFree), 0);
}

size_t xPortGetMinimumEverFreeHeapSize(void)
{
    return (size_t)MAX(atomic_get(&ArbLegacyHeapMinimum), 0);
}

TickType_t xTaskGetTickCount(void)
{
    return (TickType_t)k_uptime_ticks();
}

TickType_t xTaskGetTickCountFromISR(void)
{
    return (TickType_t)k_uptime_ticks();
}

TaskHandle_t xTaskGetCurrentTaskHandle(void)
{
    k_tid_t current = k_current_get();
    k_spinlock_key_t key = k_spin_lock(&ArbTaskRegistryLock);
    ArbTaskCompat *slot = ArbTaskFindByTidLocked(current);
    k_spin_unlock(&ArbTaskRegistryLock, key);

    if (slot != NULL)
    {
        return slot->handle;
    }

    return NULL;
}

TaskHandle_t xTaskGetHandle(const char *name)
{
    if (name == NULL)
    {
        return NULL;
    }

    k_spinlock_key_t key = k_spin_lock(&ArbTaskRegistryLock);
    for (size_t i = 0u; i < ARRAY_SIZE(ArbTasks); i++)
    {
        if (ArbTasks[i].handle == NULL || ArbTasks[i].tid == NULL)
        {
            continue;
        }
        const char *thread_name = k_thread_name_get(ArbTasks[i].tid);
        if (thread_name != NULL && strcmp(thread_name, name) == 0)
        {
            TaskHandle_t handle = ArbTasks[i].handle;
            k_spin_unlock(&ArbTaskRegistryLock, key);
            return handle;
        }
    }
    k_spin_unlock(&ArbTaskRegistryLock, key);
    return NULL;
}

TaskHandle_t xTaskGetIdleTaskHandle(void)
{
    /*
     * Zephyr 的 idle 线程不是 CMSIS 线程。CpuUsage 后续改用线程运行统计；
     * 迁移期返回空，避免把内部内核对象伪装成可通知的任务句柄。
     */
    return NULL;
}

const char *pcTaskGetName(TaskHandle_t task)
{
    if (task == NULL)
    {
        return k_thread_name_get(k_current_get());
    }

    ArbTaskCompat *slot = ArbTaskForHandle(task);
    return (slot == NULL || slot->tid == NULL) ? NULL : k_thread_name_get(slot->tid);
}

const char *pcTaskGetTaskName(TaskHandle_t task)
{
    return pcTaskGetName(task);
}

UBaseType_t uxTaskGetNumberOfTasks(void)
{
    UBaseType_t count = 0u;
    k_spinlock_key_t key = k_spin_lock(&ArbTaskRegistryLock);

    for (size_t i = 0u; i < ARRAY_SIZE(ArbTasks); i++)
    {
        if (ArbTasks[i].handle != NULL && ArbTasks[i].tid != NULL)
        {
            count++;
        }
    }
    k_spin_unlock(&ArbTaskRegistryLock, key);
    return count;
}

BaseType_t xTaskGetSchedulerState(void)
{
    return taskSCHEDULER_RUNNING;
}

void vTaskDelay(TickType_t ticks)
{
    if (ticks == 0u)
    {
        k_yield();
        return;
    }
    (void)k_sleep(K_TICKS(ticks));
}

void vTaskDelayUntil(TickType_t *last_wake, TickType_t increment)
{
    if (last_wake == NULL || increment == 0u)
    {
        return;
    }

    TickType_t next = *last_wake + increment;
    TickType_t now = xTaskGetTickCount();
    int32_t remaining = (int32_t)(next - now);
    *last_wake = next;
    if (remaining > 0)
    {
        (void)k_sleep(K_TICKS((uint32_t)remaining));
    }
}

void vTaskSuspendAll(void)
{
    k_sched_lock();
}

BaseType_t xTaskResumeAll(void)
{
    k_sched_unlock();
    return pdFALSE;
}

UBaseType_t uxTaskGetStackHighWaterMark(TaskHandle_t task)
{
    k_tid_t tid = NULL;
    if (task == NULL)
    {
        tid = k_current_get();
    }
    else
    {
        ArbTaskCompat *slot = ArbTaskForHandle(task);
        if (slot != NULL)
        {
            tid = slot->tid;
        }
    }

    size_t unused = 0u;
    if (tid == NULL || k_thread_stack_space_get(tid, &unused) != 0)
    {
        return 0u;
    }
    return (UBaseType_t)(unused / sizeof(StackType_t));
}

static int ArbTaskPriority(UBaseType_t priority)
{
    uint32_t bounded = MIN(priority, (UBaseType_t)(CONFIG_NUM_PREEMPT_PRIORITIES - 1));
    return (int)(CONFIG_NUM_PREEMPT_PRIORITIES - 1u - bounded);
}

static int ArbCmsisPriority(osPriority_t priority)
{
    int32_t normalized = (priority == osPriorityNone) ? osPriorityNormal : priority;
    normalized = CLAMP(normalized, (int32_t)osPriorityIdle, (int32_t)osPriorityISR);
    uint32_t scaled =
        ((uint32_t)(normalized - osPriorityIdle) *
         (uint32_t)(CONFIG_NUM_PREEMPT_PRIORITIES - 1)) /
        (uint32_t)(osPriorityISR - osPriorityIdle);
    return (int)(CONFIG_NUM_PREEMPT_PRIORITIES - 1u - scaled);
}

static void ArbStaticTaskEntry(void *task_ptr, void *unused1, void *unused2)
{
    ARG_UNUSED(unused1);
    ARG_UNUSED(unused2);
    StaticTask_t *task = (StaticTask_t *)task_ptr;
    task->entry(task->argument);
}

static void ArbCmsisTaskEntry(void *thread_ptr, void *unused1, void *unused2)
{
    ARG_UNUSED(unused1);
    ARG_UNUSED(unused2);
    ArbCmsisThread *thread = (ArbCmsisThread *)thread_ptr;
    thread->entry(thread->argument);
}

osStatus_t osKernelInitialize(void)
{
    return osOK;
}

osStatus_t osKernelStart(void)
{
    return osOK;
}

osKernelState_t osKernelGetState(void)
{
    return osKernelRunning;
}

osThreadId_t osThreadNew(osThreadFunc_t function,
                         void *argument,
                         const osThreadAttr_t *attributes)
{
    if (function == NULL || k_is_in_isr())
    {
        return NULL;
    }

    size_t requested_stack = 1024u;
    size_t stack_object_size = 0u;
    osPriority_t priority = osPriorityNormal;
    const char *name = "arbatos";
    void *provided_stack = NULL;

    if (attributes != NULL)
    {
        if (attributes->stack_size != 0u)
        {
            requested_stack = attributes->stack_size;
        }
        priority = attributes->priority;
        name = (attributes->name == NULL) ? name : attributes->name;
        provided_stack = attributes->stack_mem;
    }

    requested_stack = ROUND_UP(requested_stack, ARCH_STACK_PTR_ALIGN);
    ArbCmsisThread *thread = NULL;
    ArbCmsisThread *reserved_thread = NULL;
    uint8_t *stack = NULL;
    k_spinlock_key_t key = k_spin_lock(&ArbCmsisCreateLock);

    for (size_t i = 0u; i < ARRAY_SIZE(ArbCmsisThreads); i++)
    {
        if (!ArbCmsisThreads[i].used)
        {
            thread = &ArbCmsisThreads[i];
            thread->used = true;
            reserved_thread = thread;
            break;
        }
    }

    if (provided_stack != NULL)
    {
        uintptr_t aligned = ROUND_UP((uintptr_t)provided_stack, Z_KERNEL_STACK_OBJ_ALIGN);
        size_t skipped = (size_t)(aligned - (uintptr_t)provided_stack);
        if (requested_stack <= skipped + K_KERNEL_STACK_RESERVED)
        {
            thread = NULL;
        }
        else
        {
            stack = (uint8_t *)aligned;
            stack_object_size = requested_stack - skipped;
            requested_stack = ROUND_DOWN(
                stack_object_size - K_KERNEL_STACK_RESERVED,
                ARCH_STACK_PTR_ALIGN);
        }
    }
    else if (thread != NULL)
    {
        size_t start = ROUND_UP(ArbThreadStackOffset, Z_KERNEL_STACK_OBJ_ALIGN);
        stack_object_size = K_KERNEL_STACK_LEN(requested_stack);
        if (start + stack_object_size <= sizeof(ArbThreadStackPool))
        {
            stack = &ArbThreadStackPool[start];
            ArbThreadStackOffset = start + stack_object_size;
            requested_stack =
                stack_object_size - K_KERNEL_STACK_RESERVED;
        }
        else
        {
            thread = NULL;
        }
    }

    if (thread == NULL || stack == NULL)
    {
        if (reserved_thread != NULL)
        {
            reserved_thread->used = false;
        }
        k_spin_unlock(&ArbCmsisCreateLock, key);
        return NULL;
    }

    thread->entry = function;
    thread->argument = argument;
    k_spin_unlock(&ArbCmsisCreateLock, key);

    TaskHandle_t handle = (TaskHandle_t)thread;
    k_tid_t tid = k_thread_create(&thread->thread,
                                  (k_thread_stack_t *)stack,
                                  requested_stack,
                                  ArbCmsisTaskEntry,
                                  thread,
                                  NULL,
                                  NULL,
                                  ArbCmsisPriority(priority),
                                  0u,
                                  K_FOREVER);
    if (tid == NULL || ArbTaskRegister(handle, tid) == NULL)
    {
        thread->used = false;
        return NULL;
    }

    (void)k_thread_name_set(tid, name);
    k_thread_start(tid);
    return (osThreadId_t)handle;
}

osThreadId_t osThreadGetId(void)
{
    return (osThreadId_t)xTaskGetCurrentTaskHandle();
}

const char *osThreadGetName(osThreadId_t thread_id)
{
    return pcTaskGetName((TaskHandle_t)thread_id);
}

osStatus_t osThreadYield(void)
{
    k_yield();
    return osOK;
}

osStatus_t osDelay(uint32_t ticks)
{
    if (k_is_in_isr())
    {
        return osErrorISR;
    }
    vTaskDelay((TickType_t)ticks);
    return osOK;
}

osMutexId_t osMutexNew(const osMutexAttr_t *attributes)
{
    ARG_UNUSED(attributes);
    if (k_is_in_isr())
    {
        return NULL;
    }

    k_spinlock_key_t key = k_spin_lock(&ArbCmsisMutexLock);
    for (size_t i = 0u; i < ARRAY_SIZE(ArbCmsisMutexes); i++)
    {
        if (!ArbCmsisMutexes[i].used)
        {
            ArbCmsisMutexes[i].used = true;
            k_mutex_init(&ArbCmsisMutexes[i].mutex);
            k_spin_unlock(&ArbCmsisMutexLock, key);
            return (osMutexId_t)&ArbCmsisMutexes[i];
        }
    }
    k_spin_unlock(&ArbCmsisMutexLock, key);
    return NULL;
}

osStatus_t osMutexAcquire(osMutexId_t mutex_id, uint32_t timeout)
{
    if (mutex_id == NULL || k_is_in_isr())
    {
        return (mutex_id == NULL) ? osErrorParameter : osErrorISR;
    }
    ArbCmsisMutex *mutex = (ArbCmsisMutex *)mutex_id;
    int result = k_mutex_lock(&mutex->mutex, ArbTimeoutFromTicks((TickType_t)timeout));
    return (result == 0) ? osOK : ((result == -EAGAIN) ? osErrorTimeout : osErrorResource);
}

osStatus_t osMutexRelease(osMutexId_t mutex_id)
{
    if (mutex_id == NULL || k_is_in_isr())
    {
        return (mutex_id == NULL) ? osErrorParameter : osErrorISR;
    }
    ArbCmsisMutex *mutex = (ArbCmsisMutex *)mutex_id;
    return (k_mutex_unlock(&mutex->mutex) == 0) ? osOK : osErrorResource;
}

TaskHandle_t xTaskCreateStatic(TaskFunction_t entry,
                               const char *name,
                               uint32_t stack_depth,
                               void *argument,
                               UBaseType_t priority,
                               StackType_t *stack,
                               StaticTask_t *task_buffer)
{
    if (entry == NULL || stack == NULL || task_buffer == NULL || stack_depth == 0u)
    {
        return NULL;
    }

    size_t requested_stack = (size_t)stack_depth * sizeof(StackType_t);
    size_t stack_object_size = K_KERNEL_STACK_LEN(requested_stack);
    size_t start;
    k_spinlock_key_t key = k_spin_lock(&ArbCmsisCreateLock);

    start = ROUND_UP(ArbThreadStackOffset, Z_KERNEL_STACK_OBJ_ALIGN);
    if (start + stack_object_size > sizeof(ArbThreadStackPool))
    {
        k_spin_unlock(&ArbCmsisCreateLock, key);
        return NULL;
    }
    k_thread_stack_t *zephyr_stack =
        (k_thread_stack_t *)&ArbThreadStackPool[start];
    ArbThreadStackOffset = start + stack_object_size;
    k_spin_unlock(&ArbCmsisCreateLock, key);

    size_t stack_size = stack_object_size - K_KERNEL_STACK_RESERVED;
    task_buffer->entry = entry;
    task_buffer->argument = argument;

    TaskHandle_t handle = (TaskHandle_t)task_buffer;
    k_tid_t tid = k_thread_create(&task_buffer->thread,
                                  zephyr_stack,
                                  stack_size,
                                  ArbStaticTaskEntry,
                                  task_buffer,
                                  NULL,
                                  NULL,
                                  ArbTaskPriority(priority),
                                  0u,
                                  K_FOREVER);
    if (tid == NULL || ArbTaskRegister(handle, tid) == NULL)
    {
        return NULL;
    }
    (void)k_thread_name_set(tid, name);
    k_thread_start(tid);
    return handle;
}

static BaseType_t ArbTaskNotifyBits(TaskHandle_t task, uint32_t value, eNotifyAction action)
{
    ArbTaskCompat *slot = ArbTaskForHandle(task);
    if (slot == NULL)
    {
        return pdFAIL;
    }

    switch (action)
    {
        case eSetBits:
            atomic_or(&slot->notify_bits, (atomic_val_t)value);
            break;
        case eIncrement:
            /*
             * ulTaskNotifyTake() consumes the counting-notification channel.
             * Keep xTaskNotify(..., eIncrement) on that same channel so a
             * thread and an IRQ/workqueue producer have identical semantics.
             */
            k_sem_give(&slot->count_notify);
            return pdPASS;
        case eSetValueWithOverwrite:
            atomic_set(&slot->notify_bits, (atomic_val_t)value);
            break;
        case eSetValueWithoutOverwrite:
            if (!atomic_cas(&slot->notify_bits, 0, (atomic_val_t)value))
            {
                return pdFAIL;
            }
            break;
        case eNoAction:
        default:
            break;
    }

    k_sem_give(&slot->bits_notify);
    return pdPASS;
}

BaseType_t xTaskNotify(TaskHandle_t task, uint32_t value, eNotifyAction action)
{
    return ArbTaskNotifyBits(task, value, action);
}

BaseType_t xTaskNotifyFromISR(TaskHandle_t task,
                             uint32_t value,
                             eNotifyAction action,
                             BaseType_t *higher_priority_task_woken)
{
    if (higher_priority_task_woken != NULL)
    {
        *higher_priority_task_woken = pdFALSE;
    }
    return ArbTaskNotifyBits(task, value, action);
}

BaseType_t xTaskNotifyWait(uint32_t clear_on_entry,
                           uint32_t clear_on_exit,
                           uint32_t *value,
                           TickType_t timeout)
{
    ArbTaskCompat *slot = ArbTaskForHandle(NULL);
    if (slot == NULL)
    {
        return pdFAIL;
    }

    atomic_and(&slot->notify_bits, (atomic_val_t)~clear_on_entry);
    uint32_t pending = (uint32_t)atomic_get(&slot->notify_bits);
    if (pending != 0u)
    {
        /*
         * 生产者先写位再 give。若这里直接看到位，就把对应的二值信号量一并
         * 消耗，避免它遗留到下一次等待并造成一次返回 0 位的虚假唤醒。
         */
        (void)k_sem_take(&slot->bits_notify, K_NO_WAIT);
    }
    else
    {
        if (k_sem_take(&slot->bits_notify, ArbTimeoutFromTicks(timeout)) != 0)
        {
            return pdFAIL;
        }
        pending = (uint32_t)atomic_get(&slot->notify_bits);
    }

    if (value != NULL)
    {
        *value = pending;
    }
    atomic_and(&slot->notify_bits, (atomic_val_t)~clear_on_exit);
    if (atomic_get(&slot->notify_bits) != 0)
    {
        k_sem_give(&slot->bits_notify);
    }
    return pdPASS;
}

void vTaskNotifyGiveFromISR(TaskHandle_t task, BaseType_t *higher_priority_task_woken)
{
    ArbTaskCompat *slot = ArbTaskForHandle(task);
    if (slot != NULL)
    {
        k_sem_give(&slot->count_notify);
    }
    if (higher_priority_task_woken != NULL)
    {
        *higher_priority_task_woken = pdFALSE;
    }
}

uint32_t ulTaskNotifyTake(BaseType_t clear_on_exit, TickType_t timeout)
{
    ArbTaskCompat *slot = ArbTaskForHandle(NULL);
    if (slot == NULL ||
        k_sem_take(&slot->count_notify, ArbTimeoutFromTicks(timeout)) != 0)
    {
        return 0u;
    }

    uint32_t count = k_sem_count_get(&slot->count_notify) + 1u;
    if (clear_on_exit != pdFALSE)
    {
        k_sem_reset(&slot->count_notify);
    }
    return count;
}

SemaphoreHandle_t xSemaphoreCreateBinaryStatic(StaticSemaphore_t *buffer)
{
    if (buffer == NULL)
    {
        return NULL;
    }
    buffer->kind = ARB_SEMAPHORE_BINARY;
    return (k_sem_init(&buffer->object.semaphore, 0u, 1u) == 0) ? buffer : NULL;
}

SemaphoreHandle_t xSemaphoreCreateMutexStatic(StaticSemaphore_t *buffer)
{
    if (buffer == NULL)
    {
        return NULL;
    }
    buffer->kind = ARB_SEMAPHORE_MUTEX;
    k_mutex_init(&buffer->object.mutex);
    return buffer;
}

BaseType_t xSemaphoreTake(SemaphoreHandle_t semaphore, TickType_t timeout)
{
    if (semaphore == NULL)
    {
        return pdFALSE;
    }

    int result = (semaphore->kind == ARB_SEMAPHORE_MUTEX)
                     ? k_mutex_lock(&semaphore->object.mutex, ArbTimeoutFromTicks(timeout))
                     : k_sem_take(&semaphore->object.semaphore, ArbTimeoutFromTicks(timeout));
    return (result == 0) ? pdTRUE : pdFALSE;
}

BaseType_t xSemaphoreGive(SemaphoreHandle_t semaphore)
{
    if (semaphore == NULL)
    {
        return pdFALSE;
    }

    if (semaphore->kind == ARB_SEMAPHORE_MUTEX)
    {
        return (k_mutex_unlock(&semaphore->object.mutex) == 0) ? pdTRUE : pdFALSE;
    }
    k_sem_give(&semaphore->object.semaphore);
    return pdTRUE;
}

BaseType_t xSemaphoreGiveFromISR(SemaphoreHandle_t semaphore,
                                 BaseType_t *higher_priority_task_woken)
{
    if (higher_priority_task_woken != NULL)
    {
        *higher_priority_task_woken = pdFALSE;
    }
    if (semaphore == NULL || semaphore->kind != ARB_SEMAPHORE_BINARY)
    {
        return pdFALSE;
    }
    k_sem_give(&semaphore->object.semaphore);
    return pdTRUE;
}

void vSemaphoreDelete(SemaphoreHandle_t semaphore)
{
    if (semaphore != NULL && semaphore->kind == ARB_SEMAPHORE_BINARY)
    {
        k_sem_reset(&semaphore->object.semaphore);
    }
}

static void ArbTimerWorkHandler(struct k_work *work)
{
    StaticTimer_t *timer = CONTAINER_OF(work, StaticTimer_t, work);
    if (timer->callback != NULL)
    {
        timer->callback(timer);
    }
}

static void ArbTimerExpiryHandler(struct k_timer *kernel_timer)
{
    StaticTimer_t *timer = CONTAINER_OF(kernel_timer, StaticTimer_t, timer);
    (void)k_work_submit(&timer->work);
}

TimerHandle_t xTimerCreateStatic(const char *name,
                                 TickType_t period,
                                 BaseType_t auto_reload,
                                 void *timer_id,
                                 TimerCallbackFunction_t callback,
                                 StaticTimer_t *timer_buffer)
{
    if (period == 0u || callback == NULL || timer_buffer == NULL)
    {
        return NULL;
    }

    timer_buffer->name = name;
    timer_buffer->timer_id = timer_id;
    timer_buffer->callback = callback;
    timer_buffer->period = period;
    timer_buffer->auto_reload = auto_reload;
    k_work_init(&timer_buffer->work, ArbTimerWorkHandler);
    k_timer_init(&timer_buffer->timer, ArbTimerExpiryHandler, NULL);
    return timer_buffer;
}

BaseType_t xTimerStart(TimerHandle_t timer, TickType_t command_timeout)
{
    ARG_UNUSED(command_timeout);
    if (timer == NULL)
    {
        return pdFALSE;
    }

    k_timeout_t period = (timer->auto_reload != pdFALSE)
                             ? K_TICKS(timer->period)
                             : K_NO_WAIT;
    k_timer_start(&timer->timer, K_TICKS(timer->period), period);
    return pdPASS;
}

BaseType_t xTimerStop(TimerHandle_t timer, TickType_t command_timeout)
{
    ARG_UNUSED(command_timeout);
    if (timer == NULL)
    {
        return pdFALSE;
    }
    k_timer_stop(&timer->timer);
    (void)k_work_cancel(&timer->work);
    return pdPASS;
}

void *pvTimerGetTimerID(TimerHandle_t timer)
{
    return (timer == NULL) ? NULL : timer->timer_id;
}
