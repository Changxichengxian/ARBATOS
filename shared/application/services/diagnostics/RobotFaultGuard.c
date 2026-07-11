/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include "RobotFaultGuard.h"

#include "BspCan.h"
#include "BspResetEvidence.h"
#include "CanTxTask.h"
#include "LowCmd.h"
#include "RobotLifecycle.h"
#include "Watch.h"
#include "main.h"

static volatile uint32_t s_fault_entry_active;
static BspResetEvidenceRecord s_fault_record;

#define ROBOT_FAULT_CFSR_STACK_ERROR_MASK 0x00003838u

static void RobotFaultResetNow(void);

static uint8_t RobotFaultRangeContains(uint32_t begin,
                                       uint32_t end,
                                       uint32_t address,
                                       uint32_t size)
{
    return (uint8_t)(address >= begin &&
                     address < end &&
                     size <= (end - address));
}

static uint8_t RobotFaultStackReadable(const uint32_t *stack,
                                       uint32_t exc_return)
{
    const uint32_t address = (uint32_t)stack;
    const uint32_t word_count = ((exc_return & (1u << 4)) != 0u) ? 8u : 26u;
    const uint32_t required = word_count * (uint32_t)sizeof(uint32_t);

    if ((address & 0x3u) != 0u)
    {
        return 0u;
    }

#if defined(STM32H723xx)
    return (uint8_t)(
        RobotFaultRangeContains(0x20000000u, 0x20020000u, address, required) != 0u ||
        RobotFaultRangeContains(0x24000000u, 0x24050000u, address, required) != 0u);
#elif defined(STM32F427xx)
    return (uint8_t)(
        RobotFaultRangeContains(0x20000000u, 0x20030000u, address, required) != 0u ||
        RobotFaultRangeContains(0x10000000u, 0x10010000u, address, required) != 0u);
#else
    return (uint8_t)(
        RobotFaultRangeContains(0x20000000u, 0x20020000u, address, required) != 0u ||
        RobotFaultRangeContains(0x10000000u, 0x10010000u, address, required) != 0u);
#endif
}

static void RobotFaultRecordClear(BspResetEvidenceRecord *record)
{
    uint32_t *words = (uint32_t *)record;

    for (uint32_t i = 0u;
         i < (uint32_t)(sizeof(*record) / sizeof(uint32_t));
         i++)
    {
        words[i] = 0u;
    }
}

static void RobotFaultRecordFrame(BspResetEvidenceRecord *record,
                                  uint32_t *stack,
                                  uint32_t exc_return)
{
    const uint32_t *basic;

    record->excReturn = exc_return;
    record->stackPtr = (uint32_t)stack;
    if ((SCB->CFSR & ROBOT_FAULT_CFSR_STACK_ERROR_MASK) != 0u ||
        RobotFaultStackReadable(stack, exc_return) == 0u)
    {
        return;
    }

    basic = stack;
    if ((exc_return & (1u << 4)) == 0u)
    {
        /* EXC_RETURN bit4 为 0 时，基础帧固定在 18 个浮点上下文字之后。 */
        basic = stack + 18u;
    }
    if ((basic[7] & (1u << 24)) == 0u)
    {
        return;
    }

    record->r0 = basic[0];
    record->r1 = basic[1];
    record->r2 = basic[2];
    record->r3 = basic[3];
    record->r12 = basic[4];
    record->lr = basic[5];
    record->pc = basic[6];
    record->xpsr = basic[7];
}

static void RobotFaultPersist(uint32_t reason,
                              uint32_t arg0,
                              uint32_t arg1,
                              uint32_t task_handle,
                              uint32_t *stack,
                              uint32_t exc_return)
{
    BspResetEvidenceRecord *record = &s_fault_record;

    RobotFaultRecordClear(record);
    record->reason = reason;
    record->arg0 = arg0;
    record->arg1 = arg1;
    record->ipsr = __get_IPSR();
    record->msp = __get_MSP();
    record->psp = __get_PSP();
    record->cfsr = SCB->CFSR;
    record->hfsr = SCB->HFSR;
    record->dfsr = SCB->DFSR;
    record->afsr = SCB->AFSR;
    record->mmfar = SCB->MMFAR;
    record->bfar = SCB->BFAR;
    record->icsr = SCB->ICSR;
    record->shcsr = SCB->SHCSR;
    record->control = __get_CONTROL();
    record->tickMs = HAL_GetTick();
    record->bootStage = WatchDiagBootStageGet();
    record->taskHandle = task_handle;
    RobotFaultRecordFrame(record, stack, exc_return);
    BspResetEvidenceWriteFatal(record);
}

static void RobotFaultNestedReset(void)
{
    __disable_irq();
    /* 二次异常说明证据或安全输出链本身已经不可信，不能再次递归进入外设。 */
    NVIC_SystemReset();
    while (1)
    {
        __NOP();
    }
}

static uint8_t RobotFaultEnterOnce(void)
{
    __disable_irq();
    if (s_fault_entry_active != 0u)
    {
        return 0u;
    }
    s_fault_entry_active = 1u;
    __DMB();
    return 1u;
}

void RobotFaultEarlyInit(void)
{
    s_fault_entry_active = 0u;
    __DMB();
}

static TaskHandle_t RobotFaultCurrentTaskHandle(void)
{
    if (xTaskGetSchedulerState() != taskSCHEDULER_RUNNING)
    {
        return NULL;
    }
    return xTaskGetCurrentTaskHandle();
}

static const char *RobotFaultTaskNameOrUnknown(TaskHandle_t task)
{
    if (xTaskGetSchedulerState() != taskSCHEDULER_RUNNING || task == NULL)
    {
        return "?";
    }
    return pcTaskGetName(task);
}

void RobotFaultTaskAndReset(uint32_t reason,
                            uint32_t arg0,
                            uint32_t arg1,
                            TaskHandle_t task,
                            const char *task_name)
{
    const uint32_t task_handle = (uint32_t)(uintptr_t)task;

    if (RobotFaultEnterOnce() == 0u)
    {
        RobotFaultNestedReset();
    }

    /* 固定长度证据先写入备份 SRAM；随后立即锁住普通输出并直发安全帧。 */
    RobotFaultPersist(reason, arg0, arg1, task_handle, NULL, 0u);
    CanTxEmergencyStopNow();
    WatchDiagSetErrorArgs(arg0, arg1);
    WatchDiagMarkErrorHandler(HAL_GetTick(), __get_IPSR());
    WatchDiagMarkFatal(reason, task_handle, task_name);

    /* 中断和 CPU 异常上下文不能进入依赖 FreeRTOS 临界区的状态链。 */
    if (__get_IPSR() == 0u && xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
        RobotLifecycleEnterFatalFault();
        (void)LowCmdEnterEmergencyStop((uint16_t)LOWCMD_WRITER_FAULT);
    }
    RobotFaultResetNow();
}

static void RobotFaultResetNow(void)
{
    __disable_irq();
    BspCanFaultWaitTxIdle();

    if ((CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) != 0u)
    {
        __BKPT(0);
    }

    NVIC_SystemReset();
    while (1)
    {
        __NOP();
    }
}

static void RobotFaultResetFromExceptionFrame(uint32_t reason,
                                              uint32_t arg0,
                                              uint32_t arg1,
                                              uint32_t *stack,
                                              uint32_t exc_return)
{
    if (RobotFaultEnterOnce() == 0u)
    {
        RobotFaultNestedReset();
    }

    __disable_irq();
    RobotFaultPersist(reason, arg0, arg1, 0u, stack, exc_return);
    CanTxEmergencyStopNow();
    RobotFaultResetNow();
}

void RobotFaultResetFromException(uint32_t reason,
                                  uint32_t arg0,
                                  uint32_t arg1)
{
    RobotFaultResetFromExceptionFrame(reason, arg0, arg1, NULL, 0u);
}

void RobotFaultHardFaultEntry(uint32_t *stack, uint32_t exc_return)
{
    RobotFaultResetFromExceptionFrame((uint32_t)ROBOT_FAULT_REASON_HARDFAULT,
                                      SCB->HFSR,
                                      SCB->CFSR,
                                      stack,
                                      exc_return);
}

void RobotFaultDefaultHandler(void)
{
    RobotFaultResetFromException(
        (uint32_t)ROBOT_FAULT_REASON_DEFAULT_INTERRUPT,
        __get_IPSR(),
        SCB->ICSR);
}

void RobotFaultRecordAndReset(uint32_t reason,
                              uint32_t arg0,
                              uint32_t arg1)
{
    TaskHandle_t task;

    if (__get_IPSR() != 0u)
    {
        RobotFaultResetFromException(reason, arg0, arg1);
        return;
    }

    task = RobotFaultCurrentTaskHandle();
    RobotFaultTaskAndReset(reason,
                           arg0,
                           arg1,
                           task,
                           RobotFaultTaskNameOrUnknown(task));
}

void RobotFaultAssert(const char *file, uint32_t line)
{
    uint32_t file_hash = 2166136261u;

    if (file != NULL)
    {
        while (*file != '\0')
        {
            file_hash ^= (uint8_t)*file;
            file_hash *= 16777619u;
            file++;
        }
    }
    RobotFaultRecordAndReset((uint32_t)ROBOT_FAULT_REASON_ASSERT,
                             file_hash,
                             line);
}
