/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ROBOT_FAULT_GUARD_H
#define ROBOT_FAULT_GUARD_H

#include <stdint.h>

#include "BspCan.h"
#include "CanTxTask.h"
#include "LowCmd.h"
#include "main.h"
#include "RobotLifecycle.h"
#include "Watch.h"

typedef enum
{
    ROBOT_FAULT_REASON_NONE = 0u,
    ROBOT_FAULT_REASON_STACK_OVERFLOW = 1u,
    ROBOT_FAULT_REASON_MALLOC_FAILED = 2u,
    ROBOT_FAULT_REASON_ERROR_HANDLER = 3u,
    ROBOT_FAULT_REASON_HARDFAULT = 4u,
    ROBOT_FAULT_REASON_MEMMANAGE = 5u,
    ROBOT_FAULT_REASON_BUSFAULT = 6u,
    ROBOT_FAULT_REASON_USAGEFAULT = 7u,
    ROBOT_FAULT_REASON_NMI = 8u,
} RobotFaultReason;

/*
 * 这里仅处理已经无法可靠继续调度的系统致命故障。普通设备掉线和控制域故障
 * 必须走 FaultMgr 的局部策略，不能调用本文件的全局安全帧和复位入口。
 * DEVICE_DOMAIN_FAULTS_USE_FAULT_MGR：这个标记由架构检查固定上述边界。
 */

static inline TaskHandle_t RobotFaultCurrentTaskHandle(void)
{
    if (xTaskGetSchedulerState() != taskSCHEDULER_RUNNING)
    {
        return NULL;
    }
    return xTaskGetCurrentTaskHandle();
}

static inline const char *RobotFaultTaskNameOrUnknown(TaskHandle_t task)
{
    if (xTaskGetSchedulerState() != taskSCHEDULER_RUNNING || task == NULL)
    {
        return "?";
    }
    return pcTaskGetName(task);
}

static inline void RobotFaultEnterSafeStateEx(uint32_t reason,
                                                   uint32_t arg0,
                                                   uint32_t arg1,
                                                   uint32_t task_handle,
                                                   const char *task_name)
{
    /* 先直发安全帧，后面的诊断即使再次出错，也不会留下旧输出。 */
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
}

static inline void RobotFaultEnterSafeState(uint32_t reason,
                                                uint32_t arg0,
                                                uint32_t arg1,
                                                const char *task_name)
{
    RobotFaultEnterSafeStateEx(reason, arg0, arg1, 0u, task_name);
}

static inline void RobotFaultResetNow(void)
{
    __disable_irq();
    BspCanFaultWaitTxIdle();

    if ((CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) != 0U)
    {
        __BKPT(0);
    }

    NVIC_SystemReset();
}

/* 兼容现有工程入口；行为已经改为有界发送后系统复位。 */
static inline void RobotFaultHaltForever(void)
{
    RobotFaultResetNow();
}

static inline void RobotFaultResetFromException(uint32_t reason,
                                                uint32_t arg0,
                                                uint32_t arg1)
{
    (void)reason;
    (void)arg0;
    (void)arg1;

    __disable_irq();
    CanTxEmergencyStopNow();
    RobotFaultResetNow();
}

static inline void RobotFaultRecordAndReset(uint32_t reason,
                                               uint32_t arg0,
                                               uint32_t arg1)
{
    if (__get_IPSR() != 0u)
    {
        RobotFaultResetFromException(reason, arg0, arg1);
        return;
    }

    RobotFaultEnterSafeState(reason, arg0, arg1, 0);
    RobotFaultResetNow();
}

/* 旧名字保留给尚未迁移的 CubeMX USER CODE 区。 */
static inline void RobotFaultRecordAndHalt(uint32_t reason,
                                           uint32_t arg0,
                                           uint32_t arg1)
{
    RobotFaultRecordAndReset(reason, arg0, arg1);
}

#endif
