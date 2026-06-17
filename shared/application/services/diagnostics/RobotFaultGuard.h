/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ROBOT_FAULT_GUARD_H
#define ROBOT_FAULT_GUARD_H

#include <stdint.h>

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
} RobotFaultReason;

static inline void RobotFaultEnterSafeStateEx(uint32_t reason,
                                                   uint32_t arg0,
                                                   uint32_t arg1,
                                                   uint32_t task_handle,
                                                   const char *task_name)
{
    WatchDiagSetErrorArgs(arg0, arg1);
    WatchDiagMarkErrorHandler(HAL_GetTick(), __get_IPSR());
    WatchDiagMarkFatal(reason, task_handle, task_name);
    RobotLifecycleEnterFault(ROBOT_LIFECYCLE_REASON_FATAL_FAULT);
    (void)LowCmdEnterEmergencyStop((uint16_t)LOWCMD_WRITER_FAULT);
}

static inline void RobotFaultEnterSafeState(uint32_t reason,
                                                uint32_t arg0,
                                                uint32_t arg1,
                                                const char *task_name)
{
    RobotFaultEnterSafeStateEx(reason, arg0, arg1, 0u, task_name);
}

static inline void RobotFaultHaltForever(void)
{
    if ((CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) != 0U)
    {
        __BKPT(0);
    }

    __disable_irq();
    for (;;)
    {
        __NOP();
    }
}

static inline void RobotFaultRecordAndHalt(uint32_t reason,
                                               uint32_t arg0,
                                               uint32_t arg1)
{
    RobotFaultEnterSafeState(reason, arg0, arg1, 0);
    RobotFaultHaltForever();
}

#endif
