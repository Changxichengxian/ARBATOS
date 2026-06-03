/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ROBOT_FAULT_GUARD_H
#define ROBOT_FAULT_GUARD_H

#include <stdint.h>

#include "LowCmd.h"
#include "main.h"
#include "watch.h"

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
} robot_fault_reason_e;

static inline void robot_fault_enter_safe_state_ex(uint32_t reason,
                                                   uint32_t arg0,
                                                   uint32_t arg1,
                                                   uint32_t task_handle,
                                                   const char *task_name)
{
    watch_diag_set_error_args(arg0, arg1);
    watch_diag_mark_error_handler(HAL_GetTick(), __get_IPSR());
    watch_diag_mark_fatal(reason, task_handle, task_name);
    (void)LowCmdEnterEmergencyStop((uint16_t)LOWCMD_WRITER_FAULT);
}

static inline void robot_fault_enter_safe_state(uint32_t reason,
                                                uint32_t arg0,
                                                uint32_t arg1,
                                                const char *task_name)
{
    robot_fault_enter_safe_state_ex(reason, arg0, arg1, 0u, task_name);
}

static inline void robot_fault_halt_forever(void)
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

static inline void robot_fault_record_and_halt(uint32_t reason,
                                               uint32_t arg0,
                                               uint32_t arg1)
{
    robot_fault_enter_safe_state(reason, arg0, arg1, 0);
    robot_fault_halt_forever();
}

#endif
