/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "ArbatosTarget.h"

LOG_MODULE_REGISTER(arbatos, LOG_LEVEL_INF);

int main(void)
{
#if CONFIG_ARBATOS_BOOT_BANNER
    LOG_INF("ARBATOS Zephyr startup: target=%s", ArbatosTargetName());
#endif

    /*
     * Board-specific early hardware setup belongs in zephyr/boards.  The
     * scheduler starts before main(), so task creation will move to the
     * Zephyr thread adapters instead of MX_FREERTOS_Init().
     */
    ArbatosTargetStart();
    return 0;
}
