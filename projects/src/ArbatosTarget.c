/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>

#include "ArbatosTarget.h"

#if CONFIG_ARBATOS_LEGACY_SOURCES
#include "ArbatosRuntime.h"
#endif

LOG_MODULE_DECLARE(arbatos);

const char *ArbatosTargetName(void)
{
    return CONFIG_ARBATOS_ROBOT_NAME;
}

void ArbatosTargetStart(void)
{
#if CONFIG_ARBATOS_LEGACY_SOURCES
    ArbatosRuntimeStatus status = ArbatosRuntimeStart();

    if ((status != ARBATOS_RUNTIME_OK) &&
        (status != ARBATOS_RUNTIME_ALREADY_STARTED))
    {
        LOG_ERR("runtime start failed: %s", ArbatosRuntimeStatusName(status));
    }
#else
    LOG_INF("board smoke-test build; ARBATOS runtime is disabled");
#endif
}
