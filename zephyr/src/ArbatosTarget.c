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
#if CONFIG_ARBATOS_TARGET_HERO_C
    return "HERO-C";
#elif CONFIG_ARBATOS_TARGET_HERO_M
    return "HERO-M";
#elif CONFIG_ARBATOS_TARGET_INFANTRY_A
    return "INFANTRY-A";
#elif CONFIG_ARBATOS_TARGET_SENTINEL_M
    return "SENTINEL-M";
#elif CONFIG_ARBATOS_TARGET_CARRIER_A
    return "CARRIER-A";
#elif CONFIG_ARBATOS_TARGET_MINIWHEELEG_M
    return "MINIWHEELEG-M";
#elif CONFIG_ARBATOS_TARGET_MINIWHEELEG_C
    return "MINIWHEELEG-C";
#else
#error "No ARBATOS robot target selected"
#endif
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
