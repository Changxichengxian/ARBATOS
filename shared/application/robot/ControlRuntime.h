/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CONTROL_RUNTIME_H
#define CONTROL_RUNTIME_H

#include <stdint.h>

#include "ControlAlgorithm.h"
#include "ControlMgr.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CONTROL_RUNTIME_MAX_MODULES 2u
#define CONTROL_RUNTIME_FEEDBACK_TIMEOUT_MS 20u
#define CONTROL_RUNTIME_IMU_TIMEOUT_MS 20u

typedef struct
{
    const ControlAlgorithm *algorithm;
    const char *const *outputNames;
    uint8_t outputCount;
    const ControlParam *params;
    uint8_t paramCount;
    uint16_t periodMs;
    uint8_t requireImu;
} ControlModuleSpec;

/* 只能在 MotorInstRefresh() 之后、任务启动之前调用。 */
ControlResult ControlRuntimeConfigure(const ControlModuleSpec *specs, uint8_t count);
uint8_t ControlRuntimeCount(void);
const ControlController *ControlRuntimeController(uint8_t index);
ControlResult ControlRuntimeStartDefaults(void);
ControlResult ControlRuntimeRunDomain(ControlDomain domain);
uint16_t ControlRuntimePeriodMs(ControlDomain domain);

#ifdef __cplusplus
}
#endif

#endif
