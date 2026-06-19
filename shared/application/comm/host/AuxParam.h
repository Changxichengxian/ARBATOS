/*
 * SPDX-FileCopyrightText: 2026 Chen Xuan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef AUX_PARAM_H
#define AUX_PARAM_H

#include "RobotConfig.h"

#ifndef AUX_TUNE_ENABLE_PARAM_NAME_LOOKUP
#define AUX_TUNE_ENABLE_PARAM_NAME_LOOKUP 1u
#endif

typedef enum
{
    AUX_PARAM_RESULT_OK = 0u,
    AUX_PARAM_RESULT_UNKNOWN_ID,
    AUX_PARAM_RESULT_INACTIVE_SCOPE,
    AUX_PARAM_RESULT_RANGE,
    AUX_PARAM_RESULT_SAFE_REQUIRED,
    AUX_PARAM_RESULT_READONLY,
    AUX_PARAM_RESULT_BAD_ARGUMENT,
} AuxParamResult;

typedef struct
{
    uint16_t id;
    const char *name;
    const char *unit;
    fp32 min_value;
    fp32 max_value;
    uint8_t has_range;
    uint8_t safe_only;
    uint8_t active;
} AuxParamInfo;

bool_t AuxParamSetConfigParam(uint16_t id, fp32 value);
AuxParamResult AuxParamSetConfigParamEx(uint16_t id, fp32 value);
AuxParamResult AuxParamValidateConfigParam(uint16_t id, fp32 value);
bool_t AuxParamGetConfigParam(uint16_t id, fp32 *out);
uint16_t AuxParamGetCount(void);
bool_t AuxParamGetInfoByIndex(uint16_t index, AuxParamInfo *out);
bool_t AuxParamGetInfo(uint16_t id, AuxParamInfo *out);
AuxParamResult AuxParamGetLastResult(void);
uint16_t AuxParamGetLastId(void);
fp32 AuxParamGetLastValue(void);
const char *AuxParamResultName(AuxParamResult result);
#if AUX_TUNE_ENABLE_PARAM_NAME_LOOKUP
bool_t AuxParamSetConfigParamByName(const char *name, fp32 value);
AuxParamResult AuxParamSetConfigParamByNameEx(const char *name, fp32 value);
#endif

#endif
