/*
 * SPDX-FileCopyrightText: 2026 Chen Xuan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef AUX_PARAM_H
#define AUX_PARAM_H

#include "config.h"

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
} aux_param_result_e;

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
} aux_param_info_t;

bool_t aux_param_set_config_param(uint16_t id, fp32 value);
aux_param_result_e aux_param_set_config_param_ex(uint16_t id, fp32 value);
aux_param_result_e aux_param_validate_config_param(uint16_t id, fp32 value);
bool_t aux_param_get_config_param(uint16_t id, fp32 *out);
uint16_t aux_param_get_count(void);
bool_t aux_param_get_info_by_index(uint16_t index, aux_param_info_t *out);
bool_t aux_param_get_info(uint16_t id, aux_param_info_t *out);
aux_param_result_e aux_param_get_last_result(void);
uint16_t aux_param_get_last_id(void);
fp32 aux_param_get_last_value(void);
const char *aux_param_result_name(aux_param_result_e result);
#if AUX_TUNE_ENABLE_PARAM_NAME_LOOKUP
bool_t aux_param_set_config_param_by_name(const char *name, fp32 value);
aux_param_result_e aux_param_set_config_param_by_name_ex(const char *name, fp32 value);
#endif

#endif
