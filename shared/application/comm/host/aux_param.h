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
#define AUX_TUNE_ENABLE_PARAM_NAME_LOOKUP 0u
#endif

bool_t aux_param_set_config_param(uint16_t id, fp32 value);
#if AUX_TUNE_ENABLE_PARAM_NAME_LOOKUP
bool_t aux_param_set_config_param_by_name(const char *name, fp32 value);
#endif

#endif
