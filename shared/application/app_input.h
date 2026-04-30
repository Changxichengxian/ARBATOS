/*
 * SPDX-FileCopyrightText: 2026 Chen Xi <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 Chen Xi <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#pragma once

#include "control_input.h"

typedef control_input_state_t app_input_t;
typedef input_axis_e app_axis_e;
typedef input_switch_e app_switch_e;

#define app_input_update_from_rc input_update_from_rc
#define app_input_get input_get
#define app_input_axis input_axis
#define app_input_switch input_switch

