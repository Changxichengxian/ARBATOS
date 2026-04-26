/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#pragma once

#include "struct_typedef.h"

#include "app_config.h"
#include "gimbal_behaviour.h"
#include "gimbal_task.h"

#ifdef __cplusplus
extern "C" {
#endif

// SD file path for pitch calibration table.
#define PITCH_CALI_FILE_PATH "0:/pitch_cali.bin"

// Called once at boot (after tasks started). It attempts to load an existing calibration table from SD.
void pitch_cali_boot_load(void);

// Called every gimbal loop before gimbal_mode_change_control_transit().
// - It may override pitch motor mode (RAW/ENCODE) when running in pitch calibration mode.
void pitch_cali_tick_pre(gimbal_control_t *gimbal, gimbal_behaviour_e behaviour, test_mode_e test_mode);

// Called by gimbal_behaviour_control_set() when behaviour == GIMBAL_PITCH_CALI.
// - It sets yaw/pitch commands (either angle increment or raw current, depending on motor mode).
void pitch_cali_control(fp32 *yaw_cmd, fp32 *pitch_cmd, gimbal_control_t *gimbal);

// Called every gimbal loop after gimbal_control_loop() to update calibration state machine and capture data.
void pitch_cali_tick_post(const gimbal_control_t *gimbal, gimbal_behaviour_e behaviour, test_mode_e test_mode);

// Query current compensation (gravity hold + static friction breakaway currents).
// Returns 1 if table is valid and enabled; otherwise 0 and outputs are set to 0.
bool_t pitch_cali_get_comp(fp32 pitch_angle,
                           fp32 *hold_current,
                           fp32 *kick_up_current,
                           fp32 *kick_down_current);

// Returns current bullet count used by compensation (referee or manual).
uint16_t pitch_cali_get_runtime_bullet_count(void);

// Returns 1 when pitch calibration mode is actively running (so normal feedforward should be disabled).
bool_t pitch_cali_is_running(void);

#ifdef __cplusplus
}
#endif

