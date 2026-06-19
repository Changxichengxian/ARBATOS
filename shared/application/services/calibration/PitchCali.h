/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#pragma once

#include "Types.h"

#include "RobotConfig.h"
#include "GimbalBehaviour.h"
#include "GimbalControlTask.h"

#ifdef __cplusplus
extern "C" {
#endif

// SD file path for pitch calibration table.
#define PITCH_CALI_FILE_PATH "0:/pitch_cali.bin"

typedef struct
{
    uint8_t angle_points;
    uint8_t bullet_points;
    const fp32 *angle;
    const uint16_t *bullet;
    const int16_t *hold_current; // row-major: [bullet_idx * angle_points + angle_idx]
    const uint16_t *kick_up;     // row-major: [bullet_idx * angle_points + angle_idx]
    const uint16_t *kick_down;   // row-major: [bullet_idx * angle_points + angle_idx]
} PitchCaliBuiltinDesc;

// Called once at boot (after tasks started). It attempts to load an existing calibration table from SD.
void PitchCaliBootLoad(void);

// Optional target-provided built-in compensation table.
// Return 1 when out is filled, otherwise 0.
bool_t PitchCaliGetBuiltinDefault(PitchCaliBuiltinDesc *out);

// Called every gimbal loop before GimbalModeChangeControlTransit().
// - It may override pitch motor mode (RAW/ENCODE) when running in pitch calibration mode.
void PitchCaliTickPre(GimbalControl *gimbal, GimbalBehaviour behaviour, uint8_t PitchCaliMode);

// Called by GimbalBehaviourControlSet() when behaviour == GIMBAL_PITCH_CALI.
// - It sets yaw/pitch commands (either angle increment or raw current, depending on motor mode).
void PitchCaliControl(fp32 *yaw_cmd, fp32 *pitch_cmd, GimbalControl *gimbal);

// Called every gimbal loop after GimbalControlLoop() to update calibration state machine and capture data.
void PitchCaliTickPost(const GimbalControl *gimbal, GimbalBehaviour behaviour, uint8_t PitchCaliMode);

// Query current compensation (gravity hold + static friction breakaway currents).
// Returns 1 if table is valid and enabled; otherwise 0 and outputs are set to 0.
bool_t PitchCaliGetComp(fp32 pitch_angle,
                           fp32 *hold_current,
                           fp32 *kick_up_current,
                           fp32 *kick_down_current);

// Returns current bullet count used by compensation (referee or manual).
uint16_t PitchCaliGetRuntimeBulletCount(void);

// Returns 1 when pitch calibration mode is actively running (so normal feedforward should be disabled).
bool_t PitchCaliIsRunning(void);

#ifdef __cplusplus
}
#endif
