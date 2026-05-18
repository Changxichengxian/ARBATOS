/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef BUZZER_FILE_PLAYER_H
#define BUZZER_FILE_PLAYER_H

#include <stdint.h>

// TF/SD card raw file playback (FatFs). File format: raw unsigned 8-bit PCM mono ("u8") with no header.
// Path example: "0:/you_12k.u8". Note: FatFs is configured for ASCII filenames (FF_CODE_PAGE=437).
// PC-side example: `ffmpeg -y -i input.mp3 -ac 1 -ar 12000 -c:a pcm_u8 -f u8 0:/you_12k.u8`
int buzzer_pcm_play_file_u8(const char *path, uint32_t sample_rate_hz, uint8_t loop, uint8_t volume);
void buzzer_pcm_play_file_stop(void);
int32_t buzzer_pcm_play_file_last_error(void);
// 0..255 envelope derived from low-passed music samples.
uint8_t buzzer_pcm_get_music_env_u8(void);
void buzzer_pcm_reset_music_env(void);

#endif

