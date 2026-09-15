/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef SD_LOG_TASK_H
#define SD_LOG_TASK_H

#include <stdint.h>

#define SDLOG_FLASH_MAILBOX_GUARD 0x53444C31u
#define SDLOG_FLASH_PAUSE_REQUEST 0x53445051u
#define SDLOG_FLASH_PAUSE_READY 0x53445244u
#define SDLOG_FLASH_PAUSE_FAILED 0x53444641u
#define SDLOG_FLASH_BUILD_FINGERPRINT_SIZE 32u

extern const volatile uint32_t g_sdLogFlashMailboxGuard;
extern const uint8_t g_sdLogFlashBuildFingerprint[SDLOG_FLASH_BUILD_FINGERPRINT_SIZE];
extern volatile uint32_t g_sdLogFlashPauseRequest;
extern volatile uint32_t g_sdLogFlashPauseReady;

void SdLogTask(void const *argument);

#endif
