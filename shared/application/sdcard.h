/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef SDCARD_H
#define SDCARD_H

/**
  * @brief  Mount TF/SD card on SPI2 (FatFs).
  * @retval 0 on success, non-zero on failure.
  */
int sdcard_mount(void);

/**
  * @brief  Check if SD card is mounted.
  * @retval 1 mounted, 0 not mounted.
  */
int sdcard_is_mounted(void);

/**
  * @brief  Append a boot marker line to "0:/boot.txt" (best-effort).
  * @retval 0 on success, non-zero on failure.
  */
int sdcard_boot_mark(void);

#endif

