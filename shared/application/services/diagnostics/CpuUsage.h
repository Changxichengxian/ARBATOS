/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef CPU_USAGE_H
#define CPU_USAGE_H

#include <stdint.h>

void CpuUsageInit(void);
uint16_t CpuUsageGetPermille(void);

#endif

