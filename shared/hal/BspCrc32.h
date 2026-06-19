/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef BSP_CRC32_H
#define BSP_CRC32_H
#include "Types.h"

extern uint32_t get_crc32_check_sum(uint32_t *data, uint32_t len);
extern bool_t  verify_crc32_check_sum(uint32_t *data, uint32_t len);
extern void append_crc32_check_sum(uint32_t *data, uint32_t len);
#endif
