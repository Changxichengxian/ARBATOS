/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "bsp_crc32.h"
#include "main.h"
extern CRC_HandleTypeDef hcrc;

uint32_t get_crc32_check_sum(uint32_t *data, uint32_t len)
{
    return HAL_CRC_Calculate(&hcrc, data, len);
}

bool_t verify_crc32_check_sum(uint32_t *data, uint32_t len)
{
    static uint32_t crc32;
    crc32 = get_crc32_check_sum(data, len-1);
    return (crc32 == data[len-1]);
}
void append_crc32_check_sum(uint32_t *data, uint32_t len)
{
    uint32_t crc32;
    crc32 = get_crc32_check_sum(data, len-1);
    data[len-1] = crc32;
}

