/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ARBATOS_ZEPHYR_AHRS_CASE_COMPAT_H
#define ARBATOS_ZEPHYR_AHRS_CASE_COMPAT_H

/*
 * 现有 GCC 兼容实现沿用历史文件名 "AHRS.h"，仓库公共头实际名为 "Ahrs.h"。
 * 显式相对路径同时保证 Windows 和大小写敏感主机得到同一份公共声明。
 */
#include "../../../../shared/components/algorithm/Ahrs.h"

#endif
