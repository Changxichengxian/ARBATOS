/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "RobotConfig.h"
#include "LowCmd.h"

/*
 * 这份表现在分两类：
 * 1. 稳定配置：只在代码里改，不给 AUX 参数号。
 * 2. 动态配置：只保留需要临时试的量，注释里带 [ID]。
 */

/*
 * AUX 口临时调参：发送 "<id>:<value>"，例如 "1:1000"。
 * - 带 [ID] 的注释，表示还能通过 AUX 口临时改。
 * - 不带 [ID] 的注释，表示稳定配置，只能改代码默认值。
 * - AUX 口只改 RAM 里的 g_config，重启后会回到这里的默认值。
 */

#if defined(__CC_ARM) || defined(__ARMCC_VERSION)
#pragma push
#pragma diag_suppress 188
#endif
#if defined(__GNUC__) && !defined(__CC_ARM)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-braces"
#endif
Config g_config = {
#include "ConfigOperation.inc"
#include "ConfigHardware.inc"
#include "ConfigTuning.inc"
#include "ConfigInput.inc"
#include "ConfigDiagnostics.inc"
};
#if defined(__GNUC__) && !defined(__CC_ARM)
#pragma GCC diagnostic pop
#endif
#if defined(__CC_ARM) || defined(__ARMCC_VERSION)
#pragma pop
#endif
