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
 * AUX 口临时改参：发送 "<id>:<value>"（例如 "1:1000"）
 * - 编号见本文件 g_config 初始化处每行末尾的 [ID] 注释
 * - 未标 [ID] 的参数：只在 init 使用 / 未运行时应用（AUX 口不支持改）
 * - 仅修改 RAM 中的 g_config，重启后恢复默认值
 */

Config g_config = {
#include "ConfigOperation.inc"
#include "ConfigHardware.inc"
#include "ConfigTuning.inc"
#include "ConfigInput.inc"
#include "ConfigDiagnostics.inc"
};
