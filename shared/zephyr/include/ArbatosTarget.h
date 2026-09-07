/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

/* Returns the selected Robotconfig target's stable display name. */
const char *ArbatosTargetName(void);

/* Starts the selected target's platform services and application tasks. */
void ArbatosTargetStart(void);
