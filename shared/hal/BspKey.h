/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */


#ifndef BSP_KEY_H
#define BSP_KEY_H

#include "types.h"

// Board-specific pin/level is configured in BspKeyCfg.h.

// Board key (level defined by BSP_KEY_ACTIVE_LOW).

extern void BspKeyExti0Callback(void);
extern uint8_t BspKeyReadRawDown(void);
extern uint32_t BspKeyGetPressCnt(void);
extern uint32_t BspKeyGetLastPressTickMs(void);

#endif
