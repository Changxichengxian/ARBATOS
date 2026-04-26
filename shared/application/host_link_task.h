/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */


#ifndef HOST_LINK_TASK_H
#define HOST_LINK_TASK_H

// host_link_task now acts as the task-level entry point and pulls in the split
// vision and image-remote link modules for callers that still include this legacy header.
#include "vision_link.h"
#include "image_remote_link.h"

void host_link_task(void const * argument);

#endif
