/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */


#ifndef HOST_LINK_TASK_H
#define HOST_LINK_TASK_H

// HostLinkTask now acts as the task-level entry point and pulls in the split
// vision and image-remote link modules for callers that still include this legacy header.
#include "VisionLink.h"
#include "ImageRemoteLink.h"

void HostLinkTask(void const * argument);

#endif
