/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef RUNTIME_INSTANCE_H
#define RUNTIME_INSTANCE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RUNTIME_INSTANCE_INDEX_NONE 0xFFFFu

typedef enum
{
    RUNTIME_INSTANCE_KIND_UNKNOWN = 0u,
    RUNTIME_INSTANCE_KIND_TASK,
    RUNTIME_INSTANCE_KIND_DEVICE,
    RUNTIME_INSTANCE_KIND_CONTROLLER,
    RUNTIME_INSTANCE_KIND_GROUP,
    RUNTIME_INSTANCE_KIND_CUSTOM_BASE = 128u,
} RuntimeInstanceKind;

typedef enum
{
    RUNTIME_INSTANCE_STATE_UNKNOWN = 0u,
    RUNTIME_INSTANCE_STATE_DISABLED,
    RUNTIME_INSTANCE_STATE_ENABLED,
    RUNTIME_INSTANCE_STATE_ACTIVE,
    RUNTIME_INSTANCE_STATE_FAULT,
} RuntimeInstanceState;

typedef struct
{
    const char *name;
    uint8_t kind;
    uint8_t state;
    uint16_t source_id;
    uint16_t source_index;
    uint16_t parent_index;
} RuntimeInstanceRef;

static inline RuntimeInstanceRef RuntimeInstanceRefMake(const char *name,
                                                               RuntimeInstanceKind kind,
                                                               RuntimeInstanceState state,
                                                               uint16_t source_id,
                                                               uint16_t source_index,
                                                               uint16_t parent_index)
{
    RuntimeInstanceRef ref;

    ref.name = name;
    ref.kind = (uint8_t)kind;
    ref.state = (uint8_t)state;
    ref.source_id = source_id;
    ref.source_index = source_index;
    ref.parent_index = parent_index;
    return ref;
}

#ifdef __cplusplus
}
#endif

#endif
