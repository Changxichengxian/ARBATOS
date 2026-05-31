/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 Xie Yuhan <2811158416@qq.com>
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
} runtime_instance_kind_e;

typedef enum
{
    RUNTIME_INSTANCE_STATE_UNKNOWN = 0u,
    RUNTIME_INSTANCE_STATE_DISABLED,
    RUNTIME_INSTANCE_STATE_ENABLED,
    RUNTIME_INSTANCE_STATE_ACTIVE,
    RUNTIME_INSTANCE_STATE_FAULT,
} runtime_instance_state_e;

typedef struct
{
    const char *name;
    uint8_t kind;
    uint8_t state;
    uint16_t source_id;
    uint16_t source_index;
    uint16_t parent_index;
} runtime_instance_ref_t;

static inline runtime_instance_ref_t runtime_instance_ref_make(const char *name,
                                                               runtime_instance_kind_e kind,
                                                               runtime_instance_state_e state,
                                                               uint16_t source_id,
                                                               uint16_t source_index,
                                                               uint16_t parent_index)
{
    runtime_instance_ref_t ref;

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
