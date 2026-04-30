/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

#include "config.h"
#include "motor_model_db.h"

static inline const motor_model_db_entry_t *motor_cfg_model_db(motor_model_e model);
static inline const motor_model_rx_desc_t *motor_cfg_rx_desc(motor_model_e model);

static inline const motor_model_param_t *motor_cfg_model(motor_model_e model)
{
    if ((uint32_t)model >= (uint32_t)MOTOR_MODEL__COUNT)
    {
        return NULL;
    }
    return &g_motor_config.model[model];
}

static inline uint16_t motor_cfg_can_id(const motor_node_param_t *node)
{
    if (node == NULL)
    {
        return 0u;
    }
    if (node->can_id == 0u)
    {
        return 0u;
    }
    const motor_model_param_t *m = motor_cfg_model(node->model);
    if (m == NULL)
    {
        return 0u;
    }
    return (uint16_t)(m->can_id_base + (uint16_t)node->can_id);
}

static inline motor_protocol_e motor_cfg_protocol(const motor_node_param_t *node)
{
    const motor_model_db_entry_t *entry = NULL;

    if (node == NULL)
    {
        return MOTOR_PROTOCOL_RM_GROUP;
    }
    if (node->protocol != (uint8_t)MOTOR_PROTOCOL_INHERIT)
    {
        return (motor_protocol_e)node->protocol;
    }

    entry = motor_cfg_model_db(node->model);
    if (entry == NULL)
    {
        return MOTOR_PROTOCOL_RM_GROUP;
    }
    return (motor_protocol_e)entry->default_protocol;
}

static inline motor_control_mode_e motor_cfg_control_mode(const motor_node_param_t *node)
{
    const motor_model_db_entry_t *entry = NULL;

    if (node == NULL)
    {
        return MOTOR_CONTROL_MODE_CURRENT;
    }
    if (node->control_mode != (uint8_t)MOTOR_CONTROL_MODE_INHERIT)
    {
        return (motor_control_mode_e)node->control_mode;
    }

    entry = motor_cfg_model_db(node->model);
    if (entry == NULL)
    {
        return MOTOR_CONTROL_MODE_CURRENT;
    }
    return (motor_control_mode_e)entry->default_control_mode;
}

static inline uint8_t motor_cfg_has_caps(motor_model_e model, uint8_t caps)
{
    const motor_model_db_entry_t *entry = motor_cfg_model_db(model);

    if (entry == NULL)
    {
        return 0u;
    }

    return ((entry->caps & caps) == caps) ? 1u : 0u;
}

static inline uint16_t motor_cfg_feedback_id(const motor_node_param_t *node)
{
    if (node == NULL)
    {
        return 0u;
    }
    if (node->master_id != 0u)
    {
        return node->master_id;
    }
    return motor_cfg_can_id(node);
}

static inline uint8_t motor_cfg_is_rm_group_protocol(const motor_node_param_t *node)
{
    return (motor_cfg_protocol(node) == MOTOR_PROTOCOL_RM_GROUP) ? 1u : 0u;
}

static inline int16_t motor_cfg_limit_current_abs(int16_t current, int16_t max_abs)
{
    if (max_abs <= 0)
    {
        return current;
    }
    if (current > max_abs)
    {
        return max_abs;
    }
    if (current < -max_abs)
    {
        return (int16_t)-max_abs;
    }
    return current;
}

static inline int16_t motor_cfg_limit_current_model(motor_model_e model, int16_t current)
{
    const motor_model_param_t *m = motor_cfg_model(model);
    if (m == NULL)
    {
        return current;
    }
    return motor_cfg_limit_current_abs(current, m->max_current);
}

static inline int16_t motor_cfg_limit_current_node(const motor_node_param_t *node, int16_t current)
{
    if (node == NULL)
    {
        return current;
    }
    return motor_cfg_limit_current_model(node->model, current);
}

static inline fp32 motor_cfg_reduction_ratio(motor_model_e model)
{
    const motor_model_param_t *m = motor_cfg_model(model);
    return (m != NULL) ? m->reduction_ratio : 1.0f;
}

static inline const motor_model_db_entry_t *motor_cfg_model_db(motor_model_e model)
{
    return motor_model_db_get(model);
}

static inline const motor_model_rx_desc_t *motor_cfg_rx_desc(motor_model_e model)
{
    return motor_model_db_get_rx_desc(model);
}

#endif
