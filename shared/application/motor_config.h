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

// 取目标自己的只读电机型号参数；型号越界时返回 NULL。
static inline const motor_model_param_t *motor_cfg_model(motor_model_e model)
{
    if ((uint32_t)model >= (uint32_t)MOTOR_MODEL__COUNT)
    {
        return NULL;
    }
    return &g_motor_config.model[model];
}

static inline motor_transport_e motor_cfg_transport(const motor_node_param_t *node)
{
    if (node == NULL || node->transport == (uint8_t)MOTOR_TRANSPORT_INHERIT)
    {
        return MOTOR_TRANSPORT_CAN;
    }
    return (motor_transport_e)node->transport;
}

static inline uint8_t motor_cfg_node_id(const motor_node_param_t *node)
{
    return (node != NULL) ? node->can_id : 0u;
}

static inline uint8_t motor_cfg_can_bus(uint8_t fallback_bus, const motor_node_param_t *node)
{
    if (node != NULL && (node->can_bus == 1u || node->can_bus == 2u))
    {
        return node->can_bus;
    }
    return fallback_bus;
}

// 把“轴上的电机编号”换成真正的 CAN 标准帧 ID；返回 0 表示这个轴未启用。
static inline uint16_t motor_cfg_can_id(const motor_node_param_t *node)
{
    if (node == NULL)
    {
        return 0u;
    }
    if (motor_cfg_transport(node) != MOTOR_TRANSPORT_CAN)
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

// 取这个轴实际使用的电机协议；轴上没单独指定时，继承型号默认协议。
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

// 取这个轴实际使用的控制方式；轴上没单独指定时，继承型号默认控制方式。
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

// 判断某个电机型号是否同时具备指定能力位。
static inline uint8_t motor_cfg_has_caps(motor_model_e model, uint8_t caps)
{
    const motor_model_db_entry_t *entry = motor_cfg_model_db(model);

    if (entry == NULL)
    {
        return 0u;
    }

    return ((entry->caps & caps) == caps) ? 1u : 0u;
}

// 取 MIT 控制需要的限幅参数；型号不支持 MIT 或参数不完整时返回 NULL。
static inline const motor_model_mit_limits_t *motor_cfg_mit_limits(const motor_node_param_t *node)
{
    const motor_model_db_entry_t *entry = NULL;

    if (node == NULL)
    {
        return NULL;
    }

    entry = motor_cfg_model_db(node->model);
    if (entry == NULL || ((entry->caps & MOTOR_MODEL_CAP_MIT) == 0u))
    {
        return NULL;
    }

    if (entry->mit_limits.position_max <= 0.0f ||
        entry->mit_limits.velocity_max <= 0.0f ||
        entry->mit_limits.kp_max <= 0.0f ||
        entry->mit_limits.kd_max <= 0.0f ||
        entry->mit_limits.torque_max <= 0.0f)
    {
        return NULL;
    }

    return &entry->mit_limits;
}

// 取反馈帧 ID；有些电机反馈 ID 不等于命令 ID，所以允许 master_id 覆盖。
static inline uint16_t motor_cfg_feedback_id(const motor_node_param_t *node)
{
    if (node == NULL)
    {
        return 0u;
    }
    if (node->feedback_id_enable != 0u)
    {
        return node->feedback_id;
    }
    if (node->master_id != 0u)
    {
        return node->master_id;
    }
    return motor_cfg_can_id(node);
}

// 判断这个轴是否走大疆一组四电机的电流帧格式。
static inline uint8_t motor_cfg_is_rm_group_protocol(const motor_node_param_t *node)
{
    return (motor_cfg_protocol(node) == MOTOR_PROTOCOL_RM_GROUP) ? 1u : 0u;
}

// 按绝对值限幅电流；max_abs<=0 表示不限制。
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

// 按电机型号表里的最大电流限幅。
static inline int16_t motor_cfg_limit_current_model(motor_model_e model, int16_t current)
{
    const motor_model_param_t *m = motor_cfg_model(model);
    if (m == NULL)
    {
        return current;
    }
    return motor_cfg_limit_current_abs(current, m->max_current);
}

// 按轴上配置的电机型号限幅，业务控制只需要把轴节点传进来。
static inline int16_t motor_cfg_limit_current_node(const motor_node_param_t *node, int16_t current)
{
    if (node == NULL)
    {
        return current;
    }
    return motor_cfg_limit_current_model(node->model, current);
}

// 取电机减速比；型号无效时按 1.0 处理，避免上层除零或出错。
static inline fp32 motor_cfg_reduction_ratio(motor_model_e model)
{
    const motor_model_param_t *m = motor_cfg_model(model);
    return (m != NULL) ? m->reduction_ratio : 1.0f;
}

// 取共享电机能力表，里面放协议、MIT 限幅、反馈解析这些跨目标信息。
static inline const motor_model_db_entry_t *motor_cfg_model_db(motor_model_e model)
{
    return motor_model_db_get(model);
}

// 取反馈解析描述，用来按不同电机型号拆 CAN 反馈帧。
static inline const motor_model_rx_desc_t *motor_cfg_rx_desc(motor_model_e model)
{
    return motor_model_db_get_rx_desc(model);
}

#endif
