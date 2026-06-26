/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

#include "RobotConfig.h"
#include "MotorModelDb.h"

static inline const MotorModelDbEntry *MotorCfgModelDb(MotorModel model);
static inline const MotorModelRxDesc *MotorCfgRxDesc(MotorModel model);

// 取共享电机型号参数；型号越界时返回 NULL。
static inline const MotorModelParam *MotorCfgModel(MotorModel model)
{
    const MotorModelDbEntry *entry = MotorCfgModelDb(model);
    if (entry == NULL)
    {
        return NULL;
    }
    return &entry->base;
}

static inline uint8_t MotorCfgEncoderBits(MotorModel model)
{
    const MotorModelDbEntry *entry = MotorCfgModelDb(model);

    if (entry != NULL && entry->specs.encoder_bits > 0u && entry->specs.encoder_bits <= 16u)
    {
        return entry->specs.encoder_bits;
    }
    return 13u;
}

static inline uint32_t MotorCfgEncoderRange(MotorModel model)
{
    return (uint32_t)1u << MotorCfgEncoderBits(model);
}

static inline motor_transport_e MotorCfgTransport(const motor_node_param_t *node)
{
    if (node == NULL || node->transport == (uint8_t)MOTOR_TRANSPORT_INHERIT)
    {
        return MOTOR_TRANSPORT_CAN;
    }
    return (motor_transport_e)node->transport;
}

static inline uint8_t MotorCfgNodeId(const motor_node_param_t *node)
{
    return (node != NULL) ? node->can_id : 0u;
}

static inline uint8_t MotorCfgCanBus(uint8_t fallback_bus, const motor_node_param_t *node)
{
    if (node != NULL && (node->can_bus == 1u || node->can_bus == 2u))
    {
        return node->can_bus;
    }
    return fallback_bus;
}

// 把“轴上的电机编号”换成真正的 CAN 标准帧 ID；返回 0 表示这个轴未启用。
static inline uint16_t MotorCfgCanId(const motor_node_param_t *node)
{
    if (node == NULL)
    {
        return 0u;
    }
    if (MotorCfgTransport(node) != MOTOR_TRANSPORT_CAN)
    {
        return 0u;
    }
    if (node->can_id == 0u)
    {
        return 0u;
    }
    const MotorModelParam *m = MotorCfgModel(node->model);
    if (m == NULL)
    {
        return 0u;
    }
    return (uint16_t)(m->can_id_base + (uint16_t)node->can_id);
}

// 取这个轴实际使用的电机协议；轴上没单独指定时，继承型号默认协议。
static inline motor_protocol_e MotorCfgProtocol(const motor_node_param_t *node)
{
    const MotorModelDbEntry *entry = NULL;

    if (node == NULL)
    {
        return MOTOR_PROTOCOL_RM_GROUP;
    }
    if (node->protocol != (uint8_t)MOTOR_PROTOCOL_INHERIT)
    {
        return (motor_protocol_e)node->protocol;
    }

    entry = MotorCfgModelDb(node->model);
    if (entry == NULL)
    {
        return MOTOR_PROTOCOL_RM_GROUP;
    }
    return (motor_protocol_e)entry->default_protocol;
}

// 取这个轴实际使用的控制方式；轴上没单独指定时，继承型号默认控制方式。
static inline motor_control_mode_e MotorCfgControlMode(const motor_node_param_t *node)
{
    const MotorModelDbEntry *entry = NULL;

    if (node == NULL)
    {
        return MOTOR_CONTROL_MODE_CURRENT;
    }
    if (node->control_mode != (uint8_t)MOTOR_CONTROL_MODE_INHERIT)
    {
        return (motor_control_mode_e)node->control_mode;
    }

    entry = MotorCfgModelDb(node->model);
    if (entry == NULL)
    {
        return MOTOR_CONTROL_MODE_CURRENT;
    }
    return (motor_control_mode_e)entry->default_control_mode;
}

// 判断某个电机型号是否同时具备指定能力位。
static inline uint8_t MotorCfgHasCaps(MotorModel model, uint8_t caps)
{
    const MotorModelDbEntry *entry = MotorCfgModelDb(model);

    if (entry == NULL)
    {
        return 0u;
    }

    return ((entry->caps & caps) == caps) ? 1u : 0u;
}

// 取 MIT 控制需要的限幅参数；型号不支持 MIT 或参数不完整时返回 NULL。
static inline const MotorModelMitLimits *MotorCfgMitLimits(const motor_node_param_t *node)
{
    const MotorModelDbEntry *entry = NULL;

    if (node == NULL)
    {
        return NULL;
    }

    entry = MotorCfgModelDb(node->model);
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
static inline uint16_t MotorCfgFeedbackId(const motor_node_param_t *node)
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
    return MotorCfgCanId(node);
}

// 判断这个轴是否走大疆一组四电机的电流帧格式。
static inline uint8_t MotorCfgIsRmGroupProtocol(const motor_node_param_t *node)
{
    return (MotorCfgProtocol(node) == MOTOR_PROTOCOL_RM_GROUP) ? 1u : 0u;
}

// 按绝对值限幅电流；max_abs<=0 表示不限制。
static inline int16_t MotorCfgLimitCurrentAbs(int16_t current, int16_t max_abs)
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
static inline int16_t MotorCfgLimitCurrentModel(MotorModel model, int16_t current)
{
    const MotorModelParam *m = MotorCfgModel(model);
    if (m == NULL)
    {
        return current;
    }
    return MotorCfgLimitCurrentAbs(current, m->max_current);
}

// 按轴上配置的电机型号限幅，业务控制只需要把轴节点传进来。
static inline int16_t MotorCfgLimitCurrentNode(const motor_node_param_t *node, int16_t current)
{
    if (node == NULL)
    {
        return current;
    }
    return MotorCfgLimitCurrentModel(node->model, current);
}

// 取电机减速比；型号无效时按 1.0 处理，避免上层除零或出错。
static inline fp32 MotorCfgReductionRatio(MotorModel model)
{
    const MotorModelParam *m = MotorCfgModel(model);
    return (m != NULL) ? m->reduction_ratio : 1.0f;
}

// 取共享电机能力表，里面放协议、MIT 限幅、反馈解析这些跨目标信息。
static inline const MotorModelDbEntry *MotorCfgModelDb(MotorModel model)
{
    return MotorModelDbGet(model);
}

// 取反馈解析描述，用来按不同电机型号拆 CAN 反馈帧。
static inline const MotorModelRxDesc *MotorCfgRxDesc(MotorModel model)
{
    return MotorModelDbGetRxDesc(model);
}

#endif
