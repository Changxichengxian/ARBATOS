/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "motor_model_db.h"

static const motor_model_rx_desc_t s_motor_model_rx_desc[MOTOR_MODEL_RX_FMT__COUNT] =
{
    [MOTOR_MODEL_RX_FMT_NONE] =
        {
            .speed_rpm_off = MOTOR_MODEL_RX_OFF_NONE,
            .current_meas_off = MOTOR_MODEL_RX_OFF_NONE,
            .current_set_off = MOTOR_MODEL_RX_OFF_NONE,
            .temp_off = MOTOR_MODEL_RX_OFF_NONE,
            .current_set_policy = (uint8_t)MOTOR_MODEL_RX_CUR_SET_NONE,
        },
    [MOTOR_MODEL_RX_FMT_RM_STD] =
        {
            .speed_rpm_off = 2u,
            .current_meas_off = 4u,
            .current_set_off = MOTOR_MODEL_RX_OFF_NONE,
            .temp_off = 6u,
            .current_set_policy = (uint8_t)MOTOR_MODEL_RX_CUR_SET_SAME_AS_MEAS,
        },
    [MOTOR_MODEL_RX_FMT_820R] =
        {
            .speed_rpm_off = 2u,
            .current_meas_off = MOTOR_MODEL_RX_OFF_NONE,
            .current_set_off = MOTOR_MODEL_RX_OFF_NONE,
            .temp_off = MOTOR_MODEL_RX_OFF_NONE,
            .current_set_policy = (uint8_t)MOTOR_MODEL_RX_CUR_SET_NONE,
        },
    [MOTOR_MODEL_RX_FMT_6623] =
        {
            .speed_rpm_off = MOTOR_MODEL_RX_OFF_NONE,
            .current_meas_off = 2u,
            .current_set_off = 4u,
            .temp_off = MOTOR_MODEL_RX_OFF_NONE,
            .current_set_policy = (uint8_t)MOTOR_MODEL_RX_CUR_SET_FROM_FRAME,
        },
};

static const motor_model_db_entry_t s_motor_model_db[MOTOR_MODEL__COUNT] =
{
    [MOTOR_MODEL_3508] =
        {
            .base = {.can_id_base = 0x200u, .max_current = 16384, .reduction_ratio = 19.0f},
            .rx_format = MOTOR_MODEL_RX_FMT_RM_STD,
            .default_protocol = (uint8_t)MOTOR_PROTOCOL_RM_GROUP,
            .default_control_mode = (uint8_t)MOTOR_CONTROL_MODE_CURRENT,
            .caps = 0u,
            .cmd_current_range_abs = 16384,
            .torque_current_range_a = 20.0f,
            .fb_current_meas_range_abs = 0,
        },
    [MOTOR_MODEL_3510] =
        {
            .base = {.can_id_base = 0x200u, .max_current = 16000, .reduction_ratio = 19.0f},
            .rx_format = MOTOR_MODEL_RX_FMT_820R,
            .default_protocol = (uint8_t)MOTOR_PROTOCOL_RM_GROUP,
            .default_control_mode = (uint8_t)MOTOR_CONTROL_MODE_CURRENT,
            .caps = 0u,
            .cmd_current_range_abs = 32767,
            .torque_current_range_a = 0.0f,
            .fb_current_meas_range_abs = 0,
        },
    [MOTOR_MODEL_2006] =
        {
            .base = {.can_id_base = 0x200u, .max_current = 10000, .reduction_ratio = 25.0f},
            .rx_format = MOTOR_MODEL_RX_FMT_RM_STD,
            .default_protocol = (uint8_t)MOTOR_PROTOCOL_RM_GROUP,
            .default_control_mode = (uint8_t)MOTOR_CONTROL_MODE_CURRENT,
            .caps = 0u,
            .cmd_current_range_abs = 10000,
            .torque_current_range_a = 10.0f,
            .fb_current_meas_range_abs = 0,
        },
    [MOTOR_MODEL_6020] =
        {
            .base = {.can_id_base = 0x204u, .max_current = 16384, .reduction_ratio = 1.0f},
            .rx_format = MOTOR_MODEL_RX_FMT_RM_STD,
            .default_protocol = (uint8_t)MOTOR_PROTOCOL_RM_GROUP,
            .default_control_mode = (uint8_t)MOTOR_CONTROL_MODE_CURRENT,
            .caps = 0u,
            .cmd_current_range_abs = 16384,
            .torque_current_range_a = 3.0f,
            .fb_current_meas_range_abs = 0,
        },
    [MOTOR_MODEL_6623] =
        {
            .base = {.can_id_base = 0x204u, .max_current = 5000, .reduction_ratio = 1.0f},
            .rx_format = MOTOR_MODEL_RX_FMT_6623,
            .default_protocol = (uint8_t)MOTOR_PROTOCOL_RM_GROUP,
            .default_control_mode = (uint8_t)MOTOR_CONTROL_MODE_CURRENT,
            .caps = 0u,
            .cmd_current_range_abs = 5000,
            .torque_current_range_a = 0.0f,
            .fb_current_meas_range_abs = 13000,
        },
    [MOTOR_MODEL_DM_J4310_2EC_V11] =
        {
            .base = {.can_id_base = 0x000u, .max_current = 0, .reduction_ratio = 10.0f},
            .rx_format = MOTOR_MODEL_RX_FMT_NONE,
            .default_protocol = (uint8_t)MOTOR_PROTOCOL_DM_3MODE,
            .default_control_mode = (uint8_t)MOTOR_CONTROL_MODE_MIT,
            .caps = (uint8_t)(MOTOR_MODEL_CAP_MIT |
                               MOTOR_MODEL_CAP_POS_VEL |
                               MOTOR_MODEL_CAP_SPEED),
            .cmd_current_range_abs = 0,
            .torque_current_range_a = 0.0f,
            .fb_current_meas_range_abs = 0,
        },
    [MOTOR_MODEL_DM_J4310_2EC_V12] =
        {
            .base = {.can_id_base = 0x000u, .max_current = 0, .reduction_ratio = 10.0f},
            .rx_format = MOTOR_MODEL_RX_FMT_NONE,
            .default_protocol = (uint8_t)MOTOR_PROTOCOL_DM_EXT_V2,
            .default_control_mode = (uint8_t)MOTOR_CONTROL_MODE_MIT,
            .caps = (uint8_t)(MOTOR_MODEL_CAP_MIT |
                               MOTOR_MODEL_CAP_POS_VEL |
                               MOTOR_MODEL_CAP_SPEED |
                               MOTOR_MODEL_CAP_FORCE_POS |
                               MOTOR_MODEL_CAP_CAN_PARAM_RW |
                               MOTOR_MODEL_CAP_CAN_SAVE |
                               MOTOR_MODEL_CAP_CAN_BAUD_RW |
                               MOTOR_MODEL_CAP_ENABLE_CMD),
            .cmd_current_range_abs = 0,
            .torque_current_range_a = 0.0f,
            .fb_current_meas_range_abs = 0,
        },
    [MOTOR_MODEL_DM_J8009_2EC_V10] =
        {
            .base = {.can_id_base = 0x000u, .max_current = 0, .reduction_ratio = 9.0f},
            .rx_format = MOTOR_MODEL_RX_FMT_NONE,
            .default_protocol = (uint8_t)MOTOR_PROTOCOL_DM_3MODE,
            .default_control_mode = (uint8_t)MOTOR_CONTROL_MODE_MIT,
            .caps = (uint8_t)(MOTOR_MODEL_CAP_MIT |
                               MOTOR_MODEL_CAP_POS_VEL |
                               MOTOR_MODEL_CAP_SPEED),
            .cmd_current_range_abs = 0,
            .torque_current_range_a = 0.0f,
            .fb_current_meas_range_abs = 0,
        },
    [MOTOR_MODEL_DM_J8006_2EC_V11] =
        {
            .base = {.can_id_base = 0x000u, .max_current = 0, .reduction_ratio = 6.0f},
            .rx_format = MOTOR_MODEL_RX_FMT_NONE,
            .default_protocol = (uint8_t)MOTOR_PROTOCOL_DM_3MODE,
            .default_control_mode = (uint8_t)MOTOR_CONTROL_MODE_MIT,
            .caps = (uint8_t)(MOTOR_MODEL_CAP_MIT |
                               MOTOR_MODEL_CAP_POS_VEL |
                               MOTOR_MODEL_CAP_SPEED),
            .cmd_current_range_abs = 0,
            .torque_current_range_a = 0.0f,
            .fb_current_meas_range_abs = 0,
        },
    [MOTOR_MODEL_DM_J8006_2EC_V10] =
        {
            .base = {.can_id_base = 0x000u, .max_current = 0, .reduction_ratio = 6.0f},
            .rx_format = MOTOR_MODEL_RX_FMT_NONE,
            .default_protocol = (uint8_t)MOTOR_PROTOCOL_DM_EXT_V1,
            .default_control_mode = (uint8_t)MOTOR_CONTROL_MODE_MIT,
            .caps = (uint8_t)(MOTOR_MODEL_CAP_MIT |
                               MOTOR_MODEL_CAP_POS_VEL |
                               MOTOR_MODEL_CAP_SPEED |
                               MOTOR_MODEL_CAP_FORCE_POS |
                               MOTOR_MODEL_CAP_CAN_PARAM_RW |
                               MOTOR_MODEL_CAP_CAN_SAVE |
                               MOTOR_MODEL_CAP_CAN_BAUD_RW),
            .cmd_current_range_abs = 0,
            .torque_current_range_a = 0.0f,
            .fb_current_meas_range_abs = 0,
        },
};

const motor_model_db_entry_t *motor_model_db_get(motor_model_e model)
{
    if ((uint32_t)model >= (uint32_t)MOTOR_MODEL__COUNT)
    {
        return 0;
    }

    return &s_motor_model_db[model];
}

const motor_model_rx_desc_t *motor_model_db_get_rx_desc(motor_model_e model)
{
    const motor_model_db_entry_t *entry = motor_model_db_get(model);
    motor_model_rx_format_e fmt = MOTOR_MODEL_RX_FMT_RM_STD;

    if (entry != 0)
    {
        fmt = entry->rx_format;
    }

    if ((uint32_t)fmt >= (uint32_t)MOTOR_MODEL_RX_FMT__COUNT)
    {
        fmt = MOTOR_MODEL_RX_FMT_RM_STD;
    }

    return &s_motor_model_rx_desc[fmt];
}
