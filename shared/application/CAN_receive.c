/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "CAN_receive.h"

#include "cmsis_os2.h"

#include "bsp_can.h"
#include "detect_task.h"
#include "motor_config.h"
#include "sdlog.h"
#include "watch.h"

static motor_measure_t motor_chassis[4];
static motor_measure_t motor_yaw;
static motor_measure_t motor_yaw_upper;
static motor_measure_t motor_trigger;
static motor_measure_t motor_pitch;
static motor_measure_t motor_friction[4];
static volatile uint8_t last_can1ff_status = 0u;

__weak uint8_t CAN_rx_process_extra_frame(uint8_t bus, uint16_t std_id, uint8_t dlc, const uint8_t data[8]);

static int16_t can_rx_read_s16_be(const uint8_t *ptr)
{
    return (int16_t)(((uint16_t)ptr[0] << 8) | (uint16_t)ptr[1]);
}

static void can_rx_unpack_motor_measure(motor_measure_t *measure, motor_model_e model, const uint8_t data[8])
{
    const motor_model_rx_desc_t *rx = motor_cfg_rx_desc(model);

    if (measure == NULL || data == NULL || rx == NULL)
    {
        return;
    }

    if (rx->speed_rpm_off == MOTOR_MODEL_RX_OFF_NONE &&
        rx->current_meas_off == MOTOR_MODEL_RX_OFF_NONE &&
        rx->current_set_off == MOTOR_MODEL_RX_OFF_NONE &&
        rx->temp_off == MOTOR_MODEL_RX_OFF_NONE)
    {
        return;
    }

    measure->last_ecd = measure->ecd;
    measure->ecd = (uint16_t)(((uint16_t)data[0] << 8) | (uint16_t)data[1]);
    measure->speed_rpm = (rx->speed_rpm_off != MOTOR_MODEL_RX_OFF_NONE) ? can_rx_read_s16_be(&data[rx->speed_rpm_off]) : 0;
    measure->temperate = (rx->temp_off != MOTOR_MODEL_RX_OFF_NONE) ? data[rx->temp_off] : 0u;

    switch ((motor_model_rx_current_set_policy_e)rx->current_set_policy)
    {
    case MOTOR_MODEL_RX_CUR_SET_FROM_FRAME:
        measure->given_current =
            (rx->current_set_off != MOTOR_MODEL_RX_OFF_NONE) ? can_rx_read_s16_be(&data[rx->current_set_off]) : 0;
        break;
    case MOTOR_MODEL_RX_CUR_SET_SAME_AS_MEAS:
        measure->given_current =
            (rx->current_meas_off != MOTOR_MODEL_RX_OFF_NONE) ? can_rx_read_s16_be(&data[rx->current_meas_off]) : 0;
        break;
    default:
        measure->given_current = 0;
        break;
    }
}

static uint8_t can_match_nodes(uint16_t std_id, const motor_node_param_t *nodes, uint8_t count, uint8_t *out_idx)
{
    if (nodes == NULL || count == 0u)
    {
        return 0u;
    }

    for (uint8_t i = 0u; i < count; i++)
    {
        if (std_id == motor_cfg_feedback_id(&nodes[i]))
        {
            if (out_idx != NULL)
            {
                *out_idx = i;
            }
            return 1u;
        }
    }

    return 0u;
}

static uint8_t can_rx_process_node_frame(motor_measure_t *measure,
                                         const motor_node_param_t *node,
                                         uint8_t bus,
                                         uint16_t std_id,
                                         uint8_t dlc,
                                         const uint8_t data[8],
                                         uint8_t detect_toe,
                                         uint8_t use_detect)
{
    if (node == NULL)
    {
        return 0u;
    }

    if (motor_cfg_is_rm_group_protocol(node) != 0u)
    {
        can_rx_unpack_motor_measure(measure, node->model, data);
        if (use_detect != 0u)
        {
            detect_hook(detect_toe);
        }
        return 1u;
    }

    if (CAN_rx_process_extra_frame(bus, std_id, dlc, data) != 0u)
    {
        return 1u;
    }

    watch_task_error(WATCH_TASK_CAN_FEEDBACK_RX);
    return 1u;
}

__weak uint8_t CAN_rx_process_extra_frame(uint8_t bus, uint16_t std_id, uint8_t dlc, const uint8_t data[8])
{
    (void)bus;
    (void)std_id;
    (void)dlc;
    (void)data;
    return 0u;
}

void CAN_rx_process_frame(uint8_t bus, uint16_t std_id, uint8_t dlc, const uint8_t data[8])
{
    if (data == NULL)
    {
        return;
    }

    if (bus == 1u)
    {
        uint8_t idx = 0u;
        if (can_match_nodes(std_id, g_config.motor.chassis, 4u, &idx) != 0u)
        {
            (void)can_rx_process_node_frame(&motor_chassis[idx],
                                            &g_config.motor.chassis[idx],
                                            bus,
                                            std_id,
                                            dlc,
                                            data,
                                            (uint8_t)(CHASSIS_MOTOR1_TOE + idx),
                                            1u);
            return;
        }
        if (std_id == motor_cfg_feedback_id(&g_config.motor.yaw))
        {
            (void)can_rx_process_node_frame(&motor_yaw,
                                            &g_config.motor.yaw,
                                            bus,
                                            std_id,
                                            dlc,
                                            data,
                                            YAW_GIMBAL_MOTOR_TOE,
                                            1u);
            return;
        }
        if (std_id == motor_cfg_feedback_id(&g_config.motor.yaw_upper))
        {
            (void)can_rx_process_node_frame(&motor_yaw_upper,
                                            &g_config.motor.yaw_upper,
                                            bus,
                                            std_id,
                                            dlc,
                                            data,
                                            0u,
                                            0u);
            return;
        }
        if (std_id == motor_cfg_feedback_id(&g_config.motor.trigger))
        {
            (void)can_rx_process_node_frame(&motor_trigger,
                                            &g_config.motor.trigger,
                                            bus,
                                            std_id,
                                            dlc,
                                            data,
                                            TRIGGER_MOTOR_TOE,
                                            1u);
            return;
        }
        if (std_id == motor_cfg_feedback_id(&g_config.motor.pitch))
        {
            (void)can_rx_process_node_frame(&motor_pitch,
                                            &g_config.motor.pitch,
                                            bus,
                                            std_id,
                                            dlc,
                                            data,
                                            PITCH_GIMBAL_MOTOR_TOE,
                                            1u);
            return;
        }
    }
    else if (bus == 2u)
    {
        uint8_t idx = 0u;
        if (can_match_nodes(std_id, g_config.motor.friction, 4u, &idx) != 0u)
        {
            (void)can_rx_process_node_frame(&motor_friction[idx],
                                            &g_config.motor.friction[idx],
                                            bus,
                                            std_id,
                                            dlc,
                                            data,
                                            0u,
                                            0u);
            return;
        }
    }

    (void)CAN_rx_process_extra_frame(bus, std_id, dlc, data);
}

void CAN_cmd_rm_group(uint8_t bus,
                      uint16_t group_id,
                      int16_t motor1,
                      int16_t motor2,
                      int16_t motor3,
                      int16_t motor4)
{
    uint8_t data[8] = {0};
    data[0] = (uint8_t)(motor1 >> 8);
    data[1] = (uint8_t)motor1;
    data[2] = (uint8_t)(motor2 >> 8);
    data[3] = (uint8_t)motor2;
    data[4] = (uint8_t)(motor3 >> 8);
    data[5] = (uint8_t)motor3;
    data[6] = (uint8_t)(motor4 >> 8);
    data[7] = (uint8_t)motor4;

    if (bus == 1u && group_id == (uint16_t)CAN_GIMBAL_ALL_ID)
    {
        last_can1ff_status = (uint8_t)bsp_can_tx(bus, group_id, data, 8u);
        return;
    }

    (void)bsp_can_tx(bus, group_id, data, 8u);
}

void CAN_cmd_pitch_3510(int16_t pitch)
{
    uint8_t data[8] = {0};
    data[4] = (uint8_t)(pitch >> 8);
    data[5] = (uint8_t)pitch;
    (void)bsp_can_tx(1u, (uint16_t)CAN_CHASSIS_ALL_ID, data, 8u);
}

void CAN_cmd_friction_3510(int16_t f1, int16_t f2, int16_t f3, int16_t f4)
{
    CAN_cmd_rm_group(2u, (uint16_t)CAN_CHASSIS_ALL_ID, f1, f2, f3, f4);
}

void CAN_cmd_chassis_reset_ID(void)
{
    uint8_t data[8] = {0};
    (void)bsp_can_tx(1u, 0x700u, data, 8u);
}

void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    CAN_cmd_rm_group(1u, (uint16_t)CAN_CHASSIS_ALL_ID, motor1, motor2, motor3, motor4);
}

void CAN_cmd_chassis_1ff(int16_t motor205, int16_t motor206, int16_t motor207, int16_t motor208)
{
    CAN_cmd_rm_group(1u, (uint16_t)CAN_GIMBAL_ALL_ID, motor205, motor206, motor207, motor208);
}

const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
    return &motor_yaw;
}

const motor_measure_t *get_yaw_upper_gimbal_motor_measure_point(void)
{
    return &motor_yaw_upper;
}

const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
    return &motor_pitch;
}

const motor_measure_t *get_trigger_motor_measure_point(void)
{
    return &motor_trigger;
}

const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
    return &motor_chassis[(i & 0x03u)];
}

const motor_measure_t *get_friction_motor_measure_point(uint8_t i)
{
    return &motor_friction[(i & 0x03u)];
}

uint8_t CAN_get_last_1ff_status(void)
{
    return last_can1ff_status;
}

uint32_t CAN_get_last_can1_error(void)
{
    return bsp_can_get_last_error(1u);
}

uint32_t CAN_get_last_can2_error(void)
{
    return bsp_can_get_last_error(2u);
}

uint32_t CAN_get_can1_rx_drop_count(void)
{
    return bsp_can_rx_get_drop_count(1u);
}

uint32_t CAN_get_can2_rx_drop_count(void)
{
    return bsp_can_rx_get_drop_count(2u);
}

uint32_t CAN_get_can1_tx_count(void)
{
    return bsp_can_get_tx_count(1u);
}

uint32_t CAN_get_can2_tx_count(void)
{
    return bsp_can_get_tx_count(2u);
}

uint32_t CAN_get_can1_tx_fail_count(void)
{
    return bsp_can_get_tx_fail_count(1u);
}

uint32_t CAN_get_can2_tx_fail_count(void)
{
    return bsp_can_get_tx_fail_count(2u);
}
