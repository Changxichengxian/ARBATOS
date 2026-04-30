/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "CAN_receive.h"

#include "string.h"
#include "cmsis_os.h"

#include "bsp_can.h"
#include "detect_task.h"
#include "motor_config.h"
#include "sdlog.h"

static motor_measure_t motor_chassis[4];
static motor_measure_t motor_yaw;
static motor_measure_t motor_yaw_upper;
static motor_measure_t motor_trigger;
static motor_measure_t motor_pitch;
static motor_measure_t motor_friction[4];
static volatile uint8_t last_can1ff_status = 0u;
static uint8_t can1_chassis_log_decim[4];
static uint8_t can2_friction_log_decim[4];
static uint8_t can1_trigger_log_decim;

#define CAN_RX_LOG_DIV_CHASSIS 2u
#define CAN_RX_LOG_DIV_FRICTION_TRIGGER 5u

__weak uint8_t CAN_rx_process_extra_frame(uint8_t bus, uint16_t std_id, uint8_t dlc, const uint8_t data[8]);

// 大疆反馈帧里 16 位整数是高字节在前，这里统一做一次读取。
static int16_t can_rx_read_s16_be(const uint8_t *ptr)
{
    return (int16_t)(((uint16_t)ptr[0] << 8) | (uint16_t)ptr[1]);
}

// 按电机型号的反馈描述拆包，避免把不同电机的字段位置写死在任务里。
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

// 在一组轴装配里按反馈 ID 找命中的轴，下标返回给调用者。
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

// 按固定分频抽样记录 CAN 帧，避免高频电机反馈把日志写满。
static uint8_t can_rx_log_decim_hit(uint8_t *counter, uint8_t div)
{
    if (counter == NULL || div <= 1u)
    {
        return 1u;
    }

    (*counter)++;
    if (*counter < div)
    {
        return 0u;
    }

    *counter = 0u;
    return 1u;
}

// 判断这帧是否需要写日志；不同轴按带宽和调试价值使用不同抽样频率。
static uint8_t can_should_log_frame(uint8_t bus, uint16_t std_id)
{
    if (bus == 1u)
    {
        uint8_t idx = 0u;
        if (can_match_nodes(std_id, g_config.motor.chassis, 4u, &idx) != 0u)
        {
            return can_rx_log_decim_hit(&can1_chassis_log_decim[idx], CAN_RX_LOG_DIV_CHASSIS);
        }
        if (std_id == motor_cfg_feedback_id(&g_config.motor.yaw) ||
            std_id == motor_cfg_feedback_id(&g_config.motor.yaw_upper) ||
            std_id == motor_cfg_feedback_id(&g_config.motor.pitch))
        {
            return 1u;
        }
        if (std_id == motor_cfg_feedback_id(&g_config.motor.trigger))
        {
            return can_rx_log_decim_hit(&can1_trigger_log_decim, CAN_RX_LOG_DIV_FRICTION_TRIGGER);
        }
    }
    else if (bus == 2u)
    {
        uint8_t idx = 0u;
        if (can_match_nodes(std_id, g_config.motor.friction, 4u, &idx) != 0u)
        {
            return can_rx_log_decim_hit(&can2_friction_log_decim[idx], CAN_RX_LOG_DIV_FRICTION_TRIGGER);
        }
    }

    return 0u;
}

// 留给目标工程扩展特殊反馈帧；默认不处理。
__weak uint8_t CAN_rx_process_extra_frame(uint8_t bus, uint16_t std_id, uint8_t dlc, const uint8_t data[8])
{
    (void)bus;
    (void)std_id;
    (void)dlc;
    (void)data;
    return 0u;
}

// CAN 接收总入口：先按总线和轴装配找到归属，再按电机型号拆反馈。
void CAN_rx_process_frame(uint8_t bus, uint16_t std_id, uint8_t dlc, const uint8_t data[8])
{
    if (data == NULL)
    {
        return;
    }

    if (can_should_log_frame(bus, std_id) != 0u)
    {
        sdlog_can_rx_t pkt;
        pkt.bus = bus;
        pkt.dlc = dlc;
        pkt.std_id = std_id;
        for (uint8_t i = 0u; i < (uint8_t)sizeof(pkt.data); i++)
        {
            pkt.data[i] = data[i];
        }
        sdlog_write(SDLOG_TAG_CAN_RX, &pkt, (uint16_t)sizeof(pkt));
    }

    if (bus == 1u)
    {
        uint8_t idx = 0u;
        if (can_match_nodes(std_id, g_config.motor.chassis, 4u, &idx) != 0u)
        {
            can_rx_unpack_motor_measure(&motor_chassis[idx], g_config.motor.chassis[idx].model, data);
            detect_hook(CHASSIS_MOTOR1_TOE + idx);
            return;
        }
        if (std_id == motor_cfg_feedback_id(&g_config.motor.yaw))
        {
            can_rx_unpack_motor_measure(&motor_yaw, g_config.motor.yaw.model, data);
            detect_hook(YAW_GIMBAL_MOTOR_TOE);
            return;
        }
        if (std_id == motor_cfg_feedback_id(&g_config.motor.yaw_upper))
        {
            can_rx_unpack_motor_measure(&motor_yaw_upper, g_config.motor.yaw_upper.model, data);
            return;
        }
        if (std_id == motor_cfg_feedback_id(&g_config.motor.trigger))
        {
            can_rx_unpack_motor_measure(&motor_trigger, g_config.motor.trigger.model, data);
            detect_hook(TRIGGER_MOTOR_TOE);
            return;
        }
        if (std_id == motor_cfg_feedback_id(&g_config.motor.pitch))
        {
            can_rx_unpack_motor_measure(&motor_pitch, g_config.motor.pitch.model, data);
            detect_hook(PITCH_GIMBAL_MOTOR_TOE);
            return;
        }
    }
    else if (bus == 2u)
    {
        uint8_t idx = 0u;
        if (can_match_nodes(std_id, g_config.motor.friction, 4u, &idx) != 0u)
        {
            can_rx_unpack_motor_measure(&motor_friction[idx], g_config.motor.friction[idx].model, data);
            return;
        }
    }

    (void)CAN_rx_process_extra_frame(bus, std_id, dlc, data);
}

// 发送大疆一组四电机电流帧；输出位置由电机 CAN ID 自动放进 0x200 或 0x1FF。
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
