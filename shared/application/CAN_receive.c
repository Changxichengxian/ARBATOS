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
#include "actuator_cmd.h"
#include "can_mit_motor_driver.h"
#include "detect_task.h"
#include "motor_config.h"
#include "sdlog.h"
#include "watch.h"

#include <string.h>

#define CAN_RX_TWO_PI 6.28318530718f
#define CAN_RX_RADPS_TO_RPM 9.54929659f
#define CAN_RX_RPM_TO_RADPS 0.104719755f
#define CAN_RX_ECD_RANGE_F 8191.0f

static motor_measure_t motor_chassis[4];
static motor_measure_t motor_yaw;
static motor_measure_t motor_yaw_upper;
static motor_measure_t motor_trigger;
static motor_measure_t motor_pitch;
static motor_measure_t motor_friction[4];
static volatile uint8_t last_can1ff_status = 0u;

__weak uint8_t CAN_rx_process_extra_frame(uint8_t bus, uint16_t std_id, uint8_t dlc, const uint8_t data[8]);

// 大疆反馈帧里 16 位整数是高字节在前，这里统一做一次读取。
static int16_t can_rx_read_s16_be(const uint8_t *ptr)
{
    return (int16_t)(((uint16_t)ptr[0] << 8) | (uint16_t)ptr[1]);
}

// 把连续角度折回 [0, 2pi)，用于伪造旧编码器计数。
static fp32 can_rx_wrap_0_2pi(fp32 angle)
{
    int32_t turns;

    if (angle > CAN_RX_TWO_PI || angle < -CAN_RX_TWO_PI)
    {
        turns = (int32_t)(angle / CAN_RX_TWO_PI);
        angle -= (fp32)turns * CAN_RX_TWO_PI;
    }
    while (angle < 0.0f)
    {
        angle += CAN_RX_TWO_PI;
    }
    while (angle >= CAN_RX_TWO_PI)
    {
        angle -= CAN_RX_TWO_PI;
    }
    return angle;
}

// MIT 电机返回弧度位置，旧控制代码仍看 ecd，所以这里转换成 0..8191。
static uint16_t can_rx_position_to_ecd(fp32 position)
{
    const fp32 wrapped = can_rx_wrap_0_2pi(position);
    uint32_t ecd = (uint32_t)((wrapped * CAN_RX_ECD_RANGE_F / CAN_RX_TWO_PI) + 0.5f);

    if (ecd > 8191u)
    {
        ecd = 8191u;
    }
    return (uint16_t)ecd;
}

// 浮点量写回旧结构前做 int16 饱和，避免溢出反号。
static int16_t can_rx_float_to_i16_saturated(fp32 x)
{
    if (x > 32767.0f)
    {
        return 32767;
    }
    if (x < -32768.0f)
    {
        return -32768;
    }
    return (int16_t)x;
}

// 把 MIT 力矩反馈映射成“类似电流”的数值，兼容旧观察和日志字段。
static int16_t can_rx_torque_to_current_like(const can_mit_motor_limits_t *limits, fp32 torque)
{
    fp32 scaled;

    if (limits == NULL || limits->torque_max <= 0.0f)
    {
        return 0;
    }

    scaled = torque * 32767.0f / limits->torque_max;
    return can_rx_float_to_i16_saturated(scaled);
}

// 把大疆类反馈帧同步到通用执行器反馈，供新控制链按轴读取。
static void can_rx_update_actuator_feedback_from_measure(actuator_id_e actuator_id,
                                                         uint8_t bus,
                                                         uint16_t std_id,
                                                         uint8_t dlc,
                                                         const motor_measure_t *measure)
{
    actuator_feedback_t prev;
    actuator_feedback_t fb;
    uint32_t prev_rx_count = 0u;

    if ((uint32_t)actuator_id >= (uint32_t)ACTUATOR_ID__COUNT || measure == NULL)
    {
        return;
    }

    (void)memset(&fb, 0, sizeof(fb));
    if (actuator_feedback_get_copy(actuator_id, &prev) != 0u)
    {
        prev_rx_count = prev.rx_count;
    }
    fb.online = 1u;
    fb.bus = bus;
    fb.rx_dlc = dlc;
    fb.transport = (uint8_t)ACTUATOR_TRANSPORT_CAN;
    fb.rx_id = std_id;
    fb.rx_count = prev_rx_count + 1u;
    fb.last_rx_tick = osKernelGetTickCount();
    fb.ecd = measure->ecd;
    fb.speed_rpm = measure->speed_rpm;
    fb.current = measure->given_current;
    fb.temperature = measure->temperate;
    fb.position = ((fp32)measure->ecd) * CAN_RX_TWO_PI / CAN_RX_ECD_RANGE_F;
    fb.velocity = ((fp32)measure->speed_rpm) * CAN_RX_RPM_TO_RADPS;
    fb.torque = (fp32)measure->given_current;
    actuator_feedback_update(actuator_id, &fb);
}

// MIT 反馈没有旧 ecd/rpm/current 格式，这里合成一份给老任务继续用。
static void can_rx_synthesize_measure_from_mit(motor_measure_t *measure,
                                               const can_mit_motor_limits_t *limits,
                                               const can_mit_motor_feedback_t *mit)
{
    if (measure == NULL || limits == NULL || mit == NULL)
    {
        return;
    }

    measure->last_ecd = measure->ecd;
    measure->ecd = can_rx_position_to_ecd(mit->position);
    measure->speed_rpm = can_rx_float_to_i16_saturated(mit->velocity * CAN_RX_RADPS_TO_RPM);
    measure->given_current = can_rx_torque_to_current_like(limits, mit->torque);
    measure->temperate = 0u;
}

// 尝试按 MIT 协议解析某个轴；成功后同时更新旧 measure 和通用执行器反馈。
static uint8_t can_rx_process_mit_node_frame(motor_measure_t *measure,
                                             const motor_node_param_t *node,
                                             uint8_t bus,
                                             uint16_t std_id,
                                             uint8_t dlc,
                                             const uint8_t data[8],
                                             actuator_id_e actuator_id,
                                             uint8_t detect_toe,
                                             uint8_t use_detect)
{
    const can_mit_motor_limits_t *limits;
    can_mit_motor_feedback_t mit;
    actuator_feedback_t prev;
    actuator_feedback_t fb;
    uint32_t prev_rx_count = 0u;

    if (node == NULL)
    {
        return 0u;
    }

    limits = motor_cfg_mit_limits(node);
    if (limits == NULL)
    {
        return 0u;
    }

    (void)memset(&mit, 0, sizeof(mit));
    if (can_mit_motor_update_feedback(std_id, node->can_id, limits, dlc, data, &mit) == 0u)
    {
        return 0u;
    }

    can_rx_synthesize_measure_from_mit(measure, limits, &mit);

    if ((uint32_t)actuator_id < (uint32_t)ACTUATOR_ID__COUNT)
    {
        (void)memset(&fb, 0, sizeof(fb));
        if (actuator_feedback_get_copy(actuator_id, &prev) != 0u)
        {
            prev_rx_count = prev.rx_count;
        }
        fb.online = 1u;
        fb.bus = bus;
        fb.rx_dlc = dlc;
        fb.transport = (uint8_t)ACTUATOR_TRANSPORT_CAN;
        fb.rx_id = std_id;
        fb.rx_count = prev_rx_count + 1u;
        fb.last_rx_tick = mit.last_rx_tick;
        fb.position = mit.position;
        fb.velocity = mit.velocity;
        fb.torque = mit.torque;
        if (measure != NULL)
        {
            fb.ecd = measure->ecd;
            fb.speed_rpm = measure->speed_rpm;
            fb.current = measure->given_current;
            fb.temperature = measure->temperate;
        }
        actuator_feedback_update(actuator_id, &fb);
    }

    if (use_detect != 0u)
    {
        detect_hook(detect_toe);
    }
    return 1u;
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

// 按轴配置决定怎么处理反馈：大疆电流帧直接拆，MIT 帧走 MIT 解析，其他交给扩展口。
static uint8_t can_rx_process_node_frame(motor_measure_t *measure,
                                         const motor_node_param_t *node,
                                         uint8_t bus,
                                         uint16_t std_id,
                                         uint8_t dlc,
                                         const uint8_t data[8],
                                         uint8_t detect_toe,
                                         uint8_t use_detect,
                                         actuator_id_e actuator_id)
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
        can_rx_update_actuator_feedback_from_measure(actuator_id, bus, std_id, dlc, measure);
        return 1u;
    }

    if (can_rx_process_mit_node_frame(measure,
                                      node,
                                      bus,
                                      std_id,
                                      dlc,
                                      data,
                                      actuator_id,
                                      detect_toe,
                                      use_detect) != 0u)
    {
        return 1u;
    }

    if (CAN_rx_process_extra_frame(bus, std_id, dlc, data) != 0u)
    {
        return 1u;
    }

    watch_task_error(WATCH_TASK_CAN_FEEDBACK_RX);
    return 1u;
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

// CAN 接收总入口：先按总线和轴装配找到归属，再交给对应协议解析。
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
                                            1u,
                                            actuator_id_chassis(idx));
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
                                            1u,
                                            ACTUATOR_ID_YAW);
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
                                            0u,
                                            ACTUATOR_ID_YAW_UPPER);
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
                                            1u,
                                            ACTUATOR_ID_TRIGGER);
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
                                            1u,
                                            ACTUATOR_ID_PITCH);
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
                                            0u,
                                            actuator_id_friction(idx));
            return;
        }
    }

    (void)CAN_rx_process_extra_frame(bus, std_id, dlc, data);
}

// 发送大疆一组四电机电流帧；MIT 等非大疆协议不会走这里。
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
