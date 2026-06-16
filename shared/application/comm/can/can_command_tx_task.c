/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

/*
 * 阅读地图：
 * - 前段：命令限幅、RM/MIT 命令换算、单轴协议处理。
 * - 中段：在线/离线收集各轴电流命令，并记录 CAN 电流日志。
 * - 后段：按轴装配表执行发送，RM 组帧缓存最后统一发出。
 * - 入口：can_command_tx_task() 每周期收集 LowCmd，再按电机配置发到 CAN/RS485。
 */

#include "can_command_tx_task.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include "CAN_receive.h"
#include "LowCmd.h"
#include "bsp_time.h"
#include "can_mit_motor_driver.h"
#include "config.h"
#include "watch.h"
#include "detect_task.h"
#include "motor_config.h"
#include "motor_instance.h"
#include "sdlog.h"
#include "rt_profiler.h"
#include "robot_task_profile.h"
#include "robot_mode.h"
#include "robot_safety.h"

#include <string.h>

#define CAN_TX_MIT_WHEEL_CMD_PERIOD_MS 2u
#define CAN_TX_MIT_JOINT_CMD_PERIOD_MS 5u
#define CAN_TX_MIT_MAX_FRAMES_PER_MS 3u
#define CAN_TX_MIT_STATE_DISABLED 0u
#define CAN_TX_MIT_STATE_ENABLED 1u
#define CAN_TX_MIT_STATE_FAULT_MIN 8u

__weak uint8_t can_tx_process_extra_item(uint8_t bus,
                                         MotorId actuator_id,
                                         const motor_node_param_t *node,
                                         int16_t current);

static MotorCmd s_can_tx_cmd_cache[MotorCount];
static MotorId s_can_tx_cmd_cache_ids[MotorCount];
static uint8_t s_can_tx_cmd_cache_valid[MotorCount];
static uint8_t s_can_tx_cmd_expired[MotorCount];

static int16_t s_can_tx_can1_200[4];
static int16_t s_can_tx_can1_1ff[4];
static int16_t s_can_tx_can2_200[4];
static int16_t s_can_tx_can2_1ff[4];
static uint32_t s_can_tx_mit_budget_tick_ms;
static uint8_t s_can_tx_mit_budget_used;
static uint8_t s_can_tx_rm_group_configured[2][2];
static uint8_t s_can_tx_route_start_index;

#include "can_command_tx_common_helpers.inc"

#include "can_command_tx_mit_helpers.inc"

#include "can_command_tx_route_helpers.inc"

#include "can_command_tx_emit_helpers.inc"

// 目标工程可在这里接入非大疆、非 MIT 的特殊电机发送逻辑。
__weak uint8_t can_tx_process_extra_item(uint8_t bus,
                                         MotorId actuator_id,
                                         const motor_node_param_t *node,
                                         int16_t current)
{
    (void)bus;
    (void)actuator_id;
    (void)node;
    (void)current;
    return 0u;
}

__weak int can_mit_motor_send_cmd(uint8_t bus,
                                  uint16_t std_id,
                                  const can_mit_motor_limits_t *limits,
                                  const can_mit_motor_cmd_t *cmd)
{
    (void)bus;
    (void)std_id;
    (void)limits;
    (void)cmd;
    return -1;
}

__weak int can_mit_motor_send_enable(uint8_t bus, uint16_t std_id)
{
    (void)bus;
    (void)std_id;
    return -1;
}

__weak int can_mit_motor_send_disable(uint8_t bus, uint16_t std_id)
{
    (void)bus;
    (void)std_id;
    return -1;
}

__weak int can_mit_motor_send_stop(uint8_t bus,
                                   uint16_t std_id,
                                   const can_mit_motor_limits_t *limits)
{
    (void)bus;
    (void)std_id;
    (void)limits;
    return -1;
}

__weak uint8_t can_mit_motor_update_feedback(uint16_t std_id,
                                             uint8_t motor_id,
                                             const can_mit_motor_limits_t *limits,
                                             uint8_t dlc,
                                             const uint8_t data[8],
                                             can_mit_motor_feedback_t *feedback)
{
    (void)std_id;
    (void)motor_id;
    (void)limits;
    (void)dlc;
    (void)data;
    (void)feedback;
    return 0u;
}

// CAN 命令发送任务：收集各轴命令，按轴装配表转换协议并统一发出。
void can_command_tx_task(void const *pvParameters)
{
    (void)pvParameters;

    TickType_t last_wake = xTaskGetTickCount();
    can_tx_cache_rm_groups();

    while (1)
    {
        const uint64_t loop_start_us = rt_profiler_begin();
        watch_task_beat(WATCH_TASK_CAN_COMMAND_TX);
        const uint16_t period_ms = robot_profile_can_command_tx_period_ms();
        const bool_t dbus_offline = toe_is_error(DBUS_TOE);
        const uint8_t output_locked = robot_safety_output_locked();

        can_tx_exec_instances(dbus_offline ? 0u : 1u, output_locked);
        can_tx_emit_rm_frames();

        rt_profiler_end(RT_PROFILER_CAN_COMMAND_TX_LOOP, loop_start_us);
        {
            const TickType_t delay_start = xTaskGetTickCount();
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(period_ms));
            if (xTaskGetTickCount() == delay_start)
            {
                vTaskDelay(1u);
                last_wake = xTaskGetTickCount();
            }
        }
    }
}
