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
 * - 入口：CanTxTask() 每周期收集 LowCmd，再按电机配置发到 CAN/RS485。
 */

#include "CanTxTask.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include "CanReceive.h"
#include "LowCmd.h"
#include "BspTime.h"
#include "CanMitMotorDriver.h"
#include "RobotConfig.h"
#include "Watch.h"
#include "DetectTask.h"
#include "MotorConfig.h"
#include "MotorInst.h"
#include "SdLog.h"
#include "RtProf.h"
#include "RobotTaskProfile.h"
#include "RobotMode.h"
#include "RobotSafety.h"

#include <string.h>

#define CAN_TX_MIT_WHEEL_CMD_PERIOD_MS 2u
#define CAN_TX_MIT_JOINT_CMD_PERIOD_MS 5u
#define CAN_TX_MIT_MAX_FRAMES_PER_MS 3u
#define CAN_TX_MIT_STATE_DISABLED 0u
#define CAN_TX_MIT_STATE_ENABLED 1u
#define CAN_TX_MIT_STATE_FAULT_MIN 8u

__weak uint8_t CanTxProcessExtraItem(uint8_t bus,
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

#include "CanCommandTxCommonHelpers.inc"

#include "CanCommandTxMitHelpers.inc"

#include "CanCommandTxRouteHelpers.inc"

#include "CanCommandTxEmitHelpers.inc"

// 目标工程可在这里接入非大疆、非 MIT 的特殊电机发送逻辑。
__weak uint8_t CanTxProcessExtraItem(uint8_t bus,
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

__weak int CanMitMotorSendCmd(uint8_t bus,
                                  uint16_t std_id,
                                  const CanMitMotorLimits *limits,
                                  const CanMitMotorCmd *cmd)
{
    (void)bus;
    (void)std_id;
    (void)limits;
    (void)cmd;
    return -1;
}

__weak int CanMitMotorSendEnable(uint8_t bus, uint16_t std_id)
{
    (void)bus;
    (void)std_id;
    return -1;
}

__weak int CanMitMotorSendDisable(uint8_t bus, uint16_t std_id)
{
    (void)bus;
    (void)std_id;
    return -1;
}

__weak int CanMitMotorSendStop(uint8_t bus,
                                   uint16_t std_id,
                                   const CanMitMotorLimits *limits)
{
    (void)bus;
    (void)std_id;
    (void)limits;
    return -1;
}

__weak uint8_t CanMitMotorUpdateFeedback(uint16_t std_id,
                                             uint8_t motor_id,
                                             const CanMitMotorLimits *limits,
                                             uint8_t dlc,
                                             const uint8_t data[8],
                                             CanMitMotorFeedback *feedback)
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
void CanTxTask(void const *pvParameters)
{
    (void)pvParameters;

    TickType_t last_wake = xTaskGetTickCount();
    CanTxCacheRmGroups();

    while (1)
    {
        const uint64_t loop_start_us = RtProfBegin();
        WatchTaskBeat(WATCH_TASK_CAN_COMMAND_TX);
        const uint16_t period_ms = RobotProfileCanCommandTxPeriodMs();
        const bool_t dbus_offline = toe_is_error(DBUS_TOE);
        const uint8_t output_locked = RobotSafetyOutputLocked();

        CanTxExecInstances(dbus_offline ? 0u : 1u, output_locked);
        CanTxEmitRmFrames();

        RtProfEnd(RtProfCanTxLoop, loop_start_us);
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
