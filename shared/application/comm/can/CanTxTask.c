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
#include "main.h"

#include "CanReceive.h"
#include "LowCmd.h"
#include "BspCan.h"
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
#define CAN_TX_EMERGENCY_MAGIC 0x45535450u
#define CAN_TX_EMERGENCY_HASH_SEED 2166136261u

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
static uint32_t s_can_tx_mit_enable_tx_count[MotorCount];
static uint32_t s_can_tx_mit_cmd_tx_count[MotorCount];

typedef struct
{
    uint8_t bus;
    uint16_t std_id;
} CanTxEmergencyMitRoute;

static CanTxEmergencyMitRoute s_can_tx_emergency_mit[MotorCount];
static volatile uint8_t s_can_tx_emergency_mit_count;
static uint8_t s_can_tx_emergency_rm_group[3][2];
static volatile uint32_t s_can_tx_emergency_hash;
static volatile uint32_t s_can_tx_emergency_magic;
static volatile uint32_t s_can_tx_emergency_magic_inv;

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

uint32_t CanTxMitEnableTxCount(uint8_t actuator_id)
{
    return (actuator_id < (uint8_t)MotorCount) ? s_can_tx_mit_enable_tx_count[actuator_id] : 0u;
}

uint32_t CanTxMitCmdTxCount(uint8_t actuator_id)
{
    return (actuator_id < (uint8_t)MotorCount) ? s_can_tx_mit_cmd_tx_count[actuator_id] : 0u;
}

static uint32_t CanTxEmergencyHashByte(uint32_t hash, uint8_t value)
{
    return (hash ^ (uint32_t)value) * 16777619u;
}

static uint32_t CanTxEmergencyTableHash(uint8_t mit_count)
{
    uint32_t hash = CanTxEmergencyHashByte(CAN_TX_EMERGENCY_HASH_SEED, mit_count);

    for (uint8_t bus_index = 0u; bus_index < 3u; bus_index++)
    {
        hash = CanTxEmergencyHashByte(hash, s_can_tx_emergency_rm_group[bus_index][0]);
        hash = CanTxEmergencyHashByte(hash, s_can_tx_emergency_rm_group[bus_index][1]);
    }

    for (uint8_t i = 0u; i < mit_count; i++)
    {
        const CanTxEmergencyMitRoute *route = &s_can_tx_emergency_mit[i];
        hash = CanTxEmergencyHashByte(hash, route->bus);
        hash = CanTxEmergencyHashByte(hash, (uint8_t)route->std_id);
        hash = CanTxEmergencyHashByte(hash, (uint8_t)(route->std_id >> 8));
    }
    return hash;
}

static uint8_t CanTxEmergencyTableValid(uint8_t *out_mit_count)
{
    uint8_t mit_count;

    if (s_can_tx_emergency_magic != CAN_TX_EMERGENCY_MAGIC ||
        s_can_tx_emergency_magic_inv != (uint32_t)(~CAN_TX_EMERGENCY_MAGIC))
    {
        return 0u;
    }

    __DMB();
    mit_count = s_can_tx_emergency_mit_count;
    if (mit_count > (uint8_t)MotorCount)
    {
        return 0u;
    }

    for (uint8_t bus_index = 0u; bus_index < 3u; bus_index++)
    {
        if (s_can_tx_emergency_rm_group[bus_index][0] > 1u ||
            s_can_tx_emergency_rm_group[bus_index][1] > 1u)
        {
            return 0u;
        }
    }

    for (uint8_t i = 0u; i < mit_count; i++)
    {
        const CanTxEmergencyMitRoute *route = &s_can_tx_emergency_mit[i];
        if (route->bus < 1u || route->bus > 3u ||
            route->std_id < 1u || route->std_id > 0x7FFu)
        {
            return 0u;
        }
    }

    if (s_can_tx_emergency_hash != CanTxEmergencyTableHash(mit_count))
    {
        return 0u;
    }
    __DMB();
    if (s_can_tx_emergency_magic != CAN_TX_EMERGENCY_MAGIC ||
        s_can_tx_emergency_magic_inv != (uint32_t)(~CAN_TX_EMERGENCY_MAGIC))
    {
        return 0u;
    }

    if (out_mit_count != NULL)
    {
        *out_mit_count = mit_count;
    }
    return 1u;
}

/*
 * 在任务创建前把装配信息压成很小的发送表。
 * 致命异常只读这张表，不会触发 MotorInst 初始化、配置遍历或 RTOS 临界区。
 */
void CanTxEmergencyPrepare(void)
{
    uint8_t mit_count = 0u;
    uint8_t count;

    if (CanTxEmergencyTableValid(NULL) != 0u)
    {
        return;
    }
    count = MotorRouteCount();

    s_can_tx_emergency_magic = 0u;
    s_can_tx_emergency_magic_inv = 0u;
    __DMB();
    s_can_tx_emergency_mit_count = 0u;
    s_can_tx_emergency_hash = 0u;
    (void)memset(s_can_tx_emergency_mit, 0, sizeof(s_can_tx_emergency_mit));
    (void)memset(s_can_tx_emergency_rm_group, 0, sizeof(s_can_tx_emergency_rm_group));

    for (uint8_t i = 0u; i < count; i++)
    {
        const MotorRoute *route = MotorRouteGet(i);

        if (route == NULL || route->enabled == 0u ||
            route->transport != (uint8_t)MotorTransportCAN ||
            route->bus < 1u || route->bus > 3u ||
            route->canId < 1u || route->canId > 0x7FFu)
        {
            continue;
        }

        if (route->isRmGroup != 0u)
        {
            if (route->canId >= 0x201u && route->canId <= 0x204u)
            {
                s_can_tx_emergency_rm_group[route->bus - 1u][0] = 1u;
            }
            else if (route->canId >= 0x205u && route->canId <= 0x208u)
            {
                s_can_tx_emergency_rm_group[route->bus - 1u][1] = 1u;
            }
            continue;
        }

        if (route->mitLimits != NULL &&
            mit_count < (uint8_t)MotorCount)
        {
            CanTxEmergencyMitRoute *dst =
                &s_can_tx_emergency_mit[mit_count++];
            dst->bus = route->bus;
            dst->std_id = route->canId;
        }
    }

    s_can_tx_emergency_mit_count = mit_count;
    s_can_tx_emergency_hash = CanTxEmergencyTableHash(mit_count);
    __DMB();
    s_can_tx_emergency_magic_inv = (uint32_t)(~CAN_TX_EMERGENCY_MAGIC);
    __DMB();
    s_can_tx_emergency_magic = CAN_TX_EMERGENCY_MAGIC;
}

void CanTxEmergencyStopNow(void)
{
    static const uint8_t rm_zero[8] = {0u};
    static const uint8_t mit_disable[8] = {
        0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFDu};
    uint8_t mit_count = 0u;

    BspCanFaultLock();

    if (CanTxEmergencyTableValid(&mit_count) == 0u)
    {
        /* 启动早期尚未发过电机命令，仍尽力给两条经典 CAN 发 RM 零电流。 */
        (void)BspCanFaultTx(1u, (uint16_t)CAN_RM_GROUP_0X200_ID, rm_zero, 8u);
        (void)BspCanFaultTx(1u, (uint16_t)CAN_RM_GROUP_0X1FF_ID, rm_zero, 8u);
        (void)BspCanFaultTx(2u, (uint16_t)CAN_RM_GROUP_0X200_ID, rm_zero, 8u);
        (void)BspCanFaultTx(2u, (uint16_t)CAN_RM_GROUP_0X1FF_ID, rm_zero, 8u);
        return;
    }

    for (uint8_t bus_index = 0u; bus_index < 3u; bus_index++)
    {
        if (s_can_tx_emergency_rm_group[bus_index][0] != 0u)
        {
            (void)BspCanFaultTx((uint8_t)(bus_index + 1u),
                                (uint16_t)CAN_RM_GROUP_0X200_ID,
                                rm_zero,
                                8u);
        }
        if (s_can_tx_emergency_rm_group[bus_index][1] != 0u)
        {
            (void)BspCanFaultTx((uint8_t)(bus_index + 1u),
                                (uint16_t)CAN_RM_GROUP_0X1FF_ID,
                                rm_zero,
                                8u);
        }
    }

    for (uint8_t i = 0u; i < mit_count; i++)
    {
        const CanTxEmergencyMitRoute *route = &s_can_tx_emergency_mit[i];
        if (route->bus < 1u || route->bus > 3u ||
            route->std_id < 1u || route->std_id > 0x7FFu)
        {
            break;
        }
        (void)BspCanFaultTx(route->bus, route->std_id, mit_disable, 8u);
    }
}

// CAN 命令发送任务：收集各轴命令，按轴装配表转换协议并统一发出。
void CanTxTask(void const *pvParameters)
{
    (void)pvParameters;

    TickType_t last_wake = xTaskGetTickCount();
    CanTxEmergencyPrepare();
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
