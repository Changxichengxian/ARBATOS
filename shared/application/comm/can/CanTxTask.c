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
 * - 中段：收集各轴有效命令，并记录 CAN 电流日志。
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
#include "MotorConfig.h"
#include "MotorInst.h"
#include "SdLog.h"
#include "RtProf.h"
#include "RobotTaskProfile.h"
#include "RobotMode.h"
#include "RobotLifecycle.h"
#include "RobotSafety.h"
#include "CanTxCommandPolicy.h"
#include "CanTxCompletionPolicy.h"
#include "UnitreeMotorPolicy.h"

#if defined(STM32H723xx)
#include "BspUsart.h"
#include "N6014bMotorDriver.h"
#include "UnitreeMotorDriver.h"
#endif

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
                                         int16_t current,
                                         const MotorCmd *cmd,
                                         const ControlOutputStamp *owner);

static MotorCmd s_can_tx_cmd_cache[MotorCount];
static ControlOutputStamp s_can_tx_owner_cache[MotorCount];
static MotorId s_can_tx_cmd_cache_ids[MotorCount];
static uint8_t s_can_tx_cmd_cache_valid[MotorCount];
static uint8_t s_can_tx_cmd_expired[MotorCount];
static uint8_t s_can_tx_cmd_unlock_blocked[MotorCount];
static CanTxCmdExpiryLatch s_can_tx_cmd_expiry_latch[MotorCount];
static CanTxCmdUnlockBarrier s_can_tx_cmd_unlock_barrier[MotorCount];
static uint8_t s_can_tx_unlock_barrier_pending;
static uint8_t s_can_tx_unlock_barrier_active;
static uint8_t s_can_tx_output_was_locked = 1u;

static int16_t s_can_tx_can1_200[4];
static int16_t s_can_tx_can1_1ff[4];
static int16_t s_can_tx_can2_200[4];
static int16_t s_can_tx_can2_1ff[4];
static MotorId s_can_tx_rm_motor[2][2][4];
static uint32_t s_can_tx_mit_budget_tick_ms;
static uint8_t s_can_tx_mit_budget_used;
static uint8_t s_can_tx_rm_group_configured[2][2];
static uint8_t s_can_tx_route_start_index;
static uint32_t s_can_tx_mit_enable_tx_count[MotorCount];
static uint32_t s_can_tx_mit_cmd_tx_count[MotorCount];
static uint32_t s_can_tx_authority_reject_count[MotorCount];
static UnitreeMotorTxSchedule s_can_tx_unitree_schedule[MotorCount];

typedef struct
{
    uint8_t bus;
    uint16_t std_id;
} CanTxEmergencyMitRoute;

#if defined(STM32H723xx)
typedef struct
{
    uint8_t port;
    uint8_t protocol;
    uint8_t len;
    uint8_t reserved;
    uint32_t baudrate;
    uint32_t brr;
    uint8_t data[BSP_RS485_TX_IT_MAX_LEN];
} CanTxEmergencyRs485Route;
#endif

static CanTxEmergencyMitRoute s_can_tx_emergency_mit[MotorCount];
static volatile uint8_t s_can_tx_emergency_mit_count;
static uint8_t s_can_tx_emergency_rm_group[3][2];
#if defined(STM32H723xx)
static CanTxEmergencyRs485Route s_can_tx_emergency_rs485[MotorCount];
static volatile uint8_t s_can_tx_emergency_rs485_count;
#endif
static volatile uint32_t s_can_tx_emergency_hash;
static volatile uint32_t s_can_tx_emergency_magic;
static volatile uint32_t s_can_tx_emergency_magic_inv;

static void CanTxEmitRmFrames(sdlog_actuator_current_t *log);
static void CanTxForceDisabledCmd(MotorId id);

#include "CanCommandTxCommonHelpers.inc"

#include "CanCommandTxCompletionHelpers.inc"

#include "CanCommandTxMitHelpers.inc"

#include "CanCommandTxRouteHelpers.inc"

#include "CanCommandTxEmitHelpers.inc"

// 目标工程可在这里接入非大疆、非 MIT 的特殊电机发送逻辑。
__weak uint8_t CanTxProcessExtraItem(uint8_t bus,
                                         MotorId actuator_id,
                                         const motor_node_param_t *node,
                                         int16_t current,
                                         const MotorCmd *cmd,
                                         const ControlOutputStamp *owner)
{
    (void)bus;
    (void)actuator_id;
    (void)node;
    (void)current;
    (void)cmd;
    (void)owner;
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

__weak int CanMitMotorSendCmdTracked(uint8_t bus,
                                     uint16_t std_id,
                                     const CanMitMotorLimits *limits,
                                     const CanMitMotorCmd *cmd,
                                     const BspCanTxTicket *ticket,
                                     uint8_t *tracked)
{
    (void)ticket;
    if (tracked != NULL)
    {
        *tracked = 0u;
    }
    return CanMitMotorSendCmd(bus, std_id, limits, cmd);
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

uint32_t CanTxAuthorityRejectCount(uint8_t actuator_id)
{
    return (actuator_id < (uint8_t)MotorCount) ? s_can_tx_authority_reject_count[actuator_id] : 0u;
}

static uint32_t CanTxEmergencyHashByte(uint32_t hash, uint8_t value)
{
    return (hash ^ (uint32_t)value) * 16777619u;
}

#if defined(STM32H723xx)
static uint32_t CanTxEmergencyHashU32(uint32_t hash, uint32_t value)
{
    hash = CanTxEmergencyHashByte(hash, (uint8_t)value);
    hash = CanTxEmergencyHashByte(hash, (uint8_t)(value >> 8));
    hash = CanTxEmergencyHashByte(hash, (uint8_t)(value >> 16));
    return CanTxEmergencyHashByte(hash, (uint8_t)(value >> 24));
}
#endif

static uint32_t CanTxEmergencyTableHash(uint8_t mit_count, uint8_t rs485_count)
{
    uint32_t hash = CanTxEmergencyHashByte(CAN_TX_EMERGENCY_HASH_SEED, mit_count);

    hash = CanTxEmergencyHashByte(hash, rs485_count);

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
#if defined(STM32H723xx)
    for (uint8_t i = 0u; i < rs485_count; i++)
    {
        const CanTxEmergencyRs485Route *route = &s_can_tx_emergency_rs485[i];

        hash = CanTxEmergencyHashByte(hash, route->port);
        hash = CanTxEmergencyHashByte(hash, route->protocol);
        hash = CanTxEmergencyHashByte(hash, route->len);
        hash = CanTxEmergencyHashByte(hash, route->reserved);
        hash = CanTxEmergencyHashU32(hash, route->baudrate);
        hash = CanTxEmergencyHashU32(hash, route->brr);
        for (uint8_t byte_index = 0u; byte_index < (uint8_t)BSP_RS485_TX_IT_MAX_LEN; byte_index++)
        {
            hash = CanTxEmergencyHashByte(hash, route->data[byte_index]);
        }
    }
#else
    (void)rs485_count;
#endif
    return hash;
}

static uint8_t CanTxEmergencyTableValid(uint8_t *out_mit_count, uint8_t *out_rs485_count)
{
    uint8_t mit_count;
    uint8_t rs485_count = 0u;

    if (s_can_tx_emergency_magic != CAN_TX_EMERGENCY_MAGIC ||
        s_can_tx_emergency_magic_inv != (uint32_t)(~CAN_TX_EMERGENCY_MAGIC))
    {
        return 0u;
    }

    __DMB();
    mit_count = s_can_tx_emergency_mit_count;
#if defined(STM32H723xx)
    rs485_count = s_can_tx_emergency_rs485_count;
#endif
    if (mit_count > (uint8_t)MotorCount)
    {
        return 0u;
    }
    if (rs485_count > (uint8_t)MotorCount)
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

#if defined(STM32H723xx)
    for (uint8_t i = 0u; i < rs485_count; i++)
    {
        const CanTxEmergencyRs485Route *route = &s_can_tx_emergency_rs485[i];
        const uint8_t expected_len =
            (route->protocol == (uint8_t)MOTOR_PROTOCOL_UNITREE_RS485) ?
                (uint8_t)UNITREE_MOTOR_FAULT_FRAME_SIZE :
            (route->protocol == (uint8_t)MOTOR_PROTOCOL_N6014B_RS485) ?
                (uint8_t)N6014B_MOTOR_FAULT_FRAME_SIZE :
                0u;

        if (route->port > 1u || route->reserved != 0u || route->baudrate == 0u ||
            route->brr < 0x10u || route->brr > 0xFFFFu ||
            route->len == 0u || route->len > (uint8_t)BSP_RS485_TX_IT_MAX_LEN ||
            route->len != expected_len)
        {
            return 0u;
        }

        for (uint8_t previous = 0u; previous < i; previous++)
        {
            const CanTxEmergencyRs485Route *other = &s_can_tx_emergency_rs485[previous];

            if (other->port != route->port)
            {
                continue;
            }
            if ((other->baudrate == route->baudrate && other->brr != route->brr) ||
                (other->baudrate != route->baudrate && other->brr == route->brr))
            {
                /* 同端口允许逐帧切速，但波特率与 BRR 的映射必须自洽。 */
                return 0u;
            }
        }
    }
#endif

    if (s_can_tx_emergency_hash != CanTxEmergencyTableHash(mit_count, rs485_count))
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
    if (out_rs485_count != NULL)
    {
        *out_rs485_count = rs485_count;
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
    uint8_t rs485_count = 0u;
    uint8_t count;

    if (CanTxEmergencyTableValid(NULL, NULL) != 0u)
    {
        return;
    }
    count = MotorRouteCount();

    s_can_tx_emergency_magic = 0u;
    s_can_tx_emergency_magic_inv = 0u;
    __DMB();
    s_can_tx_emergency_mit_count = 0u;
#if defined(STM32H723xx)
    s_can_tx_emergency_rs485_count = 0u;
#endif
    s_can_tx_emergency_hash = 0u;
    (void)memset(s_can_tx_emergency_mit, 0, sizeof(s_can_tx_emergency_mit));
    (void)memset(s_can_tx_emergency_rm_group, 0, sizeof(s_can_tx_emergency_rm_group));
#if defined(STM32H723xx)
    (void)memset(s_can_tx_emergency_rs485, 0, sizeof(s_can_tx_emergency_rs485));
#endif

    for (uint8_t i = 0u; i < count; i++)
    {
        const MotorRoute *route = MotorRouteGet(i);

        if (route == NULL || route->enabled == 0u)
        {
            continue;
        }

#if defined(STM32H723xx)
        if (route->transport == (uint8_t)MotorTransportRS485 &&
            route->node != NULL && route->bus <= 1u &&
            rs485_count < (uint8_t)MotorCount)
        {
            CanTxEmergencyRs485Route *dst = &s_can_tx_emergency_rs485[rs485_count];
            uint16_t frame_len = 0u;

            dst->port = route->bus;
            dst->protocol = route->protocol;
            if (route->protocol == (uint8_t)MOTOR_PROTOCOL_UNITREE_RS485)
            {
                frame_len = UnitreeMotorFaultFrameBuild(route->node,
                                                        dst->data,
                                                        (uint16_t)sizeof(dst->data),
                                                        &dst->baudrate);
            }
            else if (route->protocol == (uint8_t)MOTOR_PROTOCOL_N6014B_RS485)
            {
                frame_len = N6014bMotorFaultFrameBuild(route->node,
                                                       dst->data,
                                                       (uint16_t)sizeof(dst->data),
                                                       &dst->baudrate);
            }
            if (frame_len != 0u && frame_len <= (uint16_t)sizeof(dst->data) &&
                BspRs485FaultBaudPrepare(dst->port, dst->baudrate, &dst->brr) != 0u)
            {
                dst->len = (uint8_t)frame_len;
                rs485_count++;
            }
            else
            {
                (void)memset(dst, 0, sizeof(*dst));
            }
            continue;
        }
#endif

        if (route->transport != (uint8_t)MotorTransportCAN ||
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
#if defined(STM32H723xx)
    s_can_tx_emergency_rs485_count = rs485_count;
#endif
    s_can_tx_emergency_hash = CanTxEmergencyTableHash(mit_count, rs485_count);
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
    uint8_t rs485_count = 0u;

    BspCanFaultLock();
#if defined(STM32H723xx)
    BspRs485FaultLock();
#endif

    if (CanTxEmergencyTableValid(&mit_count, &rs485_count) == 0u)
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

#if defined(STM32H723xx)
    {
        uint8_t failed_port_mask = 0u;

        for (uint8_t i = 0u; i < rs485_count; i++)
        {
            const CanTxEmergencyRs485Route *route = &s_can_tx_emergency_rs485[i];
            const uint8_t port_mask = (uint8_t)((uint8_t)1u << route->port);

            if ((failed_port_mask & port_mask) != 0u)
            {
                continue;
            }
            if (BspRs485FaultTx(route->port,
                                route->baudrate,
                                route->brr,
                                route->data,
                                route->len) != 0)
            {
                /* 故障端口不继续重复长轮询；另一端口仍独立尽力发送。 */
                failed_port_mask |= port_mask;
            }
        }
    }
#else
    (void)rs485_count;
#endif
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
        CanTxPhysicalProcessCompletions();
        WatchTaskBeat(WATCH_TASK_CAN_COMMAND_TX);
        const uint16_t period_ms = RobotProfileCanCommandTxPeriodMs();

        /*
         * 先观察上个周期留下的结论，再推进本周期。这样即使故障进入和清除
         * 都夹在两次调度之间，或发送任务停顿到快照过期，也一定先让发送门
         * 看见一次锁定，并为重新 ACTIVE 建立新的命令代次屏障。
         */
        CanTxOutputGateSync(RobotSafetyOutputLocked());
        RobotLifecycleUpdate();
        const uint8_t output_locked = RobotSafetyOutputLocked();

        CanTxOutputGateSync(output_locked);
        CanTxExecInstances(output_locked);
        CanTxPhysicalProcessCompletions();

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
