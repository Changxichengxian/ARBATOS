/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

/*
 * 阅读地图：
 * - 前段：按键判断、关节装配表读取、MIT/Unitree 协议辅助函数。
 * - 中段：MIT 反馈同步、停止命令、J0 Unitree 状态同步。
 * - 后段：手动步进各关节并处理 CAN/RS485 反馈；物理发送统一交给 CanTxTask。
 * - 入口：ArmMotionStepManual() 每周期执行手动控制。
 */

#include "ArmTask.h"
#include "ManualInput.h"

#include "FreeRTOS.h"
#include "task.h"

#include "LowCmd.h"
#include "FaultMgr.h"
#include "MotorHealth.h"
#include "MotorInst.h"
#include "MotorConfig.h"
#include "ArmMotorTable.h"
#include "RobotSafety.h"
#include "BspTime.h"

#include "ArmMotion.h"
#include "ArmFaultPolicy.h"
#include "ArmCore.h"
#include "CanMitMotorDriver.h"
#include "UnitreeMotorDriver.h"

#include <string.h>

#define ARM_J0_INDEX 0u
#define ARM_J0_CURRENT_DEFAULT 2000
#define ARM_FAULT_RECOVERY_MS 200u
#define ARM_FAULT_DEFAULT_TIMEOUT_MS 100u
#define ARM_FAULT_REASON_MOTOR (1u << 0)

volatile uint8_t g_arm_deadman_hold_ctrl = 0u;
volatile fp32 g_arm_key_speed_scale = 1.0f;
volatile fp32 g_arm_key_kd = 1.0f;
volatile int16_t g_arm_j0_current = ARM_J0_CURRENT_DEFAULT;

static ArmMotorFeedback g_arm_feedback[ARM_MOTOR_COUNT];
static CanMitMotorFeedback g_arm_mit_feedback[ARM_MOTOR_COUNT];
static ArmJ0UnitreeState g_arm_j0_unitree_state;

typedef struct
{
    FaultMgr mgr;
    uint16_t reason[ARM_MOTOR_COUNT];
    uint32_t configuredMask;
    uint32_t activeMask;
    uint32_t blockingMask;
    uint32_t recoveryMask;
    uint32_t inhibitMask;
    uint32_t holdZeroMask;
    uint32_t inhibitFailCount;
    uint32_t releaseFailCount;
    uint8_t initialized;
} ArmFaultRuntime;

static ArmFaultRuntime s_armFault;

static const ArmMotorEntry *ArmJ0Entry(void);
static const motor_node_param_t *ArmEntryNode(const ArmMotorEntry *entry);
static uint8_t ArmEntryCanBus(const ArmMotorEntry *entry);
static uint8_t ArmEntryIsUnitreeRs485(const ArmMotorEntry *entry);
static uint8_t ArmJ0UnitreeEnabled(const ArmMotorEntry *entry);
static const CanMitMotorLimits *ArmMitLimits(const ArmMotorEntry *entry);
static void ArmCopyMitFeedback(uint8_t index);
static void ArmClearMitLowCmds(void);
static void ArmClearMitLowCmd(uint8_t index);
static void ArmRefreshJ0Feedback(void);
static fp32 ArmJ0UnitreeRatioSafe(const ArmMotorEntry *entry);
static void ArmSyncJ0UnitreeState(void);
static void ArmReadJ0Unitree(const ArmMotorEntry *entry);
static void ArmBuildCoreConfig(ArmCoreConfig *out);
static void ArmBuildCoreJointParams(ArmCoreJointParam *out, uint8_t out_count);
static void ArmApplyJ0CoreOutput(const ArmCoreOutput *core_output);
static void ArmStepJ0(const ArmCoreOutput *core_output, uint8_t isolated);
static void ArmStepMit(const ArmCoreOutput *core_output, uint32_t isolatedMask);
static void ArmFaultInit(void);
static void ArmFaultUpdate(uint16_t keyMask);
static void ArmFaultSyncInhibit(const ArmCoreOutput *coreOutput);
static uint8_t ArmFaultIsolated(uint8_t index);

static void ArmBuildCoreConfig(ArmCoreConfig *out)
{
    if (out == NULL)
    {
        return;
    }

    (void)memset(out, 0, sizeof(*out));
    out->deadman_hold_ctrl = g_arm_deadman_hold_ctrl;
    out->key_speed_scale = g_arm_key_speed_scale;
    out->key_kd = g_arm_key_kd;
    out->j0_current = g_arm_j0_current;
    out->j0_unitree_key_speed_rad_s = g_config.ArmJ0Unitree.key_speed_rad_s;
    out->j0_unitree_hold_kd = g_config.ArmJ0Unitree.hold_kd;
    out->j0_unitree_drive_kd = g_config.ArmJ0Unitree.drive_kd;
}

static void ArmBuildCoreJointParams(ArmCoreJointParam *out, uint8_t out_count)
{
    uint8_t i;

    if (out == NULL)
    {
        return;
    }

    (void)memset(out, 0, (size_t)out_count * sizeof(out[0]));

    for (i = 0u; i < out_count && i < ARM_MOTOR_COUNT; i++)
    {
        const ArmMotorEntry *entry = &g_arm_motor_table[i];
        const motor_node_param_t *node = ArmEntryNode(entry);
        const CanMitMotorLimits *limits = ArmMitLimits(entry);

        out[i].enabled = (MotorCfgNodeId(node) != 0u) ? 1u : 0u;
        out[i].role = (out[i].enabled == 0u) ? (uint8_t)ARM_CORE_JOINT_ROLE_NONE :
            ((i == ARM_J0_INDEX) ? (uint8_t)ARM_CORE_JOINT_ROLE_J0 :
             ((entry->driver == ARM_MOTOR_DRIVER_CAN_MIT) ? (uint8_t)ARM_CORE_JOINT_ROLE_MIT_SPEED :
              (uint8_t)ARM_CORE_JOINT_ROLE_NONE));
        out[i].direction = entry->direction;
        out[i].key_mask = entry->key_mask;
        out[i].key_speed_rad_s = entry->key_speed_rad_s;
        out[i].max_kd = (limits != NULL) ? limits->kd_max : 0.0f;
    }
}

static const ArmMotorEntry *ArmJ0Entry(void)
{
    return &g_arm_motor_table[ARM_J0_INDEX];
}

static const motor_node_param_t *ArmEntryNode(const ArmMotorEntry *entry)
{
    uint32_t i;

    if (entry == NULL)
    {
        return NULL;
    }

    for (i = 0u; i < ARM_MOTOR_COUNT; i++)
    {
        if (entry == &g_arm_motor_table[i])
        {
            return &g_config.motor.arm[i];
        }
    }

    return NULL;
}

static uint8_t ArmEntryCanBus(const ArmMotorEntry *entry)
{
    if (entry == NULL)
    {
        return 0u;
    }
    return MotorCfgCanBus(entry->fallback_bus, ArmEntryNode(entry));
}

static uint8_t ArmEntryIsUnitreeRs485(const ArmMotorEntry *entry)
{
    const motor_node_param_t *node = ArmEntryNode(entry);

    if (entry == NULL)
    {
        return 0u;
    }
    if (MotorCfgTransport(node) != MOTOR_TRANSPORT_RS485)
    {
        return 0u;
    }
    return (MotorCfgProtocol(node) == MOTOR_PROTOCOL_UNITREE_RS485) ? 1u : 0u;
}

static uint8_t ArmJ0UnitreeEnabled(const ArmMotorEntry *entry)
{
    return (ArmEntryIsUnitreeRs485(entry) != 0u) ? 1u : 0u;
}

static const CanMitMotorLimits *ArmMitLimits(const ArmMotorEntry *entry)
{
    const motor_node_param_t *node = ArmEntryNode(entry);

    if (entry == NULL)
    {
        return NULL;
    }

    return MotorCfgMitLimits(node);
}

static const char *ArmMotorInstName(uint8_t index)
{
    switch (index)
    {
    case 0u:
        return "motor.arm0";
    case 1u:
        return "motor.arm1";
    case 2u:
        return "motor.arm2";
    case 3u:
        return "motor.arm3";
    case 4u:
        return "motor.arm4";
    case 5u:
        return "motor.arm5";
    default:
        return NULL;
    }
}

static MotorId s_arm_actuator_ids[ARM_MOTOR_COUNT];
static uint8_t s_arm_actuator_ids_ready = 0u;

static void ArmPrepareActuatorIds(void)
{
    for (uint8_t i = 0u; i < ARM_MOTOR_COUNT; i++)
    {
        const char *name = ArmMotorInstName(i);
        const MotorId resolved = MotorInstIdByName(name);
        s_arm_actuator_ids[i] = (resolved != MotorCount) ? resolved :
            MotorIdRange(Motor12, i, (uint8_t)ARM_MOTOR_COUNT);
    }
    s_arm_actuator_ids_ready = 1u;
}

static MotorId ArmActuatorId(uint8_t index)
{
    if (index >= ARM_MOTOR_COUNT)
    {
        return MotorCount;
    }
    if (s_arm_actuator_ids_ready == 0u)
    {
        return MotorIdRange(Motor12, index, (uint8_t)ARM_MOTOR_COUNT);
    }

    return s_arm_actuator_ids[index];
}

static MotorId ArmJ0ActuatorId(void)
{
    return ArmActuatorId(ARM_J0_INDEX);
}

static uint32_t ArmFaultTimeoutMs(uint8_t index)
{
    const motor_node_param_t *node;

    if (index >= ARM_MOTOR_COUNT)
    {
        return ARM_FAULT_DEFAULT_TIMEOUT_MS;
    }

    node = ArmEntryNode(&g_arm_motor_table[index]);
    if (index == ARM_J0_INDEX && ArmJ0UnitreeEnabled(ArmJ0Entry()) != 0u)
    {
        return UnitreeMotorRxTimeoutMs((node != NULL) ? node->rx_timeout_ms : 0u);
    }
    if (node != NULL && node->rx_timeout_ms >= 10u && node->rx_timeout_ms <= 1000u)
    {
        return node->rx_timeout_ms;
    }
    return ARM_FAULT_DEFAULT_TIMEOUT_MS;
}

static uint8_t ArmFaultReadHealth(uint8_t index, uint32_t nowMs, MotorHealthResult *health)
{
    return MotorHealthRead(ArmActuatorId(index),
                           nowMs,
                           ArmFaultTimeoutMs(index),
                           health);
}

static void ArmFaultInit(void)
{
    FaultMgrConfig config;

    (void)memset(&s_armFault, 0, sizeof(s_armFault));
    (void)memset(&config, 0, sizeof(config));
    config.deviceCount = (uint8_t)ARM_MOTOR_COUNT;

    for (uint8_t i = 0u; i < ARM_MOTOR_COUNT; i++)
    {
        const motor_node_param_t *node = ArmEntryNode(&g_arm_motor_table[i]);

        config.device[i].stableMs = ARM_FAULT_RECOVERY_MS;
        config.device[i].requireSafeInput = 1u;
        if (MotorCfgNodeId(node) != 0u)
        {
            s_armFault.configuredMask |= 1u << i;
        }
    }

    s_armFault.initialized =
        (FaultMgrInit(&s_armFault.mgr, &config) == FaultMgrResultOk) ? 1u : 0u;
    if (s_armFault.initialized == 0u)
    {
        s_armFault.blockingMask = s_armFault.configuredMask;
    }
}

static void ArmFaultUpdate(uint16_t keyMask)
{
    const uint32_t nowMs = BspTimeGetTickMs();
    uint32_t deviceSafeMask = 0u;

    s_armFault.activeMask = 0u;
    s_armFault.blockingMask = 0u;
    s_armFault.recoveryMask = 0u;
    if (s_armFault.initialized == 0u)
    {
        s_armFault.blockingMask = s_armFault.configuredMask;
        return;
    }

    for (uint8_t i = 0u; i < ARM_MOTOR_COUNT; i++)
    {
        const ArmMotorEntry *entry = &g_arm_motor_table[i];
        MotorHealthResult health;

        s_armFault.reason[i] = 0u;
        if ((keyMask & entry->key_mask) == 0u)
        {
            deviceSafeMask |= 1u << i;
        }
        if ((s_armFault.configuredMask & (1u << i)) == 0u)
        {
            continue;
        }

        (void)ArmFaultReadHealth(i, nowMs, &health);
        s_armFault.reason[i] = health.reasonMask;
        if (health.healthy == 0u)
        {
            s_armFault.activeMask |= 1u << i;
        }
        (void)FaultMgrSetDeviceFault(&s_armFault.mgr,
                                     i,
                                     ARM_FAULT_REASON_MOTOR,
                                     (health.healthy == 0u) ? 1u : 0u,
                                     nowMs);
    }

    (void)FaultMgrUpdate(&s_armFault.mgr, nowMs, deviceSafeMask, 0u, 1u);
    for (uint8_t i = 0u; i < ARM_MOTOR_COUNT; i++)
    {
        FaultDeviceStatus status;

        if ((s_armFault.configuredMask & (1u << i)) == 0u)
        {
            continue;
        }
        if (FaultMgrGetDeviceStatus(&s_armFault.mgr, i, &status) != FaultMgrResultOk)
        {
            s_armFault.blockingMask |= 1u << i;
            continue;
        }
        if (status.action != FaultActionRun)
        {
            s_armFault.blockingMask |= 1u << i;
        }
        if (status.recoveryPending != 0u)
        {
            s_armFault.recoveryMask |= 1u << i;
        }
    }
}

static uint8_t ArmFaultIsolated(uint8_t index)
{
    if (index >= ARM_MOTOR_COUNT)
    {
        return 1u;
    }
    if ((s_armFault.holdZeroMask & (1u << index)) != 0u)
    {
        return 1u;
    }
    if (s_armFault.initialized == 0u ||
        (s_armFault.configuredMask & (1u << index)) == 0u)
    {
        return 1u;
    }
    return (FaultMgrDeviceAction(&s_armFault.mgr, index) == FaultActionRun) ? 0u : 1u;
}

static uint32_t ArmMitEligibleMask(void)
{
    uint32_t mask = 0u;

    for (uint8_t i = 0u; i < ARM_MOTOR_COUNT; i++)
    {
        const ArmMotorEntry *entry = &g_arm_motor_table[i];

        if (entry->driver == ARM_MOTOR_DRIVER_CAN_MIT &&
            MotorCfgNodeId(ArmEntryNode(entry)) != 0u &&
            ArmMitLimits(entry) != NULL)
        {
            mask |= 1u << i;
        }
    }
    return mask;
}

static uint8_t ArmFaultCollectIds(uint32_t mask,
                                  MotorId out[ARM_MOTOR_COUNT],
                                  uint32_t *collectedMask)
{
    uint8_t count = 0u;
    uint32_t actualMask = 0u;

    for (uint8_t i = 0u; i < ARM_MOTOR_COUNT; i++)
    {
        const uint32_t bit = 1u << i;
        const MotorId id = ArmActuatorId(i);

        if ((mask & bit) == 0u ||
            (s_armFault.configuredMask & bit) == 0u ||
            (uint32_t)id >= (uint32_t)MotorCount)
        {
            continue;
        }
        out[count++] = id;
        actualMask |= bit;
    }
    if (collectedMask != NULL)
    {
        *collectedMask = actualMask;
    }
    return count;
}

static void ArmFaultWriteJ0Safety(void)
{
    const MotorId id = ArmJ0ActuatorId();

    if ((s_armFault.configuredMask & (1u << ARM_J0_INDEX)) == 0u ||
        (uint32_t)id >= (uint32_t)MotorCount)
    {
        return;
    }

    if (ArmJ0UnitreeEnabled(ArmJ0Entry()) != 0u)
    {
        MotorCmd cmd;

        (void)memset(&cmd, 0, sizeof(cmd));
        cmd.active = 1u;
        cmd.mode = (uint8_t)MotorModeDamping;
        cmd.kd = (g_config.ArmJ0Unitree.hold_kd > 0.0f) ?
                     g_config.ArmJ0Unitree.hold_kd : 0.5f;
        (void)LowCmdSetMotorFrom(id, &cmd, (uint16_t)LOWCMD_WRITER_SAFETY);
    }
    else
    {
        const int16_t zero = 0;
        (void)LowCmdSetCurrentManyFrom(&id, &zero, 1u, (uint16_t)LOWCMD_WRITER_SAFETY);
    }
}

static void ArmFaultSyncInhibit(const ArmCoreOutput *coreOutput)
{
    const uint32_t mitEligibleMask = ArmMitEligibleMask();
    const uint8_t manualStopMit =
        (coreOutput == NULL ||
         coreOutput->mit_deadman_active == 0u ||
         coreOutput->mit_move_key_active == 0u) ? 1u : 0u;
    const ArmFaultInhibitPlan plan =
        ArmFaultInhibitPlanMake(s_armFault.configuredMask,
                                s_armFault.blockingMask,
                                mitEligibleMask,
                                manualStopMit,
                                s_armFault.inhibitMask);
    MotorId ids[ARM_MOTOR_COUNT];
    uint32_t actualMask = 0u;
    uint8_t count;

    s_armFault.holdZeroMask = plan.holdZeroMask;

    count = ArmFaultCollectIds(plan.acquireMask, ids, &actualMask);
    if (count != 0u)
    {
        if (LowCmdInhibitManyFrom(ids, count, (uint16_t)LOWCMD_WRITER_SAFETY) != 0u)
        {
            s_armFault.inhibitMask |= actualMask;
        }
        else
        {
            uint32_t acquiredMask = 0u;

            /* 某一轴已被更高优先级接管时，仍要让其余轴逐个进入安全状态。 */
            for (uint8_t i = 0u; i < ARM_MOTOR_COUNT; i++)
            {
                const uint32_t bit = 1u << i;
                const MotorId id = ArmActuatorId(i);

                if ((actualMask & bit) != 0u &&
                    LowCmdInhibitManyFrom(&id, 1u, (uint16_t)LOWCMD_WRITER_SAFETY) != 0u)
                {
                    acquiredMask |= bit;
                }
            }
            s_armFault.inhibitMask |= acquiredMask;
            if (acquiredMask != actualMask)
            {
                s_armFault.inhibitFailCount++;
            }
        }
    }

    if ((plan.desiredMask & (1u << ARM_J0_INDEX)) != 0u)
    {
        /* J0 保持同级 SAFETY 阻尼/零指令，普通控制写会被局部禁写拒绝。 */
        ArmFaultWriteJ0Safety();
    }

    count = ArmFaultCollectIds(plan.releaseMask, ids, &actualMask);
    if (count == 0u)
    {
        return;
    }

    if (LowCmdClearManyFrom(ids, count, (uint16_t)LOWCMD_WRITER_SAFETY) != 0u &&
        LowCmdReleaseInhibitManyFrom(ids, count, (uint16_t)LOWCMD_WRITER_SAFETY) != 0u)
    {
        s_armFault.inhibitMask &= ~actualMask;
        return;
    }

    {
        uint32_t releasedMask = 0u;

        /* 释放也逐轴兜底，避免一个仍由更高 owner 保持的轴拖住其他已恢复轴。 */
        for (uint8_t i = 0u; i < ARM_MOTOR_COUNT; i++)
        {
            const uint32_t bit = 1u << i;
            const MotorId id = ArmActuatorId(i);

            if ((actualMask & bit) != 0u &&
                LowCmdClearManyFrom(&id, 1u, (uint16_t)LOWCMD_WRITER_SAFETY) != 0u &&
                LowCmdReleaseInhibitManyFrom(&id, 1u, (uint16_t)LOWCMD_WRITER_SAFETY) != 0u)
            {
                releasedMask |= bit;
            }
        }
        s_armFault.inhibitMask &= ~releasedMask;
        if (releasedMask != actualMask)
        {
            s_armFault.releaseFailCount++;
        }
    }
}

static uint8_t ArmMitDriveState(uint8_t online, uint8_t state)
{
    if (online == 0u)
    {
        return (uint8_t)MotorDriveStateOffline;
    }
    if (state == 1u)
    {
        return (uint8_t)MotorDriveStateEnabled;
    }
    if (state >= 8u)
    {
        return (uint8_t)MotorDriveStateFault;
    }
    return (uint8_t)MotorDriveStateDisabled;
}

static void ArmCopyMitFeedback(uint8_t index)
{
    const CanMitMotorFeedback *src;
    ArmMotorFeedback *dst;
    MotorState fb;
    MotorId actuator_id;
    const ArmMotorEntry *entry;

    if (index >= ARM_MOTOR_COUNT)
    {
        return;
    }

    entry = &g_arm_motor_table[index];
    src = &g_arm_mit_feedback[index];
    dst = &g_arm_feedback[index];

    dst->online = src->online;
    dst->rx_dlc = src->rx_dlc;
    dst->rx_id = src->rx_id;
    dst->rx_count = src->rx_count;
    dst->last_rx_tick = src->last_rx_tick;
    dst->position = src->position;
    dst->velocity = src->velocity;
    dst->torque = src->torque;

    actuator_id = ArmActuatorId(index);
    if ((uint32_t)actuator_id < (uint32_t)MotorCount)
    {
        (void)memset(&fb, 0, sizeof(fb));
        fb.online = src->online;
        fb.bus = ArmEntryCanBus(entry);
        fb.rxDlc = src->rx_dlc;
        fb.transport = (uint8_t)MotorTransportCAN;
        fb.state = src->state;
        fb.driveState = ArmMitDriveState(src->online, src->state);
        fb.rxId = src->rx_id;
        fb.rxCount = src->rx_count;
        fb.lastRxTick = src->last_rx_tick;
        fb.q = src->position;
        fb.dq = src->velocity;
        fb.tauEst = src->torque;
        LowStateUpdateMotor(actuator_id, &fb);
    }
}

static void ArmClearMitLowCmds(void)
{
    uint32_t i;

    for (i = 0u; i < ARM_MOTOR_COUNT; i++)
    {
        ArmClearMitLowCmd((uint8_t)i);
    }
}

static void ArmClearMitLowCmd(uint8_t index)
{
    if (index >= ARM_MOTOR_COUNT ||
        g_arm_motor_table[index].driver != ARM_MOTOR_DRIVER_CAN_MIT)
    {
        return;
    }

    /* inactive 才会让通用 CanTx 进入统一 Disable 路径；active 零速仍可能自动 Enable。 */
    (void)MotorInstClearId(ArmActuatorId(index));
}

static fp32 ArmJ0UnitreeRatioSafe(const ArmMotorEntry *entry)
{
    if (entry != NULL)
    {
        const motor_node_param_t *node = ArmEntryNode(entry);
        const MotorModelParam *model = (node != NULL) ? MotorCfgModel(node->model) : NULL;

        if (model != NULL && model->reduction_ratio > 0.0f)
        {
            return model->reduction_ratio;
        }
    }
    return 1.0f;
}

static void ArmSyncJ0UnitreeState(void)
{
    const ArmMotorEntry *entry = ArmJ0Entry();
    UnitreeMotorState state;
    const fp32 ratio = ArmJ0UnitreeRatioSafe(entry);

    if (UnitreeMotorGetStateCopy(&state) == 0u)
    {
        (void)memset(&g_arm_j0_unitree_state, 0, sizeof(g_arm_j0_unitree_state));
        return;
    }

    g_arm_j0_unitree_state.enabled = state.enabled;
    g_arm_j0_unitree_state.rs485_port = state.rs485_port;
    g_arm_j0_unitree_state.motor_id = (state.motor_id != 0u) ?
        state.motor_id : MotorCfgNodeId(ArmEntryNode(entry));
    g_arm_j0_unitree_state.online = state.online;
    g_arm_j0_unitree_state.last_mode = state.last_mode;
    g_arm_j0_unitree_state.motor_error = state.motor_error;
    g_arm_j0_unitree_state.motor_temp = state.motor_temp;
    g_arm_j0_unitree_state.last_tx_status = state.last_tx_status;
    g_arm_j0_unitree_state.tx_count = state.tx_count;
    g_arm_j0_unitree_state.tx_fail_count = state.tx_fail_count;
    g_arm_j0_unitree_state.rx_frame_count = state.rx_frame_count;
    g_arm_j0_unitree_state.rx_crc_fail_count = state.rx_crc_fail_count;
    g_arm_j0_unitree_state.rx_parse_error_count = state.rx_parse_error_count;
    g_arm_j0_unitree_state.last_rx_tick_ms = state.last_rx_tick_ms;
    g_arm_j0_unitree_state.cmd_output_speed_rad_s = state.cmd_speed_rad_s / ratio;
    g_arm_j0_unitree_state.cmd_output_kd = state.cmd_kd * ratio * ratio;
    g_arm_j0_unitree_state.torque_nm = state.torque_nm;
    g_arm_j0_unitree_state.joint_speed_rad_s = state.joint_speed_rad_s;
    g_arm_j0_unitree_state.joint_position_rad = state.joint_position_rad;
}

static void ArmApplyJ0CoreOutput(const ArmCoreOutput *core_output)
{
    const ArmMotorEntry *entry = ArmJ0Entry();
    const MotorCmd *cmd = NULL;

    if (core_output != NULL && core_output->joint_count > ARM_J0_INDEX)
    {
        cmd = &core_output->cmd[ARM_J0_INDEX];
    }

    if (cmd == NULL || cmd->active == 0u)
    {
        (void)MotorInstSetSpeedId(ArmJ0ActuatorId(), 0.0f, 0.0f, 0.0f);
        return;
    }

    if (ArmJ0UnitreeEnabled(entry) != 0u)
    {
        if (cmd->mode == (uint8_t)MotorModeSpeed)
        {
            (void)MotorInstSetSpeedId(ArmJ0ActuatorId(), cmd->dq, cmd->kd, cmd->tau);
        }
        else
        {
            (void)MotorInstSetSpeedId(ArmJ0ActuatorId(), 0.0f, 0.0f, 0.0f);
        }
    }
    else if (cmd->mode == (uint8_t)MotorModeCurrent)
    {
        const int16_t current = MotorCfgLimitCurrentNode(ArmEntryNode(entry), cmd->current);
        (void)MotorInstSetCurrentId(ArmJ0ActuatorId(), current);
    }
    else
    {
        (void)MotorInstSetCurrentId(ArmJ0ActuatorId(), 0);
    }
}

static void ArmReadJ0Unitree(const ArmMotorEntry *entry)
{
    if (ArmJ0UnitreeEnabled(entry) == 0u)
    {
        (void)memset(&g_arm_j0_unitree_state, 0, sizeof(g_arm_j0_unitree_state));
        return;
    }

    ArmSyncJ0UnitreeState();
}

static void ArmRefreshJ0Feedback(void)
{
    const ArmMotorEntry *entry = ArmJ0Entry();
    ArmMotorFeedback *feedback = &g_arm_feedback[ARM_J0_INDEX];

    if (ArmJ0UnitreeEnabled(entry) != 0u)
    {
        const ArmJ0UnitreeState *state;

        ArmReadJ0Unitree(entry);
        state = &g_arm_j0_unitree_state;

        feedback->online = state->online;
        feedback->rx_dlc = 0u;
        feedback->rx_id = state->motor_id;
        feedback->rx_count = state->rx_frame_count;
        feedback->last_rx_tick = state->last_rx_tick_ms;
        feedback->position = state->joint_position_rad;
        feedback->velocity = state->joint_speed_rad_s;
        feedback->torque = state->torque_nm;
        return;
    }

    {
        const motor_measure_t *measure = MotorInstMeasureConst(ArmJ0ActuatorId());

        if (measure == NULL)
        {
            (void)memset(feedback, 0, sizeof(*feedback));
            return;
        }

        feedback->online = ((measure->ecd != 0u) ||
                            (measure->speed_rpm != 0) ||
                            (measure->given_current != 0) ||
                            (measure->temperate != 0u)) ? 1u : 0u;
        feedback->rx_dlc = 8u;
        feedback->rx_id = MotorCfgFeedbackId(ArmEntryNode(entry));
        feedback->rx_count = 0u;
        feedback->last_rx_tick = 0u;
        feedback->position = (fp32)measure->ecd;
        feedback->velocity = (fp32)measure->speed_rpm;
        feedback->torque = (fp32)measure->given_current;
    }
}

static void ArmStepJ0(const ArmCoreOutput *core_output, uint8_t isolated)
{
    const ArmMotorEntry *entry = ArmJ0Entry();

    if (MotorCfgNodeId(ArmEntryNode(entry)) == 0u)
    {
        (void)MotorInstClearId(ArmJ0ActuatorId());
        return;
    }

    if (isolated != 0u)
    {
        if (ArmJ0UnitreeEnabled(entry) != 0u)
        {
            const fp32 damping = (g_config.ArmJ0Unitree.hold_kd > 0.0f) ?
                g_config.ArmJ0Unitree.hold_kd : 0.5f;
            (void)MotorInstSetDampingId(ArmJ0ActuatorId(), damping, 0.0f);
        }
        else
        {
            (void)MotorInstSetCurrentId(ArmJ0ActuatorId(), 0);
        }
    }
    else
    {
        ArmApplyJ0CoreOutput(core_output);
    }
    ArmReadJ0Unitree(entry);
}

static void ArmStepMit(const ArmCoreOutput *core_output, uint32_t isolatedMask)
{
    uint32_t i;

    /* Arm 只发布 LowCmd；MIT Disable/Enable/控制帧全部由通用 CanTx 单一发送。 */
    for (i = 0u; i < ARM_MOTOR_COUNT; i++)
    {
        const ArmMotorEntry *entry = &g_arm_motor_table[i];
        const MotorId actuator_id = ArmActuatorId((uint8_t)i);

        if (entry->driver != ARM_MOTOR_DRIVER_CAN_MIT ||
            MotorCfgNodeId(ArmEntryNode(entry)) == 0u ||
            ArmMitLimits(entry) == NULL)
        {
            continue;
        }

        if ((isolatedMask & (1u << i)) != 0u)
        {
            ArmClearMitLowCmd((uint8_t)i);
            continue;
        }

        if (core_output != NULL &&
            i < core_output->joint_count &&
            core_output->cmd[i].active != 0u &&
            core_output->cmd[i].mode == (uint8_t)MotorModeSpeed)
        {
            (void)MotorInstSetSpeedId(actuator_id,
                                                  core_output->cmd[i].dq,
                                                  core_output->cmd[i].kd,
                                                  core_output->cmd[i].tau);
        }
        else
        {
            ArmClearMitLowCmd((uint8_t)i);
        }
    }
}

void ArmMotionInit(void)
{
    ArmPrepareActuatorIds();
    (void)memset(g_arm_feedback, 0, sizeof(g_arm_feedback));
    (void)memset(g_arm_mit_feedback, 0, sizeof(g_arm_mit_feedback));
    (void)memset(&g_arm_j0_unitree_state, 0, sizeof(g_arm_j0_unitree_state));
    (void)MotorInstSetCurrentId(ArmJ0ActuatorId(), 0);
    ArmClearMitLowCmds();
    ArmReadJ0Unitree(ArmJ0Entry());
    ArmRefreshJ0Feedback();
    ArmFaultInit();
}

void ArmMotionStepManual(uint16_t key_mask)
{
    ArmCoreConfig core_cfg;
    ArmCoreJointParam core_joint[ARM_MOTOR_COUNT];
    ArmCoreInput core_input;
    ArmCoreOutput core_output;

    ArmRefreshJ0Feedback();
    ArmFaultUpdate(key_mask);
    ArmBuildCoreConfig(&core_cfg);
    ArmBuildCoreJointParams(core_joint, (uint8_t)ARM_MOTOR_COUNT);

    (void)memset(&core_input, 0, sizeof(core_input));
    core_input.key_mask = key_mask;
    core_input.ctrl_held = ((key_mask & KEY_PRESSED_OFFSET_CTRL) != 0u) ? 1u : 0u;
    core_input.reverse = ((key_mask & KEY_PRESSED_OFFSET_SHIFT) != 0u) ? 1u : 0u;
    core_input.j0_unitree_enabled = ArmJ0UnitreeEnabled(ArmJ0Entry());

    ArmCoreStepManual(&core_cfg, core_joint, (uint8_t)ARM_MOTOR_COUNT, &core_input, &core_output);

    ArmFaultSyncInhibit(&core_output);
    ArmStepJ0(&core_output, ArmFaultIsolated(ARM_J0_INDEX));
    ArmRefreshJ0Feedback();
    ArmStepMit(&core_output, s_armFault.holdZeroMask);
}

const ArmMotorFeedback *ArmMotionGetFeedback(uint8_t index)
{
    if (index >= ARM_MOTOR_COUNT)
    {
        return NULL;
    }

    return &g_arm_feedback[index];
}

uint8_t ArmMotionGetFaultStatus(ArmMotionFaultStatus *out)
{
    if (out == NULL)
    {
        return 0u;
    }

    (void)memset(out, 0, sizeof(*out));
    out->configuredMask = s_armFault.configuredMask;
    out->activeMask = s_armFault.activeMask;
    out->blockingMask = s_armFault.blockingMask;
    out->recoveryMask = s_armFault.recoveryMask;
    out->inhibitMask = s_armFault.inhibitMask;
    out->holdZeroMask = s_armFault.holdZeroMask;
    out->inhibitFailCount = s_armFault.inhibitFailCount;
    out->releaseFailCount = s_armFault.releaseFailCount;
    out->initialized = s_armFault.initialized;
    for (uint8_t i = 0u; i < ARM_MOTOR_COUNT && i < ARM_JOINT_COUNT; i++)
    {
        out->reason[i] = s_armFault.reason[i];
    }
    return 1u;
}

uint8_t ArmMotionProcessCanFeedback(uint8_t bus, uint16_t std_id, uint8_t dlc, const uint8_t data[8])
{
    uint32_t i;

    for (i = 0u; i < ARM_MOTOR_COUNT; i++)
    {
        const ArmMotorEntry *entry = &g_arm_motor_table[i];
        const motor_node_param_t *node = ArmEntryNode(entry);

        if (entry->driver != ARM_MOTOR_DRIVER_CAN_MIT ||
            MotorCfgNodeId(node) == 0u || ArmMitLimits(entry) == NULL)
        {
            continue;
        }
        if (ArmEntryCanBus(entry) != bus)
        {
            continue;
        }
        if (MotorCfgFeedbackId(node) != std_id)
        {
            continue;
        }

        if (CanMitMotorUpdateFeedback(std_id,
                                          MotorCfgNodeId(node),
                                          ArmMitLimits(entry),
                                          dlc,
                                          data,
                                          &g_arm_mit_feedback[i]) != 0u)
        {
            ArmCopyMitFeedback((uint8_t)i);
            return 1u;
        }
        return 0u;
    }

    return 0u;
}

const ArmJ0UnitreeState *ArmMotionGetJ0UnitreeState(void)
{
    ArmReadJ0Unitree(ArmJ0Entry());
    return &g_arm_j0_unitree_state;
}
