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
 * - 后段：手动步进各关节，处理 CAN/RS485 反馈，并给 CAN 发送任务补特殊轴。
 * - 入口：ArmMotionStepManual() 每周期执行手动控制。
 */

#include "ArmTask.h"
#include "ManualInput.h"

#include "FreeRTOS.h"
#include "task.h"

#include "LowCmd.h"
#include "MotorInst.h"
#include "MotorConfig.h"
#include "ArmMotorTable.h"
#include "RobotSafety.h"

#include "ArmMotion.h"
#include "ArmCore.h"
#include "CanMitMotorDriver.h"
#include "MitMotor.h"
#include "UnitreeMotorDriver.h"

#include <string.h>

#define ARM_J0_INDEX 0u
#define ARM_J0_CURRENT_DEFAULT 2000

volatile uint8_t g_arm_deadman_hold_ctrl = 0u;
volatile fp32 g_arm_key_speed_scale = 1.0f;
volatile fp32 g_arm_key_kd = 1.0f;
volatile int16_t g_arm_j0_current = ARM_J0_CURRENT_DEFAULT;

static uint8_t g_arm_mit_armed = 0u;
static ArmMotorFeedback g_arm_feedback[ARM_MOTOR_COUNT];
static CanMitMotorFeedback g_arm_mit_feedback[ARM_MOTOR_COUNT];
static ArmJ0UnitreeState g_arm_j0_unitree_state;
static uint32_t g_arm_j0_unitree_last_step_tick_ms = 0u;
static fp32 g_arm_j0_unitree_cmd_output_speed_rad_s = 0.0f;
static fp32 g_arm_j0_unitree_cmd_output_kd = 0.0f;

static const ArmMotorEntry *ArmJ0Entry(void);
static const motor_node_param_t *ArmEntryNode(const ArmMotorEntry *entry);
static uint8_t ArmEntryCanBus(const ArmMotorEntry *entry);
static uint8_t ArmEntryIsUnitreeRs485(const ArmMotorEntry *entry);
static uint8_t ArmJ0UnitreeEnabled(const ArmMotorEntry *entry);
static uint16_t ArmMitStdId(const ArmMotorEntry *entry);
static const CanMitMotorLimits *ArmMitLimits(const ArmMotorEntry *entry);
static void ArmCopyMitFeedback(uint8_t index);
static void ArmClearMitLowCmds(void);
static void ArmSendMitStopAll(void);
static void ArmSendCanMitFromActuator(const ArmMotorEntry *entry,
                                           MotorId actuator_id,
                                           const CanMitMotorLimits *limits);
static void ArmRefreshJ0Feedback(void);
static fp32 ArmJ0UnitreeRatioSafe(const ArmMotorEntry *entry, const ArmJ0UnitreeConfig *cfg);
static fp32 ArmJ0UnitreeOutputToRotorPosition(const ArmMotorEntry *entry,
                                                    const ArmJ0UnitreeConfig *cfg,
                                                    fp32 output_position_rad);
static fp32 ArmJ0UnitreeOutputToRotorSpeed(const ArmMotorEntry *entry,
                                                 const ArmJ0UnitreeConfig *cfg,
                                                 fp32 output_speed_rad_s);
static fp32 ArmJ0UnitreeOutputToRotorTorque(const ArmMotorEntry *entry,
                                                  const ArmJ0UnitreeConfig *cfg,
                                                  fp32 output_torque_nm);
static fp32 ArmJ0UnitreeOutputToRotorKp(const ArmMotorEntry *entry,
                                              const ArmJ0UnitreeConfig *cfg,
                                              fp32 output_kp);
static fp32 ArmJ0UnitreeOutputToRotorKd(const ArmMotorEntry *entry,
                                              const ArmJ0UnitreeConfig *cfg,
                                              fp32 output_kd);
static void ArmBuildJ0UnitreeConfig(UnitreeMotorConfig *out,
                                        const ArmMotorEntry *entry,
                                        const ArmJ0UnitreeConfig *cfg);
static uint8_t ArmBuildJ0MitCmdFromActuator(mit_motor_cmd_t *out,
                                                  fp32 *output_speed_rad_s,
                                                  fp32 *output_kd);
static void ArmJ0UnitreeCmdFromMit(const ArmMotorEntry *entry,
                                        const ArmJ0UnitreeConfig *cfg,
                                        const mit_motor_cmd_t *src,
                                        UnitreeMotorCmd *out);
static void ArmUpdateJ0LowStateFromUnitree(void);
static void ArmSyncJ0UnitreeState(void);
static void ArmStepJ0Unitree(const ArmMotorEntry *entry);
static void ArmBuildCoreConfig(ArmCoreConfig *out);
static void ArmBuildCoreJointParams(ArmCoreJointParam *out, uint8_t out_count);
static void ArmApplyJ0CoreOutput(const ArmCoreOutput *core_output);
static void ArmStepJ0(const ArmCoreOutput *core_output);
static void ArmStepMit(const ArmCoreOutput *core_output);

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
        const CanMitMotorLimits *limits = ArmMitLimits(entry);

        out[i].enabled = 1u;
        out[i].role = (i == ARM_J0_INDEX) ? (uint8_t)ARM_CORE_JOINT_ROLE_J0 :
            ((entry->driver == ARM_MOTOR_DRIVER_CAN_MIT) ? (uint8_t)ARM_CORE_JOINT_ROLE_MIT_SPEED :
             (uint8_t)ARM_CORE_JOINT_ROLE_NONE);
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

static uint16_t ArmMitStdId(const ArmMotorEntry *entry)
{
    const motor_node_param_t *node = ArmEntryNode(entry);

    if (entry == NULL)
    {
        return 0u;
    }

    return MotorCfgCanId(node);
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
        if (g_arm_motor_table[i].driver == ARM_MOTOR_DRIVER_CAN_MIT)
        {
            (void)MotorInstSetSpeedId(ArmActuatorId((uint8_t)i),
                                                  0.0f,
                                                  0.0f,
                                                  0.0f);
        }
    }
}

static void ArmSendMitStopAll(void)
{
    uint32_t i;

    for (i = 0u; i < ARM_MOTOR_COUNT; i++)
    {
        const ArmMotorEntry *entry = &g_arm_motor_table[i];

        if (entry->driver != ARM_MOTOR_DRIVER_CAN_MIT)
        {
            continue;
        }

        CanMitMotorSendStop(ArmEntryCanBus(entry), ArmMitStdId(entry), ArmMitLimits(entry));
    }
}

static void ArmSendCanMitFromActuator(const ArmMotorEntry *entry,
                                           MotorId actuator_id,
                                           const CanMitMotorLimits *limits)
{
    MotorCmd src;
    mit_motor_cmd_t cmd;

    if (entry == NULL || limits == NULL)
    {
        return;
    }

    (void)memset(&cmd, 0, sizeof(cmd));
    if (LowCmdGetMotor(actuator_id, &src) != 0u && src.active != 0u)
    {
        switch ((MotorMode)src.mode)
        {
        case MotorModeStateTorque:
        case MotorModePosVel:
        case MotorModeForcePos:
            cmd.position = src.q;
            cmd.velocity = src.dq;
            cmd.kp = src.kp;
            cmd.kd = src.kd;
            cmd.torque = src.tau;
            break;
        case MotorModeSpeed:
            cmd.velocity = src.dq;
            cmd.kd = src.kd;
            cmd.torque = src.tau;
            break;
        case MotorModeCurrent:
        default:
            cmd.torque = src.tau;
            break;
        }
    }

    CanMitMotorSendCmd(ArmEntryCanBus(entry), ArmMitStdId(entry), limits, &cmd);
}

static fp32 ArmJ0UnitreeRatioSafe(const ArmMotorEntry *entry, const ArmJ0UnitreeConfig *cfg)
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
    (void)cfg;

    return 1.0f;
}

static fp32 ArmJ0UnitreeOutputToRotorPosition(const ArmMotorEntry *entry,
                                                    const ArmJ0UnitreeConfig *cfg,
                                                    fp32 output_position_rad)
{
    return output_position_rad * ArmJ0UnitreeRatioSafe(entry, cfg);
}

static fp32 ArmJ0UnitreeOutputToRotorSpeed(const ArmMotorEntry *entry,
                                                 const ArmJ0UnitreeConfig *cfg,
                                                 fp32 output_speed_rad_s)
{
    return output_speed_rad_s * ArmJ0UnitreeRatioSafe(entry, cfg);
}

static fp32 ArmJ0UnitreeOutputToRotorTorque(const ArmMotorEntry *entry,
                                                  const ArmJ0UnitreeConfig *cfg,
                                                  fp32 output_torque_nm)
{
    return output_torque_nm / ArmJ0UnitreeRatioSafe(entry, cfg);
}

static fp32 ArmJ0UnitreeOutputToRotorKp(const ArmMotorEntry *entry,
                                              const ArmJ0UnitreeConfig *cfg,
                                              fp32 output_kp)
{
    const fp32 ratio = ArmJ0UnitreeRatioSafe(entry, cfg);

    if (ratio <= 0.0f)
    {
        return output_kp;
    }

    return output_kp / (ratio * ratio);
}

static fp32 ArmJ0UnitreeOutputToRotorKd(const ArmMotorEntry *entry,
                                              const ArmJ0UnitreeConfig *cfg,
                                              fp32 output_kd)
{
    const fp32 ratio = ArmJ0UnitreeRatioSafe(entry, cfg);

    if (ratio <= 0.0f)
    {
        return output_kd;
    }

    return output_kd / (ratio * ratio);
}

static uint8_t ArmBuildJ0MitCmdFromActuator(mit_motor_cmd_t *out,
                                                  fp32 *output_speed_rad_s,
                                                  fp32 *output_kd)
{
    MotorCmd src;

    if (out == NULL)
    {
        return 0u;
    }

    (void)memset(out, 0, sizeof(*out));
    if (output_speed_rad_s != NULL)
    {
        *output_speed_rad_s = 0.0f;
    }
    if (output_kd != NULL)
    {
        *output_kd = 0.0f;
    }

    if (LowCmdGetMotor(ArmJ0ActuatorId(), &src) == 0u || src.active == 0u)
    {
        return 0u;
    }

    switch ((MotorMode)src.mode)
    {
    case MotorModeStateTorque:
    case MotorModePosVel:
    case MotorModeForcePos:
        out->position = src.q;
        out->velocity = src.dq;
        out->kp = src.kp;
        out->kd = src.kd;
        out->torque = src.tau;
        break;
    case MotorModeSpeed:
        out->velocity = src.dq;
        out->kd = src.kd;
        out->torque = src.tau;
        break;
    case MotorModeCurrent:
    default:
        out->torque = src.tau;
        break;
    }

    if (output_speed_rad_s != NULL)
    {
        *output_speed_rad_s = src.dq;
    }
    if (output_kd != NULL)
    {
        *output_kd = src.kd;
    }
    return 1u;
}

static void ArmJ0UnitreeCmdFromMit(const ArmMotorEntry *entry,
                                        const ArmJ0UnitreeConfig *cfg,
                                        const mit_motor_cmd_t *src,
                                        UnitreeMotorCmd *out)
{
    if (src == NULL || out == NULL)
    {
        return;
    }

    out->position_rad = ArmJ0UnitreeOutputToRotorPosition(entry, cfg, src->position);
    out->speed_rad_s = ArmJ0UnitreeOutputToRotorSpeed(entry, cfg, src->velocity);
    out->kp = ArmJ0UnitreeOutputToRotorKp(entry, cfg, src->kp);
    out->kd = ArmJ0UnitreeOutputToRotorKd(entry, cfg, src->kd);
    out->torque_nm = ArmJ0UnitreeOutputToRotorTorque(entry, cfg, src->torque);
}

static void ArmBuildJ0UnitreeConfig(UnitreeMotorConfig *out,
                                        const ArmMotorEntry *entry,
                                        const ArmJ0UnitreeConfig *cfg)
{
    const motor_node_param_t *node = ArmEntryNode(entry);

    if (out == NULL)
    {
        return;
    }

    (void)memset(out, 0, sizeof(*out));

    if (cfg == NULL)
    {
        return;
    }

    out->enable = ArmJ0UnitreeEnabled(entry);
    out->rs485_port = (node != NULL) ? node->rs485_port : 0u;
    out->motor_id = MotorCfgNodeId(node);
    out->baudrate = (node != NULL) ? node->baudrate : 0u;
    out->rx_timeout_ms = (node != NULL) ? node->rx_timeout_ms : 0u;
}

static void ArmUpdateJ0LowStateFromUnitree(void)
{
    MotorState fb;

    (void)memset(&fb, 0, sizeof(fb));
    fb.online = g_arm_j0_unitree_state.online;
    fb.bus = g_arm_j0_unitree_state.rs485_port;
    fb.transport = (uint8_t)MotorTransportRS485;
    fb.driveState = (g_arm_j0_unitree_state.online != 0u) ?
                        (uint8_t)MotorDriveStateEnabled :
                        (uint8_t)MotorDriveStateOffline;
    fb.rxId = g_arm_j0_unitree_state.motor_id;
    fb.rxCount = g_arm_j0_unitree_state.rx_frame_count;
    fb.lastRxTick = g_arm_j0_unitree_state.last_rx_tick_ms;
    fb.q = g_arm_j0_unitree_state.joint_position_rad;
    fb.dq = g_arm_j0_unitree_state.joint_speed_rad_s;
    fb.tauEst = g_arm_j0_unitree_state.torque_nm;
    fb.temperature = (uint8_t)g_arm_j0_unitree_state.motor_temp;
    LowStateUpdateMotor(ArmJ0ActuatorId(), &fb);
}

static void ArmSyncJ0UnitreeState(void)
{
    const ArmMotorEntry *entry = ArmJ0Entry();
    const ArmJ0UnitreeConfig *cfg = &g_config.ArmJ0Unitree;
    const UnitreeMotorState *state = UnitreeMotorGetState();
    UnitreeMotorConfig driver_cfg;

    if (state == NULL)
    {
        (void)memset(&g_arm_j0_unitree_state, 0, sizeof(g_arm_j0_unitree_state));
        ArmUpdateJ0LowStateFromUnitree();
        return;
    }

    ArmBuildJ0UnitreeConfig(&driver_cfg, entry, cfg);

    g_arm_j0_unitree_state.enabled = driver_cfg.enable;
    g_arm_j0_unitree_state.rs485_port = driver_cfg.rs485_port;
    g_arm_j0_unitree_state.motor_id = (state->motor_id != 0u) ? state->motor_id : driver_cfg.motor_id;
    g_arm_j0_unitree_state.online = state->online;
    g_arm_j0_unitree_state.last_mode = state->last_mode;
    g_arm_j0_unitree_state.motor_error = state->motor_error;
    g_arm_j0_unitree_state.motor_temp = state->motor_temp;
    g_arm_j0_unitree_state.last_tx_status = state->last_tx_status;
    g_arm_j0_unitree_state.tx_count = state->tx_count;
    g_arm_j0_unitree_state.tx_fail_count = state->tx_fail_count;
    g_arm_j0_unitree_state.rx_frame_count = state->rx_frame_count;
    g_arm_j0_unitree_state.rx_crc_fail_count = state->rx_crc_fail_count;
    g_arm_j0_unitree_state.rx_parse_error_count = state->rx_parse_error_count;
    g_arm_j0_unitree_state.last_rx_tick_ms = state->last_rx_tick_ms;
    g_arm_j0_unitree_state.cmd_output_speed_rad_s = g_arm_j0_unitree_cmd_output_speed_rad_s;
    g_arm_j0_unitree_state.cmd_output_kd = g_arm_j0_unitree_cmd_output_kd;
    g_arm_j0_unitree_state.torque_nm = state->torque_nm;
    g_arm_j0_unitree_state.joint_speed_rad_s = state->joint_speed_rad_s;
    g_arm_j0_unitree_state.joint_position_rad = state->joint_position_rad;
    ArmUpdateJ0LowStateFromUnitree();
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

static void ArmStepJ0Unitree(const ArmMotorEntry *entry)
{
    const ArmJ0UnitreeConfig *cfg = &g_config.ArmJ0Unitree;
    const uint16_t period_ms = (cfg->control_period_ms == 0u) ? 5u : cfg->control_period_ms;
    const uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    UnitreeMotorConfig driver_cfg;
    mit_motor_cmd_t mit_cmd = {0};
    UnitreeMotorCmd cmd = {0};
    fp32 output_speed = 0.0f;
    fp32 output_kd = 0.0f;

    ArmBuildJ0UnitreeConfig(&driver_cfg, entry, cfg);
    UnitreeMotorRefresh(&driver_cfg);

    if (driver_cfg.enable == 0u)
    {
        UnitreeMotorStop();
        g_arm_j0_unitree_cmd_output_speed_rad_s = 0.0f;
        g_arm_j0_unitree_cmd_output_kd = 0.0f;
        g_arm_j0_unitree_last_step_tick_ms = now_ms;
        ArmSyncJ0UnitreeState();
        return;
    }

    if ((g_arm_j0_unitree_last_step_tick_ms != 0u) &&
        ((now_ms - g_arm_j0_unitree_last_step_tick_ms) < period_ms))
    {
        ArmSyncJ0UnitreeState();
        return;
    }

    g_arm_j0_unitree_last_step_tick_ms = now_ms;

    if (UnitreeMotorConfigure(&driver_cfg) == 0u)
    {
        ArmSyncJ0UnitreeState();
        return;
    }

    if (RobotSafetyOutputLocked() != 0u)
    {
        UnitreeMotorCmd zero_cmd = {0};
        g_arm_j0_unitree_cmd_output_speed_rad_s = 0.0f;
        g_arm_j0_unitree_cmd_output_kd = 0.0f;
        (void)UnitreeMotorSendCmd(&driver_cfg, &zero_cmd);
        ArmSyncJ0UnitreeState();
        return;
    }

    if (ArmBuildJ0MitCmdFromActuator(&mit_cmd, &output_speed, &output_kd) == 0u)
    {
        g_arm_j0_unitree_cmd_output_speed_rad_s = 0.0f;
        g_arm_j0_unitree_cmd_output_kd = 0.0f;
        ArmSyncJ0UnitreeState();
        return;
    }

    ArmJ0UnitreeCmdFromMit(entry, cfg, &mit_cmd, &cmd);
    g_arm_j0_unitree_cmd_output_speed_rad_s = output_speed;
    g_arm_j0_unitree_cmd_output_kd = output_kd;
    (void)UnitreeMotorSendCmd(&driver_cfg, &cmd);
    ArmSyncJ0UnitreeState();
}

static void ArmRefreshJ0Feedback(void)
{
    const ArmMotorEntry *entry = ArmJ0Entry();
    ArmMotorFeedback *feedback = &g_arm_feedback[ARM_J0_INDEX];

    if (ArmJ0UnitreeEnabled(entry) != 0u)
    {
        const ArmJ0UnitreeState *state;

        ArmSyncJ0UnitreeState();
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

static void ArmStepJ0(const ArmCoreOutput *core_output)
{
    const ArmMotorEntry *entry = ArmJ0Entry();

    ArmApplyJ0CoreOutput(core_output);
    ArmStepJ0Unitree(entry);
    if (ArmJ0UnitreeEnabled(entry) != 0u)
    {
        (void)MotorInstSetCurrentId(ArmJ0ActuatorId(), 0);
    }
}

static void ArmStepMit(const ArmCoreOutput *core_output)
{
    const uint8_t deadman = (core_output != NULL) ? core_output->mit_deadman_active : 0u;
    const uint8_t dm_active = (core_output != NULL) ? core_output->mit_move_key_active : 0u;
    uint32_t i;

    if (deadman == 0u)
    {
        if (g_arm_mit_armed != 0u)
        {
            ArmSendMitStopAll();
        }

        ArmClearMitLowCmds();
        g_arm_mit_armed = 0u;
        return;
    }

    if (dm_active != 0u && g_arm_mit_armed == 0u)
    {
        for (i = 0u; i < ARM_MOTOR_COUNT; i++)
        {
            const ArmMotorEntry *entry = &g_arm_motor_table[i];

            if (entry->driver != ARM_MOTOR_DRIVER_CAN_MIT)
            {
                continue;
            }
            if (ArmMitLimits(entry) == NULL)
            {
                continue;
            }

            CanMitMotorSendEnable(ArmEntryCanBus(entry), ArmMitStdId(entry));
        }
        g_arm_mit_armed = 1u;
    }
    else if (dm_active == 0u && g_arm_mit_armed != 0u)
    {
        ArmSendMitStopAll();
        g_arm_mit_armed = 0u;
    }

    if (g_arm_mit_armed == 0u)
    {
        return;
    }

    for (i = 0u; i < ARM_MOTOR_COUNT; i++)
    {
        const ArmMotorEntry *entry = &g_arm_motor_table[i];
        const CanMitMotorLimits *limits;
        const MotorId actuator_id = ArmActuatorId((uint8_t)i);

        if (entry->driver != ARM_MOTOR_DRIVER_CAN_MIT)
        {
            continue;
        }

        limits = ArmMitLimits(entry);
        if (limits == NULL)
        {
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
            (void)MotorInstSetSpeedId(actuator_id, 0.0f, 0.0f, 0.0f);
        }

        ArmSendCanMitFromActuator(entry, actuator_id, limits);
    }
}

void ArmMotionInit(void)
{
    ArmPrepareActuatorIds();
    (void)memset(g_arm_feedback, 0, sizeof(g_arm_feedback));
    (void)memset(g_arm_mit_feedback, 0, sizeof(g_arm_mit_feedback));
    (void)memset(&g_arm_j0_unitree_state, 0, sizeof(g_arm_j0_unitree_state));
    g_arm_mit_armed = 0u;
    g_arm_j0_unitree_last_step_tick_ms = 0u;
    g_arm_j0_unitree_cmd_output_speed_rad_s = 0.0f;
    g_arm_j0_unitree_cmd_output_kd = 0.0f;
    (void)MotorInstSetCurrentId(ArmJ0ActuatorId(), 0);
    ArmClearMitLowCmds();
    UnitreeMotorDriverInit();
    ArmSyncJ0UnitreeState();
    ArmRefreshJ0Feedback();
}

void ArmMotionStepManual(uint16_t key_mask)
{
    ArmCoreConfig core_cfg;
    ArmCoreJointParam core_joint[ARM_MOTOR_COUNT];
    ArmCoreInput core_input;
    ArmCoreOutput core_output;

    ArmBuildCoreConfig(&core_cfg);
    ArmBuildCoreJointParams(core_joint, (uint8_t)ARM_MOTOR_COUNT);

    (void)memset(&core_input, 0, sizeof(core_input));
    core_input.key_mask = key_mask;
    core_input.ctrl_held = ((key_mask & KEY_PRESSED_OFFSET_CTRL) != 0u) ? 1u : 0u;
    core_input.reverse = ((key_mask & KEY_PRESSED_OFFSET_SHIFT) != 0u) ? 1u : 0u;
    core_input.j0_unitree_enabled = ArmJ0UnitreeEnabled(ArmJ0Entry());

    ArmCoreStepManual(&core_cfg, core_joint, (uint8_t)ARM_MOTOR_COUNT, &core_input, &core_output);

    ArmStepJ0(&core_output);
    ArmRefreshJ0Feedback();
    ArmStepMit(&core_output);
}

const ArmMotorFeedback *ArmMotionGetFeedback(uint8_t index)
{
    if (index >= ARM_MOTOR_COUNT)
    {
        return NULL;
    }

    return &g_arm_feedback[index];
}

uint8_t ArmMotionProcessCanFeedback(uint8_t bus, uint16_t std_id, uint8_t dlc, const uint8_t data[8])
{
    uint32_t i;

    for (i = 0u; i < ARM_MOTOR_COUNT; i++)
    {
        const ArmMotorEntry *entry = &g_arm_motor_table[i];
        const motor_node_param_t *node = ArmEntryNode(entry);

        if (entry->driver != ARM_MOTOR_DRIVER_CAN_MIT)
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
    ArmSyncJ0UnitreeState();
    return &g_arm_j0_unitree_state;
}
