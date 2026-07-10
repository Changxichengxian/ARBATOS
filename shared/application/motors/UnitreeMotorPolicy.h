#pragma once

#include <stdint.h>

#include "LowCmd.h"

#define UNITREE_MOTOR_DEFAULT_RX_TIMEOUT_MS 50u

typedef struct
{
    fp32 torque_nm;
    fp32 speed_rad_s;
    fp32 position_rad;
    fp32 kp;
    fp32 kd;
} UnitreeMotorCmd;

typedef struct
{
    uint32_t last_tick_ms;
    uint32_t last_seq;
    uint8_t last_mode;
    uint8_t valid;
} UnitreeMotorTxSchedule;

static inline uint16_t UnitreeMotorRxTimeoutMs(uint16_t configured_ms)
{
    return (configured_ms != 0u) ? configured_ms : UNITREE_MOTOR_DEFAULT_RX_TIMEOUT_MS;
}

static inline fp32 UnitreeMotorRatioSafe(fp32 reduction_ratio)
{
    return (reduction_ratio > 0.0f) ? reduction_ratio : 1.0f;
}

static inline uint16_t UnitreeMotorWriterOrControl(uint16_t writer)
{
    return (writer != (uint16_t)LOWCMD_WRITER_NONE) ?
        writer : (uint16_t)LOWCMD_WRITER_CONTROL;
}

/*
 * CanTx 缓存之后，故障任务可能已经清命令或提高禁写优先级。
 * 只有缓存与最新快照仍是同一条、且 writer 仍有权限时，才允许物理发送。
 */
static inline uint8_t UnitreeMotorCmdSnapshotAllowed(const MotorCmd *cached,
                                                     const MotorCmd *latest,
                                                     uint16_t inhibit_writer)
{
    uint16_t cached_writer;
    uint16_t latest_writer;

    if (cached == 0 || latest == 0 ||
        cached->active == 0u || latest->active == 0u ||
        cached->mode == (uint8_t)MotorModeNone ||
        cached->mode == (uint8_t)MotorModeDisable ||
        latest->seq != cached->seq ||
        latest->mode != cached->mode)
    {
        return 0u;
    }

    cached_writer = UnitreeMotorWriterOrControl(cached->writer);
    latest_writer = UnitreeMotorWriterOrControl(latest->writer);
    if (latest_writer != cached_writer ||
        (inhibit_writer != (uint16_t)LOWCMD_WRITER_NONE &&
         cached_writer < inhibit_writer))
    {
        return 0u;
    }
    return 1u;
}

/* MotorApplied 延续 LowCmd 的输出侧单位，不能暴露协议内部的转子侧量。 */
static inline void UnitreeMotorMapAppliedOutput(const UnitreeMotorCmd *rotor,
                                               fp32 reduction_ratio,
                                               UnitreeMotorCmd *output)
{
    fp32 ratio;

    if (output == 0)
    {
        return;
    }

    output->torque_nm = 0.0f;
    output->speed_rad_s = 0.0f;
    output->position_rad = 0.0f;
    output->kp = 0.0f;
    output->kd = 0.0f;
    if (rotor == 0)
    {
        return;
    }

    ratio = UnitreeMotorRatioSafe(reduction_ratio);
    output->position_rad = rotor->position_rad / ratio;
    output->speed_rad_s = rotor->speed_rad_s / ratio;
    output->kp = rotor->kp * ratio * ratio;
    output->kd = rotor->kd * ratio * ratio;
    output->torque_nm = rotor->torque_nm * ratio;
}

static inline int16_t UnitreeMotorSafeCurrent(const MotorCmd *cached,
                                              const MotorCmd *latest,
                                              uint16_t inhibit_writer)
{
    if (UnitreeMotorCmdSnapshotAllowed(cached, latest, inhibit_writer) == 0u ||
        cached->mode != (uint8_t)MotorModeCurrent)
    {
        return 0;
    }
    return cached->current;
}

static inline uint8_t UnitreeMotorBrakeRequired(MotorMode mode)
{
    return (mode == MotorModeDisable) ? 1u : 0u;
}

static inline uint8_t UnitreeMotorTxDue(const UnitreeMotorTxSchedule *schedule,
                                       const MotorCmd *cmd,
                                       uint32_t now_ms,
                                       uint16_t period_ms)
{
    const uint8_t mode = (cmd != 0 && cmd->active != 0u &&
                          cmd->mode != (uint8_t)MotorModeNone) ?
        cmd->mode : (uint8_t)MotorModeDisable;
    const uint32_t seq = (cmd != 0) ? cmd->seq : 0u;
    const uint16_t period = (period_ms != 0u) ? period_ms : 1u;

    if (schedule == 0 || schedule->valid == 0u ||
        schedule->last_seq != seq || schedule->last_mode != mode)
    {
        return 1u;
    }
    return ((uint32_t)(now_ms - schedule->last_tick_ms) >= (uint32_t)period) ? 1u : 0u;
}

static inline void UnitreeMotorTxMark(UnitreeMotorTxSchedule *schedule,
                                     const MotorCmd *cmd,
                                     uint32_t now_ms)
{
    if (schedule == 0)
    {
        return;
    }

    schedule->last_tick_ms = now_ms;
    schedule->last_seq = (cmd != 0) ? cmd->seq : 0u;
    schedule->last_mode = (cmd != 0 && cmd->active != 0u &&
                           cmd->mode != (uint8_t)MotorModeNone) ?
        cmd->mode : (uint8_t)MotorModeDisable;
    schedule->valid = 1u;
}

/*
 * LowCmd 统一使用关节输出侧单位，Unitree 帧使用电机转子侧单位。
 * 这里集中完成减速比换算，避免 Arm 等业务任务再维护第二套协议组帧逻辑。
 */
static inline uint8_t UnitreeMotorMapLowCmd(const MotorCmd *src,
                                           fp32 reduction_ratio,
                                           UnitreeMotorCmd *out)
{
    fp32 ratio;

    if (out == 0)
    {
        return 0u;
    }

    out->torque_nm = 0.0f;
    out->speed_rad_s = 0.0f;
    out->position_rad = 0.0f;
    out->kp = 0.0f;
    out->kd = 0.0f;

    if (src == 0 || src->active == 0u ||
        src->mode == (uint8_t)MotorModeNone ||
        src->mode == (uint8_t)MotorModeDisable)
    {
        return 0u;
    }

    ratio = UnitreeMotorRatioSafe(reduction_ratio);
    switch ((MotorMode)src->mode)
    {
    case MotorModeStateTorque:
    case MotorModePosVel:
    case MotorModeForcePos:
        out->position_rad = src->q * ratio;
        out->speed_rad_s = src->dq * ratio;
        out->kp = src->kp / (ratio * ratio);
        out->kd = src->kd / (ratio * ratio);
        out->torque_nm = src->tau / ratio;
        return 1u;
    case MotorModeSpeed:
        out->speed_rad_s = src->dq * ratio;
        out->kd = src->kd / (ratio * ratio);
        out->torque_nm = src->tau / ratio;
        return 1u;
    case MotorModeDamping:
        /* 阻尼模式必须以零目标速度工作，不能继承调用者残留的 dq。 */
        out->kd = src->kd / (ratio * ratio);
        out->torque_nm = src->tau / ratio;
        return 1u;
    case MotorModeCurrent:
        /* 电流到力矩的换算依赖型号量程，由驱动实现继续处理。 */
        return 1u;
    default:
        return 0u;
    }
}
