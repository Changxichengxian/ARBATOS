/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "CanReceive.h"

#include "cmsis_os.h"

#include "BspCan.h"
#include "BspTime.h"
#include "LowCmd.h"
#include "CanMitMotorDriver.h"
#include "DetectTask.h"
#include "MotorConfig.h"
#include "MotorFeedbackEcdPolicy.h"
#include "MotorInst.h"
#include "SdLog.h"
#include "Watch.h"

#include <string.h>

#define CAN_RX_TWO_PI 6.28318530718f
#define CAN_RX_RADPS_TO_RPM 9.54929659f
#define CAN_RX_RPM_TO_RADPS 0.104719755f
#define CAN_RX_MIT_STATE_DISABLED 0u
#define CAN_RX_MIT_STATE_ENABLED 1u
#define CAN_RX_MIT_STATE_FAULT_MIN 8u

static volatile uint8_t last_can1ff_status = 0u;

typedef struct
{
    const char *name;
    MotorId fallback_id;
    MotorId resolved_id;
    uint8_t ready;
} CanRxMotorIdCache;

static CanRxMotorIdCache CanRxYawId = {"motor.yaw", Motor4, MotorCount, 0u};
static CanRxMotorIdCache CanRxYawUpperId = {"motor.yaw_upper", Motor5, MotorCount, 0u};
static CanRxMotorIdCache CanRxPitchId = {"motor.pitch", Motor6, MotorCount, 0u};
static CanRxMotorIdCache CanRxTriggerId = {"motor.trigger", Motor7, MotorCount, 0u};

static const char *const CanRxChassisMotorNames[4u] = {
    "motor.chassis0",
    "motor.chassis1",
    "motor.chassis2",
    "motor.chassis3",
};
static MotorId CanRxChassisIds[4u] = {MotorCount, MotorCount, MotorCount, MotorCount};
static uint8_t CanRxChassisIdsReady = 0u;

static const char *const CanRxFrictionMotorNames[4u] = {
    "motor.friction0",
    "motor.friction1",
    "motor.friction2",
    "motor.friction3",
};
static MotorId CanRxFrictionIds[4u] = {MotorCount, MotorCount, MotorCount, MotorCount};
static uint8_t CanRxFrictionIdsReady = 0u;

__weak uint8_t CAN_rx_process_extra_frame(uint8_t bus, uint16_t std_id, uint8_t dlc, const uint8_t data[8]);

static void CanRxPrepareMotorIdCache(CanRxMotorIdCache *cache)
{
    MotorId resolved;

    if (cache == NULL)
    {
        return;
    }
    if (cache->ready != 0u)
    {
        return;
    }

    resolved = MotorInstIdByName(cache->name);
    cache->resolved_id = (resolved != MotorCount) ? resolved : cache->fallback_id;
    cache->ready = 1u;
}

static void CanRxPrepareIndexedMotorIds(const char *const *names,
                                             MotorId *ids,
                                             uint8_t *ready,
                                             uint8_t count,
                                             MotorId fallback_first)
{
    uint8_t resolved_count;

    if (names == NULL || ids == NULL || ready == NULL || count == 0u)
    {
        return;
    }
    if (*ready != 0u)
    {
        return;
    }

    resolved_count = MotorInstResolveIds(names, count, ids, count);
    (void)resolved_count;
    for (uint8_t i = 0u; i < count; i++)
    {
        if (ids[i] == MotorCount)
        {
            ids[i] = MotorIdRange(fallback_first, i, count);
        }
    }
    *ready = 1u;
}

static MotorId CanRxCachedMotorId(const CanRxMotorIdCache *cache)
{
    if (cache == NULL)
    {
        return MotorCount;
    }
    if (cache->ready == 0u)
    {
        return cache->fallback_id;
    }

    return cache->resolved_id;
}

static MotorId CanRxCachedIndexedMotorId(const MotorId *ids,
                                              const uint8_t *ready,
                                              uint8_t count,
                                              MotorId fallback_first,
                                              uint8_t index)
{
    if (ids == NULL || ready == NULL || count == 0u)
    {
        return MotorCount;
    }
    index = (uint8_t)(index % count);
    if (*ready == 0u)
    {
        return MotorIdRange(fallback_first, index, count);
    }

    return ids[index];
}

void CAN_rx_prepare_motor_measure_points(void)
{
    CanRxPrepareMotorIdCache(&CanRxYawId);
    CanRxPrepareMotorIdCache(&CanRxYawUpperId);
    CanRxPrepareMotorIdCache(&CanRxPitchId);
    CanRxPrepareMotorIdCache(&CanRxTriggerId);
    CanRxPrepareIndexedMotorIds(CanRxChassisMotorNames,
                                     CanRxChassisIds,
                                     &CanRxChassisIdsReady,
                                     4u,
                                     Motor0);
    CanRxPrepareIndexedMotorIds(CanRxFrictionMotorNames,
                                     CanRxFrictionIds,
                                     &CanRxFrictionIdsReady,
                                     4u,
                                     Motor8);
}
// 大疆反馈帧里 16 位整数是高字节在前，这里统一做一次读取。
static int16_t CanRxReadS16Be(const uint8_t *ptr)
{
    return (int16_t)(((uint16_t)ptr[0] << 8) | (uint16_t)ptr[1]);
}

// 把连续角度折回 [0, 2pi)，用于伪造旧编码器计数。
static fp32 CanRxWrap02pi(fp32 angle)
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

// MIT 电机返回弧度位置，旧控制代码仍看 ecd，这里按电机型号转换成单圈计数。
static uint32_t CanRxNodeEcdRange(const motor_node_param_t *node)
{
    return (node != NULL) ? MotorCfgEncoderRange(node->model) : 8192u;
}

static uint16_t CanRxPositionToEcd(fp32 position, const motor_node_param_t *node)
{
    const fp32 wrapped = CanRxWrap02pi(position);
    const uint32_t ecd_range = CanRxNodeEcdRange(node);
    uint32_t ecd = (uint32_t)((wrapped * (fp32)ecd_range / CAN_RX_TWO_PI) + 0.5f);

    if (ecd >= ecd_range)
    {
        ecd = ecd_range - 1u;
    }
    return (uint16_t)ecd;
}

// 浮点量写回旧结构前做 int16 饱和，避免溢出反号。
static int16_t CanRxFloatToI16Saturated(fp32 x)
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
static int16_t CanRxTorqueToCurrentLike(const CanMitMotorLimits *limits, fp32 torque)
{
    fp32 scaled;

    if (limits == NULL || limits->torque_max <= 0.0f)
    {
        return 0;
    }

    scaled = torque * 32767.0f / limits->torque_max;
    return CanRxFloatToI16Saturated(scaled);
}

static uint8_t CanRxMitDriveState(const motor_node_param_t *node, uint8_t state)
{
    if (state >= CAN_RX_MIT_STATE_FAULT_MIN)
    {
        return (uint8_t)MotorDriveStateFault;
    }
    if (state == CAN_RX_MIT_STATE_ENABLED)
    {
        return (uint8_t)MotorDriveStateEnabled;
    }
    if (state == 0u && MotorCfgProtocol(node) == MOTOR_PROTOCOL_DM_3MODE)
    {
        return (uint8_t)MotorDriveStateReady;
    }
    if (state == CAN_RX_MIT_STATE_DISABLED)
    {
        return (uint8_t)MotorDriveStateDisabled;
    }
    return (uint8_t)MotorDriveStateReady;
}

// 把大疆类反馈帧同步到通用执行器反馈，供新控制链按轴读取。
static void CanRxUpdateLowStateFromMeasure(MotorId actuator_id,
                                                         uint8_t bus,
                                                         uint16_t std_id,
                                                         uint8_t dlc,
                                                         const motor_node_param_t *node,
                                                         const motor_measure_t *measure)
{
    MotorState prev;
    MotorState fb;
    uint32_t prev_rx_count = 0u;
    const uint32_t ecd_range = CanRxNodeEcdRange(node);

    if ((uint32_t)actuator_id >= (uint32_t)MotorCount || measure == NULL)
    {
        return;
    }

    (void)memset(&fb, 0, sizeof(fb));
    if (LowStateGetMotor(actuator_id, &prev) != 0u)
    {
        prev_rx_count = prev.rxCount;
    }
    fb.online = 1u;
    fb.bus = bus;
    fb.rxDlc = dlc;
    fb.transport = (uint8_t)MotorTransportCAN;
    fb.driveState = (uint8_t)MotorDriveStateEnabled;
    fb.rxId = std_id;
    fb.rxCount = MotorFeedbackRxCountNext(prev_rx_count);
    fb.lastRxTick = BspTimeGetTickMs();
    fb.lastEcd = (uint16_t)measure->last_ecd;
    fb.ecd = measure->ecd;
    fb.speedRpm = measure->speed_rpm;
    fb.current = measure->given_current;
    fb.temperature = measure->temperate;
    fb.q = ((fp32)measure->ecd) * CAN_RX_TWO_PI / (fp32)ecd_range;
    fb.dq = ((fp32)measure->speed_rpm) * CAN_RX_RPM_TO_RADPS;
    fb.tauEst = (fp32)measure->given_current;
    LowStateUpdateMotor(actuator_id, &fb);
}

// MIT 反馈没有旧 ecd/rpm/current 格式，这里合成一份给老任务继续用。
static void CanRxSynthesizeMeasureFromMit(motor_measure_t *measure,
                                               const CanMitMotorLimits *limits,
                                               const motor_node_param_t *node,
                                               const CanMitMotorFeedback *mit)
{
    if (measure == NULL || limits == NULL || mit == NULL)
    {
        return;
    }

    measure->last_ecd = measure->ecd;
    measure->ecd = CanRxPositionToEcd(mit->position, node);
    measure->speed_rpm = CanRxFloatToI16Saturated(mit->velocity * CAN_RX_RADPS_TO_RPM);
    measure->given_current = CanRxTorqueToCurrentLike(limits, mit->torque);
    measure->temperate = mit->mos_temperature;
}

// 尝试按 MIT 协议解析某个轴；成功后同时更新旧 measure 和通用执行器反馈。
static uint8_t CanRxProcessMitNodeFrame(motor_measure_t *measure,
                                             const motor_node_param_t *node,
                                             uint8_t bus,
                                             uint16_t std_id,
                                             uint8_t dlc,
                                             const uint8_t data[8],
                                             MotorId actuator_id,
                                             uint8_t DetectToe,
                                             uint8_t use_detect)
{
    const CanMitMotorLimits *limits;
    CanMitMotorFeedback mit;
    MotorState prev;
    MotorState fb;
    uint32_t prev_rx_count = 0u;

    if (node == NULL)
    {
        return 0u;
    }

    limits = MotorCfgMitLimits(node);
    if (limits == NULL)
    {
        return 0u;
    }

    (void)memset(&mit, 0, sizeof(mit));
    if (CanMitMotorUpdateFeedback(std_id, node->can_id, limits, dlc, data, &mit) == 0u)
    {
        return 0u;
    }

    CanRxSynthesizeMeasureFromMit(measure, limits, node, &mit);

    if ((uint32_t)actuator_id < (uint32_t)MotorCount)
    {
        (void)memset(&fb, 0, sizeof(fb));
        if (LowStateGetMotor(actuator_id, &prev) != 0u)
        {
            prev_rx_count = prev.rxCount;
        }
        fb.online = 1u;
        fb.bus = bus;
        fb.rxDlc = dlc;
        fb.rxData0 = data[0];
        fb.transport = (uint8_t)MotorTransportCAN;
        fb.motorId = mit.motor_id;
        fb.state = mit.state;
        fb.driveState = CanRxMitDriveState(node, mit.state);
        fb.rxId = std_id;
        fb.rxCount = MotorFeedbackRxCountNext(prev_rx_count);
        fb.lastRxTick = mit.last_rx_tick;
        fb.q = mit.position;
        fb.dq = mit.velocity;
        fb.tauEst = mit.torque;
        if (measure != NULL)
        {
            fb.lastEcd = (uint16_t)measure->last_ecd;
            fb.ecd = measure->ecd;
            fb.speedRpm = measure->speed_rpm;
            fb.current = measure->given_current;
            fb.temperature = measure->temperate;
        }
        else
        {
            fb.temperature = mit.mos_temperature;
        }
        LowStateUpdateMotor(actuator_id, &fb);
    }

    if (use_detect != 0u)
    {
        DetectHook(DetectToe);
    }
    return 1u;
}

// 按电机型号的反馈描述拆包，避免把不同电机的字段位置写死在任务里。
static void CanRxUnpackMotorMeasure(motor_measure_t *measure, MotorModel model, const uint8_t data[8])
{
    const MotorModelRxDesc *rx = MotorCfgRxDesc(model);

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
    measure->speed_rpm = (rx->speed_rpm_off != MOTOR_MODEL_RX_OFF_NONE) ? CanRxReadS16Be(&data[rx->speed_rpm_off]) : 0;
    measure->temperate = (rx->temp_off != MOTOR_MODEL_RX_OFF_NONE) ? data[rx->temp_off] : 0u;

    switch ((MotorModelRxCurrentSetPolicy)rx->current_set_policy)
    {
    case MOTOR_MODEL_RX_CUR_SET_FROM_FRAME:
        measure->given_current =
            (rx->current_set_off != MOTOR_MODEL_RX_OFF_NONE) ? CanRxReadS16Be(&data[rx->current_set_off]) : 0;
        break;
    case MOTOR_MODEL_RX_CUR_SET_SAME_AS_MEAS:
        measure->given_current =
            (rx->current_meas_off != MOTOR_MODEL_RX_OFF_NONE) ? CanRxReadS16Be(&data[rx->current_meas_off]) : 0;
        break;
    default:
        measure->given_current = 0;
        break;
    }
}

// 按轴配置决定怎么处理反馈：大疆电流帧直接拆，MIT 帧走 MIT 解析，其他交给扩展口。
static uint8_t CanRxProcessNodeFrame(motor_measure_t *measure,
                                         const motor_node_param_t *node,
                                         uint8_t bus,
                                         uint16_t std_id,
                                         uint8_t dlc,
                                         const uint8_t data[8],
                                         uint8_t DetectToe,
                                         uint8_t use_detect,
                                         MotorId actuator_id)
{
    if (node == NULL)
    {
        return 0u;
    }

    if (MotorCfgIsRmGroupProtocol(node) != 0u)
    {
        CanRxUnpackMotorMeasure(measure, node->model, data);
        if (use_detect != 0u)
        {
            DetectHook(DetectToe);
        }
        CanRxUpdateLowStateFromMeasure(actuator_id, bus, std_id, dlc, node, measure);
        return 1u;
    }

    if (CanRxProcessMitNodeFrame(measure,
                                      node,
                                      bus,
                                      std_id,
                                      dlc,
                                      data,
                                      actuator_id,
                                      DetectToe,
                                      use_detect) != 0u)
    {
        return 1u;
    }

    if (CAN_rx_process_extra_frame(bus, std_id, dlc, data) != 0u)
    {
        return 1u;
    }

    WatchTaskError(WATCH_TASK_CAN_FEEDBACK_RX);
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
    const MotorInst *inst = NULL;

    if (data == NULL)
    {
        return;
    }

    inst = MotorInstFindFeedback(bus, std_id);
    if (inst != NULL)
    {
        (void)CanRxProcessNodeFrame(inst->measure,
                                        inst->node,
                                        bus,
                                        std_id,
                                        dlc,
                                        data,
                                        inst->DetectToe,
                                        inst->use_detect,
                                        inst->actuator_id);
        return;
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

    if (bus == 1u && group_id == (uint16_t)CAN_RM_GROUP_0X1FF_ID)
    {
        last_can1ff_status = (uint8_t)BspCanTx(bus, group_id, data, 8u);
        return;
    }

    (void)BspCanTx(bus, group_id, data, 8u);
}

void CAN_cmd_chassis_reset_ID(void)
{
    uint8_t data[8] = {0};
    (void)BspCanTx(1u, 0x700u, data, 8u);
}

const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
    return MotorInstMeasureConst(CanRxCachedMotorId(&CanRxYawId));
}

const motor_measure_t *get_yaw_upper_gimbal_motor_measure_point(void)
{
    return MotorInstMeasureConst(CanRxCachedMotorId(&CanRxYawUpperId));
}

const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
    return MotorInstMeasureConst(CanRxCachedMotorId(&CanRxPitchId));
}

const motor_measure_t *get_trigger_motor_measure_point(void)
{
    return MotorInstMeasureConst(CanRxCachedMotorId(&CanRxTriggerId));
}

const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
    return MotorInstMeasureConst(CanRxCachedIndexedMotorId(CanRxChassisIds,
                                                                       &CanRxChassisIdsReady,
                                                                       4u,
                                                                       Motor0,
                                                                       (uint8_t)(i & 0x03u)));
}

const motor_measure_t *get_friction_motor_measure_point(uint8_t i)
{
    return MotorInstMeasureConst(CanRxCachedIndexedMotorId(CanRxFrictionIds,
                                                                       &CanRxFrictionIdsReady,
                                                                       4u,
                                                                       Motor8,
                                                                       (uint8_t)(i & 0x03u)));
}

uint8_t CAN_get_last_1ff_status(void)
{
    return last_can1ff_status;
}

uint32_t CAN_get_last_can1_error(void)
{
    return BspCanGetLastError(1u);
}

uint32_t CAN_get_last_can2_error(void)
{
    return BspCanGetLastError(2u);
}

uint32_t CAN_get_last_can3_error(void)
{
    return BspCanGetLastError(3u);
}

uint32_t CAN_get_can1_rx_drop_count(void)
{
    return BspCanRxGetDropCount(1u);
}

uint32_t CAN_get_can2_rx_drop_count(void)
{
    return BspCanRxGetDropCount(2u);
}

uint32_t CAN_get_can3_rx_drop_count(void)
{
    return BspCanRxGetDropCount(3u);
}

uint32_t CAN_get_can1_rx_count(void)
{
    return BspCanRxGetCount(1u);
}

uint32_t CAN_get_can2_rx_count(void)
{
    return BspCanRxGetCount(2u);
}

uint32_t CAN_get_can3_rx_count(void)
{
    return BspCanRxGetCount(3u);
}

uint16_t CAN_get_can1_last_rx_id(void)
{
    return BspCanRxGetLastStdId(1u);
}

uint16_t CAN_get_can2_last_rx_id(void)
{
    return BspCanRxGetLastStdId(2u);
}

uint16_t CAN_get_can3_last_rx_id(void)
{
    return BspCanRxGetLastStdId(3u);
}

uint8_t CAN_get_can1_last_rx_dlc(void)
{
    return BspCanRxGetLastDlc(1u);
}

uint8_t CAN_get_can2_last_rx_dlc(void)
{
    return BspCanRxGetLastDlc(2u);
}

uint8_t CAN_get_can3_last_rx_dlc(void)
{
    return BspCanRxGetLastDlc(3u);
}

uint16_t CAN_get_can1_last_tx_id(void)
{
    return BspCanGetLastTxStdId(1u);
}

uint16_t CAN_get_can2_last_tx_id(void)
{
    return BspCanGetLastTxStdId(2u);
}

uint16_t CAN_get_can3_last_tx_id(void)
{
    return BspCanGetLastTxStdId(3u);
}

uint8_t CAN_get_can1_last_tx_dlc(void)
{
    return BspCanGetLastTxDlc(1u);
}

uint8_t CAN_get_can2_last_tx_dlc(void)
{
    return BspCanGetLastTxDlc(2u);
}

uint8_t CAN_get_can3_last_tx_dlc(void)
{
    return BspCanGetLastTxDlc(3u);
}

uint32_t CAN_get_can1_tx_count(void)
{
    return BspCanGetTxCount(1u);
}

uint32_t CAN_get_can2_tx_count(void)
{
    return BspCanGetTxCount(2u);
}

uint32_t CAN_get_can3_tx_count(void)
{
    return BspCanGetTxCount(3u);
}

uint32_t CAN_get_can1_tx_fail_count(void)
{
    return BspCanGetTxFailCount(1u);
}

uint32_t CAN_get_can2_tx_fail_count(void)
{
    return BspCanGetTxFailCount(2u);
}

uint32_t CAN_get_can3_tx_fail_count(void)
{
    return BspCanGetTxFailCount(3u);
}

uint8_t CAN_get_can1_protocol_lec(void)
{
    return BspCanGetProtocolLastErrorCode(1u);
}

uint8_t CAN_get_can2_protocol_lec(void)
{
    return BspCanGetProtocolLastErrorCode(2u);
}

uint8_t CAN_get_can3_protocol_lec(void)
{
    return BspCanGetProtocolLastErrorCode(3u);
}

uint8_t CAN_get_can1_protocol_dlec(void)
{
    return BspCanGetProtocolDataLastErrorCode(1u);
}

uint8_t CAN_get_can2_protocol_dlec(void)
{
    return BspCanGetProtocolDataLastErrorCode(2u);
}

uint8_t CAN_get_can3_protocol_dlec(void)
{
    return BspCanGetProtocolDataLastErrorCode(3u);
}

uint8_t CAN_get_can1_protocol_activity(void)
{
    return BspCanGetProtocolActivity(1u);
}

uint8_t CAN_get_can2_protocol_activity(void)
{
    return BspCanGetProtocolActivity(2u);
}

uint8_t CAN_get_can3_protocol_activity(void)
{
    return BspCanGetProtocolActivity(3u);
}

uint8_t CAN_get_can1_error_passive(void)
{
    return BspCanGetProtocolErrorPassive(1u);
}

uint8_t CAN_get_can2_error_passive(void)
{
    return BspCanGetProtocolErrorPassive(2u);
}

uint8_t CAN_get_can3_error_passive(void)
{
    return BspCanGetProtocolErrorPassive(3u);
}

uint8_t CAN_get_can1_error_warning(void)
{
    return BspCanGetProtocolWarning(1u);
}

uint8_t CAN_get_can2_error_warning(void)
{
    return BspCanGetProtocolWarning(2u);
}

uint8_t CAN_get_can3_error_warning(void)
{
    return BspCanGetProtocolWarning(3u);
}

uint8_t CAN_get_can1_bus_off(void)
{
    return BspCanGetProtocolBusOff(1u);
}

uint8_t CAN_get_can2_bus_off(void)
{
    return BspCanGetProtocolBusOff(2u);
}

uint8_t CAN_get_can3_bus_off(void)
{
    return BspCanGetProtocolBusOff(3u);
}

uint8_t CAN_get_can1_tx_error_count(void)
{
    return BspCanGetTxErrorCount(1u);
}

uint8_t CAN_get_can2_tx_error_count(void)
{
    return BspCanGetTxErrorCount(2u);
}

uint8_t CAN_get_can3_tx_error_count(void)
{
    return BspCanGetTxErrorCount(3u);
}

uint8_t CAN_get_can1_rx_error_count(void)
{
    return BspCanGetRxErrorCount(1u);
}

uint8_t CAN_get_can2_rx_error_count(void)
{
    return BspCanGetRxErrorCount(2u);
}

uint8_t CAN_get_can3_rx_error_count(void)
{
    return BspCanGetRxErrorCount(3u);
}

uint8_t CAN_get_can1_error_logging_count(void)
{
    return BspCanGetErrorLoggingCount(1u);
}

uint8_t CAN_get_can2_error_logging_count(void)
{
    return BspCanGetErrorLoggingCount(2u);
}

uint8_t CAN_get_can3_error_logging_count(void)
{
    return BspCanGetErrorLoggingCount(3u);
}
