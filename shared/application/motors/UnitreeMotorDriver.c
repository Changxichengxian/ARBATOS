/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "UnitreeMotorDriver.h"

#include "FreeRTOS.h"
#include "task.h"

#include "LowCmd.h"
#include "BspUsart.h"
#include "main.h"
#include "MotorConfig.h"
#include "MotorFeedbackEcdPolicy.h"
#include "MotorInst.h"
#include "RobotSafety.h"

#include <string.h>

typedef enum
{
    UNITREE_MOTOR_MODE_BRAKE = 0u,
    UNITREE_MOTOR_MODE_FOC = 1u,
    UNITREE_MOTOR_MODE_CALIBRATE = 2u,
} UnitreeMotorMode;

#pragma pack(push, 1)
typedef struct
{
    uint8_t start[2];
    uint8_t motor_id;
    uint8_t head_res;
} UnitreeMotorHead;

typedef struct
{
    uint8_t mode;
    uint8_t modify_bit;
    uint8_t read_bit;
    uint8_t mid_res;
    uint32_t modify;
    int16_t torque_q8;
    int16_t speed_q7;
    int32_t position_q14;
    int16_t kp_q11;
    int16_t kd_q9;
    uint8_t low_hz_cmd_index;
    uint8_t low_hz_cmd_byte;
    uint32_t end_res;
} UnitreeMotorMasterCmd;

typedef struct
{
    UnitreeMotorHead head;
    UnitreeMotorMasterCmd cmd;
    uint32_t crc32;
} UnitreeMotorTxFrame;

typedef struct
{
    uint8_t mode;
    uint8_t read_bit;
    int8_t temp;
    uint8_t motor_error;
    uint8_t read;
    int16_t torque_q8;
    int16_t speed_q7;
    float rotor_speed_low;
    int16_t joint_speed_q7;
    float joint_speed_low;
    int16_t rotor_acc;
    int16_t joint_acc;
    int32_t rotor_pos_q14;
    int32_t joint_pos_q14;
    int16_t gyro[3];
    int16_t accel[3];
    int16_t force_gyro[3];
    int16_t force_acc[3];
    int16_t force_mag[3];
    uint8_t force_temp;
    int16_t force16;
    int8_t force8;
    uint8_t force_error;
    int8_t reserved[1];
} UnitreeMotorServoCmd;

typedef struct
{
    UnitreeMotorHead head;
    UnitreeMotorServoCmd data;
    uint32_t crc32;
} UnitreeMotorRxFrame;
#pragma pack(pop)

typedef char UnitreeMotorTxFrameFitsBsp[
    (sizeof(UnitreeMotorTxFrame) <= BSP_RS485_TX_IT_MAX_LEN) ? 1 : -1];

#define UNITREE_MOTOR_PI_F                3.1415926f
#define UNITREE_MOTOR_TWO_PI_F            (2.0f * UNITREE_MOTOR_PI_F)
#define UNITREE_MOTOR_TORQUE_SCALE        256.0f
#define UNITREE_MOTOR_SPEED_SCALE         128.0f
#define UNITREE_MOTOR_POSITION_SCALE      (16384.0f / UNITREE_MOTOR_TWO_PI_F)
#define UNITREE_MOTOR_KP_SCALE            2048.0f
#define UNITREE_MOTOR_KD_SCALE            512.0f
#define UNITREE_MOTOR_TX_WORD_COUNT       7u
#define UNITREE_MOTOR_RX_WORD_COUNT       18u
#define UNITREE_MOTOR_RX_FRAME_SIZE       ((uint16_t)sizeof(UnitreeMotorRxFrame))
#define UNITREE_MOTOR_RADPS_TO_RPM        9.5492965855f

static UnitreeMotorState g_unitree_motor_state;
static UnitreeMotorConfig g_unitree_motor_cfg;
static uint8_t g_unitree_motor_rx_buf[UNITREE_MOTOR_RX_FRAME_SIZE];
static volatile uint16_t g_unitree_motor_rx_pos = 0u;
static volatile uint8_t g_unitree_motor_rs485_ready = 0u;
static uint8_t g_unitree_motor_active_port = 0xFFu;
static uint32_t g_unitree_motor_active_baudrate = 0u;

static uint32_t UnitreeMotorCrc32Words(const uint8_t *data, uint32_t word_count);
static int16_t UnitreeMotorFloatToQ(fp32 value, fp32 scale);
static int32_t UnitreeMotorPosToQ14(fp32 value);
static fp32 UnitreeMotorQ14ToPos(int32_t value);
static fp32 UnitreeMotorFeedbackSpeed(int16_t speed_q7, float speed_low);
static void UnitreeMotorBuildTxFrame(UnitreeMotorTxFrame *frame,
                                         uint8_t motor_id,
                                         UnitreeMotorMode mode,
                                         const UnitreeMotorCmd *cmd);
static void UnitreeMotorProcessRxFrame(const uint8_t *frame_bytes);
static void UnitreeMotorIngestRxByte(uint8_t b);
static void UnitreeMotorUsart2RxByte(uint8_t b);
static void UnitreeMotorUsart3RxByte(uint8_t b);
static uint8_t UnitreeMotorUsart2Error(void);
static uint8_t UnitreeMotorUsart3Error(void);
static uint8_t UnitreeMotorSetupRs485(const UnitreeMotorConfig *cfg);
static int UnitreeMotorSendFrame(const UnitreeMotorConfig *cfg,
                                 const UnitreeMotorTxFrame *active_frame,
                                 const UnitreeMotorTxFrame *safe_frame,
                                 MotorId actuator_id,
                                 const MotorCmd *cached_cmd,
                                 uint8_t require_authority,
                                 uint8_t *used_safe_frame,
                                 uint8_t *authority_rejected);
static fp32 UnitreeMotorClampFp32(fp32 value, fp32 min_value, fp32 max_value);
static fp32 UnitreeMotorCurrentToTorque(const motor_node_param_t *node,
                                            int16_t current,
                                            const MotorModelMitLimits *limits);
static int16_t UnitreeMotorTorqueToCurrentLike(const MotorModelMitLimits *limits, fp32 torque);
static uint16_t UnitreeMotorPositionToEcd(fp32 position);
static void UnitreeMotorBuildConfigFromNode(UnitreeMotorConfig *out,
                                                 uint8_t port,
                                                 const motor_node_param_t *node);
static uint8_t UnitreeMotorBuildCmdFromActuator(const motor_node_param_t *node,
                                                     MotorId actuator_id,
                                                     int16_t current,
                                                     const MotorCmd *can_tx_cmd,
                                                     UnitreeMotorCmd *out,
                                                     MotorMode *applied_mode);
static void UnitreeMotorRefreshFeedback(MotorId actuator_id, const motor_node_param_t *node);
static void UnitreeMotorUpdateApplied(MotorId actuator_id,
                                         uint8_t port,
                                         const motor_node_param_t *node,
                                         const UnitreeMotorCmd *cmd,
                                          MotorMode mode,
                                          int16_t current,
                                          int ret,
                                          uint8_t safe_substituted);

static uint32_t UnitreeMotorCrc32Words(const uint8_t *data, uint32_t word_count)
{
    uint32_t crc32 = 0xFFFFFFFFu;
    const uint32_t polynomial = 0x04C11DB7u;

    if (data == NULL)
    {
        return 0u;
    }

    for (uint32_t i = 0u; i < word_count; i++)
    {
        uint32_t xbit = 1u << 31;
        uint32_t word =
            ((uint32_t)data[i * 4u + 0u]) |
            ((uint32_t)data[i * 4u + 1u] << 8) |
            ((uint32_t)data[i * 4u + 2u] << 16) |
            ((uint32_t)data[i * 4u + 3u] << 24);

        for (uint32_t bit = 0u; bit < 32u; bit++)
        {
            if ((crc32 & 0x80000000u) != 0u)
            {
                crc32 <<= 1;
                crc32 ^= polynomial;
            }
            else
            {
                crc32 <<= 1;
            }

            if ((word & xbit) != 0u)
            {
                crc32 ^= polynomial;
            }

            xbit >>= 1;
        }
    }

    return crc32;
}

static int16_t UnitreeMotorFloatToQ(fp32 value, fp32 scale)
{
    fp32 scaled = value * scale;

    if (scaled > 32767.0f)
    {
        scaled = 32767.0f;
    }
    else if (scaled < -32768.0f)
    {
        scaled = -32768.0f;
    }

    return (int16_t)((scaled >= 0.0f) ? (scaled + 0.5f) : (scaled - 0.5f));
}

static int32_t UnitreeMotorPosToQ14(fp32 value)
{
    fp32 scaled = value * UNITREE_MOTOR_POSITION_SCALE;

    if (scaled > 2147483647.0f)
    {
        scaled = 2147483647.0f;
    }
    else if (scaled < -2147483648.0f)
    {
        scaled = -2147483648.0f;
    }

    return (int32_t)((scaled >= 0.0f) ? (scaled + 0.5f) : (scaled - 0.5f));
}

static fp32 UnitreeMotorQ14ToPos(int32_t value)
{
    return ((fp32)value) / UNITREE_MOTOR_POSITION_SCALE;
}

static fp32 UnitreeMotorFeedbackSpeed(int16_t speed_q7, float speed_low)
{
    if ((speed_low > 0.0001f) || (speed_low < -0.0001f))
    {
        return (fp32)speed_low;
    }

    return ((fp32)speed_q7) / UNITREE_MOTOR_SPEED_SCALE;
}

static fp32 UnitreeMotorClampFp32(fp32 value, fp32 min_value, fp32 max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

static fp32 UnitreeMotorCurrentToTorque(const motor_node_param_t *node,
                                            int16_t current,
                                            const MotorModelMitLimits *limits)
{
    const MotorModelDbEntry *entry;
    int16_t range_abs = 32767;
    fp32 torque;

    if (node == NULL || limits == NULL || limits->torque_max <= 0.0f)
    {
        return 0.0f;
    }

    entry = MotorCfgModelDb(node->model);
    if (entry != NULL && entry->cmd_current_range_abs > 0)
    {
        range_abs = entry->cmd_current_range_abs;
    }

    torque = ((fp32)current) * limits->torque_max / (fp32)range_abs;
    return UnitreeMotorClampFp32(torque, -limits->torque_max, limits->torque_max);
}

static int16_t UnitreeMotorTorqueToCurrentLike(const MotorModelMitLimits *limits, fp32 torque)
{
    fp32 scaled;

    if (limits == NULL || limits->torque_max <= 0.0f)
    {
        return 0;
    }

    scaled = torque * 32767.0f / limits->torque_max;
    if (scaled > 32767.0f)
    {
        scaled = 32767.0f;
    }
    else if (scaled < -32768.0f)
    {
        scaled = -32768.0f;
    }

    return (int16_t)scaled;
}

static uint16_t UnitreeMotorPositionToEcd(fp32 position)
{
    while (position < 0.0f)
    {
        position += UNITREE_MOTOR_TWO_PI_F;
    }
    while (position >= UNITREE_MOTOR_TWO_PI_F)
    {
        position -= UNITREE_MOTOR_TWO_PI_F;
    }
    return (uint16_t)(position * 8192.0f / UNITREE_MOTOR_TWO_PI_F);
}

static void UnitreeMotorBuildConfigFromNode(UnitreeMotorConfig *out,
                                                 uint8_t port,
                                                 const motor_node_param_t *node)
{
    if (out == NULL)
    {
        return;
    }

    (void)memset(out, 0, sizeof(*out));
    if (node == NULL)
    {
        return;
    }

    out->enable = UnitreeMotorNodeSupported(node);
    out->rs485_port = (node->rs485_port <= UNITREE_MOTOR_RS485_PORT1) ? node->rs485_port : port;
    out->motor_id = MotorCfgNodeId(node);
    out->baudrate = (node->baudrate != 0u) ? node->baudrate : 4000000u;
    out->rx_timeout_ms = UnitreeMotorRxTimeoutMs(node->rx_timeout_ms);
}

static uint8_t UnitreeMotorBuildCmdFromActuator(const motor_node_param_t *node,
                                                     MotorId actuator_id,
                                                     int16_t current,
                                                     const MotorCmd *can_tx_cmd,
                                                     UnitreeMotorCmd *out,
                                                     MotorMode *applied_mode)
{
    const MotorModelMitLimits *limits = MotorCfgMitLimits(node);
    MotorCmd src;
    MotorCmd latest;
    uint16_t inhibit_writer = (uint16_t)LOWCMD_WRITER_NONE;
    uint8_t active_cmd;
    int16_t effective_current;
    MotorMode mode = MotorModeCurrent;

    if (out == NULL || applied_mode == NULL)
    {
        return 0u;
    }

    (void)memset(out, 0, sizeof(*out));
    *applied_mode = MotorModeDisable;

    if (limits == NULL)
    {
        /* 型号能力缺失时仍发送 BRAKE，不能把上一条 FOC 命令留在电机里。 */
        return 1u;
    }

    if (RobotSafetyOutputLocked() != 0u)
    {
        return 1u;
    }

    (void)current;
    (void)memset(&src, 0, sizeof(src));
    (void)memset(&latest, 0, sizeof(latest));
    if (can_tx_cmd != NULL)
    {
        src = *can_tx_cmd;
    }

    /* CanTx 的 Disable 已经是安全裁决，不需要再与业务快照匹配。 */
    if (src.mode == (uint8_t)MotorModeDisable)
    {
        return 1u;
    }

    active_cmd = (uint8_t)(LowCmdGetMotor(actuator_id, &latest) != 0u &&
                           LowCmdGetInhibitWriter(actuator_id, &inhibit_writer) != 0u &&
                           UnitreeMotorCmdSnapshotAllowed(&src, &latest, inhibit_writer) != 0u);
    effective_current = UnitreeMotorSafeCurrent(&src, &latest, inhibit_writer);

    if (active_cmd == 0u)
    {
        return 1u;
    }

    if (active_cmd != 0u)
    {
        mode = (MotorMode)src.mode;
    }

    if (mode == MotorModeDisable)
    {
        return 1u;
    }

    if (active_cmd != 0u && mode != MotorModeCurrent)
    {
        if (UnitreeMotorMapLowCmd(&src, MotorCfgReductionRatio(node->model), out) == 0u)
        {
            *applied_mode = MotorModeDisable;
            return 1u;
        }
    }
    else
    {
        out->torque_nm = UnitreeMotorCurrentToTorque(node, effective_current, limits);
    }

    out->position_rad = UnitreeMotorClampFp32(out->position_rad, -limits->position_max, limits->position_max);
    out->speed_rad_s = UnitreeMotorClampFp32(out->speed_rad_s, -limits->velocity_max, limits->velocity_max);
    out->kp = UnitreeMotorClampFp32(out->kp, 0.0f, limits->kp_max);
    out->kd = UnitreeMotorClampFp32(out->kd, 0.0f, limits->kd_max);
    out->torque_nm = UnitreeMotorClampFp32(out->torque_nm, -limits->torque_max, limits->torque_max);
    *applied_mode = mode;
    return 1u;
}

static void UnitreeMotorBuildTxFrame(UnitreeMotorTxFrame *frame,
                                       uint8_t motor_id,
                                       UnitreeMotorMode mode,
                                       const UnitreeMotorCmd *cmd)
{
    UnitreeMotorCmd safe_cmd = {0};

    if (frame == NULL)
    {
        return;
    }

    if (cmd != NULL)
    {
        safe_cmd = *cmd;
    }

    (void)memset(frame, 0, sizeof(*frame));
    frame->head.start[0] = 0xFEu;
    frame->head.start[1] = 0xEEu;
    frame->head.motor_id = motor_id;
    frame->cmd.mode = (uint8_t)mode;
    frame->cmd.modify_bit = 0xFFu;
    frame->cmd.read_bit = 0u;
    frame->cmd.mid_res = 0u;
    frame->cmd.modify = 0u;
    frame->cmd.torque_q8 = UnitreeMotorFloatToQ(safe_cmd.torque_nm, UNITREE_MOTOR_TORQUE_SCALE);
    frame->cmd.speed_q7 = UnitreeMotorFloatToQ(safe_cmd.speed_rad_s, UNITREE_MOTOR_SPEED_SCALE);
    frame->cmd.position_q14 = UnitreeMotorPosToQ14(safe_cmd.position_rad);
    frame->cmd.kp_q11 = UnitreeMotorFloatToQ(safe_cmd.kp, UNITREE_MOTOR_KP_SCALE);
    frame->cmd.kd_q9 = UnitreeMotorFloatToQ(safe_cmd.kd, UNITREE_MOTOR_KD_SCALE);
    frame->cmd.low_hz_cmd_index = 0u;
    frame->cmd.low_hz_cmd_byte = 0u;
    frame->cmd.end_res = 0u;
    frame->crc32 = UnitreeMotorCrc32Words((const uint8_t *)frame, UNITREE_MOTOR_TX_WORD_COUNT);
}

static void UnitreeMotorProcessRxFrame(const uint8_t *frame_bytes)
{
    UnitreeMotorRxFrame frame;
    uint32_t crc32;
    const UnitreeMotorConfig *cfg = &g_unitree_motor_cfg;

    if (frame_bytes == NULL)
    {
        return;
    }

    (void)memcpy(&frame, frame_bytes, sizeof(frame));

    if (frame.head.start[0] != 0xFEu || frame.head.start[1] != 0xEEu)
    {
        g_unitree_motor_state.rx_parse_error_count++;
        return;
    }

    crc32 = UnitreeMotorCrc32Words(frame_bytes, UNITREE_MOTOR_RX_WORD_COUNT);
    if (crc32 != frame.crc32)
    {
        g_unitree_motor_state.rx_crc_fail_count++;
        return;
    }

    if (cfg->enable == 0u || (cfg->motor_id != 0xBBu && frame.head.motor_id != cfg->motor_id))
    {
        g_unitree_motor_state.rx_parse_error_count++;
        return;
    }

    {
        UBaseType_t saved = taskENTER_CRITICAL_FROM_ISR();
        g_unitree_motor_state.motor_id = frame.head.motor_id;
        g_unitree_motor_state.online = 1u;
        g_unitree_motor_state.last_mode = frame.data.mode;
        g_unitree_motor_state.motor_error = frame.data.motor_error;
        g_unitree_motor_state.motor_temp = frame.data.temp;
        g_unitree_motor_state.rx_frame_count =
            MotorFeedbackRxCountNext(g_unitree_motor_state.rx_frame_count);
        g_unitree_motor_state.last_rx_tick_ms = HAL_GetTick();
        g_unitree_motor_state.torque_nm = ((fp32)frame.data.torque_q8) / UNITREE_MOTOR_TORQUE_SCALE;
        g_unitree_motor_state.joint_speed_rad_s = UnitreeMotorFeedbackSpeed(frame.data.joint_speed_q7, frame.data.joint_speed_low);
        g_unitree_motor_state.joint_position_rad = UnitreeMotorQ14ToPos(frame.data.joint_pos_q14);
        taskEXIT_CRITICAL_FROM_ISR(saved);
    }
}

static void UnitreeMotorIngestRxByte(uint8_t b)
{
    uint16_t pos = g_unitree_motor_rx_pos;

    if (pos == 0u)
    {
        if (b == 0xFEu)
        {
            g_unitree_motor_rx_buf[0] = b;
            g_unitree_motor_rx_pos = 1u;
        }
        return;
    }

    if (pos == 1u)
    {
        if (b == 0xEEu)
        {
            g_unitree_motor_rx_buf[1] = b;
            g_unitree_motor_rx_pos = 2u;
        }
        else
        {
            if (b == 0xFEu)
            {
                g_unitree_motor_rx_buf[0] = b;
                g_unitree_motor_rx_pos = 1u;
            }
            else
            {
                g_unitree_motor_rx_pos = 0u;
            }
        }
        return;
    }

    if (pos >= UNITREE_MOTOR_RX_FRAME_SIZE)
    {
        g_unitree_motor_rx_pos = 0u;
        return;
    }

    g_unitree_motor_rx_buf[pos] = b;
    pos++;

    if (pos >= UNITREE_MOTOR_RX_FRAME_SIZE)
    {
        g_unitree_motor_rx_pos = 0u;
        UnitreeMotorProcessRxFrame(g_unitree_motor_rx_buf);
        return;
    }

    g_unitree_motor_rx_pos = pos;
}

static void UnitreeMotorUsart2RxByte(uint8_t b)
{
    if (g_unitree_motor_active_port != UNITREE_MOTOR_RS485_PORT0)
    {
        return;
    }

    UnitreeMotorIngestRxByte(b);
}

static void UnitreeMotorUsart3RxByte(uint8_t b)
{
    if (g_unitree_motor_active_port != UNITREE_MOTOR_RS485_PORT1)
    {
        return;
    }

    UnitreeMotorIngestRxByte(b);
}

static uint8_t UnitreeMotorUsart2Error(void)
{
    if (g_unitree_motor_active_port == UNITREE_MOTOR_RS485_PORT0)
    {
        g_unitree_motor_state.rx_parse_error_count++;
    }

    return 0u;
}

static uint8_t UnitreeMotorUsart3Error(void)
{
    if (g_unitree_motor_active_port == UNITREE_MOTOR_RS485_PORT1)
    {
        g_unitree_motor_state.rx_parse_error_count++;
    }

    return 0u;
}

static void UnitreeMotorStop(void)
{
    if (g_unitree_motor_active_port == UNITREE_MOTOR_RS485_PORT0)
    {
        BspUsart2RxItStop();
    }
    else if (g_unitree_motor_active_port == UNITREE_MOTOR_RS485_PORT1)
    {
        BspUsart3RxItStop();
    }

    g_unitree_motor_active_port = 0xFFu;
    g_unitree_motor_active_baudrate = 0u;
    g_unitree_motor_rs485_ready = 0u;
    g_unitree_motor_rx_pos = 0u;
}

static uint8_t UnitreeMotorSetupRs485(const UnitreeMotorConfig *cfg)
{
    int ret = 1;

    if (cfg == NULL || cfg->enable == 0u)
    {
        UnitreeMotorStop();
        return 0u;
    }

    if (cfg->rs485_port > UNITREE_MOTOR_RS485_PORT1)
    {
        UnitreeMotorStop();
        return 0u;
    }

    if (g_unitree_motor_active_port == UNITREE_MOTOR_RS485_PORT0 && cfg->rs485_port != UNITREE_MOTOR_RS485_PORT0)
    {
        BspUsart2RxItStop();
    }
    else if (g_unitree_motor_active_port == UNITREE_MOTOR_RS485_PORT1 && cfg->rs485_port != UNITREE_MOTOR_RS485_PORT1)
    {
        BspUsart3RxItStop();
    }

    if (cfg->rs485_port == UNITREE_MOTOR_RS485_PORT0)
    {
        BspUsart2SetRxByteCb(UnitreeMotorUsart2RxByte);
        BspUsart2SetErrorCb(UnitreeMotorUsart2Error);
        if (g_unitree_motor_rs485_ready != 0u &&
            g_unitree_motor_active_port == cfg->rs485_port &&
            g_unitree_motor_active_baudrate == cfg->baudrate &&
            BspUsart2GetBaudrate() == cfg->baudrate)
        {
            return 1u;
        }
        g_unitree_motor_rx_pos = 0u;
        ret = BspUsart2SetBaudrate(cfg->baudrate);
        if (ret == 0)
        {
            ret = BspUsart2RxItStart();
        }
        if (ret == 0)
        {
            ret = BspUsart2TxItPrepare();
        }
    }
    else
    {
        BspUsart3SetRxByteCb(UnitreeMotorUsart3RxByte);
        BspUsart3SetErrorCb(UnitreeMotorUsart3Error);
        if (g_unitree_motor_rs485_ready != 0u &&
            g_unitree_motor_active_port == cfg->rs485_port &&
            g_unitree_motor_active_baudrate == cfg->baudrate &&
            BspUsart3GetBaudrate() == cfg->baudrate)
        {
            return 1u;
        }
        g_unitree_motor_rx_pos = 0u;
        ret = BspUsart3SetBaudrate(cfg->baudrate);
        if (ret == 0)
        {
            ret = BspUsart3RxItStart();
        }
        if (ret == 0)
        {
            ret = BspUsart3TxItPrepare();
        }
    }

    if (ret != 0)
    {
        g_unitree_motor_rs485_ready = 0u;
        g_unitree_motor_state.last_tx_status = (uint8_t)ret;
        return 0u;
    }

    g_unitree_motor_active_port = cfg->rs485_port;
    g_unitree_motor_active_baudrate = cfg->baudrate;
    g_unitree_motor_rs485_ready = 1u;
    return 1u;
}

static uint8_t UnitreeMotorActiveFrameAllowed(MotorId actuator_id,
                                              const MotorCmd *cached_cmd)
{
    MotorCmd latest;
    uint16_t inhibit_writer = (uint16_t)LOWCMD_WRITER_NONE;

    (void)memset(&latest, 0, sizeof(latest));
    return (uint8_t)(RobotSafetyOutputLocked() == 0u &&
                     cached_cmd != NULL &&
                     LowCmdGetMotor(actuator_id, &latest) != 0u &&
                     LowCmdGetInhibitWriter(actuator_id, &inhibit_writer) != 0u &&
                     UnitreeMotorCmdSnapshotAllowed(cached_cmd,
                                                    &latest,
                                                    inhibit_writer) != 0u);
}

static int UnitreeMotorStartFrame(uint8_t port, const UnitreeMotorTxFrame *frame)
{
    if (port == UNITREE_MOTOR_RS485_PORT0)
    {
        return BspUsart2TxItStart((const uint8_t *)frame, (uint16_t)sizeof(*frame));
    }
    if (port == UNITREE_MOTOR_RS485_PORT1)
    {
        return BspUsart3TxItStart((const uint8_t *)frame, (uint16_t)sizeof(*frame));
    }
    return 1;
}

static int UnitreeMotorWaitFrame(uint8_t port)
{
    if (port == UNITREE_MOTOR_RS485_PORT0)
    {
        return BspUsart2TxItWait(5u);
    }
    if (port == UNITREE_MOTOR_RS485_PORT1)
    {
        return BspUsart3TxItWait(5u);
    }
    return 1;
}

static int UnitreeMotorSendFrame(const UnitreeMotorConfig *cfg,
                                 const UnitreeMotorTxFrame *active_frame,
                                 const UnitreeMotorTxFrame *safe_frame,
                                 MotorId actuator_id,
                                 const MotorCmd *cached_cmd,
                                 uint8_t require_authority,
                                 uint8_t *used_safe_frame,
                                 uint8_t *authority_rejected)
{
    const UnitreeMotorTxFrame *selected_frame;
    uint8_t use_safe = 1u;
    int ret = 1;

    if (used_safe_frame != NULL)
    {
        *used_safe_frame = 1u;
    }
    if (authority_rejected != NULL)
    {
        *authority_rejected = 0u;
    }
    if (cfg == NULL || active_frame == NULL || safe_frame == NULL || cfg->enable == 0u)
    {
        return 1;
    }

    /*
     * 最终权限复核和 HAL 中断发送启动处于同一短临界区。失败时也启动
     * 已预构造的 BRAKE 帧；真正的串口完成等待在退出临界区之后进行。
     */
    taskENTER_CRITICAL();
    if (require_authority != 0u &&
        UnitreeMotorActiveFrameAllowed(actuator_id, cached_cmd) != 0u)
    {
        use_safe = 0u;
    }
    else if (require_authority != 0u && authority_rejected != NULL)
    {
        *authority_rejected = 1u;
    }
    selected_frame = (use_safe != 0u) ? safe_frame : active_frame;
    ret = UnitreeMotorStartFrame(cfg->rs485_port, selected_frame);
    taskEXIT_CRITICAL();

    if (ret == 0)
    {
        ret = UnitreeMotorWaitFrame(cfg->rs485_port);
    }
    if (used_safe_frame != NULL)
    {
        *used_safe_frame = use_safe;
    }

    g_unitree_motor_state.tx_count++;
    g_unitree_motor_state.last_tx_status = (uint8_t)ret;
    if (ret != 0)
    {
        g_unitree_motor_state.tx_fail_count++;
    }

    return ret;
}

static void UnitreeMotorRefresh(const UnitreeMotorConfig *cfg)
{
    const uint32_t now_ms = HAL_GetTick();
    uint16_t timeout_ms;

    if (cfg == NULL || cfg->enable == 0u)
    {
        UnitreeMotorStop();
        g_unitree_motor_state.enabled = 0u;
        g_unitree_motor_state.online = 0u;
        g_unitree_motor_state.cmd_speed_rad_s = 0.0f;
        g_unitree_motor_state.cmd_kd = 0.0f;
        return;
    }

    g_unitree_motor_state.enabled = cfg->enable;
    g_unitree_motor_state.rs485_port = cfg->rs485_port;
    g_unitree_motor_state.motor_id = cfg->motor_id;
    timeout_ms = UnitreeMotorRxTimeoutMs(cfg->rx_timeout_ms);

    if ((g_unitree_motor_state.last_rx_tick_ms == 0u) ||
        ((now_ms - g_unitree_motor_state.last_rx_tick_ms) > timeout_ms))
    {
        g_unitree_motor_state.online = 0u;
    }
}

static uint8_t UnitreeMotorConfigure(const UnitreeMotorConfig *cfg)
{
    if (cfg == NULL)
    {
        UnitreeMotorStop();
        return 0u;
    }

    g_unitree_motor_cfg = *cfg;
    g_unitree_motor_cfg.rx_timeout_ms = UnitreeMotorRxTimeoutMs(cfg->rx_timeout_ms);
    UnitreeMotorRefresh(cfg);
    return UnitreeMotorSetupRs485(&g_unitree_motor_cfg);
}

static int UnitreeMotorSendCmd(const UnitreeMotorConfig *cfg,
                               const UnitreeMotorCmd *cmd,
                               MotorMode applied_mode,
                               MotorId actuator_id,
                               const MotorCmd *cached_cmd,
                               uint8_t *used_safe_frame,
                               uint8_t *authority_rejected)
{
    UnitreeMotorCmd safe_cmd = {0};
    UnitreeMotorTxFrame active_frame;
    UnitreeMotorTxFrame safe_frame;
    int ret;

    if (cfg == NULL || cfg->enable == 0u)
    {
        return 1;
    }

    if (cmd != NULL)
    {
        safe_cmd = *cmd;
    }

    if (UnitreeMotorConfigure(cfg) == 0u)
    {
        return 1;
    }

    UnitreeMotorBuildTxFrame(&active_frame,
                            cfg->motor_id,
                            (UnitreeMotorBrakeRequired(applied_mode) != 0u) ?
                                UNITREE_MOTOR_MODE_BRAKE : UNITREE_MOTOR_MODE_FOC,
                            &safe_cmd);
    UnitreeMotorBuildTxFrame(&safe_frame,
                             cfg->motor_id,
                             UNITREE_MOTOR_MODE_BRAKE,
                             NULL);
    ret = UnitreeMotorSendFrame(cfg,
                                &active_frame,
                                &safe_frame,
                                actuator_id,
                                cached_cmd,
                                (uint8_t)(applied_mode != MotorModeDisable),
                                used_safe_frame,
                                authority_rejected);
    if (ret == 0)
    {
        /* 这里只记录真正进入总线的值，发送失败时保留上一条已执行命令。 */
        g_unitree_motor_state.cmd_speed_rad_s =
            (used_safe_frame != NULL && *used_safe_frame != 0u) ? 0.0f : safe_cmd.speed_rad_s;
        g_unitree_motor_state.cmd_kd =
            (used_safe_frame != NULL && *used_safe_frame != 0u) ? 0.0f : safe_cmd.kd;
    }
    return ret;
}

uint8_t UnitreeMotorNodeSupported(const motor_node_param_t *node)
{
    if (node == NULL)
    {
        return 0u;
    }
    if (MotorCfgTransport(node) != MOTOR_TRANSPORT_RS485)
    {
        return 0u;
    }
    return (MotorCfgProtocol(node) == MOTOR_PROTOCOL_UNITREE_RS485) ? 1u : 0u;
}

static void UnitreeMotorRefreshFeedback(MotorId actuator_id, const motor_node_param_t *node)
{
    const MotorModelMitLimits *limits = MotorCfgMitLimits(node);
    UnitreeMotorState state;
    motor_measure_t *measure;
    MotorState previous;
    MotorState fb;
    const MotorState *previous_feedback = NULL;
    uint8_t new_sample;

    if ((uint32_t)actuator_id >= (uint32_t)MotorCount ||
        UnitreeMotorGetStateCopy(&state) == 0u)
    {
        return;
    }

    if (LowStateGetMotor(actuator_id, &previous) != 0u)
    {
        previous_feedback = &previous;
    }
    (void)memset(&fb, 0, sizeof(fb));
    fb.online = state.online;
    fb.bus = state.rs485_port;
    fb.rxDlc = UNITREE_MOTOR_RX_FRAME_SIZE;
    fb.transport = (uint8_t)MotorTransportRS485;
    fb.motorId = state.motor_id;
    fb.state = state.last_mode;
    fb.driveState = (state.online == 0u) ?
        (uint8_t)MotorDriveStateOffline :
        ((state.motor_error != 0u) ? (uint8_t)MotorDriveStateFault : (uint8_t)MotorDriveStateEnabled);
    fb.rxId = state.motor_id;
    fb.rxCount = state.rx_frame_count;
    fb.lastRxTick = state.last_rx_tick_ms;
    fb.q = state.joint_position_rad;
    fb.dq = state.joint_speed_rad_s;
    fb.tauEst = state.torque_nm;
    fb.ecd = UnitreeMotorPositionToEcd(state.joint_position_rad);
    new_sample = MotorFeedbackEcdResolve(previous_feedback,
                                         fb.rxCount,
                                         fb.ecd,
                                         &fb.lastEcd);
    fb.speedRpm = UnitreeMotorFloatToQ(state.joint_speed_rad_s * UNITREE_MOTOR_RADPS_TO_RPM, 1.0f);
    fb.current = UnitreeMotorTorqueToCurrentLike(limits, state.torque_nm);
    fb.temperature = (uint8_t)state.motor_temp;
    LowStateUpdateMotor(actuator_id, &fb);

    measure = MotorInstMeasure(actuator_id);
    if (measure != NULL)
    {
        if (new_sample != 0u)
        {
            measure->last_ecd = (int16_t)fb.lastEcd;
            measure->ecd = fb.ecd;
        }
        measure->speed_rpm = fb.speedRpm;
        measure->given_current = fb.current;
        measure->temperate = fb.temperature;
    }
}

static void UnitreeMotorUpdateApplied(MotorId actuator_id,
                                         uint8_t port,
                                         const motor_node_param_t *node,
                                         const UnitreeMotorCmd *cmd,
                                          MotorMode mode,
                                          int16_t current,
                                          int ret,
                                          uint8_t safe_substituted)
{
    MotorApplied applied;
    UnitreeMotorCmd output_cmd;

    if ((uint32_t)actuator_id >= (uint32_t)MotorCount)
    {
        return;
    }

    (void)memset(&applied, 0, sizeof(applied));
    applied.active = 1u;
    applied.mode = (uint8_t)mode;
    applied.driveState = (mode == MotorModeDisable) ? (uint8_t)MotorDriveStateDisabled : (uint8_t)MotorDriveStateEnabled;
    applied.bus = port;
    applied.transport = (uint8_t)MotorTransportRS485;
    applied.protocol = (uint8_t)MotorCfgProtocol(node);
    applied.txId = MotorCfgNodeId(node);
    applied.tick = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    applied.current = current;
    if (cmd != NULL)
    {
        UnitreeMotorMapAppliedOutput(cmd, MotorCfgReductionRatio(node->model), &output_cmd);
        applied.q = output_cmd.position_rad;
        applied.dq = output_cmd.speed_rad_s;
        applied.kp = output_cmd.kp;
        applied.kd = output_cmd.kd;
        applied.tau = output_cmd.torque_nm;
    }
    if (ret != 0)
    {
        applied.flags |= (uint8_t)MotorAppliedFlagSkipped;
    }
    if (safe_substituted != 0u)
    {
        applied.flags |= (uint8_t)MotorAppliedFlagForceDisabled;
    }

    LowStateUpdateApplied(actuator_id, &applied);
}

int UnitreeMotorSendActuator(uint8_t port,
                            MotorId actuator_id,
                            const motor_node_param_t *node,
                            int16_t current,
                            const MotorCmd *can_tx_cmd)
{
    UnitreeMotorConfig cfg;
    UnitreeMotorCmd cmd;
    MotorMode applied_mode;
    uint8_t used_safe_frame = 1u;
    uint8_t authority_rejected = 0u;
    int ret;

    if (UnitreeMotorNodeSupported(node) == 0u)
    {
        return 1;
    }

    UnitreeMotorBuildConfigFromNode(&cfg, port, node);
    UnitreeMotorRefresh(&cfg);

    if (UnitreeMotorBuildCmdFromActuator(node,
                                        actuator_id,
                                        current,
                                        can_tx_cmd,
                                        &cmd,
                                        &applied_mode) == 0u)
    {
        UnitreeMotorRefreshFeedback(actuator_id, node);
        return 1;
    }

    ret = UnitreeMotorSendCmd(&cfg,
                              &cmd,
                              applied_mode,
                              actuator_id,
                              can_tx_cmd,
                              &used_safe_frame,
                              &authority_rejected);
    if (used_safe_frame != 0u)
    {
        (void)memset(&cmd, 0, sizeof(cmd));
        applied_mode = MotorModeDisable;
    }
    UnitreeMotorUpdateApplied(actuator_id,
                             cfg.rs485_port,
                             node,
                             &cmd,
                             applied_mode,
                             (applied_mode == MotorModeCurrent && can_tx_cmd != NULL) ?
                                  can_tx_cmd->current : 0,
                             ret,
                             authority_rejected);
    UnitreeMotorRefreshFeedback(actuator_id, node);
    return ret;
}

uint8_t UnitreeMotorGetStateCopy(UnitreeMotorState *out)
{
    if (out == NULL)
    {
        return 0u;
    }

    taskENTER_CRITICAL();
    *out = g_unitree_motor_state;
    taskEXIT_CRITICAL();
    return 1u;
}
