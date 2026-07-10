/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "N6014bMotorDriver.h"

#include "FreeRTOS.h"
#include "task.h"

#include "BspUsart.h"
#include "MotorConfig.h"
#include "MotorFeedbackEcdPolicy.h"
#include "MotorInst.h"
#include "UnitreeMotorDriver.h"

#include <string.h>

#define N6014B_TX_FRAME_SIZE 20u
#define N6014B_RX_FRAME_SIZE 26u
#define N6014B_CMD_CRC_LEN 16u
#define N6014B_FBK_CRC_OFFSET 2u
#define N6014B_FBK_CRC_LEN 20u
#define N6014B_HEAD0 0xFEu
#define N6014B_HEAD1 0xEEu
#define N6014B_FBK_HEAD0 0xFCu
#define N6014B_FBK_HEAD1 0xEEu
#define N6014B_PI 3.14159265358979323846f
#define N6014B_TWO_PI (2.0f * N6014B_PI)
#define N6014B_RATIO (38.0f / 3.0f)
#define N6014B_RADPS_TO_RPM 9.54929659f
#define N6014B_ECD_RANGE_F 8191.0f

typedef enum
{
    N6014B_MODE_LOCK = 0u,
    N6014B_MODE_FOC = 1u,
    N6014B_MODE_CALIBRATE = 2u,
} N6014bMode;

typedef struct
{
    uint8_t registered;
    uint8_t ready;
    uint32_t baudrate;
    uint8_t rx_buf[N6014B_RX_FRAME_SIZE];
    volatile uint16_t rx_pos;
    uint32_t rx_parse_error_count;
} N6014bPortState;

typedef struct
{
    uint8_t configured;
    uint8_t actuator_id;
    uint16_t timeout_ms;
    N6014bMotorState state;
} N6014bAxisSlot;

static N6014bAxisSlot g_n6014b_axis[N6014B_MOTOR_MAX_AXIS];
static N6014bPortState g_n6014b_port[2];

static uint8_t N6014bActuatorIdValid(MotorId id)
{
    return ((uint32_t)id < (uint32_t)MotorCount) ? 1u : 0u;
}

static N6014bAxisSlot *N6014bFindAxisSlot(MotorId id)
{
    if (N6014bActuatorIdValid(id) == 0u)
    {
        return NULL;
    }

    for (uint8_t i = 0u; i < (uint8_t)N6014B_MOTOR_MAX_AXIS; i++)
    {
        N6014bAxisSlot *slot = &g_n6014b_axis[i];
        if (slot->configured != 0u && slot->actuator_id == (uint8_t)id)
        {
            return slot;
        }
    }
    return NULL;
}

static N6014bAxisSlot *N6014bAllocAxisSlot(MotorId id)
{
    if (N6014bActuatorIdValid(id) == 0u)
    {
        return NULL;
    }

    for (uint8_t i = 0u; i < (uint8_t)N6014B_MOTOR_MAX_AXIS; i++)
    {
        N6014bAxisSlot *slot = &g_n6014b_axis[i];
        if (slot->configured == 0u)
        {
            (void)memset(slot, 0, sizeof(*slot));
            slot->configured = 1u;
            slot->actuator_id = (uint8_t)id;
            return slot;
        }
    }
    return NULL;
}

static fp32 N6014bClampFp32(fp32 value, fp32 min_value, fp32 max_value)
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

static int16_t N6014bFloatToI16(fp32 value)
{
    if (value > 32767.0f)
    {
        return 32767;
    }
    if (value < -32768.0f)
    {
        return -32768;
    }
    return (int16_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

static int32_t N6014bFloatToI32(fp32 value)
{
    if (value > 2147483647.0f)
    {
        return 2147483647;
    }
    if (value < -2147483648.0f)
    {
        return (-2147483647 - 1);
    }
    return (int32_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

static uint16_t N6014bReadU16Le(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static int16_t N6014bReadI16Le(const uint8_t *p)
{
    return (int16_t)N6014bReadU16Le(p);
}

static uint32_t N6014bReadU32Le(const uint8_t *p)
{
    return ((uint32_t)p[0]) |
           ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

static int32_t N6014bReadI32Le(const uint8_t *p)
{
    return (int32_t)N6014bReadU32Le(p);
}

static void N6014bWriteU16Le(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
}

static void N6014bWriteI16Le(uint8_t *p, int16_t value)
{
    N6014bWriteU16Le(p, (uint16_t)value);
}

static void N6014bWriteU32Le(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
    p[2] = (uint8_t)(value >> 16);
    p[3] = (uint8_t)(value >> 24);
}

static void N6014bWriteI32Le(uint8_t *p, int32_t value)
{
    N6014bWriteU32Le(p, (uint32_t)value);
}

static uint32_t N6014bCrc32WordsLe(const uint8_t *data, uint16_t len)
{
    uint32_t crc = 0xFFFFFFFFu;
    const uint32_t polynomial = 0x04C11DB7u;
    const uint16_t word_count = (uint16_t)(len / 4u);

    if (data == NULL)
    {
        return 0u;
    }

    for (uint16_t i = 0u; i < word_count; i++)
    {
        uint32_t word =
            ((uint32_t)data[i * 4u + 3u] << 24) |
            ((uint32_t)data[i * 4u + 2u] << 16) |
            ((uint32_t)data[i * 4u + 1u] << 8) |
            ((uint32_t)data[i * 4u + 0u]);
        uint32_t xbit = 0x80000000u;

        for (uint8_t bit = 0u; bit < 32u; bit++)
        {
            const uint32_t msb_crc = crc & 0x80000000u;
            crc <<= 1;
            if (msb_crc != 0u)
            {
                crc ^= polynomial;
            }
            if ((word & xbit) != 0u)
            {
                crc ^= polynomial;
            }
            xbit >>= 1;
        }
    }

    return crc;
}

static fp32 N6014bWrap02pi(fp32 angle)
{
    int32_t turns;

    if (angle > N6014B_TWO_PI || angle < -N6014B_TWO_PI)
    {
        turns = (int32_t)(angle / N6014B_TWO_PI);
        angle -= (fp32)turns * N6014B_TWO_PI;
    }
    while (angle < 0.0f)
    {
        angle += N6014B_TWO_PI;
    }
    while (angle >= N6014B_TWO_PI)
    {
        angle -= N6014B_TWO_PI;
    }
    return angle;
}

static uint16_t N6014bPositionToEcd(fp32 position)
{
    const fp32 wrapped = N6014bWrap02pi(position);
    uint32_t ecd = (uint32_t)((wrapped * N6014B_ECD_RANGE_F / N6014B_TWO_PI) + 0.5f);

    if (ecd > 8191u)
    {
        ecd = 8191u;
    }
    return (uint16_t)ecd;
}

static int16_t N6014bTorqueToCurrentLike(const MotorModelMitLimits *limits, fp32 torque)
{
    fp32 scaled;

    if (limits == NULL || limits->torque_max <= 0.0f)
    {
        return 0;
    }

    scaled = torque * 32767.0f / limits->torque_max;
    return N6014bFloatToI16(scaled);
}

static uint8_t N6014bModeByte(uint8_t motor_id, N6014bMode mode, uint8_t timeout)
{
    return (uint8_t)((motor_id & 0x0Fu) |
                     (((uint8_t)mode & 0x07u) << 4) |
                     ((timeout & 0x01u) << 7));
}

static uint8_t N6014bNodeMotorId(const motor_node_param_t *node, uint8_t *motor_id)
{
    uint16_t id;

    if (node == NULL || motor_id == NULL)
    {
        return 0u;
    }

    if (node->feedback_id_enable != 0u)
    {
        id = node->feedback_id;
    }
    else
    {
        if (node->can_id == 0u)
        {
            return 0u;
        }
        id = node->can_id;
    }

    if (id > 15u)
    {
        return 0u;
    }

    *motor_id = (uint8_t)id;
    return 1u;
}

static uint32_t N6014bNodeBaudrate(const motor_node_param_t *node)
{
    if (node == NULL || node->baudrate == 0u)
    {
        return N6014B_MOTOR_DEFAULT_BAUDRATE;
    }
    return node->baudrate;
}

static uint16_t N6014bNodeTimeoutMs(const motor_node_param_t *node)
{
    if (node == NULL || node->rx_timeout_ms == 0u)
    {
        return N6014B_MOTOR_DEFAULT_RX_TIMEOUT_MS;
    }
    return node->rx_timeout_ms;
}

static uint8_t N6014bNodeSupported(const motor_node_param_t *node)
{
    return (uint8_t)(node != NULL &&
                     MotorCfgTransport(node) == MOTOR_TRANSPORT_RS485 &&
                     MotorCfgProtocol(node) == MOTOR_PROTOCOL_N6014B_RS485);
}

static uint8_t N6014bNodeDisabled(const motor_node_param_t *node)
{
    return (uint8_t)(node != NULL &&
                     node->feedback_id_enable == 0u &&
                     node->can_id == 0u);
}

static uint8_t N6014bCmdModeUsesPosition(MotorMode mode)
{
    return (uint8_t)(mode == MotorModeStateTorque ||
                     mode == MotorModePosVel ||
                     mode == MotorModeForcePos);
}

static uint8_t N6014bCmdModeUsesVelocity(MotorMode mode)
{
    return (uint8_t)(mode == MotorModeSpeed ||
                     mode == MotorModeDamping);
}

static uint8_t N6014bDriveState(uint8_t online, uint8_t mode)
{
    if (online == 0u)
    {
        return (uint8_t)MotorDriveStateOffline;
    }
    if (mode == (uint8_t)N6014B_MODE_LOCK)
    {
        return (uint8_t)MotorDriveStateDisabled;
    }
    if (mode == (uint8_t)N6014B_MODE_FOC)
    {
        return (uint8_t)MotorDriveStateEnabled;
    }
    return (uint8_t)MotorDriveStateReady;
}

static fp32 N6014bCurrentToTorque(const motor_node_param_t *node,
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
    return N6014bClampFp32(torque, -limits->torque_max, limits->torque_max);
}

static uint8_t N6014bBuildCmdFromActuator(const motor_node_param_t *node,
                                               int16_t current,
                                               const MotorCmd *cmd,
                                               N6014bMode *mode,
                                              fp32 *position,
                                              fp32 *velocity,
                                              fp32 *kp,
                                              fp32 *kd,
                                              fp32 *torque)
{
    const MotorModelMitLimits *limits = MotorCfgMitLimits(node);
    uint8_t active_cmd;
    MotorMode cmd_mode = MotorModeCurrent;

    if (mode == NULL || position == NULL || velocity == NULL ||
        kp == NULL || kd == NULL || torque == NULL)
    {
        return 0u;
    }

    *mode = N6014B_MODE_LOCK;
    *position = 0.0f;
    *velocity = 0.0f;
    *kp = 0.0f;
    *kd = 0.0f;
    *torque = 0.0f;

    if (limits == NULL)
    {
        return 0u;
    }

    active_cmd = (uint8_t)(cmd != NULL &&
                           cmd->active != 0u &&
                           cmd->mode != (uint8_t)MotorModeNone);

    if (active_cmd != 0u)
    {
        cmd_mode = (MotorMode)cmd->mode;
    }
    if (cmd_mode == MotorModeDisable)
    {
        return 1u;
    }
    if (active_cmd == 0u && current == 0)
    {
        return 1u;
    }

    *mode = N6014B_MODE_FOC;

    if (active_cmd != 0u && N6014bCmdModeUsesPosition(cmd_mode) != 0u)
    {
        *position = cmd->q;
        *velocity = cmd->dq;
        *kp = cmd->kp;
        *kd = cmd->kd;
        *torque = cmd->tau;
    }
    else if (active_cmd != 0u && N6014bCmdModeUsesVelocity(cmd_mode) != 0u)
    {
        *velocity = cmd->dq;
        *kd = cmd->kd;
        *torque = cmd->tau;
    }
    else
    {
        *torque = N6014bCurrentToTorque(node, current, limits);
    }

    *position = N6014bClampFp32(*position, -limits->position_max, limits->position_max);
    *velocity = N6014bClampFp32(*velocity, -limits->velocity_max, limits->velocity_max);
    *kp = N6014bClampFp32(*kp, 0.0f, limits->kp_max);
    *kd = N6014bClampFp32(*kd, 0.0f, limits->kd_max);
    *torque = N6014bClampFp32(*torque, -limits->torque_max, limits->torque_max);
    return 1u;
}

static void N6014bBuildTxFrame(uint8_t frame[N6014B_TX_FRAME_SIZE],
                                  uint8_t motor_id,
                                  N6014bMode mode,
                                  fp32 position,
                                  fp32 velocity,
                                  fp32 kp,
                                  fp32 kd,
                                  fp32 torque)
{
    int16_t torque_raw;
    int16_t speed_raw;
    int32_t position_raw;
    int16_t kp_raw;
    int16_t kd_raw;
    uint32_t crc;

    (void)memset(frame, 0, N6014B_TX_FRAME_SIZE);
    frame[0] = N6014B_HEAD0;
    frame[1] = N6014B_HEAD1;
    frame[2] = N6014bModeByte(motor_id, mode, 1u);
    frame[3] = 0u;

    kp_raw = N6014bFloatToI16(kp / (N6014B_RATIO * N6014B_RATIO) * 12800.0f);
    kd_raw = N6014bFloatToI16(kd / (N6014B_RATIO * N6014B_RATIO) * 51200.0f);
    position_raw = N6014bFloatToI32(position * N6014B_RATIO * 32768.0f / N6014B_TWO_PI);
    speed_raw = N6014bFloatToI16(velocity * N6014B_RATIO * 64.0f / N6014B_TWO_PI);
    torque_raw = N6014bFloatToI16(torque / N6014B_RATIO * 2560.0f);

    N6014bWriteI16Le(&frame[4], torque_raw);
    N6014bWriteI16Le(&frame[6], speed_raw);
    N6014bWriteI32Le(&frame[8], position_raw);
    N6014bWriteI16Le(&frame[12], kp_raw);
    N6014bWriteI16Le(&frame[14], kd_raw);

    crc = N6014bCrc32WordsLe(frame, N6014B_CMD_CRC_LEN);
    N6014bWriteU32Le(&frame[16], crc);
}

static void N6014bCopyStateFromIsr(MotorId id,
                                       uint8_t port,
                                       uint8_t mode,
                                       uint8_t timeout,
                                       int8_t temp,
                                       uint8_t coil_temp,
                                       fp32 voltage,
                                       uint32_t motor_error,
                                       uint16_t motor_warn,
                                       fp32 torque,
                                       fp32 velocity,
                                       fp32 position)
{
    N6014bAxisSlot *slot;
    N6014bMotorState *state;
    UBaseType_t saved;

    if (N6014bActuatorIdValid(id) == 0u)
    {
        return;
    }

    slot = N6014bFindAxisSlot(id);
    if (slot == NULL)
    {
        return;
    }

    state = &slot->state;
    saved = taskENTER_CRITICAL_FROM_ISR();
    state->enabled = 1u;
    state->online = 1u;
    state->rs485_port = port;
    state->mode = mode;
    state->timeout = timeout;
    state->motor_temp = temp;
    state->coil_temp = coil_temp;
    state->voltage = voltage;
    state->motor_error = motor_error;
    state->motor_warn = motor_warn;
    state->torque_nm = torque;
    state->speed_rad_s = velocity;
    state->position_rad = position;
    state->rx_frame_count = MotorFeedbackRxCountNext(state->rx_frame_count);
    state->last_rx_tick_ms = (uint32_t)(xTaskGetTickCountFromISR() * portTICK_PERIOD_MS);
    taskEXIT_CRITICAL_FROM_ISR(saved);
}

static MotorId N6014bFindAxisFromIsr(uint8_t port, uint8_t motor_id)
{
    for (uint8_t i = 0u; i < (uint8_t)N6014B_MOTOR_MAX_AXIS; i++)
    {
        const N6014bAxisSlot *slot = &g_n6014b_axis[i];
        if (slot->configured != 0u &&
            slot->state.rs485_port == port &&
            slot->state.motor_id == motor_id)
        {
            return (MotorId)slot->actuator_id;
        }
    }
    return MotorCount;
}

static void N6014bMarkAxisCrcErrorFromIsr(MotorId id)
{
    N6014bAxisSlot *slot;
    UBaseType_t saved;

    if (N6014bActuatorIdValid(id) == 0u)
    {
        return;
    }

    slot = N6014bFindAxisSlot(id);
    if (slot == NULL)
    {
        return;
    }

    saved = taskENTER_CRITICAL_FROM_ISR();
    slot->state.rx_crc_fail_count++;
    taskEXIT_CRITICAL_FROM_ISR(saved);
}

static void N6014bProcessRxFrameFromIsr(uint8_t port, const uint8_t *frame)
{
    uint8_t mode_byte;
    uint8_t motor_id;
    uint8_t mode;
    uint8_t timeout;
    uint32_t crc_expect;
    uint32_t crc_actual;
    MotorId axis;
    fp32 torque;
    fp32 velocity;
    fp32 position;

    if (frame == NULL)
    {
        return;
    }

    if (frame[0] != N6014B_FBK_HEAD0 || frame[1] != N6014B_FBK_HEAD1)
    {
        g_n6014b_port[port].rx_parse_error_count++;
        return;
    }

    mode_byte = frame[2];
    motor_id = (uint8_t)(mode_byte & 0x0Fu);
    axis = N6014bFindAxisFromIsr(port, motor_id);

    crc_expect = N6014bReadU32Le(&frame[22]);
    crc_actual = N6014bCrc32WordsLe(&frame[N6014B_FBK_CRC_OFFSET], N6014B_FBK_CRC_LEN);
    if (crc_expect != crc_actual)
    {
        N6014bMarkAxisCrcErrorFromIsr(axis);
        return;
    }

    if (N6014bActuatorIdValid(axis) == 0u)
    {
        g_n6014b_port[port].rx_parse_error_count++;
        return;
    }

    mode = (uint8_t)((mode_byte >> 4) & 0x07u);
    timeout = (uint8_t)((mode_byte >> 7) & 0x01u);
    torque = ((fp32)N6014bReadI16Le(&frame[6])) / 2560.0f * N6014B_RATIO;
    velocity = ((fp32)N6014bReadI16Le(&frame[8])) / 64.0f * N6014B_TWO_PI / N6014B_RATIO;
    position = N6014B_TWO_PI * ((fp32)N6014bReadI32Le(&frame[10])) / 32768.0f / N6014B_RATIO;

    N6014bCopyStateFromIsr(axis,
                               port,
                               mode,
                               timeout,
                               (int8_t)frame[3],
                               frame[4],
                               ((fp32)frame[5]) / 2.0f,
                               N6014bReadU32Le(&frame[14]),
                               (uint16_t)((N6014bReadU16Le(&frame[18]) >> 13) & 0x07u),
                               torque,
                               velocity,
                               position);
}

static void N6014bIngestRxByte(uint8_t port, uint8_t b)
{
    N6014bPortState *p;
    uint16_t pos;

    if (port > N6014B_MOTOR_RS485_PORT1)
    {
        return;
    }

    p = &g_n6014b_port[port];
    pos = p->rx_pos;

    if (pos == 0u)
    {
        if (b == N6014B_FBK_HEAD0)
        {
            p->rx_buf[0] = b;
            p->rx_pos = 1u;
        }
        return;
    }

    if (pos == 1u)
    {
        if (b == N6014B_FBK_HEAD1)
        {
            p->rx_buf[1] = b;
            p->rx_pos = 2u;
        }
        else
        {
            p->rx_pos = (b == N6014B_FBK_HEAD0) ? 1u : 0u;
            p->rx_buf[0] = b;
        }
        return;
    }

    if (pos >= N6014B_RX_FRAME_SIZE)
    {
        p->rx_pos = 0u;
        return;
    }

    p->rx_buf[pos] = b;
    pos++;

    if (pos >= N6014B_RX_FRAME_SIZE)
    {
        p->rx_pos = 0u;
        N6014bProcessRxFrameFromIsr(port, p->rx_buf);
        return;
    }

    p->rx_pos = pos;
}

static void N6014bUsart2RxByte(uint8_t b)
{
    N6014bIngestRxByte(N6014B_MOTOR_RS485_PORT0, b);
}

static void N6014bUsart3RxByte(uint8_t b)
{
    N6014bIngestRxByte(N6014B_MOTOR_RS485_PORT1, b);
}

static uint8_t N6014bUsart2Error(void)
{
    g_n6014b_port[N6014B_MOTOR_RS485_PORT0].rx_parse_error_count++;
    return 0u;
}

static uint8_t N6014bUsart3Error(void)
{
    g_n6014b_port[N6014B_MOTOR_RS485_PORT1].rx_parse_error_count++;
    return 0u;
}

static uint8_t N6014bSetupPort(uint8_t port, uint32_t baudrate)
{
    int ret = 1;
    N6014bPortState *p;

    if (port > N6014B_MOTOR_RS485_PORT1)
    {
        return 0u;
    }

    p = &g_n6014b_port[port];
    if (p->ready != 0u && p->baudrate == baudrate)
    {
        return 1u;
    }

    if (p->registered == 0u)
    {
        if (port == N6014B_MOTOR_RS485_PORT0)
        {
            BspUsart2SetRxByteCb(N6014bUsart2RxByte);
            BspUsart2SetErrorCb(N6014bUsart2Error);
        }
        else
        {
            BspUsart3SetRxByteCb(N6014bUsart3RxByte);
            BspUsart3SetErrorCb(N6014bUsart3Error);
        }
        p->registered = 1u;
    }

    p->rx_pos = 0u;
    if (port == N6014B_MOTOR_RS485_PORT0)
    {
        ret = BspUsart2SetBaudrate(baudrate);
        if (ret == 0)
        {
            ret = BspUsart2RxItStart();
        }
    }
    else
    {
        ret = BspUsart3SetBaudrate(baudrate);
        if (ret == 0)
        {
            ret = BspUsart3RxItStart();
        }
    }

    if (ret != 0)
    {
        p->ready = 0u;
        return 0u;
    }

    p->baudrate = baudrate;
    p->ready = 1u;
    return 1u;
}

static int N6014bTx(uint8_t port, const uint8_t frame[N6014B_TX_FRAME_SIZE])
{
    if (frame == NULL)
    {
        return 1;
    }

    if (port == N6014B_MOTOR_RS485_PORT0)
    {
        return BspUsart2Tx(frame, N6014B_TX_FRAME_SIZE, 5u);
    }
    if (port == N6014B_MOTOR_RS485_PORT1)
    {
        return BspUsart3Tx(frame, N6014B_TX_FRAME_SIZE, 5u);
    }
    return 1;
}

static uint8_t N6014bUpdateAxisConfig(MotorId id,
                                         uint8_t port,
                                         uint8_t motor_id,
                                         uint16_t timeout_ms)
{
    N6014bAxisSlot *slot;
    N6014bMotorState *state;

    if (N6014bActuatorIdValid(id) == 0u)
    {
        return 0u;
    }

    taskENTER_CRITICAL();
    slot = N6014bFindAxisSlot(id);
    if (slot == NULL)
    {
        slot = N6014bAllocAxisSlot(id);
    }
    if (slot == NULL)
    {
        taskEXIT_CRITICAL();
        return 0u;
    }

    state = &slot->state;
    state->enabled = 1u;
    state->rs485_port = port;
    state->motor_id = motor_id;
    slot->timeout_ms = timeout_ms;
    taskEXIT_CRITICAL();
    return 1u;
}

static void N6014bRecordTxResult(MotorId id, int ret)
{
    N6014bAxisSlot *slot;

    if (N6014bActuatorIdValid(id) == 0u)
    {
        return;
    }

    slot = N6014bFindAxisSlot(id);
    if (slot == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    slot->state.tx_count++;
    slot->state.last_tx_status = (uint8_t)ret;
    if (ret != 0)
    {
        slot->state.tx_fail_count++;
    }
    taskEXIT_CRITICAL();
}

static void N6014bRefreshFeedback(MotorId id, const motor_node_param_t *node)
{
    const MotorModelMitLimits *limits = MotorCfgMitLimits(node);
    N6014bAxisSlot *slot;
    N6014bMotorState state;
    MotorState previous;
    MotorState fb;
    const MotorState *previous_feedback = NULL;
    motor_measure_t *measure;
    uint16_t timeout_ms;
    uint32_t now_ms;
    uint8_t new_sample;

    if (N6014bActuatorIdValid(id) == 0u)
    {
        return;
    }

    slot = N6014bFindAxisSlot(id);
    if (slot == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    state = slot->state;
    timeout_ms = slot->timeout_ms;
    taskEXIT_CRITICAL();

    if (timeout_ms == 0u)
    {
        timeout_ms = N6014B_MOTOR_DEFAULT_RX_TIMEOUT_MS;
    }

    now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    if (state.last_rx_tick_ms == 0u || (uint32_t)(now_ms - state.last_rx_tick_ms) > timeout_ms)
    {
        state.online = 0u;
        taskENTER_CRITICAL();
        slot->state.online = 0u;
        taskEXIT_CRITICAL();
    }

    if (LowStateGetMotor(id, &previous) != 0u)
    {
        previous_feedback = &previous;
    }
    (void)memset(&fb, 0, sizeof(fb));
    fb.online = state.online;
    fb.bus = state.rs485_port;
    fb.rxDlc = N6014B_RX_FRAME_SIZE;
    fb.transport = (uint8_t)MotorTransportRS485;
    fb.motorId = state.motor_id;
    fb.state = state.mode;
    fb.driveState = N6014bDriveState(state.online, state.mode);
    fb.rxId = state.motor_id;
    fb.rxCount = state.rx_frame_count;
    fb.lastRxTick = state.last_rx_tick_ms;
    fb.q = state.position_rad;
    fb.dq = state.speed_rad_s;
    fb.tauEst = state.torque_nm;
    fb.ecd = N6014bPositionToEcd(state.position_rad);
    new_sample = MotorFeedbackEcdResolve(previous_feedback,
                                         fb.rxCount,
                                         fb.ecd,
                                         &fb.lastEcd);
    fb.speedRpm = N6014bFloatToI16(state.speed_rad_s * N6014B_RADPS_TO_RPM);
    fb.current = N6014bTorqueToCurrentLike(limits, state.torque_nm);
    fb.temperature = (uint8_t)state.motor_temp;
    LowStateUpdateMotor(id, &fb);

    measure = MotorInstMeasure(id);
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

static void N6014bUpdateApplied(MotorId id,
                                  uint8_t port,
                                  uint8_t motor_id,
                                  const motor_node_param_t *node,
                                  N6014bMode mode,
                                  fp32 position,
                                  fp32 velocity,
                                   fp32 kp,
                                   fp32 kd,
                                   fp32 torque,
                                   const MotorCmd *cmd,
                                   int16_t current,
                                   int ret)
{
    MotorApplied applied;

    if (N6014bActuatorIdValid(id) == 0u)
    {
        return;
    }

    (void)memset(&applied, 0, sizeof(applied));
    applied.active = 1u;
    applied.mode = (uint8_t)MotorModeCurrent;
    applied.driveState = (mode == N6014B_MODE_LOCK) ?
                             (uint8_t)MotorDriveStateDisabled :
                             (uint8_t)MotorDriveStateEnabled;
    applied.bus = port;
    applied.transport = (uint8_t)MotorTransportRS485;
    applied.protocol = (uint8_t)MotorCfgProtocol(node);
    applied.txId = motor_id;
    applied.tick = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    applied.current = current;
    applied.q = position;
    applied.dq = velocity;
    applied.kp = kp;
    applied.kd = kd;
    applied.tau = torque;
    if (mode == N6014B_MODE_LOCK)
    {
        applied.mode = (uint8_t)MotorModeDisable;
    }
    else if (cmd != NULL && cmd->active != 0u && cmd->mode != (uint8_t)MotorModeNone)
    {
        applied.mode = cmd->mode;
    }
    if (ret != 0)
    {
        applied.flags |= (uint8_t)MotorAppliedFlagSkipped;
    }

    LowStateUpdateApplied(id, &applied);
}

void N6014bMotorDriverInit(void)
{
    taskENTER_CRITICAL();
    (void)memset(g_n6014b_axis, 0, sizeof(g_n6014b_axis));
    (void)memset(g_n6014b_port, 0, sizeof(g_n6014b_port));
    taskEXIT_CRITICAL();
}

int N6014bMotorSendActuator(uint8_t port,
                               MotorId actuator_id,
                               const motor_node_param_t *node,
                               int16_t current,
                               const MotorCmd *cmd)
{
    uint8_t motor_id;
    uint8_t frame[N6014B_TX_FRAME_SIZE];
    N6014bMode mode;
    fp32 position;
    fp32 velocity;
    fp32 kp;
    fp32 kd;
    fp32 torque;
    const uint32_t baudrate = N6014bNodeBaudrate(node);
    const uint16_t timeout_ms = N6014bNodeTimeoutMs(node);
    int ret;

    if (N6014bNodeSupported(node) == 0u)
    {
        return 1;
    }
    if (N6014bNodeDisabled(node) != 0u)
    {
        return 0;
    }
    if (N6014bNodeMotorId(node, &motor_id) == 0u)
    {
        return 1;
    }
    if (port > N6014B_MOTOR_RS485_PORT1)
    {
        return 1;
    }

    if (N6014bUpdateAxisConfig(actuator_id, port, motor_id, timeout_ms) == 0u)
    {
        return 1;
    }
    if (N6014bSetupPort(port, baudrate) == 0u)
    {
        N6014bRecordTxResult(actuator_id, 1);
        N6014bRefreshFeedback(actuator_id, node);
        return 1;
    }

    if (N6014bBuildCmdFromActuator(node,
                                       current,
                                       cmd,
                                       &mode,
                                       &position,
                                       &velocity,
                                       &kp,
                                       &kd,
                                       &torque) == 0u)
    {
        N6014bRefreshFeedback(actuator_id, node);
        return 1;
    }

    N6014bBuildTxFrame(frame, motor_id, mode, position, velocity, kp, kd, torque);
    ret = N6014bTx(port, frame);
    N6014bUpdateApplied(actuator_id,
                       port,
                       motor_id,
                       node,
                       mode,
                       position,
                       velocity,
                       kp,
                       kd,
                       torque,
                       cmd,
                       current,
                       ret);
    N6014bRecordTxResult(actuator_id, ret);
    N6014bRefreshFeedback(actuator_id, node);
    return ret;
}

const N6014bMotorState *N6014bMotorGetState(MotorId actuator_id)
{
    const N6014bAxisSlot *slot;

    if (N6014bActuatorIdValid(actuator_id) == 0u)
    {
        return 0;
    }

    slot = N6014bFindAxisSlot(actuator_id);
    return (slot != NULL) ? &slot->state : 0;
}

uint8_t CanTxProcessExtraItem(uint8_t bus,
                                  MotorId actuator_id,
                                  const motor_node_param_t *node,
                                  int16_t current,
                                  const MotorCmd *cmd)
{
    if (UnitreeMotorNodeSupported(node) != 0u)
    {
        return (UnitreeMotorSendActuator(bus, actuator_id, node, current, cmd) == 0) ? 1u : 0u;
    }

    if (N6014bNodeSupported(node) == 0u)
    {
        return 0u;
    }

    return (N6014bMotorSendActuator(bus, actuator_id, node, current, cmd) == 0) ? 1u : 0u;
}
