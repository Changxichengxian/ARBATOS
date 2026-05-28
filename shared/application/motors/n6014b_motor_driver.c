/*
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 */

#include "n6014b_motor_driver.h"

#include "FreeRTOS.h"
#include "task.h"

#include "bsp_usart.h"
#include "motor_config.h"
#include "motor_instance.h"

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
} n6014b_mode_e;

typedef struct
{
    uint8_t registered;
    uint8_t ready;
    uint32_t baudrate;
    uint8_t rx_buf[N6014B_RX_FRAME_SIZE];
    volatile uint16_t rx_pos;
    uint32_t rx_parse_error_count;
} n6014b_port_state_t;

static n6014b_motor_state_t g_n6014b_state[ACTUATOR_ID__COUNT];
static uint8_t g_n6014b_axis_configured[ACTUATOR_ID__COUNT];
static uint16_t g_n6014b_axis_timeout_ms[ACTUATOR_ID__COUNT];
static n6014b_port_state_t g_n6014b_port[2];

static uint8_t n6014b_actuator_id_valid(actuator_id_e id)
{
    return ((uint32_t)id < (uint32_t)ACTUATOR_ID__COUNT) ? 1u : 0u;
}

static fp32 n6014b_clamp_fp32(fp32 value, fp32 min_value, fp32 max_value)
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

static int16_t n6014b_float_to_i16(fp32 value)
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

static int32_t n6014b_float_to_i32(fp32 value)
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

static uint16_t n6014b_read_u16_le(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static int16_t n6014b_read_i16_le(const uint8_t *p)
{
    return (int16_t)n6014b_read_u16_le(p);
}

static uint32_t n6014b_read_u32_le(const uint8_t *p)
{
    return ((uint32_t)p[0]) |
           ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

static int32_t n6014b_read_i32_le(const uint8_t *p)
{
    return (int32_t)n6014b_read_u32_le(p);
}

static void n6014b_write_u16_le(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
}

static void n6014b_write_i16_le(uint8_t *p, int16_t value)
{
    n6014b_write_u16_le(p, (uint16_t)value);
}

static void n6014b_write_u32_le(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
    p[2] = (uint8_t)(value >> 16);
    p[3] = (uint8_t)(value >> 24);
}

static void n6014b_write_i32_le(uint8_t *p, int32_t value)
{
    n6014b_write_u32_le(p, (uint32_t)value);
}

static uint32_t n6014b_crc32_words_le(const uint8_t *data, uint16_t len)
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

static fp32 n6014b_wrap_0_2pi(fp32 angle)
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

static uint16_t n6014b_position_to_ecd(fp32 position)
{
    const fp32 wrapped = n6014b_wrap_0_2pi(position);
    uint32_t ecd = (uint32_t)((wrapped * N6014B_ECD_RANGE_F / N6014B_TWO_PI) + 0.5f);

    if (ecd > 8191u)
    {
        ecd = 8191u;
    }
    return (uint16_t)ecd;
}

static int16_t n6014b_torque_to_current_like(const motor_model_mit_limits_t *limits, fp32 torque)
{
    fp32 scaled;

    if (limits == NULL || limits->torque_max <= 0.0f)
    {
        return 0;
    }

    scaled = torque * 32767.0f / limits->torque_max;
    return n6014b_float_to_i16(scaled);
}

static uint8_t n6014b_mode_byte(uint8_t motor_id, n6014b_mode_e mode, uint8_t timeout)
{
    return (uint8_t)((motor_id & 0x0Fu) |
                     (((uint8_t)mode & 0x07u) << 4) |
                     ((timeout & 0x01u) << 7));
}

static uint8_t n6014b_node_motor_id(const motor_node_param_t *node, uint8_t *motor_id)
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

static uint32_t n6014b_node_baudrate(const motor_node_param_t *node)
{
    if (node == NULL || node->baudrate == 0u)
    {
        return N6014B_MOTOR_DEFAULT_BAUDRATE;
    }
    return node->baudrate;
}

static uint16_t n6014b_node_timeout_ms(const motor_node_param_t *node)
{
    if (node == NULL || node->rx_timeout_ms == 0u)
    {
        return N6014B_MOTOR_DEFAULT_RX_TIMEOUT_MS;
    }
    return node->rx_timeout_ms;
}

static uint8_t n6014b_node_supported(const motor_node_param_t *node)
{
    return (uint8_t)(node != NULL &&
                     motor_cfg_transport(node) == MOTOR_TRANSPORT_RS485 &&
                     motor_cfg_protocol(node) == MOTOR_PROTOCOL_N6014B_RS485);
}

static uint8_t n6014b_node_disabled(const motor_node_param_t *node)
{
    return (uint8_t)(node != NULL &&
                     node->feedback_id_enable == 0u &&
                     node->can_id == 0u);
}

static uint8_t n6014b_cmd_mode_uses_position(actuator_cmd_mode_e mode)
{
    return (uint8_t)(mode == ACTUATOR_CMD_MODE_STATE_TORQUE ||
                     mode == ACTUATOR_CMD_MODE_POS_VEL ||
                     mode == ACTUATOR_CMD_MODE_FORCE_POS);
}

static uint8_t n6014b_cmd_mode_uses_velocity(actuator_cmd_mode_e mode)
{
    return (uint8_t)(mode == ACTUATOR_CMD_MODE_SPEED);
}

static fp32 n6014b_current_to_torque(const motor_node_param_t *node,
                                     int16_t current,
                                     const motor_model_mit_limits_t *limits)
{
    const motor_model_db_entry_t *entry;
    int16_t range_abs = 32767;
    fp32 torque;

    if (node == NULL || limits == NULL || limits->torque_max <= 0.0f)
    {
        return 0.0f;
    }

    entry = motor_cfg_model_db(node->model);
    if (entry != NULL && entry->cmd_current_range_abs > 0)
    {
        range_abs = entry->cmd_current_range_abs;
    }

    torque = ((fp32)current) * limits->torque_max / (fp32)range_abs;
    return n6014b_clamp_fp32(torque, -limits->torque_max, limits->torque_max);
}

static uint8_t n6014b_build_cmd_from_actuator(const motor_node_param_t *node,
                                              actuator_id_e actuator_id,
                                              int16_t current,
                                              n6014b_mode_e *mode,
                                              fp32 *position,
                                              fp32 *velocity,
                                              fp32 *kp,
                                              fp32 *kd,
                                              fp32 *torque)
{
    const motor_model_mit_limits_t *limits = motor_cfg_mit_limits(node);
    actuator_cmd_t src;
    uint8_t have_cmd;
    uint8_t active_cmd;
    actuator_cmd_mode_e cmd_mode = ACTUATOR_CMD_MODE_CURRENT;

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

    (void)memset(&src, 0, sizeof(src));
    have_cmd = actuator_cmd_get_copy(actuator_id, &src);
    active_cmd = (uint8_t)(have_cmd != 0u &&
                           src.active != 0u &&
                           src.mode != (uint8_t)ACTUATOR_CMD_MODE_NONE);

    if (active_cmd == 0u && current == 0)
    {
        return 1u;
    }

    *mode = N6014B_MODE_FOC;
    if (active_cmd != 0u)
    {
        cmd_mode = (actuator_cmd_mode_e)src.mode;
    }

    if (active_cmd != 0u && n6014b_cmd_mode_uses_position(cmd_mode) != 0u)
    {
        *position = src.position;
        *velocity = src.velocity;
        *kp = src.kp;
        *kd = src.kd;
        *torque = src.torque;
    }
    else if (active_cmd != 0u && n6014b_cmd_mode_uses_velocity(cmd_mode) != 0u)
    {
        *velocity = src.velocity;
        *kd = src.kd;
        *torque = src.torque;
    }
    else
    {
        *torque = n6014b_current_to_torque(node, current, limits);
    }

    *position = n6014b_clamp_fp32(*position, -limits->position_max, limits->position_max);
    *velocity = n6014b_clamp_fp32(*velocity, -limits->velocity_max, limits->velocity_max);
    *kp = n6014b_clamp_fp32(*kp, 0.0f, limits->kp_max);
    *kd = n6014b_clamp_fp32(*kd, 0.0f, limits->kd_max);
    *torque = n6014b_clamp_fp32(*torque, -limits->torque_max, limits->torque_max);
    return 1u;
}

static void n6014b_build_tx_frame(uint8_t frame[N6014B_TX_FRAME_SIZE],
                                  uint8_t motor_id,
                                  n6014b_mode_e mode,
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
    frame[2] = n6014b_mode_byte(motor_id, mode, 1u);
    frame[3] = 0u;

    kp_raw = n6014b_float_to_i16(kp / (N6014B_RATIO * N6014B_RATIO) * 12800.0f);
    kd_raw = n6014b_float_to_i16(kd / (N6014B_RATIO * N6014B_RATIO) * 51200.0f);
    position_raw = n6014b_float_to_i32(position * N6014B_RATIO * 32768.0f / N6014B_TWO_PI);
    speed_raw = n6014b_float_to_i16(velocity * N6014B_RATIO * 64.0f / N6014B_TWO_PI);
    torque_raw = n6014b_float_to_i16(torque / N6014B_RATIO * 2560.0f);

    n6014b_write_i16_le(&frame[4], torque_raw);
    n6014b_write_i16_le(&frame[6], speed_raw);
    n6014b_write_i32_le(&frame[8], position_raw);
    n6014b_write_i16_le(&frame[12], kp_raw);
    n6014b_write_i16_le(&frame[14], kd_raw);

    crc = n6014b_crc32_words_le(frame, N6014B_CMD_CRC_LEN);
    n6014b_write_u32_le(&frame[16], crc);
}

static void n6014b_copy_state_from_isr(actuator_id_e id,
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
    n6014b_motor_state_t *state;
    UBaseType_t saved;

    if (n6014b_actuator_id_valid(id) == 0u)
    {
        return;
    }

    state = &g_n6014b_state[id];
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
    state->rx_frame_count++;
    state->last_rx_tick_ms = (uint32_t)(xTaskGetTickCountFromISR() * portTICK_PERIOD_MS);
    taskEXIT_CRITICAL_FROM_ISR(saved);
}

static actuator_id_e n6014b_find_axis_from_isr(uint8_t port, uint8_t motor_id)
{
    for (uint8_t i = 0u; i < (uint8_t)ACTUATOR_ID__COUNT; i++)
    {
        if (g_n6014b_axis_configured[i] != 0u &&
            g_n6014b_state[i].rs485_port == port &&
            g_n6014b_state[i].motor_id == motor_id)
        {
            return (actuator_id_e)i;
        }
    }
    return ACTUATOR_ID__COUNT;
}

static void n6014b_mark_axis_crc_error_from_isr(actuator_id_e id)
{
    UBaseType_t saved;

    if (n6014b_actuator_id_valid(id) == 0u)
    {
        return;
    }

    saved = taskENTER_CRITICAL_FROM_ISR();
    g_n6014b_state[id].rx_crc_fail_count++;
    taskEXIT_CRITICAL_FROM_ISR(saved);
}

static void n6014b_process_rx_frame_from_isr(uint8_t port, const uint8_t *frame)
{
    uint8_t mode_byte;
    uint8_t motor_id;
    uint8_t mode;
    uint8_t timeout;
    uint32_t crc_expect;
    uint32_t crc_actual;
    actuator_id_e axis;
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
    axis = n6014b_find_axis_from_isr(port, motor_id);

    crc_expect = n6014b_read_u32_le(&frame[22]);
    crc_actual = n6014b_crc32_words_le(&frame[N6014B_FBK_CRC_OFFSET], N6014B_FBK_CRC_LEN);
    if (crc_expect != crc_actual)
    {
        n6014b_mark_axis_crc_error_from_isr(axis);
        return;
    }

    if (n6014b_actuator_id_valid(axis) == 0u)
    {
        g_n6014b_port[port].rx_parse_error_count++;
        return;
    }

    mode = (uint8_t)((mode_byte >> 4) & 0x07u);
    timeout = (uint8_t)((mode_byte >> 7) & 0x01u);
    torque = ((fp32)n6014b_read_i16_le(&frame[6])) / 2560.0f * N6014B_RATIO;
    velocity = ((fp32)n6014b_read_i16_le(&frame[8])) / 64.0f * N6014B_TWO_PI / N6014B_RATIO;
    position = N6014B_TWO_PI * ((fp32)n6014b_read_i32_le(&frame[10])) / 32768.0f / N6014B_RATIO;

    n6014b_copy_state_from_isr(axis,
                               port,
                               mode,
                               timeout,
                               (int8_t)frame[3],
                               frame[4],
                               ((fp32)frame[5]) / 2.0f,
                               n6014b_read_u32_le(&frame[14]),
                               (uint16_t)((n6014b_read_u16_le(&frame[18]) >> 13) & 0x07u),
                               torque,
                               velocity,
                               position);
}

static void n6014b_ingest_rx_byte(uint8_t port, uint8_t b)
{
    n6014b_port_state_t *p;
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
        n6014b_process_rx_frame_from_isr(port, p->rx_buf);
        return;
    }

    p->rx_pos = pos;
}

static void n6014b_usart2_rx_byte(uint8_t b)
{
    n6014b_ingest_rx_byte(N6014B_MOTOR_RS485_PORT0, b);
}

static void n6014b_usart3_rx_byte(uint8_t b)
{
    n6014b_ingest_rx_byte(N6014B_MOTOR_RS485_PORT1, b);
}

static uint8_t n6014b_usart2_error(void)
{
    g_n6014b_port[N6014B_MOTOR_RS485_PORT0].rx_parse_error_count++;
    return 0u;
}

static uint8_t n6014b_usart3_error(void)
{
    g_n6014b_port[N6014B_MOTOR_RS485_PORT1].rx_parse_error_count++;
    return 0u;
}

static uint8_t n6014b_setup_port(uint8_t port, uint32_t baudrate)
{
    int ret = 1;
    n6014b_port_state_t *p;

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
            bsp_usart2_set_rx_byte_cb(n6014b_usart2_rx_byte);
            bsp_usart2_set_error_cb(n6014b_usart2_error);
        }
        else
        {
            bsp_usart3_set_rx_byte_cb(n6014b_usart3_rx_byte);
            bsp_usart3_set_error_cb(n6014b_usart3_error);
        }
        p->registered = 1u;
    }

    p->rx_pos = 0u;
    if (port == N6014B_MOTOR_RS485_PORT0)
    {
        ret = bsp_usart2_set_baudrate(baudrate);
        if (ret == 0)
        {
            ret = bsp_usart2_rx_it_start();
        }
    }
    else
    {
        ret = bsp_usart3_set_baudrate(baudrate);
        if (ret == 0)
        {
            ret = bsp_usart3_rx_it_start();
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

static int n6014b_tx(uint8_t port, const uint8_t frame[N6014B_TX_FRAME_SIZE])
{
    if (frame == NULL)
    {
        return 1;
    }

    if (port == N6014B_MOTOR_RS485_PORT0)
    {
        return bsp_usart2_tx(frame, N6014B_TX_FRAME_SIZE, 5u);
    }
    if (port == N6014B_MOTOR_RS485_PORT1)
    {
        return bsp_usart3_tx(frame, N6014B_TX_FRAME_SIZE, 5u);
    }
    return 1;
}

static void n6014b_update_axis_config(actuator_id_e id,
                                      uint8_t port,
                                      uint8_t motor_id,
                                      uint16_t timeout_ms)
{
    n6014b_motor_state_t *state;

    if (n6014b_actuator_id_valid(id) == 0u)
    {
        return;
    }

    taskENTER_CRITICAL();
    state = &g_n6014b_state[id];
    state->enabled = 1u;
    state->rs485_port = port;
    state->motor_id = motor_id;
    g_n6014b_axis_configured[id] = 1u;
    g_n6014b_axis_timeout_ms[id] = timeout_ms;
    taskEXIT_CRITICAL();
}

static void n6014b_record_tx_result(actuator_id_e id, int ret)
{
    if (n6014b_actuator_id_valid(id) == 0u)
    {
        return;
    }

    taskENTER_CRITICAL();
    g_n6014b_state[id].tx_count++;
    g_n6014b_state[id].last_tx_status = (uint8_t)ret;
    if (ret != 0)
    {
        g_n6014b_state[id].tx_fail_count++;
    }
    taskEXIT_CRITICAL();
}

static void n6014b_refresh_feedback(actuator_id_e id, const motor_node_param_t *node)
{
    const motor_model_mit_limits_t *limits = motor_cfg_mit_limits(node);
    n6014b_motor_state_t state;
    actuator_feedback_t fb;
    motor_measure_t *measure;
    uint16_t timeout_ms;
    uint32_t now_ms;

    if (n6014b_actuator_id_valid(id) == 0u)
    {
        return;
    }

    taskENTER_CRITICAL();
    state = g_n6014b_state[id];
    timeout_ms = g_n6014b_axis_timeout_ms[id];
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
        g_n6014b_state[id].online = 0u;
        taskEXIT_CRITICAL();
    }

    (void)memset(&fb, 0, sizeof(fb));
    fb.online = state.online;
    fb.bus = state.rs485_port;
    fb.rx_dlc = N6014B_RX_FRAME_SIZE;
    fb.transport = (uint8_t)ACTUATOR_TRANSPORT_RS485;
    fb.motor_id = state.motor_id;
    fb.state = state.mode;
    fb.rx_id = state.motor_id;
    fb.rx_count = state.rx_frame_count;
    fb.last_rx_tick = state.last_rx_tick_ms;
    fb.position = state.position_rad;
    fb.velocity = state.speed_rad_s;
    fb.torque = state.torque_nm;
    fb.ecd = n6014b_position_to_ecd(state.position_rad);
    fb.speed_rpm = n6014b_float_to_i16(state.speed_rad_s * N6014B_RADPS_TO_RPM);
    fb.current = n6014b_torque_to_current_like(limits, state.torque_nm);
    fb.temperature = (uint8_t)state.motor_temp;
    actuator_feedback_update(id, &fb);

    measure = motor_instance_measure(id);
    if (measure != NULL)
    {
        measure->last_ecd = (int16_t)measure->ecd;
        measure->ecd = fb.ecd;
        measure->speed_rpm = fb.speed_rpm;
        measure->given_current = fb.current;
        measure->temperate = fb.temperature;
    }
}

void n6014b_motor_driver_init(void)
{
    taskENTER_CRITICAL();
    (void)memset(g_n6014b_state, 0, sizeof(g_n6014b_state));
    (void)memset(g_n6014b_axis_configured, 0, sizeof(g_n6014b_axis_configured));
    (void)memset(g_n6014b_axis_timeout_ms, 0, sizeof(g_n6014b_axis_timeout_ms));
    (void)memset(g_n6014b_port, 0, sizeof(g_n6014b_port));
    taskEXIT_CRITICAL();
}

int n6014b_motor_send_actuator(uint8_t port,
                               actuator_id_e actuator_id,
                               const motor_node_param_t *node,
                               int16_t current)
{
    uint8_t motor_id;
    uint8_t frame[N6014B_TX_FRAME_SIZE];
    n6014b_mode_e mode;
    fp32 position;
    fp32 velocity;
    fp32 kp;
    fp32 kd;
    fp32 torque;
    const uint32_t baudrate = n6014b_node_baudrate(node);
    const uint16_t timeout_ms = n6014b_node_timeout_ms(node);
    int ret;

    if (n6014b_node_supported(node) == 0u)
    {
        return 1;
    }
    if (n6014b_node_disabled(node) != 0u)
    {
        return 0;
    }
    if (n6014b_node_motor_id(node, &motor_id) == 0u)
    {
        return 1;
    }
    if (port > N6014B_MOTOR_RS485_PORT1)
    {
        return 1;
    }

    n6014b_update_axis_config(actuator_id, port, motor_id, timeout_ms);
    if (n6014b_setup_port(port, baudrate) == 0u)
    {
        n6014b_record_tx_result(actuator_id, 1);
        n6014b_refresh_feedback(actuator_id, node);
        return 1;
    }

    if (n6014b_build_cmd_from_actuator(node,
                                       actuator_id,
                                       current,
                                       &mode,
                                       &position,
                                       &velocity,
                                       &kp,
                                       &kd,
                                       &torque) == 0u)
    {
        n6014b_refresh_feedback(actuator_id, node);
        return 1;
    }

    n6014b_build_tx_frame(frame, motor_id, mode, position, velocity, kp, kd, torque);
    ret = n6014b_tx(port, frame);
    n6014b_record_tx_result(actuator_id, ret);
    n6014b_refresh_feedback(actuator_id, node);
    return ret;
}

const n6014b_motor_state_t *n6014b_motor_get_state(actuator_id_e actuator_id)
{
    if (n6014b_actuator_id_valid(actuator_id) == 0u)
    {
        return 0;
    }
    return &g_n6014b_state[actuator_id];
}

uint8_t can_tx_process_extra_item(uint8_t bus,
                                  actuator_id_e actuator_id,
                                  const motor_node_param_t *node,
                                  int16_t current)
{
    if (n6014b_node_supported(node) == 0u)
    {
        return 0u;
    }

    return (n6014b_motor_send_actuator(bus, actuator_id, node, current) == 0) ? 1u : 0u;
}
