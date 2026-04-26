/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "unitree_motor_driver.h"

#include "FreeRTOS.h"
#include "task.h"

#include "bsp_usart.h"
#include "main.h"

#include <string.h>

typedef enum
{
    UNITREE_MOTOR_MODE_BRAKE = 0u,
    UNITREE_MOTOR_MODE_FOC = 1u,
    UNITREE_MOTOR_MODE_CALIBRATE = 2u,
} unitree_motor_mode_e;

#pragma pack(push, 1)
typedef struct
{
    uint8_t start[2];
    uint8_t motor_id;
    uint8_t head_res;
} unitree_motor_head_t;

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
} unitree_motor_master_cmd_t;

typedef struct
{
    unitree_motor_head_t head;
    unitree_motor_master_cmd_t cmd;
    uint32_t crc32;
} unitree_motor_tx_frame_t;

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
} unitree_motor_servo_cmd_t;

typedef struct
{
    unitree_motor_head_t head;
    unitree_motor_servo_cmd_t data;
    uint32_t crc32;
} unitree_motor_rx_frame_t;
#pragma pack(pop)

#define UNITREE_MOTOR_PI_F                3.1415926f
#define UNITREE_MOTOR_TWO_PI_F            (2.0f * UNITREE_MOTOR_PI_F)
#define UNITREE_MOTOR_TORQUE_SCALE        256.0f
#define UNITREE_MOTOR_SPEED_SCALE         128.0f
#define UNITREE_MOTOR_POSITION_SCALE      (16384.0f / UNITREE_MOTOR_TWO_PI_F)
#define UNITREE_MOTOR_KP_SCALE            2048.0f
#define UNITREE_MOTOR_KD_SCALE            512.0f
#define UNITREE_MOTOR_TX_WORD_COUNT       7u
#define UNITREE_MOTOR_RX_WORD_COUNT       18u
#define UNITREE_MOTOR_RX_FRAME_SIZE       ((uint16_t)sizeof(unitree_motor_rx_frame_t))

static unitree_motor_state_t g_unitree_motor_state;
static unitree_motor_config_t g_unitree_motor_cfg;
static uint8_t g_unitree_motor_rx_buf[UNITREE_MOTOR_RX_FRAME_SIZE];
static volatile uint16_t g_unitree_motor_rx_pos = 0u;
static volatile uint8_t g_unitree_motor_rs485_ready = 0u;
static uint8_t g_unitree_motor_active_port = 0xFFu;
static uint32_t g_unitree_motor_active_baudrate = 0u;

static uint32_t unitree_motor_crc32_words(const uint8_t *data, uint32_t word_count);
static int16_t unitree_motor_float_to_q(fp32 value, fp32 scale);
static int32_t unitree_motor_pos_to_q14(fp32 value);
static fp32 unitree_motor_q14_to_pos(int32_t value);
static fp32 unitree_motor_feedback_speed(int16_t speed_q7, float speed_low);
static void unitree_motor_build_tx_frame(unitree_motor_tx_frame_t *frame,
                                         uint8_t motor_id,
                                         unitree_motor_mode_e mode,
                                         const unitree_motor_cmd_t *cmd);
static void unitree_motor_process_rx_frame(const uint8_t *frame_bytes);
static void unitree_motor_ingest_rx_byte(uint8_t b);
static void unitree_motor_usart2_rx_byte(uint8_t b);
static void unitree_motor_usart3_rx_byte(uint8_t b);
static uint8_t unitree_motor_usart2_error(void);
static uint8_t unitree_motor_usart3_error(void);
static uint8_t unitree_motor_setup_rs485(const unitree_motor_config_t *cfg);
static int unitree_motor_send_frame(const unitree_motor_config_t *cfg, const unitree_motor_tx_frame_t *frame);

static uint32_t unitree_motor_crc32_words(const uint8_t *data, uint32_t word_count)
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

static int16_t unitree_motor_float_to_q(fp32 value, fp32 scale)
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

static int32_t unitree_motor_pos_to_q14(fp32 value)
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

static fp32 unitree_motor_q14_to_pos(int32_t value)
{
    return ((fp32)value) / UNITREE_MOTOR_POSITION_SCALE;
}

static fp32 unitree_motor_feedback_speed(int16_t speed_q7, float speed_low)
{
    if ((speed_low > 0.0001f) || (speed_low < -0.0001f))
    {
        return (fp32)speed_low;
    }

    return ((fp32)speed_q7) / UNITREE_MOTOR_SPEED_SCALE;
}

static void unitree_motor_build_tx_frame(unitree_motor_tx_frame_t *frame,
                                       uint8_t motor_id,
                                       unitree_motor_mode_e mode,
                                       const unitree_motor_cmd_t *cmd)
{
    unitree_motor_cmd_t safe_cmd = {0};

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
    frame->cmd.torque_q8 = unitree_motor_float_to_q(safe_cmd.torque_nm, UNITREE_MOTOR_TORQUE_SCALE);
    frame->cmd.speed_q7 = unitree_motor_float_to_q(safe_cmd.speed_rad_s, UNITREE_MOTOR_SPEED_SCALE);
    frame->cmd.position_q14 = unitree_motor_pos_to_q14(safe_cmd.position_rad);
    frame->cmd.kp_q11 = unitree_motor_float_to_q(safe_cmd.kp, UNITREE_MOTOR_KP_SCALE);
    frame->cmd.kd_q9 = unitree_motor_float_to_q(safe_cmd.kd, UNITREE_MOTOR_KD_SCALE);
    frame->cmd.low_hz_cmd_index = 0u;
    frame->cmd.low_hz_cmd_byte = 0u;
    frame->cmd.end_res = 0u;
    frame->crc32 = unitree_motor_crc32_words((const uint8_t *)frame, UNITREE_MOTOR_TX_WORD_COUNT);
}

static void unitree_motor_process_rx_frame(const uint8_t *frame_bytes)
{
    unitree_motor_rx_frame_t frame;
    uint32_t crc32;
    const unitree_motor_config_t *cfg = &g_unitree_motor_cfg;

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

    crc32 = unitree_motor_crc32_words(frame_bytes, UNITREE_MOTOR_RX_WORD_COUNT);
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
        g_unitree_motor_state.rx_frame_count++;
        g_unitree_motor_state.last_rx_tick_ms = HAL_GetTick();
        g_unitree_motor_state.torque_nm = ((fp32)frame.data.torque_q8) / UNITREE_MOTOR_TORQUE_SCALE;
        g_unitree_motor_state.joint_speed_rad_s = unitree_motor_feedback_speed(frame.data.joint_speed_q7, frame.data.joint_speed_low);
        g_unitree_motor_state.joint_position_rad = unitree_motor_q14_to_pos(frame.data.joint_pos_q14);
        taskEXIT_CRITICAL_FROM_ISR(saved);
    }
}

static void unitree_motor_ingest_rx_byte(uint8_t b)
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
        unitree_motor_process_rx_frame(g_unitree_motor_rx_buf);
        return;
    }

    g_unitree_motor_rx_pos = pos;
}

static void unitree_motor_usart2_rx_byte(uint8_t b)
{
    if (g_unitree_motor_active_port != UNITREE_MOTOR_RS485_PORT0)
    {
        return;
    }

    unitree_motor_ingest_rx_byte(b);
}

static void unitree_motor_usart3_rx_byte(uint8_t b)
{
    if (g_unitree_motor_active_port != UNITREE_MOTOR_RS485_PORT1)
    {
        return;
    }

    unitree_motor_ingest_rx_byte(b);
}

static uint8_t unitree_motor_usart2_error(void)
{
    if (g_unitree_motor_active_port == UNITREE_MOTOR_RS485_PORT0)
    {
        g_unitree_motor_state.rx_parse_error_count++;
    }

    return 0u;
}

static uint8_t unitree_motor_usart3_error(void)
{
    if (g_unitree_motor_active_port == UNITREE_MOTOR_RS485_PORT1)
    {
        g_unitree_motor_state.rx_parse_error_count++;
    }

    return 0u;
}

void unitree_motor_stop(void)
{
    if (g_unitree_motor_active_port == UNITREE_MOTOR_RS485_PORT0)
    {
        bsp_usart2_rx_it_stop();
    }
    else if (g_unitree_motor_active_port == UNITREE_MOTOR_RS485_PORT1)
    {
        bsp_usart3_rx_it_stop();
    }

    g_unitree_motor_active_port = 0xFFu;
    g_unitree_motor_active_baudrate = 0u;
    g_unitree_motor_rs485_ready = 0u;
    g_unitree_motor_rx_pos = 0u;
}

static uint8_t unitree_motor_setup_rs485(const unitree_motor_config_t *cfg)
{
    int ret = 1;

    if (cfg == NULL || cfg->enable == 0u)
    {
        unitree_motor_stop();
        return 0u;
    }

    if (cfg->rs485_port > UNITREE_MOTOR_RS485_PORT1)
    {
        unitree_motor_stop();
        return 0u;
    }

    if (g_unitree_motor_rs485_ready != 0u &&
        g_unitree_motor_active_port == cfg->rs485_port &&
        g_unitree_motor_active_baudrate == cfg->baudrate)
    {
        return 1u;
    }

    bsp_usart2_set_rx_byte_cb(unitree_motor_usart2_rx_byte);
    bsp_usart2_set_error_cb(unitree_motor_usart2_error);
    bsp_usart3_set_rx_byte_cb(unitree_motor_usart3_rx_byte);
    bsp_usart3_set_error_cb(unitree_motor_usart3_error);

    if (g_unitree_motor_active_port == UNITREE_MOTOR_RS485_PORT0 && cfg->rs485_port != UNITREE_MOTOR_RS485_PORT0)
    {
        bsp_usart2_rx_it_stop();
    }
    else if (g_unitree_motor_active_port == UNITREE_MOTOR_RS485_PORT1 && cfg->rs485_port != UNITREE_MOTOR_RS485_PORT1)
    {
        bsp_usart3_rx_it_stop();
    }

    g_unitree_motor_rx_pos = 0u;

    if (cfg->rs485_port == UNITREE_MOTOR_RS485_PORT0)
    {
        ret = bsp_usart2_set_baudrate(cfg->baudrate);
        if (ret == 0)
        {
            ret = bsp_usart2_rx_it_start();
        }
    }
    else
    {
        ret = bsp_usart3_set_baudrate(cfg->baudrate);
        if (ret == 0)
        {
            ret = bsp_usart3_rx_it_start();
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

static int unitree_motor_send_frame(const unitree_motor_config_t *cfg, const unitree_motor_tx_frame_t *frame)
{
    int ret = 1;

    if (cfg == NULL || frame == NULL || cfg->enable == 0u)
    {
        return 1;
    }

    if (cfg->rs485_port == UNITREE_MOTOR_RS485_PORT0)
    {
        ret = bsp_usart2_tx((const uint8_t *)frame, (uint16_t)sizeof(*frame), 5u);
    }
    else if (cfg->rs485_port == UNITREE_MOTOR_RS485_PORT1)
    {
        ret = bsp_usart3_tx((const uint8_t *)frame, (uint16_t)sizeof(*frame), 5u);
    }

    g_unitree_motor_state.tx_count++;
    g_unitree_motor_state.last_tx_status = (uint8_t)ret;
    if (ret != 0)
    {
        g_unitree_motor_state.tx_fail_count++;
    }

    return ret;
}

void unitree_motor_driver_init(void)
{
    unitree_motor_stop();
    (void)memset(&g_unitree_motor_cfg, 0, sizeof(g_unitree_motor_cfg));
    (void)memset(&g_unitree_motor_state, 0, sizeof(g_unitree_motor_state));
}

void unitree_motor_refresh(const unitree_motor_config_t *cfg)
{
    const uint32_t now_ms = HAL_GetTick();

    if (cfg == NULL || cfg->enable == 0u)
    {
        unitree_motor_stop();
        g_unitree_motor_state.enabled = 0u;
        g_unitree_motor_state.online = 0u;
        g_unitree_motor_state.cmd_speed_rad_s = 0.0f;
        g_unitree_motor_state.cmd_kd = 0.0f;
        return;
    }

    g_unitree_motor_state.enabled = cfg->enable;
    g_unitree_motor_state.rs485_port = cfg->rs485_port;
    g_unitree_motor_state.motor_id = cfg->motor_id;

    if ((g_unitree_motor_state.last_rx_tick_ms == 0u) ||
        ((now_ms - g_unitree_motor_state.last_rx_tick_ms) > cfg->rx_timeout_ms))
    {
        g_unitree_motor_state.online = 0u;
    }
}

uint8_t unitree_motor_configure(const unitree_motor_config_t *cfg)
{
    if (cfg == NULL)
    {
        unitree_motor_stop();
        return 0u;
    }

    g_unitree_motor_cfg = *cfg;
    unitree_motor_refresh(cfg);
    return unitree_motor_setup_rs485(cfg);
}

int unitree_motor_send_cmd(const unitree_motor_config_t *cfg, const unitree_motor_cmd_t *cmd)
{
    unitree_motor_cmd_t safe_cmd = {0};
    unitree_motor_tx_frame_t frame;

    if (cfg == NULL || cfg->enable == 0u)
    {
        return 1;
    }

    if (cmd != NULL)
    {
        safe_cmd = *cmd;
    }

    if (unitree_motor_configure(cfg) == 0u)
    {
        return 1;
    }

    g_unitree_motor_state.cmd_speed_rad_s = safe_cmd.speed_rad_s;
    g_unitree_motor_state.cmd_kd = safe_cmd.kd;

    unitree_motor_build_tx_frame(&frame, cfg->motor_id, UNITREE_MOTOR_MODE_FOC, &safe_cmd);
    return unitree_motor_send_frame(cfg, &frame);
}

const unitree_motor_state_t *unitree_motor_get_state(void)
{
    return &g_unitree_motor_state;
}
