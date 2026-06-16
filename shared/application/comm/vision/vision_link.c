/*
 * SPDX-FileCopyrightText: 2026 陈卓 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "vision_link.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "INS_task.h"
#include "referee.h"
#include "sdlog.h"
#include "CRC8_CRC16.h"
#include "external_motion_intent.h"
#include "usbd_cdc_if.h"

#define VISION_RX_FRAME_SIZE      ((uint32_t)sizeof(VisionToGimbal))
#define VISION_RX_STREAM_BUF_SIZE 128u
#define ALGORITHM_FRAME_HEADER_SIZE 3u
#define ALGORITHM_FRAME_CRC_SIZE    2u
#define ALGORITHM_AIM_FRAME_SIZE    (ALGORITHM_FRAME_HEADER_SIZE + (uint32_t)sizeof(AlgorithmAimCmd) + ALGORITHM_FRAME_CRC_SIZE)
#define ALGORITHM_MOVE_FRAME_SIZE   (ALGORITHM_FRAME_HEADER_SIZE + (uint32_t)sizeof(AlgorithmMoveCmd) + ALGORITHM_FRAME_CRC_SIZE)

#define ALGORITHM_FRAME_SOF0 0xA5u
#define ALGORITHM_FRAME_SOF1 0x5Au

typedef enum
{
    ALGORITHM_CMD_AIM = 0x01u,
    ALGORITHM_CMD_MOVE = 0x02u,
} algorithm_cmd_e;

typedef enum
{
    ALGORITHM_AIM_MODE_IDLE = 0u,
    ALGORITHM_AIM_MODE_CONTROL = 1u,
    ALGORITHM_AIM_MODE_CONTROL_FIRE = 2u,
} algorithm_aim_mode_e;

typedef struct __attribute__((packed)) AlgorithmAimCmd
{
    uint8_t mode;
    uint8_t flags;
    uint16_t timeout_ms;
    float yaw_rad;
    float yaw_vel_radps;
    float yaw_acc_radps2;
    float pitch_rad;
    float pitch_vel_radps;
    float pitch_acc_radps2;
} AlgorithmAimCmd;

typedef struct __attribute__((packed)) AlgorithmMoveCmd
{
    uint8_t mode;
    uint8_t frame;
    uint16_t flags;
    uint16_t timeout_ms;
    uint16_t reserved;
    float vx_mps;
    float vy_mps;
    float wz_radps;
    float yaw_offset_rad;
    float ax_mps2;
    float ay_mps2;
    float wz_acc_radps2;
} AlgorithmMoveCmd;

typedef char _check_GimbalToVision_size[(sizeof(GimbalToVision) == 43) ? 1 : -1];
typedef char _check_VisionToGimbal_size[(sizeof(VisionToGimbal) == 29) ? 1 : -1];
typedef char _check_AlgorithmAimCmd_size[(sizeof(AlgorithmAimCmd) == 28) ? 1 : -1];
typedef char _check_AlgorithmMoveCmd_size[(sizeof(AlgorithmMoveCmd) == 36) ? 1 : -1];

static GimbalToVision vision_tx;
static VisionToGimbal vision_rx;
static volatile bool vision_rx_updated = false;
static uint8_t vision_link_rx_stream_buf[VISION_RX_STREAM_BUF_SIZE];
static uint32_t vision_link_rx_stream_len = 0u;

static const fp32 *vision_ins_quat = NULL;
static const fp32 *vision_ins_angle = NULL;
static const fp32 *vision_ins_gyro = NULL;

static void vision_link_handle_rx(const VisionToGimbal *pkt);
static void vision_link_handle_algorithm_aim(const AlgorithmAimCmd *cmd);
static void vision_link_handle_algorithm_move(const AlgorithmMoveCmd *cmd);
static void vision_link_rx_stream_consume(const uint8_t *buf, uint32_t len);
static uint32_t vision_link_algorithm_frame_size(uint8_t cmd_id);
static bool vision_link_float_in_range(float v, float limit_abs);
static bool vision_link_algorithm_aim_valid(const AlgorithmAimCmd *cmd);
static bool vision_link_algorithm_move_valid(const AlgorithmMoveCmd *cmd);

void vision_link_init(const fp32 *quat, const fp32 *angle, const fp32 *gyro)
{
    vision_ins_quat = quat;
    vision_ins_angle = angle;
    vision_ins_gyro = gyro;

    memset(&vision_tx, 0, sizeof(vision_tx));
    vision_tx.head[0] = 'S';
    vision_tx.head[1] = 'P';
    vision_tx.mode = 0u;

    taskENTER_CRITICAL();
    memset(&vision_rx, 0, sizeof(vision_rx));
    vision_rx_updated = false;
    taskEXIT_CRITICAL();
    external_motion_intent_clear();

    vision_link_rx_stream_len = 0u;
}

bool vision_take_latest(VisionToGimbal *out)
{
    bool has_new = false;
    if (vision_rx_updated && out != NULL)
    {
        taskENTER_CRITICAL();
        *out = vision_rx;
        vision_rx_updated = false;
        taskEXIT_CRITICAL();
        has_new = true;
    }
    return has_new;
}

void vision_link_poll_tx(void)
{
    vision_tx.q[0] = 0.0f;
    vision_tx.q[1] = 0.0f;
    vision_tx.q[2] = 0.0f;
    vision_tx.q[3] = 0.0f;
    vision_tx.yaw = 0.0f;
    vision_tx.yaw_vel = 0.0f;
    vision_tx.pitch = 0.0f;
    vision_tx.pitch_vel = 0.0f;
    vision_tx.bullet_speed = 0.0f;
    vision_tx.bullet_count = 0u;
    vision_tx.crc16 = 0u;

    if (vision_ins_quat != NULL)
    {
        vision_tx.q[0] = vision_ins_quat[0];
        vision_tx.q[1] = vision_ins_quat[1];
        vision_tx.q[2] = vision_ins_quat[2];
        vision_tx.q[3] = vision_ins_quat[3];
    }
    if (vision_ins_angle != NULL)
    {
        vision_tx.yaw = vision_ins_angle[INS_YAW_ADDRESS_OFFSET];
        vision_tx.pitch = vision_ins_angle[INS_PITCH_ADDRESS_OFFSET];
    }
    if (vision_ins_gyro != NULL)
    {
        vision_tx.yaw_vel = vision_ins_gyro[INS_GYRO_Z_ADDRESS_OFFSET];
        vision_tx.pitch_vel = vision_ins_gyro[INS_GYRO_Y_ADDRESS_OFFSET];
    }

    vision_tx.bullet_speed = shoot_data_t.initial_speed;
    vision_tx.bullet_count = bullet_remaining_t.projectile_allowance_17mm;
    append_CRC16_check_sum((uint8_t *)&vision_tx, (uint32_t)sizeof(vision_tx));

    if (CDC_Transmit_FS((uint8_t *)&vision_tx, sizeof(vision_tx)) == USBD_BUSY)
    {
        return;
    }
}

void vision_link_rx_callback(uint8_t *buf, uint32_t len)
{
    vision_link_rx_stream_consume(buf, len);
}

static void vision_link_store_rx_from_isr(const VisionToGimbal *pkt)
{
    if (pkt == NULL)
    {
        return;
    }

    UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
    vision_rx = *pkt;
    vision_rx_updated = true;
    taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
}

static void vision_link_handle_rx(const VisionToGimbal *pkt)
{
    if (pkt->head[0] != 'S' || pkt->head[1] != 'P')
    {
        return;
    }
    if (!verify_CRC16_check_sum((uint8_t *)pkt, (uint32_t)sizeof(*pkt)))
    {
        return;
    }

    uint32_t yaw_bits = 0u;
    uint32_t pitch_bits = 0u;
    memcpy(&yaw_bits, (const void *)&pkt->yaw, sizeof(yaw_bits));
    memcpy(&pitch_bits, (const void *)&pkt->pitch, sizeof(pitch_bits));
    const bool yaw_has_cmd = ((yaw_bits & 0x7FFFFFFFu) != 0u);
    const bool pitch_has_cmd = ((pitch_bits & 0x7FFFFFFFu) != 0u);
    if (yaw_has_cmd || pitch_has_cmd)
    {
        sdlog_write_isr(SDLOG_TAG_VISION_RX, pkt, (uint16_t)sizeof(*pkt));
    }

    vision_link_store_rx_from_isr(pkt);
}

static void vision_link_handle_algorithm_aim(const AlgorithmAimCmd *cmd)
{
    VisionToGimbal pkt;

    if (!vision_link_algorithm_aim_valid(cmd))
    {
        return;
    }

    memset(&pkt, 0, sizeof(pkt));
    pkt.head[0] = 'S';
    pkt.head[1] = 'P';
    pkt.mode = cmd->mode;
    pkt.yaw = cmd->yaw_rad;
    pkt.yaw_vel = cmd->yaw_vel_radps;
    pkt.yaw_acc = cmd->yaw_acc_radps2;
    pkt.pitch = cmd->pitch_rad;
    pkt.pitch_vel = cmd->pitch_vel_radps;
    pkt.pitch_acc = cmd->pitch_acc_radps2;

    if (pkt.mode == (uint8_t)ALGORITHM_AIM_MODE_CONTROL ||
        pkt.mode == (uint8_t)ALGORITHM_AIM_MODE_CONTROL_FIRE)
    {
        sdlog_write_isr(SDLOG_TAG_VISION_RX, &pkt, (uint16_t)sizeof(pkt));
    }

    vision_link_store_rx_from_isr(&pkt);
}

static void vision_link_handle_algorithm_move(const AlgorithmMoveCmd *cmd)
{
    external_motion_intent_t intent;

    if (!vision_link_algorithm_move_valid(cmd))
    {
        return;
    }

    memset(&intent, 0, sizeof(intent));
    intent.mode = cmd->mode;
    intent.frame = cmd->frame;
    intent.flags = cmd->flags;
    intent.timeout_ms = cmd->timeout_ms;
    intent.vx_mps = cmd->vx_mps;
    intent.vy_mps = cmd->vy_mps;
    intent.wz_radps = cmd->wz_radps;
    intent.yaw_offset_rad = cmd->yaw_offset_rad;
    intent.ax_mps2 = cmd->ax_mps2;
    intent.ay_mps2 = cmd->ay_mps2;
    intent.wz_acc_radps2 = cmd->wz_acc_radps2;

    external_motion_intent_write_from_isr(&intent);
}

static uint32_t vision_link_algorithm_frame_size(uint8_t cmd_id)
{
    if (cmd_id == (uint8_t)ALGORITHM_CMD_AIM)
    {
        return ALGORITHM_AIM_FRAME_SIZE;
    }
    if (cmd_id == (uint8_t)ALGORITHM_CMD_MOVE)
    {
        return ALGORITHM_MOVE_FRAME_SIZE;
    }
    return 0u;
}

static bool vision_link_float_in_range(float v, float limit_abs)
{
    return (v == v && v <= limit_abs && v >= -limit_abs) ? true : false;
}

static bool vision_link_algorithm_aim_valid(const AlgorithmAimCmd *cmd)
{
    if (cmd == NULL ||
        cmd->mode > (uint8_t)ALGORITHM_AIM_MODE_CONTROL_FIRE)
    {
        return false;
    }

    return vision_link_float_in_range(cmd->yaw_rad, 1000.0f) &&
           vision_link_float_in_range(cmd->yaw_vel_radps, 1000.0f) &&
           vision_link_float_in_range(cmd->yaw_acc_radps2, 10000.0f) &&
           vision_link_float_in_range(cmd->pitch_rad, 1000.0f) &&
           vision_link_float_in_range(cmd->pitch_vel_radps, 1000.0f) &&
           vision_link_float_in_range(cmd->pitch_acc_radps2, 10000.0f);
}

static bool vision_link_algorithm_move_valid(const AlgorithmMoveCmd *cmd)
{
    if (cmd == NULL ||
        cmd->mode > (uint8_t)EXTERNAL_MOTION_MODE_STOP ||
        cmd->frame > (uint8_t)EXTERNAL_MOTION_FRAME_FIELD)
    {
        return false;
    }

    return vision_link_float_in_range(cmd->vx_mps, 100.0f) &&
           vision_link_float_in_range(cmd->vy_mps, 100.0f) &&
           vision_link_float_in_range(cmd->wz_radps, 100.0f) &&
           vision_link_float_in_range(cmd->yaw_offset_rad, 1000.0f) &&
           vision_link_float_in_range(cmd->ax_mps2, 1000.0f) &&
           vision_link_float_in_range(cmd->ay_mps2, 1000.0f) &&
           vision_link_float_in_range(cmd->wz_acc_radps2, 1000.0f);
}

static void vision_link_rx_stream_consume(const uint8_t *buf, uint32_t len)
{
    if (buf == NULL || len == 0u)
    {
        return;
    }

    if (len >= VISION_RX_STREAM_BUF_SIZE)
    {
        buf += (len - VISION_RX_STREAM_BUF_SIZE);
        len = VISION_RX_STREAM_BUF_SIZE;
        vision_link_rx_stream_len = 0u;
    }

    if ((vision_link_rx_stream_len + len) > VISION_RX_STREAM_BUF_SIZE)
    {
        const uint32_t overflow = (vision_link_rx_stream_len + len) - VISION_RX_STREAM_BUF_SIZE;
        if (overflow >= vision_link_rx_stream_len)
        {
            vision_link_rx_stream_len = 0u;
        }
        else
        {
            vision_link_rx_stream_len -= overflow;
            memmove(vision_link_rx_stream_buf,
                    &vision_link_rx_stream_buf[overflow],
                    vision_link_rx_stream_len);
        }
    }

    memcpy(&vision_link_rx_stream_buf[vision_link_rx_stream_len], buf, len);
    vision_link_rx_stream_len += len;

    uint32_t consume = 0u;
    while ((vision_link_rx_stream_len - consume) >= 2u)
    {
        const uint8_t *candidate = &vision_link_rx_stream_buf[consume];

        if (candidate[0] == 'S' && candidate[1] == 'P')
        {
            if ((vision_link_rx_stream_len - consume) < VISION_RX_FRAME_SIZE)
            {
                break;
            }
            if (verify_CRC16_check_sum((uint8_t *)candidate, VISION_RX_FRAME_SIZE))
            {
                VisionToGimbal pkt;
                memcpy(&pkt, candidate, sizeof(pkt));
                vision_link_handle_rx(&pkt);
                consume += VISION_RX_FRAME_SIZE;
                continue;
            }
            consume++;
            continue;
        }

        if (candidate[0] == ALGORITHM_FRAME_SOF0 && candidate[1] == ALGORITHM_FRAME_SOF1)
        {
            if ((vision_link_rx_stream_len - consume) < ALGORITHM_FRAME_HEADER_SIZE)
            {
                break;
            }

            const uint8_t cmd_id = candidate[2];
            const uint32_t frame_size = vision_link_algorithm_frame_size(cmd_id);
            if (frame_size == 0u)
            {
                consume++;
                continue;
            }
            if ((vision_link_rx_stream_len - consume) < frame_size)
            {
                break;
            }
            if (!verify_CRC16_check_sum((uint8_t *)candidate, frame_size))
            {
                consume++;
                continue;
            }

            const uint8_t *payload = &candidate[ALGORITHM_FRAME_HEADER_SIZE];
            if (cmd_id == (uint8_t)ALGORITHM_CMD_AIM)
            {
                AlgorithmAimCmd cmd;
                memcpy(&cmd, payload, sizeof(cmd));
                vision_link_handle_algorithm_aim(&cmd);
            }
            else if (cmd_id == (uint8_t)ALGORITHM_CMD_MOVE)
            {
                AlgorithmMoveCmd cmd;
                memcpy(&cmd, payload, sizeof(cmd));
                vision_link_handle_algorithm_move(&cmd);
            }
            consume += frame_size;
            continue;
        }

        consume++;
    }

    if (consume == 0u)
    {
        return;
    }

    vision_link_rx_stream_len -= consume;
    if (vision_link_rx_stream_len > 0u)
    {
        memmove(vision_link_rx_stream_buf,
                &vision_link_rx_stream_buf[consume],
                vision_link_rx_stream_len);
    }
}
