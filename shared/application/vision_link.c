/*
 * SPDX-FileCopyrightText: 2026 陈卓 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈卓 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "vision_link.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "INS_task.h"
#include "shoot.h"
#include "referee.h"
#include "sdlog.h"
#include "CRC8_CRC16.h"
#include "usbd_cdc_if.h"

#define VISION_RX_FRAME_SIZE      ((uint32_t)sizeof(VisionToGimbal))
#define VISION_RX_STREAM_BUF_SIZE 128u

typedef char _check_GimbalToVision_size[(sizeof(GimbalToVision) == 43) ? 1 : -1];
typedef char _check_VisionToGimbal_size[(sizeof(VisionToGimbal) == 29) ? 1 : -1];

static GimbalToVision vision_tx;
static VisionToGimbal vision_rx;
static volatile bool vision_rx_updated = false;
static uint8_t vision_link_rx_stream_buf[VISION_RX_STREAM_BUF_SIZE];
static uint32_t vision_link_rx_stream_len = 0u;

static const fp32 *vision_ins_quat = NULL;
static const fp32 *vision_ins_angle = NULL;
static const fp32 *vision_ins_gyro = NULL;

static void vision_link_handle_rx(const VisionToGimbal *pkt);
static void vision_link_rx_stream_consume(const uint8_t *buf, uint32_t len);

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

    UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
    vision_rx = *pkt;
    vision_rx_updated = true;
    taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
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
    while ((vision_link_rx_stream_len - consume) >= VISION_RX_FRAME_SIZE)
    {
        const uint8_t *candidate = &vision_link_rx_stream_buf[consume];

        if (candidate[0] != 'S' || candidate[1] != 'P')
        {
            consume++;
            continue;
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
