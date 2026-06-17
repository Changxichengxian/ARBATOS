/*
 * SPDX-FileCopyrightText: 2026 陈卓 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "VisionLink.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "InsTask.h"
#include "Referee.h"
#include "SdLog.h"
#include "CRC8_CRC16.h"
#include "ExternalMotionIntent.h"
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

static GimbalToVision VisionTx;
static VisionToGimbal VisionRx;
static volatile bool VisionRxUpdated = false;
static uint8_t VisionLinkRxStreamBuf[VISION_RX_STREAM_BUF_SIZE];
static uint32_t VisionLinkRxStreamLen = 0u;

static const fp32 *VisionInsQuat = NULL;
static const fp32 *VisionInsAngle = NULL;
static const fp32 *VisionInsGyro = NULL;

static void VisionLinkHandleRx(const VisionToGimbal *pkt);
static void VisionLinkHandleAlgorithmAim(const AlgorithmAimCmd *cmd);
static void VisionLinkHandleAlgorithmMove(const AlgorithmMoveCmd *cmd);
static void VisionLinkRxStreamConsume(const uint8_t *buf, uint32_t len);
static uint32_t VisionLinkAlgorithmFrameSize(uint8_t cmd_id);
static bool VisionLinkFloatInRange(float v, float limit_abs);
static bool VisionLinkAlgorithmAimValid(const AlgorithmAimCmd *cmd);
static bool VisionLinkAlgorithmMoveValid(const AlgorithmMoveCmd *cmd);

void VisionLinkInit(const fp32 *quat, const fp32 *angle, const fp32 *gyro)
{
    VisionInsQuat = quat;
    VisionInsAngle = angle;
    VisionInsGyro = gyro;

    memset(&VisionTx, 0, sizeof(VisionTx));
    VisionTx.head[0] = 'S';
    VisionTx.head[1] = 'P';
    VisionTx.mode = 0u;

    taskENTER_CRITICAL();
    memset(&VisionRx, 0, sizeof(VisionRx));
    VisionRxUpdated = false;
    taskEXIT_CRITICAL();
    ExternalMotionIntentClear();

    VisionLinkRxStreamLen = 0u;
}

bool VisionTakeLatest(VisionToGimbal *out)
{
    bool has_new = false;
    if (VisionRxUpdated && out != NULL)
    {
        taskENTER_CRITICAL();
        *out = VisionRx;
        VisionRxUpdated = false;
        taskEXIT_CRITICAL();
        has_new = true;
    }
    return has_new;
}

void VisionLinkPollTx(void)
{
    VisionTx.q[0] = 0.0f;
    VisionTx.q[1] = 0.0f;
    VisionTx.q[2] = 0.0f;
    VisionTx.q[3] = 0.0f;
    VisionTx.yaw = 0.0f;
    VisionTx.yaw_vel = 0.0f;
    VisionTx.pitch = 0.0f;
    VisionTx.pitch_vel = 0.0f;
    VisionTx.bullet_speed = 0.0f;
    VisionTx.bullet_count = 0u;
    VisionTx.crc16 = 0u;

    if (VisionInsQuat != NULL)
    {
        VisionTx.q[0] = VisionInsQuat[0];
        VisionTx.q[1] = VisionInsQuat[1];
        VisionTx.q[2] = VisionInsQuat[2];
        VisionTx.q[3] = VisionInsQuat[3];
    }
    if (VisionInsAngle != NULL)
    {
        VisionTx.yaw = VisionInsAngle[INS_YAW_ADDRESS_OFFSET];
        VisionTx.pitch = VisionInsAngle[INS_PITCH_ADDRESS_OFFSET];
    }
    if (VisionInsGyro != NULL)
    {
        VisionTx.yaw_vel = VisionInsGyro[INS_GYRO_Z_ADDRESS_OFFSET];
        VisionTx.pitch_vel = VisionInsGyro[INS_GYRO_Y_ADDRESS_OFFSET];
    }

    VisionTx.bullet_speed = ShootData.initial_speed;
    VisionTx.bullet_count = bullet_remaining_t.projectile_allowance_17mm;
    append_CRC16_check_sum((uint8_t *)&VisionTx, (uint32_t)sizeof(VisionTx));

    if (CDC_Transmit_FS((uint8_t *)&VisionTx, sizeof(VisionTx)) == USBD_BUSY)
    {
        return;
    }
}

void VisionLinkRxCallback(uint8_t *buf, uint32_t len)
{
    VisionLinkRxStreamConsume(buf, len);
}

static void VisionLinkStoreRxFromIsr(const VisionToGimbal *pkt)
{
    if (pkt == NULL)
    {
        return;
    }

    UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
    VisionRx = *pkt;
    VisionRxUpdated = true;
    taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
}

static void VisionLinkHandleRx(const VisionToGimbal *pkt)
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
        SdLogWriteIsr(SDLOG_TAG_VISION_RX, pkt, (uint16_t)sizeof(*pkt));
    }

    VisionLinkStoreRxFromIsr(pkt);
}

static void VisionLinkHandleAlgorithmAim(const AlgorithmAimCmd *cmd)
{
    VisionToGimbal pkt;

    if (!VisionLinkAlgorithmAimValid(cmd))
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
        SdLogWriteIsr(SDLOG_TAG_VISION_RX, &pkt, (uint16_t)sizeof(pkt));
    }

    VisionLinkStoreRxFromIsr(&pkt);
}

static void VisionLinkHandleAlgorithmMove(const AlgorithmMoveCmd *cmd)
{
    ExternalMotionIntent intent;

    if (!VisionLinkAlgorithmMoveValid(cmd))
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

    ExternalMotionIntentWriteFromIsr(&intent);
}

static uint32_t VisionLinkAlgorithmFrameSize(uint8_t cmd_id)
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

static bool VisionLinkFloatInRange(float v, float limit_abs)
{
    return (v == v && v <= limit_abs && v >= -limit_abs) ? true : false;
}

static bool VisionLinkAlgorithmAimValid(const AlgorithmAimCmd *cmd)
{
    if (cmd == NULL ||
        cmd->mode > (uint8_t)ALGORITHM_AIM_MODE_CONTROL_FIRE)
    {
        return false;
    }

    return VisionLinkFloatInRange(cmd->yaw_rad, 1000.0f) &&
           VisionLinkFloatInRange(cmd->yaw_vel_radps, 1000.0f) &&
           VisionLinkFloatInRange(cmd->yaw_acc_radps2, 10000.0f) &&
           VisionLinkFloatInRange(cmd->pitch_rad, 1000.0f) &&
           VisionLinkFloatInRange(cmd->pitch_vel_radps, 1000.0f) &&
           VisionLinkFloatInRange(cmd->pitch_acc_radps2, 10000.0f);
}

static bool VisionLinkAlgorithmMoveValid(const AlgorithmMoveCmd *cmd)
{
    if (cmd == NULL ||
        cmd->mode > (uint8_t)EXTERNAL_MOTION_MODE_STOP ||
        cmd->frame > (uint8_t)EXTERNAL_MOTION_FRAME_FIELD)
    {
        return false;
    }

    return VisionLinkFloatInRange(cmd->vx_mps, 100.0f) &&
           VisionLinkFloatInRange(cmd->vy_mps, 100.0f) &&
           VisionLinkFloatInRange(cmd->wz_radps, 100.0f) &&
           VisionLinkFloatInRange(cmd->yaw_offset_rad, 1000.0f) &&
           VisionLinkFloatInRange(cmd->ax_mps2, 1000.0f) &&
           VisionLinkFloatInRange(cmd->ay_mps2, 1000.0f) &&
           VisionLinkFloatInRange(cmd->wz_acc_radps2, 1000.0f);
}

static void VisionLinkRxStreamConsume(const uint8_t *buf, uint32_t len)
{
    if (buf == NULL || len == 0u)
    {
        return;
    }

    if (len >= VISION_RX_STREAM_BUF_SIZE)
    {
        buf += (len - VISION_RX_STREAM_BUF_SIZE);
        len = VISION_RX_STREAM_BUF_SIZE;
        VisionLinkRxStreamLen = 0u;
    }

    if ((VisionLinkRxStreamLen + len) > VISION_RX_STREAM_BUF_SIZE)
    {
        const uint32_t overflow = (VisionLinkRxStreamLen + len) - VISION_RX_STREAM_BUF_SIZE;
        if (overflow >= VisionLinkRxStreamLen)
        {
            VisionLinkRxStreamLen = 0u;
        }
        else
        {
            VisionLinkRxStreamLen -= overflow;
            memmove(VisionLinkRxStreamBuf,
                    &VisionLinkRxStreamBuf[overflow],
                    VisionLinkRxStreamLen);
        }
    }

    memcpy(&VisionLinkRxStreamBuf[VisionLinkRxStreamLen], buf, len);
    VisionLinkRxStreamLen += len;

    uint32_t consume = 0u;
    while ((VisionLinkRxStreamLen - consume) >= 2u)
    {
        const uint8_t *candidate = &VisionLinkRxStreamBuf[consume];

        if (candidate[0] == 'S' && candidate[1] == 'P')
        {
            if ((VisionLinkRxStreamLen - consume) < VISION_RX_FRAME_SIZE)
            {
                break;
            }
            if (verify_CRC16_check_sum((uint8_t *)candidate, VISION_RX_FRAME_SIZE))
            {
                VisionToGimbal pkt;
                memcpy(&pkt, candidate, sizeof(pkt));
                VisionLinkHandleRx(&pkt);
                consume += VISION_RX_FRAME_SIZE;
                continue;
            }
            consume++;
            continue;
        }

        if (candidate[0] == ALGORITHM_FRAME_SOF0 && candidate[1] == ALGORITHM_FRAME_SOF1)
        {
            if ((VisionLinkRxStreamLen - consume) < ALGORITHM_FRAME_HEADER_SIZE)
            {
                break;
            }

            const uint8_t cmd_id = candidate[2];
            const uint32_t frame_size = VisionLinkAlgorithmFrameSize(cmd_id);
            if (frame_size == 0u)
            {
                consume++;
                continue;
            }
            if ((VisionLinkRxStreamLen - consume) < frame_size)
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
                VisionLinkHandleAlgorithmAim(&cmd);
            }
            else if (cmd_id == (uint8_t)ALGORITHM_CMD_MOVE)
            {
                AlgorithmMoveCmd cmd;
                memcpy(&cmd, payload, sizeof(cmd));
                VisionLinkHandleAlgorithmMove(&cmd);
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

    VisionLinkRxStreamLen -= consume;
    if (VisionLinkRxStreamLen > 0u)
    {
        memmove(VisionLinkRxStreamBuf,
                &VisionLinkRxStreamBuf[consume],
                VisionLinkRxStreamLen);
    }
}
