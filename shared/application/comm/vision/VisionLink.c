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

#include "Referee.h"
#include "SdLog.h"
#include "Crc8Crc16.h"
#include "ChassisState.h"
#include "ExternalMotionIntent.h"
#include "GimbalState.h"
#include "usbd_cdc_if.h"

#define VISION_RX_FRAME_SIZE      ((uint32_t)sizeof(VisionToGimbal))
#define CHASSIS_RX_FRAME_SIZE     ((uint32_t)sizeof(VisionToChassis))
#define VISION_RX_STREAM_BUF_SIZE 128u
#define VISION_FRAME_SOF0         'L'
#define VISION_FRAME_SOF1         'S'
#define CHASSIS_FRAME_SOF0        'L'
#define CHASSIS_FRAME_SOF1        'C'
#define VISION_MOVE_STATE_TX_DIV    5u

typedef struct __attribute__((packed)) VisionToChassis
{
    uint8_t head[2];
    uint8_t mode;
    uint8_t frame;
    uint8_t flags;
    uint8_t timeout_10ms;
    int16_t vx_cmps;
    int16_t vy_cmps;
    int16_t wz_mradps;
    int16_t yaw_offset_mrad;
    int16_t ax_cmps2;
    int16_t ay_cmps2;
    int16_t wz_acc_mradps2;
    uint16_t crc16;
} VisionToChassis;

typedef struct __attribute__((packed)) ChassisToVision
{
    uint8_t head[2];
    uint8_t valid;
    uint8_t mode;
    int16_t vx_cmps;
    int16_t vy_cmps;
    int16_t wz_mradps;
    int16_t vx_set_cmps;
    int16_t vy_set_cmps;
    int16_t wz_set_mradps;
    uint16_t crc16;
} ChassisToVision;

typedef char _check_GimbalToVision_size[(sizeof(GimbalToVision) == 43) ? 1 : -1];
typedef char _check_VisionToGimbal_size[(sizeof(VisionToGimbal) == 29) ? 1 : -1];
typedef char _check_VisionToChassis_size[(sizeof(VisionToChassis) == 22) ? 1 : -1];
typedef char _check_ChassisToVision_size[(sizeof(ChassisToVision) == 18) ? 1 : -1];

static GimbalToVision VisionTx;
static ChassisToVision ChassisTx;
static VisionToGimbal VisionRx;
static volatile bool VisionRxUpdated = false;
static uint8_t VisionLinkRxStreamBuf[VISION_RX_STREAM_BUF_SIZE];
static uint32_t VisionLinkRxStreamLen = 0u;
static uint32_t VisionLinkTxCycle = 0u;
static VisionLinkDebug VisionLinkDbg;

static const fp32 *VisionInsQuat = NULL;

static void VisionLinkHandleRx(const VisionToGimbal *pkt);
static void VisionLinkHandleChassisCmd(const VisionToChassis *cmd);
static bool VisionLinkPollChassisStateTx(void);
static bool VisionLinkApplyGimbalState(GimbalToVision *tx);
static void VisionLinkRxStreamConsume(const uint8_t *buf, uint32_t len);
static bool VisionLinkFloatInRange(float v, float limit_abs);
static bool VisionLinkExternalMotionIntentValid(const ExternalMotionIntent *intent);
static void VisionLinkChassisCmdToIntent(const VisionToChassis *cmd, ExternalMotionIntent *intent);
static int16_t VisionLinkFp32ToI16Scaled(fp32 value, fp32 scale);
static uint32_t VisionLinkNowFromIsrMs(void);
static uint16_t VisionLinkReadLe16(const uint8_t *buf);
static uint16_t VisionLinkCalcCrc16(const uint8_t *buf, uint32_t len);
static void VisionLinkDebugStoreChassisCmd(const VisionToChassis *cmd);

void VisionLinkInit(const fp32 *quat, const fp32 *angle, const fp32 *gyro)
{
    VisionInsQuat = quat;
    (void)angle;
    (void)gyro;

    memset(&VisionTx, 0, sizeof(VisionTx));
    VisionTx.head[0] = VISION_FRAME_SOF0;
    VisionTx.head[1] = VISION_FRAME_SOF1;
    VisionTx.mode = 0u;

    taskENTER_CRITICAL();
    memset(&VisionRx, 0, sizeof(VisionRx));
    VisionRxUpdated = false;
    taskEXIT_CRITICAL();
    ExternalMotionIntentClear();

    VisionLinkRxStreamLen = 0u;
    VisionLinkTxCycle = 0u;
    memset(&VisionLinkDbg, 0, sizeof(VisionLinkDbg));
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
    VisionLinkTxCycle++;
    if ((VisionLinkTxCycle % VISION_MOVE_STATE_TX_DIV) == 0u &&
        VisionLinkPollChassisStateTx())
    {
        return;
    }

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
    (void)VisionLinkApplyGimbalState(&VisionTx);

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
    VisionLinkDbg.rx_callback_count++;
    VisionLinkDbg.rx_bytes += len;
    VisionLinkDbg.rx_last_len = len;
    VisionLinkDbg.rx_last_tick_ms = VisionLinkNowFromIsrMs();
    VisionLinkRxStreamConsume(buf, len);
}

void VisionLinkGetDebug(VisionLinkDebug *out)
{
    if (out == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    *out = VisionLinkDbg;
    taskEXIT_CRITICAL();
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
    if (pkt->head[0] != VISION_FRAME_SOF0 || pkt->head[1] != VISION_FRAME_SOF1)
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

static void VisionLinkHandleChassisCmd(const VisionToChassis *cmd)
{
    ExternalMotionIntent intent;

    if (cmd == NULL ||
        cmd->head[0] != CHASSIS_FRAME_SOF0 ||
        cmd->head[1] != CHASSIS_FRAME_SOF1)
    {
        return;
    }

    VisionLinkDebugStoreChassisCmd(cmd);
    VisionLinkChassisCmdToIntent(cmd, &intent);
    if (!VisionLinkExternalMotionIntentValid(&intent))
    {
        VisionLinkDbg.lc_invalid_count++;
        return;
    }

    VisionLinkDbg.lc_valid_count++;
    ExternalMotionIntentWriteFromIsr(&intent);
}

static bool VisionLinkApplyGimbalState(GimbalToVision *tx)
{
    GimbalState state;

    if (tx == NULL ||
        GimbalStateRead(&state) == 0u ||
        state.valid == 0u)
    {
        return false;
    }

    if (state.yaw.valid != 0u)
    {
        tx->yaw = state.yaw.angle;
        tx->yaw_vel = state.yaw.motor_gyro;
    }
    if (state.pitch.valid != 0u)
    {
        tx->pitch = state.pitch.angle;
        tx->pitch_vel = state.pitch.motor_gyro;
    }
    return true;
}

static bool VisionLinkPollChassisStateTx(void)
{
    ChassisState state;

    if (ChassisStateRead(&state) == 0u || state.valid == 0u)
    {
        return false;
    }

    memset(&ChassisTx, 0, sizeof(ChassisTx));
    ChassisTx.head[0] = CHASSIS_FRAME_SOF0;
    ChassisTx.head[1] = CHASSIS_FRAME_SOF1;
    ChassisTx.valid = 1u;
    ChassisTx.mode = state.mode;
    ChassisTx.vx_cmps = VisionLinkFp32ToI16Scaled(state.vx, 100.0f);
    ChassisTx.vy_cmps = VisionLinkFp32ToI16Scaled(state.vy, 100.0f);
    ChassisTx.wz_mradps = VisionLinkFp32ToI16Scaled(state.wz, 1000.0f);
    ChassisTx.vx_set_cmps = VisionLinkFp32ToI16Scaled(state.vx_set, 100.0f);
    ChassisTx.vy_set_cmps = VisionLinkFp32ToI16Scaled(state.vy_set, 100.0f);
    ChassisTx.wz_set_mradps = VisionLinkFp32ToI16Scaled(state.wz_set, 1000.0f);
    append_CRC16_check_sum((uint8_t *)&ChassisTx, (uint32_t)sizeof(ChassisTx));

    if (CDC_Transmit_FS((uint8_t *)&ChassisTx, sizeof(ChassisTx)) == USBD_BUSY)
    {
        return true;
    }
    return true;
}

static bool VisionLinkFloatInRange(float v, float limit_abs)
{
    return (v == v && v <= limit_abs && v >= -limit_abs) ? true : false;
}

static bool VisionLinkExternalMotionIntentValid(const ExternalMotionIntent *intent)
{
    if (intent == NULL ||
        intent->mode > (uint8_t)EXTERNAL_MOTION_MODE_STOP ||
        intent->frame > (uint8_t)EXTERNAL_MOTION_FRAME_FIELD)
    {
        return false;
    }

    return VisionLinkFloatInRange(intent->vx_mps, 100.0f) &&
           VisionLinkFloatInRange(intent->vy_mps, 100.0f) &&
           VisionLinkFloatInRange(intent->wz_radps, 100.0f) &&
           VisionLinkFloatInRange(intent->yaw_offset_rad, 1000.0f) &&
           VisionLinkFloatInRange(intent->ax_mps2, 1000.0f) &&
           VisionLinkFloatInRange(intent->ay_mps2, 1000.0f) &&
           VisionLinkFloatInRange(intent->wz_acc_radps2, 1000.0f);
}

static void VisionLinkChassisCmdToIntent(const VisionToChassis *cmd, ExternalMotionIntent *intent)
{
    if (intent == NULL)
    {
        return;
    }

    memset(intent, 0, sizeof(*intent));
    if (cmd == NULL)
    {
        return;
    }

    intent->mode = cmd->mode;
    intent->frame = cmd->frame;
    intent->flags = cmd->flags;
    intent->timeout_ms = (cmd->timeout_10ms != 0u) ? ((uint16_t)cmd->timeout_10ms * 10u) : 0u;
    intent->vx_mps = (fp32)cmd->vx_cmps * 0.01f;
    intent->vy_mps = (fp32)cmd->vy_cmps * 0.01f;
    intent->wz_radps = (fp32)cmd->wz_mradps * 0.001f;
    intent->yaw_offset_rad = (fp32)cmd->yaw_offset_mrad * 0.001f;
    intent->ax_mps2 = (fp32)cmd->ax_cmps2 * 0.01f;
    intent->ay_mps2 = (fp32)cmd->ay_cmps2 * 0.01f;
    intent->wz_acc_radps2 = (fp32)cmd->wz_acc_mradps2 * 0.001f;
}

static int16_t VisionLinkFp32ToI16Scaled(fp32 value, fp32 scale)
{
    fp32 scaled;

    if (!(value == value))
    {
        return 0;
    }

    scaled = value * scale;
    if (scaled > 32767.0f)
    {
        return 32767;
    }
    if (scaled < -32768.0f)
    {
        return -32768;
    }
    return (int16_t)scaled;
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
        VisionLinkDbg.rx_stream_overflow_count++;
    }

    if ((VisionLinkRxStreamLen + len) > VISION_RX_STREAM_BUF_SIZE)
    {
        const uint32_t overflow = (VisionLinkRxStreamLen + len) - VISION_RX_STREAM_BUF_SIZE;
        VisionLinkDbg.rx_stream_overflow_count++;
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

        if (candidate[0] == VISION_FRAME_SOF0 && candidate[1] == VISION_FRAME_SOF1)
        {
            VisionLinkDbg.ls_head_count++;
            if ((VisionLinkRxStreamLen - consume) < VISION_RX_FRAME_SIZE)
            {
                break;
            }
            if (verify_CRC16_check_sum((uint8_t *)candidate, VISION_RX_FRAME_SIZE))
            {
                VisionToGimbal pkt;
                VisionLinkDbg.ls_crc_ok_count++;
                memcpy(&pkt, candidate, sizeof(pkt));
                VisionLinkHandleRx(&pkt);
                consume += VISION_RX_FRAME_SIZE;
                continue;
            }
            VisionLinkDbg.ls_crc_fail_count++;
            consume++;
            continue;
        }

        if (candidate[0] == CHASSIS_FRAME_SOF0 && candidate[1] == CHASSIS_FRAME_SOF1)
        {
            VisionLinkDbg.lc_head_count++;
            if ((VisionLinkRxStreamLen - consume) < CHASSIS_RX_FRAME_SIZE)
            {
                break;
            }
            VisionLinkDbg.lc_last_crc_rx = VisionLinkReadLe16(&candidate[CHASSIS_RX_FRAME_SIZE - 2u]);
            VisionLinkDbg.lc_last_crc_calc = VisionLinkCalcCrc16(candidate, CHASSIS_RX_FRAME_SIZE);
            if (verify_CRC16_check_sum((uint8_t *)candidate, CHASSIS_RX_FRAME_SIZE))
            {
                VisionToChassis cmd;
                VisionLinkDbg.lc_crc_ok_count++;
                memcpy(&cmd, candidate, sizeof(cmd));
                VisionLinkHandleChassisCmd(&cmd);
                consume += CHASSIS_RX_FRAME_SIZE;
                continue;
            }
            VisionLinkDbg.lc_crc_fail_count++;
            consume++;
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

static uint32_t VisionLinkNowFromIsrMs(void)
{
    return (uint32_t)(xTaskGetTickCountFromISR() * portTICK_PERIOD_MS);
}

static uint16_t VisionLinkReadLe16(const uint8_t *buf)
{
    if (buf == NULL)
    {
        return 0u;
    }

    return (uint16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
}

static uint16_t VisionLinkCalcCrc16(const uint8_t *buf, uint32_t len)
{
    if (buf == NULL || len < 2u)
    {
        return 0u;
    }

    return get_CRC16_check_sum((uint8_t *)buf, len - 2u, 0xffffu);
}

static void VisionLinkDebugStoreChassisCmd(const VisionToChassis *cmd)
{
    if (cmd == NULL)
    {
        return;
    }

    VisionLinkDbg.lc_last_tick_ms = VisionLinkNowFromIsrMs();
    VisionLinkDbg.lc_last_mode = cmd->mode;
    VisionLinkDbg.lc_last_frame = cmd->frame;
    VisionLinkDbg.lc_last_flags = cmd->flags;
    VisionLinkDbg.lc_last_timeout_10ms = cmd->timeout_10ms;
    VisionLinkDbg.lc_last_vx_cmps = cmd->vx_cmps;
    VisionLinkDbg.lc_last_vy_cmps = cmd->vy_cmps;
    VisionLinkDbg.lc_last_wz_mradps = cmd->wz_mradps;
    VisionLinkDbg.lc_last_yaw_offset_mrad = cmd->yaw_offset_mrad;
    VisionLinkDbg.lc_last_ax_cmps2 = cmd->ax_cmps2;
    VisionLinkDbg.lc_last_ay_cmps2 = cmd->ay_cmps2;
    VisionLinkDbg.lc_last_wz_acc_mradps2 = cmd->wz_acc_mradps2;
}
