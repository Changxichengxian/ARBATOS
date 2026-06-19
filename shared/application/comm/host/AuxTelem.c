/*
 * SPDX-FileCopyrightText: 2026 Chen Xuan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "AuxTelem.h"

#include <string.h>
#include "arm_math.h"
#include "FreeRTOS.h"
#include "task.h"
#include "CanReceive.h"
#include "InsTask.h"
#include "LowCmd.h"
#include "AuxPort.h"
#include "AuxTune.h"
#include "BatteryMonitorTask.h"
#include "BspKey.h"
#include "BspTime.h"
#include "BspUsart.h"
#include "ChassisControlTask.h"
#include "RobotConfig.h"
#include "DetectTask.h"
#include "GimbalBehaviour.h"
#include "GimbalControlTask.h"
#include "GimbalState.h"
#include "HostTuneBridge.h"
#include "ManualInput.h"
#include "MemMang.h"
#include "MotorConfig.h"
#include "MotorInst.h"
#include "RobotTaskProfile.h"
#include "ShootState.h"
#include "UserLib.h"

#define AUX_TELEM_AUTO_EXTRA_BACKOFF_PCT 50u

typedef struct
{
    ManualInputState rc_copy;
    const ManualInputState *rc;
    const fp32 *quat;
    const fp32 *angle;
    const fp32 *gyro;
    const fp32 *accel;
    const GimbalMotor *yaw;
    const GimbalMotor *pitch;
    const ChassisMove *chassis;
    ShootState shoot;
    uint8_t ShootValid;
    const motor_measure_t *trigger_meas;
    const motor_measure_t *fric_meas[SHOOT_STATE_FRIC_MOTOR_COUNT];
} AuxTelemCtx;

typedef struct
{
    const char *name;
    MotorId fallback_id;
    MotorId resolved_id;
    uint8_t ready;
} AuxTelemMotorIdCache;

static AuxTelemMotorIdCache s_aux_telem_chassis_ids[4u] = {
    {"motor.chassis0", Motor0, MotorCount, 0u},
    {"motor.chassis1", Motor1, MotorCount, 0u},
    {"motor.chassis2", Motor2, MotorCount, 0u},
    {"motor.chassis3", Motor3, MotorCount, 0u},
};
static AuxTelemMotorIdCache s_aux_telem_friction_ids[4u] = {
    {"motor.friction0", Motor8, MotorCount, 0u},
    {"motor.friction1", Motor9, MotorCount, 0u},
    {"motor.friction2", Motor10, MotorCount, 0u},
    {"motor.friction3", Motor11, MotorCount, 0u},
};
static AuxTelemMotorIdCache s_aux_telem_yaw_id = {"motor.yaw", Motor4, MotorCount, 0u};
static AuxTelemMotorIdCache s_aux_telem_pitch_id = {"motor.pitch", Motor6, MotorCount, 0u};
static AuxTelemMotorIdCache s_aux_telem_trigger_id = {"motor.trigger", Motor7, MotorCount, 0u};

static void AuxTelemPrepareMotorIdCache(AuxTelemMotorIdCache *cache)
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

static MotorId AuxTelemCachedMotorId(const AuxTelemMotorIdCache *cache)
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

static MotorId AuxTelemCachedMotorIdAt(const AuxTelemMotorIdCache *cache,
                                            uint8_t count,
                                            uint8_t index)
{
    if (cache == NULL || count == 0u)
    {
        return MotorCount;
    }
    return AuxTelemCachedMotorId(&cache[(uint8_t)(index % count)]);
}

void AuxTelemPrepareMotorIds(void)
{
    for (uint8_t i = 0u; i < 4u; i++)
    {
        AuxTelemPrepareMotorIdCache(&s_aux_telem_chassis_ids[i]);
        AuxTelemPrepareMotorIdCache(&s_aux_telem_friction_ids[i]);
    }
    AuxTelemPrepareMotorIdCache(&s_aux_telem_yaw_id);
    AuxTelemPrepareMotorIdCache(&s_aux_telem_pitch_id);
    AuxTelemPrepareMotorIdCache(&s_aux_telem_trigger_id);
}

static const AuxTelemSig AuxTelemDefaultList[] =
{
    AUX_TELEM_SIG_SYS_TICK_MS,
    AUX_TELEM_SIG_SYS_AUX_CMD_SEQ,
    AUX_TELEM_SIG_SYS_BATTERY_VOLT,
    AUX_TELEM_SIG_SYS_BATTERY_PERCENT,
    AUX_TELEM_SIG_RC_CH0,
    AUX_TELEM_SIG_RC_CH1,
    AUX_TELEM_SIG_RC_CH2,
    AUX_TELEM_SIG_RC_CH3,
    AUX_TELEM_SIG_RC_CH4,
    AUX_TELEM_SIG_RC_S0,
    AUX_TELEM_SIG_RC_S1,
    AUX_TELEM_SIG_RC_MOUSE_X,
    AUX_TELEM_SIG_RC_MOUSE_Y,
    AUX_TELEM_SIG_RC_MOUSE_Z,
    AUX_TELEM_SIG_RC_MOUSE_L,
    AUX_TELEM_SIG_RC_MOUSE_R,
    AUX_TELEM_SIG_RC_KEY,
    AUX_TELEM_SIG_RC_ERROR,
    AUX_TELEM_SIG_IMU_Q0,
    AUX_TELEM_SIG_IMU_Q1,
    AUX_TELEM_SIG_IMU_Q2,
    AUX_TELEM_SIG_IMU_Q3,
    AUX_TELEM_SIG_IMU_ANGLE_YAW_RAD,
    AUX_TELEM_SIG_IMU_ANGLE_ROLL_RAD,
    AUX_TELEM_SIG_IMU_ANGLE_PITCH_RAD,
    AUX_TELEM_SIG_IMU_ANGLE_YAW_DEG,
    AUX_TELEM_SIG_IMU_ANGLE_ROLL_DEG,
    AUX_TELEM_SIG_IMU_ANGLE_PITCH_DEG,
    AUX_TELEM_SIG_IMU_GYRO_X_RAD_S,
    AUX_TELEM_SIG_IMU_GYRO_Y_RAD_S,
    AUX_TELEM_SIG_IMU_GYRO_Z_RAD_S,
    AUX_TELEM_SIG_IMU_GYRO_X_DPS,
    AUX_TELEM_SIG_IMU_GYRO_Y_DPS,
    AUX_TELEM_SIG_IMU_GYRO_Z_DPS,
    AUX_TELEM_SIG_IMU_ACCEL_X,
    AUX_TELEM_SIG_IMU_ACCEL_Y,
    AUX_TELEM_SIG_IMU_ACCEL_Z,
    AUX_TELEM_SIG_PACK_MODE,
    AUX_TELEM_SIG_GIMBAL_YAW_ANGLE,
    AUX_TELEM_SIG_GIMBAL_YAW_ANGLE_SET,
    AUX_TELEM_SIG_GIMBAL_YAW_GYRO,
    AUX_TELEM_SIG_GIMBAL_YAW_GYRO_SET,
    AUX_TELEM_SIG_GIMBAL_YAW_MOTOR_SPEED,
    AUX_TELEM_SIG_GIMBAL_YAW_CURRENT_SET,
    AUX_TELEM_SIG_GIMBAL_YAW_GIVEN_CURRENT,
    AUX_TELEM_SIG_GIMBAL_YAW_RAW_CMD_CURRENT,
    AUX_TELEM_SIG_GIMBAL_YAW_ECD,
    AUX_TELEM_SIG_GIMBAL_YAW_OFFSET_ECD,
    AUX_TELEM_SIG_GIMBAL_YAW_RPM,
    AUX_TELEM_SIG_GIMBAL_YAW_CURRENT_FB,
    AUX_TELEM_SIG_GIMBAL_YAW_TEMP,
    AUX_TELEM_SIG_GIMBAL_PITCH_ANGLE,
    AUX_TELEM_SIG_GIMBAL_PITCH_ANGLE_SET,
    AUX_TELEM_SIG_GIMBAL_PITCH_GYRO,
    AUX_TELEM_SIG_GIMBAL_PITCH_GYRO_SET,
    AUX_TELEM_SIG_GIMBAL_PITCH_MOTOR_SPEED,
    AUX_TELEM_SIG_GIMBAL_PITCH_CURRENT_SET,
    AUX_TELEM_SIG_GIMBAL_PITCH_GIVEN_CURRENT,
    AUX_TELEM_SIG_GIMBAL_PITCH_RAW_CMD_CURRENT,
    AUX_TELEM_SIG_GIMBAL_PITCH_ECD,
    AUX_TELEM_SIG_GIMBAL_PITCH_OFFSET_ECD,
    AUX_TELEM_SIG_GIMBAL_PITCH_RPM,
    AUX_TELEM_SIG_GIMBAL_PITCH_CURRENT_FB,
    AUX_TELEM_SIG_GIMBAL_PITCH_TEMP,
    AUX_TELEM_SIG_GIMBAL_YAW_ANGLE_PID_SET,
    AUX_TELEM_SIG_GIMBAL_YAW_ANGLE_PID_GET,
    AUX_TELEM_SIG_GIMBAL_YAW_ANGLE_PID_POUT,
    AUX_TELEM_SIG_GIMBAL_YAW_ANGLE_PID_IOUT,
    AUX_TELEM_SIG_GIMBAL_YAW_ANGLE_PID_DOUT,
    AUX_TELEM_SIG_GIMBAL_YAW_ANGLE_PID_OUT,
    AUX_TELEM_SIG_GIMBAL_PITCH_ANGLE_PID_SET,
    AUX_TELEM_SIG_GIMBAL_PITCH_ANGLE_PID_GET,
    AUX_TELEM_SIG_GIMBAL_PITCH_ANGLE_PID_POUT,
    AUX_TELEM_SIG_GIMBAL_PITCH_ANGLE_PID_IOUT,
    AUX_TELEM_SIG_GIMBAL_PITCH_ANGLE_PID_DOUT,
    AUX_TELEM_SIG_GIMBAL_PITCH_ANGLE_PID_OUT,
    AUX_TELEM_SIG_GIMBAL_YAW_SPEED_PID_SET,
    AUX_TELEM_SIG_GIMBAL_YAW_SPEED_PID_FDB,
    AUX_TELEM_SIG_GIMBAL_YAW_SPEED_PID_DBUF0,
    AUX_TELEM_SIG_GIMBAL_YAW_SPEED_PID_POUT,
    AUX_TELEM_SIG_GIMBAL_YAW_SPEED_PID_IOUT,
    AUX_TELEM_SIG_GIMBAL_YAW_SPEED_PID_DOUT,
    AUX_TELEM_SIG_GIMBAL_YAW_SPEED_PID_OUT,
    AUX_TELEM_SIG_GIMBAL_PITCH_SPEED_PID_SET,
    AUX_TELEM_SIG_GIMBAL_PITCH_SPEED_PID_FDB,
    AUX_TELEM_SIG_GIMBAL_PITCH_SPEED_PID_DBUF0,
    AUX_TELEM_SIG_GIMBAL_PITCH_SPEED_PID_POUT,
    AUX_TELEM_SIG_GIMBAL_PITCH_SPEED_PID_IOUT,
    AUX_TELEM_SIG_GIMBAL_PITCH_SPEED_PID_DOUT,
    AUX_TELEM_SIG_GIMBAL_PITCH_SPEED_PID_OUT,
    AUX_TELEM_SIG_CHASSIS_VX_SET,
    AUX_TELEM_SIG_CHASSIS_VY_SET,
    AUX_TELEM_SIG_CHASSIS_WZ_SET,
    AUX_TELEM_SIG_CHASSIS_VX,
    AUX_TELEM_SIG_CHASSIS_VY,
    AUX_TELEM_SIG_CHASSIS_WZ,
    AUX_TELEM_SIG_CHASSIS_YAW_OFFSET,
    AUX_TELEM_SIG_CHASSIS_YAW_OFFSET_SET,
    AUX_TELEM_SIG_CHASSIS_YAW_SET,
    AUX_TELEM_SIG_CHASSIS_YAW,
    AUX_TELEM_SIG_CHASSIS_PITCH,
    AUX_TELEM_SIG_CHASSIS_ROLL,
    AUX_TELEM_SIG_CHASSIS_SWING_KEY,
    AUX_TELEM_SIG_CHASSIS_M0_RPM,
    AUX_TELEM_SIG_CHASSIS_M0_CURRENT_CMD,
    AUX_TELEM_SIG_CHASSIS_M0_CURRENT_FB,
    AUX_TELEM_SIG_CHASSIS_M0_SPEED_SET,
    AUX_TELEM_SIG_CHASSIS_M0_SPEED,
    AUX_TELEM_SIG_CHASSIS_M0_ACCEL,
    AUX_TELEM_SIG_CHASSIS_M0_ECD,
    AUX_TELEM_SIG_CHASSIS_M0_TEMP,
    AUX_TELEM_SIG_CHASSIS_M1_RPM,
    AUX_TELEM_SIG_CHASSIS_M1_CURRENT_CMD,
    AUX_TELEM_SIG_CHASSIS_M1_CURRENT_FB,
    AUX_TELEM_SIG_CHASSIS_M1_SPEED_SET,
    AUX_TELEM_SIG_CHASSIS_M1_SPEED,
    AUX_TELEM_SIG_CHASSIS_M1_ACCEL,
    AUX_TELEM_SIG_CHASSIS_M1_ECD,
    AUX_TELEM_SIG_CHASSIS_M1_TEMP,
    AUX_TELEM_SIG_CHASSIS_M2_RPM,
    AUX_TELEM_SIG_CHASSIS_M2_CURRENT_CMD,
    AUX_TELEM_SIG_CHASSIS_M2_CURRENT_FB,
    AUX_TELEM_SIG_CHASSIS_M2_SPEED_SET,
    AUX_TELEM_SIG_CHASSIS_M2_SPEED,
    AUX_TELEM_SIG_CHASSIS_M2_ACCEL,
    AUX_TELEM_SIG_CHASSIS_M2_ECD,
    AUX_TELEM_SIG_CHASSIS_M2_TEMP,
    AUX_TELEM_SIG_CHASSIS_M3_RPM,
    AUX_TELEM_SIG_CHASSIS_M3_CURRENT_CMD,
    AUX_TELEM_SIG_CHASSIS_M3_CURRENT_FB,
    AUX_TELEM_SIG_CHASSIS_M3_SPEED_SET,
    AUX_TELEM_SIG_CHASSIS_M3_SPEED,
    AUX_TELEM_SIG_CHASSIS_M3_ACCEL,
    AUX_TELEM_SIG_CHASSIS_M3_ECD,
    AUX_TELEM_SIG_CHASSIS_M3_TEMP,
    AUX_TELEM_SIG_CHASSIS_FOLLOW_PID_SET,
    AUX_TELEM_SIG_CHASSIS_FOLLOW_PID_FDB,
    AUX_TELEM_SIG_CHASSIS_FOLLOW_PID_DBUF0,
    AUX_TELEM_SIG_CHASSIS_FOLLOW_PID_POUT,
    AUX_TELEM_SIG_CHASSIS_FOLLOW_PID_IOUT,
    AUX_TELEM_SIG_CHASSIS_FOLLOW_PID_DOUT,
    AUX_TELEM_SIG_CHASSIS_FOLLOW_PID_OUT,
    AUX_TELEM_SIG_CHASSIS_M0_SPD_PID_SET,
    AUX_TELEM_SIG_CHASSIS_M0_SPD_PID_FDB,
    AUX_TELEM_SIG_CHASSIS_M0_SPD_PID_DBUF0,
    AUX_TELEM_SIG_CHASSIS_M0_SPD_PID_POUT,
    AUX_TELEM_SIG_CHASSIS_M0_SPD_PID_IOUT,
    AUX_TELEM_SIG_CHASSIS_M0_SPD_PID_DOUT,
    AUX_TELEM_SIG_CHASSIS_M0_SPD_PID_OUT,
    AUX_TELEM_SIG_CHASSIS_M1_SPD_PID_SET,
    AUX_TELEM_SIG_CHASSIS_M1_SPD_PID_FDB,
    AUX_TELEM_SIG_CHASSIS_M1_SPD_PID_DBUF0,
    AUX_TELEM_SIG_CHASSIS_M1_SPD_PID_POUT,
    AUX_TELEM_SIG_CHASSIS_M1_SPD_PID_IOUT,
    AUX_TELEM_SIG_CHASSIS_M1_SPD_PID_DOUT,
    AUX_TELEM_SIG_CHASSIS_M1_SPD_PID_OUT,
    AUX_TELEM_SIG_CHASSIS_M2_SPD_PID_SET,
    AUX_TELEM_SIG_CHASSIS_M2_SPD_PID_FDB,
    AUX_TELEM_SIG_CHASSIS_M2_SPD_PID_DBUF0,
    AUX_TELEM_SIG_CHASSIS_M2_SPD_PID_POUT,
    AUX_TELEM_SIG_CHASSIS_M2_SPD_PID_IOUT,
    AUX_TELEM_SIG_CHASSIS_M2_SPD_PID_DOUT,
    AUX_TELEM_SIG_CHASSIS_M2_SPD_PID_OUT,
    AUX_TELEM_SIG_CHASSIS_M3_SPD_PID_SET,
    AUX_TELEM_SIG_CHASSIS_M3_SPD_PID_FDB,
    AUX_TELEM_SIG_CHASSIS_M3_SPD_PID_DBUF0,
    AUX_TELEM_SIG_CHASSIS_M3_SPD_PID_POUT,
    AUX_TELEM_SIG_CHASSIS_M3_SPD_PID_IOUT,
    AUX_TELEM_SIG_CHASSIS_M3_SPD_PID_DOUT,
    AUX_TELEM_SIG_CHASSIS_M3_SPD_PID_OUT,
    AUX_TELEM_SIG_SHOOT_FRIC_SPEED_SET,
    AUX_TELEM_SIG_SHOOT_TRIGGER_SPEED_SET,
    AUX_TELEM_SIG_SHOOT_TRIGGER_SPEED,
    AUX_TELEM_SIG_SHOOT_TRIGGER_ANGLE,
    AUX_TELEM_SIG_SHOOT_TRIGGER_ANGLE_SET,
    AUX_TELEM_SIG_SHOOT_TRIGGER_GIVEN_CURRENT,
    AUX_TELEM_SIG_SHOOT_TRIGGER_ECD_COUNT,
    AUX_TELEM_SIG_SHOOT_PRESS_L,
    AUX_TELEM_SIG_SHOOT_PRESS_R,
    AUX_TELEM_SIG_SHOOT_KEY,
    AUX_TELEM_SIG_SHOOT_HEAT_LIMIT,
    AUX_TELEM_SIG_SHOOT_HEAT,
    AUX_TELEM_SIG_SHOOT_FRIC0_RPM,
    AUX_TELEM_SIG_SHOOT_FRIC0_CURRENT_FB,
    AUX_TELEM_SIG_SHOOT_FRIC0_TEMP,
    AUX_TELEM_SIG_SHOOT_FRIC0_CURRENT_CMD,
    AUX_TELEM_SIG_SHOOT_FRIC1_RPM,
    AUX_TELEM_SIG_SHOOT_FRIC1_CURRENT_FB,
    AUX_TELEM_SIG_SHOOT_FRIC1_TEMP,
    AUX_TELEM_SIG_SHOOT_FRIC1_CURRENT_CMD,
    AUX_TELEM_SIG_SHOOT_FRIC2_RPM,
    AUX_TELEM_SIG_SHOOT_FRIC2_CURRENT_FB,
    AUX_TELEM_SIG_SHOOT_FRIC2_TEMP,
    AUX_TELEM_SIG_SHOOT_FRIC2_CURRENT_CMD,
    AUX_TELEM_SIG_SHOOT_FRIC3_RPM,
    AUX_TELEM_SIG_SHOOT_FRIC3_CURRENT_FB,
    AUX_TELEM_SIG_SHOOT_FRIC3_TEMP,
    AUX_TELEM_SIG_SHOOT_FRIC3_CURRENT_CMD,
    AUX_TELEM_SIG_SHOOT_TRIGGER_RPM,
    AUX_TELEM_SIG_SHOOT_TRIGGER_ECD,
    AUX_TELEM_SIG_SHOOT_TRIGGER_TEMP,
    AUX_TELEM_SIG_SHOOT_TRIGGER_CURRENT_FB,
    AUX_TELEM_SIG_DIAG_ACTUATOR_CHASSIS0_CURRENT,
    AUX_TELEM_SIG_DIAG_ACTUATOR_CHASSIS1_CURRENT,
    AUX_TELEM_SIG_DIAG_ACTUATOR_PITCH_CURRENT,
    AUX_TELEM_SIG_DIAG_ACTUATOR_TRIGGER_CURRENT,
    AUX_TELEM_SIG_DIAG_ACTUATOR_CHASSIS3_CURRENT,
    AUX_TELEM_SIG_DIAG_ACTUATOR_YAW_CURRENT,
    AUX_TELEM_SIG_DIAG_ACTUATOR_CHASSIS2_CURRENT,
    AUX_TELEM_SIG_DIAG_RM_GROUP_1FF_STATUS,
    AUX_TELEM_SIG_DIAG_CAN_BUS1_ERR,
    AUX_TELEM_SIG_PACK_OFFLINE,
    AUX_TELEM_SIG_DIAG_ZERO_FORCE,
    AUX_TELEM_SIG_SHOOT_TRIGGER_PID_IOUT,
    AUX_TELEM_SIG_SHOOT_TRIGGER_PID_OUT,
    AUX_TELEM_SIG_MEM_HEAP_FREE,
    AUX_TELEM_SIG_MEM_HEAP_EVER_FREE,
    AUX_TELEM_SIG_BOARD_KEY_DOWN,
    AUX_TELEM_SIG_BOARD_KEY_PRESS_CNT,
};

typedef char _check_aux_telem_default_list_fits[(sizeof(AuxTelemDefaultList) / sizeof(AuxTelemDefaultList[0]) <= AUX_TELEM_MAX_CH) ? 1 : -1];

static uint16_t AuxTelemMinPeriodMs(uint16_t channel_num);
static AuxTelemSig AuxTelemSignalAt(const AuxTelemConfig *cfg,
                                           uint8_t use_default_list,
                                           uint16_t index);
static bool_t AuxTelemSignalIsActive(AuxTelemSig sig);
static fp32 AuxTelemGetValue(const AuxTelemCtx *ctx, AuxTelemSig sig);

static const fp32 *ins_quat;
static const fp32 *ins_angle;
static const fp32 *ins_gyro;
static const fp32 *ins_accel;
static uint32_t AuxTelemTick = 0u;
static uint8_t AuxTelemFrame[(AUX_TELEM_MAX_CH + 1u) * 4u];

__weak fp32 BatteryVoltage = 0.0f;
__weak fp32 electricity_percentage = 0.0f;

void AuxTelemSetInsSources(const fp32 *quat, const fp32 *angle, const fp32 *gyro, const fp32 *accel)
{
    AuxTelemPrepareMotorIds();
    ins_quat = quat;
    ins_angle = angle;
    ins_gyro = gyro;
    ins_accel = accel;
}

void AuxTelemReset(void)
{
    AuxTelemPrepareMotorIds();
    AuxTelemTick = 0u;
}

void AuxTelemTrySendFrame(void)
{
    const AuxTelemConfig *cfg = &g_config.AuxTelem;
    const uint8_t want_uart_justfloat = ((cfg->enable == 1u) || (cfg->enable == 4u)) ? 1u : 0u;
    // Aux telemetry only runs in the dedicated tuning mode.
    const uint8_t AuxCanTx = AuxPortIsTuneMode(BspAuxLinkGetBaudrate());
    if (!(want_uart_justfloat && AuxCanTx))
    {
        return;
    }

    // Telemetry list selection:
    // - channel_num == 0: use built-in default list.
    // - channel_num != 0: send the first channel_num entries in channel_map[].
    uint8_t use_default_list = 0u;
    uint16_t channel_num = cfg->channel_num;
    uint16_t active_channel_num = 0u;
    if (channel_num == 0u)
    {
        use_default_list = 1u;
        channel_num = (uint16_t)(sizeof(AuxTelemDefaultList) / sizeof(AuxTelemDefaultList[0]));
    }
    if (channel_num > AUX_TELEM_MAX_CH)
    {
        channel_num = AUX_TELEM_MAX_CH;
    }
    if (channel_num == 0u)
    {
        return;
    }

    for (uint16_t i = 0u; i < channel_num; i++)
    {
        const AuxTelemSig sig = AuxTelemSignalAt(cfg, use_default_list, i);
        if (AuxTelemSignalIsActive(sig))
        {
            active_channel_num++;
        }
    }
    if (active_channel_num == 0u)
    {
        return;
    }

    uint16_t period_ms = cfg->period_ms;
    const uint16_t min_period_ms = AuxTelemMinPeriodMs(active_channel_num);
    if (period_ms == 0u)
    {
        uint32_t auto_ms = ((uint32_t)min_period_ms * (100u + AUX_TELEM_AUTO_EXTRA_BACKOFF_PCT) + 99u) / 100u;
        if (auto_ms < 1u)
        {
            auto_ms = 1u;
        }
        if (auto_ms > 1000u)
        {
            auto_ms = 1000u;
        }
        period_ms = (uint16_t)auto_ms;
    }
    else if (period_ms < min_period_ms)
    {
        period_ms = min_period_ms;
    }

    const uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    if ((uint32_t)(now - AuxTelemTick) < period_ms)
    {
        return;
    }

    // If TX DMA is busy, skip (non-blocking).
    if (!BspAuxLinkTxReady())
    {
        return;
    }

    AuxTelemCtx ctx = {0};
    if (ManualInputGetCurrentCopy(&ctx.rc_copy) != 0u)
    {
        ctx.rc = &ctx.rc_copy;
    }
    ctx.quat = ins_quat;
    ctx.angle = ins_angle;
    ctx.gyro = ins_gyro;
    ctx.accel = ins_accel;
    ctx.yaw = get_yaw_motor_point();
    ctx.pitch = get_pitch_motor_point();
    ctx.chassis = get_chassis_move_point();
    ctx.ShootValid = ShootStateRead(&ctx.shoot);
    ctx.trigger_meas = get_trigger_motor_measure_point();
    for (uint8_t i = 0; i < SHOOT_STATE_FRIC_MOTOR_COUNT; i++)
    {
        ctx.fric_meas[i] = get_friction_motor_measure_point(i);
    }

    uint16_t out_channel_num = 0u;
    for (uint16_t i = 0u; i < channel_num; i++)
    {
        const AuxTelemSig sig = AuxTelemSignalAt(cfg, use_default_list, i);
        if (!AuxTelemSignalIsActive(sig))
        {
            continue;
        }
        const fp32 v = AuxTelemGetValue(&ctx, sig);
        memcpy(&AuxTelemFrame[out_channel_num * 4u], &v, 4u);
        out_channel_num++;
    }

    const uint32_t tail = 0x7F800000u;
    memcpy(&AuxTelemFrame[out_channel_num * 4u], &tail, 4u);

    const uint16_t frame_len = (uint16_t)((out_channel_num + 1u) * 4u);
    if (BspAuxLinkTxDma(AuxTelemFrame, frame_len) == 0)
    {
        AuxTelemTick = now;
    }
}
static uint16_t AuxTelemMinPeriodMs(uint16_t channel_num)
{
    const uint32_t baud = BspAuxLinkGetBaudrate();
    if (baud == 0u)
    {
        return 100u;
    }

    const uint32_t bytes = ((uint32_t)channel_num + 1u) * 4u;
    const uint32_t bits = bytes * 10u; // 8N1 -> 10 bits per byte
    uint32_t ms = (bits * 1000u + baud - 1u) / baud;
    // Add 25% backoff and round up to reduce drop risk at high utilization.
    ms = (ms * 125u + 99u) / 100u;
    if (ms < 1u)
    {
        ms = 1u;
    }
    if (ms > 1000u)
    {
        ms = 1000u;
    }
    return (uint16_t)ms;
}

static AuxTelemSig AuxTelemSignalAt(const AuxTelemConfig *cfg,
                                           uint8_t use_default_list,
                                           uint16_t index)
{
    if (use_default_list)
    {
        return AuxTelemDefaultList[index];
    }

    if (cfg == NULL || cfg->channel_map[index] >= (uint16_t)AUX_TELEM_SIG__COUNT)
    {
        return AUX_TELEM_SIG_SYS_TICK_MS;
    }

    return (AuxTelemSig)cfg->channel_map[index];
}

static bool_t AuxTelemSignalIsActive(AuxTelemSig sig)
{
    const uint8_t classic_chassis_on = RobotProfileNeedClassicChassisControlTask();
    const uint8_t GimbalOn = (uint8_t)(RobotProfileNeedSingleGimbalControlTask() ||
                                        RobotProfileNeedDualGimbalControlTask());
    const uint8_t trigger_on = (uint8_t)(MotorCfgNodeId(&g_config.motor.trigger) != 0u);
    uint8_t friction_on = 0u;
    for (uint8_t i = 0u; i < SHOOT_STATE_FRIC_MOTOR_COUNT; i++)
    {
        if (MotorCfgNodeId(&g_config.motor.friction[i]) != 0u)
        {
            friction_on = 1u;
            break;
        }
    }
    const uint8_t ShootOn = (uint8_t)((trigger_on != 0u || friction_on != 0u) ? 1u : 0u);

    if ((sig >= AUX_TELEM_SIG_GIMBAL_YAW_ANGLE && sig <= AUX_TELEM_SIG_GIMBAL_PITCH_TEMP) ||
        (sig >= AUX_TELEM_SIG_GIMBAL_YAW_ANGLE_PID_SET && sig <= AUX_TELEM_SIG_GIMBAL_PITCH_SPEED_PID_OUT) ||
        sig == AUX_TELEM_SIG_DIAG_ACTUATOR_YAW_CURRENT ||
        sig == AUX_TELEM_SIG_DIAG_ACTUATOR_PITCH_CURRENT ||
        sig == AUX_TELEM_SIG_DIAG_ZERO_FORCE)
    {
        return (bool_t)GimbalOn;
    }

    if ((sig >= AUX_TELEM_SIG_CHASSIS_VX_SET && sig <= AUX_TELEM_SIG_CHASSIS_M3_SPD_PID_OUT) ||
        sig == AUX_TELEM_SIG_DIAG_ACTUATOR_CHASSIS0_CURRENT ||
        sig == AUX_TELEM_SIG_DIAG_ACTUATOR_CHASSIS1_CURRENT ||
        sig == AUX_TELEM_SIG_DIAG_ACTUATOR_CHASSIS2_CURRENT ||
        sig == AUX_TELEM_SIG_DIAG_ACTUATOR_CHASSIS3_CURRENT)
    {
        return (bool_t)classic_chassis_on;
    }

    if (sig == AUX_TELEM_SIG_SHOOT_FRIC_SPEED_SET ||
        (sig >= AUX_TELEM_SIG_SHOOT_FRIC0_RPM && sig <= AUX_TELEM_SIG_SHOOT_FRIC3_CURRENT_CMD))
    {
        return (bool_t)friction_on;
    }

    if ((sig >= AUX_TELEM_SIG_SHOOT_TRIGGER_SPEED_SET && sig <= AUX_TELEM_SIG_SHOOT_TRIGGER_ECD_COUNT) ||
        (sig >= AUX_TELEM_SIG_SHOOT_TRIGGER_RPM && sig <= AUX_TELEM_SIG_SHOOT_TRIGGER_CURRENT_FB) ||
        sig == AUX_TELEM_SIG_DIAG_ACTUATOR_TRIGGER_CURRENT ||
        sig == AUX_TELEM_SIG_SHOOT_TRIGGER_PID_IOUT ||
        sig == AUX_TELEM_SIG_SHOOT_TRIGGER_PID_OUT)
    {
        return (bool_t)trigger_on;
    }

    if (sig == AUX_TELEM_SIG_SHOOT_PRESS_L ||
        sig == AUX_TELEM_SIG_SHOOT_PRESS_R ||
        sig == AUX_TELEM_SIG_SHOOT_KEY ||
        sig == AUX_TELEM_SIG_SHOOT_HEAT_LIMIT ||
        sig == AUX_TELEM_SIG_SHOOT_HEAT)
    {
        return (bool_t)ShootOn;
    }

    return 1;
}

static fp32 AuxTelemPidField(const PidTypeDef *pid, uint8_t field)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    switch (field)
    {
    case 0:  return pid->Kp;
    case 1:  return pid->Ki;
    case 2:  return pid->Kd;
    case 3:  return pid->max_out;
    case 4:  return pid->max_iout;
    case 5:  return pid->set;
    case 6:  return pid->fdb;
    case 7:  return pid->error[0];
    case 8:  return pid->error[1];
    case 9:  return pid->Dbuf[0];
    case 10: return pid->Pout;
    case 11: return pid->Iout;
    case 12: return pid->Dout;
    case 13: return pid->out;
    default: return 0.0f;
    }
}

static fp32 AuxTelemGimbalPidField(const GimbalPid *pid, uint8_t field)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    switch (field)
    {
    case 0:  return pid->kp;
    case 1:  return pid->ki;
    case 2:  return pid->kd;
    case 3:  return pid->set;
    case 4:  return pid->get;
    case 5:  return pid->err;
    case 6:  return pid->max_out;
    case 7:  return pid->max_iout;
    case 8:  return pid->Pout;
    case 9:  return pid->Iout;
    case 10: return pid->Dout;
    case 11: return pid->out;
    default: return 0.0f;
    }
}

static fp32 AuxTelemGetValue(const AuxTelemCtx *ctx, AuxTelemSig sig)
{
    if (ctx == NULL)
    {
        return 0.0f;
    }

    // Decode repeated groups to keep the switch small.
    if (sig >= AUX_TELEM_SIG_GIMBAL_YAW_ANGLE_PID_SET && sig <= AUX_TELEM_SIG_GIMBAL_YAW_ANGLE_PID_OUT)
    {
        static const uint8_t s_gimbal_angle_pid_map[] = {3u, 4u, 8u, 9u, 10u, 11u}; // set/get/p/i/d/out
        const uint8_t off = (uint8_t)(sig - AUX_TELEM_SIG_GIMBAL_YAW_ANGLE_PID_SET);
        if (off < (uint8_t)(sizeof(s_gimbal_angle_pid_map) / sizeof(s_gimbal_angle_pid_map[0])))
        {
            return AuxTelemGimbalPidField(ctx->yaw ? &ctx->yaw->GimbalMotorAnglePid : NULL, s_gimbal_angle_pid_map[off]);
        }
        return 0.0f;
    }
    if (sig >= AUX_TELEM_SIG_GIMBAL_PITCH_ANGLE_PID_SET && sig <= AUX_TELEM_SIG_GIMBAL_PITCH_ANGLE_PID_OUT)
    {
        static const uint8_t s_gimbal_angle_pid_map[] = {3u, 4u, 8u, 9u, 10u, 11u}; // set/get/p/i/d/out
        const uint8_t off = (uint8_t)(sig - AUX_TELEM_SIG_GIMBAL_PITCH_ANGLE_PID_SET);
        if (off < (uint8_t)(sizeof(s_gimbal_angle_pid_map) / sizeof(s_gimbal_angle_pid_map[0])))
        {
            return AuxTelemGimbalPidField(ctx->pitch ? &ctx->pitch->GimbalMotorAnglePid : NULL, s_gimbal_angle_pid_map[off]);
        }
        return 0.0f;
    }
    if (sig >= AUX_TELEM_SIG_GIMBAL_YAW_SPEED_PID_SET && sig <= AUX_TELEM_SIG_GIMBAL_YAW_SPEED_PID_OUT)
    {
        static const uint8_t s_pid_map[] = {5u, 6u, 9u, 10u, 11u, 12u, 13u}; // set/fdb/dbuf0/p/i/d/out
        const uint8_t off = (uint8_t)(sig - AUX_TELEM_SIG_GIMBAL_YAW_SPEED_PID_SET);
        if (off < (uint8_t)(sizeof(s_pid_map) / sizeof(s_pid_map[0])))
        {
            return AuxTelemPidField(ctx->yaw ? &ctx->yaw->GimbalMotorGyroPid : NULL, s_pid_map[off]);
        }
        return 0.0f;
    }
    if (sig >= AUX_TELEM_SIG_GIMBAL_PITCH_SPEED_PID_SET && sig <= AUX_TELEM_SIG_GIMBAL_PITCH_SPEED_PID_OUT)
    {
        static const uint8_t s_pid_map[] = {5u, 6u, 9u, 10u, 11u, 12u, 13u}; // set/fdb/dbuf0/p/i/d/out
        const uint8_t off = (uint8_t)(sig - AUX_TELEM_SIG_GIMBAL_PITCH_SPEED_PID_SET);
        if (off < (uint8_t)(sizeof(s_pid_map) / sizeof(s_pid_map[0])))
        {
            return AuxTelemPidField(ctx->pitch ? &ctx->pitch->GimbalMotorGyroPid : NULL, s_pid_map[off]);
        }
        return 0.0f;
    }
    if (sig >= AUX_TELEM_SIG_CHASSIS_FOLLOW_PID_SET && sig <= AUX_TELEM_SIG_CHASSIS_FOLLOW_PID_OUT)
    {
        static const uint8_t s_pid_map[] = {5u, 6u, 9u, 10u, 11u, 12u, 13u}; // set/fdb/dbuf0/p/i/d/out
        const uint8_t off = (uint8_t)(sig - AUX_TELEM_SIG_CHASSIS_FOLLOW_PID_SET);
        if (off < (uint8_t)(sizeof(s_pid_map) / sizeof(s_pid_map[0])))
        {
            return AuxTelemPidField(ctx->chassis ? &ctx->chassis->ChassisAnglePid : NULL, s_pid_map[off]);
        }
        return 0.0f;
    }
    if (sig >= AUX_TELEM_SIG_CHASSIS_M0_SPD_PID_SET && sig <= AUX_TELEM_SIG_CHASSIS_M3_SPD_PID_OUT)
    {
        static const uint8_t s_pid_map[] = {5u, 6u, 9u, 10u, 11u, 12u, 13u}; // set/fdb/dbuf0/p/i/d/out
        const uint32_t off = (uint32_t)(sig - AUX_TELEM_SIG_CHASSIS_M0_SPD_PID_SET);
        const uint8_t motor = (uint8_t)(off / 7u);
        const uint8_t field = (uint8_t)(off % 7u);
        if (motor < 4u && ctx->chassis)
        {
            return AuxTelemPidField(&ctx->chassis->motor_speed_pid[motor], s_pid_map[field]);
        }
        return 0.0f;
    }
    if (sig >= AUX_TELEM_SIG_CHASSIS_M0_RPM && sig <= AUX_TELEM_SIG_CHASSIS_M3_TEMP)
    {
        const uint32_t off = (uint32_t)(sig - AUX_TELEM_SIG_CHASSIS_M0_RPM);
        const uint8_t motor = (uint8_t)(off / 8u);
        const uint8_t field = (uint8_t)(off % 8u);
        if (motor < 4u && ctx->chassis)
        {
            const ChassisMotor *m = &ctx->chassis->motor_chassis[motor];
            const motor_measure_t *mm = m->ChassisMotorMeasure;
            switch (field)
            {
            case 0: return (fp32)(mm ? mm->speed_rpm : 0);
            case 1: return (fp32)m->give_current;
            case 2: return (fp32)(mm ? mm->given_current : 0);
            case 3: return m->speed_set;
            case 4: return m->speed;
            case 5: return m->accel;
            case 6: return (fp32)(mm ? mm->ecd : 0);
            case 7: return (fp32)(mm ? mm->temperate : 0);
            default: return 0.0f;
            }
        }
        return 0.0f;
    }
    if (sig >= AUX_TELEM_SIG_SHOOT_FRIC0_RPM && sig <= AUX_TELEM_SIG_SHOOT_FRIC3_CURRENT_CMD)
    {
        const uint32_t off = (uint32_t)(sig - AUX_TELEM_SIG_SHOOT_FRIC0_RPM);
        const uint8_t motor = (uint8_t)(off / 4u);
        const uint8_t field = (uint8_t)(off % 4u);
        if (motor < SHOOT_STATE_FRIC_MOTOR_COUNT)
        {
            const motor_measure_t *mm = ctx->fric_meas[motor];
            switch (field)
            {
            case 0: return (fp32)(mm ? mm->speed_rpm : 0);
            case 1: return (fp32)(mm ? mm->given_current : 0);
            case 2: return (fp32)(mm ? mm->temperate : 0);
            case 3:
                return (fp32)LowCmdGetCurrent(AuxTelemCachedMotorIdAt(s_aux_telem_friction_ids, 4u, motor));
            default: return 0.0f;
            }
        }
        return 0.0f;
    }

    const fp32 rad2deg = 57.29577951308232f;

    switch (sig)
    {
    case AUX_TELEM_SIG_SYS_TICK_MS:
        return (fp32)BspTimeGetTickMs();
    case AUX_TELEM_SIG_SYS_AUX_CMD_SEQ:
        return (fp32)AuxTuneGetCmdSeq();
    case AUX_TELEM_SIG_SYS_BATTERY_VOLT:
        return BatteryVoltage;
    case AUX_TELEM_SIG_SYS_BATTERY_PERCENT:
        return electricity_percentage * 100.0f;

    case AUX_TELEM_SIG_RC_CH0:
    case AUX_TELEM_SIG_RC_CH1:
    case AUX_TELEM_SIG_RC_CH2:
    case AUX_TELEM_SIG_RC_CH3:
    case AUX_TELEM_SIG_RC_CH4:
        if (ctx->rc)
        {
            const uint8_t idx = (uint8_t)(sig - AUX_TELEM_SIG_RC_CH0);
            return (idx < 5u) ? (fp32)ctx->rc->rc.ch[idx] : 0.0f;
        }
        return 0.0f;
    case AUX_TELEM_SIG_RC_S0:
        return ctx->rc ? (fp32)ctx->rc->rc.s[0] : 0.0f;
    case AUX_TELEM_SIG_RC_S1:
        return ctx->rc ? (fp32)ctx->rc->rc.s[1] : 0.0f;
    case AUX_TELEM_SIG_RC_MOUSE_X:
        return ctx->rc ? (fp32)ctx->rc->mouse.x : 0.0f;
    case AUX_TELEM_SIG_RC_MOUSE_Y:
        return ctx->rc ? (fp32)ctx->rc->mouse.y : 0.0f;
    case AUX_TELEM_SIG_RC_MOUSE_Z:
        return ctx->rc ? (fp32)ctx->rc->mouse.z : 0.0f;
    case AUX_TELEM_SIG_RC_MOUSE_L:
        return ctx->rc ? (fp32)ctx->rc->mouse.press_l : 0.0f;
    case AUX_TELEM_SIG_RC_MOUSE_R:
        return ctx->rc ? (fp32)ctx->rc->mouse.press_r : 0.0f;
    case AUX_TELEM_SIG_RC_KEY:
        return ctx->rc ? (fp32)ctx->rc->key.v : 0.0f;
    case AUX_TELEM_SIG_RC_ERROR:
        return (fp32)RC_data_is_error();

    case AUX_TELEM_SIG_IMU_Q0:
    case AUX_TELEM_SIG_IMU_Q1:
    case AUX_TELEM_SIG_IMU_Q2:
    case AUX_TELEM_SIG_IMU_Q3:
        if (ctx->quat)
        {
            const uint8_t idx = (uint8_t)(sig - AUX_TELEM_SIG_IMU_Q0);
            return (idx < 4u) ? ctx->quat[idx] : 0.0f;
        }
        return 0.0f;
    case AUX_TELEM_SIG_IMU_ANGLE_YAW_RAD:
        return ctx->angle ? ctx->angle[INS_YAW_ADDRESS_OFFSET] : 0.0f;
    case AUX_TELEM_SIG_IMU_ANGLE_ROLL_RAD:
        return ctx->angle ? ctx->angle[INS_ROLL_ADDRESS_OFFSET] : 0.0f;
    case AUX_TELEM_SIG_IMU_ANGLE_PITCH_RAD:
        return ctx->angle ? ctx->angle[INS_PITCH_ADDRESS_OFFSET] : 0.0f;
    case AUX_TELEM_SIG_IMU_ANGLE_YAW_DEG:
        return ctx->angle ? (ctx->angle[INS_YAW_ADDRESS_OFFSET] * rad2deg) : 0.0f;
    case AUX_TELEM_SIG_IMU_ANGLE_ROLL_DEG:
        return ctx->angle ? (ctx->angle[INS_ROLL_ADDRESS_OFFSET] * rad2deg) : 0.0f;
    case AUX_TELEM_SIG_IMU_ANGLE_PITCH_DEG:
        return ctx->angle ? (ctx->angle[INS_PITCH_ADDRESS_OFFSET] * rad2deg) : 0.0f;
    case AUX_TELEM_SIG_IMU_GYRO_X_RAD_S:
        return ctx->gyro ? ctx->gyro[INS_GYRO_X_ADDRESS_OFFSET] : 0.0f;
    case AUX_TELEM_SIG_IMU_GYRO_Y_RAD_S:
        return ctx->gyro ? ctx->gyro[INS_GYRO_Y_ADDRESS_OFFSET] : 0.0f;
    case AUX_TELEM_SIG_IMU_GYRO_Z_RAD_S:
        return ctx->gyro ? ctx->gyro[INS_GYRO_Z_ADDRESS_OFFSET] : 0.0f;
    case AUX_TELEM_SIG_IMU_GYRO_X_DPS:
        return ctx->gyro ? (ctx->gyro[INS_GYRO_X_ADDRESS_OFFSET] * rad2deg) : 0.0f;
    case AUX_TELEM_SIG_IMU_GYRO_Y_DPS:
        return ctx->gyro ? (ctx->gyro[INS_GYRO_Y_ADDRESS_OFFSET] * rad2deg) : 0.0f;
    case AUX_TELEM_SIG_IMU_GYRO_Z_DPS:
        return ctx->gyro ? (ctx->gyro[INS_GYRO_Z_ADDRESS_OFFSET] * rad2deg) : 0.0f;
    case AUX_TELEM_SIG_IMU_ACCEL_X:
        return ctx->accel ? ctx->accel[INS_ACCEL_X_ADDRESS_OFFSET] : 0.0f;
    case AUX_TELEM_SIG_IMU_ACCEL_Y:
        return ctx->accel ? ctx->accel[INS_ACCEL_Y_ADDRESS_OFFSET] : 0.0f;
    case AUX_TELEM_SIG_IMU_ACCEL_Z:
        return ctx->accel ? ctx->accel[INS_ACCEL_Z_ADDRESS_OFFSET] : 0.0f;

    case AUX_TELEM_SIG_GIMBAL_YAW_ANGLE:
        return ctx->yaw ? ctx->yaw->angle : 0.0f;
    case AUX_TELEM_SIG_GIMBAL_YAW_ANGLE_SET:
        return ctx->yaw ? ctx->yaw->angle_set : 0.0f;
    case AUX_TELEM_SIG_GIMBAL_YAW_GYRO:
        return ctx->yaw ? ctx->yaw->motor_gyro : 0.0f;
    case AUX_TELEM_SIG_GIMBAL_YAW_GYRO_SET:
        return ctx->yaw ? ctx->yaw->motor_gyro_set : 0.0f;
    case AUX_TELEM_SIG_GIMBAL_YAW_MOTOR_SPEED:
        return ctx->yaw ? ctx->yaw->motor_speed : 0.0f;
    case AUX_TELEM_SIG_GIMBAL_YAW_CURRENT_SET:
        return ctx->yaw ? ctx->yaw->current_set : 0.0f;
    case AUX_TELEM_SIG_GIMBAL_YAW_GIVEN_CURRENT:
        return ctx->yaw ? (fp32)ctx->yaw->given_current : 0.0f;
    case AUX_TELEM_SIG_GIMBAL_YAW_RAW_CMD_CURRENT:
        return ctx->yaw ? ctx->yaw->raw_cmd_current : 0.0f;
    case AUX_TELEM_SIG_GIMBAL_YAW_ECD:
        return (ctx->yaw && ctx->yaw->GimbalMotorMeasure) ? (fp32)ctx->yaw->GimbalMotorMeasure->ecd : 0.0f;
    case AUX_TELEM_SIG_GIMBAL_YAW_OFFSET_ECD:
        return ctx->yaw ? (fp32)ctx->yaw->offset_ecd : 0.0f;
    case AUX_TELEM_SIG_GIMBAL_YAW_RPM:
        return (ctx->yaw && ctx->yaw->GimbalMotorMeasure) ? (fp32)ctx->yaw->GimbalMotorMeasure->speed_rpm : 0.0f;
    case AUX_TELEM_SIG_GIMBAL_YAW_CURRENT_FB:
        return (ctx->yaw && ctx->yaw->GimbalMotorMeasure) ? (fp32)ctx->yaw->GimbalMotorMeasure->given_current : 0.0f;
    case AUX_TELEM_SIG_GIMBAL_YAW_TEMP:
        return (ctx->yaw && ctx->yaw->GimbalMotorMeasure) ? (fp32)ctx->yaw->GimbalMotorMeasure->temperate : 0.0f;

    case AUX_TELEM_SIG_GIMBAL_PITCH_ANGLE:
        return ctx->pitch ? ctx->pitch->angle : 0.0f;
    case AUX_TELEM_SIG_GIMBAL_PITCH_ANGLE_SET:
        return ctx->pitch ? ctx->pitch->angle_set : 0.0f;
    case AUX_TELEM_SIG_GIMBAL_PITCH_GYRO:
        return ctx->pitch ? ctx->pitch->motor_gyro : 0.0f;
    case AUX_TELEM_SIG_GIMBAL_PITCH_GYRO_SET:
        return ctx->pitch ? ctx->pitch->motor_gyro_set : 0.0f;
    case AUX_TELEM_SIG_GIMBAL_PITCH_MOTOR_SPEED:
        return ctx->pitch ? ctx->pitch->motor_speed : 0.0f;
    case AUX_TELEM_SIG_GIMBAL_PITCH_CURRENT_SET:
        return ctx->pitch ? ctx->pitch->current_set : 0.0f;
    case AUX_TELEM_SIG_GIMBAL_PITCH_GIVEN_CURRENT:
        return ctx->pitch ? (fp32)ctx->pitch->given_current : 0.0f;
    case AUX_TELEM_SIG_GIMBAL_PITCH_RAW_CMD_CURRENT:
        return ctx->pitch ? ctx->pitch->raw_cmd_current : 0.0f;
    case AUX_TELEM_SIG_GIMBAL_PITCH_ECD:
        return (ctx->pitch && ctx->pitch->GimbalMotorMeasure) ? (fp32)ctx->pitch->GimbalMotorMeasure->ecd : 0.0f;
    case AUX_TELEM_SIG_GIMBAL_PITCH_OFFSET_ECD:
        return ctx->pitch ? (fp32)ctx->pitch->offset_ecd : 0.0f;
    case AUX_TELEM_SIG_GIMBAL_PITCH_RPM:
        return (ctx->pitch && ctx->pitch->GimbalMotorMeasure) ? (fp32)ctx->pitch->GimbalMotorMeasure->speed_rpm : 0.0f;
    case AUX_TELEM_SIG_GIMBAL_PITCH_CURRENT_FB:
        return (ctx->pitch && ctx->pitch->GimbalMotorMeasure) ? (fp32)ctx->pitch->GimbalMotorMeasure->given_current : 0.0f;
    case AUX_TELEM_SIG_GIMBAL_PITCH_TEMP:
        return (ctx->pitch && ctx->pitch->GimbalMotorMeasure) ? (fp32)ctx->pitch->GimbalMotorMeasure->temperate : 0.0f;

    case AUX_TELEM_SIG_CHASSIS_VX_SET:
        return ctx->chassis ? ctx->chassis->vx_set : 0.0f;
    case AUX_TELEM_SIG_CHASSIS_VY_SET:
        return ctx->chassis ? ctx->chassis->vy_set : 0.0f;
    case AUX_TELEM_SIG_CHASSIS_WZ_SET:
        return ctx->chassis ? ctx->chassis->wz_set : 0.0f;
    case AUX_TELEM_SIG_CHASSIS_VX:
        return ctx->chassis ? ctx->chassis->vx : 0.0f;
    case AUX_TELEM_SIG_CHASSIS_VY:
        return ctx->chassis ? ctx->chassis->vy : 0.0f;
    case AUX_TELEM_SIG_CHASSIS_WZ:
        return ctx->chassis ? ctx->chassis->wz : 0.0f;
    case AUX_TELEM_SIG_CHASSIS_YAW_OFFSET:
        return ctx->chassis ? ctx->chassis->ChassisYawOffset : 0.0f;
    case AUX_TELEM_SIG_CHASSIS_YAW_OFFSET_SET:
        return ctx->chassis ? ctx->chassis->ChassisYawOffsetSet : 0.0f;
    case AUX_TELEM_SIG_CHASSIS_YAW_SET:
        return ctx->chassis ? ctx->chassis->ChassisYawSet : 0.0f;
    case AUX_TELEM_SIG_CHASSIS_YAW:
        return ctx->chassis ? ctx->chassis->ChassisYaw : 0.0f;
    case AUX_TELEM_SIG_CHASSIS_PITCH:
        return ctx->chassis ? ctx->chassis->ChassisPitch : 0.0f;
    case AUX_TELEM_SIG_CHASSIS_ROLL:
        return ctx->chassis ? ctx->chassis->ChassisRoll : 0.0f;
    case AUX_TELEM_SIG_CHASSIS_SWING_KEY:
        if (ctx->rc)
        {
            const uint16_t sw = (uint16_t)ctx->rc->rc.s[CHASSIS_MODE_CHANNEL];
            const uint16_t key = ctx->rc->key.v;
            const bool_t swing = ((key & SWING_KEY) != 0u) ||
                                 ((key & CHASSIS_GYRO_SPIN_VAR_KEY) != 0u) ||
                                 switch_is_down(sw);
            return swing ? 1.0f : 0.0f;
        }
        return 0.0f;
    case AUX_TELEM_SIG_SHOOT_FRIC_SPEED_SET:
        return ctx->shoot.fric_speed_set;
    case AUX_TELEM_SIG_SHOOT_TRIGGER_SPEED_SET:
        return ctx->shoot.trigger_speed_set;
    case AUX_TELEM_SIG_SHOOT_TRIGGER_SPEED:
        return ctx->shoot.speed;
    case AUX_TELEM_SIG_SHOOT_TRIGGER_ANGLE:
        return ctx->shoot.angle;
    case AUX_TELEM_SIG_SHOOT_TRIGGER_ANGLE_SET:
        return ctx->shoot.set_angle;
    case AUX_TELEM_SIG_SHOOT_TRIGGER_GIVEN_CURRENT:
        return (fp32)ctx->shoot.given_current;
    case AUX_TELEM_SIG_SHOOT_TRIGGER_ECD_COUNT:
        return (fp32)ctx->shoot.ecd_count;
    case AUX_TELEM_SIG_SHOOT_PRESS_L:
        return (fp32)ctx->shoot.press_l;
    case AUX_TELEM_SIG_SHOOT_PRESS_R:
        return (fp32)ctx->shoot.press_r;
    case AUX_TELEM_SIG_SHOOT_KEY:
        return (fp32)ctx->shoot.key;
    case AUX_TELEM_SIG_SHOOT_HEAT_LIMIT:
        return (fp32)ctx->shoot.heat_limit;
    case AUX_TELEM_SIG_SHOOT_HEAT:
        return (fp32)ctx->shoot.heat;

    case AUX_TELEM_SIG_SHOOT_TRIGGER_RPM:
        return ctx->trigger_meas ? (fp32)ctx->trigger_meas->speed_rpm : 0.0f;
    case AUX_TELEM_SIG_SHOOT_TRIGGER_ECD:
        return ctx->trigger_meas ? (fp32)ctx->trigger_meas->ecd : 0.0f;
    case AUX_TELEM_SIG_SHOOT_TRIGGER_TEMP:
        return ctx->trigger_meas ? (fp32)ctx->trigger_meas->temperate : 0.0f;
    case AUX_TELEM_SIG_SHOOT_TRIGGER_CURRENT_FB:
        return ctx->trigger_meas ? (fp32)ctx->trigger_meas->given_current : 0.0f;

    case AUX_TELEM_SIG_DIAG_ACTUATOR_CHASSIS0_CURRENT:
        return (fp32)LowCmdGetCurrent(AuxTelemCachedMotorIdAt(s_aux_telem_chassis_ids, 4u, 0u));
    case AUX_TELEM_SIG_DIAG_ACTUATOR_CHASSIS1_CURRENT:
        return (fp32)LowCmdGetCurrent(AuxTelemCachedMotorIdAt(s_aux_telem_chassis_ids, 4u, 1u));
    case AUX_TELEM_SIG_DIAG_ACTUATOR_PITCH_CURRENT:
        return (fp32)LowCmdGetCurrent(AuxTelemCachedMotorId(&s_aux_telem_pitch_id));
    case AUX_TELEM_SIG_DIAG_ACTUATOR_TRIGGER_CURRENT:
        return (fp32)LowCmdGetCurrent(AuxTelemCachedMotorId(&s_aux_telem_trigger_id));
    case AUX_TELEM_SIG_DIAG_ACTUATOR_CHASSIS3_CURRENT:
        return (fp32)LowCmdGetCurrent(AuxTelemCachedMotorIdAt(s_aux_telem_chassis_ids, 4u, 3u));
    case AUX_TELEM_SIG_DIAG_ACTUATOR_YAW_CURRENT:
        return (fp32)LowCmdGetCurrent(AuxTelemCachedMotorId(&s_aux_telem_yaw_id));
    case AUX_TELEM_SIG_DIAG_ACTUATOR_CHASSIS2_CURRENT:
        return (fp32)LowCmdGetCurrent(AuxTelemCachedMotorIdAt(s_aux_telem_chassis_ids, 4u, 2u));
    case AUX_TELEM_SIG_DIAG_RM_GROUP_1FF_STATUS:
        return (fp32)CAN_get_last_1ff_status();
    case AUX_TELEM_SIG_DIAG_CAN_BUS1_ERR:
        return (fp32)CAN_get_last_can1_error();
    case AUX_TELEM_SIG_DIAG_ZERO_FORCE:
        return (GimbalBehaviourWatch == GIMBAL_ZERO_FORCE) ? 1.0f : 0.0f;

    case AUX_TELEM_SIG_PACK_MODE:
    {
        const uint32_t behaviour_gimbal = (uint32_t)GimbalBehaviourWatch;
        const uint32_t yaw_motor_mode = (ctx->yaw != NULL) ? (uint32_t)ctx->yaw->mode : 0u;
        const uint32_t pitch_motor_mode = (ctx->pitch != NULL) ? (uint32_t)ctx->pitch->mode : 0u;
        const uint32_t mode_chassis = (ctx->chassis != NULL) ? (uint32_t)ctx->chassis->mode : 0u;
        const uint32_t mode_chassis_last = (ctx->chassis != NULL) ? (uint32_t)ctx->chassis->last_mode : 0u;
        const uint32_t mode_shoot = (uint32_t)ctx->shoot.mode;

        const uint32_t packed = behaviour_gimbal +
                                yaw_motor_mode * 10u +
                                pitch_motor_mode * 100u +
                                mode_chassis * 1000u +
                                mode_chassis_last * 10000u +
                                mode_shoot * 100000u;
        return (fp32)packed;
    }
    case AUX_TELEM_SIG_PACK_OFFLINE:
    {
        uint32_t mask = 0u;
        const uint8_t classic_chassis_on = RobotProfileNeedClassicChassisControlTask();
        const uint8_t GimbalOn = (uint8_t)(RobotProfileNeedSingleGimbalControlTask() ||
                                            RobotProfileNeedDualGimbalControlTask());
        const uint8_t yaw_on = (uint8_t)(MotorCfgNodeId(&g_config.motor.yaw) != 0u);
        const uint8_t second_gimbal_axis_on = (uint8_t)(MotorCfgNodeId(&g_config.motor.pitch) != 0u ||
                                                        MotorCfgNodeId(&g_config.motor.yaw_upper) != 0u);
        const uint8_t trigger_on = (uint8_t)(MotorCfgNodeId(&g_config.motor.trigger) != 0u);
        GimbalState state;
        const uint8_t state_valid_gimbal =
            (GimbalStateRead(&state) != 0u && state.valid != 0u) ? 1u : 0u;
        if (toe_is_error(DBUS_TOE)) mask |= 1u << 0;
        if (classic_chassis_on && toe_is_error(CHASSIS_MOTOR1_TOE)) mask |= 1u << 1;
        if (classic_chassis_on && toe_is_error(CHASSIS_MOTOR2_TOE)) mask |= 1u << 2;
        if (classic_chassis_on && toe_is_error(CHASSIS_MOTOR3_TOE)) mask |= 1u << 3;
        if (classic_chassis_on && toe_is_error(CHASSIS_MOTOR4_TOE)) mask |= 1u << 4;
        if (GimbalOn && state_valid_gimbal != 0u)
        {
            if ((state.offline_mask & GIMBAL_STATE_OFFLINE_YAW) != 0u) mask |= 1u << 5;
            if ((state.offline_mask & (GIMBAL_STATE_OFFLINE_PITCH | GIMBAL_STATE_OFFLINE_YAW_UPPER)) != 0u) mask |= 1u << 6;
        }
        else
        {
            if (GimbalOn && yaw_on && toe_is_error(YAW_GIMBAL_MOTOR_TOE)) mask |= 1u << 5;
            if (GimbalOn && second_gimbal_axis_on && toe_is_error(PITCH_GIMBAL_MOTOR_TOE)) mask |= 1u << 6;
        }
        if (trigger_on && toe_is_error(TRIGGER_MOTOR_TOE)) mask |= 1u << 7;
        if (toe_is_error(REFEREE_TOE)) mask |= 1u << 8;
        if (toe_is_error(RM_IMU_TOE)) mask |= 1u << 9;
        if (toe_is_error(BOARD_GYRO_TOE)) mask |= 1u << 10;
        if (toe_is_error(BOARD_ACCEL_TOE)) mask |= 1u << 11;
        if (toe_is_error(BOARD_MAG_TOE)) mask |= 1u << 12;
        if (toe_is_error(OLED_TOE)) mask |= 1u << 13;
        return (fp32)mask;
    }

    case AUX_TELEM_SIG_MEM_HEAP_FREE:
        return (fp32)heap_get_free();
    case AUX_TELEM_SIG_MEM_HEAP_EVER_FREE:
        return (fp32)heap_get_ever_free();

    case AUX_TELEM_SIG_BOARD_KEY_DOWN:
        return (fp32)BspKeyReadRawDown();
    case AUX_TELEM_SIG_BOARD_KEY_PRESS_CNT:
        return (fp32)BspKeyGetPressCnt();

    case AUX_TELEM_SIG_SHOOT_TRIGGER_PID_IOUT:
        return ctx->shoot.trigger_motor_pid.Iout;
    case AUX_TELEM_SIG_SHOOT_TRIGGER_PID_OUT:
        return ctx->shoot.trigger_motor_pid.out;

    default:
        return 0.0f;
    }
}
