/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * 云台机械校准和校准结果访问。
 */

#include "GimbalTaskPrivate.h"

#if ROBOT_TASK_BUILD_ANY_GIMBAL

#include "arm_math.h"
#include "MotorConfig.h"
#include "UserLib.h"
#include "BspTime.h"
#include <math.h>


static void GimbalCalcCali(const GimbalStepCali *GimbalCali,
                           uint16_t *yaw_offset,
                           uint16_t *pitch_offset,
                           fp32 *max_yaw,
                           fp32 *min_yaw,
                           fp32 *max_pitch,
                           fp32 *min_pitch);

/**
  * @brief          gimbal cali data, set motor offset encode, max and min angle
  * @param[in]      yaw_offse:yaw middle place encode
  * @param[in]      pitch_offset:pitch place encode
  * @param[in]      max_yaw:yaw max angle
  * @param[in]      min_yaw:yaw min angle
  * @param[in]      max_yaw:pitch max angle
  * @param[in]      min_yaw:pitch min angle
  * @retval         none
  */
void GimbalCalibrationSet(GimbalControl *control,
                          const uint16_t yaw_offset,
                          const uint16_t pitch_offset,
                          const fp32 max_yaw,
                          const fp32 min_yaw,
                          const fp32 max_pitch,
                          const fp32 min_pitch)
{
    if (control == NULL)
    {
        return;
    }
    control->GimbalYawMotor.offset_ecd = yaw_offset;
    control->GimbalYawMotor.max_angle = max_yaw;
    control->GimbalYawMotor.min_angle = min_yaw;

    control->GimbalPitchMotor.offset_ecd = pitch_offset;
    control->GimbalPitchMotor.max_angle = max_pitch;
    control->GimbalPitchMotor.min_angle = min_pitch;
}

/**
  * @brief          gimbal cali calculate, return motor offset encode, max and min angle
  * @param[out]     yaw_offse:yaw middle place encode
  * @param[out]     pitch_offset:pitch place encode
  * @param[out]     max_yaw:yaw max angle
  * @param[out]     min_yaw:yaw min angle
  * @param[out]     max_yaw:pitch max angle
  * @param[out]     min_yaw:pitch min angle
  * @retval         none
  */
bool_t GimbalCalibrationStep(GimbalControl *control,
                             uint16_t *yaw_offset,
                             uint16_t *pitch_offset,
                             fp32 *max_yaw,
                             fp32 *min_yaw,
                             fp32 *max_pitch,
                             fp32 *min_pitch)
{
    if (control == NULL || yaw_offset == NULL || pitch_offset == NULL || max_yaw == NULL ||
        min_yaw == NULL || max_pitch == NULL || min_pitch == NULL ||
        control->GimbalYawMotor.GimbalMotorMeasure == NULL ||
        control->GimbalPitchMotor.GimbalMotorMeasure == NULL)
    {
        return 0;
    }
    if (control->GimbalCali.step == 0)
    {
        control->GimbalCali.step             = GIMBAL_CALI_START_STEP;
        control->GimbalCali.stableTicks = 0u;
        control->GimbalCali.startedMs = BspTimeGetTickMs();
        control->GimbalCali.max_pitch        = control->GimbalPitchMotor.angle;
        control->GimbalCali.max_pitch_ecd    = control->GimbalPitchMotor.GimbalMotorMeasure->ecd;
        control->GimbalCali.max_yaw          = control->GimbalYawMotor.angle;
        control->GimbalCali.max_yaw_ecd      = control->GimbalYawMotor.GimbalMotorMeasure->ecd;
        control->GimbalCali.min_pitch        = control->GimbalPitchMotor.angle;
        control->GimbalCali.min_pitch_ecd    = control->GimbalPitchMotor.GimbalMotorMeasure->ecd;
        control->GimbalCali.min_yaw          = control->GimbalYawMotor.angle;
        control->GimbalCali.min_yaw_ecd      = control->GimbalYawMotor.GimbalMotorMeasure->ecd;
        return 0;
    }
    else if (control->GimbalCali.step == GIMBAL_CALI_END_STEP)
    {
        GimbalCalcCali(&control->GimbalCali, yaw_offset, pitch_offset, max_yaw, min_yaw, max_pitch, min_pitch);
        (*max_yaw) -= GIMBAL_CALI_REDUNDANT_ANGLE;
        (*min_yaw) += GIMBAL_CALI_REDUNDANT_ANGLE;
        (*max_pitch) -= GIMBAL_CALI_REDUNDANT_ANGLE;
        (*min_pitch) += GIMBAL_CALI_REDUNDANT_ANGLE;
        if (!isfinite(*max_yaw) || !isfinite(*min_yaw) || !isfinite(*max_pitch) ||
            !isfinite(*min_pitch) || *max_yaw <= *min_yaw || *max_pitch <= *min_pitch)
        {
            GimbalCalibrationCancel();
            return 0;
        }
        control->GimbalYawMotor.offset_ecd              = *yaw_offset;
        control->GimbalYawMotor.max_angle      = *max_yaw;
        control->GimbalYawMotor.min_angle      = *min_yaw;
        control->GimbalPitchMotor.offset_ecd            = *pitch_offset;
        control->GimbalPitchMotor.max_angle    = *max_pitch;
        control->GimbalPitchMotor.min_angle    = *min_pitch;
        control->GimbalCali.step = 0;
        return 1;
    }
    else
    {
        return 0;
    }
}

/**
  * @brief          calc motor offset encode, max and min angle
  * @param[out]     yaw_offse:yaw middle place encode
  * @param[out]     pitch_offset:pitch place encode
  * @param[out]     max_yaw:yaw max angle
  * @param[out]     min_yaw:yaw min angle
  * @param[out]     max_yaw:pitch max angle
  * @param[out]     min_yaw:pitch min angle
  * @retval         none
  */
static void GimbalCalcCali(const GimbalStepCali *GimbalCali, uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch)
{
    if (GimbalCali == NULL || yaw_offset == NULL || pitch_offset == NULL || max_yaw == NULL || min_yaw == NULL || max_pitch == NULL || min_pitch == NULL)
    {
        return;
    }

    // 函数地图：先按安装方向算 yaw 中值/限位，再按 pitch 软限位和编码器范围算 pitch 中值/限位。
    int32_t temp_max_ecd = 0, temp_min_ecd = 0, temp_ecd = 0;
    const uint32_t yaw_ecd_range = MotorCfgEncoderRange(g_config.motor.yaw.model);
    const uint32_t pitch_ecd_range = MotorCfgEncoderRange(g_config.motor.pitch.model);
    const int32_t pitch_half_ecd_range = (int32_t)(pitch_ecd_range / 2u);
    const fp32 pitch_ecd_to_rad = MotorTransPositionToOutput(
        GIMBAL_ENCODER_TWO_PI / (fp32)pitch_ecd_range,
        MotorCfgExternalReductionRatio(&g_config.motor.pitch));

    if (YAW_TURN)
    {
        temp_ecd = GimbalCali->min_yaw_ecd - GimbalCali->max_yaw_ecd;

        if (temp_ecd < 0)
        {
        temp_ecd += (int32_t)yaw_ecd_range;
        }
        temp_ecd = GimbalCali->max_yaw_ecd + (temp_ecd / 2);

        *yaw_offset = GimbalEncoderNormalizeCount(temp_ecd, yaw_ecd_range);
        *max_yaw = -GimbalMotorAngleFromEncoder(GimbalCali->max_yaw_ecd,
                                              *yaw_offset,
                                              &g_config.motor.yaw);
        *min_yaw = -GimbalMotorAngleFromEncoder(GimbalCali->min_yaw_ecd,
                                              *yaw_offset,
                                              &g_config.motor.yaw);
    }
    else
    {

        temp_ecd = GimbalCali->max_yaw_ecd - GimbalCali->min_yaw_ecd;

        if (temp_ecd < 0)
        {
            temp_ecd += (int32_t)yaw_ecd_range;
        }
        temp_ecd = GimbalCali->max_yaw_ecd - (temp_ecd / 2);

        *yaw_offset = GimbalEncoderNormalizeCount(temp_ecd, yaw_ecd_range);
        *max_yaw = GimbalMotorAngleFromEncoder(GimbalCali->max_yaw_ecd,
                                             *yaw_offset,
                                             &g_config.motor.yaw);
        *min_yaw = GimbalMotorAngleFromEncoder(GimbalCali->min_yaw_ecd,
                                             *yaw_offset,
                                             &g_config.motor.yaw);
    }

    if (PITCH_TURN)

    {
        temp_ecd = (int32_t)(GimbalCali->max_pitch / pitch_ecd_to_rad);
        temp_max_ecd = GimbalCali->max_pitch_ecd + temp_ecd;
        temp_ecd = (int32_t)(GimbalCali->min_pitch / pitch_ecd_to_rad);
        temp_min_ecd = GimbalCali->min_pitch_ecd + temp_ecd;

        temp_max_ecd = (int32_t)GimbalEncoderNormalizeCount(temp_max_ecd, pitch_ecd_range);
        temp_min_ecd = (int32_t)GimbalEncoderNormalizeCount(temp_min_ecd, pitch_ecd_range);

        temp_ecd = temp_max_ecd - temp_min_ecd;

        if (temp_ecd > pitch_half_ecd_range)
        {
            temp_ecd -= (int32_t)pitch_ecd_range;
        }
        else if (temp_ecd < -pitch_half_ecd_range)
        {
            temp_ecd += (int32_t)pitch_ecd_range;
        }

        if (temp_max_ecd > temp_min_ecd)
        {
            temp_min_ecd += (int32_t)pitch_ecd_range;
        }

        temp_ecd = temp_max_ecd - temp_ecd / 2;

        *pitch_offset = GimbalEncoderNormalizeCount(temp_ecd, pitch_ecd_range);

        *max_pitch = -GimbalMotorAngleFromEncoder(GimbalCali->max_pitch_ecd,
                                                *pitch_offset,
                                                &g_config.motor.pitch);
        *min_pitch = -GimbalMotorAngleFromEncoder(GimbalCali->min_pitch_ecd,
                                                *pitch_offset,
                                                &g_config.motor.pitch);
    }
    else
    {
        temp_ecd = (int32_t)(GimbalCali->max_pitch / pitch_ecd_to_rad);
        temp_max_ecd = GimbalCali->max_pitch_ecd - temp_ecd;
        temp_ecd = (int32_t)(GimbalCali->min_pitch / pitch_ecd_to_rad);
        temp_min_ecd = GimbalCali->min_pitch_ecd - temp_ecd;

        temp_max_ecd = (int32_t)GimbalEncoderNormalizeCount(temp_max_ecd, pitch_ecd_range);
        temp_min_ecd = (int32_t)GimbalEncoderNormalizeCount(temp_min_ecd, pitch_ecd_range);

        temp_ecd = temp_max_ecd - temp_min_ecd;

        if (temp_ecd > pitch_half_ecd_range)
        {
            temp_ecd -= (int32_t)pitch_ecd_range;
        }
        else if (temp_ecd < -pitch_half_ecd_range)
        {
            temp_ecd += (int32_t)pitch_ecd_range;
        }

        temp_ecd = temp_max_ecd - temp_ecd / 2;

        *pitch_offset = GimbalEncoderNormalizeCount(temp_ecd, pitch_ecd_range);

        *max_pitch = GimbalMotorAngleFromEncoder(GimbalCali->max_pitch_ecd,
                                               *pitch_offset,
                                               &g_config.motor.pitch);
        *min_pitch = GimbalMotorAngleFromEncoder(GimbalCali->min_pitch_ecd,
                                               *pitch_offset,
                                               &g_config.motor.pitch);
    }
}

/**
  * @brief          return yaw motor data point
  * @param[in]      none
  * @retval         yaw motor data point
  */
/**
  * @brief          返回yaw 电机数据指针
  * @param[in]      none
  * @retval         yaw电机指针
  */
const GimbalMotor *GimbalCalibrationYawMotor(const GimbalControl *control)
{
    return (control != NULL) ? &control->GimbalYawMotor : NULL;
}

/**
  * @brief          return pitch motor data point
  * @param[in]      none
  * @retval         pitch motor data point
  */
/**
  * @brief          返回pitch 电机数据指针
  * @param[in]      none
  * @retval         pitch
  */
const GimbalMotor *GimbalCalibrationPitchMotor(const GimbalControl *control)
{
    if (control == NULL || MotorCfgNodeId(&g_config.motor.pitch) == 0u)
    {
        return NULL;
    }
    return &control->GimbalPitchMotor;
}

#endif
