/*
 * SPDX-FileCopyrightText: 2026 ?? <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Private implementation chunk included by GimbalControlTask.c.
 */

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
void set_cali_gimbal_hook(const uint16_t yaw_offset, const uint16_t pitch_offset, const fp32 max_yaw, const fp32 min_yaw, const fp32 max_pitch, const fp32 min_pitch)
{
    g_gimbal.GimbalYawMotor.offset_ecd = yaw_offset;
    g_gimbal.GimbalYawMotor.max_angle = max_yaw;
    g_gimbal.GimbalYawMotor.min_angle = min_yaw;

    g_gimbal.GimbalPitchMotor.offset_ecd = pitch_offset;
    g_gimbal.GimbalPitchMotor.max_angle = max_pitch;
    g_gimbal.GimbalPitchMotor.min_angle = min_pitch;
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
bool_t cmd_cali_gimbal_hook(uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch)
{
    if (g_gimbal.GimbalCali.step == 0)
    {
        g_gimbal.GimbalCali.step             = GIMBAL_CALI_START_STEP;
        g_gimbal.GimbalCali.max_pitch        = g_gimbal.GimbalPitchMotor.angle;
        g_gimbal.GimbalCali.max_pitch_ecd    = g_gimbal.GimbalPitchMotor.GimbalMotorMeasure->ecd;
        g_gimbal.GimbalCali.max_yaw          = g_gimbal.GimbalYawMotor.angle;
        g_gimbal.GimbalCali.max_yaw_ecd      = g_gimbal.GimbalYawMotor.GimbalMotorMeasure->ecd;
        g_gimbal.GimbalCali.min_pitch        = g_gimbal.GimbalPitchMotor.angle;
        g_gimbal.GimbalCali.min_pitch_ecd    = g_gimbal.GimbalPitchMotor.GimbalMotorMeasure->ecd;
        g_gimbal.GimbalCali.min_yaw          = g_gimbal.GimbalYawMotor.angle;
        g_gimbal.GimbalCali.min_yaw_ecd      = g_gimbal.GimbalYawMotor.GimbalMotorMeasure->ecd;
        return 0;
    }
    else if (g_gimbal.GimbalCali.step == GIMBAL_CALI_END_STEP)
    {
        GimbalCalcCali(&g_gimbal.GimbalCali, yaw_offset, pitch_offset, max_yaw, min_yaw, max_pitch, min_pitch);
        (*max_yaw) -= GIMBAL_CALI_REDUNDANT_ANGLE;
        (*min_yaw) += GIMBAL_CALI_REDUNDANT_ANGLE;
        (*max_pitch) -= GIMBAL_CALI_REDUNDANT_ANGLE;
        (*min_pitch) += GIMBAL_CALI_REDUNDANT_ANGLE;
        g_gimbal.GimbalYawMotor.offset_ecd              = *yaw_offset;
        g_gimbal.GimbalYawMotor.max_angle      = *max_yaw;
        g_gimbal.GimbalYawMotor.min_angle      = *min_yaw;
        g_gimbal.GimbalPitchMotor.offset_ecd            = *pitch_offset;
        g_gimbal.GimbalPitchMotor.max_angle    = *max_pitch;
        g_gimbal.GimbalPitchMotor.min_angle    = *min_pitch;
        g_gimbal.GimbalCali.step = 0;
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
    int16_t temp_max_ecd = 0, temp_min_ecd = 0, temp_ecd = 0;

    if (YAW_TURN)
    {
        temp_ecd = GimbalCali->min_yaw_ecd - GimbalCali->max_yaw_ecd;

        if (temp_ecd < 0)
        {
        temp_ecd += ECD_RANGE;
        }
        temp_ecd = GimbalCali->max_yaw_ecd + (temp_ecd / 2);

        ecd_format(temp_ecd);
        *yaw_offset = temp_ecd;
        *max_yaw = -motor_ecd_to_angle_change(GimbalCali->max_yaw_ecd, *yaw_offset);
        *min_yaw = -motor_ecd_to_angle_change(GimbalCali->min_yaw_ecd, *yaw_offset);
    }
    else
    {

        temp_ecd = GimbalCali->max_yaw_ecd - GimbalCali->min_yaw_ecd;

        if (temp_ecd < 0)
        {
            temp_ecd += ECD_RANGE;
        }
        temp_ecd = GimbalCali->max_yaw_ecd - (temp_ecd / 2);

        ecd_format(temp_ecd);
        *yaw_offset = temp_ecd;
        *max_yaw = motor_ecd_to_angle_change(GimbalCali->max_yaw_ecd, *yaw_offset);
        *min_yaw = motor_ecd_to_angle_change(GimbalCali->min_yaw_ecd, *yaw_offset);
    }

    if (PITCH_TURN)

    {
        temp_ecd = (int16_t)(GimbalCali->max_pitch / MOTOR_ECD_TO_RAD);
        temp_max_ecd = GimbalCali->max_pitch_ecd + temp_ecd;
        temp_ecd = (int16_t)(GimbalCali->min_pitch / MOTOR_ECD_TO_RAD);
        temp_min_ecd = GimbalCali->min_pitch_ecd + temp_ecd;

        ecd_format(temp_max_ecd);
        ecd_format(temp_min_ecd);

        temp_ecd = temp_max_ecd - temp_min_ecd;

        if (temp_ecd > HALF_ECD_RANGE)
        {
            temp_ecd -= ECD_RANGE;
        }
        else if (temp_ecd < -HALF_ECD_RANGE)
        {
            temp_ecd += ECD_RANGE;
        }

        if (temp_max_ecd > temp_min_ecd)
        {
            temp_min_ecd += ECD_RANGE;
        }

        temp_ecd = temp_max_ecd - temp_ecd / 2;

        ecd_format(temp_ecd);

        *pitch_offset = temp_ecd;

        *max_pitch = -motor_ecd_to_angle_change(GimbalCali->max_pitch_ecd, *pitch_offset);
        *min_pitch = -motor_ecd_to_angle_change(GimbalCali->min_pitch_ecd, *pitch_offset);
    }
    else
    {
        temp_ecd = (int16_t)(GimbalCali->max_pitch / MOTOR_ECD_TO_RAD);
        temp_max_ecd = GimbalCali->max_pitch_ecd - temp_ecd;
        temp_ecd = (int16_t)(GimbalCali->min_pitch / MOTOR_ECD_TO_RAD);
        temp_min_ecd = GimbalCali->min_pitch_ecd - temp_ecd;

        ecd_format(temp_max_ecd);
        ecd_format(temp_min_ecd);

        temp_ecd = temp_max_ecd - temp_min_ecd;

        if (temp_ecd > HALF_ECD_RANGE)
        {
            temp_ecd -= ECD_RANGE;
        }
        else if (temp_ecd < -HALF_ECD_RANGE)
        {
            temp_ecd += ECD_RANGE;
        }

        temp_ecd = temp_max_ecd - temp_ecd / 2;

        ecd_format(temp_ecd);

        *pitch_offset = temp_ecd;

        *max_pitch = motor_ecd_to_angle_change(GimbalCali->max_pitch_ecd, *pitch_offset);
        *min_pitch = motor_ecd_to_angle_change(GimbalCali->min_pitch_ecd, *pitch_offset);
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
const GimbalMotor *get_yaw_motor_point(void)
{
    return &g_gimbal.GimbalYawMotor;
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
const GimbalMotor *get_pitch_motor_point(void)
{
    if (MotorCfgNodeId(&g_config.motor.pitch) == 0u)
    {
        return NULL;
    }
    return &g_gimbal.GimbalPitchMotor;
}
