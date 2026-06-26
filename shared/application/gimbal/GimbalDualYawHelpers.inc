/*
 * SPDX-FileCopyrightText: 2026 Chen Xuan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Private implementation chunk included by GimbalControlTask.c.
 */

static GimbalDualYawDebugState s_dual_yaw_debug = {0};
static MotorId s_dual_yaw_upper_id = MotorCount;
static uint8_t s_dual_yaw_upper_id_ready = 0u;

uint8_t GimbalDualYawDebugRead(GimbalDualYawDebugState *out)
{
    if (out == NULL)
    {
        return 0u;
    }

    *out = s_dual_yaw_debug;
    return s_dual_yaw_debug.active;
}

static MotorId GimbalDualYawUpperMotorId(void)
{
    if (s_dual_yaw_upper_id_ready == 0u)
    {
        s_dual_yaw_upper_id = MotorInstIdByName("motor.yaw_upper");
        s_dual_yaw_upper_id_ready = 1u;
    }

    return s_dual_yaw_upper_id;
}

static uint8_t GimbalDualYawUpperOnline(void)
{
    const MotorId id = GimbalDualYawUpperMotorId();
    const MotorInst *inst = MotorInstFindByMotor(id);

    return (uint8_t)((id != MotorCount && inst != NULL && MotorInstEnabled(inst) != 0u) ? 1u : 0u);
}

static void GimbalDualYawUpperSoftLimit(fp32 *min_out, fp32 *max_out)
{
    fp32 min_v = g_config.dual_gimbal.yaw_upper_soft_limit_min;
    fp32 max_v = g_config.dual_gimbal.yaw_upper_soft_limit_max;

    if (max_v < min_v)
    {
        const fp32 t = max_v;
        max_v = min_v;
        min_v = t;
    }

    if (max_v <= min_v || fabsf(max_v) > 100.0f || fabsf(min_v) > 100.0f)
    {
        min_v = -PI;
        max_v = PI;
    }

    if (min_out != NULL)
    {
        *min_out = min_v;
    }
    if (max_out != NULL)
    {
        *max_out = max_v;
    }
}

static fp32 GimbalDualYawUpperAngle(const motor_measure_t *measure, fp32 *gyro_out)
{
    fp32 angle = 0.0f;
    fp32 gyro = 0.0f;

    if (measure != NULL)
    {
        angle = motor_ecd_to_angle_change(measure->ecd, g_config.dual_gimbal.yaw_upper_middle_ecd);
        gyro = (fp32)measure->speed_rpm * 0.104719755f;
    }

    if (g_config.dual_gimbal.yaw_upper_turn != 0u)
    {
        angle = -angle;
        gyro = -gyro;
    }

    if (gyro_out != NULL)
    {
        *gyro_out = gyro;
    }
    return angle;
}

static void GimbalDualYawDebugUpdate(const GimbalDualYawDebugState *state)
{
    if (state == NULL)
    {
        memset(&s_dual_yaw_debug, 0, sizeof(s_dual_yaw_debug));
        return;
    }

    s_dual_yaw_debug = *state;
}

static uint8_t GimbalDualYawCompensationAllowed(const GimbalRuntimeSnapshot *snapshot)
{
    const robot_run_variant_e variant = (snapshot != NULL) ? snapshot->run_variant : robot_mode_variant();

    if (snapshot == NULL ||
        snapshot->yaw_control_is_upper != 0u ||
        g_config.dual_gimbal.enable == 0u ||
        MotorCfgNodeId(&g_config.motor.yaw_upper) == 0u ||
        GimbalBehaviourWatch != GIMBAL_ANGLE ||
        g_gimbal.GimbalYawMotor.mode != GIMBAL_MOTOR_ENCODER)
    {
        return 0u;
    }

    return (uint8_t)((variant == ROBOT_RUN_VARIANT_NORMAL ||
                      variant == ROBOT_RUN_VARIANT_GIMBAL_DUAL) ? 1u : 0u);
}

static int16_t GimbalDualYawUpperControlCurrent(const GimbalRuntimeSnapshot *snapshot,
                                                        int16_t yaw_current_output)
{
    const motor_measure_t *upper_measure = get_yaw_upper_gimbal_motor_measure_point();
    const uint8_t upper_online = GimbalDualYawUpperOnline();
    fp32 upper_gyro = 0.0f;
    const fp32 upper_angle = GimbalDualYawUpperAngle(upper_measure, &upper_gyro);
    fp32 upper_min = 0.0f;
    fp32 upper_max = 0.0f;
    fp32 upper_set = 0.0f;
    fp32 upper_error = 0.0f;
    fp32 current = 0.0f;
    uint8_t upper_limited = 0u;
    GimbalDualYawDebugState debug = {0};

    GimbalDualYawUpperSoftLimit(&upper_min, &upper_max);

    debug.upper_online = upper_online;
    debug.total_set_rad = g_gimbal.GimbalYawMotor.angle_set;
    debug.big_angle_rad = g_gimbal.GimbalYawMotor.angle;
    debug.big_set_rad = g_gimbal.GimbalYawMotor.angle_set;
    debug.big_error_rad = rad_format(g_gimbal.GimbalYawMotor.angle_set - g_gimbal.GimbalYawMotor.angle);
    debug.upper_angle_rad = upper_angle;
    debug.upper_gyro_radps = upper_gyro;
    debug.yaw_current = yaw_current_output;
    debug.yaw_upper_ecd = (upper_measure != NULL) ? upper_measure->ecd : 0u;

    if (GimbalDualYawCompensationAllowed(snapshot) == 0u || upper_online == 0u)
    {
        GimbalDualYawDebugUpdate(&debug);
        return 0;
    }

    // 小 yaw 追“大 yaw 还没转到的那部分”；大 yaw 继续追总目标，小 yaw 会自然回中。
    upper_set = rad_format(g_gimbal.GimbalYawMotor.angle_set - g_gimbal.GimbalYawMotor.angle);
    if (upper_set > upper_max)
    {
        upper_set = upper_max;
        upper_limited = 1u;
    }
    else if (upper_set < upper_min)
    {
        upper_set = upper_min;
        upper_limited = 1u;
    }

    if (fabsf(upper_set) < fabsf(g_config.dual_gimbal.yaw_upper_center_deadband))
    {
        upper_set = 0.0f;
    }

    upper_error = rad_format(upper_set - upper_angle);
    current = g_config.dual_gimbal.yaw_upper_kp * upper_error -
              g_config.dual_gimbal.yaw_upper_kd * upper_gyro;

    if (upper_angle >= upper_max && current > 0.0f)
    {
        current = 0.0f;
        upper_limited = 1u;
    }
    else if (upper_angle <= upper_min && current < 0.0f)
    {
        current = 0.0f;
        upper_limited = 1u;
    }

    {
        const fp32 limit = fabsf(g_config.dual_gimbal.yaw_upper_current_limit);
        if (limit > 0.0f)
        {
            current = fp32_constrain(current, -limit, limit);
        }
    }

    debug.active = 1u;
    debug.upper_limited = upper_limited;
    debug.upper_set_rad = upper_set;
    debug.upper_error_rad = upper_error;
    debug.yaw_upper_current = GimbalApplyOutputTurn((int16_t)current, g_config.dual_gimbal.yaw_upper_turn);
    GimbalDualYawDebugUpdate(&debug);

    return debug.yaw_upper_current;
}
