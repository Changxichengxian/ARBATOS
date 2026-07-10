/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

/*
 * 阅读地图：
 * - 前段：行为模式到控制模式的选择，安全档和运行模式优先。
 * - 中段：各模式的 vx/vy/wz 或目标角输出，包括跟随、小陀螺、摇摆。
 * - 后段：摇摆中心随机切换和云台 yaw 相对角辅助函数。
 * - 被调用方：ChassisControlTask 先问这里要目标，再做运动学和 PID。
 */

#include "RobotConfig.h"
#include "RobotTaskBuildConfig.h"

#if ROBOT_TASK_BUILD_CLASSIC_CHASSIS

#include "ChassisBehaviour.h"
#include "cmsis_os.h"
#include "ChassisControlTask.h"
#include "arm_math.h"

#include "BspTime.h"
#include "ControlInput.h"
#include "ExternalMotionIntent.h"
#include "MotorConfig.h"

#include <math.h>
#include <string.h>

#ifndef HALF_ECD_RANGE
#define HALF_ECD_RANGE 4096
#endif

#ifndef ECD_RANGE
#define ECD_RANGE 8192
#endif

#ifndef MOTOR_ECD_TO_RAD
#define MOTOR_ECD_TO_RAD (g_config.gimbal.motor_ecd_to_rad)
#endif

#ifndef CHASSIS_STOP_ON_GIMBAL_STATE
#define CHASSIS_STOP_ON_GIMBAL_STATE 1u
#endif

#ifndef YAW_TURN
#define YAW_TURN (g_config.gimbal.yaw_turn)
#endif

#ifndef CHASSIS_GIMBAL_YAW_RELATIVE_TURN
#define CHASSIS_GIMBAL_YAW_RELATIVE_TURN YAW_TURN
#endif

// "Small gyro" (小陀螺) is implemented as a constant chassis yaw rate (wz) while:
// - NOT following gimbal yaw (open-loop rotation speed set-point)
// - Keeping translation (vx/vy) in the gimbal frame (rotate by -yaw_relative), so
//   the driver can still move normally while the chassis is spinning.

/**
  * @brief          when chassis behaviour mode is CHASSIS_ZERO_FORCE, the function is called
  *                 and chassis control mode is raw. The raw chassis control mode means set value
  *                 will be sent to CAN bus derectly, and the function will set all speed zero.
  * @param[out]     vx_can_set: vx speed value, it will be sent to CAN bus derectly.
  * @param[out]     vy_can_set: vy speed value, it will be sent to CAN bus derectly.
  * @param[out]     wz_can_set: wz rotate speed value, it will be sent to CAN bus derectly.
  * @param[in]      ChassisMoveRcToVector: chassis data
  * @retval         none
  */
/**
  * @brief          底盘无力的行为状态机下，底盘模式是raw，故而设定值会直接发送到can总线上故而将设定值都设置为0
  * @author         RM
  * @param[in]      vx_set前进的速度 设定值将直接发送到can总线上
  * @param[in]      vy_set左右的速度 设定值将直接发送到can总线上
  * @param[in]      wz_set旋转的速度 设定值将直接发送到can总线上
  * @param[in]      ChassisMoveRcToVector底盘数据
  * @retval         返回空
  */
static void ChassisZeroForceControl(fp32 *vx_can_set, fp32 *vy_can_set, fp32 *wz_can_set, ChassisMove *ChassisMoveRcToVector);


/**
  * @brief          when chassis behaviour mode is CHASSIS_NO_MOVE, chassis control mode is speed control mode.
  *                 chassis does not follow gimbal, and the function will set all speed zero to make chassis no move
  * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
  * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
  * @param[out]     wz_set: wz rotate speed value, positive value means counterclockwise , negative value means clockwise.
  * @param[in]      ChassisMoveRcToVector: chassis data
  * @retval         none
  */
/**
  * @brief          底盘不移动的行为状态机下，底盘模式是不跟随角度，
  * @author         RM
  * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
  * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
  * @param[in]      wz_set旋转的速度，旋转速度是控制底盘的底盘角速度
  * @param[in]      ChassisMoveRcToVector底盘数据
  * @retval         返回空
  */
static void ChassisNoMoveControl(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, ChassisMove *ChassisMoveRcToVector);

/**
  * @brief          when chassis behaviour mode is CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW, chassis control mode is speed control mode.
  *                 chassis will follow gimbal, chassis rotation speed is calculated from the angle difference.
  * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
  * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
  * @param[out]     angle_set: control angle difference between chassis and gimbal
  * @param[in]      ChassisMoveRcToVector: chassis data
  * @retval         none
  */
/**
  * @brief          底盘跟随云台的行为状态机下，底盘模式是跟随云台角度，底盘旋转速度会根据角度差计算底盘旋转的角速度
  * @author         RM
  * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
  * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
  * @param[in]      angle_set底盘与云台控制到的相对角度
  * @param[in]      ChassisMoveRcToVector底盘数据
  * @retval         返回空
  */
static void ChassisInfantryFollowGimbalYawControl(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, ChassisMove *ChassisMoveRcToVector);

// Small gyro: keep vx/vy in gimbal frame, while commanding a constant chassis wz (no yaw follow).
static fp32 ChassisGetGimbalYawRelativeAngle(const ChassisMove *control);
static bool_t ChassisGimbalTurnaroundIsActive(const ChassisMove *control);
static fp32 ChassisGimbalTurnaroundChassisFollowOffsetRad(const ChassisMove *control);
static bool_t ChassisGimbalTurnaroundGetFrameYawRelative(const ChassisMove *control,
                                                         fp32 *out_yaw_relative);
static bool_t ChassisGimbalCmdToChassisStop(const ChassisMove *control);
static bool_t ChassisGimbalFollowAvailable(const ChassisMove *control);
static bool_t ChassisBehaviourNeedsGimbalFollow(ChassisBehaviour mode);
static void ChassisGyroSpinControl(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, ChassisMove *ChassisMoveRcToVector);
static void ChassisGyroSpinVarControl(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, ChassisMove *ChassisMoveRcToVector);
static void ChassisSwingControl(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, ChassisMove *ChassisMoveRcToVector);
static void ChassisAlgorithmMoveResetSlew(void);
static bool_t ChassisAlgorithmMoveGetActive(ExternalMotionIntent *out);
static fp32 ChassisAlgorithmLimitStep(fp32 current, fp32 target, fp32 max_delta);
static fp32 ChassisAlgorithmDefaultAccel(fp32 positive_limit, fp32 negative_limit, fp32 fallback);
static void ChassisAlgorithmApplySlew(const ExternalMotionIntent *cmd,
                                         const ChassisMove *ChassisMoveRcToVector,
                                         fp32 *vx_set,
                                         fp32 *vy_set,
                                         fp32 *wz_set,
                                         fp32 start_vx,
                                         fp32 start_vy,
                                         fp32 start_wz,
                                         uint8_t slew_wz);
static void ChassisAlgorithmMoveOverride(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set,
                                            ChassisMove *ChassisMoveRcToVector);

/**
  * @brief          when chassis behaviour mode is CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW, chassis control mode is speed control mode.
  *                 chassis will follow chassis yaw, chassis rotation speed is calculated from the angle difference between set angle and chassis yaw.
  * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
  * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
  * @param[out]     angle_set: control angle[-PI, PI]
  * @param[in]      ChassisMoveRcToVector: chassis data
  * @retval         none
  */
/**
  * @brief          底盘跟随底盘yaw的行为状态机下，底盘模式是跟随底盘角度，底盘旋转速度会根据角度差计算底盘旋转的角速度
  * @author         RM
  * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
  * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
  * @param[in]      angle_set底盘设置的yaw，范围 -PI到PI
  * @param[in]      ChassisMoveRcToVector底盘数据
  * @retval         返回空
  */
static void ChassisEngineerFollowChassisYawControl(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, ChassisMove *ChassisMoveRcToVector);

/**
  * @brief          when chassis behaviour mode is CHASSIS_NO_FOLLOW_YAW, chassis control mode is speed control mode.
  *                 chassis will no follow angle, chassis rotation speed is set by wz_set.
  * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
  * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
  * @param[out]     wz_set: rotation speed,positive value means counterclockwise , negative value means clockwise
  * @param[in]      ChassisMoveRcToVector: chassis data
  * @retval         none
  */
/**
  * @brief          底盘不跟随角度的行为状态机下，底盘模式是不跟随角度，底盘旋转速度由参数直接设定
  * @author         RM
  * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
  * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
  * @param[in]      wz_set底盘设置的旋转速度,正值 逆时针旋转，负值 顺时针旋转
  * @param[in]      ChassisMoveRcToVector底盘数据
  * @retval         返回空
  */
static void ChassisNoFollowYawControl(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, ChassisMove *ChassisMoveRcToVector);



/**
  * @brief          when chassis behaviour mode is CHASSIS_OPEN, chassis control mode is raw control mode.
  *                 set value will be sent to can bus.
  * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
  * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
  * @param[out]     wz_set: rotation speed,positive value means counterclockwise , negative value means clockwise
  * @param[in]      ChassisMoveRcToVector: chassis data
  * @retval         none
  */
/**
  * @brief          底盘开环的行为状态机下，底盘模式是raw原生状态，故而设定值会直接发送到can总线上
  * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
  * @param[in]      vy_set左右的速度，正值 左移速度， 负值 右移速度
  * @param[in]      wz_set 旋转速度， 正值 逆时针旋转，负值 顺时针旋转
  * @param[in]      ChassisMoveRcToVector底盘数据
  * @retval         none
  */

static void ChassisOpenSetControl(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, ChassisMove *ChassisMoveRcToVector);






//highlight, the variable chassis behaviour mode
//留意，这个底盘行为模式变量
ChassisBehaviour ChassisBehaviourMode = CHASSIS_ZERO_FORCE;
static ChassisAlgorithmDebug s_chassis_algorithm_debug = {0};

typedef struct
{
    uint8_t active;
    uint8_t mode;
    uint8_t frame;
    uint32_t last_ms;
    fp32 vx;
    fp32 vy;
    fp32 wz;
} ChassisAlgorithmMoveSlew;

static ChassisAlgorithmMoveSlew s_move_slew = {0};

static bool_t ChassisGimbalTurnaroundIsActive(const ChassisMove *control)
{
    return (control != NULL &&
            control->fast.gimbal.valid != 0u &&
            control->fast.gimbal.turnaroundActive != 0u) ? 1 : 0;
}

static fp32 ChassisGimbalTurnaroundChassisFollowOffsetRad(const ChassisMove *control)
{
    return (control != NULL && control->fast.gimbal.valid != 0u) ?
               control->fast.gimbal.followOffsetRad :
               0.0f;
}

static bool_t ChassisGimbalTurnaroundGetFrameYawRelative(const ChassisMove *control,
                                                         fp32 *out_yaw_relative)
{
    if (control == NULL ||
        out_yaw_relative == NULL ||
        control->fast.gimbal.valid == 0u ||
        control->fast.gimbal.frameValid == 0u)
    {
        return 0;
    }

    *out_yaw_relative = control->fast.gimbal.frameYawRelative;
    return 1;
}

static bool_t ChassisGimbalCmdToChassisStop(const ChassisMove *control)
{
#if CHASSIS_STOP_ON_GIMBAL_STATE
    return (control != NULL &&
            control->fast.gimbal.valid != 0u &&
            control->fast.gimbal.chassisStop != 0u) ? 1 : 0;
#else
    (void)control;
    return 0;
#endif
}

static bool_t ChassisGimbalFollowAvailable(const ChassisMove *control)
{
    return (control != NULL &&
            control->fast.gimbal.valid != 0u &&
            control->fast.gimbal.followAvailable != 0u) ? 1 : 0;
}

static bool_t ChassisBehaviourNeedsGimbalFollow(ChassisBehaviour mode)
{
    return (mode == CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW || mode == CHASSIS_SWING) ? 1 : 0;
}

static uint16_t ChassisGetEffectiveSwitch(uint16_t raw_sw,
                                             uint8_t manual_online,
                                             uint8_t spin_pos,
                                             uint8_t follow_pos,
                                             uint8_t safe_pos)
{
    static uint8_t gate_inited = 0u;
    static uint8_t down_engaged = 0u;
    static uint16_t last_sw_raw = RC_SW_UP;
    const uint16_t follow_raw = (uint16_t)ControlInputSwitchPosToRaw(follow_pos);
    uint16_t effective_sw = raw_sw;

    if (manual_online == 0u)
    {
        gate_inited = 0u;
        down_engaged = 0u;
        last_sw_raw = (uint16_t)ControlInputSwitchPosToRaw(safe_pos);
        return raw_sw;
    }

    if (gate_inited == 0u)
    {
        gate_inited = 1u;
        down_engaged = 0u;
        last_sw_raw = raw_sw;
    }

    if (!ControlInputSwitchIsPos(raw_sw, spin_pos))
    {
        down_engaged = 0u;
    }
    else if (!ControlInputSwitchIsPos(last_sw_raw, spin_pos))
    {
        down_engaged = 1u;
    }

    if (ControlInputSwitchIsPos(raw_sw, spin_pos) && down_engaged == 0u)
    {
        effective_sw = follow_raw;
    }

    last_sw_raw = raw_sw;
    return effective_sw;
}

static bool_t ChassisAlgorithmMoveGetActive(ExternalMotionIntent *out)
{
    ExternalMotionIntent cmd;

    if (ExternalMotionIntentReadLatest(&cmd, NULL) == false)
    {
        return 0;
    }
    if (cmd.mode == (uint8_t)EXTERNAL_MOTION_MODE_IDLE ||
        cmd.mode > (uint8_t)EXTERNAL_MOTION_MODE_STOP)
    {
        ChassisAlgorithmMoveResetSlew();
        return 0;
    }
    if (cmd.frame > (uint8_t)EXTERNAL_MOTION_FRAME_FIELD)
    {
        ChassisAlgorithmMoveResetSlew();
        return 0;
    }

    if (out != NULL)
    {
        *out = cmd;
    }
    return 1;
}

static void ChassisAlgorithmMoveResetSlew(void)
{
    memset(&s_move_slew, 0, sizeof(s_move_slew));
}

void ChassisAlgorithmDebugRead(ChassisAlgorithmDebug *out)
{
    if (out == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    *out = s_chassis_algorithm_debug;
    taskEXIT_CRITICAL();
}

static int16_t ChassisAlgorithmDebugFp32ToI16(fp32 value, fp32 scale)
{
    fp32 scaled = value * scale;

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

static void ChassisAlgorithmDebugRecord(fp32 vx, fp32 vy, fp32 wz)
{
    taskENTER_CRITICAL();
    s_chassis_algorithm_debug.count++;
    s_chassis_algorithm_debug.last_ms = BspTimeGetTickMs();
    s_chassis_algorithm_debug.vx_cmps = ChassisAlgorithmDebugFp32ToI16(vx, 100.0f);
    s_chassis_algorithm_debug.vy_cmps = ChassisAlgorithmDebugFp32ToI16(vy, 100.0f);
    s_chassis_algorithm_debug.wz_mradps = ChassisAlgorithmDebugFp32ToI16(wz, 1000.0f);
    taskEXIT_CRITICAL();
}

static fp32 ChassisAlgorithmYawFrame(const ChassisMove *ChassisMoveRcToVector)
{
    if (ChassisMoveRcToVector == NULL)
    {
        return 0.0f;
    }

    fp32 yaw_frame = ChassisGetGimbalYawRelativeAngle(ChassisMoveRcToVector);
    (void)ChassisGimbalTurnaroundGetFrameYawRelative(ChassisMoveRcToVector, &yaw_frame);
    return yaw_frame;
}

static void ChassisAlgorithmFieldToChassis(const ChassisMove *ChassisMoveRcToVector,
                                               fp32 vx,
                                               fp32 vy,
                                               fp32 *vx_out,
                                               fp32 *vy_out)
{
    if (vx_out == NULL || vy_out == NULL)
    {
        return;
    }

    const fp32 yaw = (ChassisMoveRcToVector != NULL) ? ChassisMoveRcToVector->ChassisYaw : 0.0f;
    const fp32 sin_yaw = arm_sin_f32(yaw);
    const fp32 cos_yaw = arm_cos_f32(yaw);

    *vx_out = cos_yaw * vx + sin_yaw * vy;
    *vy_out = -sin_yaw * vx + cos_yaw * vy;
}

static void ChassisAlgorithmChassisToGimbal(const ChassisMove *ChassisMoveRcToVector,
                                                fp32 vx,
                                                fp32 vy,
                                                fp32 *vx_out,
                                                fp32 *vy_out)
{
    if (vx_out == NULL || vy_out == NULL)
    {
        return;
    }

    const fp32 yaw = ChassisAlgorithmYawFrame(ChassisMoveRcToVector);
    const fp32 sin_yaw = arm_sin_f32(yaw);
    const fp32 cos_yaw = arm_cos_f32(yaw);

    *vx_out = cos_yaw * vx + sin_yaw * vy;
    *vy_out = -sin_yaw * vx + cos_yaw * vy;
}

static void ChassisAlgorithmGimbalToChassis(const ChassisMove *ChassisMoveRcToVector,
                                                fp32 vx,
                                                fp32 vy,
                                                fp32 *vx_out,
                                                fp32 *vy_out)
{
    if (vx_out == NULL || vy_out == NULL)
    {
        return;
    }

    const fp32 yaw = ChassisAlgorithmYawFrame(ChassisMoveRcToVector);
    const fp32 sin_yaw = arm_sin_f32(yaw);
    const fp32 cos_yaw = arm_cos_f32(yaw);

    *vx_out = cos_yaw * vx - sin_yaw * vy;
    *vy_out = sin_yaw * vx + cos_yaw * vy;
}

static void ChassisAlgorithmResolveVxy(const ExternalMotionIntent *cmd,
                                          const ChassisMove *ChassisMoveRcToVector,
                                          uint8_t desired_frame,
                                          fp32 *vx_out,
                                          fp32 *vy_out)
{
    if (vx_out == NULL || vy_out == NULL)
    {
        return;
    }

    fp32 vx = 0.0f;
    fp32 vy = 0.0f;
    if (cmd != NULL && (cmd->flags & EXTERNAL_MOTION_FLAG_VXY_VALID) != 0u)
    {
        vx = cmd->vx_mps;
        vy = cmd->vy_mps;
    }

    if (cmd == NULL || cmd->frame == desired_frame)
    {
        *vx_out = vx;
        *vy_out = vy;
        return;
    }

    if (cmd->frame == (uint8_t)EXTERNAL_MOTION_FRAME_FIELD)
    {
        fp32 vx_chassis = 0.0f;
        fp32 vy_chassis = 0.0f;
        ChassisAlgorithmFieldToChassis(ChassisMoveRcToVector, vx, vy, &vx_chassis, &vy_chassis);
        if (desired_frame == (uint8_t)EXTERNAL_MOTION_FRAME_CHASSIS)
        {
            *vx_out = vx_chassis;
            *vy_out = vy_chassis;
            return;
        }
        if (desired_frame == (uint8_t)EXTERNAL_MOTION_FRAME_GIMBAL)
        {
            ChassisAlgorithmChassisToGimbal(ChassisMoveRcToVector, vx_chassis, vy_chassis, vx_out, vy_out);
            return;
        }
    }

    if (cmd->frame == (uint8_t)EXTERNAL_MOTION_FRAME_GIMBAL &&
        desired_frame == (uint8_t)EXTERNAL_MOTION_FRAME_CHASSIS)
    {
        ChassisAlgorithmGimbalToChassis(ChassisMoveRcToVector, vx, vy, vx_out, vy_out);
        return;
    }

    if (cmd->frame == (uint8_t)EXTERNAL_MOTION_FRAME_CHASSIS &&
        desired_frame == (uint8_t)EXTERNAL_MOTION_FRAME_GIMBAL)
    {
        ChassisAlgorithmChassisToGimbal(ChassisMoveRcToVector, vx, vy, vx_out, vy_out);
        return;
    }

    *vx_out = 0.0f;
    *vy_out = 0.0f;
}

static fp32 ChassisAlgorithmLimitStep(fp32 current, fp32 target, fp32 max_delta)
{
    const fp32 delta = target - current;
    const fp32 limit = (max_delta > 0.0f) ? max_delta : 0.0f;

    if (delta > limit)
    {
        return current + limit;
    }
    if (delta < -limit)
    {
        return current - limit;
    }
    return target;
}

static fp32 ChassisAlgorithmDefaultAccel(fp32 positive_limit, fp32 negative_limit, fp32 fallback)
{
    fp32 max_abs = fabsf(positive_limit);
    const fp32 neg_abs = fabsf(negative_limit);

    if (max_abs < neg_abs)
    {
        max_abs = neg_abs;
    }
    if (max_abs <= 0.0f)
    {
        max_abs = fallback;
    }
    return max_abs * 4.0f;
}

static void ChassisAlgorithmApplySlew(const ExternalMotionIntent *cmd,
                                         const ChassisMove *ChassisMoveRcToVector,
                                         fp32 *vx_set,
                                         fp32 *vy_set,
                                         fp32 *wz_set,
                                         fp32 start_vx,
                                         fp32 start_vy,
                                         fp32 start_wz,
                                         uint8_t slew_wz)
{
    if (cmd == NULL || ChassisMoveRcToVector == NULL ||
        vx_set == NULL || vy_set == NULL || wz_set == NULL)
    {
        return;
    }

    const uint32_t now_ms = BspTimeGetTickMs();
    if (s_move_slew.active == 0u ||
        s_move_slew.mode != cmd->mode ||
        s_move_slew.frame != cmd->frame ||
        now_ms < s_move_slew.last_ms ||
        (now_ms - s_move_slew.last_ms) > 300u)
    {
        s_move_slew.active = 1u;
        s_move_slew.mode = cmd->mode;
        s_move_slew.frame = cmd->frame;
        s_move_slew.last_ms = now_ms;
        s_move_slew.vx = start_vx;
        s_move_slew.vy = start_vy;
        s_move_slew.wz = start_wz;
    }

    uint32_t dt_ms = now_ms - s_move_slew.last_ms;
    if (dt_ms == 0u)
    {
        dt_ms = CHASSIS_CONTROL_TIME_MS;
    }
    const fp32 dt = (fp32)dt_ms * 0.001f;
    const uint8_t accel_valid = ((cmd->flags & EXTERNAL_MOTION_FLAG_ACCEL_VALID) != 0u) ? 1u : 0u;
    const ChassisControlSnapshot *fast = &ChassisMoveRcToVector->fast;

    fp32 ax_limit = ChassisAlgorithmDefaultAccel(ChassisMoveRcToVector->vx_max_speed,
                                                   ChassisMoveRcToVector->vx_min_speed,
                                                   1.0f);
    fp32 ay_limit = ChassisAlgorithmDefaultAccel(ChassisMoveRcToVector->vy_max_speed,
                                                   ChassisMoveRcToVector->vy_min_speed,
                                                   1.0f);
    fp32 wz_limit = fabsf(fast->swing_move_angle) * 4.0f;
    if (wz_limit <= 0.0f)
    {
        wz_limit = 8.0f;
    }

    if (accel_valid != 0u)
    {
        if (fabsf(cmd->ax_mps2) > 0.0f)
        {
            ax_limit = fabsf(cmd->ax_mps2);
        }
        if (fabsf(cmd->ay_mps2) > 0.0f)
        {
            ay_limit = fabsf(cmd->ay_mps2);
        }
        if (fabsf(cmd->wz_acc_radps2) > 0.0f)
        {
            wz_limit = fabsf(cmd->wz_acc_radps2);
        }
    }

    s_move_slew.vx = ChassisAlgorithmLimitStep(s_move_slew.vx, *vx_set, ax_limit * dt);
    s_move_slew.vy = ChassisAlgorithmLimitStep(s_move_slew.vy, *vy_set, ay_limit * dt);
    if (slew_wz != 0u)
    {
        s_move_slew.wz = ChassisAlgorithmLimitStep(s_move_slew.wz, *wz_set, wz_limit * dt);
    }
    else
    {
        s_move_slew.wz = *wz_set;
    }
    s_move_slew.last_ms = now_ms;

    *vx_set = s_move_slew.vx;
    *vy_set = s_move_slew.vy;
    *wz_set = s_move_slew.wz;
}

static void ChassisAlgorithmMoveOverride(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set,
                                            ChassisMove *ChassisMoveRcToVector)
{
    ExternalMotionIntent cmd;

    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || ChassisMoveRcToVector == NULL)
    {
        return;
    }
    if (ChassisBehaviourMode == CHASSIS_ZERO_FORCE || ChassisBehaviourMode == CHASSIS_NO_MOVE)
    {
        ChassisAlgorithmMoveResetSlew();
        return;
    }
    if (ChassisAlgorithmMoveGetActive(&cmd) == 0)
    {
        ChassisAlgorithmMoveResetSlew();
        return;
    }

    const fp32 start_vx = *vx_set;
    const fp32 start_vy = *vy_set;
    const fp32 start_wz = *angle_set;

    if (cmd.mode == (uint8_t)EXTERNAL_MOTION_MODE_STOP)
    {
        ChassisAlgorithmMoveResetSlew();
        *vx_set = 0.0f;
        *vy_set = 0.0f;
        *angle_set = 0.0f;
        ChassisAlgorithmDebugRecord(*vx_set, *vy_set, *angle_set);
        return;
    }

    if (cmd.mode == (uint8_t)EXTERNAL_MOTION_MODE_FOLLOW_GIMBAL)
    {
        ChassisAlgorithmResolveVxy(&cmd,
                                      ChassisMoveRcToVector,
                                      (uint8_t)EXTERNAL_MOTION_FRAME_GIMBAL,
                                      vx_set,
                                      vy_set);
        *angle_set = ((cmd.flags & EXTERNAL_MOTION_FLAG_YAW_OFFSET_VALID) != 0u) ? cmd.yaw_offset_rad : 0.0f;
        ChassisAlgorithmApplySlew(&cmd, ChassisMoveRcToVector, vx_set, vy_set, angle_set,
                                     start_vx, start_vy, start_wz, 0u);
        ChassisAlgorithmDebugRecord(*vx_set, *vy_set, *angle_set);
        return;
    }

    if (cmd.mode == (uint8_t)EXTERNAL_MOTION_MODE_NO_FOLLOW ||
        cmd.mode == (uint8_t)EXTERNAL_MOTION_MODE_GYRO_SPIN)
    {
        ChassisAlgorithmResolveVxy(&cmd,
                                      ChassisMoveRcToVector,
                                      (uint8_t)EXTERNAL_MOTION_FRAME_CHASSIS,
                                      vx_set,
                                      vy_set);

        if ((cmd.flags & EXTERNAL_MOTION_FLAG_WZ_VALID) != 0u)
        {
            *angle_set = cmd.wz_radps;
        }
        else if (cmd.mode == (uint8_t)EXTERNAL_MOTION_MODE_GYRO_SPIN)
        {
            const ChassisControlSnapshot *fast = &ChassisMoveRcToVector->fast;
            *angle_set = fast->swing_move_angle;
        }
        else
        {
            *angle_set = 0.0f;
        }
        ChassisAlgorithmApplySlew(&cmd, ChassisMoveRcToVector, vx_set, vy_set, angle_set,
                                     start_vx, start_vy, start_wz, 1u);
        ChassisAlgorithmDebugRecord(*vx_set, *vy_set, *angle_set);
    }
}


/**
  * @brief          logical judgement to assign "ChassisBehaviourMode" variable to which mode
  * @param[in]      ChassisMoveMode: chassis data
  * @retval         none
  */
/**
  * @brief          通过逻辑判断，赋值"ChassisBehaviourMode"成哪种模式
  * @param[in]      ChassisMoveMode: 底盘数据
  * @retval         none
  */
void ChassisBehaviourModeSet(ChassisMove *ChassisMoveMode)
{
    if (ChassisMoveMode == NULL)
    {
        return;
    }

    // 函数地图：先处理安全档/运行模式；再按拨杆和按键选行为；最后映射到底盘控制模式。
    const ChassisControlSnapshot *fast = &ChassisMoveMode->fast;
    const uint16_t ChassisSw = fast->mode_sw;
    const uint16_t ChassisSwEffective = ChassisGetEffectiveSwitch(ChassisSw,
                                                                       fast->manual_online,
                                                                       fast->spin_pos,
                                                                       fast->follow_pos,
                                                                       fast->safe_pos);

    // 优先级最高的安全模式：拨杆上档立即停转所有底盘电机
    if (ControlInputSwitchIsPos(ChassisSw, fast->safe_pos))
    {
        ChassisBehaviourMode = CHASSIS_ZERO_FORCE;
        ChassisMoveMode->mode = CHASSIS_VECTOR_RAW;
        return;
    }

    // chassis-only test mode: ignore gimbal follow and always use yaw stick to rotate chassis
    if (fast->ChassisOnlyMode != 0u)
    {
        ChassisBehaviourMode = CHASSIS_NO_FOLLOW_YAW;
        ChassisMoveMode->mode = CHASSIS_VECTOR_NO_FOLLOW_YAW;
        return;
    }


    //remote control  set chassis behaviour mode
    //遥控器设置模式
    if (ChassisGimbalTurnaroundIsActive(ChassisMoveMode))
    {
        ChassisBehaviourMode = CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW;
    }
    else if (ControlInputSwitchIsPos(ChassisSwEffective, fast->follow_pos))
    {
        // 普通模式：底盘跟随云台 yaw（让云台 yaw 回中，底盘转向对齐云台朝向）
        // CTRL 按下：进入小陀螺持续自转（不跟随云台角度）
        const uint16_t key_mask = fast->key_mask;
        const uint8_t want_swing = ((key_mask & fast->swing_key_mask) != 0u) ? 1u : 0u;
        const uint8_t want_gyro = ((key_mask & fast->gyro_spin_key_mask) != 0u) ? 1u : 0u;
        const uint8_t want_gyro_var = ((key_mask & fast->gyro_spin_var_key_mask) != 0u) ? 1u : 0u;
        if (want_swing != 0u)
        {
            ChassisBehaviourMode = CHASSIS_SWING;
        }
        else if (want_gyro_var != 0u)
        {
            ChassisBehaviourMode = CHASSIS_GYRO_SPIN_VAR;
        }
        else
        {
            ChassisBehaviourMode = want_gyro ? CHASSIS_GYRO_SPIN : CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW;
        }
    }
    else if (ControlInputSwitchIsPos(ChassisSwEffective, fast->spin_pos))
    {
        // 小陀螺：持续自转（类似 WH 的 SelfProtect），同时保留平移控制
        ChassisBehaviourMode = ((fast->key_mask & fast->gyro_spin_var_key_mask) != 0u) ?
                                     CHASSIS_GYRO_SPIN_VAR :
                                     CHASSIS_GYRO_SPIN;
    }

    //when gimbal in some mode, such as init mode, chassis must's move
    //当云台在某些模式下，像初始化， 底盘不动
    if (ChassisGimbalCmdToChassisStop(ChassisMoveMode))
    {
        ChassisBehaviourMode = CHASSIS_NO_MOVE;
    }
    else
    {
        ExternalMotionIntent move_cmd;
        if (ChassisAlgorithmMoveGetActive(&move_cmd) != 0)
        {
            if (move_cmd.mode == (uint8_t)EXTERNAL_MOTION_MODE_FOLLOW_GIMBAL)
            {
                ChassisBehaviourMode = CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW;
            }
            else if (move_cmd.mode == (uint8_t)EXTERNAL_MOTION_MODE_NO_FOLLOW)
            {
                ChassisBehaviourMode = CHASSIS_NO_FOLLOW_YAW;
            }
            else if (move_cmd.mode == (uint8_t)EXTERNAL_MOTION_MODE_GYRO_SPIN)
            {
                ChassisBehaviourMode = CHASSIS_GYRO_SPIN;
            }
            else if (move_cmd.mode == (uint8_t)EXTERNAL_MOTION_MODE_STOP)
            {
                ChassisBehaviourMode = CHASSIS_NO_MOVE;
            }
        }
    }

    /* 云台不可用时只放弃角度跟随，底盘平移和手动转向仍可继续。 */
    if (ChassisBehaviourNeedsGimbalFollow(ChassisBehaviourMode) != 0 &&
        ChassisGimbalFollowAvailable(ChassisMoveMode) == 0)
    {
        ChassisBehaviourMode = CHASSIS_NO_FOLLOW_YAW;
    }


    //add your own logic to enter the new mode
    //添加自己的逻辑判断进入新模式


    //accord to beheviour mode, choose chassis control mode
    //根据行为模式选择一个底盘控制模式
    if (ChassisBehaviourMode == CHASSIS_ZERO_FORCE)
    {
        ChassisMoveMode->mode = CHASSIS_VECTOR_RAW;
    }
    else if (ChassisBehaviourMode == CHASSIS_NO_MOVE)
    {
        ChassisMoveMode->mode = CHASSIS_VECTOR_NO_FOLLOW_YAW;
    }
    else if (ChassisBehaviourMode == CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW)
    {
        ChassisMoveMode->mode = CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW;
    }
    else if (ChassisBehaviourMode == CHASSIS_SWING)
    {
        ChassisMoveMode->mode = CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW;
    }
    else if (ChassisBehaviourMode == CHASSIS_GYRO_SPIN)
    {
        ChassisMoveMode->mode = CHASSIS_VECTOR_NO_FOLLOW_YAW;
    }
    else if (ChassisBehaviourMode == CHASSIS_GYRO_SPIN_VAR)
    {
        ChassisMoveMode->mode = CHASSIS_VECTOR_NO_FOLLOW_YAW;
    }
    else if (ChassisBehaviourMode == CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW)
    {
        ChassisMoveMode->mode = CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW;
    }
    else if (ChassisBehaviourMode == CHASSIS_NO_FOLLOW_YAW)
    {
        ChassisMoveMode->mode = CHASSIS_VECTOR_NO_FOLLOW_YAW;
    }
    else if (ChassisBehaviourMode == CHASSIS_OPEN)
    {
        ChassisMoveMode->mode = CHASSIS_VECTOR_RAW;
    }
}


/**
  * @brief          set control set-point. three movement param, according to difference control mode,
  *                 will control corresponding movement.in the function, usually call different control function.
  * @param[out]     vx_set, usually controls vertical speed.
  * @param[out]     vy_set, usually controls horizotal speed.
  * @param[out]     wz_set, usually controls rotation speed.
  * @param[in]      ChassisMoveRcToVector,  has all data of chassis
  * @retval         none
  */
/**
  * @brief          设置控制量.根据不同底盘控制模式，三个参数会控制不同运动.在这个函数里面，会调用不同的控制函数.
  * @param[out]     vx_set, 通常控制纵向移动.
  * @param[out]     vy_set, 通常控制横向移动.
  * @param[out]     wz_set, 通常控制旋转运动.
  * @param[in]      ChassisMoveRcToVector,  包括底盘所有信息.
  * @retval         none
  */

void ChassisBehaviourControlSet(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, ChassisMove *ChassisMoveRcToVector)
{

    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || ChassisMoveRcToVector == NULL)
    {
        return;
    }

    if (ChassisBehaviourMode == CHASSIS_ZERO_FORCE)
    {
        ChassisZeroForceControl(vx_set, vy_set, angle_set, ChassisMoveRcToVector);
    }
    else if (ChassisBehaviourMode == CHASSIS_NO_MOVE)
    {
        ChassisNoMoveControl(vx_set, vy_set, angle_set, ChassisMoveRcToVector);
    }
    else if (ChassisBehaviourMode == CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW)
    {
        ChassisInfantryFollowGimbalYawControl(vx_set, vy_set, angle_set, ChassisMoveRcToVector);
    }
    else if (ChassisBehaviourMode == CHASSIS_SWING)
    {
        ChassisSwingControl(vx_set, vy_set, angle_set, ChassisMoveRcToVector);
    }
    else if (ChassisBehaviourMode == CHASSIS_GYRO_SPIN)
    {
        ChassisGyroSpinControl(vx_set, vy_set, angle_set, ChassisMoveRcToVector);
    }
    else if (ChassisBehaviourMode == CHASSIS_GYRO_SPIN_VAR)
    {
        ChassisGyroSpinVarControl(vx_set, vy_set, angle_set, ChassisMoveRcToVector);
    }
    else if (ChassisBehaviourMode == CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW)
    {
        ChassisEngineerFollowChassisYawControl(vx_set, vy_set, angle_set, ChassisMoveRcToVector);
    }
    else if (ChassisBehaviourMode == CHASSIS_NO_FOLLOW_YAW)
    {
        ChassisNoFollowYawControl(vx_set, vy_set, angle_set, ChassisMoveRcToVector);
    }
    else if (ChassisBehaviourMode == CHASSIS_OPEN)
    {
        ChassisOpenSetControl(vx_set, vy_set, angle_set, ChassisMoveRcToVector);
    }

    ChassisAlgorithmMoveOverride(vx_set, vy_set, angle_set, ChassisMoveRcToVector);
}

/**
  * @brief          when chassis behaviour mode is CHASSIS_ZERO_FORCE, the function is called
  *                 and chassis control mode is raw. The raw chassis chontrol mode means set value
  *                 will be sent to CAN bus derectly, and the function will set all speed zero.
  * @param[out]     vx_can_set: vx speed value, it will be sent to CAN bus derectly.
  * @param[out]     vy_can_set: vy speed value, it will be sent to CAN bus derectly.
  * @param[out]     wz_can_set: wz rotate speed value, it will be sent to CAN bus derectly.
  * @param[in]      ChassisMoveRcToVector: chassis data
  * @retval         none
  */
/**
  * @brief          底盘无力的行为状态机下，底盘模式是raw，故而设定值会直接发送到can总线上故而将设定值都设置为0
  * @author         RM
  * @param[in]      vx_set前进的速度 设定值将直接发送到can总线上
  * @param[in]      vy_set左右的速度 设定值将直接发送到can总线上
  * @param[in]      wz_set旋转的速度 设定值将直接发送到can总线上
  * @param[in]      ChassisMoveRcToVector底盘数据
  * @retval         返回空
  */

static void ChassisZeroForceControl(fp32 *vx_can_set, fp32 *vy_can_set, fp32 *wz_can_set, ChassisMove *ChassisMoveRcToVector)
{
    if (vx_can_set == NULL || vy_can_set == NULL || wz_can_set == NULL || ChassisMoveRcToVector == NULL)
    {
        return;
    }
    *vx_can_set = 0.0f;
    *vy_can_set = 0.0f;
    *wz_can_set = 0.0f;
}

/**
  * @brief          when chassis behaviour mode is CHASSIS_NO_MOVE, chassis control mode is speed control mode.
  *                 chassis does not follow gimbal, and the function will set all speed zero to make chassis no move
  * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
  * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
  * @param[out]     wz_set: wz rotate speed value, positive value means counterclockwise , negative value means clockwise.
  * @param[in]      ChassisMoveRcToVector: chassis data
  * @retval         none
  */
/**
  * @brief          底盘不移动的行为状态机下，底盘模式是不跟随角度，
  * @author         RM
  * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
  * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
  * @param[in]      wz_set旋转的速度，旋转速度是控制底盘的底盘角速度
  * @param[in]      ChassisMoveRcToVector底盘数据
  * @retval         返回空
  */

static void ChassisNoMoveControl(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, ChassisMove *ChassisMoveRcToVector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || ChassisMoveRcToVector == NULL)
    {
        return;
    }
    *vx_set = 0.0f;
    *vy_set = 0.0f;
    *wz_set = 0.0f;
}

/**
  * @brief          when chassis behaviour mode is CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW, chassis control mode is speed control mode.
  *                 chassis will follow gimbal, chassis rotation speed is calculated from the angle difference.
  * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
  * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
  * @param[out]     angle_set: control angle difference between chassis and gimbal
  * @param[in]      ChassisMoveRcToVector: chassis data
  * @retval         none
  */
/**
  * @brief          底盘跟随云台的行为状态机下，底盘模式是跟随云台角度，底盘旋转速度会根据角度差计算底盘旋转的角速度
  * @author         RM
  * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
  * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
  * @param[in]      angle_set底盘与云台控制到的相对角度
  * @param[in]      ChassisMoveRcToVector底盘数据
  * @retval         返回空
  */

static void ChassisInfantryFollowGimbalYawControl(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, ChassisMove *ChassisMoveRcToVector)
{
    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || ChassisMoveRcToVector == NULL)
    {
        return;
    }

    //channel value and keyboard value change to speed set-point, in general
    //遥控器的通道值以及键盘按键 得出 一般情况下的速度设定值
    ChassisRcToControlVector(vx_set, vy_set, ChassisMoveRcToVector);

    if (ChassisGimbalTurnaroundIsActive(ChassisMoveRcToVector))
    {
        *angle_set = ChassisGetGimbalYawRelativeAngle(ChassisMoveRcToVector);
    }
    else
    {
        *angle_set = ChassisGimbalTurnaroundChassisFollowOffsetRad(ChassisMoveRcToVector);
    }
}

typedef struct
{
    uint8_t initialized;
    uint8_t center_idx;
    int8_t dir;
    uint8_t pending_center_advance;
    fp32 offset_rad;
    uint32_t last_ms;
    uint32_t next_center_jump_ms;
    uint32_t rng_state;
} ChassisSwingState;

static uint32_t ChassisSwingRngNext(ChassisSwingState *st)
{
    if (st == NULL)
    {
        return 0u;
    }

    uint32_t x = st->rng_state;
    if (x == 0u)
    {
        x = BspTimeGetTickMs() ^ 0x9E3779B9u;
        if (x == 0u)
        {
            x = 0x6C8E9CF5u;
        }
    }

    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    st->rng_state = x;
    return x;
}

static uint32_t ChassisSwingRandBetweenU32(ChassisSwingState *st, uint32_t min_v, uint32_t max_v)
{
    if (max_v < min_v)
    {
        const uint32_t t = min_v;
        min_v = max_v;
        max_v = t;
    }
    if (max_v == min_v)
    {
        return min_v;
    }

    const uint32_t span = max_v - min_v;
    const uint32_t r = ChassisSwingRngNext(st);
    return min_v + (r % (span + 1u));
}

static void ChassisSwingControl(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, ChassisMove *ChassisMoveRcToVector)
{
    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || ChassisMoveRcToVector == NULL)
    {
        return;
    }

    // 函数地图：先把平移转到云台参考系；再按三角波摆动 yaw；需要时随机跳到下一个中心角。
    static ChassisSwingState st;
    static const fp32 centers_rad[4] = {
        0.7853981633974483f,
        2.3561944901923450f,
        -2.3561944901923450f,
        -0.7853981633974483f,
    };
    const fp32 center_step_rad = 1.5707963267948966f;
    const uint32_t now_ms = BspTimeGetTickMs();
    const ChassisControlSnapshot *fast = &ChassisMoveRcToVector->fast;

    ChassisRcToControlVector(vx_set, vy_set, ChassisMoveRcToVector);

    // 摇摆时仍然尽量保持平移参考跟着云台，不然按键方向会发飘。
    const fp32 vx_raw = *vx_set;
    const fp32 vy_raw = *vy_set;
    const fp32 yaw_relative = ChassisGetGimbalYawRelativeAngle(ChassisMoveRcToVector);
    const fp32 sin_yaw = arm_sin_f32(-yaw_relative);
    const fp32 cos_yaw = arm_cos_f32(-yaw_relative);
    *vx_set = cos_yaw * vx_raw + sin_yaw * vy_raw;
    *vy_set = -sin_yaw * vx_raw + cos_yaw * vy_raw;

    fp32 amp = fast->swing_amp_rad;
    if (amp < 0.0f)
    {
        amp = -amp;
    }

    uint32_t half_period_ms = fast->swing_half_period_ms;
    if (half_period_ms < 10u)
    {
        half_period_ms = 10u;
    }

    uint32_t hold_min_ms = fast->swing_center_hold_min_ms;
    uint32_t hold_max_ms = fast->swing_center_hold_max_ms;
    if (hold_min_ms == 0u)
    {
        hold_min_ms = 5000u;
    }
    if (hold_max_ms == 0u)
    {
        hold_max_ms = 20000u;
    }

    if (st.initialized == 0u)
    {
        st.initialized = 1u;
        st.center_idx = (uint8_t)(ChassisSwingRngNext(&st) & 3u);
        st.dir = 1;
        st.pending_center_advance = 0u;
        st.offset_rad = 0.0f;
        st.last_ms = now_ms;
        st.next_center_jump_ms = now_ms + ChassisSwingRandBetweenU32(&st, hold_min_ms, hold_max_ms);
    }

    if (amp <= 0.0f)
    {
        *angle_set = rad_format(centers_rad[st.center_idx]);
        return;
    }

    uint32_t dt_ms = now_ms - st.last_ms;
    st.last_ms = now_ms;
    if (dt_ms > 200u)
    {
        dt_ms = 0u;
    }

    if (st.pending_center_advance == 0u && now_ms >= st.next_center_jump_ms)
    {
        st.pending_center_advance = 1u;
    }

    const fp32 rate_rad_per_ms = (2.0f * amp) / (fp32)half_period_ms;
    fp32 offset = st.offset_rad + (fp32)st.dir * rate_rad_per_ms * (fp32)dt_ms;

    if (st.dir > 0)
    {
        fp32 max_offset = amp;
        if (st.pending_center_advance != 0u)
        {
            max_offset = amp + center_step_rad;
        }

        if (offset >= max_offset)
        {
            if (st.pending_center_advance != 0u && max_offset > amp)
            {
                st.center_idx = (uint8_t)((st.center_idx + 1u) & 3u);
                st.pending_center_advance = 0u;
                st.next_center_jump_ms = now_ms + ChassisSwingRandBetweenU32(&st, hold_min_ms, hold_max_ms);
                offset = amp;
                st.dir = -1;
            }
            else
            {
                offset = amp;
                st.dir = -1;
            }
        }
    }
    else if (offset <= -amp)
    {
        offset = -amp;
        st.dir = 1;
    }

    st.offset_rad = offset;
    *angle_set = rad_format(centers_rad[st.center_idx] + offset);
}

static fp32 ChassisGetGimbalYawRelativeAngle(const ChassisMove *control)
{
    const GimbalMotorState *yaw_motor = (control != NULL) ? &control->fast.gimbal.yaw : NULL;

    if (control == NULL ||
        control->fast.gimbal.valid == 0u ||
        yaw_motor->valid == 0u ||
        yaw_motor->measure.valid == 0u)
    {
        return 0.0f;
    }

    const uint32_t ecd_range = control->fast.gimbal.yawEcdRange;
    if (ecd_range < 2u)
    {
        return 0.0f;
    }
    const int32_t half_ecd_range = (int32_t)(ecd_range / 2u);
    const int32_t full_ecd_range = (int32_t)ecd_range;
    int32_t relative_ecd = (int32_t)yaw_motor->measure.ecd - (int32_t)yaw_motor->offset_ecd;
    if (relative_ecd > half_ecd_range)
    {
        relative_ecd -= full_ecd_range;
    }
    else if (relative_ecd < -half_ecd_range)
    {
        relative_ecd += full_ecd_range;
    }

    fp32 angle = (fp32)relative_ecd * (6.28318530718f / (fp32)ecd_range);
    return (control->fast.gimbal.yawRelativeTurn != 0u) ? -angle : angle;
}

static void ChassisGyroSpinControl(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, ChassisMove *ChassisMoveRcToVector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || ChassisMoveRcToVector == NULL)
    {
        return;
    }

    // 1) Read translation command as usual (RC/keyboard), then rotate it into gimbal frame.
    const ChassisControlSnapshot *fast = &ChassisMoveRcToVector->fast;
    ChassisRcToControlVector(vx_set, vy_set, ChassisMoveRcToVector);
    const fp32 vx_raw = *vx_set;
    const fp32 vy_raw = *vy_set;

    const fp32 yaw_relative_meas = ChassisGetGimbalYawRelativeAngle(ChassisMoveRcToVector);
    fp32 yaw_frame = yaw_relative_meas;
    (void)ChassisGimbalTurnaroundGetFrameYawRelative(ChassisMoveRcToVector, &yaw_frame);
    const fp32 sin_yaw = arm_sin_f32(-yaw_frame);
    const fp32 cos_yaw = arm_cos_f32(-yaw_frame);
    *vx_set = cos_yaw * vx_raw + sin_yaw * vy_raw;
    *vy_set = -sin_yaw * vx_raw + cos_yaw * vy_raw;

    // 2) Constant spin rate (wz) like WH's "小陀螺" mode.
    // Use two presets so tuning can keep stability while moving.
    const uint16_t key_mask = fast->key_mask;
    fp32 spin_wz = fast->swing_no_move_angle;
    if ((key_mask & fast->front_key_mask) ||
        (key_mask & fast->back_key_mask) ||
        (key_mask & fast->left_key_mask) ||
        (key_mask & fast->right_key_mask) ||
        (fabsf(vx_raw) > 0.05f) ||
        (fabsf(vy_raw) > 0.05f))
    {
        spin_wz = fast->swing_move_angle;
    }
    *wz_set = spin_wz;
}

static void ChassisGyroSpinVarControl(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, ChassisMove *ChassisMoveRcToVector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || ChassisMoveRcToVector == NULL)
    {
        return;
    }

    ChassisGyroSpinControl(vx_set, vy_set, wz_set, ChassisMoveRcToVector);

    const fp32 spin_wz_base = *wz_set;
    const fp32 sign = (spin_wz_base >= 0.0f) ? 1.0f : -1.0f;
    const fp32 base_abs = fabsf(spin_wz_base);
    const uint32_t now_ms = BspTimeGetTickMs();
    const fp32 t = (fp32)now_ms * 0.001f;

    static uint32_t s_rng = 0u;
    static uint32_t s_jump_next_ms = 0u;
    static fp32 s_jump = 0.0f;
    static fp32 s_walk = 0.0f;

    if (s_rng == 0u)
    {
        s_rng = now_ms ^ 0x9E3779B9u;
        if (s_rng == 0u)
        {
            s_rng = 0x6C8E9CF5u;
        }
    }

    s_rng ^= s_rng << 13;
    s_rng ^= s_rng >> 17;
    s_rng ^= s_rng << 5;

    if (now_ms >= s_jump_next_ms)
    {
        const fp32 r01 = (fp32)(s_rng & 0xFFFFu) * (1.0f / 65535.0f);
        s_jump = (r01 - 0.5f) * 0.36f;

        s_rng ^= s_rng << 13;
        s_rng ^= s_rng >> 17;
        s_rng ^= s_rng << 5;
        s_jump_next_ms = now_ms + 150u + (s_rng % 451u);
    }

    s_rng ^= s_rng << 13;
    s_rng ^= s_rng >> 17;
    s_rng ^= s_rng << 5;
    s_walk += (((fp32)(s_rng & 0xFFFFu) * (1.0f / 65535.0f)) - 0.5f) * 0.01f;
    s_walk = fp32_constrain(s_walk, -0.10f, 0.10f);

    fp32 factor = 0.75f + 0.25f * arm_sin_f32((2.0f * PI * 1.2f) * t) + s_jump + s_walk;
    factor = fp32_constrain(factor, 0.5f, 1.0f);
    *wz_set = sign * base_abs * factor;
}

/**
  * @brief          when chassis behaviour mode is CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW, chassis control mode is speed control mode.
  *                 chassis will follow chassis yaw, chassis rotation speed is calculated from the angle difference between set angle and chassis yaw.
  * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
  * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
  * @param[out]     angle_set: control angle[-PI, PI]
  * @param[in]      ChassisMoveRcToVector: chassis data
  * @retval         none
  */
/**
  * @brief          底盘跟随底盘yaw的行为状态机下，底盘模式是跟随底盘角度，底盘旋转速度会根据角度差计算底盘旋转的角速度
  * @author         RM
  * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
  * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
  * @param[in]      angle_set底盘设置的yaw，范围 -PI到PI
  * @param[in]      ChassisMoveRcToVector底盘数据
  * @retval         返回空
  */

static void ChassisEngineerFollowChassisYawControl(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, ChassisMove *ChassisMoveRcToVector)
{
    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || ChassisMoveRcToVector == NULL)
    {
        return;
    }

    ChassisRcToControlVector(vx_set, vy_set, ChassisMoveRcToVector);

    *angle_set = rad_format(ChassisMoveRcToVector->ChassisYawSet -
                            ChassisMoveRcToVector->fast.angle_z_rc_sen * ChassisMoveRcToVector->fast.axis_wz);
}

/**
  * @brief          when chassis behaviour mode is CHASSIS_NO_FOLLOW_YAW, chassis control mode is speed control mode.
  *                 chassis will no follow angle, chassis rotation speed is set by wz_set.
  * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
  * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
  * @param[out]     wz_set: rotation speed,positive value means counterclockwise , negative value means clockwise
  * @param[in]      ChassisMoveRcToVector: chassis data
  * @retval         none
  */
/**
  * @brief          底盘不跟随角度的行为状态机下，底盘模式是不跟随角度，底盘旋转速度由参数直接设定
  * @author         RM
  * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
  * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
  * @param[in]      wz_set底盘设置的旋转速度,正值 逆时针旋转，负值 顺时针旋转
  * @param[in]      ChassisMoveRcToVector底盘数据
  * @retval         返回空
  */

static void ChassisNoFollowYawControl(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, ChassisMove *ChassisMoveRcToVector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || ChassisMoveRcToVector == NULL)
    {
        return;
    }

    ChassisRcToControlVector(vx_set, vy_set, ChassisMoveRcToVector);
    *wz_set = -ChassisMoveRcToVector->fast.wz_rc_sen * ChassisMoveRcToVector->fast.axis_wz;
}

/**
  * @brief          when chassis behaviour mode is CHASSIS_OPEN, chassis control mode is raw control mode.
  *                 set value will be sent to can bus.
  * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
  * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
  * @param[out]     wz_set: rotation speed,positive value means counterclockwise , negative value means clockwise
  * @param[in]      ChassisMoveRcToVector: chassis data
  * @retval         none
  */
/**
  * @brief          底盘开环的行为状态机下，底盘模式是raw原生状态，故而设定值会直接发送到can总线上
  * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
  * @param[in]      vy_set左右的速度，正值 左移速度， 负值 右移速度
  * @param[in]      wz_set 旋转速度， 正值 逆时针旋转，负值 顺时针旋转
  * @param[in]      ChassisMoveRcToVector底盘数据
  * @retval         none
  */

static void ChassisOpenSetControl(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, ChassisMove *ChassisMoveRcToVector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || ChassisMoveRcToVector == NULL)
    {
        return;
    }

    *vx_set = ChassisMoveRcToVector->fast.axis_x * ChassisMoveRcToVector->fast.open_rc_scale;
    *vy_set = -ChassisMoveRcToVector->fast.axis_y * ChassisMoveRcToVector->fast.open_rc_scale;
    *wz_set = -ChassisMoveRcToVector->fast.axis_wz * ChassisMoveRcToVector->fast.open_rc_scale;
    return;
}

#endif
