/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

/*
 * 阅读地图：
 * - 前段：遥控死区、云台行为状态选择，安全档/离线/校准优先。
 * - 中段：不同云台行为输出 yaw/pitch 控制量。
 * - 后段：双云台模式复用同一套行为逻辑。
 * - 被调用方：GimbalControlTask 先决定行为，再进入角度/速度/电流控制。
 */

#include "RobotConfig.h"
#include "RobotTaskBuildConfig.h"

#if ROBOT_TASK_BUILD_ANY_GIMBAL

#include "GimbalBehaviour.h"
#include "arm_math.h"
#include "BspBuzzer.h"
#include "RobotConfig.h"
#include "ControlInput.h"
#include "MotorConfig.h"
#include "PitchCali.h"
#include "FreeRTOS.h"
#include "task.h"

#include "UserLib.h"

//when gimbal is in calibrating, set buzzer frequency and strenght
//当云台在校准, 设置蜂鸣器频率和强度
#define GimbalWarnBuzzerOn() BuzzerToneStartLegacy(g_config.buzzer.GimbalWarnPsc, g_config.buzzer.GimbalWarnPwm)
#define GimbalWarnBuzzerOff() BuzzerToneStop()

#define int_abs(x) ((x) > 0 ? (x) : (-x))
/**
  * @brief          remote control dealline solve,because the value of rocker is not zero in middle place,
  * @param          input:the raw channel value
  * @param          output: the processed channel value
  * @param          deadline
  */
/**
  * @brief          遥控器的死区判断，因为遥控器的拨杆在中位的时候，不一定为0，
  * @param          输入的遥控器值
  * @param          输出的死区处理后遥控器值
  * @param          死区值
  */
#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }


/**
  * @brief          judge if gimbal reaches the limit by gyro
  * @param          gyro: rotation speed unit rad/s
  * @param          timing time, input "GIMBAL_CALI_STEP_TIME"
  * @param          record angle, unit rad
  * @param          feedback angle, unit rad
  * @param          record ecd, unit raw
  * @param          feedback ecd, unit raw
  * @param          cali step, +1 by one step
  */
/**
  * @brief          通过判断角速度来判断云台是否到达极限位置
  * @param          对应轴的角速度，单位rad/s
  * @param          计时时间，到达GIMBAL_CALI_STEP_TIME的时间后归零
  * @param          记录的角度 rad
  * @param          反馈的角度 rad
  * @param          记录的编码值 raw
  * @param          反馈的编码值 raw
  * @param          校准的步骤 完成一次 加一
  */
#define GimbalCaliGyroJudge(gyro, cmd_time, angle_set, angle, ecd_set, ecd, step) \
    {                                                                                \
        if ((gyro) < GIMBAL_CALI_GYRO_LIMIT)                                         \
        {                                                                            \
            (cmd_time)++;                                                            \
            if ((cmd_time) > GIMBAL_CALI_STEP_TIME)                                  \
            {                                                                        \
                (cmd_time) = 0;                                                      \
                (angle_set) = (angle);                                               \
                (ecd_set) = (ecd);                                                   \
                (step)++;                                                            \
            }                                                                        \
        }                                                                            \
    }

/**
  * @brief          gimbal behave mode set.
  * @param[in]      GimbalModeSet: gimbal data
  * @retval         none
  */
/**
  * @brief          云台行为状态机设置.
  * @param[in]      GimbalModeSet: 云台数据指针
  * @retval         none
  */
static void GimbalBehavourSet(GimbalControl *GimbalModeSet);

/**
  * @brief          when gimbal behaviour mode is GIMBAL_ZERO_FORCE, the function is called
  *                 and gimbal control mode is raw. The raw mode means set value
  *                 will be sent to CAN bus derectly, and the function will set all zero.
  * @param[out]     yaw: yaw motor current set, it will be sent to CAN bus derectly.
  * @param[out]     pitch: pitch motor current set, it will be sent to CAN bus derectly.
  * @param[in]      GimbalControlSet: gimbal data
  * @retval         none
  */
/**
  * @brief          当云台行为模式是GIMBAL_ZERO_FORCE, 这个函数会被调用,云台控制模式是raw模式.原始模式意味着
  *                 设定值会直接发送到CAN总线上,这个函数将会设置所有为0.
  * @param[in]      yaw:发送yaw电机的原始值，会直接通过can 发送到电机
  * @param[in]      pitch:发送pitch电机的原始值，会直接通过can 发送到电机
  * @param[in]      GimbalControlSet: 云台数据指针
  * @retval         none
  */
static void GimbalZeroForceControl(fp32 *yaw, fp32 *pitch, GimbalControl *GimbalControlSet);

/**
  * @brief          when gimbal behaviour mode is GIMBAL_INIT, the function is called
  *                 gimbal will lift the pitch axis and rotate yaw axis.
  * @param[out]     yaw: yaw motor angle increment, unit rad.
  * @param[out]     pitch: pitch motor angle increment, unit rad.
  * @param[in]      GimbalControlSet: gimbal data
  * @retval         none
  */
/**
  * @brief          云台初始化控制，先抬起 pitch 轴，再旋转 yaw 轴
  * @param[out]     yaw轴角度控制，为角度的增量 单位 rad
  * @param[out]     pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      云台数据指针
  * @retval         返回空
  */
static void GimbalInitControl(fp32 *yaw, fp32 *pitch, GimbalControl *GimbalControlSet);

/**
  * @brief          when gimbal behaviour mode is GIMBAL_CALI, the function is called
  *                 and gimbal control mode is raw mode. gimbal will lift the pitch axis,
  *                 and then put down the pitch axis, and rotate yaw axis counterclockwise,
  *                 and rotate yaw axis clockwise.
  * @param[out]     yaw: yaw motor current set, will be sent to CAN bus decretly
  * @param[out]     pitch: pitch motor current set, will be sent to CAN bus decretly
  * @param[in]      GimbalControlSet: gimbal data
  * @retval         none
  */
/**
  * @brief          云台校准控制，电机是raw控制，云台先抬起pitch，放下pitch，在正转yaw，最后反转yaw，记录当时的角度和编码值
  * @author         RM
  * @param[out]     yaw:发送yaw电机的原始值，会直接通过can 发送到电机
  * @param[out]     pitch:发送pitch电机的原始值，会直接通过can 发送到电机
  * @param[in]      GimbalControlSet:云台数据指针
  * @retval         none
  */
static void GimbalCaliControl(fp32 *yaw, fp32 *pitch, GimbalControl *GimbalControlSet);

/**
  * @brief          when gimbal behaviour mode is GIMBAL_ANGLE, the function is called
  *                 and gimbal control mode is encoder angle mode.
  * @param[out]     yaw: yaw axia angle increment, unit rad
  * @param[out]     pitch: pitch axia angle increment,unit rad
  * @param[in]      GimbalControlSet: gimbal data
  * @retval         none
  */
/**
  * @brief          云台编码值控制，电机角度控制，
  * @param[in]      yaw: yaw轴角度控制，为角度的增量 单位 rad
  * @param[in]      pitch: pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      GimbalControlSet: 云台数据指针
  * @retval         none
  */
static void GimbalAngleControl(fp32 *yaw, fp32 *pitch, GimbalControl *GimbalControlSet);

/**
  * @brief          when gimbal behaviour mode is GIMBAL_MOTIONLESS, the function is called
  *                 and gimbal control mode is encode mode.
  * @param[out]     yaw: yaw axia angle increment,  unit rad
  * @param[out]     pitch: pitch axia angle increment, unit rad
  * @param[in]      GimbalControlSet: gimbal data
  * @retval         none
  */
/**
  * @brief          云台进入遥控器无输入控制，电机角度控制，
  * @author         RM
  * @param[in]      yaw: yaw轴角度控制，为角度的增量 单位 rad
  * @param[in]      pitch: pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      GimbalControlSet:云台数据指针
  * @retval         none
  */
static void GimbalMotionlessControl(fp32 *yaw, fp32 *pitch, GimbalControl *GimbalControlSet);

//云台行为状态机
static GimbalBehaviour s_gimbal_behaviour = GIMBAL_ZERO_FORCE;
// 调试用可见镜像，便于 watch 窗口查看当前行为
volatile GimbalBehaviour GimbalBehaviourWatch = GIMBAL_ZERO_FORCE;

typedef struct
{
    volatile uint8_t active;
    volatile uint8_t flipped;
    uint8_t pending_flipped;
    uint8_t setpoint_done;
    uint8_t key_prev;
    fp32 target_yaw_relative;
    fp32 remaining_setpoint_rad;
    uint32_t start_ms;
} GimbalTurnaroundState;

static GimbalTurnaroundState s_turn = {0};
static const uint32_t GIMBAL_TURNAROUND_TIMEOUT_MS = 3000u;

static fp32 GimbalGetYawRelativeAngle(const GimbalMotor *yaw_motor)
{
    if (yaw_motor == NULL || yaw_motor->GimbalMotorMeasure == NULL)
    {
        return 0.0f;
    }

    const uint32_t ecd_range = MotorCfgEncoderRange(g_config.motor.yaw.model);
    const int32_t half_ecd_range = (int32_t)(ecd_range / 2u);
    const int32_t full_ecd_range = (int32_t)ecd_range;
    int32_t relative_ecd = (int32_t)yaw_motor->GimbalMotorMeasure->ecd - (int32_t)yaw_motor->offset_ecd;
    if (relative_ecd > half_ecd_range)
    {
        relative_ecd -= full_ecd_range;
    }
    else if (relative_ecd < -half_ecd_range)
    {
        relative_ecd += full_ecd_range;
    }

    return (YAW_TURN ? -1.0f : 1.0f) * ((fp32)relative_ecd * (6.28318530718f / (fp32)ecd_range));
}

bool_t GimbalTurnaroundIsActive(void)
{
    return (s_turn.active != 0u) ? 1 : 0;
}

fp32 GimbalTurnaroundChassisFollowOffsetRad(void)
{
    return (s_turn.flipped != 0u) ? PI : 0.0f;
}

bool_t GimbalTurnaroundGetFrameYawRelative(fp32 *out_yaw_relative)
{
    if (out_yaw_relative == NULL)
    {
        return 0;
    }

    if (s_turn.active == 0u)
    {
        return 0;
    }

    *out_yaw_relative = s_turn.target_yaw_relative;
    return 1;
}

/**
  * @brief          the function is called by GimbalSetMode function in GimbalControlTask.c
  *                 the function set GimbalBehaviour variable, and set motor mode.
  * @param[in]      GimbalModeSet: gimbal data
  * @retval         none
  */
/**
  * @brief          被GimbalSetMode函数调用在GimbalControlTask.c,云台行为状态机以及电机状态机设置
  * @param[out]     GimbalModeSet: 云台数据指针
  * @retval         none
  */

void GimbalBehaviourModeSet(GimbalControl *GimbalModeSet)
{
    if (GimbalModeSet == NULL)
    {
        return;
    }
    //set GimbalBehaviour variable
    //云台行为状态机设置
    GimbalBehavourSet(GimbalModeSet);

    //accoring to GimbalBehaviour, set motor control mode
    //根据云台行为状态机设置电机状态机
    if (s_gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        GimbalModeSet->GimbalYawMotor.mode = GIMBAL_MOTOR_RAW;
        GimbalModeSet->GimbalPitchMotor.mode = GIMBAL_MOTOR_RAW;
    }
    else if (s_gimbal_behaviour == GIMBAL_INIT)
    {
        GimbalModeSet->GimbalYawMotor.mode = GIMBAL_MOTOR_ENCODER;
        GimbalModeSet->GimbalPitchMotor.mode = GIMBAL_MOTOR_ENCODER;
    }
    else if (s_gimbal_behaviour == GIMBAL_CALI)
    {
        GimbalModeSet->GimbalYawMotor.mode = GIMBAL_MOTOR_RAW;
        GimbalModeSet->GimbalPitchMotor.mode = GIMBAL_MOTOR_RAW;
    }
    else if (s_gimbal_behaviour == GIMBAL_ANGLE)
    {
        GimbalModeSet->GimbalYawMotor.mode = GIMBAL_MOTOR_ENCODER;
        GimbalModeSet->GimbalPitchMotor.mode = GIMBAL_MOTOR_ENCODER;
    }
    else if (s_gimbal_behaviour == GIMBAL_MOTIONLESS)
    {
        GimbalModeSet->GimbalYawMotor.mode = GIMBAL_MOTOR_ENCODER;
        GimbalModeSet->GimbalPitchMotor.mode = GIMBAL_MOTOR_ENCODER;
    }
    else if (s_gimbal_behaviour == GIMBAL_PITCH_CALI)
    {
        // NOTE: pitch motor may be temporarily switched to RAW by the calibration state machine.
        GimbalModeSet->GimbalYawMotor.mode = GIMBAL_MOTOR_ENCODER;
        GimbalModeSet->GimbalPitchMotor.mode = GIMBAL_MOTOR_ENCODER;
    }

    // 更新调试镜像，便于在 watch 中查看行为状态
    GimbalBehaviourWatch = s_gimbal_behaviour;
}

/**
  * @brief          the function is called by GimbalSetControl function in GimbalControlTask.c
  *                 accoring to the GimbalBehaviour variable, call the corresponding function
  * @param[out]     add_yaw:yaw axis increment angle, unit rad
  * @param[out]     add_pitch:pitch axis increment angle,unit rad
  * @param[in]      GimbalModeSet: gimbal data
  * @retval         none
  */
/**
  * @brief          云台行为控制，根据不同行为采用不同控制函数
  * @param[out]     add_yaw:设置的yaw角度增加值，单位 rad
  * @param[out]     add_pitch:设置的pitch角度增加值，单位 rad
  * @param[in]      GimbalModeSet:云台数据指针
  * @retval         none
  */
void GimbalBehaviourControlSet(fp32 *add_yaw, fp32 *add_pitch, GimbalControl *GimbalControlSet)
{

    if (add_yaw == NULL || add_pitch == NULL || GimbalControlSet == NULL)
    {
        return;
    }


    if (s_gimbal_behaviour != GIMBAL_ANGLE)
    {
        s_turn.active = 0u;
        s_turn.setpoint_done = 0u;
        s_turn.remaining_setpoint_rad = 0.0f;
        s_turn.key_prev = 0u;
    }

    if (s_gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        GimbalZeroForceControl(add_yaw, add_pitch, GimbalControlSet);
    }
    else if (s_gimbal_behaviour == GIMBAL_INIT)
    {
        GimbalInitControl(add_yaw, add_pitch, GimbalControlSet);
    }
    else if (s_gimbal_behaviour == GIMBAL_CALI)
    {
        GimbalCaliControl(add_yaw, add_pitch, GimbalControlSet);
    }
    else if (s_gimbal_behaviour == GIMBAL_ANGLE)
    {
        GimbalAngleControl(add_yaw, add_pitch, GimbalControlSet);
    }
    else if (s_gimbal_behaviour == GIMBAL_MOTIONLESS)
    {
        GimbalMotionlessControl(add_yaw, add_pitch, GimbalControlSet);
    }
    else if (s_gimbal_behaviour == GIMBAL_PITCH_CALI)
    {
        PitchCaliControl(add_yaw, add_pitch, GimbalControlSet);
    }

}

/**
  * @brief          in some gimbal mode, need chassis keep no move
  * @param[in]      none
  * @retval         1: no move 0:normal
  */
/**
  * @brief          云台在某些行为下，需要底盘不动
  * @param[in]      none
  * @retval         1: no move 0:normal
  */

bool_t GimbalCmdToChassisStop(void)
{
    // Only stop chassis in safety/initialization/calibration.
    // NOTE: Do NOT stop chassis in GIMBAL_MOTIONLESS; otherwise chassis follow can't converge after you release yaw input.
    if (s_gimbal_behaviour == GIMBAL_INIT || s_gimbal_behaviour == GIMBAL_CALI || s_gimbal_behaviour == GIMBAL_PITCH_CALI || s_gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/**
  * @brief          in some gimbal mode, need shoot keep no move
  * @param[in]      none
  * @retval         1: no move 0:normal
  */
/**
  * @brief          云台在某些行为下，需要射击停止
  * @param[in]      none
  * @retval         1: no move 0:normal
  */

bool_t GimbalCmdToShootStop(void)
{
    if (s_gimbal_behaviour == GIMBAL_INIT || s_gimbal_behaviour == GIMBAL_CALI || s_gimbal_behaviour == GIMBAL_PITCH_CALI || s_gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}


/**
  * @brief          gimbal behave mode set.
  * @param[in]      GimbalModeSet: gimbal data
  * @retval         none
  */
/**
  * @brief          云台行为状态机设置.
  * @param[in]      GimbalModeSet: 云台数据指针
  * @retval         none
  */
static void GimbalBehavourSet(GimbalControl *GimbalModeSet)
{
    if (GimbalModeSet == NULL)
    {
        return;
    }

    // 函数地图：安全/运行模式优先；再处理校准和初始化；最后把拨杆状态写成行为模式。
    const GimbalControlSnapshot *fast = &GimbalModeSet->fast;
    const bool_t manual_offline = fast->manual_offline;
    const uint8_t GimbalSw = fast->mode_sw;
    const bool_t switch_safe = ControlInputSwitchIsPos(GimbalSw, fast->safe_pos);
    const bool_t yaw_only_mode = fast->run_variant == ROBOT_RUN_VARIANT_GIMBAL_YAW_ONLY;
    const bool_t yaw_easy_test_mode = fast->run_variant == ROBOT_RUN_VARIANT_GIMBAL_YAW_EASY;
    const bool_t PitchCaliMode = fast->PitchCaliMode != 0u;

    // 安全模式最高优先级：遥控上档或离线直接降为零力矩
    if (manual_offline || switch_safe)
    {
        s_gimbal_behaviour = GIMBAL_ZERO_FORCE;
        return;
    }

    // yaw-only / yaw-easy 运行变体：直接启用 yaw 角度控制，跳过 pitch 零点/校准检查
    if (yaw_only_mode || yaw_easy_test_mode)
    {
        s_gimbal_behaviour = GIMBAL_ANGLE;
        // 若未校准或数据异常，给 yaw 角度限幅一个安全默认值，避免 max/min 为脏数据导致 set 被卡死
        if (GimbalModeSet->GimbalYawMotor.max_angle <= GimbalModeSet->GimbalYawMotor.min_angle ||
            fabsf(GimbalModeSet->GimbalYawMotor.max_angle) > 1000.0f ||
            fabsf(GimbalModeSet->GimbalYawMotor.min_angle) > 1000.0f)
        {
            GimbalModeSet->GimbalYawMotor.max_angle = PI;
            GimbalModeSet->GimbalYawMotor.min_angle = -PI;
        }
        return;
    }
    //in cali mode, return
    //校准行为，return 不会设置其他的模式
    if (s_gimbal_behaviour == GIMBAL_CALI && GimbalModeSet->GimbalCali.step != GIMBAL_CALI_END_STEP)
    {
        return;
    }

    //if other operate make step change to start, means enter cali mode
    //如果外部使得校准步骤从0 变成 start，则进入校准模式
    if (GimbalModeSet->GimbalCali.step == GIMBAL_CALI_START_STEP && !manual_offline)
    {
        s_gimbal_behaviour = GIMBAL_CALI;
        return;
    }

    //init mode, judge if gimbal is in middle place
    //初始化模式判断是否到达中值位置
    if (s_gimbal_behaviour == GIMBAL_INIT)
    {
        static uint16_t init_time = 0;
        static uint16_t init_stop_time = 0;
        init_time++;

        if ((fabs(GimbalModeSet->GimbalYawMotor.angle - INIT_YAW_SET) < GIMBAL_INIT_ANGLE_ERROR &&
             fabs(GimbalModeSet->GimbalPitchMotor.angle - INIT_PITCH_SET) < GIMBAL_INIT_ANGLE_ERROR))
        {

            if (init_stop_time < GIMBAL_INIT_STOP_TIME)
            {
                init_stop_time++;
            }
        }
        else
        {

            if (init_time < GIMBAL_INIT_TIME)
            {
                init_time++;
            }
        }

        //超过初始化最大时间，或者已经稳定到中值一段时间，退出初始化状态：开关打上档，或者掉线
        if (init_time < GIMBAL_INIT_TIME && init_stop_time < GIMBAL_INIT_STOP_TIME &&
            !switch_safe && !manual_offline)
        {
            return;
        }
        else
        {
            init_stop_time = 0;
            init_time = 0;
        }
    }

    //开关控制 云台状态
    if (switch_safe)
    {
        s_gimbal_behaviour = GIMBAL_ZERO_FORCE;
    }
    else
    {
        s_gimbal_behaviour = GIMBAL_ANGLE;
    }

    //enter init mode
    //判断进入init状态机
    {
        static GimbalBehaviour last_gimbal_behaviour = GIMBAL_ZERO_FORCE;
        if (last_gimbal_behaviour == GIMBAL_ZERO_FORCE && s_gimbal_behaviour != GIMBAL_ZERO_FORCE)
        {
            s_gimbal_behaviour = GIMBAL_INIT;
        }
        last_gimbal_behaviour = s_gimbal_behaviour;
    }

    // pitch 补偿校准模式：在非安全档运行；仍保留 INIT 状态机（上电/退出安全档先回中位）
    if (PitchCaliMode)
    {
        if (s_gimbal_behaviour == GIMBAL_ANGLE || s_gimbal_behaviour == GIMBAL_MOTIONLESS)
        {
            s_gimbal_behaviour = GIMBAL_PITCH_CALI;
        }
    }



}

/**
  * @brief          when gimbal behaviour mode is GIMBAL_ZERO_FORCE, the function is called
  *                 and gimbal control mode is raw. The raw mode means set value
  *                 will be sent to CAN bus derectly, and the function will set all zero.
  * @param[out]     yaw: yaw motor current set, it will be sent to CAN bus derectly.
  * @param[out]     pitch: pitch motor current set, it will be sent to CAN bus derectly.
  * @param[in]      GimbalControlSet: gimbal data
  * @retval         none
  */
/**
  * @brief          当云台行为模式是GIMBAL_ZERO_FORCE, 这个函数会被调用,云台控制模式是raw模式.原始模式意味着
  *                 设定值会直接发送到CAN总线上,这个函数将会设置所有为0.
  * @param[in]      yaw:发送yaw电机的原始值，会直接通过can 发送到电机
  * @param[in]      pitch:发送pitch电机的原始值，会直接通过can 发送到电机
  * @param[in]      GimbalControlSet: 云台数据指针
  * @retval         none
  */
static void GimbalZeroForceControl(fp32 *yaw, fp32 *pitch, GimbalControl *GimbalControlSet)
{
    if (yaw == NULL || pitch == NULL || GimbalControlSet == NULL)
    {
        return;
    }

    *yaw = 0.0f;
    *pitch = 0.0f;
}
/**
  * @brief          when gimbal behaviour mode is GIMBAL_INIT, the function is called.
  *                 gimbal will lift the pitch axis and rotate yaw axis.
  * @param[out]     yaw: yaw motor angle increment, unit rad.
  * @param[out]     pitch: pitch motor angle increment, unit rad.
  * @param[in]      GimbalControlSet: gimbal data
  * @retval         none
  */
/**
  * @brief          云台初始化控制，电机是陀螺仪角度控制，云台先抬起pitch轴，后旋转yaw轴
  * @author         RM
  * @param[out]     yaw轴角度控制，为角度的增量 单位 rad
  * @param[out]     pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      云台数据指针
  * @retval         返回空
  */
static void GimbalInitControl(fp32 *yaw, fp32 *pitch, GimbalControl *GimbalControlSet)
{
    if (yaw == NULL || pitch == NULL || GimbalControlSet == NULL)
    {
        return;
    }

    //初始化状态控制量计算
    if (fabs(INIT_PITCH_SET - GimbalControlSet->GimbalPitchMotor.angle) > GIMBAL_INIT_ANGLE_ERROR)
    {
        *pitch = (INIT_PITCH_SET - GimbalControlSet->GimbalPitchMotor.angle) * GIMBAL_INIT_PITCH_SPEED;
        *yaw = 0.0f;
    }
    else
    {
        *pitch = (INIT_PITCH_SET - GimbalControlSet->GimbalPitchMotor.angle) * GIMBAL_INIT_PITCH_SPEED;
        *yaw = (INIT_YAW_SET - GimbalControlSet->GimbalYawMotor.angle) * GIMBAL_INIT_YAW_SPEED;
    }
}

/**
  * @brief          when gimbal behaviour mode is GIMBAL_CALI, the function is called
  *                 and gimbal control mode is raw mode. gimbal will lift the pitch axis,
  *                 and then put down the pitch axis, and rotate yaw axis counterclockwise,
  *                 and rotate yaw axis clockwise.
  * @param[out]     yaw: yaw motor current set, will be sent to CAN bus decretly
  * @param[out]     pitch: pitch motor current set, will be sent to CAN bus decretly
  * @param[in]      GimbalControlSet: gimbal data
  * @retval         none
  */
/**
  * @brief          云台校准控制，电机是raw控制，云台先抬起pitch，放下pitch，在正转yaw，最后反转yaw，记录当时的角度和编码值
  * @author         RM
  * @param[out]     yaw:发送yaw电机的原始值，会直接通过can 发送到电机
  * @param[out]     pitch:发送pitch电机的原始值，会直接通过can 发送到电机
  * @param[in]      GimbalControlSet:云台数据指针
  * @retval         none
  */
static void GimbalCaliControl(fp32 *yaw, fp32 *pitch, GimbalControl *GimbalControlSet)
{
    if (yaw == NULL || pitch == NULL || GimbalControlSet == NULL)
    {
        return;
    }
    static uint16_t cali_time = 0;

    if (GimbalControlSet->GimbalCali.step == GIMBAL_CALI_PITCH_MAX_STEP)
    {

        *pitch = GIMBAL_CALI_MOTOR_SET;
        *yaw = 0;

        //判断陀螺仪数据， 并记录最大最小角度数据
        GimbalCaliGyroJudge(GimbalControlSet->GimbalPitchMotor.motor_gyro, cali_time, GimbalControlSet->GimbalCali.max_pitch,
                               GimbalControlSet->GimbalPitchMotor.angle, GimbalControlSet->GimbalCali.max_pitch_ecd,
                               GimbalControlSet->GimbalPitchMotor.GimbalMotorMeasure->ecd, GimbalControlSet->GimbalCali.step);
    }
    else if (GimbalControlSet->GimbalCali.step == GIMBAL_CALI_PITCH_MIN_STEP)
    {
        *pitch = -GIMBAL_CALI_MOTOR_SET;
        *yaw = 0;

        GimbalCaliGyroJudge(GimbalControlSet->GimbalPitchMotor.motor_gyro, cali_time, GimbalControlSet->GimbalCali.min_pitch,
                               GimbalControlSet->GimbalPitchMotor.angle, GimbalControlSet->GimbalCali.min_pitch_ecd,
                               GimbalControlSet->GimbalPitchMotor.GimbalMotorMeasure->ecd, GimbalControlSet->GimbalCali.step);
    }
    else if (GimbalControlSet->GimbalCali.step == GIMBAL_CALI_YAW_MAX_STEP)
    {
        *pitch = 0;
        *yaw = GIMBAL_CALI_MOTOR_SET;

        GimbalCaliGyroJudge(GimbalControlSet->GimbalYawMotor.motor_gyro, cali_time, GimbalControlSet->GimbalCali.max_yaw,
                               GimbalControlSet->GimbalYawMotor.angle, GimbalControlSet->GimbalCali.max_yaw_ecd,
                               GimbalControlSet->GimbalYawMotor.GimbalMotorMeasure->ecd, GimbalControlSet->GimbalCali.step);
    }

    else if (GimbalControlSet->GimbalCali.step == GIMBAL_CALI_YAW_MIN_STEP)
    {
        *pitch = 0;
        *yaw = -GIMBAL_CALI_MOTOR_SET;

        GimbalCaliGyroJudge(GimbalControlSet->GimbalYawMotor.motor_gyro, cali_time, GimbalControlSet->GimbalCali.min_yaw,
                               GimbalControlSet->GimbalYawMotor.angle, GimbalControlSet->GimbalCali.min_yaw_ecd,
                               GimbalControlSet->GimbalYawMotor.GimbalMotorMeasure->ecd, GimbalControlSet->GimbalCali.step);
    }
    else if (GimbalControlSet->GimbalCali.step == GIMBAL_CALI_END_STEP)
    {
        cali_time = 0;
    }
}


/**
  * @brief          when gimbal behaviour mode is GIMBAL_ANGLE, the function is called
  *                 and gimbal control mode is encoder angle mode.
  * @param[out]     yaw: yaw axia angle increment, unit rad
  * @param[out]     pitch: pitch axia angle increment,unit rad
  * @param[in]      GimbalControlSet: gimbal data
  * @retval         none
  */
/**
  * @brief          云台编码值控制，电机角度控制，
  * @param[in]      yaw: yaw轴角度控制，为角度的增量 单位 rad
  * @param[in]      pitch: pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      GimbalControlSet: 云台数据指针
  * @retval         none
  */
static void GimbalAngleControl(fp32 *yaw, fp32 *pitch, GimbalControl *GimbalControlSet)
{
    if (yaw == NULL || pitch == NULL || GimbalControlSet == NULL)
    {
        return;
    }
    static int16_t yaw_channel = 0, pitch_channel = 0;
    const GimbalControlSnapshot *fast = &GimbalControlSet->fast;
    const robot_run_variant_e variant = fast->run_variant;
    const bool_t yaw_test_mode = (variant == ROBOT_RUN_VARIANT_GIMBAL_YAW_ONLY) ||
                                 (variant == ROBOT_RUN_VARIANT_GIMBAL_YAW_EASY);

    rc_deadband_limit(fast->yaw_axis, yaw_channel, fast->rc_deadband);
    rc_deadband_limit(fast->pitch_axis, pitch_channel, fast->rc_deadband);

    const uint16_t key_mask = fast->key_mask;
    const uint8_t turn_key_down = ((key_mask & fast->turn_key_mask) != 0u) ? 1u : 0u;
    const uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

    if (turn_key_down != 0u && s_turn.key_prev == 0u && s_turn.active == 0u)
    {
        s_turn.active = 1u;
        s_turn.setpoint_done = 0u;
        s_turn.pending_flipped = (s_turn.flipped != 0u) ? 0u : 1u;
        s_turn.remaining_setpoint_rad = PI;
        s_turn.target_yaw_relative = rad_format(GimbalGetYawRelativeAngle(&GimbalControlSet->GimbalYawMotor) + PI);
        s_turn.start_ms = now_ms;
    }
    s_turn.key_prev = turn_key_down;

    // yaw 单轴运行变体：放宽限幅防止 ±pi 卡死
    if (yaw_test_mode)
    {
        GimbalControlSet->GimbalYawMotor.max_angle = 100.0f;
        GimbalControlSet->GimbalYawMotor.min_angle = -100.0f;
    }

    fp32 yaw_add = -yaw_channel * fast->yaw_rc_sen - (fp32)fast->mouse_x * fast->yaw_mouse_sen;
    *pitch = pitch_channel * fast->pitch_rc_sen + (fp32)fast->mouse_y * fast->pitch_mouse_sen;

    if (s_turn.active != 0u)
    {
        yaw_add = 0.0f;

        if (s_turn.setpoint_done == 0u)
        {
            fp32 step = fabsf(fast->turn_speed);
            if (step < 0.000001f)
            {
                step = 0.04f;
            }
            if (step > s_turn.remaining_setpoint_rad)
            {
                step = s_turn.remaining_setpoint_rad;
            }

            yaw_add = (s_turn.pending_flipped != 0u) ? step : -step;
            s_turn.remaining_setpoint_rad -= step;
            if (s_turn.remaining_setpoint_rad <= 0.000001f)
            {
                s_turn.remaining_setpoint_rad = 0.0f;
                s_turn.setpoint_done = 1u;
            }
        }
        else
        {
            const fp32 yaw_rel = GimbalGetYawRelativeAngle(&GimbalControlSet->GimbalYawMotor);
            const fp32 abs_err = fabsf(rad_format(yaw_rel - s_turn.target_yaw_relative));
            const fp32 abs_gyro = fabsf(GimbalControlSet->GimbalYawMotor.motor_gyro);
            const uint32_t elapsed_ms = now_ms - s_turn.start_ms;
            if (abs_err < 0.15f && abs_gyro < 0.3f)
            {
                s_turn.flipped = s_turn.pending_flipped;
                s_turn.active = 0u;
            }
            else if (elapsed_ms >= GIMBAL_TURNAROUND_TIMEOUT_MS)
            {
                s_turn.active = 0u;
            }
        }
    }

    *yaw = yaw_add;

}

/**
  * @brief          when gimbal behaviour mode is GIMBAL_MOTIONLESS, the function is called
  *                 and gimbal control mode is encode mode.
  * @param[out]     yaw: yaw axia angle increment,  unit rad
  * @param[out]     pitch: pitch axia angle increment, unit rad
  * @param[in]      GimbalControlSet: gimbal data
  * @retval         none
  */
/**
  * @brief          云台进入遥控器无输入控制，电机角度控制，
  * @author         RM
  * @param[in]      yaw: yaw轴角度控制，为角度的增量 单位 rad
  * @param[in]      pitch: pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      GimbalControlSet:云台数据指针
  * @retval         none
  */
static void GimbalMotionlessControl(fp32 *yaw, fp32 *pitch, GimbalControl *GimbalControlSet)
{
    if (yaw == NULL || pitch == NULL || GimbalControlSet == NULL)
    {
        return;
    }
    *yaw = 0.0f;
    *pitch = 0.0f;
}

#endif
