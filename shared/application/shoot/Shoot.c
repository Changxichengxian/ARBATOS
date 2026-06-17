/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

/*
 * 阅读地图：
 * - 前段：拨杆/图传输入解释、娱乐模式蜂鸣器音乐、摩擦轮/拨弹输出清零。
 * - 中段：ShootControlLoop() 串起状态机、反馈更新、PID 电流输出。
 * - 后段：ShootSetMode() 决定射击状态，feedback_update() 维护编码器圈数和堵转信息。
 * - 输出：拨弹电流作为返回值，摩擦轮电流写入 LowCmd。
 */

#include "config.h"
#include "RobotTaskBuildConfig.h"

#if ROBOT_TASK_BUILD_SHOOT_RM

#include "Shoot.h"

#include <math.h>
#include "cmsis_os.h"

#include "BspShootTrig.h"
#include "user_lib.h"
#include "Referee.h"

#include "CanReceive.h"
#include "LowCmd.h"
#include "MotorInst.h"
#include "MotorConfig.h"
#include "GimbalState.h"
#include "ShootState.h"
#include "ControlInput.h"
#include "DetectTask.h"
#include "RobotMode.h"
#include "Pid.h"
#include "HostLinkTask.h"

// 微动开关 GPIO 是板相关差异，通过 BSP 读取。

/**
  * @brief          射击状态机设置，遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗，一直处在下，则持续发射，用于3min准备时间清理子弹
  * @param[in]      void
  * @retval         void
  */
static void ShootSetMode(void);
/**
  * @brief          update shoot feedback data.
  * @param[in]      void
  * @retval         void
  */
static void ShootFeedbackUpdate(void);

/**
  * @brief          清空摩擦轮输出（速度环 PID / 目标转速 / 电流指令）
  * @param[in]      void
  * @retval         void
  */
static void ShootClearFricOutput(void);

/**
  * @brief          摩擦轮到速判定（基于电机反馈转速）
  * @param[in]      void
  * @retval         1: ready 0: not ready
  */
static bool_t ShootFricSpeedReady(void);
static bool_t ShootGimbalCmdToShootStop(void);
static void ShootWriteState(void);

static const char *const ShootFrictionMotorNames[FRIC_MOTOR_NUM] = {
    "motor.friction0",
    "motor.friction1",
    "motor.friction2",
    "motor.friction3",
};
static MotorCurrentBind ShootFrictionCurrentBindings[FRIC_MOTOR_NUM];
static const int16_t ShootFricZeroCurrentCmd[FRIC_MOTOR_NUM] = {0};

/**
  * @brief          trigger jam reverse handling.
  * @param[in]      void
  * @retval         void
  */
static void trigger_motor_turn_back(void);

/**
  * @brief          射击控制，控制拨弹电机角度，完成一次发射
  * @param[in]      void
  * @retval         void
  */
static void ShootBulletControl(void);

static uint16_t ShootTickMs(void)
{
    return (SHOOT_CONTROL_TIME > 0u) ? SHOOT_CONTROL_TIME : 1u;
}

static uint16_t ShootU16AddSat(uint16_t value, uint16_t add_ms, uint16_t max_value)
{
    const uint32_t next = (uint32_t)value + (uint32_t)add_ms;
    return (next >= (uint32_t)max_value) ? max_value : (uint16_t)next;
}

static uint16_t ShootSwitchRawFromPos(uint8_t pos)
{
    return (uint16_t)ControlInputSwitchPosToRaw(pos);
}

static uint8_t ShootSwitchIsStop(uint16_t raw_sw)
{
    return ControlInputSwitchIsPos(raw_sw, g_config.manual_input.semantics.ShootStopPos);
}

static uint8_t ShootSwitchIsReady(uint16_t raw_sw)
{
    return ControlInputSwitchIsPos(raw_sw, g_config.manual_input.semantics.ShootReadyPos);
}

static uint8_t ShootSwitchIsFire(uint16_t raw_sw)
{
    return ControlInputSwitchIsPos(raw_sw, g_config.manual_input.semantics.ShootFirePos);
}

static input_switch_e ShootGetImageSwitchInput(void)
{
    switch (g_config.manual_input.semantics.image_vt13_shoot_switch_input)
    {
    case MANUAL_INPUT_IMAGE_SWITCH_CHASSIS:
        return INPUT_SW_CHASSIS_MODE;
    case MANUAL_INPUT_IMAGE_SWITCH_GIMBAL:
        return INPUT_SW_GIMBAL_MODE;
    case MANUAL_INPUT_IMAGE_SWITCH_SHOOT:
    default:
        return INPUT_SW_SHOOT_MODE;
    }
}

static uint16_t ShootGetRawSwitch(void)
{
    uint16_t raw_sw = (uint16_t)input_switch(INPUT_SW_SHOOT_MODE);

    if (remote_control_get_active_source() != MANUAL_INPUT_SRC_IMAGE)
    {
        return raw_sw;
    }

    ImageRemoteState image_state;
    if (!ImageRemoteGetState(&image_state) ||
        image_state.proto != SDLOG_MANUAL_INPUT_PROTO_IMAGE_VT13)
    {
        return raw_sw;
    }

    raw_sw = (uint16_t)ControlInputSwitch(ShootGetImageSwitchInput());
    return ShootSwitchIsStop(raw_sw) ? ShootSwitchRawFromPos(g_config.manual_input.semantics.ShootStopPos) :
                                          ShootSwitchRawFromPos(g_config.manual_input.semantics.ShootFirePos);
}

static uint16_t ShootGetEffectiveSwitch(void)
{
    static uint8_t gate_inited = 0u;
    static uint8_t down_engaged = 0u;
    static uint16_t last_sw_raw = RC_SW_UP;

    const uint16_t raw_sw = ShootGetRawSwitch();
    const uint8_t manual_online = toe_is_error(DBUS_TOE) ? 0u : 1u;
    const uint16_t ShootStopRaw = ShootSwitchRawFromPos(g_config.manual_input.semantics.ShootStopPos);
    const uint16_t ShootReadyRaw = ShootSwitchRawFromPos(g_config.manual_input.semantics.ShootReadyPos);
    uint16_t effective_sw = raw_sw;

    if (manual_online == 0u)
    {
        gate_inited = 0u;
        down_engaged = 0u;
        last_sw_raw = ShootStopRaw;
        return raw_sw;
    }

    if (gate_inited == 0u)
    {
        gate_inited = 1u;
        down_engaged = 0u;
        last_sw_raw = raw_sw;
    }

    if (!ShootSwitchIsFire(raw_sw))
    {
        down_engaged = 0u;
    }
    else if (!ShootSwitchIsFire(last_sw_raw))
    {
        down_engaged = 1u;
    }

    if (ShootSwitchIsFire(raw_sw) && down_engaged == 0u)
    {
        effective_sw = ShootReadyRaw;
    }

    last_sw_raw = raw_sw;
    return effective_sw;
}



static ShootControl g_shoot;          // shoot control data

const ShootControl *get_shoot_control_point(void)
{
    return &g_shoot;
}

void ShootTuneApplyFricSpeedPid(void)
{
    taskENTER_CRITICAL();
    for (uint8_t i = 0; i < FRIC_MOTOR_NUM; i++)
    {
        PidTypeDef *dst = &g_shoot.fric_speed_pid[i];
        dst->Kp = g_config.shoot.fric_speed_pid.kp;
        dst->Ki = g_config.shoot.fric_speed_pid.ki;
        dst->Kd = g_config.shoot.fric_speed_pid.kd;
        dst->max_out = g_config.shoot.fric_speed_pid.max_out;
        dst->max_iout = g_config.shoot.fric_speed_pid.max_iout;
        PID_clear(dst);
    }
    taskEXIT_CRITICAL();
}

void ShootTuneApplyTriggerPid(void)
{
    taskENTER_CRITICAL();
    g_shoot.trigger_motor_pid.Kp = g_config.shoot.trigger_angle_pid.kp;
    g_shoot.trigger_motor_pid.Ki = g_config.shoot.trigger_angle_pid.ki;
    g_shoot.trigger_motor_pid.Kd = g_config.shoot.trigger_angle_pid.kd;
    PID_clear(&g_shoot.trigger_motor_pid);
    taskEXIT_CRITICAL();
}

static bool_t ShootGimbalCmdToShootStop(void)
{
    GimbalState state;
    return (GimbalStateRead(&state) != 0u && state.valid != 0u && state.fire_allowed == 0u) ? 1 : 0;
}

static void ShootWriteState(void)
{
    ShootState state = {0};

    state.valid = 1u;
    state.mode = (uint8_t)g_shoot.mode;
    state.fric_speed_set = g_shoot.fric_speed_set;
    state.trigger_speed_set = g_shoot.trigger_speed_set;
    state.speed = g_shoot.speed;
    state.speed_set = g_shoot.speed_set;
    state.angle = g_shoot.angle;
    state.set_angle = g_shoot.set_angle;
    state.given_current = g_shoot.given_current;
    state.ecd_count = g_shoot.ecd_count;
    state.trigger_measure_ready = g_shoot.trigger_measure_ready;
    state.press_l = (uint8_t)g_shoot.press_l;
    state.press_r = (uint8_t)g_shoot.press_r;
    state.last_press_l = (uint8_t)g_shoot.last_press_l;
    state.last_press_r = (uint8_t)g_shoot.last_press_r;
    state.press_l_time = g_shoot.press_l_time;
    state.press_r_time = g_shoot.press_r_time;
    state.rc_s_time = g_shoot.rc_s_time;
    state.block_time = g_shoot.block_time;
    state.reverse_time = g_shoot.reverse_time;
    state.move_flag = (uint8_t)g_shoot.move_flag;
    state.key = (uint8_t)g_shoot.key;
    state.key_time = g_shoot.key_time;
    state.heat_limit = g_shoot.heat_limit;
    state.heat = g_shoot.heat;
    state.trigger_motor_pid = g_shoot.trigger_motor_pid;

    for (uint8_t i = 0u; i < FRIC_MOTOR_NUM && i < SHOOT_STATE_FRIC_MOTOR_COUNT; i++)
    {
        state.fric_speed_pid[i] = g_shoot.fric_speed_pid[i];
        state.fric_current_set[i] = g_shoot.fric_current_set[i];
    }

    (void)ShootStateWrite(&state);
}

static void ShootClearTriggerOutput(void)
{
    PID_clear(&g_shoot.trigger_motor_pid);
    g_shoot.trigger_speed_set = 0.0f;
    g_shoot.speed_set = 0.0f;
    g_shoot.given_current = 0;
    g_shoot.move_flag = 0;
    g_shoot.block_time = 0;
    g_shoot.reverse_time = 0;
}

static void ShootClearFricOutput(void)
{
    for (uint8_t i = 0; i < FRIC_MOTOR_NUM; i++)
    {
        PID_clear(&g_shoot.fric_speed_pid[i]);
        g_shoot.fric_current_set[i] = 0;
    }

    (void)MotorInstSetCurrentBindsBestEffort(ShootFrictionCurrentBindings,
                                                              ShootFricZeroCurrentCmd,
                                                              FRIC_MOTOR_NUM);

    g_shoot.fric_speed_ramp.out = SHOOT_FRIC_SPEED_OFF_RPM;
    g_shoot.fric_speed_set = SHOOT_FRIC_SPEED_OFF_RPM;
}

void ShootStopOutputs(void)
{
    g_shoot.mode = SHOOT_STOP;
    ShootClearTriggerOutput();
    ShootClearFricOutput();
    ShootWriteState();
}

static bool_t ShootFricSpeedReady(void)
{
    const fp32 speed_set = g_shoot.fric_speed_ramp.max_value;
    if (speed_set <= 1.0f)
    {
        return 1;
    }

    for (uint8_t i = 0; i < FRIC_MOTOR_NUM; i++)
    {
        const int8_t dir = SHOOT_FRIC_DIR(i);
        if (dir == 0)
        {
            continue;
        }

        const motor_measure_t *m = get_friction_motor_measure_point(i);
        if (m == NULL)
        {
            return 0;
        }

        if (fabsf((fp32)m->speed_rpm) < fabsf(speed_set) * SHOOT_FRIC_READY_RATIO)
        {
            return 0;
        }
    }

    return 1;
}


/**
  * @brief          射击初始化，初始化PID，遥控器指针，电机指针
  * @param[in]      void
  * @retval         返回空
  */
void ShootInit(void)
{

    const fp32 Trigger_speed_pid[3] = {TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD};
    const fp32 Fric_speed_pid[3] = {g_config.shoot.fric_speed_pid.kp, g_config.shoot.fric_speed_pid.ki, g_config.shoot.fric_speed_pid.kd};
    (void)MotorInstBindCurrent(ShootFrictionMotorNames,
                                              FRIC_MOTOR_NUM,
                                              ShootFrictionCurrentBindings,
                                              FRIC_MOTOR_NUM);
    g_shoot.mode = SHOOT_STOP;
    //遥控器指针
    g_shoot.ShootRc = get_remote_control_point();
    // motor feedback pointer
    g_shoot.ShootMotorMeasure = get_trigger_motor_measure_point();
    //初始化PID
    PID_init(&g_shoot.trigger_motor_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
    for (uint8_t i = 0; i < FRIC_MOTOR_NUM; i++)
    {
        PID_init(&g_shoot.fric_speed_pid[i], PID_POSITION, Fric_speed_pid, g_config.shoot.fric_speed_pid.max_out, g_config.shoot.fric_speed_pid.max_iout);
        g_shoot.fric_current_set[i] = 0;
    }
    // update feedback data
    ShootFeedbackUpdate();
    ramp_init(&g_shoot.fric_speed_ramp, SHOOT_CONTROL_TIME * 0.001f, SHOOT_FRIC_SPEED_RPM, SHOOT_FRIC_SPEED_OFF_RPM);
    g_shoot.fric_speed_ramp.out = SHOOT_FRIC_SPEED_OFF_RPM;
    g_shoot.fric_speed_set = SHOOT_FRIC_SPEED_OFF_RPM;
    g_shoot.ecd_count = 0;
    g_shoot.angle = (g_shoot.ShootMotorMeasure != NULL) ?
        (g_shoot.ShootMotorMeasure->ecd * MOTOR_ECD_TO_ANGLE) : 0.0f;
    g_shoot.trigger_measure_ready = 0u;
    g_shoot.given_current = 0;
    g_shoot.move_flag = 0;
    g_shoot.set_angle = g_shoot.angle;
    g_shoot.speed = 0.0f;
    g_shoot.speed_set = 0.0f;
    g_shoot.key_time = 0;
    ShootWriteState();
}

/**
  * @brief          射击循环
  * @param[in]      void
  * @retval         返回can控制值
  */
int16_t ShootControlLoop(void)
{
    static uint8_t entertain_entered = 0u;

    // 函数地图：先处理娱乐模式；再跑射击状态机；最后分别输出拨弹和摩擦轮电流。
    if (robot_mode_is_entertain() != 0u)
    {
        if (entertain_entered == 0u)
        {
            entertain_entered = 1u;
            g_shoot.mode = SHOOT_STOP;
            ShootClearTriggerOutput();
            ShootClearFricOutput();
        }
        else
        {
            g_shoot.mode = SHOOT_STOP;
        }
        ShootWriteState();
        return 0;
    }
    entertain_entered = 0u;

    ShootSetMode();        //设置状态机
    ShootFeedbackUpdate(); // update feedback data
    const bool_t allow_fric = (robot_mode_allow_shoot_fric() != 0u) ? 1 : 0;
    const bool_t allow_trigger = (robot_mode_allow_shoot_trigger() != 0u) ? 1 : 0;
    const uint16_t ShootSw = ShootGetEffectiveSwitch();
    const bool_t sw_ready = ShootSwitchIsReady(ShootSw);
    const bool_t sw_fire = ShootSwitchIsFire(ShootSw);


    if (g_shoot.mode == SHOOT_STOP)
    {
        //设置拨弹轮的速度
        g_shoot.speed_set = 0.0f;
    }
    else if (g_shoot.mode == SHOOT_READY_FRIC)
    {
        //设置拨弹轮的速度
        g_shoot.speed_set = 0.0f;
    }
    else if(g_shoot.mode ==SHOOT_READY_BULLET)
    {
        if(g_shoot.key == SWITCH_TRIGGER_OFF)
        {
            //设置拨弹轮的拨动速度,并开启堵转反转处理
            g_shoot.trigger_speed_set = READY_TRIGGER_SPEED;
            trigger_motor_turn_back();
        }
        else
        {
            g_shoot.trigger_speed_set = 0.0f;
            g_shoot.speed_set = 0.0f;
        }
        g_shoot.trigger_motor_pid.max_out = TRIGGER_READY_PID_MAX_OUT;
        g_shoot.trigger_motor_pid.max_iout = TRIGGER_READY_PID_MAX_IOUT;
    }
    else if (g_shoot.mode == SHOOT_READY)
    {
        //设置拨弹轮的速度
         g_shoot.speed_set = 0.0f;
    }
    else if (g_shoot.mode == SHOOT_BULLET)
    {
        g_shoot.trigger_motor_pid.max_out = TRIGGER_BULLET_PID_MAX_OUT;
        g_shoot.trigger_motor_pid.max_iout = TRIGGER_BULLET_PID_MAX_IOUT;
        ShootBulletControl();
    }
    else if (g_shoot.mode == SHOOT_CONTINUE_BULLET)
    {
        //设置拨弹轮的拨动速度,并开启堵转反转处理
        g_shoot.trigger_speed_set = CONTINUE_TRIGGER_SPEED;
        trigger_motor_turn_back();
    }
    else if(g_shoot.mode == SHOOT_DONE)
    {
        g_shoot.speed_set = 0.0f;
    }

    if(g_shoot.mode == SHOOT_STOP)
    {
        // STOP overwrites LowCmd with zero current. Do not run the speed PID toward 0 RPM.
        ShootClearTriggerOutput();
        ShootClearFricOutput();
    }
    else
    {
        PID_calc(&g_shoot.trigger_motor_pid, g_shoot.speed, g_shoot.speed_set);
        g_shoot.given_current = (int16_t)(g_shoot.trigger_motor_pid.out);
        if(g_shoot.mode < SHOOT_READY_BULLET)
        {
            g_shoot.given_current = 0;
        }
        // 摩擦轮目标转速斜坡：平滑起转/提速
        const fp32 fric_ramp_step = sw_ready ? (SHOOT_FRIC_SPEED_STEP_RPM_S * 0.5f) : SHOOT_FRIC_SPEED_STEP_RPM_S;
        ramp_calc(&g_shoot.fric_speed_ramp, fric_ramp_step);

    }

    if(!allow_trigger || !sw_fire)
    {
        ShootClearTriggerOutput();
    }

    if(!allow_fric)
    {
        ShootClearFricOutput();
    }

    if(!allow_fric && !allow_trigger)
    {
        g_shoot.mode = SHOOT_STOP;
    }

    // 速度环：每路用反馈转速闭环输出电流
    if (allow_fric && g_shoot.mode != SHOOT_STOP)
    {
        int16_t fric_current_cmd[FRIC_MOTOR_NUM] = {0};
        g_shoot.fric_speed_set = g_shoot.fric_speed_ramp.out;
        for (uint8_t i = 0; i < FRIC_MOTOR_NUM; i++)
        {
            const int8_t dir = SHOOT_FRIC_DIR(i);
            if (dir == 0)
            {
                g_shoot.fric_current_set[i] = 0;
                PID_clear(&g_shoot.fric_speed_pid[i]);
                fric_current_cmd[i] = 0;
                continue;
            }

            const motor_measure_t *m = get_friction_motor_measure_point(i);
            const fp32 speed_fdb = (m != NULL) ? (fp32)m->speed_rpm : 0.0f;
            const fp32 speed_set = g_shoot.fric_speed_set * (fp32)dir;
            const int16_t current_raw = (int16_t)PID_calc(&g_shoot.fric_speed_pid[i], speed_fdb, speed_set);
            const int16_t current = MotorCfgLimitCurrentNode(&g_config.motor.friction[i], current_raw);
            g_shoot.fric_current_set[i] = current;
            fric_current_cmd[i] = current;
        }

        (void)MotorInstSetCurrentBindsBestEffort(ShootFrictionCurrentBindings,
                                                                  fric_current_cmd,
                                                                  FRIC_MOTOR_NUM);
    }
    ShootWriteState();
    return g_shoot.given_current;
}

/**
  * @brief          射击状态机设置：上=停火；中=仅摩擦轮预热（拨盘不动）；下=允许拨盘进入预备/射击（等同原“中档”逻辑）
  * @param[in]      void
  * @retval         void
  */
static void ShootSetMode(void)
{
    // 函数地图：先按拨杆定大状态，再叠加运行模式、鼠标/微动开关和完成/堵转条件。
    const bool_t allow_fric = (robot_mode_allow_shoot_fric() != 0u) ? 1 : 0;
    const bool_t allow_trigger = (robot_mode_allow_shoot_trigger() != 0u) ? 1 : 0;
    const uint16_t ShootSw = ShootGetRawSwitch();
    const bool_t sw_fire = ShootSwitchIsFire(ShootSw);

    // 拨杆位置优先控制：上=停火；中=预热摩擦轮（拨盘不动）；下=允许拨盘进入 READY_BULLET/READY 等逻辑
    if (ShootSwitchIsStop(ShootSw))
    {
        g_shoot.mode = SHOOT_STOP;
    }
    else if (ShootSwitchIsReady(ShootSw))
    {
        // 进入预热：摩擦轮加速至目标，等待 READY_BULLET
        g_shoot.mode = SHOOT_READY_FRIC;
    }
    else if (ShootSwitchIsFire(ShootSw))
    {
        // 启动射击：摩擦轮提速，准备好后持续拨盘
        g_shoot.mode = SHOOT_READY_FRIC;
    }

    const bool_t fric_set_done = (g_shoot.fric_speed_ramp.out == g_shoot.fric_speed_ramp.max_value);
    const bool_t fric_ready = fric_set_done && ShootFricSpeedReady();

    if(sw_fire && g_shoot.mode == SHOOT_READY_FRIC && (fric_ready || (!allow_fric && allow_trigger)))
    {
        g_shoot.mode = SHOOT_READY_BULLET;
    }
    else if(g_shoot.mode == SHOOT_READY_BULLET && g_shoot.key == SWITCH_TRIGGER_ON)
    {
        g_shoot.mode = SHOOT_READY;
    }
    else if(g_shoot.mode == SHOOT_READY && g_shoot.key == SWITCH_TRIGGER_OFF)
    {
        g_shoot.mode = SHOOT_READY_BULLET;
    }
    else if(g_shoot.mode == SHOOT_READY)
    {
        // 仅鼠标按键边沿启动射击（下档不再自动连发/触发）
        if ((g_shoot.press_l && g_shoot.last_press_l == 0) || (g_shoot.press_r && g_shoot.last_press_r == 0))
        {
            g_shoot.mode = SHOOT_BULLET;
        }
    }
    else if(g_shoot.mode == SHOOT_DONE)
    {
        if(g_shoot.key == SWITCH_TRIGGER_OFF)
        {
            g_shoot.key_time = ShootU16AddSat(g_shoot.key_time, ShootTickMs(), SHOOT_DONE_KEY_OFF_TIME);
            if(g_shoot.key_time >= SHOOT_DONE_KEY_OFF_TIME)
            {
                g_shoot.key_time = 0;
                g_shoot.mode = SHOOT_READY_BULLET;
            }
        }
        else
        {
            g_shoot.key_time = 0;
            g_shoot.mode = SHOOT_BULLET;
        }
    }



    if(g_shoot.mode > SHOOT_READY_FRIC)
    {
        //鼠标长按一直进入射击状态 保持连发
        if ((g_shoot.press_l_time == PRESS_LONG_TIME) || (g_shoot.press_r_time == PRESS_LONG_TIME))
        {
            g_shoot.mode = SHOOT_CONTINUE_BULLET;
        }
        else if(g_shoot.mode == SHOOT_CONTINUE_BULLET)
        {
            g_shoot.mode =SHOOT_READY_BULLET;
        }
    }

    get_shoot_heat0_limit_and_heat0(&g_shoot.heat_limit, &g_shoot.heat);
    if(!toe_is_error(REFEREE_TOE) && (g_shoot.heat + SHOOT_HEAT_REMAIN_VALUE > g_shoot.heat_limit))
    {
        if(g_shoot.mode == SHOOT_BULLET ||
           g_shoot.mode == SHOOT_CONTINUE_BULLET ||
           g_shoot.mode == SHOOT_READY)
        {
            g_shoot.mode =SHOOT_READY_BULLET;
        }
    }
    //如果云台状态是 无力状态，就关闭射击
    if (ShootGimbalCmdToShootStop())
    {
        g_shoot.mode = SHOOT_STOP;
    }

    if(!allow_trigger && g_shoot.mode > SHOOT_READY_FRIC)
    {
        g_shoot.mode = SHOOT_READY_FRIC;
    }

    if(!allow_fric && !allow_trigger)
    {
        g_shoot.mode = SHOOT_STOP;
    }

}
/**
  * @brief          update shoot feedback data.
  * @param[in]      void
  * @retval         void
  */
static void ShootFeedbackUpdate(void)
{
    // 函数地图：滤波拨弹速度；维护编码器圈数/输出角；读取微动开关；更新长按和发射完成状态。
    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};
    static second_order_filter_type_t speed_filter;
    static bool_t speed_filter_inited = 0;
    const uint16_t tick_ms = ShootTickMs();
    const uint8_t trigger_online = (toe_is_error(TRIGGER_MOTOR_TOE) == 0u) ? 1u : 0u;
    ManualInputState rc_snapshot = {0};
    const uint8_t rc_valid = ManualInputGetCurrentCopy(&rc_snapshot);

    //拨弹轮电机速度滤波一下（二阶低通）
    if (!speed_filter_inited)
    {
        second_order_filter_init(&speed_filter, fliter_num, 0.0f);
        speed_filter_inited = 1;
    }

    if (trigger_online == 0u)
    {
        g_shoot.trigger_measure_ready = 0u;
    }

    if (g_shoot.ShootMotorMeasure == NULL)
    {
        g_shoot.speed = second_order_filter_cali(&speed_filter, 0.0f);
    }
    else
    {
        g_shoot.speed = second_order_filter_cali(&speed_filter,
                                                       g_shoot.ShootMotorMeasure->speed_rpm * MOTOR_RPM_TO_SPEED);
    }

    //电机圈数重置， 因为输出轴旋转一圈， 电机轴旋转 36圈，将电机轴数据处理成输出轴数据，用于控制输出轴角度
    if (g_shoot.ShootMotorMeasure != NULL)
    {
        if (g_shoot.trigger_measure_ready == 0u)
        {
            g_shoot.ecd_count = 0;
            g_shoot.angle = g_shoot.ShootMotorMeasure->ecd * MOTOR_ECD_TO_ANGLE;
            g_shoot.set_angle = g_shoot.angle;
            g_shoot.move_flag = 0;
            g_shoot.trigger_measure_ready = trigger_online;
        }
        else
        {
            if (g_shoot.ShootMotorMeasure->ecd - g_shoot.ShootMotorMeasure->last_ecd > HALF_ECD_RANGE)
            {
                g_shoot.ecd_count--;
            }
            else if (g_shoot.ShootMotorMeasure->ecd - g_shoot.ShootMotorMeasure->last_ecd < -HALF_ECD_RANGE)
            {
                g_shoot.ecd_count++;
            }

            if (g_shoot.ecd_count == FULL_COUNT)
            {
                g_shoot.ecd_count = -(FULL_COUNT - 1);
            }
            else if (g_shoot.ecd_count == -FULL_COUNT)
            {
                g_shoot.ecd_count = FULL_COUNT - 1;
            }

            g_shoot.angle = (g_shoot.ecd_count * ECD_RANGE + g_shoot.ShootMotorMeasure->ecd) * MOTOR_ECD_TO_ANGLE;
        }
    }
    //微动开关
    uint8_t trig_level = 0u;
    if (BspShootTrigReadRaw(&trig_level) != 0u)
    {
        g_shoot.key = (bool_t)trig_level;
    }
    else
    {
        g_shoot.key = (bool_t)SWITCH_TRIGGER_OFF;
    }
    //榧犳爣鎸夐敭
    g_shoot.last_press_l = g_shoot.press_l;
    g_shoot.last_press_r = g_shoot.press_r;
    g_shoot.press_l = (rc_valid != 0u) ? rc_snapshot.mouse.press_l : 0u;
    if (ImageRemoteAuxFireRequested())
    {
        g_shoot.press_l = 1u;
    }
    g_shoot.press_r = (rc_valid != 0u) ? rc_snapshot.mouse.press_r : 0u;
    //长按计时
    if (g_shoot.press_l)
    {
        if (g_shoot.press_l_time < PRESS_LONG_TIME)
        {
            g_shoot.press_l_time = ShootU16AddSat(g_shoot.press_l_time, tick_ms, PRESS_LONG_TIME);
        }
    }
    else
    {
        g_shoot.press_l_time = 0;
    }

    if (g_shoot.press_r)
    {
        if (g_shoot.press_r_time < PRESS_LONG_TIME)
        {
            g_shoot.press_r_time = ShootU16AddSat(g_shoot.press_r_time, tick_ms, PRESS_LONG_TIME);
        }
    }
    else
    {
        g_shoot.press_r_time = 0;
    }

    //射击开关下档时间计时
    const uint16_t effective_sw = ShootGetEffectiveSwitch();
    if (g_shoot.mode != SHOOT_STOP && ShootSwitchIsFire(effective_sw))
    {
        if (g_shoot.rc_s_time < RC_S_LONG_TIME)
        {
            g_shoot.rc_s_time = ShootU16AddSat(g_shoot.rc_s_time, tick_ms, RC_S_LONG_TIME);
        }
    }
    else
    {
        g_shoot.rc_s_time = 0;
    }

    // 摩擦轮只保留一个目标转速，不再区分左键/右键高速。
    g_shoot.fric_speed_ramp.max_value = SHOOT_FRIC_SPEED_RPM;


}

static void trigger_motor_turn_back(void)
{
    const uint16_t tick_ms = ShootTickMs();

    if( g_shoot.block_time < BLOCK_TIME)
    {
        g_shoot.speed_set = g_shoot.trigger_speed_set;
    }
    else
    {
        g_shoot.speed_set = -g_shoot.trigger_speed_set;
    }

    if(fabsf(g_shoot.speed) < BLOCK_TRIGGER_SPEED && g_shoot.block_time < BLOCK_TIME)
    {
        g_shoot.block_time = ShootU16AddSat(g_shoot.block_time, tick_ms, BLOCK_TIME);
        g_shoot.reverse_time = 0;
    }
    else if (g_shoot.block_time >= BLOCK_TIME && g_shoot.reverse_time < REVERSE_TIME)
    {
        g_shoot.reverse_time = ShootU16AddSat(g_shoot.reverse_time, tick_ms, REVERSE_TIME);
    }
    else
    {
        g_shoot.block_time = 0u;
    }
}

/**
  * @brief          射击控制，控制拨弹电机角度，完成一次发射
  * @param[in]      void
  * @retval         void
  */
static void ShootBulletControl(void)
{

    //每次拨动 1/4PI的角度
    if (g_shoot.move_flag == 0)
    {
        g_shoot.set_angle = rad_format(g_shoot.angle + PI_TEN);
        g_shoot.move_flag = 1;
    }
    if(g_shoot.key == SWITCH_TRIGGER_OFF)
    {

        g_shoot.mode = SHOOT_DONE;
    }
    // check whether target angle has been reached
    if (rad_format(g_shoot.set_angle - g_shoot.angle) > 0.05f)
    {
        //没到达一直设置旋转速度
        g_shoot.trigger_speed_set = TRIGGER_SPEED;
        trigger_motor_turn_back();
    }
    else
    {
        g_shoot.move_flag = 0;
    }
}

#endif
