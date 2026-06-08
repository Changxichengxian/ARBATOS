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
 * - 中段：shoot_control_loop() 串起状态机、反馈更新、PID 电流输出。
 * - 后段：shoot_set_mode() 决定射击状态，feedback_update() 维护编码器圈数和堵转信息。
 * - 输出：拨弹电流作为返回值，摩擦轮电流写入 LowCmd。
 */


#include "shoot.h"

#include <math.h>
#include "cmsis_os.h"

#include "bsp_shoot_trig.h"
#include "user_lib.h"
#include "referee.h"

#include "CAN_receive.h"
#include "LowCmd.h"
#include "motor_instance.h"
#include "motor_config.h"
#include "gimbal_state.h"
#include "shoot_state.h"
#include "control_input.h"
#include "detect_task.h"
#include "robot_mode.h"
#include "pid.h"
#include "host_link_task.h"

// 微动开关 GPIO 是板相关差异，通过 BSP 读取。

/**
  * @brief          射击状态机设置，遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗，一直处在下，则持续发射，用于3min准备时间清理子弹
  * @param[in]      void
  * @retval         void
  */
static void shoot_set_mode(void);
/**
  * @brief          update shoot feedback data.
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void);

/**
  * @brief          清空摩擦轮输出（速度环 PID / 目标转速 / 电流指令）
  * @param[in]      void
  * @retval         void
  */
static void shoot_clear_fric_output(void);

/**
  * @brief          摩擦轮到速判定（基于电机反馈转速）
  * @param[in]      void
  * @retval         1: ready 0: not ready
  */
static bool_t shoot_fric_speed_ready(void);
static bool_t shoot_gimbal_cmd_to_shoot_stop(void);
static void shoot_write_state(void);

static const char *const shoot_friction_motor_names[FRIC_MOTOR_NUM] = {
    "motor.friction0",
    "motor.friction1",
    "motor.friction2",
    "motor.friction3",
};
static motor_instance_current_binding_t shoot_friction_current_bindings[FRIC_MOTOR_NUM];
static const int16_t shoot_fric_zero_current_cmd[FRIC_MOTOR_NUM] = {0};

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
static void shoot_bullet_control(void);

static uint16_t shoot_tick_ms(void)
{
    return (SHOOT_CONTROL_TIME > 0u) ? SHOOT_CONTROL_TIME : 1u;
}

static uint16_t shoot_u16_add_sat(uint16_t value, uint16_t add_ms, uint16_t max_value)
{
    const uint32_t next = (uint32_t)value + (uint32_t)add_ms;
    return (next >= (uint32_t)max_value) ? max_value : (uint16_t)next;
}

static uint16_t shoot_switch_raw_from_pos(uint8_t pos)
{
    return (uint16_t)input_switch_pos_to_raw(pos);
}

static uint8_t shoot_switch_is_stop(uint16_t raw_sw)
{
    return input_switch_is_pos(raw_sw, g_config.manual_input.semantics.shoot_stop_pos);
}

static uint8_t shoot_switch_is_ready(uint16_t raw_sw)
{
    return input_switch_is_pos(raw_sw, g_config.manual_input.semantics.shoot_ready_pos);
}

static uint8_t shoot_switch_is_fire(uint16_t raw_sw)
{
    return input_switch_is_pos(raw_sw, g_config.manual_input.semantics.shoot_fire_pos);
}

static input_switch_e shoot_get_image_switch_input(void)
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

static uint16_t shoot_get_raw_switch(void)
{
    uint16_t raw_sw = (uint16_t)input_switch(INPUT_SW_SHOOT_MODE);

    if (remote_control_get_active_source() != MANUAL_INPUT_SRC_IMAGE)
    {
        return raw_sw;
    }

    image_remote_state_t image_state;
    if (!image_remote_get_state(&image_state) ||
        image_state.proto != SDLOG_MANUAL_INPUT_PROTO_IMAGE_VT13)
    {
        return raw_sw;
    }

    raw_sw = (uint16_t)input_switch(shoot_get_image_switch_input());
    return shoot_switch_is_stop(raw_sw) ? shoot_switch_raw_from_pos(g_config.manual_input.semantics.shoot_stop_pos) :
                                          shoot_switch_raw_from_pos(g_config.manual_input.semantics.shoot_fire_pos);
}

static uint16_t shoot_get_effective_switch(void)
{
    static uint8_t gate_inited = 0u;
    static uint8_t down_engaged = 0u;
    static uint16_t last_sw_raw = RC_SW_UP;

    const uint16_t raw_sw = shoot_get_raw_switch();
    const uint8_t manual_online = toe_is_error(DBUS_TOE) ? 0u : 1u;
    const uint16_t shoot_stop_raw = shoot_switch_raw_from_pos(g_config.manual_input.semantics.shoot_stop_pos);
    const uint16_t shoot_ready_raw = shoot_switch_raw_from_pos(g_config.manual_input.semantics.shoot_ready_pos);
    uint16_t effective_sw = raw_sw;

    if (manual_online == 0u)
    {
        gate_inited = 0u;
        down_engaged = 0u;
        last_sw_raw = shoot_stop_raw;
        return raw_sw;
    }

    if (gate_inited == 0u)
    {
        gate_inited = 1u;
        down_engaged = 0u;
        last_sw_raw = raw_sw;
    }

    if (!shoot_switch_is_fire(raw_sw))
    {
        down_engaged = 0u;
    }
    else if (!shoot_switch_is_fire(last_sw_raw))
    {
        down_engaged = 1u;
    }

    if (shoot_switch_is_fire(raw_sw) && down_engaged == 0u)
    {
        effective_sw = shoot_ready_raw;
    }

    last_sw_raw = raw_sw;
    return effective_sw;
}



shoot_control_t shoot_control;          // shoot control data

const shoot_control_t *get_shoot_control_point(void)
{
    return &shoot_control;
}

void shoot_tune_apply_fric_speed_pid(void)
{
    taskENTER_CRITICAL();
    for (uint8_t i = 0; i < FRIC_MOTOR_NUM; i++)
    {
        pid_type_def *dst = &shoot_control.fric_speed_pid[i];
        dst->Kp = g_config.shoot.fric_speed_pid.kp;
        dst->Ki = g_config.shoot.fric_speed_pid.ki;
        dst->Kd = g_config.shoot.fric_speed_pid.kd;
        dst->max_out = g_config.shoot.fric_speed_pid.max_out;
        dst->max_iout = g_config.shoot.fric_speed_pid.max_iout;
        PID_clear(dst);
    }
    taskEXIT_CRITICAL();
}

void shoot_tune_apply_trigger_pid(void)
{
    taskENTER_CRITICAL();
    shoot_control.trigger_motor_pid.Kp = g_config.shoot.trigger_angle_pid.kp;
    shoot_control.trigger_motor_pid.Ki = g_config.shoot.trigger_angle_pid.ki;
    shoot_control.trigger_motor_pid.Kd = g_config.shoot.trigger_angle_pid.kd;
    PID_clear(&shoot_control.trigger_motor_pid);
    taskEXIT_CRITICAL();
}

static bool_t shoot_gimbal_cmd_to_shoot_stop(void)
{
    gimbal_state_t state;
    return (gimbal_state_read(&state) != 0u && state.valid != 0u && state.fire_allowed == 0u) ? 1 : 0;
}

static void shoot_write_state(void)
{
    shoot_state_t state = {0};

    state.valid = 1u;
    state.mode = (uint8_t)shoot_control.shoot_mode;
    state.fric_speed_set = shoot_control.fric_speed_set;
    state.trigger_speed_set = shoot_control.trigger_speed_set;
    state.speed = shoot_control.speed;
    state.speed_set = shoot_control.speed_set;
    state.angle = shoot_control.angle;
    state.set_angle = shoot_control.set_angle;
    state.given_current = shoot_control.given_current;
    state.ecd_count = shoot_control.ecd_count;
    state.trigger_measure_ready = shoot_control.trigger_measure_ready;
    state.press_l = (uint8_t)shoot_control.press_l;
    state.press_r = (uint8_t)shoot_control.press_r;
    state.last_press_l = (uint8_t)shoot_control.last_press_l;
    state.last_press_r = (uint8_t)shoot_control.last_press_r;
    state.press_l_time = shoot_control.press_l_time;
    state.press_r_time = shoot_control.press_r_time;
    state.rc_s_time = shoot_control.rc_s_time;
    state.block_time = shoot_control.block_time;
    state.reverse_time = shoot_control.reverse_time;
    state.move_flag = (uint8_t)shoot_control.move_flag;
    state.key = (uint8_t)shoot_control.key;
    state.key_time = shoot_control.key_time;
    state.heat_limit = shoot_control.heat_limit;
    state.heat = shoot_control.heat;
    state.trigger_motor_pid = shoot_control.trigger_motor_pid;

    for (uint8_t i = 0u; i < FRIC_MOTOR_NUM && i < SHOOT_STATE_FRIC_MOTOR_COUNT; i++)
    {
        state.fric_speed_pid[i] = shoot_control.fric_speed_pid[i];
        state.fric_current_set[i] = shoot_control.fric_current_set[i];
    }

    (void)shoot_state_write(&state);
}

static void shoot_clear_trigger_output(void)
{
    PID_clear(&shoot_control.trigger_motor_pid);
    shoot_control.trigger_speed_set = 0.0f;
    shoot_control.speed_set = 0.0f;
    shoot_control.given_current = 0;
    shoot_control.move_flag = 0;
    shoot_control.block_time = 0;
    shoot_control.reverse_time = 0;
}

static void shoot_clear_fric_output(void)
{
    for (uint8_t i = 0; i < FRIC_MOTOR_NUM; i++)
    {
        PID_clear(&shoot_control.fric_speed_pid[i]);
        shoot_control.fric_current_set[i] = 0;
    }

    (void)motor_instance_cmd_set_current_bindings_best_effort(shoot_friction_current_bindings,
                                                              shoot_fric_zero_current_cmd,
                                                              FRIC_MOTOR_NUM);

    shoot_control.fric_speed_ramp.out = SHOOT_FRIC_SPEED_OFF_RPM;
    shoot_control.fric_speed_set = SHOOT_FRIC_SPEED_OFF_RPM;
}

static bool_t shoot_fric_speed_ready(void)
{
    const fp32 speed_set = shoot_control.fric_speed_ramp.max_value;
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
void shoot_init(void)
{

    const fp32 Trigger_speed_pid[3] = {TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD};
    const fp32 Fric_speed_pid[3] = {g_config.shoot.fric_speed_pid.kp, g_config.shoot.fric_speed_pid.ki, g_config.shoot.fric_speed_pid.kd};
    (void)motor_instance_bind_current_outputs(shoot_friction_motor_names,
                                              FRIC_MOTOR_NUM,
                                              shoot_friction_current_bindings,
                                              FRIC_MOTOR_NUM);
    shoot_control.shoot_mode = SHOOT_STOP;
    //遥控器指针
    shoot_control.shoot_rc = get_remote_control_point();
    // motor feedback pointer
    shoot_control.shoot_motor_measure = get_trigger_motor_measure_point();
    //初始化PID
    PID_init(&shoot_control.trigger_motor_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
    for (uint8_t i = 0; i < FRIC_MOTOR_NUM; i++)
    {
        PID_init(&shoot_control.fric_speed_pid[i], PID_POSITION, Fric_speed_pid, g_config.shoot.fric_speed_pid.max_out, g_config.shoot.fric_speed_pid.max_iout);
        shoot_control.fric_current_set[i] = 0;
    }
    // update feedback data
    shoot_feedback_update();
    ramp_init(&shoot_control.fric_speed_ramp, SHOOT_CONTROL_TIME * 0.001f, SHOOT_FRIC_SPEED_RPM, SHOOT_FRIC_SPEED_OFF_RPM);
    shoot_control.fric_speed_ramp.out = SHOOT_FRIC_SPEED_OFF_RPM;
    shoot_control.fric_speed_set = SHOOT_FRIC_SPEED_OFF_RPM;
    shoot_control.ecd_count = 0;
    shoot_control.angle = (shoot_control.shoot_motor_measure != NULL) ?
        (shoot_control.shoot_motor_measure->ecd * MOTOR_ECD_TO_ANGLE) : 0.0f;
    shoot_control.trigger_measure_ready = 0u;
    shoot_control.given_current = 0;
    shoot_control.move_flag = 0;
    shoot_control.set_angle = shoot_control.angle;
    shoot_control.speed = 0.0f;
    shoot_control.speed_set = 0.0f;
    shoot_control.key_time = 0;
    shoot_write_state();
}

/**
  * @brief          射击循环
  * @param[in]      void
  * @retval         返回can控制值
  */
int16_t shoot_control_loop(void)
{
    static uint8_t entertain_entered = 0u;

    // 函数地图：先处理娱乐模式；再跑射击状态机；最后分别输出拨弹和摩擦轮电流。
    if (robot_mode_is_entertain() != 0u)
    {
        if (entertain_entered == 0u)
        {
            entertain_entered = 1u;
            shoot_control.shoot_mode = SHOOT_STOP;
            shoot_clear_trigger_output();
            shoot_clear_fric_output();
        }
        else
        {
            shoot_control.shoot_mode = SHOOT_STOP;
        }
        shoot_write_state();
        return 0;
    }
    entertain_entered = 0u;

    shoot_set_mode();        //设置状态机
    shoot_feedback_update(); // update feedback data
    const bool_t allow_fric = (robot_mode_allow_shoot_fric() != 0u) ? 1 : 0;
    const bool_t allow_trigger = (robot_mode_allow_shoot_trigger() != 0u) ? 1 : 0;
    const uint16_t shoot_sw = shoot_get_effective_switch();
    const bool_t sw_ready = shoot_switch_is_ready(shoot_sw);
    const bool_t sw_fire = shoot_switch_is_fire(shoot_sw);


    if (shoot_control.shoot_mode == SHOOT_STOP)
    {
        //设置拨弹轮的速度
        shoot_control.speed_set = 0.0f;
    }
    else if (shoot_control.shoot_mode == SHOOT_READY_FRIC)
    {
        //设置拨弹轮的速度
        shoot_control.speed_set = 0.0f;
    }
    else if(shoot_control.shoot_mode ==SHOOT_READY_BULLET)
    {
        if(shoot_control.key == SWITCH_TRIGGER_OFF)
        {
            //设置拨弹轮的拨动速度,并开启堵转反转处理
            shoot_control.trigger_speed_set = READY_TRIGGER_SPEED;
            trigger_motor_turn_back();
        }
        else
        {
            shoot_control.trigger_speed_set = 0.0f;
            shoot_control.speed_set = 0.0f;
        }
        shoot_control.trigger_motor_pid.max_out = TRIGGER_READY_PID_MAX_OUT;
        shoot_control.trigger_motor_pid.max_iout = TRIGGER_READY_PID_MAX_IOUT;
    }
    else if (shoot_control.shoot_mode == SHOOT_READY)
    {
        //设置拨弹轮的速度
         shoot_control.speed_set = 0.0f;
    }
    else if (shoot_control.shoot_mode == SHOOT_BULLET)
    {
        shoot_control.trigger_motor_pid.max_out = TRIGGER_BULLET_PID_MAX_OUT;
        shoot_control.trigger_motor_pid.max_iout = TRIGGER_BULLET_PID_MAX_IOUT;
        shoot_bullet_control();
    }
    else if (shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
    {
        //设置拨弹轮的拨动速度,并开启堵转反转处理
        shoot_control.trigger_speed_set = CONTINUE_TRIGGER_SPEED;
        trigger_motor_turn_back();
    }
    else if(shoot_control.shoot_mode == SHOOT_DONE)
    {
        shoot_control.speed_set = 0.0f;
    }

    if(shoot_control.shoot_mode == SHOOT_STOP)
    {
        // STOP overwrites LowCmd with zero current. Do not run the speed PID toward 0 RPM.
        shoot_clear_trigger_output();
        shoot_clear_fric_output();
    }
    else
    {
        PID_calc(&shoot_control.trigger_motor_pid, shoot_control.speed, shoot_control.speed_set);
        shoot_control.given_current = (int16_t)(shoot_control.trigger_motor_pid.out);
        if(shoot_control.shoot_mode < SHOOT_READY_BULLET)
        {
            shoot_control.given_current = 0;
        }
        // 摩擦轮目标转速斜坡：平滑起转/提速
        const fp32 fric_ramp_step = sw_ready ? (SHOOT_FRIC_SPEED_STEP_RPM_S * 0.5f) : SHOOT_FRIC_SPEED_STEP_RPM_S;
        ramp_calc(&shoot_control.fric_speed_ramp, fric_ramp_step);

    }

    if(!allow_trigger || !sw_fire)
    {
        shoot_clear_trigger_output();
    }

    if(!allow_fric)
    {
        shoot_clear_fric_output();
    }

    if(!allow_fric && !allow_trigger)
    {
        shoot_control.shoot_mode = SHOOT_STOP;
    }

    // 速度环：每路用反馈转速闭环输出电流
    if (allow_fric && shoot_control.shoot_mode != SHOOT_STOP)
    {
        int16_t fric_current_cmd[FRIC_MOTOR_NUM] = {0};
        shoot_control.fric_speed_set = shoot_control.fric_speed_ramp.out;
        for (uint8_t i = 0; i < FRIC_MOTOR_NUM; i++)
        {
            const int8_t dir = SHOOT_FRIC_DIR(i);
            if (dir == 0)
            {
                shoot_control.fric_current_set[i] = 0;
                PID_clear(&shoot_control.fric_speed_pid[i]);
                fric_current_cmd[i] = 0;
                continue;
            }

            const motor_measure_t *m = get_friction_motor_measure_point(i);
            const fp32 speed_fdb = (m != NULL) ? (fp32)m->speed_rpm : 0.0f;
            const fp32 speed_set = shoot_control.fric_speed_set * (fp32)dir;
            const int16_t current_raw = (int16_t)PID_calc(&shoot_control.fric_speed_pid[i], speed_fdb, speed_set);
            const int16_t current = motor_cfg_limit_current_node(&g_config.motor.friction[i], current_raw);
            shoot_control.fric_current_set[i] = current;
            fric_current_cmd[i] = current;
        }

        (void)motor_instance_cmd_set_current_bindings_best_effort(shoot_friction_current_bindings,
                                                                  fric_current_cmd,
                                                                  FRIC_MOTOR_NUM);
    }
    shoot_write_state();
    return shoot_control.given_current;
}

/**
  * @brief          射击状态机设置：上=停火；中=仅摩擦轮预热（拨盘不动）；下=允许拨盘进入预备/射击（等同原“中档”逻辑）
  * @param[in]      void
  * @retval         void
  */
static void shoot_set_mode(void)
{
    // 函数地图：先按拨杆定大状态，再叠加运行模式、鼠标/微动开关和完成/堵转条件。
    const bool_t allow_fric = (robot_mode_allow_shoot_fric() != 0u) ? 1 : 0;
    const bool_t allow_trigger = (robot_mode_allow_shoot_trigger() != 0u) ? 1 : 0;
    const uint16_t shoot_sw = shoot_get_raw_switch();
    const bool_t sw_fire = shoot_switch_is_fire(shoot_sw);

    // 拨杆位置优先控制：上=停火；中=预热摩擦轮（拨盘不动）；下=允许拨盘进入 READY_BULLET/READY 等逻辑
    if (shoot_switch_is_stop(shoot_sw))
    {
        shoot_control.shoot_mode = SHOOT_STOP;
    }
    else if (shoot_switch_is_ready(shoot_sw))
    {
        // 进入预热：摩擦轮加速至目标，等待 READY_BULLET
        shoot_control.shoot_mode = SHOOT_READY_FRIC;
    }
    else if (shoot_switch_is_fire(shoot_sw))
    {
        // 启动射击：摩擦轮提速，准备好后持续拨盘
        shoot_control.shoot_mode = SHOOT_READY_FRIC;
    }

    const bool_t fric_set_done = (shoot_control.fric_speed_ramp.out == shoot_control.fric_speed_ramp.max_value);
    const bool_t fric_ready = fric_set_done && shoot_fric_speed_ready();

    if(sw_fire && shoot_control.shoot_mode == SHOOT_READY_FRIC && (fric_ready || (!allow_fric && allow_trigger)))
    {
        shoot_control.shoot_mode = SHOOT_READY_BULLET;
    }
    else if(shoot_control.shoot_mode == SHOOT_READY_BULLET && shoot_control.key == SWITCH_TRIGGER_ON)
    {
        shoot_control.shoot_mode = SHOOT_READY;
    }
    else if(shoot_control.shoot_mode == SHOOT_READY && shoot_control.key == SWITCH_TRIGGER_OFF)
    {
        shoot_control.shoot_mode = SHOOT_READY_BULLET;
    }
    else if(shoot_control.shoot_mode == SHOOT_READY)
    {
        // 仅鼠标按键边沿启动射击（下档不再自动连发/触发）
        if ((shoot_control.press_l && shoot_control.last_press_l == 0) || (shoot_control.press_r && shoot_control.last_press_r == 0))
        {
            shoot_control.shoot_mode = SHOOT_BULLET;
        }
    }
    else if(shoot_control.shoot_mode == SHOOT_DONE)
    {
        if(shoot_control.key == SWITCH_TRIGGER_OFF)
        {
            shoot_control.key_time = shoot_u16_add_sat(shoot_control.key_time, shoot_tick_ms(), SHOOT_DONE_KEY_OFF_TIME);
            if(shoot_control.key_time >= SHOOT_DONE_KEY_OFF_TIME)
            {
                shoot_control.key_time = 0;
                shoot_control.shoot_mode = SHOOT_READY_BULLET;
            }
        }
        else
        {
            shoot_control.key_time = 0;
            shoot_control.shoot_mode = SHOOT_BULLET;
        }
    }



    if(shoot_control.shoot_mode > SHOOT_READY_FRIC)
    {
        //鼠标长按一直进入射击状态 保持连发
        if ((shoot_control.press_l_time == PRESS_LONG_TIME) || (shoot_control.press_r_time == PRESS_LONG_TIME))
        {
            shoot_control.shoot_mode = SHOOT_CONTINUE_BULLET;
        }
        else if(shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
        {
            shoot_control.shoot_mode =SHOOT_READY_BULLET;
        }
    }

    get_shoot_heat0_limit_and_heat0(&shoot_control.heat_limit, &shoot_control.heat);
    if(!toe_is_error(REFEREE_TOE) && (shoot_control.heat + SHOOT_HEAT_REMAIN_VALUE > shoot_control.heat_limit))
    {
        if(shoot_control.shoot_mode == SHOOT_BULLET ||
           shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET ||
           shoot_control.shoot_mode == SHOOT_READY)
        {
            shoot_control.shoot_mode =SHOOT_READY_BULLET;
        }
    }
    //如果云台状态是 无力状态，就关闭射击
    if (shoot_gimbal_cmd_to_shoot_stop())
    {
        shoot_control.shoot_mode = SHOOT_STOP;
    }

    if(!allow_trigger && shoot_control.shoot_mode > SHOOT_READY_FRIC)
    {
        shoot_control.shoot_mode = SHOOT_READY_FRIC;
    }

    if(!allow_fric && !allow_trigger)
    {
        shoot_control.shoot_mode = SHOOT_STOP;
    }

}
/**
  * @brief          update shoot feedback data.
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void)
{
    // 函数地图：滤波拨弹速度；维护编码器圈数/输出角；读取微动开关；更新长按和发射完成状态。
    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};
    static second_order_filter_type_t speed_filter;
    static bool_t speed_filter_inited = 0;
    const uint16_t tick_ms = shoot_tick_ms();
    const uint8_t trigger_online = (toe_is_error(TRIGGER_MOTOR_TOE) == 0u) ? 1u : 0u;
    manual_input_state_t rc_snapshot = {0};
    const uint8_t rc_valid = manual_input_get_current_copy(&rc_snapshot);

    //拨弹轮电机速度滤波一下（二阶低通）
    if (!speed_filter_inited)
    {
        second_order_filter_init(&speed_filter, fliter_num, 0.0f);
        speed_filter_inited = 1;
    }

    if (trigger_online == 0u)
    {
        shoot_control.trigger_measure_ready = 0u;
    }

    if (shoot_control.shoot_motor_measure == NULL)
    {
        shoot_control.speed = second_order_filter_cali(&speed_filter, 0.0f);
    }
    else
    {
        shoot_control.speed = second_order_filter_cali(&speed_filter,
                                                       shoot_control.shoot_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED);
    }

    //电机圈数重置， 因为输出轴旋转一圈， 电机轴旋转 36圈，将电机轴数据处理成输出轴数据，用于控制输出轴角度
    if (shoot_control.shoot_motor_measure != NULL)
    {
        if (shoot_control.trigger_measure_ready == 0u)
        {
            shoot_control.ecd_count = 0;
            shoot_control.angle = shoot_control.shoot_motor_measure->ecd * MOTOR_ECD_TO_ANGLE;
            shoot_control.set_angle = shoot_control.angle;
            shoot_control.move_flag = 0;
            shoot_control.trigger_measure_ready = trigger_online;
        }
        else
        {
            if (shoot_control.shoot_motor_measure->ecd - shoot_control.shoot_motor_measure->last_ecd > HALF_ECD_RANGE)
            {
                shoot_control.ecd_count--;
            }
            else if (shoot_control.shoot_motor_measure->ecd - shoot_control.shoot_motor_measure->last_ecd < -HALF_ECD_RANGE)
            {
                shoot_control.ecd_count++;
            }

            if (shoot_control.ecd_count == FULL_COUNT)
            {
                shoot_control.ecd_count = -(FULL_COUNT - 1);
            }
            else if (shoot_control.ecd_count == -FULL_COUNT)
            {
                shoot_control.ecd_count = FULL_COUNT - 1;
            }

            shoot_control.angle = (shoot_control.ecd_count * ECD_RANGE + shoot_control.shoot_motor_measure->ecd) * MOTOR_ECD_TO_ANGLE;
        }
    }
    //微动开关
    uint8_t trig_level = 0u;
    if (bsp_shoot_trig_read_raw(&trig_level) != 0u)
    {
        shoot_control.key = (bool_t)trig_level;
    }
    else
    {
        shoot_control.key = (bool_t)SWITCH_TRIGGER_OFF;
    }
    //榧犳爣鎸夐敭
    shoot_control.last_press_l = shoot_control.press_l;
    shoot_control.last_press_r = shoot_control.press_r;
    shoot_control.press_l = (rc_valid != 0u) ? rc_snapshot.mouse.press_l : 0u;
    if (image_remote_aux_fire_requested())
    {
        shoot_control.press_l = 1u;
    }
    shoot_control.press_r = (rc_valid != 0u) ? rc_snapshot.mouse.press_r : 0u;
    //长按计时
    if (shoot_control.press_l)
    {
        if (shoot_control.press_l_time < PRESS_LONG_TIME)
        {
            shoot_control.press_l_time = shoot_u16_add_sat(shoot_control.press_l_time, tick_ms, PRESS_LONG_TIME);
        }
    }
    else
    {
        shoot_control.press_l_time = 0;
    }

    if (shoot_control.press_r)
    {
        if (shoot_control.press_r_time < PRESS_LONG_TIME)
        {
            shoot_control.press_r_time = shoot_u16_add_sat(shoot_control.press_r_time, tick_ms, PRESS_LONG_TIME);
        }
    }
    else
    {
        shoot_control.press_r_time = 0;
    }

    //射击开关下档时间计时
    const uint16_t effective_sw = shoot_get_effective_switch();
    if (shoot_control.shoot_mode != SHOOT_STOP && shoot_switch_is_fire(effective_sw))
    {
        if (shoot_control.rc_s_time < RC_S_LONG_TIME)
        {
            shoot_control.rc_s_time = shoot_u16_add_sat(shoot_control.rc_s_time, tick_ms, RC_S_LONG_TIME);
        }
    }
    else
    {
        shoot_control.rc_s_time = 0;
    }

    // 摩擦轮只保留一个目标转速，不再区分左键/右键高速。
    shoot_control.fric_speed_ramp.max_value = SHOOT_FRIC_SPEED_RPM;


}

static void trigger_motor_turn_back(void)
{
    const uint16_t tick_ms = shoot_tick_ms();

    if( shoot_control.block_time < BLOCK_TIME)
    {
        shoot_control.speed_set = shoot_control.trigger_speed_set;
    }
    else
    {
        shoot_control.speed_set = -shoot_control.trigger_speed_set;
    }

    if(fabsf(shoot_control.speed) < BLOCK_TRIGGER_SPEED && shoot_control.block_time < BLOCK_TIME)
    {
        shoot_control.block_time = shoot_u16_add_sat(shoot_control.block_time, tick_ms, BLOCK_TIME);
        shoot_control.reverse_time = 0;
    }
    else if (shoot_control.block_time >= BLOCK_TIME && shoot_control.reverse_time < REVERSE_TIME)
    {
        shoot_control.reverse_time = shoot_u16_add_sat(shoot_control.reverse_time, tick_ms, REVERSE_TIME);
    }
    else
    {
        shoot_control.block_time = 0u;
    }
}

/**
  * @brief          射击控制，控制拨弹电机角度，完成一次发射
  * @param[in]      void
  * @retval         void
  */
static void shoot_bullet_control(void)
{

    //每次拨动 1/4PI的角度
    if (shoot_control.move_flag == 0)
    {
        shoot_control.set_angle = rad_format(shoot_control.angle + PI_TEN);
        shoot_control.move_flag = 1;
    }
    if(shoot_control.key == SWITCH_TRIGGER_OFF)
    {

        shoot_control.shoot_mode = SHOOT_DONE;
    }
    // check whether target angle has been reached
    if (rad_format(shoot_control.set_angle - shoot_control.angle) > 0.05f)
    {
        //没到达一直设置旋转速度
        shoot_control.trigger_speed_set = TRIGGER_SPEED;
        trigger_motor_turn_back();
    }
    else
    {
        shoot_control.move_flag = 0;
    }
}
