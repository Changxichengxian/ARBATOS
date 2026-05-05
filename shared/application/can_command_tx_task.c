/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

/*
 * 阅读地图：
 * - 前段：命令限幅、RM/MIT 命令换算、单轴协议处理。
 * - 中段：在线/离线收集各轴电流命令，并记录 CAN 电流日志。
 * - 后段：按轴装配表执行发送，RM 组帧缓存最后统一发出。
 * - 入口：can_command_tx_task() 每周期收集 actuator_cmd，再按电机配置发到 CAN/RS485。
 */

#include "can_command_tx_task.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include "CAN_receive.h"
#include "actuator_cmd.h"
#include <can_command_axis_bindings.h>
#include "can_mit_motor_driver.h"
#include "config.h"
#include "watch.h"
#include "detect_task.h"
#include "motor_config.h"
#include "sdlog.h"
#include "rt_profiler.h"
#include "robot_task_profile.h"

#include <string.h>

__weak uint8_t can_tx_process_extra_item(uint8_t bus,
                                         actuator_id_e actuator_id,
                                         const motor_node_param_t *node,
                                         int16_t current);

static int16_t s_can_tx_chassis_cmd[4];
static int16_t s_can_tx_friction_cmd[4];
static int16_t s_can_tx_yaw_cmd;
static int16_t s_can_tx_yaw_upper_cmd;
static int16_t s_can_tx_pitch_cmd;
static int16_t s_can_tx_trigger_cmd;

static int16_t s_can_tx_can1_200[4];
static int16_t s_can_tx_can1_1ff[4];
static int16_t s_can_tx_can2_200[4];
static int16_t s_can_tx_can2_1ff[4];

// 测试模式会限制底盘输出，避免只测云台或射击时底盘误动。
static inline bool_t can_tx_allow_chassis(void)
{
    const test_mode_e mode = (test_mode_e)g_config.test.mode;
    return (mode == TEST_MODE_NONE) || (mode == TEST_MODE_ENTERTAIN) || (mode == TEST_MODE_CHASSIS_ONLY);
}

// 按任务周期换算日志间隔，避免每个 CAN 周期都写 SD 卡。
static uint8_t can_tx_log_due(void)
{
    static uint16_t skip = 0u;
    const uint16_t period_ms = robot_profile_can_command_tx_period_ms();
    const uint16_t log_period_ms = robot_profile_can_command_tx_log_period_ms();
    const uint16_t divisor = (log_period_ms + period_ms - 1u) / period_ms;

    if (skip != 0u)
    {
        skip--;
        return 0u;
    }

    skip = (divisor > 1u) ? (uint16_t)(divisor - 1u) : 0u;
    return 1u;
}

static inline fp32 can_tx_clamp_fp32(fp32 x, fp32 x_min, fp32 x_max)
{
    if (x < x_min)
    {
        return x_min;
    }
    if (x > x_max)
    {
        return x_max;
    }
    return x;
}

static inline uint8_t can_tx_actuator_id_valid(actuator_id_e id)
{
    return ((uint32_t)id < (uint32_t)ACTUATOR_ID__COUNT) ? 1u : 0u;
}

// 判断执行器命令是否真正带输出；MIT 电机第一次有输出前不急着使能。
static inline uint8_t can_tx_cmd_nonzero(const actuator_cmd_t *cmd)
{
    if (cmd == NULL || cmd->active == 0u)
    {
        return 0u;
    }
    if (cmd->current != 0)
    {
        return 1u;
    }
    if (cmd->position != 0.0f || cmd->velocity != 0.0f ||
        cmd->kp != 0.0f || cmd->kd != 0.0f || cmd->torque != 0.0f)
    {
        return 1u;
    }
    return 0u;
}

static inline int16_t can_tx_fp32_to_i16_saturated(fp32 x)
{
    if (x > 32767.0f)
    {
        return 32767;
    }
    if (x < -32768.0f)
    {
        return -32768;
    }
    return (int16_t)x;
}

static inline uint8_t can_tx_cmd_mode_uses_position_feedback(actuator_cmd_mode_e mode)
{
    switch (mode)
    {
    case ACTUATOR_CMD_MODE_STATE_TORQUE:
    case ACTUATOR_CMD_MODE_POS_VEL:
    case ACTUATOR_CMD_MODE_FORCE_POS:
        return 1u;
    default:
        return 0u;
    }
}

static inline uint8_t can_tx_cmd_mode_uses_velocity_feedback(actuator_cmd_mode_e mode)
{
    if (can_tx_cmd_mode_uses_position_feedback(mode) != 0u)
    {
        return 1u;
    }
    return (mode == ACTUATOR_CMD_MODE_SPEED) ? 1u : 0u;
}

// 轴配置里可以指定 CAN 总线；没指定时使用该轴的默认总线。
static inline uint8_t can_tx_node_bus(uint8_t fallback_bus, const motor_node_param_t *node)
{
    return motor_cfg_can_bus(fallback_bus, node);
}

// RM 电机仍优先吃旧双环 PID 给出的电流；只有通用状态命令才临时换算成电流。
static int16_t can_tx_build_rm_current_from_actuator(actuator_id_e actuator_id,
                                                     int16_t current)
{
    actuator_cmd_t cmd;
    actuator_feedback_t fb;
    actuator_cmd_mode_e mode;
    fp32 rm_current;

    if (actuator_cmd_get_copy(actuator_id, &cmd) == 0u || cmd.active == 0u)
    {
        return current;
    }

    mode = (actuator_cmd_mode_e)cmd.mode;
    if (can_tx_cmd_mode_uses_velocity_feedback(mode) == 0u)
    {
        return current;
    }

    if (actuator_feedback_get_copy(actuator_id, &fb) == 0u || fb.online == 0u)
    {
        return current;
    }

    rm_current = cmd.torque;
    if (can_tx_cmd_mode_uses_position_feedback(mode) != 0u)
    {
        rm_current += cmd.kp * (cmd.position - fb.position);
    }
    if (can_tx_cmd_mode_uses_velocity_feedback(mode) != 0u)
    {
        rm_current += cmd.kd * (cmd.velocity - fb.velocity);
    }

    return can_tx_fp32_to_i16_saturated(rm_current);
}

// 旧控制链常给“电流”，MIT 电机要力矩，这里按型号量程换成力矩。
static inline fp32 can_tx_current_to_mit_torque(const motor_node_param_t *node,
                                                int16_t current,
                                                const can_mit_motor_limits_t *limits)
{
    const motor_model_db_entry_t *entry;
    int16_t range_abs = 0;
    fp32 torque;

    if (node == NULL || limits == NULL || limits->torque_max <= 0.0f)
    {
        return 0.0f;
    }

    entry = motor_cfg_model_db(node->model);
    if (entry != NULL)
    {
        range_abs = entry->cmd_current_range_abs;
    }
    if (range_abs <= 0)
    {
        range_abs = 32767;
    }

    torque = ((fp32)current) * limits->torque_max / (fp32)range_abs;
    return can_tx_clamp_fp32(torque, -limits->torque_max, limits->torque_max);
}

// 把通用执行器命令翻译成 MIT 命令；没有新命令时退回电流转力矩。
static inline void can_tx_build_mit_cmd_from_actuator(const motor_node_param_t *node,
                                                      const actuator_cmd_t *src,
                                                      int16_t current,
                                                      const can_mit_motor_limits_t *limits,
                                                      mit_motor_cmd_t *out)
{
    uint8_t mode = (uint8_t)ACTUATOR_CMD_MODE_CURRENT;

    if (out == NULL)
    {
        return;
    }

    (void)memset(out, 0, sizeof(*out));

    if (src != NULL && src->active != 0u && src->mode != (uint8_t)ACTUATOR_CMD_MODE_NONE)
    {
        mode = src->mode;
    }

    if (can_tx_cmd_mode_uses_position_feedback((actuator_cmd_mode_e)mode) != 0u)
    {
        out->position = (src != NULL) ? src->position : 0.0f;
        out->velocity = (src != NULL) ? src->velocity : 0.0f;
        out->kp = (src != NULL) ? src->kp : 0.0f;
        out->kd = (src != NULL) ? src->kd : 0.0f;
        out->torque = (src != NULL) ? src->torque : 0.0f;
    }
    else if (can_tx_cmd_mode_uses_velocity_feedback((actuator_cmd_mode_e)mode) != 0u)
    {
        out->velocity = (src != NULL) ? src->velocity : 0.0f;
        out->kd = (src != NULL) ? src->kd : 0.0f;
        out->torque = (src != NULL) ? src->torque : 0.0f;
    }
    else
    {
        out->torque = can_tx_current_to_mit_torque(node, current, limits);
    }
}

// 处理单个 MIT 轴：必要时先使能，再发送本周期命令。
static inline uint8_t can_tx_process_can_mit_item(uint8_t bus,
                                                  actuator_id_e actuator_id,
                                                  const motor_node_param_t *node,
                                                  int16_t current)
{
    static uint8_t mit_enabled[ACTUATOR_ID__COUNT];
    const can_mit_motor_limits_t *limits;
    actuator_cmd_t cmd;
    mit_motor_cmd_t mit_cmd;
    uint8_t have_cmd;
    uint16_t std_id;

    if (node == NULL)
    {
        return 0u;
    }

    limits = motor_cfg_mit_limits(node);
    if (limits == NULL)
    {
        return 0u;
    }

    (void)memset(&cmd, 0, sizeof(cmd));
    have_cmd = actuator_cmd_get_copy(actuator_id, &cmd);
    if (have_cmd == 0u || cmd.active == 0u)
    {
        cmd.active = 1u;
        cmd.mode = (uint8_t)ACTUATOR_CMD_MODE_CURRENT;
        cmd.current = current;
    }

    std_id = motor_cfg_can_id(node);
    if (std_id == 0u)
    {
        return 0u;
    }

    if (can_tx_actuator_id_valid(actuator_id) != 0u &&
        mit_enabled[actuator_id] == 0u &&
        can_tx_cmd_nonzero(&cmd) != 0u)
    {
        can_mit_motor_send_enable(bus, std_id);
        mit_enabled[actuator_id] = 1u;
    }

    if (can_tx_actuator_id_valid(actuator_id) != 0u &&
        mit_enabled[actuator_id] == 0u &&
        can_tx_cmd_nonzero(&cmd) == 0u)
    {
        return 1u;
    }

    can_tx_build_mit_cmd_from_actuator(node, &cmd, current, limits, &mit_cmd);
    can_mit_motor_send_cmd(bus, std_id, limits, &mit_cmd);
    return 1u;
}

static void can_tx_log_actuator_current(const sdlog_actuator_current_t *log)
{
    if (log != NULL && can_tx_log_due() != 0u)
    {
        sdlog_write(SDLOG_TAG_ACTUATOR_CURRENT, log, (uint16_t)sizeof(*log));
    }
}

static void can_tx_clear_cmd_cache(void)
{
    (void)memset(s_can_tx_chassis_cmd, 0, sizeof(s_can_tx_chassis_cmd));
    (void)memset(s_can_tx_friction_cmd, 0, sizeof(s_can_tx_friction_cmd));
    s_can_tx_yaw_cmd = 0;
    s_can_tx_yaw_upper_cmd = 0;
    s_can_tx_pitch_cmd = 0;
    s_can_tx_trigger_cmd = 0;
}

static void can_tx_limit_friction_cmds(void)
{
    for (uint8_t i = 0u; i < 4u; i++)
    {
        if (g_config.shoot.fric_motor_dir[i] == 0)
        {
            s_can_tx_friction_cmd[i] = 0;
        }
    }
}

// 遥控离线时只保留允许的安全输出，例如娱乐模式摩擦轮或调试 yaw。
static void can_tx_collect_offline_cmds(void)
{
    const test_mode_e mode = (test_mode_e)g_config.test.mode;
    const bool_t allow_friction_offline = (mode == TEST_MODE_ENTERTAIN);

    can_tx_clear_cmd_cache();

    if (allow_friction_offline)
    {
        s_can_tx_friction_cmd[0] = actuator_cmd_get_friction_current(0);
        s_can_tx_friction_cmd[1] = actuator_cmd_get_friction_current(1);
        s_can_tx_friction_cmd[2] = actuator_cmd_get_friction_current(2);
        s_can_tx_friction_cmd[3] = actuator_cmd_get_friction_current(3);
    }
    can_tx_limit_friction_cmds();
}

// 遥控在线时从 actuator_cmd 快照收集所有轴的本周期电流命令。
static void can_tx_collect_online_cmds(void)
{
    const bool_t allow_chassis = can_tx_allow_chassis();

    can_tx_clear_cmd_cache();

    s_can_tx_chassis_cmd[0] = allow_chassis ? actuator_cmd_get_chassis_current(0) : 0;
    s_can_tx_chassis_cmd[1] = allow_chassis ? actuator_cmd_get_chassis_current(1) : 0;
    s_can_tx_chassis_cmd[2] = allow_chassis ? actuator_cmd_get_chassis_current(2) : 0;
    s_can_tx_chassis_cmd[3] = allow_chassis ? actuator_cmd_get_chassis_current(3) : 0;
    s_can_tx_yaw_cmd = actuator_cmd_get_yaw_current();
    s_can_tx_yaw_upper_cmd = actuator_cmd_get_yaw_upper_current();
    s_can_tx_pitch_cmd = actuator_cmd_get_pitch_current();
    s_can_tx_trigger_cmd = actuator_cmd_get_trigger_current();
    s_can_tx_friction_cmd[0] = actuator_cmd_get_friction_current(0);
    s_can_tx_friction_cmd[1] = actuator_cmd_get_friction_current(1);
    s_can_tx_friction_cmd[2] = actuator_cmd_get_friction_current(2);
    s_can_tx_friction_cmd[3] = actuator_cmd_get_friction_current(3);

    can_tx_limit_friction_cmds();
}

static void can_tx_log_offline_cmds(void)
{
    sdlog_actuator_current_t log = {0};

    log.yaw = s_can_tx_yaw_cmd;
    log.friction[0] = s_can_tx_friction_cmd[0];
    log.friction[1] = s_can_tx_friction_cmd[1];
    log.friction[2] = s_can_tx_friction_cmd[2];
    log.friction[3] = s_can_tx_friction_cmd[3];
    can_tx_log_actuator_current(&log);
}

static void can_tx_log_online_cmds(void)
{
    sdlog_actuator_current_t log = {0};

    log.chassis[0] = s_can_tx_chassis_cmd[0];
    log.chassis[1] = s_can_tx_chassis_cmd[1];
    log.chassis[2] = s_can_tx_chassis_cmd[2];
    log.chassis[3] = s_can_tx_chassis_cmd[3];
    log.yaw = s_can_tx_yaw_cmd;
    log.pitch = s_can_tx_pitch_cmd;
    log.trigger = s_can_tx_trigger_cmd;
    log.friction[0] = s_can_tx_friction_cmd[0];
    log.friction[1] = s_can_tx_friction_cmd[1];
    log.friction[2] = s_can_tx_friction_cmd[2];
    log.friction[3] = s_can_tx_friction_cmd[3];
    can_tx_log_actuator_current(&log);
}

// 清空大疆组帧缓存，后续按 CAN ID 填进 0x200 或 0x1FF 的四个槽位。
static void can_tx_clear_rm_frames(void)
{
    (void)memset(s_can_tx_can1_200, 0, sizeof(s_can_tx_can1_200));
    (void)memset(s_can_tx_can1_1ff, 0, sizeof(s_can_tx_can1_1ff));
    (void)memset(s_can_tx_can2_200, 0, sizeof(s_can_tx_can2_200));
    (void)memset(s_can_tx_can2_1ff, 0, sizeof(s_can_tx_can2_1ff));
}

static inline void can_tx_store_rm_current(uint8_t fallback_bus,
                                           uint16_t can_id,
                                           int16_t current)
{
    int16_t *frame_200 = (fallback_bus == 1u) ? s_can_tx_can1_200 : s_can_tx_can2_200;
    int16_t *frame_1ff = (fallback_bus == 1u) ? s_can_tx_can1_1ff : s_can_tx_can2_1ff;

    if (can_id >= 0x201u && can_id <= 0x204u)
    {
        frame_200[can_id - 0x201u] = current;
    }
    else if (can_id >= 0x205u && can_id <= 0x208u)
    {
        frame_1ff[can_id - 0x205u] = current;
    }
}

static inline void can_tx_process_rs485_axis(actuator_id_e actuator_id,
                                             const motor_node_param_t *node,
                                             int16_t current)
{
    const uint8_t port = (node != NULL) ? node->rs485_port : 0u;

    if (can_tx_process_extra_item(port, actuator_id, node, current) == 0u)
    {
        watch_task_error(WATCH_TASK_CAN_COMMAND_TX);
    }
}

static inline void can_tx_process_mit_or_extra_axis(uint8_t fallback_bus,
                                                    actuator_id_e actuator_id,
                                                    const motor_node_param_t *node,
                                                    int16_t current)
{
    const uint8_t node_bus = can_tx_node_bus(fallback_bus, node);

    if (can_tx_process_can_mit_item(node_bus, actuator_id, node, current) == 0u &&
        can_tx_process_extra_item(node_bus, actuator_id, node, current) == 0u)
    {
        watch_task_error(WATCH_TASK_CAN_COMMAND_TX);
    }
}

static inline void can_tx_process_rm_axis(uint8_t fallback_bus,
                                          actuator_id_e actuator_id,
                                          const motor_node_param_t *node,
                                          uint16_t can_id,
                                          int16_t current)
{
    const int16_t rm_current = can_tx_build_rm_current_from_actuator(actuator_id, current);
    const int16_t limited_current = motor_cfg_limit_current_node(node, rm_current);

    can_tx_store_rm_current(fallback_bus, can_id, limited_current);
}

static inline void can_tx_process_axis(uint8_t fallback_bus,
                                       actuator_id_e actuator_id,
                                       const motor_node_param_t *node,
                                       uint8_t is_rm_group,
                                       uint16_t can_id,
                                       int16_t current)
{
    if (motor_cfg_transport(node) == MOTOR_TRANSPORT_RS485)
    {
        can_tx_process_rs485_axis(actuator_id, node, current);
    }
    else if (is_rm_group == 0u)
    {
        can_tx_process_mit_or_extra_axis(fallback_bus, actuator_id, node, current);
    }
    else
    {
        can_tx_process_rm_axis(fallback_bus, actuator_id, node, can_id, current);
    }
}

// 每个轴的发送函数保持无参数，宏只负责把轴配置取出来交给普通函数。
#define CAN_TX_EXEC_AXIS(fallback_bus_, actuator_id_, node_expr_, is_rm_expr_, can_id_expr_, current_expr_) \
    do                                                                                                    \
    {                                                                                                     \
        const uint8_t fallback_bus__ = (uint8_t)(fallback_bus_);                                           \
        const actuator_id_e actuator_id__ = (actuator_id_);                                                \
        const motor_node_param_t *node__ = (node_expr_);                                                   \
        const uint8_t is_rm_group__ = (uint8_t)(is_rm_expr_);                                              \
        const uint16_t can_id__ = (uint16_t)(can_id_expr_);                                                \
        const int16_t current__ = (int16_t)(current_expr_);                                                \
        can_tx_process_axis(fallback_bus__, actuator_id__, node__, is_rm_group__, can_id__, current__);    \
    } while (0)

static inline void can_tx_exec_chassis0(void)
{
    CAN_TX_EXEC_AXIS(CAN_TX_AXIS_CHASSIS0_FALLBACK_BUS,
                     CAN_TX_AXIS_CHASSIS0_ACTUATOR_ID,
                     CAN_TX_AXIS_CHASSIS0_NODE(),
                     CAN_TX_AXIS_CHASSIS0_IS_RM_GROUP(),
                     CAN_TX_AXIS_CHASSIS0_CAN_ID(),
                     s_can_tx_chassis_cmd[0]);
}

static inline void can_tx_exec_chassis1(void)
{
    CAN_TX_EXEC_AXIS(CAN_TX_AXIS_CHASSIS1_FALLBACK_BUS,
                     CAN_TX_AXIS_CHASSIS1_ACTUATOR_ID,
                     CAN_TX_AXIS_CHASSIS1_NODE(),
                     CAN_TX_AXIS_CHASSIS1_IS_RM_GROUP(),
                     CAN_TX_AXIS_CHASSIS1_CAN_ID(),
                     s_can_tx_chassis_cmd[1]);
}

static inline void can_tx_exec_chassis2(void)
{
    CAN_TX_EXEC_AXIS(CAN_TX_AXIS_CHASSIS2_FALLBACK_BUS,
                     CAN_TX_AXIS_CHASSIS2_ACTUATOR_ID,
                     CAN_TX_AXIS_CHASSIS2_NODE(),
                     CAN_TX_AXIS_CHASSIS2_IS_RM_GROUP(),
                     CAN_TX_AXIS_CHASSIS2_CAN_ID(),
                     s_can_tx_chassis_cmd[2]);
}

static inline void can_tx_exec_chassis3(void)
{
    CAN_TX_EXEC_AXIS(CAN_TX_AXIS_CHASSIS3_FALLBACK_BUS,
                     CAN_TX_AXIS_CHASSIS3_ACTUATOR_ID,
                     CAN_TX_AXIS_CHASSIS3_NODE(),
                     CAN_TX_AXIS_CHASSIS3_IS_RM_GROUP(),
                     CAN_TX_AXIS_CHASSIS3_CAN_ID(),
                     s_can_tx_chassis_cmd[3]);
}

static inline void can_tx_exec_yaw(void)
{
    CAN_TX_EXEC_AXIS(CAN_TX_AXIS_YAW_FALLBACK_BUS,
                     CAN_TX_AXIS_YAW_ACTUATOR_ID,
                     CAN_TX_AXIS_YAW_NODE(),
                     CAN_TX_AXIS_YAW_IS_RM_GROUP(),
                     CAN_TX_AXIS_YAW_CAN_ID(),
                     s_can_tx_yaw_cmd);
}

static inline void can_tx_exec_yaw_upper(void)
{
    CAN_TX_EXEC_AXIS(CAN_TX_AXIS_YAW_UPPER_FALLBACK_BUS,
                     CAN_TX_AXIS_YAW_UPPER_ACTUATOR_ID,
                     CAN_TX_AXIS_YAW_UPPER_NODE(),
                     CAN_TX_AXIS_YAW_UPPER_IS_RM_GROUP(),
                     CAN_TX_AXIS_YAW_UPPER_CAN_ID(),
                     s_can_tx_yaw_upper_cmd);
}

static inline void can_tx_exec_pitch(void)
{
    CAN_TX_EXEC_AXIS(CAN_TX_AXIS_PITCH_FALLBACK_BUS,
                     CAN_TX_AXIS_PITCH_ACTUATOR_ID,
                     CAN_TX_AXIS_PITCH_NODE(),
                     CAN_TX_AXIS_PITCH_IS_RM_GROUP(),
                     CAN_TX_AXIS_PITCH_CAN_ID(),
                     s_can_tx_pitch_cmd);
}

static inline void can_tx_exec_trigger(void)
{
    CAN_TX_EXEC_AXIS(CAN_TX_AXIS_TRIGGER_FALLBACK_BUS,
                     CAN_TX_AXIS_TRIGGER_ACTUATOR_ID,
                     CAN_TX_AXIS_TRIGGER_NODE(),
                     CAN_TX_AXIS_TRIGGER_IS_RM_GROUP(),
                     CAN_TX_AXIS_TRIGGER_CAN_ID(),
                     s_can_tx_trigger_cmd);
}

static inline void can_tx_exec_friction0(void)
{
    CAN_TX_EXEC_AXIS(CAN_TX_AXIS_FRICTION0_FALLBACK_BUS,
                     CAN_TX_AXIS_FRICTION0_ACTUATOR_ID,
                     CAN_TX_AXIS_FRICTION0_NODE(),
                     CAN_TX_AXIS_FRICTION0_IS_RM_GROUP(),
                     CAN_TX_AXIS_FRICTION0_CAN_ID(),
                     s_can_tx_friction_cmd[0]);
}

static inline void can_tx_exec_friction1(void)
{
    CAN_TX_EXEC_AXIS(CAN_TX_AXIS_FRICTION1_FALLBACK_BUS,
                     CAN_TX_AXIS_FRICTION1_ACTUATOR_ID,
                     CAN_TX_AXIS_FRICTION1_NODE(),
                     CAN_TX_AXIS_FRICTION1_IS_RM_GROUP(),
                     CAN_TX_AXIS_FRICTION1_CAN_ID(),
                     s_can_tx_friction_cmd[1]);
}

static inline void can_tx_exec_friction2(void)
{
    CAN_TX_EXEC_AXIS(CAN_TX_AXIS_FRICTION2_FALLBACK_BUS,
                     CAN_TX_AXIS_FRICTION2_ACTUATOR_ID,
                     CAN_TX_AXIS_FRICTION2_NODE(),
                     CAN_TX_AXIS_FRICTION2_IS_RM_GROUP(),
                     CAN_TX_AXIS_FRICTION2_CAN_ID(),
                     s_can_tx_friction_cmd[2]);
}

static inline void can_tx_exec_friction3(void)
{
    CAN_TX_EXEC_AXIS(CAN_TX_AXIS_FRICTION3_FALLBACK_BUS,
                     CAN_TX_AXIS_FRICTION3_ACTUATOR_ID,
                     CAN_TX_AXIS_FRICTION3_NODE(),
                     CAN_TX_AXIS_FRICTION3_IS_RM_GROUP(),
                     CAN_TX_AXIS_FRICTION3_CAN_ID(),
                     s_can_tx_friction_cmd[3]);
}

static inline void can_tx_exec_arm0(void)
{
    const motor_node_param_t *node = CAN_TX_AXIS_ARM_NODE(0u);
    const int16_t current = actuator_cmd_get_current(ACTUATOR_ID_ARM_J0);

    if (current == 0 || motor_cfg_transport(node) != MOTOR_TRANSPORT_CAN)
    {
        return;
    }

    CAN_TX_EXEC_AXIS(CAN_TX_AXIS_ARM_FALLBACK_BUS(0u),
                     CAN_TX_AXIS_ARM_ACTUATOR_ID(0u),
                     node,
                     CAN_TX_AXIS_ARM_IS_RM_GROUP(0u),
                     CAN_TX_AXIS_ARM_CAN_ID(0u),
                     current);
}

// 离线分支只执行被允许的轴，其他轴保持 0 输出。
static void can_tx_exec_offline_axes(void)
{
    can_tx_clear_rm_frames();
    can_tx_exec_yaw();
    can_tx_exec_arm0();
    can_tx_exec_friction0();
    can_tx_exec_friction1();
    can_tx_exec_friction2();
    can_tx_exec_friction3();
}

// 在线分支执行全部已配置轴；每个轴内部再决定走大疆、MIT 或扩展处理。
static void can_tx_exec_online_axes(void)
{
    can_tx_clear_rm_frames();
    can_tx_exec_chassis0();
    can_tx_exec_chassis1();
    can_tx_exec_chassis2();
    can_tx_exec_chassis3();
    can_tx_exec_yaw();
    can_tx_exec_yaw_upper();
    can_tx_exec_pitch();
    can_tx_exec_trigger();
    can_tx_exec_arm0();
    can_tx_exec_friction0();
    can_tx_exec_friction1();
    can_tx_exec_friction2();
    can_tx_exec_friction3();
}

// 把已经缓存好的大疆 0x200/0x1FF 四电机电流帧一次性发出去。
static void can_tx_emit_rm_frames(void)
{
    CAN_cmd_rm_group(1u,
                     (uint16_t)CAN_RM_GROUP_0X200_ID,
                     s_can_tx_can1_200[0],
                     s_can_tx_can1_200[1],
                     s_can_tx_can1_200[2],
                     s_can_tx_can1_200[3]);
    CAN_cmd_rm_group(1u,
                     (uint16_t)CAN_RM_GROUP_0X1FF_ID,
                     s_can_tx_can1_1ff[0],
                     s_can_tx_can1_1ff[1],
                     s_can_tx_can1_1ff[2],
                     s_can_tx_can1_1ff[3]);
    CAN_cmd_rm_group(2u,
                     (uint16_t)CAN_RM_GROUP_0X200_ID,
                     s_can_tx_can2_200[0],
                     s_can_tx_can2_200[1],
                     s_can_tx_can2_200[2],
                     s_can_tx_can2_200[3]);
    CAN_cmd_rm_group(2u,
                     (uint16_t)CAN_RM_GROUP_0X1FF_ID,
                     s_can_tx_can2_1ff[0],
                     s_can_tx_can2_1ff[1],
                     s_can_tx_can2_1ff[2],
                     s_can_tx_can2_1ff[3]);
}

// 目标工程可在这里接入非大疆、非 MIT 的特殊电机发送逻辑。
__weak uint8_t can_tx_process_extra_item(uint8_t bus,
                                         actuator_id_e actuator_id,
                                         const motor_node_param_t *node,
                                         int16_t current)
{
    (void)bus;
    (void)actuator_id;
    (void)node;
    (void)current;
    return 0u;
}

__weak void can_mit_motor_send_cmd(uint8_t bus,
                                   uint16_t std_id,
                                   const can_mit_motor_limits_t *limits,
                                   const can_mit_motor_cmd_t *cmd)
{
    (void)bus;
    (void)std_id;
    (void)limits;
    (void)cmd;
}

__weak void can_mit_motor_send_enable(uint8_t bus, uint16_t std_id)
{
    (void)bus;
    (void)std_id;
}

__weak void can_mit_motor_send_stop(uint8_t bus,
                                    uint16_t std_id,
                                    const can_mit_motor_limits_t *limits)
{
    (void)bus;
    (void)std_id;
    (void)limits;
}

__weak uint8_t can_mit_motor_update_feedback(uint16_t std_id,
                                             uint8_t motor_id,
                                             const can_mit_motor_limits_t *limits,
                                             uint8_t dlc,
                                             const uint8_t data[8],
                                             can_mit_motor_feedback_t *feedback)
{
    (void)std_id;
    (void)motor_id;
    (void)limits;
    (void)dlc;
    (void)data;
    (void)feedback;
    return 0u;
}

// CAN 命令发送任务：收集各轴命令，按轴装配表转换协议并统一发出。
void can_command_tx_task(void const *pvParameters)
{
    (void)pvParameters;

    TickType_t last_wake = xTaskGetTickCount();

    while (1)
    {
        const uint64_t loop_start_us = rt_profiler_begin();
        watch_task_beat(WATCH_TASK_CAN_COMMAND_TX);
        const uint16_t period_ms = robot_profile_can_command_tx_period_ms();
        const bool_t dbus_offline = toe_is_error(DBUS_TOE);

        if (dbus_offline)
        {
            can_tx_collect_offline_cmds();
            can_tx_log_offline_cmds();
            can_tx_exec_offline_axes();
            can_tx_emit_rm_frames();
        }
        else
        {
            can_tx_collect_online_cmds();
            can_tx_log_online_cmds();
            can_tx_exec_online_axes();
            can_tx_emit_rm_frames();
        }

        rt_profiler_end(RT_PROFILER_CAN_COMMAND_TX_LOOP, loop_start_us);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(period_ms));
    }
}
