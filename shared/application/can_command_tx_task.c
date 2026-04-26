/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "can_command_tx_task.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"

#include "CAN_receive.h"
#include "actuator_cmd.h"
#include "config.h"
#include "watch.h"
#include "detect_task.h"
#include "motor_config.h"
#include "sdlog.h"
#include "rt_profiler.h"
#include "robot_task_profile.h"

#include <string.h>

typedef struct
{
    const motor_node_param_t *node;
    int16_t current;
} can_tx_rm_item_t;

__weak uint8_t can_tx_allow_can1_yaw_override(void);
__weak uint8_t can_tx_process_extra_item(uint8_t bus, const motor_node_param_t *node, int16_t current);

static bool_t can_tx_allow_chassis(void)
{
    const test_mode_e mode = (test_mode_e)g_config.test.mode;
    return (mode == TEST_MODE_NONE) || (mode == TEST_MODE_ENTERTAIN) || (mode == TEST_MODE_CHASSIS_ONLY);
}

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

static void can_tx_log_actuator_current(const sdlog_actuator_current_t *log)
{
    if (log != NULL && can_tx_log_due() != 0u)
    {
        sdlog_write(SDLOG_TAG_ACTUATOR_CURRENT, log, (uint16_t)sizeof(*log));
    }
}

static void can_tx_limit_friction_currents(int16_t fric[4])
{
    for (uint8_t i = 0u; i < 4u; i++)
    {
        if (g_config.shoot.fric_motor_dir[i] == 0)
        {
            fric[i] = 0;
        }
    }
}

static void can_tx_pack_rm_frames(uint8_t bus,
                                  const can_tx_rm_item_t *items,
                                  uint32_t count,
                                  int16_t out_200[4],
                                  int16_t out_1ff[4])
{
    (void)memset(out_200, 0, sizeof(int16_t) * 4u);
    (void)memset(out_1ff, 0, sizeof(int16_t) * 4u);

    if (items == NULL)
    {
        return;
    }

    for (uint32_t i = 0u; i < count; i++)
    {
        if (motor_cfg_is_rm_group_protocol(items[i].node) == 0u)
        {
            if (can_tx_process_extra_item(bus, items[i].node, items[i].current) == 0u)
            {
                watch_task_error(WATCH_TASK_CAN_COMMAND_TX);
            }
            continue;
        }
        const uint16_t can_id = motor_cfg_can_id(items[i].node);
        if (can_id >= 0x201u && can_id <= 0x204u)
        {
            out_200[can_id - 0x201u] = items[i].current;
        }
        else if (can_id >= 0x205u && can_id <= 0x208u)
        {
            out_1ff[can_id - 0x205u] = items[i].current;
        }
    }
}

__weak uint8_t can_tx_allow_can1_yaw_override(void)
{
    return 0u;
}

__weak uint8_t can_tx_process_extra_item(uint8_t bus, const motor_node_param_t *node, int16_t current)
{
    (void)bus;
    (void)node;
    (void)current;
    return 0u;
}

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
        const test_mode_e mode = (test_mode_e)g_config.test.mode;
        const bool_t allow_friction_offline = (mode == TEST_MODE_ENTERTAIN);

        if (dbus_offline)
        {
            int16_t fric[4] = {0};
            int16_t yaw = 0;
            if (allow_friction_offline)
            {
                taskENTER_CRITICAL();
                fric[0] = actuator_cmd_get_friction_current_can2(0);
                fric[1] = actuator_cmd_get_friction_current_can2(1);
                fric[2] = actuator_cmd_get_friction_current_can2(2);
                fric[3] = actuator_cmd_get_friction_current_can2(3);
                taskEXIT_CRITICAL();
            }
            if (can_tx_allow_can1_yaw_override() != 0u)
            {
                yaw = actuator_cmd_get_yaw_current_can1();
            }

            can_tx_limit_friction_currents(fric);

            sdlog_actuator_current_t log = {0};
            log.yaw = yaw;
            log.friction[0] = fric[0];
            log.friction[1] = fric[1];
            log.friction[2] = fric[2];
            log.friction[3] = fric[3];
            can_tx_log_actuator_current(&log);

            CAN_cmd_rm_group(1u, (uint16_t)CAN_CHASSIS_ALL_ID, 0, 0, 0, 0);
            {
                const can_tx_rm_item_t can1_items[] = {
                    {&g_config.motor.yaw, yaw},
                };
                int16_t can1_200[4] = {0};
                int16_t can1_1ff[4] = {0};

                can_tx_pack_rm_frames(1u,
                                      can1_items,
                                      (uint32_t)(sizeof(can1_items) / sizeof(can1_items[0])),
                                      can1_200,
                                      can1_1ff);
                CAN_cmd_rm_group(1u, (uint16_t)CAN_GIMBAL_ALL_ID, can1_1ff[0], can1_1ff[1], can1_1ff[2], can1_1ff[3]);
            }
            CAN_cmd_rm_group(2u, (uint16_t)CAN_CHASSIS_ALL_ID, fric[0], fric[1], fric[2], fric[3]);
            CAN_cmd_rm_group(2u, (uint16_t)CAN_GIMBAL_ALL_ID, 0, 0, 0, 0);
        }
        else
        {
            const bool_t allow_chassis = can_tx_allow_chassis();

            int16_t chassis[4] = {0};
            int16_t yaw = 0;
            int16_t yaw_upper = 0;
            int16_t pitch = 0;
            int16_t trigger = 0;
            int16_t fric[4] = {0};

            taskENTER_CRITICAL();
            chassis[0] = allow_chassis ? actuator_cmd_get_chassis_current_can1(0) : 0;
            chassis[1] = allow_chassis ? actuator_cmd_get_chassis_current_can1(1) : 0;
            chassis[2] = allow_chassis ? actuator_cmd_get_chassis_current_can1(2) : 0;
            chassis[3] = allow_chassis ? actuator_cmd_get_chassis_current_can1(3) : 0;
            yaw = actuator_cmd_get_yaw_current_can1();
            yaw_upper = actuator_cmd_get_yaw_upper_current_can1();
            pitch = actuator_cmd_get_pitch_current_can1();
            trigger = actuator_cmd_get_trigger_current_can1();
            fric[0] = actuator_cmd_get_friction_current_can2(0);
            fric[1] = actuator_cmd_get_friction_current_can2(1);
            fric[2] = actuator_cmd_get_friction_current_can2(2);
            fric[3] = actuator_cmd_get_friction_current_can2(3);
            taskEXIT_CRITICAL();

            can_tx_limit_friction_currents(fric);

            sdlog_actuator_current_t log = {0};
            log.chassis[0] = chassis[0];
            log.chassis[1] = chassis[1];
            log.chassis[2] = chassis[2];
            log.chassis[3] = chassis[3];
            log.yaw = yaw;
            log.pitch = pitch;
            log.trigger = trigger;
            log.friction[0] = fric[0];
            log.friction[1] = fric[1];
            log.friction[2] = fric[2];
            log.friction[3] = fric[3];
            can_tx_log_actuator_current(&log);

            {
                const can_tx_rm_item_t can1_items[] = {
                    {&g_config.motor.chassis[0], chassis[0]},
                    {&g_config.motor.chassis[1], chassis[1]},
                    {&g_config.motor.chassis[2], chassis[2]},
                    {&g_config.motor.chassis[3], chassis[3]},
                    {&g_config.motor.yaw, yaw},
                    {&g_config.motor.yaw_upper, yaw_upper},
                    {&g_config.motor.pitch, pitch},
                    {&g_config.motor.trigger, trigger},
                };
                const can_tx_rm_item_t can2_items[] = {
                    {&g_config.motor.friction[0], fric[0]},
                    {&g_config.motor.friction[1], fric[1]},
                    {&g_config.motor.friction[2], fric[2]},
                    {&g_config.motor.friction[3], fric[3]},
                };

                int16_t can1_200[4] = {0};
                int16_t can1_1ff[4] = {0};
                int16_t can2_200[4] = {0};
                int16_t can2_1ff[4] = {0};

                can_tx_pack_rm_frames(1u,
                                      can1_items,
                                      (uint32_t)(sizeof(can1_items) / sizeof(can1_items[0])),
                                      can1_200,
                                      can1_1ff);
                can_tx_pack_rm_frames(2u,
                                      can2_items,
                                      (uint32_t)(sizeof(can2_items) / sizeof(can2_items[0])),
                                      can2_200,
                                      can2_1ff);

                CAN_cmd_rm_group(1u, (uint16_t)CAN_CHASSIS_ALL_ID, can1_200[0], can1_200[1], can1_200[2], can1_200[3]);
                CAN_cmd_rm_group(1u, (uint16_t)CAN_GIMBAL_ALL_ID, can1_1ff[0], can1_1ff[1], can1_1ff[2], can1_1ff[3]);
                CAN_cmd_rm_group(2u, (uint16_t)CAN_CHASSIS_ALL_ID, can2_200[0], can2_200[1], can2_200[2], can2_200[3]);
                CAN_cmd_rm_group(2u, (uint16_t)CAN_GIMBAL_ALL_ID, can2_1ff[0], can2_1ff[1], can2_1ff[2], can2_1ff[3]);
            }
        }

        rt_profiler_end(RT_PROFILER_CAN_COMMAND_TX_LOOP, loop_start_us);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(period_ms));
    }
}
