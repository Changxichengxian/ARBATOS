/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "config.h"
#include "LowCmd.h"

/*
 * AUX 口临时改参：发送 "<id>:<value>"（例如 "1:1000"）
 * - 编号见本文件 g_config 初始化处每行末尾的 [ID] 注释
 * - 未标 [ID] 的参数：只在 init 使用 / 未运行时应用（AUX 口不支持改）
 * - 仅修改 RAM 中的 g_config，重启后恢复默认值
 */

config_t g_config = {
#include "config_operation.inc"
#include "config_hardware.inc"
#include "config_tuning.inc"
#include "config_input.inc"
#include "config_diagnostics.inc"
};

// 这些函数只判断某个参数块当前是否有效，不负责修改配置内容。
static uint8_t config_block_active_always(void)
{
    return 1u;
}

static uint8_t config_profile_module_enabled(robot_task_module_e module)
{
    const uint8_t count = g_config.profile.task_module_count;
    const uint8_t limit = (count > ROBOT_TASK_MODULE_MAX) ? ROBOT_TASK_MODULE_MAX : count;

    for (uint8_t i = 0u; i < limit; i++)
    {
        if ((robot_task_module_e)g_config.profile.task_modules[i] == module)
        {
            return 1u;
        }
    }

    return 0u;
}

static uint8_t config_block_active_gimbal_single(void)
{
    return config_profile_module_enabled(ROBOT_TASK_MODULE_SINGLE_GIMBAL);
}

static uint8_t config_block_active_gimbal_dual(void)
{
    return config_profile_module_enabled(ROBOT_TASK_MODULE_DUAL_YAW_GIMBAL);
}

static uint8_t config_block_active_locomotion_classic(void)
{
    return config_profile_module_enabled(ROBOT_TASK_MODULE_CLASSIC_CHASSIS);
}

static uint8_t config_block_active_wheelleg_servo(void)
{
    return config_profile_module_enabled(ROBOT_TASK_MODULE_WHEELLEG_SERVO);
}

static uint8_t config_block_active_wheelleg_mit(void)
{
    return config_profile_module_enabled(ROBOT_TASK_MODULE_WHEELLEG_MIT);
}

static uint8_t config_block_active_shoot_rm(void)
{
    if (g_config.motor.trigger.can_id != 0u)
    {
        return 1u;
    }

    for (uint8_t i = 0u; i < 4u; i++)
    {
        if (g_config.motor.friction[i].can_id != 0u)
        {
            return 1u;
        }
    }

    return 0u;
}

static uint8_t config_block_active_arm(void)
{
    return config_profile_module_enabled(ROBOT_TASK_MODULE_ARM);
}

// AUX 调参表：只有列在这里的块能被运行时改，g_config.motor 这种装配信息不放进来。
static const config_block_desc_t g_config_blocks[] = {
    {CONFIG_BLOCK_PROFILE, "profile", "", &g_config.profile, sizeof(g_config.profile), config_block_active_always},
    {CONFIG_BLOCK_GIMBAL_SINGLE, "gimbal.single", "001-022,026-064,350-368", &g_config.gimbal, sizeof(g_config.gimbal), config_block_active_gimbal_single},
    {CONFIG_BLOCK_GIMBAL_DUAL, "gimbal.dual", "400-499", &g_config.dual_gimbal, sizeof(g_config.dual_gimbal), config_block_active_gimbal_dual},
    {CONFIG_BLOCK_LOCOMOTION_CLASSIC, "locomotion.classic", "066-075,081,086-092,098-106,245-248", &g_config.chassis, sizeof(g_config.chassis), config_block_active_locomotion_classic},
    {CONFIG_BLOCK_LOCOMOTION_WHEELLEG_SERVO, "locomotion.wheelleg_servo", "500-599", &g_config.wheelleg_servo, sizeof(g_config.wheelleg_servo), config_block_active_wheelleg_servo},
    {CONFIG_BLOCK_LOCOMOTION_WHEELLEG_MIT, "locomotion.wheelleg_mit", "600-699", &g_config.wheelleg_mit, sizeof(g_config.wheelleg_mit), config_block_active_wheelleg_mit},
    {CONFIG_BLOCK_SHOOT_RM, "shoot.rm", "113-121,127,130-133,139-142,145-160", &g_config.shoot, sizeof(g_config.shoot), config_block_active_shoot_rm},
    {CONFIG_BLOCK_ARM_J0_UNITREE, "arm.j0_unitree", "800-803", &g_config.arm_j0_unitree, sizeof(g_config.arm_j0_unitree), config_block_active_arm},
    {CONFIG_BLOCK_COMMON_POWER, "common.power", "161-166", &g_config.power, sizeof(g_config.power), config_block_active_always},
    {CONFIG_BLOCK_COMMON_DETECT, "common.detect", "167-208,211", &g_config.detect, sizeof(g_config.detect), config_block_active_always},
    {CONFIG_BLOCK_COMMON_IMU, "common.imu", "218-219", &g_config.imu, sizeof(g_config.imu), config_block_active_always},
    {CONFIG_BLOCK_COMMON_VOLTAGE, "common.voltage", "221-223", &g_config.voltage, sizeof(g_config.voltage), config_block_active_always},
    {CONFIG_BLOCK_COMMON_BUZZER, "common.buzzer", "224-237", &g_config.buzzer, sizeof(g_config.buzzer), config_block_active_always},
    {CONFIG_BLOCK_COMMON_LED, "common.led", "238-240", &g_config.led, sizeof(g_config.led), config_block_active_always},
    {CONFIG_BLOCK_COMMON_MANUAL_INPUT, "common.manual_input", "300-309,317-318,369-378", &g_config.manual_input, sizeof(g_config.manual_input), config_block_active_always},
    {CONFIG_BLOCK_COMMON_INPUT, "common.input", "310-316,320-347", &g_config.input, sizeof(g_config.input), config_block_active_always},
    {CONFIG_BLOCK_COMMON_AUX_TELEM, "common.aux_telem", "241-243", &g_config.aux_telem, sizeof(g_config.aux_telem), config_block_active_always},
    {CONFIG_BLOCK_COMMON_OPERATION, "common.operation", "244,250-253", &g_config.operation, sizeof(g_config.operation), config_block_active_always},
    {CONFIG_BLOCK_COMMON_SDLOG, "common.sdlog", "249", &g_config.sdlog, sizeof(g_config.sdlog), config_block_active_always},
};

// 返回调参块表，同时可选返回块数量。
const config_block_desc_t *config_get_block_table(uint32_t *count)
{
    if (count != NULL)
    {
        *count = (uint32_t)(sizeof(g_config_blocks) / sizeof(g_config_blocks[0]));
    }
    return g_config_blocks;
}

// 按块 ID 查找调参块描述；找不到返回 NULL。
const config_block_desc_t *config_find_block(config_block_id_e id)
{
    for (uint32_t i = 0u; i < (uint32_t)(sizeof(g_config_blocks) / sizeof(g_config_blocks[0])); i++)
    {
        if (g_config_blocks[i].id == id)
        {
            return &g_config_blocks[i];
        }
    }
    return NULL;
}

// 对外判断某个调参块当前是否启用。
uint8_t config_block_is_active(config_block_id_e id)
{
    const config_block_desc_t *block = config_find_block(id);
    if (block == NULL)
    {
        return 0u;
    }
    if (block->is_active == NULL)
    {
        return 1u;
    }
    return block->is_active();
}
