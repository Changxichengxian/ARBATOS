/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "RobotConfig.h"
#include "LowCmd.h"

/*
 * 这份表现在分两类：
 * 1. 稳定配置：只在代码里改，不给 AUX 参数号。
 * 2. 动态配置：只保留需要临时试的量，注释里带 [ID]。
 */

/*
 * AUX 口临时调参：发送 "<id>:<value>"，例如 "1:1000"。
 * - 带 [ID] 的注释，表示还能通过 AUX 口临时改。
 * - 不带 [ID] 的注释，表示稳定配置，只能改代码默认值。
 * - AUX 口只改 RAM 里的 g_config，重启后会回到这里的默认值。
 */

#if defined(__CC_ARM) || defined(__ARMCC_VERSION)
#pragma push
#pragma diag_suppress 188
#endif
#if defined(__GNUC__) && !defined(__CC_ARM)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-braces"
#endif
Config g_config = {
#include "ConfigOperation.inc"
#include "ConfigHardware.inc"
#include "ConfigTuning.inc"
#include "ConfigInput.inc"
#include "ConfigDiagnostics.inc"
};
#if defined(__GNUC__) && !defined(__CC_ARM)
#pragma GCC diagnostic pop
#endif
#if defined(__CC_ARM) || defined(__ARMCC_VERSION)
#pragma pop
#endif

// 这些函数只判断某个参数块当前是否有效，不负责修改配置内容。
static uint8_t ConfigBlockActiveAlways(void)
{
    return 1u;
}

static uint8_t ConfigProfileModuleEnabled(RobotTaskModule module)
{
    const uint8_t count = g_config.profile.task_module_count;
    const uint8_t limit = (count > ROBOT_TASK_MODULE_MAX) ? ROBOT_TASK_MODULE_MAX : count;

    for (uint8_t i = 0u; i < limit; i++)
    {
        if ((RobotTaskModule)g_config.profile.task_modules[i] == module)
        {
            return 1u;
        }
    }

    return 0u;
}

static uint8_t ConfigBlockActiveGimbalSingle(void)
{
    return ConfigProfileModuleEnabled(ROBOT_TASK_MODULE_SINGLE_GIMBAL);
}

static uint8_t ConfigBlockActiveGimbalDual(void)
{
    return ConfigProfileModuleEnabled(ROBOT_TASK_MODULE_DUAL_YAW_GIMBAL);
}

static uint8_t ConfigBlockActiveLocomotionClassic(void)
{
    return ConfigProfileModuleEnabled(ROBOT_TASK_MODULE_CLASSIC_CHASSIS);
}

static uint8_t ConfigBlockActiveWheelLegServo(void)
{
    return ConfigProfileModuleEnabled(ROBOT_TASK_MODULE_WHEELLEG_SERVO);
}

static uint8_t ConfigBlockActiveWheelLegMit(void)
{
    return ConfigProfileModuleEnabled(ROBOT_TASK_MODULE_WHEELLEG_MIT);
}

static uint8_t ConfigBlockActiveShootRm(void)
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

static uint8_t ConfigBlockActiveArm(void)
{
    return ConfigProfileModuleEnabled(ROBOT_TASK_MODULE_ARM);
}

// AUX 调参表：只有列在这里的块能被运行时改，g_config.motor 这种装配信息不放进来。
static const ConfigBlockDesc g_config_blocks[] = {
    {CONFIG_BLOCK_PROFILE, "profile", "", &g_config.profile, sizeof(g_config.profile), ConfigBlockActiveAlways},
    {CONFIG_BLOCK_GIMBAL_SINGLE, "gimbal.single", "001-022,026-064,350-368", &g_config.gimbal, sizeof(g_config.gimbal), ConfigBlockActiveGimbalSingle},
    {CONFIG_BLOCK_GIMBAL_DUAL, "gimbal.dual", "400-499", &g_config.dual_gimbal, sizeof(g_config.dual_gimbal), ConfigBlockActiveGimbalDual},
    {CONFIG_BLOCK_LOCOMOTION_CLASSIC, "locomotion.classic", "066-075,081,086-092,098-106,245-248", &g_config.chassis, sizeof(g_config.chassis), ConfigBlockActiveLocomotionClassic},
    {CONFIG_BLOCK_LOCOMOTION_WHEELLEG_SERVO, "locomotion.WheelLegServo", "500-599", &g_config.WheelLegServo, sizeof(g_config.WheelLegServo), ConfigBlockActiveWheelLegServo},
    {CONFIG_BLOCK_LOCOMOTION_WHEELLEG_MIT, "locomotion.WheelLegMit", "600-699", &g_config.WheelLegMit, sizeof(g_config.WheelLegMit), ConfigBlockActiveWheelLegMit},
    {CONFIG_BLOCK_SHOOT_RM, "shoot.rm", "113-121,127,130-133,139-142,145-160", &g_config.shoot, sizeof(g_config.shoot), ConfigBlockActiveShootRm},
    {CONFIG_BLOCK_ARM_J0_UNITREE, "arm.j0_unitree", "800-803", &g_config.ArmJ0Unitree, sizeof(g_config.ArmJ0Unitree), ConfigBlockActiveArm},
    {CONFIG_BLOCK_COMMON_POWER, "common.power", "161-166", &g_config.power, sizeof(g_config.power), ConfigBlockActiveAlways},
    {CONFIG_BLOCK_COMMON_DETECT, "common.detect", "167-208,211", &g_config.detect, sizeof(g_config.detect), ConfigBlockActiveAlways},
    {CONFIG_BLOCK_COMMON_IMU, "common.imu", "218-219", &g_config.imu, sizeof(g_config.imu), ConfigBlockActiveAlways},
    {CONFIG_BLOCK_COMMON_VOLTAGE, "common.voltage", "221-223", &g_config.voltage, sizeof(g_config.voltage), ConfigBlockActiveAlways},
    {CONFIG_BLOCK_COMMON_BUZZER, "common.buzzer", "224-237", &g_config.buzzer, sizeof(g_config.buzzer), ConfigBlockActiveAlways},
    {CONFIG_BLOCK_COMMON_LED, "common.led", "238-240", &g_config.led, sizeof(g_config.led), ConfigBlockActiveAlways},
    {CONFIG_BLOCK_COMMON_MANUAL_INPUT, "common.manual_input", "300-309,317-318,369-378", &g_config.manual_input, sizeof(g_config.manual_input), ConfigBlockActiveAlways},
    {CONFIG_BLOCK_COMMON_INPUT, "common.input", "310-316,320-347", &g_config.input, sizeof(g_config.input), ConfigBlockActiveAlways},
    {CONFIG_BLOCK_COMMON_AUX_TELEM, "common.AuxTelem", "241-243", &g_config.AuxTelem, sizeof(g_config.AuxTelem), ConfigBlockActiveAlways},
    {CONFIG_BLOCK_COMMON_OPERATION, "common.operation", "244,250-253", &g_config.operation, sizeof(g_config.operation), ConfigBlockActiveAlways},
    {CONFIG_BLOCK_COMMON_SDLOG, "common.sdlog", "249", &g_config.sdlog, sizeof(g_config.sdlog), ConfigBlockActiveAlways},
};

// 返回调参块表，同时可选返回块数量。
const ConfigBlockDesc *ConfigGetBlockTable(uint32_t *count)
{
    if (count != NULL)
    {
        *count = (uint32_t)(sizeof(g_config_blocks) / sizeof(g_config_blocks[0]));
    }
    return g_config_blocks;
}

// 按块 ID 查找调参块描述；找不到返回 NULL。
const ConfigBlockDesc *ConfigFindBlock(ConfigBlockId id)
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
uint8_t ConfigBlockIsActive(ConfigBlockId id)
{
    const ConfigBlockDesc *block = ConfigFindBlock(id);
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
