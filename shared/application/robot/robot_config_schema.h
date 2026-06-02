/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ROBOT_CONFIG_SCHEMA_H
#define ROBOT_CONFIG_SCHEMA_H

#include <stddef.h>
#include <stdint.h>

#include "LowCmd.h"

#ifndef ROBOT_TASK_MODULE_MAX
#define ROBOT_TASK_MODULE_MAX 16u
#endif

typedef enum
{
    ROBOT_TASK_MODULE_NONE = 0u,
    ROBOT_TASK_MODULE_RC_SBUS = 1u,
    ROBOT_TASK_MODULE_HEALTH_MONITOR = 2u,
    ROBOT_TASK_MODULE_SDLOG = 3u,
    ROBOT_TASK_MODULE_CAN_COMMAND_TX = 4u,
    ROBOT_TASK_MODULE_CAN_FEEDBACK_RX = 5u,
    ROBOT_TASK_MODULE_CLASSIC_CHASSIS = 6u,
    ROBOT_TASK_MODULE_WHEELLEG_SERVO = 7u,
    ROBOT_TASK_MODULE_WHEELLEG_MIT = 8u,
    ROBOT_TASK_MODULE_SINGLE_GIMBAL = 9u,
    ROBOT_TASK_MODULE_DUAL_YAW_GIMBAL = 10u,
    ROBOT_TASK_MODULE_ARM = 11u,
    ROBOT_TASK_MODULE_IMU = 12u,
    ROBOT_TASK_MODULE_HOST_LINK = 13u,
    ROBOT_TASK_MODULE_ELRS_LINK = 14u,
    ROBOT_TASK_MODULE_REFEREE_RX = 15u,
    ROBOT_TASK_MODULE_BATTERY_MONITOR = 16u,
    ROBOT_TASK_MODULE_SERVO = 17u,
    ROBOT_TASK_MODULE_CALIBRATION = 18u,
    ROBOT_TASK_MODULE_STATUS_LED = 19u,
    ROBOT_TASK_MODULE_STARTUP_SERVICE = 20u,
} robot_task_module_e;

typedef struct
{
    uint8_t task_module_count;
    uint8_t task_modules[ROBOT_TASK_MODULE_MAX];
} task_profile_t;

#ifndef MOTOR_ARM_JOINT_COUNT
#define MOTOR_ARM_JOINT_COUNT 0u
#endif

#ifndef ROBOT_DEFAULT_DEVICE_EXTRA_COUNT
#define ROBOT_DEFAULT_DEVICE_EXTRA_COUNT 5u
#endif

#define ROBOT_DEFAULT_MOTOR_DEVICE_COUNT (12u + MOTOR_ARM_JOINT_COUNT)
#define ROBOT_DEFAULT_DEVICE_TABLE_COUNT (ROBOT_DEFAULT_MOTOR_DEVICE_COUNT + ROBOT_DEFAULT_DEVICE_EXTRA_COUNT)

#ifndef ROBOT_DEVICE_CONFIG_MAX
#define ROBOT_DEVICE_CONFIG_MAX ROBOT_DEFAULT_DEVICE_TABLE_COUNT
#endif

#define ROBOT_DEVICE_TABLE_KIND_MOTOR 1u
#define ROBOT_DEVICE_TABLE_KIND_SENSOR 2u
#define ROBOT_DEVICE_TABLE_KIND_INPUT 3u
#define ROBOT_DEVICE_TABLE_KIND_COMM 4u
#define ROBOT_DEVICE_TABLE_KIND_SERVICE 5u
#define ROBOT_DEVICE_TABLE_KIND_CUSTOM_BASE 128u

#define ROBOT_DEVICE_ROLE_NONE 0u
#define ROBOT_DEVICE_ROLE_IMU 1u
#define ROBOT_DEVICE_ROLE_MANUAL_INPUT 2u
#define ROBOT_DEVICE_ROLE_BATTERY 3u
#define ROBOT_DEVICE_ROLE_AUX_TELEM 4u
#define ROBOT_DEVICE_ROLE_SDLOG 5u
#define ROBOT_DEVICE_ROLE_CUSTOM_BASE 128u

#define ROBOT_DEVICE_MOTOR_ROLE_CHASSIS 0u
#define ROBOT_DEVICE_MOTOR_ROLE_YAW 1u
#define ROBOT_DEVICE_MOTOR_ROLE_YAW_UPPER 2u
#define ROBOT_DEVICE_MOTOR_ROLE_PITCH 3u
#define ROBOT_DEVICE_MOTOR_ROLE_TRIGGER 4u
#define ROBOT_DEVICE_MOTOR_ROLE_FRICTION 5u
#define ROBOT_DEVICE_MOTOR_ROLE_ARM 6u
#define ROBOT_DEVICE_MOTOR_ROLE_CUSTOM_BASE 128u

#define ROBOT_DEVICE_SOURCE_NONE 0xFFFFu
#define ROBOT_DEVICE_CONFIG_OFFSET_NONE 0xFFFFFFFFul

typedef struct
{
    const char *name;
    uint32_t config_offset;
    uint16_t source_id;
    uint8_t kind;
    uint8_t role;
    uint8_t role_index;
    uint8_t fallback_bus;
} robot_device_config_entry_t;

typedef struct
{
    uint8_t count;
    robot_device_config_entry_t entry[ROBOT_DEVICE_CONFIG_MAX];
} robot_device_config_table_t;

#define ROBOT_DEVICE_CONFIG_OFFSET(member_) ((uint32_t)offsetof(config_t, member_))

#define ROBOT_DEVICE_ENTRY_RAW(name_, kind_, role_, role_index_, fallback_bus_, source_id_, config_offset_) \
    {(name_), (uint32_t)(config_offset_), (uint16_t)(source_id_), (kind_), (role_), (role_index_), (fallback_bus_)}

#define ROBOT_DEVICE_ENTRY(name_, kind_, role_, role_index_, fallback_bus_, source_id_, config_member_) \
    ROBOT_DEVICE_ENTRY_RAW((name_), (kind_), (role_), (role_index_), (fallback_bus_), (source_id_), ROBOT_DEVICE_CONFIG_OFFSET(config_member_))

#define ROBOT_DEVICE_ENTRY_NO_CONFIG(name_, kind_, role_, role_index_, fallback_bus_, source_id_) \
    ROBOT_DEVICE_ENTRY_RAW((name_), (kind_), (role_), (role_index_), (fallback_bus_), (source_id_), ROBOT_DEVICE_CONFIG_OFFSET_NONE)

#define ROBOT_DEVICE_ENTRY_MOTOR(name_, role_, role_index_, fallback_bus_, source_id_, config_member_) \
    ROBOT_DEVICE_ENTRY((name_), ROBOT_DEVICE_TABLE_KIND_MOTOR, (role_), (role_index_), (fallback_bus_), (source_id_), config_member_)

#define ROBOT_DEVICE_ENTRY_SENSOR(name_, role_, role_index_, config_member_) \
    ROBOT_DEVICE_ENTRY((name_), ROBOT_DEVICE_TABLE_KIND_SENSOR, (role_), (role_index_), 0u, ROBOT_DEVICE_SOURCE_NONE, config_member_)

#define ROBOT_DEVICE_ENTRY_INPUT(name_, role_, role_index_, config_member_) \
    ROBOT_DEVICE_ENTRY((name_), ROBOT_DEVICE_TABLE_KIND_INPUT, (role_), (role_index_), 0u, ROBOT_DEVICE_SOURCE_NONE, config_member_)

#define ROBOT_DEVICE_ENTRY_COMM(name_, role_, role_index_, config_member_) \
    ROBOT_DEVICE_ENTRY((name_), ROBOT_DEVICE_TABLE_KIND_COMM, (role_), (role_index_), 0u, ROBOT_DEVICE_SOURCE_NONE, config_member_)

#define ROBOT_DEVICE_ENTRY_SERVICE(name_, role_, role_index_, config_member_) \
    ROBOT_DEVICE_ENTRY((name_), ROBOT_DEVICE_TABLE_KIND_SERVICE, (role_), (role_index_), 0u, ROBOT_DEVICE_SOURCE_NONE, config_member_)

#define ROBOT_DEFAULT_DEVICE_TABLE \
    { \
        .count = (uint8_t)ROBOT_DEFAULT_DEVICE_TABLE_COUNT, \
        .entry = { \
            ROBOT_DEVICE_ENTRY_MOTOR("motor.chassis0", ROBOT_DEVICE_MOTOR_ROLE_CHASSIS, 0u, 1u, Motor0, motor.chassis[0]), \
            ROBOT_DEVICE_ENTRY_MOTOR("motor.chassis1", ROBOT_DEVICE_MOTOR_ROLE_CHASSIS, 1u, 1u, Motor1, motor.chassis[1]), \
            ROBOT_DEVICE_ENTRY_MOTOR("motor.chassis2", ROBOT_DEVICE_MOTOR_ROLE_CHASSIS, 2u, 1u, Motor2, motor.chassis[2]), \
            ROBOT_DEVICE_ENTRY_MOTOR("motor.chassis3", ROBOT_DEVICE_MOTOR_ROLE_CHASSIS, 3u, 1u, Motor3, motor.chassis[3]), \
            ROBOT_DEVICE_ENTRY_MOTOR("motor.yaw", ROBOT_DEVICE_MOTOR_ROLE_YAW, 0u, 1u, Motor4, motor.yaw), \
            ROBOT_DEVICE_ENTRY_MOTOR("motor.yaw_upper", ROBOT_DEVICE_MOTOR_ROLE_YAW_UPPER, 0u, 1u, Motor5, motor.yaw_upper), \
            ROBOT_DEVICE_ENTRY_MOTOR("motor.pitch", ROBOT_DEVICE_MOTOR_ROLE_PITCH, 0u, 1u, Motor6, motor.pitch), \
            ROBOT_DEVICE_ENTRY_MOTOR("motor.trigger", ROBOT_DEVICE_MOTOR_ROLE_TRIGGER, 0u, 1u, Motor7, motor.trigger), \
            ROBOT_DEVICE_ENTRY_MOTOR("motor.friction0", ROBOT_DEVICE_MOTOR_ROLE_FRICTION, 0u, 2u, Motor8, motor.friction[0]), \
            ROBOT_DEVICE_ENTRY_MOTOR("motor.friction1", ROBOT_DEVICE_MOTOR_ROLE_FRICTION, 1u, 2u, Motor9, motor.friction[1]), \
            ROBOT_DEVICE_ENTRY_MOTOR("motor.friction2", ROBOT_DEVICE_MOTOR_ROLE_FRICTION, 2u, 2u, Motor10, motor.friction[2]), \
            ROBOT_DEVICE_ENTRY_MOTOR("motor.friction3", ROBOT_DEVICE_MOTOR_ROLE_FRICTION, 3u, 2u, Motor11, motor.friction[3]), \
            ROBOT_DEVICE_ENTRY_MOTOR("motor.arm0", ROBOT_DEVICE_MOTOR_ROLE_ARM, 0u, 1u, Motor12, motor.arm[0]), \
            ROBOT_DEVICE_ENTRY_MOTOR("motor.arm1", ROBOT_DEVICE_MOTOR_ROLE_ARM, 1u, 2u, Motor13, motor.arm[1]), \
            ROBOT_DEVICE_ENTRY_MOTOR("motor.arm2", ROBOT_DEVICE_MOTOR_ROLE_ARM, 2u, 2u, Motor14, motor.arm[2]), \
            ROBOT_DEVICE_ENTRY_MOTOR("motor.arm3", ROBOT_DEVICE_MOTOR_ROLE_ARM, 3u, 2u, Motor15, motor.arm[3]), \
            ROBOT_DEVICE_ENTRY_MOTOR("motor.arm4", ROBOT_DEVICE_MOTOR_ROLE_ARM, 4u, 2u, Motor16, motor.arm[4]), \
            ROBOT_DEVICE_ENTRY_MOTOR("motor.arm5", ROBOT_DEVICE_MOTOR_ROLE_ARM, 5u, 2u, Motor17, motor.arm[5]), \
            ROBOT_DEVICE_ENTRY_SENSOR("sensor.imu", ROBOT_DEVICE_ROLE_IMU, 0u, imu), \
            ROBOT_DEVICE_ENTRY_INPUT("input.manual", ROBOT_DEVICE_ROLE_MANUAL_INPUT, 0u, manual_input), \
            ROBOT_DEVICE_ENTRY_SENSOR("sensor.battery", ROBOT_DEVICE_ROLE_BATTERY, 0u, voltage), \
            ROBOT_DEVICE_ENTRY_COMM("link.aux_telem", ROBOT_DEVICE_ROLE_AUX_TELEM, 0u, aux_telem), \
            ROBOT_DEVICE_ENTRY_SERVICE("service.sdlog", ROBOT_DEVICE_ROLE_SDLOG, 0u, sdlog), \
        }, \
    }

#endif
