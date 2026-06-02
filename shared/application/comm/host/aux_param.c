/*
 * SPDX-FileCopyrightText: 2026 Chen Xuan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "aux_param.h"

#include <string.h>
#include "actuator_cmd.h"
#include "host_tune_bridge.h"
#include "manual_input.h"
#include "robot_task_profile.h"

typedef enum
{
    CONFIG_PARAM_SCOPE_COMMON = 0u,
    CONFIG_PARAM_SCOPE_GIMBAL_SINGLE,
    CONFIG_PARAM_SCOPE_GIMBAL_DUAL,
    CONFIG_PARAM_SCOPE_LOCOMOTION_CLASSIC,
    CONFIG_PARAM_SCOPE_LOCOMOTION_WHEELLEG_SERVO,
    CONFIG_PARAM_SCOPE_LOCOMOTION_WHEELLEG_MIT,
} config_param_scope_e;

typedef enum
{
    CONFIG_PARAM_APPLY_NONE = 0u,
    CONFIG_PARAM_APPLY_REMOTE_REFRESH,
    CONFIG_PARAM_APPLY_GIMBAL_YAW_SPEED_PID,
    CONFIG_PARAM_APPLY_GIMBAL_PITCH_SPEED_PID,
    CONFIG_PARAM_APPLY_GIMBAL_YAW_ANGLE_PID,
    CONFIG_PARAM_APPLY_GIMBAL_PITCH_ANGLE_PID,
    CONFIG_PARAM_APPLY_CHASSIS_MOTOR_SPEED_PID,
    CONFIG_PARAM_APPLY_CHASSIS_FOLLOW_PID,
    CONFIG_PARAM_APPLY_SHOOT_FRIC_SPEED_PID,
    CONFIG_PARAM_APPLY_SHOOT_TRIGGER_PID,
} config_param_apply_e;

#ifndef AUX_TUNE_ENABLE_PARAM_NAME_LOOKUP
#define AUX_TUNE_ENABLE_PARAM_NAME_LOOKUP 0u
#endif

#if AUX_TUNE_ENABLE_PARAM_NAME_LOOKUP
typedef struct
{
    uint16_t id;
    const char *name;
    uint8_t scope;
} config_param_desc_t;

static const config_param_desc_t g_config_param_descs[] =
{
#define CONFIG_PARAM_F32(ID, NAME, SCOPE, LVALUE, APPLY) \
    { (uint16_t)(ID), (NAME), (uint8_t)(SCOPE) },
#define CONFIG_PARAM_U16(ID, NAME, SCOPE, LVALUE, APPLY) \
    { (uint16_t)(ID), (NAME), (uint8_t)(SCOPE) },
#define CONFIG_PARAM_U8(ID, NAME, SCOPE, LVALUE, APPLY) \
    { (uint16_t)(ID), (NAME), (uint8_t)(SCOPE) },
#define CONFIG_PARAM_I8_RANGE(ID, NAME, SCOPE, LVALUE, MIN_V, MAX_V, APPLY) \
    { (uint16_t)(ID), (NAME), (uint8_t)(SCOPE) },
#define CONFIG_PARAM_BOOL(ID, NAME, SCOPE, LVALUE, APPLY) \
    { (uint16_t)(ID), (NAME), (uint8_t)(SCOPE) },
#define CONFIG_PARAM_U8_MAX(ID, NAME, SCOPE, LVALUE, MAX_V, APPLY) \
    { (uint16_t)(ID), (NAME), (uint8_t)(SCOPE) },
#define CONFIG_PARAM_U8_DEFAULT(ID, NAME, SCOPE, LVALUE, MAX_V, DEFAULT_V, APPLY) \
    { (uint16_t)(ID), (NAME), (uint8_t)(SCOPE) },
#include "config_param_list.inc"
#undef CONFIG_PARAM_F32
#undef CONFIG_PARAM_U16
#undef CONFIG_PARAM_U8
#undef CONFIG_PARAM_I8_RANGE
#undef CONFIG_PARAM_BOOL
#undef CONFIG_PARAM_U8_MAX
#undef CONFIG_PARAM_U8_DEFAULT
};
#endif

static uint8_t aux_param_to_u8(fp32 v)
{
    if (v <= 0.0f)
    {
        return 0u;
    }
    if (v >= 255.0f)
    {
        return 255u;
    }

    fp32 r = v + 0.5f;
    if (r < 0.0f)
    {
        r = 0.0f;
    }
    if (r > 255.0f)
    {
        r = 255.0f;
    }
    return (uint8_t)r;
}

static uint16_t aux_param_to_u16(fp32 v)
{
    if (v <= 0.0f)
    {
        return 0u;
    }
    if (v >= 65535.0f)
    {
        return 65535u;
    }

    fp32 r = v + 0.5f;
    if (r < 0.0f)
    {
        r = 0.0f;
    }
    if (r > 65535.0f)
    {
        r = 65535.0f;
    }
    return (uint16_t)r;
}

static int8_t aux_param_to_i8(fp32 v, int8_t min_v, int8_t max_v)
{
    if (min_v > max_v)
    {
        const int8_t t = min_v;
        min_v = max_v;
        max_v = t;
    }

    if (v <= (fp32)min_v)
    {
        return min_v;
    }
    if (v >= (fp32)max_v)
    {
        return max_v;
    }

    fp32 r = v + ((v >= 0.0f) ? 0.5f : -0.5f);
    if (r < (fp32)min_v)
    {
        r = (fp32)min_v;
    }
    if (r > (fp32)max_v)
    {
        r = (fp32)max_v;
    }
    return (int8_t)r;
}

static bool_t aux_param_config_scope_is_active(config_param_scope_e scope)
{
    switch (scope)
    {
    case CONFIG_PARAM_SCOPE_COMMON:
        return 1;
    case CONFIG_PARAM_SCOPE_GIMBAL_SINGLE:
        return (bool_t)(robot_profile_need_single_gimbal_control_task() ||
                        robot_profile_need_dual_gimbal_control_task());
    case CONFIG_PARAM_SCOPE_GIMBAL_DUAL:
        return (bool_t)robot_profile_need_dual_gimbal_control_task();
    case CONFIG_PARAM_SCOPE_LOCOMOTION_CLASSIC:
        return (bool_t)robot_profile_need_classic_chassis_control_task();
    case CONFIG_PARAM_SCOPE_LOCOMOTION_WHEELLEG_SERVO:
        return (bool_t)robot_profile_need_wheelleg_servo_task();
    case CONFIG_PARAM_SCOPE_LOCOMOTION_WHEELLEG_MIT:
        return (bool_t)robot_profile_is_wheelleg_mit();
    default:
        return 0;
    }
}

#if AUX_TUNE_ENABLE_PARAM_NAME_LOOKUP
static const config_param_desc_t *aux_param_find_config_param_by_name(const char *name)
{
    if (name == NULL || name[0] == '\0')
    {
        return NULL;
    }

    const uint32_t count = (uint32_t)(sizeof(g_config_param_descs) / sizeof(g_config_param_descs[0]));
    for (uint32_t i = 0; i < count; i++)
    {
        if (strcmp(g_config_param_descs[i].name, name) == 0)
        {
            if (!aux_param_config_scope_is_active((config_param_scope_e)g_config_param_descs[i].scope))
            {
                return NULL;
            }
            return &g_config_param_descs[i];
        }
    }
    return NULL;
}
#endif

static void aux_param_apply_gimbal_yaw_speed_pid(void)
{
    gimbal_tune_set_yaw_speed_pid(&g_config.gimbal.yaw_speed_pid, 1);
}

static void aux_param_apply_gimbal_pitch_speed_pid(void)
{
    gimbal_tune_set_pitch_speed_pid(&g_config.gimbal.pitch_speed_pid, 1);
}

static void aux_param_apply_gimbal_yaw_angle_pid(void)
{
    gimbal_tune_set_yaw_angle_pid(&g_config.gimbal.yaw_encode_angle_pid, 1);
}

static void aux_param_apply_gimbal_pitch_angle_pid(void)
{
    gimbal_tune_set_pitch_angle_pid(&g_config.gimbal.pitch_encode_angle_pid, 1);
}

static void aux_param_apply_chassis_motor_speed_pid(void)
{
    chassis_tune_set_motor_speed_pid(&g_config.chassis.motor_speed_pid, 1);
}

static void aux_param_apply_chassis_follow_pid(void)
{
    chassis_tune_set_follow_pid(&g_config.chassis.follow_gimbal_pid, 1);
}

static void aux_param_apply_shoot_fric_speed_pid(void)
{
    shoot_tune_apply_fric_speed_pid();
}

static void aux_param_apply_shoot_trigger_pid(void)
{
    shoot_tune_apply_trigger_pid();
}

static void aux_param_apply_config_param(config_param_apply_e action)
{
    switch (action)
    {
    case CONFIG_PARAM_APPLY_NONE:
        return;
    case CONFIG_PARAM_APPLY_REMOTE_REFRESH:
        remote_control_refresh();
        return;
    case CONFIG_PARAM_APPLY_GIMBAL_YAW_SPEED_PID:
        aux_param_apply_gimbal_yaw_speed_pid();
        return;
    case CONFIG_PARAM_APPLY_GIMBAL_PITCH_SPEED_PID:
        aux_param_apply_gimbal_pitch_speed_pid();
        return;
    case CONFIG_PARAM_APPLY_GIMBAL_YAW_ANGLE_PID:
        aux_param_apply_gimbal_yaw_angle_pid();
        return;
    case CONFIG_PARAM_APPLY_GIMBAL_PITCH_ANGLE_PID:
        aux_param_apply_gimbal_pitch_angle_pid();
        return;
    case CONFIG_PARAM_APPLY_CHASSIS_MOTOR_SPEED_PID:
        aux_param_apply_chassis_motor_speed_pid();
        return;
    case CONFIG_PARAM_APPLY_CHASSIS_FOLLOW_PID:
        aux_param_apply_chassis_follow_pid();
        return;
    case CONFIG_PARAM_APPLY_SHOOT_FRIC_SPEED_PID:
        aux_param_apply_shoot_fric_speed_pid();
        return;
    case CONFIG_PARAM_APPLY_SHOOT_TRIGGER_PID:
        aux_param_apply_shoot_trigger_pid();
        return;
    default:
        return;
    }
}

bool_t aux_param_set_config_param(uint16_t id, fp32 value)
{
    switch (id)
    {
#define CONFIG_PARAM_F32(ID, NAME, SCOPE, LVALUE, APPLY) \
    case ID: \
        if (!aux_param_config_scope_is_active((SCOPE))) { return 0; } \
        (LVALUE) = value; \
        aux_param_apply_config_param(APPLY); \
        return 1;
#define CONFIG_PARAM_U16(ID, NAME, SCOPE, LVALUE, APPLY) \
    case ID: \
        if (!aux_param_config_scope_is_active((SCOPE))) { return 0; } \
        (LVALUE) = aux_param_to_u16(value); \
        aux_param_apply_config_param(APPLY); \
        return 1;
#define CONFIG_PARAM_U8(ID, NAME, SCOPE, LVALUE, APPLY) \
    case ID: \
        if (!aux_param_config_scope_is_active((SCOPE))) { return 0; } \
        (LVALUE) = aux_param_to_u8(value); \
        aux_param_apply_config_param(APPLY); \
        return 1;
#define CONFIG_PARAM_I8_RANGE(ID, NAME, SCOPE, LVALUE, MIN_V, MAX_V, APPLY) \
    case ID: \
        if (!aux_param_config_scope_is_active((SCOPE))) { return 0; } \
        (LVALUE) = aux_param_to_i8(value, (MIN_V), (MAX_V)); \
        aux_param_apply_config_param(APPLY); \
        return 1;
#define CONFIG_PARAM_BOOL(ID, NAME, SCOPE, LVALUE, APPLY) \
    case ID: \
        if (!aux_param_config_scope_is_active((SCOPE))) { return 0; } \
        (LVALUE) = (aux_param_to_u8(value) != 0u) ? 1u : 0u; \
        aux_param_apply_config_param(APPLY); \
        return 1;
#define CONFIG_PARAM_U8_MAX(ID, NAME, SCOPE, LVALUE, MAX_V, APPLY) \
    case ID: \
    { \
        if (!aux_param_config_scope_is_active((SCOPE))) { return 0; } \
        uint8_t v__ = aux_param_to_u8(value); \
        if (v__ > (uint8_t)(MAX_V)) \
        { \
            v__ = (uint8_t)(MAX_V); \
        } \
        (LVALUE) = v__; \
        aux_param_apply_config_param(APPLY); \
        return 1; \
    }
#define CONFIG_PARAM_U8_DEFAULT(ID, NAME, SCOPE, LVALUE, MAX_V, DEFAULT_V, APPLY) \
    case ID: \
    { \
        if (!aux_param_config_scope_is_active((SCOPE))) { return 0; } \
        uint8_t v__ = aux_param_to_u8(value); \
        if (v__ > (uint8_t)(MAX_V)) \
        { \
            v__ = (uint8_t)(DEFAULT_V); \
        } \
        (LVALUE) = v__; \
        aux_param_apply_config_param(APPLY); \
        return 1; \
    }
#include "config_param_list.inc"
#undef CONFIG_PARAM_F32
#undef CONFIG_PARAM_U16
#undef CONFIG_PARAM_U8
#undef CONFIG_PARAM_I8_RANGE
#undef CONFIG_PARAM_BOOL
#undef CONFIG_PARAM_U8_MAX
#undef CONFIG_PARAM_U8_DEFAULT
    default:
        return 0;
    }
}

#if AUX_TUNE_ENABLE_PARAM_NAME_LOOKUP
bool_t aux_param_set_config_param_by_name(const char *name, fp32 value)
{
    const config_param_desc_t *desc = aux_param_find_config_param_by_name(name);
    if (desc == NULL)
    {
        return 0;
    }
    return aux_param_set_config_param(desc->id, value);
}
#endif
