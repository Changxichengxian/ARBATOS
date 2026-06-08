/*
 * SPDX-FileCopyrightText: 2026 Chen Xuan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "aux_param.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "host_tune_bridge.h"
#include "manual_input.h"
#include "robot_safety.h"
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

typedef enum
{
    CONFIG_PARAM_TYPE_F32 = 0u,
    CONFIG_PARAM_TYPE_U16,
    CONFIG_PARAM_TYPE_U8,
    CONFIG_PARAM_TYPE_I8,
    CONFIG_PARAM_TYPE_BOOL,
} config_param_type_e;

typedef enum
{
    CONFIG_PARAM_POLICY_AUTO = 0u,
    CONFIG_PARAM_POLICY_LIVE,
    CONFIG_PARAM_POLICY_SAFE_ONLY,
    CONFIG_PARAM_POLICY_READONLY,
} config_param_policy_e;

typedef struct
{
    uint16_t id;
    const char *name;
    uint8_t scope;
    uint8_t type;
    uint8_t apply;
    fp32 min_value;
    fp32 max_value;
    uint8_t has_range;
    const char *unit;
    uint8_t policy;
} config_param_desc_t;

static aux_param_result_e g_aux_param_last_result = AUX_PARAM_RESULT_OK;
static uint16_t g_aux_param_last_id = 0u;
static fp32 g_aux_param_last_value = 0.0f;

static const config_param_desc_t g_config_param_descs[] =
{
#define CONFIG_PARAM_F32(ID, NAME, SCOPE, LVALUE, APPLY) \
    { (uint16_t)(ID), (NAME), (uint8_t)(SCOPE), (uint8_t)CONFIG_PARAM_TYPE_F32, (uint8_t)(APPLY), 0.0f, 0.0f, 0u, "", (uint8_t)CONFIG_PARAM_POLICY_AUTO },
#define CONFIG_PARAM_F32_RANGE(ID, NAME, SCOPE, LVALUE, MIN_V, MAX_V, UNIT, APPLY, POLICY) \
    { (uint16_t)(ID), (NAME), (uint8_t)(SCOPE), (uint8_t)CONFIG_PARAM_TYPE_F32, (uint8_t)(APPLY), (fp32)(MIN_V), (fp32)(MAX_V), 1u, (UNIT), (uint8_t)(POLICY) },
#define CONFIG_PARAM_U16(ID, NAME, SCOPE, LVALUE, APPLY) \
    { (uint16_t)(ID), (NAME), (uint8_t)(SCOPE), (uint8_t)CONFIG_PARAM_TYPE_U16, (uint8_t)(APPLY), 0.0f, 65535.0f, 1u, "", (uint8_t)CONFIG_PARAM_POLICY_AUTO },
#define CONFIG_PARAM_U8(ID, NAME, SCOPE, LVALUE, APPLY) \
    { (uint16_t)(ID), (NAME), (uint8_t)(SCOPE), (uint8_t)CONFIG_PARAM_TYPE_U8, (uint8_t)(APPLY), 0.0f, 255.0f, 1u, "", (uint8_t)CONFIG_PARAM_POLICY_AUTO },
#define CONFIG_PARAM_I8_RANGE(ID, NAME, SCOPE, LVALUE, MIN_V, MAX_V, APPLY) \
    { (uint16_t)(ID), (NAME), (uint8_t)(SCOPE), (uint8_t)CONFIG_PARAM_TYPE_I8, (uint8_t)(APPLY), (fp32)(MIN_V), (fp32)(MAX_V), 1u, "", (uint8_t)CONFIG_PARAM_POLICY_AUTO },
#define CONFIG_PARAM_BOOL(ID, NAME, SCOPE, LVALUE, APPLY) \
    { (uint16_t)(ID), (NAME), (uint8_t)(SCOPE), (uint8_t)CONFIG_PARAM_TYPE_BOOL, (uint8_t)(APPLY), 0.0f, 1.0f, 1u, "", (uint8_t)CONFIG_PARAM_POLICY_AUTO },
#define CONFIG_PARAM_U8_MAX(ID, NAME, SCOPE, LVALUE, MAX_V, APPLY) \
    { (uint16_t)(ID), (NAME), (uint8_t)(SCOPE), (uint8_t)CONFIG_PARAM_TYPE_U8, (uint8_t)(APPLY), 0.0f, (fp32)(MAX_V), 1u, "", (uint8_t)CONFIG_PARAM_POLICY_AUTO },
#define CONFIG_PARAM_U8_DEFAULT(ID, NAME, SCOPE, LVALUE, MAX_V, DEFAULT_V, APPLY) \
    { (uint16_t)(ID), (NAME), (uint8_t)(SCOPE), (uint8_t)CONFIG_PARAM_TYPE_U8, (uint8_t)(APPLY), 0.0f, (fp32)(MAX_V), 1u, "", (uint8_t)CONFIG_PARAM_POLICY_AUTO },
#include "config_param_list.inc"
#undef CONFIG_PARAM_F32
#undef CONFIG_PARAM_F32_RANGE
#undef CONFIG_PARAM_U16
#undef CONFIG_PARAM_U8
#undef CONFIG_PARAM_I8_RANGE
#undef CONFIG_PARAM_BOOL
#undef CONFIG_PARAM_U8_MAX
#undef CONFIG_PARAM_U8_DEFAULT
};

static uint8_t aux_param_name_starts_with(const char *name, const char *prefix)
{
    return (uint8_t)((name != NULL && prefix != NULL &&
                      strncmp(name, prefix, strlen(prefix)) == 0) ? 1u : 0u);
}

static uint8_t aux_param_name_contains(const char *name, const char *needle)
{
    return (uint8_t)((name != NULL && needle != NULL && strstr(name, needle) != NULL) ? 1u : 0u);
}

static const config_param_desc_t *aux_param_find_config_param_by_id(uint16_t id)
{
    const uint32_t count = (uint32_t)(sizeof(g_config_param_descs) / sizeof(g_config_param_descs[0]));

    for (uint32_t i = 0u; i < count; i++)
    {
        if (g_config_param_descs[i].id == id)
        {
            return &g_config_param_descs[i];
        }
    }
    return NULL;
}

#if AUX_TUNE_ENABLE_PARAM_NAME_LOOKUP
static const config_param_desc_t *aux_param_find_config_param_by_name(const char *name)
{
    const uint32_t count = (uint32_t)(sizeof(g_config_param_descs) / sizeof(g_config_param_descs[0]));

    if (name == NULL || name[0] == '\0')
    {
        return NULL;
    }

    for (uint32_t i = 0u; i < count; i++)
    {
        if (strcmp(g_config_param_descs[i].name, name) == 0)
        {
            return &g_config_param_descs[i];
        }
    }
    return NULL;
}
#endif

static aux_param_result_e aux_param_record_result(uint16_t id,
                                                  fp32 value,
                                                  aux_param_result_e result)
{
    g_aux_param_last_id = id;
    g_aux_param_last_value = value;
    g_aux_param_last_result = result;
    return result;
}

static uint8_t aux_param_config_scope_is_active(config_param_scope_e scope)
{
    switch (scope)
    {
    case CONFIG_PARAM_SCOPE_COMMON:
        return 1u;
    case CONFIG_PARAM_SCOPE_GIMBAL_SINGLE:
        return (uint8_t)(robot_profile_need_single_gimbal_control_task() ||
                         robot_profile_need_dual_gimbal_control_task());
    case CONFIG_PARAM_SCOPE_GIMBAL_DUAL:
        return (uint8_t)robot_profile_need_dual_gimbal_control_task();
    case CONFIG_PARAM_SCOPE_LOCOMOTION_CLASSIC:
        return (uint8_t)robot_profile_need_classic_chassis_control_task();
    case CONFIG_PARAM_SCOPE_LOCOMOTION_WHEELLEG_SERVO:
        return (uint8_t)robot_profile_need_wheelleg_servo_task();
    case CONFIG_PARAM_SCOPE_LOCOMOTION_WHEELLEG_MIT:
        return (uint8_t)robot_profile_is_wheelleg_mit();
    default:
        return 0u;
    }
}

static uint8_t aux_param_desc_safe_only(const config_param_desc_t *desc)
{
    if (desc == NULL)
    {
        return 1u;
    }
    if (desc->policy == (uint8_t)CONFIG_PARAM_POLICY_LIVE)
    {
        return 0u;
    }
    if (desc->policy == (uint8_t)CONFIG_PARAM_POLICY_SAFE_ONLY ||
        desc->policy == (uint8_t)CONFIG_PARAM_POLICY_READONLY)
    {
        return 1u;
    }

    if (aux_param_name_starts_with(desc->name, "aux_telem.") != 0u ||
        aux_param_name_starts_with(desc->name, "sdlog.") != 0u)
    {
        return 0u;
    }

    return 1u;
}

static uint8_t aux_param_desc_readonly(const config_param_desc_t *desc)
{
    return (uint8_t)((desc != NULL &&
                      desc->policy == (uint8_t)CONFIG_PARAM_POLICY_READONLY) ? 1u : 0u);
}

static uint8_t aux_param_desc_range(const config_param_desc_t *desc,
                                    fp32 *min_value,
                                    fp32 *max_value,
                                    const char **unit)
{
    if (desc == NULL || min_value == NULL || max_value == NULL || unit == NULL)
    {
        return 0u;
    }

    if (desc->has_range != 0u)
    {
        *min_value = desc->min_value;
        *max_value = desc->max_value;
        *unit = desc->unit;
        return 1u;
    }

    switch (desc->id)
    {
    case 41u:
    case 42u:
    case 45u:
        *min_value = 0.0f;
        *max_value = 20000.0f;
        *unit = "cmd";
        return 1u;
    case 98u:
        *min_value = 0.0f;
        *max_value = 10.0f;
        *unit = "mps";
        return 1u;
    case 99u:
    case 100u:
    case 101u:
    case 102u:
        *min_value = 0.0f;
        *max_value = 5.0f;
        *unit = "mps";
        return 1u;
    case 106u:
        *min_value = 0.0f;
        *max_value = 60000.0f;
        *unit = "cmd";
        return 1u;
    case 113u:
    case 114u:
        *min_value = 0.0f;
        *max_value = 10000.0f;
        *unit = "rpm";
        return 1u;
    case 115u:
        *min_value = 0.0f;
        *max_value = 100000.0f;
        *unit = "rpmps";
        return 1u;
    case 116u:
        *min_value = 0.0f;
        *max_value = 1.0f;
        *unit = "ratio";
        return 1u;
    case 139u:
    case 140u:
    case 141u:
    case 145u:
    case 148u:
        *min_value = 0.0f;
        *max_value = 30.0f;
        *unit = "radps";
        return 1u;
    case 161u:
    case 162u:
    case 163u:
        *min_value = 0.0f;
        *max_value = 500.0f;
        *unit = "w";
        return 1u;
    case 164u:
    case 165u:
    case 166u:
        *min_value = 0.0f;
        *max_value = 80000.0f;
        *unit = "cmd";
        return 1u;
    case 601u:
    case 609u:
    case 618u:
    case 619u:
    case 620u:
    case 621u:
        *min_value = -6.2831853f;
        *max_value = 6.2831853f;
        *unit = "rad";
        return 1u;
    case 603u:
    case 610u:
    case 632u:
        *min_value = 0.0f;
        *max_value = 20.0f;
        *unit = "kp";
        return 1u;
    case 604u:
    case 611u:
    case 633u:
        *min_value = 0.0f;
        *max_value = 5.0f;
        *unit = "kd";
        return 1u;
    case 612u:
    case 634u:
        *min_value = -2.0f;
        *max_value = 2.0f;
        *unit = "nm";
        return 1u;
    case 613u:
    case 635u:
        *min_value = 0.0f;
        *max_value = 2.0f;
        *unit = "nm";
        return 1u;
    case 630u:
        *min_value = 0.02f;
        *max_value = 0.30f;
        *unit = "m";
        return 1u;
    case 631u:
        *min_value = 0.0f;
        *max_value = 0.15f;
        *unit = "m";
        return 1u;
    case 638u:
    case 639u:
        *min_value = 0.0f;
        *max_value = 2.0f;
        *unit = "scale";
        return 1u;
    case 641u:
        *min_value = -1.0f;
        *max_value = 1.0f;
        *unit = "gain";
        return 1u;
    case 642u:
        *min_value = 0.0f;
        *max_value = 0.35f;
        *unit = "rad";
        return 1u;
    case 643u:
        *min_value = 0.0f;
        *max_value = 0.20f;
        *unit = "radps";
        return 1u;
    case 644u:
        *min_value = 0.0f;
        *max_value = 0.20f;
        *unit = "mps";
        return 1u;
    case 645u:
        *min_value = 0.001f;
        *max_value = 1.0f;
        *unit = "ratio";
        return 1u;
    default:
        break;
    }

    if (aux_param_name_contains(desc->name, ".max_out") != 0u ||
        aux_param_name_contains(desc->name, ".max_iout") != 0u)
    {
        *min_value = 0.0f;
        *max_value = 80000.0f;
        *unit = "cmd";
        return 1u;
    }

    return 0u;
}

static uint8_t aux_param_value_is_nan(fp32 value)
{
    return (uint8_t)((value != value) ? 1u : 0u);
}

static uint8_t aux_param_value_in_range(const config_param_desc_t *desc, fp32 value)
{
    fp32 min_value = 0.0f;
    fp32 max_value = 0.0f;
    const char *unit = "";

    if (aux_param_desc_range(desc, &min_value, &max_value, &unit) == 0u)
    {
        return 1u;
    }

    (void)unit;
    return (uint8_t)((value >= min_value && value <= max_value) ? 1u : 0u);
}

static uint8_t aux_param_to_u8(fp32 v)
{
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

static int8_t aux_param_to_i8(fp32 v)
{
    fp32 r = v + ((v >= 0.0f) ? 0.5f : -0.5f);

    if (r < -128.0f)
    {
        r = -128.0f;
    }
    if (r > 127.0f)
    {
        r = 127.0f;
    }
    return (int8_t)r;
}

static aux_param_result_e aux_param_prepare_write(const config_param_desc_t *desc,
                                                  fp32 input,
                                                  fp32 *stored)
{
    if (desc == NULL || stored == NULL)
    {
        return AUX_PARAM_RESULT_BAD_ARGUMENT;
    }
    if (aux_param_config_scope_is_active((config_param_scope_e)desc->scope) == 0u)
    {
        return AUX_PARAM_RESULT_INACTIVE_SCOPE;
    }
    if (aux_param_desc_readonly(desc) != 0u)
    {
        return AUX_PARAM_RESULT_READONLY;
    }
    if (aux_param_desc_safe_only(desc) != 0u && robot_safety_output_locked() == 0u)
    {
        return AUX_PARAM_RESULT_SAFE_REQUIRED;
    }
    if (aux_param_value_is_nan(input) != 0u || aux_param_value_in_range(desc, input) == 0u)
    {
        return AUX_PARAM_RESULT_RANGE;
    }

    switch ((config_param_type_e)desc->type)
    {
    case CONFIG_PARAM_TYPE_F32:
        *stored = input;
        return AUX_PARAM_RESULT_OK;
    case CONFIG_PARAM_TYPE_U16:
        *stored = (fp32)aux_param_to_u16(input);
        return AUX_PARAM_RESULT_OK;
    case CONFIG_PARAM_TYPE_U8:
        *stored = (fp32)aux_param_to_u8(input);
        return AUX_PARAM_RESULT_OK;
    case CONFIG_PARAM_TYPE_I8:
        *stored = (fp32)aux_param_to_i8(input);
        return AUX_PARAM_RESULT_OK;
    case CONFIG_PARAM_TYPE_BOOL:
        *stored = (input >= 0.5f) ? 1.0f : 0.0f;
        return AUX_PARAM_RESULT_OK;
    default:
        return AUX_PARAM_RESULT_BAD_ARGUMENT;
    }
}

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

aux_param_result_e aux_param_set_config_param_ex(uint16_t id, fp32 value)
{
    const config_param_desc_t *desc = aux_param_find_config_param_by_id(id);
    fp32 stored = 0.0f;
    aux_param_result_e result;

    if (desc == NULL)
    {
        return aux_param_record_result(id, value, AUX_PARAM_RESULT_UNKNOWN_ID);
    }

    result = aux_param_prepare_write(desc, value, &stored);
    if (result != AUX_PARAM_RESULT_OK)
    {
        return aux_param_record_result(id, value, result);
    }

    switch (id)
    {
#define CONFIG_PARAM_F32(ID, NAME, SCOPE, LVALUE, APPLY) \
    case ID: \
        taskENTER_CRITICAL(); \
        (LVALUE) = stored; \
        taskEXIT_CRITICAL(); \
        aux_param_apply_config_param(APPLY); \
        return aux_param_record_result(id, stored, AUX_PARAM_RESULT_OK);
#define CONFIG_PARAM_F32_RANGE(ID, NAME, SCOPE, LVALUE, MIN_V, MAX_V, UNIT, APPLY, POLICY) \
    case ID: \
        taskENTER_CRITICAL(); \
        (LVALUE) = stored; \
        taskEXIT_CRITICAL(); \
        aux_param_apply_config_param(APPLY); \
        return aux_param_record_result(id, stored, AUX_PARAM_RESULT_OK);
#define CONFIG_PARAM_U16(ID, NAME, SCOPE, LVALUE, APPLY) \
    case ID: \
        taskENTER_CRITICAL(); \
        (LVALUE) = (uint16_t)stored; \
        taskEXIT_CRITICAL(); \
        aux_param_apply_config_param(APPLY); \
        return aux_param_record_result(id, stored, AUX_PARAM_RESULT_OK);
#define CONFIG_PARAM_U8(ID, NAME, SCOPE, LVALUE, APPLY) \
    case ID: \
        taskENTER_CRITICAL(); \
        (LVALUE) = (uint8_t)stored; \
        taskEXIT_CRITICAL(); \
        aux_param_apply_config_param(APPLY); \
        return aux_param_record_result(id, stored, AUX_PARAM_RESULT_OK);
#define CONFIG_PARAM_I8_RANGE(ID, NAME, SCOPE, LVALUE, MIN_V, MAX_V, APPLY) \
    case ID: \
        taskENTER_CRITICAL(); \
        (LVALUE) = (int8_t)stored; \
        taskEXIT_CRITICAL(); \
        aux_param_apply_config_param(APPLY); \
        return aux_param_record_result(id, stored, AUX_PARAM_RESULT_OK);
#define CONFIG_PARAM_BOOL(ID, NAME, SCOPE, LVALUE, APPLY) \
    case ID: \
        taskENTER_CRITICAL(); \
        (LVALUE) = (uint8_t)stored; \
        taskEXIT_CRITICAL(); \
        aux_param_apply_config_param(APPLY); \
        return aux_param_record_result(id, stored, AUX_PARAM_RESULT_OK);
#define CONFIG_PARAM_U8_MAX(ID, NAME, SCOPE, LVALUE, MAX_V, APPLY) \
    case ID: \
        taskENTER_CRITICAL(); \
        (LVALUE) = (uint8_t)stored; \
        taskEXIT_CRITICAL(); \
        aux_param_apply_config_param(APPLY); \
        return aux_param_record_result(id, stored, AUX_PARAM_RESULT_OK);
#define CONFIG_PARAM_U8_DEFAULT(ID, NAME, SCOPE, LVALUE, MAX_V, DEFAULT_V, APPLY) \
    case ID: \
        taskENTER_CRITICAL(); \
        (LVALUE) = (uint8_t)stored; \
        taskEXIT_CRITICAL(); \
        aux_param_apply_config_param(APPLY); \
        return aux_param_record_result(id, stored, AUX_PARAM_RESULT_OK);
#include "config_param_list.inc"
#undef CONFIG_PARAM_F32
#undef CONFIG_PARAM_F32_RANGE
#undef CONFIG_PARAM_U16
#undef CONFIG_PARAM_U8
#undef CONFIG_PARAM_I8_RANGE
#undef CONFIG_PARAM_BOOL
#undef CONFIG_PARAM_U8_MAX
#undef CONFIG_PARAM_U8_DEFAULT
    default:
        return aux_param_record_result(id, value, AUX_PARAM_RESULT_UNKNOWN_ID);
    }
}

aux_param_result_e aux_param_validate_config_param(uint16_t id, fp32 value)
{
    const config_param_desc_t *desc = aux_param_find_config_param_by_id(id);
    fp32 stored = 0.0f;
    aux_param_result_e result;

    if (desc == NULL)
    {
        return aux_param_record_result(id, value, AUX_PARAM_RESULT_UNKNOWN_ID);
    }
    result = aux_param_prepare_write(desc, value, &stored);
    return aux_param_record_result(id, value, result);
}

bool_t aux_param_set_config_param(uint16_t id, fp32 value)
{
    return (bool_t)((aux_param_set_config_param_ex(id, value) == AUX_PARAM_RESULT_OK) ? 1u : 0u);
}

bool_t aux_param_get_config_param(uint16_t id, fp32 *out)
{
    if (out == NULL)
    {
        return 0;
    }

    switch (id)
    {
#define CONFIG_PARAM_F32(ID, NAME, SCOPE, LVALUE, APPLY) \
    case ID: \
        taskENTER_CRITICAL(); \
        *out = (fp32)(LVALUE); \
        taskEXIT_CRITICAL(); \
        return 1;
#define CONFIG_PARAM_F32_RANGE(ID, NAME, SCOPE, LVALUE, MIN_V, MAX_V, UNIT, APPLY, POLICY) \
    case ID: \
        taskENTER_CRITICAL(); \
        *out = (fp32)(LVALUE); \
        taskEXIT_CRITICAL(); \
        return 1;
#define CONFIG_PARAM_U16(ID, NAME, SCOPE, LVALUE, APPLY) \
    case ID: \
        taskENTER_CRITICAL(); \
        *out = (fp32)(LVALUE); \
        taskEXIT_CRITICAL(); \
        return 1;
#define CONFIG_PARAM_U8(ID, NAME, SCOPE, LVALUE, APPLY) \
    case ID: \
        taskENTER_CRITICAL(); \
        *out = (fp32)(LVALUE); \
        taskEXIT_CRITICAL(); \
        return 1;
#define CONFIG_PARAM_I8_RANGE(ID, NAME, SCOPE, LVALUE, MIN_V, MAX_V, APPLY) \
    case ID: \
        taskENTER_CRITICAL(); \
        *out = (fp32)(LVALUE); \
        taskEXIT_CRITICAL(); \
        return 1;
#define CONFIG_PARAM_BOOL(ID, NAME, SCOPE, LVALUE, APPLY) \
    case ID: \
        taskENTER_CRITICAL(); \
        *out = (fp32)(LVALUE); \
        taskEXIT_CRITICAL(); \
        return 1;
#define CONFIG_PARAM_U8_MAX(ID, NAME, SCOPE, LVALUE, MAX_V, APPLY) \
    case ID: \
        taskENTER_CRITICAL(); \
        *out = (fp32)(LVALUE); \
        taskEXIT_CRITICAL(); \
        return 1;
#define CONFIG_PARAM_U8_DEFAULT(ID, NAME, SCOPE, LVALUE, MAX_V, DEFAULT_V, APPLY) \
    case ID: \
        taskENTER_CRITICAL(); \
        *out = (fp32)(LVALUE); \
        taskEXIT_CRITICAL(); \
        return 1;
#include "config_param_list.inc"
#undef CONFIG_PARAM_F32
#undef CONFIG_PARAM_F32_RANGE
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

uint16_t aux_param_get_count(void)
{
    return (uint16_t)(sizeof(g_config_param_descs) / sizeof(g_config_param_descs[0]));
}

bool_t aux_param_get_info_by_index(uint16_t index, aux_param_info_t *out)
{
    fp32 min_value = 0.0f;
    fp32 max_value = 0.0f;
    const char *unit = "";
    const config_param_desc_t *desc;
    const uint16_t count = aux_param_get_count();

    if (out == NULL || index >= count)
    {
        return 0;
    }

    desc = &g_config_param_descs[index];
    out->id = desc->id;
    out->name = desc->name;
    out->has_range = aux_param_desc_range(desc, &min_value, &max_value, &unit);
    out->min_value = min_value;
    out->max_value = max_value;
    out->unit = unit;
    out->safe_only = aux_param_desc_safe_only(desc);
    out->active = aux_param_config_scope_is_active((config_param_scope_e)desc->scope);
    return 1;
}

bool_t aux_param_get_info(uint16_t id, aux_param_info_t *out)
{
    const uint16_t count = aux_param_get_count();

    if (out == NULL)
    {
        return 0;
    }

    for (uint16_t i = 0u; i < count; i++)
    {
        if (g_config_param_descs[i].id == id)
        {
            return aux_param_get_info_by_index(i, out);
        }
    }
    return 0;
}

aux_param_result_e aux_param_get_last_result(void)
{
    return g_aux_param_last_result;
}

uint16_t aux_param_get_last_id(void)
{
    return g_aux_param_last_id;
}

fp32 aux_param_get_last_value(void)
{
    return g_aux_param_last_value;
}

const char *aux_param_result_name(aux_param_result_e result)
{
    switch (result)
    {
    case AUX_PARAM_RESULT_OK:
        return "ok";
    case AUX_PARAM_RESULT_UNKNOWN_ID:
        return "unknown_id";
    case AUX_PARAM_RESULT_INACTIVE_SCOPE:
        return "inactive_scope";
    case AUX_PARAM_RESULT_RANGE:
        return "range";
    case AUX_PARAM_RESULT_SAFE_REQUIRED:
        return "safe_required";
    case AUX_PARAM_RESULT_READONLY:
        return "readonly";
    case AUX_PARAM_RESULT_BAD_ARGUMENT:
        return "bad_argument";
    default:
        return "unknown";
    }
}

#if AUX_TUNE_ENABLE_PARAM_NAME_LOOKUP
aux_param_result_e aux_param_set_config_param_by_name_ex(const char *name, fp32 value)
{
    const config_param_desc_t *desc = aux_param_find_config_param_by_name(name);

    if (desc == NULL)
    {
        return aux_param_record_result(0u, value, AUX_PARAM_RESULT_UNKNOWN_ID);
    }
    return aux_param_set_config_param_ex(desc->id, value);
}

bool_t aux_param_set_config_param_by_name(const char *name, fp32 value)
{
    return (bool_t)((aux_param_set_config_param_by_name_ex(name, value) == AUX_PARAM_RESULT_OK) ? 1u : 0u);
}
#endif
