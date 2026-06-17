/*
 * SPDX-FileCopyrightText: 2026 Chen Xuan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "AuxParam.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "HostTuneBridge.h"
#include "ManualInput.h"
#include "RobotSafety.h"
#include "RobotTaskProfile.h"

typedef enum
{
    CONFIG_PARAM_SCOPE_COMMON = 0u,
    CONFIG_PARAM_SCOPE_GIMBAL_SINGLE,
    CONFIG_PARAM_SCOPE_GIMBAL_DUAL,
    CONFIG_PARAM_SCOPE_LOCOMOTION_CLASSIC,
    CONFIG_PARAM_SCOPE_LOCOMOTION_WHEELLEG_SERVO,
    CONFIG_PARAM_SCOPE_LOCOMOTION_WHEELLEG_MIT,
} ConfigParamScope;

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
} ConfigParamApply;

typedef enum
{
    CONFIG_PARAM_TYPE_F32 = 0u,
    CONFIG_PARAM_TYPE_U16,
    CONFIG_PARAM_TYPE_U8,
    CONFIG_PARAM_TYPE_I8,
    CONFIG_PARAM_TYPE_BOOL,
} ConfigParamType;

typedef enum
{
    CONFIG_PARAM_POLICY_AUTO = 0u,
    CONFIG_PARAM_POLICY_LIVE,
    CONFIG_PARAM_POLICY_SAFE_ONLY,
    CONFIG_PARAM_POLICY_READONLY,
} ConfigParamPolicy;

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
} ConfigParamDesc;

static AuxParamResult g_aux_param_last_result = AUX_PARAM_RESULT_OK;
static uint16_t g_aux_param_last_id = 0u;
static fp32 g_aux_param_last_value = 0.0f;

static const ConfigParamDesc g_config_param_descs[] =
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
#include "ConfigParamList.inc"
#undef CONFIG_PARAM_F32
#undef CONFIG_PARAM_F32_RANGE
#undef CONFIG_PARAM_U16
#undef CONFIG_PARAM_U8
#undef CONFIG_PARAM_I8_RANGE
#undef CONFIG_PARAM_BOOL
#undef CONFIG_PARAM_U8_MAX
#undef CONFIG_PARAM_U8_DEFAULT
};

static uint8_t AuxParamNameStartsWith(const char *name, const char *prefix)
{
    return (uint8_t)((name != NULL && prefix != NULL &&
                      strncmp(name, prefix, strlen(prefix)) == 0) ? 1u : 0u);
}

static uint8_t AuxParamNameContains(const char *name, const char *needle)
{
    return (uint8_t)((name != NULL && needle != NULL && strstr(name, needle) != NULL) ? 1u : 0u);
}

static const ConfigParamDesc *AuxParamFindConfigParamById(uint16_t id)
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
static const ConfigParamDesc *AuxParamFindConfigParamByName(const char *name)
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

static AuxParamResult AuxParamRecordResult(uint16_t id,
                                                  fp32 value,
                                                  AuxParamResult result)
{
    g_aux_param_last_id = id;
    g_aux_param_last_value = value;
    g_aux_param_last_result = result;
    return result;
}

static uint8_t AuxParamConfigScopeIsActive(ConfigParamScope scope)
{
    switch (scope)
    {
    case CONFIG_PARAM_SCOPE_COMMON:
        return 1u;
    case CONFIG_PARAM_SCOPE_GIMBAL_SINGLE:
        return (uint8_t)(RobotProfileNeedSingleGimbalControlTask() ||
                         RobotProfileNeedDualGimbalControlTask());
    case CONFIG_PARAM_SCOPE_GIMBAL_DUAL:
        return (uint8_t)RobotProfileNeedDualGimbalControlTask();
    case CONFIG_PARAM_SCOPE_LOCOMOTION_CLASSIC:
        return (uint8_t)RobotProfileNeedClassicChassisControlTask();
    case CONFIG_PARAM_SCOPE_LOCOMOTION_WHEELLEG_SERVO:
        return (uint8_t)RobotProfileNeedWheelLegServoTask();
    case CONFIG_PARAM_SCOPE_LOCOMOTION_WHEELLEG_MIT:
        return (uint8_t)RobotProfileIsWheelLegMit();
    default:
        return 0u;
    }
}

static uint8_t AuxParamDescSafeOnly(const ConfigParamDesc *desc)
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

    if (AuxParamNameStartsWith(desc->name, "AuxTelem.") != 0u ||
        AuxParamNameStartsWith(desc->name, "sdlog.") != 0u)
    {
        return 0u;
    }

    return 1u;
}

static uint8_t AuxParamDescReadonly(const ConfigParamDesc *desc)
{
    return (uint8_t)((desc != NULL &&
                      desc->policy == (uint8_t)CONFIG_PARAM_POLICY_READONLY) ? 1u : 0u);
}

static uint8_t AuxParamDescRange(const ConfigParamDesc *desc,
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

    if (AuxParamNameContains(desc->name, ".max_out") != 0u ||
        AuxParamNameContains(desc->name, ".max_iout") != 0u)
    {
        *min_value = 0.0f;
        *max_value = 80000.0f;
        *unit = "cmd";
        return 1u;
    }

    return 0u;
}

static uint8_t AuxParamValueIsNan(fp32 value)
{
    return (uint8_t)((value != value) ? 1u : 0u);
}

static uint8_t AuxParamValueInRange(const ConfigParamDesc *desc, fp32 value)
{
    fp32 min_value = 0.0f;
    fp32 max_value = 0.0f;
    const char *unit = "";

    if (AuxParamDescRange(desc, &min_value, &max_value, &unit) == 0u)
    {
        return 1u;
    }

    (void)unit;
    return (uint8_t)((value >= min_value && value <= max_value) ? 1u : 0u);
}

static uint8_t AuxParamToU8(fp32 v)
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

static uint16_t AuxParamToU16(fp32 v)
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

static int8_t AuxParamToI8(fp32 v)
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

static AuxParamResult AuxParamPrepareWrite(const ConfigParamDesc *desc,
                                                  fp32 input,
                                                  fp32 *stored)
{
    if (desc == NULL || stored == NULL)
    {
        return AUX_PARAM_RESULT_BAD_ARGUMENT;
    }
    if (AuxParamConfigScopeIsActive((ConfigParamScope)desc->scope) == 0u)
    {
        return AUX_PARAM_RESULT_INACTIVE_SCOPE;
    }
    if (AuxParamDescReadonly(desc) != 0u)
    {
        return AUX_PARAM_RESULT_READONLY;
    }
    if (AuxParamDescSafeOnly(desc) != 0u && RobotSafetyOutputLocked() == 0u)
    {
        return AUX_PARAM_RESULT_SAFE_REQUIRED;
    }
    if (AuxParamValueIsNan(input) != 0u || AuxParamValueInRange(desc, input) == 0u)
    {
        return AUX_PARAM_RESULT_RANGE;
    }

    switch ((ConfigParamType)desc->type)
    {
    case CONFIG_PARAM_TYPE_F32:
        *stored = input;
        return AUX_PARAM_RESULT_OK;
    case CONFIG_PARAM_TYPE_U16:
        *stored = (fp32)AuxParamToU16(input);
        return AUX_PARAM_RESULT_OK;
    case CONFIG_PARAM_TYPE_U8:
        *stored = (fp32)AuxParamToU8(input);
        return AUX_PARAM_RESULT_OK;
    case CONFIG_PARAM_TYPE_I8:
        *stored = (fp32)AuxParamToI8(input);
        return AUX_PARAM_RESULT_OK;
    case CONFIG_PARAM_TYPE_BOOL:
        *stored = (input >= 0.5f) ? 1.0f : 0.0f;
        return AUX_PARAM_RESULT_OK;
    default:
        return AUX_PARAM_RESULT_BAD_ARGUMENT;
    }
}

static void AuxParamApplyGimbalYawSpeedPid(void)
{
    GimbalTuneSetYawSpeedPid(&g_config.gimbal.yaw_speed_pid, 1);
}

static void AuxParamApplyGimbalPitchSpeedPid(void)
{
    GimbalTuneSetPitchSpeedPid(&g_config.gimbal.pitch_speed_pid, 1);
}

static void AuxParamApplyGimbalYawAnglePid(void)
{
    GimbalTuneSetYawAnglePid(&g_config.gimbal.yaw_encode_angle_pid, 1);
}

static void AuxParamApplyGimbalPitchAnglePid(void)
{
    GimbalTuneSetPitchAnglePid(&g_config.gimbal.pitch_encode_angle_pid, 1);
}

static void AuxParamApplyChassisMotorSpeedPid(void)
{
    ChassisTuneSetMotorSpeedPid(&g_config.chassis.motor_speed_pid, 1);
}

static void AuxParamApplyChassisFollowPid(void)
{
    ChassisTuneSetFollowPid(&g_config.chassis.follow_gimbal_pid, 1);
}

static void AuxParamApplyShootFricSpeedPid(void)
{
    ShootTuneApplyFricSpeedPid();
}

static void AuxParamApplyShootTriggerPid(void)
{
    ShootTuneApplyTriggerPid();
}

static void AuxParamApplyConfigParam(ConfigParamApply action)
{
    switch (action)
    {
    case CONFIG_PARAM_APPLY_NONE:
        return;
    case CONFIG_PARAM_APPLY_REMOTE_REFRESH:
        remote_control_refresh();
        return;
    case CONFIG_PARAM_APPLY_GIMBAL_YAW_SPEED_PID:
        AuxParamApplyGimbalYawSpeedPid();
        return;
    case CONFIG_PARAM_APPLY_GIMBAL_PITCH_SPEED_PID:
        AuxParamApplyGimbalPitchSpeedPid();
        return;
    case CONFIG_PARAM_APPLY_GIMBAL_YAW_ANGLE_PID:
        AuxParamApplyGimbalYawAnglePid();
        return;
    case CONFIG_PARAM_APPLY_GIMBAL_PITCH_ANGLE_PID:
        AuxParamApplyGimbalPitchAnglePid();
        return;
    case CONFIG_PARAM_APPLY_CHASSIS_MOTOR_SPEED_PID:
        AuxParamApplyChassisMotorSpeedPid();
        return;
    case CONFIG_PARAM_APPLY_CHASSIS_FOLLOW_PID:
        AuxParamApplyChassisFollowPid();
        return;
    case CONFIG_PARAM_APPLY_SHOOT_FRIC_SPEED_PID:
        AuxParamApplyShootFricSpeedPid();
        return;
    case CONFIG_PARAM_APPLY_SHOOT_TRIGGER_PID:
        AuxParamApplyShootTriggerPid();
        return;
    default:
        return;
    }
}

AuxParamResult AuxParamSetConfigParamEx(uint16_t id, fp32 value)
{
    const ConfigParamDesc *desc = AuxParamFindConfigParamById(id);
    fp32 stored = 0.0f;
    AuxParamResult result;

    if (desc == NULL)
    {
        return AuxParamRecordResult(id, value, AUX_PARAM_RESULT_UNKNOWN_ID);
    }

    result = AuxParamPrepareWrite(desc, value, &stored);
    if (result != AUX_PARAM_RESULT_OK)
    {
        return AuxParamRecordResult(id, value, result);
    }

    switch (id)
    {
#define CONFIG_PARAM_F32(ID, NAME, SCOPE, LVALUE, APPLY) \
    case ID: \
        taskENTER_CRITICAL(); \
        (LVALUE) = stored; \
        taskEXIT_CRITICAL(); \
        AuxParamApplyConfigParam(APPLY); \
        return AuxParamRecordResult(id, stored, AUX_PARAM_RESULT_OK);
#define CONFIG_PARAM_F32_RANGE(ID, NAME, SCOPE, LVALUE, MIN_V, MAX_V, UNIT, APPLY, POLICY) \
    case ID: \
        taskENTER_CRITICAL(); \
        (LVALUE) = stored; \
        taskEXIT_CRITICAL(); \
        AuxParamApplyConfigParam(APPLY); \
        return AuxParamRecordResult(id, stored, AUX_PARAM_RESULT_OK);
#define CONFIG_PARAM_U16(ID, NAME, SCOPE, LVALUE, APPLY) \
    case ID: \
        taskENTER_CRITICAL(); \
        (LVALUE) = (uint16_t)stored; \
        taskEXIT_CRITICAL(); \
        AuxParamApplyConfigParam(APPLY); \
        return AuxParamRecordResult(id, stored, AUX_PARAM_RESULT_OK);
#define CONFIG_PARAM_U8(ID, NAME, SCOPE, LVALUE, APPLY) \
    case ID: \
        taskENTER_CRITICAL(); \
        (LVALUE) = (uint8_t)stored; \
        taskEXIT_CRITICAL(); \
        AuxParamApplyConfigParam(APPLY); \
        return AuxParamRecordResult(id, stored, AUX_PARAM_RESULT_OK);
#define CONFIG_PARAM_I8_RANGE(ID, NAME, SCOPE, LVALUE, MIN_V, MAX_V, APPLY) \
    case ID: \
        taskENTER_CRITICAL(); \
        (LVALUE) = (int8_t)stored; \
        taskEXIT_CRITICAL(); \
        AuxParamApplyConfigParam(APPLY); \
        return AuxParamRecordResult(id, stored, AUX_PARAM_RESULT_OK);
#define CONFIG_PARAM_BOOL(ID, NAME, SCOPE, LVALUE, APPLY) \
    case ID: \
        taskENTER_CRITICAL(); \
        (LVALUE) = (uint8_t)stored; \
        taskEXIT_CRITICAL(); \
        AuxParamApplyConfigParam(APPLY); \
        return AuxParamRecordResult(id, stored, AUX_PARAM_RESULT_OK);
#define CONFIG_PARAM_U8_MAX(ID, NAME, SCOPE, LVALUE, MAX_V, APPLY) \
    case ID: \
        taskENTER_CRITICAL(); \
        (LVALUE) = (uint8_t)stored; \
        taskEXIT_CRITICAL(); \
        AuxParamApplyConfigParam(APPLY); \
        return AuxParamRecordResult(id, stored, AUX_PARAM_RESULT_OK);
#define CONFIG_PARAM_U8_DEFAULT(ID, NAME, SCOPE, LVALUE, MAX_V, DEFAULT_V, APPLY) \
    case ID: \
        taskENTER_CRITICAL(); \
        (LVALUE) = (uint8_t)stored; \
        taskEXIT_CRITICAL(); \
        AuxParamApplyConfigParam(APPLY); \
        return AuxParamRecordResult(id, stored, AUX_PARAM_RESULT_OK);
#include "ConfigParamList.inc"
#undef CONFIG_PARAM_F32
#undef CONFIG_PARAM_F32_RANGE
#undef CONFIG_PARAM_U16
#undef CONFIG_PARAM_U8
#undef CONFIG_PARAM_I8_RANGE
#undef CONFIG_PARAM_BOOL
#undef CONFIG_PARAM_U8_MAX
#undef CONFIG_PARAM_U8_DEFAULT
    default:
        return AuxParamRecordResult(id, value, AUX_PARAM_RESULT_UNKNOWN_ID);
    }
}

AuxParamResult AuxParamValidateConfigParam(uint16_t id, fp32 value)
{
    const ConfigParamDesc *desc = AuxParamFindConfigParamById(id);
    fp32 stored = 0.0f;
    AuxParamResult result;

    if (desc == NULL)
    {
        return AuxParamRecordResult(id, value, AUX_PARAM_RESULT_UNKNOWN_ID);
    }
    result = AuxParamPrepareWrite(desc, value, &stored);
    return AuxParamRecordResult(id, value, result);
}

bool_t AuxParamSetConfigParam(uint16_t id, fp32 value)
{
    return (bool_t)((AuxParamSetConfigParamEx(id, value) == AUX_PARAM_RESULT_OK) ? 1u : 0u);
}

bool_t AuxParamGetConfigParam(uint16_t id, fp32 *out)
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
#include "ConfigParamList.inc"
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

uint16_t AuxParamGetCount(void)
{
    return (uint16_t)(sizeof(g_config_param_descs) / sizeof(g_config_param_descs[0]));
}

bool_t AuxParamGetInfoByIndex(uint16_t index, AuxParamInfo *out)
{
    fp32 min_value = 0.0f;
    fp32 max_value = 0.0f;
    const char *unit = "";
    const ConfigParamDesc *desc;
    const uint16_t count = AuxParamGetCount();

    if (out == NULL || index >= count)
    {
        return 0;
    }

    desc = &g_config_param_descs[index];
    out->id = desc->id;
    out->name = desc->name;
    out->has_range = AuxParamDescRange(desc, &min_value, &max_value, &unit);
    out->min_value = min_value;
    out->max_value = max_value;
    out->unit = unit;
    out->safe_only = AuxParamDescSafeOnly(desc);
    out->active = AuxParamConfigScopeIsActive((ConfigParamScope)desc->scope);
    return 1;
}

bool_t AuxParamGetInfo(uint16_t id, AuxParamInfo *out)
{
    const uint16_t count = AuxParamGetCount();

    if (out == NULL)
    {
        return 0;
    }

    for (uint16_t i = 0u; i < count; i++)
    {
        if (g_config_param_descs[i].id == id)
        {
            return AuxParamGetInfoByIndex(i, out);
        }
    }
    return 0;
}

AuxParamResult AuxParamGetLastResult(void)
{
    return g_aux_param_last_result;
}

uint16_t AuxParamGetLastId(void)
{
    return g_aux_param_last_id;
}

fp32 AuxParamGetLastValue(void)
{
    return g_aux_param_last_value;
}

const char *AuxParamResultName(AuxParamResult result)
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
AuxParamResult AuxParamSetConfigParamByNameEx(const char *name, fp32 value)
{
    const ConfigParamDesc *desc = AuxParamFindConfigParamByName(name);

    if (desc == NULL)
    {
        return AuxParamRecordResult(0u, value, AUX_PARAM_RESULT_UNKNOWN_ID);
    }
    return AuxParamSetConfigParamEx(desc->id, value);
}

bool_t AuxParamSetConfigParamByName(const char *name, fp32 value)
{
    return (bool_t)((AuxParamSetConfigParamByNameEx(name, value) == AUX_PARAM_RESULT_OK) ? 1u : 0u);
}
#endif
