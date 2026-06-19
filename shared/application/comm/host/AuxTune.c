/*
 * SPDX-FileCopyrightText: 2026 Chen Xuan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "AuxTune.h"

#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "AuxAutotune.h"
#include "AuxParam.h"
#include "AuxPort.h"
#include "AuxTelem.h"
#include "BspUsart.h"
#include "ChassisControlTask.h"
#include "RobotConfig.h"
#include "ElrsTask.h"
#include "GimbalControlTask.h"
#include "HostTuneBridge.h"
#include "ImageRemoteLink.h"
#include "ManualInput.h"
#include "RobotSafety.h"
#include "RobotTaskProfile.h"
#include "Types.h"

static char AuxRxLine[AUX_TUNE_RX_LINE_MAX];
static volatile uint16_t AuxRxLen = 0;
static char AuxCmdLine[AUX_TUNE_RX_LINE_MAX];
static volatile bool_t AuxCmdReady = 0;
static volatile uint32_t AuxCmdSeq = 0;
static volatile bool_t AuxParamDumpActive = 0;
static uint16_t AuxParamDumpIndex = 0u;

#define AUX_TUNE_TX_LINE_MAX 192u
#define AUX_PARAM_STAGE_MAX 16u
static char AuxTxLine[AUX_TUNE_TX_LINE_MAX];

typedef struct
{
    uint16_t id;
    fp32 value;
} AuxParamStageEntry;

static bool_t AuxParamStageActive = 0;
static uint8_t AuxParamStageCount = 0u;
static AuxParamStageEntry AuxParamStage[AUX_PARAM_STAGE_MAX];

static bool_t AuxTuneHandleLine(const char *line);
static bool_t AuxTuneParseFp32(const char *s, fp32 *out);
static bool_t AuxTuneParseU16(const char *s, uint16_t *out);
static bool_t AuxTuneParseU32(const char *s, uint32_t *out);
static bool_t AuxTuneTrySendParamDump(void);
static bool_t AuxTuneSendParamLine(uint16_t index);
static bool_t AuxTuneSendParamLastResult(void);
static bool_t AuxTuneSendParamCount(void);
static bool_t AuxTuneSendTextLine(const char *line);
static bool_t AuxTuneCommitParamStage(void);
static void AuxTuneFormatFp32(char *out, uint16_t out_size, fp32 value);

uint32_t AuxTuneGetCmdSeq(void)
{
    return AuxCmdSeq;
}

void AuxTuneResetRx(void)
{
    AuxRxLen = 0u;
    AuxCmdReady = 0;
}

void AuxTuneRxStart(void)
{
    AuxRxLen = 0;
    AuxCmdReady = 0;
    AuxCmdSeq = 0;
    AuxParamStageActive = 0;
    AuxParamStageCount = 0u;
    AuxParamDumpActive = 0;
    AuxParamDumpIndex = 0u;
    AuxTelemReset();
    AuxAutotuneResetTiming();

    BspAuxLinkSetRxEventCb(NULL);
    BspAuxLinkSetRxByteCb(AuxTuneOnByte);
    BspAuxLinkSetErrorCb(AuxTuneOnUartError);
    (void)BspAuxLinkRxItStart();
}

void AuxTuneTrySendTelem(void)
{
    if (AuxTuneTrySendParamDump())
    {
        return;
    }

    if (AuxAutotuneTrySendFrame())
    {
        return;
    }

    AuxTelemTrySendFrame();
}

static bool_t AuxTuneHandleLine(const char *line)
{
    if (line == NULL)
    {
        return 0;
    }

    // Normalize input before dispatching config and tuning commands.
    char buf[AUX_TUNE_RX_LINE_MAX];
    strncpy(buf, line, sizeof(buf) - 1u);
    buf[sizeof(buf) - 1u] = '\0';

    // to lower
    for (char *p = buf; *p != '\0'; p++)
    {
        if (*p >= 'A' && *p <= 'Z')
        {
            *p = (char)(*p - 'A' + 'a');
        }
    }

    // Fast path: "<id>:<value>" sets one config parameter.
    char *colon = strchr(buf, ':');
    if (colon != NULL)
    {
        *colon = '\0';
        char *key_s = buf;
        char *v_s = colon + 1;

        while (*key_s == ' ' || *key_s == '\t')
        {
            key_s++;
        }
        while (*v_s == ' ' || *v_s == '\t')
        {
            v_s++;
        }

        char *key_end = key_s + strlen(key_s);
        while (key_end > key_s && (key_end[-1] == ' ' || key_end[-1] == '\t'))
        {
            key_end--;
        }
        *key_end = '\0';

        char *v_end = v_s + strlen(v_s);
        while (v_end > v_s && (v_end[-1] == ' ' || v_end[-1] == '\t'))
        {
            v_end--;
        }
        *v_end = '\0';

        uint16_t id = 0;
        fp32 v = 0.0f;
        if (!AuxTuneParseFp32(v_s, &v))
        {
            return 0;
        }

        if (AuxTuneParseU16(key_s, &id))
        {
            if (AuxParamSetConfigParamEx(id, v) == AUX_PARAM_RESULT_OK)
            {
                AuxCmdSeq++;
                return 1;
            }
            return 0;
        }

#if AUX_TUNE_ENABLE_PARAM_NAME_LOOKUP
        if (AuxParamSetConfigParamByNameEx(key_s, v) == AUX_PARAM_RESULT_OK)
        {
            AuxCmdSeq++;
            return 1;
        }
#endif

        return 0;
    }

    // split tokens
    char *argv[6];
    int argc = 0;
    char *p = buf;
    while (*p != '\0' && argc < (int)(sizeof(argv) / sizeof(argv[0])))
    {
        while (*p == ' ' || *p == '\t')
        {
            p++;
        }
        if (*p == '\0')
        {
            break;
        }
        argv[argc++] = p;
        while (*p != '\0' && *p != ' ' && *p != '\t')
        {
            p++;
        }
        if (*p == '\0')
        {
            break;
        }
        *p = '\0';
        p++;
    }

    if (argc == 0)
    {
        return 0;
    }

    if (strcmp(argv[0], "param") == 0 || strcmp(argv[0], "p") == 0)
    {
        if (argc < 2)
        {
            return 0;
        }

        if (strcmp(argv[1], "begin") == 0)
        {
            AuxParamStageActive = 1;
            AuxParamStageCount = 0u;
            AuxCmdSeq++;
            return 1;
        }

        if (strcmp(argv[1], "abort") == 0 || strcmp(argv[1], "cancel") == 0)
        {
            AuxParamStageActive = 0;
            AuxParamStageCount = 0u;
            AuxCmdSeq++;
            return 1;
        }

        if (strcmp(argv[1], "stage") == 0)
        {
            uint16_t id = 0u;
            fp32 value = 0.0f;

            if (argc < 4 ||
                AuxParamStageActive == 0 ||
                AuxParamStageCount >= AUX_PARAM_STAGE_MAX ||
                !AuxTuneParseU16(argv[2], &id) ||
                !AuxTuneParseFp32(argv[3], &value))
            {
                return 0;
            }
            if (AuxParamValidateConfigParam(id, value) != AUX_PARAM_RESULT_OK)
            {
                return 0;
            }
            AuxParamStage[AuxParamStageCount].id = id;
            AuxParamStage[AuxParamStageCount].value = value;
            AuxParamStageCount++;
            AuxCmdSeq++;
            return 1;
        }

        if (strcmp(argv[1], "commit") == 0)
        {
            if (!AuxTuneCommitParamStage())
            {
                return 0;
            }
            AuxCmdSeq++;
            return 1;
        }

        if (strcmp(argv[1], "count") == 0)
        {
            if (!AuxTuneSendParamCount())
            {
                return 0;
            }
            AuxCmdSeq++;
            return 1;
        }

        if (strcmp(argv[1], "last") == 0)
        {
            if (!AuxTuneSendParamLastResult())
            {
                return 0;
            }
            AuxCmdSeq++;
            return 1;
        }

        if (strcmp(argv[1], "dump") == 0 || strcmp(argv[1], "list") == 0)
        {
            AuxParamDumpActive = 1;
            AuxParamDumpIndex = 0u;
            AuxCmdSeq++;
            return 1;
        }

        if (strcmp(argv[1], "get") == 0 || strcmp(argv[1], "info") == 0)
        {
            uint16_t id = 0u;
            AuxParamInfo info;
            uint16_t index = 0u;
            const uint16_t count = AuxParamGetCount();

            if (argc < 3 || !AuxTuneParseU16(argv[2], &id))
            {
                return 0;
            }
            if (!AuxParamGetInfo(id, &info))
            {
                return 0;
            }
            for (uint16_t i = 0u; i < count; i++)
            {
                AuxParamInfo each;
                if (AuxParamGetInfoByIndex(i, &each) && each.id == id)
                {
                    index = i;
                    break;
                }
            }
            if (!AuxTuneSendParamLine(index))
            {
                return 0;
            }
            AuxCmdSeq++;
            return 1;
        }

        if (strcmp(argv[1], "set") == 0)
        {
            uint16_t id = 0u;
            fp32 value = 0.0f;

            if (argc < 4 ||
                !AuxTuneParseU16(argv[2], &id) ||
                !AuxTuneParseFp32(argv[3], &value))
            {
                return 0;
            }
            if (AuxParamSetConfigParamEx(id, value) != AUX_PARAM_RESULT_OK)
            {
                return 0;
            }
            AuxCmdSeq++;
            return 1;
        }

        return 0;
    }

    if (strcmp(argv[0], "clear") == 0)
    {
        if (RobotSafetyOutputLocked() == 0u)
        {
            return 0;
        }
        if (!RobotProfileNeedSingleGimbalControlTask())
        {
            return 0;
        }
        GimbalTuneClearPitchPid();
        AuxCmdSeq++;
        return 1;
    }

    if (strcmp(argv[0], "view") == 0)
    {
        AuxCmdSeq++;
        return 0;
    }

    if (strcmp(argv[0], "aux") == 0 || strcmp(argv[0], "u1") == 0 || strcmp(argv[0], "uart1") == 0)
    {
        if (argc < 3)
        {
            return 0;
        }

        if (strcmp(argv[1], "mode") == 0)
        {
            uint32_t baud = 0u;
            if (strcmp(argv[2], "tune") == 0)
            {
                baud = AUX_TUNE_BAUD;
            }
            else if (strcmp(argv[2], "elrs") == 0)
            {
                baud = ELRS_LINK_BAUD;
            }
            else if (strcmp(argv[2], "image") == 0)
            {
                baud = IMAGE_REMOTE_LINK_BAUD;
            }
            else
            {
                return 0;
            }

            if (!AuxPortApplyBaud(baud))
            {
                return 0;
            }
            AuxCmdSeq++;
            return 1;
        }

        if (strcmp(argv[1], "baud") == 0)
        {
            uint32_t baud = 0u;
            if (!AuxTuneParseU32(argv[2], &baud))
            {
                return 0;
            }
            if (!AuxPortApplyBaud(baud))
            {
                return 0;
            }
            AuxCmdSeq++;
            return 1;
        }

        return 0;
    }

    if (strcmp(argv[0], "at") == 0 || strcmp(argv[0], "autotune") == 0)
    {
        if (argc < 2)
        {
            return 0;
        }

        if (strcmp(argv[1], "off") == 0 || strcmp(argv[1], "stop") == 0)
        {
            AuxAutotuneStop();
            AuxCmdSeq++;
            return 1;
        }

        if (strcmp(argv[1], "period") == 0)
        {
            uint32_t period_ms = 0u;
            if (argc < 3 || !AuxTuneParseU32(argv[2], &period_ms))
            {
                return 0;
            }

            if (period_ms > 1000u)
            {
                period_ms = 1000u;
            }

            AuxAutotuneSetPeriodMs(period_ms);
            AuxCmdSeq++;
            return 1;
        }

        const char *target_s = argv[1];
        if (strcmp(argv[1], "target") == 0)
        {
            if (argc < 3)
            {
                return 0;
            }
            target_s = argv[2];
        }

        AuxAutotuneTarget target = AUX_AUTOTUNE_TARGET_NONE;
        if (!AuxAutotuneParseTarget(target_s, &target))
        {
            return 0;
        }
        if (!AuxAutotuneTargetIsActive(target))
        {
            return 0;
        }

        if (!AuxAutotuneStart(target))
        {
            return 0;
        }
        AuxCmdSeq++;
        return 1;
    }

    if (strcmp(argv[0], "cf") == 0)
    {
        if (RobotSafetyOutputLocked() == 0u)
        {
            return 0;
        }
        if (!RobotProfileNeedClassicChassisControlTask())
        {
            return 0;
        }
        if (argc >= 2 && strcmp(argv[1], "clear") == 0)
        {
            ChassisTuneClearFollowPid();
            AuxCmdSeq++;
            return 1;
        }
        if (argc < 3)
        {
            return 0;
        }

        fp32 v = 0.0f;
        if (!AuxTuneParseFp32(argv[2], &v))
        {
            return 0;
        }

        PidParam pid;
        ChassisTuneGetFollowPid(&pid);

        if (strcmp(argv[1], "kp") == 0)
        {
            pid.kp = v;
        }
        else if (strcmp(argv[1], "ki") == 0)
        {
            pid.ki = v;
        }
        else if (strcmp(argv[1], "kd") == 0)
        {
            pid.kd = v;
        }
        else if (strcmp(argv[1], "maxout") == 0)
        {
            pid.max_out = v;
        }
        else if (strcmp(argv[1], "maxiout") == 0)
        {
            pid.max_iout = v;
        }
        else
        {
            return 0;
        }

        ChassisTuneSetFollowPid(&pid, 1);
        AuxCmdSeq++;
        return 1;
    }

    if (strcmp(argv[0], "cm") == 0)
    {
        if (RobotSafetyOutputLocked() == 0u)
        {
            return 0;
        }
        if (!RobotProfileNeedClassicChassisControlTask())
        {
            return 0;
        }
        if (argc < 3)
        {
            return 0;
        }

        fp32 v = 0.0f;
        if (!AuxTuneParseFp32(argv[2], &v))
        {
            return 0;
        }

        PidParam pid;
        ChassisTuneGetMotorSpeedPid(&pid);

        if (strcmp(argv[1], "kp") == 0)
        {
            pid.kp = v;
        }
        else if (strcmp(argv[1], "ki") == 0)
        {
            pid.ki = v;
        }
        else if (strcmp(argv[1], "kd") == 0)
        {
            pid.kd = v;
        }
        else if (strcmp(argv[1], "maxout") == 0)
        {
            pid.max_out = v;
        }
        else if (strcmp(argv[1], "maxiout") == 0)
        {
            pid.max_iout = v;
        }
        else
        {
            return 0;
        }

        ChassisTuneSetMotorSpeedPid(&pid, 1);
        AuxCmdSeq++;
        return 1;
    }

    const bool_t is_pitch_speed = (strcmp(argv[0], "ps") == 0);
    const bool_t is_pitch_angle = (strcmp(argv[0], "pa") == 0);
    const bool_t is_yaw_speed = (strcmp(argv[0], "ys") == 0);
    const bool_t is_yaw_angle = (strcmp(argv[0], "ya") == 0);
    if (is_pitch_speed || is_pitch_angle || is_yaw_speed || is_yaw_angle)
    {
        if (RobotSafetyOutputLocked() == 0u)
        {
            return 0;
        }
        const bool_t single_gimbal_on = (bool_t)RobotProfileNeedSingleGimbalControlTask();
        const bool_t dual_gimbal_on = (bool_t)RobotProfileNeedDualGimbalControlTask();
        if (!single_gimbal_on && !(dual_gimbal_on && (is_yaw_speed || is_yaw_angle)))
        {
            return 0;
        }
        if (argc >= 2 && strcmp(argv[1], "clear") == 0)
        {
            if (is_pitch_speed || is_pitch_angle)
            {
                GimbalTuneClearPitchPid();
            }
            else
            {
                GimbalTuneClearYawPid();
            }
            AuxCmdSeq++;
            return 1;
        }

        if (argc < 3)
        {
            return 0;
        }

        fp32 v = 0.0f;
        if (!AuxTuneParseFp32(argv[2], &v))
        {
            return 0;
        }

        PidParam pid;
        if (is_pitch_speed)
        {
            GimbalTuneGetPitchSpeedPid(&pid);
        }
        else if (is_pitch_angle)
        {
            GimbalTuneGetPitchAnglePid(&pid);
        }
        else if (is_yaw_speed)
        {
            GimbalTuneGetYawSpeedPid(&pid);
        }
        else
        {
            GimbalTuneGetYawAnglePid(&pid);
        }

        if (strcmp(argv[1], "kp") == 0)
        {
            pid.kp = v;
        }
        else if (strcmp(argv[1], "ki") == 0)
        {
            pid.ki = v;
        }
        else if (strcmp(argv[1], "kd") == 0)
        {
            pid.kd = v;
        }
        else if (strcmp(argv[1], "maxout") == 0)
        {
            pid.max_out = v;
        }
        else if (strcmp(argv[1], "maxiout") == 0)
        {
            pid.max_iout = v;
        }
        else
        {
            return 0;
        }

        if (is_pitch_speed)
        {
            GimbalTuneSetPitchSpeedPid(&pid, 1);
        }
        else if (is_pitch_angle)
        {
            GimbalTuneSetPitchAnglePid(&pid, 1);
        }
        else if (is_yaw_speed)
        {
            GimbalTuneSetYawSpeedPid(&pid, 1);
        }
        else
        {
            GimbalTuneSetYawAnglePid(&pid, 1);
        }

        AuxCmdSeq++;
        return 1;
    }

    return 0;
}

static bool_t AuxTuneCommitParamStage(void)
{
    if (AuxParamStageActive == 0 || AuxParamStageCount == 0u)
    {
        return 0;
    }

    for (uint8_t i = 0u; i < AuxParamStageCount; i++)
    {
        if (AuxParamValidateConfigParam(AuxParamStage[i].id,
                                            AuxParamStage[i].value) != AUX_PARAM_RESULT_OK)
        {
            return 0;
        }
    }

    for (uint8_t i = 0u; i < AuxParamStageCount; i++)
    {
        if (AuxParamSetConfigParamEx(AuxParamStage[i].id,
                                          AuxParamStage[i].value) != AUX_PARAM_RESULT_OK)
        {
            return 0;
        }
    }

    AuxParamStageActive = 0;
    AuxParamStageCount = 0u;
    return 1;
}

static bool_t AuxTuneSendTextLine(const char *line)
{
    uint16_t len;

    if (line == NULL)
    {
        return 0;
    }
    if (!AuxPortIsTuneMode(BspAuxLinkGetBaudrate()))
    {
        return 0;
    }
    if (!BspAuxLinkTxReady())
    {
        return 0;
    }

    len = (uint16_t)strlen(line);
    if (len == 0u)
    {
        return 0;
    }
    if (len > (uint16_t)(AUX_TUNE_TX_LINE_MAX - 1u))
    {
        len = (uint16_t)(AUX_TUNE_TX_LINE_MAX - 1u);
    }

    return (bool_t)((BspAuxLinkTxDma((const uint8_t *)line, len) == 0) ? 1u : 0u);
}

static void AuxTuneFormatFp32(char *out, uint16_t out_size, fp32 value)
{
    uint8_t negative = 0u;
    int32_t whole;
    fp32 frac_f;
    uint32_t frac;

    if (out == NULL || out_size == 0u)
    {
        return;
    }

    if (value != value)
    {
        (void)snprintf(out, out_size, "nan");
        return;
    }

    if (value < 0.0f)
    {
        negative = 1u;
        value = -value;
    }

    whole = (int32_t)value;
    frac_f = value - (fp32)whole;
    frac = (uint32_t)(frac_f * 1000000.0f + 0.5f);
    if (frac >= 1000000u)
    {
        whole++;
        frac -= 1000000u;
    }

    if (negative != 0u)
    {
        (void)snprintf(out, out_size, "-%ld.%06lu", (long)whole, (unsigned long)frac);
    }
    else
    {
        (void)snprintf(out, out_size, "%ld.%06lu", (long)whole, (unsigned long)frac);
    }
}

static bool_t AuxTuneSendParamCount(void)
{
    const int n = snprintf(AuxTxLine,
                           sizeof(AuxTxLine),
                           "param_count=%u\r\n",
                           (unsigned int)AuxParamGetCount());

    if (n <= 0)
    {
        return 0;
    }
    return AuxTuneSendTextLine(AuxTxLine);
}

static bool_t AuxTuneSendParamLastResult(void)
{
    char value_s[28];
    const AuxParamResult result = AuxParamGetLastResult();

    AuxTuneFormatFp32(value_s, (uint16_t)sizeof(value_s), AuxParamGetLastValue());

    const int n = snprintf(AuxTxLine,
                           sizeof(AuxTxLine),
                           "param_last id=%u value=%s result=%s\r\n",
                           (unsigned int)AuxParamGetLastId(),
                           value_s,
                           AuxParamResultName(result));

    if (n <= 0)
    {
        return 0;
    }
    return AuxTuneSendTextLine(AuxTxLine);
}

static bool_t AuxTuneSendParamLine(uint16_t index)
{
    AuxParamInfo info;
    fp32 value = 0.0f;
    char value_s[28];
    char min_s[28];
    char max_s[28];
    int n;

    if (!AuxParamGetInfoByIndex(index, &info))
    {
        return 0;
    }
    if (!AuxParamGetConfigParam(info.id, &value))
    {
        return 0;
    }

    AuxTuneFormatFp32(value_s, (uint16_t)sizeof(value_s), value);
    if (info.has_range != 0u)
    {
        AuxTuneFormatFp32(min_s, (uint16_t)sizeof(min_s), info.min_value);
        AuxTuneFormatFp32(max_s, (uint16_t)sizeof(max_s), info.max_value);
    }
    else
    {
        (void)snprintf(min_s, sizeof(min_s), "*");
        (void)snprintf(max_s, sizeof(max_s), "*");
    }

    n = snprintf(AuxTxLine,
                 sizeof(AuxTxLine),
                 "param id=%u value=%s range=%s..%s unit=%s safe=%u active=%u name=%s\r\n",
                 (unsigned int)info.id,
                 value_s,
                 min_s,
                 max_s,
                 (info.unit != NULL) ? info.unit : "",
                 (unsigned int)info.safe_only,
                 (unsigned int)info.active,
                 info.name);

    if (n <= 0)
    {
        return 0;
    }
    AuxTxLine[sizeof(AuxTxLine) - 1u] = '\0';
    return AuxTuneSendTextLine(AuxTxLine);
}

static bool_t AuxTuneTrySendParamDump(void)
{
    const uint16_t count = AuxParamGetCount();
    int n;

    if (!AuxParamDumpActive)
    {
        return 0;
    }
    if (!AuxPortIsTuneMode(BspAuxLinkGetBaudrate()))
    {
        AuxParamDumpActive = 0;
        return 0;
    }
    if (!BspAuxLinkTxReady())
    {
        return 1;
    }

    if (AuxParamDumpIndex == 0u)
    {
        n = snprintf(AuxTxLine,
                     sizeof(AuxTxLine),
                     "param_dump begin count=%u\r\n",
                     (unsigned int)count);
        if (n <= 0 || !AuxTuneSendTextLine(AuxTxLine))
        {
            return 1;
        }
        AuxParamDumpIndex = 1u;
        return 1;
    }

    if ((uint16_t)(AuxParamDumpIndex - 1u) < count)
    {
        const uint16_t index = (uint16_t)(AuxParamDumpIndex - 1u);
        if (AuxTuneSendParamLine(index))
        {
            AuxParamDumpIndex++;
        }
        return 1;
    }

    n = snprintf(AuxTxLine, sizeof(AuxTxLine), "param_dump end\r\n");
    if (n > 0 && AuxTuneSendTextLine(AuxTxLine))
    {
        AuxParamDumpActive = 0;
        AuxParamDumpIndex = 0u;
    }
    return 1;
}

static bool_t AuxTuneParseFp32(const char *s, fp32 *out)
{
    if (s == NULL || out == NULL)
    {
        return 0;
    }

    int sign = 1;
    if (*s == '+')
    {
        s++;
    }
    else if (*s == '-')
    {
        sign = -1;
        s++;
    }

    bool_t has_digit = 0;
    int32_t int_part = 0;
    while (*s >= '0' && *s <= '9')
    {
        has_digit = 1;
        int_part = int_part * 10 + (*s - '0');
        s++;
    }

    fp32 value = (fp32)int_part;
    if (*s == '.')
    {
        s++;
        fp32 base = 0.1f;
        while (*s >= '0' && *s <= '9')
        {
            has_digit = 1;
            value += (fp32)(*s - '0') * base;
            base *= 0.1f;
            s++;
        }
    }

    if (!has_digit)
    {
        return 0;
    }

    *out = (fp32)sign * value;
    return 1;
}

static bool_t AuxTuneParseU16(const char *s, uint16_t *out)
{
    if (s == NULL || out == NULL)
    {
        return 0;
    }

    if (*s == '\0')
    {
        return 0;
    }

    uint32_t v = 0;
    bool_t has_digit = 0;
    while (*s >= '0' && *s <= '9')
    {
        has_digit = 1;
        v = v * 10u + (uint32_t)(*s - '0');
        if (v > 65535u)
        {
            v = 65535u;
        }
        s++;
    }

    if (!has_digit || *s != '\0')
    {
        return 0;
    }

    *out = (uint16_t)v;
    return 1;
}

static bool_t AuxTuneParseU32(const char *s, uint32_t *out)
{
    if (s == NULL || out == NULL)
    {
        return 0;
    }

    if (*s == '\0')
    {
        return 0;
    }

    uint32_t v = 0u;
    bool_t has_digit = 0;
    while (*s >= '0' && *s <= '9')
    {
        const uint32_t digit = (uint32_t)(*s - '0');
        has_digit = 1;
        if (v > ((0xFFFFFFFFu - digit) / 10u))
        {
            v = 0xFFFFFFFFu;
        }
        else
        {
            v = v * 10u + digit;
        }
        s++;
    }

    if (!has_digit || *s != '\0')
    {
        return 0;
    }

    *out = v;
    return 1;
}
void AuxTuneOnByte(uint8_t b)
{
    if (AuxCmdReady)
    {
        // Drop input until the current command is processed.
    }
    else if (b == '\r' || b == '\n')
    {
        if (AuxRxLen > 0u)
        {
            const uint16_t n = (AuxRxLen >= (AUX_TUNE_RX_LINE_MAX - 1u)) ? (AUX_TUNE_RX_LINE_MAX - 1u) : AuxRxLen;
            AuxRxLine[n] = '\0';
            memcpy(AuxCmdLine, AuxRxLine, n + 1u);
            AuxCmdReady = 1;
            AuxRxLen = 0;
        }
    }
    else if (b == 0x08u || b == 0x7Fu)
    {
        if (AuxRxLen > 0u)
        {
            AuxRxLen--;
        }
    }
    else
    {
        // Only accept printable ASCII / TAB as tuning commands. Drop binary/noise to
        // avoid accidentally changing config when using a wireless UART bridge.
        if (b == '\t' || (b >= 0x20u && b <= 0x7Eu))
        {
            if (AuxRxLen < (AUX_TUNE_RX_LINE_MAX - 1u))
            {
                AuxRxLine[AuxRxLen++] = (char)b;
            }
            else
            {
                AuxRxLen = 0;
            }
        }
        else
        {
            AuxRxLen = 0;
        }
    }
}

uint8_t AuxTuneOnUartError(void)
{
    AuxRxLen = 0;
    return 0u;
}

void AuxTunePoll(void)
{
    if (AuxCmdReady)
    {
        char line[AUX_TUNE_RX_LINE_MAX];
        taskENTER_CRITICAL();
        const bool_t ready = AuxCmdReady;
        AuxCmdReady = 0;
        if (ready)
        {
            strncpy(line, AuxCmdLine, sizeof(line) - 1u);
            line[sizeof(line) - 1u] = '\0';
        }
        else
        {
            line[0] = '\0';
        }
        taskEXIT_CRITICAL();

        if (line[0] != '\0')
        {
            (void)AuxTuneHandleLine(line);
        }
    }
}
