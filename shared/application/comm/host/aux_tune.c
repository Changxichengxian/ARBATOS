/*
 * SPDX-FileCopyrightText: 2026 Chen Xuan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "aux_tune.h"

#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "aux_autotune.h"
#include "aux_param.h"
#include "aux_port.h"
#include "aux_telem.h"
#include "bsp_usart.h"
#include "chassis_control_task.h"
#include "config.h"
#include "elrs_task.h"
#include "gimbal_control_task.h"
#include "host_tune_bridge.h"
#include "image_remote_link.h"
#include "manual_input.h"
#include "robot_safety.h"
#include "robot_task_profile.h"
#include "types.h"

static char aux_rx_line[AUX_TUNE_RX_LINE_MAX];
static volatile uint16_t aux_rx_len = 0;
static char aux_cmd_line[AUX_TUNE_RX_LINE_MAX];
static volatile bool_t aux_cmd_ready = 0;
static volatile uint32_t aux_cmd_seq = 0;
static volatile bool_t aux_param_dump_active = 0;
static uint16_t aux_param_dump_index = 0u;

#define AUX_TUNE_TX_LINE_MAX 192u
#define AUX_PARAM_STAGE_MAX 16u
static char aux_tx_line[AUX_TUNE_TX_LINE_MAX];

typedef struct
{
    uint16_t id;
    fp32 value;
} aux_param_stage_entry_t;

static bool_t aux_param_stage_active = 0;
static uint8_t aux_param_stage_count = 0u;
static aux_param_stage_entry_t aux_param_stage[AUX_PARAM_STAGE_MAX];

static bool_t aux_tune_handle_line(const char *line);
static bool_t aux_tune_parse_fp32(const char *s, fp32 *out);
static bool_t aux_tune_parse_u16(const char *s, uint16_t *out);
static bool_t aux_tune_parse_u32(const char *s, uint32_t *out);
static bool_t aux_tune_try_send_param_dump(void);
static bool_t aux_tune_send_param_line(uint16_t index);
static bool_t aux_tune_send_param_last_result(void);
static bool_t aux_tune_send_param_count(void);
static bool_t aux_tune_send_text_line(const char *line);
static bool_t aux_tune_commit_param_stage(void);
static void aux_tune_format_fp32(char *out, uint16_t out_size, fp32 value);

uint32_t aux_tune_get_cmd_seq(void)
{
    return aux_cmd_seq;
}

void aux_tune_reset_rx(void)
{
    aux_rx_len = 0u;
    aux_cmd_ready = 0;
}

void aux_tune_rx_start(void)
{
    aux_rx_len = 0;
    aux_cmd_ready = 0;
    aux_cmd_seq = 0;
    aux_param_stage_active = 0;
    aux_param_stage_count = 0u;
    aux_param_dump_active = 0;
    aux_param_dump_index = 0u;
    aux_telem_reset();
    aux_autotune_reset_timing();

    bsp_aux_link_set_rx_event_cb(NULL);
    bsp_aux_link_set_rx_byte_cb(aux_tune_on_byte);
    bsp_aux_link_set_error_cb(aux_tune_on_uart_error);
    (void)bsp_aux_link_rx_it_start();
}

void aux_tune_try_send_telem(void)
{
    if (aux_tune_try_send_param_dump())
    {
        return;
    }

    if (aux_autotune_try_send_frame())
    {
        return;
    }

    aux_telem_try_send_frame();
}

static bool_t aux_tune_handle_line(const char *line)
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
        if (!aux_tune_parse_fp32(v_s, &v))
        {
            return 0;
        }

        if (aux_tune_parse_u16(key_s, &id))
        {
            if (aux_param_set_config_param_ex(id, v) == AUX_PARAM_RESULT_OK)
            {
                aux_cmd_seq++;
                return 1;
            }
            return 0;
        }

#if AUX_TUNE_ENABLE_PARAM_NAME_LOOKUP
        if (aux_param_set_config_param_by_name_ex(key_s, v) == AUX_PARAM_RESULT_OK)
        {
            aux_cmd_seq++;
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
            aux_param_stage_active = 1;
            aux_param_stage_count = 0u;
            aux_cmd_seq++;
            return 1;
        }

        if (strcmp(argv[1], "abort") == 0 || strcmp(argv[1], "cancel") == 0)
        {
            aux_param_stage_active = 0;
            aux_param_stage_count = 0u;
            aux_cmd_seq++;
            return 1;
        }

        if (strcmp(argv[1], "stage") == 0)
        {
            uint16_t id = 0u;
            fp32 value = 0.0f;

            if (argc < 4 ||
                aux_param_stage_active == 0 ||
                aux_param_stage_count >= AUX_PARAM_STAGE_MAX ||
                !aux_tune_parse_u16(argv[2], &id) ||
                !aux_tune_parse_fp32(argv[3], &value))
            {
                return 0;
            }
            if (aux_param_validate_config_param(id, value) != AUX_PARAM_RESULT_OK)
            {
                return 0;
            }
            aux_param_stage[aux_param_stage_count].id = id;
            aux_param_stage[aux_param_stage_count].value = value;
            aux_param_stage_count++;
            aux_cmd_seq++;
            return 1;
        }

        if (strcmp(argv[1], "commit") == 0)
        {
            if (!aux_tune_commit_param_stage())
            {
                return 0;
            }
            aux_cmd_seq++;
            return 1;
        }

        if (strcmp(argv[1], "count") == 0)
        {
            if (!aux_tune_send_param_count())
            {
                return 0;
            }
            aux_cmd_seq++;
            return 1;
        }

        if (strcmp(argv[1], "last") == 0)
        {
            if (!aux_tune_send_param_last_result())
            {
                return 0;
            }
            aux_cmd_seq++;
            return 1;
        }

        if (strcmp(argv[1], "dump") == 0 || strcmp(argv[1], "list") == 0)
        {
            aux_param_dump_active = 1;
            aux_param_dump_index = 0u;
            aux_cmd_seq++;
            return 1;
        }

        if (strcmp(argv[1], "get") == 0 || strcmp(argv[1], "info") == 0)
        {
            uint16_t id = 0u;
            aux_param_info_t info;
            uint16_t index = 0u;
            const uint16_t count = aux_param_get_count();

            if (argc < 3 || !aux_tune_parse_u16(argv[2], &id))
            {
                return 0;
            }
            if (!aux_param_get_info(id, &info))
            {
                return 0;
            }
            for (uint16_t i = 0u; i < count; i++)
            {
                aux_param_info_t each;
                if (aux_param_get_info_by_index(i, &each) && each.id == id)
                {
                    index = i;
                    break;
                }
            }
            if (!aux_tune_send_param_line(index))
            {
                return 0;
            }
            aux_cmd_seq++;
            return 1;
        }

        if (strcmp(argv[1], "set") == 0)
        {
            uint16_t id = 0u;
            fp32 value = 0.0f;

            if (argc < 4 ||
                !aux_tune_parse_u16(argv[2], &id) ||
                !aux_tune_parse_fp32(argv[3], &value))
            {
                return 0;
            }
            if (aux_param_set_config_param_ex(id, value) != AUX_PARAM_RESULT_OK)
            {
                return 0;
            }
            aux_cmd_seq++;
            return 1;
        }

        return 0;
    }

    if (strcmp(argv[0], "clear") == 0)
    {
        if (robot_safety_output_locked() == 0u)
        {
            return 0;
        }
        if (!robot_profile_need_single_gimbal_control_task())
        {
            return 0;
        }
        gimbal_tune_clear_pitch_pid();
        aux_cmd_seq++;
        return 1;
    }

    if (strcmp(argv[0], "view") == 0)
    {
        aux_cmd_seq++;
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

            if (!aux_port_apply_baud(baud))
            {
                return 0;
            }
            aux_cmd_seq++;
            return 1;
        }

        if (strcmp(argv[1], "baud") == 0)
        {
            uint32_t baud = 0u;
            if (!aux_tune_parse_u32(argv[2], &baud))
            {
                return 0;
            }
            if (!aux_port_apply_baud(baud))
            {
                return 0;
            }
            aux_cmd_seq++;
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
            aux_autotune_stop();
            aux_cmd_seq++;
            return 1;
        }

        if (strcmp(argv[1], "period") == 0)
        {
            uint32_t period_ms = 0u;
            if (argc < 3 || !aux_tune_parse_u32(argv[2], &period_ms))
            {
                return 0;
            }

            if (period_ms > 1000u)
            {
                period_ms = 1000u;
            }

            aux_autotune_set_period_ms(period_ms);
            aux_cmd_seq++;
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

        aux_autotune_target_e target = AUX_AUTOTUNE_TARGET_NONE;
        if (!aux_autotune_parse_target(target_s, &target))
        {
            return 0;
        }
        if (!aux_autotune_target_is_active(target))
        {
            return 0;
        }

        if (!aux_autotune_start(target))
        {
            return 0;
        }
        aux_cmd_seq++;
        return 1;
    }

    if (strcmp(argv[0], "cf") == 0)
    {
        if (robot_safety_output_locked() == 0u)
        {
            return 0;
        }
        if (!robot_profile_need_classic_chassis_control_task())
        {
            return 0;
        }
        if (argc >= 2 && strcmp(argv[1], "clear") == 0)
        {
            chassis_tune_clear_follow_pid();
            aux_cmd_seq++;
            return 1;
        }
        if (argc < 3)
        {
            return 0;
        }

        fp32 v = 0.0f;
        if (!aux_tune_parse_fp32(argv[2], &v))
        {
            return 0;
        }

        pid_param_t pid;
        chassis_tune_get_follow_pid(&pid);

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

        chassis_tune_set_follow_pid(&pid, 1);
        aux_cmd_seq++;
        return 1;
    }

    if (strcmp(argv[0], "cm") == 0)
    {
        if (robot_safety_output_locked() == 0u)
        {
            return 0;
        }
        if (!robot_profile_need_classic_chassis_control_task())
        {
            return 0;
        }
        if (argc < 3)
        {
            return 0;
        }

        fp32 v = 0.0f;
        if (!aux_tune_parse_fp32(argv[2], &v))
        {
            return 0;
        }

        pid_param_t pid;
        chassis_tune_get_motor_speed_pid(&pid);

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

        chassis_tune_set_motor_speed_pid(&pid, 1);
        aux_cmd_seq++;
        return 1;
    }

    const bool_t is_pitch_speed = (strcmp(argv[0], "ps") == 0);
    const bool_t is_pitch_angle = (strcmp(argv[0], "pa") == 0);
    const bool_t is_yaw_speed = (strcmp(argv[0], "ys") == 0);
    const bool_t is_yaw_angle = (strcmp(argv[0], "ya") == 0);
    if (is_pitch_speed || is_pitch_angle || is_yaw_speed || is_yaw_angle)
    {
        if (robot_safety_output_locked() == 0u)
        {
            return 0;
        }
        const bool_t single_gimbal_on = (bool_t)robot_profile_need_single_gimbal_control_task();
        const bool_t dual_gimbal_on = (bool_t)robot_profile_need_dual_gimbal_control_task();
        if (!single_gimbal_on && !(dual_gimbal_on && (is_yaw_speed || is_yaw_angle)))
        {
            return 0;
        }
        if (argc >= 2 && strcmp(argv[1], "clear") == 0)
        {
            if (is_pitch_speed || is_pitch_angle)
            {
                gimbal_tune_clear_pitch_pid();
            }
            else
            {
                gimbal_tune_clear_yaw_pid();
            }
            aux_cmd_seq++;
            return 1;
        }

        if (argc < 3)
        {
            return 0;
        }

        fp32 v = 0.0f;
        if (!aux_tune_parse_fp32(argv[2], &v))
        {
            return 0;
        }

        pid_param_t pid;
        if (is_pitch_speed)
        {
            gimbal_tune_get_pitch_speed_pid(&pid);
        }
        else if (is_pitch_angle)
        {
            gimbal_tune_get_pitch_angle_pid(&pid);
        }
        else if (is_yaw_speed)
        {
            gimbal_tune_get_yaw_speed_pid(&pid);
        }
        else
        {
            gimbal_tune_get_yaw_angle_pid(&pid);
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
            gimbal_tune_set_pitch_speed_pid(&pid, 1);
        }
        else if (is_pitch_angle)
        {
            gimbal_tune_set_pitch_angle_pid(&pid, 1);
        }
        else if (is_yaw_speed)
        {
            gimbal_tune_set_yaw_speed_pid(&pid, 1);
        }
        else
        {
            gimbal_tune_set_yaw_angle_pid(&pid, 1);
        }

        aux_cmd_seq++;
        return 1;
    }

    return 0;
}

static bool_t aux_tune_commit_param_stage(void)
{
    if (aux_param_stage_active == 0 || aux_param_stage_count == 0u)
    {
        return 0;
    }

    for (uint8_t i = 0u; i < aux_param_stage_count; i++)
    {
        if (aux_param_validate_config_param(aux_param_stage[i].id,
                                            aux_param_stage[i].value) != AUX_PARAM_RESULT_OK)
        {
            return 0;
        }
    }

    for (uint8_t i = 0u; i < aux_param_stage_count; i++)
    {
        if (aux_param_set_config_param_ex(aux_param_stage[i].id,
                                          aux_param_stage[i].value) != AUX_PARAM_RESULT_OK)
        {
            return 0;
        }
    }

    aux_param_stage_active = 0;
    aux_param_stage_count = 0u;
    return 1;
}

static bool_t aux_tune_send_text_line(const char *line)
{
    uint16_t len;

    if (line == NULL)
    {
        return 0;
    }
    if (!aux_port_is_tune_mode(bsp_aux_link_get_baudrate()))
    {
        return 0;
    }
    if (!bsp_aux_link_tx_ready())
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

    return (bool_t)((bsp_aux_link_tx_dma((const uint8_t *)line, len) == 0) ? 1u : 0u);
}

static void aux_tune_format_fp32(char *out, uint16_t out_size, fp32 value)
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

static bool_t aux_tune_send_param_count(void)
{
    const int n = snprintf(aux_tx_line,
                           sizeof(aux_tx_line),
                           "param_count=%u\r\n",
                           (unsigned int)aux_param_get_count());

    if (n <= 0)
    {
        return 0;
    }
    return aux_tune_send_text_line(aux_tx_line);
}

static bool_t aux_tune_send_param_last_result(void)
{
    char value_s[28];
    const aux_param_result_e result = aux_param_get_last_result();

    aux_tune_format_fp32(value_s, (uint16_t)sizeof(value_s), aux_param_get_last_value());

    const int n = snprintf(aux_tx_line,
                           sizeof(aux_tx_line),
                           "param_last id=%u value=%s result=%s\r\n",
                           (unsigned int)aux_param_get_last_id(),
                           value_s,
                           aux_param_result_name(result));

    if (n <= 0)
    {
        return 0;
    }
    return aux_tune_send_text_line(aux_tx_line);
}

static bool_t aux_tune_send_param_line(uint16_t index)
{
    aux_param_info_t info;
    fp32 value = 0.0f;
    char value_s[28];
    char min_s[28];
    char max_s[28];
    int n;

    if (!aux_param_get_info_by_index(index, &info))
    {
        return 0;
    }
    if (!aux_param_get_config_param(info.id, &value))
    {
        return 0;
    }

    aux_tune_format_fp32(value_s, (uint16_t)sizeof(value_s), value);
    if (info.has_range != 0u)
    {
        aux_tune_format_fp32(min_s, (uint16_t)sizeof(min_s), info.min_value);
        aux_tune_format_fp32(max_s, (uint16_t)sizeof(max_s), info.max_value);
    }
    else
    {
        (void)snprintf(min_s, sizeof(min_s), "*");
        (void)snprintf(max_s, sizeof(max_s), "*");
    }

    n = snprintf(aux_tx_line,
                 sizeof(aux_tx_line),
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
    aux_tx_line[sizeof(aux_tx_line) - 1u] = '\0';
    return aux_tune_send_text_line(aux_tx_line);
}

static bool_t aux_tune_try_send_param_dump(void)
{
    const uint16_t count = aux_param_get_count();
    int n;

    if (!aux_param_dump_active)
    {
        return 0;
    }
    if (!aux_port_is_tune_mode(bsp_aux_link_get_baudrate()))
    {
        aux_param_dump_active = 0;
        return 0;
    }
    if (!bsp_aux_link_tx_ready())
    {
        return 1;
    }

    if (aux_param_dump_index == 0u)
    {
        n = snprintf(aux_tx_line,
                     sizeof(aux_tx_line),
                     "param_dump begin count=%u\r\n",
                     (unsigned int)count);
        if (n <= 0 || !aux_tune_send_text_line(aux_tx_line))
        {
            return 1;
        }
        aux_param_dump_index = 1u;
        return 1;
    }

    if ((uint16_t)(aux_param_dump_index - 1u) < count)
    {
        const uint16_t index = (uint16_t)(aux_param_dump_index - 1u);
        if (aux_tune_send_param_line(index))
        {
            aux_param_dump_index++;
        }
        return 1;
    }

    n = snprintf(aux_tx_line, sizeof(aux_tx_line), "param_dump end\r\n");
    if (n > 0 && aux_tune_send_text_line(aux_tx_line))
    {
        aux_param_dump_active = 0;
        aux_param_dump_index = 0u;
    }
    return 1;
}

static bool_t aux_tune_parse_fp32(const char *s, fp32 *out)
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

static bool_t aux_tune_parse_u16(const char *s, uint16_t *out)
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

static bool_t aux_tune_parse_u32(const char *s, uint32_t *out)
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
void aux_tune_on_byte(uint8_t b)
{
    if (aux_cmd_ready)
    {
        // Drop input until the current command is processed.
    }
    else if (b == '\r' || b == '\n')
    {
        if (aux_rx_len > 0u)
        {
            const uint16_t n = (aux_rx_len >= (AUX_TUNE_RX_LINE_MAX - 1u)) ? (AUX_TUNE_RX_LINE_MAX - 1u) : aux_rx_len;
            aux_rx_line[n] = '\0';
            memcpy(aux_cmd_line, aux_rx_line, n + 1u);
            aux_cmd_ready = 1;
            aux_rx_len = 0;
        }
    }
    else if (b == 0x08u || b == 0x7Fu)
    {
        if (aux_rx_len > 0u)
        {
            aux_rx_len--;
        }
    }
    else
    {
        // Only accept printable ASCII / TAB as tuning commands. Drop binary/noise to
        // avoid accidentally changing config when using a wireless UART bridge.
        if (b == '\t' || (b >= 0x20u && b <= 0x7Eu))
        {
            if (aux_rx_len < (AUX_TUNE_RX_LINE_MAX - 1u))
            {
                aux_rx_line[aux_rx_len++] = (char)b;
            }
            else
            {
                aux_rx_len = 0;
            }
        }
        else
        {
            aux_rx_len = 0;
        }
    }
}

uint8_t aux_tune_on_uart_error(void)
{
    aux_rx_len = 0;
    return 0u;
}

void aux_tune_poll(void)
{
    if (aux_cmd_ready)
    {
        char line[AUX_TUNE_RX_LINE_MAX];
        taskENTER_CRITICAL();
        const bool_t ready = aux_cmd_ready;
        aux_cmd_ready = 0;
        if (ready)
        {
            strncpy(line, aux_cmd_line, sizeof(line) - 1u);
            line[sizeof(line) - 1u] = '\0';
        }
        else
        {
            line[0] = '\0';
        }
        taskEXIT_CRITICAL();

        if (line[0] != '\0')
        {
            (void)aux_tune_handle_line(line);
        }
    }
}
