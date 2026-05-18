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
 * - 前段：裁判系统全局数据、UI 绘图/字符串打包和发送接口。
 * - 中段：UI demo、图形字段编码、客户端 ID 推导。
 * - 后段：referee_data_solve() 解析裁判帧，getter 提供功率/热量/机器人状态。
 * - 发送：referee_send_frame() 统一补帧头、CRC，并交给 USART。
 */

#include "referee.h"
#include "string.h"
#include "stdio.h"
#include "CRC8_CRC16.h"
#include "referee_protocol.h"
#include "bsp_usart.h"
#include "sdlog.h"

#define REFEREE_UI_DEMO_LAYER            7u
#define REFEREE_UI_DEMO_INIT_GAP_MS      120u
#define REFEREE_UI_DEMO_TEXT_PERIOD_MS   500u


frame_header_struct_t referee_receive_header;
frame_header_struct_t referee_send_header;

ext_game_state_t game_state;
ext_game_result_t game_result;
ext_game_robot_HP_t game_robot_HP_t;

ext_event_data_t field_event;
ext_referee_warning_t referee_warning_t;
ext_dart_info_t dart_info_t;

ext_robot_status_t robot_state;
ext_power_heat_data_t power_heat_data_t;
ext_robot_pos_t game_robot_pos_t;
ext_buff_data_t buff_musk_t;
ext_robot_hurt_t robot_hurt_t;
ext_shoot_data_t shoot_data_t;
ext_projectile_allowance_t bullet_remaining_t;
ext_rfid_status_t rfid_status_t;
ext_dart_client_cmd_t dart_client_cmd_t;
ext_ground_robot_position_t ground_robot_position_t;
ext_radar_mark_data_t radar_mark_data_t;
ext_sentry_info_t sentry_info_t;
ext_radar_info_t radar_info_t;
ext_robot_interactive_data_t student_interactive_data_t;

static uint16_t student_interactive_data_len = 0u;
static uint8_t referee_tx_seq = 0u;
static uint8_t referee_tx_frame_buf[REF_PROTOCOL_FRAME_MAX_SIZE];

static void referee_parse_student_interactive(const uint8_t *payload, uint16_t payload_len);
static int referee_send_frame(uint16_t cmd_id, const uint8_t *payload, uint16_t payload_len);
static void referee_copy_payload(void *dst, uint16_t dst_size, const uint8_t *payload, uint16_t payload_len);
static void referee_ui_store_u32_le(uint8_t *dst, uint32_t value);
static void referee_ui_pack_graphic(const referee_ui_graphic_t *graphic, uint8_t out[REFEREE_UI_GRAPHIC_RAW_LEN]);
static uint16_t referee_ui_count_to_data_cmd_id(uint8_t count);
static void referee_ui_set_name(uint8_t dst[3], const char name[3]);
static void referee_ui_init_graphic(referee_ui_graphic_t *graphic,
                                    const char name[3],
                                    uint8_t operation,
                                    uint8_t type,
                                    uint8_t layer,
                                    uint8_t color,
                                    uint16_t width,
                                    uint16_t start_x,
                                    uint16_t start_y);
static void referee_ui_set_number_bits(referee_ui_graphic_t *graphic, int32_t value);
static uint8_t referee_ui_utf8_to_utf16le(const char *text_utf8,
                                          uint8_t out_utf16[REFEREE_UI_STRING_UTF16_BYTES],
                                          uint16_t *out_code_units);

uint16_t referee_get_client_id_by_robot_id(uint16_t robot_id)
{
    if ((robot_id >= (uint16_t)RED_HERO && robot_id <= (uint16_t)RED_AERIAL) ||
        (robot_id >= (uint16_t)BLUE_HERO && robot_id <= (uint16_t)BLUE_AERIAL))
    {
        return (uint16_t)(0x0100u + robot_id);
    }

    return 0u;
}

uint16_t referee_get_self_client_id(void)
{
    return referee_get_client_id_by_robot_id((uint16_t)get_robot_id());
}

void referee_ui_make_line(referee_ui_graphic_t *graphic,
                          const char name[3],
                          uint8_t operation,
                          uint8_t layer,
                          uint8_t color,
                          uint16_t width,
                          uint16_t start_x,
                          uint16_t start_y,
                          uint16_t end_x,
                          uint16_t end_y)
{
    if (graphic == NULL)
    {
        return;
    }

    referee_ui_init_graphic(graphic, name, operation, REFEREE_UI_LINE, layer, color, width, start_x, start_y);
    graphic->details_d = end_x;
    graphic->details_e = end_y;
}

void referee_ui_make_rect(referee_ui_graphic_t *graphic,
                          const char name[3],
                          uint8_t operation,
                          uint8_t layer,
                          uint8_t color,
                          uint16_t width,
                          uint16_t start_x,
                          uint16_t start_y,
                          uint16_t end_x,
                          uint16_t end_y)
{
    if (graphic == NULL)
    {
        return;
    }

    referee_ui_init_graphic(graphic, name, operation, REFEREE_UI_RECT, layer, color, width, start_x, start_y);
    graphic->details_d = end_x;
    graphic->details_e = end_y;
}

void referee_ui_make_circle(referee_ui_graphic_t *graphic,
                            const char name[3],
                            uint8_t operation,
                            uint8_t layer,
                            uint8_t color,
                            uint16_t width,
                            uint16_t center_x,
                            uint16_t center_y,
                            uint16_t radius)
{
    if (graphic == NULL)
    {
        return;
    }

    referee_ui_init_graphic(graphic, name, operation, REFEREE_UI_CIRCLE, layer, color, width, center_x, center_y);
    graphic->details_c = radius;
}

void referee_ui_make_ellipse(referee_ui_graphic_t *graphic,
                             const char name[3],
                             uint8_t operation,
                             uint8_t layer,
                             uint8_t color,
                             uint16_t width,
                             uint16_t center_x,
                             uint16_t center_y,
                             uint16_t x_half_axis,
                             uint16_t y_half_axis)
{
    if (graphic == NULL)
    {
        return;
    }

    referee_ui_init_graphic(graphic, name, operation, REFEREE_UI_ELLIPSE, layer, color, width, center_x, center_y);
    graphic->details_d = x_half_axis;
    graphic->details_e = y_half_axis;
}

void referee_ui_make_arc(referee_ui_graphic_t *graphic,
                         const char name[3],
                         uint8_t operation,
                         uint8_t layer,
                         uint8_t color,
                         uint16_t width,
                         uint16_t center_x,
                         uint16_t center_y,
                         uint16_t start_angle_deg,
                         uint16_t end_angle_deg,
                         uint16_t x_half_axis,
                         uint16_t y_half_axis)
{
    if (graphic == NULL)
    {
        return;
    }

    referee_ui_init_graphic(graphic, name, operation, REFEREE_UI_ARC, layer, color, width, center_x, center_y);
    graphic->details_a = start_angle_deg;
    graphic->details_b = end_angle_deg;
    graphic->details_d = x_half_axis;
    graphic->details_e = y_half_axis;
}

void referee_ui_make_int(referee_ui_graphic_t *graphic,
                         const char name[3],
                         uint8_t operation,
                         uint8_t layer,
                         uint8_t color,
                         uint16_t width,
                         uint16_t start_x,
                         uint16_t start_y,
                         uint16_t font_size,
                         int32_t value)
{
    if (graphic == NULL)
    {
        return;
    }

    referee_ui_init_graphic(graphic, name, operation, REFEREE_UI_INT, layer, color, width, start_x, start_y);
    graphic->details_a = font_size;
    referee_ui_set_number_bits(graphic, value);
}

void referee_ui_make_float(referee_ui_graphic_t *graphic,
                           const char name[3],
                           uint8_t operation,
                           uint8_t layer,
                           uint8_t color,
                           uint16_t width,
                           uint16_t start_x,
                           uint16_t start_y,
                           uint16_t font_size,
                           fp32 value)
{
    fp32 scaled_value = value * 1000.0f;
    int32_t int_value = 0;

    if (graphic == NULL)
    {
        return;
    }

    if (scaled_value >= 2147483647.0f)
    {
        int_value = INT32_MAX;
    }
    else if (scaled_value <= -2147483648.0f)
    {
        int_value = INT32_MIN;
    }
    else if (scaled_value >= 0.0f)
    {
        int_value = (int32_t)(scaled_value + 0.5f);
    }
    else
    {
        int_value = (int32_t)(scaled_value - 0.5f);
    }

    referee_ui_init_graphic(graphic, name, operation, REFEREE_UI_FLOAT, layer, color, width, start_x, start_y);
    graphic->details_a = font_size;
    referee_ui_set_number_bits(graphic, int_value);
}

void referee_ui_make_char(referee_ui_graphic_t *graphic,
                          const char name[3],
                          uint8_t operation,
                          uint8_t layer,
                          uint8_t color,
                          uint16_t width,
                          uint16_t start_x,
                          uint16_t start_y,
                          uint16_t font_size)
{
    if (graphic == NULL)
    {
        return;
    }

    referee_ui_init_graphic(graphic, name, operation, REFEREE_UI_CHAR, layer, color, width, start_x, start_y);
    graphic->details_a = font_size;
}

int referee_ui_delete(uint16_t receiver_id, uint8_t delete_type, uint8_t layer)
{
    uint8_t payload[2] = {0};

    if (receiver_id == 0u || delete_type > (uint8_t)REFEREE_UI_DELETE_ALL)
    {
        return -1;
    }

    payload[0] = delete_type;
    payload[1] = layer;
    return referee_send_student_interactive((uint16_t)REFEREE_UI_DATA_DELETE, receiver_id, payload, (uint16_t)sizeof(payload));
}

int referee_ui_delete_self(uint8_t delete_type, uint8_t layer)
{
    return referee_ui_delete(referee_get_self_client_id(), delete_type, layer);
}

int referee_ui_send_graphics(uint16_t receiver_id, const referee_ui_graphic_t *graphics, uint8_t count)
{
    uint8_t payload[REFEREE_UI_GRAPHIC_RAW_LEN * 7u] = {0};
    uint16_t data_cmd_id = referee_ui_count_to_data_cmd_id(count);
    uint8_t i = 0u;

    if (receiver_id == 0u || graphics == NULL || data_cmd_id == 0u)
    {
        return -1;
    }

    for (i = 0u; i < count; i++)
    {
        referee_ui_pack_graphic(&graphics[i], &payload[(uint16_t)i * REFEREE_UI_GRAPHIC_RAW_LEN]);
    }

    return referee_send_student_interactive(data_cmd_id,
                                            receiver_id,
                                            payload,
                                            (uint16_t)count * REFEREE_UI_GRAPHIC_RAW_LEN);
}

int referee_ui_send_graphics_self(const referee_ui_graphic_t *graphics, uint8_t count)
{
    return referee_ui_send_graphics(referee_get_self_client_id(), graphics, count);
}

int referee_ui_send_string_utf8(uint16_t receiver_id, const referee_ui_graphic_t *graphic, const char *text_utf8)
{
    referee_ui_graphic_t graphic_copy;
    uint8_t payload[REFEREE_UI_GRAPHIC_RAW_LEN + REFEREE_UI_STRING_UTF16_BYTES] = {0};
    uint16_t code_units = 0u;

    if (receiver_id == 0u || graphic == NULL || text_utf8 == NULL)
    {
        return -1;
    }

    memcpy(&graphic_copy, graphic, sizeof(graphic_copy));
    graphic_copy.type = (uint8_t)REFEREE_UI_CHAR;
    (void)referee_ui_utf8_to_utf16le(text_utf8, &payload[REFEREE_UI_GRAPHIC_RAW_LEN], &code_units);
    graphic_copy.details_b = code_units;
    referee_ui_pack_graphic(&graphic_copy, payload);

    return referee_send_student_interactive((uint16_t)REFEREE_UI_DATA_CHAR,
                                            receiver_id,
                                            payload,
                                            (uint16_t)sizeof(payload));
}

int referee_ui_send_string_utf8_self(const referee_ui_graphic_t *graphic, const char *text_utf8)
{
    return referee_ui_send_string_utf8(referee_get_self_client_id(), graphic, text_utf8);
}

void referee_ui_demo_tick(void)
{
    // 函数地图：客户端 ID 变更就重置；按阶段先清层，再分批发送图形和文字。
    static uint8_t stage = 0u;
    static uint16_t last_client_id = 0u;
    static uint32_t last_send_ms = 0u;
    static referee_ui_graphic_t demo_graphics[5];
    static referee_ui_graphic_t demo_text_graphic;
    uint16_t client_id = referee_get_self_client_id();
    uint32_t now_ms = HAL_GetTick();
    char text[32] = {0};

    if (client_id == 0u)
    {
        stage = 0u;
        last_client_id = 0u;
        last_send_ms = 0u;
        return;
    }

    if (client_id != last_client_id)
    {
        stage = 0u;
        last_client_id = client_id;
        last_send_ms = 0u;
    }

    if (referee_tx_ready() == 0u)
    {
        return;
    }

    switch (stage)
    {
        case 0:
        {
            if (referee_ui_delete(client_id, (uint8_t)REFEREE_UI_DELETE_LAYER, REFEREE_UI_DEMO_LAYER) >= 0)
            {
                last_send_ms = now_ms;
                stage = 1u;
            }
        }
        break;

        case 1:
        {
            if ((uint32_t)(now_ms - last_send_ms) < REFEREE_UI_DEMO_INIT_GAP_MS)
            {
                break;
            }

            referee_ui_make_rect(&demo_graphics[0], "U0A", (uint8_t)REFEREE_UI_OP_ADD, REFEREE_UI_DEMO_LAYER,
                                 (uint8_t)REFEREE_UI_COLOR_GREEN, 3u, 860u, 460u, 1060u, 620u);
            referee_ui_make_line(&demo_graphics[1], "U0B", (uint8_t)REFEREE_UI_OP_ADD, REFEREE_UI_DEMO_LAYER,
                                 (uint8_t)REFEREE_UI_COLOR_CYAN, 2u, 900u, 540u, 1020u, 540u);
            referee_ui_make_line(&demo_graphics[2], "U0C", (uint8_t)REFEREE_UI_OP_ADD, REFEREE_UI_DEMO_LAYER,
                                 (uint8_t)REFEREE_UI_COLOR_CYAN, 2u, 960u, 480u, 960u, 600u);
            referee_ui_make_circle(&demo_graphics[3], "U0D", (uint8_t)REFEREE_UI_OP_ADD, REFEREE_UI_DEMO_LAYER,
                                   (uint8_t)REFEREE_UI_COLOR_YELLOW, 2u, 960u, 540u, 80u);
            referee_ui_make_line(&demo_graphics[4], "U0E", (uint8_t)REFEREE_UI_OP_ADD, REFEREE_UI_DEMO_LAYER,
                                 (uint8_t)REFEREE_UI_COLOR_ORANGE, 2u, 900u, 500u, 1020u, 580u);

            if (referee_ui_send_graphics(client_id, demo_graphics, 5u) >= 0)
            {
                last_send_ms = now_ms;
                stage = 2u;
            }
        }
        break;

        case 2:
        {
            if ((uint32_t)(now_ms - last_send_ms) < REFEREE_UI_DEMO_INIT_GAP_MS)
            {
                break;
            }

            referee_ui_make_char(&demo_text_graphic, "TXT", (uint8_t)REFEREE_UI_OP_ADD, REFEREE_UI_DEMO_LAYER,
                                 (uint8_t)REFEREE_UI_COLOR_WHITE, 2u, 780u, 680u, 20u);
            snprintf(text,
                     sizeof(text),
                     "B%u H%u/%u",
                     bullet_remaining_t.projectile_allowance_17mm,
                     power_heat_data_t.shooter_17mm_barrel_heat,
                     robot_state.shooter_barrel_heat_limit);

            if (referee_ui_send_string_utf8(client_id, &demo_text_graphic, text) >= 0)
            {
                last_send_ms = now_ms;
                stage = 3u;
            }
        }
        break;

        default:
        {
            if ((uint32_t)(now_ms - last_send_ms) < REFEREE_UI_DEMO_TEXT_PERIOD_MS)
            {
                break;
            }

            referee_ui_make_char(&demo_text_graphic, "TXT", (uint8_t)REFEREE_UI_OP_MODIFY, REFEREE_UI_DEMO_LAYER,
                                 (uint8_t)REFEREE_UI_COLOR_WHITE, 2u, 780u, 680u, 20u);
            snprintf(text,
                     sizeof(text),
                     "B%u H%u/%u",
                     bullet_remaining_t.projectile_allowance_17mm,
                     power_heat_data_t.shooter_17mm_barrel_heat,
                     robot_state.shooter_barrel_heat_limit);

            if (referee_ui_send_string_utf8(client_id, &demo_text_graphic, text) >= 0)
            {
                last_send_ms = now_ms;
            }
        }
        break;
    }
}

static void referee_ui_store_u32_le(uint8_t *dst, uint32_t value)
{
    if (dst == NULL)
    {
        return;
    }

    dst[0] = (uint8_t)(value & 0xFFu);
    dst[1] = (uint8_t)((value >> 8) & 0xFFu);
    dst[2] = (uint8_t)((value >> 16) & 0xFFu);
    dst[3] = (uint8_t)((value >> 24) & 0xFFu);
}

static void referee_ui_pack_graphic(const referee_ui_graphic_t *graphic, uint8_t out[REFEREE_UI_GRAPHIC_RAW_LEN])
{
    uint32_t config1 = 0u;
    uint32_t config2 = 0u;
    uint32_t config3 = 0u;

    if (graphic == NULL || out == NULL)
    {
        return;
    }

    memcpy(out, graphic->name, 3u);

    config1 |= ((uint32_t)graphic->operation & 0x7u);
    config1 |= (((uint32_t)graphic->type & 0x7u) << 3);
    config1 |= (((uint32_t)graphic->layer & 0xFu) << 6);
    config1 |= (((uint32_t)graphic->color & 0xFu) << 10);
    config1 |= (((uint32_t)graphic->details_a & 0x1FFu) << 14);
    config1 |= (((uint32_t)graphic->details_b & 0x1FFu) << 23);

    config2 |= ((uint32_t)graphic->width & 0x3FFu);
    config2 |= (((uint32_t)graphic->start_x & 0x7FFu) << 10);
    config2 |= (((uint32_t)graphic->start_y & 0x7FFu) << 21);

    config3 |= ((uint32_t)graphic->details_c & 0x3FFu);
    config3 |= (((uint32_t)graphic->details_d & 0x7FFu) << 10);
    config3 |= (((uint32_t)graphic->details_e & 0x7FFu) << 21);

    referee_ui_store_u32_le(&out[3], config1);
    referee_ui_store_u32_le(&out[7], config2);
    referee_ui_store_u32_le(&out[11], config3);
}

static uint16_t referee_ui_count_to_data_cmd_id(uint8_t count)
{
    switch (count)
    {
        case 1u:
            return (uint16_t)REFEREE_UI_DATA_SINGLE;
        case 2u:
            return (uint16_t)REFEREE_UI_DATA_DOUBLE;
        case 5u:
            return (uint16_t)REFEREE_UI_DATA_FIVE;
        case 7u:
            return (uint16_t)REFEREE_UI_DATA_SEVEN;
        default:
            return 0u;
    }
}

static void referee_ui_set_name(uint8_t dst[3], const char name[3])
{
    memset(dst, 0, 3u);
    if (name != NULL)
    {
        memcpy(dst, name, 3u);
    }
}

static void referee_ui_init_graphic(referee_ui_graphic_t *graphic,
                                    const char name[3],
                                    uint8_t operation,
                                    uint8_t type,
                                    uint8_t layer,
                                    uint8_t color,
                                    uint16_t width,
                                    uint16_t start_x,
                                    uint16_t start_y)
{
    if (graphic == NULL)
    {
        return;
    }

    memset(graphic, 0, sizeof(*graphic));
    referee_ui_set_name(graphic->name, name);
    graphic->operation = operation;
    graphic->type = type;
    graphic->layer = layer;
    graphic->color = color;
    graphic->width = width;
    graphic->start_x = start_x;
    graphic->start_y = start_y;
}

static void referee_ui_set_number_bits(referee_ui_graphic_t *graphic, int32_t value)
{
    uint32_t raw_value = (uint32_t)value;

    if (graphic == NULL)
    {
        return;
    }

    graphic->details_c = (uint16_t)(raw_value & 0x3FFu);
    graphic->details_d = (uint16_t)((raw_value >> 10) & 0x7FFu);
    graphic->details_e = (uint16_t)((raw_value >> 21) & 0x7FFu);
}

static uint8_t referee_ui_utf8_to_utf16le(const char *text_utf8,
                                          uint8_t out_utf16[REFEREE_UI_STRING_UTF16_BYTES],
                                          uint16_t *out_code_units)
{
    const uint8_t *src = (const uint8_t *)text_utf8;
    uint8_t bytes_written = 0u;
    uint16_t code_units = 0u;

    if (out_code_units != NULL)
    {
        *out_code_units = 0u;
    }

    if (text_utf8 == NULL || out_utf16 == NULL)
    {
        return 0u;
    }

    memset(out_utf16, 0, REFEREE_UI_STRING_UTF16_BYTES);

    while (*src != 0u && (uint16_t)(bytes_written + 1u) < REFEREE_UI_STRING_UTF16_BYTES)
    {
        uint32_t codepoint = 0u;
        uint8_t advance = 1u;

        if (src[0] < 0x80u)
        {
            codepoint = src[0];
        }
        else if (src[1] != 0u &&
                 (src[0] & 0xE0u) == 0xC0u &&
                 (src[1] & 0xC0u) == 0x80u)
        {
            codepoint = (uint32_t)(src[0] & 0x1Fu) << 6;
            codepoint |= (uint32_t)(src[1] & 0x3Fu);
            advance = 2u;
            if (codepoint < 0x80u)
            {
                codepoint = '?';
                advance = 1u;
            }
        }
        else if (src[1] != 0u &&
                 src[2] != 0u &&
                 (src[0] & 0xF0u) == 0xE0u &&
                 (src[1] & 0xC0u) == 0x80u &&
                 (src[2] & 0xC0u) == 0x80u)
        {
            codepoint = (uint32_t)(src[0] & 0x0Fu) << 12;
            codepoint |= (uint32_t)(src[1] & 0x3Fu) << 6;
            codepoint |= (uint32_t)(src[2] & 0x3Fu);
            advance = 3u;
            if (codepoint < 0x800u)
            {
                codepoint = '?';
                advance = 1u;
            }
        }
        else
        {
            codepoint = '?';
            if (src[1] != 0u &&
                src[2] != 0u &&
                src[3] != 0u &&
                (src[0] & 0xF8u) == 0xF0u)
            {
                advance = 4u;
            }
        }

        if (codepoint > 0xFFFFu || (codepoint >= 0xD800u && codepoint <= 0xDFFFu))
        {
            codepoint = '?';
        }

        out_utf16[bytes_written++] = (uint8_t)(codepoint & 0xFFu);
        out_utf16[bytes_written++] = (uint8_t)((codepoint >> 8) & 0xFFu);
        code_units++;
        src += advance;
    }

    if (out_code_units != NULL)
    {
        *out_code_units = code_units;
    }

    return bytes_written;
}




void init_referee_struct_data(void)
{
    memset(&referee_receive_header, 0, sizeof(frame_header_struct_t));
    memset(&referee_send_header, 0, sizeof(frame_header_struct_t));

    memset(&game_state, 0, sizeof(ext_game_state_t));
    memset(&game_result, 0, sizeof(ext_game_result_t));
    memset(&game_robot_HP_t, 0, sizeof(ext_game_robot_HP_t));


    memset(&field_event, 0, sizeof(ext_event_data_t));
    memset(&referee_warning_t, 0, sizeof(ext_referee_warning_t));
    memset(&dart_info_t, 0, sizeof(ext_dart_info_t));


    memset(&robot_state, 0, sizeof(ext_robot_status_t));
    memset(&power_heat_data_t, 0, sizeof(ext_power_heat_data_t));
    memset(&game_robot_pos_t, 0, sizeof(ext_robot_pos_t));
    memset(&buff_musk_t, 0, sizeof(ext_buff_data_t));
    memset(&robot_hurt_t, 0, sizeof(ext_robot_hurt_t));
    memset(&shoot_data_t, 0, sizeof(ext_shoot_data_t));
    memset(&bullet_remaining_t, 0, sizeof(ext_projectile_allowance_t));
    memset(&rfid_status_t, 0, sizeof(ext_rfid_status_t));
    memset(&dart_client_cmd_t, 0, sizeof(ext_dart_client_cmd_t));
    memset(&ground_robot_position_t, 0, sizeof(ext_ground_robot_position_t));
    memset(&radar_mark_data_t, 0, sizeof(ext_radar_mark_data_t));
    memset(&sentry_info_t, 0, sizeof(ext_sentry_info_t));
    memset(&radar_info_t, 0, sizeof(ext_radar_info_t));


    memset(&student_interactive_data_t, 0, sizeof(ext_robot_interactive_data_t));
    student_interactive_data_len = 0u;
    referee_tx_seq = 0u;
    memset(referee_tx_frame_buf, 0, sizeof(referee_tx_frame_buf));



}

void referee_data_solve(uint8_t *frame)
{
    // 函数地图：拆帧头和 cmd_id；按 cmd_id 拷贝到对应全局结构；重要数据顺手写日志。
    uint16_t cmd_id = 0;
    uint16_t payload_len = 0u;

    uint8_t index = 0;

    memcpy(&referee_receive_header, frame, sizeof(frame_header_struct_t));

    index += sizeof(frame_header_struct_t);

    memcpy(&cmd_id, frame + index, sizeof(uint16_t));
    index += sizeof(uint16_t);
    if (referee_receive_header.data_length >= (uint16_t)sizeof(uint16_t))
    {
        payload_len = (uint16_t)(referee_receive_header.data_length - sizeof(uint16_t));
    }

    switch (cmd_id)
    {
        case GAME_STATE_CMD_ID:
        {
            referee_copy_payload(&game_state, (uint16_t)sizeof(ext_game_state_t), frame + index, payload_len);
            sdlog_write(SDLOG_TAG_REF_GAME_STATE, &game_state, (uint16_t)sizeof(game_state));
        }
        break;
        case GAME_RESULT_CMD_ID:
        {
            referee_copy_payload(&game_result, (uint16_t)sizeof(ext_game_result_t), frame + index, payload_len);
            sdlog_write(SDLOG_TAG_REF_GAME_RESULT, &game_result, (uint16_t)sizeof(game_result));
        }
        break;
        case GAME_ROBOT_HP_CMD_ID:
        {
            referee_copy_payload(&game_robot_HP_t, (uint16_t)sizeof(ext_game_robot_HP_t), frame + index, payload_len);
            sdlog_write(SDLOG_TAG_REF_GAME_ROBOT_HP, &game_robot_HP_t, (uint16_t)sizeof(game_robot_HP_t));
        }
        break;


        case EVENT_DATA_CMD_ID:
        {
            referee_copy_payload(&field_event, (uint16_t)sizeof(ext_event_data_t), frame + index, payload_len);
            sdlog_write(SDLOG_TAG_REF_EVENT, &field_event, (uint16_t)sizeof(field_event));
        }
        break;
        case REFEREE_WARNING_CMD_ID:
        {
            referee_copy_payload(&referee_warning_t, (uint16_t)sizeof(ext_referee_warning_t), frame + index, payload_len);
            sdlog_write(SDLOG_TAG_REF_WARNING, &referee_warning_t, (uint16_t)sizeof(referee_warning_t));
        }
        break;
        case DART_INFO_CMD_ID:
        {
            referee_copy_payload(&dart_info_t, (uint16_t)sizeof(ext_dart_info_t), frame + index, payload_len);
            sdlog_write(SDLOG_TAG_REF_SUPPLY_ACTION, &dart_info_t, (uint16_t)sizeof(dart_info_t));
        }
        break;
        case ROBOT_STATUS_CMD_ID:
        {
            referee_copy_payload(&robot_state, (uint16_t)sizeof(ext_robot_status_t), frame + index, payload_len);
            sdlog_write(SDLOG_TAG_REF_ROBOT_STATE, &robot_state, (uint16_t)sizeof(robot_state));
        }
        break;
        case POWER_HEAT_DATA_CMD_ID:
        {
            referee_copy_payload(&power_heat_data_t, (uint16_t)sizeof(ext_power_heat_data_t), frame + index, payload_len);
            sdlog_write(SDLOG_TAG_REF_POWER_HEAT, &power_heat_data_t, (uint16_t)sizeof(power_heat_data_t));
        }
        break;
        case ROBOT_POS_CMD_ID:
        {
            referee_copy_payload(&game_robot_pos_t, (uint16_t)sizeof(ext_robot_pos_t), frame + index, payload_len);
            sdlog_write(SDLOG_TAG_REF_ROBOT_POS, &game_robot_pos_t, (uint16_t)sizeof(game_robot_pos_t));
        }
        break;
        case BUFF_DATA_CMD_ID:
        {
            referee_copy_payload(&buff_musk_t, (uint16_t)sizeof(ext_buff_data_t), frame + index, payload_len);
            sdlog_write(SDLOG_TAG_REF_BUFF, &buff_musk_t, (uint16_t)sizeof(buff_musk_t));
        }
        break;
        case RFID_STATUS_CMD_ID:
        {
            referee_copy_payload(&rfid_status_t, (uint16_t)sizeof(ext_rfid_status_t), frame + index, payload_len);
            sdlog_write(SDLOG_TAG_REF_RFID_STATUS, &rfid_status_t, (uint16_t)sizeof(rfid_status_t));
        }
        break;
        case ROBOT_HURT_CMD_ID:
        {
            referee_copy_payload(&robot_hurt_t, (uint16_t)sizeof(ext_robot_hurt_t), frame + index, payload_len);
            sdlog_write(SDLOG_TAG_REF_ROBOT_HURT, &robot_hurt_t, (uint16_t)sizeof(robot_hurt_t));
        }
        break;
        case SHOOT_DATA_CMD_ID:
        {
            referee_copy_payload(&shoot_data_t, (uint16_t)sizeof(ext_shoot_data_t), frame + index, payload_len);
            sdlog_write(SDLOG_TAG_REF_SHOOT_DATA, &shoot_data_t, (uint16_t)sizeof(shoot_data_t));
        }
        break;
        case PROJECTILE_ALLOWANCE_CMD_ID:
        {
            referee_copy_payload(&bullet_remaining_t, (uint16_t)sizeof(ext_projectile_allowance_t), frame + index, payload_len);
            sdlog_write(SDLOG_TAG_REF_BULLET_REMAINING, &bullet_remaining_t, (uint16_t)sizeof(bullet_remaining_t));
        }
        break;
        case DART_CLIENT_CMD_ID:
        {
            referee_copy_payload(&dart_client_cmd_t, (uint16_t)sizeof(ext_dart_client_cmd_t), frame + index, payload_len);
            sdlog_write(SDLOG_TAG_REF_AERIAL_ENERGY, &dart_client_cmd_t, (uint16_t)sizeof(dart_client_cmd_t));
        }
        break;
        case GROUND_ROBOT_POSITION_CMD_ID:
        {
            referee_copy_payload(&ground_robot_position_t,
                                 (uint16_t)sizeof(ext_ground_robot_position_t),
                                 frame + index,
                                 payload_len);
            sdlog_write(SDLOG_TAG_REF_GROUND_ROBOT_POSITION,
                        &ground_robot_position_t,
                        (uint16_t)sizeof(ground_robot_position_t));
        }
        break;
        case RADAR_MARK_DATA_CMD_ID:
        {
            referee_copy_payload(&radar_mark_data_t, (uint16_t)sizeof(ext_radar_mark_data_t), frame + index, payload_len);
            sdlog_write(SDLOG_TAG_REF_RADAR_MARK, &radar_mark_data_t, (uint16_t)sizeof(radar_mark_data_t));
        }
        break;
        case SENTRY_INFO_CMD_ID:
        {
            referee_copy_payload(&sentry_info_t, (uint16_t)sizeof(ext_sentry_info_t), frame + index, payload_len);
            sdlog_write(SDLOG_TAG_REF_SENTRY_INFO, &sentry_info_t, (uint16_t)sizeof(sentry_info_t));
        }
        break;
        case RADAR_INFO_CMD_ID:
        {
            referee_copy_payload(&radar_info_t, (uint16_t)sizeof(ext_radar_info_t), frame + index, payload_len);
            sdlog_write(SDLOG_TAG_REF_RADAR_INFO, &radar_info_t, (uint16_t)sizeof(radar_info_t));
        }
        break;
        case ROBOT_INTERACTIVE_DATA_CMD_ID:
        {
            referee_parse_student_interactive(frame + index, payload_len);
            sdlog_write(SDLOG_TAG_REF_ROBOT_INTERACTIVE, frame + index, payload_len);
        }
        break;
        default:
        {
            break;
        }
    }
}

void get_chassis_power_and_buffer(fp32 *power, fp32 *buffer)
{
    if (power != NULL)
    {
        *power = 0.0f;
    }
    if (buffer != NULL)
    {
        *buffer = (fp32)power_heat_data_t.buffer_energy;
    }
}

uint16_t get_chassis_power_limit(void)
{
    return robot_state.chassis_power_limit;
}

uint8_t get_robot_id(void)
{
    return robot_state.robot_id;
}

void get_shoot_heat0_limit_and_heat0(uint16_t *heat0_limit, uint16_t *heat0)
{
    *heat0_limit = robot_state.shooter_barrel_heat_limit;
    *heat0 = power_heat_data_t.shooter_17mm_barrel_heat;
}

void get_shoot_heat1_limit_and_heat1(uint16_t *heat1_limit, uint16_t *heat1)
{
    *heat1_limit = robot_state.shooter_barrel_heat_limit;
    *heat1 = power_heat_data_t.shooter_42mm_barrel_heat;
}

uint16_t get_student_interactive_data_len(void)
{
    return student_interactive_data_len;
}

uint8_t referee_tx_ready(void)
{
    return bsp_referee_tx_ready();
}

int referee_send_student_interactive(uint16_t data_cmd_id,
                                     uint16_t receiver_id,
                                     const uint8_t *user_data,
                                     uint16_t user_data_len)
{
    uint8_t payload[REF_PROTOCOL_FRAME_MAX_SIZE] = {0};
    const uint16_t sender_id = (uint16_t)get_robot_id();
    const uint16_t payload_len = (uint16_t)(6u + user_data_len);

    if (sender_id == 0u || user_data_len > (uint16_t)REFEREE_INTERACTIVE_DATA_MAX_LEN)
    {
        return -1;
    }

    payload[0] = (uint8_t)(data_cmd_id & 0x00FFu);
    payload[1] = (uint8_t)((data_cmd_id >> 8) & 0x00FFu);
    payload[2] = (uint8_t)(sender_id & 0x00FFu);
    payload[3] = (uint8_t)((sender_id >> 8) & 0x00FFu);
    payload[4] = (uint8_t)(receiver_id & 0x00FFu);
    payload[5] = (uint8_t)((receiver_id >> 8) & 0x00FFu);

    if (user_data != NULL && user_data_len != 0u)
    {
        memcpy(&payload[6], user_data, user_data_len);
    }

    return referee_send_frame(STUDENT_INTERACTIVE_DATA_CMD_ID, payload, payload_len);
}

static void referee_parse_student_interactive(const uint8_t *payload, uint16_t payload_len)
{
    student_interactive_data_len = 0u;
    memset(&student_interactive_data_t, 0, sizeof(student_interactive_data_t));

    if (payload == NULL || payload_len < 6u)
    {
        return;
    }

    student_interactive_data_t.data_cmd_id = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
    student_interactive_data_t.sender_id = (uint16_t)payload[2] | ((uint16_t)payload[3] << 8);
    student_interactive_data_t.receiver_id = (uint16_t)payload[4] | ((uint16_t)payload[5] << 8);
    payload += 6u;
    payload_len = (uint16_t)(payload_len - 6u);

    student_interactive_data_len =
        (payload_len < (uint16_t)REFEREE_INTERACTIVE_DATA_MAX_LEN) ? payload_len : (uint16_t)REFEREE_INTERACTIVE_DATA_MAX_LEN;
    if (student_interactive_data_len != 0u)
    {
        memcpy(student_interactive_data_t.user_data, payload, student_interactive_data_len);
    }
}

static void referee_copy_payload(void *dst, uint16_t dst_size, const uint8_t *payload, uint16_t payload_len)
{
    uint16_t copy_len = 0u;

    if (dst == NULL || dst_size == 0u)
    {
        return;
    }

    memset(dst, 0, dst_size);
    if (payload == NULL || payload_len == 0u)
    {
        return;
    }

    copy_len = (payload_len < dst_size) ? payload_len : dst_size;
    memcpy(dst, payload, copy_len);
}

static int referee_send_frame(uint16_t cmd_id, const uint8_t *payload, uint16_t payload_len)
{
    uint16_t frame_len = (uint16_t)(REF_HEADER_CRC_CMDID_LEN + payload_len);
    uint16_t index = 0u;

    if (frame_len > (uint16_t)sizeof(referee_tx_frame_buf))
    {
        return -1;
    }

    referee_tx_frame_buf[0] = HEADER_SOF;
    referee_tx_frame_buf[1] = (uint8_t)(payload_len & 0x00FFu);
    referee_tx_frame_buf[2] = (uint8_t)((payload_len >> 8) & 0x00FFu);
    referee_tx_frame_buf[3] = referee_tx_seq++;
    referee_tx_frame_buf[4] = 0u;
    append_CRC8_check_sum(referee_tx_frame_buf, REF_PROTOCOL_HEADER_SIZE);

    index = REF_PROTOCOL_HEADER_SIZE;
    referee_tx_frame_buf[index++] = (uint8_t)(cmd_id & 0x00FFu);
    referee_tx_frame_buf[index++] = (uint8_t)((cmd_id >> 8) & 0x00FFu);

    if (payload != NULL && payload_len != 0u)
    {
        memcpy(&referee_tx_frame_buf[index], payload, payload_len);
        index = (uint16_t)(index + payload_len);
    }

    referee_tx_frame_buf[index++] = 0u;
    referee_tx_frame_buf[index++] = 0u;
    append_CRC16_check_sum(referee_tx_frame_buf, index);
    return bsp_referee_tx(referee_tx_frame_buf, index);
}
