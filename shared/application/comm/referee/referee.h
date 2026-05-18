/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef REFEREE_H
#define REFEREE_H

#include "main.h"

#include "referee_protocol.h"

#define REFEREE_INTERACTIVE_DATA_MAX_LEN 112u
#define REFEREE_UI_GRAPHIC_RAW_LEN       15u
#define REFEREE_UI_STRING_UTF16_BYTES    30u

typedef enum
{
    REFEREE_UI_DATA_DELETE  = 0x0100,
    REFEREE_UI_DATA_SINGLE  = 0x0101,
    REFEREE_UI_DATA_DOUBLE  = 0x0102,
    REFEREE_UI_DATA_FIVE    = 0x0103,
    REFEREE_UI_DATA_SEVEN   = 0x0104,
    REFEREE_UI_DATA_CHAR    = 0x0110,
} referee_ui_data_cmd_id_t;

typedef enum
{
    REFEREE_UI_DELETE_NONE  = 0u,
    REFEREE_UI_DELETE_LAYER = 1u,
    REFEREE_UI_DELETE_ALL   = 2u,
} referee_ui_delete_type_t;

typedef enum
{
    REFEREE_UI_OP_NONE   = 0u,
    REFEREE_UI_OP_ADD    = 1u,
    REFEREE_UI_OP_MODIFY = 2u,
    REFEREE_UI_OP_DELETE = 3u,
} referee_ui_operation_t;

typedef enum
{
    REFEREE_UI_LINE    = 0u,
    REFEREE_UI_RECT    = 1u,
    REFEREE_UI_CIRCLE  = 2u,
    REFEREE_UI_ELLIPSE = 3u,
    REFEREE_UI_ARC     = 4u,
    REFEREE_UI_FLOAT   = 5u,
    REFEREE_UI_INT     = 6u,
    REFEREE_UI_CHAR    = 7u,
} referee_ui_graphic_type_t;

typedef enum
{
    REFEREE_UI_COLOR_TEAM   = 0u,
    REFEREE_UI_COLOR_YELLOW = 1u,
    REFEREE_UI_COLOR_GREEN  = 2u,
    REFEREE_UI_COLOR_ORANGE = 3u,
    REFEREE_UI_COLOR_PURPLE = 4u,
    REFEREE_UI_COLOR_PINK   = 5u,
    REFEREE_UI_COLOR_CYAN   = 6u,
    REFEREE_UI_COLOR_BLACK  = 7u,
    REFEREE_UI_COLOR_WHITE  = 8u,
} referee_ui_color_t;

typedef struct
{
    uint8_t name[3];
    uint8_t operation;
    uint8_t type;
    uint8_t layer;
    uint8_t color;
    uint16_t details_a;
    uint16_t details_b;
    uint16_t width;
    uint16_t start_x;
    uint16_t start_y;
    uint16_t details_c;
    uint16_t details_d;
    uint16_t details_e;
} referee_ui_graphic_t;

typedef enum
{
    RED_HERO        = 1,
    RED_ENGINEER    = 2,
    RED_STANDARD_1  = 3,
    RED_STANDARD_2  = 4,
    RED_STANDARD_3  = 5,
    RED_AERIAL      = 6,
    RED_SENTRY      = 7,
    RED_DART        = 8,
    RED_RADAR       = 9,
    RED_OUTPOST     = 10,
    RED_BASE        = 11,
    BLUE_HERO       = 101,
    BLUE_ENGINEER   = 102,
    BLUE_STANDARD_1 = 103,
    BLUE_STANDARD_2 = 104,
    BLUE_STANDARD_3 = 105,
    BLUE_AERIAL     = 106,
    BLUE_SENTRY     = 107,
    BLUE_DART       = 108,
    BLUE_RADAR      = 109,
    BLUE_OUTPOST    = 110,
    BLUE_BASE       = 111,
} robot_id_t;
typedef enum
{
    PROGRESS_UNSTART        = 0,
    PROGRESS_PREPARE        = 1,
    PROGRESS_SELFCHECK      = 2,
    PROGRESS_5sCOUNTDOWN    = 3,
    PROGRESS_BATTLE         = 4,
    PROGRESS_CALCULATING    = 5,
} game_progress_t;

typedef __packed struct //0x0001
{
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
    uint64_t sync_timestamp;
} ext_game_state_t;

typedef __packed struct //0x0002
{
    uint8_t winner;
} ext_game_result_t;

typedef __packed struct //0x0003
{
    uint16_t ally_1_robot_HP;
    uint16_t ally_2_robot_HP;
    uint16_t ally_3_robot_HP;
    uint16_t ally_4_robot_HP;
    uint16_t reserved;
    uint16_t ally_7_robot_HP;
    uint16_t ally_outpost_HP;
    uint16_t ally_base_HP;
} ext_game_robot_HP_t;

typedef __packed struct //0x0101
{
    uint32_t event_data;
} ext_event_data_t;

typedef __packed struct //0x0104
{
    uint8_t level;
    uint8_t offending_robot_id;
    uint8_t count;
} ext_referee_warning_t;

typedef __packed struct //0x0105
{
    uint8_t dart_remaining_time;
    uint16_t dart_info;
} ext_dart_info_t;

typedef __packed struct //0x0201
{
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t current_HP;
    uint16_t maximum_HP;
    uint16_t shooter_barrel_cooling_value;
    uint16_t shooter_barrel_heat_limit;
    uint16_t chassis_power_limit;
    uint8_t power_management_gimbal_output : 1;
    uint8_t power_management_chassis_output : 1;
    uint8_t power_management_shooter_output : 1;
} ext_robot_status_t;

typedef ext_robot_status_t ext_game_robot_state_t;

typedef __packed struct //0x0202
{
    uint16_t reserved0;
    uint16_t reserved1;
    float reserved2;
    uint16_t buffer_energy;
    uint16_t shooter_17mm_barrel_heat;
    uint16_t shooter_42mm_barrel_heat;
} ext_power_heat_data_t;

typedef __packed struct //0x0203
{
    float x;
    float y;
    float angle;
} ext_robot_pos_t;

typedef ext_robot_pos_t ext_game_robot_pos_t;

typedef __packed struct //0x0204
{
    uint8_t recovery_buff;
    uint16_t cooling_buff;
    uint8_t defence_buff;
    uint8_t vulnerability_buff;
    uint16_t attack_buff;
    uint8_t remaining_energy;
} ext_buff_data_t;

typedef ext_buff_data_t ext_buff_musk_t;

typedef __packed struct //0x0206
{
    uint8_t armor_id : 4;
    uint8_t hp_deduction_reason : 4;
} ext_robot_hurt_t;

typedef __packed struct //0x0207
{
    uint8_t bullet_type;
    uint8_t shooter_number;
    uint8_t launching_frequency;
    float initial_speed;
} ext_shoot_data_t;

typedef __packed struct //0x0208
{
    uint16_t projectile_allowance_17mm;
    uint16_t projectile_allowance_42mm;
    uint16_t remaining_gold_coin;
    uint16_t projectile_allowance_fortress;
} ext_projectile_allowance_t;

typedef ext_projectile_allowance_t ext_bullet_remaining_t;

typedef __packed struct //0x0209
{
    uint32_t rfid_status;
    uint8_t rfid_status_2;
} ext_rfid_status_t;

typedef __packed struct //0x020A
{
    uint8_t dart_launch_opening_status;
    uint8_t reserved;
    uint16_t target_change_time;
    uint16_t latest_launch_cmd_time;
} ext_dart_client_cmd_t;

typedef __packed struct //0x020B
{
    float hero_x;
    float hero_y;
    float engineer_x;
    float engineer_y;
    float standard_3_x;
    float standard_3_y;
    float standard_4_x;
    float standard_4_y;
    float reserved0;
    float reserved1;
} ext_ground_robot_position_t;

typedef __packed struct //0x020C
{
    uint16_t mark_progress;
} ext_radar_mark_data_t;

typedef __packed struct //0x020D
{
    uint32_t sentry_info;
    uint16_t sentry_info_2;
} ext_sentry_info_t;

typedef __packed struct //0x020E
{
    uint8_t radar_info;
} ext_radar_info_t;

typedef __packed struct //0x0301
{
    uint16_t data_cmd_id;
    uint16_t sender_id;
    uint16_t receiver_id;
    uint8_t user_data[REFEREE_INTERACTIVE_DATA_MAX_LEN];
} ext_robot_interactive_data_t;

typedef ext_robot_interactive_data_t ext_student_interactive_data_t;

extern ext_shoot_data_t shoot_data_t;
extern ext_projectile_allowance_t bullet_remaining_t;



extern void init_referee_struct_data(void);
extern void referee_data_solve(uint8_t *frame);

extern void get_chassis_power_and_buffer(fp32 *power, fp32 *buffer);
extern uint16_t get_chassis_power_limit(void);

extern uint8_t get_robot_id(void);

extern void get_shoot_heat0_limit_and_heat0(uint16_t *heat0_limit, uint16_t *heat0);
extern void get_shoot_heat1_limit_and_heat1(uint16_t *heat1_limit, uint16_t *heat1);
extern uint16_t get_student_interactive_data_len(void);
extern uint8_t referee_tx_ready(void);
extern uint16_t referee_get_client_id_by_robot_id(uint16_t robot_id);
extern uint16_t referee_get_self_client_id(void);
extern int referee_send_student_interactive(uint16_t data_cmd_id,
                                            uint16_t receiver_id,
                                            const uint8_t *user_data,
                                            uint16_t user_data_len);
extern void referee_ui_make_line(referee_ui_graphic_t *graphic,
                                 const char name[3],
                                 uint8_t operation,
                                 uint8_t layer,
                                 uint8_t color,
                                 uint16_t width,
                                 uint16_t start_x,
                                 uint16_t start_y,
                                 uint16_t end_x,
                                 uint16_t end_y);
extern void referee_ui_make_rect(referee_ui_graphic_t *graphic,
                                 const char name[3],
                                 uint8_t operation,
                                 uint8_t layer,
                                 uint8_t color,
                                 uint16_t width,
                                 uint16_t start_x,
                                 uint16_t start_y,
                                 uint16_t end_x,
                                 uint16_t end_y);
extern void referee_ui_make_circle(referee_ui_graphic_t *graphic,
                                   const char name[3],
                                   uint8_t operation,
                                   uint8_t layer,
                                   uint8_t color,
                                   uint16_t width,
                                   uint16_t center_x,
                                   uint16_t center_y,
                                   uint16_t radius);
extern void referee_ui_make_ellipse(referee_ui_graphic_t *graphic,
                                    const char name[3],
                                    uint8_t operation,
                                    uint8_t layer,
                                    uint8_t color,
                                    uint16_t width,
                                    uint16_t center_x,
                                    uint16_t center_y,
                                    uint16_t x_half_axis,
                                    uint16_t y_half_axis);
extern void referee_ui_make_arc(referee_ui_graphic_t *graphic,
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
                                uint16_t y_half_axis);
extern void referee_ui_make_int(referee_ui_graphic_t *graphic,
                                const char name[3],
                                uint8_t operation,
                                uint8_t layer,
                                uint8_t color,
                                uint16_t width,
                                uint16_t start_x,
                                uint16_t start_y,
                                uint16_t font_size,
                                int32_t value);
extern void referee_ui_make_float(referee_ui_graphic_t *graphic,
                                  const char name[3],
                                  uint8_t operation,
                                  uint8_t layer,
                                  uint8_t color,
                                  uint16_t width,
                                  uint16_t start_x,
                                  uint16_t start_y,
                                  uint16_t font_size,
                                  fp32 value);
extern void referee_ui_make_char(referee_ui_graphic_t *graphic,
                                 const char name[3],
                                 uint8_t operation,
                                 uint8_t layer,
                                 uint8_t color,
                                 uint16_t width,
                                 uint16_t start_x,
                                 uint16_t start_y,
                                 uint16_t font_size);
extern int referee_ui_delete(uint16_t receiver_id, uint8_t delete_type, uint8_t layer);
extern int referee_ui_delete_self(uint8_t delete_type, uint8_t layer);
extern int referee_ui_send_graphics(uint16_t receiver_id, const referee_ui_graphic_t *graphics, uint8_t count);
extern int referee_ui_send_graphics_self(const referee_ui_graphic_t *graphics, uint8_t count);
extern int referee_ui_send_string_utf8(uint16_t receiver_id, const referee_ui_graphic_t *graphic, const char *text_utf8);
extern int referee_ui_send_string_utf8_self(const referee_ui_graphic_t *graphic, const char *text_utf8);
extern void referee_ui_demo_tick(void);
#endif
