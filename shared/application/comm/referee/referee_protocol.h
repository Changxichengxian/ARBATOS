/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef REFEREE_PROTOCOL_H
#define REFEREE_PROTOCOL_H

#include "types.h"

#define HEADER_SOF 0xA5
#define REF_PROTOCOL_FRAME_MAX_SIZE         128

#define REF_PROTOCOL_HEADER_SIZE            sizeof(frame_header_struct_t)
#define REF_PROTOCOL_CMD_SIZE               2
#define REF_PROTOCOL_CRC16_SIZE             2
#define REF_HEADER_CRC_LEN                  (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE)
#define REF_HEADER_CRC_CMDID_LEN            (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + sizeof(uint16_t))
#define REF_HEADER_CMDID_LEN                (REF_PROTOCOL_HEADER_SIZE + sizeof(uint16_t))

#pragma pack(push, 1)

typedef enum
{
    GAME_STATE_CMD_ID                       = 0x0001,
    GAME_RESULT_CMD_ID                      = 0x0002,
    GAME_ROBOT_HP_CMD_ID                    = 0x0003,
    EVENT_DATA_CMD_ID                       = 0x0101,
    REFEREE_WARNING_CMD_ID                  = 0x0104,
    DART_INFO_CMD_ID                        = 0x0105,
    ROBOT_STATUS_CMD_ID                     = 0x0201,
    POWER_HEAT_DATA_CMD_ID                  = 0x0202,
    ROBOT_POS_CMD_ID                        = 0x0203,
    BUFF_DATA_CMD_ID                        = 0x0204,
    ROBOT_HURT_CMD_ID                       = 0x0206,
    SHOOT_DATA_CMD_ID                       = 0x0207,
    PROJECTILE_ALLOWANCE_CMD_ID             = 0x0208,
    RFID_STATUS_CMD_ID                      = 0x0209,
    DART_CLIENT_CMD_ID                      = 0x020A,
    GROUND_ROBOT_POSITION_CMD_ID            = 0x020B,
    RADAR_MARK_DATA_CMD_ID                  = 0x020C,
    SENTRY_INFO_CMD_ID                      = 0x020D,
    RADAR_INFO_CMD_ID                       = 0x020E,
    ROBOT_INTERACTIVE_DATA_CMD_ID           = 0x0301,
    CUSTOM_CONTROLLER_INTERACTIVE_CMD_ID    = 0x0302,
    MINI_MAP_INTERACTIVE_CMD_ID             = 0x0303,
    MINI_MAP_RADAR_DATA_CMD_ID              = 0x0305,
    MINI_MAP_PATH_CMD_ID                    = 0x0307,
    MINI_MAP_ROBOT_DATA_CMD_ID              = 0x0308,
    CUSTOM_CONTROLLER_ROBOT_DATA_CMD_ID     = 0x0309,
    CUSTOM_CLIENT_TO_UI_CMD_ID              = 0x0310,
    CUSTOM_CLIENT_TO_ROBOT_CMD_ID           = 0x0311,
    RADAR_OPPONENT_POSITION_CMD_ID          = 0x0A01,
    RADAR_OPPONENT_HP_CMD_ID                = 0x0A02,
    RADAR_OPPONENT_PROJECTILE_CMD_ID        = 0x0A03,
    RADAR_OPPONENT_MACRO_STATE_CMD_ID       = 0x0A04,
    RADAR_OPPONENT_BUFF_CMD_ID              = 0x0A05,
    RADAR_OPPONENT_INTERFERENCE_KEY_CMD_ID  = 0x0A06,

    // Old names kept here so existing business code can continue compiling.
    FIELD_EVENTS_CMD_ID                     = EVENT_DATA_CMD_ID,
    ROBOT_STATE_CMD_ID                      = ROBOT_STATUS_CMD_ID,
    BUFF_MUSK_CMD_ID                        = BUFF_DATA_CMD_ID,
    BULLET_REMAINING_CMD_ID                 = PROJECTILE_ALLOWANCE_CMD_ID,
    STUDENT_INTERACTIVE_DATA_CMD_ID         = ROBOT_INTERACTIVE_DATA_CMD_ID,
    IDCustomData                            = ROBOT_INTERACTIVE_DATA_CMD_ID + 1,
} referee_cmd_id_t;
typedef  struct
{
  uint8_t SOF;
  uint16_t data_length;
  uint8_t seq;
  uint8_t CRC8;
} frame_header_struct_t;

typedef enum
{
  STEP_HEADER_SOF  = 0,
  STEP_LENGTH_LOW  = 1,
  STEP_LENGTH_HIGH = 2,
  STEP_FRAME_SEQ   = 3,
  STEP_HEADER_CRC8 = 4,
  STEP_DATA_CRC16  = 5,
} unpack_step_e;

typedef struct
{
  frame_header_struct_t *p_header;
  uint16_t       data_len;
  uint8_t        protocol_packet[REF_PROTOCOL_FRAME_MAX_SIZE];
  unpack_step_e  unpack_step;
  uint16_t       index;
} unpack_data_t;

#pragma pack(pop)

#endif // REFEREE_PROTOCOL_H
