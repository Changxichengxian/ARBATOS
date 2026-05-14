/*
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 */

#pragma once

#include <stdint.h>

#include "struct_typedef.h"

typedef enum
{
    DM_MIT_TOOL_PHASE_BOOT = 0u,
    DM_MIT_TOOL_PHASE_SCAN,
    DM_MIT_TOOL_PHASE_FOUND,
    DM_MIT_TOOL_PHASE_WRITE_MASTER_ID,
    DM_MIT_TOOL_PHASE_WRITE_CAN_ID,
    DM_MIT_TOOL_PHASE_SAVE,
    DM_MIT_TOOL_PHASE_TEST,
    DM_MIT_TOOL_PHASE_DONE,
    DM_MIT_TOOL_PHASE_ERROR,
} dm_mit_tool_phase_e;

typedef struct
{
    uint8_t phase;
    uint8_t bus;
    uint8_t scan_id;
    uint8_t found;
    uint8_t write_armed;
    uint8_t write_done;
    uint8_t key_down;
    uint8_t rc_online;
    uint8_t rc_unlocked;
    uint8_t drive_allowed;
    uint8_t motor_enabled;
    uint8_t feedback_online;
    uint8_t feedback_state;
    uint8_t can_fd_seen;
    uint8_t can_brs_seen;
    uint8_t last_rx_flags;
    uint8_t blind_recover_done;
    uint16_t found_command_id;
    uint16_t found_master_id;
    uint16_t active_command_id;
    uint16_t active_master_id;
    uint16_t target_command_id;
    uint16_t target_master_id;
    uint16_t last_rx_id;
    uint32_t tx_count;
    uint32_t rx_count;
    uint32_t param_rx_count;
    uint32_t fd_data_bitrate;
    uint32_t last_rx_tick_ms;
    uint32_t last_rc_tick_ms;
    uint32_t last_error;
    int16_t speed_ch;
    int16_t torque_ch;
    fp32 cmd_velocity;
    fp32 cmd_torque;
    fp32 feedback_position;
    fp32 feedback_velocity;
    fp32 feedback_torque;
} dm_mit_tool_state_t;

void dm_mit_tool_task(void const *argument);
const dm_mit_tool_state_t *dm_mit_tool_get_state(void);
