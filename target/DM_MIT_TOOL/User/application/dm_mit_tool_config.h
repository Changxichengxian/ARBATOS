/*
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 */

#pragma once

/*
 * DM MIT TOOL quick config.
 *
 * Normal H3510 workflow:
 * 1. Connect exactly one motor.
 * 2. Flash and power up. If IDs differ, the tool writes them and stops.
 * 3. Power-cycle the motor and board.
 * 4. On the next boot, matching IDs enter the RC test directly.
 */

/* CAN bus: 1 or 2. */
#define DM_MIT_TOOL_BUS 1u

/* Scan range for the motor's current command ID. Keep narrow if you know it. */
#define DM_MIT_TOOL_SCAN_MIN_ID 1u
#define DM_MIT_TOOL_SCAN_MAX_ID 15u

/* Target IDs written into the motor. */
#define DM_MIT_TOOL_TARGET_COMMAND_ID 1u
#define DM_MIT_TOOL_TARGET_MASTER_ID  0x11u

/* ID writing guard. Set AUTO_WRITE to 0u when you only want to test. */
#define DM_MIT_TOOL_AUTO_WRITE 1u
#define DM_MIT_TOOL_WRITE_ARM  0xA5u

/* Motor model: 4310u, 6215u, or 3510u. */
#define DM_MIT_TOOL_MODEL 3510u

/* Test behavior after ID handling. */
#define DM_MIT_TOOL_TEST_ENABLE        1u
#define DM_MIT_TOOL_REBOOT_AFTER_WRITE 1u

/*
 * Buzzer result hints for ID handling:
 * - changed: two rising beeps
 * - failed: three low beeps
 * - already target: one medium beep
 */
#define DM_MIT_TOOL_BEEP_ENABLE 1u
#define DM_MIT_TOOL_BEEP_VOLUME 160u

/* RC unlock: 0u ignores that switch; RC_SW_UP requires the switch to be up. */
#define DM_MIT_TOOL_RC_UNLOCK_S0 RC_SW_UP
#define DM_MIT_TOOL_RC_UNLOCK_S1 RC_SW_UP

/* RC channel mapping. Channel numbers are rc.ch[index]. */
#define DM_MIT_TOOL_SPEED_CH  1u
#define DM_MIT_TOOL_TORQUE_CH 3u

/* Full-stick test limits. Start low for a loose motor on the bench. */
#define DM_MIT_TOOL_MAX_SPEED_RAD_S 0.8f
#define DM_MIT_TOOL_MAX_TORQUE_NM   0.2f

/* Safety filters. */
#define DM_MIT_TOOL_RC_DEADBAND      20
#define DM_MIT_TOOL_REQUIRE_FEEDBACK 1u

/*
 * F407 cannot receive CAN FD replies. If the scan finds nothing, this sends
 * H3510 can_br=4 (1Mbps classic CAN) and save frames across the scan range.
 */
#define DM_MIT_TOOL_BLIND_RECOVER_ENABLE 1u
#define DM_MIT_TOOL_BLIND_RECOVER_REPEAT 3u
