/*
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 */

#pragma once

/*
 * H7 DaMiao MIT motor ID tool.
 *
 * Connect exactly one motor when changing IDs. H7 can receive both classic CAN
 * and CAN FD replies, so this path waits for real feedback instead of blind
 * recovery.
 */

/* FDCAN bus: 1, 2, or 3 on the H7 board. */
#define DM_MIT_TOOL_BUS 1u

/* Scan range for the motor's current command ID. */
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

/* H7 scans several CAN FD data rates and also accepts classic CAN replies. */
#define DM_MIT_TOOL_AUTO_FD_ENABLE 1u

/*
 * This full H7 project still compiles the normal shared application files.
 * Let their weak/strong CAN extension symbols stand; the tool polls bsp_can
 * directly and does not need the extra CAN_receive hook.
 */
#define DM_MIT_TOOL_OWN_CAN_EXTRA_HOOK 0u

/* Blind recovery is for the F4 classic-CAN tool. H7 uses confirmed feedback. */
#define DM_MIT_TOOL_BLIND_RECOVER_ENABLE 0u

/* Test behavior after ID handling. */
#define DM_MIT_TOOL_TEST_ENABLE        1u
#define DM_MIT_TOOL_REBOOT_AFTER_WRITE 1u

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
