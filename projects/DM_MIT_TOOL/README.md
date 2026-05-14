# DM MIT TOOL

F407 single-motor tool for DaMiao MIT motors.

The checked-in Keil target is currently configured for a single DM-H3510 on CAN1:

- Target `ESC_ID`: `1`.
- Target `MST_ID`: `0x11`.
- Auto ID writing is armed.
- After a successful ID write, the tool stops and waits for a motor and board power-cycle.
- If the motor already has the target IDs, the tool skips writing and enters the RC test directly.
- Test limit is conservative: `0.8rad/s` velocity and `0.2Nm` torque at full stick.
- Buzzer hints: ID changed = two rising beeps; failed = three low beeps; already target = one medium beep.
- If no reply is received, the F407 tool sends a blind H3510 recovery sequence:
  write `can_br=4` and save across the configured scan range, then waits for power-cycle.

Quick edits are in:

```text
target/DM_MIT_TOOL/User/application/dm_mit_tool_config.h
```

Use it with exactly one motor on the CAN bus:

1. Flash `projects/DM_MIT_TOOL/MDK-ARM/DM_MIT_TOOL.uvprojx`.
2. Connect one DaMiao motor to CAN1 by default.
3. The firmware scans command IDs from `DM_MIT_TOOL_SCAN_MIN_ID` to `DM_MIT_TOOL_SCAN_MAX_ID`.
4. If ID writing is armed and the current ID differs from the target, it writes `MST_ID` and `ESC_ID`, sends save frames, then stays idle until power-cycle.
5. Power-cycle the motor and board.
6. If the IDs already match, it skips writing and enters the MIT command test.
7. In test mode, both RC switches must match the unlock values before the motor is enabled. By default `rc.ch[1]` controls velocity and `rc.ch[3]` controls torque.

Important config macros:

- `DM_MIT_TOOL_BUS`: `1` or `2`.
- `DM_MIT_TOOL_SCAN_MIN_ID`: default `1`.
- `DM_MIT_TOOL_SCAN_MAX_ID`: default `15`.
- `DM_MIT_TOOL_TARGET_COMMAND_ID`: target `ESC_ID`, default `1`.
- `DM_MIT_TOOL_TARGET_MASTER_ID`: target `MST_ID`, default `0x11`.
- `DM_MIT_TOOL_AUTO_WRITE`: set `1` to allow ID writing.
- `DM_MIT_TOOL_WRITE_ARM`: must be `0xA5` before writing happens.
- `DM_MIT_TOOL_MODEL`: `4310`, `6215`, or `3510`.
- `DM_MIT_TOOL_RC_UNLOCK_S0`: default `RC_SW_UP`; set `0` to ignore switch 0.
- `DM_MIT_TOOL_RC_UNLOCK_S1`: default `RC_SW_UP`; set `0` to ignore switch 1.
- `DM_MIT_TOOL_SPEED_CH`: RC channel index for velocity, default `1`.
- `DM_MIT_TOOL_TORQUE_CH`: RC channel index for torque, default `3`.
- `DM_MIT_TOOL_MAX_SPEED_RAD_S`: full-stick velocity, default `0.8f`.
- `DM_MIT_TOOL_MAX_TORQUE_NM`: full-stick torque, default `0.5f`.
- `DM_MIT_TOOL_REBOOT_AFTER_WRITE`: default `1`; stop after an ID write so the next boot tests with the saved IDs.
- `DM_MIT_TOOL_BEEP_ENABLE`: default `1`; enable ID result tones.
- `DM_MIT_TOOL_BEEP_VOLUME`: default `160`; tone volume, `0..255`.
- `DM_MIT_TOOL_REQUIRE_FEEDBACK`: default `1`; non-zero commands wait for fresh motor feedback.
- `DM_MIT_TOOL_BLIND_RECOVER_ENABLE`: default `1` here; if scan fails, send H3510 `can_br=4` recovery without waiting for feedback.
- `DM_MIT_TOOL_BLIND_RECOVER_REPEAT`: default `3`; number of blind recovery passes.

Example for changing one H3510 motor to command ID `1`, feedback ID `0x11` on CAN1:

```text
DM_MIT_TOOL_BUS=1
DM_MIT_TOOL_TARGET_COMMAND_ID=1
DM_MIT_TOOL_TARGET_MASTER_ID=0x11
DM_MIT_TOOL_AUTO_WRITE=1
DM_MIT_TOOL_WRITE_ARM=0xA5
DM_MIT_TOOL_MODEL=3510
DM_MIT_TOOL_MAX_SPEED_RAD_S=0.8f
DM_MIT_TOOL_MAX_TORQUE_NM=0.2f
DM_MIT_TOOL_REBOOT_AFTER_WRITE=1
```

After writing IDs, power-cycle the motor and board. On the next boot, matching IDs are treated as a test run and no write is attempted.

Blind recovery note: this is for the F407 classic-CAN board only. It helps when an H3510 is already in CAN FD mode, because the motor can still receive classic CAN parameter frames but the F407 cannot receive the FD reply. Use the H7 tool path if you need a confirmed read/write in FD mode.
