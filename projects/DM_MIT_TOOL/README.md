# DM MIT TOOL

F407 single-motor tool for DaMiao MIT motors.

Use it with exactly one motor on the CAN bus:

1. Flash `projects/DM_MIT_TOOL/MDK-ARM/DM_MIT_TOOL.uvprojx`.
2. Connect one DaMiao motor to CAN1 by default.
3. The firmware scans command IDs from `DM_MIT_TOOL_SCAN_MIN_ID` to `DM_MIT_TOOL_SCAN_MAX_ID`.
4. If ID writing is armed and the current ID differs from the target, it writes `MST_ID` and `ESC_ID`, then sends save frames.
5. If the IDs already match, it skips writing and enters the MIT command test.
6. In test mode, both RC switches must match the unlock values before the motor is enabled. By default `rc.ch[1]` controls velocity and `rc.ch[3]` controls torque.

Important build macros:

- `DM_MIT_TOOL_BUS`: `1` or `2`.
- `DM_MIT_TOOL_SCAN_MIN_ID`: default `1`.
- `DM_MIT_TOOL_SCAN_MAX_ID`: default `15`.
- `DM_MIT_TOOL_TARGET_COMMAND_ID`: target `ESC_ID`, default `1`.
- `DM_MIT_TOOL_TARGET_MASTER_ID`: target `MST_ID`, default `0x11`.
- `DM_MIT_TOOL_AUTO_WRITE`: set `1` to allow ID writing.
- `DM_MIT_TOOL_WRITE_ARM`: must be `0xA5` before writing happens.
- `DM_MIT_TOOL_MODEL`: `4310` or `6215`.
- `DM_MIT_TOOL_RC_UNLOCK_S0`: default `RC_SW_UP`; set `0` to ignore switch 0.
- `DM_MIT_TOOL_RC_UNLOCK_S1`: default `RC_SW_UP`; set `0` to ignore switch 1.
- `DM_MIT_TOOL_SPEED_CH`: RC channel index for velocity, default `1`.
- `DM_MIT_TOOL_TORQUE_CH`: RC channel index for torque, default `3`.
- `DM_MIT_TOOL_MAX_SPEED_RAD_S`: full-stick velocity, default `0.8f`.
- `DM_MIT_TOOL_MAX_TORQUE_NM`: full-stick torque, default `0.5f`.
- `DM_MIT_TOOL_REQUIRE_FEEDBACK`: default `1`; non-zero commands wait for fresh motor feedback.

Example for changing one motor to command ID `1`, feedback ID `0x11` on CAN1:

```text
DM_MIT_TOOL_BUS=1
DM_MIT_TOOL_TARGET_COMMAND_ID=1
DM_MIT_TOOL_TARGET_MASTER_ID=0x11
DM_MIT_TOOL_AUTO_WRITE=1
DM_MIT_TOOL_WRITE_ARM=0xA5
DM_MIT_TOOL_MODEL=4310
```

After writing IDs, power-cycle the motor if the new ID does not become active immediately.
