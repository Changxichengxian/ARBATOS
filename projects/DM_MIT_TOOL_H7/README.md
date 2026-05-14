# DM MIT TOOL H7

H7 single-motor tool for DaMiao MIT motors.

Use this when the motor may already be in CAN FD mode. Unlike the F407 tool,
this project receives real CAN FD feedback and only writes IDs after confirmed
parameter replies.

Quick config:

```text
target/DM_MIT_TOOL_H7/User/application/dm_mit_tool_config.h
```

Default checked-in target:

- Motor model: H3510.
- FDCAN bus: `1`.
- Target `ESC_ID`: `1`.
- Target `MST_ID`: `0x11`.
- ID writing is armed.
- Test limit: `0.8rad/s` velocity and `0.2Nm` torque at full stick.

Flash:

```text
projects/DM_MIT_TOOL_H7/MDK-ARM/DM_MIT_TOOL_H7.uvprojx
```

FDCAN setup:

- Nominal/arbitration phase: 1Mbps.
- Default data phase: 5Mbps.
- The tool scans common FD data rates: 5M, 4M, about 3.2M, 2.5M, 2M, and 1M.
- Classic CAN replies and CAN FD replies are both accepted.

Use it with exactly one motor on the bus when changing IDs. After a successful
write, power-cycle the motor and board.
