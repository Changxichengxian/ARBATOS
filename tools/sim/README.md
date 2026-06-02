# ARBATOS simulation tools

`robot_sim.py` reads the current `Robotconfig/<PROJECT>` files, Keil project
defines, and shared profile defaults. It estimates:

- enabled motor count and CAN bus placement;
- 1000 Hz motor feedback pressure;
- RM group command frame pressure;
- DM/MIT command frame pressure and per-ms scheduler throttling;
- CAN receive queue backlog against `ROBOT_PROFILE_CAN_FEEDBACK_RX_MAX_FRAMES_PER_WAKE`;
- rough CPU pressure from CAN frame handling and configured profiler budgets.

Run from the repository root:

```powershell
python .\tools\sim\robot_sim.py --project HERO-C
python .\tools\sim\robot_sim.py --project MINIWHEELEG-C
python .\tools\sim\robot_sim.py --project HERO-C --json
```

Useful options:

- `--duration-ms 2000`: longer scheduler simulation.
- `--can-tx-period-ms 2`: test a command-period change without editing firmware.
- `--motor-feedback-hz 1000`: default enabled CAN motor feedback rate.
- `--can-bits-per-frame 135`: conservative classical CAN frame size estimate.
- `--rx-us-per-frame 6 --tx-us-per-frame 8`: F407 CPU cost assumptions for CAN frame work.
- `--fail-on-risk`: return a non-zero exit code when the report has warnings or failures.

The tool is a pressure simulator, not a physics simulator. It is meant to catch
configuration-level overload before flashing firmware. The CPU number should be
read as an estimate until paired with real `rt_profiler` data from logs.
