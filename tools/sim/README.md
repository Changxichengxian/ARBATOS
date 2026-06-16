# ARBATOS simulation tools

`robot_sim.py` reads the current `Robotconfig/<PROJECT>` files, Keil project
defines, and shared profile defaults. It estimates:

- simulated motor traffic and CAN bus placement;
- 1000 Hz motor feedback pressure;
- RM group command frame pressure;
- DM/MIT command frame pressure and per-ms scheduler throttling;
- CAN receive queue backlog against `ROBOT_PROFILE_CAN_FEEDBACK_RX_MAX_FRAMES_PER_WAKE`;
- rough CPU pressure from CAN frame handling and configured profiler budgets.

Run from the repository root:

```powershell
.\tools\build.ps1 -Action sim -Project MINIWHEELEG-C
.\tools\build.ps1 -Action sim -Project all
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
read as an estimate until paired with real `RtProf` data from logs.
The simulator follows the firmware profile filters for ARM/MIT routes and shoot
runtime budgets, while still counting configured RM groups because the firmware
can emit zero-current group frames for configured RM motors.

## MuJoCo physics entry

The first wheel-leg MuJoCo entry lives under:

```powershell
python .\tools\mujoco\wheelleg\run_wheelleg.py --check --project MINIWHEELEG-C
python .\tools\mujoco\wheelleg\run_wheelleg.py --project MINIWHEELEG-C --duration-s 5
python .\tools\mujoco\wheelleg\run_wheelleg.py --project MINIWHEELEG-C --viewer --realtime
```

`--check` only validates the MJCF file and project configuration. A real run
requires the Python `mujoco` package and a local C compiler so the runner can
build the tiny native bridge that calls `wheelleg_core.h`.

The current MJCF is a minimal fixture, not a final wheel-leg dynamics model.
It is meant to validate the control-core-to-physics path before the full
five-bar leg constraints are modeled.

## Control core simulation path

Physics simulators such as MuJoCo should call the reusable control core instead
of the FreeRTOS task entry points. Firmware tasks read sensors/manual input and
write actuator commands; simulation runners should provide the same core input
from simulator state and apply the returned `MotorCmd` outputs to the
simulator actuators.

Current core boundary files:

- `shared/application/robot/control_core.h`: shared step metadata and actuator
  command helpers.
- `shared/application/arm/arm_core.h`: manual arm joint command core.
- `shared/application/chassis/chassis_core.h`: chassis command/state/output
  contract.
- `shared/application/gimbal/gimbal_core.h`: gimbal axis command/state/output
  contract.
- `shared/application/wheelleg/wheelleg_core.h`: wheel-leg command/state/output
  contract for future MuJoCo runners.
