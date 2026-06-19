# MuJoCo wheel-leg runner

This is the MuJoCo entry for the ARBATOS wheel-leg control core.

The runner generates a project-specific five-bar wheel-leg model from
`Robotconfig/<project>/RobotConfig.c`:

- wheels are real contact wheels attached to a free base;
- front/back active joints use the same names as `WheelLegCore.h` expects;
- each side has a constrained five-bar leg with passive knee joints;
- the wheel carrier follows the closed chain through MuJoCo equality constraints;
- the runner calls a tiny native C bridge that includes `WheelLegCore.h`.

## Requirements

Install the Python MuJoCo package and one C compiler:

```powershell
python -m pip install mujoco
```

Compiler options on Windows:

- MSVC Build Tools, providing `cl`;
- LLVM, providing `clang`;
- MinGW, providing `gcc`.
- Zig, providing `zig`.

The runner builds `tools/mujoco/wheelleg/WheelLegCoreBridge.c` into
`tmp/mujoco_wheelleg/arbatos_wheelleg_core_bridge.dll` on first run. It also
writes generated MJCF files to `tmp/mujoco_wheelleg/`.

## Smoke check

This check does not require the MuJoCo Python package or a C compiler:

```powershell
python .\tools\mujoco\wheelleg\run_wheelleg.py --check --project MINIWHEELEG-C
```

## Run

Bench PID output:

```powershell
python .\tools\mujoco\wheelleg\run_wheelleg.py --project MINIWHEELEG-C --bench --leg-branch diamond --viewer --realtime
```

Bench VMC output:

```powershell
python .\tools\mujoco\wheelleg\run_wheelleg.py --project MINIWHEELEG-C --bench --vmc --leg-branch diamond --viewer --realtime
```

Print the parameters used by the generated model:

```powershell
python .\tools\mujoco\wheelleg\run_wheelleg.py --project MINIWHEELEG-C --print-params
```

Keyboard control in the viewer:

```powershell
python .\tools\mujoco\wheelleg\run_wheelleg.py --project MINIWHEELEG-C --bench --keyboard --viewer --realtime
```

Keys:

- `W` / `S`: target speed up/down;
- `A` / `D`: yaw rate left/right;
- `Q` / `E`: target leg length down/up;
- `Z` / `C`: target foot x backward/forward;
- `X`: stop speed, yaw, and foot x target.

The default leg branch is `diamond`, which matches the expected five-bar shape.
`--leg-branch core` is kept as a diagnostic branch close to the control core's
old initialization branch.

Use a custom MJCF instead of the generated five-bar model:

```powershell
python .\tools\mujoco\wheelleg\run_wheelleg.py --project MINIWHEELEG-C --model .\tools\mujoco\wheelleg\wheelleg_minimal.xml
```

## Current limits

This runner is good for checking:

- MuJoCo setup;
- sensor sign conventions;
- wheel torque direction;
- five-bar closed-chain geometry and basic wheel-ground contact;
- bench PID/VMC output without needing the body to balance;
- diamond and core five-bar initialization branches;
- whether the reusable core can run outside FreeRTOS.

It is still not a final calibrated robot model. Geometry and control gains come
from `Robotconfig/<project>/RobotConfig.c`, and the default mass assumptions come
from `tools/wheelleg_lqr/small_3510_lqr_report.md`. Track width, wheel width,
contact material, joint damping, and detailed inertia are still simulation
assumptions. The diamond branch fixes the visible five-bar shape, but VMC
standing on that branch still needs motor direction, zero, and contact tuning.
