# MuJoCo wheel-leg runner

This is the MuJoCo entry for the ARBATOS wheel-leg control core.

The runner generates a project-specific five-bar wheel-leg model from
`Robotconfig/<project>/config.c`:

- wheels are real contact wheels attached to a free base;
- front/back active joints use the same names as `wheelleg_core.h` expects;
- each side has a constrained five-bar leg with passive knee joints;
- the wheel carrier follows the closed chain through MuJoCo equality constraints;
- the runner calls a tiny native C bridge that includes `wheelleg_core.h`.

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

The runner builds `tools/mujoco/wheelleg/wheelleg_core_bridge.c` into
`tmp/mujoco_wheelleg/arbatos_wheelleg_core_bridge.dll` on first run. It also
writes the generated MJCF to `tmp/mujoco_wheelleg/wheelleg_fivebar_<project>.xml`.

## Smoke check

This check does not require the MuJoCo Python package or a C compiler:

```powershell
python .\tools\mujoco\wheelleg\run_wheelleg.py --check --project MINIWHEELEG-C
```

## Run

Headless:

```powershell
python .\tools\mujoco\wheelleg\run_wheelleg.py --project MINIWHEELEG-C --duration-s 5
```

Viewer:

```powershell
python .\tools\mujoco\wheelleg\run_wheelleg.py --project MINIWHEELEG-C --viewer --realtime
```

Use VMC joint torques:

```powershell
python .\tools\mujoco\wheelleg\run_wheelleg.py --project MINIWHEELEG-C --vmc --viewer --realtime
```

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
- whether the reusable core can run outside FreeRTOS.

It is still not a final calibrated robot model. Link masses, inertia, friction,
motor limits, contact material, and body dimensions are conservative starting
values. The next physics step is to measure or CAD-export those values and tune
the contact/friction model against the real wheel-leg behavior.
