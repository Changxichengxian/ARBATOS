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
writes generated MJCF files to `tmp/mujoco_wheelleg/`.

## Smoke check

This check does not require the MuJoCo Python package or a C compiler:

```powershell
python .\tools\mujoco\wheelleg\run_wheelleg.py --check --project MINIWHEELEG-C
```

## Run

Bench PID output:

```powershell
python .\tools\mujoco\wheelleg\run_wheelleg.py --project MINIWHEELEG-C --bench --viewer --realtime
```

Bench VMC output:

```powershell
python .\tools\mujoco\wheelleg\run_wheelleg.py --project MINIWHEELEG-C --bench --vmc --viewer --realtime
```

Free 3D VMC standing check:

```powershell
python .\tools\mujoco\wheelleg\run_wheelleg.py --project MINIWHEELEG-C --vmc --sim-wheel-scale 0.05 --viewer --realtime
```

Leg support only, with wheel torques disabled:

```powershell
python .\tools\mujoco\wheelleg\run_wheelleg.py --project MINIWHEELEG-C --support-only --viewer --realtime
```

2D pitch-plane standing check:

```powershell
python .\tools\mujoco\wheelleg\run_wheelleg.py --project MINIWHEELEG-C --planar --support-only --viewer --realtime
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
- bench PID/VMC output without needing the body to balance;
- 2D and free-3D VMC standing checks;
- whether the reusable core can run outside FreeRTOS.

It is still not a final calibrated robot model. Link masses, inertia, friction,
motor limits, contact material, and body dimensions are conservative starting
values. The full wheel LQR output is intentionally available with
`--sim-wheel-scale 1.0`, but the current generated model needs a smaller
simulation-side wheel scale, such as `0.05`, for stable free-3D standing. The
next physics step is to measure or CAD-export those values and tune the
contact/friction model against the real wheel-leg behavior.
