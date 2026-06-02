# MuJoCo wheel-leg runner

This is the first MuJoCo entry for the ARBATOS wheel-leg control core.

It is intentionally a minimal physics fixture:

- wheels are real contact wheels attached to a free base;
- front/back leg joints are exposed so `wheelleg_core.h` can compute kinematics;
- the full closed-chain five-bar leg geometry is not modeled yet;
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
`tmp/mujoco_wheelleg/arbatos_wheelleg_core_bridge.dll` on first run.

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

## Current limits

This runner is good for checking:

- MuJoCo setup;
- sensor sign conventions;
- wheel torque direction;
- whether the reusable core can run outside FreeRTOS.

It is not a faithful wheel-leg dynamics model yet. The next physics step is to
replace the dummy front/back links with a constrained five-bar leg or a reduced
leg model whose wheel position follows the same geometry as `wheelleg_core.h`.
