# ARBATOS

ARBATOS is an STM32 and FreeRTOS firmware workspace for RoboMaster-style robots.
It focuses on reusable low-level robot control: chassis, gimbal, shooter, arm,
wheel-leg experiments, input links, actuator output, diagnostics, telemetry, and
SD-card logging.

The repository is organized as an open, contributor-facing firmware architecture:
hardware boards, robot-specific configuration, reusable runtime code, build
entry points, tools, manuals, and legal notes are separated so new robots can be
added without copying the whole stack.

**Author:** Xie Yuhan <2811158416@qq.com>  
**Repository:** <https://github.com/Changxichengxian/ARBATOS.git>

## Status

The current codebase is beyond a basic STM32 port. It includes:

- Seven firmware targets: `HERO-C`, `HERO-M`, `INFANTRY-A`, `SENTINEL-M`,
  `CARRIER-A`, `MINIWHEELEG-M`, and `MINIWHEELEG-C`.
- Three board support layers: DJI C board, DJI A board, and DM MC02 H7 board.
- Explicit FreeRTOS task selection through `g_config.profile.task_modules`.
- Multiple manual input sources: DBUS/SBUS, ELRS/CRSF, image-transmission remote
  control, USB-reserved input, and board keys.
- A unified actuator command path. Control tasks write transport-independent
  commands; the transmit task maps those commands to RM, DM, MIT-style, or
  Unitree protocols.
- Runtime motor instances, a device table, controller registry, and
  `watch.runtime` observation for moving away from hard-coded robot roles.
- Diagnostics and logging through `g_watch`, `rt_profiler`, TF/SD binary logs,
  build identity records, runtime device records, AUX telemetry, and temporary
  AUX parameter tuning.
- Local checks, Keil project manifest extraction, generated GCC/CMake firmware
  builds, SD log tools, a PID autotune tool, and a configuration pressure
  simulator.

Important limits are also documented here:

- Keil MDK-ARM projects are still the build source of truth.
- GCC/CMake builds are generated from the Keil project manifests and are
  currently buildable for all seven targets with `arm-none-eabi-gcc`, CMake, and
  Ninja. Generated files live under `build/gcc/` and are not maintained by hand.
- Command-line Keil builds still depend on the local UV4 path and installed
  device packs. The repository check does not run a real Keil Rebuild.
- High-rate control paths should read configuration through cached or snapshot
  views instead of repeatedly walking `g_config`; local checks guard the main
  high-rate boundaries.
- Dual-yaw gimbal and MIT wheel-leg control paths are wired. Subsystem-level
  protection already exists, while real-robot validation and a more unified
  safety policy are still active work.

## License

ARBATOS original code and documentation are licensed under the Apache License
2.0 unless a file or directory has its own license notice.

Apache-2.0 is a permissive open source license. It allows use, modification,
distribution, private use, and commercial use, while requiring preservation of
license and attribution notices. It also includes an explicit patent grant from
contributors.

See:

- `LICENSE`
- `legal/THIRD_PARTY.md`
- `legal/CONTRIBUTING.md`
- `legal/CLA.md`

Third-party components, vendor SDKs, libraries, and reference code keep their own
licenses.

## Repository Layout

```text
ARBATOS/
|-- boards/        # Board support packages and board-specific ports
|-- projects/      # Buildable firmware projects; Keil is the source manifest
|-- Robotconfig/   # Robot target parameters and target-specific glue
|-- shared/        # Reusable runtime, control, communication, HAL, and components
|-- manual/        # Bring-up, tuning, logging, and integration manuals
|-- tools/         # Local checks, manifest tools, log tools, simulation, utilities
|-- legal/         # License, contribution, commercial-use, and third-party notes
|-- local/         # Local-only notes, logs, and private working files
`-- .github/       # CI workflow for repository checks
```

The important separation is:

- `projects/<TARGET>/` answers "how is this firmware built and started?"
- `Robotconfig/<TARGET>/` answers "how is this robot configured?"
- `boards/<BOARD>/` answers "how does this control board connect to hardware?"
- `shared/` answers "what logic can multiple robots reuse?"
- `manual/` answers "how do I bring up, tune, or debug the robot?"

## Architecture

ARBATOS uses a four-layer firmware layout.

```text
projects/<TARGET>/
  Keil project, CubeMX Core files, middleware, startup, and build entry point.
  The generated GCC/CMake route reads these Keil project files instead of
  keeping a second hand-written project list.

Robotconfig/<TARGET>/
  Robot profile, task module list, device table, motor mounting, PID, input,
  detection, telemetry, logging, and target-specific stubs

boards/<BOARD>/
  Board ports, pin and peripheral mapping, IMU integration, board startup,
  board-specific FreeRTOS task creation when needed

shared/
  Cross-target control tasks, input links, host links, actuator command layer,
  motor protocol support, diagnostics, SD logging, HAL wrappers, and algorithms
```

This layout keeps reusable logic out of target folders. A new robot should mostly
need a new `Robotconfig/<TARGET>/` and `projects/<TARGET>/`, plus board work only
when the hardware changes.

## Supported Boards

| Board | MCU | Notes |
|---|---:|---|
| `DJI_C_F407` | STM32F407 | DJI C board support |
| `DJI_A_F427` | STM32F427 | DJI A board support |
| `DM_MC02_H7` | STM32H723 | DM MC02 H7 board support |

## Firmware Targets

| Target | Keil project | Robotconfig | Board |
|---|---|---|---|
| `HERO-C` | `projects/HERO-C/MDK-ARM/HERO-C.uvprojx` | `Robotconfig/HERO-C` | `boards/DJI_C_F407` |
| `HERO-M` | `projects/HERO-M/MDK-ARM/HERO-M.uvprojx` | `Robotconfig/HERO-M` | `boards/DM_MC02_H7` |
| `INFANTRY-A` | `projects/INFANTRY-A/MDK-ARM/INFANTRY-A.uvprojx` | `Robotconfig/INFANTRY-A` | `boards/DJI_A_F427` |
| `SENTINEL-M` | `projects/SENTINEL-M/MDK-ARM/SENTINEL-M.uvprojx` | `Robotconfig/SENTINEL-M` | `boards/DM_MC02_H7` |
| `CARRIER-A` | `projects/CARRIER-A/MDK-ARM/CARRIER-A.uvprojx` | `Robotconfig/CARRIER-A` | `boards/DJI_A_F427` |
| `MINIWHEELEG-M` | `projects/MINIWHEELEG-M/MDK-ARM/MINIWHEELEG-M.uvprojx` | `Robotconfig/MINIWHEELEG-M` | `boards/DM_MC02_H7` |
| `MINIWHEELEG-C` | `projects/MINIWHEELEG-C/MDK-ARM/MINIWHEELEG-C.uvprojx` | `Robotconfig/MINIWHEELEG-C` | `boards/DJI_C_F407` |

## Runtime Flow

Startup follows the usual STM32 and FreeRTOS path:

```text
main.c
  |
  +-- HAL and CubeMX peripheral initialization
  +-- board and shared module initialization
  +-- manual_input_init()
  +-- osKernelStart()
        |
        +-- MX_FREERTOS_Init()
              |
              +-- control_manager_init()
              +-- create static tasks
              +-- create enabled modules from g_config.profile.task_modules
```

F4 targets mainly create tasks in `projects/<TARGET>/Core/Src/freertos.c`.
The DM MC02 H7 board also has board-level entry points in
`boards/DM_MC02_H7/app/board_main.c` and
`boards/DM_MC02_H7/app/board_freertos.c`.

Known task module IDs are defined in
`shared/application/robot/robot_config_schema.h`, and their names and helper
functions live in `shared/application/robot/robot_task_profile.h`.

Current modules include:

| Module name | Purpose |
|---|---|
| `task.startup_service` | startup services, including delayed USB setup |
| `task.calibration` | calibration services |
| `task.imu` | IMU fusion and temperature control |
| `task.classic_chassis` | classic wheeled chassis control |
| `task.wheelleg_mit` | MIT-style wheel-leg experiment control |
| `task.single_gimbal` | single-yaw gimbal control |
| `task.dual_yaw_gimbal` | dual-yaw gimbal control |
| `task.arm` | arm control task |
| `task.can_feedback_rx` | drains CAN RX queues and updates motor feedback |
| `task.can_command_tx` | sends unified actuator commands to CAN/RS485 protocols |
| `task.rc_sbus` | DBUS/SBUS input parsing |
| `task.elrs_link` | ELRS/CRSF input parsing |
| `task.host_link` | USB/AUX host link, vision, telemetry, and tuning |
| `task.referee_rx` | RoboMaster referee protocol parsing |
| `task.battery_monitor` | battery and voltage monitoring |
| `task.servo` | servo output |
| `task.health_monitor` | online detection and runtime status summary |
| `task.status_led` | status LED and prompt output |
| `task.sdlog` | low-priority SD-card log flush task |

## Control and Actuator Path

Manual input sources are merged before control tasks read them:

```text
DBUS/SBUS       ELRS/CRSF       image remote       board keys
   |               |                 |                 |
   +---------------+-----------------+-----------------+
                           |
                    manual_input
                           |
                    control_input
                           |
        +------------------+------------------+
        |                  |                  |
 chassis_control_task  gimbal_control_task  shoot / arm / wheel-leg
        |                  |                  |
        +------------------+------------------+
                           |
                    LowCmd
                           |
                  can_command_tx_task
                           |
                    CAN / RS485 output
```

Feedback is handled separately:

```text
CAN interrupt
  |
bsp_can RX ring buffer
  |
can_feedback_rx_task
  |
CAN_receive / motor_instance
  |
LowState and legacy motor feedback structs
  |
control tasks / g_watch / sdlog
```

Control tasks should write actuator commands by role or actuator ID, not by raw
CAN frame details. Motor model, protocol, bus, CAN ID, limits, and feedback
format are resolved through configuration, the motor model database, motor
instances, and the unified transmit task.

## Shared Runtime Code

Useful entry points:

| Area | Main files |
|---|---|
| Manual input | `shared/application/input/manual_input.c` |
| Logical input mapping | `shared/application/input/control_input.c` |
| Image remote input | `shared/application/input/image_remote_link.c` |
| ELRS/CRSF input | `shared/application/input/elrs_task.c` |
| Host link | `shared/application/comm/host/host_link_task.c` |
| Vision link | `shared/application/comm/vision/vision_link.c` |
| Referee link | `shared/application/comm/referee/referee_rx_task.c` |
| Actuator commands | `shared/application/robot/LowCmd.c` |
| Device configuration view | `shared/application/robot/robot_device_config.h` |
| Runtime state store | `shared/application/robot/state_store.c`, `robot_state.h` |
| Controller manager | `shared/application/robot/control_manager.c` |
| Motor instances | `shared/application/motors/motor_instance.c` |
| Motor model database | `shared/application/motors/motor_model_db.c` |
| CAN feedback | `shared/application/comm/can/can_feedback_rx_task.c`, `CAN_receive.c` |
| CAN commands | `shared/application/comm/can/can_command_tx_task.c` |
| Chassis control | `shared/application/chassis/chassis_control_task.c` |
| Gimbal control | `shared/application/gimbal/gimbal_control_task.c` |
| Shooter control | `shared/application/shoot/shoot.c` |
| Arm motion | `shared/application/arm/arm_motion.c` |
| Wheel-leg control | `shared/application/wheelleg/wheelleg_mit_task.c` |
| Battery monitor | `shared/application/services/battery/battery_monitor_task.c` |
| Calibration | `shared/application/services/calibration/` |
| Diagnostics | `shared/application/services/diagnostics/watch.c`, `rt_profiler.c` |
| SD logging | `shared/application/services/storage/sdlog.c`, `sdlog_task.c` |

## Configuration Model

Each target owns a `Robotconfig/<TARGET>/config.h` and `config.c`.

The main runtime object is `g_config`. It contains:

- `profile`: explicit task module selection.
- `devices`: runtime device table used by motor instances and diagnostics.
- `motor`: motor mounting and protocol configuration.
- `gimbal`, `dual_gimbal`, `chassis`, `wheelleg_mit`, `shoot`, `arm_j0_unitree`:
  subsystem parameters.
- `manual_input` and `input`: input source policy and logical channel mapping.
- `aux_telem`: AUX telemetry signal selection.
- `detect`: online detection rules.
- `imu`, `voltage`, `power`, `buzzer`, `led`, `sdlog`, and `test`: common
  services and debug configuration.

Temporary AUX tuning is limited to fields listed in each target's
`g_config_blocks` table. Motor mounting is intentionally not treated as a normal
runtime tuning field; changing motor wiring or model usually requires editing
`Robotconfig/<TARGET>/config.c`, rebuilding, and reflashing.

## Build and Local Checks

Install:

- Keil MDK-ARM v5 and the required STM32F4 / STM32H7 device packs for the Keil
  route.
- Python 3 for repository tools.
- `arm-none-eabi-gcc`, CMake, and Ninja for the GCC/CMake route.

Open and build a target from Keil, for example:

```text
projects/HERO-C/MDK-ARM/HERO-C.uvprojx
```

Build the same target through the generated GCC/CMake route:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\build.ps1 -Action gcc-build -Project HERO-C
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\build.ps1 -Action gcc-build -Project all
```

This route reads the Keil `.uvprojx`, writes generated CMake files under
`build/gcc/<TARGET>/`, translates the startup and linker files, swaps in GCC
FreeRTOS ports and compatibility sources, then builds `.elf`, `.hex`, and `.bin`
outputs. The generated build directory is ignored by Git.

Run repository checks from PowerShell:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\build.ps1 -Action check
```

Inspect the Keil project manifest:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\build.ps1 -Action manifest -Project HERO-C
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\build.ps1 -Action manifest -Project all -Json
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\build.ps1 -Action manifest -Project all -FailOnGccBlockers
```

Probe local command-line tool availability:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\build.ps1 -Action probe
```

The check script validates project references, Robotconfig coverage, task module
mapping, profile identity macros, profiler descriptors, Python tool syntax,
simulation smoke tests, build manifest extraction, stale text patterns, and
high-rate API boundaries. It does not run a real Keil Rebuild or a full
GCC/CMake compile; use `-Action gcc-build` when compiler verification is needed.

## Tools

| Tool | Purpose |
|---|---|
| `tools/build.ps1` | top-level check, manifest, tool-probe, GCC generation, and GCC build entry point |
| `tools/check_all.ps1` | local and CI validation script |
| `tools/build/project_manifest.py` | extracts Keil project metadata and GCC readiness notes |
| `tools/build/gcc_project.py` | generates CMake/GCC build files from Keil project metadata |
| `tools/gen_build_info.ps1` | generates `shared/generated/build_info_autogen.h` for firmware logs |
| `tools/sim/robot_sim.py` | estimates CAN and CPU pressure from current configuration |
| `tools/sdlog/sdlog_viewer.py` | opens the SD log web viewer and exports records |
| `tools/sdlog/sdlog_decompress.py` | removes LZ4 block compression from current log files |
| `tools/pid_autotune/arbatos_pid_autotune.py` | PID autotune helper |
| `tools/mp3_to_u8/` | converts MP3 files to unsigned 8-bit PCM `.U8` files for buzzer playback |

Example simulator usage:

```powershell
python .\tools\sim\robot_sim.py --project HERO-C
python .\tools\sim\robot_sim.py --project MINIWHEELEG-C --json
```

The simulator is a configuration pressure check, not a physics simulator.

## Diagnostics and Logging

Main diagnostics surfaces:

- `g_watch`: watch-window friendly runtime state.
- `watch.c`: task, device, actuator, controller, and fault summary.
- `rt_profiler.c`: loop timing, maximum time, and over-budget counters.
- `detect_task.c`: target-specific online detection and status aggregation.
- `sdlog.c` and `sdlog_task.c`: TF/SD binary logging through an in-memory ring
  buffer and a low-priority file flush task.
- `BUILD_INFO`: target, board, Git commit, dirty flag, build time, config CRC, and
  schema information written into logs.
- `host_link_task.c`: AUX telemetry and temporary parameter tuning.

High-rate tasks may call `sdlog_write()`, but that still copies payload data and
enters a short critical section. New high-rate logs should be added carefully,
with frequency, payload size, and worst-case loop cost checked through real
`rt_profiler` data.

See `manual/sdlog.md` for log usage, decompression, baseline retention, and tag
rules.

## Quick Start

For a new user:

1. Install Keil MDK-ARM v5 and the required STM32 device packs.
2. Open a target project such as `projects/HERO-C/MDK-ARM/HERO-C.uvprojx`.
3. Check `Robotconfig/<TARGET>/config.c`, especially `g_config.profile`,
   `task_modules`, `g_config.devices`, `g_config.motor`, input mapping, and safe
   switch positions.
4. Check `boards/<BOARD>/` for UART, CAN, IMU, buzzer, key, SD card, and port
   assignments.
5. Build and flash the firmware.
6. Before enabling full power, confirm input, IMU, CAN feedback, task status,
   `g_watch`, AUX telemetry, and SD logs.
7. Bring up subsystems in this order: IMU, CAN feedback, single subsystem
   control, then whole-robot integration.

For detailed workflows, start with `QUICK_START.md` and `manual/README.md`.

## Adding a Robot Target

1. Copy the closest existing `Robotconfig/<TARGET>/` and `projects/<TARGET>/`.
2. Rename the Keil project and output target.
3. Update include paths so the project references exactly one
   `Robotconfig/<TARGET>`.
4. Set target identity macros in `config.h`: `ARBATOS_TARGET_NAME`,
   `ARBATOS_BOARD_NAME`, `ROBOT_PROFILE_KIND`, `ROBOT_BOARD_KIND`,
   `ROBOT_BOARD_CPU_HZ`, `ROBOT_BOARD_CAN_BUS_COUNT`, and `ROBOT_BOARD_HAS_FPU`.
5. Configure `g_config.profile.task_modules`.
6. Configure `g_config.devices` and `g_config.motor`.
7. Configure input mapping, safe switches, detection items, telemetry, and logs.
8. Add target stubs or target-specific files only when shared code cannot cover
   the target.
9. Run `tools/build.ps1 -Action check`, then build in Keil. If the target should
   support the command-line route too, also run
   `tools/build.ps1 -Action gcc-build -Project <TARGET>`.

## Adding a Board

1. Create `boards/<BOARD>/`.
2. Add board port configuration for CAN, UART, SPI, I2C, IMU, key, buzzer, SD
   card, USB, PWM, and other board peripherals.
3. Add board startup or board-level FreeRTOS code only if the project needs a
   board-owned entry point.
4. Create or update `projects/<TARGET>/` so the Keil project includes the new
   board paths.

## Adding a Motor Model or Protocol

1. Add the model enum in the target-compatible `config.h`.
2. Add the model entry, protocol capability, feedback format, limits, reduction
   ratio, and control range in `shared/application/motors/motor_model_db.c`.
3. Add or extend protocol drivers if the existing RM, DM, MIT-style, or Unitree
   paths do not cover the model.
4. Wire receive parsing through `CAN_receive.c` and transmit formatting through
   `can_command_tx_task.c`.
5. Mount the model through `g_config.motor` in the relevant Robotconfig.

## Adding a Task Module

1. Add a `ROBOT_TASK_MODULE_*` value in
   `shared/application/robot/robot_config_schema.h`.
2. Add the module name in `robot_profile_known_modules()` in
   `shared/application/robot/robot_task_profile.h`.
3. Add the module to the relevant target's `g_config.profile.task_modules`.
4. Add the task source file and task creation entry in the target project or
   board-level FreeRTOS file.
5. Add diagnostics, log fields, and a minimal validation path.
6. Update `tools/check_all.ps1` if the module requires source or task-creation
   consistency checks.

## Documentation

- `QUICK_START.md`: first-pass bring-up guide.
- `manual/README.md`: manual index.
- `manual/new-target.md`: adding a new target.
- `manual/bringup-checklist.md`: real-robot bring-up checklist.
- `manual/pid-tuning.md`: PID tuning flow.
- `manual/sdlog.md`: SD logging and replay.
- `manual/runtime-architecture.md`: direction for device and controller instance
  based runtime evolution.
- `projects/README.md`: project layer details.
- `Robotconfig/README.md`: robot configuration layer details.
- `boards/README.md`: board support layer details.
- `shared/README.md`: shared runtime layer details.
- `tools/README.md`: local tool details.
- `legal/README.md`: legal document index.

## Contributing

Before sending a patch or pull request:

- Read `legal/CONTRIBUTING.md` and `legal/CLA.md`.
- Make sure you have the right to contribute the code, data, or documentation.
- Keep target parameters in `Robotconfig/`, board ports in `boards/`, reusable
  logic in `shared/`, and build entry changes in `projects/`.
- Explain which targets, boards, or shared modules are affected.
- List any new third-party dependency and its license.
- Run:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\build.ps1 -Action check
```

The GitHub workflow at `.github/workflows/check-all.yml` runs the same local
check path.
