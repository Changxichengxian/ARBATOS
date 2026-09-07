# ARBATOS

ARBATOS is an STM32 and Zephyr firmware workspace for RoboMaster-style robots.
It focuses on reusable low-level robot control: chassis, gimbal, shooter, arm,
wheel-leg experiments, input links, actuator output, diagnostics, telemetry, and
SD-card logging.

The repository is organized as an open, contributor-facing firmware architecture:
hardware boards, robot-specific configuration, reusable runtime code, build
entry points, tools, manuals, and legal notes are separated so new robots can be
added without copying the whole stack.

**Author:** Xie Yuhan <2811158416@qq.com>  
**Repository:** <https://github.com/Changxichengxian/ARBATOS.git>

## Latest hardware milestone

On 2026-09-06, the user confirmed normal whole-vehicle motion on HERO-M running
Zephyr, including the chassis and gimbal pitch. This vehicle now uses M-board
wiring throughout; the earlier HERO-C success applied to the old C-board wiring.
See the [hardware milestone and validation limits](tests/ZephyrMusicM/NormalOutput-20260906.md).

## Status

The current codebase is beyond a basic STM32 port. It includes:

- Seven firmware targets: `HERO-C`, `HERO-M`, `INFANTRY-A`, `SENTINEL-M`,
  `CARRIER-A`, `MINIWHEELEG-M`, and `MINIWHEELEG-C`.
- Three board support layers: DJI C board, DJI A board, and DM MC02 H7 board.
- `g_config.profile.task_modules` 显式选择的业务任务由兼容层接入 Zephyr 线程启动。
- Multiple manual input sources: DBUS/SBUS, ELRS/CRSF, image-transmission remote
  control, USB-reserved input, and board keys.
- A unified actuator command path. Control tasks write transport-independent
  commands; the transmit task maps those commands to RM, DM, MIT-style, or
  Unitree protocols.
- Runtime motor instances, a device table, controller registry, and
  `watch.runtime` observation for moving away from hard-coded robot roles.
- Diagnostics and logging through `g_watch`, `RtProf`, TF/SD binary logs,
  build identity records, runtime device records, AUX telemetry, and temporary
  AUX parameter tuning.
- Zephyr 4.4 firmware builds, source-list checks, SD log tools, a PID autotune
  tool, and a configuration pressure simulator.

Important limits are also documented here:

- `main` 是唯一继续提交的分支；`zephyr` 分支保留为已结束的探索记录。
- 七个目标的正式构建入口是 `zephyr/`，使用 Zephyr 4.4、CMake、Ninja 和
  OpenOCD。它使用仓库内的显式源码清单，不读取 `.uvprojx`。
- `projects/` 中的 Keil、CubeMX、FreeRTOS 工程和旧 GCC 生成工具只作为
  显式 `legacy` 历史参考；当前构建、检查和 CI 不要求安装 Keil。
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
- `legal/ThirdParty.md`
- `legal/CONTRIBUTING.md`
- `legal/CLA.md`

Third-party components, vendor SDKs, libraries, and reference code keep their own
licenses.

## Repository Layout

```text
ARBATOS/
|-- boards/        # Board support packages and board-specific ports
|-- zephyr/        # 正式 Zephyr 4.4 工程、七目标配置和板级定义
|-- projects/      # 旧 Keil/CubeMX 工程，仅作 legacy 历史参考
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
zephyr/
  正式 Zephyr 4.4 工程、七目标 CMake 预设、板级定义、端口和显式源码清单。
  构建不读取 Keil 工程文件。

projects/<TARGET>/
  旧 Keil/CubeMX/FreeRTOS 工程，保留给历史对照和有意继续维护旧路线的人。

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
| `DjiCF407` | STM32F407 | DJI C board support |
| `DjiAF427` | STM32F427 | DJI A board support |
| `DmMc02H7` | STM32H723 | DM MC02 H7 board support |

## Firmware Targets

| Target | Keil project | Robotconfig | Board |
|---|---|---|---|
| `HERO-C` | `projects/HERO-C/MDK-ARM/HERO-C.uvprojx` | `Robotconfig/HERO-C` | `boards/DjiCF407` |
| `HERO-M` | `projects/HERO-M/MDK-ARM/HERO-M.uvprojx` | `Robotconfig/HERO-M` | `boards/DmMc02H7` |
| `INFANTRY-A` | `projects/INFANTRY-A/MDK-ARM/INFANTRY-A.uvprojx` | `Robotconfig/INFANTRY-A` | `boards/DjiAF427` |
| `SENTINEL-M` | `projects/SENTINEL-M/MDK-ARM/SENTINEL-M.uvprojx` | `Robotconfig/SENTINEL-M` | `boards/DmMc02H7` |
| `CARRIER-A` | `projects/CARRIER-A/MDK-ARM/CARRIER-A.uvprojx` | `Robotconfig/CARRIER-A` | `boards/DjiAF427` |
| `MINIWHEELEG-M` | `projects/MINIWHEELEG-M/MDK-ARM/MINIWHEELEG-M.uvprojx` | `Robotconfig/MINIWHEELEG-M` | `boards/DmMc02H7` |
| `MINIWHEELEG-C` | `projects/MINIWHEELEG-C/MDK-ARM/MINIWHEELEG-C.uvprojx` | `Robotconfig/MINIWHEELEG-C` | `boards/DjiCF407` |

## Runtime Flow

当前固件由 Zephyr 启动线程；原有 `g_config.profile.task_modules` 和业务任务通过兼容层继续使用。下面的 FreeRTOS 路径仅说明 `projects/` 中保留的 legacy 工程：

```text
main.c
  |
  +-- HAL and CubeMX peripheral initialization
  +-- board and shared module initialization
  +-- ManualInputInit()
  +-- osKernelStart()
        |
        +-- MX_FREERTOS_Init()
              |
              +-- ControlMgrInit()
              +-- create static tasks
              +-- create enabled modules from g_config.profile.task_modules
```

legacy F4 工程主要在 `projects/<TARGET>/Core/Src/freertos.c` 创建任务。
legacy DM MC02 H7 工程还有板级入口：
`boards/DmMc02H7/app/BoardMain.c` and
`boards/DmMc02H7/app/BoardFreertos.c`.

Known task module IDs are defined in
`shared/application/robot/RobotConfigSchema.h`, and their names and helper
functions live in `shared/application/robot/RobotTaskProfile.h`.

Current modules include:

The `task.*` strings are profile/config identifiers. They are kept stable as
data keys even when the C files and functions use readable PascalCase names.

| Module name | Purpose |
|---|---|
| `task.startup_service` | startup services, including delayed USB setup |
| `task.calibration` | calibration services |
| `task.imu` | IMU fusion and temperature control |
| `task.classic_chassis` | classic wheeled chassis control |
| `task.wheelleg_mit` | MIT-style wheel-leg experiment control |
| `task.single_gimbal` | single-yaw gimbal control |
| `task.DualYawGimbal` | dual-yaw gimbal control |
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
                    ManualInput
                           |
                    ControlInput
                           |
        +------------------+------------------+
        |                  |                  |
 ChassisControlTask  GimbalControlTask  Shoot / Arm / WheelLeg
        |                  |                  |
        +------------------+------------------+
                           |
                    LowCmd
                           |
                  CanTxTask
                           |
                    CAN / RS485 output
```

Feedback is handled separately:

```text
CAN interrupt
  |
BspCan RX ring buffer
  |
CanRxTask
  |
CanReceive / MotorInst
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
| Manual input | `shared/application/input/ManualInput.c` |
| Logical input mapping | `shared/application/input/ControlInput.c` |
| Image remote input | `shared/application/input/ImageRemoteLink.c` |
| ELRS/CRSF input | `shared/application/input/ElrsTask.c` |
| Host link | `shared/application/comm/host/HostLinkTask.c` |
| Vision link | `shared/application/comm/vision/VisionLink.c` |
| Referee link | `shared/application/comm/referee/RefereeRxTask.c` |
| External motion intent | `shared/application/robot/ExternalMotionIntent.c` |
| Actuator commands | `shared/application/robot/LowCmd.c` |
| Device configuration view | `shared/application/robot/RobotDeviceConfig.h` |
| Runtime state store | `shared/application/robot/StateStore.c`, `RobotState.h` |
| Controller manager | `shared/application/robot/ControlMgr.c` |
| Motor instances | `shared/application/motors/MotorInst.c` |
| Motor model database | `shared/application/motors/MotorModelDb.c` |
| CAN feedback | `shared/application/comm/can/CanRxTask.c`, `CanReceive.c` |
| CAN commands | `shared/application/comm/can/CanTxTask.c` |
| Chassis control | `shared/application/chassis/ChassisControlTask.c` |
| Gimbal control | `shared/application/gimbal/GimbalControlTask.c` |
| Shooter control | `shared/application/shoot/Shoot.c` |
| Arm motion | `shared/application/arm/ArmMotion.c` |
| Wheel-leg control | `shared/application/wheelleg/WheelLegMitTask.c` |
| Battery monitor | `shared/application/services/battery/BatteryMonitorTask.c` |
| Calibration | `shared/application/services/calibration/` |
| Diagnostics | `shared/application/services/diagnostics/Watch.c`, `RtProf.c` |
| SD logging | `shared/application/services/storage/SdLog.c`, `SdLogTask.c` |

## Configuration Model

Each target owns a `Robotconfig/<TARGET>/RobotConfig.h` and `RobotConfig.c`.

The main runtime object is `g_config`. It contains:

- `profile`: explicit task module selection.
- `devices`: runtime device table used by motor instances and diagnostics.
- `motor`: motor mounting and protocol configuration.
- `gimbal`, `dual_gimbal`, `chassis`, `wheelleg_mit`, `shoot`, `arm_j0_unitree`:
  subsystem parameters.
- `manual_input` and `input`: input source policy and logical channel mapping.
- `AuxTelem`: AUX telemetry signal selection.
- `detect`: online detection rules.
- `imu`, `voltage`, `power`, `buzzer`, `led`, `sdlog`, and `test`: common
  services and debug configuration.

Temporary AUX tuning is limited to fields listed in each target's
`g_config_blocks` table. Motor mounting is intentionally not treated as a normal
runtime tuning field; changing motor wiring or model usually requires editing
`Robotconfig/<TARGET>/RobotConfig.c`, rebuilding, and reflashing.

## Build and Local Checks

Install Zephyr 4.4 and its SDK, then ensure `west`, CMake, Ninja and OpenOCD are
available to the terminal. The exact CLion setup, CMake preset selection and
OpenOCD download configuration are in [the CLion and Zephyr guide](manual/clion-zephyr.md).
CLion 已安装；工程界面导入和硬件调试仍需分别验收，不能由命令行构建代替。

Build the default target (`HERO-M`) from PowerShell:

```powershell
pwsh -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\tools\build.ps1
```

Build one target or start all seven from clean build directories:

```powershell
pwsh -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\tools\build.ps1 -Project SENTINEL-M -Pristine
pwsh -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\tools\build.ps1 -Project all -Pristine
```

The normal output location is `out/zephyr/<target>/`; it is ignored by Git. Run
the source/configuration check separately:

```powershell
pwsh -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\tools\build.ps1 -Action check -Project all
```

`check` validates the Zephyr source list, formal target configuration and syntax
paths. It does not claim a firmware build or real-board result.

## Tools

| Tool | Purpose |
|---|---|
| `tools/build.ps1` | Zephyr check and build entry point; default is `HERO-M` |
| `tools/CheckZephyr.py` | Zephyr source-list and formal-configuration check |
| `tools/CheckAll.ps1` | legacy Keil/project check, invoked only with `-Action legacy-check` |
| `tools/build/ProjectManifest.py` | legacy Keil project manifest tool |
| `tools/build/GccProject.py` | legacy generated GCC/CMake tool |
| `tools/GenBuildInfo.ps1` | generates `shared/generated/build_info_autogen.h` for firmware logs |
| `tools/sim/RobotSim.py` | estimates CAN and CPU pressure from current configuration |
| `tools/sdlog/SdLogViewer.py` | opens the SD log web viewer and exports records |
| `tools/sdlog/SdLogDecompress.py` | removes LZ4 block compression from current log files |
| `tools/PidAutotune/arbatos_PidAutotune.py` | PID autotune helper |
| `tools/Mp3ToU8/` | converts MP3 files to unsigned 8-bit PCM `.U8` files for buzzer playback |

Example simulator usage:

```powershell
python .\tools\sim\RobotSim.py --project HERO-C
python .\tools\sim\RobotSim.py --project MINIWHEELEG-C --json
```

The simulator is a configuration pressure check, not a physics simulator.

## Diagnostics and Logging

Main diagnostics surfaces:

- `g_watch`: watch-window friendly runtime state.
- `Watch.c`: task, device, actuator, controller, and fault summary.
- `RtProf.c`: loop timing, maximum time, and over-budget counters.
- `DetectTask.c`: target-specific online detection and status aggregation.
- `SdLog.c` and `SdLogTask.c`: TF/SD binary logging through an in-memory ring
  buffer and a low-priority file flush task.
- `BUILD_INFO`: target, board, 16-character Git commit prefix, dirty flag, build
  time, config CRC, and schema information written into logs.
- `HostLinkTask.c`: AUX telemetry and temporary parameter tuning.

High-rate tasks may call `SdLogWrite()`, but that still copies payload data and
enters a short critical section. New high-rate logs should be added carefully,
with frequency, payload size, and worst-case loop cost checked through real
`RtProf` data.

See `manual/sdlog.md` for log usage, decompression, baseline retention, and tag
rules. See `manual/evaluation-boundaries.md` for SD release discipline and the
CAN hardware boundary used when evaluating the repository.

## Quick Start

For a new user:

1. 按 [CLion 和 Zephyr 开发环境](manual/clion-zephyr.md) 准备 Zephyr 4.4、SDK、CMake、Ninja 和 OpenOCD。
2. 在 CLion 中打开仓库根目录，选择 `zephyr/CMakePresets.json` 中的目标预设；工程界面导入和硬件调试仍需分别验收。
3. Check `Robotconfig/<TARGET>/RobotConfig.c`, especially `g_config.profile`,
   `task_modules`, `g_config.devices`, `g_config.motor`, input mapping, and safe
   switch positions.
4. Check `boards/<BOARD>/` for UART, CAN, IMU, buzzer, key, SD card, and port
   assignments.
5. 构建固件，并按 CLion 手册中对应板卡的 OpenOCD 配置下载。
6. Before enabling full power, confirm input, IMU, CAN feedback, task status,
   `g_watch`, AUX telemetry, and SD logs.
7. Bring up subsystems in this order: IMU, CAN feedback, single subsystem
   control, then whole-robot integration.

For detailed workflows, start with `QuickStart.md` and `manual/README.md`.

## Adding a Robot Target

1. Copy the closest existing `Robotconfig/<TARGET>/` and `projects/<TARGET>/`.
2. 在 `zephyr/targets/`、Zephyr 板级定义和 CMake 预设中加入目标；`projects/` 只有确需维护 legacy 路线时才同步。
3. Update include paths so the project references exactly one
   `Robotconfig/<TARGET>`.
4. Set target identity macros in `RobotConfig.h`: `ARBATOS_TARGET_NAME`,
   `ARBATOS_BOARD_NAME`, `ROBOT_PROFILE_KIND`, `ROBOT_BOARD_KIND`,
   `ROBOT_BOARD_CPU_HZ`, `ROBOT_BOARD_CAN_BUS_COUNT`, and `ROBOT_BOARD_HAS_FPU`.
5. Configure `g_config.profile.task_modules`.
6. Configure `g_config.devices` and `g_config.motor`.
7. Configure input mapping, safe switches, detection items, telemetry, and logs.
8. Add target stubs or target-specific files only when shared code cannot cover
   the target.
9. 运行 `tools/build.ps1 -Action check -Project <TARGET>`，再从干净目录构建该 Zephyr 目标。

## Adding a Board

1. Create `boards/<BOARD>/`.
2. Add board port configuration for CAN, UART, SPI, I2C, IMU, key, buzzer, SD
   card, USB, PWM, and other board peripherals.
3. 在 Zephyr 板级定义、`zephyr/targets/` 和启动配置中加入该板的正式入口。
4. 只有明确维护 legacy 路线时，才更新 `projects/<TARGET>/` 和 Keil 工程路径。

## Adding a Motor Model or Protocol

1. Add the model enum in the target-compatible `RobotConfig.h`.
2. Add the model entry, protocol capability, feedback format, limits, reduction
   ratio, and control range in `shared/application/motors/MotorModelDb.c`.
3. Add or extend protocol drivers if the existing RM, DM, MIT-style, or Unitree
   paths do not cover the model.
4. Wire receive parsing through `CanReceive.c` and transmit formatting through
   `CanTxTask.c`.
5. Mount the model through `g_config.motor` in the relevant Robotconfig.

## Adding a Task Module

1. Add a `ROBOT_TASK_MODULE_*` value in
   `shared/application/robot/RobotConfigSchema.h`.
2. Add the module name in `RobotProfileKnownModules()` in
   `shared/application/robot/RobotTaskProfile.h`.
3. Add the module to the relevant target's `g_config.profile.task_modules`.
4. 把任务源码加入 Zephyr 显式清单，并在 Zephyr 启动映射中接入该模块；旧
   FreeRTOS 任务入口只有维护 legacy 路线时才同步修改。
5. Add diagnostics, log fields, and a minimal validation path.
6. Update `tools/CheckAll.ps1` if the module requires source or task-creation
   consistency checks.

## Documentation

- `QuickStart.md`: first-pass bring-up guide.
- `manual/README.md`: manual index.
- `manual/clion-zephyr.md`: CLion、Zephyr 4.4 和 OpenOCD 的正式工作流。
- `manual/new-target.md`: adding a new target.
- `manual/bringup-checklist.md`: real-robot bring-up checklist.
- `manual/pid-tuning.md`: PID tuning flow.
- `manual/sdlog.md`: SD logging and replay.
- `manual/evaluation-boundaries.md`: SD release discipline and CAN scoring
  boundary.
- `manual/coding-style.md`: project coding style, Chinese comment rules, and
  legacy-style handling.
- `manual/runtime-architecture.md`: direction for device and controller instance
  based runtime evolution.
- `AlgorithmAccessProtocol.md`: compact algorithm link protocol and external
  chassis-motion command path.
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
