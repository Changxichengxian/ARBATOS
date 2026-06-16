# ARBATOS build tools

This directory contains the scriptable build-facing entry points.

Current status:

- `tools/build.ps1 -Action check` runs the repository checks used by CI.
- `tools/build.ps1 -Action manifest -Project HERO-C` prints the source files,
  include directories, defines, linker script, compiler version, and GCC
  readiness notes extracted from the Keil project.
- `tools/build.ps1 -Action gcc -Project HERO-C` generates a CMake/GCC build view
  under `build/gcc/HERO-C` without compiling it.
- `tools/build.ps1 -Action gcc-build -Project HERO-C` generates, configures, and
  builds that target with CMake, Ninja, and `arm-none-eabi-gcc`.
- `tools/build.ps1 -Action gcc-build -Project all` builds every known target
  through the generated GCC/CMake route.
- `tools/build.ps1 -Action probe` shows which local build tools are available.

The Keil `.uvprojx` files remain the current firmware build source of truth.
The GCC/CMake route reads those files and generates a separate build view
instead of duplicating project file lists by hand.

The generator adapts the Keil-only pieces for GCC:

- ARMASM startup files are translated to GNU `.S` startup files.
- Keil `.sct` scatter files are translated to GNU linker scripts.
- FreeRTOS `portable/RVDS` paths are replaced with GCC portable layers.
- ARMCC-only libraries are replaced by GCC-side sources or compatibility code.
- ARMCC keywords and small runtime gaps are covered by files in
  `tools/build/gcc_support/`.

Generated files live under `build/gcc/` and are ignored by Git. If a source file,
include path, macro, startup file, or scatter file changes in Keil, regenerate
the GCC view instead of editing generated CMake files directly.

Requirements:

- Python 3.
- `arm-none-eabi-gcc`.
- CMake.
- Ninja.
- Keil MDK-ARM v5 and device packs if you also want to build or flash from Keil.

Typical commands:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\build.ps1 -Action check
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\build.ps1 -Action manifest -Project HERO-C
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\build.ps1 -Action manifest -Project all -FailOnGccBlockers
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\build.ps1 -Action gcc -Project HERO-C
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\build.ps1 -Action gcc-build -Project HERO-C
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\build.ps1 -Action gcc-build -Project all
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\build.ps1 -Action probe
```

The repository check does not run a real Keil Rebuild and does not run the full
GCC compiler for every target. Use `-Action gcc-build` when you want actual
compiler coverage.
