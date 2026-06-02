# ARBATOS build tools

This directory contains the scriptable build-facing entry points.

Current status:

- `tools/build.ps1 -Action check` runs the repository checks used by CI.
- `tools/build.ps1 -Action manifest -Project HERO-C` prints the source files,
  include directories, defines, linker script, compiler version, and GCC
  migration blockers extracted from the Keil project.
- `tools/build.ps1 -Action probe` shows which local build tools are available.

The Keil `.uvprojx` files remain the current firmware build source of truth.
The manifest tool is the bridge toward a generated GCC/CMake build without
duplicating project file lists by hand.

Typical commands:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\build.ps1 -Action check
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\build.ps1 -Action manifest -Project HERO-C
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\build.ps1 -Action probe
```

Known GCC migration blockers are reported by the manifest tool.  Today they
usually include ARM Compiler 5 flags, ARMCC `.lib` files, ARMASM startup files,
Keil `.sct` linker scripts, and FreeRTOS `portable/RVDS` paths.
