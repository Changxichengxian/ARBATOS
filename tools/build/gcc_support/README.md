# GCC support overlay

This directory contains source files used only by the generated GCC/CMake build.
Keil projects remain unchanged and continue to use the files recorded in each
`.uvprojx`.

## FreeRTOS

`freertos/portable/GCC/*` is copied from the official FreeRTOS-Kernel V11.3.0
repository and keeps its original MIT license headers.

Source:
https://github.com/FreeRTOS/FreeRTOS-Kernel/tree/V11.3.0/portable/GCC

## Algorithm replacements

`algorithm/*` provides GCC-side replacements for ARMCC-only binary libraries in
`shared/components/algorithm`. These files are intentionally referenced only by
the generated GCC build files.
