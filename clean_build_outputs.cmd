@echo off
setlocal
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0clean_build_outputs.ps1" %*
