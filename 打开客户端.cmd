@echo off
chcp 65001 >nul
cd /d "%~dp0"
pwsh -NoLogo -NoProfile -ExecutionPolicy Bypass -File "%~dp0tools\client\StartClient.ps1"
if errorlevel 1 pause
