@echo off
setlocal EnableExtensions DisableDelayedExpansion

set "SCRIPT_DIR=%~dp0"
set "FFMPEG=%SCRIPT_DIR%ffmpeg.exe"
set "FOUND=0"
set "FAILED=0"

if not exist "%FFMPEG%" (
    echo Missing ffmpeg.exe in "%SCRIPT_DIR%".
    pause
    exit /b 1
)

if not "%~1"=="" goto arg_loop

for %%F in ("%SCRIPT_DIR%*.mp3") do (
    if exist "%%~fF" (
        set "FOUND=1"
        call :convert "%%~fF"
        if errorlevel 1 set "FAILED=1"
    )
)
goto done_args

:arg_loop
if "%~1"=="" goto done_args
    if exist "%~1" (
        set "FOUND=1"
        call :convert "%~1"
        if errorlevel 1 set "FAILED=1"
    ) else (
        echo Not found: "%~1"
        set "FAILED=1"
    )
    shift
    goto arg_loop

:done_args
if "%FOUND%"=="0" (
    echo Put MP3 files in this folder, then double-click this script.
    echo You can also drag MP3 files onto this script.
)

if "%FAILED%"=="1" (
    echo.
    echo Some files failed.
    pause
    exit /b 1
)

echo.
echo Done.
pause
exit /b 0

:convert
set "INPUT=%~1"
set "OUTPUT=%~dpn1.U8"
echo.
echo Converting "%~nx1" to "%~n1.U8"...
"%FFMPEG%" -hide_banner -y -i "%INPUT%" -vn -ac 1 -ar 12000 -af "acompressor=threshold=-18dB:ratio=2:attack=5:release=120:makeup=6,alimiter=limit=0.95" -c:a pcm_u8 -f u8 "%OUTPUT%"
if errorlevel 1 (
    echo Failed: "%INPUT%"
    exit /b 1
)
echo Wrote "%OUTPUT%"
exit /b 0
