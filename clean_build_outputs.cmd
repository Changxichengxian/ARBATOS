@echo off
setlocal EnableExtensions

set "ROOT=%~dp0"
set "DEEP=0"
set "QUIET=0"
set "WHATIF=0"
set "COUNT=0"

:parse_args
if "%~1"=="" goto parsed_args
if /i "%~1"=="/Deep" set "DEEP=1"
if /i "%~1"=="-Deep" set "DEEP=1"
if /i "%~1"=="--deep" set "DEEP=1"
if /i "%~1"=="/Quiet" set "QUIET=1"
if /i "%~1"=="-Quiet" set "QUIET=1"
if /i "%~1"=="--quiet" set "QUIET=1"
if /i "%~1"=="/WhatIf" set "WHATIF=1"
if /i "%~1"=="-WhatIf" set "WHATIF=1"
if /i "%~1"=="--what-if" set "WHATIF=1"
shift
goto parse_args

:parsed_args
call :clean_root "%ROOT%projects"
call :clean_root "%ROOT%boards"

if not "%QUIET%"=="1" (
    echo %COUNT% items matched.
    if not "%DEEP%"=="1" echo Use /Deep to also remove Keil local option/window files such as *.uvoptx and *uvgui*.
)
exit /b 0

:clean_root
set "SCAN_ROOT=%~1"
if not exist "%SCAN_ROOT%" exit /b 0

for %%P in (*.bak *.lst *.lnp *.obj *.tmp *.TMP *.__i *.crf *.o *.d *.axf *.dep *.htm *.hsc *.map *.hex *.bin *.tra *.build.log codex_build*.log *JLinkLog.txt) do (
    for /r "%SCAN_ROOT%" %%F in (%%P) do call :remove_file "%%~fF"
)

if "%DEEP%"=="1" (
    for %%P in (*.uvopt *.uvoptx *.uvoptx.* *uvgui* *.scvd *.iex *.ini) do (
        for /r "%SCAN_ROOT%" %%F in (%%P) do call :remove_file "%%~fF"
    )
)

for /d /r "%SCAN_ROOT%" %%D in (*) do (
    if /i "%%~nxD"=="Objects" call :remove_dir "%%~fD"
    if /i "%%~nxD"=="Listings" call :remove_dir "%%~fD"
    if /i "%%~nxD"=="RTE" call :remove_dir "%%~fD"
    if /i "%%~nxD"=="DebugConfig" call :remove_dir "%%~fD"
    if /i "%%~nxD"=="standard_tpye_c" call :remove_dir "%%~fD"
    if "%DEEP%"=="1" (
        if /i "%%~nxD"=="Debug" call :remove_dir "%%~fD"
        if /i "%%~nxD"=="Release" call :remove_dir "%%~fD"
        if /i "%%~nxD"=="build" call :remove_dir "%%~fD"
    )
)

for /d /r "%SCAN_ROOT%" %%D in (*) do (
    if /i "%%~nxD"=="MDK-ARM" (
        for /d %%C in ("%%~fD\*") do call :remove_dir "%%~fC"
    )
)
exit /b 0

:remove_file
if not exist "%~1" exit /b 0
set /a COUNT+=1
if "%WHATIF%"=="1" (
    echo [file] %~1
) else (
    del /f /q "%~1" >nul 2>nul
)
exit /b 0

:remove_dir
if not exist "%~1\" exit /b 0
set /a COUNT+=1
if "%WHATIF%"=="1" (
    echo [dir]  %~1
) else (
    rmdir /s /q "%~1" >nul 2>nul
)
exit /b 0
