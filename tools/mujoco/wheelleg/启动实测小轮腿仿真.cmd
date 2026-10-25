@echo off
setlocal
pushd "%~dp0..\..\.."
set "WHEELLEG_PYTHON=local\cache\wbr-reference-20261003\.venv\Scripts\python.exe"
if not exist "%WHEELLEG_PYTHON%" set "WHEELLEG_PYTHON=python"
"%WHEELLEG_PYTHON%" tools\mujoco\wheelleg\ViewMeasuredWheelLeg.py --open %*
if errorlevel 1 pause
popd
endlocal
