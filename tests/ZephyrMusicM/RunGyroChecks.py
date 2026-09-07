"""电脑检查，不接触板子。"""
from pathlib import Path
import subprocess

root = Path(__file__).resolve().parents[2]
out = root / "local/cache/zephyr-preflight-hardware/20260906/host-tests"
out.mkdir(parents=True, exist_ok=True)
vcvars = Path(r"C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat")
script = out / "Run.cmd"
script.write_text(f'@echo off\ncall "{vcvars}" >nul\nif errorlevel 1 exit /b %errorlevel%\n'
    'cl /nologo /std:c11 /utf-8 /I shared\\components\\support '
    '/I shared\\application\\services\\calibration '
    f'/Fo:"{out / "GyroCalibrationTest.obj"}" /Fe:"{out / "GyroCalibrationTest.exe"}" '
    'tests\\ZephyrMusicM\\GyroCalibrationTest.c\nif errorlevel 1 exit /b %errorlevel%\n'
    f'"{out / "GyroCalibrationTest.exe"}"\n', encoding="ascii")
result = subprocess.run(["cmd.exe", "/d", "/c", str(script)], cwd=root, capture_output=True)
log = (result.stdout + result.stderr).decode("utf-8", "replace")
(out / "result.log").write_text(log, encoding="utf-8")
print(log)
raise SystemExit(result.returncode)
