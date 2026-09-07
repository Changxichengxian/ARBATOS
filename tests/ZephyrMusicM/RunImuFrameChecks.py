"""检查 HERO-M 新坐标与其他目标旧坐标，并验证零偏格式转换。"""
import subprocess
from pathlib import Path

root = Path(__file__).resolve().parents[2]
out = root / "local/cache/zephyr-preflight-hardware/20260906/imu-axes/host-tests"
out.mkdir(parents=True, exist_ok=True)
vcvars = r"C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat"
for name in ("hero-m", "legacy"):
    exe = out / (name + ".exe")
    args = ["cl", "/nologo", "/std:c11", "/utf-8", "/W3", "/WX",
            "/Ishared/zephyr/port/sensors", "/IRobotconfig/HERO-M",
            "/Fo" + str(out / (name + ".obj")), "/Fe" + str(exe)]
    if name == "hero-m":
        args += ["/DCONFIG_ARBATOS_TARGET_HERO_M=1"]
    args += ["tests/ZephyrMusicM/ImuFrameTest.c"]
    script = out / (name + ".cmd")
    script.write_text('@echo off\ncall "' + vcvars + '" >nul\n'
                      + subprocess.list2cmdline(args)
                      + '\nif errorlevel 1 exit /b %errorlevel%\n"' + str(exe) + '"\n',
                      encoding="ascii")
    run = subprocess.run(["cmd.exe", "/d", "/c", str(script)], cwd=root, capture_output=True)
    log = (run.stdout + run.stderr).decode("utf-8", "replace")
    (out / (name + ".log")).write_text(log, encoding="utf-8")
    print(name + ": " + log, flush=True)
    if run.returncode:
        raise SystemExit(run.returncode)
