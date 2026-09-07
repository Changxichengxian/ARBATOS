"""用本机 MSVC 运行已有生产代码回归，不连接或驱动电机。"""
import json
import argparse
import subprocess
from pathlib import Path

root = Path(__file__).resolve().parents[2]
out = root / "local/cache/zephyr-preflight-hardware/20260906/safety"
out.mkdir(parents=True, exist_ok=True)
support = "shared/components/support"
robot = "shared/application/robot"
motors = "shared/application/motors"
can = "shared/application/comm/can"
cases = [
    ("ManualInputSnapshot", ["tools/tests/manual-input-stubs", "shared/application/input"], [], []),
    ("MotorHealth", [support, robot, motors], [], []),
    ("GimbalFeedbackPolicy", ["shared/application/gimbal", robot, support], [], []),
    ("LowCmd", ["tools/tests/stubs", support, robot, motors, can], [], []),
    ("RobotLifecycle", ["tools/tests/robot-lifecycle-stubs", robot], [robot + "/RobotLifecycle.c"], []),
    ("ControlMgr", [robot], [robot + "/ControlMgr.c"],
     ["/DCONTROL_MANAGER_TEST=1", "/FItools/tests/ControlMgrTestCritical.h"]),
    ("FaultMgr", [robot], [robot + "/FaultMgr.c"], []),
    ("ChassisSnapshotPolicy", [support, "shared/components/controller", robot, motors,
                               "shared/application/gimbal", can, "shared/application/chassis"],
     [motors + "/MotorHealth.c"], []),
    ("ShootFaultPolicy", ["shared/application/shoot"], [], []),
    ("ControlActuatorPolicy", [robot, support], [], []),
    ("CanTxCompletionPolicy", ["tools/tests/stubs", support, robot, can, "shared/hal"], [], []),
    ("MotorInstPermit", ["tools/tests/stubs", "Robotconfig/SENTINEL-M", support, can,
                          motors, robot, "shared/application/services/diagnostics"], [], ["/Gy"]),
]
vcvars = r"C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat"
p = argparse.ArgumentParser()
p.add_argument("--case", action="append", dest="selected")
a = p.parse_args()
previous = out / "results.json"
results = json.loads(previous.read_text(encoding="utf-8")) if a.selected and previous.exists() else []
for name, includes, extra, options in cases:
    if a.selected and name not in a.selected:
        continue
    exe = out / (name + ".exe")
    # 每个测试独立目录，避免生产源文件的 .obj 相互覆盖。
    objects = out / name
    objects.mkdir(exist_ok=True)
    args = ["cl", "/nologo", "/std:c11", "/utf-8", "/W3"]
    args += ["/I" + x for x in includes]
    args += ["/FI" + str(root / x[3:]) if x.startswith("/FI") else x for x in options]
    args += [f"/Fo{objects}\\", f"/Fe{exe}", f"tools/tests/{name}Regression.c"] + extra
    if name == "MotorInstPermit":
        args += ["/link", "/OPT:REF"]
    script = out / (name + ".cmd")
    script.write_text('@echo off\ncall "' + vcvars + '" >nul\n'
                      + subprocess.list2cmdline(args)
                      + '\nif errorlevel 1 exit /b %errorlevel%\n"' + str(exe) + '"\n',
                      encoding="ascii")
    run = subprocess.run(["cmd.exe", "/d", "/c", str(script)], cwd=root, capture_output=True)
    log = (run.stdout + run.stderr).decode("utf-8", "replace")
    (out / (name + ".log")).write_text(log, encoding="utf-8")
    results = [r for r in results if r["name"] != name]
    results.append({"name": name, "exitCode": run.returncode})
    print(name + ": " + ("PASS" if run.returncode == 0 else "FAIL"), flush=True)
    if run.returncode:
        print(log, flush=True)
(out / "results.json").write_text(json.dumps(results, indent=2), encoding="utf-8")
raise SystemExit(any(r["exitCode"] for r in results))
