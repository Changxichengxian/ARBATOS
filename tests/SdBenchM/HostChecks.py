"""仅在电脑上运行协议与结果格式检查，不连接硬件。"""

import ctypes
import importlib.util
import json
from pathlib import Path
import subprocess


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
OUT = ROOT / "local/cache/sd-bench-m/host-tests"
OUT.mkdir(parents=True, exist_ok=True)


def check_decoder():
    spec = importlib.util.spec_from_file_location("reader", HERE / "ReadResult.py")
    reader = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(reader)

    class Row(ctypes.LittleEndianStructure):
        _fields_ = [(name, ctypes.c_int32 if name == "result" else ctypes.c_uint32) for name in reader.FIELDS]

    class State(ctypes.LittleEndianStructure):
        _fields_ = [(name, ctypes.c_int32 if name in ("error", "highSpeedResult") else ctypes.c_uint32)
                    for name in "magic version size phase error currentCase completedCases fsType cardType sectorCount highSpeedResult cpuHz spiKernelHz chunkBytes startedMs finishedMs faultCfsr faultHfsr".split()]
        _fields_ += [("activeFile", ctypes.c_char * 32), ("cases", Row * 10)]

    fixture = State()
    fixture.magic = 0x5344424D
    fixture.version = 1
    fixture.size = ctypes.sizeof(State)
    fixture.phase = 5
    fixture.error = -208
    fixture.highSpeedResult = -5
    fixture.activeFile = b"0:/SB000001.BIN"
    fixture.cases[9].result = -208
    fixture.cases[9].transferErrors = 2
    decoded = reader.decode(bytes(fixture))
    assert decoded["size"] == 1144 and decoded["error"] == -208
    assert decoded["highSpeedResult"] == -5 and decoded["cases"][9]["result"] == -208
    assert decoded["cases"][9]["transferErrors"] == 2
    assert decoded["activeFile"] == "0:/SB000001.BIN"
    try:
        reader.decode(b"\0" * 1144)
        raise AssertionError("必须拒绝错误的固件标识")
    except ValueError:
        pass
    manifest = json.loads((OUT.parent / "firmware.json").read_text(encoding="utf-8"))
    assert manifest["size"] == ctypes.sizeof(State), "AXF结构与解码器不一致"
    print("诊断格式、负错误码、最后一档、无效标识、AXF大小检查: PASS")


def main():
    compiler = Path(r"C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat")
    if not compiler.is_file():
        raise SystemExit("未找到VS2022 Build Tools；可用C11编译器编译ProtocolTest.c")
    command = ('cl.exe /nologo /std:c11 /utf-8 /W4 /WX '
               '/I tests\\SdBenchM\\ProtocolStub /I shared\\hal '
               '/Fo:local\\cache\\sd-bench-m\\host-tests\\ProtocolTest.obj '
               '/Fe:local\\cache\\sd-bench-m\\host-tests\\ProtocolTest.exe '
               'tests\\SdBenchM\\ProtocolTest.c')
    script = OUT / "RunProtocol.cmd"
    script.write_text(f'@echo off\ncall "{compiler}" >nul\nif errorlevel 1 exit /b %errorlevel%\n'
                      f'{command}\nif errorlevel 1 exit /b %errorlevel%\n'
                      'local\\cache\\sd-bench-m\\host-tests\\ProtocolTest.exe\n', encoding="ascii")
    run = subprocess.run(["cmd.exe", "/d", "/c", str(script)], cwd=ROOT,
                         capture_output=True, text=True, errors="replace",
                         creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
    log = run.stdout + run.stderr
    (OUT / "protocol.log").write_text(log, encoding="utf-8")
    print(log, end="")
    if run.returncode:
        raise SystemExit(run.returncode)
    check_decoder()


if __name__ == "__main__":
    main()
