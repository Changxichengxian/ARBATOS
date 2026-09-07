"""只读连续记录板上姿态；用本地 stage.txt 标记用户动作，不向板子发运动命令。"""
import argparse
import json
import math
import struct
import time
from pathlib import Path
from elftools.elf.elffile import ELFFile
from pyocd.core.helpers import ConnectHelper

p = argparse.ArgumentParser()
p.add_argument("--elf", type=Path, required=True)
p.add_argument("--output", type=Path, required=True)
p.add_argument("--seconds", type=int, default=600)
a = p.parse_args()
a.output.mkdir(parents=True, exist_ok=True)
with a.elf.open("rb") as stream:
    elf = ELFFile(stream)
    symbols = {s.name: s["st_value"] for s in elf.get_section_by_name(".symtab").iter_symbols()}
    code_address = symbols["MPreflightRun"] & ~1
    code = None
    for segment in elf.iter_segments():
        offset = code_address - segment["p_vaddr"]
        if 0 <= offset < segment["p_filesz"]:
            code = segment.data()[offset:offset + 256]
            break
if code is None:
    raise RuntimeError("Preflight code absent")
session = ConnectHelper.session_with_chosen_probe(
    unique_id="486655686570", target_override="stm32h723xx", frequency=1000000,
    options={"connect_mode": "attach", "resume_on_disconnect": False,
             "cmsis_dap.limit_packets": True})
start = time.monotonic()
next_report = 0
count = 0
with session, (a.output / "samples.jsonl").open("x", encoding="utf-8") as out:
    target = session.board.target
    if bytes(target.read_memory_block8(code_address, 256)) != code:
        raise RuntimeError("ELF mismatch")
    if target.read32(symbols["MPreflightHeatEnable"]):
        raise RuntimeError("Turn off heater before handling board")
    print("READY: IMU capture started; heater disabled", flush=True)
    while time.monotonic() - start < a.seconds and not (a.output / "stop").exists():
        base = symbols["MPreflightDiag"]
        sequence = target.read32(base + 20)
        data = bytes(target.read_memory_block8(base, 80))
        after = target.read32(base + 20)
        if sequence != after:
            continue
        header = struct.unpack_from("<IIii6I", data)
        values = struct.unpack_from("<10f", data, 40)
        if header[0] != 0x4d505246 or not header[4] or header[6] > 50 or header[9]:
            raise RuntimeError("Invalid, stale or heated IMU state")
        if not all(math.isfinite(x) for x in values):
            raise RuntimeError("Nonfinite IMU sample")
        marker = a.output / "stage.txt"
        stage = marker.read_text(encoding="utf-8-sig").strip() if marker.exists() else "baseline"
        elapsed = time.monotonic() - start
        record = {"elapsed": elapsed, "tickMs": header[1], "sequence": header[5],
                  "stage": stage, "temperature": values[0], "gyroRadS": values[1:4],
                  "accel": values[4:7], "angleDegYRP": [math.degrees(x) for x in values[7:10]]}
        yaw, roll, pitch = record["angleDegYRP"]
        # HERO兼容接口：抬头、左转、右侧倾分别为正；保留原INS数组便于核对。
        record["heroDegPYR"] = [-roll, yaw, -pitch]
        out.write(json.dumps(record) + "\n")
        out.flush()
        count += 1
        if elapsed >= next_report:
            print(json.dumps({"seconds": round(elapsed, 1), "stage": stage,
                              "YRP": [round(x, 2) for x in record["angleDegYRP"]],
                              "heroPYR": [round(x, 2) for x in record["heroDegPYR"]],
                              "accel": [round(x, 2) for x in record["accel"]]}), flush=True)
            next_report = elapsed + 5
        time.sleep(0.04)
print(json.dumps({"samples": count, "finished": True}), flush=True)
