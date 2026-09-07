"""独占DAP只读记录接收固件；输出原始CAN帧、遥控器和IMU快照。"""
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
p.add_argument("--seconds", type=int, default=300)
p.add_argument("--stable-link", action="store_true")
a = p.parse_args()
a.output.mkdir(parents=True, exist_ok=True)
with a.elf.open("rb") as stream:
    elf = ELFFile(stream)
    symbols = {s.name: (s["st_value"], s["st_size"])
               for s in elf.get_section_by_name(".symtab").iter_symbols()}
    address = symbols["MReceiveRun"][0] & ~1
    code = None
    for segment in elf.iter_segments():
        offset = address - segment["p_vaddr"]
        if 0 <= offset < segment["p_filesz"]:
            code = segment.data()[offset:offset + 256]
            break
if code is None:
    raise RuntimeError("Receive code absent")

session = ConnectHelper.session_with_chosen_probe(
    unique_id="486655686570", target_override="stm32h723xx", frequency=1000000,
    options={"connect_mode": "attach", "resume_on_disconnect": False,
             "cmsis_dap.limit_packets": True,
             **({"cmsis_dap.deferred_transfers": False,
                 "user_script": str(Path(__file__).with_name("ReceiveDebug.py"))}
                if a.stable_link else {})})
start = time.monotonic()
next_report = 0
count = 0
with session, (a.output / "samples.jsonl").open("x", encoding="utf-8") as out:
    target = session.board.target
    if bytes(target.read_memory_block8(address, 256)) != code:
        raise RuntimeError("ELF mismatch")
    if target.read32(symbols["MPreflightHeatEnable"][0]):
        raise RuntimeError("Heater is enabled")
    base, size = symbols["MReceiveDiag"]
    if size != 1972:
        raise RuntimeError(f"Unexpected receive layout: {size}")
    print("READY: receive-only capture, no debug writes", flush=True)
    while time.monotonic() - start < a.seconds and not (a.output / "stop").exists():
        seq = target.read32(base + 4)
        if seq & 1:
            time.sleep(0.005)
            continue
        # 1MHz DAP整表读取会跨过100ms发布周期；先取前16个ID并明确报告截断。
        data = bytes(target.read_memory_block8(base, 180 + 16 * 28))
        if target.read32(base + 4) != seq:
            continue
        fields = ["magic", "sequence", "tick", "ready", "routeConflicts", "rcFrames",
                  "rcRejected", "rcOnline", "rcAge", "rcUartErrors", "rcBadSize",
                  "rcDropped", "tableFull", "txBlocked"]
        record = dict(zip(fields, struct.unpack_from("<14I", data)))
        if record["magic"] != 0x4d525843 or not record["ready"]:
            raise RuntimeError("Receive task not ready")
        record["channels"] = struct.unpack_from("<5i", data, 56)
        record["switches"] = struct.unpack_from("<2I", data, 76)
        bus_fields = ["received", "dropped", "txRequests", "txFailed", "lastError",
                      "rxErrors", "txErrors", "busOff"]
        record["buses"] = [dict(zip(bus_fields, struct.unpack_from("<8I", data, 84 + i * 32)))
                           for i in range(3)]
        if any(b["txRequests"] for b in record["buses"]):
            raise RuntimeError("Unexpected CAN send request reached driver")
        record["frames"] = []
        for i in range(16):
            offset = 180 + i * 28
            bus, ident, frames, last_tick, dlc = struct.unpack_from("<5I", data, offset)
            if not frames:
                continue
            payload = data[offset + 20:offset + 28]
            frame = dict(bus=bus, id=ident, count=frames, ageMs=record["tick"] - last_tick,
                         dlc=dlc, data=payload.hex())
            if 0x201 <= ident <= 0x20B and dlc == 8:
                # 未识别型号前不把后四字节当电流/温度；820R没有这些通用字段。
                frame.update(encoder=struct.unpack_from(">H", payload)[0],
                             secondWord=struct.unpack_from(">h", payload, 2)[0])
            record["frames"].append(frame)
        record["frameTableMayBeTruncated"] = len(record["frames"]) == 16
        imu_base = symbols["MPreflightDiag"][0]
        imu_seq = target.read32(imu_base + 20)
        imu = bytes(target.read_memory_block8(imu_base, 80))
        if target.read32(imu_base + 20) != imu_seq:
            continue
        header = struct.unpack_from("<IIii6I", imu)
        values = struct.unpack_from("<10f", imu, 40)
        if header[9] or target.read32(symbols["MPreflightHeatEnable"][0]):
            raise RuntimeError("Unexpected heater output")
        record["imu"] = dict(valid=header[4], sequence=header[5], ageMs=header[6],
                             temperature=values[0], gyro=values[1:4], accel=values[4:7],
                             pyrDeg=[-math.degrees(values[8]), math.degrees(values[7]),
                                     -math.degrees(values[9])])
        record["elapsed"] = time.monotonic() - start
        marker = a.output / "stage.txt"
        record["stage"] = marker.read_text(encoding="utf-8-sig").strip() if marker.exists() else "baseline"
        out.write(json.dumps(record) + "\n")
        out.flush()
        count += 1
        if record["elapsed"] >= next_report:
            print(json.dumps({"seconds": round(record["elapsed"], 1),
                              "canRx": [b["received"] for b in record["buses"]],
                              "rcFrames": record["rcFrames"], "rcOnline": record["rcOnline"],
                              "channels": record["channels"], "switches": record["switches"],
                              "ids": [[f["bus"], hex(f["id"]), f.get("encoder")] for f in record["frames"]],
                              "pyr": [round(x, 1) for x in record["imu"]["pyrDeg"]]}), flush=True)
            next_report = record["elapsed"] + 2
        time.sleep(0.1)
print(json.dumps({"samples": count, "finished": True}), flush=True)
