"""通过 DAP 读取静态准备固件的状态和 SD 文件，不发送车辆控制命令。"""
import argparse
import json
import struct
import time
from datetime import datetime
from pathlib import Path
from elftools.elf.elffile import ELFFile
from pyocd.core.helpers import ConnectHelper

p = argparse.ArgumentParser()
p.add_argument("action", choices=["status", "stop", "list", "read", "warm", "cool", "rtc"])
p.add_argument("--elf", type=Path, required=True)
p.add_argument("--path", default="0:/")
p.add_argument("--output", type=Path, required=True)
p.add_argument("--stable-link", action="store_true")
a = p.parse_args()
with a.elf.open("rb") as f:
    elf = ELFFile(f)
    table = elf.get_section_by_name(".symtab")
    symbols = {s.name: (s["st_value"], s["st_size"]) for s in table.iter_symbols()}
    code_address = symbols["MPreflightRun"][0] & ~1
    for segment in elf.iter_segments():
        base = segment["p_vaddr"]
        if base <= code_address < base + segment["p_filesz"]:
            code = segment.data()[code_address - base:code_address - base + 256]
            break
    else:
        raise RuntimeError("preflight code not found in ELF")
a.output.parent.mkdir(parents=True, exist_ok=True)
session = ConnectHelper.session_with_chosen_probe(
    unique_id="486655686570", target_override="stm32h723xx", frequency=1000000,
    options={"connect_mode": "attach", "resume_on_disconnect": False,
             "cmsis_dap.limit_packets": True,
             **({"cmsis_dap.deferred_transfers": False,
                 "user_script": str(Path(__file__).with_name("ReceiveDebug.py"))}
                if a.stable_link else {})})
with session:
    target = session.board.target
    if bytes(target.read_memory_block8(code_address, len(code))) != code:
        raise RuntimeError("ELF does not match the running preflight code; no request written")
    if target.read32(symbols["MPreflightDiag"][0]) != 0x4d505246:
        raise RuntimeError("preflight diagnostics not initialized; no request written")
    def addr(name):
        return symbols[name][0]
    def read(name):
        return bytes(target.read_memory_block8(addr(name), symbols[name][1]))
    def write_bytes(address, data):
        # H723 本次实测在 AXI SRAM 的半字调试写入处挂起。只做对齐的
        # 32 位读改写，保留字符串两侧字节；不让库降级为8/16位写入。
        start = address & ~3
        end = (address + len(data) + 3) & ~3
        words = target.read_memory_block32(start, (end - start) // 4)
        block = bytearray(struct.pack("<" + "I" * len(words), *words))
        block[address - start:address - start + len(data)] = data
        target.write_memory_block32(start, struct.unpack("<" + "I" * len(words), block))
        target.flush()
    def request(command, offset=0):
        if target.read32(addr("MPreflightCommand")):
            raise RuntimeError("previous request still pending")
        path = a.path.encode("utf-8") + b"\0"
        if len(path) > symbols["MPreflightPath"][1]:
            raise ValueError("path too long")
        # 静态准备固件没有电机任务。短暂停核后写请求，避免运行/睡眠切换
        # 期间访问系统SRAM；保持连接，写完立即继续运行以处理请求。
        target.halt()
        try:
            if command == 6:
                now = datetime.now().astimezone()
                target.write32(addr("MPreflightRtcDate"), int(now.strftime("%Y%m%d")))
                target.write32(addr("MPreflightRtcTime"), int(now.strftime("%H%M%S")))
                print("RTC host local time: " + now.isoformat(), flush=True)
            if command in (2, 3):
                write_bytes(addr("MPreflightPath"), path)
            target.write32(addr("MPreflightOffset"), offset)
            target.write32(addr("MPreflightCommand"), command)
            target.flush()
        finally:
            target.resume()
        deadline = time.monotonic() + 10
        while target.read32(addr("MPreflightCommand")):
            if time.monotonic() > deadline:
                raise TimeoutError("firmware request did not complete")
            time.sleep(0.01)
        result = struct.unpack("<i", read("MPreflightResult"))[0]
        if result:
            raise RuntimeError(f"firmware request failed: {result}")
        count = target.read32(addr("MPreflightBytes"))
        return bytes(target.read_memory_block8(addr("MPreflightData"), count))
    if a.action == "read":
        offset = 0
        with a.output.open("wb") as out:
            while True:
                block = request(3, offset)
                out.write(block)
                offset += len(block)
                if len(block) < symbols["MPreflightData"][1]:
                    break
        print(json.dumps({"path": a.path, "bytes": offset, "output": str(a.output)}))
    elif a.action == "list":
        data = request(2)
        a.output.write_bytes(data)
        print(data.decode("utf-8"))
    else:
        if a.action == "stop":
            request(1)
        elif a.action in ("warm", "cool"):
            request(4 if a.action == "warm" else 5)
        elif a.action == "rtc":
            request(6)
        data = read("MPreflightDiag")
        keys = ["magic", "tickMs", "mountResult", "logResult", "imuValid", "imuSequence",
                "imuAgeMs", "calibrated", "calibrating", "heater"]
        result = dict(zip(keys, struct.unpack_from("<IIii6I", data)))
        values = struct.unpack_from("<10f", data, 40)
        result.update(temperature=values[0], gyro=values[1:4], accel=values[4:7], angle=values[7:10])
        result["log"] = dict(zip(["active", "reserved8", "reserved16", "dropped", "ringUsed",
            "ringFree", "bytesFlushed", "lastSyncMs", "lastError"], struct.unpack_from("<BBH5Ii", data, 80)))
        result["path"] = read("MPreflightLogPath").split(b"\0")[0].decode("utf-8")
        result["bias"] = struct.unpack("<3f", read("InsGyroOffset"))
        result["spiErrors"] = target.read32(addr("Bmi088PortErrors"))
        if "MPreflightHeatEnable" in symbols:
            result["heatEnabled"] = target.read32(addr("MPreflightHeatEnable"))
        for name in ["ImuCalStoreDiag", "MBoardIoDiag"]:
            if name in symbols:
                raw = read(name)
                result[name] = list(struct.unpack("<" + "I" * (len(raw) // 4), raw))
        a.output.write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
        print(json.dumps(result, ensure_ascii=False))
