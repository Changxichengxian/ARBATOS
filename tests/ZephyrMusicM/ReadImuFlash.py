"""独占 DAP 只读检查校准双副本；请先结束其他 DAP 采样脚本。"""
import argparse
import json
import math
import struct
import zlib
from pathlib import Path
from pyocd.core.helpers import ConnectHelper

p = argparse.ArgumentParser()
p.add_argument("--output", type=Path, required=True)
a = p.parse_args()
a.output.mkdir(parents=True, exist_ok=True)
session = ConnectHelper.session_with_chosen_probe(
    unique_id="486655686570", target_override="stm32h723xx", frequency=1000000,
    options={"connect_mode": "attach", "resume_on_disconnect": False,
             "cmsis_dap.limit_packets": True})
result = []
with session:
    target = session.board.target
    # Zephyr hwinfo_stm32 按 Word2/1/0 的大端顺序提供板号。
    uid = bytes(target.read_memory_block8(0x1ff1e800, 12))[::-1]
    for slot, address in enumerate([0x080c0000, 0x080e0000]):
        data = bytes(target.read_memory_block8(address, 64))
        (a.output / f"slot-{slot}.bin").write_bytes(data)
        magic, version, sequence, board, *values = struct.unpack_from("<III12s4fI", data)
        bias, temperature, crc = values[:3], values[3], values[4]
        valid = (magic == 0x4d494341 and version in (1, 2) and board == uid
                 and crc == zlib.crc32(data[:40])
                 and all(math.isfinite(x) and abs(x) <= 0.0873 for x in bias)
                 and math.isfinite(temperature) and 35 <= temperature <= 45)
        result.append({"slot": slot, "address": hex(address), "valid": valid, "frameVersion": version,
                       "erased": data == b"\xff" * 64, "sequence": sequence,
                       "bias": bias if valid else None,
                       "temperature": temperature if valid else None})
(a.output / "records.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
print(json.dumps(result), flush=True)
if not any(x["valid"] for x in result):
    raise SystemExit("No valid internal Flash calibration record")
