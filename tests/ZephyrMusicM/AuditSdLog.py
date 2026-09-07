"""检查实板导出的 SDLG 文件，统计 CRC、记录时间及 IMU 数值。"""
import argparse
import hashlib
import json
import math
import statistics
import struct
import sys
import zlib
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools/sdlog"))
from SdLogDecompress import lz4_decompress_block
from SdLogViewer import RecordStreamParser

p = argparse.ArgumentParser()
p.add_argument("input", type=Path)
p.add_argument("--output", type=Path, required=True)
a = p.parse_args()
data = a.input.read_bytes()
magic, size, flags, boot, reserved = struct.unpack_from("<IHHII", data)
if magic != 0x474c4453 or size < 16 or size > len(data) or flags != 0:
    raise ValueError("invalid SDLG header")
parser = RecordStreamParser(boot)
result = {"file": str(a.input), "bytes": len(data), "sha256": hashlib.sha256(data).hexdigest(),
          "blocks": 0, "crcBlocks": 0, "records": 0, "error": None}
tags = Counter()
ticks, temps, gyros, accels, norms = [], [], [], [], []
offset = size
try:
    while offset < len(data):
        if len(data) - offset < 20:
            raise ValueError(f"incomplete block header at {offset}")
        bmagic, flags, hsize, rawsize, storedsize, crc = struct.unpack_from("<IHHIII", data, offset)
        if bmagic != 0x4b424453 or hsize < 20 or flags & ~3:
            raise ValueError(f"invalid block header at {offset}")
        end = offset + hsize + storedsize
        if end > len(data):
            raise ValueError(f"incomplete block data at {offset}")
        block = data[offset + hsize:end]
        raw = lz4_decompress_block(block, rawsize) if flags & 1 else block
        if len(raw) != rawsize:
            raise ValueError(f"raw length mismatch at {offset}")
        if flags & 2:
            if zlib.crc32(raw) & 0xffffffff != crc:
                raise ValueError(f"CRC32 mismatch at {offset}")
            result["crcBlocks"] += 1
        result["blocks"] += 1
        for tick, tag, payload in parser.feed(raw):
            tags[tag] += 1
            result["records"] += 1
            if tag == 1:
                if len(payload) != 44:
                    raise ValueError("invalid IMU payload length")
                values = struct.unpack("<11f", payload)
                if not all(math.isfinite(x) for x in values):
                    raise ValueError("non-finite IMU payload")
                ticks.append(tick)
                temps.append(values[10])
                gyros.append(values[4:7])
                accels.append(math.sqrt(sum(x*x for x in values[7:10])))
                norms.append(math.sqrt(sum(x*x for x in values[:4])))
        offset = end
    if parser._buf:
        raise ValueError("incomplete final record")
except ValueError as exc:
    result["error"] = str(exc)
result["tags"] = dict(tags)
result["validPrefixBytes"] = offset
if ticks:
    intervals = [b-a for a,b in zip(ticks,ticks[1:])]
    result["imu"] = {
        "samples": len(ticks), "firstMs": ticks[0], "lastMs": ticks[-1],
        "maxGapMs": max(intervals, default=0), "meanGapMs": statistics.mean(intervals) if intervals else 0,
        "temperatureMin": min(temps), "temperatureMax": max(temps),
        "gyroMeanRadS": [statistics.mean(x[i] for x in gyros) for i in range(3)],
        "accelNormMin": min(accels), "accelNormMax": max(accels),
        "quatNormMin": min(norms), "quatNormMax": max(norms),
    }
a.output.write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
print(json.dumps(result, ensure_ascii=False))
raise SystemExit(1 if result["error"] else 0)
