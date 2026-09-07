"""按原始记录统计接收测试，保留CSV供俯仰与编码器交叉核对。"""
import argparse
import csv
import json
import math
import statistics
import struct
import sys
import zlib
from collections import Counter, defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools/sdlog"))
from SdLogDecompress import _read_current_header, lz4_decompress_block
from SdLogViewer import RecordStreamParser, extract_series

p = argparse.ArgumentParser()
p.add_argument("input", type=Path)
p.add_argument("--output", type=Path, required=True)
a = p.parse_args()
a.output.mkdir(parents=True, exist_ok=True)
imu, rc, diagnostics, metadata = [], [], [], []
can = defaultdict(list)
tags = Counter()
with a.input.open("rb") as stream:
    _, _, boot = _read_current_header(stream)
    parser = RecordStreamParser(boot)
    while header := stream.read(20):
        if len(header) != 20:
            raise ValueError("Incomplete block header")
        magic, flags, size, raw_size, stored_size, crc = struct.unpack("<IHHIII", header)
        if magic != 0x4b424453 or size < 20 or flags & ~3:
            raise ValueError("Invalid block header")
        if len(stream.read(size - 20)) != size - 20:
            raise ValueError("Incomplete extended header")
        block = stream.read(stored_size)
        if len(block) != stored_size:
            raise ValueError("Incomplete block")
        raw = lz4_decompress_block(block, raw_size) if flags & 1 else block
        if len(raw) != raw_size or (flags & 2 and zlib.crc32(raw) & 0xffffffff != crc):
            raise ValueError("Invalid block size or CRC")
        for tick, tag, data in parser.feed(raw):
            tags[tag] += 1
            if tag == 1:
                w, x, y, z, gx, gy, gz, ax, ay, az, temp = struct.unpack("<11f", data)
                # 对应当前AHRS的get_roll/get_yaw/get_pitch，沿用HERO安装显示符号。
                pitch = -math.degrees(math.atan2(2 * (w*x + y*z), 1 - 2 * (x*x + y*y)))
                yaw = math.degrees(math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z)))
                roll = -math.degrees(math.asin(max(-1, min(1, 2 * (w*y - z*x)))))
                imu.append(dict(tickMs=tick, pitch=pitch, yaw=yaw, roll=roll, temperature=temp))
            elif tag == 0x46:
                if len(data) != 48:
                    raise ValueError("Invalid remote record size")
                source, proto, mode, count = data[:4]
                channels = struct.unpack_from("<16h", data, 4)
                rc.append(dict(tickMs=tick, source=source, protocol=proto, rangeMode=mode,
                               **{f"ch{i}": v for i, v in enumerate(channels[:count])},
                               switch0=data[36], switch1=data[37]))
            elif tag == 0x30:
                bus, dlc, ident, payload = struct.unpack("<BBH8s", data)
                row = dict(tickMs=tick, bus=bus, id=ident, dlc=dlc, data=payload.hex())
                if dlc >= 2:
                    row["firstWord"] = struct.unpack_from(">H", payload)[0]
                can[(bus, ident)].append(row)
            elif tag == 0x55:
                if len(data) != 180:
                    raise ValueError("Unknown receive status layout")
                values = struct.unpack_from("<14I", data)
                diagnostics.append(dict(tickMs=tick, ready=values[3], conflicts=values[4],
                                        rcFrames=values[5], rcRejected=values[6], rcUartErrors=values[9],
                                        rcBadSize=values[10], rcDropped=values[11], txBlocked=values[13],
                                        buses=[list(struct.unpack_from("<8I", data, 84+i*32)) for i in range(3)]))
            elif tag in (0x51, 0x52, 0x42):
                metadata.append(dict(tickMs=tick, tag=tag, values=extract_series(tag, data)))
    if parser._buf:
        raise ValueError("Incomplete record tail")

def csv_write(name, rows):
    if rows:
        fields = list(dict.fromkeys(k for row in rows for k in row))
        with (a.output / name).open("w", newline="", encoding="utf-8-sig") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows(rows)

csv_write("imu.csv", imu)
csv_write("remote.csv", rc)
csv_write("can.csv", sorted((r for rows in can.values() for r in rows), key=lambda r: r["tickMs"]))
result = dict(tags=dict(tags), metadata=metadata, can=[], receiveStatus=diagnostics)
if imu:
    result["imu"] = dict(samples=len(imu), firstMs=imu[0]["tickMs"], lastMs=imu[-1]["tickMs"],
                         ranges={k: [min(r[k] for r in imu), max(r[k] for r in imu)]
                                 for k in ("pitch", "yaw", "roll", "temperature")})
if rc:
    intervals = [b["tickMs"] - c["tickMs"] for c, b in zip(rc, rc[1:])]
    result["remote"] = dict(samples=len(rc), firstMs=rc[0]["tickMs"], lastMs=rc[-1]["tickMs"],
                            meanGapMs=statistics.mean(intervals) if intervals else 0,
                            maxGapMs=max(intervals, default=0),
                            ranges={k: [min(r[k] for r in rc if k in r), max(r[k] for r in rc if k in r)]
                                    for k in rc[0] if k.startswith("ch")},
                            switches={k: sorted({r[k] for r in rc}) for k in ("switch0", "switch1")},
                            protocols=sorted({r["protocol"] for r in rc}))
for (bus, ident), rows in sorted(can.items()):
    words = [r["firstWord"] for r in rows if "firstWord" in r]
    result["can"].append(dict(bus=bus, id=hex(ident), records=len(rows),
                              firstMs=rows[0]["tickMs"], lastMs=rows[-1]["tickMs"],
                              firstWordRange=[min(words), max(words)] if words else None))
(a.output / "analysis.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
print(json.dumps({k: v for k, v in result.items() if k not in ("metadata", "receiveStatus")}, ensure_ascii=False))
