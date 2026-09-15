"""连续记录 UART8 桌面观测，不向板子发送数据。"""
import argparse
import json
import re
import time
from pathlib import Path

import serial


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seconds", type=float, default=900)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    start = time.monotonic()
    report = start
    pending = b""
    counts = {}
    latest = {}
    with serial.Serial(args.port, 115200, timeout=0.2) as port, args.output.open("wb") as log:
        print(f"CAPTURE READY {args.port} -> {args.output}", flush=True)
        while time.monotonic() - start < args.seconds:
            data = port.read(max(1, port.in_waiting))
            if data:
                log.write(data)
                log.flush()
                pending += data
                while b"\n" in pending:
                    line, pending = pending.split(b"\n", 1)
                    text = re.sub(r"\x1b\[[0-9;]*m", "", line.decode("utf-8", errors="replace"))
                    if "@" in text:
                        fields = text[text.index("@") + 1:].strip().split(",")
                        try:
                            values = [int(value) for value in fields[1:]]
                        except ValueError:
                            continue
                        counts[fields[0]] = counts.get(fields[0], 0) + 1
                        latest[fields[0]] = values
                    elif text.strip():
                        print(text, flush=True)
                if len(pending) > 65536:
                    pending = pending[-4096:]
            now = time.monotonic()
            if now - report >= 20:
                status = {"elapsed_s": round(now - start, 1), "counts": counts, "latest": latest}
                args.output.with_suffix(".status.json").write_text(
                    json.dumps(status, indent=2), encoding="utf-8")
                print(json.dumps(status), flush=True)
                report = now
        print(json.dumps({"complete": True, "counts": counts, "latest": latest}), flush=True)


if __name__ == "__main__":
    main()
