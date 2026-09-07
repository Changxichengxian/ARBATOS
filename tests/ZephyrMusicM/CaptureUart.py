"""读取 UART8 转接串口，保留完整 UTF-8 日志。"""
import argparse
import codecs
import time
from pathlib import Path
import serial

p = argparse.ArgumentParser()
p.add_argument("--output", type=Path, required=True)
p.add_argument("--seconds", type=float, default=45)
p.add_argument("--port", default="COM6")
a = p.parse_args()
a.output.parent.mkdir(parents=True, exist_ok=True)
decoder = codecs.getincrementaldecoder("utf-8")("replace")
with serial.Serial(a.port, 115200, timeout=0.2) as port, a.output.open("w", encoding="utf-8") as out:
    port.reset_input_buffer()
    end = time.monotonic() + a.seconds
    while time.monotonic() < end:
        s = decoder.decode(port.read(max(1, port.in_waiting)))
        if s:
            out.write(s)
            out.flush()
            print(s, end="", flush=True)
    s = decoder.decode(b"", final=True)
    out.write(s)
