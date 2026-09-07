"""通过调试器读取独立 SD 测试结果；不暂停、不复位、不烧写目标。"""

import argparse
import csv
import json
from pathlib import Path
import struct
import time


FIELDS = "targetHz actualHz mode multiRead dataBytes writeUs syncUs readUs verifyUs wallMs writeBps readBps maxWriteUs maxReadUs mismatchOffset expected actual result fsResult completedBytes crc16 transferCalls transferErrors transferTimeouts lastHalStatus lastHalError".split()
PHASES = {0: "启动", 1: "就绪", 2: "写入", 3: "读回校验", 4: "完成", 5: "失败"}
MODES = {0: "旧64B", 1: "整512B", 2: "整512B+FIFO"}


def decode(data):
    head = struct.unpack_from("<4Ii5Ii7I", data)
    if head[0] != 0x5344424D or head[1] != 1 or head[2] != 1144 or len(data) != head[2]:
        raise ValueError("诊断格式不匹配；确认已烧入本次固件，勿使用旧地址")
    keys = "magic version size phase error currentCase completedCases fsType cardType sectorCount highSpeedResult cpuHz spiKernelHz chunkBytes startedMs finishedMs faultCfsr faultHfsr".split()
    result = dict(zip(keys, head))
    result["activeFile"] = data[72:104].split(b"\0", 1)[0].decode("ascii", errors="replace")
    result["phaseText"] = PHASES.get(result["phase"], "未知")
    result["cases"] = []
    for index in range(10):
        values = struct.unpack_from("<17Ii8I", data, 104 + index * 104)
        row = dict(zip(FIELDS, values))
        row["index"] = index
        row["modeText"] = MODES.get(row["mode"], "未知")
        result["cases"].append(row)
    return result


def print_rows(result):
    print(f"状态: {result['phaseText']} error={result['error']} CMD6={result['highSpeedResult']}")
    print("档位  实际MHz  模式             多块读  大小MiB  写MB/s  读MB/s  校验ms  结果")
    for row in result["cases"]:
        if row["targetHz"] == 0:
            continue
        print(f"{row['index']:>4}  {row['actualHz']/1e6:>7.2f}  {row['modeText']:<14}"
              f" {row['multiRead']:>4} {row['dataBytes']/1048576:>8.1f}"
              f" {row['writeBps']/1e6:>7.3f} {row['readBps']/1e6:>7.3f}"
              f" {row['verifyUs']/1000:>7.1f} {row['result']:>5}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path)
    parser.add_argument("--probe", help="调试器唯一ID，须与待测M板匹配")
    parser.add_argument("--snapshot", type=Path, help="仅解析已有RAM快照")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--watch", type=float, default=0, help="最多观察秒数；0只读一次")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    if args.snapshot:
        raw = args.snapshot.read_bytes()
    else:
        if not args.manifest or not args.probe:
            parser.error("读实物需同时指定 --manifest 与 --probe")
        from pyocd.core.helpers import ConnectHelper
        info = json.loads(args.manifest.read_text(encoding="utf-8"))
        with ConnectHelper.session_with_chosen_probe(
                unique_id=args.probe, target_override="stm32h723xx", frequency=1000000,
                options={"connect_mode": "attach", "resume_on_disconnect": False}) as session:
            target = session.board.target
            deadline = time.monotonic() + args.watch
            previous = None
            while True:
                raw = bytes(target.read_memory_block8(info["address"], info["size"]))
                result = decode(raw)
                progress = (result["phase"], result["currentCase"], result["completedCases"])
                if progress != previous:
                    print(f"{result['phaseText']}: 档位{result['currentCase']}, 已完成{result['completedCases']}", flush=True)
                    previous = progress
                if result["phase"] in (4, 5) or time.monotonic() >= deadline:
                    break
                time.sleep(1)
    result = decode(raw)
    (args.output / "sd-bench-ram.bin").write_bytes(raw)
    (args.output / "sd-bench-result.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    with (args.output / "sd-bench-result.csv").open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(result["cases"][0]))
        writer.writeheader()
        writer.writerows(result["cases"])
    print_rows(result)
    if result["phase"] == 5:
        raise SystemExit(1)
    if result["phase"] != 4:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
