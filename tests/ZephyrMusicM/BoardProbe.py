"""Zephyr 音乐验证：备份、只读快照及显式启动和切歌操作。"""
import argparse
import hashlib
import json
import struct
from datetime import datetime
from pathlib import Path

from elftools.elf.elffile import ELFFile
from pyocd.core.helpers import ConnectHelper
from pyocd.core.target import Target


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("action", choices=["backup", "snapshot", "reset", "verify", "next", "previous"])
    parser.add_argument("--elf", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--image", type=Path)
    parser.add_argument("--stable-link", action="store_true")
    parser.add_argument("--symbols", nargs="*", default=[])
    parser.add_argument("--track-capacity", type=int, default=64)
    parser.add_argument("--track-path-bytes", type=int, default=384)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    if args.action == "backup" and (args.output / "flash-before.bin").exists():
        raise RuntimeError("backup already exists; choose a new output directory")
    session = ConnectHelper.session_with_chosen_probe(
        unique_id="486655686570", target_override="stm32h723xx", frequency=1000000,
        options={"connect_mode": "attach", "resume_on_disconnect": False,
                 **({"cmsis_dap.limit_packets": True, "cmsis_dap.deferred_transfers": False,
                     "user_script": str(Path(__file__).with_name("ReceiveDebug.py"))}
                    if args.stable_link else {})})
    if session is None:
        raise RuntimeError("DAP not found")
    with session:
        target = session.board.target
        result = {"time": datetime.now().isoformat(), "state": str(target.get_state())}
        if args.action == "backup":
            running = target.get_state() in (Target.State.RUNNING, Target.State.SLEEPING)
            target.halt()
            try:
                copies = []
                for attempt in range(2):
                    data = bytearray()
                    for offset in range(0, 0x100000, 0x10000):
                        data.extend(target.read_memory_block8(0x08000000 + offset, 0x10000))
                        print(f"backup {attempt + 1}/2 {len(data)}/1048576", flush=True)
                    copies.append(bytes(data))
                if copies[0] != copies[1]:
                    raise RuntimeError("independent flash reads differ")
                (args.output / "flash-before.bin").write_bytes(copies[0])
                result["sha256"] = hashlib.sha256(copies[0]).hexdigest()
            finally:
                if running:
                    target.resume()
        elif args.action == "verify":
            expected = args.image.read_bytes()
            target.halt()
            data = bytes(target.read_memory_block8(0x08000000, len(expected)))
            result["bytes"] = len(data)
            result["matches"] = data == expected
            result["sha256"] = hashlib.sha256(data).hexdigest()
            if not result["matches"]:
                raise RuntimeError("flash verify failed; target remains halted")
        elif args.action == "reset":
            target.reset(reset_type=Target.ResetType.SYSRESETREQ)
            target.resume()
            result["state"] = str(target.get_state())
        elif args.action in ("next", "previous"):
            with args.elf.open("rb") as stream:
                table = ELFFile(stream).get_section_by_name(".symtab")
                entry = table.get_symbol_by_name("SubBoardMusicChange")[0]
                target.write8(entry["st_value"], 1 if args.action == "next" else 255)
                result["request"] = args.action
                result["source"] = "debugger software request; no button GPIO was changed"
        else:
            with args.elf.open("rb") as stream:
                elf = ELFFile(stream)
                table = elf.get_section_by_name(".symtab")
                for name in args.symbols:
                    entries = table.get_symbol_by_name(name)
                    if not entries:
                        result[name] = "symbol absent"
                        continue
                    entry = entries[0]
                    address, size = entry["st_value"], entry["st_size"]
                    if not 0 < size <= 65536:
                        result[name] = {"address": hex(address), "size": size}
                        continue
                    data = bytes(target.read_memory_block8(address, size))
                    (args.output / f"{name}.bin").write_bytes(data)
                    if name == "ArbLogRam":
                        magic, capacity, position, total, panic = struct.unpack_from("<5I", data)
                        ring = data[20:20 + capacity]
                        log = ring[:position] if total < capacity else ring[position:] + ring[:position]
                        result[name] = {"address": hex(address), "panic": panic,
                                        "text": log.decode("utf-8", errors="replace")}
                        continue
                    if name == "SubBoardMusicDiagData":
                        keys = ["lastError", "scans", "entries", "unsupported", "previousPresses",
                                "nextPresses", "queued", "free", "sampleHz"]
                        result[name] = dict(zip(keys, struct.unpack_from("<i8I", data)))
                        result[name].update(zip(["index", "tracks"], struct.unpack_from("<2H", data, 36)))
                        result[name].update(zip(["running", "mounted", "playing", "pd14Raw", "pd15Raw",
                                                "pd14Stable", "pd15Stable", "channels", "bits"], data[40:49]))
                        for field, offset, length in [("directory", 49, 16), ("firstFile", 65, 96), ("current", 161, 96)]:
                            result[name][field] = data[offset:offset + length].split(b"\0")[0].decode("utf-8", errors="replace")
                        continue
                    if name == "SubBoardMusicTracks":
                        result[name] = [data[i:i + args.track_path_bytes].split(b"\0")[0].decode("utf-8", errors="replace")
                                        for i in range(0, size, size // args.track_capacity) if data[i] != 0]
                        continue
                    result[name] = {"address": hex(address), "size": size,
                                    "words": list(struct.unpack("<" + "I" * (size // 4), data[:size // 4 * 4])),
                                    "strings": [x.decode("utf-8", errors="replace") for x in data.split(b"\0") if len(x) >= 5]}
            # 外设寄存器在芯片睡眠时可能不可读；RAM 状态无需暂停音乐。
        result["finalState"] = str(target.get_state())
        (args.output / f"{args.action}.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
        print(json.dumps(result, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
