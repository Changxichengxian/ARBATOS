"""只接收固件的短暂停核快照；使用本机已验证的OpenOCD与500kHz连接。"""
import argparse
import json
import struct
import subprocess
from pathlib import Path
from elftools.elf.elffile import ELFFile

p = argparse.ArgumentParser()
p.add_argument("--elf", type=Path, required=True)
p.add_argument("--output", type=Path, required=True)
p.add_argument("--run-seconds", type=int, default=0)
p.add_argument("--stop-log", action="store_true")
a = p.parse_args()
a.output.mkdir(parents=True, exist_ok=True)
root = Path(__file__).resolve().parents[2]
with a.elf.open("rb") as stream:
    elf = ELFFile(stream)
    symbols = {s.name: (s["st_value"], s["st_size"])
               for s in elf.get_section_by_name(".symtab").iter_symbols()}
    code_address = symbols["MPreflightRun"][0] & ~1
    for segment in elf.iter_segments():
        offset = code_address - segment["p_vaddr"]
        if 0 <= offset < segment["p_filesz"]:
            expected_code = segment.data()[offset:offset + 256]
            break
names = ["MPreflightDiag", "MReceiveDiag", "MPreflightLogPath", "MPreflightHeatEnable",
         "InsGyroOffset", "ImuCalStoreDiag", "MBoardIoDiag", "Bmi088PortErrors"]
script = ["source [find interface/cmsis-dap.cfg]", "adapter serial 486655686570",
          "transport select swd", "gdb_port disabled", "tcl_port disabled", "telnet_port disabled",
          "source [find target/stm32h7x.cfg]", "adapter speed 500", "reset_config none", "init", "halt"]
if a.run_seconds:
    script.extend(["mww 0x5c001004 0x700187", "resume", f"sleep {a.run_seconds * 1000}", "halt"])
if a.stop_log:
    # 写邮箱前校验板上代码、诊断标记及邮箱空闲状态；只用对齐32位写入。
    expected_path = (a.output / "expected-code.bin").resolve()
    expected_path.write_bytes(expected_code)
    command = symbols["MPreflightCommand"][0]
    script.extend([
        f"verify_image {{{expected_path.as_posix()}}} {code_address:#x} bin",
        f"if {{[read_memory {symbols['MPreflightDiag'][0]:#x} 32 1] != 0x4d505246}} {{error {{preflight not ready}}}}",
        f"if {{[read_memory {command:#x} 32 1] != 0}} {{error {{request pending}}}}",
        f"mww {command:#x} 1", "resume", "sleep 2000", "halt",
        f"if {{[read_memory {command:#x} 32 1] != 0}} {{error {{stop request pending}}}}",
        f"if {{[read_memory {symbols['MPreflightResult'][0]:#x} 32 1] != 0}} {{error {{stop request failed}}}}",
    ])
for name in names:
    address, size = symbols[name]
    script.append(f"dump_image {{{(a.output / (name + '.bin')).resolve().as_posix()}}} {address:#x} {size}")
script.extend([f"dump_image {{{(a.output / 'code.bin').resolve().as_posix()}}} {code_address:#x} 256",
               "resume", "shutdown"])
config = a.output / "snapshot.cfg"
config.write_text("\n".join(script) + "\n", encoding="utf-8")
base = root / "local/cache/zephyr-sdk/hosttools/openocd"
with (a.output / "openocd.log").open("w", encoding="utf-8") as log:
    subprocess.run([str(base / "bin/openocd.exe"), "-s", str(base / "share/openocd/scripts"),
                    "-f", str(config.resolve())], stdout=log, stderr=subprocess.STDOUT, timeout=45, check=True)
if (a.output / "code.bin").read_bytes() != expected_code:
    raise RuntimeError("Snapshot firmware does not match ELF")
def read(name):
    data = (a.output / (name + ".bin")).read_bytes()
    if len(data) != symbols[name][1]:
        raise RuntimeError("Incomplete snapshot: " + name)
    return data
imu = read("MPreflightDiag")
header = struct.unpack_from("<IIii6I", imu)
values = struct.unpack_from("<10f", imu, 40)
diag = read("MReceiveDiag")
fields = ["magic", "sequence", "tick", "ready", "routeConflicts", "rcFrames", "rcRejected",
          "rcOnline", "rcAge", "rcUartErrors", "rcBadSize", "rcDropped", "tableFull", "txBlocked"]
result = dict(zip(fields, struct.unpack_from("<14I", diag)))
result["channels"] = struct.unpack_from("<5i", diag, 56)
result["switches"] = struct.unpack_from("<2I", diag, 76)
keys = ["received", "dropped", "txRequests", "txFailed", "lastError", "rxErrors", "txErrors", "busOff"]
result["buses"] = [dict(zip(keys, struct.unpack_from("<8I", diag, 84 + i * 32))) for i in range(3)]
result["frames"] = []
for i in range(64):
    bus, ident, count, tick, dlc = struct.unpack_from("<5I", diag, 180 + i * 28)
    if count:
        result["frames"].append(dict(bus=bus, id=hex(ident), count=count, tick=tick, dlc=dlc,
                                      data=diag[200 + i*28:208 + i*28].hex()))
result["imu"] = dict(tick=header[1], valid=header[4], ageMs=header[6], heater=header[9],
                     temperature=values[0], gyro=values[1:4], accel=values[4:7], angleYRP=values[7:10])
result["log"] = dict(zip(["active", "reserved8", "reserved16", "dropped", "ringUsed", "ringFree",
                           "bytesFlushed", "lastSyncMs", "lastError"], struct.unpack_from("<BBH5Ii", imu, 80)))
result["path"] = read("MPreflightLogPath").split(b"\0")[0].decode("utf-8")
result["bias"] = struct.unpack("<3f", read("InsGyroOffset"))
for name in ("ImuCalStoreDiag", "MBoardIoDiag", "Bmi088PortErrors", "MPreflightHeatEnable"):
    data = read(name)
    result[name] = list(struct.unpack("<" + "I" * (len(data) // 4), data))
(a.output / "snapshot.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
print(json.dumps(result, ensure_ascii=False))
