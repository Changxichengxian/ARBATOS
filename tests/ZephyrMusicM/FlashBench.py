"""为已确认的桌面 MC02 烧录并完整回读校验，保留校准扇区。"""
import argparse
import hashlib
import json
import math
import shutil
import subprocess
from pathlib import Path

from elftools.elf.elffile import ELFFile


FLASH_START = 0x08000000
CALIBRATION_START = 0x080C0000
MAILBOX_GUARD = 0x53444C31
PAUSE_REQUEST = 0x53445051
PAUSE_READY = 0x53445244
PAUSE_FAILED = 0x53444641
MAILBOX_SYMBOLS = (
    "g_sdLogFlashMailboxGuard",
    "g_sdLogFlashBuildFingerprint",
    "g_sdLogFlashPauseRequest",
    "g_sdLogFlashPauseReady",
)
BUILD_FINGERPRINT_SIZE = 32


def require_hero_m_config(config_path):
    config = config_path.read_text(encoding="utf-8")
    if 'CONFIG_ARBATOS_ROBOT_NAME="HERO-M"' not in config:
        raise RuntimeError(f"Not a HERO-M image: {config_path}")


def load_running_elf(elf_path):
    with elf_path.open("rb") as stream:
        elf = ELFFile(stream)
        symtab = elf.get_section_by_name(".symtab")
        if symtab is None:
            raise RuntimeError("Running ELF has no symbol table")
        symbols = {symbol.name: (symbol["st_value"], symbol["st_size"])
                   for symbol in symtab.iter_symbols()}
        missing = [name for name in MAILBOX_SYMBOLS if name not in symbols]
        if missing:
            raise RuntimeError("Running ELF has no SD pause mailbox: " + ", ".join(missing))

        load_segments = [segment for segment in elf.iter_segments()
                         if segment["p_type"] == "PT_LOAD" and int(segment["p_filesz"]) > 0]
        sections = []
        for section in elf.iter_sections():
            address = int(section["sh_addr"])
            size = int(section["sh_size"])
            if size == 0 or not int(section["sh_flags"]) & 0x2 or section["sh_type"] == "SHT_NOBITS":
                continue
            owner = next((segment for segment in load_segments
                          if int(segment["p_vaddr"]) <= address and
                          address + size <= int(segment["p_vaddr"]) + int(segment["p_filesz"])), None)
            if owner is None:
                continue
            load_address = int(owner["p_paddr"]) + address - int(owner["p_vaddr"])
            if FLASH_START <= load_address and load_address + size <= CALIBRATION_START:
                sections.append((load_address, section.name, section.data()))

    if not sections or min(address for address, _, _ in sections) != FLASH_START:
        raise RuntimeError("Running ELF has no complete application flash image")
    image_end = max(address + len(data) for address, _, data in sections)
    image = bytearray(b"\xff" * (image_end - FLASH_START))
    written = bytearray(len(image))
    for address, name, data in sorted(sections):
        start = address - FLASH_START
        for index, value in enumerate(data):
            target = start + index
            if written[target] and image[target] != value:
                raise RuntimeError(f"Running ELF has conflicting allocated sections at {address + index:#x}: {name}")
            image[target] = value
            written[target] = 1
    segments = [(FLASH_START, bytes(image))]

    guard_address, guard_size = symbols[MAILBOX_SYMBOLS[0]]
    fingerprint_address, fingerprint_size = symbols[MAILBOX_SYMBOLS[1]]
    request_address, request_size = symbols[MAILBOX_SYMBOLS[2]]
    ready_address, ready_size = symbols[MAILBOX_SYMBOLS[3]]
    if guard_size != 4 or not FLASH_START <= guard_address < CALIBRATION_START:
        raise RuntimeError("SD pause mailbox guard is not a 32-bit flash symbol")
    if (fingerprint_size != BUILD_FINGERPRINT_SIZE or
            fingerprint_address % 4 != 0 or
            not FLASH_START <= fingerprint_address < CALIBRATION_START or
            fingerprint_address + fingerprint_size > CALIBRATION_START):
        raise RuntimeError("SD pause build fingerprint is not a 256-bit flash symbol")
    for name, address, size in (
            (MAILBOX_SYMBOLS[2], request_address, request_size),
            (MAILBOX_SYMBOLS[3], ready_address, ready_size)):
        if size != 4 or address % 4 != 0 or not 0x20000000 <= address < 0x40000000:
            raise RuntimeError(f"{name} is not an aligned 32-bit RAM symbol")
    if request_address == ready_address:
        raise RuntimeError("SD pause request and result symbols overlap")
    guard_offset = guard_address - FLASH_START
    if int.from_bytes(image[guard_offset:guard_offset + 4], "little") != MAILBOX_GUARD:
        raise RuntimeError("SD pause mailbox guard value is invalid in running ELF")
    fingerprint_offset = fingerprint_address - FLASH_START
    fingerprint = bytes(image[fingerprint_offset:fingerprint_offset + fingerprint_size])
    if fingerprint in (bytes(BUILD_FINGERPRINT_SIZE), bytes([0xff]) * BUILD_FINGERPRINT_SIZE):
        raise RuntimeError("SD pause build fingerprint is invalid in running ELF")

    return symbols, sorted(segments)


def read_segment_bytes(segments, address, size):
    for segment_address, data in segments:
        offset = address - segment_address
        if 0 <= offset and offset + size <= len(data):
            return data[offset:offset + size]
    raise RuntimeError(f"ELF image does not contain {size} bytes at {address:#x}")


def short_identity_segments(symbols, image_segments):
    guard_address = symbols[MAILBOX_SYMBOLS[0]][0]
    fingerprint_address = symbols[MAILBOX_SYMBOLS[1]][0]
    return [
        (guard_address, read_segment_bytes(image_segments, guard_address, 4)),
        (fingerprint_address, read_segment_bytes(
            image_segments, fingerprint_address, BUILD_FINGERPRINT_SIZE)),
    ]


def write_identity_segments(output, segments, stem):
    result = []
    for index, (address, data) in enumerate(segments):
        expected = (output / f"{stem}-expected-{index}.bin").resolve()
        readback = (output / f"{stem}-readback-{index}.bin").resolve()
        expected.write_bytes(data)
        result.append((address, expected, readback))
    return result


def openocd_prefix(serial, speed):
    return [
        "source [find interface/cmsis-dap.cfg]", f"adapter serial {serial}",
        "transport select swd", "gdb_port disabled", "tcl_port disabled",
        "telnet_port disabled", "source [find target/stm32h7x.cfg]",
        f"adapter speed {speed}", "reset_config none", "init",
    ]


def identity_commands(serial, speed, identity_segments):
    commands = openocd_prefix(serial, speed)
    for address, expected, readback in identity_segments:
        commands.append(f"dump_image {{{readback.as_posix()}}} {address:#x} "
                        f"{expected.stat().st_size}")
    commands.append("shutdown")
    return commands


def compare_identity_segments(identity_segments):
    for address, expected, readback in identity_segments:
        if readback.read_bytes() != expected.read_bytes():
            raise RuntimeError(f"Running firmware does not match ELF at {address:#x}")


def pause_commands(symbols, fingerprint, timeout_ms, halt_after=True):
    guard = symbols[MAILBOX_SYMBOLS[0]][0]
    fingerprint_address = symbols[MAILBOX_SYMBOLS[1]][0]
    request = symbols[MAILBOX_SYMBOLS[2]][0]
    ready = symbols[MAILBOX_SYMBOLS[3]][0]
    commands = [
        f"if {{[read_memory {guard:#x} 32 1] != {MAILBOX_GUARD:#x}}} "
        "{error {SD pause mailbox guard mismatch}}",
    ]
    for offset in range(0, BUILD_FINGERPRINT_SIZE, 4):
        expected = int.from_bytes(fingerprint[offset:offset + 4], "little")
        commands.append(
            f"if {{[read_memory {fingerprint_address + offset:#x} 32 1] != {expected:#x}}} "
            "{error {running firmware build fingerprint mismatch}}")
    commands.extend([
        f"if {{[read_memory {request:#x} 32 1] != 0}} "
        "{error {SD pause request mailbox is busy}}",
        f"if {{[read_memory {ready:#x} 32 1] != 0}} "
        "{error {SD pause result mailbox is not clear}}",
        f"mww {request:#x} {PAUSE_REQUEST:#x}",
        "set sd_pause_state 0",
        f"for {{set i 0}} {{$i < {math.ceil(timeout_ms / 20)}}} {{incr i}} {{",
        "    sleep 20",
        f"    set sd_pause_state [read_memory {ready:#x} 32 1]",
        "    if {$sd_pause_state != 0} { break }",
        "}",
        f"if {{$sd_pause_state == {PAUSE_FAILED:#x}}} "
        "{error {firmware failed to sync and close the SD log}}",
        f"if {{$sd_pause_state != {PAUSE_READY:#x}}} "
        "{error {firmware did not confirm SD log pause}}",
    ])
    if halt_after:
        commands.append("halt")
    return commands


def pause_and_halt_commands(serial, speed, symbols, fingerprint, timeout_ms,
                            full_identity_segments):
    commands = openocd_prefix(serial, speed)
    commands.extend(pause_commands(symbols, fingerprint, timeout_ms))
    for address, expected, readback in full_identity_segments:
        commands.append(f"dump_image {{{readback.as_posix()}}} {address:#x} "
                        f"{expected.stat().st_size}")
    commands.append("shutdown")
    return commands


def build_commands(args, candidate, image_size, before, after, readback):
    commands = openocd_prefix(args.serial, args.speed)
    if args.sd_removed:
        commands.append("reset halt")
    else:
        commands.append("halt")
    commands.extend([
        "mww 0x5c001004 0x700187",
        f"dump_image {{{before}}} {CALIBRATION_START:#x} 0x40000",
        f"flash write_image erase {{{candidate}}} {FLASH_START:#x} bin",
        f"verify_image {{{candidate}}} {FLASH_START:#x} bin",
        f"dump_image {{{readback}}} {FLASH_START:#x} {image_size}",
        f"dump_image {{{after}}} {CALIBRATION_START:#x} 0x40000", "shutdown",
    ])
    return commands


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--build", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--serial", required=True)
    parser.add_argument("--speed", type=int, choices=(500, 1000, 2000), default=500)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--running-elf", type=Path,
                      help="与板上当前固件完全一致且包含停写邮箱的 ELF")
    mode.add_argument("--sd-removed", action="store_true",
                      help="确认 SD 已物理移除后，允许旧固件直接进入 reset halt")
    parser.add_argument("--pause-timeout-ms", type=int, default=5000)
    args = parser.parse_args()
    if not 100 <= args.pause_timeout_ms <= 30000:
        parser.error("--pause-timeout-ms must be between 100 and 30000")
    return args


def run_openocd(root, cfg, log_path, timeout):
    base = root / "local/cache/zephyr-sdk/hosttools/openocd"
    with log_path.open("w", encoding="utf-8") as log:
        result = subprocess.run([str(base / "bin/openocd.exe"), "-s",
                                 str(base / "share/openocd/scripts"), "-f", str(cfg.resolve())],
                                stdout=log, stderr=subprocess.STDOUT, timeout=timeout)
    print(log_path.read_text(encoding="utf-8"), flush=True)
    result.check_returncode()


def verify_running_identity(root, output, serial, speed, running_elf):
    require_hero_m_config(running_elf.parent / ".config")
    symbols, segments = load_running_elf(running_elf)
    identity = short_identity_segments(symbols, segments)
    identity_segments = write_identity_segments(output, identity, "running-short")
    identity_cfg = output / "running-short-identity.cfg"
    identity_cfg.write_text("\n".join(identity_commands(
        serial, speed, identity_segments)) + "\n", encoding="utf-8")
    run_openocd(root, identity_cfg, output / "running-short-identity.log", 60)
    compare_identity_segments(identity_segments)
    fingerprint_address = symbols[MAILBOX_SYMBOLS[1]][0]
    fingerprint = read_segment_bytes(
        segments, fingerprint_address, BUILD_FINGERPRINT_SIZE)
    return symbols, segments, fingerprint


def quiesce_running_firmware(root, output, serial, speed, running_elf, timeout_ms=5000):
    """核对运行固件并只停写 SD；成功后主控继续运行，供安全取卡。"""
    output.mkdir(parents=True, exist_ok=True)
    symbols, _, fingerprint = verify_running_identity(
        root, output, serial, speed, running_elf.resolve())
    pause_cfg = output / "sd-log-quiesce.cfg"
    commands = openocd_prefix(serial, speed)
    commands.extend(pause_commands(
        symbols, fingerprint, timeout_ms, halt_after=False))
    commands.append("shutdown")
    pause_cfg.write_text("\n".join(commands) + "\n", encoding="utf-8")
    run_openocd(root, pause_cfg, output / "sd-log-quiesce.log", 60)
    return symbols


def quiesce_and_verify_running_firmware(root, output, serial, speed,
                                        running_elf, timeout_ms=5000):
    """短身份确认后停写并暂停主控，再完整核对旧固件。"""
    output.mkdir(parents=True, exist_ok=True)
    symbols, segments, fingerprint = verify_running_identity(
        root, output, serial, speed, running_elf.resolve())
    full_identity = write_identity_segments(output, segments, "running-full")
    cfg = output / "sd-log-quiesce-halt.cfg"
    cfg.write_text("\n".join(pause_and_halt_commands(
        serial, speed, symbols, fingerprint, timeout_ms, full_identity)) + "\n",
        encoding="utf-8")
    run_openocd(root, cfg, output / "sd-log-quiesce-halt.log", 120)
    compare_identity_segments(full_identity)
    return symbols


def main():
    args = parse_args()
    root = Path(__file__).resolve().parents[2]
    args.output.mkdir(parents=True, exist_ok=True)
    image = args.build / "zephyr/zephyr.bin"
    data = image.read_bytes()
    if not 0 < len(data) <= CALIBRATION_START - FLASH_START:
        raise RuntimeError("Image overlaps calibration area")
    require_hero_m_config(args.build / "zephyr/.config")

    symbols = None
    running_elf_sha256 = None
    if not args.sd_removed:
        running_elf = args.running_elf.resolve()
        symbols = quiesce_and_verify_running_firmware(
            root, args.output, args.serial, args.speed, running_elf,
            args.pause_timeout_ms)
        running_elf_sha256 = hashlib.sha256(running_elf.read_bytes()).hexdigest()

    for name in ("zephyr.bin", "zephyr.elf", ".config"):
        shutil.copyfile(args.build / "zephyr" / name, args.output / name)
    candidate = (args.output / "zephyr.bin").resolve().as_posix()
    before = (args.output / "calibration-before.bin").resolve().as_posix()
    after = (args.output / "calibration-after.bin").resolve().as_posix()
    readback = (args.output / "program-readback.bin").resolve().as_posix()
    commands = build_commands(args, candidate, len(data), before, after, readback)
    cfg = args.output / "flash.cfg"
    cfg.write_text("\n".join(commands) + "\n", encoding="utf-8")
    run_openocd(root, cfg, args.output / "flash.log", 240)
    if Path(readback).read_bytes() != data:
        raise RuntimeError("Program readback mismatch; target remains halted")
    if Path(before).read_bytes() != Path(after).read_bytes():
        raise RuntimeError("Calibration mismatch; target remains halted")
    summary = {
        "image_bytes": len(data),
        "sha256": hashlib.sha256(data).hexdigest(),
        "readback_match": True,
        "calibration_unchanged": True,
        "target_halted": True,
        "probe_serial": args.serial,
        "sd_pause_mode": "sd-removed" if args.sd_removed else "firmware-mailbox",
        "running_elf_sha256": running_elf_sha256,
    }
    (args.output / "verify.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary), flush=True)


if __name__ == "__main__":
    main()
