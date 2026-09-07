"""从现有 H723 工程提取最小依赖，生成并编译独立 SD 测试固件。"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
TEMPLATE = ROOT / "projects/SENTINEL-M/MDK-ARM/SENTINEL-M.uvprojx"
KEEP = {
    "startup_stm32h723xx.s", "system_stm32h7xx.c", "stm32h7xx_hal_timebase_tim.c",
    "stm32h7xx_hal.c", "stm32h7xx_hal_cortex.c", "stm32h7xx_hal_rcc.c",
    "stm32h7xx_hal_rcc_ex.c", "stm32h7xx_hal_pwr.c", "stm32h7xx_hal_pwr_ex.c",
    "stm32h7xx_hal_flash.c", "stm32h7xx_hal_flash_ex.c", "stm32h7xx_hal_gpio.c",
    "stm32h7xx_hal_dma.c", "stm32h7xx_hal_dma_ex.c", "stm32h7xx_hal_spi.c",
    "stm32h7xx_hal_spi_ex.c", "stm32h7xx_hal_tim.c", "stm32h7xx_hal_tim_ex.c",
    "croutine.c", "event_groups.c", "list.c", "queue.c", "stream_buffer.c",
    "tasks.c", "timers.c", "cmsis_os2.c", "heap_4.c", "port.c",
    "MemMang4.c", "ff.c", "ffsystem.c", "ffunicode.c", "Diskio.c",
    "SdSpi.c", "BspSdSpiPort.c",
}


def full_path(value):
    return (TEMPLATE.parent / value.replace("\\", "/")).resolve()


def create_project(out):
    tree = ET.parse(TEMPLATE)
    target = tree.find("./Targets/Target")
    target.find("TargetName").text = "SD-BENCH-M"
    common = target.find("./TargetOption/TargetCommonOption")
    common.find("OutputDirectory").text = str(out / "Objects") + "\\"
    common.find("ListingPath").text = str(out / "Objects") + "\\"
    common.find("OutputName").text = "SD-BENCH-M"
    for node in common.findall(".//RunUserProg1") + common.findall(".//RunUserProg2"):
        node.text = "0"
    for node in target.findall(".//IncludePath"):
        if node.text:
            node.text = ";".join([str(HERE)] + [str(full_path(p)) for p in node.text.split(";") if p])
    define = target.find("./TargetOption/TargetArmAds/Cads/VariousControls/Define")
    define.text = define.text + ",SD_BENCH_TEST"
    groups = target.find("Groups")
    sources = []
    for group in list(groups):
        files = group.find("Files")
        if files is None:
            groups.remove(group)
            continue
        for entry in list(files):
            if entry.findtext("FileName") not in KEEP:
                files.remove(entry)
                continue
            path = full_path(entry.findtext("FilePath"))
            if not path.is_file():
                raise FileNotFoundError(path)
            entry.find("FilePath").text = str(path)
            sources.append(path)
        if not len(files):
            groups.remove(group)
    found = {path.name for path in sources}
    if found != KEEP:
        raise RuntimeError(f"模板缺少依赖: {sorted(KEEP - found)}")
    group = ET.SubElement(groups, "Group")
    ET.SubElement(group, "GroupName").text = "SD bench only"
    files = ET.SubElement(group, "Files")
    for name in ("BenchMain.c", "SdBench.c"):
        path = HERE / name
        entry = ET.SubElement(files, "File")
        ET.SubElement(entry, "FileName").text = name
        ET.SubElement(entry, "FileType").text = "1"
        ET.SubElement(entry, "FilePath").text = str(path)
        sources.append(path)
    out.mkdir(parents=True, exist_ok=True)
    (out / "Objects").mkdir(exist_ok=True)
    project = out / "SD-BENCH-M.uvprojx"
    ET.indent(tree, space="  ")
    tree.write(project, encoding="utf-8", xml_declaration=True)
    manifest = {
        "target": "STM32H723VGTx / M板+副板V2 / SPI3",
        "runtime": "最小 FreeRTOS 测试程序，不包含机器人任务，不代表 Zephyr 已修复",
        "hardware_tested": False,
        "sources": {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
    }
    (out / "sources.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")
    return project


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=ROOT / "local/cache/sd-bench-m")
    parser.add_argument("--generate-only", action="store_true")
    args = parser.parse_args()
    out = args.out.resolve()
    project = create_project(out)
    print(f"独立工程: {project}", flush=True)
    if args.generate_only:
        return
    uv = shutil.which("uv4") or str(Path(os.environ["USERPROFILE"]) / ".local/bin/UV4.exe")
    if not Path(uv).is_file():
        uv = r"C:\App\Work\keil5\UV4\uVision.com"
    log = out / "build.log"
    # 避免把旧日志或上次成功的二进制误认成本次结果。
    for old in [log, out / "firmware.json", out / "Objects/SD-BENCH-M.axf", out / "Objects/SD-BENCH-M.hex"]:
        old.unlink(missing_ok=True)
    run = subprocess.run([uv, "-r", str(project), "-t", "SD-BENCH-M", "-j0", "-o", str(log)],
                         cwd=out, creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
    content = log.read_text(encoding="utf-8", errors="replace") if log.exists() else "未生成日志"
    print("\n".join(content.splitlines()[-12:]), flush=True)
    if run.returncode != 0 or not re.search(r"0 Error\(s\), 0 Warning\(s\)", content):
        raise SystemExit(f"编译未通过零错误零警告检查，退出码 {run.returncode}，日志 {log}")
    axf = out / "Objects/SD-BENCH-M.axf"
    hex_file = out / "Objects/SD-BENCH-M.hex"
    if not axf.is_file() or not hex_file.is_file():
        raise SystemExit("编译未生成本次AXF/HEX")
    from elftools.elf.elffile import ELFFile
    with axf.open("rb") as stream:
        symbols = ELFFile(stream).get_section_by_name(".symtab")
        for forbidden in ("MX_FREERTOS_Init", "HAL_FDCAN_Init", "MotorInstInit", "ChassisControlTask"):
            if symbols.get_symbol_by_name(forbidden):
                raise RuntimeError(f"独立固件意外链接机器人入口: {forbidden}")
        state = symbols.get_symbol_by_name("SdBenchState")[0]
        if state["st_size"] != 1144:
            raise RuntimeError("诊断结构大小变了，需要同步结果读取工具")
        abi = {"symbol": "SdBenchState", "address": state["st_value"], "size": state["st_size"],
               "axf_sha256": hashlib.sha256(axf.read_bytes()).hexdigest(),
               "hex_sha256": hashlib.sha256(hex_file.read_bytes()).hexdigest()}
    (out / "firmware.json").write_text(json.dumps(abi, indent=2), encoding="utf-8")
    print(f"可烧录文件: {hex_file}\n诊断地址: 0x{abi['address']:08X}, {abi['size']} 字节", flush=True)


if __name__ == "__main__":
    main()
