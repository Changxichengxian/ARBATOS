"""从 Zephyr 链接映射生成桌面端可显示的内存报告。

本模块只读取 ``local/build/<target>/zephyr`` 中的产物，不调用外部命令。
链接映射是链接器的真实结果；模块归属是按对象文件推断，因对齐、链接器
生成内容和归档库粒度的限制，不能把它当成逐行源码的精确账本。
"""

from __future__ import annotations

from collections import defaultdict
from datetime import datetime, timezone
import hashlib
from pathlib import Path
import re
import struct
from typing import Any


_TARGET = re.compile(r"[A-Za-z0-9][A-Za-z0-9_-]{0,63}\Z")
_MEMORY = re.compile(r"^(\S+)\s+(0x[0-9a-fA-F]+)\s+(0x[0-9a-fA-F]+)\s*(.*)$")
_OUTPUT = re.compile(
    r"^(\S+)\s+(0x[0-9a-fA-F]+)\s+(0x[0-9a-fA-F]+)(?:\s+load address\s+(0x[0-9a-fA-F]+))?\s*$"
)
_OUTPUT_NAME = re.compile(r"^(\S+)\s*$")
_OUTPUT_CONTINUED = re.compile(r"^\s+(0x[0-9a-fA-F]+)\s+(0x[0-9a-fA-F]+)(?:\s+load address\s+(0x[0-9a-fA-F]+))?\s*$")
_INPUT = re.compile(r"^\s+(\S+)\s+(0x[0-9a-fA-F]+)\s+(0x[0-9a-fA-F]+)\s+(.+?\.obj\))\s*$")
_INPUT_CONTINUED = re.compile(r"^\s+(0x[0-9a-fA-F]+)\s+(0x[0-9a-fA-F]+)\s+(.+?\.obj\))\s*$")
_SECTION_ONLY = re.compile(r"^\s+(\.\S+)\s*$")
_SYMBOL = re.compile(r"^\s+(0x[0-9a-fA-F]+)\s+([^\s=][^\s]*)\s*$")


def _is_reparse(path: Path) -> bool:
    return path.is_symlink() or getattr(path, "is_junction", lambda: False)()


def _safe_artifact(root: Path, target: str) -> tuple[Path | None, str | None]:
    """返回经过每一级检查的产物目录，拒绝符号链接/联接点越界。"""
    if not isinstance(target, str) or not _TARGET.fullmatch(target):
        return None, "车型名只能含字母、数字、短横线和下划线"
    try:
        root = Path(root)
        if not root.is_dir() or _is_reparse(root):
            return None, "工作区目录不可用或是重解析目录"
        real_root = root.resolve(strict=True)
        current = root
        for part in ("local", "build", target.lower(), "zephyr"):
            current = current / part
            if not current.exists():
                return None, "尚未找到该车型的构建产物，请先完成一次构建"
            if not current.is_dir() or _is_reparse(current):
                return None, "构建产物目录不能经过符号链接或联接点"
            current.resolve(strict=True).relative_to(real_root)
        return current, None
    except (OSError, ValueError):
        return None, "构建产物目录超出工作区或无法访问"


def _file_info(path: Path, root: Path) -> dict[str, Any]:
    path = path.resolve(strict=True)
    data = path.read_bytes()
    stat = path.stat()
    return {
        "path": path.relative_to(root).as_posix(),
        "bytes": len(data),
        "modified_at": datetime.fromtimestamp(stat.st_mtime, timezone.utc).isoformat(),
        "sha256": hashlib.sha256(data).hexdigest(),
    }


def _merge_size(intervals: list[tuple[int, int]]) -> int:
    if not intervals:
        return 0
    total = 0
    start, end = sorted(intervals)[0]
    for next_start, next_end in sorted(intervals)[1:]:
        if next_start > end:
            total += end - start
            start, end = next_start, next_end
        else:
            end = max(end, next_end)
    return total + end - start


def _category(object_path: str) -> tuple[str, str]:
    value = object_path.replace("\\", "/")
    lower = value.lower()
    if value.startswith("app/"):
        # app/libapp.a 在 map 中没有原始目录，只能按对象名推断业务用途。
        object_lower = lower.rsplit("/", 1)[-1]
        if any(word in object_lower for word in ("sd", "fat", "ffunicode", "diskio", "filesystem")):
            return "SD 与文件系统", "按对象名推断"
        if any(word in object_lower for word in ("watch", "health", "fault", "log", "diag", "reset", "evidence")):
            return "Watch 与诊断", "按对象名推断"
        if any(word in object_lower for word in ("wheelleg", "wheel_leg", "wheele", "leg")):
            return "轮腿", "按对象名推断"
        if any(word in object_lower for word in ("chassis", "swerve", "omni")):
            return "底盘", "按对象名推断"
        if "gimbal" in object_lower:
            return "云台", "按对象名推断"
        if any(word in object_lower for word in ("robotarm", "manipulator", "armtask")):
            return "机械臂", "按对象名推断"
        if any(word in object_lower for word in ("motor", "control", "pid", "lowcmd", "actuator")):
            return "控制与电机", "按对象名推断"
        if any(word in object_lower for word in ("can", "uart", "rs485", "elrs", "sbus", "usb", "serial", "comm", "rc")):
            return "通信", "按对象名推断"
        return "应用其他", "按对象名推断"
    if "/drivers/" in lower or lower.startswith("zephyr/drivers/"):
        return "Zephyr 驱动", "操作系统驱动"
    if "/subsys/" in lower or lower.startswith("zephyr/subsys/"):
        return "Zephyr 子系统", "操作系统子系统"
    if lower.startswith("zephyr/kernel/"):
        return "Zephyr 内核", "操作系统内核"
    if "stm32cube" in lower or "hal_stm32" in lower:
        return "STM32 HAL", "芯片厂商库"
    if "libc" in lower or "libgcc" in lower or "arm-zephyr-eabi" in lower:
        return "运行库", "C 库与编译器运行库"
    if lower.startswith("zephyr/"):
        return "Zephyr 其他", "操作系统其他部分"
    return "未归属", "无法由对象路径可靠归类"


def _object_name(raw: str) -> str:
    return raw.replace("\\", "/")


def _elf_alloc_sections(data: bytes, symbols: list | None = None) -> list[dict[str, int | str]]:
    """读取 ELF section table，只保留 SHF_ALLOC 段；不依赖 pyelftools。"""
    if len(data) < 52 or data[:4] != b"\x7fELF" or data[4] not in (1, 2) or data[5] not in (1, 2):
        raise ValueError("zephyr.elf 不是可识别的 ELF 文件")
    endian = "<" if data[5] == 1 else ">"
    is_64 = data[4] == 2
    if is_64 and len(data) < 64:
        raise ValueError("ELF 文件头不完整")
    if is_64:
        header = struct.unpack_from(endian + "HHIQQQIHHHHHH", data, 16)
        section_offset, entry_size, section_count, names_index = header[5], header[10], header[11], header[12]
        layout = endian + "IIQQQQIIQQ"
    else:
        header = struct.unpack_from(endian + "HHIIIIIHHHHHH", data, 16)
        section_offset, entry_size, section_count, names_index = header[5], header[10], header[11], header[12]
        layout = endian + "IIIIIIIIII"
    minimum = struct.calcsize(layout)
    if not entry_size or entry_size < minimum or names_index >= section_count:
        raise ValueError("ELF section table 损坏")
    if section_offset + entry_size * section_count > len(data):
        raise ValueError("ELF section table 超出文件范围")

    def entry(index: int):
        return struct.unpack_from(layout, data, section_offset + index * entry_size)

    names = entry(names_index)
    name_offset, name_size = names[4], names[5]
    if name_offset + name_size > len(data):
        raise ValueError("ELF 字符串表超出文件范围")
    strings = data[name_offset:name_offset + name_size]

    def name(offset: int) -> str:
        if offset >= len(strings):
            return ""
        return strings[offset:strings.find(b"\0", offset) if b"\0" in strings[offset:] else len(strings)].decode("utf-8", "replace")

    sections = []
    for index in range(section_count):
        item = entry(index)
        # 32/64 位两种布局中这几个字段的位置相同。
        section_name, section_type, flags, address, _offset, size = item[:6]
        if size and flags & 0x2:  # SHF_ALLOC
            sections.append({"name": name(section_name), "type": section_type, "address": address, "size": size})
    if symbols is not None:
        seen = set()
        symbol_layout = endian + ("IBBHQQ" if is_64 else "IIIBBH")
        symbol_size = struct.calcsize(symbol_layout)
        for index in range(section_count):
            table = entry(index)
            if table[1] != 2:  # SHT_SYMTAB；动态表重复项不另统计
                continue
            offset, size, link, stride = table[4], table[5], table[6], table[9]
            if stride < symbol_size or size % stride or offset + size > len(data) or link >= section_count:
                raise ValueError("ELF 符号表损坏")
            strings_entry = entry(link)
            if strings_entry[1] != 3 or strings_entry[4] + strings_entry[5] > len(data):
                raise ValueError("ELF 符号字符串表损坏")
            symbol_strings = data[strings_entry[4]:strings_entry[4] + strings_entry[5]]
            for position in range(offset, offset + size, stride):
                values = struct.unpack_from(symbol_layout, data, position)
                if is_64:
                    text_offset, info, _other, section_index, address, length = values
                else:
                    text_offset, address, length, info, _other, section_index = values
                kind = info & 15
                if not length or kind not in (1, 2) or not 0 < section_index < section_count:
                    continue
                owner = entry(section_index)
                if not owner[2] & 2 or text_offset >= len(symbol_strings):
                    continue
                # ARM Thumb 函数地址最低位是指令集标记，不是占用的字节。
                if kind == 2 and header[1] == 40:
                    address &= ~1
                if address < owner[3] or address + length > owner[3] + owner[5]:
                    continue
                end = symbol_strings.find(b"\0", text_offset)
                if end < 0:
                    raise ValueError("ELF 符号名未终止")
                symbol_name = symbol_strings[text_offset:end].decode("utf-8", "replace")
                key = (section_index, address, length)
                if key in seen or not symbol_name:
                    continue
                seen.add(key)
                symbols.append({"name": symbol_name, "address": address, "bytes": length,
                                "section": name(owner[0]), "kind": "code" if kind == 2 else "data"})
    return sections


def _parse_map(text: str) -> tuple[list[dict[str, Any]], list[dict[str, Any]], list[dict[str, Any]]]:
    """解析 Memory Configuration、输出段和已链接的输入段。

    只从 ``Linker script and memory map`` 之后读取，因此 Discarded input
    sections 中出现的对象段不会进入任何统计。
    """
    lines = text.splitlines()
    regions: list[dict[str, Any]] = []
    memory_start = next((i for i, line in enumerate(lines) if line.strip() == "Memory Configuration"), None)
    link_start = next((i for i, line in enumerate(lines) if line.strip() == "Linker script and memory map"), None)
    if memory_start is None or link_start is None:
        return regions, [], []
    for line in lines[memory_start + 2:link_start]:
        match = _MEMORY.match(line.strip())
        if not match or match.group(1) == "*default*":
            continue
        regions.append({"name": match.group(1), "origin": int(match.group(2), 16), "capacity": int(match.group(3), 16)})

    outputs: list[dict[str, Any]] = []
    inputs: list[dict[str, Any]] = []
    current: dict[str, Any] | None = None
    current_input: dict[str, Any] | None = None
    pending_section: str | None = None
    pending_output_name: str | None = None
    for line in lines[link_start + 1:]:
        output = _OUTPUT.match(line)
        if output:
            size = int(output.group(3), 16)
            current = {"name": output.group(1), "vma": int(output.group(2), 16), "size": size,
                       "lma": int(output.group(4), 16) if output.group(4) else None}
            outputs.append(current)
            current_input = None
            pending_section = None
            pending_output_name = None
            continue
        output_continued = _OUTPUT_CONTINUED.match(line)
        if output_continued and pending_output_name:
            current = {"name": pending_output_name, "vma": int(output_continued.group(1), 16),
                       "size": int(output_continued.group(2), 16),
                       "lma": int(output_continued.group(3), 16) if output_continued.group(3) else None}
            outputs.append(current)
            current_input = None
            pending_section = None
            pending_output_name = None
            continue
        item = _INPUT.match(line)
        continued = _INPUT_CONTINUED.match(line)
        if (item or (continued and pending_section)) and current and current["size"]:
            section = item.group(1) if item else pending_section
            address = item.group(2) if item else continued.group(1)
            size = item.group(3) if item else continued.group(2)
            object_name = item.group(4) if item else continued.group(3)
            current_input = {"section": section, "address": int(address, 16),
                             "size": int(size, 16), "object": _object_name(object_name),
                             "output": current, "symbols": []}
            if current_input["size"]:
                inputs.append(current_input)
            pending_section = None
            continue
        section_only = _SECTION_ONLY.match(line)
        if section_only:
            pending_section = section_only.group(1)
            current_input = None
            continue
        output_name = _OUTPUT_NAME.match(line)
        if output_name:
            pending_output_name = output_name.group(1)
            current_input = None
            pending_section = None
            continue
        symbol = _SYMBOL.match(line)
        if symbol and current_input and len(current_input["symbols"]) < 12:
            address, name = int(symbol.group(1), 16), symbol.group(2)
            if (current_input["address"] <= address < current_input["address"] + current_input["size"]
                    and not name.startswith(("PROVIDE", "ASSERT", "."))):
                current_input["symbols"].append(name)
    return regions, outputs, inputs


def build_memory_report(root: Path, target: str) -> dict[str, Any]:
    """返回 JSON 可序列化的构建内存报告；不可用产物会返回友好原因。"""
    artifact_dir, reason = _safe_artifact(root, target)
    result: dict[str, Any] = {"available": False, "target": target, "reason": reason,
                              "regions": [], "categories": [], "modules": [], "allocations": [], "limitations": [
                                  "模块按链接映射中的对象路径归类；归档库和链接器生成内容无法精确回溯到单个源码文件。",
                                  "模块用量取已分配段；区域容量条还包含起始空隙、段间对齐和链接器内容，未归属部分单列。",
                                  "带装载地址的数据段分别计入 Flash 装载量和 RAM 运行量，两个用途不会相互重复求和。",
                                  "容量条的已用量是从区域起点到最高已分配地址的跨度；free 是尾部余量，不代表运行时堆或栈的可用空间。",
                                  "变量和函数明细来自 ELF 符号尺寸，显示运行地址和预留字节；别名去重，未命名内容不在明细中，不能直接相加代替区域总量。",
                              ]}
    if artifact_dir is None:
        return result
    map_path, elf_path = artifact_dir / "zephyr.map", artifact_dir / "zephyr.elf"
    if any(_is_reparse(path) for path in (map_path, elf_path) if path.exists()):
        result["reason"] = "构建产物文件不能是符号链接或联接点"
        return result
    if not map_path.is_file() or not elf_path.is_file():
        missing = "zephyr.map" if not map_path.is_file() else "zephyr.elf"
        result["reason"] = f"构建产物不完整，缺少 {missing}"
        return result
    try:
        root_real = Path(root).resolve(strict=True)
        for path in (map_path, elf_path):
            path.resolve(strict=True).relative_to(root_real)
        map_data = map_path.read_bytes()
        regions, outputs, inputs = _parse_map(map_data.decode("utf-8", errors="replace"))
        allocation_symbols = []
        elf_sections = _elf_alloc_sections(elf_path.read_bytes(), allocation_symbols)
    except (OSError, ValueError) as exc:
        result["reason"] = f"读取构建产物失败：{exc}"
        return result
    if not regions:
        result["reason"] = "链接映射缺少 Memory Configuration，无法可靠统计"
        return result
    map_sections = {(section["name"], section["vma"], section["size"]) for section in outputs}
    if not elf_sections or any((section["name"], section["address"], section["size"]) not in map_sections
                               for section in elf_sections):
        result["reason"] = "ELF 与链接映射不完整或不匹配，请等待构建结束后刷新"
        return result

    # 名称不同、起点和长度相同的是同一物理区域的别名（例如 RAM/SRAM0）。
    canonical: dict[tuple[int, int], str] = {}
    for item in regions:
        key = (item["origin"], item["capacity"])
        item["physical_name"] = canonical.setdefault(key, item["name"])
        item["is_alias"] = item["physical_name"] != item["name"]
    physical = {item["physical_name"]: item for item in regions if not item["is_alias"]}

    def locate(address: int, size: int) -> str | None:
        if size <= 0:
            return None
        for region in physical.values():
            if region["origin"] <= address and address + size <= region["origin"] + region["capacity"]:
                return region["name"]
        return None

    # ELF 的 SHF_ALLOC VMA 是区域统计唯一来源。map 只在段名、地址、尺寸
    # 全部相同后用于把对象归属回去，不能用地址重叠猜测：tbss/rodata 可同址，
    # debug 段也可能是 0 地址。
    elf_by_key: dict[tuple[str, int, int], list[dict[str, int | str]]] = defaultdict(list)
    for elf_section in elf_sections:
        elf_by_key[(str(elf_section["name"]), int(elf_section["address"]), int(elf_section["size"]))].append(elf_section)
    valid_outputs: dict[int, dict[str, int | str]] = {}
    intervals: dict[str, list[tuple[int, int]]] = defaultdict(list)
    elf_intervals: dict[str, list[tuple[int, int]]] = defaultdict(list)
    image_intervals: dict[str, list[tuple[int, int]]] = defaultdict(list)
    for elf_section in elf_sections:
        region = locate(int(elf_section["address"]), int(elf_section["size"]))
        if region:
            interval = (int(elf_section["address"]), int(elf_section["address"]) + int(elf_section["size"]))
            intervals[region].append(interval)
            elf_intervals[region].append(interval)
            if "FLASH" in region.upper():
                image_intervals[region].append(interval)
    for section in outputs:
        matched = elf_by_key.get((section["name"], section["vma"], section["size"]), [])
        if len(matched) != 1:
            continue
        elf_section = matched[0]
        valid_outputs[id(section)] = elf_section
        if section["lma"] is not None and int(elf_section["type"]) != 8:
            lma_region = locate(section["lma"], section["size"])
            if lma_region:
                intervals[lma_region].append((section["lma"], section["lma"] + section["size"]))
                image_intervals[lma_region].append((section["lma"], section["lma"] + section["size"]))

    totals: dict[str, dict[str, Any]] = {}
    modules: dict[str, dict[str, Any]] = {}
    category_intervals: dict[str, list[tuple[int, int, str]]] = defaultdict(list)
    for item in inputs:
        output = item["output"]
        if id(output) not in valid_outputs:
            continue
        vma_region = locate(item["address"], item["size"]) or locate(output["vma"], output["size"])
        # 同一输出段的输入段通常按 VMA 排列；LMA 与 VMA 的偏移相同。
        lma_address = None
        if output["lma"] is not None and int(valid_outputs[id(output)]["type"]) != 8:
            lma_address = output["lma"] + max(0, item["address"] - output["vma"])
        lma_region = locate(lma_address, item["size"]) if lma_address is not None else None
        category_key, category_label = _category(item["object"])
        for key, label, object_key in ((category_key, category_label, item["object"]),):
            bucket = totals.setdefault(key, {"name": key, "label": label, "flash": 0, "ram": 0, "bytes": 0})
            module = modules.setdefault(object_key, {"name": object_key, "category": key, "flash": 0, "ram": 0,
                                                     "bytes": 0, "symbols": []})
            flash_regions = {name for name in (vma_region, lma_region) if name and "FLASH" in name.upper()}
            ram_regions = {name for name in (vma_region, lma_region) if name and "FLASH" not in name.upper()}
            # 同一物理用途只加一次；data 的 Flash LMA 与 RAM VMA 则各加一次。
            bucket["flash"] += item["size"] * len(flash_regions)
            bucket["ram"] += item["size"] * len(ram_regions)
            bucket["bytes"] += item["size"] * (len(flash_regions) + len(ram_regions))
            module["flash"] += item["size"] * len(flash_regions)
            module["ram"] += item["size"] * len(ram_regions)
            module["bytes"] += item["size"] * (len(flash_regions) + len(ram_regions))
            module["symbols"] = (module["symbols"] + item["symbols"])[:12]
        if vma_region:
            category_intervals[vma_region].append((item["address"], item["address"] + item["size"], category_key))
        if lma_region and lma_address is not None:
            category_intervals[lma_region].append((lma_address, lma_address + item["size"], category_key))

    report_regions = []
    for item in regions:
        source = physical[item["physical_name"]]
        section_bytes = _merge_size(intervals[source["name"]])
        region_ranges = intervals[source["name"]]
        # 链接器的区域用量以 location counter 为准：起点到最高地址之间的洞
        # 已经不能再被后续段回收，因此容量条必须采用这一跨度。
        used = max(end for _start, end in region_ranges) - source["origin"] if region_ranges else 0
        elf_used = _merge_size(elf_intervals[source["name"]])
        image_ranges = image_intervals[source["name"]]
        image_span = (max(end for _start, end in image_ranges) - source["origin"]) if image_ranges else 0
        # 重叠 ELF 段（典型是 tbss/rodata）只给其中一个类别记一次物理字节。
        category_rows_map: dict[str, int] = defaultdict(int)
        category_ranges = category_intervals[source["name"]]
        boundaries = sorted({point for start, end, _name in category_ranges for point in (start, end)})
        for start, end in zip(boundaries, boundaries[1:]):
            owner = next((name for left, right, name in category_ranges if left <= start and end <= right), None)
            if owner:
                category_rows_map[owner] += end - start
        category_rows = [{"name": name, "bytes": size} for name, size in category_rows_map.items() if size]
        assigned = sum(row["bytes"] for row in category_rows)
        remainder = max(0, used - assigned)
        if remainder:
            category_rows.append({"name": "未归属和对齐", "bytes": remainder})
        report_regions.append({"name": item["name"], "label": f"{item['name']} 区域", "origin": item["origin"],
                               "capacity": item["capacity"], "used": used, "free": max(0, item["capacity"] - used),
                               "used_percent": round(used * 100 / item["capacity"], 2) if item["capacity"] else 0,
                               "physical_name": item["physical_name"], "is_alias": item["is_alias"],
                               "unattributed_or_alignment": remainder,
                               "section_bytes": section_bytes,
                               "elf_vma_used": elf_used,
                               "image_span_used": image_span if "FLASH" in source["name"].upper() else None,
                               "image_internal_holes": max(0, image_span - section_bytes) if "FLASH" in source["name"].upper() else None,
                               "categories": sorted(category_rows, key=lambda row: row["bytes"], reverse=True)})
    result.update({
        "available": True,
        "reason": None,
        "artifacts": {"map": _file_info(map_path, root_real), "elf": _file_info(elf_path, root_real)},
        "regions": report_regions,
        "categories": sorted(totals.values(), key=lambda row: row["bytes"], reverse=True),
        "modules": sorted(modules.values(), key=lambda row: row["bytes"], reverse=True)[:80],
        "allocations": sorted(({**row, "region": locate(row["address"], row["bytes"])}
                               for row in allocation_symbols if locate(row["address"], row["bytes"])),
                              key=lambda row: row["bytes"], reverse=True),
    })
    return result
