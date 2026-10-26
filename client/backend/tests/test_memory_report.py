import importlib.util
import struct
import tempfile
import unittest
from pathlib import Path


REPO = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location("memory_report", REPO / "client/backend/memory_report.py")
REPORT = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(REPORT)


MAP = """Memory Configuration

Name             Origin             Length             Attributes
FLASH            0x08000000         0x00001000         xr
RAM              0x20000000         0x00001000         xw
RAM_ALIAS        0x20000000         0x00001000         xw
ITCM             0x00000000         0x00001000         xw
*default*        0x00000000         0xffffffff

Linker script and memory map

Discarded input sections
 .text.discarded 0x00000000 0x80 app/libapp.a(Discarded.c.obj)

.text            0x08000010 0x00000020
 .text.good      0x08000010 0x00000010 app/libapp.a(Good.c.obj)
                0x08000010 GoodSymbol
 .text.good2     0x08000020 0x00000010 app/libapp.a(Good2.c.obj)

datas            0x20000000 0x00000010 load address 0x08000030
 .data.value     0x20000000 0x00000010 app/libapp.a(Data.c.obj)
                0x20000000 DataSymbol

bss              0x20000010 0x00000008
 .bss.value      0x20000010 0x00000008 app/libapp.a(Bss.c.obj)

noinit           0x20000018 0x00000008 load address 0x08000030
 .noinit.stack   0x20000018 0x00000008 app/libapp.a(Stack.c.obj)

ram_tail         0x20000030 0x00000008
 .bss.tail       0x20000030 0x00000008 app/libapp.a(Tail.c.obj)

long_output_area
                0x20000040 0x00000008
 .long.output    0x20000040 0x00000008 app/libapp.a(BspCanZephyr.c.obj)

itcm_text        0x00000000 0x00000010
 .itcm.text      0x00000000 0x00000010 app/libapp.a(ItcmTask.c.obj)

tbss             0x08000040 0x00000008
 .tbss.value     0x08000040 0x00000008 app/libapp.a(SdLog.c.obj)

rodata           0x08000040 0x00000010
 .rodata.value   0x08000040 0x00000010 app/libapp.a(ChassisTask.c.obj)

.debug_info      0x00000000 0x00000080
 .debug_info     0x00000000 0x00000080 app/libapp.a(Debug.c.obj)
"""


def _elf32():
    """最小 ELF：debug 无 SHF_ALLOC，bss/noinit 是 SHT_NOBITS。"""
    strings = b"\0.text\0datas\0bss\0noinit\0ram_tail\0long_output_area\0itcm_text\0tbss\0rodata\0.debug_info\0.shstrtab\0"
    names = {name: strings.index(name.encode()) for name in (".text", "datas", "bss", "noinit", "ram_tail", "long_output_area", "itcm_text", "tbss", "rodata", ".debug_info", ".shstrtab")}
    data = bytearray(0x200 + 12 * 40)
    data[:16] = b"\x7fELF\x01\x01\x01" + b"\0" * 9
    struct.pack_into("<HHIIIIIHHHHHH", data, 16, 1, 40, 1, 0, 0, 0x200, 0, 52, 0, 0, 40, 12, 11)
    data[0x100:0x100 + len(strings)] = strings
    sections = [
        (0, 0, 0, 0, 0, 0),
        (names[".text"], 1, 2, 0x08000010, 0, 0x20),
        (names["datas"], 1, 2, 0x20000000, 0, 0x10),
        (names["bss"], 8, 2, 0x20000010, 0, 0x8),
        (names["noinit"], 8, 2, 0x20000018, 0, 0x8),
        (names["ram_tail"], 8, 2, 0x20000030, 0, 0x8),
        (names["long_output_area"], 1, 2, 0x20000040, 0, 0x8),
        (names["itcm_text"], 1, 2, 0, 0, 0x10),
        (names["tbss"], 8, 2, 0x08000040, 0, 0x8),
        (names["rodata"], 1, 2, 0x08000040, 0, 0x10),
        (names[".debug_info"], 1, 0, 0, 0, 0x80),
        (names[".shstrtab"], 3, 0, 0, 0x100, len(strings)),
    ]
    for index, (name, kind, flags, address, offset, size) in enumerate(sections):
        struct.pack_into("<IIIIIIIIII", data, 0x200 + index * 40, name, kind, flags, address, offset, size, 0, 0, 1, 0)
    return bytes(data)


class MemoryReportTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name) / "repo"
        self.artifact = self.root / "local/build/hero-m/zephyr"
        self.artifact.mkdir(parents=True)
        (self.artifact / "zephyr.map").write_text(MAP, encoding="utf-8")
        (self.artifact / "zephyr.elf").write_bytes(_elf32())

    def tearDown(self):
        self.tmp.cleanup()

    def test_discarded_excluded_and_data_counts_two_uses(self):
        report = REPORT.build_memory_report(self.root, "HERO-M")
        self.assertTrue(report["available"])
        self.assertNotIn("Discarded", str(report["modules"]))
        modules = {row["name"]: row for row in report["modules"]}
        self.assertEqual(modules["app/libapp.a(Data.c.obj)"]["flash"], 0x10)
        self.assertEqual(modules["app/libapp.a(Data.c.obj)"]["ram"], 0x10)
        self.assertEqual(modules["app/libapp.a(Stack.c.obj)"]["ram"], 0x8)
        self.assertIn("DataSymbol", modules["app/libapp.a(Data.c.obj)"]["symbols"])
        self.assertNotIn("Debug.c.obj", str(report["modules"]))

    def test_regions_merge_non_overlapping_sections_and_alias(self):
        report = REPORT.build_memory_report(self.root, "hero-m")
        regions = {row["name"]: row for row in report["regions"]}
        self.assertEqual(regions["FLASH"]["used"], 0x50)
        self.assertEqual(regions["FLASH"]["section_bytes"], 0x40)
        self.assertEqual(regions["RAM"]["used"], 0x48)
        self.assertEqual(regions["RAM"]["section_bytes"], 0x30)
        self.assertEqual(regions["RAM_ALIAS"]["used"], 0x48)
        self.assertTrue(regions["RAM_ALIAS"]["is_alias"])
        self.assertEqual(regions["RAM"]["elf_vma_used"], 0x30)
        self.assertEqual(regions["ITCM"]["used"], 0x10)
        self.assertEqual(next(row["bytes"] for row in regions["RAM"]["categories"] if row["name"] == "未归属和对齐"), 0x18)
        self.assertLessEqual(sum(row["bytes"] for row in regions["FLASH"]["categories"]), regions["FLASH"]["used"])
        self.assertTrue(any(row["name"] == "通信" for row in regions["RAM"]["categories"]))

    def test_app_object_categories_are_inferred(self):
        self.assertEqual(REPORT._category("app/libapp.a(SdLog.c.obj)")[0], "SD 与文件系统")
        self.assertEqual(REPORT._category("app/libapp.a(ChassisTask.c.obj)")[0], "底盘")
        self.assertEqual(REPORT._category("app/libapp.a(BspCanZephyr.c.obj)")[0], "通信")

    def test_symbol_sizes_aliases_and_thumb_address(self):
        data = bytearray(_elf32())
        struct.pack_into("<H", data, 48, 14)
        data.extend(b"\0" * 80)
        names = b"\0Function\0Alias\0Data\0Stack\0Debug\0"
        names_offset = len(data)
        data.extend(names)
        symbols_offset = len(data)
        for name, address, size, kind, section in (
            ("Function", 0x08000011, 16, 2, 1), ("Alias", 0x08000011, 16, 2, 1),
            ("Data", 0x20000000, 16, 1, 2), ("Stack", 0x20000018, 8, 1, 4),
            ("Debug", 0, 8, 1, 10),
        ):
            data.extend(struct.pack("<IIIBBH", names.index(name.encode()), address, size, kind, 0, section))
        struct.pack_into("<IIIIIIIIII", data, 0x200 + 12 * 40, 0, 3, 0, 0, names_offset, len(names), 0, 0, 1, 0)
        struct.pack_into("<IIIIIIIIII", data, 0x200 + 13 * 40, 0, 2, 0, 0, symbols_offset, 80, 12, 0, 4, 16)
        (self.artifact / "zephyr.elf").write_bytes(data)
        report = REPORT.build_memory_report(self.root, "hero-m")
        self.assertTrue(report["available"])
        symbols = {item["name"]: item for item in report["allocations"]}
        self.assertEqual(set(symbols), {"Function", "Data", "Stack"})
        self.assertEqual(symbols["Function"]["address"], 0x08000010)
        self.assertEqual(symbols["Stack"]["bytes"], 8)
        self.assertEqual(symbols["Stack"]["region"], "RAM")
        struct.pack_into("<I", data, 0x200 + 13 * 40 + 36, 0)
        (self.artifact / "zephyr.elf").write_bytes(data)
        self.assertFalse(REPORT.build_memory_report(self.root, "hero-m")["available"])

    def test_missing_and_unsafe_target_are_friendly(self):
        self.assertFalse(REPORT.build_memory_report(self.root, "NO-SUCH")["available"])
        rejected = REPORT.build_memory_report(self.root, "../hero-m")
        self.assertFalse(rejected["available"])
        self.assertIn("车型名", rejected["reason"])
        (self.artifact / "zephyr.elf").unlink()
        self.assertIn("zephyr.elf", REPORT.build_memory_report(self.root, "hero-m")["reason"])

    def test_mismatched_or_truncated_artifact_is_not_reported_as_complete(self):
        (self.artifact / "zephyr.map").write_text(MAP.replace(".text            0x08000010 0x00000020",
                                                            ".text            0x08000010 0x00000024"), encoding="utf-8")
        self.assertFalse(REPORT.build_memory_report(self.root, "hero-m")["available"])
        header = bytearray(_elf32()[:60])
        header[4] = 2
        (self.artifact / "zephyr.elf").write_bytes(header)
        self.assertFalse(REPORT.build_memory_report(self.root, "hero-m")["available"])
