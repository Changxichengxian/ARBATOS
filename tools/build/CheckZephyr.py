#!/usr/bin/env python3
"""Portable consistency checks for the formal ARBATOS Zephyr presets.

This verifies the checked-in source graph only.  It deliberately does not
configure Zephyr, compile firmware, or access a debugger.
"""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Iterable

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from config.SourcePolicy import FORBIDDEN_SOURCE


FORMAL_MODE_SYMBOLS = (
    "CONFIG_ARBATOS_MUSIC_ONLY",
    "CONFIG_ARBATOS_PREFLIGHT_ONLY",
    "CONFIG_ARBATOS_RECEIVE_ONLY",
)
SOURCE_TOKEN = re.compile(r"(?<![A-Za-z0-9_./-])([A-Za-z0-9_.-]+/[A-Za-z0-9_./-]+\.(?:c|s|lib))(?![A-Za-z0-9_./-])")


class Check:
    def __init__(self, root: Path) -> None:
        self.root = root
        self.errors: list[str] = []
        self.warnings: list[str] = []

    def error(self, message: str) -> None:
        self.errors.append(message)

    def rel(self, path: Path) -> str:
        try:
            return path.relative_to(self.root).as_posix()
        except ValueError:
            return str(path)

    def require_file(self, path: Path) -> bool:
        if not path.is_file():
            self.error(f"missing file: {self.rel(path)}")
            return False
        return True

    def targets(self) -> list[dict[str, object]]:
        generator = self.root / "tools" / "config" / "RobotConfigGen.py"
        if not self.require_file(generator):
            return []
        result = subprocess.run([sys.executable, "-X", "utf8", str(generator), "--root", str(self.root), "list", "--json"],
                                text=True, encoding="utf-8", errors="replace", capture_output=True, check=False)
        if result.returncode:
            self.error("RobotConfigGen list failed: " + (result.stderr or result.stdout).strip())
            return []
        try:
            entries = json.loads(result.stdout)
        except json.JSONDecodeError as exc:
            self.error(f"RobotConfigGen list returned invalid JSON: {exc.msg}")
            return []
        if not entries:
            self.error("RobotConfigGen list returned no RobotConfig.toml target")
        return entries

    def check_presets(self, selected: Iterable[dict[str, object]]) -> None:
        presets_path = self.root / "projects" / "CMakePresets.json"
        if not self.require_file(presets_path):
            return
        try:
            presets = json.loads(presets_path.read_text(encoding="utf-8"))
        except json.JSONDecodeError as exc:
            self.error(f"invalid JSON: projects/CMakePresets.json: {exc.msg}")
            return

        configure = {item.get("name"): item for item in presets.get("configurePresets", [])}
        builds = {item.get("name"): item for item in presets.get("buildPresets", [])}
        base = configure.get("zephyr-base")
        if not base or not base.get("hidden"):
            self.error("projects/CMakePresets.json: missing hidden zephyr-base preset")

        for project in selected:
            name, preset_name, board = str(project["name"]), str(project["preset"]), str(project["board"])
            preset = configure.get(preset_name)
            if not preset:
                # 新车型无需改受跟踪 preset；由本地 CMakeUserPresets 生成。
                continue
            if preset.get("inherits") != "zephyr-base":
                self.error(f"{name}: preset must inherit zephyr-base")
            values = preset.get("cacheVariables", {})
            if values.get("ARBATOS_ROBOT") != name:
                self.error(f"{name}: preset must set ARBATOS_ROBOT to {name}")
            if any(key in values for key in ("BOARD", "EXTRA_CONF_FILE", "DTC_OVERLAY_FILE")):
                self.error(f"{name}: preset repeats generated board/configuration fields")
            board_dir = self.root / "boards" / {"dm_mc02_h7": "DmMc02H7", "dji_a_f427": "DjiAF427", "dji_c_f407": "DjiCF407"}.get(board, "") / "zephyr"
            self.require_file(board_dir / f"{board}.yaml")
            self.require_file(board_dir / f"{board}_defconfig")
            if any(token in str(value).lower() for value in values.values()
                   for token in ("music", "preflight", "receive")):
                self.error(f"{name}: formal preset references a dedicated test mode")
            build = builds.get(preset_name)
            if not build or build.get("configurePreset") != preset_name:
                self.error(f"{name}: missing matching build preset")

    def check_target_config(self, project: dict[str, object]) -> None:
        name = str(project["name"])
        generator = self.root / "tools" / "config" / "RobotConfigGen.py"
        with tempfile.TemporaryDirectory(prefix="arbatos-check-") as output:
            result = subprocess.run([sys.executable, "-X", "utf8", str(generator), "--root", str(self.root), "generate",
                                     "--target", name, "--out", output], text=True, encoding="utf-8",
                                    errors="replace", capture_output=True, check=False)
            if result.returncode:
                self.error(f"{name}: RobotConfigGen generate failed: {(result.stderr or result.stdout).strip()}")
                return
            generated = Path(output)
            report = json.loads((generated / "robot-config.json").read_text(encoding="utf-8"))
            for source in report.get("sources", []) + report.get("port_sources", []):
                if FORBIDDEN_SOURCE.search(source):
                    self.error(f"{name}: forbidden legacy entry: {source}")
            for file_name in ("RobotTargetConfig.h", "RobotTargetProfile.inc", "RobotTargetTasks.inc",
                              "RobotTarget.cmake", "robot-config.json", "robot.conf"):
                self.require_file(generated / file_name)
            conf = (generated / "robot.conf").read_text(encoding="utf-8")
            if f'CONFIG_ARBATOS_ROBOT_NAME="{name}"' not in conf:
                self.error(f"{name}: generated robot.conf does not select its robot name")
            if "CONFIG_ARBATOS_LEGACY_SOURCES=y" not in conf:
                self.error(f"{name}: generated robot.conf must enable legacy sources")
            for symbol in FORMAL_MODE_SYMBOLS:
                if f"{symbol}=y" in conf:
                    self.error(f"{name}: generated robot.conf enables dedicated mode {symbol}")

    def check_source_graph(self) -> None:
        cmake = self.root / "projects" / "CMakeLists.txt"
        manifest = self.root / "projects" / "cmake" / "ArbatosLegacy.cmake"
        required = (cmake, manifest, self.root / "projects" / "src" / "main.c",
                    self.root / "projects" / "src" / "ArbatosTarget.c", self.root / "projects" / "Kconfig")
        if not all(self.require_file(path) for path in required):
            return
        text = cmake.read_text(encoding="utf-8") + "\n" + manifest.read_text(encoding="utf-8")
        for token in sorted(set(SOURCE_TOKEN.findall(text))):
            normalized = token.replace("\\", "/")
            if FORBIDDEN_SOURCE.search(normalized):
                self.error(f"Zephyr source graph includes forbidden legacy entry: {normalized}")
                continue
            candidate = self.root / normalized if normalized.startswith(("shared/", "Robotconfig/", "boards/")) else self.root / "projects" / normalized
            if not candidate.is_file():
                self.error(f"Zephyr source graph references missing source: {normalized}")
        if "src/main.c" not in text or "src/ArbatosTarget.c" not in text:
            self.error("projects/CMakeLists.txt: formal application entry sources are incomplete")

    def run(self, selected: list[dict[str, object]]) -> None:
        for family, board in (("DjiAF427", "dji_a_f427"), ("DjiCF407", "dji_c_f407"), ("DmMc02H7", "dm_mc02_h7")):
            directory = self.root / "boards" / family / "zephyr"
            for name in ("board.yml", "board.cmake", f"{board}.dts", f"{board}-pinctrl.dtsi", f"{board}_defconfig"):
                self.require_file(directory / name)
        self.check_source_graph()
        self.check_presets(selected)
        for project in selected:
            self.check_target_config(project)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Check ARBATOS formal Zephyr source and preset consistency.")
    parser.add_argument("--project", default="all", help="all (default) or a formal project name, e.g. HERO-M")
    parser.add_argument("--json", action="store_true", help="emit one JSON result object")
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[2], help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    check = Check(args.root.resolve())
    targets = check.targets()
    requested = args.project.lower()
    if requested == "all":
        selected = targets
    else:
        selected = [item for item in targets if requested in (str(item["name"]).lower(), str(item["preset"]).lower())]
        if not selected:
            parser.error("--project must be all or one of: " + ", ".join(str(item["name"]) for item in targets))
    check.run(selected)
    result = {"ok": not check.errors, "project": args.project, "checked_projects": [item["name"] for item in selected],
              "errors": check.errors, "warnings": check.warnings}
    if args.json:
        print(json.dumps(result, ensure_ascii=False, sort_keys=True))
    else:
        for message in check.errors:
            print(f"ERROR: {message}")
        for message in check.warnings:
            print(f"WARNING: {message}")
        print(f"CheckZephyr: {'passed' if result['ok'] else 'failed'} ({len(selected)} target(s))")
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
