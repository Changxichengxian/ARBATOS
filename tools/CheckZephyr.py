#!/usr/bin/env python3
"""Portable consistency checks for the formal ARBATOS Zephyr presets.

This verifies the checked-in source graph only.  It deliberately does not
configure Zephyr, compile firmware, access a debugger, or replace CheckAll.ps1
and its legacy Keil-project checks.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path
from typing import Iterable


PROJECTS = {
    "HERO-M": ("hero-m", "dm_mc02_h7"),
    "HERO-C": ("hero-c", "dji_c_f407"),
    "SENTINEL-M": ("sentinel-m", "dm_mc02_h7"),
    "INFANTRY-A": ("infantry-a", "dji_a_f427"),
    "CARRIER-A": ("carrier-a", "dji_a_f427"),
    "MINIWHEELEG-M": ("miniwheeleg-m", "dm_mc02_h7"),
    "MINIWHEELEG-C": ("miniwheeleg-c", "dji_c_f407"),
}
FORMAL_MODE_SYMBOLS = (
    "CONFIG_ARBATOS_MUSIC_ONLY",
    "CONFIG_ARBATOS_PREFLIGHT_ONLY",
    "CONFIG_ARBATOS_RECEIVE_ONLY",
)
FORBIDDEN_SOURCE = re.compile(
    r"(?:^|/)(?:projects/|shared/hal/|.*(?:boardmain|boardfreertos|instask)[^/]*\.c$|"
    r"boards/(?:DjiCF407|DjiAF427)/(?:bsp|devices)/|.*\.(?:s|lib)$)", re.IGNORECASE
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

    def check_presets(self, selected: Iterable[str]) -> None:
        presets_path = self.root / "zephyr" / "CMakePresets.json"
        if not self.require_file(presets_path):
            return
        try:
            presets = json.loads(presets_path.read_text(encoding="utf-8"))
        except json.JSONDecodeError as exc:
            self.error(f"invalid JSON: zephyr/CMakePresets.json: {exc.msg}")
            return

        configure = {item.get("name"): item for item in presets.get("configurePresets", [])}
        builds = {item.get("name"): item for item in presets.get("buildPresets", [])}
        base = configure.get("zephyr-base")
        if not base or not base.get("hidden"):
            self.error("zephyr/CMakePresets.json: missing hidden zephyr-base preset")

        for project in selected:
            preset_name, board = PROJECTS[project]
            preset = configure.get(preset_name)
            if not preset:
                self.error(f"{project}: missing configure preset '{preset_name}'")
                continue
            if preset.get("inherits") != "zephyr-base":
                self.error(f"{project}: preset must inherit zephyr-base")
            values = preset.get("cacheVariables", {})
            if values.get("BOARD") != board:
                self.error(f"{project}: BOARD must be {board}")
            board_dir = self.root / "zephyr" / "boards" / board
            self.require_file(board_dir / f"{board}.yaml")
            self.require_file(board_dir / f"{board}_defconfig")
            conf = values.get("EXTRA_CONF_FILE")
            expected = f"${{sourceDir}}/targets/{preset_name}.conf"
            if conf != expected:
                self.error(f"{project}: EXTRA_CONF_FILE must be {expected}")
            overlay = values.get("DTC_OVERLAY_FILE")
            if project == "SENTINEL-M":
                expected_overlay = "${sourceDir}/targets/sentinel-m.overlay"
                if overlay != expected_overlay:
                    self.error(f"{project}: DTC_OVERLAY_FILE must be {expected_overlay}")
                self.require_file(self.root / "zephyr" / "targets" / "sentinel-m.overlay")
            elif overlay is not None:
                self.error(f"{project}: formal preset must not set DTC_OVERLAY_FILE")
            if any(token in str(value).lower() for value in values.values()
                   for token in ("music", "preflight", "receive")):
                self.error(f"{project}: formal preset references a dedicated test mode")
            build = builds.get(preset_name)
            if not build or build.get("configurePreset") != preset_name:
                self.error(f"{project}: missing matching build preset")

    def check_target_config(self, project: str) -> None:
        preset_name, _ = PROJECTS[project]
        path = self.root / "zephyr" / "targets" / f"{preset_name}.conf"
        if not self.require_file(path):
            return
        text = path.read_text(encoding="utf-8")
        config_lines = {line.strip() for line in text.splitlines()}
        selected_symbol = "CONFIG_ARBATOS_TARGET_" + project.replace("-", "_")
        enabled_targets = sorted(line.split("=", 1)[0] for line in config_lines
                                 if re.fullmatch(r"CONFIG_ARBATOS_TARGET_[A-Z0-9_]+=y", line))
        if enabled_targets != [selected_symbol]:
            self.error(f"{self.rel(path)}: must enable exactly {selected_symbol}")
        if "CONFIG_ARBATOS_LEGACY_SOURCES=y" not in config_lines:
            self.error(f"{self.rel(path)}: formal target must enable CONFIG_ARBATOS_LEGACY_SOURCES")
        for symbol in FORMAL_MODE_SYMBOLS:
            if f"{symbol}=y" in config_lines:
                self.error(f"{self.rel(path)}: formal target enables dedicated mode {symbol}")

    def check_source_graph(self) -> None:
        cmake = self.root / "zephyr" / "CMakeLists.txt"
        manifest = self.root / "zephyr" / "cmake" / "ArbatosLegacy.cmake"
        required = (cmake, manifest, self.root / "zephyr" / "src" / "main.c",
                    self.root / "zephyr" / "src" / "ArbatosTarget.c", self.root / "zephyr" / "Kconfig")
        if not all(self.require_file(path) for path in required):
            return
        text = cmake.read_text(encoding="utf-8") + "\n" + manifest.read_text(encoding="utf-8")
        for token in sorted(set(SOURCE_TOKEN.findall(text))):
            normalized = token.replace("\\", "/")
            if FORBIDDEN_SOURCE.search(normalized):
                self.error(f"Zephyr source graph includes forbidden legacy entry: {normalized}")
                continue
            candidate = self.root / normalized if normalized.startswith(("shared/", "Robotconfig/", "boards/")) else self.root / "zephyr" / normalized
            if not candidate.is_file():
                self.error(f"Zephyr source graph references missing source: {normalized}")
        if "src/main.c" not in text or "src/ArbatosTarget.c" not in text:
            self.error("zephyr/CMakeLists.txt: formal application entry sources are incomplete")

    def run(self, selected: list[str]) -> None:
        self.check_source_graph()
        self.check_presets(selected)
        for project in selected:
            self.check_target_config(project)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Check ARBATOS formal Zephyr source and preset consistency.")
    parser.add_argument("--project", default="all", help="all (default) or a formal project name, e.g. HERO-M")
    parser.add_argument("--json", action="store_true", help="emit one JSON result object")
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1], help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    requested = args.project.upper()
    if requested == "ALL":
        selected = list(PROJECTS)
    elif requested in PROJECTS:
        selected = [requested]
    else:
        parser.error("--project must be all or one of: " + ", ".join(PROJECTS))

    check = Check(args.root.resolve())
    check.run(selected)
    result = {"ok": not check.errors, "project": args.project, "checked_projects": selected,
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
