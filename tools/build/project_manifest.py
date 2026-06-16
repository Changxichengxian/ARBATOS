#!/usr/bin/env python3
"""Extract build-facing project metadata from Keil uVision projects.

The Keil project is still the source of truth today.  This tool makes that
state visible to scripts and CI without copying source lists into another
build system by hand.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Iterable


REPO_ROOT = Path(__file__).resolve().parents[2]
PROJECTS_ROOT = REPO_ROOT / "projects"

SOURCE_EXTS = {".c", ".cc", ".cpp", ".cxx", ".s", ".asm"}
LIB_EXTS = {".a", ".lib"}
GCC_GENERATOR = REPO_ROOT / "tools" / "build" / "gcc_project.py"
GCC_SUPPORT_ROOT = REPO_ROOT / "tools" / "build" / "gcc_support"
GCC_SUPPORTED_ARMCC_LIBS = {
    "ahrs.lib": "GCC AHRS source",
    "arm_cortexm4lf_math.lib": "GCC math fallback",
}


def unique(items: Iterable[str], *, casefold: bool = False) -> list[str]:
    seen: set[str] = set()
    result: list[str] = []
    for item in items:
        key = item.casefold() if casefold else item
        if key in seen:
            continue
        seen.add(key)
        result.append(item)
    return result


def clean_uv_path(value: str) -> str:
    cleaned = value.strip().replace("\\", "/")
    while "//" in cleaned:
        cleaned = cleaned.replace("//", "/")
    return cleaned


def repo_path(path: Path) -> str:
    resolved = path.resolve(strict=False)
    try:
        return resolved.relative_to(REPO_ROOT).as_posix()
    except ValueError:
        return resolved.as_posix()


def resolve_project_path(project_dir: Path, value: str) -> Path | None:
    cleaned = clean_uv_path(value)
    if not cleaned or cleaned.startswith("$$"):
        return None

    candidate = Path(cleaned)
    if not candidate.is_absolute():
        candidate = project_dir / cleaned

    return candidate.resolve(strict=False)


def text_values(xml: ET.ElementTree, tag: str) -> list[str]:
    values: list[str] = []
    for node in xml.findall(f".//{tag}"):
        text = (node.text or "").strip()
        if text:
            values.append(text)
    return values


def first_text(xml: ET.ElementTree, tag: str) -> str | None:
    values = text_values(xml, tag)
    return values[0] if values else None


def split_defines(values: Iterable[str]) -> list[str]:
    defines: list[str] = []
    for value in values:
        for item in value.split(","):
            item = item.strip()
            if item:
                defines.append(item)
    return unique(defines)


def split_include_paths(values: Iterable[str]) -> list[str]:
    paths: list[str] = []
    for value in values:
        for item in value.split(";"):
            item = clean_uv_path(item)
            if item:
                paths.append(item)
    return unique(paths, casefold=True)


def classify_file(path_text: str) -> str:
    suffix = Path(clean_uv_path(path_text)).suffix.lower()
    if suffix in {".c"}:
        return "c"
    if suffix in {".cc", ".cpp", ".cxx"}:
        return "cpp"
    if suffix in {".s", ".asm"}:
        return "asm"
    if suffix in LIB_EXTS:
        return "library"
    if suffix in {".h", ".hpp"}:
        return "header"
    return "other"


def looks_like_armasm(path: Path) -> bool:
    if not path.exists() or path.suffix.lower() not in {".s", ".asm"}:
        return False

    try:
        text = path.read_text(encoding="utf-8", errors="ignore")[:20000]
    except OSError:
        return False

    return bool(
        re.search(r"(?m)^\s*(AREA|EXPORT|IMPORT|PRESERVE8|THUMB|END)\b", text)
    )


def discover_uvprojx() -> list[Path]:
    if not PROJECTS_ROOT.exists():
        return []
    return sorted(PROJECTS_ROOT.glob("*/MDK-ARM/*.uvprojx"))


def find_project_uvprojx(name: str) -> Path | None:
    expected = PROJECTS_ROOT / name / "MDK-ARM" / f"{name}.uvprojx"
    if expected.exists():
        return expected

    for path in discover_uvprojx():
        if path.parents[1].name.casefold() == name.casefold():
            return path
    return None


def make_file_entry(project_dir: Path, path_text: str) -> dict[str, Any]:
    cleaned = clean_uv_path(path_text)
    resolved = resolve_project_path(project_dir, cleaned)
    kind = classify_file(cleaned)
    entry: dict[str, Any] = {
        "path": cleaned,
        "kind": kind,
        "pack_path": cleaned.startswith("$$"),
    }
    if resolved is not None:
        entry["repo_path"] = repo_path(resolved)
        entry["exists"] = resolved.exists()
    return entry


def make_include_entry(project_dir: Path, path_text: str) -> dict[str, Any]:
    cleaned = clean_uv_path(path_text)
    resolved = resolve_project_path(project_dir, cleaned)
    entry: dict[str, Any] = {
        "path": cleaned,
        "external": cleaned.startswith("$$") or cleaned.startswith("$(") or cleaned.startswith("%"),
    }
    if resolved is not None:
        entry["repo_path"] = repo_path(resolved)
        entry["exists"] = resolved.is_dir()
    return entry


def make_scatter_entry(project_dir: Path, path_text: str) -> dict[str, Any]:
    cleaned = clean_uv_path(path_text)
    resolved = resolve_project_path(project_dir, cleaned)
    entry: dict[str, Any] = {"path": cleaned}
    if resolved is not None:
        entry["repo_path"] = repo_path(resolved)
        entry["exists"] = resolved.exists()
    return entry


def summarize_counts(files: list[dict[str, Any]], includes: list[dict[str, Any]]) -> dict[str, int]:
    counts = {
        "files": len(files),
        "sources": 0,
        "c_sources": 0,
        "cpp_sources": 0,
        "asm_sources": 0,
        "libraries": 0,
        "include_dirs": len(includes),
    }
    for entry in files:
        kind = entry["kind"]
        if kind in {"c", "cpp", "asm"}:
            counts["sources"] += 1
        if kind == "c":
            counts["c_sources"] += 1
        elif kind == "cpp":
            counts["cpp_sources"] += 1
        elif kind == "asm":
            counts["asm_sources"] += 1
        elif kind == "library":
            counts["libraries"] += 1
    return counts


def validate_manifest(
    files: list[dict[str, Any]],
    includes: list[dict[str, Any]],
    scatter_files: list[dict[str, Any]],
) -> dict[str, list[str]]:
    errors: list[str] = []
    warnings: list[str] = []

    for entry in files:
        if entry.get("pack_path"):
            continue
        if entry.get("exists") is False:
            errors.append(f"missing file: {entry['path']} => {entry.get('repo_path', '?')}")

    for entry in includes:
        if entry.get("external"):
            continue
        if entry.get("exists") is False:
            warnings.append(f"missing include dir: {entry['path']} => {entry.get('repo_path', '?')}")

    for entry in scatter_files:
        if entry.get("exists") is False:
            errors.append(f"missing scatter file: {entry['path']} => {entry.get('repo_path', '?')}")

    seen: set[str] = set()
    for entry in files:
        key = str(entry.get("repo_path", entry["path"])).casefold()
        if key in seen:
            warnings.append(f"duplicate file: {entry['path']}")
        seen.add(key)

    return {"errors": unique(errors), "warnings": unique(warnings)}


def analyze_gcc_status(
    project_dir: Path,
    defines: list[str],
    compiler_ids: list[str],
    files: list[dict[str, Any]],
    includes: list[dict[str, Any]],
    scatter_files: list[dict[str, Any]],
) -> dict[str, Any]:
    blockers: list[str] = []
    notes: list[str] = []
    has_gcc_generator = GCC_GENERATOR.exists()

    if any("V5." in value or "ARM Compiler 5" in value for value in compiler_ids):
        if has_gcc_generator:
            notes.append("Keil project records ARM Compiler 5; GCC uses generated separate flags.")
        else:
            blockers.append("Keil project records ARM Compiler 5; GCC needs a separate flag set.")

    if "__CC_ARM" in defines:
        if has_gcc_generator:
            notes.append("GCC generator filters the __CC_ARM define from generated builds.")
        else:
            blockers.append("Project defines __CC_ARM; compiler-specific code still selects ARMCC paths.")

    libraries = [entry for entry in files if entry["kind"] == "library"]
    unsupported_libraries = []
    for entry in libraries:
        name = Path(entry["path"]).name.lower()
        replacement = GCC_SUPPORTED_ARMCC_LIBS.get(name)
        if replacement is None:
            unsupported_libraries.append(entry)
        else:
            notes.append(f"{entry['path']} is replaced by {replacement} in generated GCC builds.")
    if unsupported_libraries:
        names = ", ".join(entry["path"] for entry in unsupported_libraries[:3])
        extra = "" if len(unsupported_libraries) <= 3 else f", +{len(unsupported_libraries) - 3} more"
        blockers.append(f"ARMCC .lib libraries are linked ({names}{extra}); GCC needs source or .a replacements.")

    for entry in files:
        if entry["kind"] != "asm" or "repo_path" not in entry:
            continue
        if looks_like_armasm(REPO_ROOT / entry["repo_path"]):
            if has_gcc_generator:
                notes.append("ARMASM startup is translated to a generated GNU startup .S file.")
            else:
                blockers.append("Startup assembly uses ARMASM syntax; GCC needs GNU-compatible startup .S files.")
            break

    if scatter_files and not list(project_dir.glob("*.ld")):
        if has_gcc_generator:
            notes.append("Keil .sct scatter file is translated to a generated GNU linker script.")
        else:
            blockers.append("Keil .sct scatter file is present; GCC needs a linker .ld script.")

    paths = [entry["path"] for entry in files] + [entry["path"] for entry in includes]
    if any("/portable/RVDS/" in path for path in paths):
        if (GCC_SUPPORT_ROOT / "freertos" / "portable" / "GCC" / "ARM_CM4F").exists():
            notes.append("FreeRTOS RVDS portable is replaced by the GCC portable layer.")
        else:
            blockers.append("FreeRTOS portable/RVDS is used; GCC needs portable/GCC/... in the project.")

    return {
        "ready": len(unique(blockers)) == 0,
        "blockers": unique(blockers),
        "notes": unique(notes),
    }


def load_manifest(uvprojx: Path) -> dict[str, Any]:
    project_dir = uvprojx.parent
    project_name = uvprojx.parents[1].name

    xml = ET.parse(uvprojx)
    target_names = unique(text_values(xml, "TargetName"))
    compiler_ids = unique(text_values(xml, "pCCUsed"))
    defines = split_defines(text_values(xml, "Define"))
    include_paths = split_include_paths(text_values(xml, "IncludePath"))
    file_paths = unique(text_values(xml, "FilePath"), casefold=True)
    scatter_paths = unique(text_values(xml, "ScatterFile") + text_values(xml, "ScatFile"), casefold=True)

    files = [make_file_entry(project_dir, value) for value in file_paths]
    includes = [make_include_entry(project_dir, value) for value in include_paths]
    scatter_files = [make_scatter_entry(project_dir, value) for value in scatter_paths]
    validation = validate_manifest(files, includes, scatter_files)
    gcc_status = analyze_gcc_status(project_dir, defines, compiler_ids, files, includes, scatter_files)

    return {
        "project": project_name,
        "uvprojx": repo_path(uvprojx),
        "target_names": target_names,
        "device": first_text(xml, "Device"),
        "cpu": first_text(xml, "Cpu"),
        "compiler": {
            "pCCUsed": compiler_ids,
            "uAC6": unique(text_values(xml, "uAC6")),
        },
        "defines": defines,
        "include_dirs": includes,
        "files": files,
        "scatter_files": scatter_files,
        "counts": summarize_counts(files, includes),
        "validation": validation,
        "gcc": gcc_status,
    }


def select_projects(args: argparse.Namespace) -> list[Path]:
    if args.project:
        uvprojx = find_project_uvprojx(args.project)
        if uvprojx is None:
            raise SystemExit(f"project not found: {args.project}")
        return [uvprojx]

    return discover_uvprojx()


def make_report(manifests: list[dict[str, Any]]) -> dict[str, Any]:
    return {
        "repo_root": str(REPO_ROOT),
        "projects": manifests,
        "summary": {
            "project_count": len(manifests),
            "validation_errors": sum(len(item["validation"]["errors"]) for item in manifests),
            "validation_warnings": sum(len(item["validation"]["warnings"]) for item in manifests),
            "gcc_ready_count": sum(1 for item in manifests if item["gcc"]["ready"]),
            "gcc_blocker_count": sum(len(item["gcc"]["blockers"]) for item in manifests),
        },
    }


def make_summary_report(report: dict[str, Any]) -> dict[str, Any]:
    projects: list[dict[str, Any]] = []
    for item in report["projects"]:
        projects.append(
            {
                "project": item["project"],
                "uvprojx": item["uvprojx"],
                "device": item.get("device"),
                "counts": item["counts"],
                "validation": item["validation"],
                "gcc": item["gcc"],
            }
        )

    return {
        "repo_root": report["repo_root"],
        "projects": projects,
        "summary": report["summary"],
    }


def print_human(report: dict[str, Any]) -> None:
    print("ARBATOS project manifest")
    print(f"repo: {report['repo_root']}")
    for item in report["projects"]:
        counts = item["counts"]
        compiler = item["compiler"]["pCCUsed"][0] if item["compiler"]["pCCUsed"] else "unknown"
        print("")
        print(
            f"{item['project']}: {counts['sources']} sources "
            f"({counts['c_sources']} C, {counts['asm_sources']} asm), "
            f"{counts['include_dirs']} include dirs, {counts['libraries']} libraries"
        )
        print(f"  uvprojx: {item['uvprojx']}")
        print(f"  device: {item.get('device') or 'unknown'}")
        print(f"  compiler: {compiler}")
        if item["validation"]["errors"]:
            print(f"  validation: {len(item['validation']['errors'])} error(s)")
            for error in item["validation"]["errors"][:8]:
                print(f"    - {error}")
        elif item["validation"]["warnings"]:
            print(f"  validation: ok, {len(item['validation']['warnings'])} warning(s)")
        else:
            print("  validation: ok")

        if item["gcc"]["ready"]:
            print("  gcc: ready")
            for note in item["gcc"].get("notes", []):
                print(f"    - {note}")
        else:
            print(f"  gcc: blocked by {len(item['gcc']['blockers'])} item(s)")
            for blocker in item["gcc"]["blockers"]:
                print(f"    - {blocker}")


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project", help="Project name, for example HERO-C.")
    parser.add_argument("--all", action="store_true", help="Inspect all projects. This is the default.")
    parser.add_argument("--json", action="store_true", help="Print JSON.")
    parser.add_argument("--summary-only", action="store_true", help="With --json, omit file and include lists.")
    parser.add_argument("--check", action="store_true", help="Return non-zero on manifest validation errors.")
    parser.add_argument(
        "--fail-on-gcc-blockers",
        action="store_true",
        help="Also return non-zero when a project is not GCC-ready yet.",
    )
    args = parser.parse_args(argv)

    try:
        manifests = [load_manifest(path) for path in select_projects(args)]
    except ET.ParseError as exc:
        print(f"uvprojx XML parse failed: {exc}", file=sys.stderr)
        return 1
    except OSError as exc:
        print(f"cannot read project file: {exc}", file=sys.stderr)
        return 1

    report = make_report(manifests)
    if args.json:
        json_report = make_summary_report(report) if args.summary_only else report
        print(json.dumps(json_report, indent=2))
    else:
        print_human(report)

    if args.check and report["summary"]["validation_errors"] > 0:
        return 1
    if args.fail_on_gcc_blockers and report["summary"]["gcc_blocker_count"] > 0:
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
