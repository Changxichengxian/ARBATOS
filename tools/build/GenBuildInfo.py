#!/usr/bin/env python3
"""生成稳定的固件版本头；内容未变时保留文件时间戳。"""

from __future__ import annotations

import argparse
import hashlib
import re
import shutil
import subprocess
import sys
from datetime import datetime
from pathlib import Path


def git_text(repo: Path, *args: str) -> str | None:
    git = shutil.which("git")
    if git is None:
        return None
    try:
        result = subprocess.run(
            [git, "-C", str(repo), *args],
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            encoding="utf-8",
            errors="replace",
        )
    except OSError:
        return None
    if result.returncode != 0:
        return None
    return result.stdout.strip()


def git_bytes(repo: Path, *args: str) -> bytes | None:
    git = shutil.which("git")
    if git is None:
        return None
    try:
        result = subprocess.run(
            [git, "-C", str(repo), *args],
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
        )
    except OSError:
        return None
    return result.stdout if result.returncode == 0 else None


def c_string(value: str) -> str:
    escaped = value.replace("\\", "\\\\").replace('"', '\\"').replace("\n", "\\n")
    return escaped.encode("ascii", errors="backslashreplace").decode("ascii")


SOURCE_SUFFIXES = {
    ".c", ".cc", ".cpp", ".cxx", ".h", ".hpp", ".py", ".ps1", ".cmake", ".conf",
    ".inc", ".kconfig", ".s", ".cmd", ".yml", ".yaml", ".json", ".toml", ".dts", ".dtsi", ".overlay",
    ".txt", ".md",
}


def source_untracked(repo: Path) -> list[Path] | None:
    output = git_bytes(repo, "ls-files", "--others", "--exclude-standard", "-z")
    if output is None:
        return None
    paths = []
    for item in output.split(b"\0"):
        if not item:
            continue
        path = Path(item.decode("utf-8", errors="surrogateescape"))
        if path.name in {".gitmodules", "Kconfig", "Makefile"} or path.suffix.lower() in SOURCE_SUFFIXES:
            paths.append(path)
    return sorted(paths)


def submodule_paths(repo: Path) -> list[Path] | None:
    if not (repo / ".gitmodules").is_file():
        return []
    output = git_bytes(repo, "config", "-z", "-f", ".gitmodules", "--get-regexp", r"^submodule\..*\.path$")
    if output is None:
        return None
    paths = []
    for item in output.split(b"\0"):
        if not item:
            continue
        _, value = item.split(b"\n", 1)
        paths.append(Path(value.decode("utf-8", errors="surrogateescape")))
    return sorted(paths)


def append_repo_source_state(digest, repo: Path, label: str, visited: set[Path]) -> bool | None:
    resolved = repo.resolve()
    if resolved in visited:
        return False
    visited.add(resolved)
    diff = git_bytes(repo, "diff", "--binary", "HEAD", "--")
    untracked = source_untracked(repo)
    if diff is None or untracked is None:
        return None
    dirty = bool(diff or untracked)
    digest.update(b"REPOSITORY\0" + label.encode("utf-8", errors="surrogateescape") + b"\0")
    digest.update(b"DIFF\0" + diff + b"\0")
    for path in untracked:
        candidate = repo / path
        if not candidate.is_file():
            continue
        digest.update(b"UNTRACKED\0" + str(path).encode("utf-8", errors="surrogateescape") + b"\0")
        digest.update(candidate.read_bytes())
        digest.update(b"\0")
    children = submodule_paths(repo)
    if children is None:
        return None
    for path in children:
        child = repo / path
        if child.is_dir():
            child_dirty = append_repo_source_state(digest, child, f"{label}/{path.as_posix()}", visited)
            if child_dirty is None:
                return None
            dirty = dirty or child_dirty
    return dirty


def source_state(repo: Path, sha: str, submodules: str) -> tuple[int, str]:
    digest = hashlib.sha256()
    digest.update(b"HEAD\0" + sha.encode("ascii", errors="replace") + b"\0")
    dirty = append_repo_source_state(digest, repo, ".", set())
    if dirty is None:
        return 0, "unknown"
    submodule_hash = hashlib.sha256(submodules.encode("utf-8")).hexdigest()
    if not dirty:
        return 0, f"clean:{sha}:{submodule_hash}"
    return 1, f"dirty:{digest.hexdigest()}"


def build_values(repo: Path) -> tuple[str, int, str, str]:
    if git_text(repo, "rev-parse", "--is-inside-work-tree") != "true":
        return "unknown", 0, "unknown", "unknown"

    sha = git_text(repo, "rev-parse", "--short=16", "HEAD") or "unknown"
    submodules = git_text(repo, "submodule", "status", "--recursive")
    dirty, state = source_state(repo, sha, submodules or "")
    return sha, dirty, submodules or "", state


def previous_value(content: str, macro: str) -> str | None:
    match = re.search(rf'^#define {re.escape(macro)} "(.*)"$', content, flags=re.MULTILINE)
    return match.group(1) if match else None


def render(repo: Path, previous: str) -> str:
    sha, dirty, submodules, state = build_values(repo)
    if previous_value(previous, "ARBATOS_SOURCE_STATE") == c_string(state):
        build_date = previous_value(previous, "ARBATOS_BUILD_DATE") or "unknown"
        build_time = previous_value(previous, "ARBATOS_BUILD_TIME") or "unknown"
    else:
        # 使用本地生成时刻；旧日志字段只有 9 字节，时间必须保持 HH:MM:SS。
        now = datetime.now()
        build_date = now.strftime("%Y-%m-%d")
        build_time = now.strftime("%H:%M:%S")
    return (
        "#ifndef ARBATOS_BUILD_INFO_AUTOGEN_H\n"
        "#define ARBATOS_BUILD_INFO_AUTOGEN_H\n\n"
        f'#define ARBATOS_GIT_SHA "{c_string(sha)}"\n'
        f"#define ARBATOS_BUILD_DIRTY {dirty}u\n"
        f'#define ARBATOS_BUILD_DATE "{c_string(build_date)}"\n'
        f'#define ARBATOS_BUILD_TIME "{c_string(build_time)}"\n'
        f'#define ARBATOS_GIT_SUBMODULES "{c_string(submodules)}"\n\n'
        f'#define ARBATOS_SOURCE_STATE "{c_string(state)}"\n\n'
        "#endif\n"
    )


def write_if_changed(output: Path, content: str) -> bool:
    try:
        if output.read_text(encoding="ascii") == content:
            return False
    except FileNotFoundError:
        pass
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(content, encoding="ascii", newline="\n")
    return True


def main() -> int:
    sys.stdout.reconfigure(encoding="utf-8", errors="backslashreplace")
    parser = argparse.ArgumentParser(description="生成 ARBATOS 构建版本头")
    parser.add_argument("--repo", type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    output = args.output if args.output.is_absolute() else Path.cwd() / args.output
    try:
        previous = output.read_text(encoding="ascii")
    except FileNotFoundError:
        previous = ""
    changed = write_if_changed(output, render(args.repo.resolve(), previous))
    print(f"{'generated' if changed else 'unchanged'} {output.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
