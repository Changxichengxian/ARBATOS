"""冻结本次物理模型、控制器与训练代码，避免其他调参改变实验对象。"""
from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def prepare(destination, reference_snapshot=None):
    destination = Path(destination).resolve()
    if destination.exists():
        raise FileExistsError(f"快照目录已存在，拒绝覆盖：{destination}")
    base = Path(reference_snapshot).resolve() if reference_snapshot else ROOT
    if reference_snapshot:
        previous = json.loads((base/"manifest.json").read_text(encoding="utf-8"))
        for relative, expected in previous["files"].items():
            if digest(base/relative) != expected:
                raise RuntimeError(f"原快照发生变化：{relative}")
    paths = []
    for directory in ("tools/mujoco/reference",):
        paths += [p for p in (base/directory).rglob("*")
                  if p.is_file() and p.suffix in (".py", ".c", ".h", ".json", ".npz", ".txt")]
    for directory in ("shared/application/wheelleg", "shared/application/robot", "shared/components/support"):
        paths += list((base/directory).glob("*.h"))
    paths += [base/"tools/mujoco/wheelleg/RunWheelLeg.py", base/"tools/sim/RobotSim.py"]
    paths += [p for p in (base/"local/reference/wbr_mujoco/mjcf").rglob("*") if p.is_file()]
    if not paths or not (base/"local/reference/wbr_mujoco/mjcf/scene.xml").is_file():
        raise FileNotFoundError("缺少已验证的 WBR 机械模型")
    sources = {p.relative_to(base).as_posix():p for p in paths}
    sources.update({p.relative_to(ROOT).as_posix():p for p in (ROOT/"tools/mujoco/learning").rglob("*")
                    if p.is_file() and p.suffix in (".py", ".c", ".h", ".json", ".txt")})
    before = {relative:digest(path) for relative,path in sources.items()}
    for relative in before:
        target = destination/relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(sources[relative], target)
    for relative, expected in before.items():
        if digest(sources[relative]) != expected or digest(destination/relative) != expected:
            raise RuntimeError(f"冻结期间源码发生变化，请另建快照重试：{relative}")
    sys.path.insert(0, str(destination/"tools/mujoco/wheelleg"))
    from RunWheelLeg import find_compiler
    compiler = find_compiler()
    if compiler is None:
        raise RuntimeError("未找到主机 C 编译器")
    executable, tags = compiler
    build = destination/"build"
    build.mkdir()
    extension = ".dll" if sys.platform == "win32" else ".so"
    library = build/f"wbr_learning{extension}"
    source = destination/"tools/mujoco/learning/WbrLearningBridge.c"
    includes = [destination/p for p in ("shared/application/wheelleg", "shared/application/robot",
                                       "shared/components/support")]
    if "msvc" in tags:
        command = [executable, "/nologo", "/LD", "/O2"] + [f"/I{p}" for p in includes]
        command += [str(source), f"/Fe{library}"]
    else:
        command = [executable, "cc"] if "zig" in tags else [executable]
        command += ["-shared", "-O2", "-std=c99"] + [f"-I{p}" for p in includes]
        command += [str(source), "-o", str(library)]
        if sys.platform != "win32":
            command += ["-fPIC", "-lm"]
    subprocess.run(command, cwd=build, check=True)
    manifest = dict(schema=1, source_root=str(ROOT), reference_snapshot=str(base), files=before,
                    library=library.relative_to(destination).as_posix(), library_sha256=digest(library),
                    build_command=command, scope="主机平地控制实验，非固件或实车验收")
    (destination/"manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    print(json.dumps(dict(snapshot=str(destination), files=len(before), library=str(library)), ensure_ascii=False))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--reference-snapshot", type=Path)
    args = parser.parse_args()
    prepare(args.output, args.reference_snapshot)
