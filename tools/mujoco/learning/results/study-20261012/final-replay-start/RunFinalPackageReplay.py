"""在已安装完整依赖的独立Python环境内，逐份实际重放最终12份策略。"""
from datetime import datetime
import hashlib
import importlib.metadata
import json
from pathlib import Path
import subprocess
import sys

workspace = Path(__file__).resolve().parents[1]
package = workspace / "work/final-package"
read = lambda p: json.loads(p.read_text(encoding="utf-8"))
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def require(ok, message):
    if not ok:
        raise RuntimeError(message)


expected_prefix = workspace / "work/package-clean-env"
require(Path(sys.prefix).resolve() == expected_prefix.resolve() and sys.prefix != sys.base_prefix,
    "必须使用已核验的独立Python环境")
require(not (package / "复现输出").exists() and not (package / "实际执行环境.json").exists(), "此目录已经执行过")
inventory = read(package / "交付清单.json")
require(inventory["include_trace"] and {r["label"] for r in inventory["policies"]} ==
    {f"{route}-{seed}" for route in "ABCDEF" for seed in (41, 73)}, "最终策略集合不符")
versions = dict(line.split("==") for line in (package / "requirements.txt").read_text(encoding="utf-8").splitlines() if line.strip())
require(len(versions) == 35, "完整依赖数量不符")
for name, version in versions.items():
    distribution = importlib.metadata.distribution(name)
    require(distribution.version == version and Path(distribution.locate_file("")).resolve().is_relative_to(expected_prefix),
        "依赖版本或实际来源不符：" + name)
started = datetime.now().astimezone().isoformat()
command = [sys.executable, "-u", str(package / "检查并复现.py")]
result = subprocess.run(command, check=False)
record = dict(schema=1, started_at=started, completed_at=datetime.now().astimezone().isoformat(),
    exit_code=result.returncode, python=sys.version, prefix=sys.prefix, base_prefix=sys.base_prefix,
    command=command, packages=versions, inventory_sha256=sha(package / "交付清单.json"),
    source_sha256=sha(Path(__file__)), same_computer=True, different_python_environment=True)
(package / "实际执行环境.json").write_bytes((json.dumps(record, ensure_ascii=False, indent=2) + "\n").encode("utf-8"))
require(result.returncode == 0, "实际重放未通过，完整记录已保留")
proof = read(package / "复现输出/汇总.json")
require(proof["trajectories"] == 72 and proof["all_fields_exact"] and proof["status"] == "passed", "完整重放范围不符")
print(json.dumps(dict(trajectories=72, independent_python_environment=True,
    all_fields_exact=True, environment_proof_sha256=sha(package / "实际执行环境.json")), ensure_ascii=False))
