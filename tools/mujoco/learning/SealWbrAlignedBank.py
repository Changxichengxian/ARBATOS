"""核验原控制逐例记录，保存基线库及来源，供第二次冻结后训练。"""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import shutil

from BuildWbrAlignedBank import build
from EvaluateWbrStudy import _completed_path
from WbrLearningEnv import ROOT, verify_snapshot


def seal(cases, baseline, registration, destination_root):
    read = lambda p: json.loads(p.read_text(encoding="utf-8"))
    digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    manifest = verify_snapshot()
    registered = "tools/mujoco/learning/results/study-20261012/对齐评分参考策略训练登记.json"
    if digest(registration) != manifest["files"].get(registered):
        raise RuntimeError("登记未随代码冻结")
    rule = read(registration)
    bank = build(cases, baseline)
    if bank["cases_sha256"] != rule["training_cases_sha256"]:
        raise RuntimeError("训练场景未登记")
    summary, run = read(baseline), read(baseline.parent / "run.json")
    completed_hashes = {}
    for row in summary["baseline_results"]:
        path = _completed_path(baseline.parent, "baseline", row["id"])
        item = read(path)
        if item.get("binding_sha256") != run["binding_sha256"] or item.get("row") != row:
            raise RuntimeError("基线汇总与实际逐例结果不一致："+row["id"])
        completed_hashes[row["id"]] = digest(path)
    destination_root = destination_root.resolve()
    lock_path = (destination_root / rule["training_bank_lock"]).resolve()
    lock_path.relative_to(destination_root)
    output = lock_path.parent
    if output.exists():
        raise FileExistsError("基线保存目录已存在，拒绝覆盖")
    output.mkdir(parents=True)
    bank_path = output / "training-bank.json"
    bank_path.write_bytes((json.dumps(bank,ensure_ascii=False,indent=2)+"\n").encode("utf-8"))
    shutil.copy2(baseline, output / "baseline-summary.json")
    shutil.copy2(baseline.parent / "run.json", output / "baseline-run.json")
    source_files = {path.relative_to(destination_root).as_posix():digest(path)
                    for path in (bank_path, output / "baseline-summary.json", output / "baseline-run.json")}
    result = dict(schema=1, verified_at=datetime.now().astimezone().isoformat(),
        registration_sha256=digest(registration), bank_sha256=digest(bank_path),
        source_files=source_files, completed_row_hashes=completed_hashes,
        source_snapshot=str(ROOT), source_manifest_sha256=digest(ROOT / "manifest.json"),
        library_sha256=bank["library_sha256"], experiment_identity=bank["experiment_identity"],
        cases_sha256=bank["cases_sha256"], count=len(bank["rows"]),
        note="从冻结评价器的原控制完整汇总重建，并与每条完成记录逐一核对；必须连同来源再次冻结才准训练。")
    lock_path.write_bytes((json.dumps(result,ensure_ascii=False,indent=2)+"\n").encode("utf-8"))
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("cases", "baseline", "registration", "destination-root"):
        parser.add_argument("--"+name, type=Path, required=True)
    args = parser.parse_args()
    result = seal(args.cases, args.baseline, args.registration, args.destination_root)
    print(json.dumps(dict(count=result["count"],bank_sha256=result["bank_sha256"])))
