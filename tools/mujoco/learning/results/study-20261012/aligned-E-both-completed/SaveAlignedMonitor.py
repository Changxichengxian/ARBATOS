"""核验一个筛选种子的完整24例监控；不生成扩大训练许可。"""
import argparse
from collections import Counter
from datetime import datetime
import hashlib
import json
from pathlib import Path
import shutil
import sys

workspace = Path(__file__).resolve().parents[1]
repo = Path(r"D:\ARBATOS")
base = workspace / "work/study-20261012/aligned-reference"
frozen = base / "training-frozen"
sys.path.insert(0, str(frozen / "tools/mujoco/learning"))
from BuildWbrStudyCases import canonical_bytes
from EvaluateWbrStudy import acceptance, motion_quality_acceptance, load_cases, load_baseline_cache, policy_artifacts, _completed_path
from PromoteWbrAligned import gate
from WbrLearningEnv import verify_snapshot, experiment_identity

read = lambda path: json.loads(path.read_text(encoding="utf-8"))
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()


def require(value, message):
    if not value:
        raise RuntimeError(message)


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--seed", type=int, required=True, choices=(41, 73))
args = parser.parse_args()
seed = args.seed
manifest = verify_snapshot(frozen)
identity = experiment_identity(manifest)
rule_path = frozen / "tools/mujoco/learning/results/study-20261012/对齐评分参考策略训练登记.json"
rule = read(rule_path)
cases = load_cases(frozen / "tools/mujoco/learning/cases/aligned-20261012/monitor.json")
require(cases["cases_sha256"] == rule["monitor_cases_sha256"] and cases["case_count"] == 24, "监控场景来源或数量不符")
queue = read(base / "screening/queue-state.json")
job = queue["jobs"][f"E-aligned-{seed}-monitor"]
require(job["status"] == "completed", "完整监控评价尚未结束")
evaluation = Path(job["actual_output"])
require(evaluation.resolve() == (base / f"screening/{seed}/monitor").resolve(), "评价输出目录不符")
training = base / f"screening/{seed}/train"
saved_training = repo / f"tools/mujoco/learning/results/study-20261012/aligned-E-{seed}-trained"
training_proof = read(saved_training / "训练完成核验.json")
require(training_proof["status"] == "passed" and training_proof["network_load_exact"] and training_proof["optimizer_load_exact"], "实际模型核验未通过")
require(all(digest(training / name) == digest(saved_training / name) == value for name, value in training_proof["files"].items()), "已核验最终模型或训练记录发生变化")
policy, status = read(training / "policy.json"), read(training / "status.json")
require(policy["seed"] == seed and status["status"] == "completed" and status["steps"] == policy["training_steps"] == rule["screening_first"]["steps"], "监控对象不是完成预算的最终模型")
require(policy["training_protocol"]["registration_sha256"] == digest(rule_path), "训练登记不符")
report, run = read(evaluation / "summary.json"), read(evaluation / "run.json")
binding = run["binding"]
require(run["binding_sha256"] == hashlib.sha256(canonical_bytes(binding)).hexdigest(), "评价运行绑定损坏")
require(report["run_binding_sha256"] == run["binding_sha256"], "汇总绑定不符")
for key, value in dict(split="aligned-monitor", scenarios_sha256=cases["cases_sha256"],
    library_sha256=manifest["library_sha256"], experiment_identity=identity, env_config=policy["env_config"]).items():
    require(binding[key] == report[key] == value, "评价来源不符：" + key)
require(binding["evaluator_sha256"] == digest(frozen / "tools/mujoco/learning/EvaluateWbrStudy.py"), "评价器源码不符")
require(Path(binding["policy_directory"]).resolve() == training.resolve() and binding["policy_artifacts"] == policy_artifacts(training), "实际评价权重被替换")
baseline_path = base / "baselines/monitor/summary.json"
require(binding["baseline_cache_sha256"] == digest(baseline_path), "原控制缓存来源不符")
baseline = load_baseline_cache(baseline_path, cases, manifest, identity, policy["env_config"])
require(report["baseline_results"] == baseline, "原控制配对结果改变")
for controller in ("baseline", "learned"):
    rows = report[controller + "_results"]
    by_id = {row["id"]: row for row in rows}
    require(len(rows) == len(by_id) == 24, "场景缺失或重复")
    for scenario in cases["cases"]:
        row = by_id[scenario["id"]]
        require(all(row[key] == scenario[key] for key in ("id", "seed", "randomize", "options"))
            and row["controller"] == controller and row["case"] == scenario["options"]["case"], "评价场景错配")
        completed = read(_completed_path(evaluation, controller, scenario["id"]))
        require(completed["binding_sha256"] == run["binding_sha256"] and completed["row"] == row, "实际完成记录和汇总不一致")
recomputed = acceptance(baseline, report["learned_results"])
quality = motion_quality_acceptance(baseline, report["learned_results"])
recomputed["checks"]["motion_quality"] = quality
recomputed["overall_pass"] = bool(recomputed["overall_pass"] and quality["pass"])
require(recomputed == report["acceptance"], "原指标与运动质量不能完整复算")
screen = gate(recomputed, rule["screening_first"])
target = repo / f"tools/mujoco/learning/results/study-20261012/aligned-E-{seed}-monitor"
target.mkdir(exist_ok=False)
for name in ("summary.json", "run.json"):
    shutil.copyfile(evaluation / name, target / name)
    require(digest(evaluation / name) == digest(target / name), "监控结果复制后不符")
record = dict(schema=1, checked_at=datetime.now().astimezone().isoformat(), route="E-aligned-reference", seed=seed,
    training_steps=policy["training_steps"], monitor_pairs=24, criteria=rule["screening_first"], single_seed_gate=screen,
    expansion_authorized=False, both_seeds_required=True, final_dataset_used=False,
    failed_original_cases=[row for row in recomputed["cases"] if not row["pass"]],
    failed_motion_cases=[row for row in quality["cases"] if not row["pass"]],
    original_guard_failure_counts=dict(Counter(name for row in recomputed["cases"] for name, value in row["checks"].items() if not value["pass"])),
    motion_quality_failure_counts=dict(Counter(name for row in quality["cases"] for name, value in row["checks"].items() if not value["pass"])),
    verification=dict(all_24_pairs=True, all_48_completed_rows_match=True, final_policy_exact=True, acceptance_and_quality_recomputed=True),
    sources=dict(summary_sha256=digest(target / "summary.json"), run_sha256=digest(target / "run.json"),
        checkpoint_sha256=digest(training / "checkpoint.zip"), training_proof_sha256=digest(saved_training / "训练完成核验.json"),
        registration_sha256=digest(rule_path), verifier_sha256=digest(Path(__file__))),
    boundary="这是单种子独立24例监控，不能与45例验证或60例最终验收混合。只有两种子均通过登记条件、在截止时间内且正式晋级复算通过，才可扩大训练。本记录不是扩大训练许可。")
(target / "单种子监控核验.json").write_bytes((json.dumps(record, ensure_ascii=False, indent=2) + "\n").encode("utf-8"))
print(json.dumps(dict(seed=seed, gate=screen, original_guard_failure_counts=record["original_guard_failure_counts"],
    motion_quality_failure_counts=record["motion_quality_failure_counts"]), ensure_ascii=False, indent=2))
