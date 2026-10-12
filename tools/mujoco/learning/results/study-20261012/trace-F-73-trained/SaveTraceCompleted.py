"""保存固定筛选预算完成时的实际网络、优化器及原始训练记录。"""
import argparse
from datetime import datetime
import hashlib
import io
import json
from pathlib import Path
import shutil
import sys
import zipfile

workspace = Path(__file__).resolve().parents[1]
repo = Path(r"D:\ARBATOS")
base = workspace / "work/study-20261012/long-trace"
frozen = base / "frozen"
sys.path.insert(0, str(frozen / "tools/mujoco/learning"))
import numpy as np
import torch
from stable_baselines3 import PPO
from StudyWbrPolicy import StudyPolicy
from TrainWbrAligned import state_digest
from WbrLearningEnv import verify_snapshot, experiment_identity
from RunWbrStudyQueue import make_job_binding, is_complete
from WbrFinalModelChecks import verify_final_training

read = lambda path: json.loads(path.read_text(encoding="utf-8"))
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()


def require(value, message):
    if not value:
        raise RuntimeError(message)


def save(path, value):
    path.write_bytes((json.dumps(value, ensure_ascii=False, indent=2) + "\n").encode("utf-8"))


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--seed", required=True, type=int, choices=(41, 73))
parser.add_argument("--check-only", action="store_true")
args = parser.parse_args()
seed = args.seed
manifest = verify_snapshot(frozen)
relative_rule = "tools/mujoco/learning/results/study-20261012/长程反馈单变量训练登记.json"
rule = read(frozen / relative_rule)
queue = read(base / "training/queue-state.json")
job = queue["jobs"][f"F-trace-{seed}-train"]
require(job["status"] == "completed", "固定预算训练尚未完成")
source = Path(job["actual_output"])
attempt = job["attempts"][-1]
require(source.resolve() == Path(attempt["actual_output"]).resolve() and attempt["exit_code"] == 0, "训练不属于最后一次成功尝试")
plan = read(base / "training/plan.json")
planned_job = next(item for item in plan["jobs"] if item["id"] == f"F-trace-{seed}-train")
binding = make_job_binding("aligned_train", planned_job["arguments"], plan)
require(binding == job["job_binding"], "原计划训练绑定改变")
payload = binding["payload"]
require(is_complete("aligned_train", source, planned_job["arguments"], payload["source"], payload["script_sha256"], payload["source_hashes"]), "队列实际产物不完整")
expected_source = base / ("training/73/train-resume-02" if seed == 73 else "training/41/train")
require(source.resolve() == expected_source.resolve(), "本次固定训练来源不符")
require(len(job["attempts"]) == (2 if seed == 73 else 1), "本次尝试数量不符")
require(bool(attempt["restart_without_checkpoint"]) == (seed == 73), "从零重跑来源不符")
policy, run, status = (read(source / name) for name in ("policy.json", "run.json", "status.json"))
require(status["status"] == "completed" and status["steps"] == policy["training_steps"] == rule["training_steps"][0] == 262144, "预算完成状态不符")
require(policy["seed"] == seed and policy["envs"] == rule["envs"] == 2, "种子或环境数不符")
require(policy["training_lineage"] == dict(stage="screen_first", seed=seed, promotion_sha256=None), "训练阶段不符")
require(all(policy[key] == value for key, value in run.items()), "最终模型和运行元信息不符")
require(policy["requested_total_steps"] == 262144 and policy["strict_training_checks"], "原预算或物理检查不符")
require(policy["library_sha256"] == manifest["library_sha256"] and policy["experiment_identity"] == experiment_identity(manifest), "冻结物理身份不符")
protocol = policy["training_protocol"]
require(protocol["registration_sha256"] == digest(frozen / relative_rule), "登记来源不符")
require(protocol["weights"] == rule["weights"] and protocol["algorithm"] == rule["algorithm"], "评分或算法设置改变")
require(all(value == manifest["files"]["tools/mujoco/learning/" + name] for name, value in protocol["sources"].items()), "训练代码来源不符")
lock = read(frozen / rule["training_bank_lock"])
require(protocol["bank_sha256"] == lock["bank_sha256"], "训练库来源不符")
names = ("checkpoint.zip", "policy.npz", "policy.json", "run.json", "status.json", "aligned-episodes.jsonl", "progress.jsonl")
hashes = {name: digest(source / name) for name in names}
require(hashes["checkpoint.zip"] == policy["checkpoint_sha256"] and hashes["policy.npz"] == policy["policy_sha256"], "实际模型文件哈希不符")
torch.set_num_threads(1)
with zipfile.ZipFile(source / "checkpoint.zip") as archive:
    require(archive.testzip() is None, "模型压缩包损坏")
    network = torch.load(io.BytesIO(archive.read("policy.pth")), map_location="cpu", weights_only=True)
    optimizer = torch.load(io.BytesIO(archive.read("policy.optimizer.pth")), map_location="cpu", weights_only=True)
model = PPO.load(source / "checkpoint.zip", device="cpu")
completion = verify_final_training(model, policy, run, status, rule["algorithm"])
require(model.gae_lambda == model.rollout_buffer.gae_lambda == rule["algorithm"]["gae_lambda"] == 1. and model.gamma == .999, "实际算法参数不符")
require(model.num_timesteps == 262144 and len(optimizer["state"]) == 13, "模型实际步数或优化器状态缺失")
require(state_digest(model.policy.state_dict()) == state_digest(network), "网络实际载入不一致")
require(state_digest(model.policy.optimizer.state_dict()) == state_digest(optimizer), "优化器实际载入不一致")
observations = np.random.default_rng(749).normal(0, .5, (128, 93)).astype(np.float32)
expected, _ = model.predict(observations, deterministic=True)
exported = StudyPolicy(source)
actual = np.asarray([exported.predict(row) for row in observations])
error = float(np.max(np.abs(actual - expected)))
require(np.isfinite(actual).all() and error < 2e-5, "导出数值不符")
require(np.all(actual[:, 0] >= 0) and np.all(actual <= 1) and np.all(actual[:, 1:] >= -1), "导出动作边界不符")
require(hashes == {name: digest(source / name) for name in names}, "核验期间最终产物改变")
if args.check_only:
    print(json.dumps(dict(seed=seed, source=str(source), completion=completion,
        export_max_abs_error=error, read_only=True), ensure_ascii=False))
    sys.exit(0)
target = repo / f"tools/mujoco/learning/results/study-20261012/trace-F-{seed}-trained"
target.mkdir(exist_ok=False)
for name in names:
    shutil.copyfile(source / name, target / name)
    require(digest(target / name) == hashes[name], "保存后产物不符")
record = dict(schema=1, checked_at=datetime.now().astimezone().isoformat(), status="passed", seed=seed,
    training_steps=262144, completed_training_episodes=policy["segment_episodes"], training_physical_failures=policy["segment_failures"],
    elapsed_training_seconds=policy["segment_seconds"], actual_model_steps=model.num_timesteps, actual_gae_lambda=model.gae_lambda,
    network_load_exact=True, optimizer_load_exact=True, optimizer_states=len(optimizer["state"]),
    network_state_sha256=state_digest(network), optimizer_state_sha256=state_digest(optimizer),
    export_inputs=128, export_max_abs_error=error, files=hashes, source=str(source),
    source_script_sha256=digest(Path(__file__)), frozen_manifest_sha256=digest(frozen / "manifest.json"),
    completion=completion, completion_checker_sha256=digest(workspace / "work/WbrFinalModelChecks.py"),
    frozen_environment_archive="tools/mujoco/learning/results/study-20261012/阶段恢复包-F首段.zip",
    independent_performance_verified=False,
    boundary="训练完成及实际模型载入核验，不代表控制改善。F仅登记两种子262144步；四个E/F最终模型全部锁定后，才在新24例对照，不授权扩大训练。")
save(target / "训练完成核验.json", record)
for helper in ("SaveTraceCompleted.py", "WbrFinalModelChecks.py"):
    shutil.copyfile(workspace / "work" / helper, target / helper)
print(json.dumps(dict(target=str(target), seed=seed, steps=model.num_timesteps, episodes=policy["segment_episodes"],
    physical_failures=policy["segment_failures"], export_max_abs_error=error, checkpoint_sha256=hashes["checkpoint.zip"]), ensure_ascii=False))
