"""使用已有真实模型验证训练完成边界；不训练、不评价控制性能。"""
from datetime import datetime
import hashlib
import json
from pathlib import Path
import subprocess
import sys

workspace = Path(__file__).resolve().parents[1]
study = workspace / "work/study-20261012"
base = study / "long-trace"
sys.path.insert(0, str(base / "frozen/tools/mujoco/learning"))
from stable_baselines3 import PPO
import torch
from WbrFinalModelChecks import verify_final_training

read = lambda p: json.loads(p.read_text(encoding="utf-8"))
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()

def require(ok, message):
    if not ok:
        raise RuntimeError(message)

torch.set_num_threads(1)
completed = []
for name, path in (("E-41", study / "aligned-reference/screening/41/train"),
                   ("E-73", study / "aligned-reference/screening/73/train"),
                   ("F-41", base / "training/41/train")):
    policy, run, status = (read(path / n) for n in ("policy.json", "run.json", "status.json"))
    model = PPO.load(path / "checkpoint.zip", device="cpu")
    result = verify_final_training(model, policy, run, status, policy["training_protocol"]["algorithm"])
    completed.append(dict(name=name, checkpoint_sha256=sha(path / "checkpoint.zip"), result=result))
path = base / "training/41/train"
callback_path = path / "checkpoints/step-000262144"
callback_model = PPO.load(callback_path / "checkpoint.zip", device="cpu")
require(callback_model.num_timesteps == 262144 and callback_model._n_updates == 762,
        "实际最后采样回调检查点不是预期输入")
policy, run, status = (read(path / n) for n in ("policy.json", "run.json", "status.json"))
try:
    verify_final_training(callback_model, policy, run, status, policy["training_protocol"]["algorithm"])
except RuntimeError as error:
    rejection = str(error)
    require("最后一批" in rejection, "未由真实循环进度拒绝")
else:
    raise RuntimeError("仅采样计数完成的模型被误接纳")
stop_status = {**status, "stop_reason": "stop_file"}
try:
    verify_final_training(model, policy, run, stop_status, policy["training_protocol"]["algorithm"])
except RuntimeError as error:
    require("停止请求" in str(error), "未检查主动中断")
else:
    raise RuntimeError("主动中断被误接纳")
saver = subprocess.run([sys.executable, str(workspace / "work/SaveTraceCompleted.py"), "--seed", "41", "--check-only"],
                       stdout=subprocess.PIPE, stderr=subprocess.PIPE, encoding="utf-8")
require(saver.returncode == 0 and json.loads(saver.stdout)["read_only"], "原F41实际保存检查失败："+saver.stderr)
require(not (base / "四策略锁定.json").exists() and not (base / "evaluation/plan.json").exists(), "已有评价计划，拒绝运行前置拒绝测试")
locking = subprocess.run([sys.executable, str(workspace / "work/LockTraceComparison.py")],
                         stdout=subprocess.PIPE, stderr=subprocess.PIPE, encoding="utf-8")
require(locking.returncode != 0 and "所有原定任务尚未完成" in locking.stderr, "未完成四模型没有被拒绝："+locking.stderr)
require(not (base / "四策略锁定.json").exists() and not (base / "evaluation/plan.json").exists(), "拒绝后仍创建了评价计划")
record = dict(schema=1, checked_at=datetime.now().astimezone().isoformat(), completed_models=completed,
    actual_final_sampling_checkpoint=dict(path=str(callback_path), checkpoint_sha256=sha(callback_path / "checkpoint.zip"),
        sampled_steps=262144, epoch_counter=762, progress_remaining=callback_model._current_progress_remaining, rejected=rejection),
    stopped_status_rejected=True, existing_F41_saver_check_passed=True, premature_four_model_lock_rejected=True,
    evaluation_plan_created=False,
    helpers={name:sha(workspace / "work" / name) for name in ("WbrFinalModelChecks.py", "SaveTraceCompleted.py", "LockTraceComparison.py", "StageTraceCompletedSeed.py", "CheckFinalTrainingBudget.py")},
    boundary="三个已完成实际模型均自然完成且计数768；这不是强制未来每批优化满6轮。原KL阈值允许较少轮次。F73完整最终模型尚未产生，本次未评价任何F控制性能。")
(base / "训练完成边界补检.json").write_text(json.dumps(record, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
print(json.dumps(record, ensure_ascii=False))
