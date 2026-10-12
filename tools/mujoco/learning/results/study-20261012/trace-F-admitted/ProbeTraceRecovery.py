"""在解压后的目录实际载入F检查点并重放一个原控制场景，不启动续训。"""
import hashlib
import json
from pathlib import Path
import sys

root=Path(__file__).resolve().parent
read=lambda p:json.loads(p.read_text(encoding="utf-8"))
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def require(ok,message):
    if not ok:
        raise RuntimeError(message)

inventory=read(root/"恢复包清单.json")
require(all(sha(root/name)==value for name,value in inventory["files"].items()),"包内字节不符")
learning=root/"snapshot/tools/mujoco/learning"
sys.path.insert(0,str(learning))
import numpy as np
import torch
from stable_baselines3 import PPO
from WbrLearningEnv import verify_snapshot,experiment_identity
from TrainWbrAligned import validate_training_bank
from StudyWbrPolicy import StudyPolicy
from EvaluateWbrStudy import load_cases,run_case

manifest=verify_snapshot()
require(experiment_identity(manifest)==inventory["experiment_identity"],"迁移物理身份改变")
rule_path=learning/"results/study-20261012/长程反馈单变量训练登记.json"
rule=read(rule_path)
bank=learning/"results/study-20261012/aligned-training-baseline/training-bank.json"
validate_training_bank(rule,rule_path,bank,manifest)
checkpoint=root/"recovery/F-trace-41"
contract=read(checkpoint/"policy.json")
require(sha(checkpoint/"checkpoint.zip")==contract["checkpoint_sha256"]
    and sha(checkpoint/"policy.npz")==contract["policy_sha256"],"检查点字节不符")
torch.set_num_threads(1)
model=PPO.load(checkpoint/"checkpoint.zip",device="cpu")
require(model.num_timesteps==inventory["steps"]==contract["training_steps"],"真实模型步数不符")
require(model.gae_lambda==model.rollout_buffer.gae_lambda==1. and model.gamma==.999,"真实模型算法不符")
require(contract["seed"]==41 and contract["envs"]==2 and contract["requested_total_steps"]==262144,"检查点预算或种子不符")
optimizer=model.policy.optimizer.state_dict()["state"]
require(len(optimizer)==13,"优化器状态不完整")
observations=np.random.default_rng(63419).normal(0,.5,(128,93)).astype(np.float32)
expected,_=model.predict(observations,deterministic=True)
policy=StudyPolicy(checkpoint)
actual=np.asarray([policy.predict(row) for row in observations])
error=float(np.max(np.abs(actual-expected)))
require(error<2e-5,"导出推理不符")
cases=load_cases(learning/"cases/trace-20261012/monitor.json")
scenario=cases["cases"][0]
result=run_case(scenario,None,contract["env_config"])
baseline=read(root/"evidence/monitor-baseline-summary.json")["baseline_results"][0].copy()
require(baseline.pop("controller")=="baseline" and result==baseline,"迁移后原控制未逐字段重现")
record=dict(schema=1,status="passed",steps=model.num_timesteps,optimizer_states=len(optimizer),
    actual_gae_lambda=model.gae_lambda,export_max_abs_error=error,baseline_case=scenario["id"],
    baseline_all_fields_exact=True,bank_admission_revalidated=True,experiment_identity=experiment_identity(manifest),
    library_sha256=manifest["library_sha256"],
    boundary="同机换目录实际模型载入与原控制重放；未在另一电脑验证，未运行F候选监控、续训或最终集。")
(root/"恢复核验结果.json").write_bytes((json.dumps(record,ensure_ascii=False,indent=2)+"\n").encode("utf-8"))
print(json.dumps(record,ensure_ascii=False))
