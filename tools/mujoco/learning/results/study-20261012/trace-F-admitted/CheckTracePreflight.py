"""核验F真实模型中的算法设置、续训及固定预算拒绝边界。"""
from datetime import datetime
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import torch
from stable_baselines3 import PPO

workspace=Path(__file__).resolve().parents[1]
base=workspace/"work/study-20261012/long-trace"
frozen=base/"frozen"
learning=frozen/"tools/mujoco/learning"
sys.path.insert(0,str(learning))
from WbrLearningEnv import verify_snapshot
read=lambda p:json.loads(p.read_text(encoding="utf-8"))
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def require(ok,message):
    if not ok:
        raise RuntimeError(message)

def save(p,value):
    p.parent.mkdir(parents=True,exist_ok=True)
    p.write_bytes((json.dumps(value,ensure_ascii=False,indent=2)+"\n").encode("utf-8"))

target=base/"参数及权限核验.json"
require(not target.exists(),"核验已存在")
verify_snapshot(frozen)
smoke=read(base/"实际续训核验.json")
require(all(smoke["checks"].values()),"实际续训未通过")
require(all(sha(Path(p))==digest for p,digest in smoke["files"].items()),"实际模型来源改变")
rule_path=learning/"results/study-20261012/长程反馈单变量训练登记.json"
bank=learning/"results/study-20261012/aligned-training-baseline/training-bank.json"
rule=read(rule_path)
torch.set_num_threads(1)
models=[PPO.load(base/"smoke"/name/"checkpoint.zip",device="cpu") for name in ("first","resumed")]
fields=("gamma","gae_lambda","n_steps","batch_size","n_epochs","target_kl","learning_rate")
actual=[]
for model in models:
    values={key:getattr(model,key) for key in fields}
    values["clip_range"]=model.clip_range(1.)
    require(all(values[key]==rule["algorithm"][key] for key in values),"实际PPO算法参数与登记不一致")
    require(model.rollout_buffer.gae_lambda==1. and model.rollout_buffer.gamma==.999,"实际优势估计缓存参数未改变")
    require(model.observation_space.shape==(93,) and model.action_space.shape==(3,) and model.n_envs==1,"实际网络接口变化")
    actual.append(values)
rejected={}
output=base/"rejected-unregistered-run"
common=[sys.executable,str(learning/"TrainWbrAligned.py"),"--output",str(output),"--bank",str(bank),
    "--registration",str(rule_path),"--envs","2"]
for name,extra,message in (("expanded_budget",["--seed","41","--steps","1048576"],"种子或预算未登记"),
    ("formal_seed",["--seed","101","--steps","262144"],"种子或预算未登记"),
    ("promotion",["--seed","41","--steps","262144","--promotion",str(base/"no-promotion.json")],"F有限对照不接受晋级或扩展训练")):
    result=subprocess.run(common+extra,stdout=subprocess.PIPE,stderr=subprocess.PIPE,encoding="utf-8",timeout=60)
    require(result.returncode!=0 and message in result.stderr and not output.exists(),"权限测试没有正确拒绝："+name)
    rejected[name]=dict(returncode=result.returncode,expected_error=message,output_not_created=True)
proof=dict(schema=1,checked_at=datetime.now().astimezone().isoformat(),actual_ppo_parameters=actual,
    actual_rollout_lambda_one=True,actual_rollout_gamma_unchanged=True,observation93_action3=True,
    rejected=rejected,smoke_proof_sha256=sha(base/"实际续训核验.json"),registration_sha256=sha(rule_path),
    source_sha256=sha(Path(__file__)),checks_pass=True)
save(target,proof)
plan_path=base/"baseline-queue/plan.json"
require(not plan_path.exists(),"基线计划已存在")
arguments=["--output",str(base/"baseline"),"--cases",str(learning/"cases/trace-20261012/monitor.json"),
    "--baseline-only","--action-mode","reference","--reward-mode","dense","--deadline",rule["deadline"]]
save(plan_path,dict(schema=1,stage="F_new_monitor_baseline",deadline=rule["deadline"],
    python=sys.executable,learning_directory=str(learning),jobs=[dict(id="baseline",kind="evaluate",arguments=arguments)]))
print(json.dumps(proof,ensure_ascii=False,indent=2))
