"""所有实际预检通过后才生成F固定双种子训练计划；不执行候选评价。"""
from datetime import datetime
import hashlib
import json
from pathlib import Path
import shutil
import sys

workspace=Path(__file__).resolve().parents[1]
repo=Path(r"D:\ARBATOS")
base=workspace/"work/study-20261012/long-trace"
frozen=base/"frozen"
learning=frozen/"tools/mujoco/learning"
sys.path.insert(0,str(learning))
from WbrLearningEnv import verify_snapshot,experiment_identity
from EvaluateWbrStudy import load_cases,load_baseline_cache,_completed_path
from BuildWbrStudyCases import canonical_bytes
from TrainWbrAligned import validate_training_bank

read=lambda p:json.loads(p.read_text(encoding="utf-8"))
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def require(ok,message):
    if not ok:
        raise RuntimeError(message)

def save(p,value):
    p.parent.mkdir(parents=True,exist_ok=True)
    p.write_bytes((json.dumps(value,ensure_ascii=False,indent=2)+"\n").encode("utf-8"))

target=base/"长训练准入.json"
require(not target.exists(),"已有准入结果")
manifest=verify_snapshot(frozen)
rule_path=learning/"results/study-20261012/长程反馈单变量训练登记.json"
rule=read(rule_path)
require(sha(rule_path)=="8213ad57535e58151e2548b5df5bb39f33797972ee699a5ee845eade2a5863db","F登记改变")
bank=learning/"results/study-20261012/aligned-training-baseline/training-bank.json"
validate_training_bank(rule,rule_path,bank,manifest)
smoke=read(base/"实际续训核验.json")
require(all(smoke["checks"].values()),"短训续训未通过")
expected={str((base/"smoke"/name/filename).resolve()) for name in ("first","resumed")
    for filename in ("checkpoint.zip","policy.json","policy.npz","run.json","status.json")}
require(set(smoke["files"])==expected,"短训实际产物集合不符")
require(all(sha(Path(p))==value for p,value in smoke["files"].items()),"短训产物变化")
require(sha(base/"smoke/resumed/resume-load-check.json")==smoke["resume_load_check_sha256"],"真实载入证明变化")
preflight=read(base/"参数及权限核验.json")
require(preflight["checks_pass"] and preflight["actual_rollout_lambda_one"]
    and preflight["smoke_proof_sha256"]==sha(base/"实际续训核验.json")
    and preflight["registration_sha256"]==sha(rule_path)
    and preflight["source_sha256"]==sha(workspace/"work/CheckTracePreflight.py"),"真实参数和权限证明不符")
queue=read(base/"baseline-queue/queue-state.json")
require(queue["status"]=="completed" and queue["jobs"]["baseline"]["status"]=="completed","原控制监控基线未完成")
baseline_dir=Path(queue["jobs"]["baseline"]["actual_output"])
require(baseline_dir.resolve()==(base/"baseline").resolve(),"基线目录改变")
cases_path=learning/"cases/trace-20261012/monitor.json"
cases=load_cases(cases_path)
require(cases["cases_sha256"]==rule["monitor_cases_sha256"] and cases["case_count"]==24,"新监控场景改变")
config=dict(reward_mode="dense",action_mode="reference",gas_force=450.)
baseline=load_baseline_cache(baseline_dir/"summary.json",cases,manifest,experiment_identity(manifest),config)
report,run=read(baseline_dir/"summary.json"),read(baseline_dir/"run.json")
binding=run["binding"]
require(run["binding_sha256"]==hashlib.sha256(canonical_bytes(binding)).hexdigest()
    and report["run_binding_sha256"]==run["binding_sha256"],"基线运行绑定损坏")
require(binding["evaluator_sha256"]==sha(learning/"EvaluateWbrStudy.py") and binding["policy_artifacts"] is None
    and binding["split"]=="trace-monitor","基线评价器/对象错误")
for row in baseline:
    done=read(_completed_path(baseline_dir,"baseline",row["id"]))
    require(done["row"]==row and done["binding_sha256"]==run["binding_sha256"],"基线完成记录不符")
all_pass=all(row["metrics"]["passed"] and "motion_quality" in row for row in baseline)
proof=dict(schema=1,verified_at=datetime.now().astimezone().isoformat(),eligible=all_pass,
    baseline_count=24,all_baseline_physical_pass=all_pass,all_completed_records_exact=True,
    training_bank_sha256=sha(bank),registration_sha256=sha(rule_path),
    source_files={str(p):sha(p) for p in (base/"实际续训核验.json",base/"参数及权限核验.json",
        baseline_dir/"summary.json",baseline_dir/"run.json",cases_path,rule_path,frozen/"manifest.json",Path(__file__))},
    source_model_files=smoke["files"],resume_load_check_sha256=smoke["resume_load_check_sha256"],
    experiment_identity=experiment_identity(manifest),library_sha256=manifest["library_sha256"],
    permitted_steps_each=262144,seeds=[41,73],policy_evaluation_permitted=False,
    note="只准固定两次长训练；所有四个E/F最终权重须再独立锁定才准运行新监控。任何旧评价或最终集均未获准。")
save(target,proof)
require(all_pass,"新基线有物理失败，登记要求停止F，不能换场景")
training=base/"training"
require(not training.exists(),"训练目录已存在，不能重复启动")
jobs=[]
for seed in rule["screening_seeds"]:
    arguments=["--output",str(training/str(seed)/"train"),"--bank",str(bank),"--registration",str(rule_path),
        "--seed",str(seed),"--envs","2","--steps","262144"]
    jobs.append(dict(id=f"F-trace-{seed}-train",kind="aligned_train",arguments=arguments))
plan=dict(schema=1,stage="F_fixed_two_seed_training",deadline=rule["deadline"],python=str(workspace/"work/rl-env/Scripts/python.exe"),
    learning_directory=str(learning),admission_sha256=sha(target),jobs=jobs)
save(training/"plan.json",plan)
destination=repo/"tools/mujoco/learning/results/study-20261012/trace-F-admitted"
sources=[(target,"长训练准入.json"),(base/"实际续训核验.json","实际续训核验.json"),
    (base/"参数及权限核验.json","参数及权限核验.json"),(baseline_dir/"summary.json","monitor-baseline-summary.json"),
    (baseline_dir/"run.json","monitor-baseline-run.json"),(training/"plan.json","训练计划.json"),
    (Path(__file__),"AdmitTraceTrial.py"),(workspace/"work/CheckTracePreflight.py","CheckTracePreflight.py")]
for source,name in sources:
    require(not (destination/name).exists(),"保存目标已存在")
    shutil.copyfile(source,destination/name)
print(json.dumps(dict(eligible=all_pass,baseline_count=24,registration_sha256=sha(rule_path),training_plan=str(training/"plan.json")),ensure_ascii=False))
