"""E/F四个最终模型全部完成并核验后，才生成新监控评价计划。"""
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import sys
import uuid

workspace=Path(__file__).resolve().parents[1]
study=workspace/"work/study-20261012"
base=study/"long-trace"
frozen=base/"frozen"
learning=frozen/"tools/mujoco/learning"
sys.path.insert(0,str(learning))
import numpy as np
import torch
from stable_baselines3 import PPO
from WbrLearningEnv import verify_snapshot,experiment_identity
from StudyWbrPolicy import StudyPolicy
from EvaluateWbrStudy import load_cases,create_lock,verify_lock,load_baseline_cache,_completed_path
from BuildWbrStudyCases import canonical_bytes
from RunWbrStudyQueue import make_job_binding,is_complete
from TrainWbrAligned import state_digest

read=lambda p:json.loads(p.read_text(encoding="utf-8"))
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def require(ok,message):
    if not ok:
        raise RuntimeError(message)

def save(p,value):
    """完整文件原子发布，存在时只核验，不覆盖另一份许可或计划。"""
    p.parent.mkdir(parents=True,exist_ok=True)
    encoded=(json.dumps(value,ensure_ascii=False,indent=2)+"\n").encode("utf-8")
    if p.exists():
        require(p.read_bytes()==encoded,"已有许可/计划内容不同，拒绝覆盖："+str(p))
        return
    temporary=p.with_name(p.name+"."+uuid.uuid4().hex+".tmp")
    try:
        with temporary.open("xb") as stream:
            stream.write(encoded)
            stream.flush()
            os.fsync(stream.fileno())
        try:
            os.link(temporary,p)
        except FileExistsError:
            require(p.read_bytes()==encoded,"另一进程已写入不同内容，拒绝覆盖")
    finally:
        if temporary.exists():
            temporary.unlink()

def check_resume_chain(directory,known_outputs,allow_smoke_budget_change=False):
    """只有队列实际尝试中的同协议检查点才能延续；逐段验证真实载入摘要。"""
    directory=Path(directory).resolve()
    known={Path(p).resolve() for p in known_outputs}
    visited=set()
    chain=[]
    while True:
        require(directory not in visited,"续训来源形成循环")
        visited.add(directory)
        policy=read(directory/"policy.json")
        owner=directory if directory in known else directory.parent.parent
        require(owner in known and (directory==owner or directory.parent.name=="checkpoints"),"续训来源不属于队列已知尝试")
        if policy["resume"] is None:
            require(policy["initial_steps"]==0,"从零训练的步数来源不符")
            return chain
        previous=Path(policy["resume"]).resolve()
        prior=read(previous/"policy.json")
        require(0<policy["initial_steps"]==prior["training_steps"]<policy["requested_total_steps"],"续训预算来源错误")
        for key in ("seed","envs","env_config","training_protocol","training_source_sha256","training_lineage","experiment_identity","library_sha256"):
            require(prior[key]==policy[key],"续训中协议改变："+key)
        require(prior["requested_total_steps"]==policy["requested_total_steps"] or
            (allow_smoke_budget_change and policy["seed"]==9001 and prior["requested_total_steps"]==1024 and policy["requested_total_steps"]==2048),
            "续训中预算改变")
        require(sha(previous/"checkpoint.zip")==prior["checkpoint_sha256"],"续训原检查点改变")
        proof=read(owner/"resume-load-check.json")
        require(proof["loaded_steps"]==policy["initial_steps"] and proof["checkpoint_sha256"]==prior["checkpoint_sha256"]
            and proof["training_source_sha256"]==policy["training_source_sha256"],"真实载入记录错配")
        model=PPO.load(previous/"checkpoint.zip",device="cpu")
        require(state_digest(model.policy.state_dict())==proof["network_state_sha256"]
            and state_digest(model.policy.optimizer.state_dict())==proof["optimizer_state_sha256"],"真实载入值与原检查点不同")
        chain.append(dict(from_checkpoint=str(previous),initial_steps=policy["initial_steps"],resume_load_proof_sha256=sha(owner/"resume-load-check.json")))
        del model
        directory=previous

if "--check-smoke-lineage" in sys.argv:
    torch.set_num_threads(1)
    chain=check_resume_chain(base/"smoke/resumed",[base/"smoke/first",base/"smoke/resumed"],allow_smoke_budget_change=True)
    require(len(chain)==1 and chain[0]["initial_steps"]==1024,"实际短训来源检查不符")
    print(json.dumps(dict(actual_resume_chain_checked=True,chain=chain),ensure_ascii=False))
    sys.exit(0)

manifest=verify_snapshot(frozen)
identity=experiment_identity(manifest)
rule_path=learning/"results/study-20261012/长程反馈单变量训练登记.json"
rule=read(rule_path)
require(sha(rule_path)=="8213ad57535e58151e2548b5df5bb39f33797972ee699a5ee845eade2a5863db","F登记改变")
cases_path=learning/"cases/trace-20261012/monitor.json"
cases=load_cases(cases_path)
require(cases["split"]=="trace-monitor" and cases["cases_sha256"]==rule["monitor_cases_sha256"],"不是登记的新监控集")
require(not (base/"evaluation/queue-state.json").exists(),"本批评价已经启动，不允许重新锁定")
admission=read(base/"长训练准入.json")
require(admission["eligible"] and all(sha(Path(p))==value for p,value in admission["source_files"].items()),"原准入来源改变")
f_plan=read(base/"training/plan.json")
require(f_plan["admission_sha256"]==sha(base/"长训练准入.json"),"准入证明与实际F训练计划不符")
expected_smoke_files={str((base/"smoke"/name/file).resolve()) for name in ("first","resumed")
    for file in ("checkpoint.zip","policy.json","policy.npz","run.json","status.json")}
require(set(admission["source_model_files"])==expected_smoke_files
    and all(sha(Path(p))==value for p,value in admission["source_model_files"].items()),"实际短训模型集合或字节改变")
require(admission["resume_load_check_sha256"]==sha(base/"smoke/resumed/resume-load-check.json"),"实际短训载入证明改变")
baseline_path=base/"baseline/summary.json"
config=dict(reward_mode="dense",action_mode="reference",gas_force=450.)
baseline=load_baseline_cache(baseline_path,cases,manifest,identity,config)
require(all(row["metrics"]["passed"] and "motion_quality" in row for row in baseline),"基线不完整")
baseline_run=read(base/"baseline/run.json")
require(hashlib.sha256(canonical_bytes(baseline_run["binding"])).hexdigest()==baseline_run["binding_sha256"],"基线绑定损坏")
for row in baseline:
    done=read(_completed_path(base/"baseline","baseline",row["id"]))
    require(done["row"]==row and done["binding_sha256"]==baseline_run["binding_sha256"],"原控制完成记录改变")
torch.set_num_threads(1)
records=[]
for route in ("E","F"):
    source_root=study/"aligned-reference/training-frozen" if route=="E" else frozen
    source_manifest=verify_snapshot(source_root)
    require(experiment_identity(source_manifest)==identity and source_manifest["library_sha256"]==manifest["library_sha256"],"两路线物理不一致")
    registration=source_root/"tools/mujoco/learning/results/study-20261012"/("对齐评分参考策略训练登记.json" if route=="E" else "长程反馈单变量训练登记.json")
    registered=read(registration)
    queue_root=study/"aligned-reference/screening" if route=="E" else base/"training"
    queue=read(queue_root/"queue-state.json")
    require(queue["status"]=="completed","所有原定任务尚未完成")
    plan=read(queue_root/"plan.json")
    require(queue["plan_sha256"]==sha(queue_root/"plan.json"),"队列计划改变")
    for seed in (41,73):
        name=f"{route}-"+("aligned" if route=="E" else "trace")+f"-{seed}-train"
        job=queue["jobs"][name]
        require(job["status"]=="completed","登记训练尚未完成")
        directory=Path(job["actual_output"])
        original_job=next(item for item in plan["jobs"] if item["id"]==name)
        job_binding=make_job_binding("aligned_train",original_job["arguments"],plan)
        require(job["job_binding"]==job_binding,"队列实际训练绑定改变")
        attempts=job["attempts"]
        require(attempts and directory.resolve()==Path(attempts[-1]["actual_output"]).resolve(),"训练来源不是队列最后一次实际尝试")
        payload=job_binding["payload"]
        require(is_complete("aligned_train",directory,original_job["arguments"],payload["source"],payload["script_sha256"],payload["source_hashes"]),"实际队列产物不完整")
        resume_chain=check_resume_chain(directory,[item["actual_output"] for item in attempts])
        policy,run,status=(read(directory/file) for file in ("policy.json","run.json","status.json"))
        require(status["status"]=="completed" and status["steps"]==policy["training_steps"]==262144,"最终模型不是登记完整预算")
        require(all(policy[key]==value for key,value in run.items()),"模型与运行记录不符")
        expected_protocol=dict(bank_sha256=admission["training_bank_sha256"],registration_sha256=sha(registration),
            sources={name:source_manifest["files"]["tools/mujoco/learning/"+name] for name in
                ("WbrAlignedEnv.py","WbrAlignedReward.py","TrainWbrAligned.py","TrainWbrStudy.py","TrainWbrLearning.py")},
            weights=registered["weights"],algorithm=registered["algorithm"])
        require(policy["training_protocol"]==expected_protocol,"训练协议改变")
        require(policy["seed"]==seed and policy["envs"]==2 and policy["requested_total_steps"]==262144
            and policy["library_sha256"]==manifest["library_sha256"] and policy["experiment_identity"]==identity,
            "正式对照必须来自登记种子的原固定预算训练")
        require(policy["env_config"]==config and policy["strict_training_checks"],"动作或物理检查改变")
        require(policy["training_source_sha256"]==expected_protocol["sources"]["TrainWbrAligned.py"],"训练入口错配")
        files={filename:sha(directory/filename) for filename in ("policy.json","policy.npz","checkpoint.zip","run.json","status.json")}
        require(files["checkpoint.zip"]==policy["checkpoint_sha256"] and files["policy.npz"]==policy["policy_sha256"],"模型文件改变")
        model=PPO.load(directory/"checkpoint.zip",device="cpu")
        require(model.num_timesteps==262144 and len(model.policy.optimizer.state_dict()["state"])==13,"实际模型预算或优化器不符")
        require(model.gae_lambda==registered["algorithm"]["gae_lambda"] and model.gamma==.999,"实际优势估计参数错误")
        observations=np.random.default_rng(749).normal(0,.5,(128,93)).astype(np.float32)
        expected,_=model.predict(observations,deterministic=True)
        exported=StudyPolicy(directory)
        actual=np.asarray([exported.predict(row) for row in observations])
        error=float(np.max(np.abs(actual-expected)))
        require(np.isfinite(actual).all() and error<2e-5,"实际推理与导出不符")
        require(all(sha(directory/file)==value for file,value in files.items()),"核验期间模型改变")
        records.append(dict(route=route,seed=seed,directory=str(directory),files=files,
            actual_steps=model.num_timesteps,actual_lambda=model.gae_lambda,export_error=error,
            resume_chain=resume_chain,
            registration_sha256=sha(registration),snapshot_sha256=sha(source_root/"manifest.json")))
        del model
# 上面四份全部通过后才允许写出任何一个策略锁，不能边训练边评价。
require(len(records)==4,"缺少对照模型")
jobs=[]
for row in records:
    label=f"{row['route']}-{row['seed']}"
    lock_path=base/"comparison-locks"/(label+".json")
    directory=Path(row["directory"])
    if lock_path.exists():
        lock=verify_lock(lock_path,cases_path,cases,directory,read(directory/"policy.json"),manifest,identity)
    else:
        lock=create_lock(cases_path,cases,directory,lock_path,manifest,identity)
    row["lock_sha256"]=lock["lock_sha256"]
    row["lock_file_sha256"]=sha(lock_path)
    arguments=["--output",str(base/"evaluation"/label),"--cases",str(cases_path),"--policy",str(directory),
        "--baseline-cache",str(baseline_path),"--lock",str(lock_path),"--deadline",rule["deadline"]]
    jobs.append(dict(id=label,kind="evaluate",arguments=arguments))
target=base/"四策略锁定.json"
locked_at=read(target)["locked_at"] if target.exists() else datetime.now().astimezone().isoformat()
require(datetime.fromisoformat(locked_at).utcoffset() is not None
    and datetime.fromisoformat(locked_at).timestamp()<=datetime.now().timestamp(),"已有锁定时间无效")
record=dict(schema=1,locked_at=locked_at,registration_sha256=sha(rule_path),
    cases_sha256=cases["cases_sha256"],baseline_summary_sha256=sha(baseline_path),
    all_four_completed_and_checked=True,steps_each=262144,models=records,source_sha256=sha(Path(__file__)),
    boundary="这只是一次性新监控的四策略锁定，未启动评价；不授权扩展、旧验证或最终集。")
record_bytes=(json.dumps(record,ensure_ascii=False,indent=2)+"\n").encode("utf-8")
plan=dict(schema=1,stage="F_one_time_comparison",deadline=rule["deadline"],
    python=str(workspace/"work/rl-env/Scripts/python.exe"),learning_directory=str(learning),
    all_four_policy_lock_sha256=hashlib.sha256(record_bytes).hexdigest(),jobs=jobs)
plan_path=base/"evaluation/plan.json"
plan_bytes=(json.dumps(plan,ensure_ascii=False,indent=2)+"\n").encode("utf-8")
require(not target.exists() or target.read_bytes()==record_bytes,"已有四策略锁定内容不同")
require(not plan_path.exists() or plan_path.read_bytes()==plan_bytes,"已有评价计划内容不同")
save(target,record)
save(plan_path,plan)
print(json.dumps(dict(all_four_locked=True,evaluation_plan=str(base/"evaluation/plan.json")),ensure_ascii=False))
