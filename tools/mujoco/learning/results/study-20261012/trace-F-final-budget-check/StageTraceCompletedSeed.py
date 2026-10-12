"""逐种子保存F完成模型的日志核验；不评价尚未统一锁定的策略。"""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

workspace=Path(__file__).resolve().parents[1]
repo=Path(r"D:\ARBATOS")
learning=repo/"tools/mujoco/learning"
results=learning/"results/study-20261012"
base=workspace/"work/study-20261012/long-trace"
read=lambda p:json.loads(p.read_text(encoding="utf-8"))
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def require(ok,message):
    if not ok:
        raise RuntimeError(message)

def git(*args,input=None):
    return subprocess.run(["git","-C",str(repo),*args],input=input,stdout=subprocess.PIPE,stderr=subprocess.PIPE,check=True).stdout

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument("--seed",type=int,choices=(41,73),required=True)
args=parser.parse_args()
seed=args.seed
require(Path(git("rev-parse","--show-toplevel").decode().strip()).resolve()==repo,"仓库不符")
require(git("branch","--show-current").strip()==b"main" and not git("diff","--cached","--name-only").strip(),"分支或索引需要核查")
target=results/f"trace-F-{seed}-trained"
proof=read(target/"训练完成核验.json")
require(proof["status"]=="passed" and proof["actual_model_steps"]==262144 and proof["actual_gae_lambda"]==1.,"实际完成模型未通过核验")
require(proof["completion"]["natural_budget_completion"] and proof["completion"]["progress_remaining"]==0.,"最后一批训练循环未完成")
require(all(sha(target/name)==value for name,value in proof["files"].items()),"模型或训练记录改变")
audit_dir=base/f"completed-log-audit-{seed}"
audit=read(audit_dir/"训练日志核验.json")
require(audit["complete_episodes"]==proof["completed_training_episodes"]
    and audit["physical_failure_episodes"]==proof["training_physical_failures"],"训练回合计数不符")
require(audit["registered_training_cases"]==144 and audit["run_sha256"]==sha(target/"run.json")
    and audit["captured_log_sha256"]==sha(audit_dir/"训练回合摘录.jsonl"),"日志来源改变")
parse=lambda p:[json.loads(line) for line in p.read_text(encoding="utf-8").splitlines() if line.strip()]
require(parse(target/"aligned-episodes.jsonl")==parse(audit_dir/"训练回合摘录.jsonl"),"原始日志与核验摘录不一致")
queue=read(base/"training/queue-state.json")
require(queue["jobs"][f"F-trace-{seed}-train"]["status"]=="completed","原队列任务未完成")
require(not (base/"evaluation/queue-state.json").exists(),"已启动评价，不能沿用未评价阶段说明")
live=[]
current_status={}
if queue["status"]=="running":
    current=queue["jobs"][queue["current_job"]]
    current_output=Path(current["actual_output"])
    command="Get-CimInstance Win32_Process | Where-Object { $_.Name -like 'python*' } | Select-Object ProcessId,CreationDate,CommandLine | ConvertTo-Json -Depth 3"
    processes=json.loads(subprocess.run(["pwsh","-NoProfile","-Command",command],stdout=subprocess.PIPE,stderr=subprocess.PIPE,encoding="utf-8",check=True).stdout)
    needle=str(current_output.resolve()).lower()
    live=[row for row in processes if row["ProcessId"]==queue["pid"] or needle in (row["CommandLine"] or "").lower()]
    require(any(row["ProcessId"]==queue["pid"] for row in live) and any(needle in (row["CommandLine"] or "").lower() for row in live),"后续任务没有对应的实际进程")
    if (current_output/"status.json").exists():
        current_status=read(current_output/"status.json")
doc="tools/仿真说明.md"
head=git("show","HEAD:"+doc)
require(git("show",":"+doc)==head,"说明索引已有其他修改")
working=(repo/doc).read_bytes()
marker=f"#### 单变量方案种子{seed}完成固定预算训练（2026-10-12）"
require(marker.encode("utf-8") not in working,"本阶段说明已存在")
for name in ("训练日志核验.json","训练回合摘录.jsonl"):
    require(not (target/name).exists(),"日志核验已保存")
    shutil.copyfile(audit_dir/name,target/name)
    require(sha(target/name)==sha(audit_dir/name),"保存后的日志核验不同")
shutil.copyfile(Path(__file__),target/"StageTraceCompletedSeed.py")
now=datetime.now().astimezone().isoformat()
progress_path=learning/"研究进度.json"
progress=read(progress_path)
progress["updated_at"]=now
completed=progress["long_trace_F"].setdefault("completed_seeds",{})
completed[str(seed)]=dict(training_steps=262144,training_directory=target.relative_to(repo).as_posix(),
    training_proof_sha256=sha(target/"训练完成核验.json"),training_log_proof_sha256=sha(target/"训练日志核验.json"),
    episodes=audit["complete_episodes"],covered_cases=audit["unique_cases"],
    physical_failures=audit["physical_failure_episodes"],control_guard_violation_episodes=sum(row["guard_violation_episodes"] for row in audit["tasks"].values()))
progress["long_trace_F"].update(stage="training_complete_awaiting_four_model_lock" if len(completed)==2 else "second_seed_training",
    queue_status_at_save=queue["status"],current_job_at_save=queue["current_job"],independent_performance_verified=False,
    trainer_pid=current_status.get("pid"),training_steps_at_save=current_status.get("steps"),
    os_processes_checked_at=now,os_processes_at_save=live)
if "active_process" in progress:
    for row in progress["active_process"].get("queues",[]):
        state=read(Path(row["path"]))
        row.update(status=state["status"],pid=state["pid"],current_job=state["current_job"])
    progress["active_process"].update(updated_at=now,os_processes_at_save=live)
progress["completed"].append(f"F{seed}完成262144步；实际网络及13组优化器精确载入，算法lambda=1，导出推理通过。{audit['complete_episodes']}个完整训练回合日志评分可复算，覆盖{audit['unique_cases']}/144场景，物理失败{audit['physical_failure_episodes']}次。尚未运行新监控候选评价。")
progress_path.write_bytes((json.dumps(progress,ensure_ascii=False,indent=2)+"\n").encode("utf-8"))
violations=sum(row["guard_violation_episodes"] for row in audit["tasks"].values())
appendix=f"""{marker}

F种子{seed}按原登记完成262144步，实际模型和优势估计缓存的lambda均为1。保存的网络与13组优化器状态载入值精确一致，128组输入与导出策略的最大误差为{proof['export_max_abs_error']:.3g}。最终模型、优化器、运行配置及完整日志见 `learning/results/study-20261012/trace-F-{seed}-trained/`，冻结环境仍使用已保存的F首段恢复包。

日志核验了{audit['complete_episodes']}个完整训练回合，覆盖{audit['unique_cases']}/144个训练场景；物理失败{audit['physical_failure_episodes']}次，{violations}个带探索的训练回合出现控制指标超限。累计评分由各成本重算的最大误差为{audit['maximum_reward_reconstruction_error']:.3g}。训练表现不能替代独立评价，也不据此选中途权重。只有原E/F四个最终模型全部完成并锁定后，才执行预先登记的新24例统一对照；此处没有扩大预算或运行最终60例。
"""
expected=working.rstrip(b"\r\n")+b"\n\n"+appendix.encode("utf-8")
(repo/doc).write_bytes(expected)
blob=git("hash-object","-w","--stdin",input=head.rstrip(b"\r\n")+b"\n\n"+appendix.encode("utf-8")).decode().strip()
git("update-index","--cacheinfo",f"100644,{blob},{doc}")
paths=[p.relative_to(repo).as_posix() for p in target.iterdir() if p.is_file()]+[progress_path.relative_to(repo).as_posix()]
git("add","--",*paths)
actual=set(git("diff","--cached","--name-only","-z").decode("utf-8").strip("\0").split("\0"))
require(actual=={doc,*paths},"索引混入其他工作")
require(all(git("show",":"+p)==(repo/p).read_bytes() for p in paths),"暂存字节与已核验产物不同")
require((repo/doc).read_bytes()==expected,"说明工作区改变")
git("-c","core.whitespace=blank-at-eol,space-before-tab,cr-at-eol,-blank-at-eof","diff","--cached","--check")
print(git("-c","core.quotepath=false","diff","--cached","--stat").decode("utf-8"))
