"""保存训练完成边界的实际模型检查及当前运行状态，保护其他说明修改。"""
from datetime import datetime
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

workspace=Path(__file__).resolve().parents[1]
repo=Path(r"D:\ARBATOS")
base=workspace/"work/study-20261012/long-trace"
target=repo/"tools/mujoco/learning/results/study-20261012/trace-F-final-budget-check"
read=lambda p:json.loads(p.read_text(encoding="utf-8"))
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def require(ok,message):
    if not ok:
        raise RuntimeError(message)

def git(*args,input=None):
    return subprocess.run(["git","-C",str(repo),*args],input=input,stdout=subprocess.PIPE,stderr=subprocess.PIPE,check=True).stdout

def write(path,value):
    path.write_bytes((json.dumps(value,ensure_ascii=False,indent=2)+"\n").encode("utf-8"))

require(git("branch","--show-current").strip()==b"main" and not git("diff","--cached","--name-only").strip(),"分支或索引需要核查")
proof=read(base/"训练完成边界补检.json")
require(len(proof["completed_models"])==3 and proof["actual_final_sampling_checkpoint"]["epoch_counter"]==762
    and proof["premature_four_model_lock_rejected"] and not proof["evaluation_plan_created"],"实际检查不符")
require(all(sha(workspace/"work"/name)==value for name,value in proof["helpers"].items()),"核验后程序改变")
state=read(base/"training/queue-state.json")
current=Path(state["jobs"]["F-trace-73-train"]["actual_output"])
status=read(current/"status.json")
require(state["status"]==status["status"]=="running","训练运行状态改变")
command="@(Get-CimInstance Win32_Process | Where-Object { $_.Name -like 'python*' } | Select-Object ProcessId,CreationDate,CommandLine) | ConvertTo-Json -Depth 3"
processes=json.loads(subprocess.run(["pwsh","-NoProfile","-Command",command],stdout=subprocess.PIPE,encoding="utf-8",check=True).stdout)
live=[row for row in processes if row["ProcessId"] in (state["pid"],status["pid"])]
require(len(live)==2 and any(str(current).lower() in (row["CommandLine"] or "").lower() for row in live),"没有实际队列和训练进程")
target.mkdir(exist_ok=False)
for name in proof["helpers"]:
    shutil.copyfile(workspace/"work"/name,target/name)
shutil.copyfile(__file__,target/Path(__file__).name)
shutil.copyfile(base/"训练完成边界补检.json",target/"训练完成边界补检.json")
write(target/"核验时训练状态.json",dict(status=status,os_processes=live,queue_state=state))
progress_path=repo/"tools/mujoco/learning/研究进度.json"
progress=read(progress_path)
now=datetime.now().astimezone().isoformat()
progress["updated_at"]=now
progress["next"]="完成F73从原seed73重跑的262144步后，核对自然完成标记和真实循环进度；优化轮次计数上限768，允许原KL阈值提前结束优化。按trace-F-final-budget-check内最新工具保存模型和新尝试日志，再锁定四策略进行唯一新24例比较。旧42496步尝试不计入新模型训练。"
progress["long_trace_F"].update(final_budget_check=(target/"训练完成边界补检.json").relative_to(repo).as_posix(),
    final_model_checker_sha256=proof["helpers"]["WbrFinalModelChecks.py"], latest_lock_helper_sha256=proof["helpers"]["LockTraceComparison.py"],
    training_steps_at_save=status["steps"],queue_pid=state["pid"],trainer_pid=status["pid"],os_processes_checked_at=now,os_processes_at_save=live)
progress["goal_control_observed"]=dict(status="active",updated_at_unix=1791776611)
progress["completed"].append("完成真实模型末批检查：E41/E73/F41自然完成，计数均768、内部剩余进度0；实际F41最后采样回调模型虽也是262144步，但762计数且内部进度未归零，被新检查拒绝。主动停止及未完成四模型同样拒绝。原KL优化提前结束规则保留。")
write(progress_path,progress)
doc="tools/仿真说明.md"
head=git("show","HEAD:"+doc)
require(git("show",":"+doc)==head,"说明已有其他暂存修改")
working=(repo/doc).read_bytes()
marker="#### 暂停恢复与最后一批训练完成检查（2026-10-12）"
require(marker.encode("utf-8") not in working,"阶段说明已存在")
appendix=f"""{marker}

用户明确继续后，保留F73暂停时42496步的全部模型和日志，并从原种子73重新跑固定262144步。旧检查点在采样回调中保存：32768计数对应90轮优化，不能把采样步数对齐直接当作最后一批已优化。重跑只增加先前废弃尝试的计算量，原F41不重跑，冻结源码、登记、数据及最终验收不变；没有按性能挑选模型或种子。

新完成检查同时核对原队列实际来源、自然完成且无停止原因、真实模型累计和总预算、训练循环内部进度以及批次配置。E41/E73/F41三个已有最终模型均为262144步、768轮优化、剩余进度0，检查通过。真实F41最后一批采样回调检查点也有262144步，但仅762轮、剩余进度0.0078125，被拒绝；主动停止的状态和未完成四模型的锁定请求也被拒绝。

768是当前配置的优化轮次上限，不是梯度更新次数，也不作为未来必然值。原KL阈值允许某批优化提前结束，检查保留这一算法行为。F73最终模型尚未产生；保存和锁定程序已适配本次新输出目录及实际来源，评价仍须等四份完整最终模型全部通过。证据与当前工具见 `learning/results/study-20261012/trace-F-final-budget-check/`。本检查不训练新方案、不评价控制收益，也未使用最终60例。
"""
expected=working.rstrip(b"\r\n")+b"\n\n"+appendix.encode("utf-8")
(repo/doc).write_bytes(expected)
blob=git("hash-object","-w","--stdin",input=head.rstrip(b"\r\n")+b"\n\n"+appendix.encode("utf-8")).decode().strip()
git("update-index","--cacheinfo",f"100644,{blob},{doc}")
paths=[p.relative_to(repo).as_posix() for p in target.iterdir() if p.is_file()]+[progress_path.relative_to(repo).as_posix()]
git("add","--",*paths)
actual=set(git("diff","--cached","--name-only","-z").decode("utf-8").strip("\0").split("\0"))
require(actual=={doc,*paths},"暂存混入其他修改")
require(all(git("show",":"+p)==(repo/p).read_bytes() for p in paths),"暂存内容不一致")
require((repo/doc).read_bytes()==expected,"说明工作区被覆盖")
git("-c","core.whitespace=cr-at-eol","diff","--cached","--check")
print(json.dumps(dict(target=str(target),steps=status["steps"],staged_files=len(actual)),ensure_ascii=False))
