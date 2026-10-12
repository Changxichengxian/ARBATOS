"""保存E双种子完整负结果及A至D共同验证口径；只暂存本任务文件。"""
from datetime import datetime
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys

workspace=Path(__file__).resolve().parents[1]
repo=Path(r"D:\ARBATOS")
learning=repo/"tools/mujoco/learning"
results=learning/"results/study-20261012"
study=workspace/"work/study-20261012"
base=study/"aligned-reference"
sys.path.insert(0,str(base/"training-frozen/tools/mujoco/learning"))
from PromoteWbrAligned import verify_promotion

read=lambda p:json.loads(p.read_text(encoding="utf-8"))
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def require(ok,message):
    if not ok:
        raise RuntimeError(message)

def save(p,value):
    p.write_bytes((json.dumps(value,ensure_ascii=False,indent=2)+"\n").encode("utf-8"))

def git(*args,input=None):
    return subprocess.run(["git","-C",str(repo),*args],input=input,stdout=subprocess.PIPE,stderr=subprocess.PIPE,check=True).stdout

require(git("branch","--show-current").strip()==b"main","分支不符")
require(not git("diff","--cached","--name-only").strip(),"索引已有其他工作")
queue=read(base/"screening/queue-state.json")
admission=read(base/"admission/status.json")
require(queue["status"]==admission["status"]=="completed" and admission["stage"]=="ready_for_next_stage","登记流程尚未结束")
promotion_path=base/"screening/screen-expand-promotion.json"
promotion=verify_promotion(promotion_path,require_eligible=False)
require(not promotion["eligible"] and not admission["eligible"],"并非负结果，不能套用此保存程序")
rows=[]
for seed in (41,73):
    trained=results/f"aligned-E-{seed}-trained"
    monitor=results/f"aligned-E-{seed}-monitor"
    proof=read(trained/"训练完成核验.json")
    evaluation=read(monitor/"单种子监控核验.json")
    audit_dir=base/f"completed-log-audit-{seed}"
    audit=read(audit_dir/"训练日志核验.json")
    require(proof["status"]=="passed" and all(evaluation["verification"].values()),"训练或监控核验未通过")
    require(all(sha(trained/name)==value for name,value in proof["files"].items()),"最终训练产物改变")
    require(evaluation["sources"]["training_proof_sha256"]==sha(trained/"训练完成核验.json"),"监控的训练证明改变")
    require(evaluation["sources"]["summary_sha256"]==sha(monitor/"summary.json") and evaluation["sources"]["run_sha256"]==sha(monitor/"run.json"),"监控来源变化")
    require(audit["complete_episodes"]==proof["completed_training_episodes"]==490 and audit["unique_cases"]==144,"完整回合或覆盖数不符")
    require(audit["physical_failure_episodes"]==proof["training_physical_failures"]==0,"物理失败计数不符")
    require(audit["run_sha256"]==sha(trained/"run.json") and audit["captured_log_sha256"]==sha(audit_dir/"训练回合摘录.jsonl"),"日志来源变化")
    parse=lambda p:[json.loads(line) for line in p.read_text(encoding="utf-8").splitlines() if line.strip()]
    require(parse(trained/"aligned-episodes.jsonl")==parse(audit_dir/"训练回合摘录.jsonl"),"原始日志与规范化摘录不一致")
    promoted=next(row for row in promotion["results"] if row["seed"]==seed)
    require(promoted["gate"]==evaluation["single_seed_gate"],"原晋级复算与单种子记录不一致")
    report=read(monitor/"summary.json")
    checks=report["acceptance"]["checks"]
    require(checks["stop_mean_improvement"]["paired_cases"]==checks["push_mean_improvement"]["paired_cases"]==8,"24例均值口径不符")
    rows.append(dict(seed=seed,gate=evaluation["single_seed_gate"],
        original_guard_failing_cases=len(evaluation["failed_original_cases"]),
        motion_quality_failing_cases=len(evaluation["failed_motion_cases"]),
        original_guard_failure_counts=evaluation["original_guard_failure_counts"],
        motion_quality_failure_counts=evaluation["motion_quality_failure_counts"],
        trained_directory=trained.relative_to(repo).as_posix(),monitor_directory=monitor.relative_to(repo).as_posix(),
        training_steps=proof["training_steps"],training_episodes=proof["completed_training_episodes"],
        elapsed_training_seconds=proof["elapsed_training_seconds"],
        training_guard_violation_episodes=sum(row["guard_violation_episodes"] for row in audit["tasks"].values()),
        sources={str(p):sha(p) for p in (trained/"训练完成核验.json",monitor/"单种子监控核验.json",audit_dir/"训练日志核验.json")}))
summary_path=study/"四路线完整对照核验.json"
summary=read(summary_path)
require(summary["completed_record_rows_checked"]==720 and len(summary["rows"])==8 and summary["common_baseline_45_metrics_exact"],"A至D对照尚未完整核验")
require(summary["source_script_sha256"]==sha(workspace/"work/SummarizeCompletedRoutes.py"),"共同核验程序变化")
require(all(sha(Path(p))==value for row in summary["rows"] for p,value in row["source"].items()),"共同验证来源变化")
plot=read(study/"completed-validation-plot-data.json")
figure=workspace/"outputs/轮腿已完成独立验证对照.png"
require(plot["figure_sha256"]==sha(figure) and len(plot["rows"])==8,"对照图来源变化")
require(all(sha(Path(row["source"]))==row["sha256"] for row in plot["rows"]),"对照图输入变化")

fqueue=read(study/"long-trace/training/queue-state.json")
fjob=fqueue["jobs"].get(fqueue["current_job"],{})
fstatus=read(Path(fjob["actual_output"])/"status.json")
require(fqueue["status"]==fstatus["status"]=="running","后续训练状态需要核查")
pids=[fqueue["pid"],fstatus["pid"]]
command="Get-CimInstance Win32_Process | Where-Object { $_.ProcessId -in @("+",".join(map(str,pids))+") } | Select-Object ProcessId,ParentProcessId,CreationDate,CommandLine | ConvertTo-Json -Depth 3"
processes=json.loads(subprocess.run(["pwsh","-NoProfile","-Command",command],stdout=subprocess.PIPE,stderr=subprocess.PIPE,encoding="utf-8",check=True).stdout)
require({p["ProcessId"] for p in processes}==set(pids) and all("long-trace" in p["CommandLine"] for p in processes),"后续训练没有对应的实际进程")

target=results/"aligned-E-both-completed"
require(not target.exists(),"双种子完成记录已保存")
doc="tools/仿真说明.md"
head=git("show","HEAD:"+doc)
require(git("show",":"+doc)==head,"说明索引已有其他工作")
working=(repo/doc).read_bytes()
marker="#### 新评分双种子负结果与四路线统计复核（2026-10-12）"
require(marker.encode("utf-8") not in working,"说明已存在")

target.mkdir()
for seed in (73,):
    for name in ("训练日志核验.json","训练回合摘录.jsonl"):
        destination=results/f"aligned-E-{seed}-trained"/name
        require(not destination.exists(),"日志保存目标已存在")
        shutil.copyfile(base/f"completed-log-audit-{seed}"/name,destination)
for source,name in ((promotion_path,"原双种子晋级复算.json"),(base/"admission/status.json","准备流程结束记录.json"),
    (base/"screening/queue-state.json","筛选队列结束记录.json"),
    (workspace/"work/SaveAlignedBothCompleted.py","SaveAlignedBothCompleted.py"),
    (workspace/"work/SaveAlignedCompleted.py","SaveAlignedCompleted.py"),
    (workspace/"work/SaveAlignedMonitor.py","SaveAlignedMonitor.py"),
    (workspace/"work/AnalyseAlignedTrainingLog.py","AnalyseAlignedTrainingLog.py")):
    shutil.copyfile(source,target/name)
    require(sha(source)==sha(target/name),"保存副本不一致")
now=datetime.now().astimezone().isoformat()
record=dict(schema=1,checked_at=now,route="E-aligned-reference",rows=rows,both_seed_gate_pass=promotion["eligible"],
    training_steps_each=262144,monitor_pairs_each=24,completed_evaluation_records_checked=96,
    promotion_recomputed_exact=True,expansion_authorized=False,final_dataset_used=False,
    promotion_sha256=sha(promotion_path),source_script_sha256=sha(Path(__file__)),
    boundary="E原24例监控：两种子原预算全部完成但未通过逐例条件，停止扩大训练。后续在F新24例上仅作事前登记的单变量对照，不改变此结论。")
save(target/"双种子筛选结论.json",record)
overview=results/"四路线完整验证"
overview.mkdir(exist_ok=False)
for source,name in ((summary_path,summary_path.name),(figure,"四路线完整验证对照.png"),
    (study/"completed-validation-plot-data.json","四路线完整验证图数据.json"),
    (workspace/"work/SummarizeCompletedRoutes.py","SummarizeCompletedRoutes.py"),
    (workspace/"work/PlotCompletedValidation.py","PlotCompletedValidation.py")):
    shutil.copyfile(source,overview/name)
    require(sha(source)==sha(overview/name),"共同验证保存副本不一致")

lines=["轮腿离线研究阶段结果",f"保存时间：{now}","", "尚无方案达到正式采用标准。以下结果只适用于已冻结的16.015 kg轮腿仿真与原控制版本。", "", "A至D：同一45例配对验证，每个方案两个独立种子。", "停车均值使用18个单项停车场景；受推均值使用18个单项受推场景；9个组合场景参加逐例保护。", "热指标是四个髋关节力矩平方的时间积分，使用全部45例，不是实测温度。", "720条原控制/候选完成记录核对一致，八组原控制指标逐字段相同。", "", "方案 / 种子 / 停车误差降低 / 受推误差降低 / 热指标增加 / 违反原逐例保护的场景数"]
for row in summary["rows"]:
    lines.append(f"{row['label']} / {row['seed']} / {100*row['stop_improvement']:+.4f}% / {100*row['push_improvement']:+.4f}% / {100*row['hip_heat_change']:+.4f}% / {row['original_guard_failing_cases']}/45")
lines += ["", "上述360个候选场景均通过物理检查；控制指标超限与摔倒等物理失败是两回事。", "八组均未通过各自登记筛选。A/B/C主要收益先未达标，未追加运动质量评价；D41运动质量也失败，D73该项通过。", "预算不同：A/B各131072训练步，C各40个候选，D各72个候选。不能据此作严格等算力排名。", "", "E：重新对齐评分的参考参数策略，独立24例监控。每组均完成262144训练步和490个完整回合。", "24例中的停车/受推均值各使用8个单项场景，另8个组合场景参加逐例保护。", "不能把E的24例均值与上面的45例直接比较。", "种子 / 停车误差降低 / 受推误差降低 / 热指标增加 / 原逐例保护失败 / 运动质量失败"]
for row in rows:
    g=row["gate"]
    lines.append(f"{row['seed']} / {100*g['stop_improvement']:+.4f}% / {100*g['push_improvement']:+.4f}% / {100*g['heat_change']:+.4f}% / {row['original_guard_failing_cases']}/24 / {row['motion_quality_failing_cases']}/24")
lines += ["", "两种子监控的全部96条完成记录和原晋级条件已复算；这条配置不扩大训练。", "F正在进行单变量对照，只改变训练反馈的时间权重；同一训练库、动作、预算及评分保持不变。", "须等E/F四份完整模型锁定后，才对新24例执行一次评价。尚无F性能结论。", "", "正式目标仍是三个独立种子在60个未用于调参的最终场景中，停车与受推两项平均误差各降低至少10%，且逐例和物理保护全部通过。最终60例尚未使用。", "没有将候选接入正式控制，也未连接实车。模型、优化器、实际运行日志与核验代码均随阶段保存。", ""]
report_text="\n".join(lines)
(target/"阶段结果说明.txt").write_text(report_text,encoding="utf-8")
(workspace/"outputs/轮腿研究阶段结果.txt").write_text(report_text,encoding="utf-8")

progress_path=learning/"研究进度.json"
progress=read(progress_path)
progress["updated_at"]=now
progress["aligned_reference"].update(stage="screening_complete_not_eligible",eligible=False,
    both_seed_conclusion=(target/"双种子筛选结论.json").relative_to(repo).as_posix(),queue_completed=True)
progress["long_trace_F"].update(stage="first_seed_training" if "41" in fqueue["current_job"] else "second_seed_training",
    queue_pid=fqueue["pid"],trainer_pid=fstatus["pid"],training_steps_at_save=fstatus["steps"],os_processes_checked_at=now)
queue_paths=[study/"screening/queue-state.json",study/"reference-search/screening/queue-state.json",base/"screening/queue-state.json",study/"long-trace/training/queue-state.json"]
progress["active_process"]=dict(updated_at=now,os_processes_at_save=processes,
    queues=[dict(path=str(p),status=read(p)["status"],pid=read(p)["pid"],current_job=read(p)["current_job"]) for p in queue_paths],
    note="保存时的状态；接续前必须核查实际进程，不凭历史PID重启。")
progress["completed"].append("E两个种子各262144步与原24例监控均已完成，96条评价记录和双种子晋级条件完整复算，均未通过；最终模型、优化器及完整日志已保存。不扩大这条训练配置。")
progress["completed"].append("A至D八组45例共同验证的720条原控制/候选完成记录已核对，原控制指标逐字段一致，原验收可复算。明确单项均值、组合逐例保护及热指标口径，保存对照图和完整结果表。")
progress["next"]="F按原固定计划完成41/73各262144步；E原队列和准备流程已完成且不晋级，不重启。所有E/F四模型完成后，先执行work/LockTraceComparison.py核验并锁定，再运行唯一新24例监控计划。保存完整对照及负结果/后续判断；最终60例仍未使用。"
save(progress_path,progress)
table="\n".join(f"- 种子{row['seed']}：停车误差降低{row['gate']['stop_improvement']*100:.4f}%，受推误差降低{row['gate']['push_improvement']*100:.4f}%，髋部热指标增加{row['gate']['heat_change']*100:.4f}%；原逐例保护失败{row['original_guard_failing_cases']}例，运动质量失败{row['motion_quality_failing_cases']}例。" for row in rows)
appendix=f"""{marker}

E的两个独立种子均完成262144步、490个完整训练回合，覆盖全部144个训练场景；两份最终网络和13组优化器实际载入一致，导出数值核验通过。24例监控每组全部48条原控制/候选完成记录均已逐条核对，原双种子晋级程序重新计算一致，结论为不符合扩大训练条件。

{table}

两组均无物理失败，但仍存在控制指标退步，不能据平均收益接入正式控制。原24例监控中的停车和受推均值各用8个单项场景，另8个组合场景参加逐例保护；髋热采用24例全部场景。完整双种子结论、原晋级记录和准备流程结束状态见 `learning/results/study-20261012/aligned-E-both-completed/`，第二组模型与监控见 `aligned-E-73-trained/` 和 `aligned-E-73-monitor/`。原E配置停止扩大；F仍按事先登记继续，后续对新24例的比较不改变E原筛选结论。

A至D八组共同45例验证的720条完成记录也已核对，八组原控制指标逐字段相同、原验收可复算。停车/受推均值各来自18个单项场景，9个组合场景参加逐例保护，热指标使用全部45例；四类预算不同，不能当成等算力排名。汇总和图见 `learning/results/study-20261012/四路线完整验证/`。这些结果与E原24例、F新24例分别保留，正式60例仍未使用。
"""
expected=working.rstrip(b"\r\n")+b"\n\n"+appendix.encode("utf-8")
(repo/doc).write_bytes(expected)
blob=git("hash-object","-w","--stdin",input=head.rstrip(b"\r\n")+b"\n\n"+appendix.encode("utf-8")).decode().strip()
git("update-index","--cacheinfo",f"100644,{blob},{doc}")
paths=[p.relative_to(repo).as_posix() for directory in (target,overview,results/"aligned-E-73-trained",results/"aligned-E-73-monitor") for p in directory.iterdir() if p.is_file()]
paths.append(progress_path.relative_to(repo).as_posix())
git("add","--",*paths)
actual=set(git("diff","--cached","--name-only","-z").decode("utf-8").strip("\0").split("\0"))
require(actual=={doc,*paths},"索引混入其他工作或缺少本次文件")
require(all(git("show",":"+p)==(repo/p).read_bytes() for p in paths),"暂存字节与已核验结果不同")
require((repo/doc).read_bytes()==expected,"说明工作区发生变化")
git("-c","core.whitespace=blank-at-eol,space-before-tab,cr-at-eol,-blank-at-eof","diff","--cached","--check")
print(git("-c","core.quotepath=false","diff","--cached","--stat").decode("utf-8"))
