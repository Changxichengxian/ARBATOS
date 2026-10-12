"""复核E/F四个事先锁定策略的新24例结果；不生成扩大训练许可。"""
from datetime import datetime
import hashlib
import json
from pathlib import Path
import sys

workspace=Path(__file__).resolve().parents[1]
study=workspace/"work/study-20261012"
base=study/"long-trace"
frozen=base/"frozen"
learning=frozen/"tools/mujoco/learning"
sys.path.insert(0,str(learning))
from BuildWbrStudyCases import canonical_bytes
from EvaluateWbrStudy import (acceptance,motion_quality_acceptance,load_cases,
    load_baseline_cache,policy_artifacts,_completed_path,verify_lock)
from PromoteWbrAligned import gate
from RunWbrStudyQueue import make_job_binding,is_complete
from WbrLearningEnv import verify_snapshot,experiment_identity
from TraceMonitorLockCompatibility import install as install_trace_lock_compatibility, verify_report_lock

install_trace_lock_compatibility()

read=lambda p:json.loads(p.read_text(encoding="utf-8"))
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def require(ok,message):
    if not ok:
        raise RuntimeError(message)

def verify_evaluation(directory,cases_path,baseline_path,policy_directory,manifest,lock_path=None):
    """不运行物理仿真，只从实际完成记录复核全部场景和冻结指标。"""
    cases=load_cases(cases_path)
    identity=experiment_identity(manifest)
    policy=read(policy_directory/"policy.json")
    config=policy["env_config"]
    report,run=read(directory/"summary.json"),read(directory/"run.json")
    binding=run["binding"]
    require(run["binding_sha256"]==hashlib.sha256(canonical_bytes(binding)).hexdigest()
        and report["run_binding_sha256"]==run["binding_sha256"],"评价绑定不符")
    for key,value in dict(split=cases["split"],scenarios_sha256=cases["cases_sha256"],
        library_sha256=manifest["library_sha256"],experiment_identity=identity,env_config=config).items():
        require(binding[key]==report[key]==value,"评价来源改变："+key)
    require(binding["evaluator_sha256"]==manifest["files"]["tools/mujoco/learning/EvaluateWbrStudy.py"],"评价器来源改变")
    require(Path(binding["policy_directory"]).resolve()==policy_directory.resolve()
        and binding["policy_artifacts"]==policy_artifacts(policy_directory),"评价策略不是登记模型")
    require(binding["baseline_cache_sha256"]==sha(baseline_path),"原控制缓存改变")
    if lock_path is None:
        require(binding["policy_lock_sha256"] is None,"缺少对应策略锁")
    else:
        lock=verify_lock(lock_path,cases_path,cases,policy_directory,policy,manifest,identity)
        if cases["split"]=="trace-monitor" and binding["policy_lock_sha256"] is None:
            outer_lock=verify_report_lock(directory,cases_path,policy_directory,lock_path)
            require(outer_lock["policy_lock_sha256"]==lock["lock_sha256"] and report["policy_lock_sha256"] is None,"原监控报告外层锁不符")
        else:
            require(binding["policy_lock_sha256"]==report["policy_lock_sha256"]==lock["lock_sha256"],"评价使用的策略锁改变")
    baseline=load_baseline_cache(baseline_path,cases,manifest,identity,config)
    require(report["baseline_results"]==baseline,"原控制配对结果改变")
    completed_files={}
    for controller in ("baseline","learned"):
        rows=report[controller+"_results"]
        by_id={row["id"]:row for row in rows}
        require(len(rows)==len(by_id)==cases["case_count"],"评价缺少或重复场景")
        for scenario in cases["cases"]:
            row=by_id[scenario["id"]]
            require(all(row[key]==scenario[key] for key in ("id","seed","randomize","options"))
                and row["controller"]==controller and row["case"]==scenario["options"]["case"],"场景错配")
            path=_completed_path(directory,controller,row["id"])
            done=read(path)
            require(done["binding_sha256"]==run["binding_sha256"] and done["row"]==row,"实际完成记录与汇总不一致")
            completed_files[str(path)]=sha(path)
    recomputed=acceptance(baseline,report["learned_results"])
    quality=motion_quality_acceptance(baseline,report["learned_results"])
    recomputed["checks"]["motion_quality"]=quality
    recomputed["overall_pass"]=bool(recomputed["overall_pass"] and quality["pass"])
    require(recomputed==report["acceptance"],"原指标或运动质量不能完整复算")
    return report,completed_files

def compare():
    # 在读取任何新监控候选结果之前，先要求唯一登记队列完整结束。
    plan_path=base/"evaluation/plan.json"
    queue_path=base/"evaluation/queue-state.json"
    require(plan_path.is_file() and queue_path.is_file(),"唯一新监控队列尚未开始")
    plan,queue=read(plan_path),read(queue_path)
    require(queue["status"]=="completed","新监控队列尚未全部完成")
    require(queue["plan_sha256"]==sha(plan_path),"实际评价计划改变")
    record_path=base/"四策略锁定.json"
    require(plan["all_four_policy_lock_sha256"]==sha(record_path),"实际计划未绑定四策略锁")
    locked=read(record_path)
    rule_path=learning/"results/study-20261012/长程反馈单变量训练登记.json"
    require(sha(rule_path)==locked["registration_sha256"]=="8213ad57535e58151e2548b5df5bb39f33797972ee699a5ee845eade2a5863db","登记改变")
    rule=read(rule_path)
    manifest=verify_snapshot(frozen)
    cases_path=learning/"cases/trace-20261012/monitor.json"
    cases=load_cases(cases_path)
    require(cases["cases_sha256"]==locked["cases_sha256"]==rule["monitor_cases_sha256"] and cases["case_count"]==24,"新监控集改变")
    require(locked["all_four_completed_and_checked"] and locked["steps_each"]==262144,"四策略没有完整锁定")
    locker=workspace/"work/LockTraceComparison.py"
    require(locked["source_sha256"]==sha(locker),"四策略核验程序改变")
    labels={"E-41","E-73","F-41","F-73"}
    require(len(locked["models"])==4 and {f"{r['route']}-{r['seed']}" for r in locked["models"]}==labels,"四策略集合不符")
    require(len(plan["jobs"])==4 and {r["id"] for r in plan["jobs"]}==set(queue["jobs"])==labels,"评价任务集合不符")
    baseline_path=base/"baseline/summary.json"
    require(locked["baseline_summary_sha256"]==sha(baseline_path),"共同原控制改变")
    records=[]
    sources={str(p):sha(p) for p in (record_path,plan_path,queue_path,rule_path,baseline_path,locker,
        workspace/"work/TraceMonitorLockCompatibility.py",workspace/"work/ContinueTraceComparison.py")}
    for model in locked["models"]:
        label=f"{model['route']}-{model['seed']}"
        directory=Path(model["directory"])
        require(all(sha(directory/name)==value for name,value in model["files"].items()),"锁定后最终模型改变")
        policy=read(directory/"policy.json")
        require(policy["training_steps"]==model["actual_steps"]==262144
            and policy["seed"]==model["seed"] and model["actual_lambda"]==(.97 if model["route"]=="E" else 1.),"锁定预算或算法参数不符")
        lock_path=base/"comparison-locks"/(label+".json")
        require(sha(lock_path)==model["lock_file_sha256"] and read(lock_path)["lock_sha256"]==model["lock_sha256"],"单策略锁改变")
        original=next(job for job in plan["jobs"] if job["id"]==label)
        job=queue["jobs"][label]
        require(job["status"]=="completed" and job["attempts"],"对应评价尚未完成")
        require(job["job_binding"]==make_job_binding("evaluate",original["arguments"],plan),"评价任务绑定改变")
        output=Path(job["actual_output"])
        require(output.resolve()==(base/"evaluation"/label).resolve()
            and all(Path(attempt["actual_output"]).resolve()==output.resolve() for attempt in job["attempts"]),"评价重试换了输出目录")
        locked_time=datetime.fromisoformat(locked["locked_at"])
        require(locked_time<=min(datetime.fromisoformat(attempt["started_at"]) for attempt in job["attempts"]),"评价开始前没有锁定四策略")
        payload=job["job_binding"]["payload"]
        require(is_complete("evaluate",output,original["arguments"],payload["source"],payload["script_sha256"],payload["source_hashes"]),"实际评价产物不完整")
        report,completed=verify_evaluation(output,cases_path,baseline_path,directory,manifest,lock_path)
        sources.update(completed)
        sources.update({str(p):sha(p) for p in (output/"summary.json",output/"run.json",lock_path)})
        sources.update({str(directory/name):value for name,value in model["files"].items()})
        acceptance_result=report["acceptance"]
        checks=acceptance_result["checks"]
        pair_counts={name:checks[name].get("paired_cases",0) for name in ("stop_mean_improvement","push_mean_improvement")}
        complete_means=checks["physical_complete"]["pass"] and all(value==8 for value in pair_counts.values())
        records.append(dict(route=model["route"],seed=model["seed"],gate=gate(acceptance_result,rule["screening_first"]),
            means={name:checks[name].get("learned_mean") for name in ("stop_mean_improvement","push_mean_improvement","hip_heat_mean")},
            complete_mean_comparison_available=complete_means,mean_pair_counts=pair_counts,
            failed_original_cases=[row for row in acceptance_result["cases"] if not row["pass"]],
            failed_motion_cases=[row for row in checks["motion_quality"]["cases"] if not row["pass"]],
            completed_records_checked=len(completed),evaluation_directory=str(output),policy_directory=str(directory)))
    comparisons=[]
    for seed in (41,73):
        before=next(row for row in records if row["route"]=="E" and row["seed"]==seed)
        after=next(row for row in records if row["route"]=="F" and row["seed"]==seed)
        if before["complete_mean_comparison_available"] and after["complete_mean_comparison_available"] and all(
            isinstance(value,(int,float)) and value>0 for value in before["means"].values()):
            comparisons.append(dict(seed=seed,F_relative_to_E={key:(1-after["means"][key]/before["means"][key]) for key in
                ("stop_mean_improvement","push_mean_improvement")},
                F_heat_increase_relative_to_E=after["means"]["hip_heat_mean"]/before["means"]["hip_heat_mean"]-1))
        else:
            comparisons.append(dict(seed=seed,available=False,reason="物理完成或均值条件不完整，不用剩余成功场景代替完整配对比较"))
    require(sum(row["completed_records_checked"] for row in records)==192,"四策略评价记录不完整")
    f_pass=all(row["gate"]["pass_all"] for row in records if row["route"]=="F")
    result=dict(schema=1,checked_at=datetime.now().astimezone().isoformat(),rows=records,paired_method_comparison=comparisons,
        F_both_seed_screen_pass=f_pass,expansion_authorized=False,old_E_screening_decision_changed=False,
        final_dataset_used=False,completed_records_checked=192,all_four_locked_before_evaluation=True,
        monitor_lock_binding="原监控脚本内部字段为空；保持原始记录，按事前唯一计划及四模型锁执行严格外层核验",
        source_script_sha256=sha(Path(__file__)),source_files=sources,
        next_step="另行事前登记后续研究，保持正式10%/3种子/60例目标" if f_pass else "停止此固定配置，不依据新监控继续改lambda或替换场景",
        boundary="同一新24例中停车/受推均值各用8个单项场景，另8个组合场景参加逐例保护；热指标使用全部24例。仅两个种子的有限配置对照，不作普遍因果或实车承诺。F相对E的百分比以E候选均值为分母，与相对原控制的收益分开。")
    target=base/"四策略完整对照结论.json"
    require(not target.exists(),"完整结论已经保存，拒绝覆盖")
    target.write_bytes((json.dumps(result,ensure_ascii=False,indent=2)+"\n").encode("utf-8"))
    print(json.dumps(dict(F_both_seed_screen_pass=f_pass,rows=[{k:v for k,v in row.items() if k in ("route","seed","gate")} for row in records]),ensure_ascii=False,indent=2))

if __name__=="__main__":
    compare()
