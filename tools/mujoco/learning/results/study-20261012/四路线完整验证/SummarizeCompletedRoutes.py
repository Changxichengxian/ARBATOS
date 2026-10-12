"""汇总已完成A/B/C/D的相同45例，复核统计口径、基线和720条完成记录。"""
from collections import Counter
from datetime import datetime
import hashlib
import json
from pathlib import Path
import sys

workspace=Path(__file__).resolve().parents[1]
study=workspace/"work/study-20261012"
repo=Path(r"D:\ARBATOS")
results=repo/"tools/mujoco/learning/results/study-20261012"
sys.path.insert(0,str(study/"aligned-reference/training-frozen/tools/mujoco/learning"))
from EvaluateWbrStudy import acceptance,load_cases,_completed_path,policy_artifacts
from BuildWbrStudyCases import canonical_bytes
from WbrLearningEnv import verify_snapshot

read=lambda p:json.loads(p.read_text(encoding="utf-8"))
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def require(ok,message):
    if not ok:
        raise RuntimeError(message)

cases=load_cases(study/"frozen/tools/mujoco/learning/cases/study-20261012/validation.json")
require(cases["case_count"]==45,"不是45例验证")
definitions=[("A","六维修正","screening-A-full-{seed}-evaluated","screening","A-full-{seed}","train"),
    ("B","对称动作","screening-B-symmetric-{seed}","screening","B-symmetric-{seed}","train"),
    ("C","线性修正","linear-C-{seed}-validation","screening","C-linear-{seed}","search"),
    ("D","常数参考参数","reference-D-{seed}-validation","reference-search/screening","D-reference-{seed}","search")]
original_manifest=verify_snapshot(study/"frozen")
physical_files={key:value for key,value in original_manifest["files"].items() if not key.startswith("tools/mujoco/learning/")}
for snapshot in (study/"reference-search/frozen",study/"aligned-reference/training-frozen"):
    manifest=verify_snapshot(snapshot)
    require(all(manifest["files"].get(name)==value for name,value in physical_files.items()),"共同机械/原控制源改变")
rows=[]
common_baseline=None
completed_records=0
for route,label,relative,queue_relative,job_prefix,kind in definitions:
    queue_path=study/queue_relative/"queue-state.json"
    queue=read(queue_path)
    require(queue["status"]=="completed","原定队列未完成")
    for seed in (41,73):
        directory=results/relative.format(seed=seed)
        summary_path=directory/"summary.json"
        run_path=directory/("evaluation-run.json" if route in ("A","B") else "run.json")
        report,run=read(summary_path),read(run_path)
        binding=run["binding"]
        prefix=job_prefix.format(seed=seed)
        evaluated=queue["jobs"][prefix+"-evaluate"]
        trained=queue["jobs"][prefix+"-"+kind]
        require(evaluated["status"]==trained["status"]=="completed","对应任务未完成")
        original=Path(evaluated["actual_output"])
        require(sha(original/"summary.json")==sha(summary_path) and sha(original/"run.json")==sha(run_path),"保存结果与实际队列不一致")
        require(run["binding_sha256"]==hashlib.sha256(canonical_bytes(binding)).hexdigest()
            and report["run_binding_sha256"]==run["binding_sha256"],"运行绑定不一致")
        require(report["scenarios_sha256"]==binding["scenarios_sha256"]==cases["cases_sha256"]
            and report["split"]==binding["split"]=="validation","不是相同验证场景")
        require(binding["policy_artifacts"]==policy_artifacts(Path(trained["actual_output"])),"实际策略来源变化")
        for controller in ("baseline","learned"):
            candidates=report[controller+"_results"]
            by_id={row["id"]:row for row in candidates}
            require(len(by_id)==len(candidates)==45,"场景缺失或重复")
            for scenario in cases["cases"]:
                row=by_id[scenario["id"]]
                require(all(row[key]==scenario[key] for key in ("id","seed","randomize","options"))
                    and row["controller"]==controller,"完成场景错配")
                done=read(_completed_path(original,controller,scenario["id"]))
                require(done["row"]==row and done["binding_sha256"]==run["binding_sha256"],"完成记录与汇总不一致")
                completed_records+=1
        baseline={row["id"]:{key:value for key,value in row.items() if key not in ("controller","motion_quality")}
            for row in report["baseline_results"]}
        if common_baseline is None:
            common_baseline=baseline
        require(baseline==common_baseline,"八组的共同原控制指标不是逐字段相同")
        original_checks=acceptance(report["baseline_results"],report["learned_results"])
        require(original_checks["cases"]==report["acceptance"]["cases"]
            and original_checks["thresholds"]==report["acceptance"]["thresholds"]
            and all(report["acceptance"]["checks"][key]==value for key,value in original_checks["checks"].items()),"原逐例和均值验收不能复算")
        checks=original_checks["checks"]
        failures=[row for row in original_checks["cases"] if not row["pass"]]
        guard_counts=Counter(name for row in failures for name,check in row["checks"].items() if not check["pass"])
        elapsed=[]
        for attempt in trained["attempts"]:
            require("finished_at" in attempt,"运行耗时尚未封闭")
            elapsed.append((datetime.fromisoformat(attempt["finished_at"])-datetime.fromisoformat(attempt["started_at"])).total_seconds())
        quality=report["acceptance"]["checks"].get("motion_quality")
        conclusions=read(directory/"筛选结论.json")
        passed=(all(conclusions["screening_checks"].values()) if route=="D" else
            conclusions.get("screening_gate",conclusions.get("gate",{})).get("pass_all"))
        require(passed is False,"路线结论并非已核验负结果："+prefix)
        rows.append(dict(route=route,label=label,seed=seed,validation_pairs=45,
            stop_improvement=checks["stop_mean_improvement"]["improvement_fraction"],
            push_improvement=checks["push_mean_improvement"]["improvement_fraction"],
            hip_heat_change=checks["hip_heat_mean"]["change_fraction"],
            mean_case_counts=dict(stop=checks["stop_mean_improvement"]["paired_cases"],push=checks["push_mean_improvement"]["paired_cases"],heat=45),
            physical_complete=checks["physical_complete"]["pass"],original_guard_failing_cases=len(failures),
            original_guard_counts=dict(guard_counts),failed_case_ids=[row["id"] for row in failures],
            failures_by_case_kind=dict(Counter(row["case"] for row in failures)),
            extra_motion_quality="not_evaluated" if quality is None else ("passed" if quality["pass"] else "failed"),
            screening_pass=False,training_or_search_wall_seconds=sum(elapsed),execution_attempts=len(elapsed),
            evaluation_wall_seconds=report["elapsed_s"],budget=dict(kind="PPO_steps",count=131072) if route in ("A","B") else dict(kind="candidate_parameters",count=40 if route=="C" else 72),
            source={str(p):sha(p) for p in (summary_path,run_path,directory/"筛选结论.json")},
            queue_source=str(queue_path),training_or_search_output=trained["actual_output"]))
require(completed_records==720 and all(row["mean_case_counts"]==dict(stop=18,push=18,heat=45) for row in rows),"统计口径发生变化")
record=dict(schema=1,verified_at=datetime.now().astimezone().isoformat(),rows=rows,
    candidates_evaluated=360,completed_record_rows_checked=completed_records,
    common_baseline_45_metrics_exact=True,common_nonlearning_source_files=len(physical_files),
    validation_cases_sha256=cases["cases_sha256"],physical_controller_sha256=physical_files["tools/mujoco/reference/OwnWbrBridge.c"],
    interpretation=["停车和受推的平均收益分别用18个单项场景；9个组合场景参加逐例约束，45例全部参加平均髋部热指标。",
        "平均收益是原控制与候选均值之比，不是逐例百分比的简单平均。",
        "原逐例保护含最终5%主要误差和姿态/峰值/用力等检查；这些失败数不能等同于物理失败数。",
        "A/B/C未达到原主要收益筛选时不追加运动质量评价，未评价不应标成已证实质量失败。",
        "四类预算和搜索方法不同，耗时包含同机并行任务的竞争，仅作实际工作量记录，不能视为严格等算力排名。",
        "E旧24例和F新24例监控属于另一批场景，不并入本45例比较。"],
    source_script_sha256=sha(Path(__file__)))
target=study/"四路线完整对照核验.json"
require(not target.exists(),"已有本次完整对照核验")
target.write_bytes((json.dumps(record,ensure_ascii=False,indent=2)+"\n").encode("utf-8"))
print(json.dumps(dict(checked_records=completed_records,common_baseline_exact=True,rows=[{key:row[key] for key in
    ("label","seed","stop_improvement","push_improvement","original_guard_failing_cases","training_or_search_wall_seconds")} for row in rows]),ensure_ascii=False,indent=2))
