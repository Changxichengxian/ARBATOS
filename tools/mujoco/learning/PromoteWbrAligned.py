"""依据两份完整独立评价复算已登记晋级条件，不修改任何门槛。"""
import argparse
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path

from BuildWbrStudyCases import canonical_bytes
from EvaluateWbrStudy import acceptance, motion_quality_acceptance, load_cases, load_baseline_cache, policy_artifacts
from WbrLearningEnv import ROOT, verify_snapshot, experiment_identity


def read(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def require(value, message):
    if not value:
        raise RuntimeError(message)


def gate(report, criteria):
    checks = report["checks"]
    values = dict(physical_complete=checks["physical_complete"]["pass"],
        original_case_guards=checks["all_case_guards"]["pass"],motion_quality=checks["motion_quality"]["pass"],
        stop_improvement=checks["stop_mean_improvement"].get("improvement_fraction"),
        push_improvement=checks["push_mean_improvement"].get("improvement_fraction"),
        heat_change=checks["hip_heat_mean"].get("change_fraction"))
    finite = all(isinstance(values[key],(int,float)) and math.isfinite(values[key])
                 for key in ("stop_improvement","push_improvement","heat_change"))
    passed = bool(values["physical_complete"] and values["original_case_guards"] and values["motion_quality"]
        and finite
        and values["stop_improvement"] >= criteria["minimum_stop_improvement"]
        and values["push_improvement"] >= criteria["minimum_push_improvement"]
        and values["heat_change"] <= criteria["mean_hip_heat_increase_max"])
    return dict(pass_all=passed,**values)


def evaluate_stage(registration_path, evaluations, baseline_path, stage, approved_at=None):
    require(stage in ("screen_expand","formal"),"未知晋级阶段")
    require(len(evaluations) == 2 and len({str(Path(p).resolve()) for p in evaluations}) == 2,"必须提供两个不同的评价目录")
    registration_path,baseline_path = Path(registration_path).resolve(),Path(baseline_path).resolve()
    rule = read(registration_path)
    manifest = verify_snapshot()
    identity = experiment_identity(manifest)
    registered = "tools/mujoco/learning/results/study-20261012/对齐评分参考策略训练登记.json"
    require(digest(registration_path) == manifest["files"].get(registered),"训练登记没有冻结")
    require(rule["route"] == "E-aligned-reference","训练路线错误")
    criteria = rule["screening_first" if stage == "screen_expand" else "screening_extended"]
    cases_path = ROOT / "tools/mujoco/learning/cases" / ("aligned-20261012/monitor.json" if stage == "screen_expand" else "study-20261012/validation.json")
    cases = load_cases(cases_path)
    require(cases["split"] == criteria["dataset"],"评价场景类别错误")
    if stage == "screen_expand":
        require(cases["cases_sha256"] == rule["monitor_cases_sha256"],"监控场景未登记")
    config = dict(reward_mode="dense",action_mode="reference",gas_force=450.)
    baseline = load_baseline_cache(baseline_path,cases,manifest,identity,config)
    require(all(row["metrics"]["passed"] and "motion_quality" in row for row in baseline),"原控制基线不完整")
    bank_lock = read(ROOT / rule["training_bank_lock"])
    source_names = ("WbrAlignedEnv.py","WbrAlignedReward.py","TrainWbrAligned.py","TrainWbrStudy.py","TrainWbrLearning.py")
    protocol = dict(bank_sha256=bank_lock["bank_sha256"],registration_sha256=digest(registration_path),
        sources={name:manifest["files"]["tools/mujoco/learning/"+name] for name in source_names},
        weights=rule["weights"],algorithm=rule["algorithm"])
    expected_run = dict(library_sha256=manifest["library_sha256"],experiment_identity=identity,
        env_config=config,training_protocol=protocol,requested_total_steps=criteria["steps"],
        envs=rule["envs"],strict_training_checks=True,training_source_sha256=protocol["sources"]["TrainWbrAligned.py"])
    evidence = {str(registration_path):digest(registration_path),str(cases_path):digest(cases_path),str(baseline_path):digest(baseline_path)}
    results,seeds = [],[]
    for directory in map(lambda p:Path(p).resolve(),evaluations):
        summary,run = read(directory / "summary.json"),read(directory / "run.json")
        binding = run["binding"]
        require(hashlib.sha256(canonical_bytes(binding)).hexdigest() == run["binding_sha256"],"评价运行绑定损坏")
        require(summary["run_binding_sha256"] == run["binding_sha256"],"汇总与运行不符")
        for key,value in dict(scenarios_sha256=cases["cases_sha256"],library_sha256=manifest["library_sha256"],experiment_identity=identity,env_config=config,split=cases["split"]).items():
            require(summary[key] == value and binding[key] == value,"评价来源不符："+key)
        require(binding["evaluator_sha256"] == manifest["files"]["tools/mujoco/learning/EvaluateWbrStudy.py"],"评价器源码不符")
        require(binding["baseline_cache_sha256"] == digest(baseline_path),"评价未使用登记的同版原控制基线")
        require(summary["baseline_results"] == baseline,"评价中的原控制被替换")
        policy_directory = Path(binding["policy_directory"])
        require(binding["policy_artifacts"] == policy_artifacts(policy_directory),"实际评价权重变化")
        policy,status,train_run = (read(policy_directory / filename) for filename in ("policy.json","status.json","run.json"))
        require(status["status"] == "completed" and status["steps"] == policy["training_steps"] == criteria["steps"],"必须评价登记预算完成时的最终权重")
        require(policy["kind"] == "ppo", "晋级对象必须是训练网络")
        for key,value in expected_run.items():
            require(train_run.get(key) == value and policy.get(key) == value,"训练权重来源不符："+key)
        require(all(policy.get(key) == value for key,value in train_run.items()),"最终权重与训练运行元信息不符")
        for filename,key in (("checkpoint.zip","checkpoint_sha256"),("policy.npz","policy_sha256")):
            require(digest(policy_directory / filename) == policy[key],"检查点或网络内容变化")
        seeds.append(policy["seed"])
        by_id = {row["id"]:row for row in summary["learned_results"]}
        require(len(by_id) == len(summary["learned_results"]) == cases["case_count"],"学习策略场景缺失或重复")
        for scenario in cases["cases"]:
            row = by_id[scenario["id"]]
            require(all(row[key] == scenario[key] for key in ("id","seed","randomize","options")) and row["controller"] == "learned" and row["case"] == scenario["options"]["case"],"学习场景发生替换")
        recomputed = acceptance(baseline,summary["learned_results"])
        quality = motion_quality_acceptance(baseline,summary["learned_results"])
        recomputed["checks"]["motion_quality"] = quality
        recomputed["overall_pass"] = bool(recomputed["overall_pass"] and quality["pass"])
        require(recomputed == summary["acceptance"],"评价结果不能复算")
        results.append(dict(seed=policy["seed"],evaluation=str(directory),policy=str(policy_directory),gate=gate(recomputed,criteria)))
        for path in [directory / "summary.json",directory / "run.json",*[policy_directory / name for name in ("policy.json","policy.npz","checkpoint.zip","run.json","status.json")]]:
            evidence[str(path)] = digest(path)
    require(sorted(seeds) == sorted(rule["screening_seeds"]),"种子重复或不是两个登记筛选种子")
    moment = datetime.fromisoformat(approved_at) if approved_at else datetime.now().astimezone()
    require(moment.utcoffset() is not None and moment.timestamp() <= datetime.now().timestamp(),"晋级时间无效")
    cutoff = datetime.fromisoformat(rule["latest_screen_expansion" if stage == "screen_expand" else "latest_formal_promotion"])
    return dict(schema=1,stage=stage,eligible=all(row["gate"]["pass_all"] for row in results) and moment <= cutoff,
        approved_at=moment.isoformat(),before_cutoff=moment <= cutoff,registration_sha256=digest(registration_path),
        verifier_sha256=digest(Path(__file__)),criteria=criteria,results=sorted(results,key=lambda row:row["seed"]),
        evidence_files=evidence,baseline_path=str(baseline_path),registration_path=str(registration_path),
        note="按登记条件复算两种子的完整配对结果；既不选中途权重，也不放宽原保护。")


def verify_promotion(path):
    saved = read(path)
    actual = evaluate_stage(saved["registration_path"],[row["evaluation"] for row in saved["results"]],
                            saved["baseline_path"],saved["stage"],saved["approved_at"])
    require(saved == actual and actual["eligible"],"晋级记录未通过完整复算或不符合登记条件")
    return actual


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage",choices=("screen_expand","formal"),required=True)
    parser.add_argument("--registration",type=Path,required=True)
    parser.add_argument("--evaluations",type=Path,nargs=2,required=True)
    parser.add_argument("--baseline",type=Path,required=True)
    parser.add_argument("--output",type=Path,required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    result = evaluate_stage(args.registration,args.evaluations,args.baseline,args.stage)
    args.output.write_bytes((json.dumps(result,ensure_ascii=False,indent=2)+"\n").encode("utf-8"))
    print(json.dumps(dict(eligible=result["eligible"],results=result["results"]),ensure_ascii=False,indent=2))
