"""从已保存轨迹复核轮/髋速度反馈诊断，不运行仿真，也不改验收门槛。"""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import sys

import numpy as np


def audit(snapshot, output, original_baseline):
    snapshot, output = snapshot.resolve(), output.resolve()
    sys.path.insert(0, str(snapshot / "tools/mujoco/learning"))
    from WbrLearningEnv import verify_snapshot, experiment_identity
    from DiagnoseWbrReference import validate_saved_row, registered_deadline
    from EvaluateWbrStudy import (load_cases, summarize_trace, motion_quality_metrics,
                                  acceptance, motion_quality_acceptance)
    read = lambda path: json.loads(path.read_text(encoding="utf-8"))
    digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
    def require(value, message):
        if not value:
            raise RuntimeError(message)

    manifest = verify_snapshot(snapshot)
    learning = snapshot / "tools/mujoco/learning"
    registration_path = learning / "results/study-20261012/轮髋速度反馈拆分诊断登记.json"
    registration = read(registration_path)
    cases = load_cases(learning / "cases/study-20261012/development.json")
    binding = read(output / "run.json")["binding"]
    expected = dict(script_sha256=digest(learning / "DiagnoseWbrReference.py"),
        registration_sha256=digest(registration_path), baseline_sha256=digest(original_baseline),
        cases_sha256=cases["cases_sha256"], library_sha256=manifest["library_sha256"],
        experiment_identity=experiment_identity(manifest), modes=registration["modes"],
        action_mode="split_velocity", deadline=registration["deadline"],
        evaluator_sha256=digest(learning / "EvaluateWbrStudy.py"))
    require(binding == expected, "运行来源与冻结输入不符")
    registered_deadline(registration, binding["deadline"])
    require(cases["split"] == "development" and registration["cases_sha256"] == cases["cases_sha256"],
            "实际场景不是登记的开发集")
    baseline = read(original_baseline)
    require(baseline["scenarios_sha256"] == cases["cases_sha256"], "原基线场景不符")
    equivalent = registration["equivalent_reference"]
    equivalent_path = Path(equivalent["path"])
    require(digest(equivalent_path) == equivalent["sha256"], "旧共享通道结果被改动")
    old_shared = read(equivalent_path)["results"][equivalent["reference_mode"]]
    checked, complete_groups = {}, {}
    modes = registration["modes"]
    require(registration["planned_trajectories"] == len(modes)*len(cases["cases"]), "登记轨迹总数不符")
    # 诊断可继续写入新场景；本次只核对开始时已经原子保存的文件集合。
    available = {path.name for path in output.glob("*-development-*.json")}
    for index, mode in enumerate(modes):
        rows = []
        for scenario in cases["cases"]:
            path = output / f"{mode['name']}-{scenario['id']}.json"
            if path.name not in available:
                continue
            if index > 0:
                require(modes[index-1]["name"] in complete_groups, "前一组未完成就出现后续轨迹")
            row = validate_saved_row(read(path), scenario, mode, binding)
            trace = row.get("trace")
            require(bool(trace), "轨迹缺失：" + path.name)
            target = np.concatenate((np.asarray(mode["action"], dtype=np.float32).astype(float), np.zeros(3)))
            filtered = np.zeros(6)
            previous_time = 0.
            for sample in trace:
                filtered += np.clip(target-filtered, -.2, .2)
                require(np.array_equal(filtered, sample["action"]), "实际动作与登记不符：" + path.name)
                require(sample["time_s"] > previous_time, "轨迹时间不递增：" + path.name)
                previous_time = sample["time_s"]
            require(summarize_trace(scenario, trace, row["metrics"]) == row["metrics"],
                    "窗口指标不能从轨迹复算：" + path.name)
            require(trace[-1]["hip_heat_nm2s"] == row["metrics"]["hip_heat_nm2s"], "热量末值不符")
            if row["metrics"]["passed"]:
                require(motion_quality_metrics(scenario, trace) == row["motion_quality"], "运动质量复算不符")
            checked[path.name] = digest(path)
            rows.append({key:value for key,value in row.items() if key != "trace"})
        if len(rows) == len(cases["cases"]):
            complete_groups[mode["name"]] = rows
            prior = (baseline["baseline_results"] if index == 0 else
                     old_shared if mode["name"] == equivalent["mode"] else None)
            if prior is not None:
                for key in ("seed", "randomize", "case", "options", "metrics", "motion_quality"):
                    require({row["id"]:row[key] for row in rows} == {row["id"]:row[key] for row in prior},
                            mode["name"] + "等价门不符：" + key)
    require(available == set(checked), "存在未登记场景")
    comparisons, compact = {}, {}
    for name, rows in complete_groups.items():
        if name == "zero":
            continue
        result = acceptance(complete_groups["zero"], rows)
        quality = motion_quality_acceptance(complete_groups["zero"], rows)
        result["checks"]["motion_quality"] = quality
        result["overall_pass"] = result["overall_pass"] and quality["pass"]
        comparisons[name] = result
        checks = result["checks"]
        compact[name] = dict(
            stop_improvement=checks["stop_mean_improvement"].get("improvement_fraction"),
            push_improvement=checks["push_mean_improvement"].get("improvement_fraction"),
            heat_change=checks["hip_heat_mean"].get("change_fraction"),
            physical_complete=checks["physical_complete"]["pass"],
            all_case_guards=checks["all_case_guards"]["pass"], motion_quality=quality["pass"],
            all_development_gates=result["overall_pass"],
            failed_case_guards={row["id"]:[key for key,item in row["checks"].items() if not item["pass"]]
                                for row in result["cases"] if not row["pass"]},
            failed_motion_quality={row["id"]:[key for key,item in row["checks"].items() if not item["pass"]]
                                    for row in quality["cases"] if not row["pass"]})
    complete = (len(complete_groups) == len(modes) and (output / "summary.json").is_file()
                and read(output / "status.json")["status"] == "completed")
    if complete:
        summary = read(output / "summary.json")
        require(summary["binding"] == binding and summary["results"] == complete_groups and
                summary["comparisons"] == comparisons, "完整汇总与逐场景复算不同")
        require(read(output / "status.json")["status"] == "completed", "结果齐全但诊断尚未确认完成")
    return dict(schema=1, audited_at=datetime.now().astimezone().isoformat(), complete=complete,
        checked_trajectories=len(checked), expected_trajectories=registration["planned_trajectories"],
        binding=binding, source_run_sha256=digest(output / "run.json"),
        auditor_sha256=digest(Path(__file__)), result_files=checked,
        equivalent_gates=dict(zero="zero" in complete_groups,
                              shared=equivalent["mode"] in complete_groups),
        comparison_summary=compact, comparisons=comparisons,
        source_summary_sha256=digest(output / "summary.json") if complete else None,
        note="窗口/运动质量从15ms轨迹复算；1ms物理峰值和失败检查沿用原记录，不声称从低频轨迹重建。开发诊断不代替独立验证或最终验收。")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--snapshot", type=Path, required=True)
    parser.add_argument("--results", type=Path, required=True)
    parser.add_argument("--original-baseline", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = audit(args.snapshot, args.results, args.original_baseline)
    args.output.write_bytes((json.dumps(result, ensure_ascii=False, indent=2) + "\n").encode("utf-8"))
    print(json.dumps({key:result[key] for key in ("complete", "checked_trajectories", "equivalent_gates", "comparison_summary")},
                     ensure_ascii=False, indent=2))
