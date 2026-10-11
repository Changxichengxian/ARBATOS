"""只使用已公开的开发诊断校准新训练评分；不读取独立验证或最终结果。"""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path

import numpy as np

from EvaluateWbrStudy import acceptance, motion_quality_acceptance
from WbrAlignedReward import score_trace


def calibrate(reference_results, reference_evidence, split_results, split_evidence, search_replay=None):
    read = lambda p: json.loads(p.read_text(encoding="utf-8"))
    digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    groups, sources = {}, {}
    for label, directory, evidence_path in (("reference", reference_results, reference_evidence),
                                            ("split", split_results, split_evidence)):
        evidence = read(evidence_path)
        inventory = (evidence["result_audit"]["result_files"] if label == "reference"
                     else evidence["result_files"])
        run = read(directory / "run.json")["binding"]
        sources[label] = dict(evidence_sha256=digest(evidence_path), files=inventory)
        for mode in run["modes"]:
            rows = []
            for name, expected in inventory.items():
                if not name.startswith(mode["name"]+"-development-"):
                    continue
                path = directory / name
                if digest(path) != expected:
                    raise RuntimeError("开发轨迹内容变化："+name)
                saved = read(path)
                row = saved.get("result", saved)
                if not row["id"].startswith("development-") or not row["metrics"]["passed"]:
                    raise RuntimeError("校准仅接受本批完整开发轨迹")
                rows.append(row)
            if len(rows) != 6 or len({row["id"] for row in rows}) != 6:
                raise RuntimeError("开发组不完整")
            groups[label+"/"+mode["name"]] = sorted(rows, key=lambda row:row["id"])
    baseline = groups["reference/zero"]
    by_base = {row["id"]:row for row in baseline}
    heat_scale = float(np.mean([row["metrics"]["hip_heat_nm2s"] for row in baseline]))
    for row in groups["split/zero"]:
        for key in ("seed", "randomize", "case", "options", "metrics", "motion_quality"):
            if row[key] != by_base[row["id"]][key]:
                raise RuntimeError("两批原控制不一致")
    if search_replay is not None:
        replay = read(search_replay)
        if replay.get("matches_search_cache_exactly") is not True or replay["binding"] != read(search_replay.parent / "run.json"):
            raise RuntimeError("搜索候选重放来源不符")
        rows = []
        compact = {row["id"]:row for row in replay["results"]}
        for name, expected in replay["result_files"].items():
            path = search_replay.parent / name
            if digest(path) != expected:
                raise RuntimeError("搜索重放轨迹内容变化")
            row = read(path)
            if {key:value for key,value in row.items() if key != "trace"} != compact[row["id"]]:
                raise RuntimeError("搜索重放轨迹与汇总不符")
            if row["id"] not in by_base or any(row[key] != by_base[row["id"]][key] for key in ("seed", "randomize", "case", "options")):
                raise RuntimeError("搜索重放不是相同开发场景")
            rows.append(row)
        if len(rows) != 6 or len({row["id"] for row in rows}) != 6:
            raise RuntimeError("搜索候选开发重放不完整")
        groups["search/D41"] = sorted(rows,key=lambda row:row["id"])
        sources["search_D41"] = dict(summary_sha256=digest(search_replay),
            run_sha256=digest(search_replay.parent / "run.json"), files=replay["result_files"],
            policy_sha256=replay["binding"]["policy_sha256"], coefficients=replay["binding"]["coefficients"])
    labels = {}
    for name, rows in groups.items():
        result = acceptance(baseline, rows)
        checks = result["checks"]
        feasible_improving = (checks["physical_complete"]["pass"] and checks["all_case_guards"]["pass"] and
            checks["hip_heat_mean"]["pass"] and motion_quality_acceptance(baseline, rows)["pass"] and
            checks["stop_mean_improvement"]["improvement_fraction"] > 0 and
            checks["push_mean_improvement"]["improvement_fraction"] > 0)
        labels[name] = "zero" if name.endswith("/zero") else "feasible_improving" if feasible_improving else "not_preferred"
    if "feasible_improving" not in labels.values():
        raise RuntimeError("缺少可行改善的评分对照")
    trials = []
    # 10% RMS约对应19%平方误差改善，与5%热量预算同量级的系数取4。
    # 只在公开开发记录上选择满足排序的最小保护罚项，不把排序当独立验证。
    for guard in (2., 5., 10.):
        weights = dict(primary=1., drive=.2, heat=4., pose=.02,
                       action_change=.02, action_magnitude=.001, guard=guard)
        scores = {}
        for name, rows in groups.items():
            detailed = []
            for row in rows:
                scenario = {key:row[key] for key in ("id", "seed", "randomize", "options")}
                value = score_trace(scenario, by_base[row["id"]], row, heat_scale, weights)
                detailed.append(dict(id=row["id"], **value))
            scores[name] = dict(label=labels[name], cases=detailed,
                return_sum=float(np.mean([row["return_sum"] for row in detailed])),
                discounted_return=float(np.mean([row["discounted_return"] for row in detailed])))
        checks = {}
        for metric in ("return_sum", "discounted_return"):
            zero = scores["reference/zero"][metric]
            checks[metric] = (abs(scores["split/zero"][metric]-zero) <= 1e-12 and
                all(value[metric] > zero+1e-4 if value["label"] == "feasible_improving"
                    else value[metric] < zero-1e-4 for value in scores.values() if value["label"] != "zero"))
        trials.append(dict(weights=weights, ordering_checks=checks, scores=scores))
        if all(checks.values()):
            break
    selected = trials[-1]["weights"] if all(trials[-1]["ordering_checks"].values()) else None
    return dict(schema=1, calibrated_at=datetime.now().astimezone().isoformat(), selected_weights=selected,
        gamma=.999, development_heat_scale=heat_scale, sources=sources, trials=trials,
        reward_source_sha256=digest(Path(__file__).with_name("WbrAlignedReward.py")),
        calibration_source_sha256=digest(Path(__file__)),
        scope="训练评分开发校准：排序使用已知开发结果，不是独立验证，不宣称更好训练性能；未读取45例验证或60例最终集。",
        ordering="可行且两主指标改善的已知组 > 原控制 > 已知不合格或退步组；同时检查未折扣和gamma=.999折扣回报。",
        limitations="终端罚项只是训练信号；并不保证学习策略满足约束，仍须逐项通过原独立评价。热量4权重由19%平方误差/5%热预算量级设计，最终均值热上限仍为5%。")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("reference-results", "reference-evidence", "split-results", "split-evidence", "output"):
        parser.add_argument("--"+name, type=Path, required=True)
    parser.add_argument("--search-replay", type=Path)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError("校准输出已存在，拒绝覆盖")
    result = calibrate(args.reference_results, args.reference_evidence, args.split_results, args.split_evidence, args.search_replay)
    args.output.write_bytes((json.dumps(result, ensure_ascii=False, indent=2)+"\n").encode("utf-8"))
    print(json.dumps(dict(selected_weights=result["selected_weights"], trials=[
        dict(weights=t["weights"], checks=t["ordering_checks"], scores={n:{k:v[k] for k in ("label", "return_sum", "discounted_return")}
            for n,v in t["scores"].items()}) for t in result["trials"]]), ensure_ascii=False, indent=2))
