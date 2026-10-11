"""读取已完成的筛选结果，按预先声明的门槛给出是否扩展训练的建议。"""
import argparse
import hashlib
import json
from pathlib import Path


def screening_gate(report):
    checks = report["checks"]
    means = [checks[name].get("improvement_fraction", -1.) for name in
             ("stop_mean_improvement", "push_mean_improvement")]
    primary_ok = all(row["complete"] and all(
        item["delta"] <= max(abs(item["baseline"])*.10, .001)
        for name,item in row["checks"].items() if name.endswith("_primary"))
        for row in report["cases"])
    gates = dict(physical_complete=checks["physical_complete"]["pass"],
                 stop_improvement=means[0] >= .05, push_improvement=means[1] >= .05,
                 primary_case_regression=primary_ok,
                 heat_mean=checks["hip_heat_mean"].get("change_fraction", 1.) <= .08)
    return dict(pass_all=all(gates.values()), checks=gates,
                stop_improvement=means[0], push_improvement=means[1],
                heat_change=checks["hip_heat_mean"].get("change_fraction"),
                final_thresholds_pass=report["overall_pass"],
                failing_final_case_guards=[row["id"] for row in report["cases"] if not row["pass"]])


def summarize(queue_path):
    queue_path = Path(queue_path)
    state = json.loads(queue_path.read_text(encoding="utf-8"))
    candidates = []
    for job_id, job in state["jobs"].items():
        if not job_id.endswith("-evaluate") or job["status"] != "completed":
            continue
        summary_path = Path(job["actual_output"])/"summary.json"
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
        if summary["split"] != "validation":
            continue
        route, seed = job_id.removesuffix("-evaluate").rsplit("-",1)
        candidates.append(dict(route=route, seed=int(seed),
            summary_sha256=hashlib.sha256(summary_path.read_bytes()).hexdigest(),
            summary_path=str(summary_path), gate=screening_gate(summary["acceptance"])))
    routes = []
    for name in ("A-full", "B-symmetric", "C-linear"):
        rows = [row for row in candidates if row["route"] == name]
        complete = {row["seed"] for row in rows} == {41,73}
        eligible = complete and all(row["gate"]["pass_all"] for row in rows)
        worst = min((min(row["gate"]["stop_improvement"],row["gate"]["push_improvement"])
                     for row in rows), default=None)
        routes.append(dict(route=name, screening_complete=complete, eligible=eligible,
                           worst_seed_primary_improvement=worst))
    eligible = sorted((row for row in routes if row["eligible"]),
                      key=lambda row:row["worst_seed_primary_improvement"], reverse=True)
    return dict(schema=1, queue_status=state["status"], candidates=candidates, routes=routes,
                recommended_route=eligible[0]["route"] if eligible else None,
                next_action=("从头训练101、211、307并冻结最终策略" if eligible else
                             "完成剩余筛选；若全部完成仍无通过方案，分析失败样本与动作效果，保留负结果"),
                note="筛选通过仅允许扩大训练，不代表最终验收或实车通过")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--queue", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    result = summarize(args.queue)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_bytes((json.dumps(result, ensure_ascii=False, indent=2)+"\n").encode("utf-8"))
    print(json.dumps({key:result[key] for key in ("queue_status","routes","recommended_route","next_action")}, ensure_ascii=False, indent=2))
