"""在开发集搜索六个线性反馈系数，作为强化学习的简单对照。"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import hashlib
from pathlib import Path
import time

import numpy as np

from BuildWbrStudyCases import canonical_bytes
from EvaluateWbrStudy import EvaluationInterrupted, acceptance, load_cases, run_case
from StudyWbrPolicy import linear_action
from WbrLearningEnv import experiment_identity, verify_snapshot


BOUNDS = np.asarray([4., 2., 4., .5, 2., .25])
CONFIG = dict(reward_mode="dense", action_mode="symmetric", gas_force=450.)


def write_json(path, value):
    path = Path(path)
    temporary = path.with_suffix(path.suffix+".tmp")
    temporary.write_bytes((json.dumps(value, ensure_ascii=False, indent=2)+"\n").encode("utf-8"))
    temporary.replace(path)


class LinearCandidate:
    def __init__(self, coefficients):
        self.coefficients = coefficients

    def predict(self, observation):
        return linear_action(observation, self.coefficients)


def score_result(baseline, learned):
    report = acceptance(baseline, learned)
    if not report["checks"]["physical_complete"]["pass"]:
        return -1000.-sum(not row["metrics"]["passed"] for row in learned), report
    checks = report["checks"]
    improvements = [checks[key]["improvement_fraction"] for key in
                    ("stop_mean_improvement", "push_mean_improvement")]
    heat = checks["hip_heat_mean"]["change_fraction"]
    # 所有场景等权；差的单场景及角度/热量等保护项会扣分。
    primary_excess, secondary_excess = 0., 0.
    for row in report["cases"]:
        for name, check in row["checks"].items():
            excess = max(0., check["delta"]-check["allowed_increase"])
            normalized = excess/max(abs(check["baseline"]), check["allowed_increase"], 1e-6)
            if name.endswith("_primary"):
                primary_excess = max(primary_excess, normalized)
            else:
                secondary_excess = max(secondary_excess, normalized)
    value = (min(improvements)+.25*float(np.mean(improvements))
             -2.*primary_excess-2.*max(0., heat-.05)-.2*secondary_excess)
    return float(value), report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cases", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--seed", type=int, default=41)
    parser.add_argument("--generations", type=int, default=5)
    parser.add_argument("--population", type=int, default=8)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--deadline", default="2026-10-14T02:10:00+08:00")
    args = parser.parse_args()
    if args.generations < 1 or args.population < 4:
        parser.error("至少一代、每代至少四个候选")
    cases = load_cases(args.cases)
    if cases["split"] != "development":
        raise RuntimeError("搜索只允许使用开发集，不能使用验证集或最终集")
    manifest = verify_snapshot()
    identity = experiment_identity(manifest)
    binding = dict(seed=args.seed, scenarios_sha256=cases["cases_sha256"],
                   library_sha256=manifest["library_sha256"], experiment_identity=identity,
                   env_config=CONFIG, generations=args.generations, population=args.population,
                   coefficient_bounds=BOUNDS.tolist(), source_hashes={name:hashlib.sha256(
                       Path(__file__).with_name(name).read_bytes()).hexdigest() for name in
                       ("SearchWbrLinear.py", "EvaluateWbrStudy.py", "BuildWbrStudyCases.py")})
    deadline = datetime.fromisoformat(args.deadline).timestamp() if args.deadline else None
    output = args.output.resolve()
    state_path = output/"search-state.json"
    rng = np.random.default_rng(args.seed)
    if args.resume:
        state = json.loads(state_path.read_text(encoding="utf-8"))
        if canonical_bytes(state["binding"]) != canonical_bytes(binding):
            raise RuntimeError("续跑搜索的配置或来源不一致")
        if state["status"] == "completed":
            raise RuntimeError("搜索已完成，无需重复")
        rng.bit_generator.state = state["rng_state"]
    else:
        output.mkdir(parents=True, exist_ok=False)
        state = dict(binding=binding, status="running", baseline=[], generation=0,
                     population=[], evaluations=[], best=None,
                     mean=[0.]*6, std=(BOUNDS*.5).tolist(), rng_state=rng.bit_generator.state)

    def save():
        state["rng_state"] = rng.bit_generator.state
        state["updated_at"] = datetime.now(timezone.utc).isoformat()
        write_json(state_path, state)
        write_json(output/"status.json", dict(status=state["status"], generation=state["generation"],
                   evaluations=len(state["evaluations"]), updated_at=state["updated_at"]))
        if state["best"] is not None:
            write_json(output/"policy.json", dict(kind="linear", **binding,
                       coefficients=state["best"]["coefficients"],
                       development_score=state["best"]["score"], created_at=state["updated_at"]))

    def should_stop():
        return (output/"stop.request").exists() or bool(deadline and time.time() >= deadline)

    try:
        state["status"] = "running"
        save()
        while len(state["baseline"]) < len(cases["cases"]):
            if should_stop():
                state["status"] = "interrupted"
                save()
                return
            state["baseline"].append(run_case(cases["cases"][len(state["baseline"])], None, CONFIG,
                                             stop_requested=should_stop))
            save()
        if not all(row["metrics"]["passed"] for row in state["baseline"]):
            raise RuntimeError("开发集原控制存在物理失败，不能开始系数搜索")
        if state["best"] is None:
            state["best"] = dict(coefficients=[0.]*6, score=0., generation=-1)
        while state["generation"] < args.generations:
            if not state["population"]:
                center, spread = np.asarray(state["mean"]), np.asarray(state["std"])
                samples = np.clip(rng.normal(center, spread, (args.population,6)), -BOUNDS, BOUNDS)
                samples[0] = state["best"]["coefficients"]
                state["population"] = [dict(coefficients=row.tolist(), results=[]) for row in samples]
                save()
            for index, candidate in enumerate(state["population"]):
                if "score" in candidate:
                    continue
                policy = LinearCandidate(candidate["coefficients"])
                while len(candidate["results"]) < len(cases["cases"]):
                    if should_stop():
                        state["status"] = "interrupted"
                        save()
                        return
                    case = cases["cases"][len(candidate["results"])]
                    candidate["results"].append(run_case(case, policy, CONFIG, stop_requested=should_stop))
                    save()
                candidate["score"], report = score_result(state["baseline"], candidate["results"])
                entry = dict(generation=state["generation"], index=index,
                             coefficients=candidate["coefficients"], score=candidate["score"],
                             checks=report["checks"])
                state["evaluations"].append(entry)
                if candidate["score"] > state["best"]["score"]:
                    state["best"] = entry
                print(json.dumps(entry, ensure_ascii=False), flush=True)
                save()
            elite = sorted(state["population"], key=lambda row:row["score"], reverse=True)[:max(2,args.population//4)]
            values = np.asarray([row["coefficients"] for row in elite])
            state["mean"] = values.mean(axis=0).tolist()
            state["std"] = np.maximum(values.std(axis=0), BOUNDS*.08).tolist()
            state["population"] = []
            state["generation"] += 1
            save()
        state["status"] = "completed"
        save()
    except EvaluationInterrupted:
        state["status"] = "interrupted"
        save()
    except Exception as error:
        state.update(status="failed", error=repr(error))
        save()
        raise


if __name__ == "__main__":
    main()
