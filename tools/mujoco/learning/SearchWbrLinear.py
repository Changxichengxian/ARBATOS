"""在开发集搜索线性修正或三项参考参数，作为强化学习的控制对照。"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import hashlib
import os
from pathlib import Path
import time

import numpy as np

from BuildWbrStudyCases import canonical_bytes
from EvaluateWbrStudy import EvaluationInterrupted, acceptance, load_cases, run_case, motion_quality_acceptance
from StudyWbrPolicy import linear_action, reference_action
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


class ReferenceCandidate:
    def __init__(self, coefficients):
        self.value = reference_action(coefficients)

    def predict(self, observation):
        return self.value.copy()


def reference_score(baseline, learned):
    report = acceptance(baseline,learned)
    checks = report["checks"]
    quality = motion_quality_acceptance(baseline,learned)
    checks["motion_quality"] = quality
    report["overall_pass"] = report["overall_pass"] and quality["pass"]
    if not checks["physical_complete"]["pass"]:
        return -1e9-sum(not row["metrics"]["passed"] for row in learned),report
    improvements = [checks[key]["improvement_fraction"] for key in
                    ("stop_mean_improvement","push_mean_improvement")]
    feasible = checks["all_case_guards"]["pass"] and checks["hip_heat_mean"]["pass"] and quality["pass"]
    if feasible:
        # 先满足所有保护，再比较两类任务中较差的改善；完整10%门优先。
        worst = min(improvements)
        score = worst if worst <= 0. else float(report["overall_pass"])+worst+.001*float(np.mean(improvements))
        return score,report
    violations = [max(0.,checks["hip_heat_mean"]["change_fraction"]-.05)]
    for group in (report["cases"],quality["cases"]):
        for row in group:
            for check in row["checks"].values():
                excess = max(0.,check["delta"]-check["allowed_increase"])
                violations.append(excess/max(abs(check["baseline"]),check["allowed_increase"],1e-6))
    return -1.-max(violations)-.1*sum(violations),report


def state_hash(state):
    return hashlib.sha256(canonical_bytes({key:value for key,value in state.items()
                                         if key != "state_sha256"})).hexdigest()


def validate_rows(rows,cases):
    if len(rows) > len(cases):
        raise RuntimeError("续跑缓存的场景数量过多")
    for row,scenario in zip(rows,cases):
        expected = dict(id=scenario["id"],seed=scenario["seed"],randomize=scenario["randomize"],
                        case=scenario["options"]["case"],options=scenario["options"])
        if any(row.get(key) != value for key,value in expected.items()):
            raise RuntimeError("续跑缓存场景或顺序不匹配")


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
    parser.add_argument("--family", choices=("linear","reference"), default="linear")
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
    is_reference = args.family == "reference"
    config = dict(CONFIG,action_mode="reference") if is_reference else CONFIG
    low = np.array([0.,-1.,-1.]) if is_reference else -BOUNDS
    high = np.ones(3) if is_reference else BOUNDS
    initial_mean = [.08,-.5,.5] if is_reference else [0.]*6
    initial_std = [.08,.3,.25] if is_reference else (BOUNDS*.5).tolist()
    minimum_std = np.array([.025,.08,.08]) if is_reference else BOUNDS*.08
    factory = ReferenceCandidate if is_reference else LinearCandidate
    scorer = reference_score if is_reference else score_result
    dimension = len(low)
    binding = dict(seed=args.seed, scenarios_sha256=cases["cases_sha256"],
                   library_sha256=manifest["library_sha256"], experiment_identity=identity,
                   env_config=config, generations=args.generations, population=args.population,
                   family=args.family, coefficient_lower=low.tolist(),coefficient_upper=high.tolist(),
                   initial_mean=initial_mean,initial_std=initial_std,minimum_std=minimum_std.tolist(),
                   source_hashes={name:hashlib.sha256(
                       Path(__file__).with_name(name).read_bytes()).hexdigest() for name in
                       ("SearchWbrLinear.py", "EvaluateWbrStudy.py", "BuildWbrStudyCases.py")})
    deadline = datetime.fromisoformat(args.deadline).timestamp() if args.deadline else None
    output = args.output.resolve()
    state_path = output/"search-state.json"
    rng = np.random.default_rng(args.seed)
    if not args.resume:
        output.mkdir(parents=True, exist_ok=False)
    lock_stream = (output/"search.lock").open("a+b")
    lock_stream.seek(0)
    if not lock_stream.read(1):
        lock_stream.write(b"0")
        lock_stream.flush()
    lock_stream.seek(0)
    if os.name == "nt":
        import msvcrt
        msvcrt.locking(lock_stream.fileno(),msvcrt.LK_NBLCK,1)
    else:
        import fcntl
        fcntl.flock(lock_stream.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
    if args.resume and state_path.exists():
        state = json.loads(state_path.read_text(encoding="utf-8"))
        if state.get("state_sha256") != state_hash(state):
            raise RuntimeError("搜索状态内容哈希不匹配")
        if canonical_bytes(state["binding"]) != canonical_bytes(binding):
            raise RuntimeError("续跑搜索的配置或来源不一致")
        if state["status"] == "completed":
            raise RuntimeError("搜索已完成，无需重复")
        validate_rows(state["baseline"],cases["cases"])
        for candidate in state["population"]:
            validate_rows(candidate["results"],cases["cases"])
        for rows in state["cache"].values():
            validate_rows(rows,cases["cases"])
        rng.bit_generator.state = state["rng_state"]
    else:
        if any(path.name not in ("search.lock","stop.request") for path in output.iterdir()):
            raise RuntimeError("搜索目录缺少状态，但包含其他产物")
        state = dict(binding=binding, status="running", baseline=[], generation=0,
                     population=[], evaluations=[], best=None,cache={},
                     mean=initial_mean, std=initial_std, rng_state=rng.bit_generator.state)

    def save():
        state["rng_state"] = rng.bit_generator.state
        state["updated_at"] = datetime.now(timezone.utc).isoformat()
        state["state_sha256"] = state_hash(state)
        write_json(state_path, state)
        write_json(output/"status.json", dict(status=state["status"], generation=state["generation"],
                   evaluations=len(state["evaluations"]),pid=os.getpid(),updated_at=state["updated_at"]))
        if state["best"] is not None:
            write_json(output/"policy.json", dict(kind="reference_constant" if is_reference else "linear", **binding,
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
            state["baseline"].append(run_case(cases["cases"][len(state["baseline"])], None, config,
                                             stop_requested=should_stop))
            save()
        if not all(row["metrics"]["passed"] for row in state["baseline"]):
            raise RuntimeError("开发集原控制存在物理失败，不能开始系数搜索")
        if state["best"] is None:
            state["best"] = dict(coefficients=[0.]*dimension, score=0., generation=-1)
            state["cache"][canonical_bytes([0.]*dimension).decode()] = state["baseline"]
        while state["generation"] < args.generations:
            if not state["population"]:
                center, spread = np.asarray(state["mean"]), np.asarray(state["std"])
                samples = np.clip(rng.normal(center,spread,(args.population,dimension)),low,high)
                if is_reference:
                    # 留出全边界探索，其余在当前较好参数附近采样。
                    samples[-2:] = rng.uniform(low,high,(2,dimension))
                    samples[1] = [0.,-.5,0.] if state["generation"] == 0 else np.zeros(dimension)
                    if state["generation"] == 0:
                        samples[2] = initial_mean
                samples[0] = state["best"]["coefficients"]
                state["population"] = [dict(coefficients=row.tolist(), results=[]) for row in samples]
                save()
            for index, candidate in enumerate(state["population"]):
                if "score" in candidate:
                    continue
                policy = factory(candidate["coefficients"])
                cache_key = canonical_bytes(candidate["coefficients"]).decode()
                if not candidate["results"] and cache_key in state["cache"]:
                    candidate["results"] = state["cache"][cache_key]
                while len(candidate["results"]) < len(cases["cases"]):
                    if should_stop():
                        state["status"] = "interrupted"
                        save()
                        return
                    case = cases["cases"][len(candidate["results"])]
                    candidate["results"].append(run_case(case, policy, config, stop_requested=should_stop))
                    save()
                candidate["score"], report = scorer(state["baseline"], candidate["results"])
                state["cache"][cache_key] = candidate["results"]
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
            state["std"] = np.maximum(values.std(axis=0),minimum_std).tolist()
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
