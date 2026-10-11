"""在固定开发集测量速度参考、观察器误差和受限修正的作用，不读取最终集。"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys
import time

import numpy as np


def window_metrics(trace, start, end):
    rows = [row for row in trace if start-1e-9 <= row["time_s"] <= end+1e-9]
    if not rows:
        return None
    velocity = np.array([row["true_v_mps"] for row in rows])
    reference = np.array([row["control_v_reference_mps"] for row in rows])
    command = np.array([row["command_v_mps"] for row in rows])
    observed = np.array([row["observed_v_mps"] for row in rows])
    rms = lambda value: float(np.sqrt(np.mean(np.square(value))))
    return dict(samples=len(rows), speed_rms_mps=rms(velocity),
        requested_tracking_rms_mps=rms(velocity-command),
        internal_reference_tracking_rms_mps=rms(velocity-reference),
        internal_reference_rms_mps=rms(reference), observer_error_rms_mps=rms(observed-velocity),
        mean_speed_mps=float(velocity.mean()), mean_observed_speed_mps=float(observed.mean()),
        mean_reference_mps=float(reference.mean()))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--snapshot", type=Path, required=True)
    parser.add_argument("--cases", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--modes", nargs="+", choices=("zero","wheel-positive","wheel-negative",
                                                     "leg-positive","leg-negative"), default=["zero"])
    parser.add_argument("--deadline", default="2026-10-14T02:10:00+08:00")
    args = parser.parse_args()
    learning = args.snapshot.resolve()/"tools/mujoco/learning"
    sys.path.insert(0, str(learning))
    from WbrLearningEnv import WbrLearningEnv, experiment_identity, verify_snapshot
    from EvaluateWbrStudy import load_cases, summarize_trace, acceptance
    manifest = verify_snapshot()
    cases = load_cases(args.cases)
    if cases["split"] != "development":
        raise RuntimeError("诊断只允许使用开发集")
    args.output.mkdir(parents=True, exist_ok=False)
    deadline = datetime.fromisoformat(args.deadline).timestamp()
    binding = dict(schema=1, created_at=datetime.now(timezone.utc).isoformat(),
        diagnostic_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        library_sha256=manifest["library_sha256"], experiment_identity=experiment_identity(manifest),
        cases_sha256=cases["cases_sha256"], modes=args.modes,
        note="仅用于开发集诊断；修正只读部署可得观测；真值只用于统计")
    (args.output/"run.json").write_bytes((json.dumps(binding,ensure_ascii=False,indent=2)+"\n").encode("utf-8"))
    results = []
    for mode in args.modes:
        for scenario in cases["cases"]:
            if time.time() >= deadline or (args.output/"stop.request").exists():
                return
            env = WbrLearningEnv(randomize=scenario["randomize"], strict=True,
                                 record=True, reward_mode="dense", action_mode="symmetric", gas_force=450.)
            try:
                obs,_ = env.reset(seed=scenario["seed"], options=dict(scenario["options"]))
                while True:
                    if time.time() >= deadline or (args.output/"stop.request").exists():
                        return
                    frame = obs[-31:]
                    action = np.zeros(3, dtype=np.float32)
                    if mode != "zero" and abs(frame[6]) < 1e-6:
                        axis = 2 if mode.startswith("wheel") else 1
                        polarity = 1. if mode.endswith("positive") else -1.
                        action[axis] = polarity*np.clip(frame[18]/.10,-1.,1.)
                    obs,_,terminated,truncated,_ = env.step(action)
                    next_frame = obs[-31:]
                    env.trace[-1].update(control_v_reference_mps=float(next_frame[24]),
                        observed_v_mps=float(next_frame[18]), position_error_m=float(next_frame[19]),
                        torque_request_nm=list(env.output[:6]))
                    if terminated or truncated:
                        break
                summary = env.summary()
                trace = list(env.trace)
                row = dict(id=scenario["id"], mode=mode, case=scenario["options"]["case"],
                    options=scenario["options"], metrics=summarize_trace(scenario,trace,summary),
                    diagnostics={label:window_metrics(trace,*bounds) for label,bounds in
                        {"drive":(1.,4.),"steady_drive":(3.5,4.),"stop_early":(4.,6.),
                         "stop_late":(6.,8.),"push_recovery":(2.5,4.7)}.items()})
                if not summary["passed"]:
                    # 提前失败的部分轨迹只用于找原因，不能作为低误差收益。
                    row["diagnostics_usable_for_performance"] = False
                else:
                    row["diagnostics_usable_for_performance"] = True
                path = args.output/f"{mode}-{scenario['id']}.json"
                path.write_bytes((json.dumps(dict(result=row,trace=trace),ensure_ascii=False)+"\n").encode("utf-8"))
                results.append(row)
                print(json.dumps(row,ensure_ascii=False),flush=True)
            finally:
                env.close()
    baseline = [row for row in results if row["mode"] == "zero"]
    comparisons = {}
    if baseline:
        for mode in args.modes:
            if mode != "zero":
                comparisons[mode] = acceptance(baseline,[row for row in results if row["mode"] == mode])
    result = dict(**binding, results=results, comparisons=comparisons)
    (args.output/"summary.json").write_bytes((json.dumps(result,ensure_ascii=False,indent=2)+"\n").encode("utf-8"))
    print(json.dumps(dict(status="completed", cases=len(results), output=str(args.output)),ensure_ascii=False),flush=True)


if __name__ == "__main__":
    main()
