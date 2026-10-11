"""同种子、同物理参数比较原控制与学习修正；每 1 ms 独立检查物理失败。"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

import numpy as np

from WbrLearningEnv import ROOT, WbrLearningEnv, verify_snapshot, experiment_identity


class Policy:
    def __init__(self, directory):
        directory = Path(directory)
        self.contract = json.loads((directory/"policy.json").read_text(encoding="utf-8"))
        self.weights = dict(np.load(directory/"policy.npz", allow_pickle=False))

    def predict(self, observation):
        value = np.asarray(observation, dtype=np.float32)
        count = len(self.weights)//2
        for i in range(count):
            value = value @ self.weights[f"w{i}"].T + self.weights[f"b{i}"]
            if i < count-1:
                value = np.tanh(value)
        return np.clip(value, -1., 1.)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--policy", type=Path)
    parser.add_argument("--cases", type=int, default=12)
    parser.add_argument("--seed-base", type=int, default=20000)
    parser.add_argument("--record", action="store_true")
    args = parser.parse_args()
    if args.cases < 1:
        parser.error("至少一个场景")
    if args.output.exists():
        raise FileExistsError("评价目录已存在，拒绝覆盖")
    args.output.mkdir(parents=True)
    manifest = verify_snapshot()
    policy = Policy(args.policy) if args.policy else None
    if policy and policy.contract["library_sha256"] != manifest["library_sha256"]:
        raise RuntimeError("策略训练时的控制器与当前对照版本不同")
    if policy and policy.contract["experiment_identity"] != experiment_identity(manifest):
        raise RuntimeError("策略训练时的机械模型或环境代码与当前对照版本不同")
    results = []
    started = time.perf_counter()
    for index in range(args.cases):
        case = ("stop", "push", "combined")[index%3]
        sign = 1. if index%2 == 0 else -1.
        options = dict(case=case, speed=sign*(.5 if index < 6 else .7), push=sign*(25. if index < 6 else 35.))
        randomize = index >= 6
        for name in (["baseline", "learned"] if policy else ["baseline"]):
            env = WbrLearningEnv(randomize=randomize, strict=True, record=args.record)
            try:
                observation, _ = env.reset(seed=args.seed_base+index, options=options)
                while True:
                    action = policy.predict(observation) if name == "learned" else np.zeros(6)
                    observation, _, done, truncated, info = env.step(action)
                    if done or truncated:
                        break
                row = dict(index=index, controller=name, seed=args.seed_base+index,
                           randomize=randomize, unseen_combination=case == "combined", **env.summary())
                results.append(row)
                print(json.dumps(row, ensure_ascii=False), flush=True)
                if args.record:
                    stem = f"{index:02d}-{name}"
                    (args.output/f"{stem}.json").write_text(json.dumps(env.trace, ensure_ascii=False), encoding="utf-8")
                    np.savez_compressed(args.output/f"{stem}.npz", qpos=env.poses,
                                        time=[r["time_s"] for r in env.trace])
            finally:
                env.close()
    paired_pass = {i for i in range(args.cases)
                   if len([r for r in results if r["index"] == i]) == (2 if policy else 1)
                   and all(r["passed"] for r in results if r["index"] == i)}
    comparison = {"paired_complete_cases": len(paired_pass),
                  "excluded_failed_pairs": [i for i in range(args.cases) if i not in paired_pass],
                  "metric_scope": "只比较两种控制都完整通过的同场景；失败另行判定，不用短轨迹计算收益"}
    for name in ("baseline", "learned"):
        all_rows = [r for r in results if r["controller"] == name]
        rows = [r for r in all_rows if r["index"] in paired_pass]
        comparison[name] = dict(completed=sum(r["passed"] for r in all_rows), cases=len(all_rows))
        if rows:
            for label, subset in (("trained_task_types", [r for r in rows if not r["unseen_combination"]]),
                                  ("unseen_combination", [r for r in rows if r["unseen_combination"]])):
                if subset:
                    comparison[name][label] = dict(paired_cases=len(subset),
                        velocity_rmse_mps=float(np.mean([r["velocity_rmse_mps"] for r in subset])),
                        stop_velocity_rms_mps=float(np.mean([r["stop_velocity_rms_mps"] for r in subset])),
                        peak_pitch_deg=max(r["peak_pitch_deg"] for r in subset),
                        mean_hip_heat_nm2s=float(np.mean([r["hip_heat_nm2s"] for r in subset])))
    report = dict(scope="平地停车及纵向外力；不含起身、跳跃或实车", snapshot=str(ROOT),
                  library_sha256=manifest["library_sha256"], elapsed_s=time.perf_counter()-started,
                  results=results, comparison=comparison)
    (args.output/"summary.json").write_text(json.dumps(report, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    print(json.dumps(comparison, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
