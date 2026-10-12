"""核对真实训练日志的评分账目及场景覆盖；结果不能替代独立评价。"""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path

import numpy as np


def require(value, message):
    if not value:
        raise RuntimeError(message)


def read(path):
    return json.loads(path.read_text(encoding="utf-8"))


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def analyze(training, through_step, output):
    run = read(training / "run.json")
    frozen = Path(run["snapshot"])
    cases_path = frozen / "tools/mujoco/learning/cases/aligned-20261012/training.json"
    cases = read(cases_path)
    scenarios = {row["id"]: row for row in cases["cases"]}
    weights = run["training_protocol"]["weights"]
    raw = (training / "aligned-episodes.jsonl").read_bytes()
    # 活文件最后一行可能正在追加，仅纳入已有完整换行的记录。
    lines = raw.split(b"\n")[:-1]
    rows = [json.loads(line) for line in lines if line.strip()]
    rows = [row for row in rows if row["step"] <= through_step]
    require(bool(rows), "没有已完成训练回合")
    require(all(a["step"] <= b["step"] for a, b in zip(rows, rows[1:])), "训练日志步数倒退")
    errors = []
    physical_failures = 0
    for row in rows:
        require(row["id"] in scenarios, "日志混入其他场景")
        require(row["case"] == scenarios[row["id"]]["options"]["case"], "场景类别错配")
        require(set(row["dense_costs"]) == set(weights) - {"guard"}, "训练成本字段缺失")
        values = [*row["dense_costs"].values(), row["guard_cost"], row["return_sum"], *row["violations"].values()]
        require(all(isinstance(value, (float, int)) and math.isfinite(value) for value in values), "训练评分含无效数值")
        failed = "physical_failure" in row["violations"]
        physical_failures += int(failed)
        penalty = 100. if failed else weights["guard"] * row["guard_cost"]
        expected = -sum(weights[name] * value for name, value in row["dense_costs"].items()) - penalty
        errors.append(abs(expected - row["return_sum"]))
    require(max(errors) < 1e-9, "实际回合累计评分无法由成本复算")

    def summarize(group):
        require(bool(group), "缺少一种登记任务的训练记录")
        return dict(episodes=len(group), unique_cases=len({row["id"] for row in group}),
            guard_violation_episodes=sum(bool(row["violations"]) for row in group),
            violation_counts=dict(Counter(name for row in group for name in row["violations"])),
            mean_return=float(np.mean([row["return_sum"] for row in group])),
            mean_weighted_costs={name: float(np.mean([row["dense_costs"][name] * weights[name] for row in group]))
                for name in group[0]["dense_costs"]},
            mean_guard_penalty=float(np.mean([100. if "physical_failure" in row["violations"] else weights["guard"] * row["guard_cost"] for row in group])))

    encoded = b"".join((json.dumps(row, ensure_ascii=False) + "\n").encode("utf-8") for row in rows)
    result = dict(schema=1, training=str(training.resolve()), seed=run["seed"], through_step=through_step,
        last_completed_episode_step=rows[-1]["step"], complete_episodes=len(rows),
        registered_training_cases=len(scenarios), unique_cases=len({row["id"] for row in rows}),
        case_visits=dict(sorted(Counter(row["id"] for row in rows).items())),
        physical_failure_episodes=physical_failures, maximum_reward_reconstruction_error=max(errors),
        cases_file_sha256=digest(cases_path), run_sha256=digest(training / "run.json"),
        captured_log_sha256=hashlib.sha256(encoded).hexdigest(), analyzer_sha256=digest(Path(__file__)),
        tasks={name: summarize([row for row in rows if row["case"] == name]) for name in ("stop", "push", "combined")},
        boundary="带探索动作的训练记录，仅核对评分账目、场景覆盖及超限分布；不据此选检查点或宣称独立性能收益。热项按训练库全局尺度归一，类别均值不能解释为相对本类基线的百分比。")
    output.mkdir(parents=True, exist_ok=False)
    (output / "训练回合摘录.jsonl").write_bytes(encoded)
    (output / "训练日志核验.json").write_bytes((json.dumps(result, ensure_ascii=False, indent=2) + "\n").encode("utf-8"))
    print(json.dumps({key: value for key, value in result.items() if key not in ("case_visits", "tasks")}, ensure_ascii=False))
    print(json.dumps(result["tasks"], ensure_ascii=False))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--training", required=True, type=Path)
    parser.add_argument("--through-step", required=True, type=int)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    analyze(args.training.resolve(), args.through_step, args.output.resolve())
