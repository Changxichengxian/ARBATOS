"""把同一冻结物理的原控制评价压缩成训练奖励基线库，不运行新仿真。"""
import argparse
import hashlib
import json
from pathlib import Path

from BuildWbrStudyCases import canonical_bytes
from EvaluateWbrStudy import load_cases
from WbrAlignedReward import baseline_spec
from WbrLearningEnv import ROOT, verify_snapshot, experiment_identity


def build(cases_path, baseline_path):
    read = lambda p: json.loads(p.read_text(encoding="utf-8"))
    digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    cases = load_cases(cases_path)
    if cases["split"] != "aligned-training":
        raise ValueError("训练奖励库只能使用登记的aligned-training场景")
    manifest = verify_snapshot()
    identity = experiment_identity(manifest)
    summary = read(baseline_path)
    run = read(baseline_path.parent / "run.json")
    binding = run["binding"]
    if run["binding_sha256"] != hashlib.sha256(canonical_bytes(binding)).hexdigest():
        raise RuntimeError("基线运行绑定损坏")
    if summary["run_binding_sha256"] != run["binding_sha256"]:
        raise RuntimeError("基线汇总与运行绑定不一致")
    expected = dict(scenarios_sha256=cases["cases_sha256"], experiment_identity=identity,
        library_sha256=manifest["library_sha256"],
        env_config=dict(reward_mode="dense", action_mode="reference", gas_force=450.))
    for key,value in expected.items():
        if summary[key] != value or binding[key] != value:
            raise RuntimeError("训练基线来源不符："+key)
    if binding["policy_artifacts"] is not None:
        raise RuntimeError("训练基线必须为原控制")
    if binding["evaluator_sha256"] != digest(ROOT / "tools/mujoco/learning/EvaluateWbrStudy.py"):
        raise RuntimeError("基线评价代码与冻结版本不匹配")
    by_id = {row["id"]:row for row in summary["baseline_results"]}
    if len(by_id) != len(summary["baseline_results"]) or len(by_id) != len(cases["cases"]):
        raise RuntimeError("训练基线场景缺失或重复")
    rows = []
    for scenario in cases["cases"]:
        row = by_id[scenario["id"]]
        for key in ("id", "seed", "randomize", "options"):
            if row[key] != scenario[key]:
                raise RuntimeError("训练基线场景错配")
        if row["case"] != scenario["options"]["case"] or row["controller"] != "baseline":
            raise RuntimeError("训练基线控制器或类别错配")
        rows.append(dict(scenario=scenario, baseline=row, spec=baseline_spec(row)))
    heat_scale = sum(row["baseline"]["metrics"]["hip_heat_nm2s"] for row in rows) / len(rows)
    return dict(schema=1, cases_sha256=cases["cases_sha256"], rows=rows,
        experiment_identity=identity, library_sha256=manifest["library_sha256"], heat_scale=heat_scale,
        env_config=expected["env_config"], baseline_summary_sha256=digest(baseline_path),
        baseline_run_sha256=digest(baseline_path.parent / "run.json"),
        evaluator_sha256=binding["evaluator_sha256"],
        note="同场景原控制成本仅用于训练奖励，不进入93维观测；独立监控/验证/最终均不参与此库。")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cases", type=Path, required=True)
    parser.add_argument("--baseline", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError("训练奖励库已存在")
    result = build(args.cases, args.baseline)
    args.output.write_bytes((json.dumps(result,ensure_ascii=False,indent=2)+"\n").encode("utf-8"))
    print(json.dumps(dict(cases=len(result["rows"]),heat_scale=result["heat_scale"])))
