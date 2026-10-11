"""为对齐评分的参考策略生成独立训练库和训练监控集。"""
import argparse
import json
from pathlib import Path
import random

from BuildWbrStudyCases import _signed_values, _balance_combined_pushes, document


def build(name, per_kind, seed_base, generator_seed, all_randomized):
    rng = random.Random(generator_seed)
    cases = []
    for kind in ("stop", "push", "combined"):
        speeds = _signed_values(per_kind, .35, .75, rng)
        pushes = _signed_values(per_kind, 12., 35., rng)
        if kind == "combined":
            pushes = _balance_combined_pushes(speeds, pushes)
        for index in range(per_kind):
            randomized = all_randomized or bool(index % 2)
            delay = (index % 2 if all_randomized else (index // 2) % 2) if randomized else 0
            cases.append(dict(id=f"{name}-{kind}-{index:03d}", seed=seed_base+len(cases),
                randomize=randomized, options=dict(case=kind, speed=round(speeds[index],6),
                    push=round(pushes[index],6), delay=delay, duration=8.)))
    return document(name, cases, primary_gate=False)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--original-cases", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError("训练库目录已存在，拒绝覆盖")
    datasets = dict(training=build("aligned-training",48,71000,95331,True),
                    monitor=build("aligned-monitor",8,81000,95347,False))
    previous_seeds = set()
    for name in ("development", "validation", "final", "pressure"):
        data = json.loads((args.original_cases / (name+".json")).read_text(encoding="utf-8"))
        previous_seeds.update(row["seed"] for row in data["cases"])
    used = set(previous_seeds)
    for name, data in datasets.items():
        seeds = {row["seed"] for row in data["cases"]}
        if len(seeds) != data["case_count"] or used & seeds:
            raise RuntimeError("训练/监控与已有场景随机种子重叠")
        used.update(seeds)
    args.output.mkdir(parents=True)
    for name,data in datasets.items():
        (args.output / (name+".json")).write_bytes((json.dumps(data,ensure_ascii=False,indent=2)+"\n").encode("utf-8"))
    print(json.dumps({name:dict(count=value["case_count"], sha256=value["cases_sha256"])
                      for name,value in datasets.items()},ensure_ascii=False))
