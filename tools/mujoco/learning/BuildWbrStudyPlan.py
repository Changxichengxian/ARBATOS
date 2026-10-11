"""生成串行筛选计划；路径由本机提供，步骤与预算固定。"""
import argparse
import json
from pathlib import Path
import sys


def build_plan(learning, cases, output, python, smoke=False, reference=False):
    learning, cases, output = map(lambda path:Path(path).resolve(), (learning,cases,output))
    deadline = "2026-10-14T02:10:00+08:00"
    jobs = []

    def add(job_id, kind, arguments):
        jobs.append(dict(id=job_id, kind=kind, arguments=list(map(str,arguments))))

    def train(label, seed, mode, steps=131072):
        job_id = f"{label}-{seed}-train"
        add(job_id, "train", ["--output", output/label/str(seed)/"train", "--seed", seed,
            "--steps", steps, "--envs", 2, "--reward-mode", "dense", "--action-mode", mode,
            "--deadline", deadline, "--checkpoint-every", 1024 if smoke else 16384])
        return job_id

    def baseline(mode, split="validation"):
        job_id = f"baseline-{split}-{mode}"
        add(job_id, "evaluate", ["--output", output/"baselines"/f"{split}-{mode}",
            "--cases", cases/f"{split}.json", "--baseline-only", "--action-mode", mode])
        return job_id

    def evaluate(label, seed, source, cache=None, split="validation"):
        arguments = ["--output", output/label/str(seed)/"evaluation", "--cases", cases/f"{split}.json",
                     "--policy", "{job:"+source+"}"]
        if cache:
            arguments += ["--baseline-cache", "{job:"+cache+"}"]
        add(f"{label}-{seed}-evaluate", "evaluate", arguments)

    if reference:
        cache = None
        for seed in ((41,) if smoke else (41,73)):
            job_id = f"D-reference-{seed}-search"
            add(job_id,"search",["--output",output/"D-reference"/str(seed)/"search",
                "--cases",cases/"development.json","--seed",seed,"--family","reference",
                "--population",4 if smoke else 12,"--generations",1 if smoke else 6,
                "--deadline",deadline])
            if not smoke and cache is None:
                cache = baseline("reference")
            evaluate("D-reference",seed,job_id,cache,split="development" if smoke else "validation")
    elif smoke:
        source = train("smoke", 41, "full", 1024)
        evaluate("smoke", 41, source, split="development")
    else:
        baseline("full", "development")
        source = train("A-full", 41, "full")
        cache = baseline("full")
        evaluate("A-full", 41, source, cache)
        source = train("A-full", 73, "full")
        evaluate("A-full", 73, source, cache)
        source = train("B-symmetric", 41, "symmetric")
        cache = baseline("symmetric")
        evaluate("B-symmetric", 41, source, cache)
        source = train("B-symmetric", 73, "symmetric")
        evaluate("B-symmetric", 73, source, cache)
        for seed in (41,73):
            job_id = f"C-linear-{seed}-search"
            add(job_id, "search", ["--output", output/"C-linear"/str(seed)/"search",
                "--cases", cases/"development.json", "--seed", seed,
                "--population", 8, "--generations", 5, "--deadline", deadline])
            evaluate("C-linear", seed, job_id, cache)
    stage = ("reference-" if reference else "")+("smoke" if smoke else "screening")
    return dict(schema=1, stage=stage, deadline=deadline,
                python=str(Path(python).resolve()), learning_directory=str(learning), jobs=jobs)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--learning-directory", required=True, type=Path)
    parser.add_argument("--cases", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--python", default=sys.executable)
    parser.add_argument("--smoke", action="store_true")
    parser.add_argument("--reference", action="store_true", help="独立三参数参考对照")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    plan = build_plan(args.learning_directory, args.cases, args.output, args.python, args.smoke,args.reference)
    (args.output/"plan.json").write_bytes((json.dumps(plan, ensure_ascii=False, indent=2)+"\n").encode("utf-8"))
    print(json.dumps(dict(plan=str((args.output/"plan.json").resolve()), jobs=len(plan["jobs"]))))
