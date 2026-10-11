"""按事先登记的固定参数比较参考减速与阻尼，逐个场景保存并可继续。"""
import argparse
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import sys
import time

import numpy as np


def encoded_hash(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(",",":"),
                                    ensure_ascii=False).encode("utf-8")).hexdigest()


def row_binding(scenario,mode,run_binding):
    return dict(run_binding_sha256=encoded_hash(run_binding),scenario=scenario,mode=mode)


def validate_saved_row(saved,scenario,mode,run_binding):
    if saved.get("binding") != row_binding(scenario,mode,run_binding):
        raise RuntimeError("场景缓存的参数、模式或运行身份不匹配")
    row = saved["result"]
    if saved.get("result_sha256") != encoded_hash(row):
        raise RuntimeError("场景缓存内容哈希不匹配")
    expected = dict(id=scenario["id"],seed=scenario["seed"],randomize=scenario["randomize"],
                    case=scenario["options"]["case"],options=scenario["options"])
    if any(row.get(key) != value for key,value in expected.items()):
        raise RuntimeError("场景缓存的实际场景字段不匹配")
    return row


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--snapshot", type=Path, required=True)
    parser.add_argument("--registration", type=Path, required=True)
    parser.add_argument("--cases", type=Path, required=True)
    parser.add_argument("--original-baseline", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--deadline", default="2026-10-14T02:10:00+08:00")
    args = parser.parse_args()
    sys.path.insert(0,str(args.snapshot.resolve()/"tools/mujoco/learning"))
    from WbrLearningEnv import verify_snapshot, experiment_identity
    from EvaluateWbrStudy import (load_cases, run_case, acceptance, motion_quality_acceptance,
                                  atomic_write_json, EvaluationInterrupted)
    read = lambda path: json.loads(path.read_text(encoding="utf-8"))
    digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
    manifest = verify_snapshot()
    cases = load_cases(args.cases)
    if cases["split"] != "development":
        raise RuntimeError("该诊断只允许开发集")
    registration = read(args.registration)
    modes = registration["modes"]
    if not modes or modes[0] != {"name":"zero", "action":[0,0,0]}:
        raise ValueError("第一组必须为零修正基线")
    if len({mode["name"] for mode in modes}) != len(modes):
        raise ValueError("诊断模式名称重复")
    for mode in modes:
        value = np.asarray(mode["action"],dtype=float)
        if value.shape != (3,) or not np.isfinite(value).all():
            raise ValueError("诊断动作维数或数值无效")
        if not 0 <= value[0] <= 1 or np.any(np.abs(value[1:]) > 1):
            raise ValueError("诊断动作超出登记边界")
    original = read(args.original_baseline)
    if original["scenarios_sha256"] != cases["cases_sha256"]:
        raise RuntimeError("原基线场景不匹配")
    binding = dict(script_sha256=digest(Path(__file__)), registration_sha256=digest(args.registration),
        baseline_sha256=digest(args.original_baseline), cases_sha256=cases["cases_sha256"],
        library_sha256=manifest["library_sha256"], experiment_identity=experiment_identity(manifest),
        modes=modes)
    args.output.mkdir(parents=True,exist_ok=True)
    lock_stream = (args.output/"diagnostic.lock").open("a+b")
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
    run_path = args.output/"run.json"
    if run_path.exists():
        if not args.resume or read(run_path)["binding"] != binding:
            raise RuntimeError("已有诊断须以相同代码与输入显式续跑")
        if (args.output/"summary.json").exists():
            raise RuntimeError("诊断已经完成，拒绝重新覆盖结论")
    else:
        if args.resume or list(args.output.glob("*.json")):
            raise RuntimeError("输出目录存在外来结果或缺少原运行绑定")
        atomic_write_json(run_path,dict(binding=binding))
    deadline = datetime.fromisoformat(args.deadline).timestamp()
    stopped = lambda: time.time() >= deadline or (args.output/"stop.request").exists()
    class FixedPolicy:
        def __init__(self, action):
            self.action = np.asarray(action,dtype=np.float32)
        def predict(self, observation):
            return self.action.copy()
    groups = {}
    try:
        for mode in modes:
            rows = []
            for scenario in cases["cases"]:
                if stopped():
                    raise EvaluationInterrupted("诊断停止")
                path = args.output/f"{mode['name']}-{scenario['id']}.json"
                if path.exists():
                    if not args.resume:
                        raise RuntimeError("场景已有结果但未请求续跑")
                    row = validate_saved_row(read(path),scenario,mode,binding)
                else:
                    row = run_case(scenario,FixedPolicy(mode["action"]),
                        dict(reward_mode="dense",action_mode="reference",gas_force=450.),
                        record=True,stop_requested=stopped)
                    atomic_write_json(path,dict(binding=row_binding(scenario,mode,binding),
                        result_sha256=encoded_hash(row),result=row))
                rows.append({key:value for key,value in row.items() if key != "trace"})
                atomic_write_json(args.output/"status.json",dict(status="running",pid=os.getpid(),
                    mode=mode["name"],completed_cases=len(rows),total_cases=len(cases["cases"])))
            groups[mode["name"]] = rows
            if mode["name"] == "zero":
                for key in ("metrics","motion_quality"):
                    previous = {row["id"]:row[key] for row in original["baseline_results"]}
                    current = {row["id"]:row[key] for row in rows}
                    if current != previous:
                        raise RuntimeError(f"零动作与原冻结控制的{key}不一致，停止后续诊断")
            print(json.dumps(dict(mode=mode["name"],completed=len(rows)),ensure_ascii=False),flush=True)
    except EvaluationInterrupted:
        atomic_write_json(args.output/"status.json",dict(status="stopped",pid=os.getpid()))
        return
    except Exception as error:
        atomic_write_json(args.output/"status.json",dict(status="failed",pid=os.getpid(),error=str(error)))
        raise
    comparisons = {}
    for mode, rows in groups.items():
        if mode == "zero":
            continue
        result = acceptance(groups["zero"],rows)
        quality = motion_quality_acceptance(groups["zero"],rows)
        result["checks"]["motion_quality"] = quality
        result["overall_pass"] = result["overall_pass"] and quality["pass"]
        comparisons[mode] = result
    atomic_write_json(args.output/"summary.json",dict(binding=binding,results=groups,
        comparisons=comparisons,zero_action_exactly_matches_original=True))
    atomic_write_json(args.output/"status.json",dict(status="completed",pid=os.getpid(),
        completed_trajectories=sum(len(rows) for rows in groups.values())))
    print(json.dumps(dict(status="completed",modes=len(groups)),ensure_ascii=False),flush=True)


if __name__ == "__main__":
    main()
