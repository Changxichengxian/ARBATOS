"""按固定场景评价一个研究策略，并给出逐项可复核的验收结果。"""
from __future__ import annotations

import argparse
from datetime import datetime, timedelta, timezone
import hashlib
import json
import math
from pathlib import Path
import sys
import time

import numpy as np

from BuildWbrStudyCases import TRAIN_SEEDS, canonical_bytes, cases_sha256


WINDOWS = {
    "stop": {"stop": (4.0, 8.0)},
    "push": {"push": (2.5, 4.7)},
    "combined": {"stop": (4.0, 5.0), "push": (5.0, 7.2)},
}
THRESHOLDS = {
    "mean_improvement": 0.10,
    "case_relative_regression": 0.05,
    "case_speed_absolute_mps": 0.001,
    "heat_mean_regression": 0.05,
    "heat_case_regression": 0.20,
    "secondary_relative_regression": 0.05,
    "travel_absolute_m": 0.005,
    "speed_absolute_mps": 0.005,
    "angle_absolute_deg": 0.2,
}

MOTION_QUALITY_THRESHOLDS = {
    "relative_regression": 0.05,
    "drive_tracking_absolute_mps": 0.005,
    "world_speed_absolute_mps": 0.001,
    "progress_absolute_m": 0.005,
    "lateral_absolute_m": 0.005,
    "yaw_absolute_deg": 0.5,
}


def motion_quality_metrics(case, trace):
    """只读额外遥测，不改变仿真积分、观测或控制输入。"""
    rms = lambda values: float(np.sqrt(np.mean(np.square(values))))
    result = dict(max_lateral_m=float(max(abs(row["lateral_m"]) for row in trace)),
                  max_yaw_deg=math.degrees(max(abs(row["yaw_relative_rad"]) for row in trace)),
                  windows={})
    for name,bounds in WINDOWS[case["options"]["case"]].items():
        rows = _window(trace,bounds)
        vx = np.array([row["world_vx_mps"] for row in rows])
        vy = np.array([row["world_vy_mps"] for row in rows])
        result["windows"][name] = dict(world_vx_rms_mps=rms(vx),
                                      plane_speed_rms_mps=float(np.sqrt(np.mean(vx*vx+vy*vy))))
    if case["options"]["case"] != "push":
        for name,bounds in (("drive",(1.,4.)),("steady_drive",(3.5,4.))):
            rows = _window(trace,bounds)
            result[name+"_tracking_rms_mps"] = rms([row["world_vx_mps"]-row["command_v_mps"] for row in rows])
        rows = _window(trace,(1.,4.))
        result["progress_at_stop_m"] = math.copysign(1.,case["options"]["speed"])*rows[-1]["longitudinal_m"]
    return result


def motion_quality_acceptance(baseline, learned):
    by_base, by_learned = ({row["id"]:row for row in rows} for rows in (baseline,learned))
    if set(by_base) != set(by_learned) or len(by_base) != len(baseline) or len(by_learned) != len(learned):
        raise RuntimeError("运动质量检查的场景集合不一致或重复")
    results = []
    threshold = MOTION_QUALITY_THRESHOLDS
    for case_id in sorted(by_base):
        a,b = by_base[case_id], by_learned[case_id]
        complete = a["metrics"]["passed"] and b["metrics"]["passed"]
        checks = {}
        if complete and "motion_quality" in a and "motion_quality" in b:
            aq,bq = a["motion_quality"], b["motion_quality"]
            for key,tolerance in (("max_lateral_m",threshold["lateral_absolute_m"]),
                                  ("max_yaw_deg",threshold["yaw_absolute_deg"])):
                checks[key] = _regression_check(aq[key],bq[key],threshold["relative_regression"],tolerance)
            for name in WINDOWS[a["case"]]:
                for metric in ("world_vx_rms_mps","plane_speed_rms_mps"):
                    checks[name+"_"+metric] = _regression_check(aq["windows"][name][metric],
                        bq["windows"][name][metric], threshold["relative_regression"],threshold["world_speed_absolute_mps"])
            if a["case"] != "push":
                for key in ("drive_tracking_rms_mps","steady_drive_tracking_rms_mps"):
                    checks[key] = _regression_check(aq[key],bq[key],threshold["relative_regression"],
                                                    threshold["drive_tracking_absolute_mps"])
                # 行驶进度越小越差，与误差指标的方向相反。
                progress = _regression_check(-aq["progress_at_stop_m"],-bq["progress_at_stop_m"],
                    threshold["relative_regression"],threshold["progress_absolute_m"])
                progress["baseline_progress_m"],progress["learned_progress_m"] = aq["progress_at_stop_m"],bq["progress_at_stop_m"]
                checks["progress_at_stop_m"] = progress
        results.append(dict(id=case_id, checks=checks, **{"pass":bool(complete and checks and all(
            value["pass"] for value in checks.values()))}))
    return {"pass":bool(results and all(row["pass"] for row in results)),
            "protocol":"motion-quality-20261012", "thresholds":threshold, "cases":results}


def sha256_file(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def now_cn() -> str:
    return datetime.now(timezone(timedelta(hours=8))).isoformat(timespec="seconds")


def atomic_write_json(path: Path, payload) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name+".tmp")
    temporary.write_text(json.dumps(payload, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    temporary.replace(path)


def load_cases(path: Path) -> dict:
    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    cases = payload.get("cases")
    if payload.get("schema") != 1 or not isinstance(cases, list) or not cases:
        raise ValueError("场景文件格式错误")
    if payload.get("case_count") != len(cases) or payload.get("cases_sha256") != cases_sha256(cases):
        raise RuntimeError("场景数量或 SHA-256 不匹配")
    ids = set()
    for item in cases:
        if set(item) != {"id", "seed", "randomize", "options"}:
            raise ValueError(f"场景字段不固定：{item}")
        if item["id"] in ids:
            raise ValueError(f"场景 id 重复：{item['id']}")
        ids.add(item["id"])
        options = item["options"]
        if set(options) != {"case", "speed", "push", "delay", "duration"}:
            raise ValueError(f"场景 options 字段不固定：{item['id']}")
        if options["case"] not in WINDOWS or float(options["duration"]) != 8.0:
            raise ValueError(f"场景类型或时长错误：{item['id']}")
        values = (float(options["speed"]), float(options["push"]), float(options["duration"]))
        if not all(math.isfinite(value) for value in values):
            raise ValueError(f"场景含非有限数：{item['id']}")
        pressure = payload.get("split") == "pressure"
        speed_limit = (0.8, 0.8) if pressure else (0.35, 0.75)
        push_limit = (40.0, 40.0) if pressure else (12.0, 35.0)
        if not speed_limit[0] <= abs(values[0]) <= speed_limit[1] or not push_limit[0] <= abs(values[1]) <= push_limit[1]:
            raise ValueError(f"场景超出声明范围：{item['id']}")
        if int(options["delay"]) not in (0, 1):
            raise ValueError(f"场景延迟只允许 0 或 1 个控制周期：{item['id']}")
    return payload


def _window(trace: list[dict], bounds: tuple[float, float]) -> list[dict]:
    start, end = bounds
    rows = [row for row in trace if start-1e-9 <= float(row["time_s"]) <= end+1e-9]
    if not rows:
        raise RuntimeError(f"轨迹没有覆盖统计窗口 {start}～{end} s")
    return rows


def _window_metrics(rows: list[dict], direction: float) -> dict:
    velocity = np.asarray([row["true_v_mps"] for row in rows], dtype=float)
    command = np.asarray([row["command_v_mps"] for row in rows], dtype=float)
    velocity_error = velocity-command
    x = np.asarray([row["x_m"] for row in rows], dtype=float)
    pitch = np.degrees(np.abs([row["pitch_rad"] for row in rows]))
    roll = np.degrees(np.abs([row["roll_rad"] for row in rows]))
    closure = np.asarray([row["closure_m"] for row in rows], dtype=float)
    return {
        "velocity_rms_mps": float(np.sqrt(np.mean(np.square(velocity)))),
        "velocity_error_rms_mps": float(np.sqrt(np.mean(np.square(velocity_error)))),
        "peak_speed_mps": float(np.max(np.abs(velocity))),
        "peak_velocity_error_mps": float(np.max(np.abs(velocity_error))),
        "travel_m": float(np.max(np.abs(x-x[0]))),
        "peak_reverse_mps": float(max(0.0, np.max(-math.copysign(1.0, direction)*velocity))),
        "peak_pitch_deg": float(np.max(pitch)),
        "peak_roll_deg": float(np.max(roll)),
        "max_closure_m": float(np.max(closure)),
        "samples": len(rows),
    }


def summarize_trace(case: dict, trace: list[dict], environment_summary: dict) -> dict:
    case_name = case["options"]["case"]
    passed = bool(environment_summary["passed"])
    # 物理失败可能发生在统计窗口之前。此时仍保留失败和热量，绝不把短轨迹
    # 当作低误差收益；只有完整通过的 8 s 轨迹才计算窗口指标。
    if not passed:
        return {
            "windows": {},
            "hip_heat_nm2s": float(environment_summary["hip_heat_nm2s"]),
            "simulated_s": float(environment_summary["simulated_s"]),
            "peak_pitch_1ms_deg": float(environment_summary.get("peak_pitch_1ms_deg", 0.0)),
            "peak_roll_1ms_deg": float(environment_summary.get("peak_roll_1ms_deg", 0.0)),
            "max_closure_1ms_m": float(environment_summary.get("max_closure_1ms_m", 0.0)),
            "passed": False,
            "failure": environment_summary["failure"],
        }
    windows = {name: _window_metrics(_window(trace, bounds), case["options"]["speed"])
               for name, bounds in WINDOWS[case_name].items()}
    return {
        "windows": windows,
        "hip_heat_nm2s": float(environment_summary["hip_heat_nm2s"]),
        "simulated_s": float(environment_summary["simulated_s"]),
        "peak_pitch_1ms_deg": float(environment_summary.get(
            "peak_pitch_1ms_deg", max(value["peak_pitch_deg"] for value in windows.values()))),
        "peak_roll_1ms_deg": float(environment_summary.get(
            "peak_roll_1ms_deg", max(value["peak_roll_deg"] for value in windows.values()))),
        "max_closure_1ms_m": float(environment_summary.get(
            "max_closure_1ms_m", max(value["max_closure_m"] for value in windows.values()))),
        "passed": True,
        "failure": environment_summary["failure"],
    }


def _allowed(base: float, relative: float, absolute: float) -> float:
    return max(abs(base)*relative, absolute)


def _regression_check(base: float, learned: float, relative: float, absolute: float) -> dict:
    delta = learned-base
    allowed = _allowed(base, relative, absolute)
    return {"pass": bool(delta <= allowed), "baseline": base, "learned": learned,
            "delta": delta, "allowed_increase": allowed}


def acceptance(baseline: list[dict], learned: list[dict]) -> dict:
    by_base = {row["id"]: row for row in baseline}
    by_learned = {row["id"]: row for row in learned}
    if set(by_base) != set(by_learned):
        raise RuntimeError("原控制与候选策略的场景集合不同")
    physical = all(row["metrics"]["passed"] for row in baseline+learned)
    case_checks = []
    complete_pairs = []
    for case_id in sorted(by_base):
        a, b = by_base[case_id], by_learned[case_id]
        complete = a["metrics"]["passed"] and b["metrics"]["passed"]
        item = {"id": case_id, "case": a["case"], "complete": complete, "checks": {}}
        if complete:
            complete_pairs.append((a, b))
            for window_name in WINDOWS[a["case"]]:
                primary_metric = ("velocity_error_rms_mps" if window_name == "push"
                                  else "velocity_rms_mps")
                av = a["metrics"]["windows"][window_name][primary_metric]
                bv = b["metrics"]["windows"][window_name][primary_metric]
                item["checks"][f"{window_name}_primary"] = _regression_check(
                    av, bv, THRESHOLDS["case_relative_regression"],
                    THRESHOLDS["case_speed_absolute_mps"])
            secondary = []
            if "stop" in WINDOWS[a["case"]]:
                secondary += [("stop_travel", "stop", "travel_m", THRESHOLDS["travel_absolute_m"]),
                              ("stop_reverse", "stop", "peak_reverse_mps", THRESHOLDS["speed_absolute_mps"])]
            if "push" in WINDOWS[a["case"]]:
                secondary += [("push_peak_velocity_error", "push", "peak_velocity_error_mps",
                               THRESHOLDS["speed_absolute_mps"])]
            for label, window_name, metric, absolute in secondary:
                item["checks"][label] = _regression_check(
                    a["metrics"]["windows"][window_name][metric],
                    b["metrics"]["windows"][window_name][metric],
                    THRESHOLDS["secondary_relative_regression"], absolute)
            for angle in ("peak_pitch_deg", "peak_roll_deg"):
                metric = angle.replace("_deg", "_1ms_deg")
                av = a["metrics"][metric]
                bv = b["metrics"][metric]
                item["checks"][angle] = _regression_check(
                    av, bv, THRESHOLDS["secondary_relative_regression"],
                    THRESHOLDS["angle_absolute_deg"])
            item["checks"]["hip_heat_case"] = _regression_check(
                a["metrics"]["hip_heat_nm2s"], b["metrics"]["hip_heat_nm2s"],
                THRESHOLDS["heat_case_regression"], 0.0)
        item["pass"] = bool(complete and all(check["pass"] for check in item["checks"].values()))
        case_checks.append(item)

    aggregates = {}
    for case_name, window_name in (("stop", "stop"), ("push", "push")):
        pairs = [(a, b) for a, b in complete_pairs if a["case"] == case_name]
        if not pairs:
            aggregates[case_name] = {"pass": False, "reason": "没有完整配对"}
            continue
        primary_metric = ("velocity_error_rms_mps" if window_name == "push"
                          else "velocity_rms_mps")
        av = float(np.mean([a["metrics"]["windows"][window_name][primary_metric] for a, _ in pairs]))
        bv = float(np.mean([b["metrics"]["windows"][window_name][primary_metric] for _, b in pairs]))
        improvement = 1.0-bv/av if av > 0 else float("-inf")
        aggregates[case_name] = {"pass": bool(improvement >= THRESHOLDS["mean_improvement"]),
                                 "paired_cases": len(pairs), "baseline_mean": av,
                                 "learned_mean": bv, "metric": primary_metric,
                                 "improvement_fraction": improvement}
    if complete_pairs:
        heat_a = float(np.mean([a["metrics"]["hip_heat_nm2s"] for a, _ in complete_pairs]))
        heat_b = float(np.mean([b["metrics"]["hip_heat_nm2s"] for _, b in complete_pairs]))
        heat_change = heat_b/heat_a-1.0 if heat_a > 0 else float("inf")
        heat = {"pass": bool(heat_change <= THRESHOLDS["heat_mean_regression"]),
                "baseline_mean": heat_a, "learned_mean": heat_b, "change_fraction": heat_change}
    else:
        heat = {"pass": False, "reason": "没有完整配对"}
    checks = {
        "physical_complete": {"pass": physical},
        "all_case_guards": {"pass": bool(case_checks and all(row["pass"] for row in case_checks))},
        "stop_mean_improvement": aggregates["stop"],
        "push_mean_improvement": aggregates["push"],
        "hip_heat_mean": heat,
    }
    return {"overall_pass": all(value["pass"] for value in checks.values()),
            "thresholds": THRESHOLDS, "checks": checks, "cases": case_checks}


def policy_artifacts(directory: Path) -> dict[str, str]:
    directory = Path(directory).resolve()
    contract = directory/"policy.json"
    if not contract.is_file():
        raise FileNotFoundError("策略缺少 policy.json")
    files = [contract]
    files += sorted(path for path in directory.iterdir() if path.is_file() and path != contract and
                    (path.suffix == ".npz" or "linear" in path.name.lower() or "coeff" in path.name.lower()))
    result = {path.name: sha256_file(path) for path in files}
    contract_data = json.loads(contract.read_text(encoding="utf-8"))
    if len(files) < 2:
        coefficients = contract_data.get("coefficients")
        if contract_data.get("kind") != "linear" or not isinstance(coefficients, list):
            raise FileNotFoundError("策略缺少 policy.npz 或线性系数")
        result["embedded_linear_coefficients"] = hashlib.sha256(
            canonical_bytes(coefficients)).hexdigest()
    return result


def _lock_core(cases: dict, policy_dir: Path, contract: dict, manifest: dict, identity: str) -> dict:
    return {"schema": 1, "created_at": now_cn(), "scenarios_sha256": cases["cases_sha256"],
            "split": cases["split"], "policy_directory": str(Path(policy_dir).resolve()),
            "policy_seed": contract.get("seed"),
            "policy_artifacts": policy_artifacts(policy_dir),
            "library_sha256": manifest["library_sha256"], "experiment_identity": identity,
            "env_config": contract.get("env_config")}


def registry_path(cases_path: Path) -> Path:
    return Path(cases_path).with_name(Path(cases_path).stem+".registry.json")


def create_lock(cases_path: Path, cases: dict, policy_dir: Path, target: Path,
                manifest: dict, identity: str) -> dict:
    target = Path(target).resolve()
    if target.exists():
        raise FileExistsError(f"锁文件已存在，拒绝覆盖：{target}")
    contract = json.loads((Path(policy_dir)/"policy.json").read_text(encoding="utf-8"))
    if contract.get("library_sha256") != manifest["library_sha256"] or contract.get("experiment_identity") != identity:
        raise RuntimeError("策略与当前冻结物理源不匹配")
    if cases["split"] == "final" and contract.get("seed") not in TRAIN_SEEDS:
        raise RuntimeError(f"最终策略 seed 必须属于 {TRAIN_SEEDS}")
    registry = None
    reg_path = registry_path(cases_path)
    if cases["split"] == "final" and reg_path.exists():
        registry = json.loads(reg_path.read_text(encoding="utf-8"))
        if registry.get("scenarios_sha256") != cases["cases_sha256"]:
            raise RuntimeError("最终评价登记表与当前场景哈希不匹配")
        if registry.get("closed_at"):
            raise RuntimeError("最终评价已经开始，拒绝登记新策略")
    core = _lock_core(cases, policy_dir, contract, manifest, identity)
    payload = dict(core, lock_sha256=hashlib.sha256(canonical_bytes(core)).hexdigest())
    if cases["split"] == "final":
        if registry is None:
            registry = {"schema": 1, "scenarios_sha256": cases["cases_sha256"],
                        "registered_locks": [], "registered_seeds": {},
                        "used_locks": {}, "closed_at": None}
        seed_key = str(payload["policy_seed"])
        previous = registry.setdefault("registered_seeds", {}).get(seed_key)
        if previous is not None and previous != payload["lock_sha256"]:
            raise RuntimeError(f"最终训练 seed {seed_key} 已登记另一策略")
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(payload, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    if cases["split"] == "final":
        if payload["lock_sha256"] not in registry["registered_locks"]:
            registry["registered_locks"].append(payload["lock_sha256"])
        registry["registered_seeds"][seed_key] = payload["lock_sha256"]
        atomic_write_json(reg_path, registry)
    return payload


def verify_lock(lock_path: Path, cases_path: Path, cases: dict, policy_dir: Path,
                contract: dict, manifest: dict, identity: str) -> dict:
    payload = json.loads(Path(lock_path).read_text(encoding="utf-8"))
    core = {key:value for key,value in payload.items() if key != "lock_sha256"}
    if payload.get("lock_sha256") != hashlib.sha256(canonical_bytes(core)).hexdigest():
        raise RuntimeError("最终评价锁自身哈希不匹配")
    expected = _lock_core(cases, policy_dir, contract, manifest, identity)
    for key in ("scenarios_sha256", "split", "policy_directory", "policy_seed", "policy_artifacts",
                "library_sha256", "experiment_identity", "env_config"):
        if payload.get(key) != expected.get(key):
            raise RuntimeError(f"最终评价锁字段不匹配：{key}")
    if cases["split"] == "final":
        reg_path = registry_path(cases_path)
        if not reg_path.is_file():
            raise RuntimeError("最终策略没有预先登记")
        registry = json.loads(reg_path.read_text(encoding="utf-8"))
        lock_hash = payload["lock_sha256"]
        if lock_hash not in registry.get("registered_locks", []):
            raise RuntimeError("最终策略锁未登记")
    return payload


def claim_final_lock(cases_path: Path, cases: dict, lock: dict, output: Path,
                     resume: bool, binding_hash: str) -> dict | None:
    if cases["split"] != "final":
        return None
    reg_path = registry_path(cases_path)
    registry = json.loads(reg_path.read_text(encoding="utf-8"))
    lock_hash = lock["lock_sha256"]
    resolved_output = str(Path(output).resolve())
    usage = registry.setdefault("used_locks", {}).get(lock_hash)
    if resume:
        if usage is None or usage.get("output") != resolved_output:
            raise RuntimeError("最终锁只允许回到原登记输出目录继续")
        if usage.get("binding_sha256") != binding_hash:
            raise RuntimeError("最终锁恢复时的代码、场景、策略或配置绑定不一致")
        if usage.get("status") == "completed" or usage.get("completed_at"):
            raise RuntimeError("该最终评价已有完整 summary，禁止再次运行")
        return usage
    if usage is not None:
        raise RuntimeError("该最终策略已经开始或完成评价，拒绝换目录重跑")
    if not registry.get("closed_at"):
        missing = sorted(set(map(str, TRAIN_SEEDS))-set(registry.get("registered_seeds", {})))
        if missing:
            raise RuntimeError(f"首次最终评价前必须登记三个训练 seed，尚缺：{missing}")
        registry["closed_at"] = now_cn()
    usage = {"started_at": now_cn(), "output": resolved_output,
             "binding_sha256": binding_hash, "status": "in_progress"}
    registry["used_locks"][lock_hash] = usage
    atomic_write_json(reg_path, registry)
    return usage


def complete_final_lock(cases_path: Path, cases: dict, lock: dict,
                        output: Path, summary_path: Path) -> None:
    if cases["split"] != "final":
        return
    reg_path = registry_path(cases_path)
    registry = json.loads(reg_path.read_text(encoding="utf-8"))
    usage = registry.get("used_locks", {}).get(lock["lock_sha256"])
    if usage is None or usage.get("output") != str(Path(output).resolve()):
        raise RuntimeError("最终评价完成登记与输出目录不匹配")
    if usage.get("status") == "completed":
        raise RuntimeError("最终评价已经登记完成")
    usage.update(status="completed", completed_at=now_cn(),
                 summary_sha256=sha256_file(summary_path))
    atomic_write_json(reg_path, registry)


def _cache_file(path: Path) -> Path:
    path = Path(path)
    return path/"summary.json" if path.is_dir() else path


def load_baseline_cache(path: Path, cases: dict, manifest: dict, identity: str,
                        env_config: dict) -> list[dict]:
    payload = json.loads(_cache_file(path).read_text(encoding="utf-8"))
    bindings = {"scenarios_sha256": cases["cases_sha256"],
                "library_sha256": manifest["library_sha256"],
                "experiment_identity": identity, "env_config": env_config}
    for key, expected in bindings.items():
        if payload.get(key) != expected:
            raise RuntimeError(f"基线缓存绑定不匹配：{key}")
    rows = payload.get("baseline_results")
    if not isinstance(rows, list) or len(rows) != len(cases["cases"]):
        raise RuntimeError("基线缓存不完整")
    expected = {case["id"]: case for case in cases["cases"]}
    if {row.get("id") for row in rows} != set(expected):
        raise RuntimeError("基线缓存场景 id 不匹配")
    for row in rows:
        case = expected[row["id"]]
        if row.get("seed") != case["seed"] or row.get("randomize") != case["randomize"] or row.get("options") != case["options"]:
            raise RuntimeError(f"基线缓存场景内容不匹配：{row.get('id')}")
    return rows


def run_binding(cases: dict, manifest: dict, identity: str, env_config: dict,
                policy_dir: Path | None, lock: dict | None,
                baseline_cache: Path | None, record: bool) -> dict:
    cache_file = _cache_file(baseline_cache).resolve() if baseline_cache else None
    return {
        "schema": 1,
        "split": cases["split"],
        "scenarios_sha256": cases["cases_sha256"],
        "library_sha256": manifest["library_sha256"],
        "experiment_identity": identity,
        "evaluator_sha256": sha256_file(Path(__file__)),
        "env_config": env_config,
        "policy_directory": str(Path(policy_dir).resolve()) if policy_dir else None,
        "policy_artifacts": policy_artifacts(policy_dir) if policy_dir else None,
        "policy_lock_sha256": lock["lock_sha256"] if lock else None,
        "baseline_cache_sha256": sha256_file(cache_file) if cache_file else None,
        "record": bool(record),
    }


def prepare_output(output: Path, binding: dict, resume: bool,
                   allow_resume_create: bool = False) -> str:
    output = Path(output)
    binding_hash = hashlib.sha256(canonical_bytes(binding)).hexdigest()
    run_path = output/"run.json"
    if resume:
        if not output.exists():
            if not allow_resume_create:
                raise FileNotFoundError(f"恢复目录不存在：{output}")
            output.mkdir(parents=True)
        if run_path.exists():
            saved = json.loads(run_path.read_text(encoding="utf-8"))
            if saved.get("binding_sha256") != binding_hash or saved.get("binding") != binding:
                raise RuntimeError("恢复运行的代码、场景、策略或配置绑定不一致")
        elif any(output.iterdir()):
            raise RuntimeError("恢复目录缺少 run.json 且不为空")
        else:
            atomic_write_json(run_path, {"schema": 1, "binding": binding,
                                         "binding_sha256": binding_hash})
    else:
        if output.exists():
            raise FileExistsError(f"评价输出目录已存在，拒绝覆盖：{output}")
        output.mkdir(parents=True)
        atomic_write_json(run_path, {"schema": 1, "binding": binding,
                                     "binding_sha256": binding_hash})
    return binding_hash


class EvaluationInterrupted(RuntimeError):
    pass


def run_case(scenario: dict, policy_or_none, env_config: dict, record: bool = False,
             stop_requested=None) -> dict:
    """运行一个固定场景；record=True 时在返回值中附带完整轨迹。"""
    from WbrLearningEnv import WbrLearningEnv
    env = WbrLearningEnv(randomize=bool(scenario["randomize"]), strict=True,
                         record=True, **env_config)
    try:
        observation, _ = env.reset(seed=int(scenario["seed"]),
                                   options=dict(scenario["options"]))
        origin = env.data.xpos[env.base].copy()
        origin_rotation = env.data.xmat[env.base].reshape(3,3)
        origin_yaw = math.atan2(origin_rotation[1,0],origin_rotation[0,0])
        while True:
            if stop_requested and stop_requested():
                raise EvaluationInterrupted("评价收到停止请求；未完成场景留待恢复")
            action = (np.zeros(env.action_space.shape, dtype=np.float32)
                      if policy_or_none is None else policy_or_none.predict(observation))
            observation, _, terminated, truncated, _ = env.step(action)
            velocity = env.jac @ env.data.qvel
            rotation = env.data.xmat[env.base].reshape(3,3)
            yaw = math.atan2(rotation[1,0],rotation[0,0])-origin_yaw
            env.trace[-1].update(world_vx_mps=float(velocity[0]),world_vy_mps=float(velocity[1]),
                lateral_m=float(env.data.xpos[env.base,1]-origin[1]),
                longitudinal_m=float(env.data.xpos[env.base,0]-origin[0]),
                yaw_relative_rad=math.atan2(math.sin(yaw),math.cos(yaw)))
            if terminated or truncated:
                break
        summary = env.summary()
        row = {"id": scenario["id"], "seed": scenario["seed"],
               "randomize": scenario["randomize"], "case": scenario["options"]["case"],
               "options": scenario["options"],
               "metrics": summarize_trace(scenario, env.trace, summary)}
        if summary["passed"]:
            row["motion_quality"] = motion_quality_metrics(scenario,env.trace)
        if record:
            row["trace"] = list(env.trace)
        return row
    finally:
        env.close()


def _completed_path(output: Path, label: str, case_id: str) -> Path:
    digest = hashlib.sha256(case_id.encode("utf-8")).hexdigest()[:16]
    return output/"completed"/f"{label}-{digest}.json"


def run_controller(cases: dict, env_config: dict, policy, output: Path,
                   label: str, record: bool, binding_hash: str, resume: bool,
                   cached_rows: list[dict] | None = None, deadline: float | None = None) -> list[dict]:
    results = []
    cached = {row["id"]: row for row in cached_rows} if cached_rows is not None else {}
    def stop_requested():
        return (output/"stop.request").exists() or bool(deadline and time.time() >= deadline)
    for case in cases["cases"]:
        if stop_requested():
            raise EvaluationInterrupted("评价收到停止请求；已完成场景已保存")
        completed_path = _completed_path(output, label, case["id"])
        if completed_path.exists():
            if not resume:
                raise RuntimeError(f"新评价意外存在完成行：{case['id']} {label}")
            completed = json.loads(completed_path.read_text(encoding="utf-8"))
            row = completed.get("row")
            if (completed.get("binding_sha256") != binding_hash or not isinstance(row, dict) or
                    row.get("id") != case["id"] or row.get("seed") != case["seed"] or
                    row.get("randomize") != case["randomize"] or row.get("options") != case["options"] or
                    row.get("controller") != label):
                raise RuntimeError(f"已完成场景绑定不匹配：{case['id']} {label}")
            if record and not (output/"traces"/f"{case['id']}-{label}.json").is_file():
                raise RuntimeError(f"已完成场景缺少要求的轨迹：{case['id']} {label}")
            results.append(row)
            print(json.dumps({"resumed": True, "id": case["id"], "controller": label},
                             ensure_ascii=False), flush=True)
            continue
        if case["id"] in cached:
            row = dict(cached[case["id"]])
            row["controller"] = label
            trace = None
        else:
            row = run_case(case, policy, env_config, record=record, stop_requested=stop_requested)
            trace = row.pop("trace", None)
            row["controller"] = label
        if trace is not None:
            atomic_write_json(output/"traces"/f"{case['id']}-{label}.json", trace)
        atomic_write_json(completed_path, {"schema": 1, "binding_sha256": binding_hash,
                                           "row": row})
        results.append(row)
        print(json.dumps(row, ensure_ascii=False), flush=True)
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cases", required=True, type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--baseline-only", action="store_true")
    parser.add_argument("--policy", type=Path)
    parser.add_argument("--baseline-cache", type=Path)
    parser.add_argument("--create-lock", type=Path)
    parser.add_argument("--lock", type=Path)
    parser.add_argument("--reward-mode", default="dense", choices=("dense", "legacy"))
    parser.add_argument("--action-mode", default="full", choices=("full", "symmetric"))
    parser.add_argument("--record", action="store_true")
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--deadline", default="2026-10-14T02:10:00+08:00")
    parser.add_argument("--snapshot", type=Path, help="使用已有冻结物理快照；评价脚本单独绑定其SHA")
    args = parser.parse_args()
    cases = load_cases(args.cases)
    if args.snapshot:
        sys.path.insert(0,str(args.snapshot.resolve()/"tools/mujoco/learning"))
    from WbrLearningEnv import experiment_identity, verify_snapshot
    manifest = verify_snapshot()
    identity = experiment_identity(manifest)

    if args.create_lock:
        if not args.policy or args.baseline_only:
            parser.error("--create-lock 必须同时提供 --policy")
        payload = create_lock(args.cases, cases, args.policy, args.create_lock, manifest, identity)
        print(json.dumps(payload, ensure_ascii=False, indent=2))
        return
    if bool(args.baseline_only) == bool(args.policy):
        parser.error("评价时必须二选一：--baseline-only 或 --policy")
    if args.output is None:
        parser.error("评价必须提供 --output")
    output = args.output.resolve()
    if args.record and args.baseline_cache:
        parser.error("--record 不能配合基线缓存，因为缓存不含可核验的完整轨迹")

    policy = None
    lock = None
    if args.policy:
        from StudyWbrPolicy import StudyPolicy
        policy = StudyPolicy(args.policy)
        contract = policy.contract
        if contract.get("library_sha256") != manifest["library_sha256"] or contract.get("experiment_identity") != identity:
            raise RuntimeError("策略与当前冻结物理源不匹配")
        env_config = contract.get("env_config")
        if not isinstance(env_config, dict):
            raise RuntimeError("策略契约缺少 env_config")
        if cases["split"] == "final":
            if args.lock is None:
                raise RuntimeError("最终评价必须提供预先生成的 --lock")
            lock = verify_lock(args.lock, args.cases, cases, args.policy, contract,
                               manifest, identity)
    else:
        env_config = {"reward_mode": args.reward_mode, "action_mode": args.action_mode,
                      "gas_force": 450.0}
        if cases["split"] == "final":
            raise RuntimeError("最终集禁止提前单跑基线；请锁定策略后成对评价")

    cached_baseline = None
    if args.baseline_cache:
        cached_baseline = load_baseline_cache(
            args.baseline_cache, cases, manifest, identity, env_config)
        if any(row["metrics"]["passed"] and "motion_quality" not in row for row in cached_baseline):
            raise RuntimeError("基线缓存缺少新增运动质量遥测，请用当前评价器重新生成基线")
    binding = run_binding(cases, manifest, identity, env_config, args.policy,
                          lock, args.baseline_cache, args.record)
    expected_binding_hash = hashlib.sha256(canonical_bytes(binding)).hexdigest()
    if not args.resume and output.exists():
        raise FileExistsError(f"评价输出目录已存在，拒绝覆盖：{output}")
    if args.resume:
        # 恢复时先验证输出目录中的完整运行绑定，再读取最终锁使用状态。
        binding_hash = prepare_output(output, binding, True,
                                      allow_resume_create=bool(lock))
        if lock:
            claim_final_lock(args.cases, cases, lock, output, True,
                             expected_binding_hash)
    else:
        if lock:
            claim_final_lock(args.cases, cases, lock, output, False,
                             expected_binding_hash)
        binding_hash = prepare_output(output, binding, False)
    if binding_hash != expected_binding_hash:
        raise RuntimeError("内部运行绑定哈希不一致")
    summary_path = output/"summary.json"
    if summary_path.exists():
        if lock:
            complete_final_lock(args.cases, cases, lock, output, summary_path)
            saved_summary = json.loads(summary_path.read_text(encoding="utf-8"))
            print(json.dumps(saved_summary["acceptance"], ensure_ascii=False, indent=2))
            return
        raise RuntimeError("该输出已有完整 summary，禁止再次运行")
    started = time.perf_counter()
    deadline = datetime.fromisoformat(args.deadline).timestamp() if args.deadline else None
    try:
        baseline = run_controller(cases, env_config, None, output, "baseline", args.record,
                                  binding_hash, args.resume, cached_rows=cached_baseline, deadline=deadline)
        learned = (run_controller(cases, env_config, policy, output, "learned", args.record,
                                  binding_hash, args.resume, deadline=deadline) if policy else [])
    except EvaluationInterrupted as error:
        atomic_write_json(output/"status.json", dict(status="interrupted", reason=str(error),
                          updated_at=now_cn()))
        print(json.dumps(dict(status="interrupted", output=str(output)), ensure_ascii=False), flush=True)
        return
    result = {
        "schema": 1, "created_at": now_cn(), "split": cases["split"],
        "scenarios_sha256": cases["cases_sha256"], "library_sha256": manifest["library_sha256"],
        "experiment_identity": identity, "env_config": env_config,
        "policy_lock_sha256": lock["lock_sha256"] if lock else None,
        "run_binding_sha256": binding_hash,
        "elapsed_s": time.perf_counter()-started, "baseline_results": baseline,
        "learned_results": learned,
        "acceptance": acceptance(baseline, learned) if policy else {
            "overall_pass": all(row["metrics"]["passed"] for row in baseline),
            "checks": {"physical_complete": {
                "pass": all(row["metrics"]["passed"] for row in baseline)}},
            "cases": [{"id": row["id"], "pass": row["metrics"]["passed"],
                       "failure": row["metrics"]["failure"]} for row in baseline]},
    }
    if policy:
        quality = motion_quality_acceptance(baseline,learned)
        result["acceptance"]["checks"]["motion_quality"] = quality
        result["acceptance"]["overall_pass"] = bool(result["acceptance"]["overall_pass"] and quality["pass"])
    result["motion_quality_protocol"] = "motion-quality-20261012"
    atomic_write_json(summary_path, result)
    if lock:
        complete_final_lock(args.cases, cases, lock, output, summary_path)
    print(json.dumps(result["acceptance"], ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
