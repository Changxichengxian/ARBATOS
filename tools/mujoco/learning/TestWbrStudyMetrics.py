"""不启动 MuJoCo，检查长期研究场景和验收指标的纯逻辑。"""
from __future__ import annotations

import json
from pathlib import Path
import tempfile

import numpy as np

from BuildWbrStudyCases import generate
from EvaluateWbrStudy import (EvaluationInterrupted, acceptance, claim_final_lock, complete_final_lock,
                              load_baseline_cache, prepare_output, run_controller,
                              summarize_trace, load_cases)


def make_trace(velocity: float, command: float, *, duration: float = 8.0) -> list[dict]:
    rows = []
    x = 0.0
    for time_s in np.arange(0.015, duration + 0.0001, 0.015):
        x += velocity * 0.015
        rows.append({
            "time_s": float(time_s),
            "true_v_mps": velocity,
            "command_v_mps": command,
            "x_m": x,
            "pitch_rad": 0.01,
            "roll_rad": -0.015,
            "closure_m": 0.03,
        })
    return rows


def metrics(case_name: str, velocity: float, heat: float, *, passed: bool = True) -> dict:
    case = {"options": {"case": case_name, "speed": 0.5}}
    summary = {"passed": passed, "failure": None if passed else "synthetic_failure",
               "hip_heat_nm2s": heat, "simulated_s": 8.0 if passed else 1.2}
    command = 0.0 if case_name == "stop" else 0.5
    return summarize_trace(
        case, make_trace(velocity, command, duration=8.0 if passed else 1.2), summary)


def result(case_id: str, case_name: str, velocity: float, heat: float,
           *, passed: bool = True) -> dict:
    return {"id": case_id, "case": case_name,
            "metrics": metrics(case_name, velocity, heat, passed=passed)}


def main() -> None:
    with tempfile.TemporaryDirectory() as directory:
        generated = generate(Path(directory)/"cases")
        expected = {"development": 6, "validation": 45, "final": 60, "pressure": 6}
        assert {name: value["case_count"] for name, value in generated.items()} == expected
        for name, count in expected.items():
            loaded = load_cases(Path(directory)/"cases"/f"{name}.json")
            assert loaded["case_count"] == count
        final_counts = {name: 0 for name in ("stop", "push", "combined")}
        for case in generated["final"]["cases"]:
            final_counts[case["options"]["case"]] += 1
        assert final_counts == {"stop": 24, "push": 24, "combined": 12}

    baseline = [result("s", "stop", 0.20, 100.0), result("p", "push", 0.30, 100.0)]
    learned = [result("s", "stop", 0.17, 104.0), result("p", "push", 0.33, 104.0)]
    passed = acceptance(baseline, learned)
    assert passed["overall_pass"]
    assert all(check["pass"] for check in passed["checks"].values())
    assert passed["checks"]["push_mean_improvement"]["metric"] == "velocity_error_rms_mps"

    regressed = [result("s", "stop", 0.212, 100.0), result("p", "push", 0.33, 100.0)]
    failed_guard = acceptance(baseline, regressed)
    assert not failed_guard["overall_pass"]
    stop_case = next(row for row in failed_guard["cases"] if row["id"] == "s")
    assert not stop_case["checks"]["stop_primary"]["pass"]

    failed = [result("s", "stop", 0.0, 1.0, passed=False), learned[1]]
    failed_physics = acceptance(baseline, failed)
    assert not failed_physics["checks"]["physical_complete"]["pass"]
    assert failed[0]["metrics"]["windows"] == {}

    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        binding = {"scenario": "abc", "code": "def", "env": {"gas_force": 450.0}}
        binding_hash = prepare_output(root/"evaluation", binding, resume=False)
        assert prepare_output(root/"evaluation", binding, resume=True) == binding_hash
        try:
            prepare_output(root/"evaluation", dict(binding, code="changed"), resume=True)
        except RuntimeError:
            pass
        else:
            raise AssertionError("恢复时没有拒绝代码绑定变化")

        case = {"id": "resume-001", "seed": 7, "randomize": False,
                "options": {"case": "stop", "speed": 0.5, "push": 20.0,
                            "delay": 0, "duration": 8.0}}
        cached_row = {**case, "case": "stop", "metrics": metrics("stop", 0.2, 100.0)}
        cache_path = root/"baseline.json"
        cache_path.write_text(json.dumps({
            "scenarios_sha256": "case-hash", "library_sha256": "library-hash",
            "experiment_identity": "identity",
            "env_config": {"reward_mode": "dense", "action_mode": "full",
                           "gas_force": 450.0},
            "baseline_results": [cached_row],
        }), encoding="utf-8")
        cache_cases = {"cases_sha256": "case-hash", "cases": [case]}
        cache_manifest = {"library_sha256": "library-hash"}
        assert load_baseline_cache(
            cache_path, cache_cases, cache_manifest, "identity",
            {"reward_mode": "dense", "action_mode": "full", "gas_force": 450.0}) == [cached_row]
        try:
            load_baseline_cache(
                cache_path, cache_cases, cache_manifest, "identity",
                {"reward_mode": "dense", "action_mode": "full", "gas_force": 430.0})
        except RuntimeError:
            pass
        else:
            raise AssertionError("基线缓存没有绑定气簧配置")
        first = run_controller({"cases": [case]}, {}, None, root/"evaluation",
                               "baseline", False, binding_hash, False, [cached_row])
        resumed = run_controller({"cases": [case]}, {}, None, root/"evaluation",
                                 "baseline", False, binding_hash, True, [cached_row])
        assert first == resumed
        (root/"evaluation"/"stop.request").touch()
        try:
            run_controller({"cases": [case]}, {}, None, root/"evaluation",
                           "baseline", False, binding_hash, True, [cached_row])
        except EvaluationInterrupted:
            pass
        else:
            raise AssertionError("评价没有响应停止文件")
        (root/"evaluation"/"stop.request").unlink()
        try:
            run_controller({"cases": [case]}, {}, None, root/"evaluation",
                           "baseline", False, binding_hash, True, [cached_row], deadline=1.)
        except EvaluationInterrupted:
            pass
        else:
            raise AssertionError("评价没有响应截止时间")

        cases_path = root/"final.json"
        registry = {"schema": 1, "scenarios_sha256": "scenarios",
                    "registered_locks": ["lock"],
                    "registered_seeds": {str(seed): f"lock-{seed}" for seed in (101, 211, 307)},
                    "used_locks": {}, "closed_at": None}
        cases_path.write_text("{}", encoding="utf-8")
        cases_path.with_name("final.registry.json").write_text(
            json.dumps(registry), encoding="utf-8")
        final_cases = {"split": "final", "cases_sha256": "scenarios"}
        lock = {"lock_sha256": "lock"}
        claim_final_lock(cases_path, final_cases, lock, root/"final-output", False,
                         binding_hash)
        claim_final_lock(cases_path, final_cases, lock, root/"final-output", True,
                         binding_hash)
        try:
            claim_final_lock(cases_path, final_cases, lock, root/"final-output", True,
                             "changed-binding")
        except RuntimeError:
            pass
        else:
            raise AssertionError("最终锁恢复时没有拒绝运行绑定变化")
        try:
            claim_final_lock(cases_path, final_cases, lock, root/"other-output", True,
                             binding_hash)
        except RuntimeError:
            pass
        else:
            raise AssertionError("最终锁允许换输出目录恢复")
        summary = root/"summary.json"
        summary.write_text("{}", encoding="utf-8")
        complete_final_lock(cases_path, final_cases, lock, root/"final-output", summary)
        try:
            claim_final_lock(cases_path, final_cases, lock, root/"final-output", True,
                             binding_hash)
        except RuntimeError:
            pass
        else:
            raise AssertionError("已完成最终评价允许再次恢复")

    report = {
        "scenario_generation": {"pass": True, "counts": expected},
        "passing_candidate": {"pass": passed["overall_pass"]},
        "case_regression_rejected": {"pass": not failed_guard["overall_pass"]},
        "short_physical_failure_rejected": {
            "pass": not failed_physics["overall_pass"],
            "windows_computed": bool(failed[0]["metrics"]["windows"]),
        },
        "resume_binding_and_final_once": {"pass": True},
    }
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
