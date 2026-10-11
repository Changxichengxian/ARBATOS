"""记录轮腿控制核心的物理仿真基线；通过也不代表实车或完整固件通过。"""

from __future__ import annotations

import argparse
import csv
import ctypes
import hashlib
import json
import math
import platform
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

import RunWheelLeg as runner


ACTUATORS = (
    "right_front_motor", "right_back_motor", "right_wheel_motor",
    "left_front_motor", "left_back_motor", "left_wheel_motor",
)


def git_text(*args: str) -> str:
    result = subprocess.run(
        ["git", *args], cwd=runner.REPO_ROOT, capture_output=True, encoding="utf-8", check=True,
    )
    return result.stdout.strip()


def loop_error(data) -> float:
    return max(
        math.dist(data.site(f"{side}_{link}_rod_end").xpos, data.site(f"{side}_foot_site").xpos)
        for side in ("right", "left") for link in ("front", "back")
    )


def sample(mujoco, model, data, output, args, origin) -> dict:
    roll, pitch, yaw, *_ = runner.base_orientation(mujoco, model, data)
    # 取机身原点的世界系速度，避免把质心的转动分量算成整车前进速度。
    import numpy as np
    jacobian = np.zeros((3, model.nv))
    mujoco.mj_jacBody(model, data, jacobian, None, model.body("base").id)
    velocity = jacobian @ data.qvel
    pos = data.body("base").xpos
    forward_v = float(velocity[0] * math.cos(yaw) + velocity[1] * math.sin(yaw))
    row = {
        "time_s": float(data.time), "target_v_mps": float(args.target_v),
        "pitch_deg": math.degrees(pitch), "roll_deg": math.degrees(roll),
        "yaw_deg": math.degrees(yaw), "base_x_m": float(pos[0]),
        "base_y_m": float(pos[1]), "base_z_m": float(pos[2]),
        "displacement_m": math.hypot(float(pos[0]) - origin[0], float(pos[1]) - origin[1]),
        "true_forward_v_mps": forward_v, "observer_v_mps": float(output.observer_v_mps),
        "loop_error_m": loop_error(data), "contacts": int(data.ncon),
    }
    for side, index in (("right", runner.SIDE_RIGHT), ("left", runner.SIDE_LEFT)):
        hip = data.body("base").xpos.copy()
        hip += data.body("base").xmat.reshape(3, 3) @ np.array(
            [0.0, -runner.SIM_TRACK_HALF_M if side == "right" else runner.SIM_TRACK_HALF_M, 0.0]
        )
        row[f"{side}_true_leg_m"] = float(np.linalg.norm(data.site(f"{side}_foot_site").xpos - hip))
        row[f"{side}_core_leg_m"] = float(output.leg_length_m[index])
        row[f"{side}_core_theta_rad"] = float(output.leg_theta_rad[index])
    for i, name in enumerate(ACTUATORS):
        row[f"{name}_nm"] = runner.sim_actuator_torque(i, output, args)
    return row


def run_case(lib, mujoco, options, mode: str, case: str, speed: float) -> dict:
    case_dir = options.output / f"{mode}-{case}"
    args = runner.parse_args([
        "--project", options.project, "--vmc", "--native-dir", str(case_dir),
        "--dt", str(options.dt), *([] if mode == "free" else ["--planar"]),
    ])
    args.jump_until_perf = 0.0
    config = runner.BridgeConfig()
    lib.arbatos_wheelleg_bridge_config_defaults(ctypes.byref(config))
    runner.load_project_config(config, options.project)
    model_path = runner.resolve_model_path(args, config)
    model = mujoco.MjModel.from_xml_path(str(model_path))
    model.opt.timestep = options.dt
    data = mujoco.MjData(model)
    state = runner.BridgeState()
    lib.arbatos_wheelleg_bridge_state_init(ctypes.byref(state), ctypes.byref(config))
    front, back = runner.initial_home_pose(config, args)
    runner.initialize_model_pose(mujoco, model, data, config, front, back)
    mujoco.mj_forward(model, data)
    origin = data.body("base").xpos.copy()
    ids = {i: runner.actuator_id(mujoco, model, name) for i, name in enumerate(ACTUATORS)}
    rows, failures = [], []
    command_reached = False
    steps = math.ceil(options.duration / options.dt)
    trace_path = case_dir / "trace.csv"
    writer = None
    with trace_path.open("w", encoding="utf-8", newline="") as stream:
        for step in range(steps + 1):
            if not all(math.isfinite(float(v)) for v in (*data.qpos, *data.qvel)):
                failures.append("nonfinite_state")
                break
            args.target_v = speed if data.time >= options.command_time else 0.0
            command_reached |= speed != 0.0 and args.target_v == speed
            try:
                output = runner.run_step(lib, config, state, mujoco, model, data, args)
            except RuntimeError as exc:
                failures.append(str(exc))
                break
            row = sample(mujoco, model, data, output, args, origin)
            if not all(math.isfinite(float(value)) for value in row.values()):
                failures.append("nonfinite_output")
                break
            if writer is None:
                writer = csv.DictWriter(stream, fieldnames=list(row))
                writer.writeheader()
            writer.writerow(row)
            rows.append(row)
            if max(abs(row["pitch_deg"]), abs(row["roll_deg"])) > options.fall_angle:
                failures.append("fall_angle")
            if row["base_z_m"] < float(config.wheel_radius_m) + 0.025:
                failures.append("base_too_low")
            if row["loop_error_m"] > options.loop_limit:
                failures.append("closed_chain_error")
            if failures or step == steps:
                break
            runner.apply_output(mujoco, model, data, ids, output, args)
            previous_time = float(data.time)
            mujoco.mj_step(model, data)
            mujoco.mj_forward(model, data)
            if data.time <= previous_time or any(w.number for w in data.warning):
                failures.append("mujoco_warning_or_reset")
                break

    tracking = [row for row in rows if row["time_s"] >= options.command_time + 1.0]
    rmse = (math.sqrt(sum((row["true_forward_v_mps"] - speed) ** 2 for row in tracking)
                      / len(tracking)) if tracking else None)
    if not failures:
        if case == "stand" and max(row["displacement_m"] for row in rows) > options.drift_limit:
            failures.append("stand_drift")
        if rmse is None or rmse > options.speed_rmse:
            failures.append("speed_tracking")
        if speed != 0.0 and not command_reached:
            failures.append("command_not_reached")
    base = model.body("base")
    return {
        "mode": mode, "case": case, "passed": not failures,
        "failures": failures, "time_s": float(data.time),
        "requested_speed_mps": speed, "command_reached": command_reached,
        "peak_pitch_deg": max((abs(r["pitch_deg"]) for r in rows), default=None),
        "peak_roll_deg": max((abs(r["roll_deg"]) for r in rows), default=None),
        "max_displacement_m": max((r["displacement_m"] for r in rows), default=None),
        "max_loop_error_m": max((r["loop_error_m"] for r in rows), default=None),
        "speed_rmse_mps": rmse,
        "observer_rmse_mps": (math.sqrt(sum((r["observer_v_mps"] - r["true_forward_v_mps"]) ** 2
                                           for r in rows) / len(rows)) if rows else None),
        "wheel_saturation_fraction": (sum(
            any(abs(r[f"{ACTUATORS[i]}_nm"]) >= 0.99 * float(config.max_wheel_torque_nm)
                for i in runner.WHEEL_ACTUATORS) for r in rows
        ) / len(rows) if rows else None),
        "model_total_mass_kg": float(sum(model.body_mass)),
        "base_com_local_m": base.ipos.tolist(), "base_inertia_kgm2": base.inertia.tolist(),
        "model_sha256": hashlib.sha256(model_path.read_bytes()).hexdigest(),
        "model": str(model_path), "trace": str(trace_path),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project", default="MINIWHEELEG-M")
    parser.add_argument("--modes", nargs="+", choices=("free", "planar"), default=["free", "planar"])
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--dt", type=float, default=0.003)
    parser.add_argument("--command-time", type=float, default=2.0)
    parser.add_argument("--speed", type=float, default=0.15)
    parser.add_argument("--fall-angle", type=float, default=45.0)
    parser.add_argument("--loop-limit", type=float, default=0.01)
    parser.add_argument("--drift-limit", type=float, default=0.10)
    parser.add_argument("--speed-rmse", type=float, default=0.05)
    parser.add_argument("--output", type=Path, default=runner.DEFAULT_NATIVE_DIR / datetime.now().strftime("eval-%Y%m%d-%H%M%S"))
    options = parser.parse_args()
    for name in ("duration", "dt", "speed", "fall_angle", "loop_limit", "drift_limit", "speed_rmse"):
        value = getattr(options, name)
        if not math.isfinite(value) or value <= 0:
            parser.error(f"{name} must be finite and positive")
    if not 0 <= options.command_time < options.duration - 1.0 or options.dt > options.duration:
        parser.error("command-time must leave more than one second for speed measurement; dt <= duration")
    options.output = options.output.resolve()
    options.output.mkdir(parents=True, exist_ok=True)
    mujoco = runner.import_mujoco()
    lib = runner.load_bridge(options.output / "native", rebuild=True)
    sources = [Path(__file__), runner.HERE / "RunWheelLeg.py", *runner.source_dependencies(),
               *sorted((runner.REPO_ROOT / "Robotconfig" / options.project).glob("Config*.inc")),
               runner.REPO_ROOT / "Robotconfig" / options.project / "RobotConfig.c"]
    report = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "git_head": git_text("rev-parse", "HEAD"), "mujoco": mujoco.__version__,
        "python": platform.python_version(), "command": sys.argv,
        "scope": "VMC control-core bridge only; not full firmware or physical robot acceptance",
        "thresholds": {key: value for key, value in vars(options).items() if key != "output"},
        "source_sha256": {str(path.relative_to(runner.REPO_ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                          for path in sources if path.is_file()},
        "results": [],
    }
    for mode in dict.fromkeys(options.modes):
        for case, speed in (("stand", 0.0), ("forward", options.speed), ("reverse", -options.speed)):
            result = run_case(lib, mujoco, options, mode, case, speed)
            report["results"].append(result)
            print(f"{mode}/{case}: {'PASS' if result['passed'] else 'FAIL'} "
                  f"t={result['time_s']:.3f}s {','.join(result['failures'])}", flush=True)
    report["passed"] = all(result["passed"] for result in report["results"])
    path = options.output / "summary.json"
    path.write_text(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")
    print(f"Report: {path}")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
