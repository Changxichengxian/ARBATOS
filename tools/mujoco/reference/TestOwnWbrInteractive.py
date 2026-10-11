"""用真实 WBR MuJoCo 模型验收交互指令与跳跃，不参与控制反馈。"""
from __future__ import annotations

import argparse
import csv
import ctypes
import json
from pathlib import Path
import subprocess

import mujoco
import numpy as np

from RunOwnWbr import load_core
from RunWbr import MOTORS, REVISION, ROOT, SOURCE, inspect_state
from WbrGasSpring import load_model


def command_at(time_s: float, jump_count: int,
               moving_jump: bool) -> tuple[float, float, float]:
    target_v = .20 if 5.0 <= time_s < 7.0 else (-.20 if 8.0 <= time_s < 10.0 else 0.0)
    target_yaw = .40 if 11.0 <= time_s < 13.0 else (-.40 if 14.0 <= time_s < 16.0 else 0.0)
    if moving_jump:
        jump = 1.0 if 5.50 <= time_s < 5.55 else 0.0
    else:
        jump = float(any(start <= time_s < start + .30
                         for start in (17.0 + 4.0 * index for index in range(jump_count))))
    return target_v, target_yaw, jump


def longest_true_run(times: list[float], flags: list[bool]) -> tuple[float, float, float]:
    best = (0.0, 0.0, 0.0)
    start = None
    previous = None
    for time_s, active in zip(times, flags):
        if active and start is None:
            start = time_s
        if active:
            previous = time_s
        elif start is not None:
            candidate = (previous - start + .001, start, previous)
            best = max(best, candidate)
            start = None
    if start is not None:
        best = max(best, (previous - start + .001, start, previous))
    return best


def true_runs(times: list[float], flags: list[bool]) -> list[tuple[float, float, float]]:
    runs = []
    start = None
    previous = None
    for time_s, active in zip(times, flags):
        if active and start is None:
            start = time_s
        if active:
            previous = time_s
        elif start is not None:
            runs.append((previous - start + .001, start, previous))
            start = None
    if start is not None:
        runs.append((previous - start + .001, start, previous))
    return runs


def mean_between(rows: list[dict], key: str, start: float, end: float) -> float | None:
    values = [row[key] for row in rows if start <= row["time_s"] < end]
    return float(np.mean(values)) if values else None


def diagnostic_truth(model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, float]:
    """只供验收记录，返回值绝不送入控制器。"""
    base = model.body("base_link").id
    jacobian = np.zeros((3, model.nv))
    mujoco.mj_jacSubtreeCom(model, data, jacobian, base)
    com_velocity = jacobian @ data.qvel
    floor = model.geom("floor").id
    ground_normal = 0.0
    contact_force = np.zeros(6)
    for index, contact in enumerate(data.contact):
        if floor in (contact.geom1, contact.geom2):
            mujoco.mj_contactForce(model, data, index, contact_force)
            ground_normal += max(float(contact_force[0]), 0.0)
    result = {
        "com_z_m": float(data.subtree_com[base, 2]),
        "com_vz_mps": float(com_velocity[2]),
        "ground_normal_n": ground_normal,
        "joint_limit_constraints": int(np.count_nonzero(
            data.efc_type == mujoco.mjtConstraint.mjCNSTR_LIMIT_JOINT)),
    }
    for name in ("ljoint1", "llink1child1joint", "ljoint4",
                 "rjoint1", "rlink1child1joint", "rjoint4"):
        joint = model.joint(name)
        result[f"{name}_qpos_rad"] = float(data.qpos[joint.qposadr[0]])
        result[f"{name}_qvel_radps"] = float(data.qvel[joint.dofadr[0]])
    return result


def jump_measurements(samples: list[dict], radius_m: float) -> list[dict]:
    """分别记录轮底净空和整车重心弹道，收腿造成的净空不冒充重心跳高。"""
    runs = true_runs([row["time_s"] for row in samples],
                     [row["wheel_contacts"] == 0 and row["other_floor_contacts"] == 0
                      for row in samples])
    measurements = []
    for span, start, end in runs:
        airborne = [row for row in samples if start <= row["time_s"] <= end]
        takeoff = airborne[0]
        measurements.append(dict(
            airborne_s=span, takeoff_s=start, last_airborne_s=end,
            clearance_m=max(min(row["left_wheel_z_m"], row["right_wheel_z_m"])
                            for row in airborne) - radius_m,
            com_takeoff_z_m=takeoff["com_z_m"],
            com_takeoff_vz_mps=takeoff["com_vz_mps"],
            com_rise_after_takeoff_m=max(row["com_z_m"] for row in airborne)
                                      - takeoff["com_z_m"],
            ballistic_rise_from_takeoff_v_m=max(takeoff["com_vz_mps"], 0.0) ** 2 / (2 * 9.81),
        ))
    return measurements


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path,
                        default=ROOT / "local/cache/wbr-own-interactive")
    parser.add_argument("--two-jumps", action="store_true")
    parser.add_argument("--jump-count", type=int, choices=(1, 2, 3), default=1)
    parser.add_argument("--moving-jump", action="store_true")
    parser.add_argument("--joint-peak-nm", type=float, choices=(20.0, 40.0, 45.0), default=20.0)
    parser.add_argument("--joint-continuous-nm", type=float, choices=(10.0, 20.0), default=20.0)
    parser.add_argument("--gas-spring-n", type=float, choices=(0.0, 450.0), default=0.0)
    parser.add_argument("--min-clearance-mm", type=float, default=30.0)
    parser.add_argument("--jump-profile", type=int, choices=(0, 1), default=0)
    parser.add_argument("--correct-link-collisions", action="store_true")
    args = parser.parse_args()
    if not np.isfinite(args.min_clearance_mm) or args.min_clearance_mm <= 0:
        parser.error("最低轮底净空必须是正数")
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=True)
    revision = subprocess.check_output(
        ["git", "-C", str(SOURCE), "rev-parse", "HEAD"], text=True).strip()
    if revision != REVISION:
        parser.error("WBR 版本不符")
    if subprocess.run(["git", "-C", str(SOURCE), "diff", "--quiet", "HEAD", "--", "mjcf"]).returncode:
        parser.error("WBR 模型存在修改")

    profile = json.loads((Path(__file__).with_name("OwnWbrProfile.json")).read_text(encoding="utf-8"))
    config = (ctypes.c_float * 20)(*np.asarray(profile["negative_k"]).flatten(),
        profile["theta0"], profile["tp0"], profile["wheel0"], profile["support_n"],
        profile["length_m"], profile["radius_m"], profile.get("length_kp", 1000),
        profile.get("length_kd", 60))
    lib, build_command = load_core(args.output)
    pointer = ctypes.POINTER(ctypes.c_float)
    lib.OwnWbrCommandStep.argtypes = [ctypes.c_void_p, pointer, pointer, pointer]
    lib.OwnWbrCommandStep.restype = ctypes.c_int
    lib.OwnWbrSetJointPeak.argtypes = [ctypes.c_void_p, ctypes.c_float]
    lib.OwnWbrSetJointPeak.restype = ctypes.c_int
    lib.OwnWbrSetJointLimits.argtypes = [ctypes.c_void_p, ctypes.c_float, ctypes.c_float]
    lib.OwnWbrSetJointLimits.restype = ctypes.c_int
    lib.OwnWbrSetGasSpring.argtypes = [ctypes.c_void_p, ctypes.c_float]
    lib.OwnWbrSetGasSpring.restype = ctypes.c_int
    lib.OwnWbrSetJumpProfile.argtypes = [ctypes.c_void_p, ctypes.c_int]
    lib.OwnWbrSetJumpProfile.restype = ctypes.c_int
    handle = lib.OwnWbrCreate(config)
    if not handle:
        raise RuntimeError("控制器初始化失败")
    if not lib.OwnWbrSetJointLimits(handle, args.joint_continuous_nm, args.joint_peak_nm):
        raise RuntimeError("持续或峰值力矩配置无效")
    if not lib.OwnWbrSetGasSpring(handle, args.gas_spring_n):
        raise RuntimeError("气弹簧补偿配置无效")
    core_jump_profile = args.jump_profile + (2 if args.correct_link_collisions else 0)
    if not lib.OwnWbrSetJumpProfile(handle, core_jump_profile):
        raise RuntimeError("跳跃动作配置无效")

    model = load_model(SOURCE / "mjcf/scene.xml", gas_force_n=args.gas_spring_n,
                       correct_link_collisions=args.correct_link_collisions)
    for actuator_name, joint_name in (("ljoint1_actuator", "ljoint1"),
                                      ("ljoint4_actuator", "ljoint4"),
                                      ("rjoint1_actuator", "rjoint1"),
                                      ("rjoint4_actuator", "rjoint4")):
        actuator_id = model.actuator(actuator_name).id
        joint_id = model.joint(joint_name).id
        model.actuator_ctrlrange[actuator_id] = (-args.joint_peak_nm, args.joint_peak_nm)
        model.actuator_forcerange[actuator_id] = (-args.joint_peak_nm, args.joint_peak_nm)
        model.jnt_actfrcrange[joint_id] = (-args.joint_peak_nm, args.joint_peak_nm)
    model.opt.timestep = .001
    data = mujoco.MjData(model)
    data.qpos[:] = profile["qpos"]
    mujoco.mj_forward(model, data)
    names = ["imu_quat", "imu_gyro", "imu_accel"]
    names += [f"{motor}_{kind}" for motor in MOTORS for kind in ("pos", "vel", "tor")]
    indices = np.concatenate([np.arange(model.sensor(name).adr[0],
                                         model.sensor(name).adr[0] + model.sensor(name).dim[0])
                              for name in names])
    sensor = (ctypes.c_float * 28)()
    command = (ctypes.c_float * 3)()
    output = (ctypes.c_float * 40)()
    requested_jump_count = max(args.jump_count, 2 if args.two_jumps else 1)
    if args.moving_jump:
        requested_jump_count = 1
    duration = 23.0 if args.moving_jump else 23.0 + 4.0 * (requested_jump_count - 1)
    rows: list[dict] = []
    poses: list[np.ndarray] = []
    pose_times: list[float] = []
    error = None

    try:
        for tick in range(round(duration / .001) + 1):
            requested = command_at(float(data.time), requested_jump_count, args.moving_jump)
            if tick % 3 == 0:
                sensor[:] = data.sensordata[indices]
                command[:] = requested
                status = lib.OwnWbrCommandStep(handle, sensor, command, output)
                if status:
                    error = f"core_error_{status}"
                    break
                data.ctrl[:] = output[:6]

            row = inspect_state(model, data, output, np.r_[0.0, requested[0], 0.0, 0.0,
                                                            0.0, 0.0, requested[1]])
            del row["fsm"]
            row.update(command_yaw_rate_radps=requested[1], jump_button=requested[2],
                       yaw_ref_rad=float(output[27]), ramped_yaw_rate_radps=float(output[28]),
                       ramped_v_mps=float(output[26]),
                       jump_phase=int(output[29]), jump_count=int(output[30]),
                       joint_limit_nm=float(output[31]),
                       left_theta_rad=float(output[14]), right_theta_rad=float(output[20]),
                       left_f_n=float(output[17]), right_f_n=float(output[23]),
                       left_knee_rad=float(data.qpos[8]), right_knee_rad=float(data.qpos[15]),
                       left_wheel_z_m=float(data.body("lwlink").xpos[2]),
                       right_wheel_z_m=float(data.body("rwlink").xpos[2]),
                       wheel_z_m=.5 * (float(data.body("lwlink").xpos[2])
                                       + float(data.body("rwlink").xpos[2])))
            row.update(diagnostic_truth(model, data))
            rows.append(row)
            if tick % 10 == 0:
                poses.append(data.qpos.copy())
                pose_times.append(float(data.time))
            if tick == round(duration / .001):
                break
            mujoco.mj_step(model, data)
            mujoco.mj_forward(model, data)
            if not np.isfinite(data.qpos).all() or any(warning.number for warning in data.warning):
                error = "mujoco_warning_or_nonfinite"
                break
    finally:
        lib.OwnWbrDestroy(handle)

    if not rows:
        raise RuntimeError(error or "没有仿真记录")
    times = [row["time_s"] for row in rows]
    after_jump = [row["jump_count"] > 0 and row["wheel_contacts"] == 0 and
                  row["other_floor_contacts"] == 0 for row in rows]
    air_runs = true_runs(times, after_jump)
    air_time, air_start, air_end = longest_true_run(times, after_jump)
    final_rows = [row for row in rows if row["time_s"] >= rows[-1]["time_s"] - 2.0]
    max_limits = {name: max(abs(row[name]) for row in rows) for name in (
        "ljoint1_requested_nm", "ljoint4_requested_nm", "lwheel_requested_nm",
        "rjoint1_requested_nm", "rjoint4_requested_nm", "rwheel_requested_nm")}
    expected_jumps = requested_jump_count
    failures = [] if error is None else [error]
    wheel_clearances = [max(min(row["left_wheel_z_m"], row["right_wheel_z_m"])
                            for row in rows if start <= row["time_s"] <= end)
                        - profile["radius_m"]
                        for _duration_s, start, end in air_runs]
    # 同一次触发的反弹最多算一次，不能替后续没有起跳的触发凑数。
    per_jump = []
    for jump_id in range(1, expected_jumps + 1):
        samples = [row for row in rows if row["jump_count"] == jump_id]
        measurements = jump_measurements(samples, profile["radius_m"])
        passed = any(m["airborne_s"] >= .120 and
                     m["clearance_m"] >= args.min_clearance_mm / 1000.0 and
                     m["com_takeoff_vz_mps"] > 0 and m["com_rise_after_takeoff_m"] > 0
                     for m in measurements)
        per_jump.append(dict(jump_id=jump_id, passed=passed, airborne_runs=measurements))
    valid_jump_count = sum(jump["passed"] for jump in per_jump)
    if valid_jump_count < expected_jumps:
        failures.append("jump_below_required_airtime_clearance_or_com_rise")
    if rows[-1]["time_s"] < duration - .01:
        failures.append("simulation_incomplete")
    wheel_clearance = max(wheel_clearances, default=0.0)
    if max(row["jump_count"] for row in rows) != expected_jumps:
        failures.append("jump_edge_or_cooldown_count")
    if any(row["other_floor_contacts"] for row in rows):
        failures.append("body_or_leg_touched_floor")
    if max(abs(row["pitch_deg"]) for row in rows) > 15.0 or max(abs(row["roll_deg"]) for row in rows) > 15.0:
        failures.append("tilt_over_15deg")
    if max(abs(row["pitch_deg"]) for row in final_rows) > 3.0:
        failures.append("post_jump_pitch_not_recovered")
    final_speed_rms = float(np.sqrt(np.mean([row["true_v_mps"] ** 2 for row in final_rows])))
    if final_speed_rms > .10:
        failures.append("post_jump_speed_not_recovered")
    yaw_before_left = mean_between(rows, "yaw_deg", 10.8, 11.0)
    yaw_after_left = mean_between(rows, "yaw_deg", 12.8, 13.0)
    yaw_before_right = mean_between(rows, "yaw_deg", 13.8, 14.0)
    yaw_after_right = mean_between(rows, "yaw_deg", 15.8, 16.0)
    turns_complete = all(value is not None for value in
        (yaw_before_left, yaw_after_left, yaw_before_right, yaw_after_right))
    if not turns_complete or yaw_after_left <= yaw_before_left + 10.0 or yaw_after_right >= yaw_before_right - 10.0:
        failures.append("turn_direction_or_response")
    if rows[-1]["jump_phase"] != 0 or rows[-1]["wheel_contacts"] == 0:
        failures.append("jump_not_ready_or_not_landed_at_end")
    if max(row["closure_m"] for row in rows) > .001:
        failures.append("closure_over_1mm")
    if args.jump_profile == 1 and any(row["joint_limit_constraints"] for row in rows):
        failures.append("enhanced_jump_hit_joint_limit")
    hip_keys = [name for name in max_limits if "wheel" not in name]
    if any(abs(row[name]) > row["joint_limit_nm"] + 1e-4 for row in rows for name in hip_keys):
        failures.append("hip_torque_exceeded_active_limit")
    saturation_counts = {
        name: sum(abs(row[name]) >= (5.2 if "wheel" in name else args.joint_peak_nm) - 1.0e-6
                  for row in rows)
        for name in max_limits
    }
    push_samples = [row for row in rows if row["jump_phase"] == 2]
    report = {
        "passed": not failures,
        "failures": failures,
        "simulated_s": rows[-1]["time_s"],
        "build_command": build_command,
        "source_revision": revision,
        "joint_peak_nm": args.joint_peak_nm,
        "joint_continuous_nm": args.joint_continuous_nm,
        "gas_spring_n_each": args.gas_spring_n,
        "jump_profile": args.jump_profile,
        "core_jump_profile": core_jump_profile,
        "correct_link_collisions": args.correct_link_collisions,
        "required_clearance_mm": args.min_clearance_mm,
        "required_airborne_s": .120,
        "control_inputs": "IMU and motor sensors only; contact truth used only below for evaluation",
        "expected_jump_count": expected_jumps,
        "observed_jump_count": max(row["jump_count"] for row in rows),
        "valid_jump_count": valid_jump_count,
        "per_jump": per_jump,
        "longest_both_wheels_airborne_s": air_time,
        "airborne_interval_s": [air_start, air_end],
        "all_airborne_intervals_s": [[start, end, duration_s]
                                      for duration_s, start, end in air_runs],
        "wheel_z_range_m": [min(row["wheel_z_m"] for row in rows),
                            max(row["wheel_z_m"] for row in rows)],
        "wheel_clearance_peak_m": wheel_clearance,
        "airborne_clearance_peaks_m": wheel_clearances,
        "peak_pitch_deg": max(abs(row["pitch_deg"]) for row in rows),
        "peak_roll_deg": max(abs(row["roll_deg"]) for row in rows),
        "peak_yaw_deg": max(abs(row["yaw_deg"]) for row in rows),
        "hip_speed_peak_rpm": max(abs(row[f"{name}_qvel_radps"])
            for row in rows for name in ("ljoint1", "ljoint4", "rjoint1", "rjoint4")) * 30 / np.pi,
        "left_turn_delta_deg": yaw_after_left - yaw_before_left if turns_complete else None,
        "right_turn_delta_deg": yaw_after_right - yaw_before_right if turns_complete else None,
        "forward_mean_mps": mean_between(rows, "true_v_mps", 6.0, 7.0),
        "reverse_mean_mps": mean_between(rows, "true_v_mps", 9.0, 10.0),
        "post_jump_speed_rms_mps": final_speed_rms,
        "post_jump_pitch_peak_deg": max(abs(row["pitch_deg"]) for row in final_rows),
        "stand_0_to_5_speed_rms_mps": float(np.sqrt(np.mean([
            row["true_v_mps"] ** 2 for row in rows if row["time_s"] < 5.0]))),
        "max_closure_m": max(row["closure_m"] for row in rows),
        "observer_rmse_mps": float(np.sqrt(np.mean([
            (row["odom_v_mps"] - row["true_v_mps"]) ** 2 for row in rows]))),
        "nonwheel_contact_samples": sum(row["other_floor_contacts"] > 0 for row in rows),
        "joint_limit_samples": sum(row["joint_limit_constraints"] > 0 for row in rows),
        "joint_limit_samples_by_jump_phase": {
            str(phase): sum(row["joint_limit_constraints"] > 0 and row["jump_phase"] == phase
                            for row in rows) for phase in range(6)},
        "knee_angle_ranges_rad": {
            "left": [min(row["left_knee_rad"] for row in rows),
                     max(row["left_knee_rad"] for row in rows)],
            "right": [min(row["right_knee_rad"] for row in rows),
                      max(row["right_knee_rad"] for row in rows)],
        },
        "requested_torque_abs_max_nm": max_limits,
        "requested_torque_limit_sample_counts": saturation_counts,
        "push_start_s": push_samples[0]["time_s"] if push_samples else None,
        "ramped_speed_at_push_start_mps": push_samples[0]["ramped_v_mps"] if push_samples else None,
    }
    with (args.output / "trace.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    np.savez_compressed(args.output / "trajectory.npz", time=pose_times, qpos=poses)
    (args.output / "summary.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
