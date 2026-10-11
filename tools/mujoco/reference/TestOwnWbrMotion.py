"""用真实 WBR MuJoCo 模型验收行进跳、滑行/反向制动和腿高调节。"""
from __future__ import annotations

import argparse
import csv
import ctypes
import hashlib
import json
import math
from pathlib import Path

import mujoco
import numpy as np

from RunOwnWbr import load_core
from RunWbr import MOTORS, ROOT, SOURCE, inspect_state
from WbrGasSpring import load_model
from WbrGuideRollers import MODES, guide_roller_info


def sha256(path: Path) -> str:
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(block)
    return value.hexdigest().upper()


def mean(rows: list[dict], key: str, start: float, end: float) -> float:
    values = [row[key] for row in rows if start <= row["time_s"] < end]
    return float(np.mean(values)) if values else math.nan


def command_at(scenario: str, time_s: float, speed_mps: float) -> tuple[float, float]:
    if scenario == "moving_jump":
        return (speed_mps if time_s >= 2.0 else 0.0,
                1.0 if 10.0 <= time_s < 10.05 else 0.0)
    if scenario == "coast_reverse":
        if 2.0 <= time_s < 6.0:
            return abs(speed_mps), 0.0
        if 8.0 <= time_s < 13.0:
            return -abs(speed_mps), 0.0
    return 0.0, 0.0


def contact_truth(model: mujoco.MjModel, data: mujoco.MjData) -> tuple[float, float, int]:
    base = model.body("base_link").id
    jacobian = np.zeros((3, model.nv))
    mujoco.mj_jacSubtreeCom(model, data, jacobian, base)
    com_velocity = jacobian @ data.qvel
    limits = int(np.count_nonzero(data.efc_type == mujoco.mjtConstraint.mjCNSTR_LIMIT_JOINT))
    return float(data.subtree_com[base, 2]), float(com_velocity[2]), limits


def longest_airborne(rows: list[dict], radius_m: float) -> dict:
    runs: list[list[dict]] = []
    active: list[dict] = []
    for row in rows:
        airborne = row["wheel_contacts"] == 0 and row["other_floor_contacts"] == 0
        if airborne:
            active.append(row)
        elif active:
            runs.append(active)
            active = []
    if active:
        runs.append(active)
    if not runs:
        return {"airborne_s": 0.0, "clearance_m": 0.0}
    run = max(runs, key=len)
    takeoff = run[0]
    return {
        "airborne_s": run[-1]["time_s"] - takeoff["time_s"] + 0.001,
        "takeoff_s": takeoff["time_s"],
        "landing_s": run[-1]["time_s"],
        "takeoff_speed_mps": takeoff["true_v_mps"],
        "takeoff_odom_mps": takeoff["odom_v_mps"],
        "takeoff_vz_mps": takeoff["com_vz_mps"],
        "com_rise_m": max(row["com_z_m"] for row in run) - takeoff["com_z_m"],
        "clearance_m": max(min(row["left_wheel_z_m"], row["right_wheel_z_m"])
                           for row in run) - radius_m,
    }


def common_checks(rows: list[dict]) -> dict[str, bool]:
    return {
        "no_nonwheel_floor_contact": not any(row["other_floor_contacts"] for row in rows),
        "no_joint_limit_constraint": not any(row["joint_limit_constraints"] for row in rows),
        "closure_below_1mm": max(row["closure_m"] for row in rows) <= 0.001,
        "pitch_below_15deg": max(abs(row["pitch_deg"]) for row in rows) <= 15.0,
        "roll_below_10deg": max(abs(row["roll_deg"]) for row in rows) <= 10.0,
    }


def evaluate(scenario: str, rows: list[dict], speed_mps: float,
             radius_m: float, core_error: str | None, jump_profile: str = "small",
             leg_length_m: float = .20, input_pattern: str = "static",
             home_start: bool = False) -> dict:
    safety_rows = ([row for row in rows if row.get("home_ready", False)]
                   if home_start else rows)
    checks = common_checks(safety_rows)
    measurements: dict[str, object] = {
        "peak_pitch_deg": max(abs(row["pitch_deg"]) for row in safety_rows),
        "peak_roll_deg": max(abs(row["roll_deg"]) for row in safety_rows),
        "max_closure_m": max(row["closure_m"] for row in safety_rows),
        "nonwheel_contact_samples": sum(row["other_floor_contacts"] > 0 for row in safety_rows),
        "joint_limit_samples": sum(row["joint_limit_constraints"] > 0 for row in safety_rows),
        "observer_rmse_mps": float(np.sqrt(np.mean([
            (row["odom_v_mps"] - row["true_v_mps"]) ** 2 for row in safety_rows]))),
    }
    if core_error:
        checks["controller_completed"] = False

    if scenario == "moving_jump":
        flight = longest_airborne([row for row in rows if row["jump_count"] > 0], radius_m)
        press = next((row for row in rows if row["jump_button"] > 0.5), None)
        push = next((row for row in rows if row["jump_phase"] == 2), None)
        landing_end = flight.get("landing_s", 0.0) + 0.5
        after = [row for row in rows if landing_end <= row["time_s"] < landing_end + 0.5]
        actual_at_press = press["true_v_mps"] if press else math.nan
        actual_at_push = push["true_v_mps"] if push else math.nan
        post_landing = float(np.mean([row["true_v_mps"] for row in after])) if after else math.nan
        direction = 1.0 if speed_mps >= 0 else -1.0
        speed_limit = .25 if jump_profile == "enhanced" else (3.0 if direction > 0 else 1.0)
        prepare_deadline = max(.8, (abs(speed_mps) - speed_limit) / .5 + 2.0)
        preparing = [row for row in rows if press is not None and push is not None and
                     press["time_s"] <= row["time_s"] <= push["time_s"]]
        min_prepare_speed = min((row["true_v_mps"] * direction for row in preparing), default=0.0)
        moving = abs(speed_mps) > .05
        minimum_clearance = .24 if jump_profile == "enhanced" else .09
        maximum_clearance = .27 if jump_profile == "enhanced" else .11
        checks.update({
            "jump_started_within_speed_prepare_deadline": push is not None and press is not None and
                push["time_s"] - press["time_s"] < prepare_deadline,
            "moving_prepare_did_not_stop": not moving or min_prepare_speed > .05,
            "moving_takeoff_keeps_direction": not moving or
                flight.get("takeoff_speed_mps", 0.0) * direction > .08,
            "actual_speed_near_requested_before_jump": math.isfinite(actual_at_press) and
                actual_at_press * direction >= 0.8 * abs(speed_mps),
            "both_wheels_airborne_120ms": flight["airborne_s"] >= 0.12,
            "wheel_clearance_matches_selected_jump": minimum_clearance <= flight["clearance_m"] <= maximum_clearance,
            "positive_vertical_takeoff": flight.get("takeoff_vz_mps", 0.0) > 0.0 and
                flight.get("com_rise_m", 0.0) > 0.0,
            "travel_direction_kept_after_landing": not moving or math.isfinite(post_landing) and
                post_landing * direction > 0.05,
            "jump_finished": rows[-1]["jump_phase"] == 0 and rows[-1]["jump_count"] == 1,
        })
        measurements.update({
            "requested_speed_mps": speed_mps,
            "actual_speed_at_press_mps": actual_at_press,
            "actual_speed_at_push_mps": actual_at_push,
            "post_landing_mean_speed_mps": post_landing,
            "press_to_push_s": push["time_s"] - press["time_s"] if press and push else None,
            "prepare_deadline_s": prepare_deadline,
            "minimum_prepare_speed_in_requested_direction_mps": min_prepare_speed,
            "final_speed_mps": mean(rows, "true_v_mps", rows[-1]["time_s"] - 1, rows[-1]["time_s"]),
            "flight": flight,
        })
    elif scenario == "coast_reverse":
        requested_speed = abs(speed_mps)
        release_speed = mean(rows, "true_v_mps", 5.8, 6.0)
        coast_speed = mean(rows, "true_v_mps", 7.0, 7.5)
        coast_distance = mean(rows, "x_m", 7.9, 8.0) - mean(rows, "x_m", 6.0, 6.1)
        reverse_rows = [row for row in rows if row["time_s"] >= 8.0]
        brake_speed = mean(rows, "true_v_mps", 7.95, 8.0)
        # 高速场景按 2 m/s² 参考制动时间加响应余量评分，低速保留原 1.5 秒门槛。
        brake_deadline = max(1.5, abs(brake_speed) / 2.0 + 0.35)
        zero = next((row for row in reverse_rows if row["true_v_mps"] <= 0.0), None)
        zero_time = zero["time_s"] - 8.0 if zero else math.inf
        reverse_mean = mean(rows, "true_v_mps", 11.5, 12.5)
        checks.update({
            "release_keeps_forward_motion": release_speed > 0.4 * requested_speed and
                coast_speed > 0.25 * requested_speed,
            "release_coasts_forward": coast_distance > 0.20 * requested_speed,
            "reverse_brakes_within_speed_based_deadline": zero_time <= brake_deadline,
            "reverse_direction_reached": reverse_mean < -0.5 * requested_speed,
        })
        measurements.update({
            "release_speed_mps": release_speed,
            "speed_at_reverse_request_mps": brake_speed,
            "reverse_deadline_s": brake_deadline,
            "coast_speed_1_to_1_5s_after_release_mps": coast_speed,
            "coast_distance_1_9s_m": coast_distance,
            "reverse_zero_crossing_s_after_request": zero_time,
            "reverse_mean_11_5_to_12_5_mps": reverse_mean,
            "measured_reverse_deceleration_mps2": brake_speed / zero_time if zero_time > 0 else None,
        })
    elif scenario == "jump_boundary":
        flight = longest_airborne([row for row in rows if row["jump_count"] > 0], radius_m)
        press = next((row for row in rows if row["jump_button"] > 0.5), None)
        push = next((row for row in rows if row["jump_phase"] == 2), None)
        profiles = sorted({row["active_jump_profile"] for row in rows
                           if row["jump_count"] > 0})
        expected_profile = 2 if jump_profile == "small" else 3
        direction = 1.0 if speed_mps >= 0 else -1.0
        end = rows[-1]["time_s"]
        final_speed = mean(rows, "true_v_mps", end - 1.5, end - .5)
        final_leg_target = mean(rows, "leg_target_m", end - 1.0, end)
        checks.update({
            "jump_started": press is not None and push is not None,
            "both_wheels_airborne_120ms": flight["airborne_s"] >= .12,
            "wheel_clearance_matches_selected_jump":
                (.24 if jump_profile == "enhanced" else .09) <= flight["clearance_m"] <=
                (.27 if jump_profile == "enhanced" else .11),
            "effective_profile_matches_input": profiles == [expected_profile],
            "jump_finished": rows[-1]["jump_phase"] == 0 and rows[-1]["jump_count"] == 1,
            "selected_leg_height_restored": abs(final_leg_target - leg_length_m) <= .003,
            "home_rise_completed": (not home_start or
                (any(row["rise_phase"] > 0 for row in rows) and
                 rows[-1]["rise_phase"] == 0 and rows[-1]["home_ready"])),
        })
        if input_pattern in ("settled", "simultaneous"):
            checks["moving_jump_keeps_direction"] = (push is not None and
                flight.get("takeoff_speed_mps", 0.0) * direction > .08 and
                final_speed * direction > .08)
        elif input_pattern == "delayed":
            checks["late_drive_deferred_until_enhanced_jump_finishes"] = (push is not None and
                abs(push["true_v_mps"]) < .08 and final_speed * direction > .08)
        else:
            checks["static_jump_starts_near_stationary"] = (push is not None and
                abs(push["true_v_mps"]) < .08)
        measurements.update({
            "requested_leg_length_m": leg_length_m,
            "input_pattern": input_pattern,
            "active_jump_profiles": profiles,
            "actual_speed_at_press_mps": press["true_v_mps"] if press else None,
            "actual_speed_at_push_mps": push["true_v_mps"] if push else None,
            "final_speed_mps": final_speed,
            "final_leg_target_m": final_leg_target,
            "flight": flight,
        })
    else:
        windows = {
            "low": (4.0, 4.8, 0.18),
            "high": (7.0, 7.8, 0.24),
            "middle": (10.0, 10.8, 0.20),
        }
        height_results = {}
        for name, (start, end, target) in windows.items():
            target_mean = mean(rows, "leg_target_m", start, end)
            actual_mean = 0.5 * (mean(rows, "left_leg_m", start, end) +
                                 mean(rows, "right_leg_m", start, end))
            height_results[name] = {"requested_m": target, "target_mean_m": target_mean,
                                    "actual_mean_m": actual_mean}
            checks[f"{name}_target_reached"] = abs(target_mean - target) <= 0.002
            checks[f"{name}_actual_followed"] = abs(actual_mean - target) <= 0.015
        measurements["height_windows"] = height_results

    return {
        "passed": all(checks.values()),
        "failures": [name for name, passed in checks.items() if not passed],
        "scenario": scenario,
        "checks": checks,
        "measurements": measurements,
        "control_inputs": "仅 IMU 和六个电机传感器进入控制器；位置、接触和净空真值只用于验收",
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", choices=("moving_jump", "coast_reverse", "leg_height",
                                                "jump_boundary"),
                        default="moving_jump")
    parser.add_argument("--speed", type=float, default=0.5,
                        help="moving_jump 目标速度；coast_reverse 使用其绝对值")
    parser.add_argument("--jump-profile", choices=("small", "enhanced"),
                        default="small")
    parser.add_argument("--leg-length", type=float, default=0.20,
                        help="jump_boundary 的用户腿高目标")
    parser.add_argument("--input-pattern", choices=("static", "settled", "simultaneous", "delayed"),
                        default="static", help="jump_boundary 的纵向键和跳跃组合")
    parser.add_argument("--home-start", action="store_true",
                        help="从模型 home 姿态执行控制器起身后再测试")
    parser.add_argument("--noslip-iterations", type=int, default=0,
                        help="防滑接触求解次数；0复现旧平地，10与新版综合场地相同")
    parser.add_argument("--guide-rollers", choices=MODES, default="legacy",
                        help="底部机械对照；passive 使用当前前后护角小导轮")
    parser.add_argument("--output", type=Path,
                        default=ROOT / "local/cache/wbr-motion-20261003")
    args = parser.parse_args()
    if not math.isfinite(args.speed) or abs(args.speed) > 3.0:
        parser.error("速度必须是 -3.0 到 3.0 m/s 的有限数")
    if not math.isfinite(args.leg_length) or not .18 <= args.leg_length <= .32:
        parser.error("腿长必须是 0.18 到 0.32 m 的有限数")
    if not 0 <= args.noslip_iterations <= 100:
        parser.error("防滑接触求解次数必须为0–100")
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=True)

    profile = json.loads((Path(__file__).with_name("OwnWbrProfile.json")).read_text(encoding="utf-8"))
    config = (ctypes.c_float * 20)(*np.asarray(profile["negative_k"]).flatten(),
        profile["theta0"], profile["tp0"], profile["wheel0"], profile["support_n"],
        profile["length_m"], profile["radius_m"], profile.get("length_kp", 1000),
        profile.get("length_kd", 60))
    lib, build_command = load_core(args.output)
    pointer = ctypes.POINTER(ctypes.c_float)
    for name, argtypes in (
        ("OwnWbrCommandStep", [ctypes.c_void_p, pointer, pointer, pointer]),
        ("OwnWbrSetJointLimits", [ctypes.c_void_p, ctypes.c_float, ctypes.c_float]),
        ("OwnWbrSetGasSpring", [ctypes.c_void_p, ctypes.c_float]),
        ("OwnWbrSetJumpProfile", [ctypes.c_void_p, ctypes.c_int]),
        ("OwnWbrGetJumpProfile", [ctypes.c_void_p]),
        ("OwnWbrSetDriveMode", [ctypes.c_void_p, ctypes.c_int]),
        ("OwnWbrEnableRise", [ctypes.c_void_p, ctypes.c_int]),
        ("OwnWbrSetLegLength", [ctypes.c_void_p, ctypes.c_float])):
        function = getattr(lib, name)
        function.argtypes = argtypes
        function.restype = ctypes.c_int

    handle = lib.OwnWbrCreate(config)
    if not handle:
        raise RuntimeError("控制器初始化失败")
    if not lib.OwnWbrSetJointLimits(handle, 10.0, 45.0):
        raise RuntimeError("关节力矩配置失败")
    if not lib.OwnWbrSetGasSpring(handle, 450.0):
        raise RuntimeError("气弹簧配置失败")
    core_jump_profile = 2 if args.jump_profile == "small" else 3
    if not lib.OwnWbrSetJumpProfile(handle, core_jump_profile):
        raise RuntimeError("跳跃动作配置失败")
    drive_mode = 1 if args.scenario in ("moving_jump", "coast_reverse", "jump_boundary") else 0
    if not lib.OwnWbrSetDriveMode(handle, drive_mode):
        raise RuntimeError("行进模式配置失败")
    if args.scenario == "jump_boundary" and not lib.OwnWbrSetLegLength(handle, args.leg_length):
        raise RuntimeError("边界腿高配置失败")
    if args.home_start and not lib.OwnWbrEnableRise(handle, 1):
        raise RuntimeError("起身配置失败")

    model = load_model(SOURCE / "mjcf/scene.xml", gas_force_n=450.0,
                       correct_link_collisions=True, guide_rollers=args.guide_rollers)
    model.opt.noslip_iterations = args.noslip_iterations
    for actuator_name, joint_name in (("ljoint1_actuator", "ljoint1"),
                                      ("ljoint4_actuator", "ljoint4"),
                                      ("rjoint1_actuator", "rjoint1"),
                                      ("rjoint4_actuator", "rjoint4")):
        actuator = model.actuator(actuator_name).id
        joint = model.joint(joint_name).id
        model.actuator_ctrlrange[actuator] = (-45.0, 45.0)
        model.actuator_forcerange[actuator] = (-45.0, 45.0)
        model.jnt_actfrcrange[joint] = (-45.0, 45.0)
    model.opt.timestep = 0.001
    data = mujoco.MjData(model)
    if args.home_start:
        mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    else:
        data.qpos[:len(profile["qpos"])] = profile["qpos"]
    mujoco.mj_forward(model, data)
    names = ["imu_quat", "imu_gyro", "imu_accel"]
    names += [f"{motor}_{kind}" for motor in MOTORS for kind in ("pos", "vel", "tor")]
    indices = np.concatenate([np.arange(model.sensor(name).adr[0],
                                         model.sensor(name).adr[0] + model.sensor(name).dim[0])
                              for name in names])
    sensor = (ctypes.c_float * 28)()
    command = (ctypes.c_float * 3)()
    output = (ctypes.c_float * 40)()
    duration = {"moving_jump": 24.0, "coast_reverse": 15.0, "leg_height": 11.0,
                "jump_boundary": 26.0}[args.scenario]
    rows: list[dict] = []
    core_error = None
    rise_seen = False
    home_ready = not args.home_start
    try:
        for tick in range(round(duration / 0.001) + 1):
            requested_v, jump = command_at(args.scenario, float(data.time), args.speed)
            if args.scenario == "jump_boundary":
                jump_at = 12.0
                requested_v = 0.0
                if args.input_pattern == "settled" and data.time >= 6.0:
                    requested_v = args.speed
                elif args.input_pattern == "simultaneous" and data.time >= jump_at:
                    requested_v = args.speed
                elif args.input_pattern == "delayed" and data.time >= jump_at + .10:
                    requested_v = args.speed
                jump = 1.0 if jump_at <= data.time < jump_at + .05 else 0.0
            if not home_ready:
                requested_v = 0.0
                jump = 0.0
            if tick % 3 == 0:
                if args.scenario == "leg_height":
                    target_length = 0.20
                    if 2.0 <= data.time < 5.0:
                        target_length = 0.18
                    elif 5.0 <= data.time < 8.0:
                        target_length = 0.24
                    if not lib.OwnWbrSetLegLength(handle, target_length):
                        core_error = "leg_length_api_failed"
                        break
                sensor[:] = data.sensordata[indices]
                command[:] = (requested_v, 0.0, jump)
                status = lib.OwnWbrCommandStep(handle, sensor, command, output)
                if status:
                    core_error = f"core_error_{status}"
                    break
                rise_phase = int(output[34])
                if rise_phase < 0:
                    core_error = f"rise_failed_{rise_phase}"
                    break
                if rise_phase > 0:
                    rise_seen = True
                elif rise_seen:
                    home_ready = True
                data.ctrl[:] = output[:6]
            row = inspect_state(model, data, output,
                np.r_[0.0, requested_v, 0.0, 0.0, 0.0, 0.0, 0.0])
            del row["fsm"]
            com_z, com_vz, constraints = contact_truth(model, data)
            row.update(jump_button=jump, jump_phase=int(output[29]),
                       jump_count=int(output[30]), ramped_v_mps=float(output[26]),
                       active_jump_profile=lib.OwnWbrGetJumpProfile(handle),
                       rise_phase=int(output[34]), home_ready=home_ready,
                       leg_target_m=float(output[33]), left_leg_m=float(output[12]),
                       right_leg_m=float(output[18]), com_z_m=com_z, com_vz_mps=com_vz,
                       joint_limit_constraints=constraints,
                       left_wheel_z_m=float(data.body("lwlink").xpos[2]),
                       right_wheel_z_m=float(data.body("rwlink").xpos[2]))
            row.update(left_leg_rate_mps=float(output[13]), right_leg_rate_mps=float(output[19]),
                       left_support_n=float(output[17]), right_support_n=float(output[23]),
                       joint_limit_names=";".join(model.joint(int(data.efc_id[i])).name for i in
                           np.flatnonzero(data.efc_type == mujoco.mjtConstraint.mjCNSTR_LIMIT_JOINT)))
            row["wheel_sensor_common_radps"] = 0.5 * (-float(sensor[17]) + float(sensor[26]))
            rows.append(row)
            if tick == round(duration / 0.001):
                break
            mujoco.mj_step(model, data)
            mujoco.mj_forward(model, data)
            if not np.isfinite(data.qpos).all() or any(item.number for item in data.warning):
                core_error = "mujoco_warning_or_nonfinite"
                break
    finally:
        lib.OwnWbrDestroy(handle)

    if not rows:
        raise RuntimeError(core_error or "没有仿真记录")
    with (args.output / "trace.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    report = evaluate(args.scenario, rows, args.speed, profile["radius_m"], core_error,
                      args.jump_profile, args.leg_length, args.input_pattern, args.home_start)
    report["active_jump_profiles"] = sorted({row["active_jump_profile"] for row in rows
                                              if row["jump_count"] > 0})
    report.update(simulated_s=rows[-1]["time_s"], joint_continuous_nm=10.0,
                  joint_peak_nm=45.0, wheel_limit_nm=5.2, gas_spring_n_each=450.0,
                  jump_profile=args.jump_profile, core_jump_profile=core_jump_profile,
                  leg_length_m=args.leg_length, input_pattern=args.input_pattern,
                  home_start=args.home_start,
                  correct_link_collisions=True, total_mass_kg=float(model.body_mass.sum()),
                  noslip_iterations=int(model.opt.noslip_iterations),
                  guide_rollers=guide_roller_info(model),
                  build_command=build_command,
                  controller_sha256=sha256(Path(__file__).with_name("OwnWbrBridge.c")))
    (args.output / "summary.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
