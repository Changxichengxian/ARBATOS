"""从真实 home 起身，验收带速越上 100 mm 直角台阶并驻车。"""
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
from TestOwnWbrMotion import contact_truth
from WbrGasSpring import load_model
from WbrTerrain import ground_geom_ids, ground_height


def sha256(path: Path) -> str:
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(block)
    return value.hexdigest().upper()


def terrain_contacts(model: mujoco.MjModel, data: mujoco.MjData,
                     grounds: set[int], target: int) -> tuple[list[tuple[str, str, str]], dict[str, float]]:
    wheel_bodies = {model.body("lwlink").id: "lwlink", model.body("rwlink").id: "rwlink"}
    nonwheel = []
    upward_normals: dict[str, float] = {}
    for contact in data.contact:
        if contact.geom1 in grounds:
            ground, other = contact.geom1, contact.geom2
            normal_z = float(contact.frame[2])
        elif contact.geom2 in grounds:
            ground, other = contact.geom2, contact.geom1
            normal_z = -float(contact.frame[2])
        else:
            continue
        body = int(model.geom_bodyid[other])
        body_name = model.body(body).name
        if body in wheel_bodies:
            if ground == target:
                name = wheel_bodies[body]
                upward_normals[name] = max(upward_normals.get(name, -1.0), normal_z)
        else:
            nonwheel.append((body_name, model.geom(other).name, model.geom(ground).name))
    return nonwheel, upward_normals


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trigger-x", type=float, default=6.30)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--duration", type=float, default=12.0)
    args = parser.parse_args()
    if not math.isfinite(args.trigger_x) or not 6.0 <= args.trigger_x <= 6.8:
        parser.error("触发位置必须是 6.0 到 6.8 m 的有限数")
    if not math.isfinite(args.duration) or args.duration < 8.0:
        parser.error("仿真时长至少 8 秒")
    if args.output is None:
        suffix = round(args.trigger_x * 100)
        args.output = ROOT / f"local/cache/wbr-step-jump-20261003/trigger-{suffix}-home"
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=True)

    profile_path = Path(__file__).with_name("OwnWbrProfile.json")
    profile = json.loads(profile_path.read_text(encoding="utf-8"))
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
        ("OwnWbrSetStepJump", [ctypes.c_void_p, ctypes.c_int]),
        ("OwnWbrSetDriveMode", [ctypes.c_void_p, ctypes.c_int]),
        ("OwnWbrEnableRise", [ctypes.c_void_p, ctypes.c_int])):
        function = getattr(lib, name)
        function.argtypes = argtypes
        function.restype = ctypes.c_int

    handle = lib.OwnWbrCreate(config)
    if not handle:
        raise RuntimeError("控制器初始化失败")
    assert lib.OwnWbrSetJointLimits(handle, 10.0, 45.0) == 1
    assert lib.OwnWbrSetGasSpring(handle, 450.0) == 1
    assert lib.OwnWbrSetJumpProfile(handle, 3) == 1
    assert lib.OwnWbrSetStepJump(handle, 1) == 1
    assert lib.OwnWbrSetDriveMode(handle, 1) == 1
    assert lib.OwnWbrEnableRise(handle, 1) == 1

    model = load_model(SOURCE / "mjcf/scene.xml", gas_force_n=450.0,
                       correct_link_collisions=True, terrain="course",
                       noslip_iterations=10)
    for actuator_name, joint_name in (("ljoint1_actuator", "ljoint1"),
                                      ("ljoint4_actuator", "ljoint4"),
                                      ("rjoint1_actuator", "rjoint1"),
                                      ("rjoint4_actuator", "rjoint4")):
        actuator = model.actuator(actuator_name).id
        joint = model.joint(joint_name).id
        model.actuator_ctrlrange[actuator] = (-45.0, 45.0)
        model.actuator_forcerange[actuator] = (-45.0, 45.0)
        model.jnt_actfrcrange[joint] = (-45.0, 45.0)
    model.opt.timestep = .001
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    data.qpos[0] = 6.0
    data.qpos[1] = -1.5
    mujoco.mj_forward(model, data)

    names = ["imu_quat", "imu_gyro", "imu_accel"]
    names += [f"{motor}_{kind}" for motor in MOTORS for kind in ("pos", "vel", "tor")]
    indices = np.concatenate([np.arange(model.sensor(name).adr[0],
        model.sensor(name).adr[0] + model.sensor(name).dim[0]) for name in names])
    hip_names = ("ljoint1", "ljoint4", "rjoint1", "rjoint4")
    hip_velocity_adr = {name: model.sensor(f"{name}_vel").adr[0] for name in hip_names}
    grounds = ground_geom_ids(model)
    target = model.geom("terrain_large100").id
    sensor = (ctypes.c_float * 28)()
    command = (ctypes.c_float * 3)()
    output = (ctypes.c_float * 40)()
    rows: list[dict] = []
    qpos_times = []
    qpos_samples = []
    trigger_time = None
    rise_seen = False
    home_ready = False
    home_ready_time = None
    active_api_lock_checked = False
    core_error = None
    warning_counts = [0] * len(data.warning)
    try:
        for tick in range(round(args.duration / model.opt.timestep) + 1):
            base_x = float(data.qpos[0])
            if trigger_time is None and home_ready and base_x >= args.trigger_x:
                trigger_time = float(data.time)
            elapsed = math.inf if trigger_time is None else float(data.time) - trigger_time
            drive = .25 if home_ready else 0.0
            jump = 1.0 if 0.0 <= elapsed < .05 else 0.0
            if tick % 3 == 0:
                sensor[:] = data.sensordata[indices]
                command[:] = (drive, 0.0, jump)
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
                elif rise_seen and not home_ready:
                    home_ready = True
                    home_ready_time = float(data.time)
                if int(output[29]) != 0 and not active_api_lock_checked:
                    if lib.OwnWbrSetStepJump(handle, 0) != 0 or lib.OwnWbrSetStepJump(handle, 1) != 1:
                        core_error = "step_jump_api_lock_failed"
                        break
                    active_api_lock_checked = True
                data.ctrl[:] = output[:6]

            row = inspect_state(model, data, output,
                np.r_[0.0, drive, 0.0, 0.0, 0.0, 0.0, 0.0])
            nonwheel, normals = terrain_contacts(model, data, grounds, target)
            _, _, limits = contact_truth(model, data)
            wheel_positions = [data.body("lwlink").xpos.copy(), data.body("rwlink").xpos.copy()]
            heights = [ground_height(model, float(pos[0]), float(pos[1])) for pos in wheel_positions]
            row.update(jump_button=jump, jump_phase=int(output[29]),
                jump_count=int(output[30]), v_ref_mps=float(output[26]),
                rise_phase=int(output[34]), home_ready=home_ready,
                joint_torque_limit_nm=float(output[31]),
                **{f"{name}_rpm": float(data.sensordata[hip_velocity_adr[name]]) *
                   60.0 / (2.0 * math.pi) for name in hip_names},
                left_wheel_x_m=float(wheel_positions[0][0]),
                right_wheel_x_m=float(wheel_positions[1][0]),
                left_wheel_z_m=float(wheel_positions[0][2]),
                right_wheel_z_m=float(wheel_positions[1][2]),
                left_ground_z_m=heights[0], right_ground_z_m=heights[1],
                left_target_normal_z=normals.get("lwlink", math.nan),
                right_target_normal_z=normals.get("rwlink", math.nan),
                nonwheel_ground_contacts=len(nonwheel),
                nonwheel_contact_detail=repr(nonwheel),
                joint_limit_constraints=limits)
            rows.append(row)
            if tick % 5 == 0:
                qpos_times.append(float(data.time))
                qpos_samples.append(data.qpos.copy())
            warning_counts = [max(old, int(item.number))
                              for old, item in zip(warning_counts, data.warning)]
            if tick == round(args.duration / model.opt.timestep):
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
    np.savez_compressed(args.output / "qpos-trajectory.npz",
                        time_s=np.asarray(qpos_times, dtype=np.float32),
                        qpos=np.asarray(qpos_samples, dtype=np.float32))

    final_window = [row for row in rows if rows[-1]["time_s"] - 2.0 <= row["time_s"]]
    wheel_radius = profile["radius_m"]
    stable_on_step = bool(final_window) and final_window[-1]["time_s"] - final_window[0]["time_s"] >= 1.99 and all(
        7.0 <= row["left_wheel_x_m"] <= 7.5 and
        7.0 <= row["right_wheel_x_m"] <= 7.5 and
        row["left_target_normal_z"] >= .80 and row["right_target_normal_z"] >= .80 and
        abs(row["left_wheel_z_m"] - row["left_ground_z_m"] - wheel_radius) < .015 and
        abs(row["right_wheel_z_m"] - row["right_ground_z_m"] - wheel_radius) < .015 and
        abs(row["true_v_mps"]) < .12 and
        abs(row["pitch_deg"]) < 10.0 and abs(row["roll_deg"]) < 10.0
        for row in final_window)
    safety_rows = [row for row in rows if row["home_ready"]]
    prehome_rows = [row for row in rows if not row["home_ready"]]
    contact_rows = [row for row in safety_rows if row["nonwheel_ground_contacts"]]
    limit_rows = [row for row in safety_rows if row["joint_limit_constraints"]]
    hip_metrics = {}
    for name in hip_names:
        saturated = [abs(row[f"{name}_requested_nm"]) >=
                     row["joint_torque_limit_nm"] - 1e-3 for row in safety_rows]
        longest = current = 0
        for hit in saturated:
            current = current + 1 if hit else 0
            longest = max(longest, current)
        hip_metrics[name] = {
            "peak_abs_rpm": max(abs(row[f"{name}_rpm"]) for row in safety_rows),
            "peak_abs_requested_nm": max(abs(row[f"{name}_requested_nm"]) for row in safety_rows),
            "torque_saturation_total_s": sum(saturated) * model.opt.timestep,
            "torque_saturation_longest_contiguous_s": longest * model.opt.timestep,
        }
    warnings = {str(index): number for index, number in enumerate(warning_counts) if number}
    source_path = Path(__file__).with_name("OwnWbrBridge.c")
    report = {
        "passed": core_error is None and home_ready and trigger_time is not None and
                  stable_on_step and not contact_rows and not limit_rows and not warnings and
                  active_api_lock_checked,
        "trigger_x_m": args.trigger_x,
        "trigger_time_s": trigger_time,
        "home_ready_time_s": home_ready_time,
        "initial_pose": {"x_m": 6.0, "y_m": -1.5, "kind": "真实home起身"},
        "command": {"speed_mps": .25, "w_held_through_end": True,
                    "jump_pulse_s": .05, "drive_mode": 1},
        "stable_both_wheels_on_100mm_step_last_2s": stable_on_step,
        "last_2s_peak_abs_speed_mps": max(abs(row["true_v_mps"]) for row in final_window),
        "final_left_wheel_x_m": rows[-1]["left_wheel_x_m"],
        "final_right_wheel_x_m": rows[-1]["right_wheel_x_m"],
        "peak_pitch_after_ready_deg": max(abs(row["pitch_deg"]) for row in safety_rows),
        "peak_roll_after_ready_deg": max(abs(row["roll_deg"]) for row in safety_rows),
        "nonwheel_contact_samples_after_ready": len(contact_rows),
        "joint_limit_samples_after_ready": len(limit_rows),
        "prehome_nonwheel_contact_samples_excluded": sum(
            row["nonwheel_ground_contacts"] > 0 for row in prehome_rows),
        "contact_scope": "起身完成后计入全身触地与关节限位；home起身阶段单独记录并排除",
        "step_jump_api_active_lock_verified": active_api_lock_checked,
        "mujoco_warning_counts": warnings,
        "core_error": core_error,
        "hip_motor_metrics": hip_metrics,
        "physics": {"terrain": "course", "correct_link_collisions": True,
                    "noslip_iterations": 10, "gas_spring_n_each": 450.0,
                    "joint_continuous_nm": 10.0, "joint_peak_nm": 45.0,
                    "wheel_limit_nm": 5.2, "total_mass_kg": float(model.body_mass.sum())},
        "controller_sha256": sha256(source_path),
        "build_command": build_command,
        "trajectory": "qpos-trajectory.npz，200 Hz，仅用于复现渲染和轨迹核对",
        "control_boundary": "控制器只读取IMU、六电机传感器和用户命令；位置、接触、法向与地形高度只用于触发和验收",
    }
    (args.output / "summary.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
