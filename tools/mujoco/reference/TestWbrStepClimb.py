"""从真实 home 起身，验收自主触台收腿越上 100 mm 直角台阶。"""
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
                     grounds: set[int], target: int):
    wheel_bodies = {model.body("lwlink").id: "lwlink",
                    model.body("rwlink").id: "rwlink"}
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
        ground_name = model.geom(ground).name
        if body in wheel_bodies:
            if ground == target:
                name = wheel_bodies[body]
                upward_normals[name] = max(upward_normals.get(name, -1.0), normal_z)
        else:
            nonwheel.append((body_name, model.geom(other).name, ground_name))
    return nonwheel, upward_normals


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path,
                        default=ROOT / "local/cache/wbr-step-climb-final-20261003")
    parser.add_argument("--duration", type=float, default=22.0)
    parser.add_argument("--release-at", type=float,
                        help="指定松开 W 的时刻；默认全程按住，验证台顶驻车")
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration < 14.0:
        parser.error("仿真时长至少 14 秒")
    if args.release_at is not None and (not math.isfinite(args.release_at) or
            not 8.0 <= args.release_at <= args.duration - 2.0):
        parser.error("松开 W 的时间必须给最终稳定验收留下至少 2 秒")
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=True)

    profile_path = Path(__file__).with_name("OwnWbrProfile.json")
    profile = json.loads(profile_path.read_text(encoding="utf-8"))
    config = (ctypes.c_float * 20)(*np.asarray(profile["negative_k"]).flatten(),
        profile["theta0"], profile["tp0"], profile["wheel0"],
        profile["support_n"], profile["length_m"], profile["radius_m"],
        profile.get("length_kp", 1000), profile.get("length_kd", 60))
    lib, build_command = load_core(args.output)
    pointer = ctypes.POINTER(ctypes.c_float)
    for name, argtypes in (
        ("OwnWbrCommandStep", [ctypes.c_void_p, pointer, pointer, pointer]),
        ("OwnWbrSetJointLimits", [ctypes.c_void_p, ctypes.c_float, ctypes.c_float]),
        ("OwnWbrSetGasSpring", [ctypes.c_void_p, ctypes.c_float]),
        ("OwnWbrSetJumpProfile", [ctypes.c_void_p, ctypes.c_int]),
        ("OwnWbrSetStepJump", [ctypes.c_void_p, ctypes.c_int]),
        ("OwnWbrSetStepClimb", [ctypes.c_void_p, ctypes.c_int]),
        ("OwnWbrGetStepClimbPhase", [ctypes.c_void_p]),
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
    assert lib.OwnWbrSetJumpProfile(handle, 2) == 1
    assert lib.OwnWbrSetStepJump(handle, 0) == 1
    assert lib.OwnWbrSetDriveMode(handle, 1) == 1
    assert lib.OwnWbrEnableRise(handle, 1) == 1
    assert lib.OwnWbrSetStepClimb(handle, 3) == 0
    assert lib.OwnWbrSetStepClimb(handle, 0) == 1
    assert lib.OwnWbrSetStepClimb(handle, 1) == 1
    step_jump_mutex_checked = lib.OwnWbrSetStepJump(handle, 1) == 0

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
    grounds = ground_geom_ids(model)
    target = model.geom("terrain_large100").id
    sensor = (ctypes.c_float * 28)()
    command = (ctypes.c_float * 3)()
    output = (ctypes.c_float * 40)()
    rows = []
    qpos_times = []
    qpos_samples = []
    home_ready_time = None
    trigger_time = None
    phase7_time = None
    action_api_lock_checked = False
    core_error = None
    warning_counts = [0] * len(data.warning)
    try:
        phase = 0
        for tick in range(round(args.duration / model.opt.timestep) + 1):
            time_s = float(data.time)
            drive = .5 if args.release_at is None or time_s < args.release_at else 0.0
            if tick % 3 == 0:
                sensor[:] = data.sensordata[indices]
                command[:] = (drive, 0.0, 0.0)
                status = lib.OwnWbrCommandStep(handle, sensor, command, output)
                if status:
                    core_error = f"core_error_{status}"
                    break
                phase = lib.OwnWbrGetStepClimbPhase(handle)
                if int(output[34]) < 0:
                    core_error = f"rise_failed_{int(output[34])}"
                    break
                if int(output[34]) == 0 and home_ready_time is None:
                    home_ready_time = time_s
                if phase >= 1 and trigger_time is None:
                    trigger_time = time_s
                if phase == 7 and phase7_time is None:
                    phase7_time = time_s
                if 1 <= phase <= 6 and not action_api_lock_checked:
                    action_api_lock_checked = lib.OwnWbrSetStepClimb(handle, 0) == 0
                data.ctrl[:] = output[:6]

            row = inspect_state(model, data, output,
                np.r_[0.0, drive, 0.0, 0.0, 0.0, 0.0, 0.0])
            nonwheel, normals = terrain_contacts(model, data, grounds, target)
            _, _, limits = contact_truth(model, data)
            wheel_positions = [data.body("lwlink").xpos.copy(),
                               data.body("rwlink").xpos.copy()]
            heights = [ground_height(model, float(pos[0]), float(pos[1]))
                       for pos in wheel_positions]
            allowed_body_contact = all(
                1 <= phase <= 6 and body == "base_link" and ground == "terrain_large100"
                for body, _, ground in nonwheel)
            row.update(phase=phase, rise_phase=int(output[34]),
                left_wheel_x_m=float(wheel_positions[0][0]),
                right_wheel_x_m=float(wheel_positions[1][0]),
                left_wheel_z_m=float(wheel_positions[0][2]),
                right_wheel_z_m=float(wheel_positions[1][2]),
                left_ground_z_m=heights[0], right_ground_z_m=heights[1],
                left_target_normal_z=normals.get("lwlink", math.nan),
                right_target_normal_z=normals.get("rwlink", math.nan),
                nonwheel_ground_contacts=len(nonwheel),
                nonwheel_contact_detail=repr(nonwheel),
                body_contact_allowed=allowed_body_contact,
                joint_limit_constraints=limits)
            rows.append(row)
            if tick % 5 == 0:
                qpos_times.append(time_s)
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
    after_ready = [row for row in rows if home_ready_time is not None and
                   row["time_s"] >= home_ready_time]
    prehome = [row for row in rows if home_ready_time is None or
               row["time_s"] < home_ready_time]
    contact_violations = [row for row in after_ready
                          if row["nonwheel_ground_contacts"] and
                          not row["body_contact_allowed"]]
    limit_rows = [row for row in after_ready if row["joint_limit_constraints"]]
    wheel_radius = profile["radius_m"]
    expected_phase = 7 if args.release_at is None else 8
    stable = bool(final_window) and \
        final_window[-1]["time_s"] - final_window[0]["time_s"] >= 1.99 and all(
        row["phase"] == expected_phase and row["left_wheel_x_m"] > 7.01 and
        row["right_wheel_x_m"] > 7.01 and
        row["left_wheel_z_m"] > .145 and row["right_wheel_z_m"] > .145 and
        row["left_target_normal_z"] >= .80 and row["right_target_normal_z"] >= .80 and
        abs(row["left_wheel_z_m"] - row["left_ground_z_m"] - wheel_radius) < .015 and
        abs(row["right_wheel_z_m"] - row["right_ground_z_m"] - wheel_radius) < .015 and
        row["nonwheel_ground_contacts"] == 0 and
        min(row["left_leg_m"], row["right_leg_m"]) >= .19 and
        abs(row["pitch_deg"]) < 8 and abs(row["roll_deg"]) < 8 and
        abs(row["true_v_mps"]) < .12 for row in final_window)
    warnings = {str(index): number for index, number in enumerate(warning_counts) if number}
    source_path = Path(__file__).with_name("OwnWbrBridge.c")
    report = {
        "passed": core_error is None and stable and not contact_violations and
                  not limit_rows and not warnings and step_jump_mutex_checked and
                  action_api_lock_checked and phase7_time is not None and
                  trigger_time is not None and phase7_time - trigger_time <= 2.6,
        "home_ready_time_s": home_ready_time,
        "trigger_time_s": trigger_time,
        "phase7_time_s": phase7_time,
        "action_duration_s": None if trigger_time is None or phase7_time is None
            else phase7_time - trigger_time,
        "command": {"speed_mps": .5, "release_at_s": args.release_at,
                    "w_held_through_end": args.release_at is None, "drive_mode": 1},
        "prehome_joint_limit_samples": sum(row["joint_limit_constraints"] > 0
                                            for row in prehome),
        "joint_limit_samples_after_ready": len(limit_rows),
        "contact_violations_after_ready": len(contact_violations),
        "last_2s_both_wheels_supported": stable,
        "last_2s_min_leg_m": min(min(row["left_leg_m"], row["right_leg_m"])
                                  for row in final_window),
        "last_2s_peak_pitch_deg": max(abs(row["pitch_deg"]) for row in final_window),
        "last_2s_peak_roll_deg": max(abs(row["roll_deg"]) for row in final_window),
        "last_2s_peak_speed_mps": max(abs(row["true_v_mps"]) for row in final_window),
        "phase_contact_rule": "phase1..6仅允许base_link接触terrain_large100；其它阶段非轮接触为0",
        "step_jump_mutex_checked": step_jump_mutex_checked,
        "action_api_lock_checked": action_api_lock_checked,
        "mujoco_warning_counts": warnings,
        "core_error": core_error,
        "controller_sha256": sha256(source_path),
        "build_command": build_command,
        "trajectory": "qpos-trajectory.npz，200 Hz，仅用于复现渲染和轨迹核对",
        "control_boundary": "控制仅用IMU、电机编码器/反馈和用户命令；位置、接触、法向、地形高度只用于验收",
    }
    (args.output / "summary.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
