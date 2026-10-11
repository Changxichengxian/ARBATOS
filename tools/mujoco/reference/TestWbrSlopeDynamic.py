"""从平地动态驶入、驶出真实横坡，检查找平速度和离坡残留。"""
from __future__ import annotations

import argparse
import csv
import ctypes
import hashlib
import json
import math
import os
import subprocess
import sys
from pathlib import Path

import mujoco
import numpy as np


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
REFERENCE = ROOT / "tools/mujoco/reference"
sys.path.insert(0, str(REFERENCE))
sys.path.insert(0, str(HERE))

from RunWbr import MOTORS, SOURCE, inspect_state  # noqa: E402
from WbrGasSpring import load_model  # noqa: E402
from WbrTerrain import ground_geom_ids, ground_height  # noqa: E402


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest().upper()


def load_test_core(output: Path, source: Path):
    sys.path.insert(0, str(ROOT / "tools/mujoco/wheelleg"))
    from RunWheelLeg import find_compiler
    compiler = find_compiler()
    if compiler is None:
        raise RuntimeError("缺少主机 C 编译器")
    executable, tags = compiler
    library = output / f"slope_dynamic_{os.getpid()}{'.dll' if os.name == 'nt' else '.so'}"
    includes = [ROOT / path for path in ("shared/application/wheelleg",
        "shared/application/robot", "shared/components/support")]
    if "msvc" in tags:
        command = [executable, "/nologo", "/LD", "/O2"] + [f"/I{path}" for path in includes]
        command += [str(source), f"/Fe{library}"]
    else:
        command = [executable, "cc"] if "zig" in tags else [executable]
        command += ["-shared", "-O2", "-std=c99"] + [f"-I{path}" for path in includes]
        command += [str(source), "-o", str(library)]
        if os.name != "nt":
            command += ["-fPIC", "-lm"]
    subprocess.run(command, cwd=output, check=True)
    return ctypes.CDLL(str(library)), command


def first_sustained(rows: list[dict], start: float, predicate, duration: float = .25):
    candidate = None
    for row in rows:
        if row["time_s"] < start:
            continue
        if predicate(row):
            candidate = row["time_s"] if candidate is None else candidate
            if row["time_s"] - candidate >= duration:
                return candidate
        else:
            candidate = None
    return None


def run(speed: float, direction: int, output: Path, source: Path) -> dict:
    output.mkdir(parents=True, exist_ok=True)
    profile_path = HERE / "OwnWbrProfile.json"
    profile = json.loads(profile_path.read_text(encoding="utf-8"))
    config = (ctypes.c_float * 20)(*np.asarray(profile["negative_k"]).flatten(),
        profile["theta0"], profile["tp0"], profile["wheel0"], profile["support_n"],
        profile["length_m"], profile["radius_m"], profile["length_kp"], profile["length_kd"])
    lib, build = load_test_core(output, source)
    pointer = ctypes.POINTER(ctypes.c_float)
    lib.OwnWbrCreate.argtypes = [pointer]
    lib.OwnWbrCreate.restype = ctypes.c_void_p
    lib.OwnWbrDestroy.argtypes = [ctypes.c_void_p]
    lib.OwnWbrDestroy.restype = None
    signatures = {
        "OwnWbrCommandStep": [ctypes.c_void_p, pointer, pointer, pointer],
        "OwnWbrSetJointLimits": [ctypes.c_void_p, ctypes.c_float, ctypes.c_float],
        "OwnWbrSetGasSpring": [ctypes.c_void_p, ctypes.c_float],
        "OwnWbrSetStage": [ctypes.c_void_p, ctypes.c_int],
        "OwnWbrSetJumpProfile": [ctypes.c_void_p, ctypes.c_int],
        "OwnWbrSetDriveMode": [ctypes.c_void_p, ctypes.c_int],
        "OwnWbrEnableRise": [ctypes.c_void_p, ctypes.c_int],
    }
    for name, arguments in signatures.items():
        getattr(lib, name).argtypes = arguments
        getattr(lib, name).restype = ctypes.c_int
    handle = lib.OwnWbrCreate(config)
    if not handle:
        raise RuntimeError("控制器创建失败")
    for name, values in (("OwnWbrSetJointLimits", (10.0, 45.0)),
                         ("OwnWbrSetGasSpring", (450.0,)), ("OwnWbrSetStage", (3,)),
                         ("OwnWbrSetJumpProfile", (2,)), ("OwnWbrSetDriveMode", (1,)),
                         ("OwnWbrEnableRise", (1,))):
        if not getattr(lib, name)(handle, *values):
            raise RuntimeError(f"参数配置失败：{name}")

    model = load_model(SOURCE / "mjcf/scene.xml", correct_link_collisions=True,
                       terrain="course", noslip_iterations=10)
    model.opt.timestep = .001
    hip_names = ("ljoint1_actuator", "ljoint4_actuator", "rjoint1_actuator", "rjoint4_actuator")
    hips = np.array([model.actuator(name).id for name in hip_names])
    joints = model.actuator_trnid[hips, 0]
    model.actuator_ctrlrange[hips] = (-45, 45)
    model.actuator_forcerange[hips] = (-45, 45)
    model.jnt_actfrcrange[joints] = (-45, 45)
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    start_x, goal_x = (4.5, 17.0) if direction > 0 else (17.0, 4.5)
    data.qpos[0] += start_x
    data.qpos[1] += 1.5
    mujoco.mj_forward(model, data)

    names = ["imu_quat", "imu_gyro", "imu_accel"] + [
        f"{motor}_{kind}" for motor in MOTORS for kind in ("pos", "vel", "tor")]
    indices = np.concatenate([np.arange(model.sensor(name).adr[0],
        model.sensor(name).adr[0] + model.sensor(name).dim[0]) for name in names])
    grounds = ground_geom_ids(model)
    wheels = {model.body(name).id for name in ("lwlink", "rwlink")}
    base = model.body("base_link").id
    wheel_bodies = [model.body(name).id for name in ("lwlink", "rwlink")]
    sensor = (ctypes.c_float * 28)()
    command = (ctypes.c_float * 3)()
    out = (ctypes.c_float * 40)()
    rows, poses, pose_times = [], [], []
    rise_seen = False
    ready_at = drive_at = goal_at = None
    failure = None
    unsafe_contact_ticks = limit_ticks = 0
    limit_joints, nonwheel_pairs = set(), set()
    peak_rpm = np.zeros(4)
    peak_torque = np.zeros(4)
    saturation_ticks = np.zeros(4, dtype=int)
    duration = 12.0 + 12.5 / speed
    try:
        for tick in range(math.ceil(duration / .001) + 1):
            t = float(data.time)
            if ready_at is not None and drive_at is None:
                drive_at = ready_at + .8
            requested = 0.0
            if drive_at is not None and t >= drive_at and goal_at is None:
                requested = direction * speed
            if tick % 3 == 0:
                sensor[:] = data.sensordata[indices]
                command[:] = [requested, 0.0, 0.0]
                error = lib.OwnWbrCommandStep(handle, sensor, command, out)
                if error or out[34] < 0:
                    failure = f"控制器失败 error={error}, rise={int(out[34])}"
                    break
                if out[34] > 0:
                    rise_seen = True
                elif rise_seen and ready_at is None:
                    ready_at = t
                data.ctrl[:] = out[:6]
            wheel_contacts = other_contacts = 0
            supported_wheels = set()
            for contact in data.contact:
                ground = next((g for g in (contact.geom1, contact.geom2) if g in grounds), None)
                if ground is None:
                    continue
                robot_geom = contact.geom2 if contact.geom1 == ground else contact.geom1
                if robot_geom in grounds:
                    continue
                robot_body = int(model.geom_bodyid[robot_geom])
                if robot_body in wheels:
                    wheel_contacts += 1
                    supported_wheels.add(robot_body)
                else:
                    other_contacts += 1
                    if ready_at is not None:
                        nonwheel_pairs.add((model.geom(ground).name, model.body(robot_body).name))
            limits = np.flatnonzero(data.efc_type == mujoco.mjtConstraint.mjCNSTR_LIMIT_JOINT)
            if ready_at is not None:
                unsafe_contact_ticks += int(other_contacts > 0)
                limit_ticks += int(len(limits) > 0)
                for constraint in limits:
                    limit_joints.add(model.joint(int(data.efc_id[constraint])).name)
            hip_rpm = np.abs(data.qvel[model.jnt_dofadr[joints]]) * 60.0 / (2.0 * math.pi)
            hip_torque = np.abs(np.asarray(data.ctrl)[hips])
            peak_rpm = np.maximum(peak_rpm, hip_rpm)
            peak_torque = np.maximum(peak_torque, hip_torque)
            saturation_ticks += hip_torque >= 44.999

            base_x = float(data.xpos[base, 0])
            if goal_at is None and ((direction > 0 and base_x >= goal_x) or
                                    (direction < 0 and base_x <= goal_x)):
                goal_at = t
            if tick % 5 == 0:
                row = inspect_state(model, data, out, [0, requested, 0, 0, 0, 0, 0])
                wheel_xy = [data.xpos[body, :2] for body in wheel_bodies]
                heights = [ground_height(model, float(xy[0]), float(xy[1])) for xy in wheel_xy]
                terrain_roll = math.degrees(math.atan2(heights[0] - heights[1], .4491))
                row.update(requested_v_mps=requested, terrain_roll_deg=terrain_roll,
                    leg_difference_m=float(out[12] - out[18]), wheel_contacts=wheel_contacts,
                    other_floor_contacts=other_contacts, supported_wheel_count=len(supported_wheels),
                    joint_limit_constraints=len(limits), home_ready=ready_at is not None,
                    rise_phase=int(out[34]), jump_phase=int(out[29]), jump_count=int(out[30]))
                rows.append(row)
                poses.append(data.qpos.copy())
                pose_times.append(t)
            if int(out[29]) != 0 or int(out[30]) != 0:
                failure = "无跳验收中出现跳跃"
                break
            if ready_at is not None and max(abs(out[6]), abs(out[7])) > math.radians(45):
                failure = "姿态超过45度"
                break
            if goal_at is not None and t >= goal_at + 2.0:
                break
            mujoco.mj_step(model, data)
            mujoco.mj_forward(model, data)
            if not np.isfinite(data.qpos).all() or any(item.number for item in data.warning):
                failure = "物理计算异常"
                break
    finally:
        lib.OwnWbrDestroy(handle)

    with (output / "trace.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    np.savez_compressed(output / "qpos-trajectory.npz", time_s=pose_times, qpos=poses)

    ready_rows = [row for row in rows if ready_at is not None and row["time_s"] >= ready_at]
    flat_rows = [row for row in ready_rows if (row["x_m"] >= 16.1 if direction > 0 else row["x_m"] <= 5.4)]
    section_ranges = {"entry": (5.5, 7.0), "cross10": (7.0, 10.0),
                      "between": (10.0, 11.0), "cross15": (11.0, 14.0),
                      "exit": (14.0, 16.0)}
    sections = {}
    for name, (x0, x1) in section_ranges.items():
        selected = [row for row in ready_rows if x0 <= row["x_m"] <= x1]
        if selected:
            sections[name] = dict(max_abs_roll_deg=max(abs(row["roll_deg"]) for row in selected),
                mean_abs_roll_deg=sum(abs(row["roll_deg"]) for row in selected) / len(selected),
                max_abs_pitch_deg=max(abs(row["pitch_deg"]) for row in selected),
                max_abs_leg_difference_m=max(abs(row["leg_difference_m"]) for row in selected))
    flat_start = next((row["time_s"] for row in ready_rows
        if (row["x_m"] >= 16.0 if direction > 0 else row["x_m"] <= 5.5)), None)
    recovered = None if flat_start is None else first_sustained(ready_rows, flat_start,
        lambda row: abs(row["roll_deg"]) < 2.0 and abs(row["leg_difference_m"]) < .010)
    report = dict(speed_mps=speed, direction="+x" if direction > 0 else "-x",
        initialization=f"真实 home 从 x={start_x}, y=1.5 起身，动态驶向 x={goal_x}",
        ready_at_s=ready_at, drive_at_s=drive_at, goal_at_s=goal_at,
        end_time_s=float(data.time), end_x_m=float(data.xpos[base, 0]),
        failure=failure, sections=sections,
        post_exit_recovery_s=None if recovered is None else recovered - flat_start,
        post_exit_max_abs_roll_deg=max((abs(row["roll_deg"]) for row in flat_rows), default=None),
        post_exit_max_abs_leg_difference_m=max((abs(row["leg_difference_m"]) for row in flat_rows), default=None),
        post_exit_last_roll_deg=flat_rows[-1]["roll_deg"] if flat_rows else None,
        post_exit_last_leg_difference_m=flat_rows[-1]["leg_difference_m"] if flat_rows else None,
        max_abs_roll_deg=max((abs(row["roll_deg"]) for row in ready_rows), default=None),
        max_abs_pitch_deg=max((abs(row["pitch_deg"]) for row in ready_rows), default=None),
        min_supported_wheels=min((row["supported_wheel_count"] for row in ready_rows), default=None),
        nonwheel_ground_contact_ticks=unsafe_contact_ticks, nonwheel_ground_pairs=sorted(nonwheel_pairs),
        joint_limit_ticks=limit_ticks, joint_limit_names=sorted(limit_joints),
        hip_peak_rpm=dict(zip(hip_names, peak_rpm.tolist())),
        hip_peak_torque_nm=dict(zip(hip_names, peak_torque.tolist())),
        hip_45nm_saturation_s=dict(zip(hip_names, (saturation_ticks * .001).tolist())),
        gas_spring_n_per_leg=450, hip_continuous_nm=10, hip_peak_nm=45,
        source_path=str(source), source_sha256=digest(source),
        terrain_sha256=digest(REFERENCE / "WbrTerrain.py"),
        trajectory={"path": str(output / "qpos-trajectory.npz"),
                    "fields": ["time_s", "qpos"], "sample_rate_hz": 200,
                    "purpose": "仅用于按实测轨迹复现渲染"},
        build_command=[str(item) for item in build])
    report["passed"] = (failure is None and goal_at is not None and unsafe_contact_ticks == 0 and
        limit_ticks == 0 and recovered is not None and recovered - flat_start < 1.0 and
        report["max_abs_roll_deg"] is not None and report["max_abs_roll_deg"] < 10.0 and
        report["max_abs_pitch_deg"] is not None and report["max_abs_pitch_deg"] < 15.0)
    (output / "summary.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--speed", type=float, default=.5)
    parser.add_argument("--direction", choices=("forward", "reverse"), default="forward")
    parser.add_argument("--output", type=Path,
                        default=ROOT / "local/cache/wbr-slope-dynamic-20261003/formal")
    parser.add_argument("--source", type=Path, default=HERE / "OwnWbrBridge.c")
    args = parser.parse_args()
    if not .1 <= args.speed <= 1.0:
        parser.error("速度必须在0.1–1.0 m/s之间")
    source = args.source.resolve()
    if not source.is_file():
        parser.error(f"控制器源文件不存在：{source}")
    result = run(args.speed, 1 if args.direction == "forward" else -1,
                 args.output.resolve(), source)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
