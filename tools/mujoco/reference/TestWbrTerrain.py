"""从真实 home 起身，不触发跳跃，验收浅坑、缓起伏和直角台阶。"""
from __future__ import annotations

import argparse
import csv
import ctypes
import json
import math
from pathlib import Path

import mujoco
import numpy as np

from RunOwnWbr import load_core
from RunWbr import MOTORS, ROOT, SOURCE, inspect_state
from WbrGasSpring import load_model
from WbrTerrain import COURSE_FINISH_X_M, COURSE_SEGMENTS, ground_geom_ids, ground_height, scenario_pose


def run(speed: float, output: Path, scenario: str = "course", noslip_iterations: int = 10) -> dict:
    output.mkdir(parents=True, exist_ok=True)
    profile = json.loads(Path(__file__).with_name("OwnWbrProfile.json").read_text(encoding="utf-8"))
    config = (ctypes.c_float * 20)(*np.asarray(profile["negative_k"]).flatten(),
        profile["theta0"], profile["tp0"], profile["wheel0"], profile["support_n"],
        profile["length_m"], profile["radius_m"], profile["length_kp"], profile["length_kd"])
    lib, build = load_core(output)
    pointer = ctypes.POINTER(ctypes.c_float)
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
                         ("OwnWbrSetJumpProfile", (2,)),
                         ("OwnWbrSetDriveMode", (1 if scenario == "course" else 0,)),
                         ("OwnWbrEnableRise", (1 if scenario == "course" else 0,))):
        if not getattr(lib, name)(handle, *values):
            raise RuntimeError(f"参数配置失败：{name}")
    model = load_model(SOURCE / "mjcf/scene.xml", correct_link_collisions=True,
                       terrain="course", noslip_iterations=noslip_iterations)
    model.opt.timestep = .001
    hips = np.array([model.actuator(name).id for name in
        ("ljoint1_actuator", "ljoint4_actuator", "rjoint1_actuator", "rjoint4_actuator")])
    joints = model.actuator_trnid[hips, 0]
    model.actuator_ctrlrange[hips] = (-45, 45)
    model.actuator_forcerange[hips] = (-45, 45)
    model.jnt_actfrcrange[joints] = (-45, 45)
    data = mujoco.MjData(model)
    if scenario == "course":
        mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    else:
        data.qpos[:] = scenario_pose(model, profile["qpos"], scenario)
    mujoco.mj_forward(model, data)
    names = ["imu_quat", "imu_gyro", "imu_accel"] + [
        f"{motor}_{kind}" for motor in MOTORS for kind in ("pos", "vel", "tor")]
    indices = np.concatenate([np.arange(model.sensor(name).adr[0],
        model.sensor(name).adr[0] + model.sensor(name).dim[0]) for name in names])
    grounds = ground_geom_ids(model)
    wheels = {model.body(name).id for name in ("lwlink", "rwlink")}
    base = model.body("base_link").id
    sensor, command, out = (ctypes.c_float * 28)(), (ctypes.c_float * 3)(), (ctypes.c_float * 40)()
    rows = []
    rise_seen = False
    ready_at = None if scenario == "course" else 0.0
    finish_at = None
    failure = None
    duration = 8 + 7.5 / speed if scenario == "course" else 30.0
    start_xy = data.xpos[base, :2].copy()
    max_displacement = 0.0
    unsafe_contact_ticks = limit_ticks = 0
    max_pitch = max_roll = 0.0
    limit_joints, nonwheel_pairs, touched = set(), set(), set()
    contact_times = {model.geom(index).name: [None, None] for index in grounds}
    max_x = float(data.xpos[base, 0])
    late_supported_wheels = 2
    try:
        for tick in range(math.ceil(duration / .001) + 1):
            t = float(data.time)
            requested = speed if scenario == "course" and ready_at is not None and t >= ready_at + 1.0 else 0.0
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
                ground = next((geom for geom in (contact.geom1, contact.geom2) if geom in grounds), None)
                if ground is None:
                    continue
                robot_geom = contact.geom2 if contact.geom1 == ground else contact.geom1
                if robot_geom in grounds:
                    continue
                name = model.geom(ground).name
                touched.add(name)
                if contact_times[name][0] is None:
                    contact_times[name][0] = t
                contact_times[name][1] = t
                if int(model.geom_bodyid[robot_geom]) in wheels:
                    wheel_contacts += 1
                    supported_wheels.add(int(model.geom_bodyid[robot_geom]))
                else:
                    other_contacts += 1
                    if ready_at is not None:
                        nonwheel_pairs.add((name, model.body(int(model.geom_bodyid[robot_geom])).name))
            limits = np.flatnonzero(data.efc_type == mujoco.mjtConstraint.mjCNSTR_LIMIT_JOINT)
            if ready_at is not None:
                unsafe_contact_ticks += int(other_contacts > 0)
                limit_ticks += int(len(limits) > 0)
                for constraint in limits:
                    limit_joints.add(model.joint(int(data.efc_id[constraint])).name)
                max_pitch = max(max_pitch, abs(math.degrees(out[7])))
                max_roll = max(max_roll, abs(math.degrees(out[6])))
            max_x = max(max_x, float(data.xpos[base, 0]))
            if scenario != "course" and t >= 25:
                late_supported_wheels = min(late_supported_wheels, len(supported_wheels))
            max_displacement = max(max_displacement, float(np.linalg.norm(data.xpos[base, :2] - start_xy)))
            if tick % 5 == 0:
                row = inspect_state(model, data, out, [0, requested, 0, 0, 0, 0, 0])
                row.update(wheel_contacts=wheel_contacts, other_floor_contacts=other_contacts,
                    supported_wheel_count=len(supported_wheels),
                    joint_limit_constraints=len(limits), home_ready=ready_at is not None,
                    rise_phase=int(out[34]), jump_phase=int(out[29]), jump_count=int(out[30]),
                    left_ground_m=ground_height(model, float(data.body("lwlink").xpos[0]),
                                               float(data.body("lwlink").xpos[1])),
                    right_ground_m=ground_height(model, float(data.body("rwlink").xpos[0]),
                                                float(data.body("rwlink").xpos[1])))
                rows.append(row)
            if int(out[29]) != 0 or int(out[30]) != 0:
                failure = "无跳验收中出现跳跃"
                break
            if ready_at is not None and max(abs(out[6]), abs(out[7])) > math.radians(45):
                failure = "姿态超过45度"
                break
            if scenario == "course" and data.xpos[base, 0] >= COURSE_FINISH_X_M and finish_at is None:
                finish_at = t
            if finish_at is not None and t >= finish_at + 1.0:
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
    report = dict(scenario=scenario, speed_mps=speed, ready_at_s=ready_at, finish_at_s=finish_at,
        noslip_iterations=noslip_iterations,
        initialization="home 起身后无跳行驶" if scenario == "course" else "profile 站姿完整刚体旋转至横坡中部，未从平地驶上坡",
        max_horizontal_displacement_m=max_displacement,
        end_horizontal_displacement_m=float(np.linalg.norm(data.xpos[base, :2] - start_xy)),
        end_roll_deg=math.degrees(out[6]), end_pitch_deg=math.degrees(out[7]),
        end_x_m=float(data.xpos[base, 0]), max_x_m=max_x, end_time_s=float(data.time),
        max_pitch_deg=max_pitch, max_roll_deg=max_roll,
        nonwheel_ground_contact_ticks=unsafe_contact_ticks, joint_limit_ticks=limit_ticks,
        joint_limit_names=sorted(limit_joints), nonwheel_ground_pairs=sorted(nonwheel_pairs),
        last5s_min_supported_wheels=late_supported_wheels if scenario != "course" else None,
        ground_contact_times=contact_times, jump_count=int(out[30]), failure=failure,
        total_mass_kg=float(model.body_mass.sum()), hip_continuous_nm=10, hip_peak_nm=45,
        core_jump_profile=2,
        build_command=build, course=[list(item) for item in COURSE_SEGMENTS])
    complete = finish_at is not None if scenario == "course" else data.time >= duration - .001
    late = [row for row in rows if row["time_s"] >= rows[-1]["time_s"] - 5.0]
    report["last5s_max_speed_mps"] = max(abs(row["true_v_mps"]) for row in late)
    report["last5s_max_roll_deg"] = max(abs(row["roll_deg"]) for row in late)
    report["end_leg_lengths_m"] = [float(out[12]), float(out[18])]
    report["passed"] = (failure is None and complete and not unsafe_contact_ticks
        and not limit_ticks and max_pitch <= 15 and max_roll <= (10 if scenario == "course" else 20)
        and (scenario == "course" or (max_displacement < .15 and late_supported_wheels == 2
            and report["last5s_max_roll_deg"] < 2.0 and report["last5s_max_speed_mps"] < .02)))
    (output / "summary.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--speed", type=float, default=.5)
    parser.add_argument("--scenario", choices=("course", "cross10", "cross15"), default="course")
    parser.add_argument("--noslip-iterations", type=int, default=10,
                        help="接触无滑移迭代数；0复现原软接触基线，默认10")
    parser.add_argument("--output", type=Path, default=ROOT / "local/cache/wbr-terrain-20261003")
    args = parser.parse_args()
    if not math.isfinite(args.speed) or not .1 <= args.speed <= 1.0:
        parser.error("速度必须为0.1–1.0 m/s有限数")
    if args.noslip_iterations < 0:
        parser.error("无滑移迭代数必须非负")
    report = run(args.speed, args.output.resolve(), args.scenario, args.noslip_iterations)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
