"""固定控制核心，比较恒推力气弹簧、站立腿长和真实膝挡块承重。"""
from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
import csv
import ctypes
import hashlib
import json
from pathlib import Path
import time

import mujoco
import numpy as np

from RunOwnWbr import load_core
from RunWbr import MOTORS
from WbrGasSpring import load_model
from WbrMotorTelemetry import WbrMotorTelemetry


ROOT = Path(__file__).resolve().parents[3]
HERE = Path(__file__).parent
HIP_NAMES = ("ljoint1", "ljoint4", "rjoint1", "rjoint4")
KNEE_NAMES = ("llink1child1joint", "rlink1child1joint")
DT = .003


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest().upper()


def configure_library(path):
    lib = ctypes.CDLL(str(path))
    pointer = ctypes.POINTER(ctypes.c_float)
    lib.OwnWbrCreate.argtypes, lib.OwnWbrCreate.restype = [pointer], ctypes.c_void_p
    lib.OwnWbrDestroy.argtypes = [ctypes.c_void_p]
    lib.OwnWbrCommandStep.argtypes = [ctypes.c_void_p, pointer, pointer, pointer]
    lib.OwnWbrCommandStep.restype = ctypes.c_int
    for name, arguments in (
        ("OwnWbrSetGasSpring", [ctypes.c_void_p, ctypes.c_float]),
        ("OwnWbrSetJointLimits", [ctypes.c_void_p, ctypes.c_float, ctypes.c_float]),
    ):
        function = getattr(lib, name)
        function.argtypes, function.restype = arguments, ctypes.c_int
    return lib


def run_case(task):
    started = time.monotonic()
    output_dir = Path(task["output"])
    force, length = task["force_n"], task["length_m"]
    mode = task["mode"]
    case_name = f"{mode}-F{force:g}-L{round(length * 1000)}"
    start_length = task.get("start_length_m") or length
    push_n = task.get("push_n", 0.)
    if abs(start_length - length) > 1e-8:
        case_name += f"-from{round(start_length * 1000)}"
    if push_n:
        case_name += f"-push{push_n:g}"
    case_dir = output_dir / case_name
    case_dir.mkdir(parents=True, exist_ok=True)
    model = load_model(gas_force_n=force, correct_link_collisions=True,
                       terrain="flat", guide_rollers="passive",
                       noslip_iterations=10, knee_range="cad120")
    model.opt.timestep = .001
    poses = json.loads((ROOT / "local/cache/wbr-user-record-20261004/spring-static-support.json").read_text(encoding="utf-8"))["rows"]
    pose = next(row for row in poses if abs(row["length_m"] - start_length) < 1e-8)
    stop_pose = next(row for row in poses if abs(row["length_m"] - .180) < 1e-8)
    knee_ids = [model.joint(name).id for name in KNEE_NAMES]
    for side, jid in enumerate(knee_ids):
        stop_q = stop_pose["qpos"][int(model.jnt_qposadr[jid])]
        model.jnt_range[jid, 1 if side == 0 else 0] = stop_q
    stop_check = mujoco.MjData(model)
    stop_check.qpos[:] = stop_pose["qpos"]
    mujoco.mj_forward(model, stop_check)
    # 参考腿机构位于XZ平面，两侧CAD轴向偏置不计作腿长。
    stop_lengths = [float(np.linalg.norm((stop_check.body(side + "wlink").xpos -
        stop_check.body(side + "link1").xpos)[[0, 2]])) for side in ("l", "r")]
    if max(abs(value - .18) for value in stop_lengths) > 1e-6:
        raise RuntimeError("挡块闭链姿态不等于180mm")
    hips = np.array([model.actuator(name + "_actuator").id for name in HIP_NAMES])
    hip_dofs = model.jnt_dofadr[model.actuator_trnid[hips, 0]]
    model.actuator_ctrlrange[hips] = [-10., 10.]
    model.actuator_forcerange[hips] = [-10., 10.]
    model.jnt_actfrcrange[model.actuator_trnid[hips, 0]] = [-10., 10.]
    data = mujoco.MjData(model)
    data.qpos[:] = pose["qpos"]
    mujoco.mj_forward(model, data)
    profile = json.loads((HERE / "OwnWbrProfile.json").read_text(encoding="utf-8"))
    # 挡块模式只改变支撑前馈；不改电机输出，不用接触真值反馈控制。
    support = profile["support_n"]
    if mode == "stop":
        support = min(stop_pose["gravity_opposes_extension_n"] / 2 - 5.,
                      stop_pose["gas_assists_extension_n"] / 2 * force / 450.)
    config = (ctypes.c_float * 20)(*np.asarray(profile["negative_k"]).flatten(),
        profile["theta0"], profile["tp0"], profile["wheel0"], support,
        length, profile["radius_m"], profile["length_kp"], profile["length_kd"])
    lib = configure_library(task["library"])
    handle = lib.OwnWbrCreate(config)
    if not handle:
        raise RuntimeError("控制器初始化失败")
    assert lib.OwnWbrSetGasSpring(handle, force) == 1
    assert lib.OwnWbrSetJointLimits(handle, 10., 45.) == 1
    # 本实验不跳跃、不起身，不开启限于450N调好的跳跃动作档。
    names = ["imu_quat", "imu_gyro", "imu_accel"]
    names += [f"{motor}_{kind}" for motor in MOTORS for kind in ("pos", "vel", "tor")]
    indices = np.concatenate([np.arange(model.sensor(n).adr[0],
        model.sensor(n).adr[0] + model.sensor(n).dim[0]) for n in names])
    sensor, command, output = (ctypes.c_float * 28)(), (ctypes.c_float * 3)(), (ctypes.c_float * 40)()
    telemetry = WbrMotorTelemetry(model)
    columns = ["time_s", "pitch_deg", "roll_deg", "true_v_mps", "x_m", "y_m",
               "left_length_m", "right_length_m"]
    columns += [f"{name}_torque_nm" for name in HIP_NAMES]
    columns += [f"{name}_speed_radps" for name in HIP_NAMES]
    columns += [f"{name}_stop_reaction_nm" for name in ("left", "right")]
    columns += ["other_ground_contacts", "left_wheel_contact", "right_wheel_contact",
                "left_wheel_estimate_w", "right_wheel_estimate_w", "other_joint_limits"]
    columns += ["left_geometric_length_m", "right_geometric_length_m", "base_height_m",
                "true_vy_mps", "yaw_deg", "base_wx_radps", "base_wy_radps", "base_wz_radps",
                "left_wheel_normal_n", "right_wheel_normal_n",
                "left_stop_penetration_rad", "right_stop_penetration_rad", "closure_error_m"]
    rows = []
    failure = None
    floor_id = model.geom("floor").id
    wheel_bodies = [model.body("lwlink").id, model.body("rwlink").id]
    initial_x = float(data.qpos[0])
    base_id = model.body("base_link").id
    contact_force = np.zeros(6)
    try:
        for tick in range(round(task["seconds"] / DT)):
            sensor[:] = data.sensordata[indices]
            error = lib.OwnWbrCommandStep(handle, sensor, command, output)
            if error:
                failure = f"core_error_{error}"
                break
            data.ctrl[:] = output[:6]
            # 复验时短暂水平推机身，之后撤力；不提供竖直支撑。
            data.xfrc_applied[base_id, 0] = push_n if 15. <= data.time < 15.2 else 0.
            mujoco.mj_step(model, data, nstep=3)
            mujoco.mj_forward(model, data)
            torque = data.actuator_force[hips] * model.actuator_gear[hips, 0]
            pitch, roll = np.rad2deg(float(output[7])), np.rad2deg(float(output[6]))
            reactions = [0., 0.]
            other_limits = 0
            for index in range(data.nefc):
                if data.efc_type[index] == mujoco.mjtConstraint.mjCNSTR_LIMIT_JOINT:
                    joint = int(data.efc_id[index])
                    if joint in knee_ids:
                        side = knee_ids.index(joint)
                        q = data.qpos[model.jnt_qposadr[joint]]
                        short_end = model.jnt_range[joint, 1 if side == 0 else 0]
                        if abs(q - short_end) < .02:
                            reactions[side] += float(data.efc_force[index])
                        else:
                            other_limits += 1
                    else:
                        other_limits += 1
            ground = [0, 0]
            wheel_load = [0., 0.]
            other_contacts = 0
            for contact_index, contact in enumerate(data.contact):
                if floor_id not in (contact.geom1, contact.geom2):
                    continue
                geom = contact.geom2 if contact.geom1 == floor_id else contact.geom1
                body = int(model.geom_bodyid[geom])
                if body in wheel_bodies:
                    side = wheel_bodies.index(body)
                    ground[side] += 1
                    mujoco.mj_contactForce(model, data, contact_index, contact_force)
                    wheel_load[side] += max(0., float(contact_force[0]))
                else:
                    other_contacts += 1
            motors = telemetry.snapshot(data)
            base_rotation = data.xmat[base_id].reshape(3, 3)
            geometric_length = [np.linalg.norm((base_rotation.T @
                (data.body(side + "wlink").xpos - data.body(side + "link1").xpos))[[0, 2]])
                for side in ("l", "r")]
            penetrations = [max(0., (1 if side == 0 else -1) *
                (data.qpos[model.jnt_qposadr[jid]] - model.jnt_range[jid, 1 if side == 0 else 0]))
                for side, jid in enumerate(knee_ids)]
            equality = data.efc_pos[data.efc_type == mujoco.mjtConstraint.mjCNSTR_EQUALITY]
            closure_error = float(np.max(np.linalg.norm(equality.reshape(-1, 3), axis=1)))
            row = [float(data.time), pitch, roll, float(data.qvel[0]), float(data.qpos[0]),
                   float(data.qpos[1]), float(output[12]), float(output[18]),
                   *torque, *data.qvel[hip_dofs], *reactions, other_contacts, *ground,
                   *[wheel["electrical_power_estimate_w"] for wheel in motors["wheels"]], other_limits,
                   *geometric_length, float(data.qpos[2]), float(data.qvel[1]),
                   np.rad2deg(float(output[8])), *data.qvel[3:6], *wheel_load, *penetrations, closure_error]
            rows.append(row)
            if not np.all(np.isfinite(data.qpos)) or max(abs(pitch), abs(roll)) > 35:
                failure = "nonfinite_or_fall"
                break
    finally:
        lib.OwnWbrDestroy(handle)
    array = np.asarray(rows).reshape((-1, len(columns)))
    sampled = array[::10]
    with (case_dir / "trace.csv").open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.writer(stream)
        writer.writerow(columns)
        writer.writerows(sampled.tolist())
    window = array[array[:, 0] >= task["settle_seconds"]]
    report = dict(case=case_name, force_n=force, target_length_m=length, mode=mode,
        support_feedforward_n_each=support, stop_length_m=.180,
        start_length_m=start_length, horizontal_push_n=push_n,
        stop_preload_n_each=5. if mode == "stop" else 0.,
        hip_frictionloss_nm=model.dof_frictionloss[hip_dofs].tolist(),
        stop_geometric_length_m=stop_lengths,
        knee_ranges_rad=model.jnt_range[knee_ids].tolist(),
        seconds=task["seconds"], settle_seconds=task["settle_seconds"],
        actual_seconds=float(array[-1, 0]) if len(array) else 0., failure=failure,
        controller_sha256=task["controller_sha256"], library_sha256=digest(task["library"]),
        motor_order=HIP_NAMES, columns=columns,
        simulation_boundary="1ms物理/3ms控制；相同独立闭链静态姿态释放，无起身；不注入外部支撑；接触仅用于统计",
        energy_boundary="髋使用扭矩平方热负担指标，未标定6216P电流常数，绝对电流/电池耗电不冒充测量",
        wall_seconds=time.monotonic() - started)
    if len(array):
        report.update(full_pitch_peak_deg=float(np.max(np.abs(array[:, 1]))),
            full_roll_peak_deg=float(np.max(np.abs(array[:, 2]))),
            full_speed_peak_mps=float(np.max(np.abs(array[:, 3]))),
            full_nonwheel_ground_samples=int(np.sum(array[:, 18] > 0)))
    if len(window):
        tau, vel = window[:, 8:12], window[:, 12:16]
        report.update(hip_mean_nm=tau.mean(0).tolist(),
            hip_abs_mean_nm=np.abs(tau).mean(0).tolist(),
            hip_rms_nm=np.sqrt(np.mean(tau * tau, axis=0)).tolist(),
            hip_peak_nm=np.max(np.abs(tau), axis=0).tolist(),
            hip_heat_index_nm2=float(np.mean(np.sum(tau * tau, axis=1))),
            hip_mechanical_mean_w=np.mean(tau * vel, axis=0).tolist(),
            hip_mechanical_abs_mean_w=np.mean(np.abs(tau * vel), axis=0).tolist(),
            hip_limit_fraction=np.mean(np.abs(tau) >= 9.99, axis=0).tolist(),
            actual_length_mean_m=window[:, 6:8].mean(0).tolist(),
            actual_length_range_m=np.stack((window[:, 6:8].min(0), window[:, 6:8].max(0))).tolist(),
            pitch_peak_deg=float(np.max(np.abs(window[:, 1]))),
            roll_peak_deg=float(np.max(np.abs(window[:, 2]))),
            speed_rms_mps=float(np.sqrt(np.mean(window[:, 3] ** 2))),
            x_span_m=float(np.ptp(window[:, 4])),
            x_displacement_from_release_m=float(window[-1, 4] - initial_x),
            stop_contact_fraction=np.mean(window[:, 16:18] > .01, axis=0).tolist(),
            both_stops_contact_fraction=float(np.mean(np.all(window[:, 16:18] > .01, axis=1))),
            stop_reaction_p05_nm=np.percentile(window[:, 16:18], 5, axis=0).tolist(),
            stop_reaction_mean_nm=window[:, 16:18].mean(0).tolist(),
            stop_reaction_peak_nm=window[:, 16:18].max(0).tolist(),
            nonwheel_ground_samples=int(np.sum(window[:, 18] > 0)),
            wheel_support_fraction=np.mean(window[:, 19:21] > 0, axis=0).tolist(),
            wheel_electrical_mean_w=window[:, 21:23].mean(0).tolist(),
            other_joint_limit_samples=int(np.sum(window[:, 23] > 0)))
        report.update(geometric_length_mean_m=window[:, 24:26].mean(0).tolist(),
            geometric_length_max_error_m=float(np.max(np.abs(window[:, 24:26] - length))),
            base_height_mean_m=float(window[:, 26].mean()),
            base_height_range_m=[float(window[:, 26].min()), float(window[:, 26].max())],
            planar_speed_rms_mps=float(np.sqrt(np.mean(window[:, 3] ** 2 + window[:, 27] ** 2))),
            y_span_m=float(np.ptp(window[:, 5])), yaw_span_deg=float(np.ptp(window[:, 28])),
            angular_speed_peak_radps=float(np.max(np.linalg.norm(window[:, 29:32], axis=1))),
            wheel_normal_mean_n=window[:, 32:34].mean(0).tolist(),
            wheel_normal_p05_n=np.percentile(window[:, 32:34], 5, axis=0).tolist(),
            wheel_loaded_fraction=np.mean(window[:, 32:34] > 1., axis=0).tolist(),
            stop_penetration_max_rad=window[:, 34:36].max(0).tolist(),
            closure_error_max_m=float(window[:, 36].max()))
        report["stable"] = bool(failure is None and window[-1, 0] >= task["seconds"] - .01 and
            report["pitch_peak_deg"] < 8 and report["roll_peak_deg"] < 8 and
            report["speed_rms_mps"] < .08 and not report["nonwheel_ground_samples"] and
            not report["other_joint_limit_samples"] and min(report["wheel_support_fraction"]) > .99 and
            not any(w.number for w in data.warning) and report["planar_speed_rms_mps"] < .08 and
            report["y_span_m"] < .05 and report["yaw_span_deg"] < 5. and
            min(report["wheel_loaded_fraction"]) > .99 and report["closure_error_max_m"] < .001)
        report["height_within_5mm"] = bool(np.max(np.abs(window[:, 6:8] - length)) < .005)
        report["stop_seated"] = bool(mode == "stop" and report["stable"] and
            min(report["stop_contact_fraction"]) > .95)
    else:
        report.update(stable=False, height_within_5mm=False, stop_seated=False)
    report["mujoco_warnings"] = {str(i): int(w.number) for i, w in enumerate(data.warning) if w.number}
    (case_dir / "summary.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--forces", type=float, nargs="+", default=[0, 200, 250, 300, 350, 400, 450, 500])
    parser.add_argument("--lengths", type=float, nargs="+", default=[.18, .20, .22, .24, .28, .32])
    parser.add_argument("--seconds", type=float, default=70.)
    parser.add_argument("--settle-seconds", type=float, default=10.)
    parser.add_argument("--workers", type=int, default=3)
    parser.add_argument("--library", type=Path, help="复用本扫描已编译且来源已核对的控制器库")
    parser.add_argument("--start-length", type=float, help="复验从另一腿长收至目标，仍用同一控制器")
    parser.add_argument("--push-n", type=float, default=0., help="复验15秒处施加0.2秒水平推力")
    parser.add_argument("--modes", nargs="+", choices=("normal", "stop"), default=["normal", "stop"])
    args = parser.parse_args()
    args.output = args.output.resolve()
    if not all(np.isfinite(f) and 0 <= f <= 1000 for f in args.forces):
        parser.error("气弹簧恒推力必须在0至1000N之间")
    if not 0 <= args.settle_seconds < args.seconds or args.workers < 1:
        parser.error("统计起点必须早于结束时间，进程数至少为1")
    available_lengths = (.18, .19, .20, .22, .24, .26, .28, .30, .32)
    for value in args.lengths + ([] if args.start_length is None else [args.start_length]):
        if not any(abs(value - candidate) < 1e-8 for candidate in available_lengths):
            parser.error("腿长必须使用已经独立求解的180至320mm静态姿态")
    if not np.isfinite(args.push_n) or abs(args.push_n) > 30:
        parser.error("水平扰动仅限正负30N内")
    if args.push_n and (args.seconds < 15.2 or args.settle_seconds <= 15.2):
        parser.error("扰动复验的统计窗口必须在15.2秒撤力之后")
    args.output.mkdir(parents=True, exist_ok=True)
    script_before = digest(__file__)
    (args.output / "SweepWbrGasSpring.executed.py").write_bytes(Path(__file__).read_bytes())
    source = HERE / "OwnWbrBridge.c"
    before = digest(source)
    if args.library:
        origin = args.library.resolve().parent / "summary.json"
        if not origin.is_file() or json.loads(origin.read_text(encoding="utf-8"))["controller_sha256"] != before:
            parser.error("复用库缺少同目录扫描来源记录，或控制器源码哈希不一致")
        origin_reports = json.loads(origin.read_text(encoding="utf-8"))["results"]
        if digest(args.library) not in {report["library_sha256"] for report in origin_reports}:
            parser.error("复用库哈希与来源扫描记录不一致")
        library, command = configure_library(args.library.resolve()), ["reuse", str(args.library.resolve())]
    else:
        library, command = load_core(args.output)
    tasks = []
    for mode in args.modes:
        for length in args.lengths:
            if mode == "stop" and abs(length - .18) > 1e-8:
                continue
            for force in args.forces:
                tasks.append(dict(output=str(args.output.resolve()), library=str(Path(library._name).resolve()),
                    force_n=force, length_m=length, mode=mode, seconds=args.seconds,
                    start_length_m=args.start_length, push_n=args.push_n,
                    settle_seconds=args.settle_seconds, controller_sha256=before))
    reports = []
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        futures = [pool.submit(run_case, task) for task in tasks]
        for future in as_completed(futures):
            report = future.result()
            reports.append(report)
            print(json.dumps({key: report.get(key) for key in ("case", "stable", "height_within_5mm", "stop_seated",
                "actual_length_mean_m", "hip_rms_nm", "hip_heat_index_nm2", "stop_contact_fraction", "wall_seconds", "failure")}, ensure_ascii=False), flush=True)
    summary = dict(controller_sha256=before, controller_unchanged=before == digest(source),
        script_sha256=script_before, build_command=command,
        source_sha256={name: digest(HERE / name) for name in
            ("OwnWbrProfile.json", "WbrGasSpring.py", "WbrGuideRollers.py", "WbrKneeRange.py",
             "WbrLinkCollision.py", "WbrMotorTelemetry.py")},
        pose_source_sha256=digest(ROOT / "local/cache/wbr-user-record-20261004/spring-static-support.json"),
        mujoco_version=mujoco.__version__,
        stop_source="用户确认180mm；参考闭链膝关节限位近似实体挡块",
        results=reports)
    (args.output / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
