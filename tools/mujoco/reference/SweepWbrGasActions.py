"""固定动作与180mm实体挡块，比较气弹簧推力；缓存诊断副本不改正式控制器。"""
from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
import csv
import ctypes
import difflib
import hashlib
import json
import math
from pathlib import Path
import sys
import types

import mujoco
import numpy as np

import TestWbrImpactClimb as climb
from RunWbr import MOTORS, inspect_state
from WbrGasSpring import load_model
from WbrMotorTelemetry import WbrMotorTelemetry

ROOT = Path(__file__).resolve().parents[3]
HERE = Path(__file__).parent
HIPS = ("ljoint1", "ljoint4", "rjoint1", "rjoint4")
KNEES = ("llink1child1joint", "rlink1child1joint")
EXPECTED = "328B302BC32BEFE45F1AEA796AB5A6C75CE300D1B0F28CE8B64DD4286235BD3A"


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest().upper()


def save(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2,
                                   allow_nan=False) + "\n", encoding="utf-8")


def apply_stop(model):
    poses = json.loads((ROOT / "local/cache/wbr-user-record-20261004/spring-static-support.json").read_text(encoding="utf-8"))["rows"]
    pose = next(row for row in poses if abs(row["length_m"] - .18) < 1e-8)
    ids = [model.joint(name).id for name in KNEES]
    for side, jid in enumerate(ids):
        model.jnt_range[jid, 1 if side == 0 else 0] = pose["qpos"][int(model.jnt_qposadr[jid])]
    check = mujoco.MjData(model)
    check.qpos[:len(pose["qpos"])] = pose["qpos"]
    mujoco.mj_forward(model, check)
    lengths = [float(np.linalg.norm((check.body(side + "wlink").xpos -
               check.body(side + "link1").xpos)[[0, 2]])) for side in ("l", "r")]
    if max(abs(x - .18) for x in lengths) > 1e-6:
        raise RuntimeError("挡块闭链姿态不等于180mm")
    home = model.key("home").qpos
    violations = []
    for jid in ids:
        q = float(home[int(model.jnt_qposadr[jid])])
        lo, hi = model.jnt_range[jid]
        if q < lo - 1e-6 or q > hi + 1e-6:
            violations.append(dict(joint=model.joint(jid).name, qpos_rad=q,
                                   range_rad=[float(lo), float(hi)]))
    return dict(stop_length_m=.18, verified_lengths_m=lengths,
                knee_ranges_rad=model.jnt_range[ids].tolist(),
                home_initial_limit_violations=violations)


class Energy:
    def __init__(self, model):
        self.telemetry = WbrMotorTelemetry(model)
        self.rows = []
        self.dt = float(model.opt.timestep)

    def sample(self, data):
        # 场景加载后可能另设步长，须以开始采样时的实际物理步长计时。
        dt = float(self.telemetry.model.opt.timestep)
        if self.rows and not math.isclose(dt, self.dt, rel_tol=0., abs_tol=1e-12):
            raise RuntimeError("能耗采样不支持运行中改变物理步长")
        self.dt = dt
        mujoco.mj_forward(self.telemetry.model, data)
        value = self.telemetry.snapshot(data)
        row = dict(time_s=float(data.time),
                   wheel_power_w=value["electrical_power_estimate_w"])
        for hip in value["hips"]:
            name = hip["name"].removesuffix("_actuator")
            row[name + "_torque_nm"] = hip["torque_nm"]
            row[name + "_speed_radps"] = hip["speed_radps"]
        self.rows.append(row)

    def summary(self, window):
        duration = len(self.rows) * self.dt
        axes = {}
        for name in HIPS:
            torque = np.array([row[name + "_torque_nm"] for row in self.rows])
            speed = np.array([row[name + "_speed_radps"] for row in self.rows])
            integral = float(np.sum(torque**2) * self.dt)
            power = torque * speed
            above = np.abs(torque) > 10.0001
            longest = current = 0
            for flag in above:
                current = current + 1 if flag else 0
                longest = max(longest, current)
            axes[name] = dict(rms_nm=math.sqrt(integral / duration) if duration else None,
                peak_nm=float(np.max(np.abs(torque))) if len(torque) else None,
                torque_squared_integral_nm2s=integral,
                positive_mechanical_work_j=float(np.sum(np.maximum(power, 0)) * self.dt),
                negative_mechanical_work_j=float(np.sum(np.minimum(power, 0)) * self.dt),
                peak_speed_radps=float(np.max(np.abs(speed))) if len(speed) else None,
                time_above_10nm_s=float(above.sum() * self.dt),
                longest_above_10nm_s=longest * self.dt)
        powers = np.array([row["wheel_power_w"] for row in self.rows])
        return dict(requested_window_s=window, recorded_window_s=duration,
            full_window=duration >= window - .0011, hips=axes,
            wheel_estimated_positive_electrical_wh=float(np.maximum(powers, 0).sum() * self.dt / 3600),
            wheel_estimated_signed_electrical_wh=float(powers.sum() * self.dt / 3600),
            wheel_estimate=self.telemetry.metadata(),
            meaning="统一窗包含初始化后的稳站、动作与等待；起身/上台另含起身，独立跳跃从直立姿态开始，不是净单次跳跃能耗；髋积分仅电机主动输出；轮电能为拟合估计，负功率不认作电池回收")

    def write(self, path):
        if self.rows:
            with Path(path).open("w", encoding="utf-8", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(self.rows[0]))
                writer.writeheader()
                writer.writerows(self.rows)


def prepare_source(output):
    formal = HERE / "OwnWbrBridge.c"
    if digest(formal) != EXPECTED:
        raise RuntimeError("正式控制器哈希已变，需重新核对")
    original = formal.read_bytes()
    old = b"state->gas_spring_force < 449.0f ||\n                          state->gas_spring_force > 451.0f ||"
    newline = b"\r\n" if b"\r\n" in original else b"\n"
    old = old.replace(b"\n", newline)
    new = old.replace(b"449.0f", b"0.0f").replace(b"451.0f", b"1000.0f")
    if original.count(old) != 1:
        raise RuntimeError("450N守卫结构不符，禁止宽泛替换")
    target = output / "OwnWbrBridgeGasDiagnostic.c"
    target.write_bytes(original.replace(old, new))
    # 引用同目录动作阶段头文件，内容保持逐字一致。
    (output / "OwnWbrStages.h").write_bytes((HERE / "OwnWbrStages.h").read_bytes())
    diff = "".join(difflib.unified_diff(original.decode("utf-8").splitlines(True),
        target.read_text(encoding="utf-8").splitlines(True),
        fromfile=str(formal), tofile=str(target)))
    (output / "诊断控制器差异.patch").write_text(diff, encoding="utf-8")
    save(output / "诊断控制器来源.json", dict(formal_sha256=digest(formal),
        diagnostic_sha256=digest(target),
        stages_sha256=digest(HERE / "OwnWbrStages.h"),
        scope="仅把跳跃档气弹簧参数准入449..451N放宽至0..1000N；10/45力矩、速度、时序与动作预算保持原文"))
    return target


def configure(lib):
    pointer = ctypes.POINTER(ctypes.c_float)
    lib.OwnWbrCommandStep.argtypes = [ctypes.c_void_p, pointer, pointer, pointer]
    lib.OwnWbrCommandStep.restype = ctypes.c_int
    for name, tail in (("OwnWbrSetGasSpring", [ctypes.c_float]),
        ("OwnWbrSetJointLimits", [ctypes.c_float, ctypes.c_float]),
        ("OwnWbrSetJumpProfile", [ctypes.c_int]), ("OwnWbrSetStage", [ctypes.c_int]),
        ("OwnWbrSetClimbFold", [ctypes.c_int]), ("OwnWbrEnableRise", [ctypes.c_int])):
        fn = getattr(lib, name)
        fn.argtypes, fn.restype = [ctypes.c_void_p, *tail], ctypes.c_int


def contact_metrics(model, data):
    wheel_load = [0., 0.]
    wheels = [model.body(s + "wlink").id for s in ("l", "r")]
    force = np.zeros(6)
    for index in range(data.ncon):
        contact = data.contact[index]
        bodies = [int(model.geom_bodyid[g]) for g in (contact.geom1, contact.geom2)]
        for side, wheel in enumerate(wheels):
            if wheel in bodies:
                other = bodies[1] if bodies[0] == wheel else bodies[0]
                if model.body_rootid[other] != model.body_rootid[wheel]:
                    mujoco.mj_contactForce(model, data, index, force)
                    wheel_load[side] += abs(float(force[0] * np.asarray(contact.frame).reshape(3, 3)[0, 2]))
    short, others, penetration = [], [], []
    ids = [model.joint(name).id for name in KNEES]
    for side, jid in enumerate(ids):
        q = float(data.qpos[int(model.jnt_qposadr[jid])])
        limit = float(model.jnt_range[jid, 1 if side == 0 else 0])
        penetration.append(max(0., (1 if side == 0 else -1) * (q - limit)))
    for index in np.flatnonzero(data.efc_type == mujoco.mjtConstraint.mjCNSTR_LIMIT_JOINT):
        jid = int(data.efc_id[index])
        q = float(data.qpos[int(model.jnt_qposadr[jid])])
        if jid in ids and abs(q - model.jnt_range[jid, 1 if ids.index(jid) == 0 else 0]) < .02:
            short.append(model.joint(jid).name)
        else:
            others.append(model.joint(jid).name)
    return dict(left_wheel_normal_n=wheel_load[0], right_wheel_normal_n=wheel_load[1],
                short_stop_constraints=short, other_joint_limit_constraints=others,
                stop_penetration_rad=penetration)


def run_flat(task, case, force, action):
    model = load_model(gas_force_n=force, correct_link_collisions=True,
        terrain="flat", guide_rollers="passive", noslip_iterations=10, knee_range="cad120")
    model.opt.timestep = .001
    stop = apply_stop(model)
    if stop["home_initial_limit_violations"]:
        return dict(passed=False, failures=["home_outside_180mm_stop"], physics=stop)
    for name in HIPS:
        aid = model.actuator(name + "_actuator").id
        jid = model.joint(name).id
        model.actuator_ctrlrange[aid] = model.actuator_forcerange[aid] = [-45., 45.]
        model.jnt_actfrcrange[jid] = [-45., 45.]
    lib, build = climb.load_core(case, Path(task["source"]))
    configure(lib)
    profile = json.loads((HERE / "OwnWbrProfile.json").read_text(encoding="utf-8"))
    config = (ctypes.c_float * 20)(*np.asarray(profile["negative_k"]).flatten(),
        profile["theta0"], profile["tp0"], profile["wheel0"], profile["support_n"],
        profile["length_m"], profile["radius_m"], profile["length_kp"], profile["length_kd"])
    handle = lib.OwnWbrCreate(config)
    if not handle:
        raise RuntimeError("控制器初始化失败")
    jump_profile = 3 if action == "jump250" else 2
    for name, args in (("OwnWbrSetClimbFold", (0,)), ("OwnWbrSetGasSpring", (force,)),
        ("OwnWbrSetJointLimits", (10., 45.)), ("OwnWbrSetJumpProfile", (jump_profile,)),
        ("OwnWbrSetStage", (3,)), ("OwnWbrEnableRise", (int(action == "rise"),))):
        if getattr(lib, name)(handle, *args) != 1:
            raise RuntimeError(name + "配置失败")
    data = mujoco.MjData(model)
    if action == "rise":
        home_path = Path(task["home"])
        home = json.loads(home_path.read_text(encoding="utf-8"))
        data.qpos[:len(home["qpos"])] = home["qpos"]
        stop.update(initial_pose_source=str(home_path), initial_pose_sha256=digest(home_path),
                    initial_pose_meaning="独立闭链有效低趴姿态，保留原home机身姿态")
    else:
        poses_path = ROOT / "local/cache/wbr-user-record-20261004/spring-static-support.json"
        poses = json.loads(poses_path.read_text(encoding="utf-8"))["rows"]
        pose = next(row for row in poses if abs(row["length_m"] - .20) < 1e-8)
        data.qpos[:len(pose["qpos"])] = pose["qpos"]
        stop.update(initial_pose_source=str(poses_path), initial_pose_sha256=digest(poses_path),
                    initial_pose_meaning="独立200mm直立闭链姿态，仅用于跳跃；不作为起身初态")
    mujoco.mj_forward(model, data)
    stop["home_initial_max_contact_penetration_m"] = max(
        (-float(data.contact[i].dist) for i in range(data.ncon)), default=0.)
    stop["home_initial_closure_m"] = float(inspect_state(model, data, np.zeros(40), np.zeros(7))["closure_m"])
    if stop["home_initial_max_contact_penetration_m"] > .005 or stop["home_initial_closure_m"] > .001:
        lib.OwnWbrDestroy(handle)
        return dict(passed=False, failures=["home_initial_penetration_or_closure_invalid"], physics=stop)
    names = ["imu_quat", "imu_gyro", "imu_accel"] + [f"{m}_{kind}" for m in MOTORS for kind in ("pos", "vel", "tor")]
    indices = np.concatenate([np.arange(model.sensor(n).adr[0], model.sensor(n).adr[0] + model.sensor(n).dim[0]) for n in names])
    sensor, command, output = (ctypes.c_float * 28)(), (ctypes.c_float * 3)(), (ctypes.c_float * 40)()
    energy, rows, failures = Energy(model), [], []
    ready = None if action == "rise" else 0.
    air_start = None
    longest_air = clearance = 0.
    try:
        for tick in range(round(task["seconds"] / .001)):
            t = float(data.time)
            jump = action != "rise" and ready is not None and ready + 3 <= t < ready + 3.15
            if tick % 3 == 0:
                sensor[:] = data.sensordata[indices]
                command[:] = (0., 0., float(jump))
                error = lib.OwnWbrCommandStep(handle, sensor, command, output)
                if error or int(output[34]) < 0:
                    failures.append(f"control_or_rise_failed_{error}_{int(output[34])}")
                    break
                data.ctrl[:] = output[:6]
            if ready is None and int(output[34]) == 0 and output[35] > 0:
                ready = t
            row = inspect_state(model, data, output, np.zeros(7))
            row.update(time_s=t, rise_phase=int(output[34]), jump_phase=int(output[29]), jump_count=int(output[30]), joint_limit_nm=float(output[31]))
            row.update(contact_metrics(model, data))
            rows.append(row)
            airborne = int(output[30]) > 0 and row["wheel_contacts"] == 0
            if airborne:
                air_start = t if air_start is None else air_start
                longest_air = max(longest_air, t - air_start + .001)
                clearance = max(clearance, min(float(data.body(s + "wlink").xpos[2]) for s in ("l", "r")) - profile["radius_m"])
            else:
                air_start = None
            energy.sample(data)
            mujoco.mj_step(model, data)
            mujoco.mj_forward(model, data)
            if not np.isfinite(data.qpos).all() or any(w.number for w in data.warning):
                failures.append("mujoco_warning_or_nonfinite")
                break
    finally:
        lib.OwnWbrDestroy(handle)
    if ready is None or ready > task["max_rise_seconds"]:
        failures.append("rise_timeout")
    final = [r for r in rows if r["time_s"] >= task["seconds"] - 2]
    if not final or len(final) < 1999 or max(max(abs(r["pitch_deg"]), abs(r["roll_deg"])) for r in final) > 5 or math.sqrt(sum(r["true_v_mps"]**2 for r in final) / len(final)) > .1 or any(min(r["left_wheel_normal_n"], r["right_wheel_normal_n"]) <= 5 for r in final) or max(r["y_m"] for r in final) - min(r["y_m"] for r in final) > .02:
        failures.append("final_2s_not_stable")
    after = [r for r in rows if ready is not None and r["time_s"] >= ready]
    if any(r["other_floor_contacts"] > 0 for r in after):
        failures.append("nonwheel_contact_after_rise")
    if any(r["closure_m"] > .001 for r in after):
        failures.append("closure_over_1mm")
    if any(r["other_joint_limit_constraints"] for r in rows):
        failures.append("other_joint_limit_anytime")
    if any(max(r["stop_penetration_rad"]) > .01 for r in rows):
        failures.append("short_stop_penetration_over_0.01rad")
    for row in rows:
        if any(abs(row[name + "_requested_nm"]) > min(45., row["joint_limit_nm"]) + .0001 for name in HIPS):
            failures.append("joint_torque_limit_exceeded")
            break
    target = [.24, .27] if action == "jump250" else [.09, .11]
    if action != "rise" and not target[0] <= clearance <= target[1]:
        failures.append("jump_height_outside_target_band")
    if action != "rise" and (longest_air < .12 or int(output[30]) != 1 or int(output[29]) != 0):
        failures.append("jump_incomplete_or_airtime_below_120ms")
    stats = energy.summary(task["seconds"])
    if any(axis["peak_nm"] is not None and axis["peak_nm"] > 45.0001 for axis in stats["hips"].values()):
        failures.append("actual_torque_over_45nm")
    if any(axis["longest_above_10nm_s"] > 3.505 for axis in stats["hips"].values()):
        failures.append("continuous_peak_duration_over_3.5s")
    if not stats["full_window"]:
        failures.append("measurement_window_incomplete")
    energy.write(case / "motor-trace.csv")
    event_energy = None
    event_start = next((r["time_s"] for r in rows if r["jump_phase"] != 0), None)
    event_end = None
    if event_start is not None:
        stable_start = None
        for row in rows:
            if row["time_s"] <= event_start:
                continue
            stable = row["jump_phase"] == 0 and max(abs(row["pitch_deg"]), abs(row["roll_deg"])) < 5 and abs(row["true_v_mps"]) < .1 and min(row["left_wheel_normal_n"], row["right_wheel_normal_n"]) > 5
            stable_start = row["time_s"] if stable and stable_start is None else stable_start if stable else None
            if stable_start is not None and row["time_s"] - stable_start >= 2:
                event_end = row["time_s"]
                break
        if event_end is not None:
            all_samples = energy.rows
            energy.rows = [r for r in all_samples if event_start <= r["time_s"] < event_end]
            event_energy = energy.summary(event_end - event_start)
            event_energy.update(start_s=event_start, end_s=event_end,
                                meaning="跳跃状态首次启动至动作结束且双轮受载连续回稳2秒的事件窗")
            energy.rows = all_samples
    if rows:
        with (case / "trace.csv").open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
    rise_rows = [r for r in rows if ready is None or r["time_s"] <= ready]
    return dict(passed=not failures, failures=failures, physics=stop, energy=stats,
        action_completed_stable_budget_passed=not [f for f in failures if f != "jump_height_outside_target_band"],
        height_target_band_passed=target[0] <= clearance <= target[1] if action != "rise" else None,
        rise_executed=action == "rise", rise_completed_s=ready if action == "rise" else None,
        rise_max_displacement_m=max((math.hypot(r["x_m"]-rows[0]["x_m"], r["y_m"]-rows[0]["y_m"]) for r in rise_rows), default=None) if action == "rise" else None,
        jump_profile=jump_profile, jump_clearance_m=clearance, jump_airborne_s=longest_air,
        jump_event_energy=event_energy,
        jump_required_clearance_band_m=target if action != "rise" else None,
        support_feedforward_n_each=profile["support_n"],
        support_mode="普通支撑前馈，不做实体挡块承重时的运行中自动卸载",
        stop_penetration_max_rad=max((max(r["stop_penetration_rad"]) for r in rows), default=0.),
        stop_contact_samples=sum(bool(r["short_stop_constraints"]) for r in rows),
        final_roll_peak_deg=max((abs(r["roll_deg"]) for r in final), default=None),
        final_wheel_load_min_n=[min((r[key] for r in final), default=None) for key in ("left_wheel_normal_n", "right_wheel_normal_n")],
        build_command=build)


def run_climb(task, case, force):
    path = HERE / "TestWbrImpactClimb.py"
    source = path.read_text(encoding="utf-8")
    for old, new in (("OwnWbrSetGasSpring(handle, 450.0)", f"OwnWbrSetGasSpring(handle, {force!r})"),
        ("gas_force_n=450.0", f"gas_force_n={force!r}"),
        ('"gas_spring_n_each": 450.0', f'"gas_spring_n_each": {force!r}'),
        ('int(args.knee_range == "cad120")', '0')):
        if source.count(old) != 1:
            raise RuntimeError("动态入口气弹簧配置结构已改变")
        source = source.replace(old, new)
    suite = types.ModuleType("gas_action_climb")
    suite.__file__ = str(path)
    exec(compile(source, str(path), "exec"), suite.__dict__)
    state = {}
    def stopped_model(*args, **kwargs):
        model = load_model(*args, **kwargs)
        state["stop"] = apply_stop(model)
        if state["stop"]["home_initial_limit_violations"]:
            raise ValueError("home_outside_180mm_stop")
        home_path = Path(task["home"])
        home = json.loads(home_path.read_text(encoding="utf-8"))
        model.key("home").qpos[:len(home["qpos"])] = home["qpos"]
        state["stop"].update(initial_pose_source=str(home_path), initial_pose_sha256=digest(home_path))
        check = mujoco.MjData(model)
        mujoco.mj_resetDataKeyframe(model, check, model.key("home").id)
        mujoco.mj_forward(model, check)
        initial_penetration = max((-float(check.contact[i].dist) for i in range(check.ncon)), default=0.)
        initial_closure = float(inspect_state(model, check, np.zeros(40), np.zeros(7))["closure_m"])
        state["stop"].update(home_initial_max_contact_penetration_m=initial_penetration,
                            home_initial_closure_m=initial_closure)
        if initial_penetration > .005 or initial_closure > .001:
            raise ValueError("home_initial_penetration_or_closure_invalid")
        state["energy"] = Energy(model)
        return model
    class Physics:
        def __getattr__(self, name):
            return getattr(mujoco, name)
        def mj_step(self, model, data, *args, **kwargs):
            state["energy"].sample(data)
            return mujoco.mj_step(model, data, *args, **kwargs)
    suite.load_model, suite.mujoco = stopped_model, Physics()
    original_joint_limits = suite.joint_limit_names
    state["legacy_any_joint_limit_ticks"] = 0
    state["other_joint_limit_ticks"] = 0
    state["short_stop_constraint_ticks"] = 0
    state["stop_penetration_max_rad"] = 0.
    def classify_limits(model, data):
        original = original_joint_limits(model, data)
        metrics = contact_metrics(model, data)
        state["legacy_any_joint_limit_ticks"] += bool(original)
        state["other_joint_limit_ticks"] += bool(metrics["other_joint_limit_constraints"])
        state["short_stop_constraint_ticks"] += bool(metrics["short_stop_constraints"])
        state["stop_penetration_max_rad"] = max(state["stop_penetration_max_rad"], max(metrics["stop_penetration_rad"]))
        return metrics["other_joint_limit_constraints"]
    suite.joint_limit_names = classify_limits
    sys.argv = [str(path), "--source", task["source"], "--output", str(case),
        "--platform", "200", "--speed", "1.35", "--guide-rollers", "passive",
        "--knee-range", "cad120", "--climb-assist", "--seconds", str(task["seconds"])]
    try:
        suite.main()
        report = json.loads((case / "summary.json").read_text(encoding="utf-8"))
    except ValueError as exc:
        if str(exc) not in ("home_outside_180mm_stop", "home_initial_penetration_or_closure_invalid"):
            raise
        report = dict(passed=False, failures=[str(exc)])
    report["physical_stop"] = state.get("stop")
    report["limit_classification"] = {key: state[key] for key in ("legacy_any_joint_limit_ticks", "other_joint_limit_ticks", "short_stop_constraint_ticks", "stop_penetration_max_rad")}
    report["limit_classification"]["reason"] = "180mm膝短端为实体承重挡块，其接触单列；其它关节及膝长端沿用失败判据；短端软穿透超过0.01rad失败"
    if state["stop_penetration_max_rad"] > .01:
        report["passed"] = False
    if state["other_joint_limit_ticks"]:
        report["passed"] = False
        report.setdefault("failures", []).append("other_joint_limit_anytime")
    if "energy" in state:
        report["energy"] = state["energy"].summary(task["seconds"])
        state["energy"].write(case / "motor-trace.csv")
        report["passed"] = report["passed"] and report["energy"]["full_window"]
    report["passed"] = report["passed"] and report.get("climb_completed", False) and report.get("motor_budget_ok", False)
    report["climb_fold_mode"] = 0
    report["adapted_runner_sha256"] = hashlib.sha256(source.encode("utf-8")).hexdigest().upper()
    return report


def run_case(task):
    case = Path(task["output"]) / f'{task["action"]}-F{task["force"]:g}'
    case.mkdir(parents=True, exist_ok=True)
    try:
        report = run_climb(task, case, task["force"]) if task["action"] == "climb200" else run_flat(task, case, task["force"], task["action"])
    except Exception as exc:
        report = dict(passed=False, failures=[type(exc).__name__ + ": " + str(exc)])
    report.update(action=task["action"], gas_force_n_each=task["force"],
                  diagnostic_sha256=digest(task["source"]), source_formal_sha256=EXPECTED,
                  eligible_for_ranking=bool(report["passed"]))
    report["action_runner_sha256"] = task["runner_sha256"]
    report["action_runner_changed_during_run"] = digest(__file__) != task["runner_sha256"]
    if report["action_runner_changed_during_run"]:
        report["passed"] = report["eligible_for_ranking"] = False
        report.setdefault("failures", []).append("action_runner_changed_during_run")
    save(case / "action-summary.json", report)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--forces", type=float, nargs="+", default=[300., 350., 400., 450.])
    parser.add_argument("--actions", nargs="+", choices=("rise", "jump100", "jump250", "climb200"), default=["rise", "jump100", "jump250", "climb200"])
    parser.add_argument("--seconds", type=float, default=22.)
    parser.add_argument("--max-rise-seconds", type=float, default=5.)
    parser.add_argument("--workers", type=int, default=1)
    parser.add_argument("--home", type=Path, help="独立解得的闭链有效低趴JSON，含qpos数组")
    args = parser.parse_args()
    if args.workers < 1 or args.seconds < 10 or not math.isfinite(args.seconds) or any(not math.isfinite(f) or not 0 <= f <= 1000 for f in args.forces):
        parser.error("参数超出有效范围")
    args.output.mkdir(parents=True, exist_ok=True)
    if any(action in ("rise", "climb200") for action in args.actions) and args.home is None:
        parser.error("起身/上台必须提供有效低趴 --home；禁止复用原无效home")
    source = prepare_source(args.output)
    tasks = [dict(output=str(args.output.resolve()), source=str(source.resolve()), force=force,
        action=action, seconds=args.seconds, max_rise_seconds=args.max_rise_seconds,
        home=str(args.home.resolve()) if args.home else None, runner_sha256=digest(__file__))
        for force in args.forces for action in args.actions]
    results = []
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        for future in as_completed([pool.submit(run_case, task) for task in tasks]):
            report = future.result()
            results.append(report)
            print(json.dumps(dict(action=report["action"], force=report["gas_force_n_each"],
                passed=report["passed"], failures=report.get("failures")), ensure_ascii=False), flush=True)
    save(args.output / "summary.json", dict(rows=results, formal_sha256=digest(HERE / "OwnWbrBridge.c"),
        action_window_s=args.seconds, max_rise_seconds=args.max_rise_seconds,
        source_boundary="诊断参数准入放宽；失败或缺少完整窗不可排序为优选"))
    if digest(HERE / "OwnWbrBridge.c") != EXPECTED:
        raise RuntimeError("正式控制器在测试期间发生变化")


if __name__ == "__main__":
    main()
