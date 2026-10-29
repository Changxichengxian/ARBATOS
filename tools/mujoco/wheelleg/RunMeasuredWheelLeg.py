"""实测小轮腿的独立模型、控制设计与离线验证；不修改车型固件增益。"""

from __future__ import annotations

import argparse
import copy
import ctypes
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import scipy
from scipy.linalg import expm, solve_discrete_are

import RunWheelLeg as legacy

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
PROFILE = HERE / "小轮腿实测参数.json"
SIDES = ("right", "left")
MODES = {"vmc": "VMC · 固定高度", "bench": "板凳 · 固定高度",
         "bench-height": "板凳 · 可变高度", "vmc-height": "VMC · 可变高度"}
TERRAINS = {"flat": "平地", "left-ramp": "左轮上坡 · 12 mm", "right-ramp": "右轮上坡 · 12 mm",
            "cross": "横坡 · 4°", "uphill": "上坡驻车 · 6°", "downhill": "下坡驻车 · 6°"}
Float5 = ctypes.c_float * 5
Float6 = ctypes.c_float * 6
Float8 = ctypes.c_float * 8
Float12 = ctypes.c_float * 12
Float2 = ctypes.c_float * 2


def write_json(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def load_profile(path=PROFILE):
    profile = json.loads(Path(path).read_text(encoding="utf-8"))
    m, a, c = profile["measured"], profile["assumed"], profile["control"]
    for value in (m["lower_link_m"], m["hip_spacing_m"], m["core_assembly_mass_kg"],
                  a["upper_link_m"], a["wheel_radius_m"], a["track_m"],
                  a["leg_links_mass_total_kg"], a["wheel_ends_mass_total_kg"]):
        if not math.isfinite(value) or value <= 0:
            raise ValueError("尺寸和质量必须为有限正数")
    if m["core_assembly_mass_kg"] <= 4 * a["output_rotating_mass_each_kg"]:
        raise ValueError("输出旋转件质量超过核心组件称重")
    ratio = c["control_dt_s"] / c["physics_dt_s"]
    if ratio < 1 or not math.isclose(ratio, round(ratio)):
        raise ValueError("控制周期必须是物理步长的正整数倍")
    if not c["leg_range_m"][0] <= c["leg_length_m"] <= c["leg_range_m"][1]:
        raise ValueError("默认腿长不在测试范围内")
    return profile


class Core:
    def __init__(self, profile, cache):
        cache = Path(cache).resolve()
        cache.mkdir(parents=True, exist_ok=True)
        source = HERE / "MeasuredWheelLegBridge.c"
        dependencies = [source] + legacy.source_dependencies()[1:]
        digest = hashlib.sha256(b"".join(p.read_bytes() for p in dependencies)).hexdigest()[:16]
        target = cache / ("measured_core_" + digest + legacy.lib_suffix())
        if not target.exists():
            compiler = legacy.find_compiler()
            if compiler is None:
                zig = Path("C:/Users/28111/AppData/Local/Microsoft/WinGet/Links/zig.exe")
                if zig.exists():
                    compiler = str(zig.resolve()), ["zig"]
            if compiler is None:
                raise RuntimeError("未找到本机 C 编译器")
            exe, tags = compiler
            includes = [ROOT / "shared/application/wheelleg", ROOT / "shared/application/robot",
                        ROOT / "shared/components/support"]
            if "msvc" in tags:
                command = [exe, "/nologo", "/LD", "/O2"] + [f"/I{p}" for p in includes]
                command += [str(source), f"/Fo{cache / 'MeasuredWheelLegBridge.obj'}", f"/Fe{target}"]
            else:
                command = [exe] + (["cc"] if "zig" in tags else [])
                command += ["-shared", "-O2", "-std=c99", "-Wall", "-Wextra", "-Werror"]
                command += [f"-I{p}" for p in includes] + [str(source), "-o", str(target)]
                if os.name != "nt":
                    command.append("-lm")
            env = os.environ.copy()
            env["ZIG_GLOBAL_CACHE_DIR"] = str(cache / "zig-global")
            env["ZIG_LOCAL_CACHE_DIR"] = str(cache / "zig-local")
            if "zig" in tags:
                env["ZIG_LIB_DIR"] = str(Path(exe).resolve().parent / "lib")
            subprocess.run(command, check=True, cwd=cache, env=env)
        self.lib = ctypes.CDLL(str(target))
        self.lib.WheelLegSimLeg.argtypes = [ctypes.POINTER(ctypes.c_float), ctypes.c_float,
                                          ctypes.c_float, ctypes.POINTER(ctypes.c_float)]
        self.lib.WheelLegSimLeg.restype = ctypes.c_int
        self.lib.WheelLegSimBalance.argtypes = [ctypes.POINTER(ctypes.c_float)] * 3
        self.lib.WheelLegSimBalance.restype = None
        m, a = profile["measured"], profile["assumed"]
        self.geo = Float5(a["upper_link_m"], m["lower_link_m"], m["lower_link_m"],
                          a["upper_link_m"], m["hip_spacing_m"])

    def leg(self, front, back):
        out = Float8()
        if not self.lib.WheelLegSimLeg(self.geo, front, back, out):
            raise ValueError("五连杆运动学无效或接近奇异")
        return np.array(out, dtype=float)

    def balance(self, gain, state):
        out = Float2()
        self.lib.WheelLegSimBalance(Float12(*np.asarray(gain).ravel()), Float6(*state), out)
        return np.array(out, dtype=float)


def make_config(profile):
    cfg = legacy.BridgeConfig()
    m, a = profile["measured"], profile["assumed"]
    cfg.geometry = legacy.Geometry(a["upper_link_m"], m["lower_link_m"], m["lower_link_m"],
                                   a["upper_link_m"], m["hip_spacing_m"])
    cfg.wheel_radius_m = a["wheel_radius_m"]
    return cfg


def model_xml(profile, terrain="flat"):
    m, a, c = profile["measured"], profile["assumed"], profile["control"]
    cfg = make_config(profile)
    tree = ET.fromstring(legacy.build_fivebar_model_xml(cfg, c["leg_length_m"] + a["wheel_radius_m"]))
    tree.set("model", "MINIWHEELEG_measured_20261023")
    tree.find("option").attrib.update(timestep=str(c["physics_dt_s"]), integrator="implicitfast",
                                      cone="elliptic", iterations="80", tolerance="1e-10")
    ET.SubElement(tree, "visual")
    ET.SubElement(tree.find("visual"), "global", offwidth="1280", offheight="800")
    tree.find("default/joint").attrib.update(armature="0", damping=str(a["joint_damping_nm_s"]))
    tree.find("default/geom").attrib.update(condim="3", friction=f"{a['floor_friction']} 0.005 0.0001",
                                           solref="0.004 1", contype="1", conaffinity="2")
    base = tree.find("worldbody/body")
    for child in list(base.findall("geom")):
        base.remove(child)
    size = a["body_size_m"]
    fixed_mass = m["core_assembly_mass_kg"] - 4 * a["output_rotating_mass_each_kg"]
    ET.SubElement(base, "geom", name="body", type="box", pos=f"0 0 {a['body_com_z_m']}",
                  size=" ".join(str(v / 2) for v in size), mass=str(fixed_mass), material="body_mat")
    tree.find("worldbody/geom").attrib.update(contype="2", conaffinity="1")
    for index, side in enumerate(SIDES):
        y = (-1 if index == 0 else 1) * a["track_m"] / 2
        carrier = base.find(f"body[@name='{side}_carrier']")
        wheel = carrier.find("body")
        base.remove(carrier)
        front_rod = base.find(f"body[@name='{side}_front_crank']/body")
        wheel.set("pos", f"0 0 {-m['lower_link_m']}")
        front_rod.append(wheel)
        wheel.find("joint").attrib.update(damping=str(a["wheel_damping_nm_s"]), armature="0")
        wheel.find("geom").attrib.update(size=f"{a['wheel_radius_m']} {a['wheel_width_m'] / 2}",
                                          mass=str(a["wheel_ends_mass_total_kg"] * 0.4375),
                                          friction=f"{a['floor_friction']} 0.005 0.0001")
        ET.SubElement(front_rod, "geom", name=f"{side}_hub", type="sphere", size="0.009",
                      pos=f"0 0 {-m['lower_link_m']}", mass=str(a["wheel_ends_mass_total_kg"] * 0.0625),
                      rgba="0.7 0.7 0.75 1")
        for end in ("front", "back"):
            crank = base.find(f"body[@name='{side}_{end}_crank']")
            crank.set("pos", f"{(-1 if end == 'front' else 1) * m['hip_spacing_m'] / 2} {y} 0")
            crank.find("joint").attrib.update(armature=str(a["hip_armature_kg_m2"]), range="-2.8 2.8",
                                               damping=str(a["joint_damping_nm_s"]))
            pivot = crank.find(f"geom[@name='{side}_{end}_pivot_geom']")
            pivot.attrib.update(type="cylinder", size="0.025 0.008", euler="1.5707963268 0 0",
                                mass=str(a["output_rotating_mass_each_kg"]), contype="1", conaffinity="2")
            crank.find(f"geom[@name='{side}_{end}_crank_geom']").attrib.update(
                mass=str(a["leg_links_mass_total_kg"] * 0.1), contype="1", conaffinity="2")
            rod = crank.find("body")
            rod.find("joint").attrib.update(range="-3.1 3.1", damping=str(a["joint_damping_nm_s"]))
            rod.find("geom").attrib.update(mass=str(a["leg_links_mass_total_kg"] * 0.15),
                                            contype="1", conaffinity="2")
    equality = tree.find("equality")
    equality.clear()
    for side in SIDES:
        ET.SubElement(equality, "connect", name=f"{side}_loop", site1=f"{side}_front_rod_end",
                      site2=f"{side}_back_rod_end", solref="0.003 1", solimp="0.99 0.999 0.001")
    for actuator in tree.find("actuator"):
        limit = c["wheel_torque_limit_nm"] if "wheel" in actuator.get("name") else c["joint_torque_limit_nm"]
        actuator.set("ctrlrange", f"{-limit} {limit}")
        if 'wheel' in actuator.get('name') and a.get('motor_torque_lag_s', 0) > 0:
            # 响应时间是待辨识假设；不能把 CAN 指令当作瞬时真实力矩。
            actuator.tag = 'general'
            actuator.set('dyntype', 'filterexact')
            actuator.set('dynprm', str(a['motor_torque_lag_s']))
    if terrain not in TERRAINS:
        raise ValueError("未知地形")
    ground = tree.find("worldbody/geom")
    if terrain in ("cross", "uphill", "downhill"):
        ground.set("euler", {"cross": "0.06981317 0 0", "uphill": "0 -0.10471976 0",
                             "downhill": "0 0.10471976 0"}[terrain])
    if terrain in ("left-ramp", "right-ramp"):
        y0, y1 = (.02, .30) if terrain == "left-ramp" else (-.30, -.02)
        for index, (x0, x1, h0, h1) in enumerate(((.15, .55, 0., .012),
                                                 (.55, 1.05, .012, .012),
                                                 (1.05, 1.45, .012, 0.))):
            name = f"ramp_{index}"
            vertices = [(x, y, z) for x, h in ((x0, h0), (x1, h1))
                        for y in (y0, y1) for z in (-.02, h)]
            ET.SubElement(tree.find("asset"), "mesh", name=name,
                          vertex=" ".join(str(v) for point in vertices for v in point))
            ET.SubElement(tree.find("worldbody"), "geom", name=name, type="mesh", mesh=name,
                          contype="2", conaffinity="1", rgba="0.32 0.47 0.53 1")
    return ET.tostring(tree, encoding="unicode")


def pose(model, cfg, length, z):
    # z=[世界腿角theta, 无滑滚动轮程s, 机身俯仰pitch]；这里只用于离线设计。
    theta, distance, pitch = z
    data = mujoco.MjData(model)
    radius = float(cfg.wheel_radius_m)
    data.qpos[:7] = [distance - length * math.sin(theta), 0, radius + length * math.cos(theta),
                    math.cos(pitch / 2), 0, math.sin(pitch / 2), 0]
    alpha = theta + pitch
    front, back = legacy.inverse_fivebar_point(cfg, length * math.sin(alpha), length * math.cos(alpha))
    legacy.initialize_model_pose(mujoco, model, data, cfg, front, back)
    for side in SIDES:
        knee = float(data.joint(side + "_front_knee").qpos[0])
        data.joint(side + "_wheel_hinge").qpos[0] = distance / radius - pitch - front - knee
    mujoco.mj_forward(model, data)
    return data.qpos.copy()


def tangent(model, cfg, length, z):
    basis = np.empty((model.nv, 3))
    eps = 1e-5
    for index in range(3):
        shift = np.zeros(3)
        shift[index] = eps
        column = np.empty(model.nv)
        mujoco.mj_differentiatePos(model, column, 2 * eps,
                                  pose(model, cfg, length, z - shift), pose(model, cfg, length, z + shift))
        basis[:, index] = column
    return basis


def generalized_inputs(model, data, core):
    force = np.zeros((model.nv, 2))
    for side in SIDES:
        q = [float(data.joint(f"{side}_{end}_hinge").qpos[0]) for end in ("front", "back")]
        leg = core.leg(*q)
        for index, end in enumerate(("front", "back")):
            force[int(model.joint(f"{side}_{end}_hinge").dofadr[0]), 1] = -leg[6 + index]
        force[int(model.joint(f"{side}_wheel_hinge").dofadr[0]), 0] = 1
    return force


def design(model, cfg, core, profile, length):
    c = profile["control"]
    zero = np.zeros(3)
    q0 = pose(model, cfg, length, zero)
    basis = tangent(model, cfg, length, zero)
    data = mujoco.MjData(model)
    data.qpos[:] = q0
    mujoco.mj_forward(model, data)
    mass = np.zeros((model.nv, model.nv))
    mujoco.mj_fullM(model, data, mass)
    reduced_mass = basis.T @ mass @ basis
    input_accel = np.linalg.solve(reduced_mass, basis.T @ generalized_inputs(model, data, core))
    forces = np.empty((3, 3))
    eps = 1e-4
    for index in range(3):
        values = []
        for sign in (-1, 1):
            z = zero.copy()
            z[index] = sign * eps
            data.qpos[:] = pose(model, cfg, length, z)
            data.qvel[:] = 0
            mujoco.mj_forward(model, data)
            values.append(tangent(model, cfg, length, z).T @ (-data.qfrc_bias))
        forces[:, index] = (values[1] - values[0]) / (2 * eps)
    forces[:, 1] = 0  # 水平平移没有重力回复力。
    damping = basis.T @ np.diag(model.dof_damping) @ basis
    a = np.zeros((6, 6))
    b = np.zeros((6, 2))
    a[[0, 2, 4], [1, 3, 5]] = 1
    a[np.ix_([1, 3, 5], [0, 2, 4])] = np.linalg.solve(reduced_mass, forces)
    a[np.ix_([1, 3, 5], [1, 3, 5])] = -np.linalg.solve(reduced_mass, damping)
    b[[1, 3, 5]] = input_accel
    augmented = np.zeros((8, 8))
    augmented[:6, :6], augmented[:6, 6:] = a, b
    discrete = expm(augmented * c["control_dt_s"])
    ad, bd = discrete[:6, :6], discrete[:6, 6:]
    q, r = np.diag(c["lqr_q_diag"]), np.diag(c["lqr_r_diag"])
    riccati = solve_discrete_are(ad, bd, q, r)
    k = np.linalg.solve(r + bd.T @ riccati @ bd, bd.T @ riccati @ ad)
    poles = np.linalg.eigvals(ad - bd @ k)
    rank = np.linalg.matrix_rank(np.hstack([np.linalg.matrix_power(a, i) @ b for i in range(6)]))
    if rank != 6 or max(abs(poles)) >= 1 or min(np.linalg.eigvalsh(reduced_mass)) <= 0:
        raise RuntimeError("模型约化或平衡控制设计失败")
    return {"length_m": length, "negative_k": (-k).tolist(), "A": a.tolist(), "B": b.tolist(),
            "reduced_mass": reduced_mass.tolist(), "controllability_rank": int(rank),
            "spectral_radius": float(max(abs(poles))), "qpos": q0.tolist(),
            "scope": "固定腿长、两腿对称、理想无滑滚动的局部设计；最终在自由三维接触模型验证"}


def design_bench(designed, profile):
    # 将有限刚度的位置环并入六状态模型；不能把真实电机抱姿当成刚性焊死。
    c = profile["control"]
    a, full_b = np.array(designed["A"]), np.array(designed["B"])
    cfg, length, eps = make_config(profile), designed["length_m"], 1e-4
    plus = np.array(legacy.inverse_fivebar_point(cfg, length * math.sin(eps), length * math.cos(eps)))
    minus = np.array(legacy.inverse_fivebar_point(cfg, -length * math.sin(eps), length * math.cos(eps)))
    slope = float(np.sum(((plus - minus) / (2 * eps)) ** 2))
    position_loop = slope * np.array([c["bench_joint_kp_nm_rad"], c["bench_joint_kd_nms_rad"], 0, 0,
                                      c["bench_joint_kp_nm_rad"], c["bench_joint_kd_nms_rad"]])
    a -= np.outer(full_b[:, 1], position_loop)
    b = full_b[:, :1]
    augmented = np.zeros((7, 7))
    augmented[:6, :6], augmented[:6, 6:] = a, b
    discrete = expm(augmented * profile["control"]["control_dt_s"])
    ad, bd = discrete[:6, :6], discrete[:6, 6:]
    q = np.diag(profile["control"]["bench_lqr_q_diag"])
    r = np.array([[profile["control"]["bench_lqr_r"]]])
    riccati = solve_discrete_are(ad, bd, q, r)
    k = np.linalg.solve(r + bd.T @ riccati @ bd, bd.T @ riccati @ ad)
    radius = float(max(abs(np.linalg.eigvals(ad - bd @ k))))
    if radius >= 1:
        raise RuntimeError("板凳轮端平衡设计失败")
    gain = np.zeros((2, 6))
    gain[0] = -k[0]
    return {"length_m": designed["length_m"], "negative_k": gain.tolist(), "spectral_radius": radius}


def design_modes(model, cfg, core, profile):
    # 启动时离线求解；控制周期内仅按实测腿长插值，不在线解 Riccati 方程。
    low, high = profile["control"]["leg_range_m"]
    lengths = sorted(set([*np.linspace(low, high, 8), profile["control"]["leg_length_m"]]))
    vmc = [design(model, cfg, core, profile, float(length)) for length in lengths]
    return {"vmc": vmc, "bench": [design_bench(item, profile) for item in vmc]}


def geometry_checks(model, cfg, core, profile):
    max_loop, max_fk, max_j = 0.0, 0.0, 0.0
    c = profile["control"]
    for length in np.linspace(*c["leg_range_m"], 8):
        for angle in (-0.08, 0, 0.08):
            data = mujoco.MjData(model)
            data.qpos[:] = pose(model, cfg, length, np.array([angle, 0, 0]))
            mujoco.mj_forward(model, data)
            for side in SIDES:
                loop = data.site(side + "_front_rod_end").xpos - data.site(side + "_back_rod_end").xpos
                max_loop = max(max_loop, float(np.linalg.norm(loop)))
                angles = np.array([data.joint(side + "_front_hinge").qpos[0], data.joint(side + "_back_hinge").qpos[0]])
                values = core.leg(*angles)
                max_fk = max(max_fk, abs(values[0] - length * math.sin(angle)), abs(values[1] - length * math.cos(angle)))
                for index in range(2):
                    plus, minus = angles.copy(), angles.copy()
                    plus[index] += .001
                    minus[index] -= .001
                    numeric = (core.leg(*plus)[2:4] - core.leg(*minus)[2:4]) / .002
                    analytic = np.array([values[4 + index], -values[6 + index]])
                    max_j = max(max_j, float(max(abs(numeric - analytic))))
    expected = (profile["measured"]["core_assembly_mass_kg"] + profile["assumed"]["leg_links_mass_total_kg"]
                + profile["assumed"]["wheel_ends_mass_total_kg"])
    actual = float(sum(model.body_mass))
    if abs(actual - expected) > 1e-8 or max_loop > 2e-6 or max_fk > 2e-6 or max_j > .002:
        raise RuntimeError(f"模型检查失败: mass={actual}/{expected} loop={max_loop} fk={max_fk} jac={max_j}")
    return {"mass_kg": actual, "expected_mass_kg": expected, "closure_error_m": max_loop,
            "core_forward_error_m": max_fk, "vmc_jacobian_error": max_j,
            "note": "几何采样通过不代表实物全行程或自碰撞通过"}


class Controller:
    def __new__(cls, model, core, profile, design_result, mode="vmc", schedule=None):
        if mode not in MODES:
            raise ValueError("未知控制模式")
        if mode.startswith("bench"):
            from BenchFirmware import Controller as FirmwareController
            return FirmwareController(model, profile, mode)
        return super().__new__(cls)

    def __init__(self, model, core, profile, design_result, mode="vmc", schedule=None):
        if mode not in MODES:
            raise ValueError("未知控制模式")
        self.model, self.core, self.profile = model, core, profile
        self.mode = mode
        self.variable_height = mode.endswith("-height")
        self.bench = mode.startswith("bench")
        self.cfg = make_config(profile)
        self.gain = np.array(design_result["negative_k"])
        self.length = design_result["length_m"]
        self.target_length = self.length
        self.height_speed = 0.0
        self.hold_joints = np.array(legacy.inverse_fivebar_point(self.cfg, 0, self.length))
        self.schedule = None
        if self.variable_height:
            if schedule is None:
                raise ValueError("变高度控制需要预先设计的增益表")
            items = schedule["bench" if self.bench else "vmc"]
            self.schedule = (np.array([item["length_m"] for item in items]),
                             np.array([item["negative_k"] for item in items]))
        elif self.bench:
            self.gain = np.array(design_bench(design_result, profile)["negative_k"])
        self.reference = 0.0
        self.yaw_reference = 0.0
        self.speed = 0.0
        self.yaw_rate = 0.0
        self.driving = False
        self.last_alpha = None
        self.last_length = None
        self.last_rod = None
        self.saturated = 0
        self.samples = 0
        self.enabled = True
        self.state = np.zeros(6)
        self.leg_lengths = np.zeros(2)
        self.leg_angles = np.zeros(2)
        self.observed_speed = 0.0

    def step(self, data, target_speed=0.0, target_yaw=0.0, target_length=None):
        # 控制输入只取理想姿态、角速度、关节位置/速度；世界位置仅供评分。
        a, c = self.profile["assumed"], self.profile["control"]
        dt = c["control_dt_s"]
        roll, pitch, yaw = legacy.quat_to_euler_wxyz(data.qpos[3:7])
        gyro = data.qvel[3:6].copy()
        if not np.all(np.isfinite(data.qpos)) or not np.all(np.isfinite(data.qvel)) or max(abs(roll), abs(pitch)) > .65:
            self.enabled = False
        if not self.enabled:
            data.ctrl[:] = 0
            return
        if self.variable_height:
            if target_length is not None:
                if not math.isfinite(target_length):
                    raise ValueError("腿长必须为有限数值")
                self.target_length = float(np.clip(target_length, *c["leg_range_m"]))
            error = self.target_length - self.length
            wanted_speed = math.copysign(min(c["height_speed_m_s"], math.sqrt(2 * c["height_accel_m_s2"] * abs(error))), error)
            self.height_speed += float(np.clip(wanted_speed - self.height_speed,
                                               -c["height_accel_m_s2"] * dt, c["height_accel_m_s2"] * dt))
            increment = self.height_speed * dt
            if abs(increment) >= abs(error) and increment * error >= 0:
                self.length, self.height_speed = self.target_length, 0.0
            else:
                self.length += increment
            bounded = float(np.clip(self.length, *c["leg_range_m"]))
            if bounded != self.length:
                self.length, self.height_speed = bounded, 0.0
        legs, alpha, lengths, rods, wheels, speeds = [], [], [], [], [], []
        for side in SIDES:
            front = data.joint(side + "_front_hinge")
            back = data.joint(side + "_back_hinge")
            wheel = data.joint(side + "_wheel_hinge")
            values = self.core.leg(float(front.qpos[0]), float(back.qpos[0]))
            legs.append(values)
            alpha.append(values[3])
            lengths.append(values[2])
            # 下连杆角由主动关节正解得到，不假设被动膝关节安装编码器。
            elbow_x = -self.profile["measured"]["hip_spacing_m"] / 2 - a["upper_link_m"] * math.sin(front.qpos[0])
            elbow_down = a["upper_link_m"] * math.cos(front.qpos[0])
            rod = math.atan2(-(values[0] - elbow_x), values[1] - elbow_down)
            index = len(rods)
            rod_rate = 0.0 if self.last_rod is None else legacy.wrap_pi(rod - self.last_rod[index]) / dt
            rods.append(rod)
            wheels.append(a["wheel_radius_m"] * (wheel.qpos[0] + rod + pitch))
            speeds.append(a["wheel_radius_m"] * (wheel.qvel[0] + rod_rate + gyro[1]))
        alpha, lengths = np.array(alpha), np.array(lengths)
        dalpha = np.zeros(2) if self.last_alpha is None else (alpha - self.last_alpha) / dt
        dlength = np.zeros(2) if self.last_length is None else (lengths - self.last_length) / dt
        self.last_alpha, self.last_length = alpha, lengths
        self.last_rod = np.array(rods)
        self.leg_lengths = lengths
        self.leg_angles = alpha
        self.observed_speed = float(np.mean(speeds))
        target_speed = float(np.clip(target_speed, -c["drive_speed_limit_m_s"], c["drive_speed_limit_m_s"]))
        target_yaw = float(np.clip(target_yaw, -c["yaw_rate_limit_rad_s"], c["yaw_rate_limit_rad_s"]))
        braking = target_speed * self.speed < 0 or abs(target_speed) < abs(self.speed)
        braking_limit = c["bench_braking_limit_m_s2"] if self.bench else c["braking_limit_m_s2"]
        delta = (braking_limit if braking else c["acceleration_limit_m_s2"]) * dt
        self.speed += np.clip(target_speed - self.speed, -delta, delta)
        if abs(target_speed) > .001:
            self.driving = True
        if self.driving:
            # 行驶和制动期间释放位置误差，避免积累的路程目标把停车变成倒车回摆。
            self.reference = float(np.mean(wheels))
            if abs(self.speed) < .001 and abs(self.observed_speed) < .025:
                self.driving = False
        self.state = np.array([np.mean(alpha) - pitch, np.mean(dalpha) - gyro[1],
                               np.mean(wheels) - self.reference, np.mean(speeds) - self.speed, pitch, gyro[1]])
        if self.schedule is not None:
            lengths_table, gains = self.schedule
            measured = float(np.clip(np.mean(lengths), lengths_table[0], lengths_table[-1]))
            index = min(len(lengths_table) - 2, max(0, np.searchsorted(lengths_table, measured) - 1))
            fraction = (measured - lengths_table[index]) / (lengths_table[index + 1] - lengths_table[index])
            self.gain = gains[index] * (1 - fraction) + gains[index + 1] * fraction
        wheel_torque, hip_torque = self.core.balance(self.gain, self.state)
        total_mass = (self.profile["measured"]["core_assembly_mass_kg"]
                      + a["leg_links_mass_total_kg"] + a["wheel_ends_mass_total_kg"])
        support = total_mass * 9.81 / 2
        if self.variable_height and not self.bench:
            # 轮端质量直接由地面支撑，腿杆只将部分重量传至髋轴。
            support = (self.profile["measured"]["core_assembly_mass_kg"] + .5 * a["leg_links_mass_total_kg"]) * 9.81 / 2
        yaw_step = c["yaw_acceleration_limit_rad_s2"] * dt
        self.yaw_rate += np.clip(target_yaw - self.yaw_rate, -yaw_step, yaw_step)
        roll_target = float(np.clip(-math.atan(self.observed_speed * self.yaw_rate / 9.81), -.18, .18))
        if self.variable_height:
            margin = max(0., min(self.length - c["leg_range_m"][0], c["leg_range_m"][1] - self.length))
            roll_limit = math.asin(min(1., 2 * margin / a["track_m"]))
            roll_target = float(np.clip(roll_target, -roll_limit, roll_limit))
        roll_moment = c["roll_kp_nm_rad"] * (roll_target - roll) - c["roll_kd_nms_rad"] * gyro[0]
        split = -c["leg_split_kp_nm_rad"] * (alpha[0] - alpha[1]) - c["leg_split_kd_nms_rad"] * (dalpha[0] - dalpha[1])
        self.yaw_reference = legacy.wrap_pi(self.yaw_reference + self.yaw_rate * dt)
        yaw_error = float(np.clip(legacy.wrap_pi(self.yaw_reference - yaw), -c["yaw_error_limit_rad"], c["yaw_error_limit_rad"]))
        self.yaw_reference = legacy.wrap_pi(yaw + yaw_error)
        yaw_kp = c["bench_yaw_kp_nm_rad"] if self.bench else c["yaw_kp_nm_rad"]
        yaw_kd = c["bench_yaw_kd_nms_rad"] if self.bench else c["yaw_kd_nms_rad"]
        turn = float(np.clip(yaw_kp * yaw_error + yaw_kd * (self.yaw_rate - gyro[2]),
                             -c["turn_torque_limit_nm"], c["turn_torque_limit_nm"]))
        requested = []
        for index, side in enumerate(SIDES):
            sign = -1 if index == 0 else 1
            desired_length = self.length + sign * a["track_m"] / 2 * math.sin(roll_target)
            if self.bench:
                # 板凳使用关节位置环，轮子单独负责俯仰平衡；不叠加 VMC 髋平衡输出。
                desired_length = float(np.clip(desired_length, *c["leg_range_m"]))
                target = np.array(legacy.inverse_fivebar_point(self.cfg, 0, desired_length)) if self.variable_height else self.hold_joints
                q = np.array([data.joint(f"{side}_{end}_hinge").qpos[0] for end in ("front", "back")])
                dq = np.array([data.joint(f"{side}_{end}_hinge").qvel[0] for end in ("front", "back")])
                # 目标姿态的静态重力补偿，避免位置环靠明显下沉才产生支撑力。
                feedforward = self.core.leg(*target)[4:6] * support
                torque = feedforward + c["bench_joint_kp_nm_rad"] * (target - q) - c["bench_joint_kd_nms_rad"] * dq
            else:
                if self.variable_height:
                    desired_length = float(np.clip(desired_length, *c["leg_range_m"]))
                force = np.clip(support + c["length_kp_n_m"] * (desired_length - lengths[index])
                                + c["length_kd_ns_m"] * (self.height_speed - dlength[index])
                                + sign * roll_moment / a["track_m"], 0, 40)
                virtual = hip_torque + (split if index == 0 else -split)
                torque = legs[index][4:6] * force - legs[index][6:8] * virtual
            requested.extend([*torque, wheel_torque - sign * turn])
        requested = np.array(requested)
        if not np.all(np.isfinite(requested)):
            self.enabled = False
            data.ctrl[:] = 0
            return
        limited = np.clip(requested, self.model.actuator_ctrlrange[:, 0], self.model.actuator_ctrlrange[:, 1])
        self.saturated += int(np.any(abs(requested - limited) > 1e-8))
        self.samples += 1
        data.ctrl[:] = limited


def run_case(model, cfg, core, profile, designed, name, output, viewer=False, duration=14):
    data = mujoco.MjData(model)
    data.qpos[:] = designed["qpos"]
    if name.startswith("tilt"):
        angle = math.radians(3) * (-1 if "minus" in name else 1)
        data.qpos[:] = pose(model, cfg, designed["length_m"], np.array([0, 0, angle]))
    mujoco.mj_forward(model, data)
    controller = Controller(model, core, profile, designed)
    c = profile["control"]
    stride = round(c["control_dt_s"] / c["physics_dt_s"])
    trace, positions, velocities, times = [], [], [], []
    max_closure = 0.0
    max_limit_violation = 0.0
    bad_contact = False
    viewer_handle = None
    command = {"speed": 0.0, "yaw": 0.0, "reset": False}
    if viewer:
        from mujoco import viewer as mjviewer
        def key(code):
            if code == 265:
                command["speed"] = min(.25, command["speed"] + .05)
            elif code == 264:
                command["speed"] = max(-.25, command["speed"] - .05)
            elif code == 263:
                command["yaw"] = .4
            elif code == 262:
                command["yaw"] = -.4
            elif code in (32, ord('X')):
                command["speed"] = command["yaw"] = 0.0
            elif code == ord('R'):
                command["reset"] = True
        viewer_handle = mjviewer.launch_passive(model, data, key_callback=key)
        viewer_handle.cam.distance = .65
        viewer_handle.cam.azimuth = 125
        viewer_handle.cam.elevation = -18
        write_json(Path(output) / "viewer_ready.json", {"pid": os.getpid(), "ready": True,
                   "controls": "上下键调速度，左右键转向，X清零指令，R重置；关闭窗口结束"})
        print("Viewer ready: Up/Down speed, Left/Right turn, X stop, R reset.", flush=True)
    start_clock = time.perf_counter()
    try:
        for step in range(round(duration / c["physics_dt_s"])):
            if viewer_handle is not None and not viewer_handle.is_running():
                break
            if command["reset"]:
                mujoco.mj_resetData(model, data)
                data.qpos[:] = designed["qpos"]
                mujoco.mj_forward(model, data)
                controller = Controller(model, core, profile, designed)
                command.update(speed=0.0, yaw=0.0, reset=False)
            t = float(data.time)
            target = command["speed"] if viewer else ((-.15 if name == "reverse" else .15) if name in ("drive", "reverse") and 2 <= t < 6 else 0.0)
            if step % stride == 0:
                controller.step(data, target, command["yaw"] if viewer else (.3 if name == "turn" and 2 <= t < 6 else 0.0))
            if not viewer:
                data.xfrc_applied[:] = 0
            if name == "push" and 3 <= t < 3.2:
                data.xfrc_applied[model.body("base").id, 0] = 2.0
            if name == "side-push" and 3 <= t < 3.2:
                data.xfrc_applied[model.body("base").id, 1] = 2.0
            mujoco.mj_step(model, data)
            for joint_id in np.flatnonzero(model.jnt_limited):
                q = data.qpos[model.jnt_qposadr[joint_id]]
                lo, hi = model.jnt_range[joint_id]
                max_limit_violation = max(max_limit_violation, float(lo - q), float(q - hi))
            for side in SIDES:
                error = np.linalg.norm(data.site(side + "_front_rod_end").xpos - data.site(side + "_back_rod_end").xpos)
                max_closure = max(max_closure, float(error))
            for contact in data.contact:
                names = [model.geom(int(i)).name for i in contact.geom]
                if "floor" in names and not any("wheel_geom" in n for n in names):
                    bad_contact = True
            if step % (30 if viewer else stride) == 0:
                roll, pitch, yaw = legacy.quat_to_euler_wxyz(data.qpos[3:7])
                trace.append([data.time, data.qpos[0], data.qvel[0], pitch, roll, yaw, target,
                              *controller.leg_lengths, *data.ctrl, *controller.state,
                              controller.reference, controller.speed, controller.enabled])
                positions.append(data.qpos.copy())
                velocities.append(data.qvel.copy())
                times.append(float(data.time))
            if viewer_handle is not None and step % 16 == 0:
                viewer_handle.cam.lookat[:] = data.xpos[model.body("base").id]
                viewer_handle.sync()
                time.sleep(max(0, start_clock + step * c["physics_dt_s"] - time.perf_counter()))
            if (not controller.enabled and not viewer) or not np.all(np.isfinite(data.qpos)):
                break
    finally:
        if viewer_handle is not None:
            viewer_handle.close()
    result = np.asarray(trace)
    if len(result) == 0:
        raise RuntimeError("没有生成控制采样")
    tail = result[result[:, 0] > max(0, result[-1, 0] - 2)]
    moving = result[(result[:, 0] > 3.5) & (result[:, 0] < 5.5)]
    summary = {"name": name, "duration_s": float(data.time), "completed": bool(data.time >= duration - .005),
               "peak_pitch_deg": float(np.degrees(max(abs(result[:, 3])))),
               "peak_roll_deg": float(np.degrees(max(abs(result[:, 4])))),
               "final_speed_rms_m_s": float(np.sqrt(np.mean(tail[:, 2] ** 2))),
               "final_x_m": float(result[-1, 1]), "final_reference_m": float(controller.reference),
               "drive_speed_rms_error_m_s": float(np.sqrt(np.mean((moving[:, 2] - (-.15 if name == "reverse" else .15)) ** 2))) if name in ("drive", "reverse") and len(moving) else None,
               "max_closure_error_m": max_closure, "nonwheel_floor_contact": bad_contact,
               "max_joint_limit_violation_rad": max_limit_violation,
               "final_pitch_deg": float(np.degrees(result[-1, 3])),
               "final_yaw_error_deg": float(np.degrees(legacy.wrap_pi(result[-1, 5] - controller.yaw_reference))),
               "saturation_fraction": controller.saturated / max(controller.samples, 1),
               "max_motor_torque_nm": np.max(abs(result[:, 9:15]), axis=0).tolist(),
               "mujoco_warnings": int(sum(data.warning.number)), "controller_active": controller.enabled}
    summary["passed"] = (summary["completed"] and summary["controller_active"] and not bad_contact
                          and summary["mujoco_warnings"] == 0 and max_closure < .001
                          and max_limit_violation < .01 and abs(summary["final_pitch_deg"]) < 1
                          and abs(summary["final_yaw_error_deg"]) < 5
                          and summary["peak_pitch_deg"] < 15 and summary["peak_roll_deg"] < 10
                          and summary["final_speed_rms_m_s"] < .03
                          and abs(summary["final_x_m"] - summary["final_reference_m"]) < .05
                          and (summary["drive_speed_rms_error_m_s"] is None or summary["drive_speed_rms_error_m_s"] < .04))
    output = Path(output)
    np.savez_compressed(output / f"{name}.npz", trace=result, qpos=positions, qvel=velocities, time=times)
    header = "time,x,vx,pitch,roll,yaw,target_v,right_length,left_length,rf_tau,rb_tau,rw_tau,lf_tau,lb_tau,lw_tau,theta,dtheta,s_error,v_error,pitch_feedback,gyro_y,s_ref,v_ref,enabled"
    np.savetxt(output / f"{name}.csv", result, delimiter=",", header=header, comments="")
    write_json(output / f"{name}.json", summary)
    print(json.dumps(summary, ensure_ascii=True), flush=True)
    return summary


def sensitivity(core, profile, designed, output):
    # 只改被控物理模型，保持名义增益、支撑前馈和传感器标定不变。
    variations = [("com-low", "body_com_z_m", 0.0), ("com-high", "body_com_z_m", .04),
                  ("rotating-light", "output_rotating_mass_each_kg", .005),
                  ("rotating-heavy", "output_rotating_mass_each_kg", .03),
                  ("wheels-light", "wheel_ends_mass_total_kg", .256),
                  ("wheels-heavy", "wheel_ends_mass_total_kg", .384),
                  ("links-light", "leg_links_mass_total_kg", .096),
                  ("links-heavy", "leg_links_mass_total_kg", .144)]
    results = []
    for label, field, value in variations:
        plant = copy.deepcopy(profile)
        plant["assumed"][field] = value
        folder = Path(output) / label
        folder.mkdir()
        xml = model_xml(plant)
        (folder / "model.xml").write_text(xml, encoding="utf-8")
        write_json(folder / "plant_parameters.json", plant)
        model = mujoco.MjModel.from_xml_string(xml)
        cases = [run_case(model, make_config(plant), core, profile, designed, name, folder)
                 for name in ("tilt-plus", "drive", "push")]
        results.append({"variation": label, "field": field, "value": value, "cases": cases,
                        "passed": all(s["passed"] for s in cases)})
    write_json(Path(output) / "sensitivity.json", {"controller": "frozen nominal", "results": results,
                                                  "passed": all(s["passed"] for s in results)})
    return all(s["passed"] for s in results)


def render(model, path, qpos):
    from PIL import Image
    data = mujoco.MjData(model)
    data.qpos[:] = qpos
    mujoco.mj_forward(model, data)
    camera = mujoco.MjvCamera()
    camera.lookat[:] = data.xpos[model.body("base").id]
    camera.distance, camera.azimuth, camera.elevation = .65, 130, -18
    with mujoco.Renderer(model, height=800, width=1280) as renderer:
        renderer.update_scene(data, camera=camera)
        Image.fromarray(renderer.render()).save(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", type=Path, default=PROFILE)
    parser.add_argument("--output", type=Path, default=ROOT / "local/cache" / ("wheelleg-measured-" + time.strftime("%Y%m%d-%H%M%S")))
    parser.add_argument("--check", action="store_true")
    parser.add_argument("--suite", action="store_true")
    parser.add_argument("--sensitivity", action="store_true")
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--render", action="store_true")
    parser.add_argument("--scenario", choices=("stand", "tilt-plus", "tilt-minus", "drive", "reverse", "push", "side-push", "turn"), default="stand")
    parser.add_argument("--duration", type=float, default=14.0)
    args = parser.parse_args()
    if args.duration <= 0 or not math.isfinite(args.duration):
        parser.error("duration 必须为有限正数")
    if args.viewer and (args.suite or args.sensitivity or args.check):
        parser.error("交互窗口不能同时运行批量检查")
    profile = load_profile(args.profile)
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    if (output / "parameters.json").exists():
        raise SystemExit("输出目录已有实验记录，请使用新目录")
    write_json(output / "parameters.json", profile)
    core = Core(profile, ROOT / "local/cache/measured-wheelleg-native")
    cfg = make_config(profile)
    xml = model_xml(profile)
    (output / "model.xml").write_text(xml, encoding="utf-8")
    model = mujoco.MjModel.from_xml_string(xml)
    checks = geometry_checks(model, cfg, core, profile)
    designed = design(model, cfg, core, profile, profile["control"]["leg_length_m"])
    write_json(output / "model_checks.json", checks)
    write_json(output / "controller.json", designed)
    source_paths = [Path(__file__), args.profile.resolve(), HERE / "MeasuredWheelLegBridge.c", HERE / "RunWheelLeg.py"] + legacy.source_dependencies()[1:]
    write_json(output / "sources.json", {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in source_paths})
    write_json(output / "runtime.json", {"python": sys.version, "executable": sys.executable,
               "mujoco": mujoco.__version__, "numpy": np.__version__, "scipy": scipy.__version__})
    print(json.dumps({"output": str(output), "checks": checks, "spectral_radius": designed["spectral_radius"]}, ensure_ascii=True), flush=True)
    if args.render:
        render(model, output / "model.png", designed["qpos"])
    if args.check:
        return
    names = ("stand", "tilt-plus", "tilt-minus", "drive", "reverse", "push", "side-push", "turn") if args.suite else (args.scenario,)
    summaries = [run_case(model, cfg, core, profile, designed, name, output, args.viewer, args.duration) for name in names]
    write_json(output / "summary.json", {"cases": summaries, "passed": all(s["passed"] for s in summaries),
                                         "scope": "假设质量分布的简化小车物理仿真，非实车验收"})
    sensitivity_ok = sensitivity(core, profile, designed, output) if args.sensitivity else True
    if not args.viewer and (not all(s["passed"] for s in summaries) or not sensitivity_ok):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
