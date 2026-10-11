"""用 WBR 的 MuJoCo 质量与约束离线计算 ARBATOS 自有轮腿 LQR。

本脚本只读取 MJCF 与独立求得的直立姿态。它不加载上游控制器，也不读取
上游控制轨迹。控制状态统一为：

    [theta, dtheta, x, v, pitch_core, gyro_core]

其中 x 采用 MuJoCo 世界 +X，pitch_core 与 MuJoCo pitch 反号。两个输入是每侧同时
施加的规范轮力矩和虚拟髋力矩。输出 negative_k 可直接用于当前 core 的
``u = k * state_error`` 形式。
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import mujoco
import numpy as np
from scipy.linalg import expm, solve_discrete_are
from scipy.optimize import least_squares


ROOT = Path(__file__).resolve().parents[3]
HERE = Path(__file__).resolve().parent
MODEL_PATH = ROOT / "local/reference/wbr_mujoco/mjcf/scene.xml"
OUTPUT_PATH = ROOT / "local/cache/wbr-own-20261003/regenerated-profile.json"

CONTROL_DT = 0.003
WHEEL_RADIUS = 0.057997

# 独立几何探针得到的 core 规范坐标。
CORE_FRONT_OFFSET = 1.804837140117
CORE_BACK_OFFSET = -1.486361593218
L1 = 0.214999936171
L2 = 0.254000234356
L3 = 0.254000234356
L4 = 0.214999936171
L5 = 0.0

SIDES = {
    "left": {
        "prefix": "l",
        "sign": -1.0,
        "joint1": "ljoint1",
        "joint4": "ljoint4",
        "wheel_joint": "lwheel",
        "wheel_body": "lwlink",
    },
    "right": {
        "prefix": "r",
        "sign": 1.0,
        "joint1": "rjoint1",
        "joint4": "rjoint4",
        "wheel_joint": "rwheel",
        "wheel_body": "rwlink",
    },
}

# 这是设计权重，不是从上游控制器复制的增益。量纲顺序与 STATE_NAMES 一致。
STATE_NAMES = ["theta_rad", "dtheta_radps", "x_m", "v_mps", "pitch_core_rad", "gyro_core_radps"]
INPUT_NAMES = ["wheel_torque_each_side_nm", "hip_virtual_torque_each_side_nm"]
Q_DIAG = np.array([80.0, 1.5, 40.0, 200.0, 1000.0, 20.0], dtype=float)
R_DIAG = np.array([1.0, 0.5], dtype=float)


def wrap_pi(value: float) -> float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi


def quat_pitch_wxyz(quat: np.ndarray) -> float:
    w, x, y, z = (float(v) for v in quat)
    sinp = 2.0 * (w * y - z * x)
    return math.asin(max(-1.0, min(1.0, sinp)))


def joint_qpos(model: mujoco.MjModel, qpos: np.ndarray, name: str) -> float:
    return float(qpos[int(model.joint(name).qposadr[0])])


def joint_dof(model: mujoco.MjModel, name: str) -> int:
    return int(model.joint(name).dofadr[0])


def core_leg(model: mujoco.MjModel, qpos: np.ndarray, side_name: str) -> dict[str, float]:
    side = SIDES[side_name]
    sign = float(side["sign"])
    q1 = joint_qpos(model, qpos, str(side["joint1"]))
    q4 = joint_qpos(model, qpos, str(side["joint4"]))
    front = CORE_FRONT_OFFSET - sign * q4
    back = CORE_BACK_OFFSET - sign * q1
    phi1 = math.pi * 0.5 + front
    phi4 = math.pi * 0.5 + back
    xb = L1 * math.cos(phi1)
    yb = L1 * math.sin(phi1)
    xd = L5 + L4 * math.cos(phi4)
    yd = L4 * math.sin(phi4)
    lbd = math.hypot(xd - xb, yd - yb)
    a0 = 2.0 * L2 * (xd - xb)
    b0 = 2.0 * L2 * (yd - yb)
    c0 = L2 * L2 + lbd * lbd - L3 * L3
    disc = a0 * a0 + b0 * b0 - c0 * c0
    if lbd <= 1.0e-9 or disc < -1.0e-10 or abs(a0 + c0) < 1.0e-12:
        raise RuntimeError(f"{side_name} core FK 越过奇异位形: disc={disc:.6g}")
    phi2 = 2.0 * math.atan2(b0 + math.sqrt(max(0.0, disc)), a0 + c0)
    xc = xb + L2 * math.cos(phi2)
    yc = yb + L2 * math.sin(phi2)
    phi3 = math.atan2(yb - yd + L2 * math.sin(phi2), xb - xd + L2 * math.cos(phi2))
    phi0 = math.atan2(yc, xc - L5 * 0.5)
    length = math.hypot(xc - L5 * 0.5, yc)
    alpha = math.pi * 0.5 - phi0
    return {
        "front": front,
        "back": back,
        "phi0": phi0,
        "phi1": phi1,
        "phi2": phi2,
        "phi3": phi3,
        "phi4": phi4,
        "length": length,
        "alpha": alpha,
    }


def set_state(model: mujoco.MjModel, data: mujoco.MjData, qpos: np.ndarray, qvel: np.ndarray | None = None) -> None:
    data.qpos[:] = qpos
    data.qvel[:] = 0.0 if qvel is None else qvel
    data.ctrl[:] = 0.0
    data.qfrc_applied[:] = 0.0
    mujoco.mj_forward(model, data)


def configuration_values(model: mujoco.MjModel, data: mujoco.MjData, qpos: np.ndarray) -> dict[str, float]:
    set_state(model, data, qpos)
    pitch_core = -quat_pitch_wxyz(np.asarray(data.body("base_link").xquat))
    legs = {name: core_leg(model, qpos, name) for name in SIDES}
    theta_left = wrap_pi(legs["left"]["alpha"] - pitch_core)
    theta_right = wrap_pi(legs["right"]["alpha"] - pitch_core)
    return {
        "theta": 0.5 * (theta_left + theta_right),
        "theta_left": theta_left,
        "theta_right": theta_right,
        "theta_difference": wrap_pi(theta_left - theta_right),
        "left_length": legs["left"]["length"],
        "right_length": legs["right"]["length"],
        "x": float(data.body("base_link").xpos[0]),
        "pitch_core": pitch_core,
    }


def integrate_position(model: mujoco.MjModel, qpos: np.ndarray, tangent: np.ndarray, scale: float) -> np.ndarray:
    result = np.array(qpos, dtype=float, copy=True)
    mujoco.mj_integratePos(model, result, np.asarray(tangent, dtype=float), float(scale))
    return result


def numeric_configuration_jacobian(
    model: mujoco.MjModel,
    qpos: np.ndarray,
    evaluator,
    output_count: int,
    eps: float = 2.0e-7,
) -> np.ndarray:
    jac = np.zeros((output_count, model.nv), dtype=float)
    basis = np.zeros(model.nv, dtype=float)
    for dof in range(model.nv):
        basis[:] = 0.0
        basis[dof] = 1.0
        plus = np.asarray(evaluator(integrate_position(model, qpos, basis, eps)), dtype=float)
        minus = np.asarray(evaluator(integrate_position(model, qpos, basis, -eps)), dtype=float)
        jac[:, dof] = (plus - minus) / (2.0 * eps)
    return jac


def body_jacobian(model: mujoco.MjModel, data: mujoco.MjData, body_name: str) -> tuple[np.ndarray, np.ndarray]:
    jacp = np.zeros((3, model.nv), dtype=float)
    jacr = np.zeros((3, model.nv), dtype=float)
    mujoco.mj_jacBody(model, data, jacp, jacr, model.body(body_name).id)
    return jacp, jacr


def site_jacobian(model: mujoco.MjModel, data: mujoco.MjData, site_name: str) -> np.ndarray:
    jacp = np.zeros((3, model.nv), dtype=float)
    jacr = np.zeros((3, model.nv), dtype=float)
    mujoco.mj_jacSite(model, data, jacp, jacr, model.site(site_name).id)
    return jacp


def reduction_basis(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    qpos: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, dict[str, float]]:
    set_state(model, data, qpos)
    rows: list[np.ndarray] = []
    connect_y_rows: list[np.ndarray] = []

    # 四个 connect 各保留矢状面的 x/z 速度闭合，共 8 行。
    for prefix in ("l", "r"):
        for index in (1, 2):
            delta = site_jacobian(model, data, f"{prefix}link4_site{index}") - site_jacobian(
                model, data, f"{prefix}link1_site{index}"
            )
            rows.extend((delta[0], delta[2]))
            connect_y_rows.append(delta[1])

    # 轮心保持在地面、禁止侧滑；车体只保留纵向和平面俯仰。
    wheel_jacs: dict[str, tuple[np.ndarray, np.ndarray]] = {}
    for side_name, side in SIDES.items():
        jacp, jacr = body_jacobian(model, data, str(side["wheel_body"]))
        wheel_jacs[side_name] = (jacp, jacr)
        rows.extend((jacp[2], jacp[1]))
    _base_jacp, base_jacr = body_jacobian(model, data, "base_link")
    rows.extend((base_jacr[0], base_jacr[2]))

    def leg_outputs(candidate: np.ndarray) -> np.ndarray:
        left = core_leg(model, candidate, "left")
        right = core_leg(model, candidate, "right")
        return np.array([left["length"], right["length"], wrap_pi(left["alpha"] - right["alpha"])])

    leg_jac = numeric_configuration_jacobian(model, qpos, leg_outputs, 3)
    rows.extend((leg_jac[0], leg_jac[1], leg_jac[2]))

    # 整车前进方向是 MuJoCo +X。适配层定义的轮绝对角速度同样以前进为正，
    # 因而满足 v_x-r*omega_abs=0。
    for side_name in ("left", "right"):
        jacp, jacr = wheel_jacs[side_name]
        rows.append(jacp[0] - WHEEL_RADIUS * jacr[1])

    constraint_jac = np.vstack(rows)

    def reduced_coordinates(candidate: np.ndarray) -> np.ndarray:
        values = configuration_values(model, data, candidate)
        return np.array([values["theta"], values["x"], values["pitch_core"]])

    state_jac = numeric_configuration_jacobian(model, qpos, reduced_coordinates, 3)
    stacked = np.vstack((constraint_jac, state_jac))
    rhs = np.vstack((np.zeros((constraint_jac.shape[0], 3)), np.eye(3)))
    if np.linalg.matrix_rank(constraint_jac, tol=1.0e-8) != 17:
        raise RuntimeError(f"约束雅可比秩异常: {np.linalg.matrix_rank(constraint_jac, tol=1.0e-8)}")
    if np.linalg.matrix_rank(stacked, tol=1.0e-8) != model.nv:
        raise RuntimeError(f"约束与状态坐标不能唯一确定速度: rank={np.linalg.matrix_rank(stacked, tol=1.0e-8)}")
    # 约束表按物理含义保留了 19 行，其中两行在当前对称模型上冗余；
    # 独立秩仍是 17。最小二乘解同时满足全部约束与三个状态坐标定义。
    basis, _residuals, _rank, _singular = np.linalg.lstsq(stacked, rhs, rcond=None)
    evidence = {
        "constraint_rank": float(np.linalg.matrix_rank(constraint_jac, tol=1.0e-8)),
        "stacked_rank": float(np.linalg.matrix_rank(stacked, tol=1.0e-8)),
        "stacked_condition": float(np.linalg.cond(stacked)),
        "constraint_basis_max_abs": float(np.max(np.abs(constraint_jac @ basis))),
        "state_basis_max_abs": float(np.max(np.abs(state_jac @ basis - np.eye(3)))),
        "omitted_connect_y_basis_max_abs": float(np.max(np.abs(np.vstack(connect_y_rows) @ basis))),
    }
    return basis, constraint_jac, state_jac, evidence


def full_mass_matrix(model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
    matrix = np.zeros((model.nv, model.nv), dtype=float)
    mujoco.mj_fullM(model, data, matrix)
    return matrix


def input_force_matrix(model: mujoco.MjModel, qpos: np.ndarray) -> np.ndarray:
    force = np.zeros((model.nv, 2), dtype=float)
    for side_name, side in SIDES.items():
        sign = float(side["sign"])
        force[joint_dof(model, str(side["wheel_joint"])), 0] = sign
        leg = core_leg(model, qpos, side_name)
        sin32 = math.sin(leg["phi3"] - leg["phi2"])
        if abs(sin32) < 1.0e-7 or leg["length"] <= 1.0e-7:
            raise RuntimeError(f"{side_name} VMC 雅可比奇异")
        front_per_tp = (
            L1
            * math.cos(leg["phi0"] - leg["phi3"])
            * math.sin(leg["phi1"] - leg["phi2"])
            / (leg["length"] * sin32)
        )
        back_per_tp = (
            L4
            * math.cos(leg["phi0"] - leg["phi2"])
            * math.sin(leg["phi3"] - leg["phi4"])
            / (leg["length"] * sin32)
        )
        force[joint_dof(model, str(side["joint1"])), 1] = -sign * back_per_tp
        force[joint_dof(model, str(side["joint4"])), 1] = -sign * front_per_tp
    return force


def reduced_terms(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    qpos: np.ndarray,
    qvel: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, dict[str, float]]:
    basis, _constraint_jac, _state_jac, evidence = reduction_basis(model, data, qpos)
    set_state(model, data, qpos, qvel)
    mass = full_mass_matrix(model, data)
    reduced_mass = basis.T @ mass @ basis
    input_force = input_force_matrix(model, qpos)
    smooth_force = np.asarray(data.qfrc_passive - data.qfrc_bias, dtype=float)
    evidence.update(
        {
            "mass_symmetry_max_abs": float(np.max(np.abs(reduced_mass - reduced_mass.T))),
            "mass_min_eigenvalue": float(np.min(np.linalg.eigvalsh(reduced_mass))),
            "mass_max_eigenvalue": float(np.max(np.linalg.eigvalsh(reduced_mass))),
            "mass_condition": float(np.linalg.cond(reduced_mass)),
        }
    )
    return basis, reduced_mass, input_force, smooth_force, evidence


def reduced_acceleration(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    qpos: np.ndarray,
    reduced_velocity: np.ndarray,
    control: np.ndarray,
) -> tuple[np.ndarray, dict[str, float]]:
    basis, _constraint_jac, _state_jac, _evidence = reduction_basis(model, data, qpos)
    qvel = basis @ np.asarray(reduced_velocity, dtype=float)
    basis, reduced_mass, input_force, smooth_force, evidence = reduced_terms(model, data, qpos, qvel)
    rhs = basis.T @ (smooth_force + input_force @ np.asarray(control, dtype=float))
    return np.linalg.solve(reduced_mass, rhs), evidence


def linearize(model: mujoco.MjModel, data: mujoco.MjData, qpos0: np.ndarray):
    basis0, reduced_mass, input_force, smooth_force, evidence = reduced_terms(model, data, qpos0)
    reduced_input_force = basis0.T @ input_force
    reduced_bias_force = basis0.T @ smooth_force
    control_bias, *_ = np.linalg.lstsq(reduced_input_force, -reduced_bias_force, rcond=None)
    equilibrium_residual = reduced_bias_force + reduced_input_force @ control_bias
    input_accel = np.linalg.solve(reduced_mass, reduced_input_force)

    position_accel = np.zeros((3, 3), dtype=float)
    velocity_accel = np.zeros((3, 3), dtype=float)
    position_eps = np.array([2.0e-5, 2.0e-5, 2.0e-5])
    velocity_eps = np.array([2.0e-4, 2.0e-4, 2.0e-4])
    zero_velocity = np.zeros(3)
    for index in range(3):
        tangent = basis0[:, index]
        qplus = integrate_position(model, qpos0, tangent, position_eps[index])
        qminus = integrate_position(model, qpos0, tangent, -position_eps[index])
        aplus, _ = reduced_acceleration(model, data, qplus, zero_velocity, control_bias)
        aminus, _ = reduced_acceleration(model, data, qminus, zero_velocity, control_bias)
        position_accel[:, index] = (aplus - aminus) / (2.0 * position_eps[index])

        vplus = np.zeros(3)
        vminus = np.zeros(3)
        vplus[index] = velocity_eps[index]
        vminus[index] = -velocity_eps[index]
        aplus, _ = reduced_acceleration(model, data, qpos0, vplus, control_bias)
        aminus, _ = reduced_acceleration(model, data, qpos0, vminus, control_bias)
        velocity_accel[:, index] = (aplus - aminus) / (2.0 * velocity_eps[index])

    # 世界 X 平移是 MJCF 的严格对称性。沿该方向有限差分得到的微小项只来自
    # 数值雅可比和约束最小二乘误差，不能作为虚假的位置回复力写进 LQR 模型。
    translation_invariance_raw = position_accel[:, 1].copy()
    position_accel[:, 1] = 0.0

    # z=[theta,x,pitch] 转为 core 使用的交错六状态。
    a = np.zeros((6, 6), dtype=float)
    b = np.zeros((6, 2), dtype=float)
    a[0, 1] = 1.0
    a[2, 3] = 1.0
    a[4, 5] = 1.0
    position_columns = (0, 2, 4)
    velocity_columns = (1, 3, 5)
    acceleration_rows = (1, 3, 5)
    for reduced_row, full_row in enumerate(acceleration_rows):
        a[full_row, list(position_columns)] = position_accel[reduced_row]
        a[full_row, list(velocity_columns)] = velocity_accel[reduced_row]
        b[full_row] = input_accel[reduced_row]

    evidence.update(
        {
            "projected_force_without_bias_norm": float(np.linalg.norm(reduced_bias_force)),
            "projected_equilibrium_residual_norm": float(np.linalg.norm(equilibrium_residual)),
            "projected_equilibrium_residual_max_abs": float(np.max(np.abs(equilibrium_residual))),
            "translation_invariance_raw_accel_per_m": translation_invariance_raw.tolist(),
            "translation_invariance_enforced": True,
        }
    )
    return a, b, control_bias, basis0, position_accel, velocity_accel, evidence


def discretize(a: np.ndarray, b: np.ndarray, dt: float) -> tuple[np.ndarray, np.ndarray]:
    block = np.zeros((a.shape[0] + b.shape[1], a.shape[1] + b.shape[1]), dtype=float)
    block[: a.shape[0], : a.shape[1]] = a
    block[: a.shape[0], a.shape[1] :] = b
    discrete = expm(block * dt)
    return discrete[: a.shape[0], : a.shape[1]], discrete[: a.shape[0], a.shape[1] :]


def controllability_matrix(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    columns = [b]
    term = np.array(b, copy=True)
    for _ in range(1, a.shape[0]):
        term = a @ term
        columns.append(term)
    return np.hstack(columns)


def design_lqr(ad: np.ndarray, bd: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    q = np.diag(Q_DIAG)
    r = np.diag(R_DIAG)
    p = solve_discrete_are(ad, bd, q, r)
    gain = np.linalg.solve(r + bd.T @ p @ bd, bd.T @ p @ ad)
    poles = np.linalg.eigvals(ad - bd @ gain)
    return gain, p, poles


def linearization_probe(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    qpos0: np.ndarray,
    basis0: np.ndarray,
    control_bias: np.ndarray,
    position_accel: np.ndarray,
    velocity_accel: np.ndarray,
    input_accel: np.ndarray,
) -> dict[str, float | list[float]]:
    dz = np.array([2.0e-5, -1.5e-5, 1.0e-5])
    dv = np.array([1.0e-4, -8.0e-5, 6.0e-5])
    du = np.array([1.0e-4, -1.0e-4])
    qtest = integrate_position(model, qpos0, basis0 @ dz, 1.0)
    actual, _ = reduced_acceleration(model, data, qtest, dv, control_bias + du)
    predicted = position_accel @ dz + velocity_accel @ dv + input_accel @ du
    error = actual - predicted
    return {
        "actual_acceleration": actual.tolist(),
        "predicted_acceleration": predicted.tolist(),
        "error": error.tolist(),
        "error_norm": float(np.linalg.norm(error)),
        "relative_error": float(np.linalg.norm(error) / max(np.linalg.norm(actual), 1.0e-12)),
    }


def standing_pose(model: mujoco.MjModel) -> dict:
    """只解闭链、腿长、轮高和重心条件，不运行任何控制器。"""
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    seed = data.qpos.copy()
    wheel_indices = [int(model.joint(n).qposadr[0]) for n in ("lwheel", "rwheel")]

    def residual(values):
        data.qpos[:] = seed
        data.qpos[7:], data.qpos[2] = values[:-1], values[-1]
        mujoco.mj_forward(model, data)
        result = []
        for side in ("l", "r"):
            for number in (1, 2):
                result.extend((data.site(f"{side}link4_site{number}").xpos
                               - data.site(f"{side}link1_site{number}").xpos)[[0, 2]])
            wheel = data.body(side + "wlink").xpos
            hip = data.body(side + "link1").xpos
            result.extend([wheel[2] - .060, np.linalg.norm((wheel - hip)[[0, 2]]) - .20])
        result.extend([data.qpos[i] * .1 for i in wheel_indices])
        result.append(data.subtree_com[1, 0] - .5 * (data.body("lwlink").xpos[0]
                                                     + data.body("rwlink").xpos[0]))
        return np.array(result)

    solution = least_squares(residual, np.r_[seed[7:], .10], max_nfev=500,
                             xtol=1e-12, ftol=1e-12, gtol=1e-12)
    error = float(np.max(np.abs(residual(solution.x))))
    if not solution.success or error > 1e-7:
        raise RuntimeError(f"初始站姿未解出：{error}")
    return {"qpos": data.qpos.tolist(), "leg_length": .20, "residual": error}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=OUTPUT_PATH)
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    pose = standing_pose(model)
    data = mujoco.MjData(model)
    qpos0 = np.asarray(pose["qpos"], dtype=float)
    if qpos0.shape != (model.nq,):
        raise RuntimeError(f"static pose qpos 长度错误: {qpos0.shape}, expected {(model.nq,)}")
    # 静态求解时轮心按标称 0.060 m 放置；碰撞网格的等效半径是 0.057997 m。
    # 只做全车竖直平移，不改变关节姿态或约束线性化。
    qpos0 = qpos0.copy()
    qpos0[2] -= 0.060 - WHEEL_RADIUS

    values = configuration_values(model, data, qpos0)
    a, b, control_bias, basis0, position_accel, velocity_accel, evidence = linearize(model, data, qpos0)
    unit_input_force = input_force_matrix(model, qpos0)
    actuator_input_mapping = {}
    for side_name, side in SIDES.items():
        actuator_input_mapping[side_name] = {
            "joint1_per_unit_hip": float(unit_input_force[joint_dof(model, str(side["joint1"])), 1]),
            "joint4_per_unit_hip": float(unit_input_force[joint_dof(model, str(side["joint4"])), 1]),
            "wheel_per_unit_wheel": float(unit_input_force[joint_dof(model, str(side["wheel_joint"])), 0]),
        }
    ad, bd = discretize(a, b, CONTROL_DT)
    gain, riccati, poles = design_lqr(ad, bd)
    ctrb = controllability_matrix(ad, bd)
    ctrb_singular = np.linalg.svd(ctrb, compute_uv=False)
    continuous_ctrb = controllability_matrix(a, b)
    continuous_ctrb_singular = np.linalg.svd(continuous_ctrb, compute_uv=False)
    discrete_rank_tolerance = float(ctrb_singular[0] * 1.0e-9)
    probe = linearization_probe(
        model,
        data,
        qpos0,
        basis0,
        control_bias,
        position_accel,
        velocity_accel,
        b[[1, 3, 5], :],
    )

    total_mass = float(sum(model.body_mass[1:]))
    result = {
        "source": {
            "model": str(MODEL_PATH.relative_to(ROOT)).replace("\\", "/"),
            "static_pose": "independent closed-chain/COM solve in DesignOwnWbr.py",
            "static_pose_residual_m": pose["residual"],
            "uses_upstream_controller": False,
            "uses_upstream_trajectory": False,
        },
        "coordinates": {
            "states": STATE_NAMES,
            "inputs": INPUT_NAMES,
            "x_definition": "+MuJoCo base_link world x",
            "pitch_definition": "-MuJoCo base_link pitch",
            "theta_definition": "mean(core alpha left/right)-pitch_core",
            "input_definition": "same canonical per-side command applied to both sides",
            "actuator_input_mapping": actuator_input_mapping,
            "bridge_mapping_check": "wheel=s*wheelTorque; joint1=-s*coreBackTorque; joint4=-s*coreFrontTorque",
        },
        "model": {
            "total_mass_kg": total_mass,
            "wheel_collision_radius_m": WHEEL_RADIUS,
            "control_dt_s": CONTROL_DT,
            "physics_dt_s": 0.001,
            "leg_geometry_m": {"l1": L1, "l2": L2, "l3": L3, "l4": L4, "l5": L5},
            "equilibrium_coordinates": values,
        },
        "static_feedforward": {
            "wheel_torque_each_side_nm": float(control_bias[0]),
            "hip_virtual_torque_each_side_nm": float(control_bias[1]),
        },
        "weights": {"Q_diag": Q_DIAG.tolist(), "R_diag": R_DIAG.tolist()},
        "continuous": {"A": a.tolist(), "B": b.tolist()},
        "discrete_3ms": {"A": ad.tolist(), "B": bd.tolist()},
        "lqr": {
            "standard_K_for_u_equals_minus_Kx": gain.tolist(),
            "negative_K_for_WheelLegCore": (-gain).tolist(),
            "wheel_row_negative_K": (-gain[0]).tolist(),
            "hip_row_negative_K": (-gain[1]).tolist(),
            "closed_loop_poles": [[float(value.real), float(value.imag)] for value in poles],
            "riccati_symmetry_max_abs": float(np.max(np.abs(riccati - riccati.T))),
        },
        "evidence": {
            **evidence,
            "continuous_controllability_rank": int(np.linalg.matrix_rank(continuous_ctrb)),
            "continuous_controllability_smallest_singular": float(continuous_ctrb_singular[-1]),
            "discrete_controllability_rank": int(np.linalg.matrix_rank(ctrb, tol=discrete_rank_tolerance)),
            "discrete_controllability_rank_tolerance": discrete_rank_tolerance,
            "controllability_smallest_singular": float(ctrb_singular[-1]),
            "controllability_smallest_to_largest_ratio": float(ctrb_singular[-1] / ctrb_singular[0]),
            "closed_loop_spectral_radius": float(np.max(np.abs(poles))),
            "linearization_probe": probe,
        },
        "limits": [
            "轮地接触被理想无滑滚动、固定轮高与禁止侧滑约束代替；未线性化接触软化和干摩擦。",
            "腿长固定为 0.20 m；该结果尚未做 0.16--0.36 m 增益调度。",
            "Q/R 是本脚本的保守起始权重，仿真限幅和扰动测试通过前不能用于实车。",
        ],
    }
    support_n = total_mass * 9.81 / (2.0 * max(math.cos(float(values["theta"])), 0.2))
    # RunOwnWbr.py 直接读取的简洁 profile 字段。
    result.update(
        {
            "negative_k": (-gain).tolist(),
            "theta0": float(values["theta"]),
            "tp0": float(control_bias[1]),
            "wheel0": float(control_bias[0]),
            "support_n": float(support_n),
            "length_m": float(pose["leg_length"]),
            "radius_m": WHEEL_RADIUS,
            "length_kp": 1000.0,
            "length_kd": 60.0,
            "qpos": qpos0.tolist(),
        }
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    print(f"output={args.output}")
    print(f"constraint_rank={int(evidence['constraint_rank'])} residual={evidence['constraint_basis_max_abs']:.3e}")
    print(f"mass_eigen_min={evidence['mass_min_eigenvalue']:.6g} condition={evidence['mass_condition']:.6g}")
    print(f"equilibrium_control={control_bias.tolist()} residual={evidence['projected_equilibrium_residual_norm']:.3e}")
    print(f"continuous_controllability_rank={np.linalg.matrix_rank(continuous_ctrb)}")
    print(f"discrete_controllability_rank={np.linalg.matrix_rank(ctrb, tol=discrete_rank_tolerance)}")
    print(f"closed_loop_spectral_radius={np.max(np.abs(poles)):.9f}")
    print(f"linearization_relative_error={probe['relative_error']:.3e}")
    print("negative_K_for_WheelLegCore=")
    print(-gain)


if __name__ == "__main__":
    main()
