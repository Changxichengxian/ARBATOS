"""给原版 WBR MuJoCo 模型叠加两根内部恒力气弹簧。

450 N、30/50 mm 安装距离和 0.2164 rad 安装角来自开源控制器。这里把
450 N 当作恒力参考；用户实物的行程、温度和长度-力曲线仍未知。
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path

import mujoco
import numpy as np

from WbrTerrain import apply_terrain
from WbrGuideRollers import apply_guide_rollers, register_guide_rollers
from WbrLinkCollision import apply_link_collision_refinement, register_link_collision
from WbrKneeRange import apply_knee_range, apply_stop180_home, register_knee_range


ROOT = Path(__file__).resolve().parents[3]
DEFAULT_MODEL_PATH = ROOT / "local/reference/wbr_mujoco/mjcf/scene.xml"
DEFAULT_GAS_FORCE_N = 450.0

# 开源控制器参数。连杆方向改用 MJCF 的实际 CAD 关节向量，不沿用控制器中
# 四舍五入后的 0.220/0.260 m。
UPPER_TO_KNEE_XZ_M = np.array([-0.214234, -0.0181319], dtype=float)
LOWER_TO_WHEEL_XZ_M = np.array([0.233282, -0.100477], dtype=float)
UPPER_MOUNT_M = 0.03
LOWER_MOUNT_M = 0.05
MOUNT_ANGLE_RAD = 0.2164

MOTOR_NAMES = (
    "ljoint1_actuator", "ljoint4_actuator", "lwheel_actuator",
    "rjoint1_actuator", "rjoint4_actuator", "rwheel_actuator",
)

# 原 CAD mesh 的默认凸包会让同腿错层连杆在正常工作区持续产生假接触。
# 修正仅由 load_model 的显式开关启用，默认仍保留原模型，便于复现历史结果。
LINK_COLLISION_EXCLUDES = (
    ("llink1", "llink4"),
    ("llink1", "llink4_child3"),
    ("rlink1", "rlink4"),
    ("rlink1", "rlink4_child3"),
)


@dataclass(frozen=True)
class _GasSpringConfig:
    force_n: float
    site_pairs: tuple[tuple[int, int], ...]


# MuJoCo 在编译另一个模型时也可能以临时 Python 包装对象调用全局被动力回调。
# 直接用 MjModel 做 WeakKeyDictionary 键会在该路径抛出异常；其 hash 对应底层
# mjModel 指针，在回调包装对象之间保持一致。
_configs: dict[int, _GasSpringConfig] = {}
_callback_installed = False
_previous_passive_callback = None


def _mount_positions() -> tuple[np.ndarray, np.ndarray]:
    """返回上、下连杆局部坐标中的安装点；Y 修正在建模时按左右侧补入。"""
    upper_angle = math.atan2(-UPPER_TO_KNEE_XZ_M[1], UPPER_TO_KNEE_XZ_M[0])
    mount_angle = upper_angle + MOUNT_ANGLE_RAD - math.pi * 0.5
    upper = np.array([
        UPPER_MOUNT_M * math.cos(mount_angle),
        0.0,
        -UPPER_MOUNT_M * math.sin(mount_angle),
    ])
    lower_unit = LOWER_TO_WHEEL_XZ_M / np.linalg.norm(LOWER_TO_WHEEL_XZ_M)
    lower = np.array([LOWER_MOUNT_M * lower_unit[0], 0.0,
                      LOWER_MOUNT_M * lower_unit[1]])
    return upper, lower


def _passive_callback(model: mujoco.MjModel, data: mujoco.MjData) -> None:
    if _previous_passive_callback is not None:
        _previous_passive_callback(model, data)
    config = _configs.get(hash(model))
    if config is not None and config.force_n > 0.0:
        data.qfrc_passive[:] += gas_generalized_force(model, data)


def _ensure_passive_callback() -> None:
    global _callback_installed, _previous_passive_callback
    if _callback_installed:
        return
    previous = mujoco.get_mjcb_passive()
    if previous is not _passive_callback:
        _previous_passive_callback = previous
        mujoco.set_mjcb_passive(_passive_callback)
    _callback_installed = True


def load_model(model_path: str | Path = DEFAULT_MODEL_PATH,
               gas_force_n: float = DEFAULT_GAS_FORCE_N,
               enabled: bool = True,
               correct_link_collisions: bool = False,
               terrain: str = "flat",
               noslip_iterations: int = 0,
               guide_rollers: str = "legacy",
               knee_range: str = "upstream") -> mujoco.MjModel:
    """加载 WBR，并在不增加执行器的前提下安装内部气弹簧被动力。

    ``gas_force_n`` 允许任意非负有限值，0 等价于关闭。450 N 只对应开源
    参考参数。``correct_link_collisions`` 开启时排除四组已经由原 STL 实形
    核实为凸包假接触的同腿连杆，并把会填平孔洞的 link4_child1 拆成真实
    材料截面的凸片；地面和其余碰撞保持不变。返回模型保持原来的 6 个电机
    及其顺序。``terrain`` 默认 flat；course 加入真实浅坑和台阶。
    ``noslip_iterations`` 默认0，显式开启时抑制软接触的缓慢侧向滑移。
    ``guide_rollers`` 默认 legacy 保留原碰撞；fixed/passive 拆分底盘装配碰撞，
    增加两侧横轴固定对照/被动参考导轮，原关节前缀不变，按布局追加自由度。
    ``knee_range`` 默认 upstream 保留原范围；cad120 只把两侧膝关节短端
    放到原 CAD 零角；stop180 使用独立闭链解的180mm实体挡块短端。
    """
    force_n = float(gas_force_n)
    if not math.isfinite(force_n) or force_n < 0.0:
        raise ValueError("gas_force_n 必须是非负有限值")
    if terrain not in ("flat", "course"):
        raise ValueError("terrain 必须为 flat 或 course")
    if isinstance(noslip_iterations, bool) or not isinstance(noslip_iterations, int) or noslip_iterations < 0:
        raise ValueError("noslip_iterations 必须为非负整数")

    # MuJoCo 3.14 在编译模型时不能保留 Python mjcb_passive；若本模块已被
    # 另一模型启用，先暂停，编译完成后立即恢复。这样可连续加载物理模型和
    # 不受力的渲染模型。
    restore_callback = _callback_installed
    if restore_callback:
        mujoco.set_mjcb_passive(None)
    spec = mujoco.MjSpec.from_file(str(Path(model_path).resolve()))
    knee_range_metadata = apply_knee_range(spec, knee_range)
    spec.option.noslip_iterations = noslip_iterations
    apply_terrain(spec, terrain)
    roller_info = apply_guide_rollers(spec, guide_rollers)
    link_collision_info = {
        "enabled": False,
        "method": "原始单凸包碰撞",
        "added_collision_geoms": 0,
    }
    if correct_link_collisions:
        for index, (body1, body2) in enumerate(LINK_COLLISION_EXCLUDES):
            spec.add_exclude(name=f"gas_link_collision_{index}",
                             bodyname1=body1, bodyname2=body2)
        link_collision_info = apply_link_collision_refinement(spec, model_path)
    upper, lower = _mount_positions()
    # 子连杆在 CAD 中带有横向装配偏移；补回该偏移，使气弹簧处于各腿的
    # X-Z 运动平面内，避免凭空产生横向力矩。
    definitions = (
        ("left", "llink1", "llink1_child1", 0.008),
        ("right", "rlink1", "rlink1_child1", 0.0175),
    )
    for side, upper_body, lower_body, lower_y in definitions:
        spec.body(upper_body).add_site(
            name=f"{side}_gas_upper", pos=upper,
            size=[0.004], rgba=[0.2, 0.8, 1.0, 1.0])
        lower_pos = lower.copy()
        lower_pos[1] = lower_y
        spec.body(lower_body).add_site(
            name=f"{side}_gas_lower", pos=lower_pos,
            size=[0.004], rgba=[1.0, 0.65, 0.15, 1.0])
        # 只用于显示两安装点之间的实体走向；刚度和阻尼均为零，450 N 仍仅
        # 由上面的精确恒力回调施加，避免重复受力。
        visual = spec.add_tendon(name=f"{side}_gas_visual")
        visual.width = 0.006
        visual.rgba = [1.0, 0.5, 0.08, 1.0]
        visual.wrap_site(f"{side}_gas_upper")
        visual.wrap_site(f"{side}_gas_lower")

    try:
        model = spec.compile()
    finally:
        if restore_callback:
            mujoco.set_mjcb_passive(_passive_callback)
    actual_motors = tuple(model.actuator(index).name for index in range(model.nu))
    register_guide_rollers(model, roller_info)
    register_link_collision(model, link_collision_info)
    if knee_range == "stop180":
        knee_range_metadata["home_adjustment"] = apply_stop180_home(model)
    register_knee_range(model, knee_range_metadata)
    if actual_motors != MOTOR_NAMES:
        raise RuntimeError(f"WBR 电机顺序变化：{actual_motors}")
    pairs = tuple(
        (model.site(f"{side}_gas_upper").id, model.site(f"{side}_gas_lower").id)
        for side in ("left", "right")
    )
    _configs[hash(model)] = _GasSpringConfig(force_n if enabled else 0.0, pairs)
    _ensure_passive_callback()
    return model


def gas_spring_state(model: mujoco.MjModel, data: mujoco.MjData,
                     gas_force_n: float | None = None) -> list[dict]:
    """返回每根气弹簧的世界坐标、长度和成对内力，供验证与显示使用。"""
    config = _configs.get(hash(model))
    if config is None:
        raise ValueError("模型不是由 WbrGasSpring.load_model 创建")
    force_n = config.force_n if gas_force_n is None else float(gas_force_n)
    if not math.isfinite(force_n) or force_n < 0.0:
        raise ValueError("gas_force_n 必须是非负有限值")
    result = []
    for side, (upper_id, lower_id) in zip(("left", "right"), config.site_pairs):
        upper = np.asarray(data.site_xpos[upper_id], dtype=float).copy()
        lower = np.asarray(data.site_xpos[lower_id], dtype=float).copy()
        delta = lower - upper
        length = float(np.linalg.norm(delta))
        if length <= 1.0e-9:
            raise RuntimeError(f"{side} 气弹簧两安装点重合")
        lower_force = force_n * delta / length
        result.append({
            "side": side,
            "upper_site_id": upper_id,
            "lower_site_id": lower_id,
            "upper_body_id": int(model.site_bodyid[upper_id]),
            "lower_body_id": int(model.site_bodyid[lower_id]),
            "upper_m": upper,
            "lower_m": lower,
            "length_m": length,
            "upper_force_n": -lower_force,
            "lower_force_n": lower_force,
        })
    return result


def gas_generalized_force(model: mujoco.MjModel, data: mujoco.MjData,
                          gas_force_n: float | None = None) -> np.ndarray:
    """计算两根气弹簧施加的广义被动力，不读取或修改控制器输入。"""
    qfrc = np.zeros(model.nv, dtype=float)
    zero_torque = np.zeros(3, dtype=float)
    for spring in gas_spring_state(model, data, gas_force_n):
        mujoco.mj_applyFT(model, data, spring["upper_force_n"], zero_torque,
                          spring["upper_m"], spring["upper_body_id"], qfrc)
        mujoco.mj_applyFT(model, data, spring["lower_force_n"], zero_torque,
                          spring["lower_m"], spring["lower_body_id"], qfrc)
    return qfrc


def gas_constraint_projected_motor_torque(
        model: mujoco.MjModel, data: mujoco.MjData,
        gas_force_n: float | None = None) -> dict[str, tuple[float, float]]:
    """把完整 CAD 闭链中的气弹簧力投影到两侧主动髋关节。

    这是分析函数，使用 MuJoCo 的等式约束雅可比，不传给实时控制器。实时 C
    补偿使用 ``gas_motor_torque_from_raw`` 的编码器可计算四杆近似。
    """
    qfrc = gas_generalized_force(model, data, gas_force_n)
    jacobian = np.asarray(data.efc_J).reshape(data.nefc, model.nv)
    equality_type = int(mujoco.mjtConstraint.mjCNSTR_EQUALITY)
    result: dict[str, tuple[float, float]] = {}
    definitions = (
        ("left", ("Left_loop1", "Left_loop2"), ("ljoint1", "ljoint4"),
         ("llink1child1joint", "llink4child1joint", "llink4child2joint",
          "llink4child3joint")),
        ("right", ("Right_loop1", "Right_loop2"), ("rjoint1", "rjoint4"),
         ("rlink1child1joint", "rlink4child1joint", "rlink4child2joint",
          "rlink4child3joint")),
    )
    for side, equality_names, active_names, passive_names in definitions:
        equality_ids = {model.equality(name).id for name in equality_names}
        rows = [index for index in range(data.nefc)
                if int(data.efc_type[index]) == equality_type and
                int(data.efc_id[index]) in equality_ids]
        active = [int(model.joint(name).dofadr[0]) for name in active_names]
        passive = [int(model.joint(name).dofadr[0]) for name in passive_names]
        passive_per_active = -np.linalg.pinv(jacobian[np.ix_(rows, passive)]) @ \
            jacobian[np.ix_(rows, active)]
        allowed_velocity = np.zeros((model.nv, 2))
        allowed_velocity[active, :] = np.eye(2)
        allowed_velocity[passive, :] = passive_per_active
        torque = allowed_velocity.T @ qfrc
        result[side] = (float(torque[0]), float(torque[1]))
    return result


def gas_potential_energy(model: mujoco.MjModel, data: mujoco.MjData,
                         gas_force_n: float | None = None) -> float:
    """恒推力模型的势能 U=-F*L；只用于虚功和方向验证。"""
    config = _configs.get(hash(model))
    if config is None:
        raise ValueError("模型不是由 WbrGasSpring.load_model 创建")
    force_n = config.force_n if gas_force_n is None else float(gas_force_n)
    return -force_n * sum(item["length_m"] for item in gas_spring_state(model, data, force_n))


def gas_motor_torque_from_raw(q1: float, q4: float, side: str,
                              gas_force_n: float = DEFAULT_GAS_FORCE_N) -> tuple[float, float]:
    """由两个髋电机编码器角计算物理气弹簧对主动关节的等效力矩。

    返回值采用 MuJoCo 原始 ``joint1/joint4`` 正方向。闭链解法与开源
    ``leg.cpp::calc_spring_force`` 一致，但使用 MJCF 的实际 0.215/0.254 m
    连杆和 CAD 零位方向。
    """
    if side not in ("left", "right"):
        raise ValueError("side 必须是 left 或 right")
    force_n = float(gas_force_n)
    if not math.isfinite(force_n) or force_n < 0.0:
        raise ValueError("gas_force_n 必须是非负有限值")
    sign = -1.0 if side == "left" else 1.0
    l1 = float(np.linalg.norm(UPPER_TO_KNEE_XZ_M))
    l2 = float(np.linalg.norm(LOWER_TO_WHEEL_XZ_M))
    phi1_zero = math.atan2(-UPPER_TO_KNEE_XZ_M[1], UPPER_TO_KNEE_XZ_M[0])
    # link4 的第一个实际 CAD 向量；左右模型在 X-Z 平面相同。
    front_xz = np.array([0.0972737, 0.023191], dtype=float)
    phi4_zero = math.atan2(-front_xz[1], front_xz[0])
    phi1 = phi1_zero + sign * float(q1)
    phi4 = phi4_zero + sign * float(q4)

    xdb = l1 * (math.cos(phi4) - math.cos(phi1))
    ydb = l1 * (math.sin(phi4) - math.sin(phi1))
    a0, b0 = 2.0 * l2 * xdb, 2.0 * l2 * ydb
    c0 = xdb * xdb + ydb * ydb
    discriminant = a0 * a0 + b0 * b0 - c0 * c0
    if discriminant <= 0.0:
        raise ValueError("当前关节角没有有效的五连杆闭链解")
    phi2 = 2.0 * math.atan2(b0 + math.sqrt(discriminant), a0 + c0)
    cx = l1 * math.cos(phi1) + l2 * math.cos(phi2)
    cy = l1 * math.sin(phi1) + l2 * math.sin(phi2)
    phi3 = math.pi + math.atan2(l1 * math.sin(phi4) - cy,
                                l1 * math.cos(phi4) - cx)

    p_angle = phi1 + MOUNT_ANGLE_RAD - math.pi * 0.5
    px, py = UPPER_MOUNT_M * math.cos(p_angle), UPPER_MOUNT_M * math.sin(p_angle)
    qx = l1 * math.cos(phi1) + LOWER_MOUNT_M * math.cos(phi2)
    qy = l1 * math.sin(phi1) + LOWER_MOUNT_M * math.sin(phi2)
    dx, dy = qx - px, qy - py
    spring_length = math.hypot(dx, dy)
    if spring_length <= 1.0e-9 or abs(math.sin(phi3 - phi2)) < 0.05:
        raise ValueError("当前姿态接近气弹簧或五连杆奇异位形")
    fx, fy = force_n * dx / spring_length, force_n * dy / spring_length

    inv = 1.0 / (l2 * math.sin(phi3 - phi2))
    # 对 |B+l2*e(phi2)-D|=l2 隐式求导。开源 leg.cpp 这两项符号相反，
    # 会破坏“整条腿刚体同转时弹簧长度不变”的虚功关系，因此这里采用由
    # 安装点距离直接推导并经有限差分核对的符号。
    dphi2_dphi1 = -l1 * math.sin(phi3 - phi1) * inv
    dphi2_dphi4 = l1 * math.sin(phi3 - phi4) * inv
    dpx1 = -UPPER_MOUNT_M * math.sin(p_angle)
    dpy1 = UPPER_MOUNT_M * math.cos(p_angle)
    dqx1 = -l1 * math.sin(phi1) - LOWER_MOUNT_M * math.sin(phi2) * dphi2_dphi1
    dqy1 = l1 * math.cos(phi1) + LOWER_MOUNT_M * math.cos(phi2) * dphi2_dphi1
    dqx4 = -LOWER_MOUNT_M * math.sin(phi2) * dphi2_dphi4
    dqy4 = LOWER_MOUNT_M * math.cos(phi2) * dphi2_dphi4
    tau_phi1 = fx * (dqx1 - dpx1) + fy * (dqy1 - dpy1)
    tau_phi4 = fx * dqx4 + fy * dqy4
    return sign * tau_phi1, sign * tau_phi4
