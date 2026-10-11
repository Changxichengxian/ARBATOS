"""给 WBR 原模型选择可追溯的膝关节收腿参考范围。"""
from __future__ import annotations

from copy import deepcopy
import math

import mujoco
import numpy as np


_UPSTREAM_JOINTS = {
    "llink1child1joint": {
        "axis": (0.0, -1.0, 0.0),
        "range_rad": (-1.09439510, -0.26179938),
    },
    "rlink1child1joint": {
        "axis": (0.0, 1.0, 0.0),
        "range_rad": (0.26179938, 1.09439510),
    },
}
_CAD_VECTORS = {
    "llink1_child1": (-0.214234, -0.0181319),
    "lwlink": (0.233282, -0.100477),
    "rlink1_child1": (-0.214234, -0.0181319),
    "rwlink": (0.233282, -0.100477),
}
_info: dict[int, dict] = {}
# 180mm闭链静态解：保留长端，只限制左右膝的收腿短端。
_STOP180_RAD = (-0.27977696097759014, 0.27977696097754406)
_HOME_PASSIVE = {
    "llink1child1joint": (-1.09, -1.033069496860784),
    "llink4child1joint": (1.09, 1.033067408837384),
    "llink4child2joint": (-.617, -.5661364808957638),
    "llink4child3joint": (1.09, 1.0330704340441883),
    "rlink1child1joint": (1.09, 1.0330694968607848),
    "rlink4child1joint": (-1.09, -1.0330674088373835),
    "rlink4child2joint": (.617, .5661364808957629),
    "rlink4child3joint": (-1.09, -1.0330704340441876),
}
_HOME_LIFT_M = .016102804407977993


def _values(values) -> list[float]:
    return [float(value) for value in values]


def apply_knee_range(spec: mujoco.MjSpec, mode: str = "upstream") -> dict:
    """校验原模型后选择膝关节范围；不改变长端或其他模型参数。"""
    if mode not in ("upstream", "cad120", "stop180"):
        raise ValueError("knee_range 必须为 upstream、cad120 或 stop180")

    before = {}
    for name, expected in _UPSTREAM_JOINTS.items():
        joint = spec.joint(name)
        actual_axis = np.asarray(joint.axis, dtype=float)
        actual_range = np.asarray(joint.range, dtype=float)
        if joint.limited != mujoco.mjtLimited.mjLIMITED_TRUE:
            raise ValueError(f"{name} 应启用关节范围")
        if not np.allclose(actual_axis, expected["axis"], rtol=0.0, atol=1.0e-9):
            raise ValueError(f"{name} 轴向与原模型不符：{actual_axis.tolist()}")
        if not np.allclose(actual_range, expected["range_rad"], rtol=0.0, atol=1.0e-9):
            raise ValueError(f"{name} 范围与原模型不符：{actual_range.tolist()}")
        before[name] = {
            "axis": _values(actual_axis),
            "range_rad": _values(actual_range),
        }

    vectors = {}
    for body_name, expected_xz in _CAD_VECTORS.items():
        actual_xz = np.asarray(spec.body(body_name).pos, dtype=float)[[0, 2]]
        if not np.allclose(actual_xz, expected_xz, rtol=0.0, atol=1.0e-9):
            raise ValueError(f"{body_name} CAD X/Z 向量与原模型不符：{actual_xz.tolist()}")
        vectors[body_name] = _values(actual_xz)

    if mode == "cad120":
        left = spec.joint("llink1child1joint")
        right = spec.joint("rlink1child1joint")
        left.range = [float(left.range[0]), 0.0]
        right.range = [0.0, float(right.range[1])]
    elif mode == "stop180":
        left = spec.joint("llink1child1joint")
        right = spec.joint("rlink1child1joint")
        left.range = [float(left.range[0]), _STOP180_RAD[0]]
        right.range = [_STOP180_RAD[1], float(right.range[1])]

    after = {
        name: {
            "axis": _values(spec.joint(name).axis),
            "range_rad": _values(spec.joint(name).range),
        }
        for name in _UPSTREAM_JOINTS
    }
    hip_to_wheel = np.asarray(_CAD_VECTORS["llink1_child1"]) + \
        np.asarray(_CAD_VECTORS["lwlink"])
    return {
        "mode": mode,
        "joints_before": before,
        "joints_after": after,
        "cad_reference": {
            "body_xz_vectors_m": vectors,
            "hip_to_wheel_xz_m": _values(hip_to_wheel),
            "hip_to_wheel_length_mm": float(math.hypot(*hip_to_wheel) * 1000.0),
        },
        "stop_length_m": .18 if mode == "stop180" else None,
        "stop_source": ("用户确认髋轴到轮轴的机构平面腿长180mm；"
            "spring-static-support.json闭链解，两侧XZ腿长独立复核为180mm") if mode == "stop180" else None,
        "statement": ("180mm实体挡块的膝短端近似；只改两膝短端，保留长端、碰撞与气簧几何")
            if mode == "stop180" else "CAD参考收腿范围，实车挡块/气弹簧行程未标定",
    }


def register_knee_range(model: mujoco.MjModel, info: dict) -> None:
    """登记已编译模型实际采用的膝关节范围说明。"""
    _info[hash(model)] = deepcopy(info)


def apply_stop180_home(model: mujoco.MjModel) -> dict:
    """只修正原home的被动闭链关节及初始地面间隙，不扶正或预站立。"""
    pose = model.key("home").qpos
    original = pose.copy()
    for name, (expected, solved) in _HOME_PASSIVE.items():
        address = int(model.jnt_qposadr[model.joint(name).id])
        if abs(pose[address] - expected) > 1e-8:
            raise ValueError(f"{name} 原home角度已变化，不能套用固定闭链修正")
        pose[address] = solved
    pose[2] += _HOME_LIFT_M
    for name in ("ljoint1", "ljoint4", "rjoint1", "rjoint4"):
        address = int(model.jnt_qposadr[model.joint(name).id])
        if pose[address] != original[address]:
            raise ValueError("home修正改变了主动关节")
    if not np.array_equal(pose[3:7], original[3:7]):
        raise ValueError("home修正改变了机身姿态")
    data = mujoco.MjData(model)
    data.qpos[:] = pose
    mujoco.mj_forward(model, data)
    closure = max(float(np.linalg.norm(data.site(f"{side}link4_site{number}").xpos -
                  data.site(f"{side}link1_site{number}").xpos))
                  for side in ("l", "r") for number in (1, 2))
    floor = model.geom("floor").id
    penetration = max((max(0., -float(contact.dist)) for contact in data.contact
                       if floor in (contact.geom1, contact.geom2)), default=0.)
    if closure > .001 or penetration > 1e-9:
        raise ValueError(f"修正后的home无效：闭链误差{closure}m，地面穿透{penetration}m")
    return dict(passive_joint_angles_rad={name: solved for name, (_, solved) in _HOME_PASSIVE.items()},
        base_lift_m=_HOME_LIFT_M, closure_m=closure, ground_penetration_m=penetration,
        original_active_joints_preserved=True, original_base_orientation_preserved=True,
        source="2026-10-12独立低趴闭链求解，仅调整8个被动关节；固定常数无需缓存文件",
        verified_pose_sha256="8A0049F47C987CCBDF486101563E8FE2AA030FDB51AC1524BBA4A5D42BDD870E")


def knee_range_info(model: mujoco.MjModel) -> dict:
    """返回范围说明的副本，避免调用方改写登记内容。"""
    return deepcopy(_info.get(hash(model), {"mode": "unknown"}))
