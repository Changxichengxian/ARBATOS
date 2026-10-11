"""从原CAD装配表面拆分底盘碰撞，并安装可锁定对照的被动参考导轮。"""
from __future__ import annotations

from functools import lru_cache
import hashlib
from pathlib import Path
import struct

import mujoco
import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components


ROOT = Path(__file__).resolve().parents[3]
BASE_STL = ROOT / "local/reference/wbr_mujoco/mjcf/meshes/base_link.STL"
ROLLER_RADIUS = .016
ROLLER_HALF_WIDTH = .010
ROLLER_MASS = .008
ROLLER_CONTACT_TIMECONST = .004
ROLLER_CONTACT_SOLMIX = 1000.0
ROLLER_ROWS = 17
ROLLER_COLUMNS = 2
# 端轮护住前后壳角，其余各排均布，避免只移端轮留下大空隙。
ROLLER_X = tuple(float(x) for x in np.linspace(-.275, .275, ROLLER_ROWS))
ROLLER_CENTERS = tuple((float(x), float(y), 0.0)
                       for x in ROLLER_X
                       for y in (-.11, .11))
MODES = ("legacy", "fixed", "passive")
_info = {}


@lru_cache(maxsize=4)
def _cad_pieces(path: str, modified_ns: int):
    raw = Path(path).read_bytes()
    count = struct.unpack_from("<I", raw, 80)[0]
    if len(raw) != 84+50*count:
        raise ValueError("底盘CAD须为原始二进制STL")
    dtype = np.dtype([("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attr", "<u2")])
    triangles = np.ndarray(count, dtype=dtype, buffer=raw, offset=84)["vertices"]
    vertices, ids = np.unique(triangles.reshape(-1, 3), axis=0, return_inverse=True)
    faces = ids.reshape(-1, 3)
    rows = np.concatenate([faces[:, 0], faces[:, 1], faces[:, 2]])
    cols = np.concatenate([faces[:, 1], faces[:, 2], faces[:, 0]])
    graph = coo_matrix((np.ones(len(rows)), (rows, cols)), shape=(len(vertices), len(vertices))).tocsr()
    count, labels = connected_components(graph, directed=False)
    pieces = []
    for index in range(count):
        part = vertices[labels == index]
        if len(part) < 4:
            continue
        if part[:, 2].max() < .064 and np.ptp(part[:, 0]) > .5:
            # 底架本身是连通框，不以它的凸包填满框内空隙；沿X和Y分段保留表面。
            selected = triangles[labels[faces[:, 0]] == index]
            centers = selected.mean(axis=1)
            bins = np.column_stack([np.floor((centers[:, 0]+.28)/.07),
                                    np.digitize(centers[:, 1], [-.08, .08])])
            for key in np.unique(bins, axis=0):
                sub = selected[np.all(bins == key, axis=1)].reshape(-1, 3)
                if len(sub) >= 4:
                    pieces.append(np.unique(sub, axis=0))
        else:
            pieces.append(part)
    return pieces, count, hashlib.sha256(raw).hexdigest()


def _parallel_axis(vector):
    return np.dot(vector, vector)*np.eye(3)-np.outer(vector, vector)


def apply_guide_rollers(spec, mode: str = "legacy", cad_path: Path = BASE_STL) -> dict:
    if mode not in MODES:
        raise ValueError("guide_rollers 必须为 legacy、fixed 或 passive")
    info = dict(mode=mode, reference_geometry=True, roller_radius_m=ROLLER_RADIUS,
                layout="two_side_transverse_axis_rows", roller_count=len(ROLLER_CENTERS),
                rows=ROLLER_ROWS, columns=ROLLER_COLUMNS,
                axis="Y横轴，前后滚动；不具备全向滚动能力",
                x_pitch_m=.550/(ROLLER_ROWS-1), y_pitch_m=.220,
                x_spacings_m=np.diff(ROLLER_X).tolist(), end_rows_x_m=[-.275, .275],
                center_clear_width_m=.200,
                roller_half_width_m=ROLLER_HALF_WIDTH, roller_mass_kg_each=ROLLER_MASS,
                roller_centers_m=[list(x) for x in ROLLER_CENTERS],
                assumption="用户确认导轮约20mm宽，仅左右两侧布置，横轴前后滚动，中间没有导轮，前后端轮护住壳角。每侧17只、16mm半径、每只8g及端轮中心±275mm为参考几何；中间200mm留空。端轮按CAD低位壳角的前后倾斜包络布置，不代表所有横滚或斜向棱角都受保护。其余尺寸/质量待实物确认。",
                friction_policy="沿用原底盘接触摩擦；固定/被动导轮几何、摩擦、质量相同，固定用关节等式约束锁定。",
                contact_timeconst_s=ROLLER_CONTACT_TIMECONST,
                contact_solmix=ROLLER_CONTACT_SOLMIX,
                contact_parameters_applied=mode != "legacy",
                contact_policy="导轮使用4ms参考接触时间常数、混合权重1000；保留原摩擦、接触阻抗和refsafe保护。避免默认软接触压入超过壳底间隙；尚无实测材料刚度标定。",
                mass_policy=f"从14kg原base_link质量及惯量中扣除全部滚轮{len(ROLLER_CENTERS)*ROLLER_MASS*1000:.0f}g，零位整车质量/重心/惯量保持原值")
    if mode == "legacy":
        return info
    body = spec.body("base_link")
    original = next(geom for geom in body.geoms if geom.type == mujoco.mjtGeom.mjGEOM_MESH)
    friction = original.friction.copy()
    # 原CAD完整保留用于显示；碰撞逐装配表面建模，车身和底板碰撞仍然存在。
    original.contype = 0
    original.conaffinity = 0
    pieces, component_count, digest = _cad_pieces(str(cad_path.resolve()), cad_path.stat().st_mtime_ns)
    added = 0
    for index, vertices in enumerate(pieces):
        extent = np.ptp(vertices, axis=0)
        center = (vertices.min(axis=0)+vertices.max(axis=0))/2
        if np.linalg.matrix_rank(vertices-center, tol=1e-7) < 3:
            # 薄板不能编译成零体积凸包，以0.5mm最小厚度保留碰撞。
            body.add_geom(name=f"guide_chassis_{index}", type=mujoco.mjtGeom.mjGEOM_BOX,
                          pos=center, size=np.maximum(extent/2, .00025), friction=friction,
                          rgba=[.5, .5, .5, 0], group=3, mass=0)
        else:
            meshname = f"guide_cad_{index}"
            spec.add_mesh(name=meshname, uservert=vertices.ravel().tolist(),
                          inertia=mujoco.mjtMeshInertia.mjMESH_INERTIA_SHELL)
            body.add_geom(name=f"guide_chassis_{index}", type=mujoco.mjtGeom.mjGEOM_MESH,
                          meshname=meshname, friction=friction, rgba=[.5, .5, .5, 0], group=3, mass=0)
        added += 1
    mass = float(body.mass)
    com = body.ipos.copy()
    xx, yy, zz, xy, xz, yz = body.fullinertia
    inertia = np.array([[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]])
    cylinder_inertia = np.array([ROLLER_MASS*(3*ROLLER_RADIUS**2+(2*ROLLER_HALF_WIDTH)**2)/12,
                                .5*ROLLER_MASS*ROLLER_RADIUS**2,
                                ROLLER_MASS*(3*ROLLER_RADIUS**2+(2*ROLLER_HALF_WIDTH)**2)/12])
    remaining_mass = mass-len(ROLLER_CENTERS)*ROLLER_MASS
    remaining_com = (mass*com-sum(ROLLER_MASS*np.array(p) for p in ROLLER_CENTERS))/remaining_mass
    remaining_inertia = inertia+mass*_parallel_axis(com)-remaining_mass*_parallel_axis(remaining_com)
    for center in ROLLER_CENTERS:
        remaining_inertia -= np.diag(cylinder_inertia)+ROLLER_MASS*_parallel_axis(np.array(center))
    if np.linalg.eigvalsh(remaining_inertia).min() <= 0:
        raise ValueError("导轮质量分配导致底盘惯量无效")
    body.mass = remaining_mass
    body.ipos = remaining_com
    body.fullinertia = [remaining_inertia[0, 0], remaining_inertia[1, 1], remaining_inertia[2, 2],
                        remaining_inertia[0, 1], remaining_inertia[0, 2], remaining_inertia[1, 2]]
    for index, center in enumerate(ROLLER_CENTERS):
        roller = body.add_body(name=f"guide_roller_{index}", pos=center, mass=ROLLER_MASS,
                               ipos=[0, 0, 0], inertia=cylinder_inertia)
        roller.add_joint(name=f"guide_roller_joint_{index}", type=mujoco.mjtJoint.mjJNT_HINGE,
                         axis=[0, 1, 0], damping=.00002 if mode == "passive" else 0,
                         frictionloss=.00001 if mode == "passive" else 0,
                         limited=False)
        if mode == "fixed":
            spec.add_equality(name=f"guide_fixed_{index}", type=mujoco.mjtEq.mjEQ_JOINT,
                              name1=f"guide_roller_joint_{index}", data=[0]*11,
                              solref=[.001, 1], solimp=[.9999, .9999, .001, .5, 2])
        roller.add_geom(name=f"guide_roller_geom_{index}", type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                        size=[ROLLER_RADIUS, ROLLER_HALF_WIDTH], quat=[math_sqrt_half(), math_sqrt_half(), 0, 0],
                        solref=[ROLLER_CONTACT_TIMECONST, 1], solmix=ROLLER_CONTACT_SOLMIX,
                        friction=friction, rgba=[1, .55, .08, 1], mass=0)
        spec.add_exclude(name=f"guide_parent_{index}", bodyname1="base_link", bodyname2=f"guide_roller_{index}")
    # 导轮追加在原关节之后；已有21/20自由度位置维持前缀。
    for key in spec.keys:
        if len(key.qpos):
            key.qpos = list(key.qpos)+[0]*len(ROLLER_CENTERS)
        if len(key.qvel):
            key.qvel = list(key.qvel)+[0]*len(ROLLER_CENTERS)
    info.update(cad_sha256=digest, cad_components=component_count, chassis_collision_pieces=added,
                added_qpos=len(ROLLER_CENTERS), added_qvel=len(ROLLER_CENTERS), added_actuators=0)
    return info


def math_sqrt_half():
    return float(np.sqrt(.5))


def register_guide_rollers(model, info):
    _info[hash(model)] = info


def guide_roller_info(model) -> dict:
    return dict(_info.get(hash(model), dict(mode="unknown")))
