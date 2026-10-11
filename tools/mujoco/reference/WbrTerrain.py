"""WBR 的真实浅坑、缓坡和直角台阶；高度只供显示与验收使用。"""
from __future__ import annotations

import math
import weakref
import hashlib
import io
import json
from itertools import combinations, product
from pathlib import Path

import mujoco
import numpy as np


# 每段为 x 起点、终点、两端高度，单位 m；中间试验道宽 1.2 m。
COURSE_SEGMENTS = (
    ("floor", -20.0, 2.0, 0.0, 0.0),
    ("terrain_pit_down", 2.0, 2.2, 0.0, -.015),
    ("terrain_pit_bottom", 2.2, 2.4, -.015, -.015),
    ("terrain_pit_up", 2.4, 2.6, -.015, 0.0),
    ("terrain_flat_after_pit", 2.6, 3.0, 0.0, 0.0),
    ("terrain_wave_up", 3.0, 3.3, 0.0, .010),
    ("terrain_wave_down", 3.3, 3.6, .010, 0.0),
    ("terrain_flat_before_step15", 3.6, 4.0, 0.0, 0.0),
    ("terrain_step15", 4.0, 4.25, .015, .015),
    ("terrain_flat_before_step20", 4.25, 5.0, 0.0, 0.0),
    ("terrain_step20", 5.0, 5.25, .020, .020),
    ("terrain_flat_before_step30", 5.25, 6.0, 0.0, 0.0),
    ("terrain_step30", 6.0, 6.25, .030, .030),
    ("terrain_finish", 6.25, 20.0, 0.0, 0.0),
)
COURSE_FINISH_X_M = 6.8
LARGE_STEPS = (("terrain_large100", 7.0, 7.5, -1.5, .10),
               ("terrain_large150", 8.0, 8.5, -1.5, .15),
               ("terrain_large200", 9.0, 9.5, -1.5, .20))
# 原 LARGE_STEPS 是 0.5 m 深的窄台。这个平台单独用于整车上台与驻车验收。
STANDARD_CLIMB_PLATFORMS = (
    ("terrain_climb200_platform", 9.0, 11.0, -18.5, -15.5, .20),
)
CROSS_SLOPES = (("terrain_cross10", 7.0, 10.0, 1.0, 2.0, 10.0),
                ("terrain_cross15", 11.0, 14.0, 1.0, 2.0, 15.0))
# 两端平顺落回侧通道，中间连接两种横坡；每段三角面本身也是碰撞面。
CROSS_TRANSITIONS = (("terrain_cross_in", 5.5, 7.0, 0.0, 10.0),
                     ("terrain_cross_between", 10.0, 11.0, 10.0, 15.0),
                     ("terrain_cross_out", 14.0, 16.0, 15.0, 0.0))
CROSS_TRANSITION_PARTS = 12
BOUNDS_M = dict(x=[-30.0, 50.0], y=[-20.0, 20.0])
RMUL_ASSET = Path(__file__).with_name("assets") / "Rmul2026.npz"
RMUL_CENTER_M = np.array([42.0, 0.0, 0.0])
RMUL_BOUNDS_M = dict(x=[35.7, 48.3], y=[-4.3, 4.3])
EXPLORATION_ZONES = (
    dict(name="playground", location="playground", label="自由平地", x_m=[-26, -6], y_m=[-17, -4],
         entry_m=[-12, -9], description="宽阔平地，可自由转弯与绕行"),
    dict(name="stairs", location="stairs", label="宽台阶与连续阶梯", x_m=[-4, 12], y_m=[-14, -5],
         entry_m=[-4, -9], description="40/60/80/100 mm宽台阶，旁侧60 mm连续阶梯"),
    dict(name="climb200", location="climb200", label="200 mm 宽平台", x_m=[6.2, 11.2], y_m=[-18.5, -15.5],
         entry_m=[6.5, -17], description="200 mm高、2 m深、3 m宽的整车上台验收平台"),
    dict(name="ramps", location="ramps", label="纵坡桥与下坡", x_m=[15, 40], y_m=[5, 14],
         entry_m=[15, 9], description="10%坡桥与20%坡道，各有上坡、平台与下坡"),
    dict(name="rough", location="rough", label="浅坑、起伏、搓板与斜块", x_m=[15, 41], y_m=[-15, -5],
         entry_m=[15, -9], description="80 mm浅坑、35 mm起伏、18 mm搓板，旁侧独立斜块"),
    dict(name="rmul2026", location="rmul2026", label="RMUL 2026 原尺寸场地",
         x_m=RMUL_BOUNDS_M["x"], y_m=RMUL_BOUNDS_M["y"], entry_m=[37, 0],
         description="原始 STEP 毫米尺寸转换，12.6×8.6 m，保留围栏、坡台和通道"),
)
_GROUND_CACHE = weakref.WeakKeyDictionary()


def cross_edge_height(degrees: float, y: float) -> float:
    return .03 + (y - 1.0) * math.tan(math.radians(degrees)) if degrees else 0.0


def cross_transition_height(x: float, y: float) -> float:
    """与实际三角面逐片一致，仅供地形显示和验收。"""
    if not 1.0 <= y <= 2.0:
        return math.nan
    for _, x0, x1, a0, a1 in CROSS_TRANSITIONS:
        if not x0 <= x <= x1:
            continue
        grid_x = (x - x0) / (x1 - x0) * CROSS_TRANSITION_PARTS
        part = min(int(grid_x), CROSS_TRANSITION_PARTS - 1)
        u, v = grid_x - part, y - 1.0
        heights = []
        for t in (part / CROSS_TRANSITION_PARTS, (part + 1) / CROSS_TRANSITION_PARTS):
            blend = t * t * (3.0 - 2.0 * t)
            heights.append([(1-blend) * cross_edge_height(a0, edge) +
                            blend * cross_edge_height(a1, edge) for edge in (1.0, 2.0)])
        z00, z01 = heights[0]
        z10, z11 = heights[1]
        return ((1-u)*z00 + (u-v)*z10 + v*z11 if v <= u else
                (1-v)*z00 + u*z11 + (v-u)*z01)
    return math.nan


def apply_terrain(spec: mujoco.MjSpec, terrain: str = "flat") -> None:
    """flat 不修改模型；course 替换无限平面，确保浅坑没有被平面填平。"""
    if terrain not in ("flat", "course"):
        raise ValueError("terrain 必须为 flat 或 course")
    if terrain == "flat":
        return
    floor = spec.geom("floor")
    properties = {name: getattr(floor, name) for name in
                  ("material", "contype", "conaffinity", "condim", "priority", "solmix",
                   "margin", "gap", "group")}
    for name in ("friction", "solref", "solimp", "rgba"):
        properties[name] = np.array(getattr(floor, name), copy=True)
    spec.delete(floor)
    thickness = .30
    def surface(name):
        visual = dict(properties)
        color = ([.15, .48, .85, 1] if "pit" in name and "flat" not in name else
                 [.55, .20, .10, 1] if "large" in name else
                 [1.0, .48, .12, 1] if "step" in name and "flat" not in name else
                 [.16, .58, .28, 1] if "cross" in name else
                [.55, .65, .30, 1] if "wave" in name else None)
        if color is not None:
            visual.update(material="", rgba=color)
        return visual
    for name, x0, x1, z0, z1 in COURSE_SEGMENTS:
        angle = -math.atan2(z1 - z0, x1 - x0)
        sine, cosine = math.sin(angle), math.cos(angle)
        spec.worldbody.add_geom(name=name, type=mujoco.mjtGeom.mjGEOM_BOX,
            size=[(x1 - x0) / (2 * cosine), .6, thickness / 2],
            pos=[(x0 + x1) / 2 - thickness / 2 * sine, 0.0,
                 (z0 + z1) / 2 - thickness / 2 * cosine],
            quat=[math.cos(angle / 2), 0.0, math.sin(angle / 2), 0.0], **surface(name))
    # 两侧留独立的平地通道；中间浅坑仍保持真实凹陷。
    for name, y in (("terrain_side_left", 1.8), ("terrain_side_right", -1.8)):
        spec.worldbody.add_geom(name=name, type=mujoco.mjtGeom.mjGEOM_BOX,
            size=[20.0, 1.2, thickness / 2], pos=[0.0, y, -thickness / 2],
            **surface(name))
    for name, x0, x1, y, height in LARGE_STEPS:
        spec.worldbody.add_geom(name=name, type=mujoco.mjtGeom.mjGEOM_BOX,
            size=[(x1 - x0) / 2, .4, (height + thickness) / 2],
            pos=[(x0 + x1) / 2, y, (height - thickness) / 2], **surface(name))
    for name, x0, x1, y0, y1, degrees in CROSS_SLOPES:
        angle = math.radians(degrees)
        sine, cosine = math.sin(angle), math.cos(angle)
        middle_height = .03 + (y1 - y0) / 2 * math.tan(angle)
        spec.worldbody.add_geom(name=name, type=mujoco.mjtGeom.mjGEOM_BOX,
            size=[(x1 - x0) / 2, (y1 - y0) / (2 * cosine), thickness / 2],
            pos=[(x0 + x1) / 2, (y0 + y1) / 2 + thickness / 2 * sine,
                 middle_height - thickness / 2 * cosine],
            quat=[math.cos(angle / 2), math.sin(angle / 2), 0.0, 0.0], **surface(name))
    faces = [0, 1, 2, 5, 4, 3, 0, 3, 4, 0, 4, 1,
             1, 4, 5, 1, 5, 2, 2, 5, 3, 2, 3, 0]
    for name, x0, x1, a0, a1 in CROSS_TRANSITIONS:
        for part in range(CROSS_TRANSITION_PARTS):
            corners = []
            for t, y in ((part / CROSS_TRANSITION_PARTS, 1.0),
                         ((part + 1) / CROSS_TRANSITION_PARTS, 1.0),
                         ((part + 1) / CROSS_TRANSITION_PARTS, 2.0),
                         (part / CROSS_TRANSITION_PARTS, 2.0)):
                blend = t * t * (3.0 - 2.0 * t)
                z = (1-blend) * cross_edge_height(a0, y) + blend * cross_edge_height(a1, y)
                corners.append([x0 + (x1-x0)*t, y, z])
            for half, indices in enumerate(((0, 1, 2), (0, 2, 3))):
                mesh_name = f"{name}_{part:02d}_{half}"
                top = [corners[index] for index in indices]
                vertices = top + [[v[0], v[1], -thickness] for v in top]
                spec.add_mesh(name=mesh_name, uservert=np.asarray(vertices).ravel().tolist(), userface=faces)
                spec.worldbody.add_geom(name=mesh_name, type=mujoco.mjtGeom.mjGEOM_MESH,
                    meshname=mesh_name, **surface(name))
    # 外圈四块地面不覆盖旧中心道，保留浅坑与原试验的碰撞、摩擦和坐标。
    def box(name, x0, x1, y0, y1, z0=0.0, z1=0.0, color=None, roll=0.0):
        angle = -math.atan2(z1 - z0, x1 - x0)
        if roll:
            angle = roll
            normal = np.array([0.0, -math.sin(angle), math.cos(angle)])
            quat = [math.cos(angle / 2), math.sin(angle / 2), 0.0, 0.0]
            size = [(x1-x0)/2, (y1-y0)/(2*math.cos(angle)), thickness/2]
        else:
            normal = np.array([math.sin(angle), 0.0, math.cos(angle)])
            quat = [math.cos(angle / 2), 0.0, math.sin(angle / 2), 0.0]
            size = [(x1-x0)/(2*math.cos(angle)), (y1-y0)/2, thickness/2]
        center = np.array([(x0+x1)/2, (y0+y1)/2, (z0+z1)/2]) - normal*thickness/2
        visual = surface(name)
        if color is not None:
            visual.update(material="", rgba=color)
        spec.worldbody.add_geom(name=name, type=mujoco.mjtGeom.mjGEOM_BOX,
            size=size, pos=center, quat=quat, **visual)

    box("terrain_outer_west", -30, -20, -20, 20)
    box("terrain_outer_east", 20, 50, -20, 20)
    box("terrain_outer_north", -20, 20, 3, 20)
    # 分块绕开新增浅坑；坑里不铺任何隐藏的零高地板。
    box("terrain_outer_south_west", -20, 19, -20, -3)
    box("terrain_outer_south_lower", 19, 20, -20, -10.5)
    box("terrain_outer_south_upper", 19, 20, -7.5, -3)
    # 东侧地面也留出同一浅坑的位置。
    spec.delete(spec.geom("terrain_outer_east"))
    # CAD 区域不铺旧零高地板，薄壳下的低地面和通道按源模型保留。
    box("terrain_outer_east_far", 23, 35.7, -20, 20)
    box("terrain_outer_rmul_east", 48.3, 50, -20, 20)
    box("terrain_outer_rmul_lower", 35.7, 48.3, -20, -4.3)
    box("terrain_outer_rmul_upper", 35.7, 48.3, 4.3, 20)
    box("terrain_outer_east_lower", 20, 23, -20, -10.5)
    box("terrain_outer_east_upper", 20, 23, -7.5, 20)
    orange = [1.0, .45, .10, 1]
    for index, height in enumerate((.04, .06, .08, .10)):
        x0 = -1 + index*2.5
        box(f"terrain_wide_step{int(height*1000)}", x0, x0+1.2, -10.5, -7.5, height, height, orange)
    # 与中心道 0.5 m 深窄台分开命名和布置，避免把越过台尾后的下落误判为上台失败。
    for name, x0, x1, y0, y1, height in STANDARD_CLIMB_PLATFORMS:
        box(name, x0, x1, y0, y1, height, height, [.72, .30, .82, 1])
    # 旁边独立的四级阶梯含下降段，每级60 mm，宽3 m。
    for index, height in enumerate((.06, .12, .18, .24, .24, .18, .12, .06)):
        box(f"terrain_stairs_{index}", index, index+1, -14, -11, height, height,
            [1.0, .62, .18, 1])
    green = [.12, .65, .42, 1]
    for index, (x0, x1, z0, z1) in enumerate(((18, 23, 0, .5), (23, 27, .5, .5), (27, 32, .5, 0))):
        box(f"terrain_bridge_{index}", x0, x1, 7.5, 10.5, z0, z1, green)
    for index, (x0, x1, z0, z1) in enumerate(((18, 22, 0, .8), (22, 24, .8, .8), (24, 28, .8, 0))):
        box(f"terrain_steep_ramp_{index}", x0, x1, 11, 14, z0, z1, [.15, .45, .7, 1])
    for index, (x0, x1, z0, z1) in enumerate(((19, 20, 0, -.08), (20, 22, -.08, -.08), (22, 23, -.08, 0))):
        box(f"terrain_explore_pit_{index}", x0, x1, -10.5, -7.5, z0, z1, [.12, .40, .85, 1])
    for index in range(12):
        x0 = 25 + index*.5
        z0, z1 = (0.0, .035) if index % 2 == 0 else (.035, 0.0)
        box(f"terrain_wave_explore_{index}", x0, x0+.5, -10.5, -7.5, z0, z1, [.65, .72, .20, 1])
    for index in range(18):
        x0 = 33 + index*.3
        box(f"terrain_washboard_{index}", x0, x0+.10, -10.5, -7.5, .018, .018, [.75, .48, .25, 1])
    for index in range(20):
        x0 = 18 + index*.9
        y0 = -14.5 + (index % 3)*.8
        height = .04 + (index % 4)*.012
        box(f"terrain_tilt_block_{index}", x0, x0+.45, y0, y0+.55, height, height,
            [.48, .44, .40, 1], roll=math.radians(8 if index % 2 else -8))
    _apply_rmul(spec, surface)


def _exact_box(points):
    """只把八个真实角点组成的长方体换成原生盒体，不近似其他凸块。"""
    if points.shape != (8, 3):
        return None
    vectors = points[1:] - points[0]
    for indices in combinations(range(7), 3):
        edges = vectors[list(indices)]
        lengths = np.linalg.norm(edges, axis=1)
        if (lengths <= 1e-10).any():
            continue
        axes = edges / lengths[:, None]
        if np.max(np.abs(axes @ axes.T - np.eye(3))) > 1e-10:
            continue
        corners = points[0] + np.asarray(list(product((0, 1), repeat=3))) @ edges
        if np.max(np.min(np.linalg.norm(corners[:, None]-points[None, :], axis=2), axis=1)) > 1e-9:
            continue
        rotation = axes.T.copy()
        if np.linalg.det(rotation) < 0:
            rotation[:, 2] *= -1
        quaternion = np.empty(4)
        mujoco.mju_mat2Quat(quaternion, rotation.ravel())
        return points.mean(axis=0), lengths / 2, quaternion
    return None


def _apply_rmul(spec: mujoco.MjSpec, surface) -> None:
    """加载经 STEP 逐面分解的凸薄棱柱；凹面孔洞不能用整场地凸包代替。"""
    if not RMUL_ASSET.is_file():
        raise FileNotFoundError(f"缺少 CAD 地形资产：{RMUL_ASSET}；请运行 ConvertWbrStep.py")
    payload = RMUL_ASSET.read_bytes()
    metadata = json.loads(RMUL_ASSET.with_suffix(".json").read_text(encoding="utf-8"))
    metadata.update(asset_name=RMUL_ASSET.name, asset_sha256=hashlib.sha256(payload).hexdigest(),
        world_center_m=RMUL_CENTER_M.tolist(),
        world_bounds_m=dict(x=RMUL_BOUNDS_M["x"], y=RMUL_BOUNDS_M["y"],
            z=[metadata["local_bounds_m"][0][2], metadata["local_bounds_m"][1][2]]))
    # 把加载时的身份写进编译模型，之后文件更新不会篡改旧模型的记录元数据。
    representation = dict(method="exact_cuboid_or_original_convex_prism", boxes=0, mesh_prisms=0,
        corner_tolerance_m=1e-9, orthogonality_tolerance=1e-10, merged_source_parts=[],
        static_body_grid_m=2.0, independent_large_extent_m=4.0)
    bodies = {}
    with np.load(io.BytesIO(payload), allow_pickle=False) as asset:
        vertices, offsets = asset["vertices"], asset["offsets"]
        parts = [vertices[start:end] for start, end in zip(offsets[:-1], offsets[1:])]
        shared_edges, merged, replacements = {}, set(), {}
        for index, points in enumerate(parts):
            if points.shape != (6, 3):
                continue
            for a, b in combinations(range(3), 2):
                edge = tuple(sorted(tuple(np.round(points[k], 10)) for k in (a, b, a+3, b+3)))
                for previous in shared_edges.get(edge, []):
                    if previous in merged or index in merged:
                        continue
                    union = np.concatenate((parts[previous], points))
                    _, unique = np.unique(np.round(union, 10), axis=0, return_index=True)
                    union = union[unique]
                    box = _exact_box(union)
                    if box is None:
                        continue
                    # 共用完整侧面、八角点、体积相等三项一起保证并集为原矩形，不能补洞。
                    volume = sum(abs(np.dot(np.cross(p[1]-p[0], p[2]-p[0]), p[3]-p[0]))/2
                                 for p in (parts[previous], points))
                    if not math.isclose(volume, 8*np.prod(box[1]), rel_tol=1e-10, abs_tol=1e-12):
                        continue
                    merged.update((previous, index))
                    replacements[previous] = union
                    representation["merged_source_parts"].append([previous, index])
                shared_edges.setdefault(edge, []).append(index)
        for index, original_points in enumerate(parts):
            if index in merged and index not in replacements:
                continue
            points = replacements.get(index, original_points)
            center = points.mean(axis=0)
            name = f"terrain_rmul_{index:04d}"
            # 每个棱柱是独立凸块；仅由它自身生成凸包，面外的通道仍为空。
            visual = surface(name)
            height = float(points[:, 2].max())
            vertical = float(np.ptp(points[:, 2])) > .05 and height > .1
            color = ([.65, .68, .72, 1] if vertical else [.85, .48, .18, 1] if height > .15 else
                     [.27, .63, .45, 1] if height > .005 else [.25, .34, .43, 1])
            visual.update(material="", rgba=color)
            # 将静态候选按空间分组；局部原点只改坐标表达，不增加运动关节。
            key = tuple(np.floor((center+RMUL_CENTER_M)[:2]/2).astype(int))
            if np.max(np.ptp(points, axis=0)) > 4.0:
                key = ("large", index)
            if key not in bodies:
                bodies[key] = spec.worldbody.add_body(name=f"rmul_cell_{key[0]}_{key[1]}",
                    pos=center+RMUL_CENTER_M if key[0] == "large" else
                        [(key[0]+.5)*2, (key[1]+.5)*2, 0])
            box = _exact_box(points)
            if box is not None:
                center, size, quaternion = box
                bodies[key].add_geom(name=name, type=mujoco.mjtGeom.mjGEOM_BOX,
                    pos=center+RMUL_CENTER_M-bodies[key].pos, size=size, quat=quaternion, **visual)
                representation["boxes"] += 1
            else:
                spec.add_mesh(name=name, uservert=(points-center).ravel().tolist())
                bodies[key].add_geom(name=name, type=mujoco.mjtGeom.mjGEOM_MESH,
                    meshname=name, pos=center+RMUL_CENTER_M-bodies[key].pos, **visual)
                representation["mesh_prisms"] += 1
    metadata["collision_representation"] = representation
    spec.add_text(name="terrain_rmul_metadata", data=json.dumps(metadata, ensure_ascii=True))


def terrain_info(model: mujoco.MjModel) -> dict:
    """返回界面可直接显示的坐标约定；大台阶不属于已验收的小地形路线。"""
    course = len(ground_geom_ids(model)) > 1
    metadata_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TEXT, "terrain_rmul_metadata")
    imported = None
    if metadata_id >= 0:
        start, size = model.text_adr[metadata_id], model.text_size[metadata_id]
        imported = json.loads(model.text_data[start:start+size-1].decode("utf-8"))
    return dict(kind="course" if course else "flat", forward_axis="+x",
        width_y_m=BOUNDS_M["y"] if course else None,
        bounds_m=BOUNDS_M if course else None,
        zones=list(EXPLORATION_ZONES) if course else [],
        imported_course=imported,
        finish_x_m=COURSE_FINISH_X_M if course else None,
        main_course=[dict(name=name, x_m=[x0, x1], z_m=[z0, z1])
                     for name, x0, x1, z0, z1 in COURSE_SEGMENTS] if course else [],
        large_steps=[dict(name=name, x_m=[x0, x1], y_m=[y - .4, y + .4], height_m=height)
                     for name, x0, x1, y, height in LARGE_STEPS] if course else [],
        standard_climb_platforms=[dict(name=name, x_m=[x0, x1], y_m=[y0, y1],
                                       height_m=height)
                                  for name, x0, x1, y0, y1, height
                                  in STANDARD_CLIMB_PLATFORMS] if course else [],
        cross_slopes=[dict(name=name, x_m=[x0, x1], y_m=[y0, y1], angle_deg=degrees,
                          low_edge_height_m=.03, uphill_axis="+y")
                      for name, x0, x1, y0, y1, degrees in CROSS_SLOPES] if course else [],
        cross_transitions=[dict(name=name, x_m=[x0, x1], y_m=[1, 2],
                                angles_deg=[a0, a1])
                           for name, x0, x1, a0, a1 in CROSS_TRANSITIONS] if course else [])


def scenario_pose(model: mujoco.MjModel, qpos, location: str = "start") -> np.ndarray:
    """变换传入姿态；横坡须传 profile 站姿，起点/台阶入口可传 home。

    横坡按完整刚体旋转及平移初始化，仅用于试验起点，不是行驶控制或瞬移。
    """
    pose = np.array(qpos, dtype=float, copy=True)
    if pose.shape != (model.nq,) or not np.isfinite(pose).all():
        raise ValueError("qpos 尺寸错误或含非有限值")
    if location == "start":
        return pose
    if len(ground_geom_ids(model)) <= 1:
        raise ValueError("测试位置需要 course 地形")
    zone_location = "climb200" if location == "step_jump200" else location
    zone = next((zone for zone in EXPLORATION_ZONES if zone["location"] == zone_location), None)
    if zone is not None:
        pose[:2] += [4.5, -17.0] if location == "step_jump200" else zone["entry_m"]
        pose[2] += ground_height(model, *pose[:2])
        # 原 home 有少量预穿入；新入口按实际接触抬高，避免重置就埋进地面。
        data = mujoco.MjData(model)
        data.qpos[:] = pose
        mujoco.mj_forward(model, data)
        grounds = ground_geom_ids(model)
        depth = min((float(contact.dist) for contact in data.contact
            if (contact.geom1 in grounds) != (contact.geom2 in grounds)), default=0.0)
        if depth < 0.0:
            pose[2] += .001-depth
        return pose
    if location in ("large_steps", "step_climb", "step_auto"):
        # 右侧平地通道留出一米起身与接近距离。
        pose[:2] += np.array([6.0, -1.5])
        return pose
    if location == "slope_course":
        pose[:2] += np.array([4.5, 1.5])
        return pose
    selected = next((slope for slope in CROSS_SLOPES if location == slope[0].removeprefix("terrain_")), None)
    if selected is None:
        raise ValueError("未知 location；支持原试验入口及 playground、stairs、ramps、rough、rmul2026")
    _, x0, x1, y0, y1, degrees = selected
    angle = math.radians(degrees)
    quat = np.array([math.cos(angle / 2), math.sin(angle / 2), 0.0, 0.0])
    rotation = np.empty(9)
    mujoco.mju_quat2Mat(rotation, quat)
    center_x, center_y = (x0 + x1) / 2, (y0 + y1) / 2
    pose[:3] = rotation.reshape(3, 3) @ pose[:3] + np.array(
        [center_x, center_y, ground_height(model, center_x, center_y)])
    original = pose[3:7].copy()
    mujoco.mju_mulQuat(pose[3:7], quat, original)
    return pose


def ground_geom_ids(model: mujoco.MjModel) -> frozenset[int]:
    """统一返回原 floor 和全部 terrain_ 地形，接触检查不能只认 floor。"""
    return _ground_index(model)["ids"]


def _ground_index(model: mujoco.MjModel) -> dict:
    """只缓存本模型的固定地形；弱引用不延长模型寿命，也不按地址复用旧几何。"""
    cached = _GROUND_CACHE.get(model)
    if cached is not None:
        return cached
    ids = frozenset(index for index in range(model.ngeom)
        if model.geom(index).name == "floor" or model.geom(index).name.startswith("terrain_"))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    boxes, triangles, planes = [], [], []
    for index in ids:
        kind = model.geom_type[index]
        center = data.geom_xpos[index].copy()
        rotation = data.geom_xmat[index].reshape(3, 3).copy()
        if kind == mujoco.mjtGeom.mjGEOM_BOX:
            boxes.append((center, rotation.T, model.geom_size[index].copy()))
        elif kind == mujoco.mjtGeom.mjGEOM_PLANE:
            planes.append((center, rotation[:, 2]))
        elif kind == mujoco.mjtGeom.mjGEOM_MESH:
            mesh = model.geom_dataid[index]
            start, count = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
            vertices = model.mesh_vert[start:start+count] @ rotation.T + center
            start, count = model.mesh_faceadr[mesh], model.mesh_facenum[mesh]
            triangles.extend(vertices[model.mesh_face[start:start+count]])
    # 每次查询只做数组运算，不遍历机器人几何、不重新算旋转。
    cached = dict(ids=ids, planes=planes,
        centers=np.asarray([box[0] for box in boxes]).reshape(-1, 3),
        inverse=np.asarray([box[1] for box in boxes]).reshape(-1, 3, 3),
        sizes=np.asarray([box[2] for box in boxes]).reshape(-1, 3),
        triangles=np.asarray(triangles).reshape(-1, 3, 3))
    centers, inverse, sizes = cached["centers"], cached["inverse"], cached["sizes"]
    extent = np.einsum("nij,nj->ni", np.abs(inverse.transpose(0, 2, 1)), sizes)
    cached["box_grid"] = _projection_grid(centers[:, :2]-extent[:, :2], centers[:, :2]+extent[:, :2])
    triangles = cached["triangles"]
    cached["triangle_grid"] = _projection_grid(triangles[:, :, :2].min(axis=1), triangles[:, :, :2].max(axis=1))
    _GROUND_CACHE[model] = cached
    return cached


def _projection_grid(lower, upper) -> dict:
    """按一米网格缓存投影候选，CAD 场地查询不遍历全部数万三角面。"""
    cells = {}
    for index, (start, end) in enumerate(zip(lower, upper)):
        # 浮点边界保留邻格，最终仍由精确求交排除；不会补出假地板。
        minimum = np.floor(start-1e-8).astype(int)
        maximum = np.floor(end+1e-8).astype(int)
        for x in range(minimum[0], maximum[0]+1):
            for y in range(minimum[1], maximum[1]+1):
                cells.setdefault((x, y), []).append(index)
    return {cell: np.asarray(indices, dtype=int) for cell, indices in cells.items()}


def ground_height(model: mujoco.MjModel, x: float, y: float) -> float:
    """返回坐标处的真实地表高度；有限场地外为 NaN，不用于控制反馈。

    台阶边界取两侧较高表面。盒体用竖直射线求交，斜块侧面也计入真实碰撞高度。
    """
    if not math.isfinite(x) or not math.isfinite(y):
        raise ValueError("地表坐标必须有限")
    cache = _ground_index(model)
    heights = []
    cell = (math.floor(x), math.floor(y))
    boxes = cache["box_grid"].get(cell, np.empty(0, dtype=int))
    centers, inverse, sizes = cache["centers"][boxes], cache["inverse"][boxes], cache["sizes"][boxes]
    if len(centers):
        origin = np.einsum("nij,nj->ni", inverse, np.array([x, y, 0.0]) - centers)
        direction = inverse[:, :, 2]
        parallel = np.abs(direction) < 1e-12
        safe_direction = np.where(parallel, 1.0, direction)
        a, b = (-sizes-origin)/safe_direction, (sizes-origin)/safe_direction
        low = np.where(parallel, -np.inf, np.minimum(a, b))
        high = np.where(parallel, np.inf, np.maximum(a, b))
        lower, upper = low.max(axis=1), high.min(axis=1)
        hit = (~(parallel & (np.abs(origin) > sizes+1e-9)).any(axis=1)) & (lower <= upper+1e-9)
        if hit.any():
            heights.append(float(upper[hit].max()))
    selected = cache["triangle_grid"].get(cell, np.empty(0, dtype=int))
    triangles = cache["triangles"][selected]
    if len(triangles):
        a = triangles[:, 0]
        u, v = triangles[:, 1]-a, triangles[:, 2]-a
        det = u[:, 0]*v[:, 1] - u[:, 1]*v[:, 0]
        valid = np.abs(det) > 1e-12
        safe_det = np.where(valid, det, 1.0)
        dx, dy = x-a[:, 0], y-a[:, 1]
        s = (dx*v[:, 1]-dy*v[:, 0])/safe_det
        t = (u[:, 0]*dy-u[:, 1]*dx)/safe_det
        hit = valid & (s >= -1e-8) & (t >= -1e-8) & (s+t <= 1+1e-8)
        if hit.any():
            heights.append(float((a[:, 2]+s*u[:, 2]+t*v[:, 2])[hit].max()))
    for center, normal in cache["planes"]:
        if abs(normal[2]) > 1e-12:
            heights.append(float(center[2]-(normal[0]*(x-center[0])+normal[1]*(y-center[1]))/normal[2]))
    return max(heights) if heights else math.nan
