"""把会填平孔洞的连杆凸包替换成原 STL 实形的凸片组合。"""
from __future__ import annotations

from functools import lru_cache
import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np


DATA_PATH = Path(__file__).with_name("assets") / "WbrLinkCollision.json"
_info: dict[int, dict] = {}


@lru_cache(maxsize=1)
def _load_data() -> dict:
    data = json.loads(DATA_PATH.read_text(encoding="utf-8"))
    if data.get("format_version") != 1:
        raise ValueError("连杆碰撞细化数据版本不受支持")
    if set(data.get("bodies", {})) != {"llink4_child1", "rlink4_child1"}:
        raise ValueError("连杆碰撞细化数据缺少左右侧目标")
    return data


def _prism_vertices(polygon_xz, y_bounds) -> np.ndarray:
    polygon = np.asarray(polygon_xz, dtype=float)
    if polygon.ndim != 2 or polygon.shape[0] < 3 or polygon.shape[1] != 2:
        raise ValueError("连杆碰撞凸片顶点格式无效")
    y_min, y_max = map(float, y_bounds)
    if not y_max > y_min:
        raise ValueError("连杆碰撞凸片厚度无效")
    return np.vstack((
        np.column_stack((polygon[:, 0], np.full(len(polygon), y_min), polygon[:, 1])),
        np.column_stack((polygon[:, 0], np.full(len(polygon), y_max), polygon[:, 1])),
    ))


def apply_link_collision_refinement(spec: mujoco.MjSpec,
                                    model_path: str | Path) -> dict:
    """细化左右 link4_child1；原网格继续显示，质量和惯量由原 body 保持。"""
    data = _load_data()
    mesh_directory = Path(model_path).resolve().parent / "meshes"
    info = {
        "enabled": True,
        "method": data["method"],
        "data_file": DATA_PATH.name,
        "refined_bodies": [],
        "added_collision_geoms": 0,
        "sources": {},
    }
    for body_name, body_data in data["bodies"].items():
        source = mesh_directory / body_data["mesh_filename"]
        if not source.is_file():
            raise FileNotFoundError(f"找不到连杆碰撞源网格：{source}")
        actual_digest = hashlib.sha256(source.read_bytes()).hexdigest().upper()
        expected_digest = body_data["source_sha256"].upper()
        if actual_digest != expected_digest:
            raise ValueError(
                f"{body_name} STL 已变化，不能套用旧凸分解："
                f"期望 {expected_digest}，实际 {actual_digest}")

        body = spec.body(body_name)
        candidates = [geom for geom in body.geoms
                      if geom.type == mujoco.mjtGeom.mjGEOM_MESH and
                      geom.meshname == body_name]
        if len(candidates) != 1:
            raise ValueError(f"{body_name} 应恰好有一个原始显示/碰撞网格")
        original = candidates[0]
        collision = {
            "contype": int(original.contype),
            "conaffinity": int(original.conaffinity),
            "condim": int(original.condim),
            "priority": int(original.priority),
            "friction": np.asarray(original.friction).copy(),
            "solmix": float(original.solmix),
            "solref": np.asarray(original.solref).copy(),
            "solimp": np.asarray(original.solimp).copy(),
            "margin": float(original.margin),
            "gap": float(original.gap),
        }
        # 原网格只关闭碰撞，mesh、材质、颜色和 body 惯性参数均不改。
        original.contype = 0
        original.conaffinity = 0
        pieces = body_data["pieces_xz_m"]
        for index, polygon in enumerate(pieces):
            mesh_name = f"refined_{body_name}_{index:02d}"
            vertices = _prism_vertices(polygon, body_data["y_bounds_m"])
            spec.add_mesh(name=mesh_name, uservert=vertices.reshape(-1))
            body.add_geom(
                name=mesh_name,
                type=mujoco.mjtGeom.mjGEOM_MESH,
                meshname=mesh_name,
                mass=0,
                rgba=[0, 0, 0, 0],
                group=3,
                **collision,
            )
        info["refined_bodies"].append(body_name)
        info["added_collision_geoms"] += len(pieces)
        info["sources"][body_name] = {
            "mesh_filename": source.name,
            "sha256": actual_digest,
            "convex_piece_count": len(pieces),
            "source_volume_m3": body_data["source_volume_m3"],
            "section_area_m2": body_data["section_area_m2"],
            "section_hole_count": body_data["section_hole_count"],
        }
    return info


def register_link_collision(model: mujoco.MjModel, info: dict) -> None:
    _info[hash(model)] = dict(info)


def link_collision_info(model: mujoco.MjModel) -> dict:
    return dict(_info.get(hash(model), {"enabled": False}))
