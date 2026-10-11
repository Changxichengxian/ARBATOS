"""验证 link4_child1 碰撞细化没有填孔，也没有改质量、惯量和显示网格。"""
from __future__ import annotations

import json
from pathlib import Path

import mujoco
import numpy as np

from WbrGasSpring import load_model
from WbrLinkCollision import DATA_PATH, link_collision_info


FALSE_POINTS = {
    "llink4_child1": [-0.06394833751394019, -0.02050000056624452, -0.07466495381716515],
    "rlink4_child1": [-0.06486586645035686, -0.008499999647028631, -0.07543889707464736],
}
SURFACE_POINTS = {
    "llink4_child1": [-0.0694335600165768, -0.020500000566244125, -0.06973730827475697],
    "rlink4_child1": [-0.07022809297695838, -0.008499999530613422, -0.07062174487553602],
}


def polygon_area(polygon) -> float:
    points = np.asarray(polygon, dtype=float)
    return float(abs(np.dot(points[:, 0], np.roll(points[:, 1], -1)) -
                     np.dot(points[:, 1], np.roll(points[:, 0], -1))) * .5)


def polygon_contains(polygon, point, tolerance=1e-10) -> bool:
    vertices = np.asarray(polygon, dtype=float)
    point = np.asarray(point, dtype=float)
    edges = np.roll(vertices, -1, axis=0) - vertices
    relative = point - vertices
    cross = edges[:, 0] * relative[:, 1] - edges[:, 1] * relative[:, 0]
    return bool(np.all(cross >= -tolerance) or np.all(cross <= tolerance))


def body_geom_ids(model, body_name):
    body_id = model.body(body_name).id
    return [index for index in range(model.ngeom)
            if int(model.geom_bodyid[index]) == body_id]


def main() -> int:
    data = json.loads(DATA_PATH.read_text(encoding="utf-8"))
    legacy = load_model(enabled=False, correct_link_collisions=False)
    refined = load_model(enabled=False, correct_link_collisions=True)
    info = link_collision_info(refined)
    checks = {}
    passed = True
    for body_name, body_data in data["bodies"].items():
        polygons = body_data["pieces_xz_m"]
        y_min, y_max = body_data["y_bounds_m"]
        reconstructed_volume = sum(polygon_area(polygon) for polygon in polygons) * (y_max-y_min)
        false_point = np.asarray(FALSE_POINTS[body_name], dtype=float)
        surface_point = np.asarray(SURFACE_POINTS[body_name], dtype=float)
        false_inside = any(polygon_contains(polygon, false_point[[0, 2]]) for polygon in polygons)
        surface_inside = any(polygon_contains(polygon, surface_point[[0, 2]]) for polygon in polygons)

        old_ids = body_geom_ids(legacy, body_name)
        new_ids = body_geom_ids(refined, body_name)
        old_original = next(index for index in old_ids
                            if legacy.geom_dataid[index] == legacy.mesh(body_name).id)
        new_original = next(index for index in new_ids
                            if refined.geom_dataid[index] == refined.mesh(body_name).id)
        piece_ids = [index for index in new_ids
                     if refined.geom(index).name.startswith(f"refined_{body_name}_")]
        body_old = legacy.body(body_name).id
        body_new = refined.body(body_name).id
        body_unchanged = bool(
            legacy.body_mass[body_old] == refined.body_mass[body_new] and
            np.array_equal(legacy.body_inertia[body_old], refined.body_inertia[body_new]) and
            np.array_equal(legacy.body_ipos[body_old], refined.body_ipos[body_new]))
        display_unchanged = bool(
            legacy.geom_dataid[old_original] == refined.geom_dataid[new_original] and
            np.array_equal(legacy.geom_rgba[old_original], refined.geom_rgba[new_original]))
        contact_parameters_preserved = bool(all(
            refined.geom_condim[index] == legacy.geom_condim[old_original] and
            refined.geom_priority[index] == legacy.geom_priority[old_original] and
            np.array_equal(refined.geom_friction[index], legacy.geom_friction[old_original]) and
            np.array_equal(refined.geom_solref[index], legacy.geom_solref[old_original]) and
            np.array_equal(refined.geom_solimp[index], legacy.geom_solimp[old_original])
            for index in piece_ids))
        item_passed = bool(
            not false_inside and surface_inside and
            abs(reconstructed_volume-body_data["source_volume_m3"]) < 1e-11 and
            len(piece_ids) == len(polygons) == 37 and
            refined.geom_contype[new_original] == 0 and
            refined.geom_conaffinity[new_original] == 0 and
            body_unchanged and display_unchanged and contact_parameters_preserved)
        passed &= item_passed
        checks[body_name] = {
            "passed": item_passed,
            "false_hull_point_inside_refined_collision": false_inside,
            "real_surface_point_inside_refined_collision": surface_inside,
            "piece_count": len(piece_ids),
            "reconstructed_volume_m3": reconstructed_volume,
            "source_volume_m3": body_data["source_volume_m3"],
            "volume_error_m3": abs(reconstructed_volume-body_data["source_volume_m3"]),
            "body_mass_inertia_unchanged": body_unchanged,
            "display_mesh_and_rgba_unchanged": display_unchanged,
            "contact_parameters_preserved": contact_parameters_preserved,
        }
    passed &= bool(
        info["enabled"] and info["added_collision_geoms"] == 74 and
        refined.ngeom == legacy.ngeom + 74 and
        np.array_equal(legacy.qpos0, refined.qpos0))
    print(json.dumps({
        "passed": passed,
        "legacy_ngeom": legacy.ngeom,
        "refined_ngeom": refined.ngeom,
        "added_collision_geoms": refined.ngeom-legacy.ngeom,
        "metadata": info,
        "checks": checks,
    }, ensure_ascii=False, indent=2))
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
