"""核对 STEP 导入场地的真实碰撞、入口、尺寸与截面，并保存实际渲染。"""
from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
import mujoco
from PIL import Image

from WbrTerrain import RMUL_CENTER_M, apply_terrain, ground_geom_ids, ground_height, terrain_info


ASSET = Path(__file__).with_name("assets") / "Rmul2026.npz"


def source_preview(output: Path) -> None:
    with np.load(ASSET) as asset:
        triangles = asset["source_triangles"]
    figure = plt.figure(figsize=(13, 8))
    axis = figure.add_subplot(111, projection="3d")
    z = triangles[:, :, 2].mean(axis=1)
    colors = plt.cm.terrain((z+.1)/.52)
    axis.add_collection3d(Poly3DCollection(triangles, facecolors=colors, linewidths=.15,
        edgecolors=(.1, .15, .2, .3)))
    axis.set(xlim=(-6.4, 6.4), ylim=(-4.4, 4.4), zlim=(-.1, .6),
        xlabel="CAD X / 1000 (m)", ylabel="CAD Z / 1000 (m)", zlabel="-CAD Y / 1000 (m)")
    axis.set_box_aspect((12.6, 8.6, 1.7))
    axis.view_init(elev=45, azim=-55)
    axis.set_title("RMUL2026 STEP: 12.6 x 8.6 m; floor z=0; max obstacle z=0.42 m")
    figure.tight_layout()
    figure.savefig(output / "cad-source-preview.png", dpi=140)
    plt.close(figure)


def source_height(triangles, x, y):
    a = triangles[:, 0]
    u, v = triangles[:, 1]-a, triangles[:, 2]-a
    det = u[:, 0]*v[:, 1]-u[:, 1]*v[:, 0]
    valid = np.abs(det) > 1e-12
    denominator = np.where(valid, det, 1.0)
    dx, dy = x-a[:, 0], y-a[:, 1]
    s = (dx*v[:, 1]-dy*v[:, 0])/denominator
    t = (u[:, 0]*dy-u[:, 1]*dx)/denominator
    hit = valid & (s >= -1e-8) & (t >= -1e-8) & (s+t <= 1+1e-8)
    return float((a[:, 2]+s*u[:, 2]+t*v[:, 2])[hit].max()) if hit.any() else math.nan


def collision_check(output: Path) -> None:
    spec = mujoco.MjSpec.from_string('<mujoco><worldbody><light pos="42 0 8" diffuse=".9 .9 .9"/>'
        '<geom name="floor" type="plane" size="0 0 .1" rgba=".35 .4 .45 1"/></worldbody></mujoco>')
    apply_terrain(spec, "course")
    started = time.perf_counter()
    model = spec.compile()
    compile_seconds = time.perf_counter()-started
    model.vis.global_.offwidth = 1280
    model.vis.global_.offheight = 800
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    geom = np.zeros(1, dtype=np.int32)
    with np.load(ASSET) as asset:
        triangles = asset["source_triangles"]
    random = np.random.default_rng(20261004)
    points = random.uniform([-6.2, -4.2], [6.2, 4.2], (3000, 2))
    deviations, exact_surface_deviations, tolerated_shell_edges = [], [], []
    for x, y in points:
        world_x, world_y = x+RMUL_CENTER_M[0], y+RMUL_CENTER_M[1]
        expected = source_height(triangles, x, y)
        height = ground_height(model, world_x, world_y)
        distance = mujoco.mj_ray(model, data, np.array([world_x, world_y, 3.0]),
            np.array([0.0, 0.0, -1.0]), None, 1, -1, geom)
        ray = 3-distance if distance >= 0 else math.nan
        assert abs(height-ray) < 2e-6, (x, y, height, ray)
        deviations.append(abs(height-expected))
        if abs(height-expected) > 2e-6:
            # 壳面补厚的竖面投影最多扩展1 mm；只允许这些明确的边缘差异。
            cross = np.cross(triangles[:, 1]-triangles[:, 0], triangles[:, 2]-triangles[:, 0])
            vertical = np.abs(cross[:, 2]) <= np.linalg.norm(cross, axis=1)*.05
            starts, ends = triangles[vertical, :, :2], np.roll(triangles[vertical, :, :2], -1, axis=1)
            vector = ends-starts
            length = (vector*vector).sum(axis=2)
            t = np.clip(((np.array([x, y])-starts)*vector).sum(axis=2)/np.maximum(length, 1e-16), 0, 1)
            near = np.linalg.norm(starts+t[:, :, None]*vector-np.array([x, y]), axis=2).min()
            assert near <= .00101, (x, y, height, expected, near)
            tolerated_shell_edges.append([float(x), float(y), float(height-expected)])
        else:
            exact_surface_deviations.append(abs(height-expected))
    assert ground_height(model, 37, 0) < .05
    figure, axes = plt.subplots(2, 1, figsize=(12, 6), sharex=True)
    for axis, y in zip(axes, (-2.0, 0.0)):
        xs = np.linspace(-6.25, 6.25, 751)
        source = [source_height(triangles, x, y) for x in xs]
        actual = [ground_height(model, x+42, y) for x in xs]
        axis.plot(xs, source, label="STEP source", lw=2)
        axis.plot(xs, actual, label="MuJoCo collision", lw=1, ls="--")
        axis.set(ylabel="height (m)", title=f"CAD cross section: local y={y:.1f} m")
        axis.grid(alpha=.25)
        axis.legend(loc="upper right")
    axes[-1].set_xlabel("local x (m), world x = local x + 42 m")
    figure.tight_layout()
    figure.savefig(output / "cad-collision-sections.png", dpi=140)
    plt.close(figure)
    renderer = mujoco.Renderer(model, height=800, width=1280)
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [42, 0, .10]
    camera.distance = 20
    render_times = {}
    try:
        for name, elevation in (("top", -90), ("oblique", -48)):
            camera.azimuth = 90 if name == "top" else 120
            camera.elevation = elevation
            renderer.update_scene(data, camera=camera)
            frame = renderer.render()
            Image.fromarray(frame).save(output / f"cad-mujoco-{name}.png")
            started = time.perf_counter()
            for _ in range(10):
                renderer.update_scene(data, camera=camera)
                renderer.render()
            render_times[name] = (time.perf_counter()-started)/10
    finally:
        renderer.close()
    started = time.perf_counter()
    for x, y in points:
        ground_height(model, x+42, y)
    queries = len(points)/(time.perf_counter()-started)
    metadata = json.loads(ASSET.with_suffix(".json").read_text(encoding="utf-8"))
    imported = terrain_info(model)["imported_course"]
    assert len(imported["asset_sha256"]) == 64
    assert imported["source_sha256"] == metadata["source_sha256"]
    report = dict(passed=True, metadata=metadata, ground_geoms=len(ground_geom_ids(model)),
        imported_course=imported,
        compile_seconds=compile_seconds, source_comparison_samples=len(points),
        max_source_height_deviation_m=max(deviations), shell_edge_differences=tolerated_shell_edges,
        max_source_height_deviation_away_from_shell_edges_m=max(exact_surface_deviations),
        ground_queries_per_second_cad=queries, render_seconds_per_frame_1280x800=render_times,
        entry_ground_m=ground_height(model, 37, 0), controller_obstacle_acceptance="未逐障碍验证控制效果")
    (output / "summary.json").write_text(json.dumps(report, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("local/cache/wbr-rmul2026-20261004"))
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    source_preview(output)
    collision_check(output)


if __name__ == "__main__":
    main()
