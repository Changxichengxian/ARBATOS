"""把真实仿真记录渲染成视频，不重新计算或修改控制轨迹。"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import imageio.v2 as imageio
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from RunWbr import ROOT, SOURCE, digest


def label(t, scenario, own=False):
    if t < .5:
        return "初始姿态"
    if t < 2 and not own:
        return "自行撑起"
    if scenario == "drive":
        if 10 <= t < 15:
            return "正向行驶   +0.25 m/s"
        if 18 <= t < 23:
            return "反向行驶   -0.25 m/s"
        if t >= 15:
            return "停车与恢复平衡"
    return "原地站立"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, default=ROOT / "local/cache/wbr-reference-20261003/drive")
    parser.add_argument("--source", type=Path, default=SOURCE)
    args = parser.parse_args()
    report = json.loads((args.run / "summary.json").read_text(encoding="utf-8"))
    own = report.get("controller") == "ARBATOS WheelLegCore"
    for name, expected in report["source_sha256"].items():
        if name.startswith("mjcf/") and digest(args.source / name) != expected:
            raise RuntimeError(f"模型与仿真记录不一致：{name}")
    track = np.load(args.run / "trajectory.npz")
    times, poses = track["time"], track["qpos"]
    model = mujoco.MjModel.from_xml_path(str(args.source / "mjcf/scene.xml"))
    model.vis.global_.offwidth, model.vis.global_.offheight = 1280, 720
    data = mujoco.MjData(model)
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [.30 if report["scenario"] == "drive" else -.20, 0, .24]
    camera.distance, camera.azimuth, camera.elevation = 2.5, 130, -20
    font_path = Path("C:/Windows/Fonts/msyh.ttc")
    if font_path.exists():
        font, small = ImageFont.truetype(str(font_path), 28), ImageFont.truetype(str(font_path), 21)
    else:
        font = small = ImageFont.load_default()
    path = args.run / "仿真演示.mp4"
    snapshots = []
    snapshot_times = (0, 1, 3, 12, 21, 29)
    with mujoco.Renderer(model, height=720, width=1280) as renderer, imageio.get_writer(
        path, fps=25, codec="libx264", quality=8, pixelformat="yuv420p",
    ) as writer:
        for frame, t in enumerate(np.arange(0, times[-1], 1 / 25)):
            index = min(int(np.searchsorted(times, t)), len(times) - 1)
            data.qpos[:] = poses[index]
            mujoco.mj_forward(model, data)
            renderer.update_scene(data, camera=camera)
            picture = Image.fromarray(renderer.render())
            draw = ImageDraw.Draw(picture)
            draw.rectangle((0, 0, 1280, 93), fill="#102232")
            title = "RM 机械模型 + 你的 ARBATOS WheelLegCore" if own else "RM 开源轮腿 · MuJoCo 原模型 + 原控制算法"
            draw.text((25, 12), title, font=font, fill="white")
            draw.text((25, 53), f"{t:05.2f} s     {label(t, report['scenario'], own)}     自由机身 / 无外部支撑", font=small, fill="#9ee9dc")
            draw.rectangle((0, 677, 1280, 720), fill="#102232")
            footer = ("ARBATOS 控制 3 ms / 物理 1 ms · 从直立姿态释放 · 实际仿真轨迹回放" if own else
                      "CosmosMount/wbr_mujoco · 81a1035 · 1 ms 控制/物理步长 · 实际仿真轨迹回放")
            draw.text((25, 687), footer, font=small, fill="#d6dfe9")
            writer.append_data(np.asarray(picture))
            if any(abs(t - value) < .001 for value in snapshot_times):
                snapshots.append(picture.copy())
            if abs(t - 12) < .001:
                picture.save(args.run / "站立预览.png")
            if frame % 125 == 0:
                print(f"rendered {t:.0f}/{times[-1]:.0f} s", flush=True)
    sheet = Image.new("RGB", (1280, 1080), "#102232")
    for i, picture in enumerate(snapshots):
        sheet.paste(picture.resize((640, 360)), ((i % 2) * 640, (i // 2) * 360))
    sheet.save(args.run / "过程预览.jpg", quality=92)
    (args.run / "video.json").write_text(json.dumps({
        "trajectory_sha256": digest(args.run / "trajectory.npz"),
        "video_sha256": digest(path), "fps": 25,
        "source_revision": report["source_revision"], "renderer_sha256": digest(Path(__file__)),
    }, indent=2) + "\n", encoding="utf-8")
    print(path)


if __name__ == "__main__":
    main()
