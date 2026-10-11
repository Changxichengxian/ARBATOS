"""回放带速跳台的实际关节记录；不重算动作或改变轨迹。"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path

import imageio.v2 as imageio
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from WbrGasSpring import load_model


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    args = parser.parse_args()
    report = json.loads((args.run / "summary.json").read_text(encoding="utf-8"))
    path = args.run / "qpos-trajectory.npz"
    track = np.load(path)
    rows = list(csv.DictReader((args.run / "trace.csv").open(encoding="utf-8")))
    times = np.asarray([float(r["time_s"]) for r in rows])
    model = load_model(gas_force_n=report["gas_force_n"], terrain="course",
        guide_rollers="passive", correct_link_collisions=True, knee_range="stop180", noslip_iterations=10)
    model.vis.global_.offwidth, model.vis.global_.offheight = 1280, 720
    data = mujoco.MjData(model)
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [9., -17., .28]
    camera.distance, camera.azimuth, camera.elevation = 2.6, 100., -12.
    font = ImageFont.truetype("C:/Windows/Fonts/msyh.ttc", 27)
    small = ImageFont.truetype("C:/Windows/Fonts/msyh.ttc", 21)
    start = max(0., report["trigger_time_s"] - .25)
    end = min(float(track["time_s"][-1]), report["trigger_time_s"] + 5.)
    snapshots = []
    output = args.run / "带速跳台实录.mp4"
    with mujoco.Renderer(model, height=720, width=1280) as renderer, imageio.get_writer(
            output, fps=30, codec="libx264", quality=8, pixelformat="yuv420p") as writer:
        for t in np.arange(start, end, 1/30):
            i = min(int(np.searchsorted(track["time_s"], t)), len(track["time_s"])-1)
            r = rows[min(int(np.searchsorted(times, t)), len(rows)-1)]
            data.qpos[:] = track["qpos"][i]
            mujoco.mj_forward(model, data)
            renderer.update_scene(data, camera=camera)
            picture = Image.fromarray(renderer.render())
            draw = ImageDraw.Draw(picture)
            draw.rectangle((0, 0, 1280, 87), fill="#102232")
            draw.text((22, 8), f"你的控制代码 · 8009 20/40 Nm · {report['gas_force_n']:.0f} N 气簧 · 200 mm 台阶", font=font, fill="white")
            draw.text((22, 49), f"t={t:.2f}s   实际车速 {float(r['true_v_mps']):+.2f} m/s   轮底 {1000*min(float(r['left_bottom_m']),float(r['right_bottom_m'])):.0f} mm   跳跃阶段 {r['jump_phase']}", font=small, fill="#9ee9dc")
            draw.rectangle((0, 678, 1280, 720), fill="#102232")
            status = "通过本次仿真验收" if report["passed"] else "未通过本次验收"
            draw.text((22, 685), f"{status} · 真实轨迹回放 · 180 mm 物理挡块 · 不代表实车热与结构验收", font=small, fill="white")
            writer.append_data(np.asarray(picture))
            if any(abs(t-v) < 1/60 for v in (start+.2, report["takeoff_time_s"] or start, (report["takeoff_time_s"] or start)+.18, (report["takeoff_time_s"] or start)+.3, start+2.3, end-.1)):
                snapshots.append(picture.copy())
    sheet = Image.new("RGB", (1280, 1080), "#102232")
    for i, picture in enumerate(snapshots[:6]):
        sheet.paste(picture.resize((640, 360)), (i%2*640, i//2*360))
    sheet.save(args.run / "跳台过程.jpg", quality=93)
    (args.run / "video.json").write_text(json.dumps(dict(trajectory_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        video_sha256=hashlib.sha256(output.read_bytes()).hexdigest(), start_s=start, end_s=end, fps=30), indent=2)+"\n", encoding="utf-8")
    print(output)


if __name__ == "__main__":
    main()
