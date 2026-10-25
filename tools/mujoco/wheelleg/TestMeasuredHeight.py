"""在同一自由三维模型中验证板凳和 VMC 升降、驾驶及停车。"""

import argparse
import hashlib
from pathlib import Path
import shutil

import mujoco
import numpy as np

import RunMeasuredWheelLeg as sim
from TestMeasuredDriving import settling


def run(model, core, profile, designed, schedule, mode, scene, output):
    data = mujoco.MjData(model)
    data.qpos[:] = designed["qpos"]
    mujoco.mj_forward(model, data)
    controller = sim.Controller(model, core, profile, designed, mode, schedule)
    dt = profile["control"]["physics_dt_s"]
    stride = round(profile["control"]["control_dt_s"] / dt)
    trace, poses = [], []
    closure = joint_limit = 0.0
    nonwheel = False
    for step in range(round(16 / dt)):
        t = float(data.time)
        height = .120 if 1 <= t < 5 else .085 if 5 <= t < 9 else .100
        if scene in ("low", "high"):
            height = .085 if scene == "low" else .120
        if scene == "height-reverse":
            height = (.120 if int((t - 1) / .4) % 2 == 0 else .085) if 1 <= t < 9 else .100
        speed = (1.0 if scene != "reverse" else -1.0) if scene not in ("stand", "spin") and 3 <= t < 12 else 0.0
        turn = (1.2 if scene != "right" else -1.2) if scene in ("curve", "right", "reverse", "spin") and 4 <= t < 12 else 0.0
        if step % stride == 0:
            controller.step(data, speed, turn, height)
        mujoco.mj_step(model, data)
        for side in sim.SIDES:
            closure = max(closure, float(np.linalg.norm(data.site(side + "_front_rod_end").xpos - data.site(side + "_back_rod_end").xpos)))
        for joint_id in np.flatnonzero(model.jnt_limited):
            position = data.qpos[model.jnt_qposadr[joint_id]]
            low, high = model.jnt_range[joint_id]
            joint_limit = max(joint_limit, float(low - position), float(position - high))
        for contact in data.contact:
            names = [model.geom(int(i)).name for i in contact.geom]
            nonwheel |= "floor" in names and not any("wheel_geom" in name for name in names)
        if step % stride == 0:
            roll, pitch, yaw = sim.legacy.quat_to_euler_wxyz(data.qpos[3:7])
            velocity = data.xmat[model.body("base").id].reshape(3, 3).T @ data.qvel[:3]
            legs = [core.leg(data.joint(side + "_front_hinge").qpos[0], data.joint(side + "_back_hinge").qpos[0]) for side in sim.SIDES]
            trace.append([t, velocity[0], data.qvel[5], roll, pitch, legs[0][3] - legs[1][3],
                          speed, turn, controller.target_length, controller.length, legs[0][2], legs[1][2],
                          *data.qpos[:3], *data.ctrl])
            poses.append(data.qpos.copy())
        if not controller.enabled:
            break
    rows = np.array(trace)
    steady = rows[(rows[:, 0] >= 10) & (rows[:, 0] < 12)]
    plateau = rows[((rows[:, 0] >= 3.5) & (rows[:, 0] < 5)) |
                   ((rows[:, 0] >= 7.5) & (rows[:, 0] < 9)) | (rows[:, 0] >= 14)]
    if scene == "height-reverse":
        plateau = rows[rows[:, 0] >= 14]
    result = {"mode": mode, "scene": scene, "duration_s": float(data.time), "active": bool(controller.enabled),
              "peak_pitch_deg": float(np.degrees(max(abs(rows[:, 4])))),
              "peak_roll_deg": float(np.degrees(max(abs(rows[:, 3])))),
              "peak_split_deg": float(np.degrees(max(abs(rows[:, 5])))),
              "height_plateau_error_mm": float(1000 * max(abs(np.mean(plateau[:, 10:12], axis=1) - plateau[:, 8]))) if len(plateau) else 1000.,
              "height_min_max_mm": [float(1000 * min(rows[:, 10:12].ravel())), float(1000 * max(rows[:, 10:12].ravel()))],
              "applied_height_min_max_mm": [float(1000 * min(rows[:, 9])), float(1000 * max(rows[:, 9]))],
              "speed_rms_error": float(np.sqrt(np.mean((steady[:, 1] - steady[:, 6]) ** 2))) if len(steady) else 100.,
              "yaw_rms_error": float(np.sqrt(np.mean((steady[:, 2] - steady[:, 7]) ** 2))) if len(steady) else 100.,
              "stop_settle_s": settling(rows, 1, 12, .03), "turn_settle_s": settling(rows, 2, 12, .05),
              "closure_m": closure, "joint_limit_rad": joint_limit, "nonwheel_contact": bool(nonwheel),
              "warnings": int(sum(data.warning.number)), "saturation_fraction": controller.saturated / max(1, controller.samples)}
    result["passed"] = bool(data.time >= 15.99 and controller.enabled and not nonwheel and not result["warnings"]
                            and closure < .001 and joint_limit < .01 and result["peak_pitch_deg"] < 15
                            and result["peak_roll_deg"] < 12 and result["peak_split_deg"] < 8
                            and result["height_plateau_error_mm"] < 3 and result["speed_rms_error"] < .08
                            and min(rows[:, 9]) >= .085 - 1e-9 and max(rows[:, 9]) <= .120 + 1e-9
                            and result["yaw_rms_error"] < .15 and result["stop_settle_s"] < (1.2 if controller.bench else .9)
                            and result["turn_settle_s"] < .8)
    name = mode + "_" + scene
    np.savez_compressed(output / (name + ".npz"), trace=rows, qpos=poses)
    sim.write_json(output / (name + ".json"), result)
    print(result, flush=True)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--modes", nargs="+", choices=list(sim.MODES), default=list(sim.MODES))
    parser.add_argument("--scenes", nargs="+", default=["stand", "drive", "curve", "right", "reverse", "low", "high", "spin", "height-reverse"])
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    profile = sim.load_profile()
    core = sim.Core(profile, sim.ROOT / "local/cache/measured-wheelleg-native")
    cfg = sim.make_config(profile)
    xml = sim.model_xml(profile)
    model = mujoco.MjModel.from_xml_string(xml)
    designed = sim.design(model, cfg, core, profile, profile["control"]["leg_length_m"])
    schedule = sim.design_modes(model, cfg, core, profile)
    (args.output / "model.xml").write_text(xml, encoding="utf-8")
    sim.write_json(args.output / "parameters.json", profile)
    sim.write_json(args.output / "gains.json", schedule)
    for name in ("RunMeasuredWheelLeg.py", "TestMeasuredHeight.py", "MeasuredWheelLegBridge.c"):
        shutil.copy2(sim.HERE / name, args.output / name)
    sim.write_json(args.output / "sources.json", {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in args.output.glob("*.py")})
    results = [run(model, core, profile, designed, schedule, mode, scene, args.output) for mode in args.modes for scene in args.scenes]
    sim.write_json(args.output / "summary.json", {"cases": results, "passed": all(item["passed"] for item in results)})
    if not all(item["passed"] for item in results):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
