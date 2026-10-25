"""检查小轮腿连续驾驶质量：实际速度、停车、转向和两腿前后错位。"""

import argparse
import hashlib
import json
from pathlib import Path
import shutil

import mujoco
import numpy as np

import RunMeasuredWheelLeg as sim


def command(name, t):
    if name == "long-spin":
        return 0.0, 1.2 if 1 <= t < 24 else 0.0
    if name == "long-curve":
        return (1.0 if 1 <= t < 24 else 0.0), (1.2 if 2 <= t < 24 else 0.0)
    if name in ("forward", "reverse"):
        return ((-1.0 if name == "reverse" else 1.0) if 1 <= t < 4 else 0.0), 0.0
    if name in ("spin-left", "spin-right"):
        return 0.0, ((-1.2 if name == "spin-right" else 1.2) if 1 <= t < 4 else 0.0)
    if name in ("curve-left", "curve-right"):
        return (.8 if 1 <= t < 5 else 0.0), ((-1.2 if name == "curve-right" else 1.2) if 2 <= t < 5 else 0.0)
    if name == "reverse-curve":
        return (-.8 if 1 <= t < 5 else 0.0), (1.2 if 2 <= t < 5 else 0.0)
    if name == "switchback":
        return (1.0 if 1 <= t < 4 else -1.0 if 4 <= t < 7 else 0.0), (1.2 if 2 <= t < 5 else -1.2 if 5 <= t < 7 else 0.0)
    raise ValueError(name)


def settling(trace, column, start, tolerance):
    after = trace[trace[:, 0] >= start]
    violations = after[np.abs(after[:, column]) > tolerance]
    return float(violations[-1, 0] - start) if len(violations) else 0.0


def run(module, model, cfg, core, profile, designed, name, folder, duration=9):
    if name.startswith("long-"):
        duration = 28
    data = mujoco.MjData(model)
    data.qpos[:] = designed["qpos"]
    mujoco.mj_forward(model, data)
    controller = module.Controller(model, core, profile, designed)
    dt = profile["control"]["physics_dt_s"]
    stride = round(profile["control"]["control_dt_s"] / dt)
    trace, poses = [], []
    nonwheel = False
    closure = 0.0
    joint_limit = 0.0
    for step in range(round(duration / dt)):
        t = float(data.time)
        v, yaw_rate = command(name, t)
        if step % stride == 0:
            controller.step(data, v, yaw_rate)
        mujoco.mj_step(model, data)
        for side in sim.SIDES:
            closure = max(closure, float(np.linalg.norm(data.site(side + "_front_rod_end").xpos - data.site(side + "_back_rod_end").xpos)))
        for joint_id in np.flatnonzero(model.jnt_limited):
            q = data.qpos[model.jnt_qposadr[joint_id]]
            lo, hi = model.jnt_range[joint_id]
            joint_limit = max(joint_limit, float(lo - q), float(q - hi))
        for contact in data.contact:
            names = [model.geom(int(i)).name for i in contact.geom]
            nonwheel |= "floor" in names and not any("wheel_geom" in n for n in names)
        if step % stride == 0:
            roll, pitch, yaw = sim.legacy.quat_to_euler_wxyz(data.qpos[3:7])
            # 独立评分使用机身朝向投影后的物理速度，转向后不能继续用世界 X 速度。
            velocity = data.xmat[model.body("base").id].reshape(3, 3).T @ data.qvel[:3]
            legs = [core.leg(data.joint(s + "_front_hinge").qpos[0], data.joint(s + "_back_hinge").qpos[0]) for s in sim.SIDES]
            split = legs[0][3] - legs[1][3]
            trace.append([t, velocity[0], data.qvel[5], roll, pitch, split, v, yaw_rate,
                          *data.qpos[:3], legs[0][2], legs[1][2], *data.ctrl])
            poses.append(data.qpos.copy())
        if not controller.enabled:
            break
    rows = np.array(trace)
    stop = 24.0 if name.startswith("long-") else 7.0 if name == "switchback" else 5.0 if name in ("curve-left", "curve-right", "reverse-curve") else 4.0
    # 反向指令的瞬态单独看姿态与停车；稳态误差从最后指令保持 1.5 秒后计算。
    steady_start = 6.5 if name == "switchback" else 4.0 if name.startswith("long-") else 3.0
    steady = rows[(rows[:, 0] >= steady_start) & (rows[:, 0] < stop)]
    result = {
        "name": name, "duration_s": float(data.time), "active": bool(controller.enabled),
        "speed_rms_error": float(np.sqrt(np.mean((steady[:, 1] - steady[:, 6]) ** 2))) if len(steady) else 100.0,
        "yaw_rate_rms_error": float(np.sqrt(np.mean((steady[:, 2] - steady[:, 7]) ** 2))) if len(steady) else 100.0,
        "stop_settle_s": settling(rows, 1, stop, .03), "turn_settle_s": settling(rows, 2, stop, .05),
        "peak_split_deg": float(np.degrees(max(abs(rows[:, 5])))),
        "peak_pitch_deg": float(np.degrees(max(abs(rows[:, 4])))),
        "peak_roll_deg": float(np.degrees(max(abs(rows[:, 3])))),
        "stop_displacement_m": float(np.linalg.norm(rows[-1, 8:10] - rows[np.argmin(abs(rows[:, 0] - stop)), 8:10])),
        "nonwheel_contact": bool(nonwheel), "closure_m": closure, "joint_limit_rad": joint_limit,
        "warnings": int(sum(data.warning.number)), "saturation_fraction": controller.saturated / max(1, controller.samples),
    }
    after = rows[rows[:, 0] >= stop]
    if name in ("forward", "reverse"):
        sign = -1 if name == "reverse" else 1
        result["rebound_m_s"] = max(0.0, float(max(-sign * after[:, 1])))
        accelerating = rows[(rows[:, 0] >= 1) & (rows[:, 0] < 4) & (sign * rows[:, 1] >= .9)]
        result["rise_90_s"] = float(accelerating[0, 0] - 1) if len(accelerating) else None
    result["passed"] = bool(data.time >= duration - .005 and controller.enabled and not nonwheel
                             and not result["warnings"] and closure < .001 and joint_limit < .01
                             and result["peak_pitch_deg"] < 12 and result["peak_roll_deg"] < 12
                             and result["peak_split_deg"] < 8 and result["speed_rms_error"] < .08
                             and result["yaw_rate_rms_error"] < .15 and result["stop_settle_s"] < .8
                             and result["turn_settle_s"] < .8 and result.get("rebound_m_s", 0) < .03
                             and ("rise_90_s" not in result or (result["rise_90_s"] is not None and result["rise_90_s"] < 1.3)))
    np.savez_compressed(folder / f"{name}.npz", trace=rows, qpos=poses)
    sim.write_json(folder / f"{name}.json", result)
    print(json.dumps(result), flush=True)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--profile", type=Path, default=sim.PROFILE)
    parser.add_argument("--cases", nargs="+", default=["forward", "reverse", "spin-left", "spin-right", "curve-left", "curve-right", "reverse-curve", "switchback"])
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    profile = sim.load_profile(args.profile)
    core = sim.Core(profile, sim.ROOT / "local/cache/measured-wheelleg-native")
    cfg = sim.make_config(profile)
    xml = sim.model_xml(profile)
    model = mujoco.MjModel.from_xml_string(xml)
    designed = sim.design(model, cfg, core, profile, profile["control"]["leg_length_m"])
    (args.output / "model.xml").write_text(xml, encoding="utf-8")
    sim.write_json(args.output / "parameters.json", profile)
    sim.write_json(args.output / "controller.json", designed)
    for filename in ("RunMeasuredWheelLeg.py", "TestMeasuredDriving.py", "MeasuredWheelLegBridge.c"):
        shutil.copy2(sim.HERE / filename, args.output / filename)
    sim.write_json(args.output / "sources.json", {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in args.output.glob("*.py")})
    cases = [run(sim, model, cfg, core, profile, designed, name, args.output) for name in args.cases]
    sim.write_json(args.output / "driving_summary.json", {"cases": cases, "passed": all(c["passed"] for c in cases)})
    if not all(c["passed"] for c in cases):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
