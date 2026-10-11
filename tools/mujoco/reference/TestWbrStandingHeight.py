"""从真实起身姿态升高，验收车底间隙、双轮支撑及持续扭矩。"""
from __future__ import annotations

import argparse
import ctypes
import hashlib
import json
import math
from pathlib import Path

import mujoco
import numpy as np

from RunOwnWbr import load_core
from RunWbr import MOTORS
from WbrGasSpring import load_model
from WbrGuideRollers import guide_roller_info, ROLLER_CENTERS
from WbrLinkCollision import link_collision_info
import TestOwnWbrRise as rise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=True)
    source = Path(__file__).with_name("OwnWbrBridge.c")
    source_bytes = source.read_bytes()
    lib, build = load_core(args.output)
    rise.configure_library(lib)
    lib.OwnWbrSetLegLength.argtypes = [ctypes.c_void_p, ctypes.c_float]
    lib.OwnWbrSetLegLength.restype = ctypes.c_int
    profile = json.loads(Path(__file__).with_name("OwnWbrProfile.json").read_text(encoding="utf-8"))
    config = (ctypes.c_float * 20)(*np.asarray(profile["negative_k"]).flatten(),
        profile["theta0"], profile["tp0"], profile["wheel0"], profile["support_n"],
        profile["length_m"], profile["radius_m"], profile["length_kp"], profile["length_kd"])
    rise.CORRECT_LINK_COLLISIONS = True
    model = load_model(terrain="course", guide_rollers="passive", correct_link_collisions=True, noslip_iterations=10)
    model.opt.timestep = .001
    hips = []
    for name in ("ljoint1", "ljoint4", "rjoint1", "rjoint4"):
        index = model.actuator(name + "_actuator").id
        hips.append(index)
        model.actuator_ctrlrange[index] = [-45, 45]
        model.actuator_forcerange[index] = [-45, 45]
        model.jnt_actfrcrange[model.joint(name).id] = [-45, 45]
    rollers = np.array([model.geom(f"guide_roller_geom_{i}").id for i in range(len(ROLLER_CENTERS))])
    names = ["imu_quat", "imu_gyro", "imu_accel"] + [f"{m}_{k}" for m in MOTORS for k in ("pos", "vel", "tor")]
    indices = np.concatenate([np.arange(model.sensor(n).adr[0],
        model.sensor(n).adr[0] + model.sensor(n).dim[0]) for n in names])
    rows = []
    for target in (.24, .28, .31, .32):
        data = mujoco.MjData(model)
        mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
        data.qpos[0] = -12
        mujoco.mj_forward(model, data)
        handle = rise.make_handle(lib, config, True)
        sensor, command, output = (ctypes.c_float * 28)(), (ctypes.c_float * 3)(), (ctypes.c_float * 40)()
        ready, error = None, None
        limit_ticks = nonwheel_ticks = 0
        trace = []
        try:
            for tick in range(14001):
                if tick % 3 == 0:
                    sensor[:] = data.sensordata[indices]
                    code = lib.OwnWbrCommandStep(handle, sensor, command, output)
                    if code or int(output[34]) < 0:
                        error = dict(t=float(data.time), code=code, rise=int(output[34]))
                        break
                    if ready is None and int(output[34]) == 0:
                        ready = float(data.time)
                        if not lib.OwnWbrSetLegLength(handle, target):
                            raise RuntimeError("控制器拒绝腿长目标")
                    data.ctrl[:] = output[:6]
                mujoco.mj_step(model, data)
                mujoco.mj_forward(model, data)
                supports = set()
                for contact in data.contact:
                    geoms = [int(contact.geom1), int(contact.geom2)]
                    terrain = next((g for g in geoms if model.geom(g).name == "floor" or
                                    model.geom(g).name.startswith("terrain_")), None)
                    if terrain is not None:
                        other = geoms[1] if geoms[0] == terrain else geoms[0]
                        supports.add(model.body(int(model.geom_bodyid[other])).name)
                if ready is not None:
                    limit_ticks += int(np.any(data.efc_type == mujoco.mjtConstraint.mjCNSTR_LIMIT_JOINT))
                    nonwheel_ticks += int(bool(supports - {"lwlink", "rwlink"}))
                if tick % 5 == 0:
                    axis_z = data.geom_xmat[rollers].reshape(-1, 3, 3)[:, 2, 2]
                    # 圆柱轴倾斜后，半径和半宽都影响最低点；不能只减固定半径。
                    extent = (model.geom_size[rollers, 0] * np.sqrt(np.maximum(0, 1 - axis_z**2)) +
                              model.geom_size[rollers, 1] * np.abs(axis_z))
                    trace.append(dict(t=float(data.time), leg=[float(output[12]), float(output[18])],
                        clearance_m=float(np.min(data.geom_xpos[rollers, 2] - extent)),
                        tilt_deg=max(abs(math.degrees(output[6])), abs(math.degrees(output[7]))),
                        speed_mps=float(np.linalg.norm(data.qvel[:3])),
                        hip_peak_nm=float(np.max(np.abs(data.actuator_force[hips]))),
                        both_wheels={"lwlink", "rwlink"} <= supports))
                if not np.isfinite(data.qpos).all() or any(w.number for w in data.warning):
                    error = dict(t=float(data.time), physics_warning=True)
                    break
        finally:
            lib.OwnWbrDestroy(handle)
        steady = [r for r in trace if r["t"] >= 11]
        passed = (error is None and ready is not None and len(steady) >= 600 and
            limit_ticks == 0 and nonwheel_ticks == 0 and
            all(r["both_wheels"] and r["tilt_deg"] < 3 and r["speed_mps"] < .02 and
                r["hip_peak_nm"] <= 10.01 and (target < .31 or r["clearance_m"] > .20) for r in steady))
        row = dict(target_m=target, passed=passed, ready_s=ready, error=error,
            post_ready_limit_ticks=limit_ticks, post_ready_nonwheel_ticks=nonwheel_ticks,
            clearance_min_m=min((r["clearance_m"] for r in steady), default=None),
            steady_hip_peak_nm=max((r["hip_peak_nm"] for r in steady), default=None),
            steady_speed_max_mps=max((r["speed_mps"] for r in steady), default=None),
            steady_tilt_max_deg=max((r["tilt_deg"] for r in steady), default=None))
        rows.append(row)
        (args.output / f"trace-{target:.2f}.json").write_text(json.dumps(trace), encoding="utf-8")
        print(json.dumps(row, ensure_ascii=False), flush=True)
    report = dict(passed=all(r["passed"] for r in rows) and source.read_bytes() == source_bytes,
        rows=rows, terrain="course", start_x_m=-12,
        source_sha256=hashlib.sha256(source_bytes).hexdigest(), build=build,
        guide_rollers=guide_roller_info(model), link_collision=link_collision_info(model))
    (args.output / "summary.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
