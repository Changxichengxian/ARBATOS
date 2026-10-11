"""对照相同碰撞形状与质量的固定/被动导轮，记录真实滚动和100mm触台。"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import threading
import time

import mujoco
import numpy as np

from WbrGasSpring import load_model, MOTOR_NAMES
from WbrGuideRollers import guide_roller_info, ROLLER_CENTERS, ROLLER_RADIUS, BASE_STL, _cad_pieces


ROOT = Path(__file__).resolve().parents[3]


def rolling_bench(mode):
    model = load_model(guide_rollers=mode, correct_link_collisions=True, enabled=False)
    model.opt.timestep = .001
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    data.qpos[2] = .0165
    mujoco.mj_forward(model, data)
    start = data.qpos[0]
    max_speed = 0
    contacts = 0
    normal = 0
    step_times = []
    ncon_max = 0
    wall_start = time.perf_counter()
    for _ in range(1200):
        # 整车低姿态放在导轮上，30N水平推力小于相同摩擦的固定底部滑动门槛。
        data.xfrc_applied[model.body("base_link").id] = [30, 0, 0, 0, 0, 0]
        before = time.perf_counter()
        mujoco.mj_step(model, data)
        mujoco.mj_forward(model, data)
        step_times.append((time.perf_counter()-before)*1e6)
        ncon_max = max(ncon_max, int(data.ncon))
        ids = [model.joint(f"guide_roller_joint_{i}").dofadr[0] for i in range(len(ROLLER_CENTERS))]
        max_speed = max(max_speed, float(np.max(np.abs(data.qvel[ids]))))
        for index, contact in enumerate(data.contact):
            names = (model.geom(contact.geom1).name, model.geom(contact.geom2).name)
            if any(name.startswith("guide_roller_geom") for name in names):
                contacts += 1
                force = np.zeros(6)
                mujoco.mj_contactForce(model, data, index, force)
                normal += max(float(force[0]), 0)
    wall_elapsed = time.perf_counter()-wall_start
    return dict(mode=mode, travel_m=float(data.qpos[0]-start), end_vx=float(data.qvel[0]),
                roller_max_speed_radps=max_speed, roller_contacts=contacts, normal_impulse_ns=normal*.001,
                roller_angles=data.qpos[21:].tolist(), finite=bool(np.isfinite(data.qpos).all()),
                rotated_rollers=int(np.count_nonzero(np.abs(data.qpos[21:])>.5)), ncon_max=ncon_max,
                physics_step_median_us=float(np.median(step_times[200:])),
                physics_step_p95_us=float(np.percentile(step_times[200:],95)),
                physics_step_max_us=float(np.max(step_times[200:])),
                wall_time_s=wall_elapsed, entire_bench_real_time_factor=1.2/wall_elapsed,
                metadata=guide_roller_info(model))


def mass_check():
    models = [load_model(guide_rollers=mode) for mode in ("legacy", "fixed", "passive")]
    masses = []
    inertias = []
    for model in models:
        data = mujoco.MjData(model)
        mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
        mujoco.mj_forward(model, data)
        mass_matrix = np.zeros((model.nv, model.nv))
        mujoco.mj_fullM(model, data, mass_matrix)
        masses.append(float(model.body_mass.sum()))
        inertias.append(mass_matrix[:6, :6])
        assert tuple(model.actuator(i).name for i in range(model.nu)) == MOTOR_NAMES
    assert models[0].nq == 21 and all(model.nq == 21+len(ROLLER_CENTERS) for model in models[1:])
    assert np.allclose(masses, masses[0], atol=1e-12)
    assert all(np.allclose(matrix, inertias[0], atol=1e-10) for matrix in inertias[1:])
    assert all(np.array_equal(models[0].jnt_qposadr, model.jnt_qposadr[:models[0].njnt]) for model in models[1:])
    return dict(masses_kg=masses, free_body_inertia_max_error=max(float(np.max(abs(m-inertias[0]))) for m in inertias),
                original_joint_prefix_preserved=True)


def end_corner_check():
    pieces, _, _ = _cad_pieces(str(BASE_STL.resolve()), BASE_STL.stat().st_mtime_ns)
    vertices = np.unique(np.concatenate(pieces), axis=0)
    corners = vertices[(np.abs(vertices[:, 0]) >= .270) & (vertices[:, 2] <= 0)]
    assert len(corners), "原CAD中未找到前后低位壳角，需重新核对护角范围"
    centers = np.asarray(ROLLER_CENTERS)
    end_x = np.where(corners[:, 0] > 0, centers[:, 0].max(), centers[:, 0].min())
    margins = ROLLER_RADIUS-np.hypot(corners[:, 0]-end_x, corners[:, 2])
    # 对宽台阶、无横滚的前后倾斜，比较XZ支撑包络；不声称护住任意斜向窄棱。
    assert float(margins.min()) > .001, margins.tolist()
    return dict(cad_corner_vertices=len(corners), min_pitch_plane_margin_m=float(margins.min()),
                scope="宽台阶、无横滚的前后倾斜；不代表任意横滚或斜向棱角均受保护")


def climb_test(mode, output, speed, duration, automatic=False, stop_on_failure=False, leg_length=.20):
    from ViewOwnWbr import LiveSimulation
    output.mkdir(parents=True, exist_ok=True)
    bridge_source = Path(__file__).with_name("OwnWbrBridge.c")
    source_bytes = bridge_source.read_bytes()
    class DrivenSimulation(LiveSimulation):
        def reset(self, *args, **kwargs):
            super().reset(*args, **kwargs)
            if automatic:
                self.data.qpos[0] = 4.5
                mujoco.mj_forward(self.model, self.data)
                self.last_reset["qpos"] = self.data.qpos.tolist()

        def inputs(self):
            result = list(super().inputs())
            result[6] = np.array([speed if self.own_ready else 0, 0, 0], dtype=float)
            result[8] = False
            return tuple(result)
    simulation = DrivenSimulation(output, terrain="course", guide_rollers=mode)
    assert bridge_source.read_bytes() == source_bytes, "编译期间C源码变化，不能按当前文件哈希认定本次测试"
    (output/"OwnWbrBridge.c").write_bytes(source_bytes)
    simulation.location = "step_auto" if automatic else "step_climb"
    simulation.leg_length = leg_length
    samples = []
    errors = []
    def run():
        try:
            simulation.physics()
        except Exception as exc:
            errors.append(repr(exc))
    thread = threading.Thread(target=run, daemon=True)
    thread.start()
    deadline = time.monotonic()+max(40, duration*5)
    simulation.request_record("start")
    try:
        previous = -1
        while time.monotonic() < deadline:
            with simulation.lock:
                state = dict(simulation.state)
            t = state.get("sim_time")
            if t is not None and t != previous:
                samples.append({key:state.get(key) for key in ("sim_time", "running", "physics_running", "status",
                    "pitch_deg", "roll_deg", "course_x_m", "v_mps", "step_climb_phase", "wheel_support_surfaces")})
                previous = t
            if (t is not None and t >= duration) or errors:
                break
            if stop_on_failure and t is not None and t > 2 and state.get("running") is False:
                break
            time.sleep(.03)
    finally:
        simulation.stop_event.set()
        thread.join(15)
    assert not thread.is_alive() and not errors, errors
    assert samples[-1]["sim_time"] >= duration or (stop_on_failure and not samples[-1]["running"]), samples[-1]
    (output/"guide-test-trace.json").write_text(json.dumps(samples, ensure_ascii=False, indent=2), encoding="utf-8")
    recording = simulation.recorder.status()
    jsonl = (output/"recordings"/recording["filename"]).with_suffix(".jsonl")
    frames = [json.loads(line) for line in jsonl.read_text(encoding="utf-8").splitlines()]
    frames = [item for item in frames if item["type"] == "sample"]
    first_contact = next((item for item in frames if any("terrain_large100" in (c["geom1"], c["geom2"])
                         for c in item["frame"]["contacts"])), None)
    impact = None
    if first_contact:
        frame = first_contact["frame"]
        yaw = np.deg2rad(frame["yaw_deg"])
        impact = dict(t=first_contact["sim_time_s"], actual_forward_mps=float(
            frame["qvel"][0]*np.cos(yaw)+frame["qvel"][1]*np.sin(yaw)),
            observer_mps=frame["estimated_forward_mps"], roll_deg=frame["roll_deg"], pitch_deg=frame["pitch_deg"])
        index = frames.index(first_contact)
        if index:
            before = frames[index-1]
            previous_yaw = np.deg2rad(before["frame"]["yaw_deg"])
            impact["before_contact_t"] = before["sim_time_s"]
            impact["before_contact_forward_mps"] = float(before["frame"]["qvel"][0]*np.cos(previous_yaw)+
                                                        before["frame"]["qvel"][1]*np.sin(previous_yaw))
    guide_contacts = sum(any("guide_roller" in c["geom1"] or "guide_roller" in c["geom2"]
                            for c in item["frame"]["contacts"]) for item in frames)
    guide_angles = max((max(abs(value) for value in item["frame"]["qpos"][21:]) for item in frames), default=0)
    # 顶沿斜向接触不能算真正站上平台，用记录中的姿态复算轮中心和支撑法向。
    replay = mujoco.MjData(simulation.model)
    top_checks = []
    for item in frames:
        frame = item["frame"]
        replay.qpos[:] = frame["qpos"]
        replay.qvel[:] = frame["qvel"]
        mujoco.mj_forward(simulation.model, replay)
        support = {"lwlink": False, "rwlink": False}
        for contact in replay.contact:
            names = [simulation.model.geom(int(g)).name for g in (contact.geom1, contact.geom2)]
            if "terrain_large100" not in names or abs(float(contact.frame[2])) <= .9:
                continue
            other = int(contact.geom2) if names[0] == "terrain_large100" else int(contact.geom1)
            body = simulation.model.body(int(simulation.model.geom_bodyid[other])).name
            if body in support:
                position = replay.body(body).xpos
                support[body] = position[0] >= 7.005 and position[2] >= .150
        top_checks.append(dict(t=item["sim_time_s"], top=all(support.values()),
                               stable=frame["running"] and abs(frame["roll_deg"]) < 10 and
                               abs(frame["pitch_deg"]) < 10 and np.linalg.norm(frame["qvel"][:2]) < .1))
    final_window = [check for check in top_checks if check["t"] >= duration-2]
    accepted = bool(final_window) and all(check["top"] and check["stable"] for check in final_window)
    return dict(mode=mode, speed_input_mps=speed, leg_length_m=leg_length, end=samples[-1], max_x=max(x["course_x_m"] for x in samples if x["course_x_m"] is not None),
                phases=sorted(set(x["step_climb_phase"] for x in samples if x["step_climb_phase"] is not None)),
                automatic=automatic, first_contact=impact, roller_contact_samples=guide_contacts,
                accepted100=accepted, ever_both_wheels_on_top=any(check["top"] for check in top_checks),
                acceptance="末2秒双轮中心过顶沿且平台垂直支撑、running、roll/pitch各小于10度、XY速度小于0.1m/s",
                initial_x_m=4.5 if automatic else 6.0,
                roller_max_abs_angle=guide_angles, own_bridge_sha256=simulation.record_metadata["own_bridge_sha256"],
                recording=recording,metadata=guide_roller_info(simulation.model))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "local/cache/wbr-guide-rollers-20261004/tests")
    parser.add_argument("--climb", action="store_true")
    parser.add_argument("--automatic", action="store_true")
    parser.add_argument("--speed", type=float, default=.30)
    parser.add_argument("--duration", type=float, default=16)
    args = parser.parse_args()
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=True)
    result = dict(mass=mass_check(), end_corners=end_corner_check(),
                  rolling=[rolling_bench(mode) for mode in ("fixed", "passive")])
    fixed, passive = result["rolling"]
    assert passive["roller_contacts"] > 0 and fixed["roller_contacts"] > 0
    assert max(abs(x) for x in fixed["roller_angles"]) < .02
    assert passive["roller_max_speed_radps"] > 1 and max(abs(x) for x in passive["roller_angles"]) > .5
    assert passive["rotated_rollers"] == len(ROLLER_CENTERS)
    assert passive["travel_m"] > fixed["travel_m"]+.03, result
    if args.climb:
        result["climb100"] = [climb_test(mode,args.output/mode,args.speed,args.duration,args.automatic)
                              for mode in ("fixed","passive")]
    (args.output/"summary.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False))


if __name__ == "__main__":
    main()
