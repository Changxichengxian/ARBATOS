"""检查网页真实控制器的8009限制、180mm挡块和两种上台入口互斥。"""
from __future__ import annotations

import argparse
import ctypes
import hashlib
import json
from pathlib import Path
import time

import mujoco
import numpy as np

from ViewOwnWbr import HIP_CONTINUOUS_NM, HIP_PEAK_NM, LiveSimulation
from WbrGasSpring import load_model
from WbrKneeRange import knee_range_info
from WbrTerrain import scenario_pose


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("local/cache/wbr-viewer-8009-20261012/config-test"))
    args = parser.parse_args()
    args.output = args.output.resolve()
    source = Path(__file__).with_name("OwnWbrBridge.c")
    source_hash = hashlib.sha256(source.read_bytes()).hexdigest().upper()
    sim = LiveSimulation(args.output, terrain="course")
    rows = []
    try:
        assert sim.knee_range == "stop180"
        assert sim.state["hip_model"] == "8009"
        assert (sim.state["hip_continuous_nm"], sim.state["hip_peak_nm"]) == (20., 40.)
        assert sim.state["hip_rated_rpm"] is None
        metadata = json.loads((args.output / "session.json").read_text(encoding="utf-8"))
        assert (metadata["hip_motor_continuous_nm"], metadata["hip_motor_peak_nm"]) == (20., 40.)
        assert metadata["gas_spring_n_each"] == 450
        # 复核低趴home，跳台另用已有200mm直立姿态；两种动作共用同一平台。
        home = sim.model.key("home").qpos.copy()
        home_data = mujoco.MjData(sim.model)
        home_data.qpos[:] = home
        mujoco.mj_forward(sim.model, home_data)
        closure = max(float(np.linalg.norm(home_data.site(f"{side}link4_site{number}").xpos -
            home_data.site(f"{side}link1_site{number}").xpos)) for side in ("l", "r") for number in (1, 2))
        floor = sim.model.geom("floor").id
        penetration = max((max(0., -float(c.dist)) for c in home_data.contact
            if floor in (c.geom1, c.geom2)), default=0.)
        assert closure <= .001 and penetration <= 1e-9
        jump_pose = scenario_pose(sim.model, home, "step_jump200")
        climb_pose = scenario_pose(sim.model, home, "climb200")
        assert np.allclose(jump_pose[:2], [4.5, -17.0])
        assert np.allclose(climb_pose[:2], [6.5, -17.0])
        assert np.array_equal(jump_pose[2:], climb_pose[2:])
        for location, profile, expected_mode in (("step_jump200", 1, 0), ("large_steps", 1, 0), ("climb200", 0, 2)):
            sim.reset("own", 3, .32 if location == "climb200" else .20, profile, location)
            if location == "step_jump200":
                assert np.allclose(sim.data.qpos, scenario_pose(sim.model, sim.profile_pose, location))
                assert not sim.entry_height_pending
            assert sim.handle and sim.step_climb_mode == expected_mode
            assert np.allclose(sim.model.actuator_ctrlrange[sim._hip_actuators], [-40, 40])
            assert np.allclose(sim.model.actuator_forcerange[sim._hip_actuators], [-40, 40])
            assert np.allclose(sim.model.jnt_actfrcrange[sim._hip_joints], [-40, 40])
            assert sim.own_lib.OwnWbrGetJumpProfile(sim.handle) == profile + 2
            if location in ("large_steps", "step_jump200"):
                with sim.input_lock:
                    sim.controller, sim.stage, sim.jump_profile = "own", 3, profile
                    sim.location, sim.drive_speed = location, 3.0
                    sim.keys, sim.input_at = {"w"}, time.monotonic()
                    sim.stop_requested = sim.motor_stop_latched = False
                applied_speed = sim.inputs()[6][0]
                assert abs(applied_speed - sim.own_lib.OwnWbrGetJumpSpeedLimit(sim.handle)) < 1e-6
            rows.append(dict(location=location, climb_mode=sim.step_climb_mode,
                jump_profile=sim.own_lib.OwnWbrGetJumpProfile(sim.handle),
                jump_speed_limit_mps=sim.own_lib.OwnWbrGetJumpSpeedLimit(sim.handle),
                initial_xy_m=sim.data.qpos[:2].tolist()))
        for mode in ("upstream", "cad120", "stop180"):
            model = load_model(knee_range=mode, guide_rollers="passive", correct_link_collisions=True)
            info = knee_range_info(model)
            assert info["mode"] == mode
            if mode == "stop180":
                poses_path = Path(__file__).resolve().parents[3] / "local/cache/wbr-user-record-20261004/spring-static-support.json"
                poses = json.loads(poses_path.read_text(encoding="utf-8"))["rows"]
                pose = next(row for row in poses if abs(row["length_m"] - .18) < 1e-8)
                data = mujoco.MjData(model)
                data.qpos[:] = pose["qpos"]
                mujoco.mj_forward(model, data)
                for side, name, edge in (("l", "llink1child1joint", 1), ("r", "rlink1child1joint", 0)):
                    jid = model.joint(name).id
                    assert abs(data.qpos[int(model.jnt_qposadr[jid])] - model.jnt_range[jid, edge]) < 1e-12
                    length = np.linalg.norm((data.body(side + "wlink").xpos - data.body(side + "link1").xpos)[[0, 2]])
                    assert abs(length - .18) < 1e-6
        assert hashlib.sha256(source.read_bytes()).hexdigest().upper() == source_hash
        report = dict(passed=True, hip_continuous_nm=HIP_CONTINUOUS_NM, hip_peak_nm=HIP_PEAK_NM,
            knee_range=sim.knee_range_info, home_closure_m=closure, home_ground_penetration_m=penetration,
            rows=rows, controller_sha256=source_hash,
            boundary="真实模型与编译控制器参数检查；未启动网页服务，未代替带速跳200mm台动作验收")
        (args.output / "config-summary.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(report, ensure_ascii=False))
    finally:
        if sim.handle:
            sim.own_lib.OwnWbrDestroy(sim.handle)
            sim.handle = None


if __name__ == "__main__":
    main()
