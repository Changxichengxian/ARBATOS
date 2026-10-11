"""验证 R 保留物理状态尝试自起、T 返回入口，以及倒地后物理继续。"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import threading
import time

import mujoco
import numpy as np

from ViewOwnWbr import LiveSimulation
from WbrReset import local_reset_pose
from WbrTerrain import ground_geom_ids, ground_height


ROOT = Path(__file__).resolve().parents[3]


def yaw_of(pose):
    w, x, y, z = pose[3:7]
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def yaw_distance(left, right):
    return abs(math.atan2(math.sin(left-right), math.cos(left-right)))


def min_ground_distance(model, data):
    grounds = ground_geom_ids(model)
    return min((float(c.dist) for c in data.contact
                if (c.geom1 in grounds) != (c.geom2 in grounds)), default=0.0)


class ResetSimulation(LiveSimulation):
    def __init__(self, *args, seed=None, **kwargs):
        super().__init__(*args, **kwargs)
        self.seed = seed
        self.reset_checks = []
        self.thread_error = None
        self.kick = False
        self.kick_started = None
        self.kick_finished = False
        self.failed_samples = []
        self.fail_recovery_once = False

    def reset(self, *args, **kwargs):
        mode = kwargs.get("reset_mode", args[5] if len(args) > 5 else "global")
        if self.fail_recovery_once and mode == "recover":
            self.fail_recovery_once = False
            if self.handle:
                self.own_lib.OwnWbrDestroy(self.handle)
                self.handle = None
            # 模拟旧控制器销毁后新建失败，T 仍须能被物理线程处理。
            raise RuntimeError("测试：控制器创建失败")
        super().reset(*args, **kwargs)
        if self.seed is not None:
            x, y, yaw = self.seed
            self.seed = None
            current = self.data.qpos.copy()
            current[:2] = [x, y]
            current[3:7] = [math.cos(yaw/2), 0, 0, math.sin(yaw/2)]
            # 仅设定测试初始条件；运行中的 R 绝不调用这个扶正工具。
            self.data.qpos[:] = local_reset_pose(self.model, self.model.key("home").qpos, current)
            mujoco.mj_forward(self.model, self.data)
        self.reset_checks.append(dict(last_reset=self.last_reset,
            velocity_max=float(np.max(np.abs(self.data.qvel))),
            initial_ground_depth=min_ground_distance(self.model, self.data)))

    def inputs(self):
        # 在物理线程中施加真实侧向冲击和翻滚力矩，避免另一线程修改MuJoCo数据。
        if self.kick and self.own_ready and not self.kick_finished:
            if self.kick_started is None:
                self.kick_started = float(self.data.time)
            if self.data.time-self.kick_started < .20:
                self.data.xfrc_applied[self.model.body("base_link").id] = [0, 1000, 0, 500, 0, 0]
            else:
                self.data.xfrc_applied[:] = 0
                self.kick_finished = True
        else:
            self.data.xfrc_applied[:] = 0
        if self.state.get("running") is False and "倒地" in self.state.get("status", ""):
            self.failed_samples.append(dict(t=float(self.data.time), qpos=self.data.qpos.tolist(),
                                            ctrl=self.data.ctrl.tolist(), state=dict(self.state)))
        return super().inputs()

    def run_test(self):
        try:
            self.physics()
        except Exception as exc:
            self.thread_error = repr(exc)


def wait_state(simulation, predicate, seconds=12):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if simulation.thread_error:
            raise AssertionError(simulation.thread_error)
        with simulation.lock:
            state = dict(simulation.state)
        if predicate(state):
            return state
        time.sleep(.02)
    raise AssertionError(dict(reason="等待真实仿真状态超时", state=state))


def close_thread(simulation, thread):
    simulation.stop_event.set()
    thread.join(15)
    assert not thread.is_alive(), "物理线程未退出"
    assert simulation.thread_error is None, simulation.thread_error


def direct_checks(simulation):
    assert simulation.state.get("limited_recovery_supported") is True
    cases = (("flat", -12, -9, .7), ("flat_reverse", -15, -11, -2.2),
             ("step100", 7.1, -9, 1.2), ("bridge_uphill", 20, 9, .4),
             ("cross15", 12.5, 1.5, -1.1))
    result = []
    for name, x, y, yaw in cases:
        current = simulation.model.key("home").qpos.copy()
        current[:len(simulation.profile["qpos"])] = simulation.profile["qpos"]
        current[:2] = [x, y]
        # 当前姿态含横滚和俯仰；R 必须逐项保留完整物理状态。
        yaw_quat = np.array([math.cos(yaw/2), 0, 0, math.sin(yaw/2)])
        tilt = np.array([math.cos(.35), math.sin(.35), 0, 0])
        mujoco.mju_mulQuat(current[3:7], yaw_quat, tilt)
        current[2] = ground_height(simulation.model, x, y) + .3
        simulation.data.qpos[:] = current
        simulation.data.qvel[:] = 3
        simulation.data.time = 17.25
        velocity = simulation.data.qvel.copy()
        mujoco.mj_forward(simulation.model, simulation.data)
        # 最后一例同时验证旧页面 local 兼容请求也不会瞬移。
        simulation.reset("own", 3, .20, location="playground", reset_mode="local" if name == "cross15" else "recover")
        assert np.array_equal(simulation.data.qvel, velocity)
        assert np.array_equal(simulation.data.qpos, current)
        assert simulation.data.time == 17.25
        assert simulation.last_reset["mode"] == "recover"
        result.append(dict(case=name, original_yaw=yaw_of(current),
                           last_reset=simulation.last_reset,
                           min_ground_distance=min_ground_distance(simulation.model, simulation.data)))
    simulation.reset("own", 3, .20, location="playground", reset_mode="global")
    assert np.allclose(simulation.data.qpos[:2], [-12, -9])
    assert yaw_distance(yaw_of(simulation.data.qpos), 0) < 1e-12
    assert simulation.last_reset["mode"] == "global"
    assert np.max(np.abs(simulation.data.qvel)) == 0 and simulation.data.time == 0
    simulation.location = "playground"
    previous_owner = simulation.control_owner
    try:
        simulation.command([], reset=True, reset_mode="recover", location="start",
                           client_id="stale-tab", claim_control=True)
        raise AssertionError("过期页面的 R 请求不应变成 global")
    except ValueError:
        pass
    assert simulation.location == "playground" and simulation.control_owner == previous_owner
    own_global_reset = simulation.last_reset

    simulation.reset("upstream", 3, .20, location="start", reset_mode="global")
    simulation.controller, simulation.stage, simulation.location = "upstream", 3, "start"
    simulation.control_owner = "driver"
    simulation.data.qpos[0] = 1.25
    simulation.data.qvel[:] = 2.5
    simulation.data.time = 8.75
    handle = simulation.handle
    active_controller = simulation.active_controller
    pose, velocity = simulation.data.qpos.copy(), simulation.data.qvel.copy()
    last_reset = simulation.last_reset
    for mode in ("recover", "local"):
        try:
            simulation.reset("upstream", 3, .20, location="start", reset_mode=mode)
            raise AssertionError("原作者控制器不能直接进入本轮受限自起")
        except ValueError:
            pass
        assert simulation.handle == handle and simulation.active_controller == active_controller
        assert np.array_equal(simulation.data.qpos, pose)
        assert np.array_equal(simulation.data.qvel, velocity) and simulation.data.time == 8.75
        assert simulation.last_reset is last_reset

        try:
            simulation.command([], reset=True, reset_mode=mode,
                               client_id="other-tab", claim_control=True)
            raise AssertionError("原作者控制器的 R 请求必须在提交前拒绝")
        except ValueError:
            pass
        assert simulation.control_owner == "driver"
        assert not simulation.reset_requested and simulation.reset_mode == "global"
    simulation.reset("own", 3, .20, location="playground", reset_mode="global")
    simulation.controller, simulation.stage, simulation.location = "own", 3, "playground"
    return dict(recovery_continuity_cases=result, global_reset=own_global_reset,
                upstream_recovery_rejected=dict(direct_modes=["recover", "local"],
                    command_modes=["recover", "local"], handle_preserved=True,
                    physical_state_preserved=True, owner_preserved=True))


def rise_check(output, name, seed):
    simulation = ResetSimulation(output / name, terrain="course", seed=seed)
    simulation.location = "playground"
    thread = threading.Thread(target=simulation.run_test, daemon=True)
    thread.start()
    try:
        state = wait_state(simulation, lambda state: state.get("own_ready") and state.get("running") and
                           state.get("sim_time", 0) > 4, seconds=15)
        assert state["physics_running"]
        assert abs(state["roll_deg"]) < 20 and abs(state["pitch_deg"]) < 10, state
        assert simulation.reset_checks[0]["velocity_max"] == 0
        assert simulation.reset_checks[0]["initial_ground_depth"] >= -1e-6
        return dict(case=name, state=state, initial=simulation.reset_checks[0])
    finally:
        close_thread(simulation, thread)


def fall_recovery_check(output):
    simulation = ResetSimulation(output / "fall-and-recover", terrain="course")
    simulation.location = "playground"
    thread = threading.Thread(target=simulation.run_test, daemon=True)
    thread.start()
    result = {}
    try:
        ready = wait_state(simulation, lambda state: state.get("own_ready") and state.get("running") and
                           state.get("sim_time", 0) > 4)
        simulation.request_record("start")
        simulation.kick = True
        failed = wait_state(simulation, lambda state: not state.get("running", True) and
                            "倒地" in state.get("status", ""))
        initial_time = failed["sim_time"]
        continued = wait_state(simulation, lambda state: state.get("sim_time", 0) > initial_time+.45)
        assert continued["physics_running"] and not continued["running"], continued
        first, last = simulation.failed_samples[0], simulation.failed_samples[-1]
        assert last["t"]-first["t"] > .3
        displacement = float(np.linalg.norm(np.asarray(last["qpos"])-np.asarray(first["qpos"])))
        assert displacement > .01, displacement
        assert all(max(abs(x) for x in sample["ctrl"]) == 0 for sample in simulation.failed_samples)
        result.update(ready=ready, failed=failed, continued=continued,
                      failed_physics_duration=last["t"]-first["t"], failed_pose_change_norm=displacement,
                      zero_ctrl_samples=len(simulation.failed_samples), external_force_n=1000,
                      external_roll_torque_nm=500, disturbance_duration_s=.20)
        simulation.kick = False
        simulation.command([], reset=True, reset_mode="recover")
        reset = wait_state(simulation, lambda state: (state.get("last_reset") or {}).get("mode") == "recover")
        audit = reset["last_reset"]
        assert reset.get("rise_phase") in range(11, 16), reset
        assert audit["qpos"] == audit["origin_qpos"]
        assert audit["qvel"] == audit["origin_qvel"]
        assert audit["time"] == audit["origin_time"]
        recovery_result = wait_state(simulation, lambda state: state.get("sim_time", 0) > audit["time"]+.1 and
                                     (state.get("own_ready") or not state.get("running")), seconds=20)
        assert recovery_result["physics_running"]
        simulation.command([], reset=True, reset_mode="global")
        global_state = wait_state(simulation, lambda state: (state.get("last_reset") or {}).get("mode") == "global")
        assert np.allclose(global_state["last_reset"]["qpos"][:2], [-12, -9])
        global_ready = wait_state(simulation, lambda state: state.get("own_ready") and state.get("running") and
                                  state.get("sim_time", 0) > 4, seconds=15)
        result.update(recovery_request=audit, recovery_result=recovery_result,
                      self_righted=bool(recovery_result.get("own_ready")), global_ready=global_ready)
        return result
    finally:
        close_thread(simulation, thread)


def failed_creation_check(output):
    simulation = ResetSimulation(output / "creation-failure", terrain="course")
    simulation.location = "playground"
    thread = threading.Thread(target=simulation.run_test, daemon=True)
    thread.start()
    try:
        wait_state(simulation, lambda state: state.get("own_ready") and state.get("running"))
        simulation.fail_recovery_once = True
        simulation.command([], reset=True, reset_mode="recover")
        failed = wait_state(simulation, lambda state: not state.get("running") and
                            "控制器创建失败" in state.get("status", ""))
        wait_state(simulation, lambda state: state.get("sim_time", 0) > failed["sim_time"]+.10)
        assert thread.is_alive() and not simulation.reset_in_progress
        simulation.command([], reset=True, reset_mode="global")
        ready = wait_state(simulation, lambda state: state.get("own_ready") and state.get("running") and
                           (state.get("last_reset") or {}).get("mode") == "global")
        return dict(failure=failed, global_recovered=ready)
    finally:
        close_thread(simulation, thread)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "local/cache/wbr-turn-recovery-20261004/reset-tests")
    parser.add_argument("--fall-only", action="store_true")
    args = parser.parse_args()
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=True)
    result = dict(checks={}, source_sha256={})
    for name in ("ViewOwnWbr.py", "WbrReset.py", "OwnWbrBridge.c", "TestWbrViewerReset.py"):
        result["source_sha256"][name] = hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest()
    try:
        if not args.fall_only:
            simulation = ResetSimulation(args.output / "direct", terrain="course")
            result["checks"]["direct"] = direct_checks(simulation)
            if simulation.handle:
                simulation.own_lib.OwnWbrDestroy(simulation.handle)
                simulation.handle = None
            for name, seed in (("flat_yaw070", (-12, -9, .7)), ("flat_yaw_reverse", (-15, -9, -2.2)),
                               ("step100", (7.1, -9, 1.2)), ("uphill", (20, 9, .4))):
                result["checks"][name] = rise_check(args.output, name, seed)
        result["checks"]["fall_recovery"] = fall_recovery_check(args.output)
        result["checks"]["creation_failure"] = failed_creation_check(args.output)
        result["passed"] = True
    except Exception as exc:
        result["passed"] = False
        result["error"] = repr(exc)
        raise
    finally:
        (args.output / "summary.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
        print(json.dumps(dict(passed=result["passed"], checks=list(result["checks"]),
                              error=result.get("error"), output=str(args.output)), ensure_ascii=False))


if __name__ == "__main__":
    main()
