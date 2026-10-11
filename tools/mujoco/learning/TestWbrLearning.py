"""学习接口的零修正等价、错误清零、观测边界及环境可重复性检查。"""
from __future__ import annotations

import ctypes
import json
import time

import numpy as np
from stable_baselines3.common.env_checker import check_env

from WbrLearningEnv import WbrLearningEnv, verify_snapshot, LIMITS


def run():
    verify_snapshot()
    started = time.perf_counter()
    env = WbrLearningEnv(randomize=False, strict=True)
    checks = {}
    try:
        observation, _ = env.reset(seed=77, options=dict(case="stop", speed=.5, push=25.))
        checks["observation_shape"] = observation.shape == (93,)
        p = env.profile
        config = (ctypes.c_float*20)(*np.asarray(p["negative_k"]).flatten(), p["theta0"], p["tp0"],
            p["wheel0"], p["support_n"], .2, p["radius_m"], p["length_kp"], p["length_kd"])
        a, b = env.lib.OwnWbrCreate(config), env.lib.OwnWbrCreate(config)
        assert a and b
        ptr = ctypes.POINTER(ctypes.c_float)
        env.lib.OwnWbrSetStage.argtypes = [ctypes.c_void_p, ctypes.c_int]
        out_a, out_b, action = (ctypes.c_float*40)(), (ctypes.c_float*40)(), (ctypes.c_float*6)()
        try:
            for h in (a,b):
                assert env.lib.OwnWbrSetGasSpring(h, 450.) == 1
                assert env.lib.OwnWbrSetJointLimits(h, 20., 40.) == 1
            exact = True
            for _ in range(120):
                assert env.lib.OwnWbrCommandStep(a, env.sensor, env.command, out_a) == 0
                assert env.lib.WbrLearningStep(b, env.sensor, env.command, action, out_b) == 0
                exact &= bytes(out_a) == bytes(out_b)
                env.step(np.zeros(6))
            checks["zero_action_bitwise_equal_120_frames"] = bool(exact)
            for invalid in (float("nan"), float("inf"), -float("inf")):
                action[:] = [invalid]*6
                out_b[:] = [123.]*40
                code = env.lib.WbrLearningStep(b, env.sensor, env.command, action, out_b)
                assert code != 0 and np.count_nonzero(out_b) == 0
            checks["invalid_action_clears_output"] = True
            action[:] = [0.]*6
            saved = env.sensor[4]
            env.sensor[4] = float("nan")
            code = env.lib.WbrLearningStep(b, env.sensor, env.command, action, out_b)
            checks["invalid_sensor_clears_output"] = bool(code != 0 and np.count_nonzero(out_b) == 0)
            env.sensor[4] = saved
            action[:] = [100.]*6
            code = env.lib.WbrLearningStep(b, env.sensor, env.command, action, out_b)
            checks["large_action_remains_bounded"] = code == 0 and bool(np.all(np.abs(out_b[:6]) <= LIMITS+1e-6))
        finally:
            env.lib.OwnWbrDestroy(a)
            env.lib.OwnWbrDestroy(b)
        old = env._frame().copy()
        env.data.qpos[0] += 100.
        checks["world_position_not_observed"] = bool(np.array_equal(old, env._frame()))
        traces = []
        for _ in range(2):
            env.reset(seed=123, options=dict(case="push", speed=.5, push=25.))
            trace = []
            for _ in range(30):
                obs, reward, done, truncated, _ = env.step(np.full(6, .1))
                trace.append(np.concatenate([obs, [reward, float(done), float(truncated)]]))
            traces.append(np.asarray(trace))
        checks["reset_reproducible"] = bool(np.array_equal(*traces))
        check_env(env, warn=True, skip_render_check=True)
        checks["gymnasium_api"] = True
        checks["elapsed_s"] = time.perf_counter()-started
        print(json.dumps(checks, ensure_ascii=False, indent=2))
        assert all(v for k,v in checks.items() if k != "elapsed_s")
    finally:
        env.close()


if __name__ == "__main__":
    run()
