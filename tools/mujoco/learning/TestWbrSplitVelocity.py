"""拆分速度反馈通道的等效性、独立性和状态恢复检查。"""
import argparse
import ctypes
import json
from pathlib import Path
import sys

import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--snapshot", required=True, type=Path)
    args = parser.parse_args()
    sys.path.insert(0, str(args.snapshot / "tools/mujoco/learning"))
    from WbrLearningEnv import WbrLearningEnv, verify_snapshot, LIMITS
    verify_snapshot()
    env = WbrLearningEnv(randomize=False, strict=True, action_mode="split_velocity")
    handles, checks = [], {}
    def require(value, message):
        if not value:
            raise AssertionError(message)
    try:
        env.reset(seed=101, options=dict(case="stop", speed=.5))
        p = env.profile
        config = (ctypes.c_float*20)(*np.asarray(p["negative_k"]).flatten(), p["theta0"], p["tp0"],
            p["wheel0"], p["support_n"], .2, p["radius_m"], p["length_kp"], p["length_kd"])
        def pair():
            values = [env.lib.OwnWbrCreate(config) for _ in range(2)]
            require(all(values), "创建控制器失败")
            handles.extend(values)
            for handle in values:
                require(env.lib.OwnWbrSetGasSpring(handle, 450.) == 1, "气簧配置失败")
            return values
        oa, ob = (ctypes.c_float*40)(), (ctypes.c_float*40)()
        left, right = (ctypes.c_float*3)(), (ctypes.c_float*3)()
        split = env.lib.WbrLearningSplitVelocityStep
        reference = env.lib.WbrLearningReferenceStep
        a, b = pair()
        for i in range(120):
            env.command[:] = (.5 if i < 60 else 0., 0., 0.)
            require(env.lib.OwnWbrCommandStep(a, env.sensor, env.command, oa) == 0, "原控制失败")
            require(split(b, env.sensor, env.command, right, ob) == 0, "零动作失败")
            require(bytes(oa) == bytes(ob), "零动作不等价")
        checks["zero_exact_drive_and_stop"] = True
        a, b = pair()
        for i in range(160):
            env.command[:] = (.5 if i < 80 else 0., 0., 0.)
            gain = (-.5, .5, 0., 1.)[i % 4]
            left[:] = (.25, 0., gain)
            right[:] = (.25, gain, gain)
            require(reference(a, env.sensor, env.command, left, oa) == 0, "共享接口失败")
            require(split(b, env.sensor, env.command, right, ob) == 0, "拆分接口失败")
            require(bytes(oa) == bytes(ob), "相同轮/髋倍率不等价于旧共享接口")
        checks["equal_wheel_hip_gains_match_reference_with_braking"] = True
        for action, same, changed, label in (
                ([0., .5, 0.], [0,1,3,4,16,22], [2,5], "wheel_only"),
                ([0., 0., .5], [2,5], [0,1,3,4,16,22], "hip_only")):
            a, b = pair()
            right[:] = action
            env.command[:] = (.5, 0., 0.)
            for _ in range(90):
                require(env.lib.OwnWbrCommandStep(a, env.sensor, env.command, oa) == 0, "原行驶失败")
                require(split(b, env.sensor, env.command, right, ob) == 0, "拆分行驶失败")
                require(bytes(oa) == bytes(ob), "行驶期间意外启用拆分反馈")
            env.command[:] = (0., 0., 0.)
            require(env.lib.OwnWbrCommandStep(a, env.sensor, env.command, oa) == 0, "原停车失败")
            require(split(b, env.sensor, env.command, right, ob) == 0, "拆分停车失败")
            x, y = np.asarray(oa), np.asarray(ob)
            require(np.array_equal(x[same], y[same]), "改动传入无关输出通道")
            require(np.any(x[changed] != y[changed]), "目标通道未产生作用")
            right[:] = (0., 0., 0.)
            require(env.lib.OwnWbrCommandStep(a, env.sensor, env.command, oa) == 0, "原恢复失败")
            require(split(b, env.sensor, env.command, right, ob) == 0, "拆分恢复失败")
            require(bytes(oa) == bytes(ob), "增益没有在调用后恢复")
            checks[label + "_independent_and_restored"] = True
        a, b = pair()
        for index in range(3):
            for invalid in (float("nan"), float("inf")):
                right[:] = (0., 0., 0.)
                right[index] = invalid
                ob[:] = [123.]*40
                require(split(b, env.sensor, env.command, right, ob) != 0 and not np.any(ob), "错误动作未拒绝清零")
        right[:] = (0., 0., 0.)
        require(env.lib.OwnWbrCommandStep(a, env.sensor, env.command, oa) == 0, "原状态校验失败")
        require(split(b, env.sensor, env.command, right, ob) == 0 and bytes(oa) == bytes(ob), "错误动作污染控制状态")
        checks["all_action_channels_reject_nonfinite_without_mutation"] = True
        right[:] = (100., 100., -100.)
        require(split(b, env.sensor, env.command, right, ob) == 0, "限幅动作失败")
        require(np.all(np.abs(ob[:6]) <= LIMITS+1e-6), "力矩越界")
        checks["extreme_actions_keep_motor_limits"] = True
        require(env.action_space.shape == (3,) and np.array_equal(env.action_space.low, [0,-1,-1]), "动作契约错误")
        checks["three_actions_with_nonnegative_braking"] = True
    finally:
        for handle in handles:
            env.lib.OwnWbrDestroy(handle)
        env.close()
    print(json.dumps(checks, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
