"""参考修正的零动作、有效范围、状态恢复及错误输入检查。"""
import argparse
import ctypes
import json
import sys

import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--snapshot", required=True)
    args = parser.parse_args()
    from pathlib import Path
    sys.path.insert(0, str(Path(args.snapshot)/"tools/mujoco/learning"))
    from WbrLearningEnv import WbrLearningEnv, verify_snapshot, LIMITS
    verify_snapshot()
    env = WbrLearningEnv(randomize=False, strict=True, action_mode="reference")
    handles = []
    checks = {}
    try:
        env.reset(seed=101, options=dict(case="stop", speed=.5))
        p = env.profile
        config = (ctypes.c_float*20)(*np.asarray(p["negative_k"]).flatten(), p["theta0"], p["tp0"],
            p["wheel0"], p["support_n"], .2, p["radius_m"], p["length_kp"], p["length_kd"])
        def pair():
            result = [env.lib.OwnWbrCreate(config) for _ in range(2)]
            assert all(result)
            handles.extend(result)
            for handle in result:
                assert env.lib.OwnWbrSetGasSpring(handle, 450.) == 1
            return result
        a,b = pair()
        oa,ob = (ctypes.c_float*40)(), (ctypes.c_float*40)()
        action = (ctypes.c_float*3)()
        for i in range(120):
            env.command[:] = (.5 if i < 60 else 0., 0., 0.)
            assert env.lib.OwnWbrCommandStep(a, env.sensor, env.command, oa) == 0
            assert env.lib.WbrLearningReferenceStep(b, env.sensor, env.command, action, ob) == 0
            assert bytes(oa) == bytes(ob)
        checks["zero_exact_during_drive_and_stop"] = True
        a,b = pair()
        env.command[:] = (.5,0.,0.)
        action[:] = (1.,1.,-1.)
        for _ in range(90):
            assert env.lib.OwnWbrCommandStep(a, env.sensor, env.command, oa) == 0
            assert env.lib.WbrLearningReferenceStep(b, env.sensor, env.command, action, ob) == 0
            assert bytes(oa) == bytes(ob)
        checks["nonzero_drive_disables_reference_tuning"] = True
        env.command[:] = (0.,0.,0.)
        action[:] = (1.,0.,0.)
        assert env.lib.OwnWbrCommandStep(a, env.sensor, env.command, oa) == 0
        assert env.lib.WbrLearningReferenceStep(b, env.sensor, env.command, action, ob) == 0
        assert abs((oa[26]-ob[26])-.0036) < 1e-7
        checks["additional_deceleration_is_1_2_mps2"] = True
        a,b = pair()
        env.command[:] = (0.,0.,0.)
        for invalid in (float("nan"), float("inf"), -float("inf")):
            action[:] = (1.,invalid,0.)
            ob[:] = [123.]*40
            assert env.lib.WbrLearningReferenceStep(b,env.sensor,env.command,action,ob) != 0
            assert np.count_nonzero(ob) == 0
        action[:] = (1.,1.,1.)
        for values,index in ((env.sensor,4),(env.command,0)):
            saved = values[index]
            values[index] = float("nan")
            ob[:] = [123.]*40
            assert env.lib.WbrLearningReferenceStep(b,env.sensor,env.command,action,ob) != 0
            assert np.count_nonzero(ob) == 0
            values[index] = saved
        action[:] = (0.,0.,0.)
        assert env.lib.OwnWbrCommandStep(a,env.sensor,env.command,oa) == 0
        assert env.lib.WbrLearningReferenceStep(b,env.sensor,env.command,action,ob) == 0
        assert bytes(oa) == bytes(ob)
        checks["invalid_input_clears_without_state_mutation"] = True
        action[:] = (0.,1.,1.)
        assert env.lib.OwnWbrCommandStep(a,env.sensor,env.command,oa) == 0
        assert env.lib.WbrLearningReferenceStep(b,env.sensor,env.command,action,ob) == 0
        action[:] = (0.,0.,0.)
        assert env.lib.OwnWbrCommandStep(a,env.sensor,env.command,oa) == 0
        assert env.lib.WbrLearningReferenceStep(b,env.sensor,env.command,action,ob) == 0
        assert bytes(oa) == bytes(ob)
        checks["gain_changes_do_not_accumulate"] = True
        action[:] = (100.,100.,-100.)
        assert env.lib.WbrLearningReferenceStep(b,env.sensor,env.command,action,ob) == 0
        assert np.all(np.abs(ob[:6]) <= LIMITS+1e-6)
        checks["out_of_range_actions_keep_motor_limits"] = True
        try:
            env.step(np.zeros(6))
            raise AssertionError("未拒绝错误维数")
        except ValueError:
            assert np.count_nonzero(env.data.ctrl) == 0
        checks["three_dimensional_reference_action_contract"] = True
    finally:
        for handle in handles:
            env.lib.OwnWbrDestroy(handle)
        env.close()
    print(json.dumps(checks, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
