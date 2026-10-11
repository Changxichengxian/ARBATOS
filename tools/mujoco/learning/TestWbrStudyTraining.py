"""比较两种动作表示、零修正、密集评分和导出数值的边界。"""
import json
import numpy as np

from WbrLearningEnv import WbrLearningEnv, verify_snapshot
from StudyWbrPolicy import linear_action


def main():
    verify_snapshot()
    full = WbrLearningEnv(randomize=True, strict=True, reward_mode="dense", action_mode="full")
    symmetric = WbrLearningEnv(randomize=True, strict=True, reward_mode="dense", action_mode="symmetric")
    checks = {}
    try:
        a,_ = full.reset(seed=903,options=dict(case="stop",speed=.6,delay=1))
        b,_ = symmetric.reset(seed=903,options=dict(case="stop",speed=.6,delay=1))
        assert np.array_equal(a,b)
        for i in range(80):
            action = np.sin(np.arange(3)+i*.03)*.15
            a,ra,da,ta,_ = full.step(np.tile(action,2))
            b,rb,db,tb,_ = symmetric.step(action)
            assert np.array_equal(a,b) and ra == rb and da == db and ta == tb
            assert not da
        checks["symmetric_and_tiled_full_exact"] = True
        assert np.array_equal(linear_action(a,np.zeros(6)),np.zeros(3))
        assert np.all(np.abs(linear_action(a,np.full(6,1e6))) <= 1.)
        checks["linear_zero_and_bounded"] = True
        try:
            symmetric.step(np.zeros(6))
            raise AssertionError("错误维度未被拒绝")
        except ValueError:
            assert np.count_nonzero(symmetric.data.ctrl) == 0
        checks["wrong_dimension_clears_output"] = True
        assert full.metrics["peak_pitch_1ms_deg"] >= full.metrics["peak_pitch_deg"]-1e-9
        assert full.metrics["peak_roll_1ms_deg"] >= full.metrics["peak_roll_deg"]-1e-9
        checks["physics_peaks_cover_policy_samples"] = True
        assert np.isfinite(ra)
        checks["dense_reward_finite"] = True
    finally:
        full.close()
        symmetric.close()
    print(json.dumps(checks,ensure_ascii=False,indent=2))


if __name__ == "__main__":
    main()
