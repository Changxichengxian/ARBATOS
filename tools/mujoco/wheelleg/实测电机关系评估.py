"""用独立时间窗检查日志中的电机反馈与位置、速度、前馈关系。"""
import importlib.util
import json
import pickle
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location('sdviewer', ROOT / 'tools/sdlog/SdLogViewer.py')
module = importlib.util.module_from_spec(spec)
sys.modules['sdviewer'] = module
spec.loader.exec_module(module)
path = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / 'local/cache/wheelleg-joystick-20261029/日志0046/解析数据.pkl'
with path.open('rb') as stream:
    data = pickle.load(stream)

reported = False
for role in ('RF', 'RB', 'LF', 'LB', 'RW', 'LW'):
    series = data.series['WHEELLEG_MIT_MOTOR_DIAG.' + role]
    t = np.asarray(series.ticks_ms) / 1000
    f = {k: np.asarray(v) for k, v in series.fields.items()}
    x = np.column_stack((f['cmd_kp'] * (f['cmd_position_rad'] - f['fb_position_rad']),
                         f['cmd_kd'] * (f['cmd_velocity_radps'] - f['fb_velocity_radps']),
                         f['cmd_torque_nm'], np.ones(len(t))))
    y = f['fb_torque_nm']
    good = (f['cmd_active'] != 0) & (f['fb_online'] != 0) & np.isfinite(x).all(axis=1) & np.isfinite(y)
    train = good & (t >= 70) & (t < 120)
    test = good & (t >= 272) & (t < 285)
    if train.sum() < 10 or test.sum() < 10:
        continue
    reported = True
    coef, _, rank, singular = np.linalg.lstsq(x[train], y[train], rcond=None)
    baseline = np.array([1., 1., 1., 0.])
    def rmse(mask, c):
        return float(np.sqrt(np.mean((x[mask] @ c - y[mask]) ** 2)))
    print(json.dumps(dict(role=role, train=int(train.sum()), test=int(test.sum()), rank=int(rank),
                          singular=singular.tolist(), coefficients=coef.tolist(),
                          baseline_train=rmse(train, baseline), baseline_test=rmse(test, baseline),
                          fit_train=rmse(train, coef), fit_test=rmse(test, coef),
                          feedback_range=[float(y[test].min()), float(y[test].max())]), ensure_ascii=False))
if not reported:
    raise SystemExit('指定日志缺少足够的 70–120 秒拟合样本或 272–285 秒验证样本')
