"""仅供训练的无量纲评分；观测、物理控制和独立验收由原接口负责。"""
import math

import numpy as np

from EvaluateWbrStudy import WINDOWS, acceptance, motion_quality_acceptance

POLICY_DT = .015


def in_window(time_s, bounds):
    return bounds[0]-1e-9 <= time_s <= bounds[1]+1e-9


def baseline_spec(row):
    """同场景原控制整段成本只作尺度，不提供逐时刻动作目标。"""
    if not row["metrics"]["passed"] or "motion_quality" not in row:
        raise ValueError("训练基线必须完整通过物理检查并有运动质量记录")
    windows = row["metrics"]["windows"]
    primary = {}
    for name, item in windows.items():
        metric = "velocity_error_rms_mps" if name == "push" else "velocity_rms_mps"
        primary[name] = max(item[metric]**2, .001**2)*item["samples"]
    duration = row["options"]["duration"]
    sample_times = np.arange(1, math.ceil(duration/POLICY_DT)+1)*POLICY_DT
    drive_count = sum(in_window(float(t), (1.,4.)) for t in sample_times)
    drive = (max(row["motion_quality"]["drive_tracking_rms_mps"]**2, .005**2)*drive_count
             if row["case"] != "push" else None)
    return dict(primary=primary, drive=drive, duration=duration)


def step_costs(scenario, sample, previous_action, previous_heat, spec, heat_scale):
    """每15ms步的密集成本；热项取本步内1ms积分的增量。"""
    time_s = float(sample["time_s"])
    kind = scenario["options"]["case"]
    primary = 0.
    for name, bounds in WINDOWS[kind].items():
        if in_window(time_s, bounds):
            velocity = sample["true_v_mps"]
            error = velocity-sample["command_v_mps"] if name == "push" else velocity
            primary += error**2 / spec["primary"][name] / len(WINDOWS[kind])
    drive = ((sample["world_vx_mps"]-sample["command_v_mps"])**2/spec["drive"]
             if spec["drive"] is not None and in_window(time_s, (1.,4.)) else 0.)
    delta_heat = sample["hip_heat_nm2s"]-previous_heat
    if heat_scale <= 0 or delta_heat < -1e-9:
        raise ValueError("无效热量尺度或热量积分倒退")
    action = np.asarray(sample["action"], dtype=float)[:3]
    change = float(np.square(action-np.asarray(previous_action, dtype=float)[:3]).sum())
    pose = ((sample["pitch_rad"]/.1)**2+(sample["roll_rad"]/.1)**2)*POLICY_DT/spec["duration"]
    magnitude = float(np.square(action).sum())*POLICY_DT/spec["duration"]
    result = dict(primary=primary, drive=drive, heat=max(0.,delta_heat)/heat_scale,
                  pose=pose, action_change=change, action_magnitude=magnitude)
    if not all(math.isfinite(value) and value >= 0 for value in result.values()):
        raise ValueError("训练成本出现无效数值")
    return result


def guard_cost(baseline, candidate):
    """用原逐例及运动质量容差惩罚最严重超限；平均收益/热量仍由独立评价裁决。"""
    if not candidate["metrics"]["passed"]:
        return 100., {"physical_failure": 100.}
    checks = acceptance([baseline], [candidate])["cases"][0]["checks"]
    quality = motion_quality_acceptance([baseline], [candidate])["cases"][0]["checks"]
    violations = {}
    for name, check in [("control/"+k,v) for k,v in checks.items()] + [("motion/"+k,v) for k,v in quality.items()]:
        if not check["pass"]:
            allowed = check["allowed_increase"]
            violations[name] = min(10., max(0., (check["delta"]-allowed)/max(allowed, 1e-12)))
    return max(violations.values(), default=0.), violations


def weighted_cost(terms, weights):
    return sum(terms[name]*weights[name] for name in terms)


def score_trace(scenario, baseline, candidate, heat_scale, weights, gamma=.999):
    """离线检查与在线训练共用的逐步成本定义。"""
    spec = baseline_spec(baseline)
    previous_action, previous_heat = np.zeros(3), 0.
    totals = {name:0. for name in ("primary", "drive", "heat", "pose", "action_change", "action_magnitude")}
    rewards = []
    for index,sample in enumerate(candidate["trace"]):
        if not candidate["metrics"]["passed"] and index == len(candidate["trace"])-1:
            # 与在线包装一致：物理失败末帧可能无效，跳过密集项后固定罚100。
            rewards.append(0.)
            break
        terms = step_costs(scenario, sample, previous_action, previous_heat, spec, heat_scale)
        for name,value in terms.items():
            totals[name] += value
        rewards.append(-weighted_cost(terms, weights))
        previous_action, previous_heat = sample["action"], sample["hip_heat_nm2s"]
    penalty, violations = guard_cost(baseline, candidate)
    if not rewards:
        raise ValueError("缺少轨迹")
    rewards[-1] -= (100. if not candidate["metrics"]["passed"] else weights["guard"]*penalty)
    return dict(return_sum=float(sum(rewards)),
        discounted_return=float(sum(value*gamma**i for i,value in enumerate(rewards))),
        dense_costs=totals, guard_cost=penalty, violations=violations)
