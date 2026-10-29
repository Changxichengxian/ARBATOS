"""验证保留余量后的全高度操作组合：急反打、转向、侧倾、升降和停车。"""
import argparse
import json
import math
from pathlib import Path

import BenchFirmware as bench

sim = bench.sim


def run(name, height, lag_ms, seed=0):
    profile = sim.load_profile()
    profile['assumed']['motor_torque_lag_s'] = lag_ms/1000
    model = sim.mujoco.MjModel.from_xml_string(sim.model_xml(profile))
    data = sim.mujoco.MjData(model)
    data.qpos[:] = sim.pose(model, sim.make_config(profile), height, sim.np.zeros(3))
    sim.mujoco.mj_forward(model, data)
    controller = bench.Controller(model, profile, 'bench-height')
    random = sim.np.random.default_rng(seed)
    next_command = 2.
    v = yaw = roll = 0.
    goal = height
    peak = peak_wheel = 0.
    min_goal, max_goal = height, height
    late = []
    for step in range(32000):
        t = step*.001
        if 2 <= t < 24:
            if name == 'height-cycle':
                goal = .12 if t < 9 or t >= 16 else .06
                v = .8*(-1 if int(t/2.5)%2 else 1)
                yaw = 3.6*(-1 if int(t/1.3)%2 else 1)
                roll = -1 if int(t/4)%2 else 1
            elif t >= next_command:
                v = float(random.choice([-.8, 0., .8]))
                yaw = float(random.choice([-3.6, 0., 3.6]))
                roll = float(random.choice([-1., 0., 1.]))
                next_command = t+float(random.uniform(.2, 1.5))
        elif t >= 24:
            v = yaw = roll = 0.
            goal = .09 if name == 'height-cycle' else height
        if step % 3 == 0:
            controller.step(data, v, yaw, goal, roll_axis=roll)
            value = bench.observe(data)
            peak = max(peak, abs(value.pitch), abs(value.roll))
            peak_wheel = max(peak_wheel, *map(abs, value.wheelRate))
            min_goal = min(min_goal, controller.length)
            max_goal = max(max_goal, controller.length)
            if not controller.enabled:
                break
            assert abs(controller.native.speed)+.09*abs(controller.native.yawRate) <= .70001
            if t > 30:
                late.append(float(sim.np.linalg.norm(data.qvel[:2])))
        controller.apply(data)
        sim.mujoco.mj_step(model, data)
    return dict(case=name, height_mm=height*1000, lag_ms=lag_ms, seed=seed,
                enabled=controller.enabled, time=float(data.time), peak_deg=math.degrees(peak),
                peak_wheel_rate=peak_wheel, height_command_range_mm=[min_goal*1000, max_goal*1000],
                late_mean_speed=float(sim.np.mean(late)) if late else None,
                passed=bool(controller.enabled and late and sim.np.mean(late)<.035 and peak<math.radians(18)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--lag-ms', type=float, default=3.)
    args = parser.parse_args()
    results = []
    args.output.parent.mkdir(parents=True, exist_ok=True)
    for name, height, seed in [('height-cycle', .06, 0)]+[
            ('random', h/1000, seed) for h in (60, 70, 80, 85, 90, 100, 110, 120) for seed in (11, 29)]:
        result = run(name, height, args.lag_ms, seed)
        results.append(result)
        print(json.dumps(result), flush=True)
        args.output.write_text(json.dumps(results, indent=2), encoding='utf-8')
    assert all(row['passed'] for row in results)


if __name__ == '__main__':
    main()
