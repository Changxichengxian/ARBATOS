"""复现满速直接反向，并检查升降、停车和连续换向。"""
import argparse
import json
import math
from pathlib import Path

import BenchFirmware as bench

sim = bench.sim
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--lag-ms', type=float, default=3.)
args = parser.parse_args()
profile = sim.load_profile()
profile['assumed']['motor_torque_lag_s'] = args.lag_ms / 1000
cfg = sim.make_config(profile)
results = []

for name, direction, height, yaw, interval in (
        ('forward-reverse', 1, .09, 0., 3.),
        ('reverse-forward', -1, .09, 0., 3.),
        ('high-forward-reverse', 1, .11, 0., 3.),
        ('high-reverse-forward', -1, .11, 0., 3.),
        ('turn-forward-reverse', 1, .09, 3.6, 3.),
        ('ch3-while-reversing', 1, .09, 0., 3.),
        ('rapid-reverse', 1, .09, 0., .5)):
    model = sim.mujoco.MjModel.from_xml_string(sim.model_xml(profile))
    data = sim.mujoco.MjData(model)
    data.qpos[:] = sim.pose(model, cfg, .09, sim.np.zeros(3))
    sim.mujoco.mj_forward(model, data)
    controller = bench.Controller(model, profile, 'bench-height')
    rows, moving, peak = [], [], 0.
    height_checkpoints = {}
    stopped_at = None
    for step in range(20000):
        t = step * .001
        target = 0. if t < 4 or t >= 16 else direction * .5 * (-1 if int((t-4)/interval) % 2 else 1)
        turn = yaw if 4 <= t < 16 else 0.
        if step % 3 == 0:
            height_axis = (1. if 2<t<6 else -1. if 8<t<15 else 0.) if name == 'ch3-while-reversing' else None
            controller.step(data, target, turn, height, height_axis=height_axis)
            if name == 'ch3-while-reversing':
                if 6.1<t<7.9:
                    assert abs(controller.length-.12)<.00001
                if 15.1<t:
                    assert abs(controller.length-.06)<.00001
                if step % 300 == 0:
                    height_checkpoints[str(round(t, 2))] = controller.length
            values = bench.observe(data)
            peak = max(peak, abs(values.pitch), abs(values.roll))
            actual = float((data.xmat[model.body('base').id].reshape(3, 3).T @ data.qvel[:3])[0])
            if not controller.enabled:
                stopped_at = t
                break
            rows.append([t, target, actual, controller.speed, values.pitch, values.roll,
                         *controller.output.torque, *controller.leg_lengths])
            if interval == 3. and 4 <= t < 16 and (t-4) % interval > 2:
                moving.append(abs(actual-target))
        controller.apply(data)
        sim.mujoco.mj_step(model, data)
    final_speed = float(sim.np.linalg.norm(data.qvel[:2]))
    row = dict(case=name, enabled=controller.enabled, stopped_at_s=stopped_at,
               height_checkpoints=height_checkpoints,
               peak_angle_deg=math.degrees(peak), final_speed=final_speed,
               moving_mean_abs_error=float(sim.np.mean(moving)) if moving else None,
               passed=bool(controller.enabled and final_speed < .08 and
                           (not moving or sim.np.mean(moving) < .10)))
    results.append(row)
    print(json.dumps(row), flush=True)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    sim.np.savez_compressed(args.output.with_name(args.output.stem+'-'+name+'.npz'), trace=rows)
args.output.write_text(json.dumps({'wheel_torque_lag_ms': args.lag_ms, 'cases': results,
                                  'passed': all(r['passed'] for r in results)}, indent=2), encoding='utf-8')
assert all(r['passed'] for r in results)
