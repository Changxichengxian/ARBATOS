"""分高度执行平地满杆反向和满转向，测量可通过的速度，再保留 20% 余量。"""
import argparse
import json
import math
from pathlib import Path

import BenchFirmware as bench

sim = bench.sim


def run(height, speed, lag_ms=3., turn=3.6):
    profile = sim.load_profile()
    profile['assumed']['motor_torque_lag_s'] = lag_ms/1000
    model = sim.mujoco.MjModel.from_xml_string(sim.model_xml(profile))
    data = sim.mujoco.MjData(model)
    data.qpos[:] = sim.pose(model, sim.make_config(profile), height, sim.np.zeros(3))
    sim.mujoco.mj_forward(model, data)
    controller = bench.Controller(model, profile, 'bench-height')
    peak = peak_rate = peak_actual = 0.
    late = []
    for step in range(20000):
        t = step*.001
        v = speed if 2<t<5 or 8<t<11 else -speed if 5<=t<8 or 11<=t<14 else 0.
        yaw = turn if 8<=t<11 else -turn if 11<=t<14 else 0.
        if step % 3 == 0:
            controller.step(data, v, yaw, height)
            value = bench.observe(data)
            peak = max(peak, abs(value.roll), abs(value.pitch))
            peak_rate = max(peak_rate, *map(abs, value.wheelRate))
            peak_actual = max(peak_actual, float(sim.np.linalg.norm(data.qvel[:2])))
            if not controller.enabled:
                break
            if t > 18:
                late.append(float(sim.np.linalg.norm(data.qvel[:2])))
        controller.apply(data)
        sim.mujoco.mj_step(model, data)
    return dict(height_mm=round(height*1000), requested_speed=speed, turn=turn, lag_ms=lag_ms,
                enabled=controller.enabled, final_time=float(data.time),
                peak_angle_deg=math.degrees(peak), peak_wheel_rate=peak_rate,
                peak_actual_speed=peak_actual, late_mean_speed=float(sim.np.mean(late)) if late else None,
                passed=bool(controller.enabled and late and sim.np.mean(late)<.03 and peak<math.radians(18)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--heights', default='60,70,80,90,100,110,120')
    parser.add_argument('--speeds', default='0.5,0.6,0.7,0.8,0.9,1.0,1.1')
    parser.add_argument('--lag-ms', type=float, default=3.)
    parser.add_argument('--turn', type=float, default=3.6)
    args = parser.parse_args()
    results = []
    args.output.parent.mkdir(parents=True, exist_ok=True)
    for height in map(float, args.heights.split(',')):
        for speed in map(float, args.speeds.split(',')):
            result = run(height/1000, speed, args.lag_ms, args.turn)
            results.append(result)
            print(json.dumps(result), flush=True)
            args.output.write_text(json.dumps(results, indent=2), encoding='utf-8')


if __name__ == '__main__':
    main()
