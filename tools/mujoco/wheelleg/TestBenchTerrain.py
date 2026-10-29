"""用实际板凳控制代码检查横坡、单侧高差、驻坡和遥控侧倾。"""
import argparse
import inspect
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import BenchFirmware as bench

sim = bench.sim


def run(case, output):
    profile = sim.load_profile()
    tree = ET.fromstring(sim.model_xml(profile, case.get('terrain', 'flat')))
    ground = tree.find('worldbody/geom')
    cross = math.radians(case.get('cross', 0))
    grade = math.radians(case.get('grade', 0))
    ground.set('euler', f'{cross} {-grade} 0')
    model = sim.mujoco.MjModel.from_xml_string(ET.tostring(tree, encoding='unicode'))
    data = sim.mujoco.MjData(model)
    data.qpos[:] = sim.pose(model, sim.make_config(profile), .09, sim.np.zeros(3))
    data.qpos[3:7] = [math.cos(cross/2), math.sin(cross/2), 0, 0]
    controller = bench.Controller(model, profile, 'bench-height')
    can_roll = 'roll_axis' in inspect.signature(controller.step).parameters
    sim.mujoco.mj_forward(model, data)
    rows = []
    peak = 0.
    for step in range(25000):
        t = step*.001
        speed = .15 if case.get('drive') and 2 < t < (14 if case.get('cross_ramp') else 7) else 0.
        roll_axis = case.get('tilt', 0) if 5 < t < 13 else 0.
        if step % 3 == 0:
            kwargs = {'roll_axis': roll_axis} if can_roll else {}
            controller.step(data, speed, 0., .09, **kwargs)
            value = bench.observe(data)
            peak = max(peak, abs(value.pitch), abs(value.roll))
            rows.append([t, data.qpos[0], data.qpos[1], value.pitch, value.roll,
                         data.qvel[0], *controller.leg_lengths, controller.trim,
                         *controller.output.torque])
            if not controller.enabled:
                break
        controller.apply(data)
        sim.mujoco.mj_step(model, data)
    trace = sim.np.array(rows)
    tail = trace[trace[:, 0] > 22]
    middle = trace[(trace[:, 0] > 10) & (trace[:, 0] < 13)]
    parked = trace[trace[:, 0] > (17 if case.get('cross_ramp') else 10 if case.get('drive') else 5)]
    r = dict(case=case, enabled=controller.enabled, final_time=float(data.time),
             peak_angle_deg=math.degrees(peak),
             final_roll_deg=math.degrees(float(sim.np.mean(tail[:, 4]))) if len(tail) else None,
             tilt_roll_deg=math.degrees(float(sim.np.mean(middle[:, 4]))) if len(middle) else None,
             final_speed=float(sim.np.mean(tail[:, 5])) if len(tail) else None,
             park_travel_m=float(sim.np.ptp(parked[:, 1])) if len(parked) else None,
             late_travel_m=float(sim.np.ptp(tail[:, 1])) if len(tail) else None,
             late_speed_rms=float(sim.np.sqrt(sim.np.mean(tail[:, 5]**2))) if len(tail) else None,
             leg_mm=[float(v*1000) for v in trace[-1, 6:8]])
    r['passed'] = bool(controller.enabled and len(tail) and abs(r['final_roll_deg']) < 1.0 and
                       abs(r['final_speed']) < .015 and r['park_travel_m'] < .04 and
                       r['late_travel_m'] < .005 and r['late_speed_rms'] < .02 and
                       (not case.get('tilt') or abs(r['tilt_roll_deg']-10*case['tilt']) < 1.0))
    sim.np.savez_compressed(output.with_name(output.stem+'-'+case['name']+'.npz'), trace=trace)
    return r


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--case')
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    cases = [dict(name='cross-left', cross=4), dict(name='cross-right', cross=-4),
             dict(name='park-up', grade=6), dict(name='park-down', grade=-6),
             dict(name='drive-park-up', grade=6, drive=True),
             dict(name='tilt-positive', tilt=1), dict(name='tilt-negative', tilt=-1),
             dict(name='left-ramp-stop', terrain='left-ramp', drive=True),
             dict(name='right-ramp-stop', terrain='right-ramp', drive=True),
             dict(name='left-ramp-cross', terrain='left-ramp', drive=True, cross_ramp=True)]
    results = []
    for case in cases:
        if args.case and args.case != case['name']:
            continue
        result = run(case, args.output)
        results.append(result)
        print(json.dumps(result), flush=True)
    args.output.write_text(json.dumps(results, indent=2), encoding='utf-8')
    assert results and all(row['passed'] for row in results)


if __name__ == '__main__':
    main()
