"""对共享固件 VMC 做平地、急反向、转弯、升降和地形回归。"""
import argparse
import json
import math
from pathlib import Path
import RunMeasuredWheelLeg as sim
from VmcFirmware import Controller


def run(case, output):
    profile = sim.load_profile()
    h = case.get('height', .09)
    profile['control']['leg_length_m'] = h
    profile['assumed']['motor_torque_lag_s'] = case.get('lag', .003)
    model = sim.mujoco.MjModel.from_xml_string(sim.model_xml(profile, case.get('terrain', 'flat')))
    data = sim.mujoco.MjData(model)
    data.qpos[:] = sim.pose(model, sim.make_config(profile), h,
                            sim.np.array([0, 0, case.get('pitch', 0)]))
    if case.get('terrain') == 'cross':
        data.qpos[3:7] = [math.cos(math.radians(2)), math.sin(math.radians(2)), 0, 0]
    sim.mujoco.mj_forward(model, data)
    ctrl = Controller(model, profile, 'vmc-height')
    rows, hit_body = [], False
    closure = 0.
    for k in range(14000):
        t = float(data.time)
        speed = case.get('speed', 0.) if 1 < t < 8 else 0.
        if case.get('reverse') and 4 < t < 8:
            speed = -speed
        turn = case.get('turn', 0.) if 2 < t < 8 else 0.
        length = (.12 if 2 < t < 8 else .06) if case.get('height_change') else h
        if k % 3 == 0:
            ctrl.step(data, speed, turn, length)
        ctrl.apply(data)
        sim.mujoco.mj_step(model, data)
        if k % 3 == 0:
            roll, pitch, yaw = sim.legacy.quat_to_euler_wxyz(data.qpos[3:7])
            local = data.xmat[model.body('base').id].reshape(3, 3).T @ data.qvel[:3]
            rows.append([t, roll, pitch, local[0], data.qvel[5], speed, turn,
                         *ctrl.leg_lengths, *ctrl.leg_angles, *data.ctrl, *data.qpos[:2]])
            for side in sim.SIDES:
                closure = max(closure, float(sim.np.linalg.norm(data.site(side+'_front_rod_end').xpos-
                                                               data.site(side+'_back_rod_end').xpos)))
            for contact in data.contact:
                names = [model.geom(int(i)).name for i in contact.geom]
                if 'floor' in names and not any('wheel_geom' in name for name in names):
                    hit_body = True
            if not ctrl.enabled:
                break
    rows = sim.np.array(rows)
    steady = rows[(rows[:, 0] > 6) & (rows[:, 0] < 8)]
    tail = rows[rows[:, 0] > 12]
    result = dict(case=case, active=ctrl.enabled, reject=ctrl.reject, time=float(data.time),
                  peak_pitch_deg=float(sim.np.degrees(abs(rows[:, 2]).max())),
                  peak_roll_deg=float(sim.np.degrees(abs(rows[:, 1]).max())),
                  peak_split_deg=float(sim.np.degrees(abs(rows[:, 9]-rows[:, 10]).max())),
                  steady_speed_error=float(abs(steady[:, 3]-steady[:, 5]).mean()) if len(steady) else None,
                  steady_yaw_error=float(abs(steady[:, 4]-steady[:, 6]).mean()) if len(steady) else None,
                  final_speed=float(abs(tail[:, 3]).mean()) if len(tail) else None,
                  tail_travel_m=float(sim.np.linalg.norm(sim.np.ptp(tail[:, -2:], axis=0))) if len(tail) else None,
                  body_contact=hit_body, closure_m=closure,
                  warnings=int(sum(data.warning.number)))
    result['passed'] = bool(ctrl.enabled and data.time > 13.99 and not hit_body and
                            not result['warnings'] and closure < .001 and
                            result['peak_pitch_deg'] < 15 and result['peak_roll_deg'] < 12 and
                            result['peak_split_deg'] < 8 and result['steady_speed_error'] < .10 and
                            result['steady_yaw_error'] < .25 and result['final_speed'] < .03 and
                            result['tail_travel_m'] < .05)
    sim.np.savez_compressed(output / (case['name']+'.npz'), trace=rows)
    print(json.dumps(result), flush=True)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--case')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    cases = [dict(name=f'{round(h*1000)}-{name}', height=h, **kw)
             for h in (.06, .09, .12)
             for name, kw in [('stand', {}), ('tilt', {'pitch': .05}),
                              ('reverse', {'speed': .5, 'reverse': True}),
                              ('turn', {'speed': .3, 'turn': 1.2})]]
    cases += [dict(name='height-drive', height=.06, height_change=True, speed=.3),
              dict(name='spin-left', turn=3.6), dict(name='spin-right', turn=-3.6),
              dict(name='lag1-reverse', speed=.5, reverse=True, lag=.001),
              dict(name='lag6-reverse', speed=.5, reverse=True, lag=.006)]
    cases += [dict(name='60-full-turn', height=.06, speed=.7, turn=3.6),
              dict(name='120-full-turn', height=.12, speed=.5, turn=3.6),
              dict(name='60-fast-reverse', height=.06, speed=.7, reverse=True)]
    cases += [dict(name=terrain, terrain=terrain, speed=.15 if 'ramp' in terrain else 0.)
              for terrain in ('left-ramp', 'right-ramp', 'cross', 'uphill', 'downhill')]
    results = [run(c, args.output) for c in cases if not args.case or c['name'] == args.case]
    sim.write_json(args.output/'summary.json', {'cases': results, 'passed': bool(results) and all(r['passed'] for r in results)})
    if not results or not all(r['passed'] for r in results):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
