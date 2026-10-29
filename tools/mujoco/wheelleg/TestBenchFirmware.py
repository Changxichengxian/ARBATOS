"""在三维接触模型中运行实际板凳 C 控制代码，同时检查输出拒绝条件。"""
import ctypes as C
import argparse
import json
import math
import sys
from pathlib import Path
import RunMeasuredWheelLeg as sim

parser = argparse.ArgumentParser()
parser.add_argument('--quick-wheel', action='store_true')
parser.add_argument('--quick-drive', action='store_true')
parser.add_argument('--speed', type=float, default=.5)
parser.add_argument('--turn', type=float, default=3.6)
parser.add_argument('--output', type=Path)
parser.add_argument('--height', action='store_true')
parser.add_argument('--bias-seconds', type=float, default=30.)
parser.add_argument('--lag-ms', type=float)
parser.add_argument('--legacy-damping', action='store_true')
parser.add_argument('--rx-jitter', action='store_true', help='反馈接收时间交替提前 2 ms，覆盖 0042 的 1/5 ms 间隔')
args = parser.parse_args()

from BenchFirmware import F2, F3, F4, F6, Input, State, Output, Wheel, lib, observe, apply
root = sim.ROOT
# 实物曾三次跨过 +/-12.5 边界；两个方向都应连续，重复帧不能再次积分。
for direction in (-1, 1):
    wheel = Wheel()
    assert lib.WheelLegBenchWheelObserve(C.byref(wheel), direction*12.49, 0, 100)
    assert lib.WheelLegBenchWheelObserve(C.byref(wheel), -direction*12.49, direction*13, 110)
    assert abs(wheel.position - direction*.02) < 2e-6
    before = bytes(wheel)
    assert lib.WheelLegBenchWheelObserve(C.byref(wheel), -direction*12.49, 0, 110)
    assert bytes(wheel) == before
    assert not lib.WheelLegBenchWheelObserve(C.byref(wheel), -direction*12.49, 51, 112)
    assert not lib.WheelLegBenchWheelObserve(C.byref(wheel), -direction*12.49, 0, 191)
wheel = Wheel()
assert lib.WheelLegBenchWheelObserve(C.byref(wheel), 0, 0, 0xfffffff0)
assert lib.WheelLegBenchWheelObserve(C.byref(wheel), .02, 0, 4)
assert abs(wheel.position - .02) < 1e-6
assert not lib.WheelLegBenchWheelObserve(C.byref(wheel), 3, 0, 6)
for name in ('WheelLegBenchJointFromRaw', 'WheelLegBenchJointToRaw'):
    function = getattr(lib, name)
    function.argtypes = [C.c_uint, C.c_float]
    function.restype = C.c_float
# 来自用户“拉平、前面竖直、后面竖直”三份原始记录，不从实现里的方向表推导期望值。
calibration = [(2, .340466926, -1.197642481), (1, -1.014915694, .496490425),
               (4, -1.029411765, .537308309), (3, .293926909, -1.273937591)]
for i, (motor_id, horizontal, vertical) in enumerate(calibration):
    assert abs(lib.WheelLegBenchJointFromRaw(i, vertical)) < 1e-6
    angle = lib.WheelLegBenchJointFromRaw(i, horizontal)
    expected = math.pi / 2 if i % 2 == 0 else -math.pi / 2
    assert abs(angle - expected) < .07, (motor_id, angle, expected)
    assert abs(lib.WheelLegBenchJointToRaw(i, angle) - horizontal) < 1e-6
profile = sim.load_profile()
if args.lag_ms is not None:
    profile['assumed']['motor_torque_lag_s'] = args.lag_ms / 1000
cfg = sim.make_config(profile)
model = sim.mujoco.MjModel.from_xml_string(sim.model_xml(profile))
# 实际 0.45 是主控前馈上限，电机本地阻尼合成后受协议的 1 N·m 范围约束。
model.actuator_ctrlrange[[2, 5]] = [-1., 1.]


data = sim.mujoco.MjData(model)
data.qpos[:] = sim.pose(model, cfg, .1, sim.np.zeros(3))
sim.mujoco.mj_forward(model, data)
nominal = observe(data)
state, out = State(), Output()
assert lib.WheelLegBenchStep(C.byref(state), C.byref(nominal), C.byref(out))
assert max(abs(out.torque[i]) for i in (2,5)) < 1e-5
hold = list(state.hold)
moved = Input.from_buffer_copy(nominal)
moved.joint[0] += .005
assert lib.WheelLegBenchStep(C.byref(state), C.byref(moved), C.byref(out))
assert max(abs(a-b) for a,b in zip(state.hold, hold)) <= .001801
lib.WheelLegBenchReset(C.byref(state))
assert lib.WheelLegBenchStep(C.byref(state), C.byref(moved), C.byref(out))
assert abs(state.hold[0] - moved.joint[0]) <= .001801
for height, dt in ((float('nan'), .003), (.059, .003), (.121, .003), (.1, .02)):
    before = bytes(state)
    assert not lib.WheelLegBenchSetHeight(C.byref(state), height, dt)
    assert bytes(state) == before
for axis, dt in ((float('nan'), .003), (1., .02)):
    before = bytes(state)
    assert not lib.WheelLegBenchHeightAxis(C.byref(state), axis, dt)
    assert bytes(state) == before
before = list(state.hold)
assert lib.WheelLegBenchHeightAxis(C.byref(state), 0., .003)
assert list(state.hold) == before
invalid = 0
for index in range(C.sizeof(Input)//4):
    bad = Input.from_buffer_copy(nominal)
    C.cast(C.pointer(bad), C.POINTER(C.c_float))[index] = float('nan')
    state, out = State(), Output()
    assert not lib.WheelLegBenchStep(C.byref(state), C.byref(bad), C.byref(out)), index
    assert list(out.torque) == [0]*6
    invalid += 1
for attr, value in [('pitch', .4), ('roll', -.4), ('dt', .02)]:
    bad = Input.from_buffer_copy(nominal)
    setattr(bad, attr, value)
    assert not lib.WheelLegBenchStep(C.byref(State()), C.byref(bad), C.byref(Output()))

for wheel in range(2):
    for rate in (-153.5, -35.1, 35.1, 153.5):
        bad = Input.from_buffer_copy(nominal)
        bad.wheelRate[wheel] = rate
        for ready in (0, 1):
            state, out = State(ready=ready), Output()
            assert not lib.WheelLegBenchStep(C.byref(state), C.byref(bad), C.byref(out))
            assert list(out.torque) == [0]*6
    bad = Input.from_buffer_copy(nominal)
    bad.wheelRate[wheel] = 13.7435913  # 本次静止左轮的异常反馈不能直接接入平衡。
    assert not lib.WheelLegBenchStep(C.byref(State()), C.byref(bad), C.byref(Output()))

results = []
quick_drive = '--quick-drive' in sys.argv
quick_wheel = '--quick-wheel' in sys.argv or quick_drive
cases = [('stand', 0), ('tilt-plus', 3), ('tilt-minus', -3)]
if quick_wheel:
    cases += [('pitch-bias-plus', 0), ('pitch-bias-minus', 0),
              ('drive-stop', 0), ('reverse-stop', 0), ('turn', 0), ('turn-reverse', 0),
              ('drive-turn', 0), ('reverse-turn', 0)]
if not quick_wheel:
    cases += [('drive-stop', 0), ('turn', 0), ('push', 0)]
if quick_drive:
    cases = [('drive-stop', 0), ('reverse-stop', 0), ('turn', 0), ('turn-reverse', 0),
             ('drive-turn', 0), ('reverse-turn', 0)]
for name, angle in cases:
    data = sim.mujoco.MjData(model)
    angle = math.radians(angle)
    data.qpos[:] = sim.pose(model, cfg, .09 if quick_wheel else .1, sim.np.array([-angle, 0, angle]))
    sim.mujoco.mj_forward(model, data)
    state, out = State(), Output()
    wheel_observers, wheel_origins = [Wheel(), Wheel()], None
    lib.BenchTrimReset()
    trim = 0.
    pitch_bias = .04 if name == 'pitch-bias-plus' else -.04 if name == 'pitch-bias-minus' else 0.
    late_speed = []
    peak = 0
    moving_speed, moving_yaw = [], []
    stopped = False
    duration_steps = round(args.bias_seconds * 1000) if pitch_bias else 10000
    for step in range(duration_steps):
        t = step * .001
        data.xfrc_applied[:] = 0
        if name == 'push' and 2 < t < 2.15:
            data.xfrc_applied[model.body('base').id, 0] = 2
        if step % 3 == 0:
            command_v = (args.speed if name in ('drive-stop', 'drive-turn') else -args.speed if name in ('reverse-stop', 'reverse-turn') else 0) if 1 < t < 4 else 0
            command_yaw = (args.turn if name in ('turn', 'drive-turn') else -args.turn if name in ('turn-reverse', 'reverse-turn') else 0) if 1 < t < 4 else 0
            values = observe(data, command_v, command_yaw)
            if quick_wheel:
                values.pitch += pitch_bias
                values.pitchTrim = trim
                if wheel_origins is None:
                    wheel_origins = list(values.wheel)
                for i, wheel in enumerate(wheel_observers):
                    raw = (values.wheel[i] - wheel_origins[i] + 12.49 + 12.5) % 25 - 12.5
                    raw = round((raw + 12.5) * 65535 / 25) * 25 / 65535 - 12.5
                    # 0038 回中时原始轮速均方根约 2.6 rad/s；此为同量级压力输入，不是假定噪声已辨识。
                    report = values.wheelRate[i] + 3.7 * math.sin(2*math.pi*37.3*t) if wheel.ready else 0
                    rx_tick = step-(2 if args.rx_jitter and (step//3)%2 else 0)
                    assert lib.WheelLegBenchWheelObserve(C.byref(wheel), raw, report, rx_tick)
                    values.wheel[i], values.wheelRate[i] = wheel.position, wheel.rate
            peak = max(peak, abs(values.pitch), abs(values.roll))
            if not lib.WheelLegBenchStep(C.byref(state), C.byref(values), C.byref(out)):
                stopped = True
                break
            if args.legacy_damping:
                out.wheelTargetRate[:] = [0., 0.]
            if quick_wheel:
                before_trim = trim
                trim = lib.BenchTrimStep(out.state[3]+state.speed+state.parkSpeed, values.dt,
                    abs(values.pitch)<.12 and abs(values.gyro[1])<.5 and abs(out.state[3]+state.speed)<.25,
                    state.speed, state.yawRate)
                assert abs(trim) <= .060001
                if abs(state.speed) > .001 or abs(state.yawRate) > .001:
                    assert trim == before_trim, '主动操作期间平衡补偿不应变化'
            if 3 < t < 4:
                # 转弯后世界 X 速度不能代表车头方向的行驶速度。
                moving_speed.append(float((data.xmat[model.body('base').id].reshape(3, 3).T @ data.qvel[:3])[0]))
                moving_yaw.append(float(values.gyro[2]))
            assert max(abs(out.torque[i]) for i in (2, 5)) <= .450001
            assert max(abs(out.torque[i]) for i in (0, 1, 3, 4)) <= 1.500001
        apply(data, out)
        sim.mujoco.mj_step(model, data)
        if pitch_bias and step >= duration_steps - 2000:
            late_speed.append(float(data.qvel[0]))
    final = observe(data)
    mean_speed = sum(late_speed)/len(late_speed) if late_speed else float(data.qvel[0])
    mean_moving_speed = sum(moving_speed)/len(moving_speed) if moving_speed else 0.
    mean_moving_yaw = sum(moving_yaw)/len(moving_yaw) if moving_yaw else 0.
    expected_v = args.speed if name in ('drive-stop', 'drive-turn') else -args.speed if name in ('reverse-stop', 'reverse-turn') else 0.
    expected_yaw = args.turn if name in ('turn', 'drive-turn') else -args.turn if name in ('turn-reverse', 'reverse-turn') else 0.
    demand = abs(expected_v)+.09*abs(expected_yaw)
    if demand > .70:
        expected_v *= .70/demand
        expected_yaw *= .70/demand
    # “能走、能停”不能替代速度跟踪；转弯也按车身前向速度评分。
    tolerance_v = .08 if 'turn' in name else .05
    motion_ok = abs(mean_moving_speed - expected_v) < tolerance_v and abs(mean_moving_yaw - expected_yaw) < .3
    if pitch_bias:
        motion_ok = True  # 偏置场景按后面两秒的收敛条件评分。
    result = dict(case=name, duration_s=duration_steps/1000, stopped=stopped, peak_angle_deg=math.degrees(peak),
                  final_pitch_deg=math.degrees(final.pitch), final_speed=float(data.qvel[0]),
                  mean_late_speed=mean_speed, trim_deg=math.degrees(trim),
                  moving_speed=mean_moving_speed, moving_yaw_rate=mean_moving_yaw,
                  final_yaw_rate=float(final.gyro[2]),
                  passed=bool(not stopped and abs(final.pitch) < .08 and abs(data.qvel[0]) < .08
                              and abs(final.gyro[2]) < .08 and motion_ok
                              and (not pitch_bias or abs(mean_speed)<.01)))
    results.append(result)
    print(json.dumps(result), flush=True)
output = args.output or (root / 'local/cache/wheelleg-joystick-20261029/90mm摇杆采集检查.json' if quick_wheel
          else root / 'local/cache/wheelleg-resume-20261026/板凳固件仿真检查.json')
output.parent.mkdir(parents=True, exist_ok=True)
output.write_text(json.dumps({'invalid_inputs': invalid+3, 'speed_target': args.speed,
                  'yaw_target': args.turn, 'legacy_damping': args.legacy_damping,
                  'wheel_torque_lag_s': profile['assumed'].get('motor_torque_lag_s', 0), 'cases': results,
                  'passed': all(r['passed'] for r in results)}, indent=2), encoding='utf-8')
assert all(r['passed'] for r in results)
if args.height:
    from BenchFirmware import Controller
    height_results = []
    for speed, yaw in ((0., 0.), (.2, .6), (-.2, -.6), (.5, 0.), (-.5, 0.)):
        data = sim.mujoco.MjData(model)
        data.qpos[:] = sim.pose(model, cfg, .09, sim.np.zeros(3))
        sim.mujoco.mj_forward(model, data)
        controller = Controller(model, profile, 'bench-height')
        peak, lengths = 0., []
        for step in range(18000):
            t = step * .001
            target = .09 if t < 2 or t >= 11 else .105 if t < 6.5 else .11
            if step % 3 == 0:
                controller.step(data, speed if 2<t<14 else 0., yaw if 2<t<14 else 0., target)
                if not controller.enabled:
                    break
                values = observe(data)
                peak = max(peak, abs(values.pitch), abs(values.roll))
                if 6<t<6.5 or 10.5<t<11 or 16<t:
                    lengths.append((target, *controller.leg_lengths))
            controller.apply(data)
            sim.mujoco.mj_step(model, data)
        error = max(abs(length-target) for target,*legs in lengths for length in legs) if lengths else 1.
        row = dict(speed=speed, yaw=yaw, enabled=controller.enabled,
                   peak_angle_deg=math.degrees(peak), max_settled_length_error_m=float(error),
                   final_speed=float(sim.np.linalg.norm(data.qvel[:2])),
                   passed=bool(controller.enabled and error<.006 and sim.np.linalg.norm(data.qvel[:2])<.08))
        height_results.append(row)
        print(json.dumps(row), flush=True)
    output.with_name(output.stem+'-变高度.json').write_text(json.dumps(height_results, indent=2), encoding='utf-8')
    assert all(row['passed'] for row in height_results)
if quick_wheel:
    sys.exit(0)

class AutoState(C.Structure):
    _fields_ = [('balance', State), ('start', F4), ('target', F4),
                ('elapsed', C.c_float), ('duration', C.c_float), ('settled', C.c_float),
                ('phase', C.c_uint32), ('reject', C.c_uint32), ('rejectValue', C.c_float)]
lib.WheelLegBenchAutoStep.argtypes = [C.POINTER(AutoState), C.POINTER(Input), C.POINTER(Output)]
lib.WheelLegBenchAutoStep.restype = C.c_int
calibration_auto, calibration_out = AutoState(), Output()
assert lib.WheelLegBenchAutoStep(C.byref(calibration_auto), C.byref(nominal), C.byref(calibration_out))
raw_targets = []
for i, (motor_id, horizontal, vertical) in enumerate(calibration):
    raw = lib.WheelLegBenchJointToRaw(i, calibration_auto.target[i])
    assert abs(raw) < 12.5 and abs(calibration_auto.target[i]) < 2.4, (motor_id, raw)
    raw_targets.append(dict(id=motor_id, raw_target_rad=raw, model_target_rad=calibration_auto.target[i]))
print(json.dumps(dict(calibrated_centered_targets=raw_targets)), flush=True)
for joint in range(4):
    for direction in (-1., 1.):
        fast = Input.from_buffer_copy(nominal)
        fast.jointRate[joint] = direction*3.01
        auto, out = AutoState(), Output()
        assert not lib.WheelLegBenchAutoStep(C.byref(auto), C.byref(fast), C.byref(out))
        assert auto.reject == 20+joint and list(out.torque) == [0]*6 and out.jointDamping == 0
        assert not lib.WheelLegBenchStep(C.byref(State()), C.byref(fast), C.byref(out))
        assert list(out.torque) == [0]*6 and out.jointDamping == 0

auto_results = []
for height in (.149, .12, .085):
    data = sim.mujoco.MjData(model)
    data.qpos[:] = sim.pose(model, cfg, height, sim.np.zeros(3))
    sim.mujoco.mj_forward(model, data)
    auto, out = AutoState(), Output()
    entered = None
    for step in range(15000):
        # 对应人扶住机身保持直立；腿仍承重，达到目标后松手。
        if auto.phase != 3:
            data.qpos[3:7] = (1, 0, 0, 0)
            data.qvel[3:6] = 0
        if step % 3 == 0:
            values = observe(data)
            accepted = lib.WheelLegBenchAutoStep(C.byref(auto), C.byref(values), C.byref(out))
            assert accepted, (height, step, auto.phase, auto.elapsed, list(values.joint), list(auto.target))
            if auto.phase != 3:
                assert out.torque[2] == 0 and out.torque[5] == 0
                assert max(abs(out.torque[i]) for i in (0,1,3,4)) <= 1.000001
            elif entered is None:
                entered = step*.001
        apply(data, out)
        sim.mujoco.mj_step(model, data)
    final = observe(data)
    row = dict(start_length_m=height, balance_entered_s=entered,
               final_lengths=list(out.length), final_pitch_deg=math.degrees(final.pitch),
               passed=bool(entered is not None and abs(final.pitch)<.08 and max(abs(x-.060) for x in out.length)<.006))
    auto_results.append(row)
    print(json.dumps(row), flush=True)
    assert row['passed']

# 堵住关节时必须因跟踪误差退出，不能宣告到位或给轮子力矩。
data = sim.mujoco.MjData(model)
data.qpos[:] = sim.pose(model, cfg, .149, sim.np.zeros(3))
sim.mujoco.mj_forward(model, data)
frozen = observe(data)
auto, out = AutoState(), Output()
rejected = False
for _ in range(4000):
    if not lib.WheelLegBenchAutoStep(C.byref(auto), C.byref(frozen), C.byref(out)):
        rejected = True
        break
    assert auto.phase != 3 and out.torque[2] == 0 and out.torque[5] == 0
assert rejected and list(out.torque) == [0]*6
(output.parent/'自动收腿仿真检查.json').write_text(json.dumps(
    dict(cases=auto_results, blocked_joint_rejected=rejected, passed=True), indent=2),encoding='utf8')

# 收腿后可以等候落地；未开放轮子时，长时间保持也不得进入平衡。
lib.WheelLegBenchPoseStep.argtypes = [C.POINTER(AutoState), C.POINTER(Input), C.POINTER(Output), C.c_uint32]
lib.WheelLegBenchPoseStep.restype = C.c_int
pose_results = []
for height in (.149, .085):
    data = sim.mujoco.MjData(model)
    data.qpos[:] = sim.pose(model, cfg, height, sim.np.zeros(3))
    sim.mujoco.mj_forward(model, data)
    auto, out = AutoState(), Output()
    ready_at = None
    entered = None
    for step in range(20000):
        allowed = 14000 <= step < 18000
        if auto.phase != 3:
            data.qpos[3:7] = (1, 0, 0, 0)
            data.qvel[3:6] = 0
        if step % 3 == 0:
            values = observe(data)
            assert lib.WheelLegBenchPoseStep(C.byref(auto), C.byref(values), C.byref(out), allowed), (height, step, auto.reject)
            if auto.phase == 4 and ready_at is None:
                ready_at = step * .001
            if not allowed:
                assert auto.phase != 3 and out.torque[2] == 0 and out.torque[5] == 0
            elif auto.phase == 3 and entered is None:
                entered = step * .001
        apply(data, out)
        sim.mujoco.mj_step(model, data)
    assert ready_at is not None and ready_at < 14 and entered is not None
    assert auto.phase == 4 and out.torque[2] == 0 and out.torque[5] == 0
    row = dict(start_length_m=height, hold_ready_s=ready_at, balance_entered_s=entered,
               final_lengths=list(out.length), wheels_stop_retains_pose=True, passed=True)
    pose_results.append(row)
    print(json.dumps(row), flush=True)
(output.parent/'收腿保持分段检查.json').write_text(json.dumps(pose_results, indent=2), encoding='utf8')

# 实物记录显示关节输出滞后，单看即时力矩模型会漏掉振荡。
from collections import deque
lag_results = []
for height in (.149, .085):
    for delay_ms, lag_ms, gain in ((12, 3, 1.), (20, 10, 1.), (20, 10, .65)):
        data = sim.mujoco.MjData(model)
        data.qpos[:] = sim.pose(model, cfg, height, sim.np.zeros(3))
        sim.mujoco.mj_forward(model, data)
        auto, out = AutoState(), Output()
        commands = deque([(sim.np.zeros(6), sim.np.array(observe(data).joint)) for _ in range(delay_ms)])
        actual = sim.np.zeros(6)
        entered, rejected, peak_rate, late_peak, peak_time = None, None, 0., 0., 0.
        alpha = 1. - math.exp(-1. / lag_ms)
        for step in range(15000):
            if auto.phase != 3:
                data.qpos[3:7] = (1, 0, 0, 0)
                data.qvel[3:6] = 0
            if step % 3 == 0:
                values = observe(data)
                if not lib.WheelLegBenchAutoStep(C.byref(auto), C.byref(values), C.byref(out)):
                    rejected = int(auto.reject)
                    break
                if auto.phase == 3 and entered is None: entered = step*.001
            commands.append((sim.np.array(out.torque), sim.np.array(out.jointTarget)))
            delayed, delayed_target = commands.popleft()
            data.ctrl[:] = out.torque
            for i, side in enumerate(sim.SIDES):
                data.ctrl[i*3+2] -= out.wheelDamping * data.joint(f'{side}_wheel_hinge').qvel[0]
                for j, end in enumerate(('front','back')):
                    k = i*3+j
                    rate = data.joint(f'{side}_{end}_hinge').qvel[0]
                    if abs(float(rate)) > peak_rate:
                        peak_rate, peak_time = abs(float(rate)), step*.001
                    if step >= 150: late_peak = max(late_peak, abs(float(rate)))
                    position = data.joint(f'{side}_{end}_hinge').qpos[0]
                    effort = delayed[k] + out.jointStiffness*(delayed_target[i*2+j]-position) - out.jointDamping*rate
                    actual[k] += alpha*(gain*effort-actual[k])
                    data.ctrl[k] = actual[k]
            sim.mujoco.mj_step(model, data)
        row = dict(start_length_m=height, command_delay_ms=delay_ms, current_lag_ms=lag_ms,
                   effort_gain=gain, balance_entered_s=entered, rejected=rejected,
                   peak_joint_rate_rad_s=peak_rate, peak_time_s=peak_time,
                   rate_after_support_ramp_rad_s=late_peak, final_lengths=list(out.length),
                   passed=rejected is None and entered is not None and peak_rate < 3. and late_peak < 1.5)
        lag_results.append(row)
        print(json.dumps(row),flush=True)
lag_path = output.parent/'板凳延迟仿真检查.json'
lag_path.write_text(json.dumps(lag_results,indent=2),encoding='utf8')
assert all(r['passed'] for r in lag_results)
