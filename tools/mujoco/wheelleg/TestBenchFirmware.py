"""在三维接触模型中运行实际板凳 C 控制代码，同时检查输出拒绝条件。"""
import ctypes as C
import json
import math
import subprocess
from pathlib import Path
import RunMeasuredWheelLeg as sim

F2, F3, F4, F6 = (C.c_float * n for n in (2, 3, 4, 6))
class Input(C.Structure):
    _fields_ = [('joint', F4), ('jointRate', F4), ('wheel', F2), ('wheelRate', F2),
                ('pitch', C.c_float), ('roll', C.c_float), ('yaw', C.c_float),
                ('gyro', F3), ('targetV', C.c_float), ('targetYaw', C.c_float), ('dt', C.c_float)]
class State(C.Structure):
    _fields_ = [('hold', F4), ('feedforward', F4), ('reference', C.c_float),
                ('speed', C.c_float), ('yawRate', C.c_float), ('yawReference', C.c_float),
                ('ready', C.c_uint32), ('driving', C.c_uint32)]
class Output(C.Structure):
    _fields_ = [('torque', F6), ('state', F6), ('length', F2), ('alpha', F2), ('jointDamping', C.c_float)]

root = sim.ROOT
folder = root / 'local/build/host-tests/bench-firmware'
folder.mkdir(parents=True, exist_ok=True)
zig = Path('C:/Users/28111/AppData/Local/Microsoft/WinGet/Links/zig.exe')
includes = [root/'shared/application/wheelleg', root/'shared/application/robot', root/'shared/components/support']
dll = folder / 'bench.dll'
subprocess.run([str(zig), 'cc', '-shared', '-O2', '-std=c99', '-Wall', '-Wextra', '-Werror',
                *['-I'+str(p) for p in includes], str(root/'shared/application/wheelleg/WheelLegBench.c'),
                '-o', str(dll)], check=True)
lib = C.CDLL(str(dll))
lib.WheelLegBenchStep.argtypes = [C.POINTER(State), C.POINTER(Input), C.POINTER(Output)]
lib.WheelLegBenchStep.restype = C.c_int
lib.WheelLegBenchReset.argtypes = [C.POINTER(State)]
profile = sim.load_profile()
cfg = sim.make_config(profile)
model = sim.mujoco.MjModel.from_xml_string(sim.model_xml(profile))

def observe(data, target=0., turn=0.):
    roll, pitch, yaw = sim.legacy.quat_to_euler_wxyz(data.qpos[3:7])
    values = Input(pitch=pitch, roll=roll, yaw=yaw, gyro=F3(*data.qvel[3:6]),
                   targetV=target, targetYaw=turn, dt=.003)
    for i, side in enumerate(sim.SIDES):
        for j, end in enumerate(('front', 'back')):
            node = data.joint(f'{side}_{end}_hinge')
            values.joint[2*i+j], values.jointRate[2*i+j] = node.qpos[0], node.qvel[0]
        node = data.joint(f'{side}_wheel_hinge')
        values.wheel[i], values.wheelRate[i] = node.qpos[0], node.qvel[0]
    return values

def apply(data, out):
    # 电机本地阻尼每个物理步更新，不等待主控下一次反馈和发送。
    data.ctrl[:] = out.torque
    for i, side in enumerate(sim.SIDES):
        for j, end in enumerate(('front', 'back')):
            data.ctrl[i*3+j] -= out.jointDamping * data.joint(f'{side}_{end}_hinge').qvel[0]

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
assert list(state.hold) == hold
lib.WheelLegBenchReset(C.byref(state))
assert lib.WheelLegBenchStep(C.byref(state), C.byref(moved), C.byref(out))
assert abs(state.hold[0] - moved.joint[0]) < 1e-7
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

results = []
for name, angle in [('stand', 0), ('tilt-plus', 3), ('tilt-minus', -3), ('drive-stop', 0), ('turn', 0), ('push', 0)]:
    data = sim.mujoco.MjData(model)
    angle = math.radians(angle)
    data.qpos[:] = sim.pose(model, cfg, .1, sim.np.array([-angle, 0, angle]))
    sim.mujoco.mj_forward(model, data)
    state, out = State(), Output()
    peak = 0
    stopped = False
    for step in range(10000):
        t = step * .001
        data.xfrc_applied[:] = 0
        if name == 'push' and 2 < t < 2.15:
            data.xfrc_applied[model.body('base').id, 0] = 2
        if step % 3 == 0:
            values = observe(data, .25 if name == 'drive-stop' and 1 < t < 4 else 0,
                             .4 if name == 'turn' and 1 < t < 4 else 0)
            peak = max(peak, abs(values.pitch), abs(values.roll))
            if not lib.WheelLegBenchStep(C.byref(state), C.byref(values), C.byref(out)):
                stopped = True
                break
            assert max(abs(out.torque[i]) for i in (2, 5)) <= .450001
            assert max(abs(out.torque[i]) for i in (0, 1, 3, 4)) <= 1.500001
        apply(data, out)
        sim.mujoco.mj_step(model, data)
    final = observe(data)
    result = dict(case=name, stopped=stopped, peak_angle_deg=math.degrees(peak),
                  final_pitch_deg=math.degrees(final.pitch), final_speed=float(data.qvel[0]),
                  passed=bool(not stopped and abs(final.pitch) < .08 and abs(data.qvel[0]) < .08))
    results.append(result)
    print(json.dumps(result), flush=True)
output = root / 'local/cache/wheelleg-calibration-20261024/板凳固件仿真检查.json'
output.write_text(json.dumps({'invalid_inputs': invalid+3, 'cases': results,
                  'passed': all(r['passed'] for r in results)}, indent=2), encoding='utf-8')
assert all(r['passed'] for r in results)

class AutoState(C.Structure):
    _fields_ = [('balance', State), ('start', F4), ('target', F4),
                ('elapsed', C.c_float), ('duration', C.c_float), ('settled', C.c_float),
                ('phase', C.c_uint32), ('reject', C.c_uint32), ('rejectValue', C.c_float)]
lib.WheelLegBenchAutoStep.argtypes = [C.POINTER(AutoState), C.POINTER(Input), C.POINTER(Output)]
lib.WheelLegBenchAutoStep.restype = C.c_int
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
    for step in range(10000):
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
               passed=bool(entered is not None and abs(final.pitch)<.08 and max(abs(x-.1) for x in out.length)<.006))
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

# 实物记录显示关节输出滞后，单看即时力矩模型会漏掉振荡。
from collections import deque
lag_results = []
for height in (.149, .085):
    for delay_ms, lag_ms, gain in ((12, 3, 1.), (20, 10, 1.), (20, 10, .65)):
        data = sim.mujoco.MjData(model)
        data.qpos[:] = sim.pose(model, cfg, height, sim.np.zeros(3))
        sim.mujoco.mj_forward(model, data)
        auto, out = AutoState(), Output()
        commands = deque([sim.np.zeros(6) for _ in range(delay_ms)])
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
            commands.append(sim.np.array(out.torque))
            delayed = commands.popleft()
            data.ctrl[:] = out.torque
            for i, side in enumerate(sim.SIDES):
                for j, end in enumerate(('front','back')):
                    k = i*3+j
                    rate = data.joint(f'{side}_{end}_hinge').qvel[0]
                    if abs(float(rate)) > peak_rate:
                        peak_rate, peak_time = abs(float(rate)), step*.001
                    if step >= 150: late_peak = max(late_peak, abs(float(rate)))
                    actual[k] += alpha*(gain*(delayed[k]-out.jointDamping*rate)-actual[k])
                    data.ctrl[k] = actual[k]
            sim.mujoco.mj_step(model, data)
        row = dict(start_length_m=height, command_delay_ms=delay_ms, current_lag_ms=lag_ms,
                   effort_gain=gain, balance_entered_s=entered, rejected=rejected,
                   peak_joint_rate_rad_s=peak_rate, peak_time_s=peak_time,
                   rate_after_support_ramp_rad_s=late_peak, final_lengths=list(out.length),
                   passed=rejected is None and entered is not None and peak_rate < 3. and late_peak < 1.5)
        lag_results.append(row)
        print(json.dumps(row),flush=True)
lag_path = root/'local/cache/wheelleg-calibration-20261024/板凳延迟仿真检查.json'
lag_path.write_text(json.dumps(lag_results,indent=2),encoding='utf8')
assert all(r['passed'] for r in lag_results)
