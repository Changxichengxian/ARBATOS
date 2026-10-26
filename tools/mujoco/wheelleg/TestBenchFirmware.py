"""在三维接触模型中运行实际板凳 C 控制代码，同时检查输出拒绝条件。"""
import ctypes as C
import json
import math
import re
import subprocess
import sys
from pathlib import Path
import RunMeasuredWheelLeg as sim

F2, F3, F4, F6 = (C.c_float * n for n in (2, 3, 4, 6))
class Input(C.Structure):
    _fields_ = [('joint', F4), ('jointRate', F4), ('wheel', F2), ('wheelRate', F2),
                ('pitch', C.c_float), ('roll', C.c_float), ('yaw', C.c_float),
                ('gyro', F3), ('targetV', C.c_float), ('targetYaw', C.c_float), ('dt', C.c_float),
                ('pitchTrim', C.c_float)]
class State(C.Structure):
    _fields_ = [('hold', F4), ('feedforward', F4), ('reference', C.c_float),
                ('speed', C.c_float), ('yawRate', C.c_float), ('yawReference', C.c_float),
                ('ready', C.c_uint32), ('driving', C.c_uint32)]
class Output(C.Structure):
    _fields_ = [('torque', F6), ('state', F6), ('length', F2), ('alpha', F2), ('jointDamping', C.c_float),
                ('jointTarget', F4), ('jointStiffness', C.c_float), ('wheelDamping', C.c_float)]
class Wheel(C.Structure):
    _fields_ = [('previous', C.c_float), ('position', C.c_float), ('rate', C.c_float),
               ('tick', C.c_uint32), ('ready', C.c_uint32)]

root = sim.ROOT
folder = root / 'local/build/host-tests/bench-firmware'
folder.mkdir(parents=True, exist_ok=True)
zig = Path('C:/Users/28111/AppData/Local/Microsoft/WinGet/Links/zig.exe')
includes = [root/'shared/application/wheelleg', root/'shared/application/robot', root/'shared/components/support']
dll = folder / 'bench.dll'
# 直接编译现有慢调函数的源码，避免仿真另写一套积分逻辑。
helpers = (root/'shared/application/wheelleg/WheelLegMitControlHelpers.inc').read_text(encoding='utf-8')
trim_code = helpers[helpers.index('static uint8_t WheelLegPitchTrimIdle('):helpers.index('static void WheelLegPitchTrimUpdate(fp32')]
task_code = (root/'shared/application/wheelleg/WheelLegMitTask.c').read_text(encoding='utf-8')
trim_defaults = '\n'.join('#define '+a+' '+b for a,b in re.findall(r'^#define (WHEELLEG_PITCH_TRIM_\w+) ([^\r\n]+)',task_code,re.M))
trim_test = folder/'trim.c'
trim_test.write_text('''#include <math.h>
#include <stdint.h>
#include <string.h>
#include "WheelLegBench.h"
typedef float fp32;
typedef struct { unsigned pitch_trim_enable, rc_deadband;
    float pitch_trim_max_rad, pitch_trim_rate_radps, pitch_trim_v_deadband_mps,
          pitch_trim_lpf, pitch_trim_gain; } WheelLegMitConfig;
static struct { WheelLegMitConfig WheelLegMit; } g_config;
static struct { float pitch_auto_offset_rad, pitch_trim_v_lpf_mps, v_mps; } s_wheelleg;
#define WHEELLEG_LQR_MOTION_EPS .001f
static float WheelLegAbs(float v) { return fabsf(v); }
static float WheelLegClamp(float v,float lo,float hi) { return fminf(fmaxf(v,lo),hi); }
static int WheelLegAxisInDeadband(int16_t a,unsigned d) { return fabsf((float)a)<=d; }
static float WheelLegCfgPositiveOrDefault(float v,float d,float max) { return WheelLegClamp(v<=0?d:v,0,max); }
''' + trim_defaults + '\n' + trim_code + '''
void BenchTrimReset(void) { memset(&s_wheelleg,0,sizeof(s_wheelleg)); }
float BenchTrimStep(float v,float dt,int allowed) {
    static const WheelLegMitConfig cfg={.pitch_trim_enable=1,
        .pitch_trim_gain=WHEELLEG_BENCH_TRIM_GAIN,.pitch_trim_max_rad=WHEELLEG_BENCH_TRIM_LIMIT,
        .pitch_trim_rate_radps=WHEELLEG_BENCH_TRIM_RATE,.pitch_trim_v_deadband_mps=WHEELLEG_BENCH_TRIM_DEADBAND,
        .pitch_trim_lpf=WHEELLEG_BENCH_TRIM_LPF};
    s_wheelleg.v_mps=v;
    if(allowed) WheelLegPitchTrimUpdateConfigured(&cfg,dt,0,0,0,0);
    return s_wheelleg.pitch_auto_offset_rad;
}
''',encoding='utf-8')
subprocess.run([str(zig), 'cc', '-shared', '-O2', '-std=c99', '-Wall', '-Wextra', '-Werror',
                *['-I'+str(p) for p in includes], str(root/'shared/application/wheelleg/WheelLegBench.c'),
                str(trim_test), '-o', str(dll)], check=True)
lib = C.CDLL(str(dll))
lib.BenchTrimStep.argtypes = [C.c_float, C.c_float, C.c_int]
lib.BenchTrimStep.restype = C.c_float
lib.WheelLegBenchStep.argtypes = [C.POINTER(State), C.POINTER(Input), C.POINTER(Output)]
lib.WheelLegBenchStep.restype = C.c_int
lib.WheelLegBenchReset.argtypes = [C.POINTER(State)]
lib.WheelLegBenchWheelObserve.argtypes = [C.POINTER(Wheel), C.c_float, C.c_float, C.c_uint32]
lib.WheelLegBenchWheelObserve.restype = C.c_int
# 实物曾三次跨过 +/-12.5 边界；两个方向都应连续，重复帧不能再次积分。
for direction in (-1, 1):
    wheel = Wheel()
    assert lib.WheelLegBenchWheelObserve(C.byref(wheel), direction*12.49, 0, 100)
    assert lib.WheelLegBenchWheelObserve(C.byref(wheel), -direction*12.49, direction*13, 110)
    assert abs(wheel.position - direction*.02) < 2e-6
    before = bytes(wheel)
    assert lib.WheelLegBenchWheelObserve(C.byref(wheel), -direction*12.49, 0, 110)
    assert bytes(wheel) == before
    assert not lib.WheelLegBenchWheelObserve(C.byref(wheel), -direction*12.49, 36, 112)
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
        data.ctrl[i*3+2] -= out.wheelDamping * data.joint(f'{side}_wheel_hinge').qvel[0]
        for j, end in enumerate(('front', 'back')):
            node = data.joint(f'{side}_{end}_hinge')
            data.ctrl[i*3+j] += out.jointStiffness * (out.jointTarget[i*2+j] - node.qpos[0]) - out.jointDamping * node.qvel[0]

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
quick_wheel = '--quick-wheel' in sys.argv
cases = [('stand', 0), ('tilt-plus', 3), ('tilt-minus', -3)]
if quick_wheel:
    cases += [('pitch-bias-plus', 0), ('pitch-bias-minus', 0)]
if not quick_wheel:
    cases += [('drive-stop', 0), ('turn', 0), ('push', 0)]
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
    stopped = False
    for step in range(30000 if pitch_bias else 10000):
        t = step * .001
        data.xfrc_applied[:] = 0
        if name == 'push' and 2 < t < 2.15:
            data.xfrc_applied[model.body('base').id, 0] = 2
        if step % 3 == 0:
            values = observe(data, .25 if name == 'drive-stop' and 1 < t < 4 else 0,
                             .4 if name == 'turn' and 1 < t < 4 else 0)
            if quick_wheel:
                values.pitch += pitch_bias
                values.pitchTrim = trim
                if wheel_origins is None:
                    wheel_origins = list(values.wheel)
                for i, wheel in enumerate(wheel_observers):
                    raw = (values.wheel[i] - wheel_origins[i] + 12.49 + 12.5) % 25 - 12.5
                    raw = round((raw + 12.5) * 65535 / 25) * 25 / 65535 - 12.5
                    # 模拟本次原始轮速来回约 +/-13 rad/s，控制仍只使用连续角度估算值。
                    report = values.wheelRate[i] + (13 if step % 6 == 0 else -13) if wheel.ready else 0
                    assert lib.WheelLegBenchWheelObserve(C.byref(wheel), raw, report, step)
                    values.wheel[i], values.wheelRate[i] = wheel.position, wheel.rate
            peak = max(peak, abs(values.pitch), abs(values.roll))
            if not lib.WheelLegBenchStep(C.byref(state), C.byref(values), C.byref(out)):
                stopped = True
                break
            if quick_wheel:
                trim = lib.BenchTrimStep(out.state[3]+state.speed, values.dt,
                    abs(values.pitch)<.12 and abs(values.gyro[1])<.5 and abs(out.state[3]+state.speed)<.25)
                assert abs(trim) <= .060001
            assert max(abs(out.torque[i]) for i in (2, 5)) <= .450001
            assert max(abs(out.torque[i]) for i in (0, 1, 3, 4)) <= 1.500001
        apply(data, out)
        sim.mujoco.mj_step(model, data)
        if pitch_bias and step >= 28000:
            late_speed.append(float(data.qvel[0]))
    final = observe(data)
    mean_speed = sum(late_speed)/len(late_speed) if late_speed else float(data.qvel[0])
    result = dict(case=name, stopped=stopped, peak_angle_deg=math.degrees(peak),
                  final_pitch_deg=math.degrees(final.pitch), final_speed=float(data.qvel[0]),
                  mean_late_speed=mean_speed, trim_deg=math.degrees(trim),
                  passed=bool(not stopped and abs(final.pitch) < .08 and abs(data.qvel[0]) < .08
                              and (not pitch_bias or abs(mean_speed)<.01)))
    results.append(result)
    print(json.dumps(result), flush=True)
output = root / 'local/cache/wheelleg-resume-20261026' / ('90mm轮端阻尼检查.json' if quick_wheel else '板凳固件仿真检查.json')
output.parent.mkdir(parents=True, exist_ok=True)
output.write_text(json.dumps({'invalid_inputs': invalid+3, 'cases': results,
                  'passed': all(r['passed'] for r in results)}, indent=2), encoding='utf-8')
assert all(r['passed'] for r in results)
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
    assert min(horizontal, vertical) < raw < max(horizontal, vertical), (motor_id, raw)
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
               passed=bool(entered is not None and abs(final.pitch)<.08 and max(abs(x-.090) for x in out.length)<.006))
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
    assert ready_at is not None and ready_at < 10 and entered is not None
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
