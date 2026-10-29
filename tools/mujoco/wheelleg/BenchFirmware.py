"""实车板凳 C 控制器的 MuJoCo 接口，网页与离线检查共用。"""
import ctypes as C
import hashlib
import math
import re
import subprocess
from pathlib import Path
import RunMeasuredWheelLeg as sim

F2, F3, F4, F6 = (C.c_float * n for n in (2, 3, 4, 6))
class Input(C.Structure):
    _fields_ = [('joint', F4), ('jointRate', F4), ('wheel', F2), ('wheelRate', F2),
                ('pitch', C.c_float), ('roll', C.c_float), ('yaw', C.c_float),
                ('gyro', F3), ('targetV', C.c_float), ('targetYaw', C.c_float), ('dt', C.c_float),
                ('pitchTrim', C.c_float), ('targetRoll', C.c_float)]
class State(C.Structure):
    _fields_ = [('hold', F4), ('feedforward', F4), ('reference', C.c_float),
                ('speed', C.c_float), ('yawRate', C.c_float), ('yawReference', C.c_float),
                ('acceleration', C.c_float),
                ('height', C.c_float), ('rollTarget', C.c_float), ('supportDelta', C.c_float),
                ('rollBias', C.c_float), ('parkIntegral', C.c_float), ('parkSpeed', C.c_float),
                ('ready', C.c_uint32), ('driving', C.c_uint32),
                ('reject', C.c_uint32), ('rejectValue', C.c_float),
                ('pitchRateX1', C.c_float), ('pitchRateX2', C.c_float),
                ('pitchRateY1', C.c_float), ('pitchRateY2', C.c_float)]
class Output(C.Structure):
    _fields_ = [('torque', F6), ('state', F6), ('length', F2), ('alpha', F2), ('jointDamping', C.c_float),
                ('jointTarget', F4), ('jointStiffness', C.c_float), ('wheelDamping', C.c_float),
                ('wheelTargetRate', F2)]
class Wheel(C.Structure):
    _fields_ = [('previous', C.c_float), ('position', C.c_float), ('rate', C.c_float),
               ('tick', C.c_uint32), ('ready', C.c_uint32),
               ('reject', C.c_uint32), ('rejectValue', C.c_float),
               ('overspeedTick', C.c_uint32), ('overspeed', C.c_uint32)]

root = sim.ROOT
folder = root / 'local/build/host-tests/bench-firmware'
folder.mkdir(parents=True, exist_ok=True)
zig = Path('C:/Users/28111/AppData/Local/Microsoft/WinGet/Links/zig.exe')
includes = [root/'shared/application/wheelleg', root/'shared/application/robot', root/'shared/components/support']
digest = hashlib.sha256(Path(__file__).read_bytes() + b''.join((root / path).read_bytes() for path in (
    'shared/application/wheelleg/WheelLegBench.c', 'shared/application/wheelleg/WheelLegBench.h',
    'shared/application/wheelleg/WheelLegBenchGains.inc', 'shared/application/wheelleg/WheelLegCore.h',
    'shared/application/wheelleg/WheelLegMitControlHelpers.inc', 'shared/application/wheelleg/WheelLegMitTask.c'))).hexdigest()[:16]
dll = folder / ('bench_' + digest + '.dll')
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
float BenchTrimStep(float v,float dt,int allowed,float targetV,float targetYaw) {
    static const WheelLegMitConfig cfg={.pitch_trim_enable=1,
        .pitch_trim_gain=WHEELLEG_BENCH_TRIM_GAIN,.pitch_trim_max_rad=WHEELLEG_BENCH_TRIM_LIMIT,
        .pitch_trim_rate_radps=WHEELLEG_BENCH_TRIM_RATE,.pitch_trim_v_deadband_mps=WHEELLEG_BENCH_TRIM_DEADBAND,
        .pitch_trim_lpf=WHEELLEG_BENCH_TRIM_LPF};
    s_wheelleg.v_mps=v;
    if(allowed) WheelLegPitchTrimUpdateConfigured(&cfg,dt,0,0,targetV,targetYaw);
    return s_wheelleg.pitch_auto_offset_rad;
}
''',encoding='utf-8')
if not dll.exists():
    subprocess.run([str(zig), 'cc', '-shared', '-O2', '-std=c99', '-Wall', '-Wextra', '-Werror',
                    *['-I'+str(p) for p in includes], str(root/'shared/application/wheelleg/WheelLegBench.c'),
                    str(trim_test), '-o', str(dll)], check=True)
lib = C.CDLL(str(dll))
lib.BenchTrimStep.argtypes = [C.c_float, C.c_float, C.c_int, C.c_float, C.c_float]
lib.BenchTrimStep.restype = C.c_float
lib.WheelLegBenchStep.argtypes = [C.POINTER(State), C.POINTER(Input), C.POINTER(Output)]
lib.WheelLegBenchStep.restype = C.c_int
lib.WheelLegBenchReset.argtypes = [C.POINTER(State)]
lib.WheelLegBenchSetHeight.argtypes = [C.POINTER(State), C.c_float, C.c_float]
lib.WheelLegBenchSetHeight.restype = C.c_int
lib.WheelLegBenchGetHeight.argtypes = [C.POINTER(State)]
lib.WheelLegBenchGetHeight.restype = C.c_float
lib.WheelLegBenchHeightAxis.argtypes = [C.POINTER(State), C.c_float, C.c_float]
lib.WheelLegBenchHeightAxis.restype = C.c_int
lib.WheelLegBenchWheelObserve.argtypes = [C.POINTER(Wheel), C.c_float, C.c_float, C.c_uint32]
lib.WheelLegBenchWheelObserve.restype = C.c_int
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
        data.ctrl[i*3+2] += out.wheelDamping * (out.wheelTargetRate[i] - data.joint(f'{side}_wheel_hinge').qvel[0])
        for j, end in enumerate(('front', 'back')):
            node = data.joint(f'{side}_{end}_hinge')
            data.ctrl[i*3+j] += out.jointStiffness * (out.jointTarget[i*2+j] - node.qpos[0]) - out.jointDamping * node.qvel[0]


class Controller:
    """调用固件控制和轮角观测；电机位置、速度环在每个物理步执行。"""

    def __init__(self, model, profile, mode='bench'):
        if not math.isclose(profile['control']['control_dt_s'], .003):
            raise ValueError('当前实车板凳控制周期固定为 3 ms')
        self.model, self.profile, self.mode = model, profile, mode
        self.model.actuator_ctrlrange[[2, 5]] = [-1., 1.]
        self.core = sim.Core(profile, sim.ROOT / 'local/cache/measured-wheelleg-native')
        self.variable_height = mode.endswith('-height')
        self.native, self.output = State(), Output()
        self.wheels = [Wheel(), Wheel()]
        self.origins = None
        self.enabled = True
        self.length = self.target_length = .09
        self.leg_lengths = sim.np.zeros(2)
        self.leg_angles = sim.np.zeros(2)
        self.state = sim.np.zeros(6)
        self.observed_speed = self.speed = self.reference = self.trim = 0.
        self.saturated = self.samples = 0
        self.pitch_bias = 0.
        lib.BenchTrimReset()

    def step(self, data, target_speed=0., target_yaw=0., target_length=None, height_axis=None, roll_axis=0.):
        if not self.enabled:
            return
        values = observe(data, target_speed, target_yaw)
        values.pitch += self.pitch_bias
        values.pitchTrim = self.trim
        values.targetRoll = float(roll_axis) * math.radians(10)
        if self.origins is None:
            self.origins = list(values.wheel)
        tick = round(float(data.time)*1000) & 0xffffffff
        for i, wheel in enumerate(self.wheels):
            raw = (values.wheel[i] - self.origins[i] + 24.99) % 25 - 12.5
            raw = round((raw+12.5)*65535/25)*25/65535-12.5
            if not lib.WheelLegBenchWheelObserve(C.byref(wheel), raw, values.wheelRate[i], tick):
                self.enabled = False
                return
            values.wheel[i], values.wheelRate[i] = wheel.position, wheel.rate
        height_moving = False
        if self.variable_height and height_axis is not None and self.native.ready:
            if not lib.WheelLegBenchHeightAxis(C.byref(self.native), height_axis, values.dt):
                self.enabled = False
                return
            self.target_length = float(lib.WheelLegBenchGetHeight(C.byref(self.native)))
            height_moving = abs(height_axis) > .001
        elif self.variable_height and target_length is not None:
            if not math.isfinite(target_length):
                raise ValueError('腿长必须为有限数值')
            self.target_length = float(sim.np.clip(target_length, .06, .12))
            height_moving = abs(self.target_length-self.length) > .00001
            if self.native.ready and not lib.WheelLegBenchSetHeight(
                    C.byref(self.native), self.target_length, values.dt):
                self.enabled = False
                return
        self.enabled = bool(lib.WheelLegBenchStep(C.byref(self.native), C.byref(values), C.byref(self.output)))
        if not self.enabled:
            return
        self.state = sim.np.array(self.output.state)
        self.leg_lengths, self.leg_angles = sim.np.array(self.output.length), sim.np.array(self.output.alpha)
        self.observed_speed = self.output.state[3] + self.native.speed + self.native.parkSpeed
        self.speed, self.reference = self.native.speed, self.native.reference
        self.trim = lib.BenchTrimStep(self.observed_speed, values.dt,
            not height_moving and abs(roll_axis)<.001 and abs(values.gyro[0])<.1 and
            abs(values.pitch)<.12 and abs(values.gyro[1])<.5 and abs(self.observed_speed)<.25,
            self.native.speed, self.native.yawRate)
        self.length = float(lib.WheelLegBenchGetHeight(C.byref(self.native)))
        self.saturated += int(any(abs(self.output.torque[i]) >= .44999 for i in (2, 5)))
        self.samples += 1

    def apply(self, data):
        if self.enabled:
            apply(data, self.output)
        else:
            data.ctrl[:] = 0
