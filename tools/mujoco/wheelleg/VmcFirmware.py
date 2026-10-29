"""MuJoCo 的固件 VMC 适配；参数从车型源码读取，控制运算不在 Python 重写。"""
import ctypes as C
import hashlib
import math
import re
import subprocess
from pathlib import Path
import RunMeasuredWheelLeg as sim
from MotorProtocol import MotorBank

F2, F3, F4, F20 = (C.c_float * n for n in (2, 3, 4, 20))
ROOT = sim.ROOT
CONFIG = ROOT / 'Robotconfig/MINIWHEELEG-M/ConfigTuning.inc'


def config_source():
    text = CONFIG.read_text(encoding='utf-8')
    text = text[text.index('.WheelLegMit ='):text.index('// Shoot config')]
    text = re.sub(r'#include "([^"\n]+)"',
                  lambda m: (CONFIG.parent / m[1]).read_text(encoding='utf-8'), text)
    return re.sub(r'/\*.*?\*/|//[^\n]*', '', text, flags=re.S)


def field(text, name):
    found = re.search(r'\.' + re.escape(name) + r'\s*=\s*', text)
    if found is None:
        raise ValueError('车型缺少 VMC 参数：' + name)
    start = found.end()
    if text[start] != '{':
        return text[start:text.index(',', start)].strip()
    depth = 0
    for i in range(start, len(text)):
        depth += (text[i] == '{') - (text[i] == '}')
        if depth == 0:
            return text[start:i+1]
    raise ValueError('参数括号不完整：' + name)


def native():
    text = config_source()
    cfgmap = dict(minLegM='min_leg_length_m', maxLegM='max_leg_length_m',
                  wheelScale='lqr_wheel_torque_scale', hipScale='lqr_hip_torque_scale',
                  supportBiasN='support_bias_n', maxSupportN='max_support_force_n',
                  maxJointNm='max_joint_torque_nm', maxWheelNm='max_wheel_torque_nm',
                  trimEnable='pitch_trim_enable', trimGain='pitch_trim_gain',
                  trimMaxRad='pitch_trim_max_rad', trimRateRadps='pitch_trim_rate_radps',
                  trimVDeadbandMps='pitch_trim_v_deadband_mps', trimLpf='pitch_trim_lpf')
    task = (ROOT / 'shared/application/wheelleg/WheelLegMitTask.c').read_text(encoding='utf-8')
    defaults = [re.search(r'^#define '+key+r'\s+[^\r\n]+', task, re.M)[0]
                for key in ('WHEELLEG_MANUAL_FOOT_X_RANGE_M', 'WHEELLEG_TARGET_FOOT_X_SLEW_MPS',
                            'WHEELLEG_TARGET_LEG_SLEW_MPS')]
    lines = defaults + ['static const float measuredPoly[12][4] = ' + field(text, 'lqr_poly') + ';',
             'static void MeasuredVmcConfigure(MeasuredVmc *v) {',
             'v->config.lqrPoly = measuredPoly;', 'v->config.commonForward = 1u;']
    for dest, source in cfgmap.items():
        lines.append(f'v->config.{dest} = {field(text, source)};')
    for pid, dest in [('turn_pid', 'turn'), ('roll_pid', 'roll')]:
        values = field(text, pid)[1:-1].split(',')
        for suffix, index in [('Kp', 0), ('Kd', 2), ('Max', 3)]:
            lines.append(f'v->config.{dest}{suffix} = {values[index]};')
    for name, dest in [('leg_length_pid', 'legPid[0]'), ('leg_split_pid', 'splitPid')]:
        values = field(text, name)[1:-1].split(',')
        for member, value in zip(('kp', 'ki', 'kd', 'max_out', 'max_iout'), values):
            lines.append(f'v->control.{dest}.{member} = {value};')
    lines.append('v->control.legPid[1] = v->control.legPid[0];')
    for side in ('RIGHT', 'LEFT'):
        lines.append(f'v->config.pitchOffsetRad[WHEELLEG_SIDE_{side}] = '
                     + field(text, f'pitch_balance_offset_{side.lower()}_rad') + ';')
    for name in ('l1_m', 'l2_m', 'l3_m', 'l4_m', 'l5_m'):
        lines.append(f'v->geometry.{name} = {field(text, name)};')
    for dest, source in [('radius', 'wheel_radius_m'), ('observerLpf', 'observer_lpf'),
                         ('attitudeLimit', 'attitude_limit_rad')]:
        lines.append(f'v->{dest} = {field(text, source)};')
    lines.append('}')
    generated = '\n'.join(lines)
    paths = [sim.HERE / 'MeasuredVmcBridge.c', ROOT / 'shared/application/wheelleg/WheelLegVmc.c',
             ROOT / 'shared/application/wheelleg/WheelLegVmc.h', ROOT / 'shared/application/wheelleg/WheelLegCore.h',
             ROOT / 'shared/application/wheelleg/WheelLegMsg.h', ROOT / 'shared/application/robot/ControlCore.h',
             ROOT / 'shared/application/wheelleg/WheelLegBench.c', ROOT / 'shared/application/wheelleg/WheelLegBench.h',
             ROOT / 'shared/application/wheelleg/WheelLegBenchGains.inc']
    digest = hashlib.sha256(generated.encode() + b''.join(p.read_bytes() for p in paths)).hexdigest()[:16]
    folder = ROOT / 'local/build/host-tests/vmc-firmware' / digest
    folder.mkdir(parents=True, exist_ok=True)
    dll = folder / 'vmc.dll'
    if not dll.exists():
        (folder / 'MeasuredVmcConfig.inc').write_text(generated, encoding='utf-8')
        includes = [folder, ROOT / 'shared/application/wheelleg', ROOT / 'shared/application/robot',
                    ROOT / 'shared/components/support']
        zig = Path('C:/Users/28111/AppData/Local/Microsoft/WinGet/Links/zig.exe')
        subprocess.run([str(zig), 'cc', '-shared', '-O2', '-std=c99', '-Wall', '-Wextra', '-Werror',
                        *['-I'+str(p) for p in includes], str(paths[0]), str(paths[1]),
                        str(ROOT / 'shared/application/wheelleg/WheelLegBench.c'), '-o', str(dll)], check=True)
    lib = C.CDLL(str(dll))
    lib.MeasuredVmcCreate.restype = C.c_void_p
    lib.MeasuredVmcFree.argtypes = [C.c_void_p]
    lib.MeasuredVmcStep.argtypes = [C.c_void_p] + [C.POINTER(C.c_float)] * 6 + [C.c_float] * 5 + [C.c_uint32, C.POINTER(C.c_float)]
    lib.MeasuredVmcStep.restype = C.c_int
    return lib


class Controller:
    def __init__(self, model, profile, mode='vmc'):
        if not math.isclose(profile['control']['control_dt_s'], .003):
            raise ValueError('当前实车 VMC 控制周期为 3 ms')
        self.lib = native()
        text = config_source()
        self.max_speed = float(field(text, 'max_v_mps').rstrip('f'))
        self.max_yaw = float(field(text, 'max_yaw_rate_radps').rstrip('f'))
        self.handle = self.lib.MeasuredVmcCreate()
        if not self.handle:
            raise MemoryError('无法建立 VMC 状态')
        self.model, self.profile, self.mode = model, profile, mode
        self.variable_height = mode.endswith('-height')
        self.bench = False
        self.protocol = profile['assumed'].get('protocol_quantization', True)
        self.motors = MotorBank()
        self.model.actuator_ctrlrange[[2, 5]] = [-1., 1.]
        self.length = self.target_length = profile['control']['leg_length_m']
        self.leg_lengths, self.leg_angles = sim.np.zeros(2), sim.np.zeros(2)
        self.state, self.torque = sim.np.zeros(6), sim.np.zeros(6)
        self.speed = self.reference = self.observed_speed = self.trim = 0.
        self.yaw_rate = 0.
        self.saturated = self.samples = 0
        self.enabled = True
        self.reject = 0

    def __del__(self):
        if getattr(self, 'handle', None):
            self.lib.MeasuredVmcFree(self.handle)
            self.handle = None

    def step(self, data, target_speed=0., target_yaw=0., target_length=None):
        if not self.enabled:
            return
        dt = self.profile['control']['control_dt_s']
        target_speed = float(sim.np.clip(target_speed, -self.max_speed, self.max_speed))
        target_yaw = float(sim.np.clip(target_yaw, -self.max_yaw, self.max_yaw))
        if self.variable_height and target_length is not None:
            if not math.isfinite(target_length):
                raise ValueError('腿长必须为有限数值')
            self.target_length = float(sim.np.clip(target_length, .06, .12))
        q = [float(data.joint(f'{side}_{end}_hinge').qpos[0])
             for side in sim.SIDES for end in ('front', 'back')]
        dq = [float(data.joint(f'{side}_{end}_hinge').qvel[0])
              for side in sim.SIDES for end in ('front', 'back')]
        w = [float(data.joint(f'{side}_wheel_hinge').qvel[0]) for side in sim.SIDES]
        wp = [float(data.joint(f'{side}_wheel_hinge').qpos[0]) for side in sim.SIDES]
        if self.protocol:
            feedback = [self.motors.feedback_joint(motor, value, rate)
                        for motor, value, rate in zip((0, 1, 3, 4), q, dq)]
            q, dq = map(list, zip(*feedback))
            wheels = [self.motors.feedback_wheel(i*3+2, position, velocity)
                      for i, (position, velocity) in enumerate(zip(wp, w))]
            wp = [row[0] * (-1 if i == 0 else 1) for i, row in enumerate(wheels)]
            w = [row[1] for row in wheels]
        else:
            wp = [(position+12.5) % 25 - 12.5 for position in wp]
        angles = sim.legacy.quat_to_euler_wxyz(data.qpos[3:7])
        out = F20()
        accepted = self.lib.MeasuredVmcStep(self.handle, F4(*q), F4(*dq), F2(*wp), F2(*w), F3(*angles),
            F3(*data.qvel[3:6]), dt, target_speed, target_yaw, self.target_length, self.trim,
            round(float(data.time)*1000) & 0xffffffff, out)
        self.enabled = accepted == 1
        self.reject = 0 if self.enabled else accepted
        self.torque = sim.np.array(out[:6])
        self.leg_lengths, self.leg_angles = sim.np.array(out[6:8]), sim.np.array(out[8:10])
        self.reference, self.observed_speed = out[14:16]
        if self.enabled:
            self.length = out[16]
            self.trim = out[17]
            self.speed, self.yaw_rate = out[18:20]
        self.saturated += int(any(abs(out[i]) >= .44999 for i in (2, 5)))
        self.samples += 1
        if self.protocol and self.enabled:
            self.motors.set_commands([0]*6, [0]*6, [0]*6, [0]*6, self.torque)
        self.apply(data)

    def apply(self, data):
        if self.enabled and self.protocol:
            self.motors.apply(data)
        else:
            data.ctrl[:] = self.torque if self.enabled else 0.
