"""回放 0042 暴露的短接收间隔和 60 mm 承重到位条件。"""
import ctypes as C
import json
import math

from BenchFirmware import F4, Input, State, Output, Wheel, lib


class Auto(C.Structure):
    _fields_ = [('balance', State), ('start', F4), ('target', F4),
                ('elapsed', C.c_float), ('duration', C.c_float), ('settled', C.c_float),
                ('phase', C.c_uint32), ('reject', C.c_uint32), ('rejectValue', C.c_float)]


lib.WheelLegBenchPoseStep.argtypes = [C.POINTER(Auto), C.POINTER(Input), C.POINTER(Output), C.c_uint32]
lib.WheelLegBenchPoseStep.restype = C.c_int

# 0042 中相邻两次真实反馈：1 ms 接收时间差不能直接当作物理采样间隔。
for previous, raw, tick in ((7.234493255615234, 7.271496295928955, 191244),
                            (7.481688976287842, 7.521744251251221, 212604)):
    wheel = Wheel()
    assert lib.WheelLegBenchWheelObserve(C.byref(wheel), previous, 0, tick)
    assert lib.WheelLegBenchWheelObserve(C.byref(wheel), raw, 13.75, tick+1)
    assert wheel.tick == tick and wheel.position == 0
    assert lib.WheelLegBenchWheelObserve(C.byref(wheel), raw+.04, 13.75, tick+5)
    assert abs(wheel.position-(raw+.04-previous)) < 2e-6

for direction in (-1, 1):
    wheel = Wheel()
    assert lib.WheelLegBenchWheelObserve(C.byref(wheel), 0, 0, 100)
    for i in range(1, 100):
        # 真实每 3 ms 采样，接收任务形成 1/5 ms 交替间隔，平均仍为 3 ms。
        stamp = 100+3*i-(2 if i % 2 else 0)
        raw = (direction*25*.003*i+12.5) % 25-12.5
        assert lib.WheelLegBenchWheelObserve(C.byref(wheel), raw, direction*25, stamp)
    assert abs(wheel.rate-direction*25) < .01
    assert not lib.WheelLegBenchWheelObserve(C.byref(wheel), raw, direction*51, stamp+1)
    wheel = Wheel()
    assert lib.WheelLegBenchWheelObserve(C.byref(wheel), 0, 0, 100)
    assert not lib.WheelLegBenchWheelObserve(C.byref(wheel), direction*.2, 0, 101)
    wheel = Wheel()
    assert lib.WheelLegBenchWheelObserve(C.byref(wheel), 0, 0, 100)
    assert not lib.WheelLegBenchWheelObserve(C.byref(wheel), direction*.1081, 0, 103)

# 0044 的单帧上报尖峰不再立刻撤掉平衡；持续超速仍须在 12 ms 确认。
for direction in (-1, 1):
    wheel = Wheel()
    assert lib.WheelLegBenchWheelObserve(C.byref(wheel), 0, 0, 100)
    assert lib.WheelLegBenchWheelObserve(C.byref(wheel), .06*direction, 36*direction, 103)
    assert lib.WheelLegBenchWheelObserve(C.byref(wheel), .12*direction, 24*direction, 106)
    for tick in (109,112,115,118):
        assert lib.WheelLegBenchWheelObserve(C.byref(wheel), .02*(tick-100)*direction, 36*direction, tick)
    assert not lib.WheelLegBenchWheelObserve(C.byref(wheel), .42*direction, 36*direction, 121)
    assert wheel.reject == 2

values = Input(joint=F4(1.52056134, -1.57892704, 1.55718291, -1.58045328),
               jointRate=F4(-.02197838, -.00732613, .00732613, .00732613),
               pitch=.05499186, roll=.02421227, dt=.003)
auto, out = Auto(), Output()
for i in range(2200):
    assert lib.WheelLegBenchPoseStep(C.byref(auto), C.byref(values), C.byref(out), 0), auto.reject
    assert out.wheelDamping == 0 and out.torque[2] == 0 and out.torque[5] == 0
    if auto.phase == 4:
        break
assert auto.phase == 4
ready_time = (i+1)*.003
# 保留堵转保护；角差较大不能因为速度为零就被当作到位。
values.joint[0] -= .15
auto, out = Auto(), Output()
for i in range(2500):
    accepted = lib.WheelLegBenchPoseStep(C.byref(auto), C.byref(values), C.byref(out), 0)
    assert auto.phase != 4 and out.wheelDamping == 0
    if not accepted:
        break
assert not accepted

# 在实际 C 控制器中核对滤波频响：低频平衡不衰减，36 Hz 保留必要阻尼。
filter_gains = {}
for hz in (1., 36.):
    state, out = State(), Output()
    values = Input(joint=F4(1.52056134, -1.57892704, 1.55718291, -1.58045328), dt=.003)
    incoming, outgoing = [], []
    for i in range(4000):
        values.gyro[1] = .1*math.sin(2*math.pi*hz*i*.003)
        assert lib.WheelLegBenchStep(C.byref(state), C.byref(values), C.byref(out)), state.reject
        if i > 2000:
            incoming.append(values.gyro[1]**2)
            outgoing.append(out.state[5]**2)
    gain = math.sqrt(sum(outgoing)/sum(incoming))
    assert (.99 < gain < 1.01) if hz == 1. else (.74 < gain < .76)
    filter_gains[str(hz)] = gain
print(json.dumps({'short_interval_cases_passed': True, 'overspeed_and_jump_rejected': True,
                  'loaded_60mm_ready_s': ready_time, 'blocked_pose_rejected': True,
                  'pitch_filter_amplitude_gains': filter_gains}))
