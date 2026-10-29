"""MIT 协议字段的确定性限幅和量化；不模拟电机内部采样与通信延迟。

主控发送字段按 CanMitMotorDriver.c 的单精度运算和加 0.5 取整。
电机反馈的编码过程未在本仓库公开，这里假定同样采用就近量化；
主控对反馈整数的解码则与驱动一致。不要把该假定当作实测电机行为。
"""

import ctypes
import math


def _f32(value):
    return ctypes.c_float(value).value


def _limits(kind):
    if kind == 'joint':
        return 30.0, 10.0
    if kind == 'wheel':
        return 280.0, 1.0
    raise ValueError("kind 必须为 'joint' 或 'wheel'")


def _field(value, lower, upper, bits):
    """返回编码后经主控公式解码的值。"""
    value = _f32(value)
    if not math.isfinite(value):
        raise ValueError('协议字段必须为有限数值')
    lower, upper = _f32(lower), _f32(upper)
    value = min(max(value, lower), upper)
    span = _f32(upper - lower)
    maximum = (1 << bits) - 1
    scaled = _f32(_f32(_f32(value - lower) * maximum) / span)
    code = int(_f32(scaled + 0.5))
    return _f32(_f32(_f32(code * span) / maximum) + lower)


def feedback(pos, vel, kind):
    """返回主控解码后的 (位置, 速度)，反馈编码使用上面的假定。"""
    velocity, _ = _limits(kind)
    return (_field(pos, -12.5, 12.5, 16),
            _field(vel, -velocity, velocity, 12))


def command(p, v, kp, kd, t, kind):
    """返回经过主控编码并按相同量程解码的五个命令字段。"""
    velocity, torque = _limits(kind)
    return (_field(p, -12.5, 12.5, 16),
            _field(v, -velocity, velocity, 12),
            _field(kp, 0.0, 500.0, 12),
            _field(kd, 0.0, 5.0, 12),
            _field(t, -torque, torque, 12))


class MotorBank:
    """六个电机：右前、右后、右轮、左前、左后、左轮。"""

    # 与 WheelLegBenchJointDir 和实车轮电机方向一致。
    _sign = (1, 1, -1, -1, -1, 1)
    _zero = (-1.197642481, .496490425, 0.0, .537308309, -1.273937591, 0.0)
    _names = ('right_front_hinge', 'right_back_hinge', 'right_wheel_hinge',
              'left_front_hinge', 'left_back_hinge', 'left_wheel_hinge')

    def __init__(self):
        self.commands = [(0.0, 0.0, 0.0, 0.0, 0.0)] * 6

    def set_commands(self, p, v, kp, kd, t):
        """传入五组各六个值；输入输出均为模型关节方向。"""
        fields = (p, v, kp, kd, t)
        if any(len(values) != 6 for values in fields):
            raise ValueError('每组命令必须包含六个电机')
        result = []
        for i in range(6):
            sign, zero = self._sign[i], self._zero[i]
            kind = 'wheel' if i % 3 == 2 else 'joint'
            raw = command(zero + sign * p[i], sign * v[i], kp[i], kd[i],
                          sign * t[i], kind)
            result.append((sign * (raw[0] - zero), sign * raw[1], raw[2], raw[3], sign * raw[4]))
        self.commands = result

    def apply(self, data):
        """每个物理步用真实关节状态计算本地位置和速度环的力矩。"""
        for i, name in enumerate(self._names):
            joint = data.joint(name)
            p, v, kp, kd, t = self.commands[i]
            torque = t + kp * (p - float(joint.qpos[0])) + kd * (v - float(joint.qvel[0]))
            limit = 1.0 if i % 3 == 2 else 10.0
            data.ctrl[i] = min(max(torque, -limit), limit)

    def feedback_joint(self, i, q, dq):
        """i 是六电机索引；返回模型方向的量化位置和速度。"""
        if i not in (0, 1, 3, 4):
            raise ValueError('关节索引必须为 0、1、3 或 4')
        sign, zero = self._sign[i], self._zero[i]
        p, v = feedback(zero + sign * q, sign * dq, 'joint')
        return sign * (p - zero), sign * v

    def feedback_wheel(self, i, q, dq):
        """i 是六电机索引；返回电机原始轮角和模型方向轮速。

        上游轮角观测器应先将原始轮角乘对应方向，再进行展开。
        """
        if i not in (2, 5):
            raise ValueError('轮索引必须为 2 或 5')
        sign = self._sign[i]
        raw = (sign * q + 12.5) % 25.0 - 12.5
        p, v = feedback(raw, sign * dq, 'wheel')
        return p, sign * v


def _check():
    for kind, velocity, torque in (('joint', 30.0, 10.0), ('wheel', 280.0, 1.0)):
        assert feedback(-100, -1000, kind) == (-12.5, -velocity)
        assert feedback(100, 1000, kind) == (12.5, velocity)
        assert command(-100, -1000, -1, -1, -100, kind) == (-12.5, -velocity, 0.0, 0.0, -torque)
        assert command(100, 1000, 1000, 10, 100, kind) == (12.5, velocity, 500.0, 5.0, torque)
        p, v, kp, kd, t = command(0, 0, 0, 0, 0, kind)
        assert p > 0 and v > 0 and t > 0 and kp == kd == 0
        assert math.isclose(p, 12.5 / 65535, abs_tol=2e-6)
        assert math.isclose(v, velocity / 4095, abs_tol=2e-5)
        assert math.isclose(t, torque / 4095, abs_tol=2e-6)
    bank = MotorBank()
    bank.set_commands([0] * 6, [0] * 6, [0] * 6, [0] * 6, [0] * 6)
    assert bank.commands[2][1] < 0 < bank.commands[5][1]
    for i in (0, 1, 3, 4):
        p, v = bank.feedback_joint(i, 0, 0)
        assert abs(p) < .0004 and abs(v) < .008
    assert bank.feedback_wheel(2, 0, 0)[1] < 0 < bank.feedback_wheel(5, 0, 0)[1]


if __name__ == '__main__':
    _check()
    print('MIT 协议量化检查通过')
