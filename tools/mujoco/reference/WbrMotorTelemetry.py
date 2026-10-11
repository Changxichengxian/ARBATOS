"""仿真电机遥测：复用仓库功率系数，气弹簧等被动力不计入电机扭矩。"""
from __future__ import annotations

import hashlib
import math
from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[3]
POWER_SOURCE = ROOT / "shared/components/controller/ChassisPowerLimiter.c"
MOTOR_SOURCE = ROOT / "shared/application/motors/MotorModelDb.c"
TORQUE_SOURCE = ("https://cdn-hz.robomaster.com/robomasters/public/document/"
                "RoboMaster%20M3508%20P19%20Brushless%20DC%20Gear%20Motor.pdf")
MOTOR_NAMES = ("ljoint1_actuator", "ljoint4_actuator", "lwheel_actuator",
               "rjoint1_actuator", "rjoint4_actuator", "rwheel_actuator")


class WbrMotorTelemetry:
    def __init__(self, model, torquePerAmpNm: float = 0.3, reduction: float | None = None):
        text = POWER_SOURCE.read_text(encoding="utf-8")
        section = text.split("case MOTOR_MODEL_3508:", 1)[1].split("break;", 1)[0]
        self.coefficients = [float(re.search(rf"k{i}\s*=\s*([-0-9.]+)f", section)[1]) for i in range(6)]
        motors = MOTOR_SOURCE.read_text(encoding="utf-8").split("[MOTOR_MODEL_3508] =", 1)[1]
        motors = motors.split("[MOTOR_MODEL_3510]", 1)[0]
        sourceRatio = float(re.search(r"reduction_ratio\s*=\s*([0-9.]+)f", motors)[1])
        self.reduction = sourceRatio if reduction is None else float(reduction)
        self.torquePerAmpNm = float(torquePerAmpNm)
        if not all(math.isfinite(x) and x > 0 for x in (self.reduction, self.torquePerAmpNm)):
            raise ValueError("电机减速比和输出轴扭矩常数必须为正有限数")
        self.model = model
        self.axes = []
        for name in MOTOR_NAMES:
            actuator = model.actuator(name).id
            joint = int(model.actuator_trnid[actuator, 0])
            self.axes.append((name, actuator, int(model.jnt_dofadr[joint]),
                              float(model.actuator_gear[actuator, 0])))

    def metadata(self) -> dict:
        return dict(kind="simulation_estimate", model="M3508", coefficients=self.coefficients,
                    reduction=self.reduction, outputTorquePerAmpNm=self.torquePerAmpNm,
                    torqueConstantSource=TORQUE_SOURCE, torqueConstantPage=7,
                    coefficientSource=str(POWER_SOURCE.relative_to(ROOT)),
                    coefficientSourceSha256=hashlib.sha256(POWER_SOURCE.read_bytes()).hexdigest(),
                    motorSourceSha256=hashlib.sha256(MOTOR_SOURCE.read_bytes()).hexdigest(),
                    electricalMeaning="拟合电功率；负值仅代表制动模型结果，不能作为电池回收功率",
                    currentMeaning="由减速输出轴电机扭矩除以官方0.3 Nm/A得到转矩电流，非电池电流",
                    speedMeaning="MuJoCo输出轴rad/s乘内置减速比，得到转子rad/s",
                    mechanicalMeaning="同一关节坐标的电机扭矩×角速度；负值表示吸收机械能",
                    limits="未模拟电调/电池/减速器效率和温升；系数沿用仓库，尚无该仿真硬件标定")

    def snapshot(self, data) -> dict:
        rows = []
        for name, actuator, dof, gear in self.axes:
            # actuator_force 已经过执行器限幅；乘 transmission gear 才是关节输出力矩。
            torque = float(data.actuator_force[actuator]) * gear
            speed = float(data.qvel[dof])
            row = dict(name=name, torque_nm=torque, speed_radps=speed,
                       mechanical_power_w=torque * speed)
            if "wheel" in name:
                current = torque / self.torquePerAmpNm
                omega = speed * self.reduction
                k0, k1, k2, k3, k4, k5 = self.coefficients
                power = k0 + k1*current + k2*abs(omega) + k3*current*omega + k4*current**2 + k5*omega**2
                row.update(current_estimate_a=current, rotor_speed_radps=omega,
                           electrical_power_estimate_w=power,
                           braking=torque*speed < 0)
            rows.append(row)
        return dict(wheels=[rows[2], rows[5]], hips=[rows[i] for i in (0, 1, 3, 4)],
                    electrical_power_estimate_w=rows[2]["electrical_power_estimate_w"] +
                                                rows[5]["electrical_power_estimate_w"],
                    wheel_mechanical_power_w=rows[2]["mechanical_power_w"] + rows[5]["mechanical_power_w"])
