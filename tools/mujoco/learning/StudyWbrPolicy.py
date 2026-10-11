"""统一读取学习网络和可解释的线性修正；推理只用部署可获得的观测。"""
from pathlib import Path
import json

import numpy as np


def linear_action(observation, coefficients):
    frame = np.asarray(observation, dtype=np.float32)[-31:]
    c = np.asarray(coefficients, dtype=np.float32)
    if c.shape != (6,) or not np.isfinite(c).all():
        raise ValueError("线性修正系数必须是6个有限数")
    speed_error, position_error = frame[24]-frame[18], frame[19]
    gravity_x, pitch_rate = frame[3], frame[1]*4.
    wheel = np.dot(c[:4], [speed_error, position_error, gravity_x, pitch_rate])
    leg = np.dot(c[4:], [gravity_x, pitch_rate])
    return np.clip([0., leg, wheel], -1., 1.).astype(np.float32)


class StudyPolicy:
    def __init__(self, directory):
        directory = Path(directory)
        self.contract = json.loads((directory/"policy.json").read_text(encoding="utf-8"))
        self.kind = self.contract.get("kind", "ppo")
        if self.kind not in ("ppo", "linear"):
            raise ValueError("未知策略类型")
        if self.kind == "ppo":
            self.weights = dict(np.load(directory/"policy.npz", allow_pickle=False))

    def predict(self, observation):
        if self.kind == "linear":
            value = linear_action(observation, self.contract["coefficients"])
            if self.contract["env_config"]["action_mode"] == "full":
                value = np.tile(value, 2)
            return value
        value = np.asarray(observation, dtype=np.float32)
        count = len(self.weights)//2
        for i in range(count):
            value = value @ self.weights[f"w{i}"].T + self.weights[f"b{i}"]
            if i < count-1:
                value = np.tanh(value)
        return np.clip(value, -1., 1.)
