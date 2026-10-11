"""训练专用包装：固定训练库、归一化密集评分，原93维观测原样返回。"""
import hashlib
import json
import math
from pathlib import Path

import gymnasium as gym
import numpy as np

from BuildWbrStudyCases import cases_sha256
from EvaluateWbrStudy import summarize_trace, motion_quality_metrics
from WbrAlignedReward import baseline_spec, step_costs, weighted_cost, guard_cost
from WbrLearningEnv import WbrLearningEnv, verify_snapshot, experiment_identity


class WbrAlignedEnv(gym.Wrapper):
    def __init__(self, bank_path, bank_sha256, weights):
        bank_path = Path(bank_path)
        raw = bank_path.read_bytes()
        if hashlib.sha256(raw).hexdigest() != bank_sha256:
            raise RuntimeError("训练奖励库内容变化")
        bank = json.loads(raw)
        manifest = verify_snapshot()
        if (bank["experiment_identity"] != experiment_identity(manifest) or
                bank["library_sha256"] != manifest["library_sha256"]):
            raise RuntimeError("训练库与当前冻结物理不匹配")
        cases = [row["scenario"] for row in bank["rows"]]
        if bank["schema"] != 1 or cases_sha256(cases) != bank["cases_sha256"]:
            raise RuntimeError("训练库场景哈希错误")
        if not cases or len({row["id"] for row in cases}) != len(cases):
            raise RuntimeError("训练场景为空或重复")
        expected_config = dict(reward_mode="dense", action_mode="reference", gas_force=450.)
        if bank["env_config"] != expected_config:
            raise RuntimeError("训练库控制配置错误")
        for item in bank["rows"]:
            scenario, baseline = item["scenario"], item["baseline"]
            if not scenario["id"].startswith("aligned-training-"):
                raise RuntimeError("不得把监控/验证/最终场景放入训练库")
            if any(scenario[key] != baseline[key] for key in ("id", "seed", "randomize", "options")):
                raise RuntimeError("奖励基线与训练场景错配")
            if item["spec"] != baseline_spec(baseline):
                raise RuntimeError("奖励归一化参数不匹配")
        expected_heat = sum(item["baseline"]["metrics"]["hip_heat_nm2s"] for item in bank["rows"])/len(cases)
        if bank["heat_scale"] != expected_heat:
            raise RuntimeError("全训练库热量尺度错误")
        super().__init__(WbrLearningEnv(randomize=True, strict=True, record=True, **expected_config))
        self.bank, self.weights = bank, dict(weights)
        self.selector = np.random.default_rng(0)
        self.deck, self.cursor = [], 0
        self.item = None

    def reset(self, *, seed=None, options=None):
        if options:
            raise ValueError("训练包装不接受外部场景替换")
        if seed is not None:
            self.selector = np.random.default_rng(seed)
            self.deck, self.cursor = [], 0
        if self.cursor >= len(self.deck):
            self.deck = self.selector.permutation(len(self.bank["rows"])).tolist()
            self.cursor = 0
        self.item = self.bank["rows"][self.deck[self.cursor]]
        self.cursor += 1
        case = self.item["scenario"]
        self.env.randomize = bool(case["randomize"])
        observation, info = self.env.reset(seed=case["seed"], options=dict(case["options"]))
        self.origin = self.env.data.xpos[self.env.base].copy()
        matrix = self.env.data.xmat[self.env.base].reshape(3,3)
        self.origin_yaw = math.atan2(matrix[1,0],matrix[0,0])
        self.previous_action, self.previous_heat = np.zeros(3), 0.
        self.total_return = 0.
        self.cost_totals = {name:0. for name in ("primary", "drive", "heat", "pose", "action_change", "action_magnitude")}
        return observation, info

    def step(self, action):
        observation, _, terminated, truncated, info = self.env.step(action)
        velocity = self.env.jac @ self.env.data.qvel
        matrix = self.env.data.xmat[self.env.base].reshape(3,3)
        yaw = math.atan2(matrix[1,0],matrix[0,0])-self.origin_yaw
        sample = self.env.trace[-1]
        sample.update(world_vx_mps=float(velocity[0]), world_vy_mps=float(velocity[1]),
            lateral_m=float(self.env.data.xpos[self.env.base,1]-self.origin[1]),
            longitudinal_m=float(self.env.data.xpos[self.env.base,0]-self.origin[0]),
            yaw_relative_rad=math.atan2(math.sin(yaw),math.cos(yaw)))
        penalty, violations = 0., {}
        if terminated:
            # 物理失败不能通过提前结束少交成本；不对可能无效的末帧计算密集奖励。
            reward, penalty, violations = -100., 100., {"physical_failure":100.}
        else:
            terms = step_costs(self.item["scenario"], sample, self.previous_action, self.previous_heat,
                               self.item["spec"], self.bank["heat_scale"])
            for name,value in terms.items():
                self.cost_totals[name] += value
            reward = -weighted_cost(terms, self.weights)
            if truncated:
                candidate = self.candidate_result()
                penalty, violations = guard_cost(self.item["baseline"], candidate)
                reward -= self.weights["guard"]*penalty
        self.previous_action, self.previous_heat = sample["action"], sample["hip_heat_nm2s"]
        self.total_return += reward
        if terminated or truncated:
            info["aligned_episode"] = dict(id=self.item["scenario"]["id"],
                case=self.item["scenario"]["options"]["case"], return_sum=self.total_return,
                dense_costs=self.cost_totals, guard_cost=penalty, violations=violations)
        # 这里的8秒定义完整训练任务，末帧已有整段约束罚分。
        # 返回真正终止，避免训练框架对时间截断继续加下一状态价值。
        return observation, float(reward), bool(terminated or truncated), False, info

    def candidate_result(self, include_trace=False):
        case = self.item["scenario"]
        summary = self.env.summary()
        row = dict(id=case["id"], seed=case["seed"], randomize=case["randomize"],
            options=dict(case["options"]), case=case["options"]["case"],
            metrics=summarize_trace(case, self.env.trace, summary))
        if summary["passed"]:
            row["motion_quality"] = motion_quality_metrics(case, self.env.trace)
        if include_trace:
            row["trace"] = list(self.env.trace)
        return row
