"""用实际训练库隔离测量末尾误差信号的直接传播，不运行策略训练。"""
from datetime import datetime
import hashlib
import inspect
import json
from pathlib import Path

import gymnasium as gym
import numpy as np
import torch
from stable_baselines3.common.buffers import RolloutBuffer

workspace = Path(__file__).resolve().parents[1]
base = workspace / "work/study-20261012/aligned-reference"
rule_path = base / "training-frozen/tools/mujoco/learning/results/study-20261012/对齐评分参考策略训练登记.json"
rule = json.loads(rule_path.read_text(encoding="utf-8"))
gamma = rule["algorithm"]["gamma"]
trace_lambda = rule["algorithm"]["gae_lambda"]
steps = rule["algorithm"]["n_steps"]
episode_steps = 534
dt = .015
digest = lambda path: hashlib.sha256(Path(path).read_bytes()).hexdigest()
torch.set_num_threads(1)

def experiment(lam):
    buffer = RolloutBuffer(steps, gym.spaces.Box(-np.inf, np.inf, (93,), np.float32),
        gym.spaces.Box(np.array([0., -1., -1.], dtype=np.float32), np.ones(3, dtype=np.float32)),
        device="cpu", gamma=gamma, gae_lambda=lam, n_envs=rule["envs"])
    # 保持价值估计为零，只注入一个末尾单位负奖励，隔离这条直接信号。
    buffer.rewards[episode_steps - 1, :] = -1.
    buffer.episode_starts[0, :] = 1.
    buffer.episode_starts[episode_steps, :] = 1.
    buffer.compute_returns_and_advantage(torch.zeros(rule["envs"]), np.ones(rule["envs"], dtype=bool))
    actual = -buffer.advantages[:episode_steps, 0].astype(np.float64)
    reference = (gamma * lam) ** np.arange(episode_steps - 1, -1, -1)
    if not np.allclose(actual, reference, rtol=3e-5, atol=1e-10):
        raise RuntimeError("实际库输出与指数项不一致")
    if np.any(buffer.advantages[episode_steps:]) or not np.array_equal(buffer.advantages[:, 0], buffer.advantages[:, 1]):
        raise RuntimeError("跨回合传播或两个环境的隔离结果不符")
    return actual, float(np.max(np.abs(actual - reference)))

current, error = experiment(trace_lambda)
comparison, other_error = experiment(1.)
rows = []
for requested_seconds in (.0, .495, 1.005, 1.995, 3., 4.995):
    lag = round(requested_seconds / dt)
    index = episode_steps - 1 - lag
    rows.append(dict(lag_steps=lag, lag_seconds=lag * dt,
        current_direct_coefficient=float(current[index]), lambda_one_reference=float(comparison[index])))
frozen_learning = base / "training-frozen/tools/mujoco/learning"
record = dict(schema=1, checked_at=datetime.now().astimezone().isoformat(),
    gamma=gamma, gae_lambda=trace_lambda, policy_dt_seconds=dt, rollout_steps=steps,
    envs=rule["envs"], isolated_episode_steps=episode_steps,
    scenario="零价值估计，完整回合末尾单位负奖励；其它奖励为零，之后明确开始新回合。",
    actual_training_library_used=True, formula="direct_coefficient = (gamma * gae_lambda) ** lag_steps",
    current_maximum_absolute_formula_error=error, reference_maximum_absolute_formula_error=other_error,
    rows=rows, no_cross_episode_leakage=True,
    source_files={str(path): digest(path) for path in (Path(inspect.getfile(RolloutBuffer)), rule_path,
        frozen_learning / "WbrAlignedEnv.py", frozen_learning / "TrainWbrAligned.py", Path(__file__))},
    references=["https://arxiv.org/abs/1506.02438", "https://people.eecs.berkeley.edu/~russell/papers/icml99-shaping.pdf"],
    findings="登记实现将逐例约束罚分放在回合末尾。实际库的直接末尾误差项随时间间隔指数衰减，提示反馈时序值得进一步检验。",
    limitations=["没有运行新策略训练，没有更改正在运行的评分或超参数。",
        "这是隔离单位信号的诊断，不是实际策略梯度大小、完整学习能力或控制性能。",
        "实际价值函数可传播未来约束信息，密集奖励及优势归一化也会影响训练；不能据此断言退步原因已证明。",
        "lambda=1只作数值参照，增大该参数会改变偏差与方差，未证明会改善控制。",
        "任何新评分时序都须独立登记、验证完整与折扣回报及终止边界，再用新监控场景评价。"])
target = base / "credit-timing-diagnostic.json"
if target.exists():
    raise FileExistsError(target)
target.write_bytes((json.dumps(record, ensure_ascii=False, indent=2) + "\n").encode("utf-8"))
print(json.dumps(dict(rows=rows, maximum_absolute_error=error, no_cross_episode_leakage=True), ensure_ascii=False, indent=2))
