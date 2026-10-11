"""在冻结的 WBR 模型上训练小幅修正策略，保存可复现记录和纯数值网络。"""
from __future__ import annotations

import argparse
import importlib.metadata
import json
import os
from pathlib import Path
import time

os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")

import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv

from WbrLearningEnv import ROOT, WbrLearningEnv, verify_snapshot, experiment_identity, POLICY_DT, FRAME_SIZE, HISTORY


class Progress(BaseCallback):
    def __init__(self, output):
        super().__init__()
        self.output, self.started, self.last = Path(output), time.perf_counter(), 0
        self.episodes = 0
        self.failures = 0

    def _on_step(self):
        for info in self.locals["infos"]:
            if "measurements" in info:
                self.episodes += 1
                self.failures += int(not info["measurements"]["passed"])
        if self.num_timesteps-self.last >= 1024:
            self.last = self.num_timesteps
            elapsed = time.perf_counter()-self.started
            value = dict(steps=self.num_timesteps, elapsed_s=elapsed,
                         steps_per_s=self.num_timesteps/max(elapsed, 1e-6),
                         episodes=self.episodes, failures=self.failures)
            print(json.dumps(value), flush=True)
            with (self.output/"progress.jsonl").open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(value)+"\n")
        return True


def export_policy(model, output):
    layers = [m for m in model.policy.mlp_extractor.policy_net if isinstance(m, torch.nn.Linear)]
    layers.append(model.policy.action_net)
    arrays = {}
    for i, layer in enumerate(layers):
        arrays[f"w{i}"] = layer.weight.detach().cpu().numpy()
        arrays[f"b{i}"] = layer.bias.detach().cpu().numpy()
    target = Path(output)/"policy.npz"
    np.savez(target, **arrays)
    rng = np.random.default_rng(823)
    observations = rng.normal(0, .5, (200, FRAME_SIZE*HISTORY)).astype(np.float32)
    expected, _ = model.predict(observations, deterministic=True)
    actual = observations
    for i in range(len(layers)):
        actual = actual @ arrays[f"w{i}"].T + arrays[f"b{i}"]
        if i < len(layers)-1:
            actual = np.tanh(actual)
    lower, upper = model.action_space.low, model.action_space.high
    error = float(np.max(np.abs(np.clip(actual,lower,upper)-expected)))
    if error > 2e-5:
        raise RuntimeError(f"纯数值网络与训练框架输出不一致：{error}")
    return dict(layers=[layer.in_features for layer in layers]+[layers[-1].out_features],
                parameters=sum(a.size for a in arrays.values()), export_max_abs_error=error,
                action_low=lower.tolist(), action_high=upper.tolist(),
                policy_dt_s=POLICY_DT, history_frames=HISTORY, frame_size=FRAME_SIZE,
                scope="仅限本次平地停车/纵向抗扰，不支持起身或跳跃的学习控制")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--steps", type=int, default=32768)
    parser.add_argument("--envs", type=int, default=2)
    parser.add_argument("--seed", type=int, default=31)
    args = parser.parse_args()
    if args.steps <= 0 or not 1 <= args.envs <= 4:
        parser.error("步数必须为正，环境数限定 1～4")
    if args.output.exists():
        raise FileExistsError("训练输出目录已存在，拒绝覆盖")
    args.output.mkdir(parents=True)
    manifest = verify_snapshot()
    torch.set_num_threads(1)
    constructors = [lambda: Monitor(WbrLearningEnv(randomize=True, strict=True)) for _ in range(args.envs)]
    env = DummyVecEnv(constructors) if args.envs == 1 else SubprocVecEnv(constructors, start_method="spawn")
    try:
        model = PPO("MlpPolicy", env, seed=args.seed, device="cpu", verbose=0,
            learning_rate=2e-4, n_steps=512, batch_size=256, n_epochs=6,
            gamma=.995, gae_lambda=.95, clip_range=.15, target_kl=.025,
            policy_kwargs=dict(net_arch=dict(pi=[64,64], vf=[64,64]),
                               activation_fn=torch.nn.Tanh, log_std_init=-1.5))
        # 初始确定性输出就是原控制，探索噪声只在主机仿真里启用。
        torch.nn.init.zeros_(model.policy.action_net.weight)
        torch.nn.init.zeros_(model.policy.action_net.bias)
        started = time.perf_counter()
        model.learn(total_timesteps=args.steps, callback=Progress(args.output))
        elapsed = time.perf_counter()-started
        model.save(args.output/"ppo_checkpoint")
        contract = export_policy(model, args.output)
        contract.update(seed=args.seed, training_steps=model.num_timesteps, training_seconds=elapsed,
            snapshot=str(ROOT), library_sha256=manifest["library_sha256"],
            experiment_identity=experiment_identity(manifest), strict_training_checks=True,
            versions={p:importlib.metadata.version(p) for p in ("mujoco","numpy","torch","gymnasium","stable-baselines3")})
        (args.output/"policy.json").write_text(json.dumps(contract, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
        print(json.dumps(contract, ensure_ascii=False), flush=True)
    finally:
        env.close()


if __name__ == "__main__":
    main()
