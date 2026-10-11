"""有检查点、可续训的轮腿研究训练入口；保留每次续训的边界。"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import functools
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import time

os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")

import torch
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv

from TrainWbrLearning import export_policy
from WbrLearningEnv import ROOT, WbrLearningEnv, verify_snapshot, experiment_identity


def write_json(path, value):
    path = Path(path)
    temporary = path.with_suffix(path.suffix+".tmp")
    temporary.write_bytes((json.dumps(value, ensure_ascii=False, indent=2)+"\n").encode("utf-8"))
    temporary.replace(path)


def make_env(config):
    return Monitor(WbrLearningEnv(randomize=True, strict=True, **config))


class StudyProgress(BaseCallback):
    def __init__(self, output, metadata, checkpoint_every, deadline):
        super().__init__()
        self.output, self.metadata = Path(output), metadata
        self.every, self.deadline = checkpoint_every, deadline
        self.started, self.last_log = time.perf_counter(), 0
        self.last_checkpoint = metadata["initial_steps"]
        self.episodes, self.failures, self.stop_reason = 0, 0, None

    def save(self, directory):
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        temporary = directory/"checkpoint.pending.zip"
        self.model.save(temporary)
        checkpoint = directory/"checkpoint.zip"
        temporary.replace(checkpoint)
        contract = export_policy(self.model, directory)
        contract.update(self.metadata)
        contract.update(kind="ppo", training_steps=self.model.num_timesteps,
                        segment_seconds=time.perf_counter()-self.started,
                        segment_episodes=self.episodes, segment_failures=self.failures,
                        stop_reason=self.stop_reason,
                        checkpoint_sha256=hashlib.sha256(checkpoint.read_bytes()).hexdigest(),
                        policy_sha256=hashlib.sha256((directory/"policy.npz").read_bytes()).hexdigest(),
                        created_at=datetime.now(timezone.utc).isoformat())
        write_json(directory/"policy.json", contract)
        return contract

    def _on_step(self):
        for info in self.locals["infos"]:
            if "measurements" in info:
                self.episodes += 1
                self.failures += int(not info["measurements"]["passed"])
        if self.num_timesteps-self.last_log >= 4096:
            self.last_log = self.num_timesteps
            elapsed = time.perf_counter()-self.started
            progress = dict(steps=self.num_timesteps, segment_seconds=elapsed,
                steps_per_s=(self.num_timesteps-self.metadata["initial_steps"])/max(elapsed,1e-6),
                episodes=self.episodes, failures=self.failures, pid=os.getpid())
            print(json.dumps(progress), flush=True)
            with (self.output/"progress.jsonl").open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(progress)+"\n")
            write_json(self.output/"status.json", dict(status="running", **progress))
        if self.num_timesteps-self.last_checkpoint >= self.every:
            directory = self.output/"checkpoints"/f"step-{self.num_timesteps:09d}"
            self.save(directory)
            self.last_checkpoint = self.num_timesteps
            write_json(self.output/"latest-checkpoint.json", dict(path=str(directory.resolve()),
                       steps=self.num_timesteps))
        if self.num_timesteps % 128 == 0:
            if (self.output/"stop.request").exists():
                self.stop_reason = "stop_file"
            elif self.deadline and time.time() >= self.deadline:
                self.stop_reason = "deadline"
            if self.stop_reason:
                return False
        return True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--steps", type=int, default=131072, help="包括续训前步数的总目标")
    parser.add_argument("--envs", type=int, choices=(1,2), default=2)
    parser.add_argument("--seed", type=int, default=41)
    parser.add_argument("--reward-mode", choices=("legacy","dense"), default="dense")
    parser.add_argument("--action-mode", choices=("full","symmetric"), default="full")
    parser.add_argument("--resume", type=Path)
    parser.add_argument("--checkpoint-every", type=int, default=16384)
    parser.add_argument("--deadline", default="2026-10-14T02:10:00+08:00")
    args = parser.parse_args()
    if args.steps <= 0 or args.checkpoint_every < 1024:
        parser.error("步数必须为正，检查点间隔至少1024步")
    deadline = datetime.fromisoformat(args.deadline).timestamp() if args.deadline else None
    if deadline and time.time() >= deadline:
        raise RuntimeError("研究期限已到，不启动训练")
    if args.output.exists():
        raise FileExistsError("训练目录已存在；续训也应使用新目录")
    manifest = verify_snapshot()
    config = dict(reward_mode=args.reward_mode, action_mode=args.action_mode, gas_force=450.)
    metadata = dict(seed=args.seed, env_config=config, strict_training_checks=True,
        snapshot=str(ROOT), library_sha256=manifest["library_sha256"],
        experiment_identity=experiment_identity(manifest), initial_steps=0,
        training_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        resume=None, requested_total_steps=args.steps, deadline=args.deadline,
        versions={p:importlib.metadata.version(p) for p in ("mujoco","numpy","torch","gymnasium","stable-baselines3")})
    if args.resume:
        old = json.loads((args.resume/"policy.json").read_text(encoding="utf-8"))
        for key in ("seed", "env_config", "library_sha256", "experiment_identity", "training_source_sha256"):
            if old[key] != metadata[key]:
                raise RuntimeError(f"续训配置不一致：{key}")
        if hashlib.sha256((args.resume/"checkpoint.zip").read_bytes()).hexdigest() != old["checkpoint_sha256"]:
            raise RuntimeError("检查点校验失败")
        metadata.update(initial_steps=old["training_steps"], resume=str(args.resume.resolve()),
                        resume_semantics="保留网络及优化器；环境与随机序列重新初始化，不是逐步重放")
    args.output.mkdir(parents=True)
    write_json(args.output/"run.json", metadata)
    torch.set_num_threads(1)
    constructors = [functools.partial(make_env, config) for _ in range(args.envs)]
    env = DummyVecEnv(constructors) if args.envs == 1 else SubprocVecEnv(constructors, start_method="spawn")
    callback = StudyProgress(args.output, metadata, args.checkpoint_every, deadline)
    try:
        if args.resume:
            model = PPO.load(args.resume/"checkpoint.zip", env=env, device="cpu")
            model.set_random_seed(args.seed+metadata["initial_steps"])
        else:
            model = PPO("MlpPolicy", env, seed=args.seed, device="cpu", verbose=0,
                learning_rate=3e-4, n_steps=512, batch_size=256, n_epochs=6,
                gamma=.995, gae_lambda=.95, clip_range=.15, target_kl=.025,
                policy_kwargs=dict(net_arch=dict(pi=[64,64],vf=[64,64]),
                                   activation_fn=torch.nn.Tanh, log_std_init=-.8))
            torch.nn.init.zeros_(model.policy.action_net.weight)
            torch.nn.init.zeros_(model.policy.action_net.bias)
        remaining = args.steps-model.num_timesteps
        if remaining <= 0:
            raise ValueError("检查点已达到目标步数，无需续训")
        try:
            model.learn(total_timesteps=remaining, reset_num_timesteps=not bool(args.resume), callback=callback)
        except KeyboardInterrupt:
            callback.stop_reason = "keyboard_interrupt"
        callback.save(args.output)
        status = "completed" if model.num_timesteps >= args.steps else "interrupted"
        write_json(args.output/"status.json", dict(status=status, steps=model.num_timesteps,
            stop_reason=callback.stop_reason, pid=os.getpid(), updated_at=datetime.now(timezone.utc).isoformat()))
        print(json.dumps(dict(status=status, steps=model.num_timesteps, output=str(args.output)), ensure_ascii=False), flush=True)
    except Exception as error:
        write_json(args.output/"status.json", dict(status="failed", error=repr(error), pid=os.getpid()))
        raise
    finally:
        env.close()


if __name__ == "__main__":
    main()
