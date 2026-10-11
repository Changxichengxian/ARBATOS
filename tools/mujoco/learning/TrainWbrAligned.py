"""使用冻结训练库与对齐评分训练参考策略；续训保留网络/优化器。"""
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
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv

from TrainWbrStudy import StudyProgress, write_json
from WbrAlignedEnv import WbrAlignedEnv
from WbrLearningEnv import ROOT, verify_snapshot, experiment_identity


def make_env(bank_path, bank_sha256, weights):
    return Monitor(WbrAlignedEnv(bank_path, bank_sha256, weights))


def validate_training_bank(registration, registration_path, bank_path, manifest):
    digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    lock_relative = registration["training_bank_lock"]
    lock_path = ROOT / lock_relative
    if lock_relative not in manifest["files"] or digest(lock_path) != manifest["files"][lock_relative]:
        raise RuntimeError("尚未冻结已核验的训练基线许可记录")
    bank_lock = json.loads(lock_path.read_text(encoding="utf-8"))
    if bank_lock["registration_sha256"] != digest(registration_path) or bank_lock["bank_sha256"] != digest(bank_path):
        raise RuntimeError("训练基线或登记与独立许可记录不符")
    for relative, expected in bank_lock["source_files"].items():
        if relative not in manifest["files"] or manifest["files"][relative] != expected or digest(ROOT / relative) != expected:
            raise RuntimeError("训练基线原始运行/汇总与冻结许可不符")


def resume_lineage(registration, seed, resume_path, previous, promotion, promotion_sha256):
    """扩展训练必须继承实际晋级的短训权重，之后只能接回同一来源的检查点。"""
    stage = (promotion["stage"] if promotion else
             "smoke" if seed == registration["smoke_seed"] else "screen_first")
    lineage = dict(stage=stage,seed=seed,promotion_sha256=promotion_sha256)
    if stage == "screen_expand":
        if resume_path is None or previous is None:
            raise RuntimeError("筛选扩展必须从被晋级的短训最终检查点续训")
        origins = [row for row in promotion["results"] if row["seed"] == seed]
        if len(origins) != 1:
            raise RuntimeError("晋级记录缺少唯一同种子的短训来源")
        origin = Path(origins[0]["policy"]).resolve()
        origin_path = origin / "policy.json"
        origin_contract = json.loads(origin_path.read_text(encoding="utf-8"))
        origin_sha = hashlib.sha256(origin_path.read_bytes()).hexdigest()
        lineage.update(origin_policy=str(origin),origin_policy_sha256=origin_sha,
            origin_checkpoint_sha256=origin_contract["checkpoint_sha256"],
            origin_steps=registration["screening_first"]["steps"])
        if origin_contract["training_steps"] != lineage["origin_steps"]:
            raise RuntimeError("晋级来源不是短训最终预算")
        if Path(resume_path).resolve() == origin:
            if previous != origin_contract:
                raise RuntimeError("续训输入不是实际晋级的短训最终权重")
        elif (previous.get("training_lineage") != lineage or
              previous.get("requested_total_steps") != max(registration["training_steps"]) or
              previous.get("training_steps",0) < lineage["origin_steps"]):
            raise RuntimeError("扩展检查点不是从实际晋级权重继承的训练")
    elif previous is not None and previous.get("training_lineage") != lineage:
        raise RuntimeError("续训检查点的阶段或训练来源不匹配")
    return lineage


class AlignedProgress(StudyProgress):
    def _on_step(self):
        episodes = [info["aligned_episode"] for info in self.locals["infos"] if "aligned_episode" in info]
        if episodes:
            with (self.output / "aligned-episodes.jsonl").open("a", encoding="utf-8") as stream:
                for episode in episodes:
                    stream.write(json.dumps(dict(step=self.num_timesteps, **episode), ensure_ascii=False)+"\n")
        return super()._on_step()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--bank", type=Path, required=True)
    parser.add_argument("--registration", type=Path, required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--steps", type=int, required=True)
    parser.add_argument("--envs", type=int, choices=(1,2), default=2)
    parser.add_argument("--resume", type=Path)
    parser.add_argument("--promotion", type=Path, help="扩展或正式训练时必须提供已复核的晋级记录")
    args = parser.parse_args()
    read = lambda p: json.loads(p.read_text(encoding="utf-8"))
    digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    registration = read(args.registration)
    if registration["route"] != "E-aligned-reference":
        raise ValueError("训练登记路线不匹配")
    smoke = args.seed == registration["smoke_seed"]
    allowed = registration["smoke_steps"] if smoke else registration["training_steps"]
    if args.steps not in allowed or args.seed not in registration["screening_seeds"]+registration["formal_seeds"]+[registration["smoke_seed"]]:
        raise ValueError("种子或预算未登记")
    if not smoke and args.envs != registration["envs"]:
        raise ValueError("长训练环境数必须按登记执行")
    deadline = datetime.fromisoformat(registration["deadline"]).timestamp()
    if time.time() >= deadline:
        raise RuntimeError("研究期限已到")
    if args.output.exists():
        raise FileExistsError("输出目录已存在；续训也使用新目录")
    promotion, promotion_sha256 = None, None
    if not smoke and args.steps == max(registration["training_steps"]):
        if args.promotion is None:
            raise ValueError("扩大预算或正式训练必须先复核晋级记录")
        promotion = read(args.promotion)
        stage = "formal" if args.seed in registration["formal_seeds"] else "screen_expand"
        if promotion.get("eligible") is not True or promotion.get("stage") != stage or promotion.get("registration_sha256") != digest(args.registration):
            raise RuntimeError("晋级记录不适用")
        approved = datetime.fromisoformat(promotion["approved_at"])
        cutoff = datetime.fromisoformat(registration["latest_formal_promotion"] if stage == "formal"
                                       else registration["latest_screen_expansion"])
        if approved.utcoffset() is None or approved > cutoff or approved.timestamp() > time.time():
            raise RuntimeError("晋级时间没有留足后续训练和评价预算")
        if not promotion.get("evidence_files"):
            raise RuntimeError("晋级记录缺少可复核来源")
        for filename, expected in promotion["evidence_files"].items():
            if digest(Path(filename)) != expected:
                raise RuntimeError("晋级依据发生变化")
        from PromoteWbrAligned import verify_promotion
        verify_promotion(args.promotion)
        promotion_sha256 = digest(args.promotion)
    if args.seed in registration["formal_seeds"] and args.steps != max(registration["training_steps"]):
        raise ValueError("正式种子必须使用登记的完整预算")
    manifest = verify_snapshot()
    registered_name = "tools/mujoco/learning/results/study-20261012/对齐评分参考策略训练登记.json"
    if digest(args.registration) != manifest["files"].get(registered_name):
        raise RuntimeError("登记与冻结版本不匹配")
    if registration["reward_source_sha256"] != manifest["files"]["tools/mujoco/learning/WbrAlignedReward.py"]:
        raise RuntimeError("评分代码已偏离登记")
    validate_training_bank(registration, args.registration, args.bank, manifest)
    bank = read(args.bank)
    if bank["cases_sha256"] != registration["training_cases_sha256"]:
        raise RuntimeError("训练场景库未登记")
    algorithm = registration["algorithm"]
    sources = {name:manifest["files"]["tools/mujoco/learning/"+name] for name in
               ("WbrAlignedEnv.py", "WbrAlignedReward.py", "TrainWbrAligned.py", "TrainWbrStudy.py", "TrainWbrLearning.py")}
    protocol = dict(bank_sha256=digest(args.bank), registration_sha256=digest(args.registration),
                    sources=sources, weights=registration["weights"], algorithm=algorithm)
    metadata = dict(seed=args.seed, envs=args.envs, env_config=bank["env_config"], training_protocol=protocol,
        strict_training_checks=True, snapshot=str(ROOT), library_sha256=manifest["library_sha256"],
        experiment_identity=experiment_identity(manifest), initial_steps=0, resume=None,
        training_source_sha256=digest(Path(__file__)), requested_total_steps=args.steps,
        deadline=registration["deadline"], promotion_sha256=promotion_sha256,
        versions={p:importlib.metadata.version(p) for p in ("mujoco","numpy","torch","gymnasium","stable-baselines3")})
    previous = None
    if args.resume:
        previous = read(args.resume / "policy.json")
        for key in ("seed", "envs", "env_config", "training_protocol", "library_sha256", "experiment_identity", "training_source_sha256"):
            if previous[key] != metadata[key]:
                raise RuntimeError("续训配置不一致："+key)
        if digest(args.resume / "checkpoint.zip") != previous["checkpoint_sha256"]:
            raise RuntimeError("检查点内容不匹配")
        metadata.update(initial_steps=previous["training_steps"], resume=str(args.resume.resolve()),
            resume_semantics="保留网络和优化器；重新初始化物理回合及训练库洗牌，非逐步重放。场景选择种子为训练种子加已训练步数。")
    metadata["training_lineage"] = resume_lineage(registration,args.seed,args.resume,previous,promotion,promotion_sha256)
    args.output.mkdir(parents=True)
    write_json(args.output / "run.json", metadata)
    torch.set_num_threads(1)
    constructors = [functools.partial(make_env, str(args.bank.resolve()), protocol["bank_sha256"], registration["weights"])
                    for _ in range(args.envs)]
    env = None
    try:
        env = DummyVecEnv(constructors) if args.envs == 1 else SubprocVecEnv(constructors, start_method="spawn")
        callback = AlignedProgress(args.output, metadata, registration["checkpoint_every"], deadline)
        if args.resume:
            model = PPO.load(args.resume / "checkpoint.zip", env=env, device="cpu")
            model.set_random_seed(args.seed+metadata["initial_steps"])
        else:
            model = PPO("MlpPolicy", env, seed=args.seed, device="cpu", verbose=0,
                learning_rate=algorithm["learning_rate"], n_steps=algorithm["n_steps"],
                batch_size=algorithm["batch_size"], n_epochs=algorithm["n_epochs"],
                gamma=algorithm["gamma"], gae_lambda=algorithm["gae_lambda"],
                clip_range=algorithm["clip_range"], target_kl=algorithm["target_kl"],
                policy_kwargs=dict(net_arch=dict(pi=[64,64],vf=[64,64]),
                    activation_fn=torch.nn.Tanh, log_std_init=algorithm["log_std_init"]))
            torch.nn.init.zeros_(model.policy.action_net.weight)
            with torch.no_grad():
                model.policy.action_net.bias.copy_(torch.tensor(algorithm["initial_action"],dtype=torch.float32))
        remaining = args.steps-model.num_timesteps
        if remaining <= 0:
            raise ValueError("检查点已达到目标步数")
        try:
            model.learn(total_timesteps=remaining, reset_num_timesteps=not bool(args.resume), callback=callback)
        except KeyboardInterrupt:
            callback.stop_reason = "keyboard_interrupt"
        callback.save(args.output)
        status = "completed" if model.num_timesteps >= args.steps else "interrupted"
        write_json(args.output / "status.json", dict(status=status, steps=model.num_timesteps,
            stop_reason=callback.stop_reason, pid=os.getpid(), updated_at=datetime.now(timezone.utc).isoformat()))
        print(json.dumps(dict(status=status, steps=model.num_timesteps, output=str(args.output)),ensure_ascii=False))
    except Exception as error:
        write_json(args.output / "status.json", dict(status="failed", error=repr(error), pid=os.getpid()))
        raise
    finally:
        if env is not None:
            env.close()


if __name__ == "__main__":
    main()
