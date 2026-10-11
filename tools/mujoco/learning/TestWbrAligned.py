"""对齐训练的实质检查：数据隔离、推理边界、原物理一致与回合结算。"""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys
import tempfile


def require(value, message):
    if not value:
        raise RuntimeError(message)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--snapshot", type=Path, required=True)
    parser.add_argument("--baseline", type=Path)
    parser.add_argument("--original-baseline", type=Path)
    parser.add_argument("--bank", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    sys.path.insert(0,str(args.snapshot / "tools/mujoco/learning"))
    import numpy as np
    import torch
    from stable_baselines3 import PPO
    from EvaluateWbrStudy import load_cases
    from WbrLearningEnv import WbrLearningEnv, verify_snapshot, experiment_identity
    from TrainWbrLearning import export_policy
    from StudyWbrPolicy import StudyPolicy
    from WbrAlignedEnv import WbrAlignedEnv
    from WbrAlignedReward import score_trace
    torch.set_num_threads(1)
    read = lambda p: json.loads(p.read_text(encoding="utf-8"))
    digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    learning = args.snapshot / "tools/mujoco/learning"
    manifest = verify_snapshot()
    registration_path = learning / "results/study-20261012/对齐评分参考策略训练登记.json"
    rule = read(registration_path)
    checks = {}
    used = set()
    for name in ("development", "validation", "final", "pressure"):
        cases = load_cases(learning / "cases/study-20261012" / (name+".json"))
        used.update(row["seed"] for row in cases["cases"])
    for name,count in (("training",48),("monitor",8)):
        cases = load_cases(learning / "cases/aligned-20261012" / (name+".json"))
        seeds = {row["seed"] for row in cases["cases"]}
        require(len(seeds) == 3*count and not seeds & used, "训练数据重叠")
        used.update(seeds)
        require(cases["cases_sha256"] == rule[name+"_cases_sha256"],"场景登记不符")
        require(Counter(row["options"]["case"] for row in cases["cases"]) == dict(stop=count,push=count,combined=count), "任务不均衡")
        for kind in ("stop","push","combined"):
            rows = [row for row in cases["cases"] if row["options"]["case"] == kind]
            require(sum(row["options"]["speed"] > 0 for row in rows) == count//2,"速度方向不均衡")
            require(sum(row["options"]["push"] > 0 for row in rows) == count//2,"推力方向不均衡")
            if kind == "combined":
                require(sum(row["options"]["speed"]*row["options"]["push"] > 0 for row in rows) == count//2,"同反向受推不均衡")
        if name == "training":
            require(all(row["randomize"] for row in cases["cases"]),"训练随机化遗漏")
            require(Counter(row["options"]["delay"] for row in cases["cases"]) == {0:72,1:72},"延迟不均衡")
    checks["case_balance_and_seed_isolation"] = True
    export_errors = {}
    with tempfile.TemporaryDirectory(prefix="wbr-aligned-check-") as temp:
        for mode in ("reference","symmetric","full"):
            env = WbrLearningEnv(randomize=False,strict=True,reward_mode="dense",action_mode=mode)
            try:
                model = PPO("MlpPolicy",env,device="cpu",seed=902,n_steps=32,batch_size=16,
                    policy_kwargs=dict(net_arch=dict(pi=[64,64],vf=[64,64]),activation_fn=torch.nn.Tanh))
                with torch.no_grad():
                    model.policy.action_net.weight.zero_()
                    model.policy.action_net.bias.copy_(torch.tensor(([-.4,1.5,-1.5]*2)[:env.action_space.shape[0]]))
                directory = Path(temp) / mode
                directory.mkdir()
                contract = export_policy(model,directory)
                contract.update(kind="ppo",env_config=dict(action_mode=mode))
                (directory / "policy.json").write_text(json.dumps(contract),encoding="utf-8")
                observation = np.random.default_rng(42).normal(size=93).astype(np.float32)
                actual = StudyPolicy(directory).predict(observation)
                expected,_ = model.predict(observation,deterministic=True)
                require(np.array_equal(actual,expected),"导出动作边界不一致")
                require(actual[0] == (0. if mode == "reference" else np.float32(-.4)),"首动作边界错误")
                export_errors[mode] = contract["export_max_abs_error"]
                del contract["action_low"],contract["action_high"]
                (directory / "policy.json").write_text(json.dumps(contract),encoding="utf-8")
                require(np.array_equal(StudyPolicy(directory).predict(observation),expected),"旧契约兼容边界错误")
            finally:
                env.close()
    checks["export_and_legacy_action_bounds"] = export_errors
    if args.baseline or args.original_baseline:
        require(bool(args.baseline and args.original_baseline),"基线对照需同时提供两份汇总")
        before,after = read(args.original_baseline),read(args.baseline)
        require(after["experiment_identity"] == experiment_identity(manifest),"新基线身份不符")
        before_rows = before["baseline_results"] if "baseline_results" in before else before["baseline"]
        compact = lambda rows: {row["id"]:{key:value for key,value in row.items() if key not in ("controller","trace")} for row in rows}
        require(len(before_rows) == len(after["baseline_results"]) == 6 and compact(before_rows) == compact(after["baseline_results"]),"新旧6例零修正物理或质量不一致")
        checks["six_zero_cases_all_metrics_exact"] = True
    if args.bank:
        from TrainWbrAligned import validate_training_bank
        bank = read(args.bank)
        validate_training_bank(rule,registration_path,args.bank,manifest)
        with tempfile.TemporaryDirectory(prefix="wbr-aligned-tamper-") as temp:
            changed = dict(bank, heat_scale=bank["heat_scale"]*1.01)
            path = Path(temp) / "changed.json"
            path.write_text(json.dumps(changed),encoding="utf-8")
            try:
                validate_training_bank(rule,registration_path,path,manifest)
            except RuntimeError as error:
                require("独立许可" in str(error),"未在独立基线许可处拒绝")
            else:
                raise RuntimeError("篡改基线未被拒绝")
        checks["bank_replacement_rejected"] = True
        wrapped = WbrAlignedEnv(args.bank,digest(args.bank),rule["weights"])
        raw = WbrLearningEnv(randomize=True,strict=True,record=True,**bank["env_config"])
        try:
            for kind in ("stop","push","combined"):
                selector = next(seed for seed in range(1000) if bank["rows"][np.random.default_rng(seed).permutation(len(bank["rows"]))[0]]["scenario"]["options"]["case"] == kind)
                left,_ = wrapped.reset(seed=selector)
                case = wrapped.item["scenario"]
                raw.randomize = case["randomize"]
                right,_ = raw.reset(seed=case["seed"],options=dict(case["options"]))
                require(np.array_equal(left,right),"包装改变初始观测")
                total,index = 0.,0
                while True:
                    action = np.array([.02+.01*np.sin(index*.03),-.5+.1*np.cos(index*.017),.2*np.sin(index*.023)])
                    left,reward,done,truncated,info = wrapped.step(action)
                    right,_,raw_done,raw_truncated,_ = raw.step(action)
                    require(np.array_equal(left,right) and np.array_equal(wrapped.env.data.qpos,raw.data.qpos)
                        and np.array_equal(wrapped.env.data.qvel,raw.data.qvel) and wrapped.env.summary() == raw.summary(),"包装改变原控制/传感器观测")
                    total += reward
                    index += 1
                    if done:
                        require(not truncated and not raw_done and raw_truncated,"8秒未结算为完整训练回合")
                        break
                value = score_trace(case,wrapped.item["baseline"],wrapped.candidate_result(True),bank["heat_scale"],rule["weights"])
                require(abs(value["return_sum"]-total) < 1e-10,"完整回合在线离线评分不符")
                require(value["dense_costs"] == info["aligned_episode"]["dense_costs"] and value["violations"] == info["aligned_episode"]["violations"],"密集成本/终端保护不符")
                checks["wrapper_"+kind] = dict(samples=index,online_offline_error=abs(value["return_sum"]-total),same_original_physics=True)
            wrapped.reset(seed=8)
            total = 0.
            for _ in range(10):
                _,reward,done,_,_ = wrapped.step(np.zeros(3))
                require(not done,"故障注入前意外结束")
                total += reward
            wrapped.env.error = "injected_preflight_failure"
            _,reward,done,truncated,info = wrapped.step(np.zeros(3))
            total += reward
            value = score_trace(wrapped.item["scenario"],wrapped.item["baseline"],wrapped.candidate_result(True),bank["heat_scale"],rule["weights"])
            require(done and not truncated and reward == -100. and value["return_sum"] == total,"故障末帧或终端奖励不符")
            checks["physical_failure_terminal_and_offline_exact"] = True
        finally:
            wrapped.close()
            raw.close()
    result = dict(schema=1,checks=checks,experiment_identity=experiment_identity(manifest),
        library_sha256=manifest["library_sha256"],test_source_sha256=digest(Path(__file__)),
        bank_sha256=digest(args.bank) if args.bank else None)
    args.output.write_bytes((json.dumps(result,ensure_ascii=False,indent=2)+"\n").encode("utf-8"))
    print(json.dumps(checks,ensure_ascii=False,indent=2))


if __name__ == "__main__":
    main()
