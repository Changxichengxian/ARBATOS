"""只重跑被暂停的F73；保留旧记录，保证最终模型完成相同训练预算。"""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys

workspace = Path(__file__).resolve().parents[1]
base = workspace / "work/study-20261012/long-trace"
learning = base / "frozen/tools/mujoco/learning"
sys.path.insert(0, str(learning))
import RunWbrStudyQueue as queue
from WbrLearningEnv import verify_snapshot
from TrainWbrAligned import state_digest
from stable_baselines3 import PPO
import torch

read = lambda p: json.loads(p.read_text(encoding="utf-8"))
digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
original_selector = queue.training_checkpoint
original_output = base / "training/73/train"
original_plan = base / "training/plan.json"
known_hashes = {
    16384: None,
    32768: "a2cc6c7df02b4b4351452b8d1391bfc454606721772fb422e9fdf8202784067b",
    42496: "471659866b8b1adae1105946c9343d32a0024b2092ca0d199c41753a661df389",
}

def require(value, message):
    if not value:
        raise RuntimeError(message)

def restart_only_paused_seed(directories, arguments, source, script_sha, source_hashes=None):
    require([Path(p).resolve() for p in directories] == [original_output.resolve()], "本入口仅允许第一次F73恢复")
    require(queue.option(arguments, "--seed") == "73" and queue.option(arguments, "--steps") == "262144", "种子或预算不符")
    latest = original_selector(directories, arguments, source, script_sha, source_hashes)
    require(latest == original_output, "最新保存点发生变化")
    require(read(latest / "policy.json")["training_steps"] == 42496, "旧暂停计数变化")
    require(digest(latest / "checkpoint.zip") == known_hashes[42496], "旧暂停模型变化")
    # 该模型可用于一般续训，但不满足本次相同完整更新预算的严格比较要求。
    # 显式启用原队列的从零重跑分支，旧记录不删除，不伪造已完成步数。
    return None

def main():
    require(sys.argv[1:] in ([], ["--check-only"]), "只支持执行或预检")
    require(digest(original_plan) == "19cb27160aab686d3dfe06f2b4e3e1348fb1b9121d67f2fd38e4b1f6ea0703ce", "原计划变化")
    require(digest(learning / "RunWbrStudyQueue.py") == "d3facba30a312907821c6dc24e6253227af684ef40e9f4763ee01f439c61bdd8", "原队列变化")
    manifest = verify_snapshot(base / "frozen")
    plan, state = read(original_plan), read(base / "training/queue-state.json")
    require(state["status"] == "needs_attention" and state["jobs"]["F-trace-41-train"]["status"] == "completed", "队列非已知暂停状态")
    require(len(state["jobs"]["F-trace-73-train"]["attempts"]) == 1, "拒绝重复重跑")
    job = plan["jobs"][1]
    binding = queue.make_job_binding(job["kind"], job["arguments"], plan)
    require(binding == state["jobs"][job["id"]]["job_binding"], "步骤来源改变")
    p = binding["payload"]
    args = (job["arguments"], p["source"], p["script_sha256"], p["source_hashes"])
    require(restart_only_paused_seed([original_output], *args) is None, "未选择原队列从零路径")
    for invalid in ([base / "training/41/train"], [original_output, base / "training/73/train-resume-02"]):
        try:
            restart_only_paused_seed(invalid, *args)
        except RuntimeError:
            pass
        else:
            raise RuntimeError("错误来源未被拒绝")
    torch.set_num_threads(1)
    evidence = []
    for candidate in (original_output, *sorted((original_output / "checkpoints").iterdir())):
        require(original_selector([candidate], *args) == candidate, "旧检查点绑定不符")
        model = PPO.load(candidate / "checkpoint.zip", device="cpu")
        policy = read(candidate / "policy.json")
        steps = model.num_timesteps
        require(steps == policy["training_steps"] and steps in known_hashes, "出现未知检查点")
        if known_hashes[steps]:
            require(digest(candidate / "checkpoint.zip") == known_hashes[steps], "模型字节变化")
        batch = model.n_steps * model.n_envs
        require(batch == 2048 and model.n_epochs == 6 and model.gae_lambda == 1, "实际训练算法不符")
        complete = steps % batch == 0 and model._n_updates == steps // batch * model.n_epochs
        require(not complete, "存在可完整续训的检查点，需重新考虑方案")
        evidence.append(dict(path=str(candidate), steps=steps, n_updates=model._n_updates,
            expected_full_batch_updates=steps // batch * model.n_epochs,
            steps_aligned=steps % batch == 0, full_update_boundary=complete,
            checkpoint_sha256=digest(candidate / "checkpoint.zip"),
            network_state_sha256=state_digest(model.policy.state_dict()),
            optimizer_state_sha256=state_digest(model.policy.optimizer.state_dict())))
    proof = dict(schema=1, checked_at=datetime.now(timezone.utc).isoformat(), action="restart_seed_73_from_zero",
        same_seed=73, final_training_steps=262144, discarded_attempt_steps=42496,
        expected_final_update_counter=768, checkpoints=evidence,
        plan_sha256=digest(original_plan), frozen_file_count=len(manifest["files"]),
        recovery_entry_sha256=digest(Path(__file__)), wrong_source_checks_passed=2,
        reason="原检查点在采样回调中保存，32768计数对应90次更新而非96次；42496计数对应120次更新且不对齐批次。为保持最终完整训练预算，保留旧尝试并从原seed73重跑。",
        performance_selection=False, frozen_sources_changed=False,
        boundary="额外耗费先前42496步计算；最终候选仍是原种子、算法、数据及262144步。F41不重跑，旧模型不删除，未读取F性能结果。")
    (base / "暂停重跑预检.json").write_text(json.dumps(proof, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    print(json.dumps(proof, ensure_ascii=False), flush=True)
    if sys.argv[1:] == ["--check-only"]:
        return
    require(not (base / "training/stop.request").exists(), "用户恢复的停止标记尚未解除")
    queue.training_checkpoint = restart_only_paused_seed
    sys.argv = [str(learning / "RunWbrStudyQueue.py"), "--plan", str(original_plan), "--resume"]
    queue.main()

if __name__ == "__main__":
    main()
