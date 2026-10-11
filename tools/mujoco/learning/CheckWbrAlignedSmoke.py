"""核验真实1024至2048步续训：实际网络、优化器载入值及后续更新。"""
import argparse
import hashlib
import io
import json
from pathlib import Path
import zipfile

import numpy as np
import torch
from stable_baselines3 import PPO

from TrainWbrAligned import state_digest
from WbrLearningEnv import verify_snapshot,experiment_identity,ROOT
from StudyWbrPolicy import StudyPolicy


def check(first,resumed):
    read = lambda p:json.loads(p.read_text(encoding="utf-8"))
    digest = lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    def require(value,message):
        if not value:
            raise RuntimeError(message)
    manifest = verify_snapshot()
    first,resumed = Path(first).resolve(),Path(resumed).resolve()
    a,b = read(first / "policy.json"),read(resumed / "policy.json")
    for directory,policy,steps in ((first,a,1024),(resumed,b,2048)):
        status = read(directory / "status.json")
        require(status["status"] == "completed" and status["steps"] == policy["training_steps"] == steps,"短训步数未完成")
        require(policy["seed"] == 9001 and policy["envs"] == 1,"短训种子或环境数不符")
        require(policy["experiment_identity"] == experiment_identity(manifest) and policy["library_sha256"] == manifest["library_sha256"],"短训物理版本不符")
        for filename,key in (("checkpoint.zip","checkpoint_sha256"),("policy.npz","policy_sha256")):
            require(digest(directory / filename) == policy[key],"保存的模型内容不符")
        require(all(policy[key] == value for key,value in read(directory / "run.json").items()),"模型与运行元信息不符")
    require(a["initial_steps"] == 0 and b["initial_steps"] == 1024 and Path(b["resume"]).resolve() == first,"实际续训来源不符")
    require(a["training_protocol"] == b["training_protocol"] and a["training_lineage"] == b["training_lineage"],"续训协议发生改变")
    proof = read(resumed / "resume-load-check.json")
    require(proof["loaded_steps"] == 1024 and proof["checkpoint_sha256"] == a["checkpoint_sha256"],"训练载入记录错配")
    require(proof["training_source_sha256"] == manifest["files"]["tools/mujoco/learning/TrainWbrAligned.py"],"载入核验代码不符")
    with zipfile.ZipFile(first / "checkpoint.zip") as archive:
        require(archive.testzip() is None,"原检查点压缩包损坏")
        network = torch.load(io.BytesIO(archive.read("policy.pth")),map_location="cpu",weights_only=True)
        optimizer = torch.load(io.BytesIO(archive.read("policy.optimizer.pth")),map_location="cpu",weights_only=True)
    require(state_digest(network) == proof["network_state_sha256"],"实际训练入口没有完整载入原网络")
    require(state_digest(optimizer) == proof["optimizer_state_sha256"],"实际训练入口没有完整载入原优化器")
    require(bool(optimizer["state"]),"原训练未形成优化器状态")
    with zipfile.ZipFile(resumed / "checkpoint.zip") as archive:
        require(archive.testzip() is None,"续训检查点压缩包损坏")
        next_network = torch.load(io.BytesIO(archive.read("policy.pth")),map_location="cpu",weights_only=True)
        next_optimizer = torch.load(io.BytesIO(archive.read("policy.optimizer.pth")),map_location="cpu",weights_only=True)
    require(set(optimizer["state"]) == set(next_optimizer["state"]),"优化器参数映射变化")
    old_steps = [float(value["step"]) for value in optimizer["state"].values()]
    new_steps = [float(next_optimizer["state"][key]["step"]) for key in optimizer["state"]]
    require(all(new >= old for new,old in zip(new_steps,old_steps)) and any(new > old for new,old in zip(new_steps,old_steps)),"优化器没有在原步数上继续更新")
    require(state_digest(network) != state_digest(next_network),"续训没有更新网络")
    torch.set_num_threads(1)
    model = PPO.load(resumed / "checkpoint.zip",device="cpu")
    observations = np.random.default_rng(749).normal(0,.5,(128,93)).astype(np.float32)
    expected,_ = model.predict(observations,deterministic=True)
    policy = StudyPolicy(resumed)
    actual = np.asarray([policy.predict(row) for row in observations])
    error = float(np.max(np.abs(actual-expected)))
    require(error < 2e-5 and np.all(actual[:,0] >= 0) and np.all(actual <= 1) and np.all(actual[:,1:] >= -1),"续训导出数值或动作边界错误")
    require(model._n_updates > proof["loaded_updates"],"训练更新计数未增加")
    return dict(schema=1,checks=dict(actual_1024_to_2048=True,network_load_exact=True,optimizer_load_exact=True,
        optimizer_updates_continue=True,network_updated=True,export_matches=True),
        export_max_abs_error=error,optimizer_states=len(old_steps),optimizer_step_before=old_steps,optimizer_step_after=new_steps,
        experiment_identity=experiment_identity(manifest),library_sha256=manifest["library_sha256"],
        first=str(first),resumed=str(resumed),test_source_sha256=digest(Path(__file__)),
        files={str(directory / name):digest(directory / name) for directory in (first,resumed)
            for name in ("checkpoint.zip","policy.json","policy.npz","run.json","status.json")},
        resume_load_check_sha256=digest(resumed / "resume-load-check.json"),
        boundary="真实网络和优化器精确载入后继续更新；物理回合/洗牌重新开始，不宣称逐步重放，也不代表长训练已达到性能目标。")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("first","resumed","output"):
        parser.add_argument("--"+name,type=Path,required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    result = check(args.first,args.resumed)
    args.output.write_bytes((json.dumps(result,ensure_ascii=False,indent=2)+"\n").encode("utf-8"))
    print(json.dumps(dict(checks=result["checks"],export_max_abs_error=result["export_max_abs_error"],optimizer_states=result["optimizer_states"]),ensure_ascii=False,indent=2))
