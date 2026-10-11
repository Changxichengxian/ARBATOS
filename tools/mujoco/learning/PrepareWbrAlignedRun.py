"""等待原控制基线后核验、冻结并运行真实短训；通过才启动登记筛选。"""
import argparse
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

from RunWbrStudyQueue import process_alive,request_child_stop,source_binding,write_json

REPO = Path(__file__).resolve().parents[3]


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def require(value,message):
    if not value:
        raise RuntimeError(message)


def verify_frozen(root):
    manifest = read(root / "manifest.json")
    require(all(digest(root / name) == expected for name,expected in manifest["files"].items()),"冻结文件发生变化")
    require(digest(root / manifest["library"]) == manifest["library_sha256"],"冻结动态库发生变化")
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--study-dir",type=Path,required=True)
    parser.add_argument("--original-baseline",type=Path,required=True)
    parser.add_argument("--resume",action="store_true")
    args = parser.parse_args()
    base = args.study_dir.resolve()
    output = base / "admission"
    output.mkdir(exist_ok=True)
    baseline_frozen = base / "baseline-frozen"
    frozen = base / "training-frozen"
    current = REPO / "tools/mujoco/learning"
    relative_rule = "tools/mujoco/learning/results/study-20261012/对齐评分参考策略训练登记.json"
    rule = read(REPO / relative_rule)
    deadline = datetime.fromisoformat(rule["deadline"]).timestamp()
    inputs = {str(path):digest(path) for path in [*current.glob("*.py"),REPO / relative_rule,args.original_baseline.resolve(),base / "baseline-queue/plan.json"]}
    binding = dict(study_dir=str(base),deadline=rule["deadline"],inputs=inputs,python=sys.executable)
    lock_stream = (output / "admission.lock").open("a+b")
    lock_stream.seek(0)
    if not lock_stream.read(1):
        lock_stream.write(b"0")
        lock_stream.flush()
    lock_stream.seek(0)
    if os.name == "nt":
        import msvcrt
        msvcrt.locking(lock_stream.fileno(),msvcrt.LK_NBLCK,1)
    else:
        import fcntl
        fcntl.flock(lock_stream.fileno(),fcntl.LOCK_EX | fcntl.LOCK_NB)
    state_path = output / "status.json"
    if state_path.exists():
        require(args.resume,"准备流程已有状态，请先核查后显式续跑")
        state = read(state_path)
        require(state["binding"] == binding,"准备源码或输入变化，不能混用旧流程")
        require(not state.get("child_pid") or not process_alive(state["child_pid"]),"之前的子任务仍在运行，禁止重复启动")
    else:
        state = dict(schema=1,binding=binding,steps={})
    state.update(status="running",pid=os.getpid(),child_pid=None)
    process = None

    def save(stage):
        state.update(stage=stage,updated_at=datetime.now().astimezone().isoformat())
        write_json(state_path,state)

    def guard():
        require(time.time() < deadline,"研究期限已到")
        require(not (output / "stop.request").exists(),"准备流程收到停止请求")
        require(all(digest(path) == expected for path,expected in inputs.items()),"准备源码或输入在运行中发生变化")

    def run(name,arguments):
        nonlocal process
        guard()
        save(name)
        command = [sys.executable,"-u",*map(str,arguments)]
        log = output / (name+".log")
        step = dict(command=command,started_at=datetime.now().astimezone().isoformat(),log=str(log))
        state["steps"][name] = step
        with log.open("ab") as stream:
            process = subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT,
                creationflags=subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0)
            state["child_pid"] = process.pid
            save(name)
            while process.poll() is None:
                if time.time() >= deadline or (output / "stop.request").exists():
                    # 队列或训练程序的--output位置不同，这里请求其队列根目录停止。
                    if "--plan" in command:
                        child_output = Path(command[command.index("--plan")+1]).parent
                    else:
                        child_output = Path(command[command.index("--output")+1]).parent if "--output" in command else output
                    request_child_stop(process,child_output,step)
                time.sleep(2.)
            code = process.returncode
        state["child_pid"] = None
        process = None
        step.update(exit_code=code,finished_at=datetime.now().astimezone().isoformat())
        save(name)
        require(code == 0,"准备步骤失败："+name)

    def plan_queue(directory,jobs,stage):
        directory.mkdir(exist_ok=True)
        plan_path = directory / "plan.json"
        plan = dict(schema=1,stage=stage,deadline=rule["deadline"],python=sys.executable,
            learning_directory=str(frozen / "tools/mujoco/learning"),jobs=jobs)
        if plan_path.exists():
            require(read(plan_path) == plan,"既有运行计划发生变化")
        else:
            write_json(plan_path,plan)
        state_file = directory / "queue-state.json"
        if state_file.exists():
            previous = read(state_file)
            require(not process_alive(previous["pid"]),"既有队列仍在运行，禁止重复启动")
            # 完成标记不能代替产物核验；队列续跑会逐项复查并跳过完整步骤。
        run(stage,[frozen / "tools/mujoco/learning/RunWbrStudyQueue.py","--plan",plan_path,
                   *(["--resume"] if state_file.exists() else [])])
        queue = read(state_file)
        require(queue["status"] == "completed","子队列没有完整完成："+stage)
        return queue

    try:
        save("waiting_baselines")
        while True:
            guard()
            queue = read(base / "baseline-queue/queue-state.json")
            if queue["status"] == "completed":
                break
            require(queue["status"] == "running" and process_alive(queue["pid"]),"基线队列未完成且没有活进程，需要核查")
            time.sleep(10.)
        original_manifest = verify_frozen(baseline_frozen)
        for name,count in (("training",144),("monitor",24)):
            report = read(base / "baselines" / name / "summary.json")
            require(len(report["baseline_results"]) == count and report["acceptance"]["overall_pass"],"训练或监控原控制基线不完整")
        old_learning = baseline_frozen / "tools/mujoco/learning"
        bank_lock = REPO / rule["training_bank_lock"]
        if not bank_lock.exists():
            run("seal_bank",[old_learning / "SealWbrAlignedBank.py","--cases",old_learning / "cases/aligned-20261012/training.json",
                "--baseline",base / "baselines/training/summary.json","--registration",baseline_frozen / relative_rule,"--destination-root",REPO])
        lock = read(bank_lock)
        require(lock["registration_sha256"] == digest(REPO / relative_rule),"训练库许可登记不符")
        require(all(digest(REPO / name) == expected for name,expected in lock["source_files"].items()),"训练库许可源文件变化")
        bank = read(bank_lock.parent / "training-bank.json")
        require(bank["baseline_summary_sha256"] == digest(base / "baselines/training/summary.json"),"训练库来自其他基线")
        if not (frozen / "manifest.json").exists():
            run("freeze_training",[current / "PrepareWbrLearning.py","--output",frozen,"--reference-snapshot",baseline_frozen,"--reuse-library"])
        manifest = verify_frozen(frozen)
        require(source_binding(frozen / "tools/mujoco/learning") == source_binding(old_learning),"补入基线改变了物理或推理身份")
        require(all(manifest["files"][path.relative_to(REPO).as_posix()] == expected for name,expected in inputs.items()
                    if (path := Path(name)).is_relative_to(current)),"训练冻结不是登记的准备代码")
        frozen_learning = frozen / "tools/mujoco/learning"
        frozen_bank = frozen / rule["training_bank_lock"]
        frozen_bank = frozen_bank.parent / "training-bank.json"
        preflight = output / "wrapper-preflight.json"
        if not preflight.exists():
            run("wrapper_check",[frozen_learning / "TestWbrAligned.py","--snapshot",frozen,"--bank",frozen_bank,
                "--baseline",base / "baselines/development/summary.json","--original-baseline",args.original_baseline.resolve(),"--output",preflight])
        proof = read(preflight)
        require(proof["bank_sha256"] == digest(frozen_bank) and proof["test_source_sha256"] == digest(frozen_learning / "TestWbrAligned.py"),"包装检查来源不符")
        require(all(proof["checks"].get("wrapper_"+name,{}).get("same_original_physics") for name in ("stop","push","combined"))
                and proof["checks"].get("physical_failure_terminal_and_offline_exact"),"包装检查没有完整通过")
        smoke_root = base / "training-smoke"
        smoke_root.mkdir(exist_ok=True)
        common = ["--bank",str(frozen_bank),"--registration",str(frozen / relative_rule),"--seed","9001","--envs","1"]
        smoke = plan_queue(smoke_root,[dict(id="first",kind="aligned_train",arguments=["--output",str(smoke_root / "first"),"--steps","1024",*common]),
            dict(id="resumed",kind="aligned_train",arguments=["--output",str(smoke_root / "resumed"),"--steps","2048",*common,"--resume","{job:first}"])],"real_training_smoke")
        smoke_proof = output / "training-resume-preflight.json"
        if not smoke_proof.exists():
            run("resume_check",[frozen_learning / "CheckWbrAlignedSmoke.py","--first",smoke["jobs"]["first"]["actual_output"],
                "--resumed",smoke["jobs"]["resumed"]["actual_output"],"--output",smoke_proof])
        proof = read(smoke_proof)
        require(all(proof["checks"].values()) and proof["test_source_sha256"] == digest(frozen_learning / "CheckWbrAlignedSmoke.py"),"真实短训续训未通过")
        require(proof["first"] == smoke["jobs"]["first"]["actual_output"]
                and proof["resumed"] == smoke["jobs"]["resumed"]["actual_output"],"续训证明来自其他短训")
        expected_files = {str(Path(smoke["jobs"][name]["actual_output"]) / filename)
                          for name in ("first","resumed")
                          for filename in ("checkpoint.zip","policy.json","policy.npz","run.json","status.json")}
        require(set(proof["files"]) == expected_files
                and all(digest(path) == expected for path,expected in proof["files"].items()),"续训证明中的实际模型或运行文件已变化")
        require(proof["resume_load_check_sha256"] == digest(Path(proof["resumed"]) / "resume-load-check.json"),"实际网络和优化器载入证明已变化")
        jobs = []
        screening_root = base / "screening"
        for seed in rule["screening_seeds"]:
            name = "E-aligned-"+str(seed)
            jobs.append(dict(id=name+"-train",kind="aligned_train",arguments=["--output",str(screening_root / str(seed) / "train"),
                "--bank",str(frozen_bank),"--registration",str(frozen / relative_rule),"--seed",str(seed),"--envs",str(rule["envs"]),"--steps",str(rule["screening_first"]["steps"])]))
            jobs.append(dict(id=name+"-monitor",kind="evaluate",arguments=["--output",str(screening_root / str(seed) / "monitor"),
                "--cases",str(frozen_learning / "cases/aligned-20261012/monitor.json"),"--policy","{job:"+name+"-train}",
                "--baseline-cache",str(base / "baselines/monitor/summary.json"),"--deadline",rule["deadline"]]))
        screening = plan_queue(screening_root,jobs,"screening_first")
        promotion_path = screening_root / "screen-expand-promotion.json"
        if not promotion_path.exists():
            run("screening_promotion",[frozen_learning / "PromoteWbrAligned.py","--stage","screen_expand","--registration",frozen / relative_rule,
                "--evaluations",*[screening["jobs"]["E-aligned-"+str(seed)+"-monitor"]["actual_output"] for seed in rule["screening_seeds"]],
                "--baseline",base / "baselines/monitor/summary.json","--output",promotion_path])
        run("promotion_recheck",["-c","import sys; sys.path.insert(0,sys.argv[1]); from PromoteWbrAligned import verify_promotion; verify_promotion(sys.argv[2], require_eligible=False)",frozen_learning,promotion_path])
        state.update(status="completed",promotion=str(promotion_path),eligible=read(promotion_path)["eligible"])
        save("ready_for_next_stage")
    except BaseException as error:
        state.update(status="needs_attention",error=repr(error))
        if process is not None and process.poll() is None:
            request_child_stop(process,output,state["steps"].get(state.get("stage"),{}))
        save(state.get("stage","failed"))
        raise
    finally:
        lock_stream.close()


if __name__ == "__main__":
    main()
